"""The stages of section 5 in order: camera frames in, a Snapshot out.

    back-project (5.2) -> table plane (5.3) -> clusters (5.4) -> SAM masks (5.5) -> fuse views (5.6)
    -> measure (5.8) -> identify (5.7) -> known-size fallback, support, ids, confidence (5.8)
    -> change mask against the reference (6)

SAM and DINOv2 are optional: without a segmenter the cluster pixels are the masks, without an
embedder every object is "unknown" and has no embedding.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass

import cv2
import numpy as np

from . import describe as desc
from .change import change_mask
from .embed import hsv_histogram, masked_crop
from .fuse import View, associate, footprint_cells, polygon_cells
from .geometry import (TableFrame, dbscan, fit_plane, pixels_to_points, plane_height_at, plane_heights,
                       ray_plane, statistical_outliers, tilt_deg, transform, voxel_downsample)
from .segment import depth_poor_masks, refine
from .types import CameraFrame, ObjectState, PerceptionError, Snapshot, camera_bit

log = logging.getLogger("restore_perception.pipeline")

MIN_MEASURE_POINTS = 10  # fewer points than this give no measured size at all


@dataclass
class _Camera:
    """One camera's valid depth pixels inside the workspace box."""

    frame: CameraFrame
    bit: int
    pixels: np.ndarray  # N x 2 (v, u)
    points: np.ndarray  # N x 3 world, float32
    index_map: np.ndarray  # H x W int32: index into points, -1 = none
    heights: np.ndarray | None = None  # above the table plane


class _Timer:
    def __init__(self):
        self.last = time.monotonic()
        self.timings = {}

    def lap(self, stage):
        now = time.monotonic()
        self.timings[stage] = round((now - self.last) * 1000.0, 1)
        self.last = now


class Pipeline:
    def __init__(self, config, objects=None, segmenter=None, embedder=None, gallery=None):
        self.cfg = config
        self.objects = objects or {}
        self.segmenter = segmenter if config["segment"]["enabled"] else None
        self.embedder = embedder if config["identify"]["enabled"] else None
        self.gallery = gallery
        self.last_table_height = None
        ws = config["workspace"]
        self._low, self._high = np.array(ws["min"], np.float32), np.array(ws["max"], np.float32)

    # ---------- public ----------

    def run(self, frames, snapshot_id=None, reference=None, identify=True, keep_unplaced=False):
        """One snapshot from posed frames. reference: a previous Snapshot for the change mask (6).

        keep_unplaced keeps objects that only have masks (depth-poor and not named), as registration
        needs; a normal snapshot drops them because nothing gives them a size or a place.
        """
        timer = _Timer()
        warnings = []
        snapshot_id = snapshot_id or time.strftime("%Y%m%d-%H%M%S")
        cams = self._back_project(frames)
        timer.lap("back_project")
        plane, table, table_height = self._table(cams, warnings)
        timer.lap("table")
        candidates = self._cluster(cams, table)
        timer.lap("cluster")
        views = self._views(cams, candidates, plane, table, snapshot_id)
        timer.lap("segment")
        objects, lowest = self._fuse(associate(views, self.cfg["fuse"]), cams, table)
        timer.lap("fuse")
        if identify:
            self._identify(objects, cams)
            timer.lap("identify")
        objects = self._finish(objects, lowest, cams, plane, table, keep_unplaced)
        timer.lap("describe")
        snapshot = Snapshot(snapshot_id, max(f.stamp for f in frames), plane, objects, list(frames),
                            warnings, timer.timings, table_height)
        if reference is not None:
            self._change(snapshot, reference)
            timer.lap("change")
        self.last_table_height = table_height
        snapshot.timings["total"] = round(sum(v for k, v in snapshot.timings.items() if k != "total"), 1)
        return snapshot

    def register(self, frames, name, up_side=""):
        """Section 7.1: store the views of the single object on the table. Returns (name, views, added,
        snapshot)."""
        if self.embedder is None or self.gallery is None:
            raise PerceptionError("registration needs DINOv2 and a gallery")
        snapshot = self.run(frames, f"register-{time.strftime('%Y%m%d-%H%M%S')}", identify=False, keep_unplaced=True)
        if len(snapshot.objects) != 1:
            raise PerceptionError(f"registration expects exactly one object on the table, found "
                                  f"{len(snapshot.objects)}")
        obj = snapshot.objects[0]
        by_name = {f.name: f for f in snapshot.frames}
        crops, histograms, cameras = [], [], []
        for camera, mask in obj.masks.items():
            if mask.any():
                crops.append(masked_crop(by_name[camera].color, mask))
                histograms.append(hsv_histogram(by_name[camera].color, mask))
                cameras.append(camera)
        if not crops:
            raise PerceptionError("the object has no mask in any camera")
        size = None if obj.depth_poor or len(obj.points) < self.cfg["describe"]["min_points"] else obj.size
        stored_name, total = self.gallery.add(name, crops, self.embedder.embed(crops), np.stack(histograms),
                                              size, up_side, cameras)
        return stored_name, total, len(crops), snapshot

    # ---------- 5.2 ----------

    def _back_project(self, frames):
        if not frames:
            raise PerceptionError("no camera frames")
        stride = int(self.cfg["workspace"]["pixel_stride"])
        cams = []
        for i, frame in enumerate(frames):
            if frame.T_world_cam is None:
                raise PerceptionError(f"{frame.name} ({frame.serial}) has no pose: calibrate it (section 3.4)")
            h, w = frame.depth.shape
            if (frame.intrinsics.width, frame.intrinsics.height) != (w, h):
                raise PerceptionError(f"{frame.name}: intrinsics are for {frame.intrinsics.width}x"
                                      f"{frame.intrinsics.height}, the image is {w}x{h}")
            v, u = np.nonzero(frame.depth[::stride, ::stride] > 0)
            v, u = v * stride, u * stride
            points = transform(frame.T_world_cam.astype(np.float32),
                               pixels_to_points(frame.depth, frame.intrinsics.matrix(), v, u))
            inside = np.all((points >= self._low) & (points <= self._high), axis=1)
            v, u, points = v[inside], u[inside], points[inside]
            index_map = np.full((h, w), -1, np.int32)
            index_map[v, u] = np.arange(len(v), dtype=np.int32)
            cams.append(_Camera(frame, camera_bit(frame.name, i), np.column_stack([v, u]), points, index_map))
        if sum(len(c.points) for c in cams) < 100:
            raise PerceptionError("almost no depth inside the workspace box: check the workspace parameters "
                                  "and the calibration")
        return cams

    # ---------- 5.3 ----------

    def _table(self, cams, warnings):
        cfg = self.cfg["table"]
        points = np.concatenate([c.points for c in cams]).astype(np.float64)
        centroids, _ = voxel_downsample(points, cfg["voxel_m"])
        plane, _ = fit_plane(centroids, cfg["distance_threshold_m"], int(cfg["ransac_iterations"]))
        tilt = tilt_deg(plane)
        if tilt > cfg["max_tilt_deg"]:
            raise PerceptionError(f"table not found or camera moved: the largest plane is tilted {tilt:.1f} "
                                  f"degrees from world up")
        centre = (self._low[:2] + self._high[:2]) / 2.0
        height = float(plane_height_at(plane, *centre))
        if self.last_table_height is not None and abs(height - self.last_table_height) > cfg["max_height_change_m"]:
            warnings.append(f"table not found or camera moved: the table height changed by "
                            f"{1000 * (height - self.last_table_height):+.0f} mm since the previous capture")
        for c in cams:
            c.heights = plane_heights(c.points, plane.astype(np.float32))
        return plane, TableFrame(plane, centre), height

    # ---------- 5.4 ----------

    def _cluster(self, cams, table):
        """Candidates as {camera index: indices into that camera's points}."""
        cfg = self.cfg["cluster"]
        min_height = self.cfg["table"]["object_min_height_m"]
        owner, local, points = [], [], []
        for ci, c in enumerate(cams):
            idx = np.nonzero(c.heights > min_height)[0]
            owner.append(np.full(len(idx), ci))
            local.append(idx)
            points.append(c.points[idx])
        owner, local, points = np.concatenate(owner), np.concatenate(local), np.concatenate(points)
        if len(points) == 0:
            return []
        centroids, ids = voxel_downsample(points.astype(np.float64), cfg["voxel_m"])
        labels = dbscan(centroids, cfg["eps_m"], int(cfg["min_points"]))[ids]
        percentile = self.cfg["describe"]["height_percentile"]
        candidates = []
        for k in range(labels.max() + 1):
            members = np.nonzero(labels == k)[0]
            if len(members) < cfg["min_cluster_points"]:
                continue
            t = table.to_table(points[members])
            extent = max(np.ptp(t[:, 0]), np.ptp(t[:, 1]))
            if extent > cfg["max_extent_m"] or np.percentile(t[:, 2], percentile) < cfg["min_height_m"]:
                continue
            candidates.append({ci: local[members[owner[members] == ci]] for ci in range(len(cams))})
        return candidates

    # ---------- 5.5 ----------

    def _view(self, cam, mask, idx, score, table):
        points = cam.points[idx]
        xy = table.to_table(points)[:, :2]
        return View(cam.frame.name, mask, points, score, footprint_cells(xy, self.cfg["fuse"]["grid_m"]),
                    xy.mean(axis=0))

    def _depth_poor_view(self, cam, mask, score, plane, table):
        """A view with no points: its footprint is the mask outline projected onto the plane at an
        assumed object height, which is enough to associate it across cameras."""
        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        outline = np.concatenate(contours).reshape(-1, 2)
        hits = ray_plane(outline, cam.frame.intrinsics.matrix(), cam.frame.T_world_cam, plane,
                         self.cfg["segment"]["depth_poor"]["assumed_height_m"])
        hits = hits[np.all(np.isfinite(hits), axis=1)]
        if len(hits) < 3:
            return None
        centre = hits.mean(axis=0)
        if np.any(centre[:2] < self._low[:2]) or np.any(centre[:2] > self._high[:2]):
            return None
        xy = table.to_table(hits)[:, :2]
        return View(cam.frame.name, mask, np.zeros((0, 3), np.float32), score,
                    polygon_cells(xy, self.cfg["fuse"]["grid_m"]), xy.mean(axis=0), depth_poor=True)

    def _views(self, cams, candidates, plane, table, snapshot_id):
        """Per camera, one View per object: SAM masks when a segmenter is set, else cluster pixels."""
        scfg, ccfg = self.cfg["segment"], self.cfg["cluster"]
        pad = int(ccfg["box_padding_px"])
        grow, close = np.ones((3, 3), np.uint8), np.ones((7, 7), np.uint8)
        views = [[] for _ in cams]
        for ci, cam in enumerate(cams):
            h, w = cam.frame.depth.shape
            seen = [cand[ci] for cand in candidates if len(cand.get(ci, ())) >= ccfg["min_camera_points"]]
            if self.segmenter is not None and seen:
                self.segmenter.set_image((snapshot_id, cam.frame.name), cam.frame.color)
            claimed = np.zeros((h, w), bool)  # a pixel belongs to one object per camera
            for idx in seen:
                v, u = cam.pixels[idx, 0], cam.pixels[idx, 1]
                target = np.zeros((h, w), np.uint8)
                target[v, u] = 1
                target = cv2.morphologyEx(cv2.dilate(target, grow), cv2.MORPH_CLOSE, close).astype(bool)
                if self.segmenter is None:
                    mask = target & ~claimed
                    claimed |= mask
                    views[ci].append(self._view(cam, mask, idx, None, table))
                    continue
                box = [max(int(u.min()) - pad, 0), max(int(v.min()) - pad, 0),
                       min(int(u.max()) + pad, w - 1), min(int(v.max()) + pad, h - 1)]
                for mask, score in refine(self.segmenter, target, box, scfg):
                    mask = mask & ~claimed
                    own = cam.index_map[mask]
                    own = own[own >= 0]
                    own = own[cam.heights[own] >= scfg["mask_min_height_m"]]
                    if len(own) >= scfg["min_mask_points"]:
                        claimed |= mask
                        views[ci].append(self._view(cam, mask, own, score, table))
            dp = scfg["depth_poor"]
            if self.segmenter is not None and dp["enabled"] and (not dp["cameras"] or cam.frame.name in dp["cameras"]):
                for mask, score in depth_poor_masks(self.segmenter, cam.frame.color, [v.mask for v in views[ci]], dp):
                    view = self._depth_poor_view(cam, mask, score, plane, table)
                    if view is not None:
                        views[ci].append(view)
        return views

    # ---------- 5.6 ----------

    def _fuse(self, groups, cams, table):
        cfg = self.cfg["fuse"]
        bits = {c.frame.name: c.bit for c in cams}
        objects, lowest = [], {}
        for group in groups:
            with_points = [v.points for v in group if len(v.points)]
            points = np.concatenate(with_points) if with_points else np.zeros((0, 3), np.float32)
            if len(points) > cfg["outlier_neighbors"]:
                points = points[~statistical_outliers(points, int(cfg["outlier_neighbors"]), cfg["outlier_std_ratio"])]
            scores = [v.score for v in group if v.score is not None]
            obj = ObjectState(
                id="", points=points.astype(np.float32), masks={v.camera: v.mask for v in group},
                cameras_seen=int(np.bitwise_or.reduce([bits[v.camera] for v in group])),
                sam_score=float(np.mean(scores)) if scores else None,
                depth_poor=all(v.depth_poor for v in group))
            if len(points) >= MIN_MEASURE_POINTS and not obj.depth_poor:
                lowest[id(obj)] = self._measure(obj, table)
            objects.append(obj)
        return objects, lowest

    # ---------- 5.8 (geometry) ----------

    def _measure(self, obj, table):
        """Pose and size as if the object stood on the table; returns its lowest point height, which
        _finish uses to lift it onto a supporting object."""
        d = desc.describe_points(obj.points, table, self.cfg["describe"])
        obj.position = table.to_world([[d["centre"][0], d["centre"][1], d["top"] / 2.0]])[0]
        obj.yaw = d["yaw"]
        obj.orientation_xyzw = table.orientation(d["yaw"])
        obj.size = np.array([d["length"], d["width"], d["top"]])
        obj.footprint = table.to_world(d["footprint"])
        return d["lowest"]

    def _lift(self, obj, lowest, table):
        """The object rests on another: its height starts at its lowest points."""
        top = float(obj.size[2])
        centre = table.to_table(obj.position[None])[0]
        obj.bottom_height = lowest
        obj.size[2] = max(top - lowest, 0.0)
        obj.position = table.to_world([[centre[0], centre[1], lowest + obj.size[2] / 2.0]])[0]

    def _known_size(self, obj, size, frames, plane, table):
        """Section 5.8 fallback: the registered size, placed where the masks say."""
        pose = desc.known_size_pose(obj.masks, frames, plane, table, size, self.cfg["describe"])
        if pose is None:
            return False
        (cx, cy), yaw = pose
        length, width, height = (float(v) for v in size)
        obj.size = np.array([length, width, height])
        obj.yaw = yaw
        obj.position = table.to_world([[cx, cy, obj.bottom_height + height / 2.0]])[0]
        obj.orientation_xyzw = table.orientation(yaw)
        c, s = np.cos(yaw), np.sin(yaw)
        corners = np.array([[length, width], [-length, width], [-length, -width], [length, -width]]) / 2.0
        corners = corners @ np.array([[c, s], [-s, c]]) + [cx, cy]
        obj.footprint = table.to_world(np.column_stack([corners, np.zeros(4)]))
        obj.known_size = True
        return True

    # ---------- 5.7 ----------

    def _identify(self, objects, cams):
        if self.embedder is None or not objects:
            return
        frames = {c.frame.name: c.frame for c in cams}
        crops, owners, histograms = [], [], []
        for i, obj in enumerate(objects):
            per_camera = []
            for camera, mask in obj.masks.items():
                if mask.any():
                    crops.append(masked_crop(frames[camera].color, mask))
                    owners.append(i)
                    per_camera.append(hsv_histogram(frames[camera].color, mask))
            histograms.append(np.mean(per_camera, axis=0) if per_camera else None)
        embeddings = self.embedder.embed(crops)
        owners = np.array(owners)
        for i, obj in enumerate(objects):
            rows = embeddings[owners == i]
            if len(rows):
                mean = rows.mean(axis=0)  # average over the cameras, normalise again
                obj.embedding = (mean / np.linalg.norm(mean)).astype(np.float32)
        if self.gallery is None or not self.gallery.names():
            return
        idx = [i for i, obj in enumerate(objects) if len(obj.embedding)]
        sizes = [objects[i].size if len(objects[i].points) >= MIN_MEASURE_POINTS and not objects[i].depth_poor
                 else None for i in idx]
        matches = self.gallery.match(np.stack([objects[i].embedding for i in idx]), [histograms[i] for i in idx],
                                     sizes, self.objects, self.cfg["identify"])
        for i, match in zip(idx, matches):
            objects[i].match = match
            objects[i].label = match["label"]
            objects[i].label_score = float(match["score"])

    # ---------- 5.8 (the rest) ----------

    def _finish(self, objects, lowest, cams, plane, table, keep_unplaced=False):
        cfg = self.cfg["describe"]
        frames = {c.frame.name: c.frame for c in cams}
        kept = []
        for obj in objects:
            info = self.objects.get(obj.label) if obj.label != "unknown" else None
            few_points = obj.depth_poor or len(obj.points) < cfg["min_points"]
            if info:
                obj.symmetry, obj.up_side_matters = info["symmetry"], info["up_side_matters"]
                obj.depth_reliable = info["depth_reliable"]
            else:
                obj.depth_reliable = not few_points
            obj.up_side = obj.match.get("up_side", "") if obj.label != "unknown" else ""
            size = None
            if obj.label != "unknown":
                size = (info or {}).get("size_m") or (self.gallery.measured_size(obj.label) if self.gallery else None)
            if size is not None and (few_points or not obj.depth_reliable):
                self._known_size(obj, size, frames, plane, table)
            placed = obj.known_size or (not obj.depth_poor and len(obj.points) >= MIN_MEASURE_POINTS)
            if placed or keep_unplaced:
                kept.append(obj)

        order = sorted(kept, key=lambda o: tuple(np.round(table.to_table(o.position[None])[0, :2], 3)))
        cells = {}
        for i, obj in enumerate(order):
            obj.id = f"obj_{i:02d}"
            cells[obj.id] = polygon_cells(table.to_table(obj.footprint)[:, :2], self.cfg["fuse"]["grid_m"])
        # Lowest objects first, so a supporting object already has its final height.
        for obj in sorted(order, key=lambda o: lowest.get(id(o)) or 0.0):
            low = None if obj.known_size else lowest.get(id(obj))
            obj.support_id = desc.resting_on(obj, low, order, cells, cfg["support_min_bottom_m"])
            if obj.support_id != "table":
                self._lift(obj, low, table)
        for obj in order:
            obj.confidence = desc.confidence(obj, cfg["min_points"])
        return order

    # ---------- 6 ----------

    def _change(self, snapshot, reference):
        cfg = self.cfg["change"]
        previous = {f.name: f for f in reference.frames}
        for frame in snapshot.frames:
            ref = previous.get(frame.name)
            if ref is None or ref.color.shape != frame.color.shape:
                continue
            changed = change_mask(ref, frame, cfg)
            share = float(changed.mean())
            snapshot.changed_fraction[frame.name] = round(share, 4)
            if share > cfg["bumped_fraction"]:
                snapshot.warnings.append(f"{frame.name}: {share:.0%} of the image changed since the reference; "
                                         f"was the camera bumped? Recalibrate it (section 6)")
            for obj in snapshot.objects:
                mask = obj.masks.get(frame.name)
                if mask is not None and mask.any():
                    part = float(np.count_nonzero(changed & mask) / np.count_nonzero(mask))
                    obj.changed = part if obj.changed is None else max(obj.changed, part)
