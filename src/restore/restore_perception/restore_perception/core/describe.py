"""Section 5.8: pose, size, footprint, support and confidence of each object."""

from __future__ import annotations

import math

import cv2
import numpy as np

from .geometry import ray_plane


def _long_side_yaw(box):
    """Yaw of the longer side of a cv2.boxPoints rectangle, in [-pi/2, pi/2)."""
    e1, e2 = box[1] - box[0], box[2] - box[1]
    edge = e1 if np.linalg.norm(e1) >= np.linalg.norm(e2) else e2
    yaw = math.atan2(edge[1], edge[0])
    return (yaw + math.pi / 2) % math.pi - math.pi / 2


def rectangle(xy):
    """minAreaRect of 2D points: centre, (length, width) with length >= width, and yaw."""
    rect = cv2.minAreaRect(np.asarray(xy, np.float32).reshape(-1, 1, 2))
    (cx, cy), (w, h), _ = rect
    yaw = _long_side_yaw(cv2.boxPoints(rect))
    return np.array([cx, cy]), (max(w, h), min(w, h)), yaw


def footprint_polygon(xy, epsilon):
    hull = cv2.convexHull(np.asarray(xy, np.float32).reshape(-1, 1, 2))
    return cv2.approxPolyDP(hull, epsilon, True).reshape(-1, 2).astype(np.float64)


def describe_points(points, table, cfg):
    """Footprint rectangle on the table plane and height of an object's points, all in the table frame.

    Returns centre (x, y), yaw, length, width, top (95th percentile height), lowest (1st percentile)
    and the footprint polygon (z = 0). The lowest visible point says little on its own: the lower half
    of a lying cylinder is hidden from above. Whether the object rests on another is decided later
    from the objects below it (resting_on).
    """
    t = table.to_table(points)
    centre, (length, width), yaw = rectangle(t[:, :2])
    polygon = footprint_polygon(t[:, :2], cfg["footprint_epsilon_m"])
    return {
        "centre": centre,
        "yaw": yaw,
        "length": float(length),
        "width": float(width),
        "top": float(np.percentile(t[:, 2], cfg["height_percentile"])),
        "lowest": float(np.percentile(t[:, 2], 1.0)),
        "footprint": np.column_stack([polygon, np.zeros(len(polygon))]),
    }


def _silhouette(mask, frame, plane, table, height, grid, origin, shape):
    """Grid cells inside the mask's outline projected onto the plane at `height` above the table."""
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    outline = np.concatenate(contours).reshape(-1, 2)
    hits = ray_plane(outline, frame.intrinsics.matrix(), frame.T_world_cam, plane, height)
    hits = hits[np.all(np.isfinite(hits), axis=1)]
    if len(hits) < 3:
        return None
    xy = table.to_table(hits)[:, :2]
    cells = np.round((xy - origin) / grid).astype(np.int32)
    region = np.zeros(shape, np.uint8)
    cv2.fillPoly(region, [cv2.convexHull(cells.reshape(-1, 1, 2))], 1)
    return region.astype(bool)


def known_size_pose(masks, frames, plane, table, size, cfg, extent=0.6):
    """Section 5.8 fallback for objects with poor depth: centre and yaw from the masks alone.

    Every camera's mask outline is projected onto the plane at the object's mid-height and the
    projections are intersected, like a visual hull. With one camera the projection is stretched
    away from the camera, but roughly about the object's centre. Returns (centre xy, yaw) in the
    table frame, or None.
    """
    grid = float(cfg["hull_grid_m"])
    hits = []
    for name, mask in masks.items():
        frame = frames[name]
        ys, xs = np.nonzero(mask)
        centre_px = np.array([[xs.mean(), ys.mean()]])
        hit = ray_plane(centre_px, frame.intrinsics.matrix(), frame.T_world_cam, plane, size[2] / 2.0)
        if np.all(np.isfinite(hit)):
            hits.append(table.to_table(hit)[0, :2])
    if not hits:
        return None
    origin = np.mean(hits, axis=0) - extent / 2.0
    shape = (int(extent / grid), int(extent / grid))
    region = None
    for name, mask in masks.items():
        cells = _silhouette(mask, frames[name], plane, table, size[2] / 2.0, grid, origin, shape)
        if cells is not None:
            region = cells if region is None else region & cells
    if region is None or not region.any():
        return None
    rows, cols = np.nonzero(region)
    xy = np.column_stack([cols, rows]) * grid + origin
    centre, _, yaw = rectangle(xy)
    return centre, yaw


def resting_on(obj, lowest, others, cells, min_bottom, gap=0.02, min_overlap=0.2):
    """Section 5.8: id of the object this one rests on, or 'table'.

    An object rests on another when its lowest points are above min_bottom, at least min_overlap of
    its footprint lies over the other's (touching neighbours only share a border) and the other's top
    is within `gap` below or above those lowest points. The highest such object wins.
    """
    if lowest is None or lowest <= min_bottom:
        return "table"
    best, best_top = "table", -1.0
    own = cells[obj.id]
    for other in others:
        if other is obj or not len(own):
            continue
        top = other.bottom_height + float(other.size[2])
        overlap = len(np.intersect1d(own, cells[other.id], assume_unique=True)) / len(own)
        if overlap >= min_overlap and lowest - gap <= top <= lowest + gap and top > best_top:
            best, best_top = other.id, top
    return best


def confidence(obj, min_points):
    """Geometric mean of the SAM score, the point count, the camera count and the gallery score."""
    terms = [
        obj.sam_score if obj.sam_score is not None else 0.8,
        0.5 if obj.depth_poor else min(1.0, len(obj.points) / max(min_points, 1)),
        1.0 if bin(obj.cameras_seen).count("1") >= 2 else 0.8,
        obj.label_score if obj.label != "unknown" else 0.5,
    ]
    terms = [min(max(float(t), 1e-3), 1.0) for t in terms]
    return float(np.exp(np.mean(np.log(terms))))
