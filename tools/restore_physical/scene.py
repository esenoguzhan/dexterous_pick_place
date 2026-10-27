"""Scene perception on aras PC: fixed D435 from the Jetson over HTTP + restore_perception's geometry.

SAM 2.1 and DINOv2 stay off (they need the Jetson's GPU and its DDS link): masks come from depth
clusters above the table plane, identity from colour and size (match.py). The workspace box is the
white tabletop sheet, so the arm, the pendant and people around the table are never candidates.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

import jetson
from restore_perception.core import overlay
from restore_perception.core.config import load_config
from restore_perception.core.extrinsics import load_extrinsics
from restore_perception.core.pipeline import Pipeline

HERE = Path(__file__).resolve().parent
CONFIG = HERE / "config" / "scene.yaml"
EXTRINSICS = HERE / "config" / "extrinsics.yaml"
OUT = Path("/run_data/scenes")


def settings():
    return yaml.safe_load(CONFIG.read_text())


def pipeline_config(cfg):
    overrides = {
        "cameras": dict(jetson.NAMES),
        "workspace": {"min": cfg["workspace"]["min"], "max": cfg["workspace"]["max"], "pixel_stride": 2},
        "segment": {"enabled": False},
        "identify": {"enabled": False},
    }
    for key in ("table", "cluster", "describe"):
        if key in cfg:
            overrides[key] = cfg[key]
    return load_config(None, overrides)


def hsv_signature(color, mask):
    """Hue-saturation histogram of the object's pixels (8 x 4 bins) plus mean value, for matching."""
    hsv = cv2.cvtColor(color, cv2.COLOR_RGB2HSV)
    m = mask.astype(np.uint8)
    hist = cv2.calcHist([hsv], [0, 1], m, [12, 4], [0, 180, 0, 256]).ravel()
    hist /= max(hist.sum(), 1.0)
    v = float(hsv[..., 2][mask].mean()) / 255.0 if mask.any() else 0.0
    s = float(hsv[..., 1][mask].mean()) / 255.0 if mask.any() else 0.0
    return hist, s, v


class Scene:
    def __init__(self):
        self.cfg = settings()
        self.pcfg = pipeline_config(self.cfg)
        self.cam = load_extrinsics(EXTRINSICS)[jetson.FIXED]
        self.pipeline = Pipeline(self.pcfg)

    def capture(self, name=None, frames=10, save=True):
        """Objects on the sheet as plain dicts in world, plus the frame and the raw snapshot."""
        frame = jetson.capture(jetson.FIXED, frames=frames)
        frame.T_world_cam = self.cam.world_pose()
        snap = self.pipeline.run([frame], name or time.strftime("%H%M%S"))
        objects = []
        plane = self._refine_plane(frame, np.asarray(snap.table_plane, float))
        for obj in snap.objects:
            mask = obj.masks.get(frame.name)
            hist, sat, val = hsv_signature(frame.color, mask) if mask is not None else (np.zeros(48), 0, 0)
            # Mixed pixels at the silhouette (between the object and the table behind it) smear the
            # footprint towards the background. The top surface has none: its points give the axis.
            pts = obj.points.astype(float)
            h = pts @ plane[:3] + plane[3]
            top = float(np.percentile(h, 95)) if len(pts) else 0.0
            band = pts[h >= top - self.cfg["top_band_m"]]
            centre = np.median(band[:, :2], axis=0) if len(band) >= 5 else np.asarray(obj.position[:2])
            radius = float(np.percentile(np.linalg.norm(band[:, :2] - centre, axis=1), 90)) if len(band) >= 5 else 0.0
            top_centre, axis = centre, None
            wall = pts[(h > 0.2 * top) & (h < 0.8 * top)]
            fit = fit_axis(wall[:, :2], np.random.default_rng(0)) if len(wall) else None
            if fit is not None and fit[2] > 0.35 and fit[3] > 60.0 and abs(fit[1] - radius) < 0.012:
                centre, radius, axis = fit[0], fit[1], {"inliers": round(fit[2], 2), "span_deg": round(fit[3])}
            table_z = float(-(plane[3] + plane[0] * centre[0] + plane[1] * centre[1]) / plane[2])
            objects.append({
                "id": obj.id,
                "position": [float(centre[0]), float(centre[1]), table_z + top / 2.0],
                "yaw": float(obj.yaw),
                "size": [float(v) for v in obj.size[:2]] + [top],
                "top_radius": radius,
                "top_centre": [float(v) for v in top_centre],
                "axis_fit": axis,
                "top_z": table_z + top,
                "box_position": [float(v) for v in obj.position],
                "points": int(len(obj.points)),
                "pixels": int(mask.sum()) if mask is not None else 0,
                "hist": hist.tolist(), "sat": sat, "val": val,
                "footprint": [[float(a) for a in p[:2]] for p in obj.footprint],
            })
        objects = [o for o in objects if self._keep(o)]
        centre = (np.array(self.cfg["workspace"]["min"][:2]) + self.cfg["workspace"]["max"][:2]) / 2.0
        result = {"name": name or "", "stamp": time.time(),
                  "table_height": float(-(plane[3] + plane[:2] @ centre) / plane[2]),
                  "table_plane": [float(v) for v in plane], "objects": objects,
                  "warnings": list(snap.warnings)}
        if save:
            OUT.mkdir(parents=True, exist_ok=True)
            stem = OUT / (name or time.strftime("%Y%m%d-%H%M%S"))
            keep = {o["id"] for o in objects}
            image = overlay.draw(frame, [o for o in snap.objects if o.id in keep])
            for o in objects:
                self._label(image, o)
            cv2.imwrite(str(stem) + ".jpg", image[..., ::-1])
            cv2.imwrite(str(stem) + "_color.png", frame.color[..., ::-1])
            np.save(str(stem) + "_depth.npy", frame.depth)
            Path(str(stem) + ".json").write_text(json.dumps(result, indent=1))
        return result, frame, snap

    def _refine_plane(self, frame, plane, band=0.006, stride=4):
        """Least-squares plane through the sheet points near the RANSAC plane (which wobbles ~2 deg)."""
        h, w = frame.depth.shape
        v, u = np.mgrid[0:h:stride, 0:w:stride]
        z = frame.depth[v, u]
        ok = z > 0
        u, v, z = u[ok], v[ok], z[ok]
        K = frame.intrinsics.matrix()
        p = np.c_[(u - K[0, 2]) / K[0, 0] * z, (v - K[1, 2]) / K[1, 1] * z, z]
        T = frame.T_world_cam
        p = p @ T[:3, :3].T + T[:3, 3]
        lo, hi = np.array(self.cfg["workspace"]["min"]), np.array(self.cfg["workspace"]["max"])
        p = p[np.all((p >= lo) & (p <= hi), axis=1)]
        p = p[np.abs(p @ plane[:3] + plane[3]) < band]
        if len(p) < 500:
            return plane
        A = np.c_[p[:, 0], p[:, 1], np.ones(len(p))]
        a, b, c = np.linalg.lstsq(A, p[:, 2], rcond=None)[0]
        n = np.array([-a, -b, 1.0])
        s = np.linalg.norm(n)
        return np.r_[n / s, -c / s]

    def _keep(self, o):
        f = self.cfg["filter"]
        length, width, height = o["size"]
        return (height >= f["min_height_m"] and max(length, width) <= f["max_extent_m"]
                and o["points"] >= f["min_points"])

    def _label(self, image, o):
        # The axis at the top (white cross) and at the base (white dot); the top-surface centre (red).
        for z, style in ((o["top_z"], "cross"), (o["top_z"] - o["size"][2], "dot")):
            uv = self.project(np.array([o["position"][0], o["position"][1], z]))
            if uv is not None:
                if style == "cross":
                    cv2.drawMarker(image, (int(uv[0]), int(uv[1])), (255, 255, 255), cv2.MARKER_CROSS, 16, 2)
                else:
                    cv2.circle(image, (int(uv[0]), int(uv[1])), 4, (255, 255, 255), -1)
        tc = self.project(np.array([o["top_centre"][0], o["top_centre"][1], o["top_z"]]))
        if tc is not None:
            cv2.circle(image, (int(tc[0]), int(tc[1])), 5, (255, 0, 0), 2)
        uv = self.project(np.array(o["position"]))
        if uv is None:
            return
        u, v = int(uv[0]), int(uv[1])
        text = f"{o['id']} ({o['position'][0]:.3f},{o['position'][1]:.3f}) h{1000 * o['size'][2]:.0f}"
        cv2.putText(image, text, (u + 8, v + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(image, text, (u + 8, v + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)

    def project(self, p_world, K=None):
        T = np.linalg.inv(self.cam.world_pose())
        p = T[:3, :3] @ p_world + T[:3, 3]
        if p[2] <= 0:
            return None
        K = K if K is not None else self._K
        return (K @ (p / p[2]))[:2]

    @property
    def _K(self):
        i = yaml.safe_load(EXTRINSICS.read_text())["cameras"][jetson.FIXED]["intrinsics"]
        return np.array([[i["fx"], 0, i["cx"]], [0, i["fy"], i["cy"]], [0, 0, 1.0]])


def fit_axis(xy, rng, r_range=(0.015, 0.06), tol=0.003, iters=300):
    """Upright cylinder axis from its visible side wall: RANSAC circle through the wall points (xy).

    The camera sees about half of the wall, so the arc pins the centre down well; mixed pixels at the
    silhouette and the shiny lid (which bias the top surface) fall outside the arc and are dropped.
    Returns (centre, radius, inlier share, arc span in degrees) or None.
    """
    if len(xy) < 30:
        return None
    best = None
    for _ in range(iters):
        a, b, c = xy[rng.choice(len(xy), 3, replace=False)]
        d = 2 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]))
        if abs(d) < 1e-9:
            continue
        ux = ((a @ a) * (b[1] - c[1]) + (b @ b) * (c[1] - a[1]) + (c @ c) * (a[1] - b[1])) / d
        uy = ((a @ a) * (c[0] - b[0]) + (b @ b) * (a[0] - c[0]) + (c @ c) * (b[0] - a[0])) / d
        centre = np.array([ux, uy])
        radius = float(np.linalg.norm(a - centre))
        if not r_range[0] <= radius <= r_range[1]:
            continue
        inl = np.abs(np.linalg.norm(xy - centre, axis=1) - radius) < tol
        if best is None or inl.sum() > best[0].sum():
            best = (inl, centre, radius)
    if best is None:
        return None
    pts = xy[best[0]]
    # Algebraic least-squares refinement on the inliers: x^2 + y^2 + D x + E y + F = 0.
    A = np.c_[pts, np.ones(len(pts))]
    D, E, F = np.linalg.lstsq(A, -(pts ** 2).sum(axis=1), rcond=None)[0]
    centre = np.array([-D / 2, -E / 2])
    radius = float(np.sqrt(max(centre @ centre - F, 0.0)))
    ang = np.degrees(np.arctan2(pts[:, 1] - centre[1], pts[:, 0] - centre[0]))
    span = float(np.ptp(np.sort(ang))) if len(ang) else 0.0
    return centre, radius, float(best[0].mean()), span


def pixel_to_world(frame, T_world_cam, u, v, window=5):
    """World point of a pixel from the median depth around it."""
    d = frame.depth[max(v - window, 0):v + window + 1, max(u - window, 0):u + window + 1]
    d = d[d > 0]
    if not len(d):
        return None
    z = float(np.median(d))
    K = frame.intrinsics.matrix()
    p = np.array([(u - K[0, 2]) / K[0, 0] * z, (v - K[1, 2]) / K[1, 1] * z, z, 1.0])
    return (T_world_cam @ p)[:3]
