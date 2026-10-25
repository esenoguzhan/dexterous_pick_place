"""Ray-cast colour, depth and object-label images of boxes and cylinders on a table, for the tests.

World frame as in the simulation: z up, table top at TABLE_Z. Cameras use the optical frame
convention (x right, y down, z forward).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation

from restore_perception.core.types import CameraFrame, Intrinsics

TABLE_Z = 0.90
TABLE_HALF = (0.70, 0.50)
TABLE_COLOR = (150, 112, 74)
D435 = Intrinsics(912.5, 910.8, 655.2, 375.6, 1280, 720, "distortion.inverse_brown_conrady", [0.0] * 5)


def look_at(eye, target):
    """4x4 pose of an optical frame at eye looking at target, image 'up' towards world +z."""
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    z = target - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, [0.0, 0.0, 1.0])
    x /= np.linalg.norm(x)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([x, np.cross(z, x), z])
    T[:3, 3] = eye
    return T


@dataclass
class Shape:
    kind: str  # "box" or "cylinder" (axis along local z)
    centre: tuple  # world, metres
    dims: tuple  # box: full length, width, height; cylinder: radius, length
    color: tuple
    yaw_deg: float = 0.0
    lying: bool = False  # cylinder only: axis horizontal, along yaw
    no_depth: bool = False  # the camera returns no depth on it (mirror-like metal)
    R: np.ndarray = field(init=False)

    def __post_init__(self):
        rot = Rotation.from_euler("z", self.yaw_deg, degrees=True)
        if self.lying:
            rot = rot * Rotation.from_euler("y", 90, degrees=True)
        self.R = rot.as_matrix()

    @property
    def resting_size(self):
        """[length, width, height] as it rests, the way objects.yaml lists sizes."""
        if self.kind == "box":
            return np.array(self.dims, float)
        r, length = self.dims
        return np.array([length, 2 * r, 2 * r]) if self.lying else np.array([2 * r, 2 * r, length])


def on_table(kind, dims, x, y, color, yaw_deg=0.0, lying=False, **kw):
    """A shape resting on the table at (x, y)."""
    if kind == "box":
        height = dims[2]
    else:
        height = 2 * dims[0] if lying else dims[1]
    return Shape(kind, (x, y, TABLE_Z + height / 2.0), dims, color, yaw_deg, lying, **kw)


def _hit_box(o, d, half):
    with np.errstate(divide="ignore", invalid="ignore"):
        t1 = (-half - o) / d
        t2 = (half - o) / d
    near = np.nanmax(np.minimum(t1, t2), axis=1)
    far = np.nanmin(np.maximum(t1, t2), axis=1)
    hit = (far >= near) & (far > 0)
    t = np.where(near > 0, near, far)
    normal_axis = np.argmax(np.minimum(t1, t2), axis=1)
    n = np.zeros_like(o)
    n[np.arange(len(o)), normal_axis] = -np.sign(d[np.arange(len(o)), normal_axis])
    return np.where(hit, t, np.inf), n


def _hit_cylinder(o, d, r, half_len):
    a = d[:, 0] ** 2 + d[:, 1] ** 2
    b = 2 * (o[:, 0] * d[:, 0] + o[:, 1] * d[:, 1])
    c = o[:, 0] ** 2 + o[:, 1] ** 2 - r ** 2
    disc = b ** 2 - 4 * a * c
    t_side = np.full(len(o), np.inf)
    ok = (disc >= 0) & (a > 1e-12)
    t0 = np.full(len(o), np.inf)
    t0[ok] = (-b[ok] - np.sqrt(disc[ok])) / (2 * a[ok])
    z0 = o[:, 2] + t0 * d[:, 2]
    side = ok & (t0 > 0) & (np.abs(z0) <= half_len)
    t_side[side] = t0[side]
    t_cap = np.full(len(o), np.inf)
    for sign in (-1.0, 1.0):
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (sign * half_len - o[:, 2]) / d[:, 2]
        p = o + t[:, None] * d
        cap = (t > 0) & (p[:, 0] ** 2 + p[:, 1] ** 2 <= r ** 2)
        t_cap = np.where(cap & (t < t_cap), t, t_cap)
    t = np.minimum(t_side, t_cap)
    p = o + np.where(np.isfinite(t), t, 0.0)[:, None] * d
    n = np.where((t_cap < t_side)[:, None], np.column_stack([np.zeros(len(o)), np.zeros(len(o)), np.sign(p[:, 2])]),
                 np.column_stack([p[:, 0], p[:, 1], np.zeros(len(o))]) / r)
    return t, n


def render(shapes, T_world_cam, intr=D435, noise=True, rng=None, dropout=0.01):
    """Colour (RGB uint8), depth (float32 metres, 0 = no data) and labels (-1 = no object)."""
    rng = rng or np.random.default_rng(0)
    w, h = intr.width, intr.height
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    dirs_c = np.stack([(u - intr.cx) / intr.fx, (v - intr.cy) / intr.fy, np.ones_like(u, float)], -1).reshape(-1, 3)
    R, origin = T_world_cam[:3, :3], T_world_cam[:3, 3]
    d = dirs_c @ R.T  # not normalised: t is then the optical-frame z (depth)
    o = np.broadcast_to(origin, d.shape)

    best = np.full(len(d), np.inf)
    label = np.full(len(d), -1)
    normal = np.zeros_like(d)
    with np.errstate(divide="ignore", invalid="ignore"):
        t_table = (TABLE_Z - origin[2]) / d[:, 2]
    p = origin + t_table[:, None] * d
    table = (t_table > 0) & (np.abs(p[:, 0]) <= TABLE_HALF[0]) & (np.abs(p[:, 1]) <= TABLE_HALF[1])
    best[table] = t_table[table]
    normal[table] = [0, 0, 1]
    label[table] = -2
    for i, s in enumerate(shapes):
        lo = (o - np.asarray(s.centre)) @ s.R  # local frame
        ld = d @ s.R
        if s.kind == "box":
            t, n = _hit_box(lo, ld, np.asarray(s.dims) / 2.0)
        else:
            t, n = _hit_cylinder(lo, ld, s.dims[0], s.dims[1] / 2.0)
        closer = t < best
        best[closer] = t[closer]
        label[closer] = i
        normal[closer] = (n @ s.R.T)[closer]

    light = np.array([0.3, -0.4, 0.85])
    light /= np.linalg.norm(light)
    shade = 0.55 + 0.45 * np.clip(normal @ light, 0, 1)
    color = np.zeros((len(d), 3))
    color[label == -2] = TABLE_COLOR
    for i, s in enumerate(shapes):
        color[label == i] = s.color
    color *= shade[:, None]
    depth = np.where(np.isfinite(best), best, 0.0)
    for i, s in enumerate(shapes):
        if s.no_depth:
            depth[label == i] = 0.0
    if noise:
        color += rng.normal(0, 3.0, color.shape)
        depth = depth + rng.normal(0, 1.0, depth.shape) * (0.0005 + 0.001 * depth ** 2) * (depth > 0)
        depth[rng.random(len(depth)) < dropout] = 0.0
    depth[label == -1] = 0.0  # nothing behind the table in range
    color = np.clip(color, 0, 255).astype(np.uint8).reshape(h, w, 3)
    labels = np.where(label >= 0, label, -1).reshape(h, w)
    return color, depth.astype(np.float32).reshape(h, w), labels


def frames(shapes, cameras, rng=None, noise=True):
    """CameraFrames with poses, and the ground-truth label image per camera name."""
    rng = rng or np.random.default_rng(0)
    out, labels = [], {}
    for name, (serial, T) in cameras.items():
        color, depth, lab = render(shapes, T, noise=noise, rng=rng)
        out.append(CameraFrame(name, serial, color, depth, D435, 1000.0, T.copy(), 10))
        labels[name] = lab
    return out, labels


# Fixed camera at the side of the table, about 45 degrees from vertical; hand camera from the observe
# pose on the other side, steeper.
CAMERAS = {
    "camera_1": ("263522073705", look_at([0.45, -0.70, 1.55], [0.0, 0.0, TABLE_Z])),
    "camera_2": ("262722075125", look_at([-0.30, 0.35, 1.45], [0.0, 0.0, TABLE_Z])),
}


class FakeSegmenter:
    """Answers SAM prompts from the ground-truth labels. A box gets the single object with the most
    pixels inside it (so touching objects must be split), a point gets the object under it."""

    def __init__(self, labels, frames=None):
        self.labels = labels
        self.current = None
        self.calls = {"box": 0, "point": 0}
        # automask gets only the image, like SAM's generator: find the camera by its colour buffer.
        self._by_image = {f.color.ctypes.data: labels[f.name] for f in frames or []}

    def set_image(self, key, image):
        self.current = self.labels[key[1]]

    def predict(self, box=None, points=None, labels=None, multimask=True):
        lab = self.current
        if box is not None:
            self.calls["box"] += 1
            x0, y0, x1, y1 = (int(v) for v in box)
            ids, counts = np.unique(lab[y0:y1 + 1, x0:x1 + 1], return_counts=True)
            keep = ids >= 0
            if not keep.any():
                return np.zeros((0,) + lab.shape, bool), np.zeros(0, np.float32)
            obj = ids[keep][np.argmax(counts[keep])]
        else:
            self.calls["point"] += 1
            u, v = (int(c) for c in points[0])
            obj = lab[v, u]
            if obj < 0:
                return np.zeros((0,) + lab.shape, bool), np.zeros(0, np.float32)
        return (lab == obj)[None], np.array([0.95], np.float32)

    def automask(self, image, points_per_side):
        lab = self._by_image[image.ctypes.data]
        return [((lab == i), 0.95) for i in np.unique(lab) if i >= 0]


class FakeEmbedder:
    """Hue-saturation histogram of the crop as the 'embedding': distinct colours, distinct objects."""

    def embed(self, crops):
        from restore_perception.core.embed import crop_histogram

        rows = [crop_histogram(c) for c in crops]
        rows = [r / (np.linalg.norm(r) or 1.0) for r in rows]
        return np.array(rows, np.float32).reshape(len(crops), -1)
