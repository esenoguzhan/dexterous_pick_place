"""Hand-made scenes for the core tests: objects as perception reports them."""

from __future__ import annotations

import math

import numpy as np
from restore_perception.core.types import SYM_CONTINUOUS, SYM_NONE, ObjectState
from scipy.spatial.transform import Rotation

from restore_task.core.scene import Scene

TABLE_Z = 0.90
PLANE = np.array([0.0, 0.0, 1.0, -TABLE_Z])

# label: size [m], symmetry, embedding axis
KINDS = {
    "wooden_cube": ([0.055, 0.055, 0.055], 4, 0),
    "sponge": ([0.090, 0.060, 0.045], 2, 1),
    "tuna_can": ([0.085, 0.085, 0.040], SYM_CONTINUOUS, 2),
    "mouse": ([0.115, 0.075, 0.045], SYM_NONE, 3),
    "soup_can": ([0.110, 0.073, 0.073], 2, 4),
    "wooden_egg": ([0.060, 0.045, 0.045], 2, 5),
}


def box_cloud(size, bump=False, seed=0):
    """Points on the top and sides of a box centred at the origin; a bump at +x breaks its symmetry.
    Different seeds sample different points, like two independent scans."""
    rng = np.random.default_rng(seed)
    l, w, h = size
    pts = [np.column_stack([rng.uniform(-l / 2, l / 2, 400), rng.uniform(-w / 2, w / 2, 400), np.full(400, h / 2)])]
    for sx in (-1, 1):
        pts.append(np.column_stack([np.full(150, sx * l / 2), rng.uniform(-w / 2, w / 2, 150),
                                    rng.uniform(-h / 2, h / 2, 150)]))
    if bump:
        pts.append(np.column_stack([rng.uniform(l / 2 - 0.02, l / 2, 200), rng.uniform(-w / 4, w / 4, 200),
                                    np.full(200, h / 2 + 0.02)]))
    return np.vstack(pts)


def obj(oid, label, x, y, yaw_deg=0.0, support="table", z=None, reported_yaw_deg=None, changed=None,
        confidence=0.9, up_side="", labelled=True, embedding=True):
    """An object resting at (x, y). Perception reports yaw modulo 180 degrees (footprint long side);
    reported_yaw_deg overrides what it reports, the cloud always has the true yaw."""
    size, symmetry, axis = KINDS[label]
    yaw = math.radians(yaw_deg)
    reported = math.radians(reported_yaw_deg if reported_yaw_deg is not None else yaw_deg)
    reported = (reported + math.pi / 2) % math.pi - math.pi / 2
    z = TABLE_Z + size[2] / 2 if z is None else z
    R = Rotation.from_euler("z", yaw).as_matrix()
    corners = np.array([[1, 1], [-1, 1], [-1, -1], [1, -1]]) * np.array(size[:2]) / 2
    footprint = np.column_stack([corners @ R[:2, :2].T + [x, y], np.full(4, TABLE_Z)])
    emb = np.zeros(8, np.float32)
    emb[axis] = 1.0
    return ObjectState(
        id=oid, label=label if labelled else "unknown", label_score=0.9,
        position=np.array([x, y, z]), orientation_xyzw=Rotation.from_euler("z", reported).as_quat(),
        yaw=reported, size=np.array(size), footprint=footprint, symmetry=symmetry,
        up_side_matters=label == "sponge", up_side=up_side, support_id=support,
        embedding=emb if embedding else np.zeros(0, np.float32),
        points=(box_cloud(size, bump=label == "mouse") @ R.T + [x, y, z]).astype(np.float32),
        confidence=confidence, changed=changed)


def scene(name, *objects):
    return Scene(name, PLANE.copy(), list(objects))
