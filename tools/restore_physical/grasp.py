"""Palm-down top grasp geometry for the Inspire hand (right_hand_base_link frame).

Hand frame: +z along the fingers (palm centre z = 0.12 m, fingertips z = 0.24 m), +y out of the
palm, +x towards the little finger (the thumb sits at -x). For a top grasp the palm faces down
(hand +y = world -z) and the fingers point away from the arm base, so the arm reaches straight out.
The object's top centre sits under the palm between its centre and the finger roots.
"""

from __future__ import annotations

import math

import numpy as np
import yaml
from pathlib import Path

CONFIG = Path(__file__).resolve().parent / "config" / "grasp.yaml"
ARM_BASE_XY = np.array([-0.32, 0.0])  # world, main's URDF


def settings():
    return yaml.safe_load(CONFIG.read_text())


def radial_yaw(xy):
    d = np.asarray(xy[:2]) - ARM_BASE_XY
    return math.atan2(d[1], d[0])


def hand_rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    z = np.array([c, s, 0.0])          # fingers
    y = np.array([0.0, 0.0, -1.0])     # palm normal, down
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


_SHAPE = {}


def underside(cfg):
    """Height map of the open hand's underside (hand_shape.py) with the wrist camera box added."""
    if "depth" not in _SHAPE:
        d = np.load(Path(__file__).resolve().parent / "config" / "hand_shape_open.npz")
        depth = d["depth"].copy()
        x0, z0, g = float(d["x0"]), float(d["z0"]), float(d["grid"])
        xs = x0 + g * (np.arange(depth.shape[0]) + 0.5)
        zs = z0 + g * (np.arange(depth.shape[1]) + 0.5)
        (cx0, cx1), (cy0, cy1), (cz0, cz1) = cfg["camera_box"]
        cam = (xs[:, None] >= cx0) & (xs[:, None] <= cx1) & (zs[None, :] >= cz0) & (zs[None, :] <= cz1)
        depth[cam] = np.maximum(depth[cam], cy1)
        _SHAPE.update(depth=depth, xs=xs, zs=zs)
    return _SHAPE["depth"], _SHAPE["xs"], _SHAPE["zs"]


def clearance_under(px, pz, radius, cfg):
    """How far the hand (and wrist camera) reach out of the palm over an object's top disk."""
    depth, xs, zs = underside(cfg)
    reach = radius + cfg["side_margin_m"]
    inside = (xs[:, None] - px) ** 2 + (zs[None, :] - pz) ** 2 <= reach ** 2
    return float(depth[inside].max()) if inside.any() else -1.0


def palm_point(radius, cfg=None):
    """Hand-frame point for the object's top centre, or None when the hand would land on it.

    The grip point is where the thumb tip meets the index and middle fingertips when they close:
    about grip_point in the hand frame. The top goes there unless the hand underside (thumb base,
    wrist camera) reaches deeper over the object's disk, in which case it goes that much lower.
    """
    cfg = cfg or settings()
    px, py, pz = cfg["grip_point"]
    py = max(py, clearance_under(px, pz, radius, cfg) + cfg["top_margin_m"])
    return None if py > cfg["palm_y_max"] else (float(px), float(py), float(pz))


def hand_pose(top_point, yaw, clearance, radius=0.035, cfg=None):
    """T_world_hand with the object's top centre at the palm point, `clearance` m below it."""
    cfg = cfg or settings()
    point = palm_point(radius, cfg)
    if point is None:
        raise ValueError(f"no grasp point under the palm for a {2000 * radius:.0f} mm object")
    px, py, pz = point
    R = hand_rotation(yaw)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(top_point, float) - R @ np.array([px, py + clearance, pz])
    return T


def flange_pose(T_world_hand, T_flange_hand):
    return T_world_hand @ np.linalg.inv(T_flange_hand)
