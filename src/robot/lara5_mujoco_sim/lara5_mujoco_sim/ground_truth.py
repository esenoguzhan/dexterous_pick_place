"""Ground-truth poses of the free objects in the MuJoCo scene (for perception tests).

The simulator publishes them as JSON on /sim/objects (std_msgs/String) so that a perception
evaluation can compare its estimates with the true object poses without parsing the sim config.
"""

from __future__ import annotations

import json
import math

import mujoco
import numpy as np

SYM_CONTINUOUS = "continuous"


def object_dimensions(obj):
    """Full [length, width, height] in metres of a configured object (MuJoCo sizes are half extents)."""
    size = [float(v) for v in obj["size"]]
    kind = obj["type"]
    if kind == "box":
        return [2 * size[0], 2 * size[1], 2 * size[2]]
    if kind == "cylinder":
        return [2 * size[0], 2 * size[0], 2 * size[1]]
    if kind == "sphere":
        return [2 * size[0]] * 3
    raise ValueError(f"unsupported object type {kind!r}")


def yaw_symmetry(obj):
    """Yaw symmetry of the footprint: 'continuous' for round objects, 4 for a square box, else 2."""
    if obj["type"] in ("cylinder", "sphere"):
        return SYM_CONTINUOUS
    length, width, _ = object_dimensions(obj)
    return 4 if abs(length - width) < 1e-6 else 2


def body_quaternion_wxyz(obj):
    """MuJoCo body quaternion for the optional yaw_deg of a configured object."""
    yaw = math.radians(float(obj.get("yaw_deg", 0.0)))
    return [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]


def yaw_from_matrix(rotation):
    """Rotation about world Z of a body: direction of its local +X axis in the XY plane."""
    return math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))


def long_axis_yaw(obj, yaw):
    """Yaw of the footprint's long side (the perception convention), given the body yaw."""
    length, width, _ = object_dimensions(obj)
    return yaw + (math.pi / 2 if width > length + 1e-6 else 0.0)


def ground_truth(model, data, objects, stamp=None):
    """Dict with every configured object's true pose, dimensions and yaw symmetry."""
    entries = []
    for obj in objects:
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, obj["name"])
        if body < 0:
            continue
        rotation = np.asarray(data.xmat[body]).reshape(3, 3)
        w, x, y, z = (float(v) for v in data.xquat[body])
        yaw = yaw_from_matrix(rotation)
        entries.append({
            "name": obj["name"],
            "type": obj["type"],
            "size_m": object_dimensions(obj),
            "position": [float(v) for v in data.xpos[body]],
            "orientation_xyzw": [x, y, z, w],
            "yaw_rad": yaw,
            "long_axis_yaw_rad": long_axis_yaw(obj, yaw),
            "yaw_symmetry": yaw_symmetry(obj),
        })
    return {"frame_id": "world", "stamp": stamp, "objects": entries}


def ground_truth_json(model, data, objects, stamp=None):
    return json.dumps(ground_truth(model, data, objects, stamp))
