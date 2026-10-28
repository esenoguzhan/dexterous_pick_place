"""Forward kinematics of world -> lara5_flange straight from the URDF the robot runs (robot_description).

The safety filter's FK service refuses configurations near a collision, so an IK that steps through
it gives up on the first intermediate pose near the table. This chain has no collision check; the
IK result is checked by the safety filter afterwards.
"""

from __future__ import annotations

import subprocess
import xml.etree.ElementTree as ET
from functools import lru_cache

import numpy as np
from scipy.spatial.transform import Rotation

ARM = [f"lara5_joint{i}" for i in range(1, 7)]


def _origin(el):
    o = el.find("origin")
    xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
    rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    T[:3, 3] = xyz
    return T


@lru_cache(maxsize=1)
def chain(tip="lara5_flange", root="world"):
    """[(fixed transform, axis or None, joint name)] from root to tip."""
    out = subprocess.run(["ros2", "param", "get", "/robot_state_publisher", "robot_description", "--hide-type"],
                         capture_output=True, text=True, timeout=30).stdout
    urdf = ET.fromstring(out[out.index("<"):])
    by_child = {j.find("child").get("link"): j for j in urdf.findall("joint")}
    joints, link = [], tip
    while link != root:
        j = by_child[link]
        axis = None
        if j.get("type") in ("revolute", "continuous"):
            a = j.find("axis")
            axis = np.array([float(v) for v in (a.get("xyz") if a is not None else "1 0 0").split()])
        joints.append((_origin(j), axis, j.get("name")))
        link = j.find("parent").get("link")
    return tuple(reversed(joints))


def fk(q, tip="lara5_flange"):
    values = dict(zip(ARM, q))
    T = np.eye(4)
    for T_origin, axis, name in chain(tip):
        T = T @ T_origin
        if axis is not None:
            R = np.eye(4)
            R[:3, :3] = Rotation.from_rotvec(axis * values.get(name, 0.0)).as_matrix()
            T = T @ R
    return T
