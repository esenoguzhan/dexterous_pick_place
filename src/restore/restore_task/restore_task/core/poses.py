"""Small pose helpers: 4x4 transforms, yaw about world z, angle wrapping."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation


def wrap(angle, period=2 * math.pi):
    """Angle wrapped to (-period/2, period/2]."""
    half = period / 2.0
    wrapped = (angle + half) % period - half
    return half if math.isclose(wrapped, -half) else wrapped


def rot_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def transform(rotation=None, translation=(0.0, 0.0, 0.0)):
    T = np.eye(4)
    if rotation is not None:
        T[:3, :3] = rotation
    T[:3, 3] = translation
    return T


def from_xyz_rpy(xyz, rpy):
    return transform(Rotation.from_euler("xyz", rpy).as_matrix(), xyz)


def from_xyz_quat(xyz, quat_xyzw):
    return transform(Rotation.from_quat(quat_xyzw).as_matrix(), xyz)


def to_xyz_quat(T):
    return T[:3, 3].copy(), Rotation.from_matrix(T[:3, :3]).as_quat()


def inverse(T):
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def world_yaw(rotation):
    """Yaw about world z of a rotation matrix or xyzw quaternion: its x axis projected onto xy."""
    R = Rotation.from_quat(rotation).as_matrix() if np.shape(rotation) == (4,) else np.asarray(rotation)
    return math.atan2(R[1, 0], R[0, 0])


def lifted(T, dz):
    """The same pose moved by dz along world z."""
    out = T.copy()
    out[2, 3] += dz
    return out


@dataclass
class Placement:
    """An object pose on the table: centre in world, yaw about world z."""

    x: float
    y: float
    z: float
    yaw: float

    def matrix(self):
        return transform(rot_z(self.yaw), (self.x, self.y, self.z))

    @property
    def xy(self):
        return np.array([self.x, self.y])

    def as_dict(self):
        return {"x": round(self.x, 4), "y": round(self.y, 4), "z": round(self.z, 4),
                "yaw_deg": round(math.degrees(self.yaw), 1)}
