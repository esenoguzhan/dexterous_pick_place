"""Scenes the orchestrator works on: perception snapshots from the capture service or from disk.

No ROS imports: a SceneSnapshot message is read field by field, so tests can pass any object with
the same attributes.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from restore_perception.core.types import ObjectState

from .poses import world_yaw


@dataclass
class Scene:
    snapshot_id: str
    table_plane: np.ndarray  # a, b, c, d; normal up
    objects: list  # restore_perception ObjectState

    def by_id(self):
        return {o.id: o for o in self.objects}

    def table_z(self, x, y):
        a, b, c, d = self.table_plane
        return float(-(a * x + b * y + d) / c)


def yaw_of(obj):
    """Yaw about world z (perception's yaw is about the table normal, within a degree of it)."""
    return world_yaw(np.asarray(obj.orientation_xyzw, float))


def cloud_points(cloud):
    """x, y, z of a PointCloud2 with float32 x, y, z fields."""
    if cloud is None or not cloud.width * cloud.height:
        return np.zeros((0, 3), np.float32)
    offsets = {f.name: f.offset for f in cloud.fields}
    raw = np.frombuffer(bytes(cloud.data), np.uint8).reshape(-1, cloud.point_step)
    dtype = np.dtype(">f4" if cloud.is_bigendian else "<f4")
    return np.stack([raw[:, offsets[k]:offsets[k] + 4].copy().view(dtype)[:, 0] for k in "xyz"], axis=1)


def from_msg(msg):
    objects = []
    for m in msg.objects:
        p, q = m.pose.position, m.pose.orientation
        objects.append(ObjectState(
            id=m.id, label=m.label, label_score=float(m.label_score),
            position=np.array([p.x, p.y, p.z]), orientation_xyzw=np.array([q.x, q.y, q.z, q.w]),
            size=np.array([m.size.x, m.size.y, m.size.z]),
            footprint=np.array([[f.x, f.y, f.z] for f in m.footprint]).reshape(-1, 3),
            symmetry=int(m.symmetry), up_side_matters=bool(m.up_side_matters), up_side=m.up_side,
            support_id=m.support_id, embedding=np.asarray(m.embedding, np.float32),
            points=cloud_points(m.cloud), cameras_seen=int(m.cameras_seen),
            depth_reliable=bool(m.depth_reliable), confidence=float(m.confidence)))
    return Scene(msg.snapshot_id, np.asarray(msg.table_plane, float), objects)


def from_disk(folder):
    """A snapshot saved by perception (data/snapshots/<id>/)."""
    from restore_perception.core.snapshot_io import load_snapshot

    snap = load_snapshot(folder)
    return Scene(snap.snapshot_id, np.asarray(snap.table_plane, float), snap.objects)
