"""Camera poses from the calibration file (section 3.4), keyed by serial number.

The file is extrinsics.yaml as the test bench's calibration page saves it. Each camera entry is the
transform parent_frame -> frame_id: `world` for a fixed camera, the flange for a camera on the hand.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from .geometry import pose_matrix
from .types import PerceptionError


@dataclass
class CameraExtrinsics:
    serial: str
    name: str
    mount: str  # "fixed" or "hand"
    parent_frame: str
    frame_id: str
    T_parent_cam: np.ndarray

    @property
    def on_hand(self):
        return self.mount == "hand"

    def world_pose(self, T_world_flange=None):
        """Pose of the colour optical frame in world; a hand camera needs the flange pose."""
        if not self.on_hand:
            return self.T_parent_cam
        if T_world_flange is None:
            raise PerceptionError(f"{self.name} is on the hand; its pose needs the flange pose from TF")
        return T_world_flange @ self.T_parent_cam


def load_extrinsics(path):
    """serial -> CameraExtrinsics; an empty dict when the file does not exist."""
    if not path or not Path(path).exists():
        return {}
    data = yaml.safe_load(Path(path).read_text()) or {}
    cameras = {}
    for serial, cam in (data.get("cameras") or {}).items():
        cameras[str(serial)] = CameraExtrinsics(
            serial=str(serial),
            name=cam["name"],
            mount=cam.get("mount", "fixed"),
            parent_frame=cam["parent_frame"],
            frame_id=cam.get("frame_id", f"{cam['name']}_color_optical_frame"),
            T_parent_cam=pose_matrix(cam["translation"], cam["rotation_xyzw"]),
        )
    return cameras
