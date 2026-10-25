"""Data passed between the pipeline stages. Plain dataclasses and NumPy, no ROS."""

from __future__ import annotations

import re
from dataclasses import dataclass, field

import numpy as np

# ObjectState.symmetry: these two codes, or k >= 2 for k-fold yaw symmetry.
SYM_CONTINUOUS = 0
SYM_NONE = 1


class PerceptionError(RuntimeError):
    """A capture or a pipeline stage failed; the message is meant for the caller."""


@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    model: str = ""
    coeffs: list = field(default_factory=lambda: [0.0] * 5)

    def matrix(self):
        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])

    def to_dict(self):
        return {"fx": self.fx, "fy": self.fy, "cx": self.cx, "cy": self.cy, "width": self.width,
                "height": self.height, "model": self.model, "coeffs": list(self.coeffs)}

    @classmethod
    def from_dict(cls, d):
        return cls(float(d["fx"]), float(d["fy"]), float(d["cx"]), float(d["cy"]), int(d["width"]),
                   int(d["height"]), str(d.get("model", "")), list(d.get("coeffs") or [0.0] * 5))


@dataclass
class CameraFrame:
    """One camera's capture (5.1): the newest colour frame, the median depth, intrinsics and pose."""

    name: str  # ROS camera name, e.g. camera_1
    serial: str
    color: np.ndarray  # H x W x 3, RGB, uint8
    depth: np.ndarray  # H x W, metres, float32, 0 = no data; on the colour pixel grid
    intrinsics: Intrinsics
    stamp: float  # seconds since the epoch
    T_world_cam: np.ndarray | None = None  # 4x4 pose of the colour optical frame in world
    frames: int = 1  # depth frames in the median
    optical_frame: str = ""  # TF frame of the image when a ROS topic names it

    @property
    def frame_id(self):
        return self.optical_frame or f"{self.name}_color_optical_frame"


def camera_bit(name, index):
    """cameras_seen bit of a camera: camera_N gives 1 << (N - 1), others their list position."""
    match = re.fullmatch(r"camera_(\d+)", name)
    position = int(match.group(1)) - 1 if match else index
    return 1 << min(max(position, 0), 7)


@dataclass
class ObjectState:
    """One object of a snapshot; the fields of restore_interfaces/ObjectState plus internals."""

    id: str
    label: str = "unknown"
    label_score: float = 0.0
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    orientation_xyzw: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))
    yaw: float = 0.0  # about the table normal [rad], long side; defined modulo 180 degrees
    size: np.ndarray = field(default_factory=lambda: np.zeros(3))  # length, width, height [m]
    footprint: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))  # polygon, world frame
    symmetry: int = SYM_NONE
    up_side_matters: bool = False
    up_side: str = ""
    support_id: str = "table"
    embedding: np.ndarray = field(default_factory=lambda: np.zeros(0, np.float32))
    points: np.ndarray = field(default_factory=lambda: np.zeros((0, 3), np.float32))  # world frame
    cameras_seen: int = 0
    depth_reliable: bool = True
    confidence: float = 0.0
    # Not in the message.
    masks: dict = field(default_factory=dict)  # camera name -> H x W bool
    sam_score: float | None = None
    known_size: bool = False  # pose from the known-size fallback (5.8)
    depth_poor: bool = False  # found by SAM's automatic masks, not by geometry (5.5)
    match: dict = field(default_factory=dict)  # gallery details: candidate, margin, top, view
    bottom_height: float = 0.0  # lowest points above the table [m]; > 0 when resting on something
    changed: float | None = None  # share of the mask that changed since the reference (6); None: no reference


@dataclass
class Snapshot:
    snapshot_id: str
    stamp: float
    table_plane: np.ndarray  # a, b, c, d with a*x + b*y + c*z + d = 0, normal pointing up
    objects: list
    frames: list
    warnings: list = field(default_factory=list)
    timings: dict = field(default_factory=dict)  # stage -> milliseconds
    table_height: float = 0.0  # plane height at the workspace centre [m]
    changed_fraction: dict = field(default_factory=dict)  # camera name -> share of changed pixels (6)
