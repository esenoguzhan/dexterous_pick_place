"""Orchestrator parameters (docs/task_architecture.md, section 11).

DEFAULTS holds every parameter; config/task.yaml repeats them with comments. A file may list only
what it changes, and an unknown key is an error. Values marked "measure" in task.yaml are
placeholders until they are measured on the real hand.
"""

from __future__ import annotations

import copy
import math
from pathlib import Path

import yaml

# Hand joint order used everywhere in this package: thumb rotation, thumb bend, then the fingers.
_THUMB_OPPOSED = 1.16

DEFAULTS = {
    "world_frame": "world",
    "flange_frame": "lara5_flange",
    "tcp_frame": "right_hand_base_link",
    # flange -> palm frame from the URDF (right_hand_mount_joint); TF replaces it when available.
    "flange_to_tcp": {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, -math.pi / 2]},
    # The observe pose: the arm out of both camera views (perception section 5.0). Measure.
    "observe_joints": {"lara5_joint1": 0.0, "lara5_joint2": 0.0, "lara5_joint3": 1.5708,
                       "lara5_joint4": 0.0, "lara5_joint5": 1.5708, "lara5_joint6": 0.0},
    "observe_tolerance_rad": 0.01,
    "joint_speed_rad_s": 0.5,
    "reach_area": {"min": [-0.55, -0.35], "max": [0.55, 0.35]},  # world xy the hand can reach. Measure.
    "default_primitive": "wrap_top",  # for objects without a grasp in objects.yaml
    "diff": {  # section 5
        "moved_distance_m": 0.015,
        "moved_yaw_deg": 10.0,
        "noise_changed": 0.1,  # change-mask share under which small displacements are noise
        "min_confidence": 0.5,
        "unknown_min_cosine": 0.7,
        "unknown_size_tolerance": 0.3,
        "icp_iterations": 30,
        "icp_ambiguity": 0.2,
        "icp_min_points": 30,
    },
    "order": {  # section 6
        "home_clearance_m": 0.01,
        "grid_m": 0.005,
        "parking_step_m": 0.02,
    },
    "pick_place": {  # section 7
        "approach_mode": "move_to_pose",  # or approach_node (lara5_object_approach, fixed offset)
        "pre_distance_m": 0.10,
        "descend_duration_s": 3.0,
        "release_height_m": 0.005,
        "max_attempts": 2,
        "hand_radius_m": 0.06,  # neighbours closer than this to the grasp centre rank a grasp last
        "hand_close_s": 1.0,
        "hand_open_s": 1.0,
        "slow_open_s": 2.5,  # objects that roll
    },
    "primitives": {  # section 7.2; every number here is a placeholder to measure on the real hand
        "wrap_top": {
            "approach": "top", "palm_height_m": 0.15, "tcp_rpy": [math.pi, 0.0, 0.0], "yaws_deg": [0.0, 180.0],
            "fingers": ["thumb", "index", "middle", "ring", "pinky"], "open_margin_m": 0.025, "squeeze_m": 0.008,
            "width_to_angles": [[0.02, [_THUMB_OPPOSED, 0.50, 1.30, 1.30, 1.30, 1.30]],
                                [0.10, [_THUMB_OPPOSED, 0.10, 0.30, 0.30, 0.30, 0.30]]],
        },
        "wrap_wide": {
            "approach": "top", "palm_height_m": 0.13, "tcp_rpy": [math.pi, 0.0, 0.0], "yaws_deg": [0.0, 180.0],
            "fingers": ["thumb", "index", "middle", "ring", "pinky"], "open_margin_m": 0.025, "squeeze_m": 0.012,
            "width_to_angles": [[0.04, [_THUMB_OPPOSED, 0.45, 1.20, 1.20, 1.20, 1.20]],
                                [0.12, [_THUMB_OPPOSED, 0.05, 0.10, 0.10, 0.10, 0.10]]],
        },
        "pinch_wide": {
            "approach": "top", "palm_height_m": 0.16, "tcp_rpy": [math.pi, 0.0, 0.0], "yaws_deg": [0.0, 180.0],
            "fingers": ["thumb", "index", "middle"], "open_margin_m": 0.025, "squeeze_m": 0.008,
            "width_to_angles": [[0.02, [_THUMB_OPPOSED, 0.50, 1.20, 1.20, 1.43, 1.43]],
                                [0.10, [_THUMB_OPPOSED, 0.10, 0.25, 0.25, 1.43, 1.43]]],
        },
        "tripod_top": {
            "approach": "top", "palm_height_m": 0.16, "tcp_rpy": [math.pi, 0.0, 0.0], "yaws_deg": [0.0, 180.0],
            "fingers": ["thumb", "index", "middle"], "open_margin_m": 0.02, "squeeze_m": 0.006,
            "width_to_angles": [[0.02, [_THUMB_OPPOSED, 0.50, 1.25, 1.25, 1.43, 1.43]],
                                [0.08, [_THUMB_OPPOSED, 0.15, 0.40, 0.40, 1.43, 1.43]]],
        },
        "wrap_body": {
            # From the side: the palm frame's z points horizontally at the object, away from the handle.
            "approach": "side", "palm_distance_m": 0.05, "tcp_rpy": [0.0, -math.pi / 2, 0.0],
            "yaws_deg": [90.0, -90.0],
            "fingers": ["thumb", "index", "middle", "ring", "pinky"], "open_margin_m": 0.03, "squeeze_m": 0.008,
            "width_to_angles": [[0.04, [_THUMB_OPPOSED, 0.45, 1.20, 1.20, 1.20, 1.20]],
                                [0.10, [_THUMB_OPPOSED, 0.10, 0.30, 0.30, 0.30, 0.30]]],
        },
    },
    "objects": {},  # per label: {primitive: ..., squeeze_m: ...}
    "grasp_check": {  # section 7.6, as shares of each finger's squeeze span
        "held_ratio": 0.5,  # a finger stopped at least this share of the span short of its target holds
        "missed_ratio": 0.2,  # every finger within this share of its target: nothing between them
        "drop_ratio": 0.5,  # a held finger that closes this share of the span further: dropped
        "settle_s": 0.3,
    },
    "budget_s": 600.0,
    "capture_retries": 2,
}

_OPEN_KEYS = {("observe_joints",), ("primitives",), ("objects",)}


def _merge(base, override, path=()):
    for key, value in override.items():
        here = path + (key,)
        if here in _OPEN_KEYS:  # free-form maps: entries are added, or updated key by key
            if not isinstance(value, dict):
                raise ValueError(f"{'.'.join(here)} must be a mapping")
            merged = dict(base.get(key) or {})
            for name, item in value.items():
                if isinstance(item, dict) and isinstance(merged.get(name), dict):
                    merged[name] = {**merged[name], **item}
                else:
                    merged[name] = item
            base[key] = merged
        elif key not in base:
            raise ValueError(f"unknown parameter {'.'.join(here)}")
        elif isinstance(base[key], dict):
            if not isinstance(value, dict):
                raise ValueError(f"{'.'.join(here)} must be a mapping")
            _merge(base[key], value, here)
        else:
            base[key] = value


def load_config(path=None, overrides=None):
    config = copy.deepcopy(DEFAULTS)
    if path:
        _merge(config, yaml.safe_load(Path(path).read_text()) or {})
    if overrides:
        _merge(config, overrides)
    return config


def config_dir():
    """config/ of the installed package (ament index), else of the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("restore_task")) / "config"
    except Exception:
        return Path(__file__).resolve().parents[2] / "config"


def objects_file():
    """restore_perception's objects.yaml, the one list of object sizes, symmetries and grasps."""
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("restore_perception")) / "config" / "objects.yaml"
    except Exception:
        return Path(__file__).resolve().parents[3] / "restore_perception" / "config" / "objects.yaml"


def primitive_for(label, objects, config):
    """Name and parameters of the grasp primitive of an object, with per-object overrides."""
    override = config["objects"].get(label, {})
    name = override.get("primitive") or (objects.get(label) or {}).get("grasp") or config["default_primitive"]
    if name not in config["primitives"]:
        raise ValueError(f"{label}: grasp primitive {name!r} is not in the primitives list")
    params = dict(config["primitives"][name])
    params.update({k: v for k, v in override.items() if k != "primitive"})
    return name, params
