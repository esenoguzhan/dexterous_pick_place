"""Pipeline parameters (docs/perception_architecture.md) and objects.yaml (section 7.2).

DEFAULTS holds every parameter. config/perception.yaml repeats them with comments; a file only has to
list the values it changes, and an unknown key is an error, so a typo cannot silently fall back to a
default.
"""

from __future__ import annotations

import copy
from pathlib import Path

import yaml

from .types import SYM_CONTINUOUS, SYM_NONE

DEFAULTS = {
    "world_frame": "world",
    "flange_frame": "lara5_flange",
    # Serial number to ROS camera name; defaults follow compose.realsense.yaml.
    "cameras": {"263522073705": "camera_1", "262722075125": "camera_2"},
    "capture": {  # 3.3 and 5.1
        "frames": 10,
        "min_valid_fraction": 0.5,
        "timeout_s": 5.0,
        "color_size": [1280, 720],
        "depth_size": [848, 480],
        "fps": 15,
        "laser_power": 250.0,
        "min_depth_m": 0.15,
        "max_depth_m": 2.0,
    },
    "workspace": {  # 5.2, world frame; simulation values, measure the real table
        "min": [-0.6, -0.4, 0.88],
        "max": [0.6, 0.4, 1.30],
        "pixel_stride": 2,  # back-project every 2nd pixel in each direction; masks stay full resolution
    },
    "table": {  # 5.3
        "voxel_m": 0.005,
        "distance_threshold_m": 0.008,
        "ransac_iterations": 1000,
        "max_tilt_deg": 10.0,
        "max_height_change_m": 0.01,
        "object_min_height_m": 0.01,
    },
    "cluster": {  # 5.4
        "voxel_m": 0.004,
        "eps_m": 0.015,
        "min_points": 20,  # neighbouring voxels for a core voxel, itself included
        "min_cluster_points": 50,  # full-resolution points
        "max_extent_m": 0.30,
        "min_height_m": 0.015,
        "min_camera_points": 30,  # a camera sees a candidate when it has this many of its points
        "box_padding_px": 10,
    },
    "segment": {  # 5.5
        "enabled": True,
        "mask_min_height_m": 0.005,
        "min_mask_points": 30,
        "split_coverage": 0.7,
        "max_splits": 3,
        "min_split_px": 300,
        "depth_poor": {
            "enabled": False,  # about 7 s per camera on the Orin Nano
            "cameras": [],  # camera names; empty = every camera
            "points_per_side": 24,
            "min_area_frac": 0.0005,
            "max_area_frac": 0.05,
            "max_overlap": 0.1,
            "assumed_height_m": 0.03,
        },
    },
    "fuse": {  # 5.6
        "max_distance_m": 0.02,
        "min_iou": 0.3,
        "grid_m": 0.005,
        "outlier_neighbors": 20,
        "outlier_std_ratio": 2.0,
    },
    "identify": {  # 5.7
        "enabled": True,
        "min_score": 0.6,
        "min_margin": 0.05,
        "size_tolerance": 0.3,
        "histogram_margin": 0.1,
    },
    "describe": {  # 5.8
        "height_percentile": 95.0,
        "min_points": 200,  # fewer points: known-size fallback
        "support_min_bottom_m": 0.015,
        "footprint_epsilon_m": 0.003,
        "cloud_voxel_m": 0.003,
        "hull_grid_m": 0.002,
    },
    "change": {  # 6
        "depth_threshold_m": 0.01,
        "color_threshold": 25.0,  # Lab distance
        "min_region_px": 200,
        "bumped_fraction": 0.5,
    },
    "observe": {  # 5.0, used by the ROS node
        "require_observe_pose": False,
        "joints": {},  # joint name: position [rad]
        "tolerance_rad": 0.01,
        "max_speed_rad_s": 0.01,
        "settle_s": 0.5,
        "max_age_s": 0.5,
    },
    "models": {
        "sam2_root": "/opt/sam2",
        "sam2_checkpoint": "/models/sam2.1_hiera_small.pt",
        "sam2_config": "configs/sam2.1/sam2.1_hiera_s.yaml",
        "dino_dir": "/models/dinov2-small",
    },
}

# Keys whose values are free-form mappings rather than fixed parameter groups.
_OPEN_KEYS = {("cameras",), ("observe", "joints")}


def _merge(base, override, path=()):
    for key, value in override.items():
        here = path + (key,)
        if key not in base:
            raise ValueError(f"unknown parameter {'.'.join(here)}")
        if isinstance(base[key], dict) and here not in _OPEN_KEYS:
            if not isinstance(value, dict):
                raise ValueError(f"{'.'.join(here)} must be a mapping")
            _merge(base[key], value, here)
        else:
            base[key] = value


def load_config(path=None, overrides=None):
    """DEFAULTS, updated from a YAML file and then from a dict."""
    config = copy.deepcopy(DEFAULTS)
    if path:
        data = yaml.safe_load(Path(path).read_text()) or {}
        _merge(config, data)
    if overrides:
        _merge(config, overrides)
    config["cameras"] = {str(k): str(v) for k, v in config["cameras"].items()}
    return config


def config_dir():
    """config/ of the installed package (ament index), else of the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("restore_perception")) / "config"
    except Exception:  # no ROS, or the package is not installed
        return Path(__file__).resolve().parents[2] / "config"


def _symmetry(value):
    if value in (None, "none"):
        return SYM_NONE
    if value == "continuous":
        return SYM_CONTINUOUS
    if isinstance(value, dict) and int(value.get("n", 0)) >= 2:
        return int(value["n"])
    raise ValueError(f"symmetry must be none, continuous or {{n: k}} with k >= 2, not {value!r}")


def load_objects(path=None):
    """objects.yaml: name -> size_m, symmetry code, grasp and flags. Missing file: no objects."""
    if not path or not Path(path).exists():
        return {}
    data = yaml.safe_load(Path(path).read_text()) or {}
    objects = {}
    for name, item in (data.get("objects") or {}).items():
        item = dict(item or {})
        objects[str(name)] = {
            **item,
            "size_m": [float(v) for v in item["size_m"]] if item.get("size_m") else None,
            "symmetry": _symmetry(item.get("symmetry")),
            "up_side_matters": bool(item.get("up_side_matters", False)),
            "depth_reliable": bool(item.get("depth_reliable", True)),
        }
    return objects
