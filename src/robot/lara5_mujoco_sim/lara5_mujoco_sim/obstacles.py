"""Convert the safety filter's world-frame obstacles to fixed MuJoCo geometry.

No MuJoCo/ROS import is needed, so geometry conversion can be tested without
starting physics or a ROS node. Input box sizes and cylinder heights are full
lengths, matching the safety filter's conventions.
"""

import math

import yaml


def _vector(values, length, label):
    if not isinstance(values, (list, tuple)) or len(values) != length:
        raise ValueError(f"{label} requires {length} numbers")
    result = [float(value) for value in values]
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} must be finite")
    return result


def obstacle_geometry(obstacle):
    """Return MJCF geometry attributes, or None for an explicitly omitted geom."""
    name = obstacle.get("name")
    if not isinstance(name, str) or not name:
        raise ValueError("Static obstacles require a nonempty name")
    shape = obstacle.get("shape", "box")
    lengths = {"box": 3, "sphere": 1, "cylinder": 2}
    if shape not in lengths:
        raise ValueError(f"Unsupported obstacle shape: {shape}")
    size = _vector(obstacle.get("size"), lengths[shape], f"{name}.size")
    if any(value <= 0 for value in size):
        raise ValueError(f"{name}.size must be positive")
    position = _vector(obstacle.get("position", [0, 0, 0]), 3, f"{name}.position")
    roll, pitch, yaw = _vector(obstacle.get("rpy", [0, 0, 0]), 3, f"{name}.rpy")
    rgba = _vector(obstacle.get("rgba", [0.55, 0.55, 0.55, 1]), 4, f"{name}.rgba")
    if any(value < 0 or value > 1 for value in rgba):
        raise ValueError(f"{name}.rgba must lie in [0, 1]")
    enabled = obstacle.get("mujoco", True)
    if not isinstance(enabled, bool):
        raise ValueError(f"{name}.mujoco must be a boolean")
    if not enabled:
        return None
    if shape == "box":
        size = [value / 2 for value in size]
    elif shape == "cylinder":
        size[1] /= 2
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    quaternion = [cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
                  cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy]
    fmt = lambda values: " ".join(f"{value:.12g}" for value in values)
    return {"name": f"obstacle_{name}", "type": shape, "size": fmt(size),
            "pos": fmt(position), "quat": fmt(quaternion), "rgba": fmt(rgba)}


def load_obstacles(path):
    """Validate one shared safety/planning/simulation obstacle YAML file."""
    with open(path, encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict) or not isinstance(document.get("obstacles"), list):
        raise ValueError("Obstacle file requires an obstacles list")
    names = set()
    for obstacle in document["obstacles"]:
        if not isinstance(obstacle, dict):
            raise ValueError("Every obstacle must be a mapping")
        obstacle_geometry(obstacle)
        if obstacle["name"] in names:
            raise ValueError(f"Duplicate obstacle name: {obstacle['name']}")
        names.add(obstacle["name"])
    return document["obstacles"]
