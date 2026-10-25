"""Pinhole camera geometry in either world or a robot link's local frame."""

import math

import numpy as np


def camera_axes(position, target, up=None):
    """Return camera-to-parent rotation: MuJoCo +X right, +Y up, -Z forward."""
    position, target = np.asarray(position, dtype=float), np.asarray(target, dtype=float)
    if position.shape != (3,) or target.shape != (3,) or not np.isfinite([position, target]).all():
        raise ValueError("Camera position/target must be finite XYZ vectors")
    forward = target - position
    if np.linalg.norm(forward) < 1e-6:
        raise ValueError("Camera position and target must differ")
    forward /= np.linalg.norm(forward)
    explicit_up = up is not None
    up = np.asarray([0.0, 0.0, 1.0] if up is None else up, dtype=float)
    if up.shape != (3,) or not np.isfinite(up).all() or np.linalg.norm(up) < 1e-6:
        raise ValueError("Camera up must be a finite nonzero XYZ vector")
    up = up / np.linalg.norm(up)
    if abs(forward @ up) > 0.99:
        if explicit_up:
            raise ValueError("Camera up must not be parallel to its viewing direction")
        up = np.array([0.0, 1.0, 0.0])
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    return np.column_stack((right, np.cross(right, forward), -forward))


def intrinsics(width, height, fovy):
    if width <= 0 or height <= 0 or not 0 < fovy < 180:
        raise ValueError("Invalid camera resolution or vertical field of view")
    focal = height / (2.0 * math.tan(math.radians(fovy) / 2.0))
    return np.array([[focal, 0.0, (width - 1) / 2.0],
                     [0.0, focal, (height - 1) / 2.0], [0.0, 0.0, 1.0]])


def optical_rotation(position, target, up=None):
    """ROS optical +X right, +Y down, +Z forward, expressed in the parent."""
    return camera_axes(position, target, up) @ np.diag([1.0, -1.0, -1.0])


def camera_pose(config):
    """Optical-to-parent pose from a look-at mount or calibrated ROS quaternion.

    A non-null orientation_xyzw takes precedence over target/up. Translation is
    always the optical centre in parent_frame, not a housing or screw position.
    """
    position = np.asarray(config["position"], dtype=float)
    if position.shape != (3,) or not np.isfinite(position).all():
        raise ValueError("Camera position must be a finite XYZ vector")
    quaternion = config.get("orientation_xyzw")
    if quaternion is None:
        return position, optical_rotation(position, config["target"], config.get("up"))
    quaternion = np.asarray(quaternion, dtype=float)
    if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
        raise ValueError("Camera orientation_xyzw must contain four finite numbers")
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(norm) or abs(norm - 1.0) > 0.01:
        raise ValueError("Camera orientation_xyzw must be a unit quaternion")
    x, y, z, w = quaternion / norm
    rotation = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])
    return position, rotation


def depth_limits(config):
    minimum = float(config.get("depth_min_m", 0.0))
    maximum = float(config.get("depth_max_m", math.inf))
    if not math.isfinite(minimum) or minimum < 0 or math.isnan(maximum) or maximum <= minimum:
        raise ValueError("Camera depth range requires 0 <= depth_min_m < depth_max_m")
    return minimum, maximum


def mask_depth(depth, config):
    """Metres as float32; zero marks invalid or out-of-working-range depth."""
    minimum, maximum = depth_limits(config)
    depth = np.asarray(depth, dtype=np.float32)
    valid = np.isfinite(depth) & (depth > 0) & (depth >= minimum) & (depth <= maximum)
    return np.where(valid, depth, 0.0).astype("<f4")
