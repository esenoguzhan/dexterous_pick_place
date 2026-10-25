"""Geometry and validation for the simulation's provisional palm-down grasp."""

import math
import numpy as np

HAND = ["right_thumb_finger_proximal_yaw", "right_index_finger_proximal",
        "right_middle_finger_proximal", "right_ring_finger_proximal",
        "right_pinky_finger_proximal", "right_thumb_finger_proximal_pitch"]


def vector3(values, name):
    result = np.asarray(values, dtype=float)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError(f"{name} requires three finite values")
    return result


def palm_down_rotation(yaw):
    """Flange rotation: palm normal points down, fingers point horizontally.

    The active hand mount rotates hand-local axes -90 degrees about flange Z.
    This differs from the old downward-flange, noncontact approach posture.
    """
    c, s = math.cos(yaw), math.sin(yaw)
    world_yaw = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])
    hand = np.array([[1., 0, 0], [0, 0, 1.], [0, -1., 0]])
    mount_inverse = np.array([[0, -1., 0], [1., 0, 0], [0, 0, 1.]])
    return world_yaw @ hand @ mount_inverse


def hand_offset_in_flange(offset):
    x, y, z = vector3(offset, "grasp_offset_hand")
    return np.array([y, -x, z])


def flange_for_object(center, rotation, object_offset):
    return vector3(center, "object center") - np.asarray(rotation) @ vector3(object_offset, "object offset")


def quaternion_xyzw(rotation):
    """Stable matrix conversion, including palm-down rotations with zero trace."""
    r = np.asarray(rotation, dtype=float)
    matrix = np.array([
        [r[0, 0]-r[1, 1]-r[2, 2], r[0, 1]+r[1, 0], r[0, 2]+r[2, 0], r[2, 1]-r[1, 2]],
        [r[0, 1]+r[1, 0], r[1, 1]-r[0, 0]-r[2, 2], r[1, 2]+r[2, 1], r[0, 2]-r[2, 0]],
        [r[0, 2]+r[2, 0], r[1, 2]+r[2, 1], r[2, 2]-r[0, 0]-r[1, 1], r[1, 0]-r[0, 1]],
        [r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1], np.trace(r)]]) / 3.0
    _, vectors = np.linalg.eigh(matrix)
    result = vectors[:, -1]
    return result if result[3] >= 0 else -result


def validate_destination(position, extent, others, table_height=0.90, margin=0.04):
    """Only upright placement on the known tabletop; reject occupied/off-table goals."""
    position, extent = vector3(position, "place_position"), vector3(extent, "object dimensions")
    if np.any(extent <= 0):
        raise ValueError("Object dimensions must be positive")
    if abs(position[2] - extent[2] / 2 - table_height) > 0.005:
        raise ValueError("place_position is the settled object CENTRE; its bottom must be on the tabletop")
    if np.any(np.abs(position[:2]) + extent[:2]/2 + margin > [0.60, 0.40]):
        raise ValueError("Destination is outside the tabletop or too close to its edge")
    for name, center, dimensions in others:
        if np.all(np.abs(position[:2] - np.asarray(center)[:2]) <
                  (extent[:2] + np.asarray(dimensions)[:2])/2 + margin):
            raise ValueError(f"Destination overlaps or is too close to {name}")
    return position


def grip_slipped(reference, observed, translation_limit=0.02, angle_limit=0.25):
    """Relative object-to-flange drift, independent of world-frame carrying motion."""
    p0, q0 = reference
    p1, q1 = observed
    angle = 2 * math.acos(float(np.clip(abs(np.dot(q0, q1)), 0, 1)))
    return np.linalg.norm(np.asarray(p1)-p0) > translation_limit or angle > angle_limit
