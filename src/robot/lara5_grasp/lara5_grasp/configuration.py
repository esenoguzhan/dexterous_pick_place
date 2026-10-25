"""Closure settings for the repository's Inspire right-hand model."""

import math


HAND_JOINTS = (
    "right_thumb_finger_proximal_yaw", "right_index_finger_proximal",
    "right_middle_finger_proximal", "right_ring_finger_proximal",
    "right_pinky_finger_proximal", "right_thumb_finger_proximal_pitch",
)
# Limits from lara5_inspire_hand_description/urdf/inspire_hand_macro.xacro.
UPPER_LIMITS = (1.1641, 1.4381, 1.4381, 1.4381, 1.4381, 0.5864)
DEFAULT_CLOSE = (1.1, 1.3, 1.3, 1.3, 1.3, 0.5)


def validate_closure(positions, duration):
    positions = tuple(float(value) for value in positions)
    duration = float(duration)
    if len(positions) != len(HAND_JOINTS):
        raise ValueError("close_positions requires six joint angles")
    for name, value, upper in zip(HAND_JOINTS, positions, UPPER_LIMITS):
        if not math.isfinite(value) or not 0.0 <= value <= upper:
            raise ValueError(f"{name} must be between 0 and {upper} radians")
    if not math.isfinite(duration) or duration <= 0 or duration >= 2**31:
        raise ValueError("duration must be finite, positive and below 2**31 seconds")
    nanoseconds = round(duration * 1_000_000_000)
    if nanoseconds <= 0 or nanoseconds // 1_000_000_000 >= 2**31:
        raise ValueError("duration is outside ROS Duration range")
    return positions, nanoseconds
