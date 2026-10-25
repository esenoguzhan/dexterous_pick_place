"""Check closure configuration against the hand model and ROS duration range."""

import math

import pytest

from lara5_grasp.configuration import DEFAULT_CLOSE, HAND_JOINTS, UPPER_LIMITS, validate_closure


def test_default_preset_matches_existing_hand_command_by_joint_name():
    positions, nanoseconds = validate_closure(DEFAULT_CLOSE, 3.0)
    expected = {"right_thumb_finger_proximal_yaw": 1.1,
                "right_thumb_finger_proximal_pitch": 0.5,
                "right_index_finger_proximal": 1.3,
                "right_middle_finger_proximal": 1.3,
                "right_ring_finger_proximal": 1.3,
                "right_pinky_finger_proximal": 1.3}
    assert dict(zip(HAND_JOINTS, positions)) == expected
    assert nanoseconds == 3_000_000_000


def test_fractional_duration_is_preserved():
    _, nanoseconds = validate_closure(UPPER_LIMITS, 2.75)
    assert divmod(nanoseconds, 1_000_000_000) == (2, 750_000_000)


@pytest.mark.parametrize("positions", [(), DEFAULT_CLOSE[:-1], DEFAULT_CLOSE + (0.0,),
                                      (-0.1,) + DEFAULT_CLOSE[1:],
                                      (1.2,) + DEFAULT_CLOSE[1:],
                                      DEFAULT_CLOSE[:-1] + (0.6,),
                                      (math.nan,) + DEFAULT_CLOSE[1:],
                                      (math.inf,) + DEFAULT_CLOSE[1:]])
def test_invalid_angles_are_rejected(positions):
    with pytest.raises(ValueError):
        validate_closure(positions, 3.0)


@pytest.mark.parametrize("duration", [0.0, -1.0, math.nan, math.inf, 1e-12, 2**31])
def test_invalid_durations_are_rejected(duration):
    with pytest.raises(ValueError):
        validate_closure(DEFAULT_CLOSE, duration)
