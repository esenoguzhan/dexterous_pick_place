"""Mount calibration and working-range checks, without a renderer or ROS node."""

import math

import numpy as np
import pytest

from lara5_mujoco_sim.camera import camera_pose, depth_limits, mask_depth


def test_measured_optical_quaternion_replaces_look_at_without_moving_origin():
    config = {"position": [.01, .08, -.08], "orientation_xyzw": [0., 0., math.sqrt(.5), math.sqrt(.5)],
              "target": [0, 0, 0], "up": [0, 0, 0]}
    position, rotation = camera_pose(config)
    np.testing.assert_allclose(position, config["position"])
    np.testing.assert_allclose(rotation, [[0, -1, 0], [1, 0, 0], [0, 0, 1]], atol=1e-12)


@pytest.mark.parametrize("quaternion", [[0, 0, 0, 0], [0, 0, 0, 2], [0, 0, 1], [0, 0, math.nan, 1]])
def test_invalid_optical_quaternion_is_rejected(quaternion):
    with pytest.raises(ValueError, match="orientation_xyzw"):
        camera_pose({"position": [0, .08, -.08], "orientation_xyzw": quaternion})


def test_camera_position_is_validated_for_measured_pose():
    with pytest.raises(ValueError, match="position"):
        camera_pose({"position": [math.inf, 0, 0], "orientation_xyzw": [0, 0, 0, 1]})


def test_depth_is_in_metres_and_invalid_or_out_of_range_values_are_zero():
    config = {"depth_min_m": .2, "depth_max_m": 3.}
    result = mask_depth(np.array([[math.nan, math.inf, -1., 0., .19, .2, .23, 3., 3.1]]), config)
    np.testing.assert_allclose(result, [[0, 0, 0, 0, 0, .2, .23, 3., 0]])
    assert result.dtype == np.dtype('<f4')


@pytest.mark.parametrize("config", [{"depth_min_m": -.1}, {"depth_min_m": math.nan},
                                    {"depth_min_m": .3, "depth_max_m": .2}, {"depth_max_m": math.nan}])
def test_invalid_depth_range_is_rejected(config):
    with pytest.raises(ValueError, match="depth range"):
        depth_limits(config)
