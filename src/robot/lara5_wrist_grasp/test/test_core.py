"""Real-camera encodings and bounded geometry; no robot connections."""

from types import SimpleNamespace as NS

import numpy as np
import pytest

from lara5_sim_approach.vision import image_array
from lara5_wrist_grasp.core import bounded_target, detect_target, require_hardware
from lara5_wrist_grasp.selection import DEFAULTS


def image(values, encoding, big=False, padding=0):
    values = np.asarray(values)
    rows = values.view(np.uint8).reshape(values.shape[0], -1)
    rows = np.pad(rows, ((0, 0), (0, padding)))
    return NS(encoding=encoding, is_bigendian=big, height=values.shape[0], width=values.shape[1],
              step=rows.shape[1], data=rows.tobytes(), header=NS(frame_id='optical'))


def test_real_depth_converts_mm_to_metres_with_stride_and_endianness():
    msg = image(np.array([[0, 350], [1000, 65535]], dtype='>u2'), '16UC1', True, 4)
    assert np.allclose(image_array(msg), [[0, .350], [1., 65.535]])
    with pytest.raises(ValueError):
        image_array(msg, 0.)


def test_bgr_and_float_depth_are_decoded_without_reversing_depth_units():
    assert image_array(image(np.array([[[10, 20, 30]]], dtype=np.uint8), 'bgr8')).tolist() == [[[30, 20, 10]]]
    assert np.allclose(image_array(image(np.array([[.35]], dtype='<f4'), '32FC1')), [[.35]])


def target_inputs():
    rgb = np.zeros((40, 40, 3), np.uint8)
    rgb[10:30, 10:30, 1] = 255
    depth = np.full((40, 40), 700, np.uint16)
    depth[10:30, 10:30] = 350
    info = NS(height=40, width=40, k=[100., 0., 20., 0., 100., 20., 0., 0., 1.],
              header=NS(frame_id='optical'))
    return image(rgb, 'rgb8'), image(depth, '16UC1'), info


def detect(rgb, depth, info):
    return detect_target(rgb, depth, info, np.eye(4),
                         dict(DEFAULTS, roi=[0., 0., 1., 1.], min_pixels=40,
                              surface_to_grasp_xyz=[0., 0., .03]))


def test_explicit_grasp_reference_is_localized_from_real_depth_units():
    center = detect(*target_inputs())
    assert np.allclose(center, [-.00175, -.00175, .38], atol=.002)


def test_missing_depth_and_wrong_optical_frame_are_rejected():
    rgb, depth, info = target_inputs()
    depth.data = bytes(len(depth.data))
    with pytest.raises(ValueError, match='valid depth'):
        detect(rgb, depth, info)
    depth.header.frame_id = 'another_camera'
    with pytest.raises(ValueError, match='optical frame'):
        detect(rgb, depth, info)


def test_ambiguous_depth_objects_are_rejected():
    rgb, depth, info = target_inputs()
    depths = np.full((40, 40), 700, np.uint16)
    depths[6:16, 6:16] = 350
    depths[23:38, 23:38] = 400
    with pytest.raises(ValueError, match='unambiguous'):
        detect(rgb, image(depths, '16UC1'), info)


def test_step_limits_preserve_manually_chosen_orientation_and_reject_remote_target():
    current = np.eye(4); current[:3, 3] = [.3, -.4, .5]
    goal, error = bounded_target(current, [.36, -.4, .5], np.zeros(3), current[:3, 3],
        .01, .08, .1, np.array([-.8, -.8, .05]), np.array([.8, .8, 1.]))
    assert error == pytest.approx(.06)
    assert goal[0, 3] == pytest.approx(.31)
    assert np.array_equal(goal[:3, :3], current[:3, :3])
    with pytest.raises(ValueError, match='envelope'):
        bounded_target(current, [.5, -.4, .5], np.zeros(3), current[:3, 3],
            .01, .08, .1, np.array([-.8, -.8, .05]), np.array([.8, .8, 1.]))


def test_simulation_description_cannot_unlock_hardware():
    with pytest.raises(ValueError, match='real NEURA'):
        require_hardware('<robot><ros2_control><hardware><plugin>topic_based_ros2_control/TopicBasedSystem</plugin></hardware></ros2_control></robot>')
    require_hardware('<robot><ros2_control><hardware><plugin>neuraPlugin/PositionAndVelocityHW</plugin></hardware></ros2_control></robot>')
