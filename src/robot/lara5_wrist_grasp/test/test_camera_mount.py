"""CAD mounting chain and driver optical transform direction; no robot motion."""

from pathlib import Path
import subprocess

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from tf2_ros import LookupException

from lara5_object_approach.grasp_geometry import transform
from lara5_wrist_grasp.camera_mount import CameraMount, DEFAULTS, fixed_chain
from lara5_wrist_grasp.alignment import alignment_observation
from lara5_wrist_grasp.selection import DEFAULTS as SELECTION_DEFAULTS, select_target
from test_core import target_inputs
from test_workflow import description


def config(**changes):
    return dict(DEFAULTS, optical_frame='wrist_color_optical_frame', **changes)


def actual_description():
    path = Path(get_package_share_directory('lara5_inspire_hand_description'))/'urdf/lara5_inspire_hand.urdf.xacro'
    return subprocess.check_output(['xacro', str(path)], text=True)


def test_actual_cad_chain_reproduces_documented_flange_depth_origin():
    mount = CameraMount(config())
    flange_depth = fixed_chain(actual_description(), 'lara5_flange', mount.parent) @ mount.holder_depth
    assert np.allclose(flange_depth[:3, 3], [.08895144087464547, -.0175, .01502290077437976])
    assert np.allclose(flange_depth[:3, :3].T @ flange_depth[:3, :3], np.eye(3))
    assert mount.source.endswith('lara5_mujoco_sim/config/mujoco_sim.yaml')


def test_sdk_depth_to_color_mapping_must_be_inverted_in_cad_chain():
    mount = CameraMount(config())
    color_depth = transform([.014, -.001, .002], [0., 0., np.sin(.03), np.cos(.03)])
    depth_color = np.linalg.inv(color_depth)
    actual, evidence = mount.compose(description(), depth_color)
    flange_depth = fixed_chain(description(), 'lara5_flange', mount.parent) @ mount.holder_depth
    optical_point = np.array([.04, -.02, .32, 1.])
    assert np.allclose(actual @ optical_point, flange_depth @ np.linalg.inv(color_depth) @ optical_point)
    assert not np.allclose(actual, flange_depth @ color_depth)
    assert not evidence['alignment_verified']
    assert evidence['nominal_optical_origin'] == 'depth'


def test_resolver_requests_driver_tf_in_depth_parent_color_child_direction():
    class Buffer:
        def lookup_transform(self, target, source, stamp):
            self.request = (target, source, stamp)
            msg = TransformStamped()
            msg.header.frame_id, msg.child_frame_id = target, source
            msg.transform.rotation.w = 1.
            msg.transform.translation.x = -.015
            return msg
    mount, buffer = CameraMount(config()), Buffer()
    _, report = mount.resolve(description(), buffer, 'capture_time')
    assert buffer.request == ('wrist_depth_optical_frame', 'wrist_color_optical_frame', 'capture_time')
    assert report['driver_T_depth_color']['xyz_m'] == [-.015, 0., 0.]


def test_missing_driver_tf_never_falls_back_to_cad_depth_or_identity():
    class Missing:
        def lookup_transform(self, *args):
            raise LookupException('no sensor TF')
    with pytest.raises(ValueError, match='Missing driver'):
        CameraMount(config()).resolve(description(), Missing(), None)
    with pytest.raises(ValueError, match='no identity fallback'):
        CameraMount(config()).compose(description())
    with pytest.raises(ValueError, match='unity fallback'):
        CameraMount(config()).compose(description(), np.eye(4))


def test_measured_color_pose_replaces_entire_nominal_chain_without_applying_extrinsics_twice():
    mount = CameraMount(config(camera_mount_mode='measured_color',
        camera_calibration_confirmed=True, camera_to_flange_xyz=[.12, -.03, .02],
        camera_to_flange_xyzw=[0., 0., 0., 1.]))
    result, report = mount.resolve('', None, None)
    assert np.allclose(result, transform([.12, -.03, .02], [0., 0., 0., 1.]))
    assert report['alignment_verified'] and report['nominal_source_file'] is None
    assert 'driver_T_depth_color' not in report


@pytest.mark.parametrize('urdf', [
    '<robot><link name="lara5_flange"/></robot>',
    '<robot><link name="lara5_flange"/><link name="holder"/><joint type="revolute"><parent link="lara5_flange"/><child link="holder"/></joint></robot>',
    '<robot><link name="lara5_flange"/><link name="holder"/><joint type="fixed"><parent link="holder"/><child link="holder"/></joint></robot>'])
def test_missing_movable_or_cyclic_mounts_are_rejected(urdf):
    with pytest.raises(ValueError):
        fixed_chain(urdf, 'lara5_flange', 'holder')


@pytest.mark.parametrize('changes', [dict(camera_mount_mode='guess'),
    dict(depth_optical_frame='wrist_color_optical_frame'),
    dict(camera_to_flange_xyz=[.01, 0., 0.]),
    dict(camera_mount_mode='measured_color', camera_to_flange_xyzw=[0., 0., 0., 0.])])
def test_invalid_configuration_cannot_create_a_camera_mount(changes):
    with pytest.raises(ValueError):
        CameraMount(config(**changes))


def test_alignment_correction_rotates_with_robot_orientation_without_changing_it():
    rgb, depth, info = target_inputs()
    # Match the message attributes used for evidence timestamps.
    from types import SimpleNamespace
    info.header.stamp = SimpleNamespace(sec=42, nanosec=0)
    c = dict(SELECTION_DEFAULTS, base_frame='lara5_root_link', base_alignment_confirmed=False)
    color, selected = select_target(rgb, depth, info, c)
    base = transform([.3, -.2, .5], [0., 0., np.sin(np.pi/4), np.cos(np.pi/4)])
    flange_color = transform([0., 0., .01], [0., 0., 0., 1.])
    hand_offset = np.array([0., 0., .3])
    image, report = alignment_observation(color, info, c, selected, base, flange_color,
        hand_offset, dict(mode='nominal_cad', alignment_verified=False))
    expected_reference = (base @ flange_color @ np.r_[selected.reference_optical, 1.])[:3]
    expected_correction = expected_reference-(base[:3, 3]+base[:3, :3] @ hand_offset)
    assert np.allclose(report['correction_base_m'], expected_correction)
    assert np.allclose(report['proposed_flange_xyz_m'], base[:3, 3]+expected_correction)
    assert report['source_stamp'] == 42. and not report['motion_authorized']
    assert not report['path_checked'] and not report['robot_base_alignment_verified']
    assert image.shape == color.shape
