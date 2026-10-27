"""Real ROS optical/robot TF plus RGB-D; preview has no motion interfaces."""

import numpy as np
import pytest
from geometry_msgs.msg import TransformStamped
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from lara5_wrist_grasp.alignment_node import PREFIX, WristAlignmentPreview
from test_selection_workflow import Harness
from test_workflow import description


class AlignmentHarness(Harness):
    def __init__(self, **overrides):
        self.publish_robot_tf = self.publish_driver_tf = True
        self.robot_xyz = [.3, -.4, .5]
        self.driver_xyz = [-.015, .001, .002]
        self.robot_description = description()
        super().__init__(node_class=WristAlignmentPreview, output_prefix=PREFIX,
                         ready_state='alignment_preview_ready', **overrides)

    def camera(self):
        super().camera()
        if not hasattr(self, 'tf_pub'):
            self.tf_pub = TransformBroadcaster(self.peer)
            self.description_pub = self.peer.create_publisher(String, '/robot_description',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.description_pub.publish(String(data=self.robot_description))
        if not self.publish:
            return
        transforms = []
        for enabled, parent, child, xyz in (
            (self.publish_robot_tf, 'lara5_root_link', 'lara5_flange', self.robot_xyz),
            (self.publish_driver_tf, 'wrist_depth_optical_frame', 'wrist_color_optical_frame', self.driver_xyz)):
            if enabled:
                msg = TransformStamped()
                msg.header.frame_id = parent; msg.child_frame_id = child; msg.header.stamp = self.stamp
                t = msg.transform
                t.translation.x, t.translation.y, t.translation.z = map(float, xyz)
                t.rotation.w = 1.
                transforms.append(msg)
        self.tf_pub.sendTransform(transforms)


@pytest.fixture
def h():
    harness = AlignmentHarness()
    try:
        yield harness
    finally:
        harness.close()


def test_unconfirmed_cad_alignment_is_visible_without_robot_clients_or_commands(h):
    report = h.status['latest']
    assert h.images and h.images[-1].encoding == 'rgb8'
    assert not report['camera_mount']['alignment_verified']
    assert report['camera_mount']['mode'] == 'nominal_cad'
    assert report['camera_mount']['driver_T_depth_color']['xyz_m'] == h.driver_xyz
    assert report['selected_pixels'] == 400 and not report['object_center_inferred']
    assert not report['motion_authorized'] and not report['closure_ready'] and not report['path_checked']
    assert not h.node.get_client_names_and_types_by_node(h.node.get_name(), h.node.get_namespace())
    publications = h.node.get_publisher_names_and_types_by_node(h.node.get_name(), h.node.get_namespace())
    assert all(name in ('/rosout', '/parameter_events') or name.startswith(h.prefix+'/')
               for name, _ in publications)


def test_preview_uses_same_mount_resolver_and_grasp_geometry_as_approach(h):
    from rclpy.time import Time
    flange_color, _ = h.node.mount.resolve(description(), h.node.tf, Time.from_msg(h.stamp))
    report = h.status['latest']
    reference_flange = (flange_color @ np.r_[report['grasp_reference_optical_m'], 1.])[:3]
    assert np.allclose(report['grasp_reference_base_m'], np.asarray(h.robot_xyz)+reference_flange)
    assert np.allclose(report['correction_base_m'], reference_flange-h.node.offset)


@pytest.mark.parametrize('fault', ['driver_tf', 'robot_tf', 'unity', 'description', 'malformed_description', 'disconnect'])
def test_missing_alignment_evidence_clears_reference_instead_of_using_old_geometry(h, fault):
    if fault == 'driver_tf': h.publish_driver_tf = False
    if fault == 'robot_tf': h.publish_robot_tf = False
    if fault == 'unity': h.driver_xyz = [0., 0., 0.]
    if fault == 'description': h.robot_description = ''
    if fault == 'malformed_description': h.robot_description = '<broken'
    if fault == 'disconnect': h.publish = False
    h.wait(lambda: h.status.get('state') != 'alignment_preview_ready')
    assert h.status['latest'] is None and not h.status['motion_authorized']
    if fault == 'unity':
        assert 'unity fallback' in h.status['error']


def test_alignment_parameters_cannot_change_mount_source_live(h):
    assert not h.node.set_parameters([Parameter('camera_mount_mode', value='measured_color')])[0].successful
    assert h.node.cfg['camera_mount_mode'] == 'nominal_cad'


def test_verified_measured_mount_preview_still_cannot_authorize_motion():
    h = AlignmentHarness(camera_mount_mode='measured_color', camera_calibration_confirmed=True,
        base_alignment_confirmed=True, grasp_reference_confirmed=True,
        camera_to_flange_xyz=[0., 0., -.05])
    try:
        assert h.status['latest']['camera_mount']['mode'] == 'measured_color'
        assert h.status['latest']['camera_mount']['alignment_verified']
        assert not h.status['latest']['motion_authorized']
        assert not h.status['closure_ready']
    finally:
        h.close()
