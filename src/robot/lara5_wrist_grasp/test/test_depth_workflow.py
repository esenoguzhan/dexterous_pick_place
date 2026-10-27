"""ROS RGB-D transport and diagnostic triggers only; no hardware or controllers."""

import json
import time
import uuid

import numpy as np
import pytest
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Empty, String

from lara5_wrist_grasp.depth_node import PREFIX, WristDepthCheck


class Harness:
    def __init__(self, **overrides):
        self.prefix = '/depth_test_'+uuid.uuid4().hex
        self.context = Context(); self.context.init()
        self.peer = Node('camera_fixture', context=self.context)
        params = dict(capture_seconds=.12, min_frames=5, feedback_max_age=.3,
            roi=[.1, .1, .9, .9], color_topic=self.prefix+'/color',
            depth_topic=self.prefix+'/depth', info_topic=self.prefix+'/info')
        params.update(overrides)
        remaps = []
        for topic in ('capture_start', 'capture_grasp', 'status', 'report', 'image'):
            remaps += ['-r', PREFIX+'/'+topic+':='+self.prefix+'/'+topic]
        self.node = WristDepthCheck(context=self.context, cli_args=['--ros-args', *remaps],
            parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        self.report = {}; self.images = []; self.status = {}
        self.depth_mm = 130; self.publish = True; self.replay = False; self.stamp = None
        self.rgb = self.peer.create_publisher(Image, self.prefix+'/color', qos_profile_sensor_data)
        self.depth = self.peer.create_publisher(Image, self.prefix+'/depth', qos_profile_sensor_data)
        self.info = self.peer.create_publisher(CameraInfo, self.prefix+'/info', qos_profile_sensor_data)
        self.start = self.peer.create_publisher(Empty, self.prefix+'/capture_start', 1)
        self.grasp = self.peer.create_publisher(Empty, self.prefix+'/capture_grasp', 1)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.peer.create_subscription(String, self.prefix+'/report', self.on_report, retained)
        self.peer.create_subscription(String, self.prefix+'/status', self.on_status, retained)
        self.peer.create_subscription(Image, self.prefix+'/image', self.images.append, qos_profile_sensor_data)
        self.peer.create_timer(.02, self.camera)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node); self.executor.add_node(self.peer)
        self.wait(lambda: self.node.last is not None and self.start.get_subscription_count() == 1)

    def on_report(self, msg):
        self.report = json.loads(msg.data)

    def on_status(self, msg):
        self.status = json.loads(msg.data)

    def camera(self):
        if not self.publish:
            return
        if not self.replay or self.stamp is None:
            self.stamp = self.peer.get_clock().now().to_msg()
        rgb = Image(height=20, width=20, encoding='rgb8', step=60,
                    data=np.full((20, 20, 3), 100, np.uint8).tobytes())
        depth = Image(height=20, width=20, encoding='16UC1', step=40,
                      data=np.full((20, 20), self.depth_mm, np.uint16).tobytes())
        info = CameraInfo(height=20, width=20, k=[100., 0., 10., 0., 100., 10., 0., 0., 1.])
        for message in (rgb, depth, info):
            message.header.frame_id = 'wrist_color_optical_frame'
            message.header.stamp = self.stamp
        self.rgb.publish(rgb); self.depth.publish(depth); self.info.publish(info)

    def wait(self, predicate, timeout=4.):
        end = time.monotonic()+timeout
        while not predicate():
            if time.monotonic() > end:
                raise AssertionError(f'Diagnostic timeout: {self.status}, {self.report}')
            self.executor.spin_once(timeout_sec=.02)

    def result(self, label):
        self.wait(lambda: label in self.report.get('recordings', {}))
        return self.report['recordings'][label]

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node(); self.peer.destroy_node(); self.context.shutdown()


@pytest.fixture
def h():
    harness = Harness()
    try:
        yield harness
    finally:
        harness.close()


def test_start_and_grasp_recordings_compare_raw_depth_without_motion(h):
    h.depth_mm = 250
    h.start.publish(Empty())
    first = h.result('start')
    assert first['quality_criteria_met'] and first['approach_range_compatible']
    h.depth_mm = 130
    h.grasp.publish(Empty())
    final = h.result('grasp')
    assert final['quality_criteria_met'] and not final['approach_range_compatible']
    assert final['depth_median_m'] == pytest.approx(.13)
    assert not h.report['motion_authorized'] and len(h.report['recordings']) == 2
    assert h.images and h.images[-1].encoding == 'rgb8'
    publications = h.node.get_publisher_names_and_types_by_node(h.node.get_name(), h.node.get_namespace())
    assert all(name in ('/rosout', '/parameter_events') or name.startswith(h.prefix+'/')
               for name, _ in publications)
    assert not h.node.get_client_names_and_types_by_node(h.node.get_name(), h.node.get_namespace())


def test_cached_camera_frames_cannot_count_as_stable_depth(h):
    h.start.publish(Empty())
    h.wait(lambda: h.node.active is not None and len(h.node.active['samples']) >= 2)
    h.replay = True
    result = h.result('start')
    assert not result['quality_criteria_met'] and not result['capture_completed']
    assert any('timestamp' in reason for reason in result['reasons'])


def test_disconnected_camera_times_out_instead_of_using_previous_good_window(h):
    h.publish = False
    h.grasp.publish(Empty())
    result = h.result('grasp')
    assert not result['quality_criteria_met'] and not result['capture_completed']
    assert 'insufficient_new_frames' in result['reasons']
    h.wait(lambda: h.status.get('state') == 'stale_camera')
    # A warning canvas replaces the previous valid RGB overlay after disconnection.
    h.wait(lambda: h.images[-1].height == 480)


def test_live_roi_update_and_capture_configuration_lock(h):
    h.start.publish(Empty())
    h.wait(lambda: h.node.active is not None)
    assert not h.node.set_parameters([Parameter('roi', value=[.2, .2, .8, .8])])[0].successful
    h.result('start')
    assert h.node.set_parameters([Parameter('roi', value=[.2, .2, .8, .8])])[0].successful
    h.wait(lambda: h.node.last is not None)
    assert h.node.last['roi_pixels'] == [4, 4, 16, 16]


def test_comparison_uses_supplied_approach_config_without_changing_it(tmp_path):
    config = tmp_path/'hardware.yaml'
    content = 'wrist_approach:\n  ros__parameters:\n    min_depth: 0.12\n    max_depth: 0.5\n'
    config.write_text(content)
    harness = Harness(approach_config_file=str(config))
    try:
        harness.grasp.publish(Empty())
        assert harness.result('grasp')['approach_range_compatible']
        assert harness.report['approach_range_m'] == [.12, .5]
        assert config.read_text() == content
    finally:
        harness.close()
