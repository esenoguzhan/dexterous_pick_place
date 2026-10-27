"""ROS camera-only preview and live settings; isolated synthetic camera, no robot."""

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
from std_msgs.msg import String

from lara5_wrist_grasp.selection_node import PREFIX, WristObjectPreview


class Harness:
    def __init__(self, node_class=WristObjectPreview, output_prefix=PREFIX,
                 ready_state='selected', **overrides):
        self.prefix = '/selection_test_'+uuid.uuid4().hex
        self.context = Context(); self.context.init()
        self.peer = Node('selection_camera_fixture', context=self.context)
        params = dict(color_topic=self.prefix+'/color', depth_topic=self.prefix+'/depth',
                      info_topic=self.prefix+'/info', feedback_max_age=.2)
        params.update(overrides)
        remaps = []
        for topic in ('image', 'status'):
            remaps += ['-r', output_prefix+'/'+topic+':='+self.prefix+'/'+topic]
        self.node = node_class(context=self.context, cli_args=['--ros-args', *remaps],
            parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        self.status, self.images = {}, []
        self.publish = True; self.replay = False; self.stamp = None; self.frame = 'wrist_color_optical_frame'
        self.depths = np.full((40, 40), 700, np.uint16)
        self.depths[10:30, 10:30] = 350
        self.rgb = self.peer.create_publisher(Image, self.prefix+'/color', qos_profile_sensor_data)
        self.depth = self.peer.create_publisher(Image, self.prefix+'/depth', qos_profile_sensor_data)
        self.info = self.peer.create_publisher(CameraInfo, self.prefix+'/info', qos_profile_sensor_data)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.peer.create_subscription(String, self.prefix+'/status', self.on_status, retained)
        self.peer.create_subscription(Image, self.prefix+'/image', self.images.append, qos_profile_sensor_data)
        self.peer.create_timer(.02, self.camera)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node); self.executor.add_node(self.peer)
        try:
            self.wait(lambda: self.status.get('state') == ready_state)
        except BaseException:
            self.close()
            raise

    def on_status(self, message):
        self.status = json.loads(message.data)

    def camera(self):
        if not self.publish:
            return
        if not self.replay or self.stamp is None:
            self.stamp = self.peer.get_clock().now().to_msg()
        rgb = Image(height=40, width=40, encoding='rgb8', step=120,
                    data=np.full((40, 40, 3), 100, np.uint8).tobytes())
        depth = Image(height=40, width=40, encoding='16UC1', step=80, data=self.depths.tobytes())
        info = CameraInfo(height=40, width=40, k=[100., 0., 20., 0., 100., 20., 0., 0., 1.])
        for message in (rgb, depth, info):
            message.header.frame_id = self.frame; message.header.stamp = self.stamp
        self.rgb.publish(rgb); self.depth.publish(depth); self.info.publish(info)

    def wait(self, predicate, timeout=4.):
        end = time.monotonic()+timeout
        while not predicate():
            if time.monotonic() > end:
                raise AssertionError(f'Selection timeout: {self.status}')
            self.executor.spin_once(timeout_sec=.02)

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


def test_preview_has_no_robot_clients_or_command_publishers(h):
    assert h.images and h.images[-1].encoding == 'rgb8'
    assert h.status['read_only'] and not h.status['motion_authorized'] and not h.status['closure_ready']
    assert h.status['latest']['selected_pixels'] == 400
    assert not h.node.get_client_names_and_types_by_node(h.node.get_name(), h.node.get_namespace())
    publications = h.node.get_publisher_names_and_types_by_node(h.node.get_name(), h.node.get_namespace())
    assert all(name in ('/rosout', '/parameter_events') or name.startswith(h.prefix+'/')
               for name, _ in publications)


def test_live_offset_and_roi_updates_use_new_observations(h):
    assert h.node.set_parameters([Parameter('surface_to_grasp_xyz', value=[0., 0., .03])])[0].successful
    h.wait(lambda: h.status.get('latest') is not None and
           abs(h.status['latest']['grasp_reference_optical_m'][2]-.38) < .001)
    assert h.node.set_parameters([Parameter('roi', value=[.3, .1, .9, .9])])[0].successful
    h.wait(lambda: h.status.get('state') == 'invalid_selection')
    assert h.status['latest'] is None
    assert not h.node.set_parameters([Parameter('roi', value=[0., 0., 0., 1.])])[0].successful
    assert h.node.cfg['roi'] == [.3, .1, .9, .9]


@pytest.mark.parametrize('fault', ['disconnect', 'replay', 'frame', 'ambiguity'])
def test_invalid_or_stale_preview_never_keeps_an_accepted_reference(h, fault):
    if fault == 'disconnect': h.publish = False
    if fault == 'replay': h.replay = True
    if fault == 'frame': h.frame = 'wrong_optical_frame'
    if fault == 'ambiguity': h.depths[6:15, 6:15] = 250
    h.wait(lambda: h.status.get('state') != 'selected')
    assert h.status['latest'] is None
    if fault in ('disconnect', 'replay', 'frame'):
        h.wait(lambda: h.images[-1].height == 480)


def test_preview_reads_same_selection_configuration_as_hardware(tmp_path):
    path = tmp_path/'hardware.yaml'
    content = ('wrist_approach:\n  ros__parameters:\n'
               '    surface_to_grasp_xyz: [0.0, 0.0, 0.02]\n'
               '    min_depth: 0.12\n    max_depth: 0.8\n')
    path.write_text(content)
    harness = Harness(approach_config_file=str(path))
    try:
        assert harness.status['latest']['grasp_reference_optical_m'][2] == pytest.approx(.37)
        assert harness.node.cfg['min_depth'] == .12
        assert path.read_text() == content
    finally:
        harness.close()
