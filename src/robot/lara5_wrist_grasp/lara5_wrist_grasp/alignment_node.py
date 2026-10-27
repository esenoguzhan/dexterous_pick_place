"""CAD/color alignment preview from camera, robot description and capture TF only."""

import signal
import time
from collections import deque
import xml.etree.ElementTree as ET

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .alignment import alignment_observation
from .camera_mount import CameraMount, DEFAULTS, stamped_matrix
from .core import grasp_offset
from .selection_node import WristObjectPreview


PREFIX = '/arm/wrist/alignment'


class WristAlignmentPreview(WristObjectPreview):
    def __init__(self, **kwargs):
        super().__init__(node_name='wrist_alignment_preview', prefix=PREFIX,
            success_state='alignment_preview_ready', extra_defaults=dict(DEFAULTS,
                base_frame='lara5_root_link', base_alignment_confirmed=False,
                hand_to_grasp_xyz=[-.025, .085, .15]), **kwargs)
        self.mount = CameraMount(self.cfg)
        self.description = ''
        self.offset = None
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description', self.on_description, retained)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pending_frames = deque(maxlen=10)
        self.create_timer(.02, self.process_frames)

    def on_description(self, message):
        if message.data != self.description:
            self.description = message.data
            self.offset = None

    def on_frames(self, rgb, depth, info):
        # TF frequently reaches the executor just after its matching images.
        # Defer processing until the EXACT capture-time transform is available;
        # never substitute latest robot TF or block the camera callback.
        self.pending_frames.append((rgb, depth, info, time.monotonic()))

    def process_frames(self):
        if not self.pending_frames:
            return
        c = self.cfg
        selected = None
        for frame in reversed(self.pending_frames):
            stamp = rclpy.time.Time.from_msg(frame[0].header.stamp)
            if (self.tf.can_transform(c['base_frame'], 'lara5_flange', stamp)
                    and (c['camera_mount_mode'] == 'measured_color' or
                         self.tf.can_transform(c['depth_optical_frame'], c['optical_frame'], stamp))):
                selected = frame
                break
        if selected is not None:
            while self.pending_frames:
                removed = self.pending_frames.popleft()
                if removed is selected:
                    break
            super().on_frames(*selected[:3])
        elif time.monotonic()-self.pending_frames[0][3] > min(.05, c['feedback_max_age']):
            self.latest = None
            self.camera_invalid = True
            self.error = 'Waiting for capture-time robot and driver optical TF'

    def selection_result(self, rgb, color, info, selected):
        if not self.description:
            raise ValueError('Waiting for robot_description to resolve CAD flange/hand/holder chain')
        if self.offset is None:
            try:
                self.offset = grasp_offset(self.description, self.cfg['hand_to_grasp_xyz'])
            except ET.ParseError as error:
                raise ValueError('Invalid robot_description for hand/camera geometry') from error
        stamp = rclpy.time.Time.from_msg(rgb.header.stamp)
        try:
            base_flange = stamped_matrix(self.tf.lookup_transform(self.cfg['base_frame'], 'lara5_flange', stamp))
        except TransformException as error:
            raise ValueError('Capture-time robot base-to-flange TF unavailable') from error
        flange_color, mount_report = self.mount.resolve(self.description, self.tf, stamp)
        return alignment_observation(color, info, self.cfg, selected, base_flange,
                                     flange_color, self.offset, mount_report)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = False

    def interrupt(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = WristAlignmentPreview()
    try:
        while not stopping and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
