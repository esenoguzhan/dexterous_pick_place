"""Read-only object-selection preview; no TF, robot clients or motion topics."""

import copy
import json
import math
import signal
import time

import message_filters
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
import yaml

from lara5_sim_approach.vision import image_array
from .depth_check import measure
from .selection import DEFAULTS, overlay, select_target, validate


PREFIX = '/arm/wrist/selection'


class WristObjectPreview(Node):
    def __init__(self, node_name='wrist_object_preview', prefix=PREFIX, extra_defaults=None,
                 success_state='selected', **kwargs):
        super().__init__(node_name, **kwargs)
        self.success_state = success_state
        path = self.declare_parameter('approach_config_file', '',
                                      ParameterDescriptor(read_only=True)).value
        supplied = {}
        if path:
            with open(path, encoding='utf-8') as source:
                supplied = yaml.safe_load(source)['wrist_approach']['ros__parameters']
        defaults = dict(DEFAULTS, color_topic='/cameras/wrist/color/image_raw',
            depth_topic='/cameras/wrist/aligned_depth_to_color/image_raw',
            info_topic='/cameras/wrist/color/camera_info', optical_frame='wrist_color_optical_frame',
            feedback_max_age=.5, synchronization_slop=.03)
        defaults.update(extra_defaults or {})
        self.cfg = {k: self.declare_parameter(k, supplied.get(k, v),
            ParameterDescriptor(read_only=k not in DEFAULTS)).value for k, v in defaults.items()}
        validate(self.cfg)
        for key in ('feedback_max_age', 'synchronization_slop'):
            if not math.isfinite(self.cfg[key]) or self.cfg[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        self.last_source = self.last_receipt = self.latest = None
        self.error = None
        self.camera_invalid = True
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, prefix+'/status', retained)
        self.image_pub = self.create_publisher(Image, prefix+'/image', qos_profile_sensor_data)
        self.rgb = message_filters.Subscriber(self, Image, self.cfg['color_topic'], qos_profile=qos_profile_sensor_data)
        self.depth = message_filters.Subscriber(self, Image, self.cfg['depth_topic'], qos_profile=qos_profile_sensor_data)
        self.info = message_filters.Subscriber(self, CameraInfo, self.cfg['info_topic'], qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb, self.depth, self.info], 10, self.cfg['synchronization_slop'])
        self.sync.registerCallback(self.on_frames)
        self.add_on_set_parameters_callback(self.parameters)
        self.create_timer(.2, self.tick)

    def parameters(self, parameters):
        try:
            proposed = dict(self.cfg)
            for parameter in parameters:
                if parameter.name not in DEFAULTS:
                    raise ValueError('Only selection settings may change live')
                proposed[parameter.name] = parameter.value
            validate(proposed)
        except (ValueError, TypeError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        self.cfg = proposed
        self.latest = None
        self.error = 'Settings changed; waiting for a new selection'
        return SetParametersResult(successful=True)

    def publish_image(self, output, header):
        self.image_pub.publish(Image(header=copy.deepcopy(header), height=output.shape[0],
            width=output.shape[1], encoding='rgb8', step=output.shape[1]*3, data=output.tobytes()))

    def selection_result(self, rgb, color, info, selected):
        return overlay(color, info, self.cfg, selected), selected.report()

    def on_frames(self, rgb, depth, info):
        self.camera_invalid = True
        try:
            now = self.get_clock().now().nanoseconds/1e9
            stamps = [m.header.stamp.sec+m.header.stamp.nanosec/1e9 for m in (rgb, depth, info)]
            if any(s <= 0 or not 0 <= now-s <= self.cfg['feedback_max_age'] for s in stamps):
                raise ValueError('Stale or future camera source timestamp')
            if max(stamps)-min(stamps) > self.cfg['synchronization_slop']:
                raise ValueError('Unsynchronized RGB/depth/CameraInfo')
            if rgb.header.frame_id != self.cfg['optical_frame']:
                raise ValueError('Unexpected camera optical frame')
            if self.last_source is not None and stamps[1] <= self.last_source:
                raise ValueError('Repeated or backwards depth timestamp')
            measure(rgb, depth, info, self.cfg['roi'], self.cfg['depth_scale'],
                    self.cfg['min_depth'], self.cfg['max_depth'])
            self.camera_invalid = False
            self.last_source = stamps[1]
            self.last_receipt = time.monotonic()
            color, selected = select_target(rgb, depth, info, self.cfg)
            output, self.latest = self.selection_result(rgb, color, info, selected)
            self.error = None
            self.publish_image(output, rgb.header)
        except (ValueError, TypeError) as error:
            self.error, self.latest = str(error), None
            # Show the selection region even when there is no isolated object.
            # Stale/repeated frames are replaced with a warning by tick().
            if not self.camera_invalid:
                try:
                    self.publish_image(overlay(image_array(rgb), info, self.cfg,
                                               error=self.error), rgb.header)
                except (ValueError, TypeError):
                    pass

    def tick(self):
        stale = self.last_receipt is None or time.monotonic()-self.last_receipt > self.cfg['feedback_max_age']
        state = 'stale_camera' if stale else ('invalid_selection' if self.error else self.success_state)
        status = dict(state=state, error=self.error, read_only=True, motion_authorized=False,
            closure_ready=False, optical_frame=self.cfg['optical_frame'],
            settings={k: self.cfg[k] for k in DEFAULTS},
            latest=self.latest if not stale and not self.error else None)
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))
        if stale or self.error:
            # Invalid selection overlays remain useful for ROI adjustment;
            # replace them only for stale/replayed/invalid camera data.
            if stale or self.camera_invalid:
                output = np.zeros((480, 640, 3), dtype=np.uint8)
                output = overlay(output, None, self.cfg, error=self.error or 'Waiting for fresh camera frames')
                message = Image()
                message.header.frame_id = self.cfg['optical_frame']
                message.header.stamp = self.get_clock().now().to_msg()
                self.publish_image(output, message.header)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = False

    def interrupt(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = WristObjectPreview()
    try:
        while not stopping and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
