"""Camera-only ROI monitor and explicitly triggered start/grasp recordings."""

import copy
import json
import math
import signal
import time

import cv2
import message_filters
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Empty, String
import yaml

from .depth_check import measure, roi_bounds, summarize


PREFIX = '/arm/wrist/depth_check'


class WristDepthCheck(Node):
    def __init__(self, **kwargs):
        super().__init__('wrist_depth_check', **kwargs)
        defaults = dict(
            color_topic='/cameras/wrist/color/image_raw',
            depth_topic='/cameras/wrist/aligned_depth_to_color/image_raw',
            info_topic='/cameras/wrist/color/camera_info', optical_frame='wrist_color_optical_frame',
            approach_config_file='', approach_min_depth=.20, approach_max_depth=.80,
            depth_scale=.001, roi=[.45, .45, .55, .55], capture_seconds=3., min_frames=30,
            feedback_max_age=.5, synchronization_slop=.03, min_valid_fraction=.8,
            max_spatial_spread_m=.02, max_temporal_spread_m=.005,
            nominal_grasp_depth_m=.1293, expected_surface_depth_m=0., max_reference_error_m=.01)
        self.cfg = {key: self.declare_parameter(key, value, ParameterDescriptor(
            read_only=key not in ('roi', 'expected_surface_depth_m'))).value
            for key, value in defaults.items()}
        c = self.cfg
        roi_bounds(c['roi'], 640, 480)
        for key, default in defaults.items():
            if isinstance(default, (float, int)) and (not math.isfinite(c[key]) or c[key] < 0
                    or (key != 'expected_surface_depth_m' and c[key] == 0)):
                raise ValueError(f'{key} must be finite and positive (reference distance may be zero)')
        if c['min_valid_fraction'] > 1 or c['capture_seconds'] > 30 or c['min_frames'] < 2:
            raise ValueError('Require fraction <= 1, capture <= 30 seconds and at least two frames')
        if c['approach_config_file']:
            with open(c['approach_config_file'], encoding='utf-8') as source:
                approach = yaml.safe_load(source)['wrist_approach']['ros__parameters']
            c['approach_min_depth'] = float(approach['min_depth'])
            c['approach_max_depth'] = float(approach['max_depth'])
        low, high = c['approach_min_depth'], c['approach_max_depth']
        if not (math.isfinite(low) and math.isfinite(high) and 0 < low < high):
            raise ValueError('Invalid approach comparison range')
        self.active = None
        self.last = None
        self.last_source = None
        self.last_receipt = None
        self.error = None
        self.recordings = {}
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, PREFIX+'/status', retained)
        self.report_pub = self.create_publisher(String, PREFIX+'/report', retained)
        self.image_pub = self.create_publisher(Image, PREFIX+'/image', qos_profile_sensor_data)
        self.create_subscription(Empty, PREFIX+'/capture_start', lambda _: self.capture('start'), 1)
        self.create_subscription(Empty, PREFIX+'/capture_grasp', lambda _: self.capture('grasp'), 1)
        self.rgb = message_filters.Subscriber(self, Image, c['color_topic'], qos_profile=qos_profile_sensor_data)
        self.depth = message_filters.Subscriber(self, Image, c['depth_topic'], qos_profile=qos_profile_sensor_data)
        self.info = message_filters.Subscriber(self, CameraInfo, c['info_topic'], qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb, self.depth, self.info], 10, c['synchronization_slop'])
        self.sync.registerCallback(self.on_frames)
        self.add_on_set_parameters_callback(self.parameters)
        self.create_timer(.2, self.tick)
        self.get_logger().info('Camera-only depth check: place ROI on the object surface; no motion interfaces')

    def parameters(self, parameters):
        updates = {}
        try:
            if self.active:
                raise ValueError('Wait for the recording to finish before changing the ROI/reference')
            for param in parameters:
                if param.name == 'roi':
                    roi_bounds(param.value, 640, 480)
                elif param.name == 'expected_surface_depth_m':
                    if not math.isfinite(param.value) or param.value < 0:
                        raise ValueError('Reference depth must be finite and nonnegative')
                else:
                    raise ValueError('Only roi and expected_surface_depth_m may change live')
                updates[param.name] = param.value
        except (ValueError, TypeError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        self.cfg.update(updates)
        self.last = None
        return SetParametersResult(successful=True)

    def capture(self, label):
        if self.active:
            self.get_logger().warning('Recording already active; duplicate trigger discarded')
            return
        now = time.monotonic()
        self.active = dict(label=label, samples=[], started=now,
            ros_started=self.get_clock().now().nanoseconds/1e9,
            deadline=now+self.cfg['capture_seconds']+2.,
            roi=list(self.cfg['roi']), expected=self.cfg['expected_surface_depth_m'])
        # A failed replacement must never leave an older successful report for
        # the same named position looking current while recording is underway.
        self.recordings.pop(label, None)
        self.publish_report()
        self.get_logger().info(f'Recording {label}: keep camera and object still')

    def on_frames(self, rgb, depth, info):
        try:
            now = self.get_clock().now().nanoseconds/1e9
            stamps = [m.header.stamp.sec+m.header.stamp.nanosec/1e9 for m in (rgb, depth, info)]
            if any(s <= 0 or not 0 <= now-s <= self.cfg['feedback_max_age'] for s in stamps):
                raise ValueError('Stale or future camera source timestamp')
            if max(stamps)-min(stamps) > self.cfg['synchronization_slop']:
                raise ValueError('RGB/depth/CameraInfo timestamps are not synchronized')
            if rgb.header.frame_id != self.cfg['optical_frame']:
                raise ValueError('Unexpected camera optical frame')
            source = stamps[1]
            if self.last_source is not None and source <= self.last_source:
                raise ValueError('Repeated or backwards depth timestamp; need new sensor frames')
            color, patch, stats = measure(rgb, depth, info, self.cfg['roi'], self.cfg['depth_scale'],
                self.cfg['approach_min_depth'], self.cfg['approach_max_depth'])
            self.last_source, self.last_receipt = source, time.monotonic()
            self.last, self.error = stats, None
            self.overlay(rgb, color, patch, stats)
            if self.active and min(stamps) >= self.active['ros_started']:
                samples = self.active['samples']
                if samples and source-samples[-1][0] > self.cfg['feedback_max_age']:
                    raise ValueError('Camera frame gap during recording')
                samples.append((source, stats))
                if source-samples[0][0] >= self.cfg['capture_seconds']:
                    self.finish()
        except (ValueError, TypeError) as error:
            self.error = str(error)
            if self.active:
                self.finish(self.error)

    def finish(self, failure=None):
        recording, self.active = self.active, None
        result = summarize(recording['samples'], self.cfg, recording['expected'])
        if failure:
            result['quality_criteria_met'] = False
            result['reasons'].append(failure)
        result.update(roi=recording['roi'], completed_stamp=self.get_clock().now().nanoseconds/1e9,
                      capture_completed=failure is None)
        self.recordings[recording['label']] = result
        self.publish_report()
        self.get_logger().info(f"{recording['label']}: quality={result['quality_criteria_met']}, "
            f"current_range={result['approach_range_compatible']}, median_m={result['depth_median_m']}")

    def report_context(self):
        c = self.cfg
        return dict(read_only=True, optical_frame=c['optical_frame'],
            approach_range_m=[c['approach_min_depth'], c['approach_max_depth']],
            approach_config_file=c['approach_config_file'],
            nominal_grasp_depth_m=c['nominal_grasp_depth_m'], geometry_source='nominal_CAD_reference_only',
            nominal_grasp_inside_approach_range=c['approach_min_depth'] <= c['nominal_grasp_depth_m'] <= c['approach_max_depth'],
            criteria={k: c[k] for k in ('capture_seconds', 'min_frames', 'min_valid_fraction',
                'max_spatial_spread_m', 'max_temporal_spread_m', 'max_reference_error_m')})

    def publish_report(self):
        report = dict(self.report_context(), recordings=self.recordings,
            active_capture=self.active['label'] if self.active else None,
            motion_authorized=False,
            interpretation='Surface-depth feasibility only; confirm ROI/object, geometry and depth accuracy separately')
        self.report_pub.publish(String(data=json.dumps(report, allow_nan=False)))

    def tick(self):
        now = time.monotonic()
        if self.active and now > self.active['deadline']:
            self.finish('Timed out waiting for enough fresh camera frames')
        stale = self.last_receipt is None or now-self.last_receipt > self.cfg['feedback_max_age']
        state = 'capturing' if self.active else 'observing'
        if self.error:
            state = 'invalid_camera_data'
        elif stale:
            state = 'waiting_for_camera' if self.last_receipt is None else 'stale_camera'
        payload = dict(self.report_context(), state=state, error=self.error,
            active_capture=self.active['label'] if self.active else None,
            recorded_frames=len(self.active['samples']) if self.active else 0,
            roi=self.cfg['roi'], latest=self.last if not stale and not self.error else None)
        self.status_pub.publish(String(data=json.dumps(payload, allow_nan=False)))
        if stale or self.error:
            # Do not leave a healthy-looking frozen camera overlay in the viewer.
            output = np.zeros((480, 640, 3), dtype=np.uint8)
            for i, line in enumerate(('READ ONLY | '+state,
                                      self.error or 'Waiting for fresh RGB-D and CameraInfo')):
                cv2.putText(output, line, (10, 30+i*30), cv2.FONT_HERSHEY_SIMPLEX,
                            .46, (255, 180, 0), 1)
            warning = Image(height=480, width=640, encoding='rgb8', step=640*3, data=output.tobytes())
            warning.header.frame_id = self.cfg['optical_frame']
            warning.header.stamp = self.get_clock().now().to_msg()
            self.image_pub.publish(warning)

    def overlay(self, message, color, patch, stats):
        output = color.copy()
        x0, y0, x1, y1 = stats['roi_pixels']
        valid = np.isfinite(patch) & (patch > 0)
        within = valid & (patch >= self.cfg['approach_min_depth']) & (patch <= self.cfg['approach_max_depth'])
        tint = np.zeros_like(output[y0:y1, x0:x1])
        tint[:] = [255, 50, 50]  # RGB: missing depth
        tint[valid] = [255, 180, 0]  # measured but outside configured approach range
        tint[within] = [0, 220, 220]
        output[y0:y1, x0:x1] = (0.65*output[y0:y1, x0:x1]+0.35*tint).astype(np.uint8)
        cv2.rectangle(output, (x0, y0), (x1-1, y1-1), (255, 255, 255), 1)
        median = stats['depth_median_m']
        depth_text = f'{median*100:.1f} cm' if median is not None else 'NO DEPTH'
        lines = [f'READ ONLY | ROI: {depth_text} | valid {stats["valid_fraction"]:.0%}',
                 'red: missing | amber: outside approach range | cyan: inside',
                 f'capture: {self.active["label"] if self.active else "idle"} | manually select OBJECT surface']
        for i, line in enumerate(lines):
            y = 18+i*20
            cv2.putText(output, line, (5, y), cv2.FONT_HERSHEY_SIMPLEX, .43, (0, 0, 0), 3)
            cv2.putText(output, line, (5, y), cv2.FONT_HERSHEY_SIMPLEX, .43, (255, 255, 255), 1)
        self.image_pub.publish(Image(header=copy.deepcopy(message.header), height=output.shape[0],
            width=output.shape[1], encoding='rgb8', step=output.shape[1]*3, data=output.tobytes()))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = False

    def interrupt(_signum, _frame):
        nonlocal stopping
        stopping = True

    # Terminal SIGINT and launch's forwarded SIGINT may both arrive. Mark the
    # stop request without raising another KeyboardInterrupt during cleanup.
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = WristDepthCheck()
    try:
        while not stopping and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
