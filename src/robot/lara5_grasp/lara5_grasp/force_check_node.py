"""Read-only hand force recordings. Subscribes to measured state; commands nothing."""

import json
import math
import signal
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String
import yaml

from .configuration import DEFAULT_CLOSE, HAND_JOINTS
from .force_check import assess, closure_peak_speed, measurement, summarize
from .hardware import REPEAT_TOLERANCE, source_time

PREFIX = '/arm/grasp/force_check'
# Compared with the closure; loaded from hand_grasp in grasp_config_file if given.
CANDIDATE = dict(hardware_force_baseline=[0.]*6, hardware_force_contact_thresholds=[0.]*6,
                 hardware_force_limits=[0.]*6)
CLOSURE = dict(close_positions=list(DEFAULT_CLOSE), max_joint_step=.05, max_joint_speed=.20,
               max_joint_acceleration=.50, min_segment_duration=.25, open_start_tolerance=.08)
LIVE = ('loaded_channel', 'loaded_reference_n')


class HandForceCheck(Node):
    def __init__(self, **kwargs):
        super().__init__('hand_force_check', **kwargs)
        defaults = dict(CANDIDATE, **CLOSURE,
            measured_topic='/right_hand/measured_state', grasp_config_file='',
            feedback_max_age=.5, open_capture_seconds=3., min_capture_seconds=1., max_capture_seconds=60.,
            min_samples=30, stationary_tolerance=.02, loaded_motion_tolerance=.05,
            coverage_fraction=.9, speed_window_seconds=.2,
            threshold_margin_factor=1.5, threshold_margin_raw=2.,
            loaded_channel='', loaded_reference_n=0.)
        self.cfg = {key: self.declare_parameter(key, value, ParameterDescriptor(
            read_only=key not in LIVE)).value for key, value in defaults.items()}
        c = self.cfg
        c.update(self.load_grasp_config())
        for key, default in defaults.items():
            if (isinstance(default, (float, int)) and key != 'loaded_reference_n'
                    and (not math.isfinite(c[key]) or c[key] <= 0)):
                raise ValueError(f'{key} must be finite and positive')
        if (c['threshold_margin_factor'] < 1 or c['coverage_fraction'] > 1 or c['min_samples'] < 3
                or not c['min_capture_seconds'] <= c['max_capture_seconds'] <= 300):
            raise ValueError('Require margin factor >= 1, coverage <= 1, at least three samples '
                             'and min <= max capture <= 300 seconds')
        self.validate_live(c['loaded_channel'], c['loaded_reference_n'])
        self.active = None
        self.recordings = {}
        self.latest = self.last_source = self.last_receipt = self.error = self.reload_error = None
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, PREFIX+'/status', retained)
        self.report_pub = self.create_publisher(String, PREFIX+'/report', retained)
        for kind in ('unloaded_open', 'unloaded_motion', 'loaded'):
            self.create_subscription(Empty, f'{PREFIX}/record_{kind}', lambda _, k=kind: self.capture(k), 1)
        self.create_subscription(Empty, PREFIX+'/stop', lambda _: self.stop(), 1)
        self.create_subscription(Empty, PREFIX+'/reload', lambda _: self.reload(), 1)
        self.create_subscription(JointState, c['measured_topic'], self.on_measured, qos_profile_sensor_data)
        self.add_on_set_parameters_callback(self.parameters)
        self.create_timer(.2, self.tick)
        self.publish_report()
        self.get_logger().info('Read-only hand force check: records measured state; sends no commands')

    def load_grasp_config(self):
        """Candidate and closure settings; the closure's own YAML wins when given."""
        values = {key: self.get_parameter(key).value for key in (*CANDIDATE, *CLOSURE)}
        if self.cfg['grasp_config_file']:
            with open(self.cfg['grasp_config_file'], encoding='utf-8') as source:
                grasp = yaml.safe_load(source)['hand_grasp']['ros__parameters']
            # The same file the closure loads, so the comparison cannot diverge.
            values.update({key: grasp[key] for key in values if key in grasp})
        for key in (*CANDIDATE, 'close_positions'):
            values[key] = [float(v) for v in values[key]]
            if len(values[key]) != 6 or not all(math.isfinite(v) for v in values[key]):
                raise ValueError(f'{key} requires six finite values in HAND_JOINTS order')
        for key in CLOSURE:
            if key != 'close_positions':
                values[key] = float(values[key])
                if not math.isfinite(values[key]) or values[key] <= 0:
                    raise ValueError(f'{key} must be finite and positive')
        return values

    def reload(self):
        # Re-evaluate the stored recordings against an edited configuration.
        if self.active:
            self.get_logger().warning('Wait for the recording to finish before reloading')
            return
        try:
            self.cfg.update(self.load_grasp_config())
            self.reload_error = None
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
            self.reload_error = f'Reload failed; previous settings kept: {error}'
        self.publish_report()

    @staticmethod
    def validate_live(channel, reference):
        if channel and channel not in HAND_JOINTS:
            raise ValueError(f'loaded_channel must be empty or one of {", ".join(HAND_JOINTS)}')
        if not math.isfinite(reference) or reference < 0:
            raise ValueError('loaded_reference_n must be finite and nonnegative (zero means unknown)')

    def parameters(self, parameters):
        values = {key: self.cfg[key] for key in LIVE}
        try:
            if self.active:
                raise ValueError('Wait for the recording to finish before changing loaded settings')
            for param in parameters:
                if param.name not in LIVE:
                    raise ValueError('Only loaded_channel and loaded_reference_n may change live')
                values[param.name] = param.value
            self.validate_live(values['loaded_channel'], values['loaded_reference_n'])
        except (ValueError, TypeError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        self.cfg.update(values)
        return SetParametersResult(successful=True)

    def key(self, kind):
        return f'loaded/{self.cfg["loaded_channel"]}' if kind == 'loaded' else kind

    def capture(self, kind):
        if self.active:
            self.get_logger().warning('Recording already active; duplicate trigger discarded')
            return
        if kind == 'loaded' and not self.cfg['loaded_channel']:
            self.get_logger().error('Set loaded_channel to the pressed joint before recording a load')
            return
        now = time.monotonic()
        limit = self.cfg['open_capture_seconds'] if kind == 'unloaded_open' else self.cfg['max_capture_seconds']
        self.active = dict(kind=kind, key=self.key(kind), samples=[], repeats=0,
            ros_started=self.get_clock().now().nanoseconds/1e9, deadline=now+limit+2.,
            channel=self.cfg['loaded_channel'], reference=self.cfg['loaded_reference_n'])
        # A failed replacement must not leave an older result looking current.
        self.recordings.pop(self.active['key'], None)
        self.publish_report()
        self.get_logger().info(f'Recording {self.active["key"]}'
                               + ('' if kind == 'unloaded_open' else '; publish stop when done'))

    def stop(self):
        if self.active:
            self.finish()

    def on_measured(self, msg):
        try:
            stamp = source_time(msg)
            age = self.get_clock().now().nanoseconds/1e9-stamp
            if stamp <= 0 or not -.1 <= age <= self.cfg['feedback_max_age']:
                raise ValueError('Stale or future physical hand source timestamp')
            positions, forces = measurement(msg)
            if self.last_source is not None:
                if stamp < self.last_source-REPEAT_TOLERANCE:
                    raise ValueError('Physical hand source time moved backwards')
                if stamp <= self.last_source+REPEAT_TOLERANCE:
                    # Cached republish of the same physical read: not new evidence.
                    self.last_receipt = time.monotonic()
                    if self.active:
                        self.active['repeats'] += 1
                    return
            self.last_source, self.last_receipt = stamp, time.monotonic()
            self.latest, self.error = (positions, forces), None
            active = self.active
            if active and stamp >= active['ros_started']:
                samples = active['samples']
                if samples and stamp-samples[-1][0] > self.cfg['feedback_max_age']:
                    raise ValueError('Physical measurement gap during recording')
                samples.append((stamp, positions, forces))
                span = stamp-samples[0][0]
                if span >= (self.cfg['open_capture_seconds'] if active['kind'] == 'unloaded_open'
                            else self.cfg['max_capture_seconds']):
                    self.finish()
        except (ValueError, TypeError) as error:
            self.error = str(error)
            if self.active:
                self.finish(self.error)

    def finish(self, failure=None):
        recording, self.active = self.active, None
        recording.update(failure=failure, completed_stamp=self.get_clock().now().nanoseconds/1e9)
        self.recordings[recording['key']] = recording
        summary = self.publish_report()[recording['key']]['summary']
        self.get_logger().info(f"{recording['key']}: quality={summary['quality_criteria_met']} "
                               f"reasons={summary['reasons']}")

    def evaluated(self):
        # Raw samples are kept, so a reloaded closure configuration applies to all.
        result = {}
        for key, rec in self.recordings.items():
            summary = summarize(rec['kind'], rec['samples'], self.cfg, rec['channel'], rec['reference'])
            if rec['failure']:
                summary['quality_criteria_met'] = False
                summary['reasons'].append(rec['failure'])
            summary.update(cached_repeats_ignored=rec['repeats'], capture_completed=rec['failure'] is None,
                           completed_stamp=rec['completed_stamp'])
            result[key] = dict(summary=summary, samples=rec['samples'])
        return result

    def report_context(self):
        c = self.cfg
        return dict(read_only=True, hand_commands_sent=False, measured_topic=c['measured_topic'],
            grasp_config_file=c['grasp_config_file'], channel_order=list(HAND_JOINTS),
            units='raw driver actuator-force units; NOT N or Nm',
            candidate={k: c[k] for k in CANDIDATE},
            closure=dict({k: c[k] for k in CLOSURE}, nominal_peak_speed_rad_s=closure_peak_speed(c)),
            criteria={k: c[k] for k in ('open_capture_seconds', 'min_capture_seconds', 'max_capture_seconds',
                'min_samples', 'stationary_tolerance', 'loaded_motion_tolerance', 'coverage_fraction',
                'speed_window_seconds', 'threshold_margin_factor', 'threshold_margin_raw')})

    def publish_report(self):
        evaluated = self.evaluated()
        report = dict(self.report_context(),
            recordings={key: rec['summary'] for key, rec in evaluated.items()},
            active_capture=self.active['key'] if self.active else None, reload_error=self.reload_error,
            assessment=assess(evaluated, self.cfg, self.cfg),
            interpretation='Evidence for choosing raw force settings; confirmation remains an operator decision')
        self.report_pub.publish(String(data=json.dumps(report, allow_nan=False)))
        return evaluated

    def tick(self):
        now = time.monotonic()
        if self.active and now > self.active['deadline']:
            self.finish('Timed out waiting for new physical hand measurements')
        stale = self.last_receipt is None or now-self.last_receipt > self.cfg['feedback_max_age']
        state = 'recording' if self.active else 'observing'
        if self.error:
            state = 'invalid_hand_data'
        elif stale:
            state = 'waiting_for_hand' if self.last_receipt is None else 'stale_hand'
        latest = None
        if self.latest and not stale and not self.error:
            positions, forces = self.latest
            baseline, thresholds = self.cfg['hardware_force_baseline'], self.cfg['hardware_force_contact_thresholds']
            latest = {name: dict(position_rad=p, force_raw=f, deviation_from_candidate_baseline_raw=abs(f-b),
                                 above_candidate_threshold=t > 0 and abs(f-b) >= t)
                      for name, p, f, b, t in zip(HAND_JOINTS, positions, forces, baseline, thresholds)}
        payload = dict(read_only=True, state=state, error=self.error,
            active_capture=self.active['key'] if self.active else None,
            recorded_measurements=len(self.active['samples']) if self.active else 0,
            loaded_channel=self.cfg['loaded_channel'], loaded_reference_n=self.cfg['loaded_reference_n'],
            latest=latest)
        self.status_pub.publish(String(data=json.dumps(payload, allow_nan=False)))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = False

    def interrupt(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = HandForceCheck()
    try:
        while not stopping and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
