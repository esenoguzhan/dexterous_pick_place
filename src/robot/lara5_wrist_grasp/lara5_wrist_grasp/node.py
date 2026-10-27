"""Explicit wrist-only final approach. No hand commands, lift or external camera."""

from collections import deque
import copy
import json
import math
import signal
import threading
import time
import uuid

import message_filters
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lara5_safety_msgs.action import MoveToPose
from lara5_safety_msgs.srv import ClosureHandoff, GetFramePose, PlanCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformListener

from lara5_grasp.configuration import HAND_JOINTS
from lara5_object_approach.grasp_geometry import pose_components, transform
from .core import (bounded_target, grasp_offset, pose_matrix,
                   require_hardware, rotation_error, vector)
from .selection import (DEFAULTS as SELECTION_DEFAULTS, select_candidates,
                        validate as validate_selection)
from .camera_mount import CameraMount, DEFAULTS as CAMERA_DEFAULTS
from .tracking import (DEFAULTS as TRACKING_DEFAULTS, Candidate, ConsistentFrames,
                       TargetTracker, validate as validate_tracking)

ARM = tuple(f'lara5_joint{i}' for i in range(1, 7))
TERMINAL = (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)


class StopUnconfirmed(RuntimeError):
    pass


class WristApproach(Node):
    def __init__(self, **kwargs):
        super().__init__('wrist_approach', **kwargs)
        defaults = dict(
            execution_enabled=False, **CAMERA_DEFAULTS, **TRACKING_DEFAULTS,
            base_alignment_confirmed=False,
            base_frame='lara5_root_link', optical_frame='wrist_color_optical_frame',
            hand_to_grasp_xyz=[-.025, .085, .15], **SELECTION_DEFAULTS,
            feedback_max_age=.5, synchronization_slop=.03,
            max_step=.01, max_initial_error=.08, max_travel=.10, max_path_length=.12,
            max_iterations=20, position_tolerance=.003, angle_tolerance=.02,
            open_tolerance=.04, joint_tolerance=.01, settle_time=.3,
            target_motion_tolerance=.02, max_cartesian_speed=.005, max_joint_speed=.15,
            min_segment_duration=2., service_timeout=15., goal_timeout=5.,
            execution_timeout_margin=10., cancel_timeout=5., overall_timeout=120.,
            workspace_min=[-.8, -.8, .05], workspace_max=[.8, .8, 1.0],
            color_topic='/cameras/wrist/color/image_raw',
            depth_topic='/cameras/wrist/aligned_depth_to_color/image_raw',
            info_topic='/cameras/wrist/color/camera_info')
        self.cfg = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        c = self.cfg
        validate_selection(c)
        validate_tracking(c)
        for key, value in defaults.items():
            if isinstance(value, (float, int)) and not isinstance(value, bool):
                if not math.isfinite(c[key]) or c[key] <= 0:
                    raise ValueError(f'{key} must be finite and positive')
        if c['base_frame'] != 'lara5_root_link':
            raise ValueError('Hardware wrist task uses lara5_root_link explicitly')
        if c['max_step'] > .02 or c['max_initial_error'] > .15 or c['max_cartesian_speed'] > .02:
            raise ValueError('Final-approach caps: step <= 2 cm, initial error <= 15 cm, speed <= 2 cm/s')
        if c['max_depth'] <= c['min_depth'] or c['max_joint_speed'] > .25:
            raise ValueError('Invalid depth range or joint speed above 0.25 rad/s')
        self.camera_mount = CameraMount(c)
        self.mount_report = None
        self.tracker = TargetTracker(c)
        self.monitor_context = None
        self.frame_error = None
        self.last_received_source = None
        self.acquisition_evidence = self.completion_evidence = None
        self.lower = vector(c['workspace_min'], 3, 'workspace_min')
        self.upper = vector(c['workspace_max'], 3, 'workspace_max')
        if np.any(self.lower >= self.upper):
            raise ValueError('workspace_min must be below workspace_max')
        self.frames = None
        self.frame_history = deque(maxlen=10)
        self.samples = {}
        self.history = deque()
        self.description = ''
        self.offset = None
        self.busy = False
        self.blocked = False
        self.closed = False
        self.ready = None
        self.ready_checked = None
        # Closure request holding the consumed approach result. Blocks arm commands.
        self.handoff = None
        self.grasp_active = False
        self.stop = threading.Event()
        self.worker = None
        self.delivery = self.goal_handle = self.result = None
        self.cancel_sent = False
        self.request_id = None
        self.run_deadline = None
        self.event = dict(state='idle', message='Manually position open hand, then request a wrist preview')
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, '/arm/wrist/status', retained)
        self.object_pub = self.create_publisher(PoseStamped, '/arm/wrist/object_pose', retained)
        self.target_pub = self.create_publisher(PoseStamped, '/arm/wrist/flange_target', retained)
        self.create_subscription(String, '/robot_description', self.on_description, retained)
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(String, '/arm/grasp/status', self.on_grasp, retained)
        self.create_subscription(Empty, '/arm/wrist/preview', lambda _: self.start(False), 1)
        self.create_subscription(Empty, '/arm/wrist/approach', lambda _: self.start(True), 1)
        self.create_subscription(Empty, '/arm/wrist/cancel', lambda _: self.request_stop(), 1)
        self.create_service(ClosureHandoff, '/arm/wrist/closure_handoff', self.on_handoff)
        self.rgb = message_filters.Subscriber(self, Image, c['color_topic'], qos_profile=qos_profile_sensor_data)
        self.depth = message_filters.Subscriber(self, Image, c['depth_topic'], qos_profile=qos_profile_sensor_data)
        self.info = message_filters.Subscriber(self, CameraInfo, c['info_topic'], qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb, self.depth, self.info], 10, c['synchronization_slop'])
        self.sync.registerCallback(self.on_frames)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.fk = self.create_client(GetFramePose, '/safety/arm/get_frame_pose')
        self.planner = self.create_client(PlanCartesianPath, '/safety/arm/plan_cartesian_path')
        self.arm = ActionClient(self, MoveToPose,
                                self.resolve_topic_name('/safety/arm/move_to_pose'))
        self.timer = self.create_timer(.1, self.tick)

    def report(self, state, message, **values):
        self.event = dict(state=state, message=message, **values)
        self.publish_status()
        self.get_logger().info(f'{state}: {message}')

    def publish_status(self):
        value = dict(self.event, mode='hardware', request_id=self.request_id,
                     active_request_id=self.request_id if self.busy else None,
                     execution_enabled=self.cfg['execution_enabled'],
                     ready_for_closure=self.ready is not None,
                     closure_handoff=self.handoff,
                     stamp=self.get_clock().now().nanoseconds/1e9,
                     camera_mount_mode=self.cfg['camera_mount_mode'],
                     camera_alignment_verified=self.cfg['camera_calibration_confirmed'],
                     camera_mount=self.mount_report,
                     target_tracking=self.tracker.report(),
                     acquisition_evidence=self.acquisition_evidence,
                     completion_evidence=self.completion_evidence,
                     holder_camera_collision_checked=False, target_collision_checked=False)
        if self.ready is not None:
            value['arm_positions'] = self.ready.tolist()
        self.status.publish(String(data=json.dumps(value)))

    def on_description(self, message):
        if self.description and message.data != self.description and (self.busy or self.ready is not None):
            self.frame_error = 'Robot description changed during target tracking'
        self.description = message.data

    def on_grasp(self, message):
        try:
            self.grasp_active = bool(json.loads(message.data).get('active_request_id'))
        except (ValueError, TypeError):
            self.grasp_active = True

    def on_handoff(self, request, response):
        # Runs in the default mutually exclusive callback group, like start() and
        # tick(), so each check-and-transfer is atomic. start() clears readiness and
        # the worker sets it only after its last arm command, so a claim cannot
        # overlap a correction, even just before the finished worker clears busy.
        closure = request.closure_request_id
        try:
            if not closure:
                raise ValueError('Handoff requires the closure request id')
            if request.operation == 'release':
                if self.handoff != closure:
                    raise ValueError('This closure does not hold the approach handoff')
                self.handoff = None
                self.report('released', 'Hand closure ended; a new explicit approach is required '
                            'before another closure', closure_request_id=closure)
            elif request.operation == 'claim':
                if self.handoff is not None:
                    raise ValueError('Another hand closure holds the approach handoff')
                if self.blocked:
                    raise ValueError('Wrist arm stop is unconfirmed')
                if self.ready is None:
                    raise ValueError('Wrist approach is still running' if self.busy
                                     else 'No unconsumed verified wrist approach result')
                if (self.ready_checked is None
                        or time.monotonic()-self.ready_checked > self.cfg['feedback_max_age']):
                    raise ValueError('Verified wrist approach has not been rechecked recently')
                if np.max(np.abs(self.measured()-self.ready)) > self.cfg['joint_tolerance']:
                    raise ValueError('Arm moved after verified wrist positioning')
                response.approach_request_id = self.request_id
                response.arm_positions = self.ready.tolist()
                self.handoff, self.ready = closure, None
                self.report('handed_off', 'Verified approach consumed by one hand closure; '
                            'arm commands blocked until it ends', closure_request_id=closure)
            else:
                raise ValueError('Handoff operation must be claim or release')
        except ValueError as error:
            response.success, response.message = False, str(error)
            response.approach_request_id, response.arm_positions = '', []
            return response
        response.success, response.message = True, f'{request.operation} accepted'
        return response

    def on_frames(self, rgb, depth, info):
        source = depth.header.stamp.sec+depth.header.stamp.nanosec/1e9
        if self.last_received_source is not None and source <= self.last_received_source:
            self.frame_error = 'Repeated or backwards camera source timestamp'
            return
        self.last_received_source = source
        if not self.busy and self.ready is None:
            self.frame_error = None
        self.frames = (rgb, depth, info, time.monotonic())
        self.frame_history.append(self.frames)

    def on_joints(self, message):
        if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
            self.samples = {}; self.history.clear()
            return
        now = time.monotonic()
        stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
        samples = dict(self.samples)
        for name, value in zip(message.name, message.position):
            if name in ARM+HAND_JOINTS:
                if not math.isfinite(value):
                    self.samples = {}; self.history.clear()
                    return
                samples[name] = (value, now, stamp)
        self.samples = samples
        if set(ARM).issubset(samples):
            self.history.append((now, np.array([samples[n][0] for n in ARM])))
            while self.history and now-self.history[0][0] > 2*self.cfg['settle_time']:
                self.history.popleft()

    def measured(self):
        now, ros_now = time.monotonic(), self.get_clock().now().nanoseconds/1e9
        samples = self.samples
        values = []
        for name in ARM+HAND_JOINTS:
            if name not in samples:
                raise ValueError(f'Missing measured joint {name}')
            value, receipt, stamp = samples[name]
            if now-receipt > self.cfg['feedback_max_age'] or not 0 <= ros_now-stamp <= self.cfg['feedback_max_age']:
                raise ValueError(f'Stale measured joint {name}')
            values.append(value)
        if max(abs(v) for v in values[6:]) > self.cfg['open_tolerance']:
            raise ValueError('Hand must remain open during final approach')
        return np.asarray(values[:6])

    def stationary(self, arm):
        history = list(self.history)
        if (not history or time.monotonic()-history[0][0] < self.cfg['settle_time']
                or any(np.max(np.abs(arm-past)) > self.cfg['joint_tolerance'] for _, past in history)):
            raise ValueError('Wait for measured arm to settle')

    def observe(self):
        try:
            if self.frame_error:
                raise ValueError(self.frame_error)
            return self._observe()
        except Exception as error:
            self.tracker.fail(error)
            raise

    def _observe(self):
        self.measured()
        if not self.frames:
            raise ValueError('No synchronized wrist RGB-D and CameraInfo')
        # Robot TF can arrive just after a camera frame. Use the most recent
        # still-fresh capture with its EXACT timestamp available in TF, rather
        # than substituting latest TF or blocking the ROS executor.
        selected = None
        for frame in reversed(list(self.frame_history)):
            rgb = frame[0]
            if self.tf.can_transform(self.cfg['base_frame'], 'lara5_flange',
                                     rclpy.time.Time.from_msg(rgb.header.stamp)):
                selected = frame
                break
        if selected is None:
            raise ValueError('Capture-time flange TF is unavailable for recent wrist frames')
        rgb, depth, info, receipt = selected
        now = self.get_clock().now().nanoseconds/1e9
        for msg in (rgb, depth, info):
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            if time.monotonic()-receipt > self.cfg['feedback_max_age'] or not 0 <= now-stamp <= self.cfg['feedback_max_age']:
                raise ValueError('Stale wrist camera measurement')
        if rgb.header.frame_id != self.cfg['optical_frame']:
            raise ValueError('Unexpected camera optical frame; check serial and optical_frame config')
        # Capture-time robot TF and one shared camera resolver serve preview/motion.
        stamped = self.tf.lookup_transform(self.cfg['base_frame'], 'lara5_flange',
                                           rclpy.time.Time.from_msg(rgb.header.stamp))
        t = stamped.transform
        base_flange = transform([t.translation.x, t.translation.y, t.translation.z],
                                [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
        c = self.cfg
        mount, self.mount_report = self.camera_mount.resolve(
            self.description, self.tf, rclpy.time.Time.from_msg(rgb.header.stamp))
        stamps = [m.header.stamp.sec+m.header.stamp.nanosec/1e9 for m in (rgb, depth, info)]
        if max(stamps)-min(stamps) > c['synchronization_slop']:
            raise ValueError('Unsynchronized RGB/depth/CameraInfo source timestamps')
        profile = (rgb.header.frame_id, depth.header.frame_id, info.header.frame_id,
                   info.width, info.height, tuple(info.k), tuple(info.d),
                   tuple(np.round(mount.flatten(), 8)))
        if self.tracker.stamp == stamps[1]:
            center = self.tracker.observe([], stamps[1], profile)
        else:
            _, selected = select_candidates(rgb, depth, info, c)
            if self.tracker.seed is None:
                if len(selected) != 1:
                    raise ValueError('Initial target acquisition requires one unambiguous depth object')
                if selected[0].depth_spread_m > c['max_depth_spread']:
                    raise ValueError('Selected surface depth spread exceeds max_depth_spread')
            candidates = [Candidate.from_selection(item, base_flange @ mount) for item in selected
                          if item.depth_spread_m <= c['max_depth_spread']]
            center = self.tracker.observe(candidates, stamps[1], profile)
        observed = self.pose(self.object_matrix(center))
        observed.header.stamp = copy.deepcopy(rgb.header.stamp)
        self.object_pub.publish(observed)
        return center, base_flange

    def acquire_target(self):
        evidence = ConsistentFrames(self.cfg, self.get_clock().now().nanoseconds/1e9)
        deadline = time.monotonic()+self.cfg['service_timeout']
        self.report('acquiring', 'Confirming one stationary depth target across new camera frames')
        orientation = None
        while time.monotonic() < deadline:
            self.check_deadline()
            center, flange = self.observe()
            arm = self.measured()
            try:
                self.stationary(arm)
            except ValueError:
                evidence = ConsistentFrames(self.cfg, self.get_clock().now().nanoseconds/1e9)
                self.acquisition_evidence = evidence.report()
                time.sleep(.02)
                continue
            if orientation is None:
                orientation = flange[:3, :3].copy()
            elif rotation_error(flange[:3, :3], orientation) > self.cfg['angle_tolerance']:
                raise ValueError('Manually chosen orientation changed during acquisition')
            confirmed = evidence.add(self.tracker.stamp, center)
            self.acquisition_evidence = evidence.report()
            if confirmed:
                return center, flange
            time.sleep(.02)
        raise ValueError('Target acquisition timed out waiting for consistent new camera frames')

    def check_deadline(self):
        if self.stop.is_set() or self.closed:
            raise RuntimeError('operator cancellation')
        if self.run_deadline is not None and time.monotonic() > self.run_deadline:
            raise RuntimeError('Overall final-approach time limit exceeded')

    def confirm_alignment(self, baseline, orientation, current):
        evidence = ConsistentFrames(self.cfg, self.get_clock().now().nanoseconds/1e9)
        deadline = time.monotonic()+self.cfg['service_timeout']
        self.report('verifying', 'Checking settled grasp alignment across distinct camera measurements')
        while time.monotonic() < deadline:
            self.check_deadline()
            self.monitor(baseline, orientation)
            arm = self.measured()
            try:
                self.stationary(arm)
            except ValueError:
                evidence = ConsistentFrames(self.cfg, self.get_clock().now().nanoseconds/1e9)
                self.completion_evidence = evidence.report()
                time.sleep(.02)
                continue
            measured = self.flange()
            center, captured = self.observe()
            stamp = self.tracker.stamp
            if (np.linalg.norm(measured[:3, 3]-current[:3, 3]) > self.cfg['position_tolerance']
                    or rotation_error(measured[:3, :3], orientation) > self.cfg['angle_tolerance']
                    or np.linalg.norm(measured[:3, 3]-captured[:3, 3]) > self.cfg['position_tolerance']):
                raise ValueError('Measured arm endpoint changed during final alignment verification')
            error = np.linalg.norm(center-(captured[:3, 3]+captured[:3, :3] @ self.offset))
            if error > self.cfg['position_tolerance']:
                self.completion_evidence = None
                return False
            confirmed = evidence.add(stamp, center)
            self.completion_evidence = evidence.report()
            if confirmed:
                return True
            time.sleep(.02)
        raise ValueError('Final alignment timed out waiting for consistent new camera frames')

    @staticmethod
    def object_matrix(center):
        result = np.eye(4); result[:3, 3] = center
        return result

    def pose(self, matrix):
        xyz, q = pose_components(matrix)
        msg = PoseStamped()
        msg.header.frame_id = self.cfg['base_frame']
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, xyz)
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = map(float, q)
        return msg

    def wait(self, future, timeout, monitor=None):
        deadline = time.monotonic()+timeout
        if self.stop.is_set() or self.closed:
            raise RuntimeError('operator cancellation')
        while not future.done():
            if self.stop.is_set() or self.closed:
                raise RuntimeError('operator cancellation')
            if time.monotonic() > deadline:
                raise RuntimeError('Native service/action timeout')
            if self.run_deadline is not None and time.monotonic() > self.run_deadline:
                raise RuntimeError('Overall final-approach time limit exceeded')
            if monitor:
                monitor()
            time.sleep(.02)
        result = future.result()
        if self.stop.is_set() or self.closed:
            raise RuntimeError('operator cancellation')
        if result is None:
            raise RuntimeError('Native service/action returned no response')
        return result

    def call(self, client, request):
        if not client.service_is_ready():
            raise ValueError(f'Native safety service unavailable: {client.srv_name}')
        monitor = (lambda: self.monitor(*self.monitor_context)) if self.monitor_context is not None else None
        response = self.wait(client.call_async(request), self.cfg['service_timeout'], monitor)
        if not response.success:
            raise ValueError(response.message)
        return response

    def flange(self):
        response = self.call(self.fk, GetFramePose.Request(frame='lara5_flange',
                                      reference_frame=self.cfg['base_frame']))
        if response.pose.header.frame_id != self.cfg['base_frame']:
            raise ValueError('Native FK returned an unexpected reference frame')
        return pose_matrix(response.pose.pose)

    def plan(self, current, target):
        c = self.cfg
        duration = max(c['min_segment_duration'], np.linalg.norm(target[:3, 3]-current[:3, 3])/c['max_cartesian_speed'])
        response = self.call(self.planner, PlanCartesianPath.Request(
            target=self.pose(target), relative=False, duration=float(duration)))
        path = response.trajectory
        if set(path.joint_names) != set(ARM) or not path.points:
            raise ValueError('Planner returned no complete six-joint trajectory')
        points = path.points
        previous, previous_time = self.measured(), 0.
        speed = 0.
        for point in points:
            if len(point.positions) != 6 or not all(math.isfinite(v) for v in point.positions):
                raise ValueError('Invalid native joint path')
            q = np.array([point.positions[path.joint_names.index(n)] for n in ARM])
            seconds = point.time_from_start.sec+point.time_from_start.nanosec/1e9
            if np.max(np.abs(q-previous)) > .2 or seconds < previous_time:
                raise ValueError('Native joint path has a jump or invalid time ordering')
            if seconds > previous_time:
                speed = max(speed, float(np.max(np.abs(q-previous))/(seconds-previous_time)))
            elif np.max(np.abs(q-previous)) > c['joint_tolerance']:
                raise ValueError('Native zero-time state differs from measured joints')
            previous, previous_time = q, seconds
        if previous_time <= 0:
            raise ValueError('Native plan duration must be positive')
        # Native motion uses a minimum-jerk clock. Increase duration conservatively
        # for its peak speed and request a new checked path at the longer duration.
        duration = max(duration, previous_time)
        if speed*1.875 > c['max_joint_speed']:
            duration *= speed*1.875/c['max_joint_speed']
        endpoint = self.call(self.fk, GetFramePose.Request(
            start_state=JointState(name=list(ARM), position=previous.tolist()),
            frame='lara5_flange', reference_frame=c['base_frame']))
        solved = pose_matrix(endpoint.pose.pose)
        if (np.linalg.norm(solved[:3, 3]-target[:3, 3]) > c['position_tolerance']
                or rotation_error(solved[:3, :3], target[:3, :3]) > c['angle_tolerance']):
            raise ValueError('Native path endpoint does not match wrist target')
        return float(duration)

    def start(self, execute):
        if self.busy or self.grasp_active or self.handoff:
            self.report('rejected_busy', 'An approach or hand closure is active; command discarded')
            return
        self.ready = None
        self.tracker = TargetTracker(self.cfg)
        self.monitor_context = None
        self.acquisition_evidence = self.completion_evidence = None
        self.stop.clear()
        self.busy = True
        self.request_id = str(uuid.uuid4())
        self.delivery = self.goal_handle = self.result = None
        self.cancel_sent = False
        self.report('requested', 'Starting fresh wrist preview' if not execute else 'Starting fresh final approach')
        self.worker = threading.Thread(target=self.run, args=(execute,), daemon=True)
        self.worker.start()

    def accepted(self, future):
        try:
            self.goal_handle = future.result()
            if self.goal_handle is not None and self.goal_handle.accepted:
                self.result = self.goal_handle.get_result_async()
                if self.stop.is_set():
                    self.cancel_goal()
        except Exception:
            # Ownership remains busy until a terminal result can be confirmed.
            pass

    def cancel_goal(self):
        if self.goal_handle is not None and self.goal_handle.accepted and not self.cancel_sent:
            try:
                self.goal_handle.cancel_goal_async()
                self.cancel_sent = True
            except Exception as error:
                self.get_logger().error(f'Cancellation delivery failed; ownership retained: {error}')

    def request_stop(self):
        self.stop.set()
        self.ready = None
        self.cancel_goal()

    def confirm_stop(self):
        if self.delivery is None:
            return
        self.request_stop()
        deadline = time.monotonic()+self.cfg['cancel_timeout']
        while time.monotonic() < deadline:
            if self.goal_handle is not None and not self.goal_handle.accepted:
                return
            if self.result is not None and self.result.done():
                result = self.result.result()
                if result is not None and result.status in TERMINAL:
                    return
            time.sleep(.02)
        self.blocked = True
        raise StopUnconfirmed('Arm stop unconfirmed; commands remain blocked until terminal result')

    def monitor(self, baseline, orientation):
        if self.grasp_active or self.handoff:
            raise ValueError('Hand closure is active; arm correction is not permitted')
        center, measured = self.observe()
        if np.linalg.norm(center-baseline) > self.cfg['target_motion_tolerance']:
            raise ValueError('Target moved or wrist estimate changed beyond the correction limit')
        if rotation_error(measured[:3, :3], orientation) > self.cfg['angle_tolerance']:
            raise ValueError('Flange orientation changed during the fixed-orientation approach')

    def move(self, target, duration, baseline, orientation):
        if self.stop.is_set() or self.closed or self.grasp_active or self.handoff:
            raise RuntimeError('Arm correction canceled before goal dispatch')
        if not self.arm.server_is_ready():
            raise ValueError('Safety-filtered arm action unavailable')
        self.delivery = self.arm.send_goal_async(MoveToPose.Goal(
            target=self.pose(target), relative=False, duration=duration))
        self.delivery.add_done_callback(self.accepted)
        self.wait(self.delivery, self.cfg['goal_timeout'], lambda: self.monitor(baseline, orientation))
        # accepted() executes on the ROS executor; allow its result registration.
        end = time.monotonic()+self.cfg['goal_timeout']
        while self.goal_handle is None or (self.goal_handle.accepted and self.result is None):
            if self.stop.is_set() or time.monotonic() > end:
                raise RuntimeError('Goal acceptance is unconfirmed')
            time.sleep(.01)
        if not self.goal_handle.accepted:
            raise ValueError('Native arm action rejected the final-approach segment')
        outcome = self.wait(self.result, duration+self.cfg['execution_timeout_margin'],
                            lambda: self.monitor(baseline, orientation))
        if outcome.status != GoalStatus.STATUS_SUCCEEDED or not outcome.result.success:
            raise ValueError('Native arm motion did not complete successfully')
        self.delivery = self.goal_handle = self.result = None
        self.cancel_sent = False
        # Require genuinely new, settled joint and camera feedback after result.
        result_time = time.monotonic()
        deadline = result_time+self.cfg['service_timeout']
        while time.monotonic() < deadline:
            if self.stop.is_set():
                raise RuntimeError('operator cancellation')
            self.monitor(baseline, orientation)
            try:
                arm = self.measured(); self.stationary(arm)
                if (all(self.samples[n][1] > result_time for n in ARM)
                        and self.frames and self.frames[3] > result_time):
                    measured = self.flange()
                    if (np.linalg.norm(measured[:3, 3]-target[:3, 3]) <= self.cfg['position_tolerance']
                            and rotation_error(measured[:3, :3], target[:3, :3]) <= self.cfg['angle_tolerance']):
                        return measured
            except ValueError:
                pass
            time.sleep(.03)
        raise ValueError('Measured final-approach endpoint was not verified')

    def run(self, execute):
        try:
            c = self.cfg
            self.run_deadline = time.monotonic()+c['overall_timeout']
            if execute and not (c['camera_calibration_confirmed'] and c['base_alignment_confirmed']):
                raise ValueError('Verify camera alignment against the actual assembly and robot base before motion')
            if execute and not c['execution_enabled']:
                raise ValueError('Execution disabled; review preview and enable it in the hardware config')
            if execute and not c['grasp_reference_confirmed']:
                raise ValueError('Verify surface_to_grasp_xyz for the test object/view before motion')
            require_hardware(self.description)
            self.offset = grasp_offset(self.description, c['hand_to_grasp_xyz'])
            arm = self.measured(); self.stationary(arm)
            center, camera_flange = self.acquire_target()
            current = self.flange()
            if (np.linalg.norm(current[:3, 3]-camera_flange[:3, 3]) > c['position_tolerance']
                    or rotation_error(current[:3, :3], camera_flange[:3, :3]) > c['angle_tolerance']):
                raise ValueError('Capture-time flange TF and measured native FK disagree')
            initial, orientation, baseline = current[:3, 3].copy(), current[:3, :3].copy(), center.copy()
            self.monitor_context = (baseline, orientation)
            target, error = bounded_target(current, center, self.offset, initial, c['max_step'],
                c['max_initial_error'], c['max_travel'], self.lower, self.upper)
            full = current.copy(); full[:3, 3] = center-current[:3, :3] @ self.offset
            self.target_pub.publish(self.pose(full))
            self.report('planning', 'Checking wrist-derived final approach with native NEURA IK', error_m=error)
            full_duration = self.plan(current, full) if error > c['position_tolerance'] else 0.
            if not execute:
                self.report('preview_ready', 'Checked final approach; no arm or hand command sent',
                            error_m=error, flange_xyz=full[:3, 3].tolist(), duration=full_duration,
                            camera_alignment_verified=c['camera_calibration_confirmed'],
                            base_alignment_verified=c['base_alignment_confirmed'])
                return
            deadline = time.monotonic()+c['overall_timeout']
            path_length = 0.
            for iteration in range(c['max_iterations']):
                if self.stop.is_set() or time.monotonic() > deadline:
                    raise RuntimeError('Final approach canceled or overall time limit exceeded')
                self.monitor(baseline, orientation)
                center, _ = self.observe()
                target, error = bounded_target(current, center, self.offset, initial, c['max_step'],
                    c['max_initial_error'], c['max_travel'], self.lower, self.upper)
                if error <= c['position_tolerance']:
                    if not self.confirm_alignment(baseline, orientation, current):
                        continue
                    self.stationary(self.measured())
                    self.ready = self.measured().copy()
                    self.ready_checked = time.monotonic()
                    self.report('succeeded', 'Wrist positioning verified; ready for explicit physical hand closure',
                                error_m=error, corrections=iteration)
                    return
                distance = float(np.linalg.norm(target[:3, 3]-current[:3, 3]))
                path_length += distance
                if path_length > c['max_path_length']:
                    raise ValueError('Cumulative visual correction path limit exceeded')
                before = self.measured().copy()
                duration = self.plan(current, target)
                self.stationary(self.measured())
                if np.max(np.abs(before-self.measured())) > c['joint_tolerance']:
                    raise ValueError('Arm moved while planning; discard this command')
                self.monitor(baseline, orientation)
                self.report('approaching', 'Executing bounded wrist correction', correction=iteration+1, error_m=error)
                current = self.move(target, duration, baseline, orientation)
            raise ValueError('Visual correction iteration limit exceeded')
        except Exception as error:
            self.ready = None
            operator_cancel = self.stop.is_set()
            try:
                self.confirm_stop()
            except StopUnconfirmed as stop_error:
                self.report('error', str(stop_error))
            else:
                self.report('canceled' if operator_cancel else 'failed', str(error))
        finally:
            self.run_deadline = None
            self.monitor_context = None
            if not self.blocked:
                self.busy = False
            self.publish_status()

    def tick(self):
        if self.stop.is_set() and self.busy:
            self.cancel_goal()
        if self.blocked and self.result is not None and self.result.done():
            try:
                outcome = self.result.result()
            except Exception:
                outcome = None
            if outcome is not None and outcome.status in TERMINAL:
                self.blocked = self.busy = False
                self.report('failed', 'Late terminal arm result confirmed; new explicit request is required')
        # Only a handoff claim consumes readiness. Grasp status can overtake its
        # claim, so it never invalidates; finger motion fails the open-hand check.
        if self.ready is not None and not self.busy:
            try:
                arm = self.measured()
                if np.max(np.abs(arm-self.ready)) > self.cfg['joint_tolerance']:
                    raise ValueError('Arm moved after verified wrist positioning')
                center, flange = self.observe()
                grasp_center = flange[:3, 3]+flange[:3, :3] @ self.offset
                if np.linalg.norm(center-grasp_center) > self.cfg['position_tolerance']:
                    raise ValueError('Object no longer aligned with the verified grasp region')
                self.ready_checked = time.monotonic()
            except Exception as error:
                self.ready = None
                self.report('invalidated', str(error))
        self.publish_status()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    node = WristApproach()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.request_stop()
        deadline = time.monotonic()+node.cfg['cancel_timeout']+1.
        while ((node.worker and node.worker.is_alive()) or node.blocked) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        if (node.worker and node.worker.is_alive()) or node.blocked:
            node.get_logger().error('Arm termination unconfirmed; check the hardware controller')
        node.closed = True
        node.destroy_node()
        rclpy.shutdown()
