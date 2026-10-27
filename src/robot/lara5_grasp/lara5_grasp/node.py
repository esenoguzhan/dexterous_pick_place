"""Measured, contact-limited hand closure. Never commands the arm or reopens."""

from collections import deque
import copy
import json
import math
import signal
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from lara5_safety_msgs.msg import SimGraspFeedback
from lara5_safety_msgs.srv import ClosureHandoff
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .configuration import DEFAULT_CLOSE, HAND_JOINTS, UPPER_LIMITS, validate_closure
from .policy import contact_digits, freeze_contacts, next_positions, reached
from .hardware import ForceEvidence, physical_contacts, source_time, validate_force_limits

ARM_JOINTS = tuple(f'lara5_joint{i}' for i in range(1, 7))


def ros_duration(seconds):
    ns = round(seconds*1e9)
    return Duration(sec=ns//1_000_000_000, nanosec=ns % 1_000_000_000)


def distance(left, right):
    return max(abs(a-b) for a, b in zip(left, right))


class HandGrasp(Node):
    def __init__(self, **kwargs):
        super().__init__('hand_grasp', **kwargs)
        def param(name, default):
            return self.declare_parameter(name, default).value
        self.positions, _ = validate_closure(param('close_positions', list(DEFAULT_CLOSE)), 1.)
        self.opposition = param('thumb_opposition', .8)
        defaults = dict(max_joint_step=.05, max_joint_speed=.20, max_joint_acceleration=.50,
            min_segment_duration=.25, contact_preload=.04, hold_preload=.25,
            hold_preload_duration=3., contact_threshold_n=.15,
            max_contact_force_n=16., feedback_max_age=.5, open_start_tolerance=.08,
            joint_tolerance=.04, tracking_tolerance=.08, arm_stationary_tolerance=.01,
            arm_stationary_window=.20, verification_timeout=1., goal_response_timeout=5.,
            segment_timeout_margin=2., closure_timeout=60., cancel_timeout=5.)
        for name, default in defaults.items():
            value = param(name, default)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
            setattr(self, name, value)
        if not 0 <= self.opposition <= self.positions[0]:
            raise ValueError('thumb_opposition must lie between zero and close_positions[0]')
        if self.contact_preload > self.max_joint_step or self.max_contact_force_n <= self.contact_threshold_n:
            raise ValueError('Preload must not exceed step; maximum force must exceed contact threshold')
        if self.hold_preload > .25:
            raise ValueError('The finite simulation holding preload must not exceed 0.25 rad')
        self.feedback_mode = param('feedback_mode', 'simulation')
        self.target_object = param('target_object_name', 'green_cylinder')
        if self.feedback_mode not in ('simulation', 'joint_only', 'hardware') or not self.target_object:
            raise ValueError('feedback_mode must be simulation, joint_only or hardware')
        self.hardware_enabled = param('hardware_execution_enabled', False)
        self.force_confirmed = param('hardware_force_limits_confirmed', False)
        self.force_baseline = param('hardware_force_baseline', [0.]*6)
        self.force_thresholds = param('hardware_force_contact_thresholds', [0.]*6)
        self.force_limits = param('hardware_force_limits', [0.]*6)
        self.force_stable_time = param('hardware_contact_stable_time', .25)
        self.force_stable_samples = param('hardware_contact_stable_samples', 5)
        if not math.isfinite(self.force_stable_time) or self.force_stable_time <= 0:
            raise ValueError('hardware_contact_stable_time must be positive')
        # Validate the evidence rule at startup rather than at the first closure.
        ForceEvidence(self.force_stable_samples, self.force_stable_time, self.feedback_max_age)
        self.physical = None
        self.handoff = None
        self.releasing = None
        self.samples, self.history, self.contact = {}, deque(), None
        self.active = None
        self.stopping = False
        self.last_id = None
        self.hand = ActionClient(self, FollowJointTrajectory,
            self.resolve_topic_name('/hand_controller/follow_joint_trajectory'))
        self.status = self.create_publisher(String, '/arm/grasp/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.command = self.create_subscription(Empty, '/arm/grasp/target', self.on_trigger, 1)
        self.cancel_command = self.create_subscription(Empty, '/arm/grasp/cancel',
            lambda _: self.abort('operator cancellation'), 1)
        self.states = self.create_subscription(JointState, '/joint_states', self.on_state, 20)
        self.contacts = None
        if self.feedback_mode == 'hardware':
            self.contacts = self.create_subscription(JointState, '/right_hand/measured_state',
                self.on_physical, qos_profile_sensor_data)
            self.handoff = self.create_client(ClosureHandoff, '/arm/wrist/closure_handoff')
        else:
            self.contacts = self.create_subscription(SimGraspFeedback, '/sim/grasp_feedback', self.on_contact, 20)
        self.timer = self.create_timer(.02, self.on_timer)
        self.report('idle', 'Waiting for explicit closure; require open hand and stationary arm')

    @property
    def active_request(self):
        return self.active['id'] if self.active else None

    def report(self, state, message, request_id=None, **values):
        event = dict(state=state, message=message, request_id=request_id or self.last_id,
            active_request_id=self.active_request, retention_verified=False,
            feedback_mode=self.feedback_mode)
        event.update(values)
        self.status.publish(String(data=json.dumps(event)))
        self.get_logger().info(f'{state}: {message}')

    def on_state(self, msg):
        if (len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name)
                or any(not math.isfinite(v) for v in msg.position)):
            self.samples.clear(); self.history.clear()
            return
        now = time.monotonic()
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        for name, value in zip(msg.name, msg.position):
            if name in HAND_JOINTS+ARM_JOINTS:
                self.samples[name] = (value, now, stamp)
        try:
            arm = self.measured(ARM_JOINTS)
            if not self.history or now-self.history[-1][0] >= .01:
                self.history.append((now, arm))
            while self.history and now-self.history[0][0] > 2*self.arm_stationary_window:
                self.history.popleft()
        except ValueError:
            self.history.clear()

    def on_contact(self, msg):
        if msg.object_name != self.target_object:
            return
        try:
            contact_digits(msg.contact_links, msg.contact_normal_forces, self.contact_threshold_n)
            if msg.header.frame_id != 'world':
                raise ValueError('Unexpected simulation feedback frame')
            self.contact = (copy.deepcopy(msg), time.monotonic())
        except ValueError:
            self.contact = None

    def on_physical(self, msg):
        # Keep the physical source stamp. A newly received cached sample is old.
        self.physical = (copy.deepcopy(msg), time.monotonic())
        ctx = self.active
        if ctx is None or ctx['force_fault']:
            return
        try:
            # Check every measurement, not only those the 50 Hz timer samples.
            # Stability counts only new source times; cached repeats add nothing.
            _, digits = physical_contacts(msg, self.force_baseline, self.force_thresholds, self.force_limits)
            ctx['forces'].add(source_time(msg), 'thumb' in digits and bool(digits-{'thumb'}))
        except ValueError as error:
            ctx['force_fault'] = str(error)

    def hardware_ready(self):
        if not self.hardware_enabled or not self.force_confirmed:
            raise ValueError('Enable hardware execution and validate physical force limits in the hardware config')
        validate_force_limits(self.force_baseline, self.force_thresholds, self.force_limits)

    def measured(self, names=HAND_JOINTS):
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds/1e9
        values = []
        for name in names:
            if name not in self.samples:
                raise ValueError(f'Missing joint feedback: {name}')
            value, receipt, stamp = self.samples[name]
            if stamp <= 0 or now-receipt > self.feedback_max_age or not -.1 <= ros_now-stamp <= self.feedback_max_age:
                raise ValueError(f'Stale joint feedback: {name}')
            values.append(value)
        if names == HAND_JOINTS and any(v < -.01 or v > upper+.01 for v, upper in zip(values, UPPER_LIMITS)):
            raise ValueError('Measured hand is outside model joint limits')
        return values

    def feedback(self):
        if self.feedback_mode == 'hardware':
            validate_force_limits(self.force_baseline, self.force_thresholds, self.force_limits)
            if not self.physical:
                raise ValueError('Missing stamped physical hand feedback; rebuild inspire_dftp_e2')
            msg, receipt = self.physical
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            age = self.get_clock().now().nanoseconds/1e9-stamp
            if stamp <= 0 or time.monotonic()-receipt > self.feedback_max_age or not -.1 <= age <= self.feedback_max_age:
                raise ValueError('Stale physical hand force/angle measurement')
            if self.active and self.active['force_fault']:
                raise ValueError(self.active['force_fault'])
            _, digits = physical_contacts(msg, self.force_baseline, self.force_thresholds, self.force_limits)
            positions = dict(zip(msg.name, msg.position))
            if distance(self.measured(), [positions[n] for n in HAND_JOINTS]) > self.tracking_tolerance:
                raise ValueError('Physical hand and joint-state feedback disagree')
            return msg, digits
        if self.feedback_mode == 'joint_only':
            return None, set()
        if not self.contact:
            raise ValueError('Missing valid simulation contact/force feedback for configured fixture')
        msg, receipt = self.contact
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        age = self.get_clock().now().nanoseconds/1e9-stamp
        gap = time.monotonic()-receipt
        if stamp <= 0 or gap > self.feedback_max_age or not -.1 <= age <= self.feedback_max_age:
            raise ValueError(f'Stale simulation contact/force feedback '
                             f'(measurement_age={age:.3f}s, receive_gap={gap:.3f}s, '
                             f'limit={self.feedback_max_age:.3f}s)')
        if sum(msg.contact_normal_forces) > self.max_contact_force_n:
            raise ValueError('Target contact normal-force limit exceeded')
        return msg, contact_digits(msg.contact_links, msg.contact_normal_forces, self.contact_threshold_n)

    def preconditions(self):
        q, arm = self.measured(), self.measured(ARM_JOINTS)
        if self.feedback_mode == 'hardware':
            self.hardware_ready()
        if max(abs(v) for v in q) > self.open_start_tolerance:
            raise ValueError('Hand must start open; reopen explicitly before a new closure')
        if (not self.history or time.monotonic()-self.history[0][0] < self.arm_stationary_window
                or any(distance(arm, past) > self.arm_stationary_tolerance for _, past in self.history)):
            raise ValueError('Arm is not measured stationary; wait for positioning to finish')
        self.feedback()
        if self.feedback_mode == 'hardware' and self.feedback()[1]:
            raise ValueError('Hand is already loaded before closure; check baseline or contact')
        return q, arm

    def on_trigger(self, _):
        request_id = str(uuid.uuid4())
        if self.active or self.stopping:
            self.report('rejected_busy', 'Closure active or stopping; trigger discarded', request_id)
            return
        self.last_id = request_id
        if not self.hand.server_is_ready():
            self.report('unavailable', 'Hand action unavailable; trigger discarded')
            return
        if self.handoff is not None and not self.handoff.service_is_ready():
            self.report('unavailable', 'Wrist approach handoff unavailable; trigger discarded')
            return
        try:
            q, arm = self.preconditions()
        except ValueError as error:
            self.report('rejected_state', str(error))
            return
        self.active = dict(id=request_id, arm=arm, phase='opposing_thumb', state='ready',
            frozen={}, seen=set(), handle=None, failure=None, cancel_reason=None, cancel_sent=False,
            stop_reported=False, global_deadline=time.monotonic()+self.closure_timeout,
            step=0, stalled=0, previous=q, controller_motion_completed=False,
            handoff=None, approach_request_id=None, force_fault=None,
            forces=ForceEvidence(self.force_stable_samples, self.force_stable_time, self.feedback_max_age))
        if self.handoff is not None:
            self.claim()
            return
        self.report('requested', 'Starting bounded, measured hand closure')
        self.advance()

    def claim(self):
        # The wrist task grants each verified approach result to one closure and
        # blocks its arm commands until this closure releases it.
        ctx = self.active
        ctx.update(state='claiming', handoff='requested', deadline=time.monotonic()+self.goal_response_timeout)
        self.report('claiming', 'Claiming the verified wrist approach for this closure; no hand command yet')
        try:
            self.handoff.call_async(ClosureHandoff.Request(operation='claim', closure_request_id=ctx['id'])
                ).add_done_callback(lambda future: self.on_claim(ctx, future))
        except Exception as error:
            self.abort(f'Handoff claim delivery unknown: {error}')

    def on_claim(self, ctx, future):
        if self.active is not ctx:
            return
        try:
            response = future.result()
        except Exception as error:
            self.abort(f'Handoff claim result unknown: {error}')
            return
        if response is None or not response.success:
            ctx['handoff'] = None
            self.finish('rejected_state', 'Wrist approach handoff refused: '
                        + (response.message if response else 'no response'))
            return
        ctx.update(handoff='held', approach_request_id=response.approach_request_id)
        try:
            positions = list(response.arm_positions)
            if (not response.approach_request_id or len(positions) != 6
                    or any(not math.isfinite(v) for v in positions)):
                raise ValueError('Malformed wrist approach handoff')
            # Re-check after the round trip; the wrist result must match this arm.
            q, arm = self.preconditions()
            if distance(arm, positions) > self.arm_stationary_tolerance:
                raise ValueError('Arm moved since the verified wrist approach')
        except ValueError as error:
            self.abort(str(error))
            return
        ctx.update(state='ready', arm=arm, previous=q)
        self.report('requested', 'Starting bounded, measured hand closure',
                    approach_request_id=response.approach_request_id)
        self.advance()

    def release(self, ctx):
        # Requests from one client reach the wrist in order, so this also releases
        # a claim granted after its local timeout.
        held = ctx['handoff'] == 'held'
        try:
            future = self.handoff.call_async(ClosureHandoff.Request(
                operation='release', closure_request_id=ctx['id']))
        except Exception as error:
            self.get_logger().error(f'Handoff release delivery failed; restart both tasks: {error}')
            return
        self.releasing = future
        future.add_done_callback(lambda done: self.on_release(ctx['id'], held, done))

    def on_release(self, request_id, held, future):
        if self.releasing is future:
            self.releasing = None
        try:
            response = future.result()
        except Exception:
            response = None
        if held and (response is None or not response.success):
            self.get_logger().error(f'Handoff release for {request_id} unconfirmed; '
                                    'wrist commands stay blocked until both tasks restart')

    def observe(self):
        ctx = self.active
        q = self.measured()
        if distance(self.measured(ARM_JOINTS), ctx['arm']) > self.arm_stationary_tolerance:
            raise ValueError('Arm moved during independent closure')
        msg, digits = self.feedback()
        ctx['seen'].update(digits)
        freeze_contacts(ctx['frozen'], digits, q, self.positions,
                        0. if self.feedback_mode == 'hardware' else self.contact_preload)
        return q, msg, digits

    def advance(self):
        ctx = self.active
        try:
            q, _, digits = self.observe()
            target = list(self.positions)
            if ctx['phase'] == 'opposing_thumb':
                target = list(q); target[0] = self.opposition
                if reached([q[0]], [target[0]], {0: 0} if 0 in ctx['frozen'] else {}, self.joint_tolerance):
                    ctx['phase'] = 'closing'; target = list(self.positions)
            bilateral = 'thumb' in digits and bool(digits-{'thumb'})
            if ctx['phase'] == 'closing' and (bilateral or reached(q, target, ctx['frozen'], self.joint_tolerance)):
                if bilateral and self.feedback_mode == 'simulation':
                    # First contact is too weak to support the cylinder. Apply
                    # one finite flexion preload, never a feedback-driven ratchet.
                    # Thumb opposition remains at its contact-limited command.
                    command = list(q)
                    for index, value in ctx['frozen'].items():
                        command[index] = value
                    command = [max(0., min(upper, value))
                               for value, upper in zip(command, UPPER_LIMITS)]
                    for index in range(1, 6):
                        command[index] = min(self.positions[index], command[index]+self.hold_preload)
                    ctx['phase'] = 'preloading_hold'
                    self.send(command)
                    return
                self.begin_hold()
                return
            command = next_positions(q, target, ctx['frozen'], self.max_joint_step)
            self.send(command)
        except ValueError as error:
            self.abort(str(error))

    def send(self, command):
        ctx = self.active
        q = self.measured()
        delta = distance(q, command)
        duration = max(self.min_segment_duration, 1.5*delta/self.max_joint_speed,
                       math.sqrt(6*delta/self.max_joint_acceleration))
        if ctx['phase'] == 'preloading_hold':
            duration = max(duration, self.hold_preload_duration)
        goal = FollowJointTrajectory.Goal(trajectory=JointTrajectory(joint_names=list(HAND_JOINTS), points=[
            JointTrajectoryPoint(positions=q, velocities=[0.]*6, time_from_start=ros_duration(0)),
            JointTrajectoryPoint(positions=list(command), velocities=[0.]*6, time_from_start=ros_duration(duration))]),
            goal_time_tolerance=ros_duration(1.))
        ctx.update(state='sending', command=list(command), handle=None, cancel_sent=False,
                   deadline=time.monotonic()+self.goal_response_timeout, duration=duration, step=ctx['step']+1)
        self.report(ctx['phase'], 'Sending bounded hand-only segment', step=ctx['step'],
                    contact_digits=sorted(ctx['seen']))
        try:
            self.hand.send_goal_async(goal).add_done_callback(lambda future: self.on_goal(ctx, future))
        except Exception as error:
            self.abort(f'Goal delivery unknown: {error}')

    def on_goal(self, ctx, future):
        if self.active is not ctx:
            return
        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                self.finish('failed', ctx['failure'] or 'Hand controller rejected segment', hold_mode='last_controller_target')
                return
            ctx['handle'] = handle
            handle.get_result_async().add_done_callback(lambda result: self.on_result(ctx, result))
            if ctx['cancel_reason']:
                self.send_cancel()
            else:
                ctx.update(state='moving', deadline=time.monotonic()+ctx['duration']+self.segment_timeout_margin)
        except Exception as error:
            self.abort(f'Goal acceptance unknown: {error}')

    def on_result(self, ctx, future):
        if self.active is not ctx:
            return
        try:
            outcome = future.result()
        except Exception as error:
            self.abort(f'Controller result unknown: {error}')
            return
        if outcome is None or outcome.status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
            self.abort('Controller termination unconfirmed')
            return
        ctx['handle'] = None
        if ctx['cancel_reason']:
            self.after_stop()
            return
        if (outcome.status != GoalStatus.STATUS_SUCCEEDED or outcome.result is None
                or outcome.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
            ctx['failure'] = 'Hand controller did not complete segment'
            self.after_stop()
            return
        now = time.monotonic()
        ctx.update(state='verifying', result_time=now, deadline=now+self.verification_timeout)

    def verify(self):
        ctx = self.active
        q = self.measured()
        if any(self.samples[name][1] < ctx['result_time'] for name in HAND_JOINTS):
            return
        if ctx['phase'] == 'holding' and ctx['failure']:
            msg, digits = None, set()
        else:
            q, msg, digits = self.observe()
        # Contact can intentionally prevent the commanded preload being reached.
        preloaded = ctx['phase'] == 'preloading_hold'
        if any(abs(q[i]-ctx['command'][i]) > self.tracking_tolerance +
               (self.hold_preload if preloaded and i in ctx['frozen'] else 0.)
               for i in range(6) if preloaded or ctx['phase'] == 'holding' or i not in ctx['frozen']):
            ctx['verification_error'] = 'Incomplete measured closure without target contact'
            return
        if ctx['phase'] in ('holding', 'preloading_hold'):
            ctx['controller_motion_completed'] = True
            if self.feedback_mode == 'hardware' and not ctx['failure']:
                if not ('thumb' in digits and digits-{'thumb'}):
                    self.finish('failed', 'No bilateral physical force evidence; retention unverified',
                                closure_outcome='no_bilateral_force', hold_mode='measured_position')
                    return
                if not ctx['forces'].stable():
                    ctx['verification_error'] = ('Bilateral force not confirmed by enough new physical '
                                                 'measurements spanning the stable time')
                    return
            if ctx['failure']:
                self.finish('canceled' if ctx['failure'] == 'operator cancellation' else 'failed',
                            ctx['failure'], hold_mode='measured_position', measured_hold_verified=True)
            elif self.feedback_mode == 'simulation' and not digits:
                self.finish('failed', 'Closure completed without current target finger contact',
                            closure_outcome='no_object_contact', hold_mode='measured_position')
            else:
                self.finish('succeeded', 'Contact-limited closure completed; retention not verified' if digits
                    else 'Joint motion completed; contact and retention unverified',
                    closure_outcome='contact_limited' if digits else 'motion_only',
                    contact_evidence=bool(digits), current_contact_digits=sorted(digits),
                    contact_links=list(msg.contact_links) if msg and self.feedback_mode == 'simulation' else [],
                    contact_normal_forces_n=list(msg.contact_normal_forces) if msg and self.feedback_mode == 'simulation' else [],
                    actuator_forces_raw=list(msg.effort) if msg and self.feedback_mode == 'hardware' else [],
                    force_evidence_only=self.feedback_mode == 'hardware',
                    holding_preload_rad=self.hold_preload if preloaded else 0.,
                    measured_positions=q, hold_mode='contact_preload' if preloaded else 'measured_position',
                    measured_hold_verified=True)
            return
        progress = distance(q, ctx['previous'])
        ctx['stalled'] = ctx['stalled']+1 if progress < .001 else 0
        ctx['previous'] = q
        if ctx['stalled'] >= 3:
            self.abort('Incomplete measured closure: no joint progress without sufficient target contact')
            return
        ctx['state'] = 'ready'
        self.advance()

    def begin_hold(self):
        ctx = self.active
        q = self.measured()
        command = [max(0., min(upper, value)) for value, upper in zip(q, UPPER_LIMITS)]
        if not ctx['failure']:
            for index, value in ctx['frozen'].items():
                command[index] = value
        ctx.update(phase='holding', state='ready', cancel_reason=None, cancel_sent=False)
        self.send(command)

    def abort(self, reason):
        ctx = self.active
        if ctx is None:
            return
        if ctx['failure'] is None:
            ctx['failure'] = reason
        if ctx['state'] == 'claiming':
            # No hand command was sent; finish() releases any late claim grant.
            self.finish('canceled' if ctx['failure'] == 'operator cancellation' else 'failed', ctx['failure'])
            return
        if ctx['state'] in ('ready', 'verifying'):
            self.after_stop()
            return
        if ctx['cancel_reason'] is None:
            ctx.update(cancel_reason=reason, state='canceling', deadline=time.monotonic()+self.cancel_timeout)
            self.report('canceling', reason+'; requesting hand cancellation')
        self.send_cancel()

    def send_cancel(self):
        ctx = self.active
        if ctx['handle'] is not None and not ctx['cancel_sent']:
            ctx['cancel_sent'] = True
            try:
                ctx['handle'].cancel_goal_async()
            except Exception as error:
                self.report('error', f'Cancellation delivery failed; stop unconfirmed: {error}')

    def after_stop(self):
        ctx = self.active
        if ctx['phase'] == 'holding' or self.stopping:
            self.finish('canceled' if ctx['failure'] == 'operator cancellation' else 'failed',
                        ctx['failure'] or 'Hand motion canceled', hold_mode='controller_cancel', measured_hold_verified=False)
            return
        try:
            self.begin_hold()
        except ValueError:
            self.finish('failed', ctx['failure'] or 'Feedback unavailable after controller stop',
                        hold_mode='controller_cancel', measured_hold_verified=False)

    def on_timer(self):
        ctx = self.active
        if ctx is None:
            return
        now = time.monotonic()
        if ctx['state'] == 'claiming':
            if now > ctx['deadline']:
                self.abort('Wrist approach handoff claim unconfirmed')
            return
        if ctx['state'] == 'canceling':
            if now > ctx['deadline'] and not ctx['stop_reported']:
                ctx['stop_reported'] = True
                self.report('error', 'Hand stop unconfirmed; new triggers remain blocked')
            return
        try:
            if ctx['phase'] == 'holding' and ctx['failure']:
                self.measured()
            else:
                self.observe()
            if now > ctx['deadline'] or now > ctx['global_deadline']:
                self.abort(ctx.get('verification_error', 'Closure phase timed out'))
            elif ctx['state'] == 'verifying':
                self.verify()
        except ValueError as error:
            self.abort(str(error))

    def finish(self, state, message, **values):
        ctx = self.active
        self.active = None
        if ctx:
            values.setdefault('controller_motion_completed', ctx['controller_motion_completed'])
            values.setdefault('contact_evidence', False)
            values['contacted_digits_during_closure'] = sorted(ctx['seen'])
            values['segments'] = ctx['step']
            if ctx['approach_request_id']:
                values['approach_request_id'] = ctx['approach_request_id']
            if self.feedback_mode == 'hardware':
                values['force_evidence'] = ctx['forces'].report()
        self.report(state, message, **values)
        if ctx and ctx['handoff']:
            self.release(ctx)

    def cancel_active(self):
        self.stopping = True
        if self.active:
            self.abort('operator cancellation')


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(_signal, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    node = None
    try:
        node = HandGrasp()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.cancel_active()
            deadline = time.monotonic()+node.cancel_timeout
            while (rclpy.ok() and (node.active_request is not None or node.releasing is not None)
                    and time.monotonic()<deadline):
                rclpy.spin_once(node, timeout_sec=.05)
            if node.active_request is not None:
                node.get_logger().error('Hand stop not confirmed before exit; check controller')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
