"""Real wrist and hand tasks share one handoff; fake native backends, no hardware."""

import copy
import json
import time

from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String

from lara5_grasp.configuration import HAND_JOINTS
from lara5_grasp.node import HandGrasp
from test_workflow import Harness, wait_for


class HandSide:
    """Hardware-mode hand task on the wrist harness's joint states and handoff."""
    def __init__(self, h):
        self.h, prefix = h, h.prefix
        self.goals, self.pending, self.events = [], [], []
        self.release, self.loaded = True, True
        remaps = []
        for source, target in (('/arm/grasp/status', 'grasp'), ('/arm/grasp/target', 'grasp_target'),
                ('/arm/grasp/cancel', 'grasp_cancel'), ('/hand_controller/follow_joint_trajectory', 'hand'),
                ('/joint_states', 'states'), ('/right_hand/measured_state', 'physical'),
                ('/arm/wrist/closure_handoff', 'handoff')):
            remaps += ['-r', source+':='+prefix+'/'+target]
        params = dict(feedback_mode='hardware', hardware_execution_enabled=True,
            hardware_force_limits_confirmed=True, hardware_force_contact_thresholds=[.15]*6,
            hardware_force_limits=[16.]*6, hardware_contact_stable_time=.04,
            close_positions=[.15]*5+[.10], thumb_opposition=.05, arm_stationary_window=.04,
            feedback_max_age=.3, verification_timeout=.5, goal_response_timeout=1., cancel_timeout=.2)
        self.node = HandGrasp(context=h.context, cli_args=['--ros-args', *remaps],
            parameter_overrides=[Parameter(key, value=value) for key, value in params.items()])
        group = ReentrantCallbackGroup()
        self.server = ActionServer(h.peer, FollowJointTrajectory, prefix+'/hand',
            execute_callback=self.execute, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group)
        self.physical = h.peer.create_publisher(JointState, prefix+'/physical', 10)
        self.trigger = h.peer.create_publisher(Empty, prefix+'/grasp_target', 1)
        h.peer.create_subscription(String, prefix+'/grasp', lambda msg: self.events.append(json.loads(msg.data)),
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.timer = h.peer.create_timer(.01, self.tick, callback_group=group)
        h.executor.add_node(self.node)
        wait_for(lambda: self.trigger.get_subscription_count() == 1 and self.node.hand.server_is_ready()
                 and self.node.handoff.service_is_ready() and self.node.physical is not None
                 and len(self.node.samples) == 12)

    async def execute(self, handle):
        self.goals.append(copy.deepcopy(handle.request))
        future = Future(); self.pending.append((handle, future, time.monotonic()))
        await future
        if handle.is_cancel_requested:
            handle.canceled(); return FollowJointTrajectory.Result(error_code=-1)
        self.h.hand_q = list(handle.request.trajectory.points[-1].positions)
        handle.succeed()
        return FollowJointTrajectory.Result(error_code=0)

    def tick(self):
        forces = [0.]*6
        if self.loaded and len(self.goals) >= 3:
            forces[1] = forces[5] = .4
        msg = JointState(name=list(HAND_JOINTS), position=list(self.h.hand_q), effort=forces)
        msg.header.stamp = self.h.peer.get_clock().now().to_msg()
        self.physical.publish(msg)
        for handle, future, started in list(self.pending):
            if (self.release and time.monotonic()-started > .03) or handle.is_cancel_requested:
                self.pending.remove((handle, future, started)); future.set_result(None)

    def event(self, state):
        wait_for(lambda: any(event['state'] == state for event in self.events))
        return next(event for event in reversed(self.events) if event['state'] == state)

    def close(self):
        self.release = True
        deadline = time.monotonic()+3
        while (self.pending or self.node.releasing is not None) and time.monotonic() < deadline:
            time.sleep(.01)
        self.h.executor.remove_node(self.node)
        self.server.destroy(); self.node.destroy_node()


def test_real_hand_closure_consumes_the_verified_approach_once():
    h = Harness({'execution_enabled': True})
    hand = None
    try:
        hand = HandSide(h)
        h.approach.publish(Empty()); ready = h.event('succeeded')
        arm_goals = len(h.goals)
        hand.release = False
        hand.trigger.publish(Empty()); wait_for(lambda: len(hand.goals) == 1)
        held = h.event('handed_off')
        assert held['closure_request_id'] == hand.node.active_request
        # Closing fingers occlude the wrist camera; closure uses hand feedback only.
        h.peer.no_camera = True
        # While the hand closes, no wrist command can move the arm.
        h.approach.publish(Empty()); h.event('rejected_busy')
        hand.release = True
        done = hand.event('succeeded')
        assert done['request_id'] == held['closure_request_id'] and done['contact_evidence']
        assert done['approach_request_id'] == ready['request_id'] and not done['retention_verified']
        assert done['force_evidence']['bilateral_new_measurements'] >= 5
        assert not any(e['state'] in ('invalidated', 'failed') for e in h.events)
        released = h.event('released')
        assert released['closure_request_id'] == done['request_id']
        assert len(h.goals) == arm_goals and h.node.ready is None and h.node.handoff is None
        # Reopen without a new approach: the consumed result cannot start another closure.
        hand.loaded = False; h.hand_q = [0.]*6
        wait_for(lambda: max(abs(hand.node.samples[name][0]) for name in HAND_JOINTS) == 0.)
        time.sleep(.1)
        goals = len(hand.goals)
        hand.trigger.publish(Empty())
        wait_for(lambda: hand.events[-1]['state'] == 'rejected_state'
                 and hand.events[-1]['request_id'] != done['request_id'])
        assert 'unconsumed' in hand.events[-1]['message'] and len(hand.goals) == goals
    finally:
        try:
            if hand:
                hand.close()
        finally:
            h.close()
