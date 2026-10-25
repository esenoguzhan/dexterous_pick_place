"""Real ROS transport, synthetic measured feedback and an isolated fake hand."""
import copy
import json
import threading
import time
import uuid

import pytest
from control_msgs.action import FollowJointTrajectory
from lara5_safety_msgs.msg import SimGraspFeedback
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String

from lara5_grasp.configuration import HAND_JOINTS
from lara5_grasp.node import ARM_JOINTS, HandGrasp


def wait_for(predicate, timeout=6):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if predicate(): return
        time.sleep(.01)
    raise AssertionError('Closure workflow timed out')


class Harness:
    def __init__(self, with_server=True, parameters=None):
        prefix='/grasp_test_'+uuid.uuid4().hex
        self.context=Context(); self.context.init()
        self.peer=Node('fake_hand_peer',context=self.context)
        params=dict(close_positions=[.15]*5+[.10],thumb_opposition=.05,
            arm_stationary_window=.04,feedback_max_age=.2,verification_timeout=.25,
            goal_response_timeout=.2,cancel_timeout=.2)
        params.update(parameters or {})
        remaps=[]
        for source,suffix in (('/arm/grasp/target','target'),('/arm/grasp/status','status'),
            ('/hand_controller/follow_joint_trajectory','hand'),('/joint_states','state'),('/sim/grasp_feedback','contact')):
            remaps.extend(['-r',source+':='+prefix+'/'+suffix])
        self.grasp=HandGrasp(context=self.context,parameter_overrides=[Parameter(n,value=v) for n,v in params.items()],
                             cli_args=['--ros-args',*remaps])
        self.q=[0.]*6; self.arm=[0.]*6
        self.goals,self.pending,self.events=[],[],[]
        self.release=True; self.accept=True; self.abort=False; self.move=True
        self.send_states=True; self.send_contacts=True; self.stamp_offset=0.
        self.mode='bilateral'; self.force=.4; self.cancel_complete=True
        self.late_accept=False; self.acceptance=None; self.cancels=0
        group=ReentrantCallbackGroup()
        self.server=None
        if with_server:
            self.server=ActionServer(self.peer,FollowJointTrajectory,prefix+'/hand',
                execute_callback=self.execute,goal_callback=self.on_goal,cancel_callback=self.cancel,
                callback_group=group)
        self.pub=self.peer.create_publisher(Empty,prefix+'/target',1)
        self.state_pub=self.peer.create_publisher(JointState,prefix+'/state',10)
        self.contact_pub=self.peer.create_publisher(SimGraspFeedback,prefix+'/contact',10)
        self.sub=self.peer.create_subscription(String,prefix+'/status',lambda msg:self.events.append(json.loads(msg.data)),
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.timer=self.peer.create_timer(.01,self.tick,callback_group=group)
        self.executor=SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.peer); self.executor.add_node(self.grasp)
        self.thread=threading.Thread(target=self.executor.spin);self.thread.start()
        try:
            wait_for(lambda:self.pub.get_subscription_count()==1 and len(self.grasp.samples)==12
                     and self.grasp.contact is not None and self.grasp.history
                     and time.monotonic()-self.grasp.history[0][0]>.04)
            if with_server:wait_for(self.grasp.hand.server_is_ready)
        except BaseException:self.close();raise

    async def on_goal(self,_):
        if self.late_accept:
            self.acceptance=Future();await self.acceptance
        return GoalResponse.ACCEPT if self.accept else GoalResponse.REJECT

    def cancel(self,_):
        self.cancels+=1;return CancelResponse.ACCEPT

    async def execute(self,handle):
        self.goals.append(copy.deepcopy(handle.request))
        future=Future();self.pending.append((handle,future,time.monotonic()));await future
        if handle.is_cancel_requested:
            handle.canceled();return FollowJointTrajectory.Result(error_code=-1)
        if self.move:self.q=list(handle.request.trajectory.points[-1].positions)
        if self.abort:handle.abort()
        else:handle.succeed()
        return FollowJointTrajectory.Result(error_code=-1 if self.abort else 0)

    def tick(self):
        stamp=self.peer.get_clock().now().to_msg()
        stamp.sec-=round(self.stamp_offset)
        if self.send_states:
            msg=JointState(name=list(HAND_JOINTS+ARM_JOINTS),position=self.q+self.arm)
            msg.header.stamp=stamp;self.state_pub.publish(msg)
        if self.send_contacts:
            msg=SimGraspFeedback(object_name='green_cylinder')
            msg.header.frame_id='world';msg.header.stamp=stamp
            if self.mode=='always' or (self.mode=='bilateral' and len(self.goals)>=3):
                msg.contact_links=['right_thumb_distal_link','right_index_distal_link']
                msg.contact_normal_forces=[self.force]*2
            self.contact_pub.publish(msg)
        for handle,future,started in list(self.pending):
            if (self.release and time.monotonic()-started>.03) or (handle.is_cancel_requested and self.cancel_complete):
                self.pending.remove((handle,future,started));future.set_result(None)

    def send(self):self.pub.publish(Empty())
    def event(self,state):
        wait_for(lambda:any(e['state']==state for e in self.events))
        return next(e for e in reversed(self.events) if e['state']==state)
    def close(self):
        if self.acceptance and not self.acceptance.done():self.acceptance.set_result(None)
        self.release=True;self.cancel_complete=True
        wait_for(lambda:not self.pending)
        self.executor.shutdown(timeout_sec=5);self.thread.join(timeout=5)
        if self.server:self.server.destroy()
        self.grasp.destroy_node();self.peer.destroy_node();self.context.shutdown()


@pytest.fixture
def h():
    harness=Harness()
    try:yield harness
    finally:harness.close()


def test_contact_limited_closure_has_only_hand_goals_and_distinct_evidence(h):
    h.send();done=h.event('succeeded')
    assert done['contact_evidence'] and done['controller_motion_completed']
    assert done['retention_verified'] is False and done['closure_outcome']=='contact_limited'
    assert done['hold_mode']=='contact_preload'
    assert done['holding_preload_rad']==.25
    assert done['request_id']==h.event('opposing_thumb')['request_id']
    assert done['active_request_id'] is None
    assert len(h.goals)>=3
    for goal in h.goals:
        assert list(goal.trajectory.joint_names)==list(HAND_JOINTS)
        assert len(goal.trajectory.points)==2
    for point0,point1 in [(g.trajectory.points[0],g.trajectory.points[1]) for g in h.goals[:-1]]:
        assert max(abs(a-b) for a,b in zip(point0.positions,point1.positions))<=.05+1e-9
    preload=h.event('preloading_hold')
    assert preload['request_id']==done['request_id']
    last=h.goals[-1].trajectory.points
    duration=last[-1].time_from_start.sec+last[-1].time_from_start.nanosec/1e9
    assert duration>=3.
    delta=max(abs(a-b) for a,b in zip(last[0].positions,last[-1].positions))
    assert 1.5*delta/duration<=.20 and 6*delta/duration**2<=.50
    publishers=h.peer.get_publisher_names_and_types_by_node(h.grasp.get_name(),h.grasp.get_namespace())
    assert not any('approach/target' in name or 'joint_trajectory' in name for name,_ in publishers)


def test_no_object_contact_is_not_grasp_success(h):
    h.mode='none';h.send()
    done=h.event('failed')
    assert done['closure_outcome']=='no_object_contact'
    assert done['controller_motion_completed'] and not done['contact_evidence']
    assert done['hold_mode']=='measured_position'


@pytest.mark.parametrize('condition',['closed','missing','stale','arm_moving','no_contacts'])
def test_preconditions_reject_without_hand_motion(h,condition):
    if condition=='closed':
        h.q=[.2]*6;wait_for(lambda:h.grasp.samples[HAND_JOINTS[0]][0]==.2)
    elif condition=='missing':
        h.send_states=False;h.grasp.samples.clear()
    elif condition=='stale':
        h.stamp_offset=5.;wait_for(lambda:h.peer.get_clock().now().nanoseconds/1e9-h.grasp.samples[HAND_JOINTS[0]][2]>4)
    elif condition=='arm_moving':
        h.arm[0]=.2;wait_for(lambda:h.grasp.samples[ARM_JOINTS[0]][0]==.2)
    else:
        h.send_contacts=False;h.grasp.contact=None
    h.send();h.event('rejected_state');assert not h.goals


def test_busy_trigger_does_not_replace_active_closure(h):
    h.release=False;h.send();h.event('opposing_thumb')
    h.send();rejected=h.event('rejected_busy')
    assert rejected['request_id']!=rejected['active_request_id']
    h.release=True;h.event('succeeded')


@pytest.mark.parametrize('fault',['arm','force','feedback','contact_feedback','invalid_forces','joint_stall','abort'])
def test_motion_failures_stop_and_never_reopen_or_report_success(h,fault):
    h.release=False;h.send();wait_for(lambda:len(h.goals)==1)
    if fault=='arm':h.arm[0]+=.1
    if fault=='force':
        h.mode='always';h.force=20.
    if fault=='feedback':h.send_states=False
    if fault=='contact_feedback':h.send_contacts=False
    if fault=='invalid_forces':h.mode='always';h.force=float('nan')
    if fault=='joint_stall':h.move=False
    if fault=='abort':h.abort=True
    h.release=True
    h.event('failed')
    assert not any(e['state']=='succeeded' for e in h.events)
    assert h.grasp.active_request is None


def test_shutdown_cancellation_and_unknown_stop_lock(h):
    h.release=False;h.cancel_complete=False;h.send();wait_for(lambda:len(h.goals)==1)
    h.grasp.cancel_active();h.event('error')
    assert h.grasp.active_request
    h.send();h.event('rejected_busy')
    h.cancel_complete=True;h.event('canceled')
    assert h.grasp.active_request is None


def test_late_acceptance_after_timeout_is_canceled(h):
    h.release=False;h.late_accept=True;h.send();wait_for(lambda:h.acceptance is not None)
    h.event('canceling');assert h.grasp.active_request
    h.late_accept=False;h.acceptance.set_result(None);h.release=True
    h.event('failed');assert h.cancels>=1


def test_missing_controller():
    h=Harness(with_server=False)
    try:h.send();h.event('unavailable');assert not h.goals
    finally:h.close()


def test_force_limit_during_finite_holding_preload_stops_without_lift(h):
    h.release=False
    h.send()
    wait_for(lambda:len(h.goals)==1)
    # The synthetic contact is available from startup, so only the preload
    # segment follows thumb opposition; retain it until the fault arrives.
    h.mode='always'
    h.release=True
    wait_for(lambda:any(e['state']=='preloading_hold' for e in h.events))
    h.mode='always';h.force=20.
    h.event('failed')
    assert not any(e['state']=='succeeded' for e in h.events)


def test_explicit_joint_only_mode_does_not_claim_contact():
    h=Harness(parameters={'feedback_mode':'joint_only'})
    try:
        h.mode='none';h.send();done=h.event('succeeded')
        assert not done['contact_evidence'] and not done['retention_verified']
        assert done['closure_outcome']=='motion_only'
    finally:h.close()
