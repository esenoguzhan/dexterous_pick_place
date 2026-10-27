"""Real ROS transport, synthetic measured feedback and an isolated fake hand."""
import copy
import json
import threading
import time
import uuid

import pytest
from control_msgs.action import FollowJointTrajectory
from lara5_safety_msgs.msg import SimGraspFeedback
from lara5_safety_msgs.srv import ClosureHandoff
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
    def __init__(self, with_server=True, parameters=None, with_handoff=True):
        prefix='/grasp_test_'+uuid.uuid4().hex
        self.context=Context(); self.context.init()
        self.peer=Node('fake_hand_peer',context=self.context)
        params=dict(close_positions=[.15]*5+[.10],thumb_opposition=.05,
            arm_stationary_window=.04,feedback_max_age=.2,verification_timeout=.25,
            goal_response_timeout=.2,cancel_timeout=.2)
        params.update(parameters or {})
        remaps=[]
        for source,suffix in (('/arm/grasp/target','target'),('/arm/grasp/status','status'),
            ('/hand_controller/follow_joint_trajectory','hand'),('/joint_states','state'),('/sim/grasp_feedback','contact'),
            ('/right_hand/measured_state','physical'),('/arm/wrist/closure_handoff','handoff'),('/arm/grasp/cancel','cancel')):
            remaps.extend(['-r',source+':='+prefix+'/'+suffix])
        self.grasp=HandGrasp(context=self.context,parameter_overrides=[Parameter(n,value=v) for n,v in params.items()],
                             cli_args=['--ros-args',*remaps])
        self.q=[0.]*6; self.arm=[0.]*6
        self.goals,self.pending,self.events=[],[],[]
        self.release=True; self.accept=True; self.abort=False; self.move=True
        self.send_states=True; self.send_contacts=True; self.stamp_offset=0.
        self.send_physical=True; self.physical_stamp_offset=0.
        self.repeat_on_contact=False; self.repeated_stamp=None; self.spike=False
        self.handoffs=[]; self.grant=True; self.claim_gate=None; self.claim_arm=None
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
        self.physical_pub=self.peer.create_publisher(JointState,prefix+'/physical',10)
        self.cancel_pub=self.peer.create_publisher(Empty,prefix+'/cancel',1)
        if with_handoff:
            self.peer.create_service(ClosureHandoff,prefix+'/handoff',self.on_handoff,callback_group=group)
        self.sub=self.peer.create_subscription(String,prefix+'/status',lambda msg:self.events.append(json.loads(msg.data)),
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.timer=self.peer.create_timer(.01,self.tick,callback_group=group)
        self.executor=SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.peer); self.executor.add_node(self.grasp)
        self.thread=threading.Thread(target=self.executor.spin);self.thread.start()
        try:
            wait_for(lambda:self.pub.get_subscription_count()==1 and self.cancel_pub.get_subscription_count()==1
                     and len(self.grasp.samples)==12
                     and (self.grasp.physical is not None if self.grasp.feedback_mode=='hardware'
                          else self.grasp.contact is not None) and self.grasp.history
                     and time.monotonic()-self.grasp.history[0][0]>.04)
            if with_server:wait_for(self.grasp.hand.server_is_ready)
            if with_handoff and self.grasp.handoff:wait_for(self.grasp.handoff.service_is_ready)
        except BaseException:self.close();raise

    async def on_handoff(self,request,response):
        # Fake wrist authority: records order, can refuse, delay or alter a claim.
        self.handoffs.append((request.operation,request.closure_request_id))
        if request.operation=='claim' and self.claim_gate is not None:await self.claim_gate
        response.success=self.grant or request.operation=='release'
        response.message='synthetic' if response.success else 'synthetic refusal'
        if request.operation=='claim' and response.success:
            response.approach_request_id='approach-1'
            response.arm_positions=list(self.claim_arm or self.arm)
        return response

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
        if self.send_physical:
            forces=[0.]*6
            if self.mode=='always' or (self.mode=='bilateral' and len(self.goals)>=3):
                forces[1]=forces[5]=self.force
            # Deliberately reverse order to model driver register ordering.
            msg=JointState(name=list(reversed(HAND_JOINTS)),
                position=list(reversed(self.q)),effort=list(reversed(forces)))
            msg.header.stamp=copy.deepcopy(stamp)
            msg.header.stamp.sec-=round(self.physical_stamp_offset)
            if self.repeat_on_contact and any(forces):
                # Model a driver that only republishes its cached measurement.
                self.repeated_stamp=self.repeated_stamp or copy.deepcopy(msg.header.stamp)
                msg.header.stamp=copy.deepcopy(self.repeated_stamp)
            if self.spike:
                # One over-limit reading 3 ms before a normal one in the same tick.
                early=copy.deepcopy(msg);early.effort=[20.]*6
                ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec-3_000_000
                early.header.stamp.sec,early.header.stamp.nanosec=divmod(ns,10**9)
                self.physical_pub.publish(early);self.spike=False
            self.physical_pub.publish(msg)
        for handle,future,started in list(self.pending):
            if (self.release and time.monotonic()-started>.03) or (handle.is_cancel_requested and self.cancel_complete):
                self.pending.remove((handle,future,started));future.set_result(None)

    def send(self):self.pub.publish(Empty())
    def event(self,state):
        wait_for(lambda:any(e['state']==state for e in self.events))
        return next(e for e in reversed(self.events) if e['state']==state)
    def operations(self):return [operation for operation,_ in self.handoffs]
    def close(self):
        if self.acceptance and not self.acceptance.done():self.acceptance.set_result(None)
        if self.claim_gate and not self.claim_gate.done():self.claim_gate.set_result(None)
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


def physical_harness(with_handoff=True,**params):
    options=dict(feedback_mode='hardware',hardware_execution_enabled=True,
        hardware_force_limits_confirmed=True,hardware_force_contact_thresholds=[.15]*6,
        hardware_force_limits=[16.]*6,hardware_contact_stable_time=.04,verification_timeout=.5)
    options.update(params)
    return Harness(parameters=options,with_handoff=with_handoff)


def test_hardware_closure_uses_physical_named_feedback_without_simulation_contacts():
    h=physical_harness()
    try:
        h.send_contacts=False;h.grasp.contact=None
        h.send();done=h.event('succeeded')
        assert done['feedback_mode']=='hardware' and done['force_evidence_only']
        assert done['contact_evidence'] and not done['retention_verified']
        assert done['holding_preload_rad']==0 and done['contact_normal_forces_n']==[]
        assert done['contact_links']==[] and len(done['actuator_forces_raw'])==6
        assert not any(e['state']=='preloading_hold' for e in h.events)
        assert done['approach_request_id']=='approach-1'
        wait_for(lambda:len(h.handoffs)==2)
        assert h.handoffs==[('claim',done['request_id']),('release',done['request_id'])]
    finally:h.close()


@pytest.mark.parametrize('fault',['disabled','unconfigured_force','stale_physical','handoff_refused'])
def test_hardware_preconditions_send_no_hand_command(fault):
    params={}
    if fault=='disabled':params['hardware_execution_enabled']=False
    if fault=='unconfigured_force':params['hardware_force_limits_confirmed']=False
    h=physical_harness(**params)
    try:
        if fault=='handoff_refused':h.grant=False
        if fault=='stale_physical':
            h.physical_stamp_offset=5.
            wait_for(lambda:h.grasp.physical[0].header.stamp.sec<h.peer.get_clock().now().to_msg().sec-3)
        h.send();rejected=h.event('rejected_state')
        # A refused claim is not held, so nothing is released.
        time.sleep(.1)
        assert not h.goals and h.operations()==(['claim'] if fault=='handoff_refused' else [])
        assert h.grasp.active_request is None
        if fault=='handoff_refused':assert 'refused' in rejected['message']
    finally:h.close()


def test_hardware_closure_requires_the_wrist_handoff_service():
    h=physical_harness(with_handoff=False)
    try:h.send();h.event('unavailable');assert not h.goals
    finally:h.close()


def test_handoff_claim_completes_before_any_hand_command_and_blocks_triggers():
    h=physical_harness(goal_response_timeout=2.)
    try:
        h.claim_gate=Future();h.send();claiming=h.event('claiming')
        assert claiming['active_request_id']==claiming['request_id']
        h.send();h.event('rejected_busy')
        time.sleep(.1);assert not h.goals and h.operations()==['claim']
        h.claim_gate.set_result(None);done=h.event('succeeded')
        assert done['request_id']==claiming['request_id']
    finally:h.close()


@pytest.mark.parametrize('fault',['timeout','cancel','moved'])
def test_failed_handoff_releases_any_late_grant_without_hand_motion(fault):
    h=physical_harness()
    try:
        h.claim_gate=Future()
        if fault=='moved':h.claim_arm=[.05]+[0.]*5
        h.send();claiming=h.event('claiming')
        if fault=='cancel':h.cancel_pub.publish(Empty())
        if fault=='moved':h.claim_gate.set_result(None)
        done=h.event('canceled' if fault=='cancel' else 'failed')
        assert done['request_id']==claiming['request_id'] and done['active_request_id'] is None
        assert {'timeout':'unconfirmed','cancel':'cancellation','moved':'moved'}[fault] in done['message']
        # Release follows the claim from the same client, so the wrist frees a late grant.
        wait_for(lambda:h.operations()==['claim','release'])
        assert h.handoffs[1][1]==claiming['request_id']
        if not h.claim_gate.done():h.claim_gate.set_result(None)
        time.sleep(.2);assert not h.goals and h.grasp.active_request is None
    finally:h.close()


@pytest.mark.parametrize('fault',['force','stale','arm'])
def test_hardware_faults_cancel_motion_and_do_not_report_retention(fault):
    h=physical_harness()
    try:
        h.release=False;h.send();wait_for(lambda:len(h.goals)==1)
        if fault=='force':h.mode='always';h.force=20.
        if fault=='stale':h.send_physical=False
        if fault=='arm':h.arm[0]=.1
        h.event('canceling');h.release=True
        done=h.event('failed')
        assert not done['retention_verified'] and not any(e['state']=='succeeded' for e in h.events)
    finally:h.close()


def test_hardware_stability_ignores_repeated_checks_of_one_cached_measurement():
    # The former elapsed-time rule accepted one fresh cached sample re-checked
    # across the stable time. Only new source times may count.
    h=physical_harness(feedback_max_age=1.,verification_timeout=.5)
    try:
        h.repeat_on_contact=True;h.send();done=h.event('failed')
        assert 'new physical measurements' in done['message']
        assert done['force_evidence']['bilateral_new_measurements']==1
        assert not any(e['state']=='succeeded' for e in h.events)
    finally:h.close()


def test_hardware_success_reports_distinct_measurement_evidence():
    h=physical_harness()
    try:
        h.send();done=h.event('succeeded')
        evidence=done['force_evidence']
        assert evidence['bilateral_new_measurements']>=evidence['required_measurements']==5
        assert evidence['source_span_seconds']>=evidence['required_span_seconds']
        assert not done['retention_verified']
    finally:h.close()


def test_force_limit_is_checked_on_every_measurement_between_timer_samples():
    h=physical_harness()
    try:
        h.release=False;h.send();wait_for(lambda:len(h.goals)==1)
        h.spike=True;wait_for(lambda:not h.spike)
        h.release=True;done=h.event('failed')
        assert 'limit exceeded' in done['message']
        assert not any(e['state']=='succeeded' for e in h.events)
    finally:h.close()


def test_hardware_full_motion_without_bilateral_force_is_failure():
    h=physical_harness()
    try:
        h.mode='none';h.send()
        assert h.event('failed')['closure_outcome']=='no_bilateral_force'
    finally:h.close()
