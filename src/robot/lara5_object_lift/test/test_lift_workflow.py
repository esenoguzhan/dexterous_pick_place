"""Isolated ROS services/actions and synthetic object physics; never real arm calls."""
import copy
import json
import threading
import time
import uuid

import numpy as np
import pytest
from lara5_safety_msgs.action import MoveToPose
from lara5_safety_msgs.msg import SimGraspFeedback
from lara5_safety_msgs.srv import GetFramePose, SetGraspScene
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
from lara5_object_approach.grasp_geometry import from_rpy
from lara5_object_approach.preview import ros_pose
from lara5_object_lift.node import ARM_JOINTS, ObjectLift


def wait_for(predicate,timeout=6):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if predicate():return
        time.sleep(.01)
    raise AssertionError('Lift workflow timed out')


class Model:
    def check(self,q):
        q=np.asarray(q,dtype=float)
        if q.shape!=(6,) or not np.isfinite(q).all():raise ValueError('Bad toy arm')
        return q
    def fk(self,q):return from_rpy(self.check(q)[:3],[0.,0.,0.])


class Harness:
    def __init__(self,server=True):
        self.context=Context();self.context.init()
        self.peer=Node('lift_peer',context=self.context)
        self.prefix='/lift_test_'+uuid.uuid4().hex
        remaps=[]
        for source,suffix in (('/arm/lift/target','target'),('/arm/lift/status','status'),
            ('/arm/lift/markers','markers'),('/arm/grasp/status','grasp'),('/arm/approach/status','approach'),
            ('/joint_states','states'),('/sim/grasp_feedback','feedback'),
            ('/safety/arm/move_to_pose','motion'),('/safety/arm/get_frame_pose','fk'),
            ('/safety/arm/set_grasp_scene','scene')):
            remaps.extend(['-r',source+':='+self.prefix+'/'+suffix])
        self.node=ObjectLift(context=self.context,cli_args=['--ros-args',*remaps],parameter_overrides=[
            Parameter(name,value=value) for name,value in dict(grip_settle_time=.05,grip_timeout=.3,
                hold_settle_time=.05,endpoint_timeout=.4,feedback_max_age=.2,contact_loss_grace=.05,
                goal_response_timeout=.2,service_timeout=.5,cancel_timeout=.2).items()])
        self.node.model=Model()
        self.arm=[.1,-.225,1.035,0.,0.,0.];self.hand=[.8,.6,.6,.6,.6,.1]
        self.grasp_hand=self.hand.copy();self.object=[.25,-.2,.95]
        self.authorized=True;self.grip=True;self.send_state=True;self.send_feedback=True
        self.fault=None;self.release=True;self.accept=True;self.cancel_complete=True
        self.goals=[];self.scenes=[];self.pending=[];self.events=[]
        self.late_accept=False;self.acceptance=None;self.cancels=0
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_sub=self.peer.create_subscription(String,self.prefix+'/status',lambda msg:self.events.append(json.loads(msg.data)),qos)
        self.pub=self.peer.create_publisher(Empty,self.prefix+'/target',1)
        self.state_pub=self.peer.create_publisher(JointState,self.prefix+'/states',10)
        self.feedback_pub=self.peer.create_publisher(SimGraspFeedback,self.prefix+'/feedback',10)
        self.grasp_pub=self.peer.create_publisher(String,self.prefix+'/grasp',qos)
        group=ReentrantCallbackGroup()
        self.fk=self.peer.create_service(GetFramePose,self.prefix+'/fk',self.on_fk,callback_group=group)
        self.scene=self.peer.create_service(SetGraspScene,self.prefix+'/scene',self.on_scene,callback_group=group)
        self.server=ActionServer(self.peer,MoveToPose,self.prefix+'/motion',execute_callback=self.execute,
            goal_callback=self.on_goal,cancel_callback=self.on_cancel,callback_group=group) if server else None
        self.timer=self.peer.create_timer(.01,self.tick,callback_group=group)
        self.executor=SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node);self.executor.add_node(self.peer)
        self.thread=threading.Thread(target=self.executor.spin);self.thread.start()
        try:
            wait_for(lambda:self.pub.get_subscription_count()==1 and len(self.node.samples)==12
                     and self.node.grasp is not None and self.node.contact is not None
                     and self.node.scene.service_is_ready() and self.node.fk.service_is_ready())
            if server:wait_for(self.node.motion.server_is_ready)
        except BaseException:self.close();raise

    def tick(self):
        stamp=self.peer.get_clock().now().to_msg()
        if self.send_state:
            msg=JointState(name=list(ARM_JOINTS+HAND_JOINTS),position=self.arm+self.hand)
            msg.header.stamp=stamp;self.state_pub.publish(msg)
        if self.send_feedback:
            msg=SimGraspFeedback(object_name='green_cylinder');msg.header.stamp=stamp;msg.header.frame_id='world'
            msg.object_pose=ros_pose(from_rpy(self.object,[0.,0.,0.]))
            msg.object_in_flange=ros_pose(from_rpy(np.asarray(self.object)-self.arm[:3],[0.,0.,0.]))
            if self.grip:
                msg.contact_links=['right_thumb_4_link','right_index_distal_link'];msg.contact_normal_forces=[1.]*2
                msg.opposing_contacts=True
            msg.supported=self.object[2]<.97 or self.fault=='supported'
            msg.linear_speed=.001;self.feedback_pub.publish(msg)
        self.grasp_pub.publish(String(data=json.dumps(dict(state='succeeded' if self.authorized else 'failed',
            request_id='fixture_grasp',active_request_id=None,contact_evidence=True,controller_motion_completed=True,
            measured_hold_verified=True,feedback_mode='simulation',measured_positions=self.grasp_hand))))
        for handle,future,started in list(self.pending):
            if (handle.is_cancel_requested and self.cancel_complete) or (self.release and time.monotonic()-started>.08):
                self.pending.remove((handle,future,started));future.set_result(None)

    def on_fk(self,request,response):
        response.success=self.fault!='fk';response.pose.header.frame_id='world'
        response.pose.pose=ros_pose(Model().fk(request.start_state.position));return response

    def on_scene(self,request,response):
        self.scenes.append(copy.deepcopy(request))
        response.success=not (self.fault=='scene' and request.attach_object)
        response.message='fixture scene rejected' if not response.success else 'scene updated'
        if self.fault=='cleanup' and not request.attach_object:response.success=False
        return response

    async def on_goal(self,request):
        if self.late_accept:
            self.acceptance=Future();await self.acceptance
        return GoalResponse.ACCEPT if self.accept else GoalResponse.REJECT

    def on_cancel(self,handle):self.cancels+=1;return CancelResponse.ACCEPT

    async def execute(self,handle):
        self.goals.append(copy.deepcopy(handle.request))
        done=Future();self.pending.append((handle,done,time.monotonic()));await done
        if handle.is_cancel_requested:
            handle.canceled();return MoveToPose.Result(success=False,message='canceled')
        if self.fault in ('abort','cleanup'):
            handle.abort();return MoveToPose.Result(success=False,message='fixture controller failure')
        p=handle.request.target.pose.position
        delta=p.z-self.arm[2]
        self.arm[:3]=[p.x,p.y,p.z]
        if self.fault!='slip':self.object[2]+=delta
        handle.succeed();return MoveToPose.Result(success=True,message='fixture motion complete')

    def send(self):self.pub.publish(Empty())
    def event(self,state):
        wait_for(lambda:any(e['state']==state for e in self.events))
        return next(e for e in reversed(self.events) if e['state']==state)
    def close(self):
        self.release=True;self.cancel_complete=True
        if self.acceptance and not self.acceptance.done():self.acceptance.set_result(None)
        wait_for(lambda:not self.pending)
        self.executor.shutdown(timeout_sec=5);self.thread.join(timeout=5)
        if self.server:self.server.destroy()
        self.node.destroy_node();self.peer.destroy_node();self.context.shutdown()


@pytest.fixture
def h():
    harness=Harness()
    try:yield harness
    finally:harness.close()


def test_verified_two_stage_lift_requires_object_following_and_removes_support_exception(h):
    h.send();result=h.event('succeeded')
    assert result['retention_verified'] and result['object_rise_m']==pytest.approx(.10)
    assert len(h.goals)==2
    assert [g.target.pose.position.z for g in h.goals]==pytest.approx([1.075,1.135])
    assert not any(g.relative for g in h.goals)
    assert h.scenes[0].attach_object and h.scenes[0].allow_support_contact
    assert h.scenes[1].attach_object and not h.scenes[1].allow_support_contact
    assert h.scenes[0].object_size.z==pytest.approx(.12)
    assert result['supported'] is False and result['payload_collision_checked'] is True
    h.send();h.event('rejected_state');assert len(h.goals)==2


@pytest.mark.parametrize('fault',['fk','scene'])
def test_failed_preparation_never_commands_arm(h,fault):
    h.fault=fault;h.send();h.event('failed');assert not h.goals


def test_no_grasp_result_rejects_before_motion(h):
    h.authorized=False;wait_for(lambda:h.node.grasp is None)
    h.send();h.event('rejected_state');assert not h.goals and not h.scenes


def test_nonopposing_contact_never_sends_lift(h):
    h.grip=False;wait_for(lambda:h.node.contact and not h.node.contact[0].opposing_contacts)
    h.send();h.event('failed');assert not h.goals and not h.scenes


@pytest.mark.parametrize('fault',['slip','supported','abort'])
def test_failed_test_lift_prevents_second_stage_and_cleans_collision_payload(h,fault):
    h.fault=fault;h.send();h.event('failed')
    assert len(h.goals)==1 and not h.scenes[-1].attach_object
    assert not any(e['retention_verified'] for e in h.events)


def test_lost_feedback_cancels_and_preserves_hand(h):
    h.release=False;h.send();wait_for(lambda:len(h.goals)==1)
    before=h.hand.copy();h.send_feedback=False
    h.event('failed');assert h.cancels==1 and h.hand==before
    assert not h.scenes[-1].attach_object


def test_busy_request_does_not_start_another_lift(h):
    h.release=False;h.send();wait_for(lambda:len(h.goals)==1)
    h.send();h.event('rejected_busy');assert len(h.goals)==1
    h.release=True;h.event('succeeded')


def test_unknown_stop_remains_locked_until_terminal_controller_result(h):
    h.release=False;h.cancel_complete=False;h.send();wait_for(lambda:len(h.goals)==1)
    h.node.cancel_active();h.event('error');assert h.node.active_request
    h.send();h.event('rejected_busy')
    h.cancel_complete=True;h.event('canceled');assert h.node.active_request is None


def test_late_goal_acceptance_is_canceled(h):
    h.release=False;h.late_accept=True;h.send();wait_for(lambda:h.acceptance is not None)
    h.event('canceling');assert h.node.active_request
    h.late_accept=False;h.acceptance.set_result(None);h.release=True
    h.event('failed');assert h.cancels>=1 and len(h.goals)==1


def test_cleanup_error_never_releases_unknown_model(h):
    h.fault='cleanup';h.send();h.event('error')
    assert h.node.active_request and h.node.scene_error
    h.send();h.event('rejected_busy')


def test_explicit_hand_open_restores_open_collision_model(h):
    h.send();h.event('succeeded');h.hand=[0.]*6
    wait_for(lambda:h.scenes and not h.scenes[-1].hand_state.name)
    wait_for(lambda:not h.node.scene_owned)


def test_unavailable_motion_action_discards_trigger():
    h=Harness(server=False)
    try:h.send();h.event('unavailable');assert not h.goals
    finally:h.close()
