"""Explicit, physics-verified two-stage lift; never commands fingers or welds objects."""

import copy
import json
from pathlib import Path
import signal
import time
import uuid

import numpy as np
import rclpy
import xacro
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped, Vector3
from lara5_grasp.configuration import HAND_JOINTS, UPPER_LIMITS
from lara5_grasp.policy import contact_digits
from lara5_object_approach.grasp_geometry import transform
from lara5_object_approach.planning_geometry import ArmModel, pose_error
from lara5_object_approach.preview import ros_pose
from lara5_safety_msgs.action import MoveToPose
from lara5_safety_msgs.msg import SimGraspFeedback
from lara5_safety_msgs.srv import GetFramePose, SetGraspScene
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String
from visualization_msgs.msg import Marker, MarkerArray

ARM_JOINTS = tuple(f'lara5_joint{i}' for i in range(1,7))


def matrix(pose):
    p,q=pose.position,pose.orientation
    return transform([p.x,p.y,p.z],[q.x,q.y,q.z,q.w])


class ObjectLift(Node):
    def __init__(self,**kwargs):
        super().__init__('object_lift',**kwargs)
        def param(name,default):return self.declare_parameter(name,default).value
        self.world=param('world_frame','world');self.flange=param('flange_link','lara5_flange')
        self.target_object=param('target_object_name','green_cylinder')
        defaults=dict(lift_height=.10,test_lift_height=.04,test_lift_duration=4.,final_lift_duration=6.,
            feedback_max_age=.5,grip_settle_time=.35,grip_timeout=3.,contact_threshold_n=.15,
            max_contact_force_n=16.,contact_loss_grace=.15,slip_tolerance=.012,tilt_tolerance=.15,collision_padding=.01,
            hand_tolerance=.08,arm_start_tolerance=.01,position_tolerance=.003,angle_tolerance=.02,
            hold_settle_time=.75,endpoint_timeout=4.,max_final_object_speed=.03,service_timeout=10.,
            goal_response_timeout=10.,motion_timeout=30.,cancel_timeout=5.)
        for name,default in defaults.items():
            value=param(name,default)
            if not np.isfinite(value) or value<=0:raise ValueError(name+' must be finite and positive')
            setattr(self,name,value)
        self.object_size=list(param('object_size',[.06,.06,.10]))
        if (len(self.object_size)!=3 or not np.isfinite(self.object_size).all()
                or min(self.object_size)<=0 or max(self.object_size)+2*self.collision_padding>.5):
            raise ValueError('object_size must contain three positive dimensions below 0.5 m')
        if not (self.slip_tolerance < self.test_lift_height < self.lift_height <= .20):
            raise ValueError('Require slip_tolerance < test_lift_height < lift_height <= 0.20 m')
        if self.hold_settle_time>=self.endpoint_timeout or self.grip_settle_time>=self.grip_timeout:
            raise ValueError('Settling intervals must be shorter than their verification deadlines')
        if not self.world or not self.flange or not self.target_object:
            raise ValueError('World, flange and target fixture must be named')
        description=param('robot_description','')
        if not description:
            source=Path(get_package_share_directory('lara5_inspire_hand_description'))/'urdf/lara5_inspire_hand.urdf.xacro'
            description=xacro.process_file(str(source)).toxml()
        self.model=ArmModel(description,self.world,self.flange,list(ARM_JOINTS))
        self.samples={};self.contact=None;self.grasp=None;self.approach_busy=False
        self.active=None;self.last_id=None;self.used_grasp=None;self.stopping=False
        self.scene_owned=False;self.scene_error=False
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status=self.create_publisher(String,'/arm/lift/status',qos)
        self.markers=self.create_publisher(MarkerArray,'/arm/lift/markers',qos)
        self.states=self.create_subscription(JointState,'/joint_states',self.on_state,20)
        self.contacts=self.create_subscription(SimGraspFeedback,'/sim/grasp_feedback',self.on_contact,20)
        self.grasp_sub=self.create_subscription(String,'/arm/grasp/status',self.on_grasp,qos)
        self.approach_sub=self.create_subscription(String,'/arm/approach/status',self.on_approach,qos)
        self.command=self.create_subscription(Empty,'/arm/lift/target',self.on_trigger,1)
        self.motion=ActionClient(self,MoveToPose,self.resolve_topic_name('/safety/arm/move_to_pose'))
        self.fk=self.create_client(GetFramePose,'/safety/arm/get_frame_pose')
        self.scene=self.create_client(SetGraspScene,'/safety/arm/set_grasp_scene')
        self.timer=self.create_timer(.05,self.tick)
        self.clear_markers()
        self.report('idle','Waiting for explicit lift after successful contact-limited closure')

    @property
    def active_request(self):return self.active['id'] if self.active else None

    def report(self,state,message,**values):
        payload=dict(state=state,message=message,request_id=self.last_id,active_request_id=self.active_request,
            retention_verified=False,holder_camera_collision_checked=False,other_scene_objects_checked=False)
        payload.update(values)
        self.status.publish(String(data=json.dumps(payload)))
        self.get_logger().info(f'{state}: {message}')

    def clear_markers(self):self.markers.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))

    def on_state(self,msg):
        if (len(msg.name)!=len(msg.position) or len(set(msg.name))!=len(msg.name)
                or not np.isfinite(msg.position).all()):self.samples.clear();return
        receipt=time.monotonic();stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        for name,value in zip(msg.name,msg.position):
            if name in ARM_JOINTS+HAND_JOINTS:self.samples[name]=(value,receipt,stamp)

    def measured(self,names=ARM_JOINTS):
        now=time.monotonic();ros_now=self.get_clock().now().nanoseconds/1e9
        values=[]
        for name in names:
            sample=self.samples.get(name)
            if sample is None:raise ValueError('Missing measured joint '+name)
            value,receipt,stamp=sample
            if stamp<=0 or now-receipt>self.feedback_max_age or not -.1<=ros_now-stamp<=self.feedback_max_age:
                raise ValueError('Stale measured joint '+name)
            values.append(value)
        if names==ARM_JOINTS:return self.model.check(values)
        if any(v < -.01 or v>upper+.01 for v,upper in zip(values,UPPER_LIMITS)):
            raise ValueError('Measured hand outside model limits')
        return np.asarray(values)

    def on_contact(self,msg):
        if msg.object_name==self.target_object:self.contact=(copy.deepcopy(msg),time.monotonic())

    def feedback(self):
        if self.contact is None:raise ValueError('Missing simulation object/contact feedback')
        msg,receipt=self.contact
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        age=self.get_clock().now().nanoseconds/1e9-stamp
        if stamp<=0 or time.monotonic()-receipt>self.feedback_max_age or not -.1<=age<=self.feedback_max_age:
            raise ValueError('Stale simulation object/contact feedback')
        if msg.header.frame_id!=self.world or not np.isfinite(msg.linear_speed) or msg.linear_speed<0:
            raise ValueError('Invalid simulation feedback frame or speed')
        digits=contact_digits(msg.contact_links,msg.contact_normal_forces,self.contact_threshold_n)
        if sum(msg.contact_normal_forces)>self.max_contact_force_n:raise ValueError('Contact force limit exceeded')
        relative=matrix(msg.object_in_flange);world=matrix(msg.object_pose)
        grip=msg.opposing_contacts and 'thumb' in digits and bool(digits-{'thumb'})
        return msg,relative,world,grip

    def on_grasp(self,msg):
        try:
            event=json.loads(msg.data)
            positions=event.get('measured_positions',[])
            good=(event.get('state')=='succeeded' and event.get('contact_evidence') is True
                and event.get('controller_motion_completed') is True and event.get('measured_hold_verified') is True
                and event.get('feedback_mode')=='simulation' and event.get('request_id')
                and len(positions)==6 and np.isfinite(positions).all())
            self.grasp=event if good else None
        except (ValueError,TypeError):self.grasp=None

    def on_approach(self,msg):
        try:
            event=json.loads(msg.data);state=event.get('state')
            self.approach_busy=bool(event.get('active_request_id')) or state in ('planning','planned','moving_to_pregrasp','moving_to_grasp',
                'verifying_pregrasp','verifying_grasp','canceling','error')
        except (ValueError,TypeError):self.approach_busy=True

    def on_trigger(self,_):
        if self.active or self.stopping:
            self.report('rejected_busy','Lift active or stopping; trigger discarded',
                        request_id=str(uuid.uuid4()));return
        self.last_id=str(uuid.uuid4())
        if self.scene_error:
            self.report('unavailable','Collision context is unconfirmed; restart the simulation backend');return
        try:
            if self.approach_busy:raise ValueError('Positioning task is active')
            if not self.grasp:raise ValueError('Require successful simulation grasp with measured contact hold')
            if self.used_grasp==self.grasp['request_id']:raise ValueError('This grasp already had a lift attempt; perform a fresh grasp')
            arm=self.measured().copy();hand=self.measured(HAND_JOINTS).copy()
            if max(abs(hand-np.asarray(self.grasp['measured_positions'])))>self.hand_tolerance:
                raise ValueError('Hand changed since successful closure')
            sample,relative,world,_=self.feedback()
            if not self.motion.server_is_ready() or not self.fk.service_is_ready() or not self.scene.service_is_ready():
                self.report('unavailable','Native motion, FK and grasp-context services are required');return
        except ValueError as error:
            self.report('rejected_state',str(error));return
        self.active=dict(id=self.last_id,grasp_id=self.grasp['request_id'],arm=arm,hand=hand,
            relative=relative,object_z=float(world[2,3]),start_pose=self.model.fk(arm),stage=0,state='preflight',
            deadline=time.monotonic()+self.grip_timeout,stable_since=None,contact_lost_since=None,
            handle=None,pending=None,scene_maybe=False,cancel_reason=None,cancel_sent=False,stop_reported=False)
        self.clear_markers()
        self.report('verifying_grasp','Checking sustained opposing contact before any lift')

    def guard(self,ctx,require_grip=True):
        arm=self.measured();hand=self.measured(HAND_JOINTS)
        if self.approach_busy:raise ValueError('Positioning task became active during lift')
        if not self.grasp or self.grasp['request_id']!=ctx['grasp_id']:raise ValueError('Grasp task changed during lift')
        if max(abs(hand-ctx['hand']))>self.hand_tolerance:raise ValueError('Hand posture changed during lift')
        sample,relative,world,grip=self.feedback()
        slip=float(np.linalg.norm(relative[:3,3]-ctx['relative'][:3,3]))
        tilt=float(np.arccos(np.clip(relative[:3,2]@ctx['relative'][:3,2],-1.,1.)))
        if slip>self.slip_tolerance or tilt>self.tilt_tolerance:
            raise ValueError(f'Object slipped relative to hand ({slip:.4f} m, tilt {tilt:.3f} rad)')
        if require_grip:
            if grip:ctx['contact_lost_since']=None
            elif ctx['contact_lost_since'] is None:ctx['contact_lost_since']=time.monotonic()
            elif time.monotonic()-ctx['contact_lost_since']>self.contact_loss_grace:
                raise ValueError('Opposing target contact lost during lift')
        return arm,hand,sample,relative,world,grip

    def call(self,ctx,client,request,callback):
        ctx['deadline']=time.monotonic()+self.service_timeout
        try:
            future=client.call_async(request);ctx['pending']=(client,future)
            def done(result):
                if self.active is not ctx:return
                ctx['pending']=None
                try:
                    response=result.result()
                    if response is None:raise ValueError('Native service returned no response')
                    callback(ctx,response)
                except Exception as error:self.fail(ctx,str(error))
            future.add_done_callback(done)
        except Exception as error:self.fail(ctx,str(error))

    def fk_request(self,q):
        state=JointState(name=list(ARM_JOINTS),position=list(map(float,q)))
        state.header.stamp=self.get_clock().now().to_msg()
        return GetFramePose.Request(start_state=state,frame=self.flange,reference_frame=self.world)

    def checked_fk(self,response,desired):
        if not response.success or response.pose.header.frame_id!=self.world:
            raise ValueError('Native FK failed or returned an unexpected frame')
        actual=matrix(response.pose.pose);errors=pose_error(actual,desired)
        if errors[0]>self.position_tolerance or errors[1]>self.angle_tolerance:
            raise ValueError(f'Measured flange endpoint mismatch {errors}')
        return actual,errors

    def on_start_fk(self,ctx,response):
        arm,*_=self.guard(ctx)
        if max(abs(arm-ctx['arm']))>self.arm_start_tolerance:raise ValueError('Arm moved before lift dispatch')
        self.checked_fk(response,ctx['start_pose'])
        ctx['targets']=[]
        for height in (self.test_lift_height,self.lift_height):
            target=ctx['start_pose'].copy();target[2,3]+=height;ctx['targets'].append(target)
        self.publish_path(ctx)
        self.set_scene(ctx,True)

    def scene_request(self,ctx,support,carried=True):
        try:hand=self.measured(HAND_JOINTS)
        except ValueError:hand=ctx['hand']
        hand=np.clip(hand,0,np.asarray(UPPER_LIMITS))
        request=SetGraspScene.Request(hand_state=JointState(name=list(HAND_JOINTS),position=hand.tolist()))
        if carried:
            request.touch_object='lift_'+self.target_object
            request.attach_object=True;request.allow_support_contact=support
            request.object_in_flange=ros_pose(ctx['relative'])
            size=[value+2*self.collision_padding for value in self.object_size]
            request.object_size=Vector3(x=size[0],y=size[1],z=size[2])
        return request

    def set_scene(self,ctx,support):
        ctx['state']='context';ctx['scene_maybe']=True
        self.report('preparing_lift','Updating measured closed-hand/payload collision geometry',
                    initial_support_exception=support)
        def updated(context,response):
            if not response.success:raise ValueError('Grasp collision context rejected: '+response.message)
            self.scene_owned=True
            self.send_move(context)
        self.call(ctx,self.scene,self.scene_request(ctx,support),updated)

    def send_move(self,ctx):
        arm,_,_,_,_,grip=self.guard(ctx)
        if not grip:raise ValueError('Opposing contact is absent at lift dispatch')
        expected=ctx.get('phase_start',ctx['arm'])
        if max(abs(arm-expected))>self.arm_start_tolerance:raise ValueError('Arm changed before lift dispatch')
        target=PoseStamped();target.header.frame_id=self.world
        target.header.stamp=self.get_clock().now().to_msg();target.pose=ros_pose(ctx['targets'][ctx['stage']])
        goal=MoveToPose.Goal(target=target,relative=False,
            duration=self.test_lift_duration if ctx['stage']==0 else self.final_lift_duration)
        self.used_grasp=ctx['grasp_id']
        ctx.update(state='sending',handle=None,cancel_reason=None,cancel_sent=False,
                   deadline=time.monotonic()+self.goal_response_timeout)
        self.report('test_lifting' if ctx['stage']==0 else 'lifting','Requesting checked vertical arm motion',
                    lift_height_m=self.test_lift_height if ctx['stage']==0 else self.lift_height)
        try:self.motion.send_goal_async(goal).add_done_callback(lambda result:self.on_goal(ctx,result))
        except Exception as error:self.fail(ctx,'Lift goal delivery unknown: '+str(error))

    def on_goal(self,ctx,future):
        if self.active is not ctx:return
        try:
            handle=future.result()
            if handle is None or not handle.accepted:
                ctx['handle']=None;ctx['state']='stopped'
                self.fail(ctx,ctx['cancel_reason'] or 'Native lift goal rejected');return
            ctx['handle']=handle
            handle.get_result_async().add_done_callback(lambda result:self.on_result(ctx,result))
            if ctx['cancel_reason']:self.cancel_goal(ctx)
            else:ctx.update(state='moving',deadline=time.monotonic()+self.motion_timeout)
        except Exception as error:self.fail(ctx,'Cannot obtain lift acceptance: '+str(error))

    def on_result(self,ctx,future):
        if self.active is not ctx:return
        try:outcome=future.result()
        except Exception as error:self.fail(ctx,'Lift result unknown: '+str(error));return
        if outcome is None or outcome.status not in (GoalStatus.STATUS_SUCCEEDED,GoalStatus.STATUS_CANCELED,GoalStatus.STATUS_ABORTED):
            self.fail(ctx,'Lift controller termination unconfirmed');return
        ctx['handle']=None;ctx['state']='stopped'
        if ctx['cancel_reason']:
            self.fail(ctx,ctx['cancel_reason']);return
        if outcome.status!=GoalStatus.STATUS_SUCCEEDED or outcome.result is None or not outcome.result.success:
            self.fail(ctx,outcome.result.message if outcome.result else 'Native lift motion failed');return
        now=time.monotonic()
        ctx.update(state='verifying',deadline=now+self.endpoint_timeout,result_time=now,stable_since=None)
        self.report('verifying_test_lift' if ctx['stage']==0 else 'verifying_lift',
                    'Checking measured object rise, loss of table support and retained grip')

    def endpoint(self,ctx):
        arm,_,sample,relative,world,grip=self.guard(ctx)
        actual=self.model.fk(arm);errors=pose_error(actual,ctx['targets'][ctx['stage']])
        rise=float(world[2,3]-ctx['object_z'])
        height=self.test_lift_height if ctx['stage']==0 else self.lift_height
        if (not grip or sample.supported or sample.linear_speed>self.max_final_object_speed
            or rise<height-self.slip_tolerance or abs(rise-height)>self.slip_tolerance
            or errors[0]>self.position_tolerance or errors[1]>self.angle_tolerance):
            raise ValueError(f'Lift unverified: object rise {rise:.4f} m, supported={sample.supported}, flange error={errors}')
        return arm,dict(object_rise_m=rise,measured_flange_xyz=actual[:3,3].tolist(),
            flange_error_m_rad=list(errors),relative_slip_m=float(np.linalg.norm(relative[:3,3]-ctx['relative'][:3,3])),
            opposing_contacts=True,supported=False,contact_links=list(sample.contact_links),
            contact_normal_forces_n=list(sample.contact_normal_forces))

    def verify(self,ctx):
        if any(self.samples[name][1]<ctx['result_time'] for name in ARM_JOINTS+HAND_JOINTS):return
        if self.contact[1]<ctx['result_time']:return
        try:arm,metrics=self.endpoint(ctx)
        except ValueError as error:
            ctx['stable_since']=None;ctx['verification_error']=str(error);return
        now=time.monotonic()
        if ctx['stable_since'] is None:ctx['stable_since']=now
        if now-ctx['stable_since']<self.hold_settle_time:return
        ctx['state']='endpoint_fk'
        def verified(context,response):
            self.checked_fk(response,context['targets'][context['stage']])
            arm_now,values=self.endpoint(context)
            if max(abs(arm_now-arm))>self.arm_start_tolerance:raise ValueError('Arm moved during native endpoint verification')
            if context['stage']==0:
                context.update(stage=1,phase_start=arm_now.copy())
                self.set_scene(context,False)
            else:
                self.active=None
                self.report('succeeded','Object lift and retained grip verified',retention_verified=True,
                    verification_interval_s=self.hold_settle_time,payload_collision_checked=True,**values)
        self.call(ctx,self.fk,self.fk_request(arm),verified)

    def fail(self,ctx,reason):
        if self.active is not ctx:return
        if ctx['state']=='cleaning':
            self.scene_error=True;ctx.update(state='error',deadline=float('inf'))
            self.report('error','Collision cleanup unconfirmed: '+reason);return
        if ctx['state'] in ('sending','moving','canceling'):
            if ctx['cancel_reason'] is None:
                ctx.update(cancel_reason=reason,state='canceling',deadline=time.monotonic()+self.cancel_timeout)
                self.report('canceling',reason+'; requesting arm cancellation; hand stays held')
            self.cancel_goal(ctx);return
        self.cleanup(ctx,reason)

    def cancel_goal(self,ctx):
        if ctx['handle'] is not None and not ctx['cancel_sent']:
            ctx['cancel_sent']=True
            try:ctx['handle'].cancel_goal_async()
            except Exception as error:self.report('error','Cancellation delivery failed; stop unconfirmed: '+str(error))

    def cleanup(self,ctx,reason):
        if ctx.get('pending'):
            client,future=ctx['pending'];ctx['pending']=None
            client.remove_pending_request(future);future.cancel()
        if not ctx.get('scene_maybe') and not self.scene_owned:
            self.active=None;self.clear_markers()
            self.report('canceled' if reason=='operator cancellation' else 'failed',reason);return
        ctx.update(state='cleaning',cleanup_reason=reason)
        reset=ctx.get('reset_open',False)
        request=SetGraspScene.Request() if reset else self.scene_request(ctx,False,False)
        def cleaned(context,response):
            if not response.success:raise ValueError(response.message)
            self.active=None;self.scene_owned=not reset
            self.clear_markers()
            if reset:self.report('idle','Open-hand collision model restored after explicit hand reset')
            else:self.report('canceled' if reason=='operator cancellation' else 'failed',reason,
                             collision_model='measured_hand_only',hand_reopen_commanded=False)
        self.call(ctx,self.scene,request,cleaned)

    def tick(self):
        ctx=self.active
        if ctx is None:
            if self.scene_owned and not self.scene_error and not self.stopping:
                try:
                    hand=self.measured(HAND_JOINTS)
                    if max(abs(hand))<.05:
                        self.grasp=None
                        self.active=dict(id=str(uuid.uuid4()),hand=hand,state='stopped',pending=None,
                                         scene_maybe=True,reset_open=True)
                        self.cleanup(self.active,'explicit open reset')
                except ValueError:pass
            return
        if ctx['state'] in ('canceling','error'):
            if ctx['state']=='canceling' and time.monotonic()>ctx['deadline'] and not ctx['stop_reported']:
                ctx['stop_reported']=True;self.report('error','Lift stop unconfirmed; new triggers remain blocked')
            return
        if time.monotonic()>ctx['deadline']:
            self.fail(ctx,ctx.get('verification_error','Lift phase timed out'));return
        if ctx['state']=='cleaning':return
        try:
            arm,_,_,_,_,grip=self.guard(ctx,ctx['state']!='preflight')
            if ctx['state'] in ('preflight','start_fk','context'):
                expected=ctx.get('phase_start',ctx['arm'])
                if max(abs(arm-expected))>self.arm_start_tolerance:raise ValueError('Arm moved while preparing lift')
            if ctx['state']=='preflight':
                if not grip:
                    ctx['stable_since']=None
                    ctx['verification_error']='Opposing grip did not stabilize; no lift motion was sent'
                    return
                if ctx['stable_since'] is None:ctx['stable_since']=time.monotonic()
                if time.monotonic()-ctx['stable_since']>=self.grip_settle_time:
                    ctx['state']='start_fk'
                    self.call(ctx,self.fk,self.fk_request(arm),self.on_start_fk)
            elif ctx['state']=='verifying':self.verify(ctx)
        except ValueError as error:self.fail(ctx,str(error))

    def publish_path(self,ctx):
        line=Marker();line.header.frame_id=self.world;line.header.stamp=self.get_clock().now().to_msg()
        line.ns='lift';line.id=0;line.type=Marker.LINE_STRIP;line.pose.orientation.w=1.
        line.scale.x=.006;line.color.r=1.;line.color.b=.8;line.color.a=1.
        line.points=[Point(x=float(p[0,3]),y=float(p[1,3]),z=float(p[2,3]))
                     for p in (ctx['start_pose'],*ctx['targets'])]
        self.markers.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL),line]))

    def cancel_active(self):
        self.stopping=True
        if self.active and self.active['state'] not in ('cleaning','error'):
            self.fail(self.active,'operator cancellation')


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(_signum,_frame):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupt)
    node=None
    try:
        node=ObjectLift();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node:
            node.cancel_active();deadline=time.monotonic()+node.cancel_timeout
            while rclpy.ok() and node.active_request and time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.05)
            if node.active_request:node.get_logger().error('Lift stop/context cleanup unconfirmed before exit')
            node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
