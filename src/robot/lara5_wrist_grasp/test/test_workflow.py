"""Real ROS clients against isolated fake native services/actions; no hardware."""

import copy
import json
import threading
import time
import uuid

import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped, TransformStamped
from lara5_safety_msgs.action import MoveToPose
from lara5_safety_msgs.srv import ClosureHandoff, GetFramePose, PlanCartesianPath
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.task import Future
from sensor_msgs.msg import JointState, Image, CameraInfo
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Empty, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from lara5_grasp.configuration import HAND_JOINTS
from lara5_wrist_grasp.node import ARM, WristApproach
from lara5_wrist_grasp.tracking import Candidate


def wait_for(predicate, timeout=6.):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if predicate(): return
        time.sleep(.01)
    raise AssertionError('Wrist workflow timeout')


class FakeCameraWrist(WristApproach):
    """Transport workflow tests inject a known camera estimate, not robot state."""
    def _observe(self):
        self.measured()
        if self.peer.no_camera:
            raise ValueError('Stale wrist camera measurement')
        center = self.peer.center.copy()
        grid = np.array([[x, y, 0.] for x in np.linspace(-.025, .025, 12)
                         for y in np.linspace(-.025, .025, 12)])
        candidate = Candidate(center, grid+center)
        center = self.tracker.observe([candidate], self.peer.camera_stamp, ('synthetic_camera',))
        return center, self.peer.matrix()


def description():
    result = '<robot name="fake"><link name="lara5_flange"/><link name="right_hand_base_link"/>'
    result += '<joint name="mount" type="fixed"><parent link="lara5_flange"/><child link="right_hand_base_link"/></joint>'
    result += '<link name="right_wrist_camera_holder"/><joint name="holder" type="fixed"><parent link="right_hand_base_link"/><child link="right_wrist_camera_holder"/><origin xyz="0 0 -0.0115"/></joint>'
    for i, name in enumerate(HAND_JOINTS):
        result += f'<link name="digit{i}"/><joint name="{name}" type="revolute"><parent link="right_hand_base_link"/><child link="digit{i}"/><axis xyz="0 0 1"/><limit lower="0" upper="2"/></joint>'
    return result+'<ros2_control><hardware><plugin>neuraPlugin/PositionAndVelocityHW</plugin></hardware></ros2_control></robot>'


class Harness:
    def __init__(self, parameters=None, real_camera=False):
        prefix = self.prefix = '/wrist_test_'+uuid.uuid4().hex
        self.context = Context(); self.context.init()
        self.peer = Node('fake_native', context=self.context)
        self.real_camera = real_camera
        self.world_camera = False
        self.camera_replay = False
        self.camera_stamp = None
        self.scene_hook = None
        self.peer.no_camera = False
        self.peer.center = np.array([.32, -.4, .5])
        self.q = [0.]*6
        self.hand_q = [0.]*6
        self.pose = np.array([.3, -.4, .5])
        self.peer.matrix = self.matrix
        self.goals, self.pending, self.events = [], [], []
        self.release = True; self.cancel_complete = True; self.cancels = 0
        self.accept = True; self.abort = False; self.bad_endpoint = False
        self.reject_plan = False; self.send_states = True; self.move = True
        self.late_accept = False; self.acceptance = None
        self.group = ReentrantCallbackGroup()
        params = dict(camera_calibration_confirmed=True, base_alignment_confirmed=True,
            camera_mount_mode='measured_color',
            grasp_reference_confirmed=True,
            hand_to_grasp_xyz=[0., 0., 0.], settle_time=.04, feedback_max_age=.3,
            service_timeout=1., goal_timeout=.2, cancel_timeout=.2)
        params.update(parameters or {})
        if real_camera:
            params.update(color_topic=prefix+'/color', depth_topic=prefix+'/depth', info_topic=prefix+'/info')
            if params['camera_mount_mode'] == 'measured_color':
                params['camera_to_flange_xyz'] = [0., 0., -.3]
        remaps = []
        sources = {'/arm/wrist/status':'status','/arm/wrist/preview':'preview',
            '/arm/wrist/approach':'approach','/arm/wrist/cancel':'cancel',
            '/arm/wrist/object_pose':'object','/arm/wrist/flange_target':'flange',
            '/arm/grasp/status':'grasp','/robot_description':'description','/joint_states':'states',
            '/arm/wrist/closure_handoff':'handoff',
            '/safety/arm/get_frame_pose':'fk','/safety/arm/plan_cartesian_path':'plan',
            '/safety/arm/move_to_pose':'move'}
        for source, target in sources.items(): remaps += ['-r', source+':='+prefix+'/'+target]
        node_type = WristApproach if real_camera else FakeCameraWrist
        self.node = node_type(context=self.context, cli_args=['--ros-args', *remaps],
            parameter_overrides=[Parameter(key, value=value) for key, value in params.items()])
        self.node.peer = self.peer
        self.node.description = description()
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.peer.create_publisher(JointState, prefix+'/states', 10)
        self.preview = self.peer.create_publisher(Empty, prefix+'/preview', 1)
        self.approach = self.peer.create_publisher(Empty, prefix+'/approach', 1)
        self.cancel_pub = self.peer.create_publisher(Empty, prefix+'/cancel', 1)
        self.grasp_pub = self.peer.create_publisher(String, prefix+'/grasp', retained)
        self.handoff_client = self.peer.create_client(ClosureHandoff, prefix+'/handoff')
        if real_camera:
            self.tfpub = TransformBroadcaster(self.peer)
            self.rgb_pub = self.peer.create_publisher(Image, prefix+'/color', qos_profile_sensor_data)
            self.depth_pub = self.peer.create_publisher(Image, prefix+'/depth', qos_profile_sensor_data)
            self.info_pub = self.peer.create_publisher(CameraInfo, prefix+'/info', qos_profile_sensor_data)
            rgb = np.zeros((40, 40, 3), np.uint8); rgb[10:30, 10:30, 1] = 255
            self.rgb_msg = Image(height=40, width=40, step=120, encoding='rgb8', data=rgb.tobytes())
            depths = np.full((40, 40), 700, np.uint16)
            depths[10:30, 10:30] = 300
            self.depth_msg = Image(height=40, width=40, step=80, encoding='16UC1', data=depths.tobytes())
            self.info_msg = CameraInfo(height=40,width=40,k=[100.,0.,14.,0.,100.,20.,0.,0.,1.])
        self.peer.create_subscription(String, prefix+'/status',
            lambda msg:self.events.append(json.loads(msg.data)), retained)
        self.peer.create_service(GetFramePose, prefix+'/fk', self.fk, callback_group=self.group)
        self.peer.create_service(PlanCartesianPath, prefix+'/plan', self.plan, callback_group=self.group)
        self.server = ActionServer(self.peer, MoveToPose, prefix+'/move',
            execute_callback=self.execute, goal_callback=self.goal, cancel_callback=self.cancel,
            callback_group=self.group)
        self.peer.create_timer(.01, self.tick, callback_group=self.group)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.peer); self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin); self.thread.start()
        try:
            wait_for(lambda:len(self.node.samples)==12 and self.node.history
                and time.monotonic()-self.node.history[0][0]>.04 and self.node.arm.server_is_ready()
                and self.node.fk.service_is_ready() and self.node.planner.service_is_ready()
                and self.approach.get_subscription_count()==1 and self.handoff_client.service_is_ready())
        except BaseException as error:
            details = dict(samples=len(self.node.samples), history=len(self.node.history),
                arm=self.node.arm.server_is_ready(), fk=self.node.fk.service_is_ready(),
                planner=self.node.planner.service_is_ready(), subscribers=self.approach.get_subscription_count(),
                fk_name=self.node.fk.srv_name)
            self.close()
            raise AssertionError(str(details)) from error

    def matrix(self):
        result = np.eye(4); result[:3, 3] = self.pose
        return result

    def fk(self, request, response):
        position = self.pose.copy()
        if request.start_state.name:
            q = dict(zip(request.start_state.name, request.start_state.position))
            position = np.array([.3+q[ARM[0]], -.4+q[ARM[1]], .5+q[ARM[2]]])
            if self.bad_endpoint: position[0] += .1
        response.success = True
        response.pose = PoseStamped()
        response.pose.header.frame_id = 'lara5_root_link'
        p = response.pose.pose.position
        p.x, p.y, p.z = map(float, position)
        response.pose.pose.orientation.w = 1.
        return response

    def plan(self, request, response):
        response.success = not self.reject_plan
        response.message = 'synthetic collision rejection' if self.reject_plan else 'valid'
        p = request.target.pose.position
        target = [p.x-.3, p.y+.4, p.z-.5, 0., 0., 0.]
        end = JointTrajectoryPoint(positions=target)
        end.time_from_start.sec = 4
        response.trajectory = JointTrajectory(joint_names=list(ARM), points=[
            JointTrajectoryPoint(positions=self.q), end])
        return response

    async def goal(self, _):
        if self.late_accept:
            self.acceptance = Future(); await self.acceptance
        return GoalResponse.ACCEPT if self.accept else GoalResponse.REJECT

    def cancel(self, _):
        self.cancels += 1
        return CancelResponse.ACCEPT

    async def execute(self, handle):
        self.goals.append(copy.deepcopy(handle.request))
        future = Future(); self.pending.append((handle, future, time.monotonic()))
        await future
        if handle.is_cancel_requested:
            handle.canceled(); return MoveToPose.Result(success=False)
        if self.move:
            p = handle.request.target.pose.position
            self.pose = np.array([p.x, p.y, p.z])
            self.q = [p.x-.3, p.y+.4, p.z-.5, 0., 0., 0.]
        if self.abort: handle.abort()
        else: handle.succeed()
        return MoveToPose.Result(success=not self.abort)

    def tick(self):
        if self.real_camera and not self.peer.no_camera:
            if not self.camera_replay or self.camera_stamp is None:
                self.camera_stamp = self.peer.get_clock().now().to_msg()
            stamp = self.camera_stamp
            if self.world_camera:
                depth = np.full((40, 40), 700, np.uint16)
                u = int(round(14.+100.*(.32-self.pose[0])/.3))
                v = int(round(20.+100.*(-.4-self.pose[1])/.3))
                depth[v-6:v+6, u-6:u+6] = 300
                if self.scene_hook: self.scene_hook(depth)
                self.depth_msg.data = depth.tobytes()
            tf = TransformStamped()
            tf.header.frame_id = 'lara5_root_link'; tf.child_frame_id = 'lara5_flange'
            tf.header.stamp = stamp
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float,self.pose)
            tf.transform.rotation.w = 1.
            transforms = [tf]
            if self.node.cfg['camera_mount_mode'] == 'nominal_cad':
                driver = TransformStamped()
                driver.header.frame_id = 'wrist_depth_optical_frame'
                driver.child_frame_id = 'wrist_color_optical_frame'
                driver.header.stamp = stamp
                driver.transform.rotation.w = 1.
                driver.transform.translation.x = -.015
                transforms.append(driver)
            self.tfpub.sendTransform(transforms)
            for publisher,msg in ((self.rgb_pub,self.rgb_msg),(self.depth_pub,self.depth_msg),(self.info_pub,self.info_msg)):
                msg.header.stamp = stamp; msg.header.frame_id = 'wrist_color_optical_frame'
                publisher.publish(msg)
        elif not self.peer.no_camera:
            if not self.camera_replay or not hasattr(self.peer, 'camera_stamp'):
                self.peer.camera_stamp = self.peer.get_clock().now().nanoseconds/1e9
            self.node.frames = (None, None, None, time.monotonic())
        if self.send_states:
            msg = JointState(name=list(ARM+HAND_JOINTS), position=self.q+self.hand_q)
            msg.header.stamp = self.peer.get_clock().now().to_msg()
            self.state_pub.publish(msg)
        for handle, future, started in list(self.pending):
            if (self.release and time.monotonic()-started>.03) or (handle.is_cancel_requested and self.cancel_complete):
                self.pending.remove((handle,future,started)); future.set_result(None)

    def event(self, state):
        wait_for(lambda:any(event['state']==state for event in self.events))
        return next(event for event in reversed(self.events) if event['state']==state)

    def handoff(self, operation, closure):
        future = self.handoff_client.call_async(ClosureHandoff.Request(
            operation=operation, closure_request_id=closure))
        wait_for(future.done)
        return future.result()

    def close(self):
        self.node.request_stop()
        self.release = self.cancel_complete = True
        if self.acceptance and not self.acceptance.done(): self.acceptance.set_result(None)
        wait_for(lambda:not self.pending and (not self.node.worker or not self.node.worker.is_alive()))
        self.executor.shutdown(timeout_sec=3); self.thread.join(timeout=3)
        self.node.closed = True
        self.server.destroy(); self.node.destroy_node(); self.peer.destroy_node(); self.context.shutdown()


def test_preview_checks_path_without_arm_or_hand_commands():
    h = Harness()
    try:
        h.preview.publish(Empty()); done=h.event('preview_ready')
        assert not done['ready_for_closure'] and not h.goals
    finally:h.close()


def test_unconfirmed_alignment_allows_read_only_path_preview():
    h = Harness({'camera_calibration_confirmed':False, 'base_alignment_confirmed':False})
    try:
        h.preview.publish(Empty()); done=h.event('preview_ready')
        assert not done['camera_alignment_verified'] and not done['base_alignment_verified']
        assert not done['ready_for_closure'] and not h.goals
    finally:h.close()


def test_actual_rgbd_topics_and_capture_time_tf_reach_native_preview():
    h=Harness(real_camera=True)
    try:
        wait_for(lambda:h.node.frames is not None)
        h.preview.publish(Empty());done=h.event('preview_ready')
        assert np.allclose(done['flange_xyz'],[.3165,-.4015,.5],atol=.002)
        assert not h.goals and not done['ready_for_closure']
    finally:h.close()


def test_nominal_cad_and_real_optical_tf_reach_path_preview_but_unverified_mount_cannot_move():
    from lara5_object_approach.grasp_geometry import transform
    from lara5_wrist_grasp.camera_mount import CameraMount, DEFAULTS
    config = dict(DEFAULTS, optical_frame='wrist_color_optical_frame')
    mount, _ = CameraMount(config).compose(description(), transform([-.015, 0., 0.], [0., 0., 0., 1.]))
    reference_flange = (mount @ [.0165, -.0015, .3, 1.])[:3]
    h = Harness(dict(camera_mount_mode='nominal_cad', camera_calibration_confirmed=False,
        execution_enabled=True, hand_to_grasp_xyz=(reference_flange-[.02, 0., 0.]).tolist()), real_camera=True)
    try:
        wait_for(lambda:h.node.frames is not None)
        h.preview.publish(Empty()); done=h.event('preview_ready')
        assert np.allclose(done['flange_xyz'], [.32, -.4, .5], atol=.002)
        assert done['camera_mount']['mode'] == 'nominal_cad'
        assert not done['camera_alignment_verified'] and not h.goals
        wait_for(lambda:not h.node.busy)
        h.approach.publish(Empty()); h.event('failed')
        assert not h.goals and h.node.ready is None
    finally:h.close()


def test_execution_disabled_is_not_a_motion_command():
    h=Harness()
    try:
        h.approach.publish(Empty()); h.event('failed'); assert not h.goals
    finally:h.close()


def test_bounded_execution_finishes_at_verified_pose_then_invalidates_on_arm_drift():
    h=Harness({'execution_enabled':True})
    try:
        h.approach.publish(Empty()); done=h.event('succeeded')
        assert done['ready_for_closure'] and len(h.goals)==2
        assert done['completion_evidence']['distinct_frames'] >= 3
        assert done['completion_evidence']['source_span_seconds'] >= .15
        assert np.allclose(h.pose,h.peer.center,atol=.003)
        previous=np.array([.3,-.4,.5])
        for goal in h.goals:
            p=goal.target.pose.position; target=np.array([p.x,p.y,p.z])
            assert np.linalg.norm(target-previous)<=.010001
            assert goal.target.pose.orientation.w==1.
            previous=target
        h.q[0]+=.03
        h.event('invalidated'); assert h.node.ready is None
        assert not h.handoff('claim', 'after-drift').success and h.node.handoff is None
    finally:h.close()


def test_verified_approach_is_handed_to_one_closure_and_blocks_arm_until_release():
    h=Harness({'execution_enabled':True})
    try:
        h.approach.publish(Empty()); done=h.event('succeeded')
        claim=h.handoff('claim', 'closure-1')
        assert claim.success and claim.approach_request_id==done['request_id']
        assert np.allclose(claim.arm_positions, done['arm_positions'])
        held=h.event('handed_off')
        assert held['closure_handoff']=='closure-1' and not held['ready_for_closure']
        # The result is consumed: neither the holder nor another closure claims again.
        assert not h.handoff('claim', 'closure-1').success
        assert not h.handoff('claim', 'closure-2').success
        goals=len(h.goals)
        h.approach.publish(Empty()); h.event('rejected_busy')
        h.preview.publish(Empty()); time.sleep(.2)
        assert len(h.goals)==goals and h.node.handoff=='closure-1'
        assert not h.handoff('release', 'closure-2').success and h.node.handoff=='closure-1'
        assert h.handoff('release', 'closure-1').success
        released=h.event('released')
        assert released['closure_handoff'] is None and not released['ready_for_closure']
        refused=h.handoff('claim', 'closure-3')
        assert not refused.success and 'unconsumed' in refused.message
        assert not refused.arm_positions and not refused.approach_request_id
    finally:h.close()


def test_handoff_is_refused_before_success_and_while_the_arm_corrects():
    h=Harness({'execution_enabled':True})
    try:
        assert not h.handoff('claim', 'early').success
        assert not h.handoff('unknown', 'early').success and not h.handoff('claim', '').success
        h.release=False; h.approach.publish(Empty()); wait_for(lambda:len(h.goals)==1)
        busy=h.handoff('claim', 'during')
        assert not busy.success and 'running' in busy.message and h.node.handoff is None
        h.release=True; h.event('succeeded')
        assert h.handoff('claim', 'after').success
    finally:h.close()


def test_grasp_status_that_overtakes_its_claim_does_not_invalidate_readiness():
    h=Harness({'execution_enabled':True})
    try:
        h.approach.publish(Empty()); h.event('succeeded')
        h.grasp_pub.publish(String(data=json.dumps(dict(
            state='claiming', request_id='closure-1', active_request_id='closure-1'))))
        wait_for(lambda:h.node.grasp_active)
        time.sleep(.3)
        assert h.node.ready is not None and not any(e['state']=='invalidated' for e in h.events)
        # Grasp activity still blocks new arm commands.
        h.approach.publish(Empty()); h.event('rejected_busy')
        assert h.handoff('claim', 'closure-1').success
    finally:h.close()


def test_real_rgbd_tracks_the_same_surface_as_the_wrist_moves_then_confirms_new_frames():
    h = Harness({'execution_enabled':True, 'service_timeout':2.}, real_camera=True)
    try:
        h.world_camera = True
        time.sleep(.05)
        h.approach.publish(Empty()); done = h.event('succeeded')
        assert done['ready_for_closure'] and h.goals
        assert done['completion_evidence']['distinct_frames'] >= 3
        assert done['completion_evidence']['source_span_seconds'] >= .15
        assert np.linalg.norm(h.pose-done['target_tracking']['latest_reference_base_m']) <= .003
        # Coarse synthetic pixels add up to 1.5 mm per lateral coordinate to
        # the configured 3 mm observed-reference tolerance.
        assert np.linalg.norm(h.pose-[.32, -.4, .5]) < .0052
        assert done['target_tracking']['active']
        assert not done['target_tracking']['semantic_identity_verified']
    finally:h.close()


def test_target_disappearance_does_not_switch_to_a_new_foreground_cluster_during_motion():
    h = Harness({'execution_enabled':True, 'service_timeout':2.}, real_camera=True)
    try:
        h.world_camera = True
        time.sleep(.05)
        h.release = False
        h.approach.publish(Empty()); wait_for(lambda:len(h.goals)==1)
        def replacement(depth):
            depth[:] = 700
            depth[7:17, 23:33] = 250
        h.scene_hook = replacement
        done = h.event('failed')
        assert 'lost' in done['message']
        assert h.cancels >= 1 and len(h.goals) == 1 and not done['ready_for_closure']
    finally:h.close()


def test_a_separate_distractor_does_not_change_the_tracked_target():
    h = Harness({'execution_enabled':True, 'service_timeout':2.}, real_camera=True)
    try:
        h.world_camera = True
        time.sleep(.05)
        h.release = False
        h.approach.publish(Empty()); wait_for(lambda:len(h.goals)==1)
        h.scene_hook = lambda depth:depth.__setitem__((slice(25, 34), slice(25, 34)), 250)
        h.release = True
        done = h.event('succeeded')
        assert done['ready_for_closure'] and done['target_tracking']['active']
    finally:h.close()


@pytest.mark.parametrize('fault', ['replay', 'disconnect', 'cancel'])
def test_final_confirmation_cannot_finish_without_new_stable_measurements(fault):
    h = Harness({'execution_enabled':True})
    try:
        h.peer.center = h.pose.copy()
        h.approach.publish(Empty()); h.event('verifying')
        if fault == 'replay': h.camera_replay = True
        if fault == 'disconnect': h.peer.no_camera = True
        if fault == 'cancel': h.cancel_pub.publish(Empty())
        done = h.event('canceled' if fault == 'cancel' else 'failed')
        assert not done['ready_for_closure'] and not h.goals
        assert not any(e['state'] == 'succeeded' for e in h.events)
    finally:h.close()


def test_replayed_real_camera_messages_cancel_active_correction():
    h = Harness({'execution_enabled':True, 'service_timeout':2.}, real_camera=True)
    try:
        h.world_camera = True
        time.sleep(.05)
        h.release = False
        h.approach.publish(Empty()); wait_for(lambda:len(h.goals)==1)
        h.camera_replay = True
        done = h.event('failed')
        assert 'timestamp' in done['message']
        assert h.cancels >= 1 and len(h.goals) == 1
    finally:h.close()


@pytest.mark.parametrize('fault',['camera','calibration','reference','collision','endpoint'])
def test_invalid_inputs_and_native_rejections_cannot_send_motion(fault):
    params={'execution_enabled':True}
    if fault=='calibration':params['camera_calibration_confirmed']=False
    if fault=='reference':params['grasp_reference_confirmed']=False
    h=Harness(params)
    try:
        if fault=='camera':h.peer.no_camera=True
        if fault=='collision':h.reject_plan=True
        if fault=='endpoint':h.bad_endpoint=True
        h.approach.publish(Empty());h.event('failed');assert not h.goals
    finally:h.close()


@pytest.mark.parametrize('fault',['camera','target','state','cancel'])
def test_fault_during_correction_cancels_without_another_segment(fault):
    h=Harness({'execution_enabled':True})
    try:
        h.release=False;h.approach.publish(Empty());wait_for(lambda:len(h.goals)==1)
        if fault=='camera':h.peer.no_camera=True
        if fault=='target':h.peer.center[0]+=.04
        if fault=='state':h.send_states=False
        if fault=='cancel':h.cancel_pub.publish(Empty())
        h.event('canceled' if fault=='cancel' else 'failed')
        assert h.cancels>=1 and len(h.goals)==1 and h.node.ready is None
    finally:h.close()


def test_unknown_stop_retains_ownership_until_late_terminal_result():
    h=Harness({'execution_enabled':True})
    try:
        h.release=False;h.cancel_complete=False
        h.approach.publish(Empty());wait_for(lambda:len(h.goals)==1)
        h.cancel_pub.publish(Empty());h.event('error');assert h.node.busy
        h.approach.publish(Empty());h.event('rejected_busy');assert len(h.goals)==1
        h.cancel_complete=True;wait_for(lambda:not h.node.busy)
        assert h.node.ready is None
    finally:h.close()


def test_delayed_acceptance_is_canceled_without_losing_goal_ownership():
    h=Harness({'execution_enabled':True})
    try:
        h.release=False;h.late_accept=True;h.approach.publish(Empty())
        wait_for(lambda:h.acceptance is not None)
        h.event('error');assert h.node.busy
        h.late_accept=False;h.acceptance.set_result(None)
        wait_for(lambda:h.cancels>=1 and not h.node.busy)
        assert h.node.ready is None
    finally:h.close()
