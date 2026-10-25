"""Opt-in regression: cancel outer safety action before inner goal acceptance.

Synthetic stationary feedback, an isolated fake controller and no robot/simulator.
Requires LARA5_NATIVE_FORWARDING_TEST=1, ROS_LOCALHOST_ONLY=1 and a private domain.
"""

import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from lara5_safety_msgs.action import MoveToPose
from lara5_safety_msgs.srv import GetFramePose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import yaml

from lara5_object_approach.planning_geometry import duration


@pytest.mark.skipif(os.environ.get('LARA5_NATIVE_FORWARDING_TEST') != '1', reason='Opt-in native forwarding only')
@pytest.mark.parametrize('kind', ['trajectory', 'pose'])
def test_cancel_before_inner_acceptance_is_forwarded(tmp_path, kind):
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '0', '42', '43')
    prefix = '/native_forward_'+uuid.uuid4().hex[:8]
    share = Path(get_package_share_directory('lara5_safety_filter'))
    robot = Path(get_package_share_directory('lara5_controller_manager'))
    urdf = subprocess.check_output(['xacro', str(robot/'urdf/lara5_inspire_hand_ros2_control.urdf.xacro'),
                                   'deployment_mode:=simulation'], text=True)
    params = yaml.safe_load((share/'config/safety_filter.yaml').read_text())['safety_filter']['ros__parameters']
    params.update(robot_description=urdf, obstacles_file=str(share/'config/obstacles.yaml'),
        input_action=prefix+'/checked', output_action=prefix+'/fake_controller',
        move_to_pose_action=prefix+'/unused_move', target_pose_topic=prefix+'/unused_pose',
        input_trajectory_topic=prefix+'/unused_trajectory', output_trajectory_topic=prefix+'/unused_output')
    config = tmp_path/'params.yaml'
    config.write_text(yaml.safe_dump({'safety_filter': {'ros__parameters': params}}))
    remaps = ['-r', '/joint_states:='+prefix+'/states']
    for suffix in ('plan_cartesian_path', 'get_frame_pose', 'check_trajectory'):
        remaps += ['-r', '/safety/arm/'+suffix+':='+prefix+'/'+suffix]
    log = open(tmp_path/'native.log', 'w+')
    backend = subprocess.Popen(['ros2', 'run', 'lara5_safety_filter', 'safety_filter_node', '--ros-args',
        '--params-file', str(config), *remaps], stdout=log, stderr=log, start_new_session=True)
    rclpy.init()
    peer = rclpy.create_node('native_forwarding_peer')
    executor = SingleThreadedExecutor(); executor.add_node(peer)
    acceptance, executed, cancellations = Future(), [], []
    completing = []
    async def accept(_):
        await acceptance
        return GoalResponse.ACCEPT
    def cancel(handle):
        cancellations.append(handle)
        return CancelResponse.ACCEPT
    async def execute(handle):
        executed.append(handle)
        done = Future(); completing.append((handle, done))
        await done
        handle.canceled()
        return FollowJointTrajectory.Result(error_code=0)
    server = ActionServer(peer, FollowJointTrajectory, prefix+'/fake_controller',
        execute_callback=execute, goal_callback=accept, cancel_callback=cancel,
        callback_group=ReentrantCallbackGroup())
    client = ActionClient(peer, FollowJointTrajectory if kind=='trajectory' else MoveToPose,
                          prefix+('/checked' if kind=='trajectory' else '/unused_move'))
    fk_client = peer.create_client(GetFramePose, prefix+'/get_frame_pose')
    publisher = peer.create_publisher(JointState, prefix+'/states', 10)
    names = [f'lara5_joint{i}' for i in range(1, 7)]
    q = [0., 0., 1.5708, 0., 1.5708, 0.]
    def tick():
        msg = JointState(name=names, position=q)
        msg.header.stamp = peer.get_clock().now().to_msg(); publisher.publish(msg)
        for handle, done in completing:
            if handle.is_cancel_requested and not done.done(): done.set_result(None)
    timer = peer.create_timer(.02, tick)
    def until(predicate, timeout=8):
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.02)
            if predicate(): return
        log.flush()
        raise AssertionError('Native forwarding timeout:\n'+(tmp_path/'native.log').read_text())
    try:
        until(lambda: client.server_is_ready(), 30)
        # Allow native feedback and the fake controller endpoint discovery.
        until(lambda: publisher.get_subscription_count()>0)
        for _ in range(20): executor.spin_once(timeout_sec=.02)
        trajectory = JointTrajectory(joint_names=names, points=[
            JointTrajectoryPoint(positions=q, velocities=[0.]*6, time_from_start=duration(1.))])
        if kind=='trajectory':
            goal = FollowJointTrajectory.Goal(trajectory=trajectory)
        else:
            until(fk_client.service_is_ready)
            frame=fk_client.call_async(GetFramePose.Request(start_state=JointState(name=names,position=q),
                frame='lara5_flange',reference_frame='world'))
            until(frame.done);assert frame.result().success
            target=frame.result().pose;target.pose.position.z+=.01
            goal=MoveToPose.Goal(target=target,relative=False,duration=1.)
        sent = client.send_goal_async(goal)
        until(sent.done); handle = sent.result(); assert handle.accepted
        result = handle.get_result_async()
        canceled = handle.cancel_goal_async(); until(canceled.done)
        assert canceled.result().goals_canceling
        assert not executed  # Inner controller still has no accepted goal.
        acceptance.set_result(None)
        until(result.done)
        assert len(cancellations) == len(executed) == 1
        assert result.result().status == GoalStatus.STATUS_CANCELED
    finally:
        executor.shutdown(); server.destroy(); client.destroy(); peer.destroy_node(); rclpy.shutdown()
        os.killpg(backend.pid, signal.SIGINT)
        try: backend.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(backend.pid, signal.SIGKILL); backend.wait(timeout=5)
        log.close()
