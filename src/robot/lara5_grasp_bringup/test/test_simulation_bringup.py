"""Launch the real combined simulation, inspect readiness; send no motion goals."""

import json
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from lara5_safety_msgs.msg import SimGraspFeedback
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import String
import yaml


def test_combined_launch_is_idle_unique_and_keeps_task_configs_scoped(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_DOMAIN_ID', str(170+os.getpid() % 30))
    # Distinct overrides catch a leaked generic config_file argument between
    # the three included task launches, without issuing any motion input.
    def config(package, filename, node, parameter, value):
        source = Path(get_package_share_directory(package))/'config'/filename
        payload = yaml.safe_load(source.read_text())
        payload[node]['ros__parameters'][parameter] = value
        output = tmp_path/filename
        output.write_text(yaml.safe_dump(payload))
        return str(output)
    approach_config = config('lara5_object_approach', 'approach.yaml', 'object_approach',
                             'pregrasp_flange_clearance', .31)
    grasp_config = config('lara5_grasp', 'grasp.yaml', 'hand_grasp', 'max_joint_step', .04)
    lift_config = config('lara5_object_lift', 'lift.yaml', 'object_lift', 'lift_height', .11)
    visible = os.environ.get('LARA5_BRINGUP_VIEWER_TEST') == '1'
    if visible:
        assert os.environ.get('DISPLAY'), 'Visible regression needs a private display'
    log = open(tmp_path/'bringup.log', 'w+')
    process = subprocess.Popen(['ros2', 'launch', 'lara5_grasp_bringup', 'simulation.launch.py',
        'sim_headless:='+('false' if visible else 'true'), 'rviz:='+('true' if visible else 'false'),
        'approach_config:='+approach_config, 'grasp_config:='+grasp_config, 'lift_config:='+lift_config],
        stdout=log, stderr=log, start_new_session=True)
    context = Context(); context.init()
    peer = Node('bringup_readiness_probe', context=context)
    executor = SingleThreadedExecutor(context=context); executor.add_node(peer)
    statuses, frames, contacts = {}, set(), []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = []
    for task in ('approach', 'grasp', 'lift'):
        subscriptions.append(peer.create_subscription(String, '/arm/'+task+'/status',
            lambda msg, task=task: statuses.update({task: json.loads(msg.data)}), qos))
    for camera in ('external_camera', 'wrist_camera'):
        for stream in ('color', 'depth'):
            topic = f'/sim/{camera}/{stream}/image_raw'
            subscriptions.append(peer.create_subscription(Image, topic,
                lambda msg, topic=topic: frames.add(topic), 1))
    subscriptions.append(peer.create_subscription(SimGraspFeedback, '/sim/grasp_feedback',
        lambda msg: contacts.append(msg) if msg.object_name == 'green_cylinder' else None, 10))
    arm = ActionClient(peer, FollowJointTrajectory, '/arm_trajectory_controller/follow_joint_trajectory')
    hand = ActionClient(peer, FollowJointTrajectory, '/hand_controller/follow_joint_trajectory')
    controllers = peer.create_client(ListControllers, '/controller_manager/list_controllers')
    approach = peer.create_client(GetParameters, '/object_approach/get_parameters')
    grasp = peer.create_client(GetParameters, '/hand_grasp/get_parameters')
    lift = peer.create_client(GetParameters, '/object_lift/get_parameters')
    def until(predicate, timeout=35):
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.02)
            assert process.poll() is None, (tmp_path/'bringup.log').read_text()[-6000:]
            if predicate(): return
        pytest.fail('Combined launch readiness timed out:\n'+(tmp_path/'bringup.log').read_text()[-6000:])
    try:
        until(lambda: len(statuses)==3 and len(frames)==4 and contacts
            and arm.server_is_ready() and hand.server_is_ready() and controllers.service_is_ready()
            and approach.service_is_ready() and grasp.service_is_ready() and lift.service_is_ready()
            and (not visible or 'grasp_bringup_rviz' in peer.get_node_names()))
        names = peer.get_node_names()
        for name in ('robot_state_publisher', 'controller_manager', 'safety_filter',
                     'mujoco_sim', 'object_approach', 'hand_grasp', 'object_lift'):
            assert names.count(name)==1, names
        assert names.count('grasp_bringup_rviz')==(1 if visible else 0), names
        assert 'approach_execution_rviz' not in names and 'rviz2' not in names
        assert all(status['state']=='idle' and status['active_request_id'] is None
                   for status in statuses.values())
        for topic in ('/arm/approach/target', '/arm/grasp/target', '/arm/lift/target'):
            assert len(peer.get_subscriptions_info_by_topic(topic))==1
            assert not peer.get_publishers_info_by_topic(topic)
        # Spawners finish independently. Camera/joint traffic and arm/hand
        # actions can appear before the broadcaster is registered; poll the
        # service until every required controller is actually active.
        response = None
        def controllers_active():
            nonlocal response
            if response is None:
                response = controllers.call_async(ListControllers.Request())
                return False
            if not response.done(): return False
            active = {controller.name: controller.state for controller in response.result().controller}
            response = None
            return all(active.get(name)=='active' for name in
                       ('joint_state_broadcaster', 'arm_trajectory_controller', 'hand_controller'))
        until(controllers_active)
        a = approach.call_async(GetParameters.Request(names=['pregrasp_flange_clearance']))
        b = grasp.call_async(GetParameters.Request(names=['max_joint_step']))
        c = lift.call_async(GetParameters.Request(names=['lift_height']))
        until(lambda: a.done() and b.done() and c.done())
        assert a.result().values[0].double_value==pytest.approx(.31)
        assert b.result().values[0].double_value==pytest.approx(.04)
        assert c.result().values[0].double_value==pytest.approx(.11)
        assert len(contacts[-1].contact_links)==len(contacts[-1].contact_normal_forces)
    finally:
        executor.shutdown(); arm.destroy(); hand.destroy(); peer.destroy_node(); context.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try: process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=5)
        log.close()
