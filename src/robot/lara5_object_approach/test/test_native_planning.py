"""Opt-in native planning integration, synthetic feedback and no controllers.

Run only in an isolated container/domain with ROS_LOCALHOST_ONLY=1 and
LARA5_NATIVE_PLANNING_TEST=1. The backend's motion inputs/outputs are remapped
to unused names. This test calls read-only services and publishes preview poses.
"""

import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import yaml

from lara5_object_approach.preview import GraspPreview


@pytest.mark.skipif(os.environ.get("LARA5_NATIVE_PLANNING_TEST") != "1", reason="Opt-in native planning only")
def test_native_two_phase_plans_and_rejections(tmp_path):
    assert os.environ.get("ROS_LOCALHOST_ONLY") == "1"
    assert os.environ.get("ROS_DOMAIN_ID") not in (None, "0", "42")
    prefix = "/native_plan_"+uuid.uuid4().hex[:8]
    robot = Path(get_package_share_directory("lara5_controller_manager"))
    safety = Path(get_package_share_directory("lara5_safety_filter"))
    urdf = subprocess.check_output(["xacro", str(robot/"urdf/lara5_inspire_hand_ros2_control.urdf.xacro"),
                                    "deployment_mode:=simulation"], text=True)
    params = yaml.safe_load((safety/"config/safety_filter.yaml").read_text())["safety_filter"]["ros__parameters"]
    params.update(robot_description=urdf,
                  obstacles_file=str(safety/"config/obstacles.yaml"), input_action=prefix+"/unused_action",
                  output_action=prefix+"/no_controller", move_to_pose_action=prefix+"/unused_move",
                  target_pose_topic=prefix+"/unused_pose", input_trajectory_topic=prefix+"/unused_trajectory",
                  output_trajectory_topic=prefix+"/no_controller_topic")
    config = tmp_path/"params.yaml"
    config.write_text(yaml.safe_dump({"safety_filter": {"ros__parameters": params}}))
    remaps = ["-r", "/joint_states:="+prefix+"/states"]
    for suffix in ("plan_cartesian_path", "get_frame_pose", "check_trajectory"):
        remaps += ["-r", "/safety/arm/"+suffix+":="+prefix+"/"+suffix]
    log = open(tmp_path/"native.log", "w+")
    server = subprocess.Popen(["ros2", "run", "lara5_safety_filter", "safety_filter_node", "--ros-args",
        "--params-file", str(config), *remaps], stdout=log, stderr=log, start_new_session=True)
    rclpy.init()
    peer = rclpy.create_node("native_planning_observer")
    preview = GraspPreview(cli_args=["--ros-args", *remaps], parameter_overrides=[
        Parameter("plan_trajectories", value=True)])
    publisher = peer.create_publisher(PoseStamped, "/arm/approach/preview/target", 1)
    state_pub = peer.create_publisher(JointState, prefix+"/states", 10)
    q = [0., 0., 1.5708, 0., 1.5708, 0.]
    def state():
        message = JointState(name=preview.planner.names+preview.planner.open_names, position=q+[0.]*6)
        message.header.stamp = peer.get_clock().now().to_msg()
        state_pub.publish(message)
    timer = peer.create_timer(.02, state)
    events = []
    subscription = peer.create_subscription(String, "/arm/approach/preview/plan_status",
        lambda msg: events.append(json.loads(msg.data)),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    executor = SingleThreadedExecutor()
    executor.add_node(peer); executor.add_node(preview)
    results = []
    def spin_until(predicate, timeout):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
            if predicate(): return
        raise AssertionError("native planning integration timed out")
    try:
        spin_until(lambda: publisher.get_subscription_count() and len(preview.planner.samples) == 12
            and all(c.service_is_ready() for c in (preview.planner.plan_client,
                    preview.planner.fk_client, preview.planner.check_client)), 30)
        cases = [([.25, -.2, .95], -math.pi/2, True), ([.22, -.18, .95], -math.pi/2, True),
                 ([.28, -.22, .95], -math.pi/2, True), ([.25, -.2, .95], 0., False),
                 ([3., -.2, .95], 0., False)]
        for xyz, yaw, accepted in cases:
            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            previous = len(events)
            publisher.publish(pose)
            spin_until(lambda: any(e["state"] in ("plan_ready", "plan_failed", "rejected_state")
                                  for e in events[previous:]), 65)
            result = dict(object_xyz=xyz, yaw=yaw, status=events[-1])
            if events[-1]["state"] == "plan_ready":
                context = preview.planner.valid
                result["paths_xyz"] = [[p[:3, 3].tolist() for p in path] for path in context["paths"]]
            results.append(result)
            print(json.dumps(result["status"]), flush=True)
            assert (events[-1]["state"] == "plan_ready") == accepted
            if accepted:
                assert len(preview.planner.valid["phases"]) == 2
                assert max(e[0] for e in events[-1]["endpoint_errors_m_rad"]) < .001
    finally:
        output = Path(os.environ.get("LARA5_NATIVE_PLAN_REPORT", "/tmp/native_two_phase_review.json"))
        output.write_text(json.dumps(results, indent=2))
        executor.shutdown(); preview.destroy_node(); peer.destroy_node(); rclpy.shutdown()
        os.killpg(server.pid, signal.SIGINT)
        try: server.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(server.pid, signal.SIGKILL); server.wait(timeout=5)
        log.close()
