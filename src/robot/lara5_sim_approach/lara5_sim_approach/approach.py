#!/usr/bin/env python3
"""Plan/execute a noncontact approach to a manually selected simulation prop."""

import copy
import fcntl
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import message_filters
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from geometry_msgs.msg import PoseStamped, Quaternion
from lara5_safety_msgs.msg import SceneBox
from lara5_safety_msgs.srv import CheckTrajectory, GetFramePose, PlanCartesianPath, SetSceneBoxes
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import yaml

from lara5_sim_approach.trajectory import (duration, interpolation_samples, retime_cartesian,
                                         seconds, validate_trajectory)
from lara5_sim_approach.vision import detect_objects, image_array
from lara5_mujoco_sim.obstacles import load_obstacles
from lara5_sim_approach.routing import RouteRejected, find_route, waypoint_routes

ARM = [f"lara5_joint{i}" for i in range(1, 7)]


def rotation_matrix(quat):
    q = np.array([quat.x, quat.y, quat.z, quat.w], dtype=float)
    q /= np.linalg.norm(q)
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class Approach(Node):
    def __init__(self):
        super().__init__("sim_object_approach")
        self.run_lock = open("/tmp/lara5_sim_approach.lock", "w", encoding="utf-8")
        try:
            fcntl.flock(self.run_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError("Another approach client already owns this simulation") from error
        default = str(Path(get_package_share_directory("lara5_mujoco_sim")) / "config/mujoco_sim.yaml")
        obstacles = str(Path(get_package_share_directory("lara5_safety_filter")) / "config/obstacles.yaml")
        for name, value in {"object_name": "red_cube", "execute": False, "config_file": default,
                            "obstacles_file": obstacles,
                            "output_file": "/tmp/lara5_approach.json", "freshness": 2.0,
                            "flange_clearance": 0.35, "preapproach_distance": 0.10,
                            "route_attempts": 28, "planning_timeout": 30.0,
                            "lift_step": 0.08, "detour_margin": 0.12}.items():
            self.declare_parameter(name, value)
        with open(self.get_parameter("config_file").value, encoding="utf-8") as stream:
            self.config = yaml.safe_load(stream)
        self.static_obstacles = load_obstacles(self.get_parameter("obstacles_file").value)
        self.get_logger().info(f"Static obstacle file: {self.get_parameter('obstacles_file').value}; "
                               f"bodies: {[obj['name'] for obj in self.static_obstacles]}")
        self.target = self.get_parameter("object_name").value
        names = [obj["name"] for obj in self.config.get("objects", [])]
        if self.target not in names:
            raise ValueError(f"Unknown object {self.target!r}; choose one of {names}")
        self.frames = None
        self.joints = None
        self.joint_received = self.sim_received = 0.0
        self.description = ""
        self.active_goal = None
        self.active_result = None
        self.hand_reference = {}
        self.joint_limits = {}
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/robot_description", self.on_description, transient)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.create_subscription(JointState, "/sim/joint_states", self.on_sim_state, 10)
        self.rgb = message_filters.Subscriber(self, Image, "/sim/external_camera/color/image_raw")
        self.depth = message_filters.Subscriber(self, Image, "/sim/external_camera/depth/image_raw")
        self.info = message_filters.Subscriber(self, CameraInfo, "/sim/external_camera/camera_info")
        self.sync = message_filters.TimeSynchronizer([self.rgb, self.depth, self.info], 5)
        self.sync.registerCallback(self.on_frames)
        self.plan_pub = self.create_publisher(JointTrajectory, "/sim/planned_arm_trajectory", transient)
        self.pose_pub = self.create_publisher(PoseStamped, "/sim/approach_target", transient)
        self.fk_client = self.create_client(GetFramePose, "/safety/arm/get_frame_pose")
        self.cart_client = self.create_client(PlanCartesianPath, "/safety/arm/plan_cartesian_path")
        self.safety_scene = self.create_client(SetSceneBoxes, "/safety/arm/set_scene_boxes")
        self.safety_check = self.create_client(CheckTrajectory, "/safety/arm/check_trajectory")
        self.arm = ActionClient(self, FollowJointTrajectory,
                                "/arm_trajectory_controller/follow_joint_trajectory")

    def on_description(self, msg):
        self.description = msg.data
        for joint in ET.fromstring(self.description).findall("joint"):
            if joint.get("name") in ARM:
                limit = joint.find("limit")
                self.joint_limits[joint.get("name")] = (float(limit.get("lower")), float(limit.get("upper")))

    def on_joints(self, msg):
        if set(ARM).issubset(msg.name) and len(msg.position) == len(msg.name):
            self.joints, self.joint_received = msg, time.monotonic()

    def on_sim_state(self, _msg):
        self.sim_received = time.monotonic()

    def on_frames(self, rgb, depth, info):
        if rgb.header.frame_id == depth.header.frame_id == info.header.frame_id:
            self.frames = (rgb, depth, info)

    def wait(self, predicate, timeout, reason):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError(reason)

    def call(self, client, request, timeout=30.0):
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"Service unavailable: {client.srv_name}")
        future = client.call_async(request)
        self.wait(future.done, timeout, f"Service timed out: {client.srv_name}")
        result = future.result()
        if result is None:
            raise RuntimeError(f"Service failed: {client.srv_name}")
        return result

    def assert_simulation(self):
        plugins = [p.text for p in ET.fromstring(self.description).findall("ros2_control/hardware/plugin")]
        if not plugins or any(p != "topic_based_ros2_control/TopicBasedSystem" for p in plugins):
            raise RuntimeError("This prototype requires a simulation-only robot description")
        if time.monotonic() - self.sim_received > self.get_parameter("freshness").value:
            raise RuntimeError("Simulation heartbeat is missing/stale")
        if time.monotonic() - self.joint_received > self.get_parameter("freshness").value:
            raise RuntimeError("Robot joint feedback is stale")
        current = dict(zip(self.joints.name, self.joints.position))
        if any(abs(current.get(n, math.inf)-v) > 0.02 for n, v in self.hand_reference.items()):
            raise RuntimeError("Hand posture changed during the fixed-hand approach experiment")

    def observations(self):
        if self.frames is None:
            raise RuntimeError("No synchronized external RGB/depth/CameraInfo sample")
        rgb, depth, info = self.frames
        stamp = rclpy.time.Time.from_msg(rgb.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if not 0 <= age <= self.get_parameter("freshness").value:
            raise RuntimeError("External camera sample is stale or in a different clock domain")
        transform = self.tf.lookup_transform("world", rgb.header.frame_id, stamp).transform
        result = detect_objects(image_array(rgb), image_array(depth), info.k,
                                rotation_matrix(transform.rotation),
                                [transform.translation.x, transform.translation.y, transform.translation.z],
                                self.config["objects"])
        missing = set(obj["name"] for obj in self.config["objects"]) - result.keys()
        if missing:
            raise RuntimeError(f"Scene incomplete: not confidently visible: {sorted(missing)}")
        return result

    def state(self):
        return self.arm_state(self.joints)

    @staticmethod
    def arm_state(state):
        positions = dict(zip(state.name, state.position))
        return JointState(name=ARM, position=[positions[n] for n in ARM])

    def apply_scene(self, detections):
        # This is the same native scene used to plan, preview and execute.
        safety = SetSceneBoxes.Request(frame_id="world")
        for detection in detections.values():
            box = SceneBox(name=f"demo_{detection.name}")
            box.center.x, box.center.y, box.center.z = map(float, detection.center)
            box.size.x, box.size.y, box.size.z = map(float, detection.dimensions)
            safety.boxes.append(box)
        response = self.call(self.safety_scene, safety)
        if not response.success:
            raise RuntimeError(f"Safety filter rejected perception scene: {response.message}")

    def endpoint_state(self, path):
        return self.arm_state(JointState(name=list(path.joint_names), position=list(path.points[-1].positions)))

    def flange_pose(self, state):
        request = GetFramePose.Request(start_state=self.arm_state(state),
                                       frame="lara5_flange", reference_frame="world")
        response = self.call(self.fk_client, request)
        if not response.success:
            raise RouteRejected(f"NEURA forward kinematics failed: {response.message}")
        return response.pose.pose

    def native_segment(self, start_state, target, relative=False):
        request = PlanCartesianPath.Request(start_state=self.arm_state(start_state),
                                             target=target, relative=relative)
        response = self.call(self.cart_client, request)
        if not response.success:
            raise RouteRejected(f"Native Cartesian planning failed: {response.message}")
        if response.trajectory.points and (set(response.trajectory.joint_names) != set(ARM) or
                                           len(response.trajectory.joint_names) != len(ARM)):
            raise RouteRejected("Native IK returned an invalid arm joint list")
        return response.trajectory

    def global_plan(self, xyz):
        # Freeze one measured start for every candidate; no movement while planning.
        start = self.state()
        initial_pose = self.flange_pose(start)
        current_xyz = (initial_pose.position.x, initial_pose.position.y, initial_pose.position.z)
        routes = waypoint_routes(current_xyz, xyz, self.get_parameter("lift_step").value,
                                  self.get_parameter("detour_margin").value)
        orientations = [Quaternion(x=math.cos(yaw/2), y=math.sin(yaw/2), z=0.0, w=0.0)
                        for yaw in (0, math.pi/2, math.pi, -math.pi/2)]
        initial_q = initial_pose.orientation
        orientations.sort(key=lambda q: -abs(q.x*initial_q.x + q.y*initial_q.y +
                                             q.z*initial_q.z + q.w*initial_q.w))

        def attempt(route, orientation):
            state = copy.deepcopy(start)
            raw = JointTrajectory(joint_names=ARM,
                                  points=[JointTrajectoryPoint(positions=list(start.position))])
            for index, xyz_point in enumerate(route.waypoints):
                target = PoseStamped()
                target.header.frame_id = "world"
                target.pose.position.x, target.pose.position.y, target.pose.position.z = map(float, xyz_point)
                # Lift initially with the measured orientation, then turn toward the goal.
                target.pose.orientation = copy.deepcopy(initial_q if index == 0 and len(route.waypoints) > 1
                                                         else orientation)
                segment = self.native_segment(state, target)
                for point in segment.points:
                    positions = dict(zip(segment.joint_names, point.positions))
                    ordered = [positions[name] for name in ARM]
                    if max(abs(a-b) for a, b in zip(ordered, raw.points[-1].positions)) > 1e-10:
                        raw.points.append(JointTrajectoryPoint(positions=ordered))
                if segment.points:
                    state = self.endpoint_state(segment)
            if len(raw.points) == 1:
                raw.points.append(copy.deepcopy(raw.points[0]))  # Already at the pre-approach pose.
            try:
                path = retime_cartesian(raw, velocity=0.21, acceleration=0.21)
                self.validate(path)
                local = self.cartesian_plan(self.endpoint_state(path),
                                             self.get_parameter("preapproach_distance").value)
                self.validate(local)
            except ValueError as error:
                raise RouteRejected(str(error)) from error
            return path, local

        route, paths, count = find_route(
            routes, orientations, attempt,
            max_attempts=self.get_parameter("route_attempts").value,
            timeout=self.get_parameter("planning_timeout").value,
            on_reject=lambda name, why: self.get_logger().debug(f"Rejected NEURA route {name}: {why}"))
        self.planned_approach = paths[1]
        self.selected_route = route.name
        self.get_logger().info(f"NEURA IK accepted route {route.name} after {count} candidate(s); "
                               "global motion and final approach both validated")
        return paths[0]

    def cartesian_plan(self, start_state, distance):
        # Native NEURA IK, seeded by the predicted or measured arm configuration.
        # World-relative down keeps the reached flange orientation unchanged.
        target = PoseStamped()
        target.header.frame_id = "world"
        target.pose.orientation.w = 1.0
        target.pose.position.z = -distance
        path = self.native_segment(start_state, target, relative=True)
        if not path.points:
            raise RouteRejected("Native Cartesian approach contains no motion")
        positions = dict(zip(start_state.name, start_state.position))
        path.points.insert(0, JointTrajectoryPoint(positions=[positions[n] for n in path.joint_names]))
        # Preserve native IK geometry; conservative cubic timing stays on checked segments.
        return retime_cartesian(path)

    def validate(self, path):
        validate_trajectory(path, ARM)
        native = CheckTrajectory.Request()
        native.start_state = JointState(name=list(path.joint_names), position=list(path.points[0].positions))
        native.trajectory.joint_names = list(path.joint_names)
        # Native IK checks geometry; also check the post-processed controller splines.
        # This is discrete checking, not a continuous swept-volume guarantee.
        for positions, velocities, accelerations in interpolation_samples(path):
            native.trajectory.points.append(JointTrajectoryPoint(positions=positions.tolist()))
            for name, value in zip(path.joint_names, positions):
                low, high = self.joint_limits[name]
                if not low - 1e-6 <= value <= high + 1e-6:
                    raise RouteRejected(f"Interpolated trajectory exceeds {name} position limits")
            if np.max(np.abs(velocities)) > 0.6 + 1e-4 or np.max(np.abs(accelerations)) > 0.6 + 1e-4:
                raise RouteRejected("Interpolated trajectory exceeds demo dynamic limits")
        checked = self.call(self.safety_check, native)
        if not checked.valid:
            raise RouteRejected(f"Safety filter rejected trajectory preview: {checked.message}")

    def scene_unchanged(self, baseline):
        self.assert_simulation()
        observed = self.observations()
        for name, original in baseline.items():
            if np.linalg.norm(original.center - observed[name].center) > 0.02:
                raise RuntimeError(f"Camera feedback invalidated the plan: {name} estimate changed "
                                   f"from {original.center.tolist()} to {observed[name].center.tolist()}")

    def execute_path(self, path, baseline):
        self.scene_unchanged(baseline)
        current = dict(zip(self.joints.name, self.joints.position))
        if max(abs(current[n]-v) for n, v in zip(path.joint_names, path.points[0].positions)) > 0.04:
            raise RuntimeError("Measured start state differs from the planned trajectory")
        if not self.arm.wait_for_server(timeout_sec=10):
            raise RuntimeError("Arm trajectory action is unavailable")
        goal = FollowJointTrajectory.Goal(trajectory=path, goal_time_tolerance=duration(3.0))
        goal.path_tolerance = [JointTolerance(name=n, position=0.15) for n in ARM]
        goal.goal_tolerance = [JointTolerance(name=n, position=0.03) for n in ARM]
        future = self.arm.send_goal_async(goal)
        self.wait(future.done, 10, "Trajectory goal acknowledgement timed out")
        self.active_goal = future.result()
        if not self.active_goal.accepted:
            self.active_goal = None
            raise RuntimeError("Controller rejected trajectory")
        result = self.active_goal.get_result_async()
        self.active_result = result
        deadline = time.monotonic() + seconds(path.points[-1].time_from_start) + 10
        last_check = 0.0
        while rclpy.ok() and not result.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() > deadline:
                raise RuntimeError("Trajectory execution timed out")
            if time.monotonic() - last_check > 0.5:
                self.scene_unchanged(baseline)
                last_check = time.monotonic()
        outcome = result.result()
        self.active_goal = None
        if outcome.status != 4 or outcome.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f"Controller failed: {outcome.result.error_code}: {outcome.result.error_string}")

    def cancel(self):
        if self.active_goal is not None and self.active_goal.accepted:
            future = self.active_goal.cancel_goal_async()
            self.wait(future.done, 5, "Controller cancellation acknowledgement timed out")
            if self.active_result is not None:
                self.wait(self.active_result.done, 5, "Controller did not finish cancellation")
            self.active_goal = None

    def run(self):
        self.get_logger().info(f"Waiting for simulated robot and external camera; selected={self.target}")
        self.wait(lambda: bool(self.description and self.joints is not None and self.frames and self.sim_received),
                  30, "Simulation/camera inputs not ready")
        self.assert_simulation()
        self.hand_reference = {n: v for n, v in zip(self.joints.name, self.joints.position)
                               if n.startswith("right_")}
        for name in ("freshness", "flange_clearance", "preapproach_distance", "lift_step", "detour_margin", "planning_timeout"):
            value = self.get_parameter(name).value
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if self.get_parameter("flange_clearance").value < 0.30:
            raise ValueError("This open-hand demo requires at least 0.30 m flange clearance")
        baseline = self.observations()
        capture_stamp = copy.deepcopy(self.frames[0].header.stamp)
        selected = baseline[self.target]
        self.get_logger().info(f"RGB-D target top in world: {selected.top.tolist()} ({selected.pixels} pixels)")
        self.apply_scene(baseline)
        above = selected.top.copy()
        above[2] += self.get_parameter("flange_clearance").value + self.get_parameter("preapproach_distance").value
        pose = PoseStamped()
        pose.header.frame_id, pose.header.stamp = "world", self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, above)
        pose.pose.orientation.x = 1.0
        self.pose_pub.publish(pose)
        self.get_logger().info("Planning global motion with NEURA IK Cartesian waypoint routing")
        global_path = self.global_plan(above)
        local_path = self.planned_approach
        self.plan_pub.publish(global_path)
        report = {
            "object": self.target, "source": "rendered_rgb_depth", "mode": "plan_only",
            "global_planner": "NEURA IK Cartesian waypoint routing", "approach_planner": "NEURA IK",
            "selected_route": self.selected_route,
            "execution": "safety-filtered FollowJointTrajectory",
            "obstacles_file": self.get_parameter("obstacles_file").value,
            "camera_stamp": {"sec": capture_stamp.sec, "nanosec": capture_stamp.nanosec},
            "target_top_world": selected.top.tolist(),
            "phases": [{"name": name, "joint_names": list(path.joint_names), "points": [
                {"positions": list(p.positions), "velocities": list(p.velocities),
                 "accelerations": list(p.accelerations), "time": seconds(p.time_from_start)}
                for p in path.points]} for name, path in [("global", global_path), ("approach", local_path)]],
        }
        output = Path(self.get_parameter("output_file").value)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        self.get_logger().info(f"Both phases planned; saved {output}")
        if self.get_parameter("execute").value:
            self.get_logger().info("Executing NEURA IK global route in simulation")
            self.execute_path(global_path, baseline)
            # Recompute the final approach from actual joints after global tracking.
            self.scene_unchanged(baseline)
            local_path = self.cartesian_plan(self.state(), self.get_parameter("preapproach_distance").value)
            self.validate(local_path)
            self.get_logger().info("Executing NEURA IK Cartesian noncontact approach")
            # Record the trajectory actually replanned from measured joint state.
            report["phases"][1]["points"] = [
                {"positions": list(p.positions), "velocities": list(p.velocities),
                 "accelerations": list(p.accelerations), "time": seconds(p.time_from_start)}
                for p in local_path.points]
            self.execute_path(local_path, baseline)
            self.scene_unchanged(baseline)
            report["mode"] = "executed"
            report["final_joint_positions"] = dict(zip(self.joints.name, self.joints.position))
            flange = self.flange_pose(self.state()).position
            report["final_flange_world"] = [flange.x, flange.y, flange.z]
            desired = selected.top + [0.0, 0.0, self.get_parameter("flange_clearance").value]
            report["flange_goal_error_m"] = float(np.linalg.norm(
                np.array([flange.x, flange.y, flange.z]) - desired))
            output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
            self.get_logger().info("Approach complete; hand remains open above the selected object")
        else:
            self.get_logger().info("PLAN ONLY: rerun with -p execute:=true to move the simulated arm")


def main(node_class=Approach):
    # Keep the context alive on Ctrl+C long enough to cancel an active action.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    code = 0
    try:
        node = node_class()
        node.run()
    except (Exception, KeyboardInterrupt) as error:
        code = 1
        if node is not None:
            node.get_logger().error(str(error) or "Interrupted")
            try:
                node.cancel()
            except Exception as cancel_error:
                node.get_logger().error(f"Cancellation failed: {cancel_error}")
        else:
            print(error)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
