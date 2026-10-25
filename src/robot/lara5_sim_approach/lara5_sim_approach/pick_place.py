"""Contact-verified, simulation-only pick/place state machine using native NEURA IK."""

import copy
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import message_filters
import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Vector3
from lara5_safety_msgs.msg import SceneBox, SimGraspFeedback
from lara5_safety_msgs.srv import SetGraspScene
from rclpy.action import ActionClient
from sensor_msgs.msg import CameraInfo, Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import TransformException

from lara5_sim_approach.approach import ARM, Approach, rotation_matrix
from lara5_sim_approach.grasping import (HAND, flange_for_object, grip_slipped,
    hand_offset_in_flange, palm_down_rotation, quaternion_xyzw, validate_destination, vector3)
from lara5_sim_approach.routing import RouteRejected, find_route, waypoint_routes
from lara5_sim_approach.trajectory import duration, retime_cartesian, seconds
from lara5_sim_approach.vision import Detection, detect_objects, dimensions, image_array


def pose_values(pose):
    p, q = pose.position, pose.orientation
    return np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w])


def make_pose(position, rotation):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, position)
    pose.orientation = Quaternion(**dict(zip(("x", "y", "z", "w"), map(float, quaternion_xyzw(rotation)))))
    return pose


class PickPlace(Approach):
    def __init__(self):
        super().__init__()
        for name, value in {
            "place_position": [-0.25, 0.30, -1.0],  # Negative Z derives the settled centre from shape height.
            "table_height": 0.90, "grasp_offset_hand": [-0.025, 0.085, 0.150],
            "close_positions": [0.70, 0.90, 0.90, 0.90, 0.90, 0.35],
            "pregrasp_height": 0.06, "lift_height": 0.15, "test_lift": 0.04,
            "release_gap": 0.04, "hand_duration": 3.0, "hand_check_steps": 6,
            "grasp_timeout": 6.0, "settle_timeout": 6.0, "position_tolerance": 0.025,
            "wrist_max_correction": 0.03, "feedback_timeout": 1.0,
        }.items():
            self.declare_parameter(name, value)
        self.wrist_frames = None
        self.wrist_rgb = message_filters.Subscriber(self, Image, "/sim/wrist_camera/color/image_raw")
        self.wrist_depth = message_filters.Subscriber(self, Image, "/sim/wrist_camera/depth/image_raw")
        self.wrist_info = message_filters.Subscriber(self, CameraInfo, "/sim/wrist_camera/camera_info")
        self.wrist_sync = message_filters.TimeSynchronizer(
            [self.wrist_rgb, self.wrist_depth, self.wrist_info], 5)
        self.wrist_sync.registerCallback(self.on_wrist)
        self.feedback = {}
        self.create_subscription(SimGraspFeedback, "/sim/grasp_feedback", self.on_grasp_feedback, 20)
        self.grasp_scene = self.create_client(SetGraspScene, "/safety/arm/set_grasp_scene")
        self.hand = ActionClient(self, FollowJointTrajectory, "/hand_controller/follow_joint_trajectory")
        self.target_may_be_occluded = False
        self.grip_reference = None
        self.contact_lost_at = None
        self.report = {"mode": "planning", "object": self.target, "events": [], "phases": [],
                       "planner": "NEURA IK", "grasp": "MuJoCo contact/friction; no weld or teleport"}
        self.context_dirty = False
        self.motion_started = False

    def p(self, name):
        return self.get_parameter(name).value

    def on_wrist(self, rgb, depth, info):
        if rgb.header.frame_id == depth.header.frame_id == info.header.frame_id:
            self.wrist_frames = rgb, depth, info

    def on_grasp_feedback(self, message):
        self.feedback[message.object_name] = message, time.monotonic()
        if message.object_name == self.target:
            if message.opposing_contacts:
                self.contact_lost_at = None
            elif self.contact_lost_at is None:
                self.contact_lost_at = time.monotonic()

    def latest_feedback(self):
        message, received = self.feedback[self.target]
        age = (self.get_clock().now() - rclpy.time.Time.from_msg(message.header.stamp)).nanoseconds / 1e9
        if time.monotonic() - received > self.p("feedback_timeout") or not 0 <= age <= self.p("feedback_timeout"):
            raise RuntimeError("Simulation grasp feedback is stale")
        for pose in (message.object_pose, message.object_in_flange):
            position, quaternion = pose_values(pose)
            if not np.isfinite(position).all() or not np.isfinite(quaternion).all() or abs(np.linalg.norm(quaternion)-1) > 1e-3:
                raise RuntimeError("Simulation grasp feedback contains an invalid pose")
        if message.header.frame_id != "world" or not math.isfinite(message.linear_speed) or message.linear_speed < 0:
            raise RuntimeError("Simulation grasp feedback has an invalid frame or speed")
        return message

    def camera_sample(self, frames):
        if frames is None:
            raise RuntimeError("No synchronized RGB-D camera sample")
        rgb, depth, info = frames
        stamp = rclpy.time.Time.from_msg(rgb.header.stamp)
        age = (self.get_clock().now()-stamp).nanoseconds/1e9
        if not 0 <= age <= self.p("freshness"):
            raise RuntimeError("RGB-D camera is stale")
        transform = self.tf.lookup_transform("world", rgb.header.frame_id, stamp).transform
        return rgb, depth, info, transform

    def detect(self, frames, objects, min_pixels=20):
        rgb, depth, info, transform = self.camera_sample(frames)
        return detect_objects(image_array(rgb), image_array(depth), info.k,
                              rotation_matrix(transform.rotation),
                              [transform.translation.x, transform.translation.y, transform.translation.z],
                              objects, min_pixels=min_pixels)

    def observations(self):
        result = self.detect(self.frames, self.config["objects"])
        required = {obj["name"] for obj in self.config["objects"]}
        if self.target_may_be_occluded:
            required.discard(self.target)
        if required - result.keys():
            raise RuntimeError(f"External scene is incomplete: {sorted(required-result.keys())}")
        return result

    def scene_unchanged(self, baseline):
        self.assert_simulation()
        observed = self.observations()
        for name, original in baseline.items():
            if name == self.target and self.target_may_be_occluded:
                continue
            if np.linalg.norm(original.center-observed[name].center) > 0.02:
                raise RuntimeError(f"Object {name} moved unexpectedly; stopping")
        if self.target_may_be_occluded:
            self.camera_sample(self.wrist_frames)  # Camera freshness remains required even during occlusion.
        if self.grip_reference is not None:
            message = self.latest_feedback()
            if self.contact_lost_at is not None and time.monotonic()-self.contact_lost_at > 0.30:
                raise RuntimeError("Opposing grasp contacts were lost; stopping with the hand held")
            if grip_slipped(self.grip_reference, pose_values(message.object_in_flange)):
                raise RuntimeError("Object slipped relative to the flange; stopping with the hand held")

    def measured_hand(self):
        measured = dict(zip(self.joints.name, self.joints.position))
        return np.array([measured[name] for name in HAND])

    def boxes(self, detections):
        boxes = []
        for item in detections.values():
            box = SceneBox(name=f"demo_{item.name}")
            box.center.x, box.center.y, box.center.z = map(float, item.center)
            box.size.x, box.size.y, box.size.z = map(float, item.dimensions)
            boxes.append(box)
        return boxes

    def configure_scene(self, hand, detections, touch=False, attached=None, support_contact=False):
        request = SetGraspScene.Request(boxes=self.boxes(detections))
        if hand is not None:
            request.hand_state = JointState(name=HAND, position=list(map(float, hand)))
        if touch or attached is not None:
            request.touch_object = f"demo_{self.target}"
            request.touch_links = self.touch_links
        if attached is not None:
            request.attach_object = True
            request.allow_support_contact = support_contact
            request.boxes = [box for box in request.boxes if box.name != request.touch_object]
            request.object_in_flange = copy.deepcopy(attached)
            # Bounding box follows the physical object orientation, with 10 mm per-side padding.
            request.object_size = Vector3(**dict(zip(("x", "y", "z"), map(float, self.extent+0.02))))
        response = self.call(self.grasp_scene, request)
        if not response.success:
            raise RuntimeError(f"Grasp scene rejected: {response.message}")
        self.context_dirty = hand is not None

    def stationary(self, state):
        return JointTrajectory(joint_names=ARM, points=[
            JointTrajectoryPoint(positions=list(state.position), velocities=[0.0]*6, time_from_start=duration(t))
            for t in (0., 1.)])

    def check_hand_sweep(self, state, begin, end, detections, attached=None):
        # The arm is stationary while fingers move. Rebuild each sampled posture,
        # including mimic joints, and check it against the native collision scene.
        for fraction in np.linspace(0., 1., self.p("hand_check_steps")+1):
            hand = np.asarray(begin) + fraction*(np.asarray(end)-begin)
            self.configure_scene(hand, detections, touch=attached is None, attached=attached)
            self.validate(self.stationary(state))

    def line(self, state, pose):
        target = PoseStamped()
        target.header.frame_id, target.pose = "world", copy.deepcopy(pose)
        raw = self.native_segment(state, target)
        # Include the explicit start even if the service returns no motion.
        if not raw.points:
            return self.stationary(state)
        # Normalize native joint ordering before joining Cartesian segments.
        columns = [raw.joint_names.index(name) for name in ARM]
        raw = JointTrajectory(joint_names=ARM, points=[JointTrajectoryPoint(
            positions=[point.positions[index] for index in columns]) for point in raw.points])
        ordered = dict(zip(state.name, state.position))
        if any(abs(a-b) > 1e-10 for a, b in zip(raw.points[0].positions,
                                               [ordered[n] for n in raw.joint_names])):
            raw.points.insert(0, JointTrajectoryPoint(positions=[ordered[n] for n in raw.joint_names]))
        if len(raw.points) == 1:
            raw.points.append(copy.deepcopy(raw.points[0]))
        try:
            path = retime_cartesian(raw, velocity=0.10, acceleration=0.20)
        except ValueError as error:
            raise RouteRejected(str(error)) from error
        self.validate(path)
        return path

    def route(self, state, pose):
        start = self.flange_pose(state)
        routes = waypoint_routes(pose_values(start)[0], pose_values(pose)[0],
                                  self.p("lift_step"), self.p("detour_margin"))

        def attempt(route, _orientation):
            current = state
            raw = JointTrajectory(joint_names=ARM, points=[JointTrajectoryPoint(positions=list(state.position))])
            for index, xyz in enumerate(route.waypoints):
                waypoint = copy.deepcopy(pose)
                waypoint.position.x, waypoint.position.y, waypoint.position.z = map(float, xyz)
                if index == 0 and len(route.waypoints) > 1:
                    waypoint.orientation = copy.deepcopy(start.orientation)
                segment = self.line(current, waypoint)
                for point in segment.points[1:]:
                    raw.points.append(JointTrajectoryPoint(positions=list(point.positions)))
                current = self.endpoint_state(segment)
            try:
                path = retime_cartesian(raw, velocity=0.15, acceleration=0.20)
            except ValueError as error:
                raise RouteRejected(str(error)) from error
            self.validate(path)
            return path

        selected, path, _count = find_route(routes, [pose.orientation], attempt,
            max_attempts=min(7, self.p("route_attempts")), timeout=self.p("planning_timeout"))
        return path, selected.name

    def nominal_payload(self, rotation):
        # The object is assumed upright before grasp. Runtime carry uses measured
        # simulator contact feedback to account for its actual captured transform.
        return make_pose(self.offset, rotation.T)

    def carry_paths(self, start, rotation, payload, detections, hand):
        # The physical object starts on the table. Only its carried planning box
        # may contact that support during the strictly vertical test lift.
        self.configure_scene(hand, detections, attached=payload, support_contact=True)
        current = self.flange_pose(start)
        paths = []
        for name, delta in (("test_lift", self.p("test_lift")),
                            ("lift", self.p("lift_height")-self.p("test_lift"))):
            current.position.z += delta
            path = self.line(start, current)
            paths.append((name, path))
            start = self.endpoint_state(path)
            if name == "test_lift":
                self.configure_scene(hand, detections, attached=payload)
                self.validate(self.stationary(start))  # Require full clearance before transfer.
        object_offset = pose_values(payload)[0]
        above = flange_for_object(self.destination+[0, 0, self.p("lift_height")], rotation, object_offset)
        path, _route = self.route(start, make_pose(above, rotation))
        paths.append(("transfer", path))
        start = self.endpoint_state(path)
        release_center = self.destination+[0, 0, self.p("release_gap")]
        release = make_pose(flange_for_object(release_center, rotation, object_offset), rotation)
        path = self.line(start, release)
        paths.append(("lower_for_release", path))
        return paths, release_center

    def preview(self, center, yaw):
        rotation = palm_down_rotation(yaw)
        grasp = make_pose(flange_for_object(center, rotation, self.offset), rotation)
        pre = copy.deepcopy(grasp)
        pre.position.z += self.p("pregrasp_height")
        self.configure_scene(self.open_hand, self.baseline)
        global_path, route = self.route(self.state(), pre)
        self.configure_scene(self.open_hand, self.baseline, touch=True)
        approach = self.line(self.endpoint_state(global_path), grasp)
        grasp_state = self.endpoint_state(approach)
        self.check_hand_sweep(grasp_state, self.open_hand, self.closed_hand, self.baseline)
        payload = self.nominal_payload(rotation)
        carry, release_center = self.carry_paths(grasp_state, rotation, payload, self.baseline, self.closed_hand)
        release_state = self.endpoint_state(carry[-1][1])
        released = self.release_scene(release_center)
        self.check_hand_sweep(release_state, self.closed_hand, self.open_hand, released)
        self.configure_scene(self.open_hand, released, touch=True)
        retreat_pose = copy.deepcopy(self.flange_pose(release_state))
        retreat_pose.position.z += self.p("pregrasp_height")
        retreat = self.line(release_state, retreat_pose)
        self.configure_scene(self.open_hand, self.baseline)
        return {"rotation": rotation, "grasp": grasp, "pregrasp": global_path, "approach": approach,
                "carry": carry, "retreat": retreat, "route": route}

    def release_scene(self, center):
        scene = dict(self.baseline)
        scene[self.target] = Detection(self.target, np.asarray(center), self.extent+0.03,
                                        np.asarray(center)+[0, 0, self.extent[2]/2], 0)
        return scene

    def event(self, phase):
        self.report["events"].append({"phase": phase, "time": time.time()})
        self.get_logger().info(f"Pick/place: {phase}")
        self.save()

    def record_path(self, name, path):
        self.report["phases"].append({"name": name, "joint_names": list(path.joint_names), "points": [
            {"positions": list(point.positions), "velocities": list(point.velocities),
             "time": seconds(point.time_from_start)} for point in path.points]})
        self.save()

    def save(self):
        output = Path(self.p("output_file"))
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(self.report, indent=2)+"\n", encoding="utf-8")

    def execute_hand(self, values, phase):
        self.event(phase)
        self.hand_reference = {}  # This phase intentionally moves the hand, with the arm stationary.
        arm_start = np.asarray(self.state().position)
        if not self.hand.wait_for_server(timeout_sec=10):
            raise RuntimeError("Hand trajectory action is unavailable")
        path = JointTrajectory(joint_names=HAND, points=[
            JointTrajectoryPoint(positions=list(map(float, self.measured_hand())), time_from_start=duration(0.)),
            JointTrajectoryPoint(positions=list(map(float, values)), time_from_start=duration(self.p("hand_duration")))])
        goal = FollowJointTrajectory.Goal(trajectory=path, goal_time_tolerance=duration(2.))
        # Contact may prevent fingers from reaching the commanded closing angle.
        # Controller success is NOT evidence of grasp success; contacts/test lift are required.
        goal.path_tolerance = [JointTolerance(name=name, position=1.5) for name in HAND]
        goal.goal_tolerance = [JointTolerance(name=name, position=1.5) for name in HAND]
        future = self.hand.send_goal_async(goal)
        self.wait(future.done, 10, "Hand goal acknowledgement timed out")
        self.active_goal = future.result()
        if not self.active_goal.accepted:
            self.active_goal = None
            raise RuntimeError("Hand controller rejected the goal")
        self.active_result = self.active_goal.get_result_async()
        deadline = time.monotonic()+self.p("hand_duration")+8
        while not self.active_result.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            self.scene_unchanged(self.baseline)
            if np.max(np.abs(np.asarray(self.state().position)-arm_start)) > 0.02:
                raise RuntimeError("Arm moved during the stationary finger phase")
            if time.monotonic() > deadline:
                raise RuntimeError("Hand execution timed out")
        outcome = self.active_result.result()
        self.active_goal = None
        if outcome.status != 4 or outcome.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f"Hand controller failed: {outcome.result.error_string}")
        settle_at = time.monotonic()+0.5
        self.wait(lambda: time.monotonic() >= settle_at, 2, "Hand did not settle")
        self.hand_reference = dict(zip(HAND, self.measured_hand()))
        sample = self.latest_feedback()
        self.report.setdefault("hand_checks", []).append({
            "phase": phase, "measured_positions": dict(self.hand_reference),
            "contact_links": list(sample.contact_links), "opposing_contacts": sample.opposing_contacts,
            "supported": sample.supported})
        self.save()

    def stable_grip(self):
        stable_since = None

        def ready():
            nonlocal stable_since
            self.scene_unchanged(self.baseline)
            if not self.latest_feedback().opposing_contacts:
                stable_since = None
                return False
            if stable_since is None:
                stable_since = time.monotonic()
            return time.monotonic()-stable_since >= 0.35

        self.wait(ready, self.p("grasp_timeout"), "No stable thumb/finger grasp; no lift was sent")

    def wrist_refine(self, baseline_center):
        received = None

        def ready():
            nonlocal received
            try:
                result = self.detect(self.wrist_frames, [self.object_config], min_pixels=12)
            except TransformException:
                return False  # Wait for robot TF to catch up with this image stamp.
            received = result.get(self.target)
            return received is not None

        self.wait(ready, 5, "Selected object not confidently visible to the wrist camera; no contact approach sent")
        correction = received.center-baseline_center
        if np.linalg.norm(correction) > self.p("wrist_max_correction"):
            raise RuntimeError(f"Wrist/external target disagreement is {np.linalg.norm(correction):.3f} m")
        self.report["wrist_correction_world"] = correction.tolist()
        return received.center

    def setup(self):
        self.wait(lambda: bool(self.description and self.joints is not None and self.frames and
                              self.wrist_frames and self.sim_received and self.target in self.feedback),
                  30, "Simulation, cameras and grasp feedback not ready; use the rebuilt pick_place launch")
        self.assert_simulation()
        for name in ("pregrasp_height", "lift_height", "test_lift", "release_gap", "hand_duration",
                     "grasp_timeout", "settle_timeout", "position_tolerance", "wrist_max_correction",
                     "feedback_timeout", "planning_timeout", "freshness", "table_height"):
            if not math.isfinite(self.p(name)) or self.p(name) <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if self.p("hand_check_steps") < 2 or self.p("lift_height") <= self.p("test_lift"):
            raise ValueError("Need at least two hand checks and lift_height > test_lift")
        if self.p("release_gap") < 0.04:
            raise ValueError("release_gap must be at least 0.04 m for the padded payload and clearance")
        if self.p("test_lift") < 0.04:
            raise ValueError("test_lift must be at least 0.04 m to clear the padded payload from its support")
        self.closed_hand = np.asarray(self.p("close_positions"), dtype=float)
        if self.closed_hand.shape != (6,) or not np.isfinite(self.closed_hand).all():
            raise ValueError("close_positions requires six finite angles in documented HAND order")
        root = ET.fromstring(self.description)
        self.touch_links = [link.get("name") for link in root.findall("link")
                            if link.get("name", "").startswith("right_")]
        limits = {joint.get("name"): joint.find("limit") for joint in root.findall("joint")}
        for name, angle in zip(HAND, self.closed_hand):
            limit = limits[name]
            if not float(limit.get("lower")) <= angle <= float(limit.get("upper")):
                raise ValueError(f"Closing angle for {name} is outside its limits")
        self.open_hand = np.zeros(6)
        if np.max(np.abs(self.measured_hand())) > 0.10:
            raise RuntimeError("Start with an open hand; restart the simulation before a fresh pick/place trial")
        self.hand_reference = dict(zip(HAND, self.measured_hand()))
        self.object_config = next(obj for obj in self.config["objects"] if obj["name"] == self.target)
        self.extent = dimensions(self.object_config)
        self.offset = hand_offset_in_flange(self.p("grasp_offset_hand"))
        self.baseline = self.observations()
        self.destination = vector3(self.p("place_position"), "place_position")
        if self.destination[2] < 0:
            self.destination[2] = self.p("table_height")+self.extent[2]/2
        validate_destination(self.destination, self.extent,
            [(name, item.center, item.dimensions) for name, item in self.baseline.items() if name != self.target],
            table_height=self.p("table_height"))
        self.report["place_position_world"] = self.destination.tolist()
        self.report["source_center_world"] = self.baseline[self.target].center.tolist()
        self.report["grasp_offset_hand"] = list(self.p("grasp_offset_hand"))
        self.report["close_positions"] = dict(zip(HAND, map(float, self.closed_hand)))
        self.get_logger().info(f"Placement target (settled object centre, world metres): {self.destination.tolist()}")

    def run(self):
        execute = self.p("execute")
        try:
            self.setup()
            self.event("preview_complete_cycle")
            plan, failures = None, []
            for yaw in (0., math.pi/2, math.pi, -math.pi/2):
                try:
                    plan = self.preview(self.baseline[self.target].center, yaw)
                    break
                except RouteRejected as error:
                    failures.append(str(error))
                    self.get_logger().debug(f"Rejected grasp yaw {yaw}: {error}")
            if plan is None:
                raise RouteRejected(f"All palm-down grasp candidates rejected: {failures[-1]}")
            self.report["preview"] = True
            self.report["selected_route"] = plan["route"]
            for name, path in [("pregrasp", plan["pregrasp"]), ("contact_approach", plan["approach"]),
                               *plan["carry"], ("retreat", plan["retreat"])]:
                self.record_path("preview_"+name, path)
            if not execute:
                self.report["mode"] = "plan_only"
                self.event("plan_only_complete_no_arm_or_hand_commands")
                return
            self.report["mode"] = "executing"
            self.event("pregrasp")
            self.configure_scene(self.measured_hand(), self.baseline)
            self.motion_started = True
            self.execute_path(plan["pregrasp"], self.baseline)
            self.event("wrist_camera_refinement")
            center = self.wrist_refine(self.baseline[self.target].center)
            self.target_may_be_occluded = True
            self.configure_scene(self.measured_hand(), self.baseline, touch=True)
            grasp = make_pose(flange_for_object(center, plan["rotation"], self.offset), plan["rotation"])
            approach = self.line(self.state(), grasp)
            self.check_hand_sweep(self.endpoint_state(approach), self.measured_hand(),
                                  self.closed_hand, self.baseline)
            self.configure_scene(self.measured_hand(), self.baseline, touch=True)
            self.event("contact_approach")
            self.record_path("contact_approach", approach)
            self.execute_path(approach, self.baseline)
            self.execute_hand(self.closed_hand, "close_hand")
            self.event("verify_opposing_contacts")
            self.stable_grip()
            feedback = self.latest_feedback()
            object_rotation = rotation_matrix(feedback.object_pose.orientation)
            if object_rotation[2, 2] < math.cos(0.20):
                raise RuntimeError("Object tilted too far for this upright tabletop placement prototype")
            payload = copy.deepcopy(feedback.object_in_flange)
            self.grip_reference = pose_values(payload)
            carry, release_center = self.carry_paths(self.state(), plan["rotation"], payload,
                                                     self.baseline, self.measured_hand())
            # Verify the entire revised transfer and opening sweep before the test lift.
            release_state = self.endpoint_state(carry[-1][1])
            self.check_hand_sweep(release_state, self.measured_hand(), self.open_hand,
                                  self.release_scene(release_center))
            self.configure_scene(self.measured_hand(), self.baseline, attached=payload, support_contact=True)
            for name, _preview in carry:
                # Replan each phase from measured joints to account for controller tracking.
                current = self.flange_pose(self.state())
                if name in ("test_lift", "lift"):
                    current.position.z += self.p("test_lift") if name == "test_lift" else (
                        self.p("lift_height")-self.p("test_lift"))
                    path = self.line(self.state(), current)
                else:
                    height = self.p("lift_height") if name == "transfer" else self.p("release_gap")
                    desired = make_pose(flange_for_object(self.destination+[0, 0, height],
                        plan["rotation"], pose_values(payload)[0]), plan["rotation"])
                    path = self.route(self.state(), desired)[0] if name == "transfer" else self.line(self.state(), desired)
                self.event(name)
                self.record_path(name, path)
                self.execute_path(path, self.baseline)
                self.scene_unchanged(self.baseline)
                if name == "test_lift":
                    self.stable_grip()
                    lifted = self.latest_feedback()
                    if lifted.supported or lifted.object_pose.position.z-self.baseline[self.target].center[2] < 0.02:
                        raise RuntimeError("Test lift did not lift the object off its support; transfer not sent")
                    self.configure_scene(self.measured_hand(), self.baseline, attached=payload)
                    self.validate(self.stationary(self.state()))
            released_scene = self.release_scene(pose_values(self.latest_feedback().object_pose)[0])
            self.grip_reference = None  # Release intentionally removes grasp contacts.
            self.configure_scene(self.measured_hand(), released_scene, touch=True)
            self.check_hand_sweep(self.state(), self.measured_hand(), self.open_hand, released_scene)
            self.execute_hand(self.open_hand, "open_hand_release")
            if np.max(np.abs(self.measured_hand())) > 0.15:
                raise RuntimeError("Hand did not reopen; object release is not confirmed")
            self.event("verify_release_and_settle")
            stable_since = None

            def settled():
                nonlocal stable_since
                self.scene_unchanged(self.baseline)
                sample = self.latest_feedback()
                ok = sample.supported and not sample.contact_links and sample.linear_speed < 0.02
                if not ok:
                    stable_since = None
                    return False
                if stable_since is None:
                    stable_since = time.monotonic()
                return time.monotonic()-stable_since >= 0.5

            self.wait(settled, self.p("settle_timeout"), "Object did not release and settle on a support")
            final = pose_values(self.latest_feedback().object_pose)[0]
            error = float(np.linalg.norm(final-self.destination))
            self.report["final_object_center_world"] = final.tolist()
            self.report["placement_error_m"] = error
            if error > self.p("position_tolerance"):
                raise RuntimeError(f"Released object is {error:.3f} m from the desired centre")
            self.baseline[self.target] = self.release_scene(final)[self.target]
            self.configure_scene(self.measured_hand(), self.baseline, touch=True)
            retreat = self.flange_pose(self.state())
            retreat.position.z += self.p("pregrasp_height")
            path = self.line(self.state(), retreat)
            self.event("retreat")
            self.record_path("retreat", path)
            self.execute_path(path, self.baseline)
            after_retreat = self.latest_feedback()
            final = pose_values(after_retreat.object_pose)[0]
            error = float(np.linalg.norm(final-self.destination))
            if not after_retreat.supported or after_retreat.contact_links or error > self.p("position_tolerance"):
                raise RuntimeError("Object placement changed during retreat")
            self.report["final_object_center_world"] = final.tolist()
            self.report["placement_error_m"] = error
            self.baseline[self.target] = self.release_scene(final)[self.target]
            self.configure_scene(None, self.baseline)
            self.report["mode"] = "completed"
            self.event("pick_place_complete")
        except (Exception, KeyboardInterrupt) as error:
            self.report["mode"] = "failed"
            self.report["error"] = str(error) or "Interrupted"
            self.save()
            raise
        finally:
            # Planning never leaves a fictitious closed hand/payload in the scene.
            # During an execution failure keep the real grasp context and hand held.
            if (not execute or not self.motion_started) and self.context_dirty and hasattr(self, "baseline"):
                self.configure_scene(None, self.baseline)
