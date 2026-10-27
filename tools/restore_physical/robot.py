"""Thin client for the real LARA5 + Inspire hand stack (controller_manager.launch.py, robot mode).

Every arm motion goes through the safety filter: joint goals through the checked
/arm_trajectory_controller/follow_joint_trajectory, Cartesian goals through /safety/arm/move_to_pose.
After each motion the measured joints are compared with the goal, so a motion that the robot did
not carry out is reported instead of assumed.
"""

from __future__ import annotations

import math
import os
import re
import subprocess
import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from geometry_msgs.msg import Point, Vector3
from lara5_safety_msgs.action import MoveToPose
from lara5_safety_msgs.msg import SceneBox
from lara5_safety_msgs.srv import CheckTrajectory, GetFramePose, PlanCartesianPath, SetSceneBoxes

ARM = [f"lara5_joint{i}" for i in range(1, 7)]
# hand_controller order; 0 = open, larger = closed [rad]
HAND = ["right_thumb_finger_proximal_yaw", "right_index_finger_proximal", "right_middle_finger_proximal",
        "right_ring_finger_proximal", "right_pinky_finger_proximal", "right_thumb_finger_proximal_pitch"]
HAND_OPEN = [0.0] * 6
ROBOT_STATE = "/run_data/robot_state"  # neura/robot_state.cpp, built in the container
NEURA_LIB = "/ws/build/neura_driver/neura_bundle/usr/lib"


def machine_state():
    """Read-only status bits of the arm controller (a second, read-only neura_cpp_api client)."""
    env = dict(os.environ, LD_LIBRARY_PATH=NEURA_LIB)
    out = subprocess.run([ROBOT_STATE], capture_output=True, text=True, timeout=20, env=env).stdout
    bits = dict(re.findall(r"(\w+)/b\d+=(\d)", out))
    return {k: int(v) for k, v in bits.items()}


def pose_to_matrix(p: Pose) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]).as_matrix()
    T[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return T


def matrix_to_pose(T: np.ndarray) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in T[:3, 3])
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (float(v) for v in q)
    return p


def _duration(seconds: float) -> DurationMsg:
    return DurationMsg(sec=int(seconds), nanosec=int((seconds % 1.0) * 1e9))


class MotionError(RuntimeError):
    pass


class Robot(Node):
    def __init__(self, name="restore_robot"):
        super().__init__(name)
        self._lock = threading.Lock()
        self._joints: dict[str, float] = {}
        self._stamp = 0.0
        self.create_subscription(JointState, "/joint_states", self._on_joints, qos_profile_sensor_data)
        self.arm_client = ActionClient(self, FollowJointTrajectory, "/arm_trajectory_controller/follow_joint_trajectory")
        self.hand_client = ActionClient(self, FollowJointTrajectory, "/hand_controller/follow_joint_trajectory")
        self.pose_client = ActionClient(self, MoveToPose, "/safety/arm/move_to_pose")
        self.fk_client = self.create_client(GetFramePose, "/safety/arm/get_frame_pose")
        self.plan_client = self.create_client(PlanCartesianPath, "/safety/arm/plan_cartesian_path")
        self.scene_client = self.create_client(SetSceneBoxes, "/safety/arm/set_scene_boxes")
        self.check_client = self.create_client(CheckTrajectory, "/safety/arm/check_trajectory")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self)
        self._spin = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin.start()

    # ---------- state ----------

    def _on_joints(self, msg):
        with self._lock:
            for n, p in zip(msg.name, msg.position):
                self._joints[n] = p
            self._stamp = time.monotonic()

    def wait_ready(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if all(j in self._joints for j in ARM):
                    break
            time.sleep(0.05)
        else:
            raise MotionError("no /joint_states for the arm")
        for client, name in ((self.arm_client, "arm"), (self.hand_client, "hand"), (self.pose_client, "move_to_pose")):
            if not client.wait_for_server(timeout_sec=timeout):
                raise MotionError(f"{name} action server not available")
        if not self.fk_client.wait_for_service(timeout_sec=timeout):
            raise MotionError("get_frame_pose service not available")

    def arm_joints(self) -> np.ndarray:
        with self._lock:
            if time.monotonic() - self._stamp > 0.5:
                raise MotionError("/joint_states is stale")
            return np.array([self._joints[j] for j in ARM])

    def hand_joints(self) -> np.ndarray:
        with self._lock:
            return np.array([self._joints.get(j, math.nan) for j in HAND])

    def frame_pose(self, frame="lara5_flange", reference="world") -> np.ndarray:
        req = GetFramePose.Request(frame=frame, reference_frame=reference)
        res = self._call(self.fk_client, req, 5.0)
        if not res.success:
            raise MotionError(f"FK {reference}->{frame}: {res.message}")
        return pose_to_matrix(res.pose.pose)

    def tf(self, frame, reference="world", timeout=2.0) -> np.ndarray:
        """Latest TF (includes the measured hand joints, unlike the safety filter's FK)."""
        from rclpy.duration import Duration
        from rclpy.time import Time
        t = self.tf_buffer.lookup_transform(reference, frame, Time(), timeout=Duration(seconds=timeout))
        T = np.eye(4)
        q = t.transform.rotation
        T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        return T

    # ---------- helpers ----------

    def _call(self, client, request, timeout):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise MotionError(f"service {client.srv_name} timed out")
            time.sleep(0.01)
        return future.result()

    def _wait(self, future, timeout, what):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise MotionError(f"{what}: timed out")
            time.sleep(0.01)
        return future.result()

    def _run_action(self, client, goal, timeout, what, feedback=None):
        handle = self._wait(client.send_goal_async(goal, feedback_callback=feedback), 10.0, what + " (send)")
        if not handle.accepted:
            raise MotionError(f"{what}: goal rejected")
        try:
            return self._wait(handle.get_result_async(), timeout, what)
        except MotionError:
            handle.cancel_goal_async()
            raise

    # ---------- arm ----------

    def move_joints(self, target, duration=None, max_speed=0.3, tol=0.01, _retry=True):
        """Joint-space move through the safety filter. duration default: from max_speed [rad/s]."""
        target = np.asarray(target, float)
        start = self.arm_joints()
        if duration is None:
            duration = max(2.0, 1.875 * float(np.max(np.abs(target - start))) / max_speed)
        traj = JointTrajectory(joint_names=ARM)
        traj.points = [JointTrajectoryPoint(positions=[float(v) for v in target], velocities=[0.0] * 6,
                                            time_from_start=_duration(duration))]
        goal = FollowJointTrajectory.Goal(trajectory=traj)
        result = self._run_action(self.arm_client, goal, duration + 20.0, "joint move")
        code = result.result.error_code
        time.sleep(0.3)
        err = float(np.max(np.abs(self.arm_joints() - target)))
        if code == 0 and tol < err < 0.35 and _retry:
            # Frozen short (see move_pose): a second, slow move from where the arm is.
            return self.move_joints(target, max_speed=min(max_speed, 0.15), tol=tol, _retry=False)
        if code != 0 or err > tol:
            raise MotionError(f"joint move failed: code {code} '{result.result.error_string}', "
                              f"final error {err:.4f} rad")
        return err

    def move_pose(self, T_world_flange, duration=0.0, frame="world", tol_m=0.004, speed=0.04):
        """Straight-line Cartesian move of lara5_flange, planned and checked by the safety filter.

        The arm trails its command by ~0.4 s, and the driver drops every command more than 10 deg
        from the measured joints, so a move can end with the arm frozen short of its goal while the
        controller reports success. The line is therefore run slowly (`speed` m/s), and a short
        joint move from wherever the arm stopped to the plan's last waypoint finishes it.
        """
        start = self.frame_pose("lara5_flange", frame)
        dist = float(np.linalg.norm(T_world_flange[:3, 3] - start[:3, 3]))
        duration = max(float(duration), dist / speed, 1.5)
        plan = self.plan_pose(T_world_flange, frame, duration)
        if not plan.success:
            raise MotionError(f"cartesian move: {plan.message}")
        end = np.array(plan.trajectory.points[-1].positions)
        goal = MoveToPose.Goal()
        goal.target = PoseStamped()
        goal.target.header.frame_id = frame
        goal.target.pose = matrix_to_pose(T_world_flange)
        goal.duration = duration
        result = self._run_action(self.pose_client, goal, 120.0 + duration, "cartesian move")
        if not result.result.success:
            raise MotionError(f"cartesian move: {result.result.message}")
        time.sleep(0.3)
        if float(np.max(np.abs(self.arm_joints() - end))) > 0.003:
            self.move_joints(end, max_speed=0.15, tol=0.003)
        reached = self.frame_pose("lara5_flange", frame)
        err = float(np.linalg.norm(reached[:3, 3] - T_world_flange[:3, 3]))
        if err > tol_m:
            raise MotionError(f"cartesian move ended {1000 * err:.1f} mm from the target")
        return result.result

    def check_joints(self, start, target, duration=5.0):
        """(valid, message) for a straight joint move start -> target, checked like a real goal."""
        req = CheckTrajectory.Request()
        req.start_state = JointState(name=ARM, position=[float(v) for v in start])
        req.trajectory = JointTrajectory(joint_names=ARM)
        req.trajectory.points = [JointTrajectoryPoint(positions=[float(v) for v in target], velocities=[0.0] * 6,
                                                      time_from_start=_duration(duration))]
        res = self._call(self.check_client, req, 30.0)
        return res.valid, res.message

    def plan_pose(self, T_world_flange, frame="world", duration=0.0, start=None):
        req = PlanCartesianPath.Request()
        if start is not None:
            req.start_state = JointState(name=ARM, position=[float(v) for v in start])
        req.target = PoseStamped()
        req.target.header.frame_id = frame
        req.target.pose = matrix_to_pose(T_world_flange)
        req.duration = float(duration)
        res = self._call(self.plan_client, req, 30.0)
        return res

    def set_scene_boxes(self, boxes):
        """Replace the perception obstacles: [(name, centre xyz, size xyz)] in world."""
        req = SetSceneBoxes.Request(frame_id="world")
        req.boxes = [SceneBox(name=n, center=Point(x=float(c[0]), y=float(c[1]), z=float(c[2])),
                              size=Vector3(x=float(z[0]), y=float(z[1]), z=float(z[2]))) for n, c, z in boxes]
        res = self._call(self.scene_client, req, 10.0)
        if not res.success:
            raise MotionError(f"set_scene_boxes: {res.message}")

    # ---------- hand ----------

    def hand(self, angles, duration=1.5):
        traj = JointTrajectory(joint_names=HAND)
        traj.points = [JointTrajectoryPoint(positions=[float(v) for v in angles], time_from_start=_duration(duration))]
        goal = FollowJointTrajectory.Goal(trajectory=traj)
        self._run_action(self.hand_client, goal, duration + 10.0, "hand move")
        time.sleep(0.3)
        return self.hand_joints()

    def close(self):
        self._executor.shutdown(timeout_sec=2.0)
        self._spin.join(timeout=2.0)
        self.destroy_node()
        rclpy.try_shutdown()


def start(name="restore_robot") -> Robot:
    if not rclpy.ok():
        rclpy.init()
    robot = Robot(name)
    robot.wait_ready()
    return robot
