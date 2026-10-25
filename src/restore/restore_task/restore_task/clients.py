"""Blocking ROS calls for the state machine: perception, the arm, the hand, the approach node, TF.

The state machine runs in the Restore action's thread; replies arrive on the executor's other
threads (a reentrant callback group), so waiting on a future here does not block them.
"""

from __future__ import annotations

import json
import math
import threading
import time

import numpy as np
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from lara5_safety_msgs.action import MoveToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration as RclDuration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from restore_interfaces.srv import CaptureScene

from .core.hand import HAND_JOINTS
from .core.poses import from_xyz_quat, from_xyz_rpy, to_xyz_quat

ARM_JOINTS = [f"lara5_joint{i}" for i in range(1, 7)]
APPROACH_DONE = {"succeeded", "failed", "rejected_invalid", "rejected_busy", "unavailable", "canceled", "error"}


class TaskError(RuntimeError):
    """Something the run cannot continue from."""


class MotionRejected(TaskError):
    """The safety filter or the approach node refused or failed a motion: try something else."""


class TaskCancelled(TaskError):
    pass


def pose_stamped(T, frame):
    msg = PoseStamped()
    msg.header.frame_id = frame
    xyz, q = to_xyz_quat(T)
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in xyz)
    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = (
        float(v) for v in q)
    return msg


def _duration(seconds):
    nanosec = int(round(seconds * 1e9))
    return Duration(sec=nanosec // 10**9, nanosec=nanosec % 10**9)


class Robot:
    def __init__(self, node, cfg, group):
        self.node, self.cfg = node, cfg
        self.world, self.flange, self.tcp = cfg["world_frame"], cfg["flange_frame"], cfg["tcp_frame"]
        self.cancelled = threading.Event()
        self._active = None  # goal handle of the running action, for cancellation
        self._lock = threading.Lock()
        self._joints = {}
        self.capture_client = node.create_client(CaptureScene, "/perception/capture_scene", callback_group=group)
        self.move_client = ActionClient(node, MoveToPose, "/safety/arm/move_to_pose", callback_group=group)
        self.arm_client = ActionClient(node, FollowJointTrajectory, "/arm_trajectory_controller/follow_joint_trajectory",
                                       callback_group=group)
        self.hand_client = ActionClient(node, FollowJointTrajectory, "/hand_controller/follow_joint_trajectory",
                                        callback_group=group)
        node.create_subscription(JointState, "/joint_states", self._on_joints, qos_profile_sensor_data,
                                 callback_group=group)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.use_approach = cfg["pick_place"]["approach_mode"] == "approach_node"
        self._approach_events = []
        if self.use_approach:
            # lara5_object_approach: volatile command topic, retained status topic.
            self.approach_pub = node.create_publisher(PoseStamped, "/arm/approach/target", QoSProfile(depth=1))
            node.create_subscription(
                String, "/arm/approach/status", self._on_approach,
                QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE),
                callback_group=group)

    # ---------- plumbing ----------

    def _on_joints(self, msg):
        with self._lock:
            self._joints.update(zip(msg.name, msg.position))

    def _on_approach(self, msg):
        with self._lock:
            self._approach_events.append((time.monotonic(), json.loads(msg.data)))

    def _wait(self, future, timeout, what):
        deadline = time.monotonic() + timeout
        while not future.done():
            if self.cancelled.is_set():
                raise TaskCancelled("cancelled")
            if time.monotonic() > deadline:
                raise TaskError(f"{what}: no answer within {timeout:.0f} s")
            time.sleep(0.01)
        return future.result()

    def _send(self, client, goal, timeout, what):
        handle = self._wait(client.send_goal_async(goal), 10.0, what)
        if handle is None or not handle.accepted:
            raise MotionRejected(f"{what}: goal rejected")
        self._active = handle
        try:
            return self._wait(handle.get_result_async(), timeout, what)
        finally:
            self._active = None

    def cancel(self):
        self.cancelled.set()
        handle = self._active
        if handle is not None:
            handle.cancel_goal_async()

    def missing(self):
        """Interfaces that are not up yet."""
        out = []
        if not self.capture_client.service_is_ready():
            out.append("/perception/capture_scene")
        for client, name in ((self.move_client, "/safety/arm/move_to_pose"),
                             (self.arm_client, "arm follow_joint_trajectory"),
                             (self.hand_client, "hand follow_joint_trajectory")):
            if not client.server_is_ready():
                out.append(name)
        if self.use_approach and self.approach_pub.get_subscription_count() == 0:
            out.append("/arm/approach/target (lara5_object_approach)")
        with self._lock:
            if not all(j in self._joints for j in ARM_JOINTS + HAND_JOINTS):
                out.append("/joint_states")
        return out

    # ---------- perception ----------

    def capture(self, name, save):
        request = CaptureScene.Request(name=name, save_to_disk=save)
        response = self._wait(self.capture_client.call_async(request), 120.0, "capture")
        if not response.success:
            raise TaskError(f"capture failed: {response.message}")
        return response.snapshot

    # ---------- arm ----------

    def move_to(self, T_world_flange, duration=0.0, what="move"):
        goal = MoveToPose.Goal(target=pose_stamped(T_world_flange, self.world), relative=False,
                               duration=float(duration))
        result = self._send(self.move_client, goal, 180.0, what)
        if result.status != GoalStatus.STATUS_SUCCEEDED or not result.result.success:
            raise MotionRejected(f"{what}: {result.result.message or 'did not succeed'}")

    def _trajectory(self, client, names, positions, seconds, what):
        point = JointTrajectoryPoint(positions=[float(p) for p in positions], time_from_start=_duration(seconds))
        goal = FollowJointTrajectory.Goal(trajectory=JointTrajectory(joint_names=list(names), points=[point]))
        result = self._send(client, goal, seconds + 30.0, what)
        if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != 0:
            raise MotionRejected(f"{what}: {result.result.error_string or 'did not succeed'}")

    def joint_positions(self, names):
        with self._lock:
            return np.array([self._joints.get(n, math.nan) for n in names])

    def at_observe(self):
        target = self.cfg["observe_joints"]
        now = self.joint_positions(list(target))
        return bool(np.all(np.abs(now - np.array(list(target.values()))) <= self.cfg["observe_tolerance_rad"]))

    def observe(self):
        """Joint-space move to the observe pose, through the safety filter."""
        if self.at_observe():
            return
        target = self.cfg["observe_joints"]
        now = self.joint_positions(list(target))
        travel = np.nanmax(np.abs(np.array(list(target.values())) - now)) if np.isfinite(now).any() else 1.0
        self._trajectory(self.arm_client, list(target), list(target.values()),
                         max(2.0, float(travel) / self.cfg["joint_speed_rad_s"]), "observe pose")

    def approach(self, T_world_object):
        """lara5_object_approach: the flange to its configured offset above this object pose."""
        start = time.monotonic()
        self.approach_pub.publish(pose_stamped(T_world_object, self.world))
        deadline = start + 180.0
        while True:
            if self.cancelled.is_set():
                raise TaskCancelled("cancelled")
            with self._lock:
                event = next((e for t, e in self._approach_events if t >= start and e.get("state") in APPROACH_DONE),
                             None)
            if event is not None:
                if event["state"] != "succeeded":
                    raise MotionRejected(f"approach {event['state']}: {event.get('message', '')}")
                return
            if time.monotonic() > deadline:
                raise TaskError("approach: no result within 180 s")
            time.sleep(0.02)

    # ---------- hand ----------

    def hand_to(self, positions, seconds, what="hand"):
        self._trajectory(self.hand_client, HAND_JOINTS, positions, seconds, what)

    def hand_positions(self, settle):
        """Finger angles once the fingers have come to rest (the hand controller is open loop)."""
        time.sleep(settle)
        return self.joint_positions(HAND_JOINTS)

    # ---------- TF ----------

    def _lookup(self, parent, child):
        try:
            t = self.tf_buffer.lookup_transform(parent, child, Time(), timeout=RclDuration(seconds=0.5))
        except Exception:
            return None
        r, p = t.transform.rotation, t.transform.translation
        return from_xyz_quat([p.x, p.y, p.z], [r.x, r.y, r.z, r.w])

    def flange_pose(self):
        return self._lookup(self.world, self.flange)

    def flange_to_tcp(self):
        T = self._lookup(self.flange, self.tcp)
        if T is None:
            fallback = self.cfg["flange_to_tcp"]
            T = from_xyz_rpy(fallback["xyz"], fallback["rpy"])
        return T
