"""A fake robot and table for the orchestrator's ROS tests.

It serves what the real system serves, with the same names and types:

    /perception/capture_scene              objects as perception reports them (yaw modulo 180 degrees)
    /safety/arm/move_to_pose               the flange jumps to the target; a held object moves with it
    /arm_trajectory_controller/follow_joint_trajectory
    /hand_controller/follow_joint_trajectory   fingers stop where an object under the palm blocks them
    /arm/approach/target, /arm/approach/status like lara5_object_approach (0.40 m above, palm down)
    /joint_states, TF world -> lara5_flange -> right_hand_base_link

Knobs: reject_moves (refuse the next n MoveToPose goals), miss_grasps (the next n closes miss).
"""

from __future__ import annotations

import json
import math
import threading
from dataclasses import dataclass

import numpy as np
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, TransformStamped
from geometry_msgs.msg import PoseStamped
from lara5_safety_msgs.action import MoveToPose
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from restore_interfaces.msg import ObjectState, SceneSnapshot
from restore_interfaces.srv import CaptureScene
from restore_perception.core.types import SYM_CONTINUOUS
from restore_task.clients import ARM_JOINTS
from restore_task.core import hand
from restore_task.core.config import primitive_for
from restore_task.core.poses import from_xyz_quat, from_xyz_rpy, inverse, rot_z, to_xyz_quat, transform, world_yaw

TABLE_Z = 0.90
T_FLANGE_TCP = from_xyz_rpy([0, 0, 0], [0, 0, -math.pi / 2])  # as in the URDF
T_OBSERVE = transform(from_xyz_rpy([0, 0, 0], [math.pi, 0, 0])[:3, :3], (0.0, 0.0, 1.5))
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)

KINDS = {  # label: size [m], symmetry
    "wooden_cube": ([0.055, 0.055, 0.055], 4),
    "sponge": ([0.090, 0.060, 0.045], 2),
    "tuna_can": ([0.085, 0.085, 0.040], SYM_CONTINUOUS),
    "soup_can": ([0.110, 0.073, 0.073], 2),
}


@dataclass
class Thing:
    label: str
    x: float
    y: float
    yaw: float = 0.0  # true yaw [rad]

    @property
    def size(self):
        return np.array(KINDS[self.label][0])

    @property
    def z(self):
        return TABLE_Z + self.size[2] / 2.0

    def matrix(self):
        return transform(rot_z(self.yaw), (self.x, self.y, self.z))


class FakeRobot(Node):
    def __init__(self, things, objects, cfg):
        super().__init__("fake_robot")
        self.things, self.objects, self.cfg = things, objects, cfg
        self.lock = threading.Lock()
        self.flange = T_OBSERVE.copy()
        self.arm = np.full(6, 0.3)
        self.hand = np.zeros(6)
        self.holding, self.T_flange_thing = None, None
        self.reject_moves = self.miss_grasps = 0
        self.moves, self.approaches, self.captures = 0, 0, 0
        self.log = []  # grasp and release events, for test messages
        group = ReentrantCallbackGroup()
        self.create_service(CaptureScene, "/perception/capture_scene", self.on_capture, callback_group=group)
        ActionServer(self, MoveToPose, "/safety/arm/move_to_pose", self.on_move, callback_group=group)
        ActionServer(self, FollowJointTrajectory, "/arm_trajectory_controller/follow_joint_trajectory", self.on_arm,
                     callback_group=group)
        ActionServer(self, FollowJointTrajectory, "/hand_controller/follow_joint_trajectory", self.on_hand,
                     callback_group=group)
        self.create_subscription(PoseStamped, "/arm/approach/target", self.on_approach, QoSProfile(depth=1),
                                 callback_group=group)
        self.approach_status = self.create_publisher(String, "/arm/approach/status", LATCHED)
        self.approach_status.publish(String(data=json.dumps({"state": "idle"})))
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.tf = TransformBroadcaster(self)
        static = StaticTransformBroadcaster(self)
        static.sendTransform(self._tf("lara5_flange", "right_hand_base_link", T_FLANGE_TCP))
        self.create_timer(0.02, self.tick, callback_group=group)

    def _tf(self, parent, child, T):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = parent, child
        xyz, q = to_xyz_quat(T)
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (float(v) for v in xyz)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = (
            float(v) for v in q)
        return t

    def tick(self):
        with self.lock:
            msg = JointState(name=ARM_JOINTS + hand.HAND_JOINTS, position=[float(v) for v in [*self.arm, *self.hand]])
            flange = self.flange.copy()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(msg)
        self.tf.sendTransform(self._tf("world", "lara5_flange", flange))

    # ---------- perception ----------

    def on_capture(self, request, response):
        with self.lock:
            self.captures += 1
            things = sorted(self.things, key=lambda t: (round(t.x, 3), round(t.y, 3)))
            snap = SceneSnapshot(snapshot_id=request.name or f"capture_{self.captures}",
                                 table_plane=[0.0, 0.0, 1.0, -TABLE_Z])
            snap.header.frame_id = "world"
            for i, t in enumerate(things):
                size, symmetry = KINDS[t.label]
                reported = (t.yaw + math.pi / 2) % math.pi - math.pi / 2  # footprint long side: modulo 180
                corners = np.array([[1, 1], [-1, 1], [-1, -1], [1, -1]]) * np.array(size[:2]) / 2
                corners = corners @ rot_z(t.yaw)[:2, :2].T + [t.x, t.y]
                o = ObjectState(id=f"obj_{i:02d}", label=t.label, label_score=0.9, symmetry=symmetry,
                                support_id="table", cameras_seen=3, depth_reliable=True, confidence=0.9)
                o.pose.position.x, o.pose.position.y, o.pose.position.z = t.x, t.y, float(t.z)
                o.pose.orientation.z, o.pose.orientation.w = math.sin(reported / 2), math.cos(reported / 2)
                o.size.x, o.size.y, o.size.z = (float(v) for v in size)
                o.footprint = [Point(x=float(c[0]), y=float(c[1]), z=TABLE_Z) for c in corners]
                embedding = np.zeros(8, np.float32)
                embedding[list(KINDS).index(t.label)] = 1.0
                o.embedding = [float(v) for v in embedding]
                snap.objects.append(o)
        response.success, response.message, response.snapshot = True, f"{len(things)} objects", snap
        return response

    # ---------- arm ----------

    def _set_flange(self, T):
        self.flange = T
        if self.holding is not None:
            moved = T @ self.T_flange_thing
            self.holding.x, self.holding.y = float(moved[0, 3]), float(moved[1, 3])
            self.holding.yaw = world_yaw(moved[:3, :3])

    def on_move(self, handle):
        target = handle.request.target.pose
        T = from_xyz_quat([target.position.x, target.position.y, target.position.z],
                          [target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w])
        handle.succeed()
        with self.lock:
            if self.reject_moves > 0:
                self.reject_moves -= 1
                return MoveToPose.Result(success=False, message="no IK solution (fake)")
            if abs(T[0, 3]) > 0.8 or abs(T[1, 3]) > 0.6 or T[2, 3] < TABLE_Z:
                return MoveToPose.Result(success=False, message="out of reach (fake)")
            self.moves += 1
            self._set_flange(T)
            self.log.append(f"move to ({T[0, 3]:.3f}, {T[1, 3]:.3f}, {T[2, 3]:.3f})")
        return MoveToPose.Result(success=True, message="done")

    def on_arm(self, handle):
        positions = np.array(handle.request.trajectory.points[-1].positions)
        handle.succeed()
        with self.lock:
            self.arm = positions
            observe = np.array([self.cfg["observe_joints"][j] for j in ARM_JOINTS])
            if np.allclose(positions, observe, atol=1e-6):
                self._set_flange(T_OBSERVE.copy())
                self.log.append("observe pose")
        return FollowJointTrajectory.Result(error_code=0)

    def on_approach(self, msg):
        q = msg.pose.orientation
        with self.lock:
            self.approaches += 1
            if abs(q.x) > 1e-3 or abs(q.y) > 1e-3:  # as lara5_object_approach
                state = "rejected_invalid"
            else:
                T_object = from_xyz_quat([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
                                         [q.x, q.y, q.z, q.w])
                self._set_flange(T_object @ from_xyz_rpy([0, 0, 0.40], [math.pi, 0, 0]))
                state = "succeeded"
        self.approach_status.publish(String(data=json.dumps({"state": state, "message": "fake"})))

    # ---------- hand ----------

    def on_hand(self, handle):
        trajectory = handle.request.trajectory
        named = dict(zip(trajectory.joint_names, trajectory.points[-1].positions))
        target = np.array([named[j] for j in hand.HAND_JOINTS])
        handle.succeed()
        with self.lock:
            closing = target[2] > self.hand[2] + 0.05
            opening = target[2] < self.hand[2] - 0.05
            if closing and self.holding is None:
                tcp = self.flange @ T_FLANGE_TCP
                # Only an object within finger reach is caught: the palm 10 to 20 cm above its top
                # (task.yaml's palm heights are 13 to 16 cm). A pre-shape higher up catches nothing.
                under = [t for t in self.things if math.hypot(tcp[0, 3] - t.x, tcp[1, 3] - t.y) < 0.01
                         and 0.10 < tcp[2, 3] - (t.z + t.size[2] / 2) < 0.20]
                if under and self.miss_grasps > 0:
                    self.miss_grasps -= 1
                    under = []
                if under:
                    thing = under[0]
                    _, primitive = primitive_for(thing.label, self.objects, self.cfg)
                    closing_axis = tcp[:3, 1]
                    along = np.abs(thing.matrix()[:3, :2].T @ closing_axis)
                    width = float(along[0] * thing.size[0] + along[1] * thing.size[1])
                    contact = hand.angles_for_width(primitive, width)
                    stopped = target.copy()
                    for finger in primitive["fingers"]:
                        j = hand.CLOSING_JOINT[finger]
                        stopped[j] = min(target[j], contact[j])
                    target = stopped
                    self.holding, self.T_flange_thing = thing, inverse(self.flange) @ thing.matrix()
                    self.log.append(f"grasp {thing.label} at ({thing.x:.3f}, {thing.y:.3f}) width {width:.3f}")
                else:
                    self.log.append(f"close on nothing at ({tcp[0, 3]:.3f}, {tcp[1, 3]:.3f}, {tcp[2, 3]:.3f})")
            elif opening and self.holding is not None:
                t = self.holding
                self.log.append(f"release {t.label} at ({t.x:.3f}, {t.y:.3f}) yaw {math.degrees(t.yaw):.1f}")
                self.holding = None  # it drops onto the table where it is: Thing.z is always on the table
            elif self.holding is not None:
                target = np.minimum(target, self.hand)  # the object keeps the fingers where they are
            self.hand = target
        return FollowJointTrajectory.Result(error_code=0)
