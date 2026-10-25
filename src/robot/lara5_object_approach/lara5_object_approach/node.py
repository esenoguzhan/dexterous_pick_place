"""One pose command, one direct approach, no perception feedback or grasping."""

import copy
import json
import math
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lara5_safety_msgs.action import MoveToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from .geometry import finite_vector, object_to_flange, quaternion_from_rpy


class ObjectApproach(Node):
    def __init__(self, **kwargs):
        super().__init__("object_approach", **kwargs)
        self.world_frame = self.declare_parameter("world_frame", "world").value
        self.offset = finite_vector(self.declare_parameter(
            "object_to_flange_xyz", [0.0, 0.0, 0.20]).value, 3, "object_to_flange_xyz")
        self.rpy = finite_vector(self.declare_parameter(
            "object_to_flange_rpy", [-math.pi/2, math.pi/2, 0.0]).value, 3, "object_to_flange_rpy")
        quaternion_from_rpy(self.rpy)
        self.duration = self.declare_parameter("duration", 0.0).value
        if not self.world_frame or not math.isfinite(self.duration) or self.duration < 0:
            raise ValueError("world_frame must be nonempty and duration finite and nonnegative")

        self.active_request = None
        self.goal_handle = None
        self.stopping = False
        self.cancel_sent = False
        # Humble's ActionClient does not apply a remap of the action's root name.
        # Resolve it explicitly so deployments and isolated tests use the same backend.
        motion_action = self.resolve_topic_name("/safety/arm/move_to_pose")
        self.motion = ActionClient(self, MoveToPose, motion_action)
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, "/arm/approach/status", status_qos)
        # Commands are deliberately volatile: node startup cannot replay a latched pose.
        self.command = self.create_subscription(
            PoseStamped, "/arm/approach/target", self.on_target, QoSProfile(depth=1))
        self.report("idle", "Waiting for a manually supplied object pose")

    def report(self, state, message, request_id=None):
        event = {"state": state, "message": message, "request_id": request_id,
                 "active_request_id": self.active_request}
        self.status.publish(String(data=json.dumps(event)))
        self.get_logger().info(f"{state}: {message}")

    def on_target(self, object_pose):
        request_id = str(uuid.uuid4())
        if self.stopping or self.active_request is not None:
            self.report("rejected_busy", "Approach active or node stopping; command discarded", request_id)
            return
        try:
            if object_pose.header.frame_id != self.world_frame:
                raise ValueError(f"object pose must be in '{self.world_frame}'")
            p, q = object_pose.pose.position, object_pose.pose.orientation
            position, orientation = object_to_flange(
                (p.x, p.y, p.z), (q.x, q.y, q.z, q.w), self.offset, self.rpy)
        except (ValueError, OverflowError) as error:
            self.report("rejected_invalid", str(error), request_id)
            return
        if not self.motion.server_is_ready():
            self.report("unavailable", "Native MoveToPose action unavailable; command discarded", request_id)
            return

        target = copy.deepcopy(object_pose)
        target.pose.position.x, target.pose.position.y, target.pose.position.z = position
        q = target.pose.orientation
        q.x, q.y, q.z, q.w = orientation
        goal = MoveToPose.Goal(target=target, relative=False, duration=self.duration)
        self.active_request = request_id
        self.report("planning", "Requesting direct Cartesian approach", request_id)
        try:
            future = self.motion.send_goal_async(goal)
            future.add_done_callback(self.on_goal_response)
        except Exception as error:
            self.finish("failed", f"Could not send approach: {error}")

    def on_goal_response(self, future):
        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                self.finish("failed", "Native motion backend rejected the approach")
                return
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(self.on_result)
            self.report("executing", "Native motion accepted; waiting for completion", self.active_request)
            if self.stopping:
                self.cancel_active()
        except Exception as error:
            # Keep ownership if acceptance is unknown; do not allow another motion.
            self.report("error", f"Cannot obtain goal response: {error}", self.active_request)

    def on_result(self, future):
        try:
            outcome = future.result()
            if outcome.status == GoalStatus.STATUS_SUCCEEDED and outcome.result.success:
                self.finish("succeeded", outcome.result.message or "Approach complete")
            elif outcome.status == GoalStatus.STATUS_CANCELED:
                self.finish("canceled", outcome.result.message or "Approach canceled")
            else:
                self.finish("failed", outcome.result.message or "Native motion did not succeed")
        except Exception as error:
            # A transport error does not prove that physical motion has stopped.
            self.report("error", f"Cannot obtain motion result: {error}", self.active_request)

    def finish(self, state, message):
        request_id = self.active_request
        self.active_request = None
        self.goal_handle = None
        self.cancel_sent = False
        self.report(state, message, request_id)

    def cancel_active(self):
        self.stopping = True
        if self.goal_handle is not None and not self.cancel_sent:
            self.cancel_sent = True
            self.goal_handle.cancel_goal_async()
            self.report("canceling", "Requested native motion cancellation", self.active_request)


def main():
    # Leave ROS alive on Ctrl+C so an accepted action can be canceled before exit.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = ObjectApproach()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.cancel_active()
            deadline = time.monotonic() + 5.0
            while rclpy.ok() and node.active_request is not None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            if node.active_request is not None:
                node.get_logger().error("Motion stop not confirmed before exit; check the native backend")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
