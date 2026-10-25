"""Close the hand once per topic command; never command the arm."""

import json
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Empty, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .configuration import DEFAULT_CLOSE, HAND_JOINTS, validate_closure


class HandGrasp(Node):
    def __init__(self, **kwargs):
        super().__init__("hand_grasp", **kwargs)
        positions = self.declare_parameter("close_positions", list(DEFAULT_CLOSE)).value
        duration = self.declare_parameter("duration", 3.0).value
        self.positions, self.duration_ns = validate_closure(positions, duration)
        self.active_request = None
        self.goal_handle = None
        self.stopping = False
        self.cancel_sent = False
        # Resolve explicitly because Humble does not remap the action root here.
        action_name = self.resolve_topic_name("/hand_controller/follow_joint_trajectory")
        self.hand = ActionClient(self, FollowJointTrajectory, action_name)
        self.status = self.create_publisher(
            String, "/arm/grasp/status",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # Never replay a retained command when this task starts.
        self.command = self.create_subscription(
            Empty, "/arm/grasp/target", self.on_trigger, QoSProfile(depth=1))
        self.report("idle", "Waiting for a hand-closure command")

    def report(self, state, message, request_id=None):
        event = {"state": state, "message": message, "request_id": request_id,
                 "active_request_id": self.active_request}
        self.status.publish(String(data=json.dumps(event)))
        self.get_logger().info(f"{state}: {message}")

    def on_trigger(self, _message):
        request_id = str(uuid.uuid4())
        if self.stopping or self.active_request is not None:
            self.report("rejected_busy", "Hand closure active or node stopping; command discarded", request_id)
            return
        if not self.hand.server_is_ready():
            self.report("unavailable", "Hand controller action unavailable; command discarded", request_id)
            return

        # The controller starts at its current hand state. This trajectory contains
        # only the six hand joints; no arm, scene, lift or release commands exist.
        point = JointTrajectoryPoint(
            positions=list(self.positions), velocities=[0.0] * len(HAND_JOINTS),
            time_from_start=Duration(sec=self.duration_ns // 1_000_000_000,
                                     nanosec=self.duration_ns % 1_000_000_000))
        goal = FollowJointTrajectory.Goal(
            trajectory=JointTrajectory(joint_names=list(HAND_JOINTS), points=[point]),
            goal_time_tolerance=Duration(sec=2))
        self.active_request = request_id
        self.report("requested", "Requesting hand closure", request_id)
        try:
            self.hand.send_goal_async(goal).add_done_callback(self.on_goal_response)
        except Exception as error:
            self.finish("failed", f"Could not send hand closure: {error}")

    def on_goal_response(self, future):
        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                self.finish("failed", "Hand controller rejected closure")
                return
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(self.on_result)
            self.report("executing", "Closing hand", self.active_request)
            if self.stopping:
                self.cancel_active()
        except Exception as error:
            self.report("error", f"Cannot obtain goal response: {error}", self.active_request)

    def on_result(self, future):
        try:
            outcome = future.result()
            result = outcome.result
            if (outcome.status == GoalStatus.STATUS_SUCCEEDED and
                    result.error_code == FollowJointTrajectory.Result.SUCCESSFUL):
                self.finish("succeeded", "Hand closure motion completed; object retention not verified")
            elif outcome.status == GoalStatus.STATUS_CANCELED:
                self.finish("canceled", result.error_string or "Hand closure canceled")
            else:
                self.finish("failed", f"Hand closure failed ({result.error_code}): {result.error_string}")
        except Exception as error:
            # Unknown motion state keeps this task busy until the backend is checked.
            self.report("error", f"Cannot obtain closure result: {error}", self.active_request)

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
            self.report("canceling", "Requested hand motion cancellation", self.active_request)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = HandGrasp()
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
                node.get_logger().error("Hand stop not confirmed before exit; check the hand controller")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
