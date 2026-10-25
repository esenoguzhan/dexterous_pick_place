"""Independent object-pose command: two-phase positioning with an open hand."""

import signal
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from visualization_msgs.msg import Marker, MarkerArray

from .execution import TrajectoryExecutor
from .grasp_setup import configure_grasp, target_geometry
from .visualization import make_grasp_markers


class ObjectApproach(Node):
    def __init__(self, **kwargs):
        super().__init__("object_approach", **kwargs)
        description, flange, names, positions = configure_grasp(self)
        self.radius = self.declare_parameter("cylinder_radius", .03).value
        self.height = self.declare_parameter("cylinder_height", .10).value
        self.table = self.declare_parameter("table_height", .90).value
        if not np.isfinite([self.radius, self.height, self.table]).all() or min(self.radius, self.height) <= 0:
            raise ValueError("cylinder/table geometry must be valid")
        self.markers = self.create_publisher(MarkerArray, "/arm/approach/preview/markers",
                                            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.planner = TrajectoryExecutor(self, description, flange, names, positions)
        # Explicit volatile commands cannot replay a retained pose on startup.
        self.command = self.create_subscription(
            PoseStamped, "/arm/approach/target", self.on_target, QoSProfile(depth=1))

    @property
    def active_request(self):
        return self.planner.active_request

    def on_target(self, message):
        if self.planner.busy:
            self.planner.report("rejected_busy", "Approach active or stopping; new target discarded")
            return
        try:
            obj, flange, grasp, hand = target_geometry(self, message)
        except (ValueError, OverflowError) as error:
            self.planner.reject_input(error)
            self.markers.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
            return
        self.markers.publish(make_grasp_markers(self, obj, flange, grasp, hand, self.get_clock().now().to_msg()))
        self.planner.request(message, obj, flange, grasp)

    def cancel_active(self):
        self.planner.cancel_active()


def main():
    # Keep ROS alive while Ctrl+C cancellation awaits a terminal action result.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    node = None
    try:
        node = ObjectApproach()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.cancel_active()
            deadline = time.monotonic()+node.planner.cancel_timeout
            while rclpy.ok() and node.active_request is not None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
            if node.active_request is not None:
                node.get_logger().error("Controller stop not confirmed before exit; check the arm backend")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
