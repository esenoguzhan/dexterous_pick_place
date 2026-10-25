"""Static overlays with optional read-only trajectory planning; no motion clients."""

import json

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .grasp_geometry import pose_components
from .grasp_setup import configure_grasp, target_geometry


def ros_pose(matrix):
    position, quaternion = pose_components(matrix)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, position)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, quaternion)
    return pose


class GraspPreview(Node):
    def __init__(self, **kwargs):
        super().__init__("grasp_preview", **kwargs)
        def param(name, default):
            return self.declare_parameter(name, default).value

        description, flange, names, positions = configure_grasp(self)
        self.radius = param("cylinder_radius", 0.03)
        self.height = param("cylinder_height", 0.10)
        self.table = param("table_height", 0.90)
        if (not self.world_frame or not np.isfinite([self.radius, self.height, self.table]).all()
                or self.radius <= 0 or self.height <= 0):
            raise ValueError("world frame and cylinder/table geometry must be valid")
        self.planner = None
        if self.declare_parameter("plan_trajectories", False).value:
            from .planning import TrajectoryPreview
            self.planner = TrajectoryPreview(self, description, flange, names, positions)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers = self.create_publisher(MarkerArray, "/arm/approach/preview/markers", retained)
        self.flange = self.create_publisher(PoseStamped, "/arm/approach/preview/flange_pose", retained)
        self.grasp = self.create_publisher(PoseStamped, "/arm/approach/preview/grasp_pose", retained)
        self.status = self.create_publisher(String, "/arm/approach/preview/status", retained)
        self.command = self.create_subscription(
            PoseStamped, "/arm/approach/preview/target", self.on_target, 1)
        self.report("idle", "Waiting for manual pose; planning preview enabled" if self.planner
                    else "Waiting for manual pose; static preview only")

    def report(self, state, message, **values):
        self.status.publish(String(data=json.dumps(dict(state=state, message=message, **values))))
        self.get_logger().info(f"{state}: {message}")

    def on_target(self, message):
        if self.planner is not None and self.planner.busy:
            self.planner.report("rejected_busy", "A plan is active; new target discarded")
            return
        try:
            world_object, world_flange, world_grasp, world_hand = target_geometry(self, message)
            position = world_object[:3, 3]
            stamp = self.get_clock().now().to_msg()
            array = self.make_markers(world_object, world_flange, world_grasp, world_hand, stamp)
        except (ValueError, OverflowError) as error:
            self.markers.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
            self.report("rejected_invalid", str(error))
            if self.planner is not None:
                self.planner.reject_input(error)
            return
        self.markers.publish(array)
        for publisher, matrix in ((self.flange, world_flange), (self.grasp, world_grasp)):
            output = PoseStamped()
            output.header.frame_id = self.world_frame
            output.header.stamp = stamp
            output.pose = ros_pose(matrix)
            publisher.publish(output)
        flange_xyz, flange_q = pose_components(world_flange)
        self.report("preview_ready", "Geometry only; reachability, trajectory and grasp are unverified",
                    flange_xyz=list(flange_xyz), flange_xyzw=list(flange_q),
                    cylinder_bottom_minus_table_m=float(position[2]-self.height/2-self.table))
        if self.planner is not None:
            self.planner.request(message, world_object, world_flange, world_grasp)

    def make_markers(self, obj, flange, grasp, hand, stamp):
        from .visualization import make_grasp_markers
        return make_grasp_markers(self, obj, flange, grasp, hand, stamp)


def main():
    rclpy.init()
    node = None
    try:
        node = GraspPreview()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
