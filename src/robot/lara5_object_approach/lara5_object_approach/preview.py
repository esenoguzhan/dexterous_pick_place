"""Static overlays with optional read-only trajectory planning; no motion clients."""

import json
from pathlib import Path

import numpy as np
import rclpy
import xacro
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .geometry import object_to_flange
from .grasp_geometry import HandModel, from_rpy, grasp_targets, pose_components, transform


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

        self.world_frame = param("world_frame", "world")
        flange = param("flange_link", "lara5_flange")
        hand = param("hand_link", "right_hand_base_link")
        names = param("open_joint_names", ["right_thumb_finger_proximal_yaw",
                      "right_index_finger_proximal", "right_middle_finger_proximal",
                      "right_ring_finger_proximal", "right_pinky_finger_proximal",
                      "right_thumb_finger_proximal_pitch"])
        positions = param("open_joint_positions", [0.0]*6)
        self.hand_grasp = from_rpy(param("hand_to_grasp_xyz", [-0.025, 0.085, 0.15]),
                                   param("hand_to_grasp_rpy", [-np.pi/2, 0.0, 0.0]))
        self.object_grasp = from_rpy(param("object_to_grasp_xyz", [0.0]*3),
                                     param("object_to_grasp_rpy", [np.pi, 0.0, 0.0]))
        self.radius = param("cylinder_radius", 0.03)
        self.height = param("cylinder_height", 0.10)
        self.table = param("table_height", 0.90)
        if (not self.world_frame or not np.isfinite([self.radius, self.height, self.table]).all()
                or self.radius <= 0 or self.height <= 0):
            raise ValueError("world frame and cylinder/table geometry must be valid")
        description = param("robot_description", "")
        if not description:
            path = (Path(get_package_share_directory("lara5_inspire_hand_description"))
                    / "urdf/lara5_inspire_hand.urdf.xacro")
            description = xacro.process_file(str(path)).toxml()
        self.model = HandModel(description, flange, hand, names, positions)
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
            if message.header.frame_id != self.world_frame:
                raise ValueError(f"object pose must be in '{self.world_frame}'")
            p, q = message.pose.position, message.pose.orientation
            # Match the existing approach input convention, including yaw-only checks.
            position, quaternion = object_to_flange(
                [p.x, p.y, p.z], [q.x, q.y, q.z, q.w], [0, 0, 0], [0, 0, 0])
            world_object = transform(position, quaternion)
            world_flange, world_grasp, world_hand = grasp_targets(
                world_object, self.model.flange_hand, self.hand_grasp, self.object_grasp)
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
        markers = [Marker(action=Marker.DELETEALL)]

        def add(namespace, kind, matrix, scale, rgba):
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = self.world_frame, stamp
            marker.ns, marker.id, marker.type = namespace, len(markers), kind
            marker.action = Marker.ADD
            marker.pose = ros_pose(matrix)
            marker.scale.x, marker.scale.y, marker.scale.z = map(float, scale)
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = map(float, rgba)
            markers.append(marker)
            return marker

        for name, geometry, matrix in self.model.visuals(hand):
            mesh = geometry.find("mesh")
            if mesh is None:
                raise ValueError(f"Preview requires mesh visual for hand link {name}")
            scale = [float(v) for v in mesh.get("scale", "1 1 1").split()]
            marker = add("open_hand", Marker.MESH_RESOURCE, matrix, scale, [0.25, 0.65, 1, 0.75])
            marker.mesh_resource = mesh.get("filename")
            marker.mesh_use_embedded_materials = False
        add("manual_object", Marker.CYLINDER, obj, [2*self.radius, 2*self.radius, self.height],
            [0.15, 0.8, 0.2, 0.75])
        surface = np.eye(4)
        surface[:3, 3] = [obj[0, 3], obj[1, 3], self.table-0.001]
        add("table_reference", Marker.CUBE, surface, [0.45, 0.45, 0.002], [0.5, 0.5, 0.5, 0.35])
        add("flange", Marker.SPHERE, flange, [0.015]*3, [1, 0.5, 0, 1])
        add("grasp_origin", Marker.SPHERE, grasp, [0.012]*3, [1, 1, 0, 1])
        for axis, color in enumerate(([1, 0.1, 0.1, 1], [0.1, 1, 0.1, 1], [0.1, 0.7, 1, 1])):
            arrow = add("grasp_axes", Marker.ARROW, grasp, [0.004, 0.009, 0.014], color)
            endpoint = [0., 0., 0.]
            endpoint[axis] = 0.07
            arrow.points = [Point(), Point(x=endpoint[0], y=endpoint[1], z=endpoint[2])]
        label_pose = np.eye(4)
        label_pose[:3, 3] = obj[:3, 3] + [0, 0, 0.27]
        label = add("notice", Marker.TEXT_VIEW_FACING, label_pose, [0, 0, 0.018], [1, 1, 1, 1])
        label.text = "STATIC GRASP PREVIEW - no robot motion"
        return MarkerArray(markers=markers)


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
