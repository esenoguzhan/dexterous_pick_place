"""Shared static grasp overlay; the actual moving robot is shown by MuJoCo."""

import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def make_grasp_markers(self, obj, flange, grasp, hand, stamp):
    from .preview import ros_pose
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
    label.text = "STATIC TARGET HAND - actual robot shown in MuJoCo"
    return MarkerArray(markers=markers)
