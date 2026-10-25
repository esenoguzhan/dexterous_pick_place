"""Shared grasp preset and object-pose validation for preview and execution."""

from pathlib import Path

import numpy as np
import xacro
from ament_index_python.packages import get_package_share_directory

from .geometry import object_to_flange
from .grasp_geometry import HandModel, from_rpy, grasp_targets, transform


def configure_grasp(node):
    def param(name, default):
        return node.declare_parameter(name, default).value
    node.world_frame = param("world_frame", "world")
    if not node.world_frame:
        raise ValueError("world_frame must be nonempty")
    flange = param("flange_link", "lara5_flange")
    hand = param("hand_link", "right_hand_base_link")
    names = param("open_joint_names", ["right_thumb_finger_proximal_yaw",
                  "right_index_finger_proximal", "right_middle_finger_proximal",
                  "right_ring_finger_proximal", "right_pinky_finger_proximal",
                  "right_thumb_finger_proximal_pitch"])
    positions = param("open_joint_positions", [0.0]*6)
    node.hand_grasp = from_rpy(param("hand_to_grasp_xyz", [-.025, .085, .15]),
                               param("hand_to_grasp_rpy", [-np.pi/2, 0., 0.]))
    node.object_grasp = from_rpy(param("object_to_grasp_xyz", [0.]*3),
                                 param("object_to_grasp_rpy", [np.pi, 0., 0.]))
    description = param("robot_description", "")
    if not description:
        path = Path(get_package_share_directory("lara5_inspire_hand_description"))/"urdf/lara5_inspire_hand.urdf.xacro"
        description = xacro.process_file(str(path)).toxml()
    node.model = HandModel(description, flange, hand, names, positions)
    node.flange_grasp = node.model.flange_hand @ node.hand_grasp
    return description, flange, names, positions


def target_geometry(node, message):
    if message.header.frame_id != node.world_frame:
        raise ValueError(f"object pose must be in '{node.world_frame}'")
    p, q = message.pose.position, message.pose.orientation
    position, quaternion = object_to_flange(
        [p.x, p.y, p.z], [q.x, q.y, q.z, q.w], [0., 0., 0.], [0., 0., 0.])
    obj = transform(position, quaternion)
    flange, grasp, hand = grasp_targets(obj, node.model.flange_hand, node.hand_grasp, node.object_grasp)
    return obj, flange, grasp, hand
