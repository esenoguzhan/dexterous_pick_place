"""RGB-D conversion and bounded final-approach geometry without ROS transport."""

import math
import xml.etree.ElementTree as ET

import numpy as np

from lara5_object_approach.grasp_geometry import HandModel, transform
from lara5_grasp.configuration import HAND_JOINTS
from .selection import select_target


def vector(values, size, name):
    result = np.asarray(values, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f'{name} requires {size} finite values')
    return result


def pose_matrix(pose):
    p, q = pose.position, pose.orientation
    return transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


def rotation_error(left, right):
    return math.acos(float(np.clip((np.trace(left.T @ right)-1)/2, -1, 1)))


def grasp_offset(description, hand_xyz):
    model = HandModel(description, 'lara5_flange', 'right_hand_base_link', HAND_JOINTS, [0.]*6)
    return (model.flange_hand @ np.r_[vector(hand_xyz, 3, 'hand_to_grasp_xyz'), 1.])[:3]


def require_hardware(description):
    plugins = [item.text for item in ET.fromstring(description).findall('ros2_control/hardware/plugin')]
    if ('neuraPlugin/PositionAndVelocityHW' not in plugins
            or any('TopicBasedSystem' in p or 'GenericSystem' in p for p in plugins)):
        raise ValueError('Wrist hardware task requires the real NEURA driver description')


def detect_target(rgb, depth, info, base_optical, config):
    """Return the explicit grasp reference in base coordinates, not object centre."""
    _, selected = select_target(rgb, depth, info, config)
    return (base_optical @ np.r_[selected.reference_optical, 1.])[:3]


def bounded_target(current, grasp_reference, offset, initial, max_step, max_initial_error,
                   max_travel, workspace_min, workspace_max):
    desired = vector(grasp_reference, 3, 'grasp_reference')-current[:3, :3] @ offset
    error = desired-current[:3, 3]
    if np.linalg.norm(desired-initial) > max_initial_error:
        raise ValueError('Object grasp pose is outside the manually positioned final-approach envelope')
    step = error*min(1., max_step/max(np.linalg.norm(error), 1e-12))
    endpoint = current[:3, 3]+step
    if np.any(current[:3, 3] < workspace_min) or np.any(current[:3, 3] > workspace_max):
        raise ValueError('Current flange is outside the configured robot-base workspace')
    if np.linalg.norm(endpoint-initial) > max_travel:
        raise ValueError('Total wrist correction displacement limit exceeded')
    if (np.any(endpoint < workspace_min) or np.any(endpoint > workspace_max)
            or np.any(desired < workspace_min) or np.any(desired > workspace_max)):
        raise ValueError('Final approach leaves the configured robot-base workspace')
    result = current.copy()
    result[:3, 3] = endpoint
    return result, float(np.linalg.norm(error))
