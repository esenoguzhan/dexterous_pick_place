"""Section 7: grasp candidates, pick poses and place poses.

    T_world_flange = T_world_object · T_object_tcp(primitive, yaw) · inverse(T_flange_tcp)

The place pose reuses the grasp's T_object_flange on the target's object frame, so the object ends
up on the target, whatever grasp was used.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from restore_perception.core.types import SYM_CONTINUOUS

from .poses import from_xyz_rpy, inverse, lifted, rot_z, transform, world_yaw, wrap


@dataclass
class Grasp:
    yaw: float  # grasp yaw in the object frame
    T_object_tcp: np.ndarray
    T_object_flange: np.ndarray
    width: float  # object extent across the closing direction
    pre_grasp: np.ndarray  # world flange poses
    grasp: np.ndarray
    lift: np.ndarray
    wrist_turn: float  # |flange yaw change| from the current flange pose
    tight: bool  # a neighbour is within the hand radius


def candidate_yaws(primitive, symmetry):
    """Grasp yaws in the object frame: the primitive's own, times the object's symmetric copies."""
    base = [math.radians(a) for a in primitive["yaws_deg"]]
    if symmetry == SYM_CONTINUOUS:
        copies = [i * math.pi / 6 for i in range(12)]
    elif symmetry >= 2:
        copies = [i * 2 * math.pi / symmetry for i in range(symmetry)]
    else:
        copies = [0.0]
    yaws = []
    for b in base:
        for c in copies:
            y = wrap(b + c)
            if all(abs(wrap(y - other)) > 1e-6 for other in yaws):
                yaws.append(y)
    return yaws


def width_across(size, yaw):
    """Footprint extent across the closing direction: the object's y axis turned by the grasp yaw."""
    length, width = float(size[0]), float(size[1])
    return abs(math.cos(yaw)) * width + abs(math.sin(yaw)) * length


def object_to_tcp(primitive, size, yaw):
    """Palm frame in the object frame for one grasp yaw."""
    if primitive["approach"] == "top":
        offset = (0.0, 0.0, float(size[2]) / 2.0 + primitive["palm_height_m"])
    else:  # side: the palm faces the object's centre from distance width/2 + palm_distance
        offset = (width_across(size, yaw + math.pi / 2) / 2.0 + primitive["palm_distance_m"], 0.0, 0.0)
    return transform(rot_z(yaw)) @ from_xyz_rpy(offset, primitive["tcp_rpy"])


def _approach_axis(T_world_tcp):
    """The palm frame's z axis: the direction the hand moves in to grasp."""
    return T_world_tcp[:3, 2]


def candidates(task, primitive, T_flange_tcp, flange_yaw, neighbours, cfg):
    """Grasps for a task, the smallest wrist turn first; grasps near a neighbour go last.

    neighbours: list of N x 2 footprints (world xy) of the other objects."""
    T_world_object = task.current.matrix()
    T_tcp_flange = inverse(T_flange_tcp)
    out = []
    for yaw in candidate_yaws(primitive, task.symmetry):
        T_object_tcp = object_to_tcp(primitive, task.size, yaw)
        T_world_tcp = T_world_object @ T_object_tcp
        grasp = T_world_tcp @ T_tcp_flange
        pre = grasp.copy()
        pre[:3, 3] -= _approach_axis(T_world_tcp) * cfg["pre_distance_m"]
        centre = T_world_tcp[:2, 3] if primitive["approach"] == "top" else task.current.xy
        tight = any(len(n) and np.min(np.linalg.norm(np.asarray(n) - centre, axis=1)) < cfg["hand_radius_m"]
                    for n in neighbours)
        out.append(Grasp(yaw, T_object_tcp, T_object_tcp @ T_tcp_flange, width_across(task.size, yaw), pre, grasp,
                         lifted(grasp, cfg["pre_distance_m"]), abs(wrap(world_yaw(grasp[:3, :3]) - flange_yaw)),
                         tight))
    return sorted(out, key=lambda g: (g.tight, g.wrist_turn))


@dataclass
class PlacePoses:
    pre_place: np.ndarray
    place: np.ndarray
    retreat: np.ndarray


def place_poses(target, grasp, primitive, rolls, cfg):
    """Flange poses to put the object down on `target` (a Placement): above, down, and away."""
    release = 0.0 if rolls else cfg["release_height_m"]
    T_world_object = lifted(target.matrix(), release)
    place = T_world_object @ grasp.T_object_flange
    if primitive["approach"] == "top":
        retreat = lifted(place, cfg["pre_distance_m"])
    else:  # pull the open hand back out along its approach axis; from there it can rise
        retreat = place.copy()
        retreat[:3, 3] -= _approach_axis(T_world_object @ grasp.T_object_tcp) * cfg["pre_distance_m"]
    return PlacePoses(lifted(place, cfg["pre_distance_m"]), place, retreat)


def object_in_hand(T_world_flange, grasp):
    """Where the held object is when the flange is at T_world_flange."""
    return T_world_flange @ inverse(grasp.T_object_flange)
