"""Static grasp geometry from the installed URDF; no ROS or motion clients."""

import math
import xml.etree.ElementTree as ET

import numpy as np

from .geometry import finite_vector, quaternion_from_rpy


def transform(xyz, quaternion):
    """T_parent_child, quaternion in ROS xyzw order."""
    xyz = finite_vector(xyz, 3, "translation")
    q = np.asarray(finite_vector(quaternion, 4, "quaternion"))
    norm = np.linalg.norm(q)
    if abs(norm - 1.0) > 0.01:
        raise ValueError("quaternion must be unit length")
    x, y, z, w = q / norm
    result = np.eye(4)
    result[:3, :3] = [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    result[:3, 3] = xyz
    return result


def from_rpy(xyz, rpy):
    return transform(xyz, quaternion_from_rpy(rpy))


def pose_components(matrix):
    """Stable unit quaternion conversion, including rotations of 180 degrees."""
    r = np.asarray(matrix)[:3, :3]
    k = np.array([
        [r[0, 0]-r[1, 1]-r[2, 2], r[0, 1]+r[1, 0], r[0, 2]+r[2, 0], r[2, 1]-r[1, 2]],
        [r[0, 1]+r[1, 0], r[1, 1]-r[0, 0]-r[2, 2], r[1, 2]+r[2, 1], r[0, 2]-r[2, 0]],
        [r[0, 2]+r[2, 0], r[1, 2]+r[2, 1], r[2, 2]-r[0, 0]-r[1, 1], r[1, 0]-r[0, 1]],
        [r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1], np.trace(r)]]) / 3
    _, vectors = np.linalg.eigh(k)
    q = vectors[:, -1]
    if q[3] < 0:
        q = -q
    return tuple(np.asarray(matrix)[:3, 3]), tuple(q)


def origin(element):
    if element is None:
        return np.eye(4)
    xyz = [float(v) for v in element.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in element.get("rpy", "0 0 0").split()]
    return from_rpy(xyz, rpy)


class HandModel:
    """URDF hand subtree with a fixed mount and explicit open joint posture."""

    def __init__(self, urdf, flange_link, hand_link, joint_names, joint_positions):
        root = ET.fromstring(urdf)
        self.links = {link.get("name"): link for link in root.findall("link")}
        joints = root.findall("joint")
        self.by_child = {j.find("child").get("link"): j for j in joints}
        if flange_link not in self.links or hand_link not in self.links:
            raise ValueError("flange_link and hand_link must exist in the URDF")
        if len(joint_names) != len(joint_positions) or len(set(joint_names)) != len(joint_names):
            raise ValueError("open joint names and positions must match and be unique")
        positions = dict(zip(joint_names, finite_vector(joint_positions, len(joint_names), "open posture")))
        all_names = {j.get("name") for j in joints}
        if set(positions) - all_names:
            raise ValueError("open posture contains an unknown joint")
        mount = np.eye(4)
        current = hand_link
        while current != flange_link:
            joint = self.by_child.get(current)
            if joint is None or joint.get("type") != "fixed":
                raise ValueError("hand must have a fixed joint chain to the flange")
            mount = origin(joint.find("origin")) @ mount
            current = joint.find("parent").get("link")
        self.flange_hand = mount
        self.hand_links = {hand_link: np.eye(4)}
        self.resolved_positions = {}
        self.open_positions = positions
        remaining = list(joints)
        while True:
            added = False
            for joint in list(remaining):
                parent = joint.find("parent").get("link")
                if parent not in self.hand_links:
                    continue
                kind = joint.get("type")
                angle = 0.0
                mimic = joint.find("mimic")
                if mimic is not None:
                    leader = mimic.get("joint")
                    if leader not in self.resolved_positions:
                        continue
                    angle = (self.resolved_positions[leader] * float(mimic.get("multiplier", 1))
                             + float(mimic.get("offset", 0)))
                elif kind != "fixed":
                    if joint.get("name") not in positions:
                        raise ValueError(f"Missing open posture for {joint.get('name')}")
                    angle = positions[joint.get("name")]
                motion = np.eye(4)
                if kind in ("revolute", "continuous"):
                    axis = np.array([float(v) for v in joint.find("axis").get("xyz").split()])
                    axis_norm = np.linalg.norm(axis)
                    if not np.isfinite(axis_norm) or axis_norm == 0:
                        raise ValueError("joint axis must be finite and nonzero")
                    axis /= axis_norm
                    motion = transform([0, 0, 0], [*(axis * math.sin(angle/2)), math.cos(angle/2)])
                elif kind != "fixed":
                    raise ValueError(f"Unsupported hand joint type: {kind}")
                limit = joint.find("limit")
                if kind == "revolute" and limit is not None:
                    if not float(limit.get("lower")) <= angle <= float(limit.get("upper")):
                        raise ValueError(f"Open posture exceeds limits for {joint.get('name')}")
                child = joint.find("child").get("link")
                self.hand_links[child] = self.hand_links[parent] @ origin(joint.find("origin")) @ motion
                self.resolved_positions[joint.get("name")] = angle
                remaining.remove(joint)
                added = True
            if not added:
                break
        if any(j.find("parent").get("link") in self.hand_links for j in remaining):
            raise ValueError("Unresolved mimic joints in hand subtree")

    def visuals(self, world_hand):
        for name, hand_link in self.hand_links.items():
            for visual in self.links[name].findall("visual"):
                yield name, visual.find("geometry"), world_hand @ hand_link @ origin(visual.find("origin"))


def grasp_targets(world_object, flange_hand, hand_grasp, object_grasp):
    flange_grasp = flange_hand @ hand_grasp
    world_grasp = world_object @ object_grasp
    world_flange = world_grasp @ np.linalg.inv(flange_grasp)
    return world_flange, world_grasp, world_flange @ flange_hand
