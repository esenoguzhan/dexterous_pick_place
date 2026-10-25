"""URDF FK and conservative, bounded interpolation for planning-only previews."""

import copy
import math
import xml.etree.ElementTree as ET

import numpy as np
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .grasp_geometry import origin, transform


def seconds(value):
    if value.sec < 0 or not 0 <= value.nanosec < 1_000_000_000:
        raise ValueError("trajectory duration must be nonnegative and normalized")
    return value.sec + value.nanosec/1e9


def duration(value):
    ns = round(float(value)*1e9)
    return Duration(sec=ns//1_000_000_000, nanosec=ns % 1_000_000_000)


class ArmModel:
    """FK in the configured world frame, using the same installed robot URDF."""

    def __init__(self, urdf, world, flange, joint_names):
        root = ET.fromstring(urdf)
        joints = {j.get("name"): j for j in root.findall("joint")}
        by_child = {j.find("child").get("link"): j for j in joints.values()}
        self.names = list(joint_names)
        if len(self.names) != 6 or len(set(self.names)) != 6:
            raise ValueError("arm_joint_names must contain six unique joints")
        self.lower, self.upper, self.velocity = [], [], []
        for name in self.names:
            j = joints.get(name)
            if j is None or j.get("type") != "revolute":
                raise ValueError(f"Expected bounded revolute arm joint {name}")
            limit = j.find("limit")
            lo, hi, velocity = (float(limit.get(k)) for k in ("lower", "upper", "velocity"))
            if not np.isfinite([lo, hi, velocity]).all() or lo >= hi or velocity <= 0:
                raise ValueError("invalid URDF joint limits")
            self.lower.append(lo); self.upper.append(hi); self.velocity.append(velocity)
        self.lower, self.upper, self.velocity = map(np.asarray, (self.lower, self.upper, self.velocity))
        self.chain = []
        current, visited = flange, set()
        active = set()
        while current != world:
            if current in visited or current not in by_child:
                raise ValueError(f"No URDF chain from {world} to {flange}")
            visited.add(current)
            j = by_child[current]
            name = j.get("name")
            axis = None
            if j.get("type") != "fixed":
                if name not in self.names:
                    raise ValueError(f"Unconfigured moving joint {name} in flange chain")
                active.add(name)
                element = j.find("axis")
                axis = np.array([float(v) for v in (element.get("xyz") if element is not None else "1 0 0").split()])
                if axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis) == 0:
                    raise ValueError("invalid URDF joint axis")
                axis /= np.linalg.norm(axis)
            self.chain.append((name, origin(j.find("origin")), axis))
            current = j.find("parent").get("link")
        if active != set(self.names):
            raise ValueError("flange chain must contain all configured arm joints")
        self.chain.reverse()

    def check(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError("arm state needs six finite positions")
        if np.any(q < self.lower-1e-8) or np.any(q > self.upper+1e-8):
            raise ValueError("arm position exceeds URDF limits")
        return q

    def fk(self, q):
        q = self.check(q)
        positions = dict(zip(self.names, q))
        result = np.eye(4)
        for name, fixed, axis in self.chain:
            result = result @ fixed
            if axis is not None:
                angle = positions[name]
                result = result @ transform([0, 0, 0], [*(axis*math.sin(angle/2)), math.cos(angle/2)])
        return result


def pregrasp_target(world_object, final_flange, world_grasp, clearance):
    """Back off the final grasp axis until flange Z is clearance above object Z."""
    if not math.isfinite(clearance) or clearance <= 0:
        raise ValueError("pregrasp_flange_clearance must be positive")
    back = -world_grasp[:3, 2]
    height = world_object[2, 3]+clearance-final_flange[2, 3]
    if back[2] < .5 or height <= 0:
        raise ValueError("pregrasp must be above the final flange along a downward grasp axis")
    result = final_flange.copy()
    result[:3, 3] += back*(height/back[2])
    return result


def pose_error(actual, desired):
    distance = float(np.linalg.norm(actual[:3, 3]-desired[:3, 3]))
    cosine = (np.trace(actual[:3, :3].T @ desired[:3, :3])-1)/2
    return distance, math.acos(float(np.clip(cosine, -1, 1)))


def retime(raw, start, model, max_step, velocity, acceleration, max_points=2000):
    """Zero-velocity cubic segments stay on the collision-checked joint segments.

    Their peak speed is 1.5*delta/dt and peak acceleration is 6*delta/dt**2.
    Acceleration is bounded but can jump at waypoints; no jerk limit is claimed.
    Native timestamps are validated and retained as minimum segment durations.
    """
    if (len(raw.joint_names) != 6 or set(raw.joint_names) != set(model.names)
            or len(set(raw.joint_names)) != 6):
        raise ValueError("native trajectory must specify each arm joint exactly once")
    if len(raw.points) > max_points:
        raise ValueError("native trajectory exceeds waypoint limit")
    if not np.isfinite([max_step, velocity, acceleration]).all() or min(max_step, velocity, acceleration) <= 0:
        raise ValueError("trajectory limits must be finite and positive")
    columns = [raw.joint_names.index(name) for name in model.names]
    previous = model.check(start)
    result = JointTrajectory(joint_names=model.names)
    result.points.append(JointTrajectoryPoint(positions=previous.tolist(), velocities=[0.]*6,
                                               time_from_start=duration(0)))
    elapsed, native_time = 0., 0.
    speed = np.minimum(model.velocity, velocity)
    for point in raw.points:
        stamp = seconds(point.time_from_start)
        if stamp <= native_time:
            raise ValueError("native trajectory timestamps must strictly increase from zero")
        if len(point.positions) != 6:
            raise ValueError("every native waypoint needs six positions")
        for values in (point.velocities, point.accelerations, point.effort):
            if values and (len(values) != 6 or not np.isfinite(values).all()):
                raise ValueError("malformed native trajectory vector")
        current = model.check(np.asarray(point.positions)[columns])
        delta = np.abs(current-previous)
        if np.max(delta) > max_step+1e-8:
            raise ValueError("discontinuous native IK waypoints")
        interval = max(stamp-native_time, .1, float(np.max(1.5*delta/speed)),
                       math.sqrt(6*float(np.max(delta))/acceleration))
        elapsed += interval
        result.points.append(JointTrajectoryPoint(positions=current.tolist(), velocities=[0.]*6,
                                                   time_from_start=duration(elapsed)))
        previous, native_time = current, stamp
    if not raw.points:
        # Native API reports an empty path when already at target. Endpoint FK
        # must still prove that claim before this stationary phase is accepted.
        point = copy.deepcopy(result.points[0])
        point.time_from_start = duration(1.)
        result.points.append(point)
    return result


def path_poses(path, model, start_pose, target, tolerance=.003, angle_tolerance=.01):
    """Check interpolated FK remains on the requested Cartesian line and bounds."""
    a, b = start_pose[:3, 3], target[:3, 3]
    direction = b-a
    length2 = float(direction @ direction)
    same_rotation = pose_error(start_pose, target)[1] < 1e-7
    poses = []
    for left, right in zip(path.points, path.points[1:]):
        q0, q1 = np.asarray(left.positions), np.asarray(right.positions)
        # Cubic interpolation stays on this same joint segment, so geometric
        # samples can be parameterized directly by joint progress.
        count = max(2, math.ceil(float(np.max(np.abs(q1-q0)))/.02))
        for u in np.linspace(0, 1, count+1):
            pose = model.fk(q0+(q1-q0)*u)
            s = float((pose[:3, 3]-a)@direction/length2) if length2 > 1e-12 else 0.
            closest = a+np.clip(s, 0, 1)*direction
            if np.linalg.norm(pose[:3, 3]-closest) > tolerance:
                raise ValueError("interpolated FK leaves the requested Cartesian segment")
            if same_rotation and pose_error(pose, target)[1] > angle_tolerance:
                raise ValueError("interpolated FK changes the final approach orientation")
        poses.append(model.fk(right.positions))
    return [start_pose, *poses]
