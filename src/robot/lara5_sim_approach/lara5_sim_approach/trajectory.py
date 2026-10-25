"""Validate and conservatively time a sampled Cartesian joint path."""

import math

import numpy as np
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def duration(seconds):
    nanoseconds = round(float(seconds) * 1e9)
    return Duration(sec=nanoseconds // 1_000_000_000, nanosec=nanoseconds % 1_000_000_000)


def seconds(value):
    return value.sec + value.nanosec / 1e9


def retime_cartesian(path, velocity=0.12, acceleration=0.25):
    """Each checked segment uses cubic smoothstep (zero endpoint velocities).

    Position + velocity controller interpolation reproduces this cubic exactly.
    The curve stays on the checked straight joint segment. Acceleration is bounded
    but discontinuous between segments; no jerk-limited behavior is claimed.
    """
    if velocity <= 0 or acceleration <= 0 or not path.points:
        raise ValueError("Positive limits and a nonempty Cartesian path are required")
    result = JointTrajectory(joint_names=list(path.joint_names))
    elapsed = 0.0
    previous = np.asarray(path.points[0].positions, dtype=float)
    for index, point in enumerate(path.points):
        current = np.asarray(point.positions, dtype=float)
        if len(current) != len(path.joint_names) or not np.isfinite(current).all():
            raise ValueError("Invalid Cartesian path joint positions")
        if index:
            delta = float(np.max(np.abs(current - previous)))
            if delta > 0.15:
                raise ValueError("Discontinuous Cartesian IK solution")
            elapsed += max(0.05, 1.5 * delta / velocity, math.sqrt(6 * delta / acceleration))
        result.points.append(JointTrajectoryPoint(
            positions=current.tolist(), velocities=[0.0] * len(current), time_from_start=duration(elapsed)))
        previous = current
    if len(result.points) < 2:
        raise ValueError("Cartesian path contains no motion")
    return result


def validate_trajectory(path, joint_names):
    if set(path.joint_names) != set(joint_names) or len(path.joint_names) != len(joint_names):
        raise ValueError("Trajectory must specify each arm joint exactly once")
    previous = -1.0
    if len(path.points) < 2:
        raise ValueError("Trajectory is empty or has no motion")
    for point in path.points:
        t = seconds(point.time_from_start)
        if not math.isfinite(t) or t < 0 or t <= previous:
            raise ValueError("Trajectory timestamps must strictly increase")
        if len(point.positions) != len(joint_names) or len(point.velocities) != len(joint_names):
            raise ValueError("Position and velocity vectors are required by this executor")
        if not np.isfinite(point.positions + point.velocities).all():
            raise ValueError("Trajectory contains nonfinite values")
        if point.accelerations and (len(point.accelerations) != len(joint_names)
                                    or not np.isfinite(point.accelerations).all()):
            raise ValueError("Invalid trajectory acceleration vector")
        previous = t


def interpolation_samples(path, max_dt=0.1):
    """Sample the cubic/quintic spline used by the existing trajectory controller.

    Return position, velocity, acceleration; include interior points so validation
    checks spline overshoot as well as the original IK waypoints.
    """
    for left, right in zip(path.points, path.points[1:]):
        interval = seconds(right.time_from_start) - seconds(left.time_from_start)
        if interval <= 0:
            raise ValueError("Trajectory timestamps must increase")
        p0, p1 = np.asarray(left.positions), np.asarray(right.positions)
        v0, v1 = np.asarray(left.velocities), np.asarray(right.velocities)
        c0, c1 = p0, v0 * interval
        if left.accelerations and right.accelerations:
            c2 = 0.5 * np.asarray(left.accelerations) * interval**2
            dp = p1 - c0 - c1 - c2
            dv = v1 * interval - c1 - 2*c2
            da = np.asarray(right.accelerations) * interval**2 - 2*c2
            coefficients = [c0, c1, c2, 10*dp-4*dv+0.5*da,
                            -15*dp+7*dv-da, 6*dp-3*dv+0.5*da]
        else:
            coefficients = [c0, c1, 3*(p1-p0)-(2*v0+v1)*interval,
                            -2*(p1-p0)+(v0+v1)*interval]
        samples = max(2, math.ceil(interval/max_dt), math.ceil(float(np.max(np.abs(p1-p0)))/0.02))
        for u in np.linspace(0, 1, samples+1):
            position = sum(c*u**i for i, c in enumerate(coefficients))
            velocity = sum(i*c*u**(i-1) for i, c in enumerate(coefficients) if i) / interval
            acceleration = sum(i*(i-1)*c*u**(i-2) for i, c in enumerate(coefficients) if i > 1) / interval**2
            yield position, velocity, acceleration
