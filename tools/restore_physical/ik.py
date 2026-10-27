"""Numeric IK on top of the safety filter's forward kinematics (/safety/arm/get_frame_pose).

Only used to find joint-space waypoints (ready and hover poses); the motions themselves are
collision-checked by the safety filter, and straight segments use its own NEURA IK.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState

import robot as rb
from lara5_safety_msgs.srv import GetFramePose

LIMITS = np.array([[-3.14, 3.14], [-3.14, 3.14], [-2.61, 2.61], [-6.28, 6.28], [-3.14, 3.14], [-6.28, 6.28]])


def fk(r: rb.Robot, q, frame="lara5_flange", reference="world") -> np.ndarray:
    req = GetFramePose.Request(frame=frame, reference_frame=reference)
    req.start_state = JointState(name=rb.ARM, position=[float(v) for v in q])
    res = r._call(r.fk_client, req, 5.0)
    if not res.success:
        raise rb.MotionError(res.message)
    return rb.pose_to_matrix(res.pose.pose)


def error(T, target, w_rot=0.3):
    dp = target[:3, 3] - T[:3, 3]
    dr = Rotation.from_matrix(target[:3, :3] @ T[:3, :3].T).as_rotvec()
    return np.r_[dp, w_rot * dr]


def solve(r: rb.Robot, target, seed, frame="lara5_flange", iters=80, tol_m=1e-4, tol_rad=1e-3, damping=0.02):
    """Damped least squares from seed; returns (q, position error m, rotation error rad)."""
    q = np.array(seed, float)
    eps = 1e-4
    for _ in range(iters):
        try:
            T = fk(r, q, frame)
        except rb.MotionError:  # the safety filter refuses colliding states
            return q, float("inf"), float("inf")
        e = error(T, target)
        pos, rot = np.linalg.norm(e[:3]), np.linalg.norm(e[3:]) / 0.3
        if pos < tol_m and rot < tol_rad:
            break
        J = np.zeros((6, 6))
        for i in range(6):
            dq = q.copy()
            dq[i] += eps
            try:
                J[:, i] = (error(T, target) - error(fk(r, dq, frame), target)) / eps
            except rb.MotionError:
                return q, float("inf"), float("inf")
        step = J.T @ np.linalg.solve(J @ J.T + damping ** 2 * np.eye(6), e)
        step = np.clip(step, -0.3, 0.3)
        q = np.clip(q + step, LIMITS[:, 0], LIMITS[:, 1])
    try:
        T = fk(r, q, frame)
    except rb.MotionError:
        return q, float("inf"), float("inf")
    e = error(T, target)
    return q, float(np.linalg.norm(e[:3])), float(np.linalg.norm(e[3:]) / 0.3)
