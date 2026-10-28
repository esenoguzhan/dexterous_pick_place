"""Numeric IK for joint-space waypoints (ready, hover and pre-approach poses).

Damped least squares on the URDF chain (kin.py), so intermediate iterates near the table do not
stop it; only the final configuration goes through the safety filter's FK, which refuses states
that collide. The motions themselves are still checked by the safety filter.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState

import kin
import robot as rb
from lara5_safety_msgs.srv import GetFramePose

LIMITS = np.array([[-3.14, 3.14], [-3.14, 3.14], [-2.61, 2.61], [-6.28, 6.28], [-3.14, 3.14], [-6.28, 6.28]])


def fk(r: rb.Robot, q, frame="lara5_flange", reference="world") -> np.ndarray:
    """The safety filter's FK: raises MotionError for a configuration in (near) collision."""
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


def solve(r: rb.Robot, target, seed, frame="lara5_flange", iters=200, tol_m=1e-4, tol_rad=1e-3, damping=0.02):
    """Damped least squares from seed; returns (q, position error m, rotation error rad).

    inf errors: no convergence, or the safety filter refuses the result (collision)."""
    q = np.array(seed, float)
    eps = 1e-5
    for _ in range(iters):
        T = kin.fk(q, frame)
        e = error(T, target)
        if np.linalg.norm(e[:3]) < tol_m and np.linalg.norm(e[3:]) / 0.3 < tol_rad:
            break
        J = np.zeros((6, 6))
        for i in range(6):
            dq = q.copy()
            dq[i] += eps
            J[:, i] = (e - error(kin.fk(dq, frame), target)) / eps
        step = J.T @ np.linalg.solve(J @ J.T + damping ** 2 * np.eye(6), e)
        q = np.clip(q + np.clip(step, -0.3, 0.3), LIMITS[:, 0], LIMITS[:, 1])
    try:
        T = fk(r, q, frame)
    except rb.MotionError:
        return q, float("inf"), float("inf")
    e = error(T, target)
    return q, float(np.linalg.norm(e[:3])), float(np.linalg.norm(e[3:]) / 0.3)
