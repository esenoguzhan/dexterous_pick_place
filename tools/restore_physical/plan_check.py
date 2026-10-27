"""plan_check.py DZ [DURATION]: plan (not run) a vertical Cartesian move and print its timing per joint."""
import sys

import numpy as np

import robot as rb
from lara5_safety_msgs.srv import PlanCartesianPath
from geometry_msgs.msg import PoseStamped

dz = float(sys.argv[1])
duration = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
r = rb.start("restore_plan_check")
T = r.frame_pose("lara5_flange", "world")
T[2, 3] += dz
req = PlanCartesianPath.Request()
req.target = PoseStamped()
req.target.header.frame_id = "world"
req.target.pose = rb.matrix_to_pose(T)
req.duration = duration
res = r._call(r.plan_client, req, 30.0)
print(res.success, res.message)
pts = res.trajectory.points
t = np.array([p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in pts])
q = np.array([p.positions for p in pts])
v = np.array([p.velocities for p in pts]) if pts and len(pts[0].velocities) else None
print("points", len(pts), "duration", t[-1] if len(t) else None)
print("start", np.round(q[0], 3).tolist(), "measured", np.round(r.arm_joints(), 3).tolist())
dq = np.diff(q, axis=0) / np.maximum(np.diff(t), 1e-6)[:, None]
print("max finite-diff joint speed", np.round(np.abs(dq).max(axis=0), 3).tolist())
print("total change", np.round(q[-1] - q[0], 3).tolist())
print("first times", np.round(t[:5], 3).tolist())
r.close()
