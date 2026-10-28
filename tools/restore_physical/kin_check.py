"""kin_check.py: local URDF FK vs the safety filter's FK at the measured joints and a few others."""
import time

import numpy as np

import ik
import kin
import motion as mo
import robot as rb

r = rb.start("restore_kin_check")
for q in [r.arm_joints()] + mo.load_seeds()[:3]:
    try:
        a, b = kin.fk(q), ik.fk(r, q)
        print("position diff mm", round(1000 * float(np.linalg.norm(a[:3, 3] - b[:3, 3])), 3),
              "rotation diff deg", round(float(np.degrees(np.arccos(np.clip((np.trace(a[:3, :3].T @ b[:3, :3]) - 1) / 2, -1, 1)))), 4))
    except rb.MotionError as exc:
        print("service refused:", str(exc)[:60])
t = time.monotonic()
for _ in range(100):
    kin.fk(r.arm_joints())
print(f"local FK {1000 * (time.monotonic() - t) / 100:.2f} ms")
r.close()
import os
os._exit(0)
