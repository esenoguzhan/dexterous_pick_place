"""jog.py JOINT DELTA [DURATION]: move one arm joint (1-6) by DELTA rad and back."""
import sys
import time

import numpy as np

import robot as rb

joint, delta = int(sys.argv[1]), float(sys.argv[2])
duration = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
r = rb.start("restore_jog")
q0 = r.arm_joints()
q1 = q0.copy()
q1[joint - 1] += delta
print("start", np.round(q0, 4).tolist())
t = time.monotonic()
err = r.move_joints(q1, duration)
print(f"there in {time.monotonic() - t:.1f} s, error {err:.4f}", np.round(r.arm_joints(), 4).tolist())
err = r.move_joints(q0, duration)
print(f"back, error {err:.4f}")
r.close()
