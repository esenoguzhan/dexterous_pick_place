"""grasp_test.py [--clearance M] [--lift M]: from the hover pose, descend, grasp, lift, put back, release.

Run after hover.py --go. Saves fixed-camera images to /run_data/grasp_test_*.jpg at each stage.
"""
import argparse

import cv2
import numpy as np

import grasp as gr
import jetson
import robot as rb

ap = argparse.ArgumentParser()
ap.add_argument("--descend", type=float, default=0.15)
ap.add_argument("--lift", type=float, default=0.05)
ap.add_argument("--fingers", type=float, default=1.3)
ap.add_argument("--thumb-yaw", type=float, default=1.16)
ap.add_argument("--thumb-pitch", type=float, default=0.4)
a = ap.parse_args()


def snap(tag):
    f = jetson.capture(jetson.FIXED, frames=1)
    cv2.imwrite(f"/run_data/grasp_test_{tag}.jpg", f.color[..., ::-1])


r = rb.start("restore_grasp_test")
T0 = r.frame_pose("lara5_flange", "world")
down = T0.copy()
down[2, 3] -= a.descend
print("descend to flange z", round(down[2, 3], 3))
r.move_pose(down, duration=5.0)
snap("1_down")
h = r.hand([0.0, a.fingers, a.fingers, a.fingers, a.fingers, 0.0], 2.0)
print("fingers stopped at", np.round(h, 3).tolist())
h = r.hand([a.thumb_yaw, h[1], h[2], h[3], h[4], 0.0], 1.5)
print("thumb yaw at", np.round(h, 3).tolist())
h = r.hand([h[0], h[1], h[2], h[3], h[4], a.thumb_pitch], 1.0)
print("thumb pitch at", np.round(h, 3).tolist())
snap("2_closed")
up = down.copy()
up[2, 3] += a.lift
r.move_pose(up, duration=3.0)
snap("3_lifted")
print("lifted; hand", np.round(r.hand_joints(), 3).tolist())
r.move_pose(down, duration=3.0)
h = r.hand_joints()
r.hand([h[0], h[1], h[2], h[3], h[4], 0.0], 1.0)   # thumb first: it collides with the index otherwise
r.hand([0.0, h[1], h[2], h[3], h[4], 0.0], 1.0)
r.hand([0.0] * 6, 1.5)
r.move_pose(T0, duration=4.0)
snap("4_released")
print("back at hover")
r.close()
