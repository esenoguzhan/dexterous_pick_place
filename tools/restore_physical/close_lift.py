"""close_lift.py [--lift M]: at the grasp pose, close (fingers, then thumb), lift, photograph, put back, release, go up."""
import argparse

import cv2
import numpy as np

import jetson
import restore as rs

ap = argparse.ArgumentParser()
ap.add_argument("--lift", type=float, default=0.05)
ap.add_argument("--keep", action="store_true", help="stay lifted, holding")
ap.add_argument("--descend", type=float, default=0.0, help="go this far down first")
ap.add_argument("--staged", action="store_true", help="fingers first, then the thumb (default: all at once)")
a = ap.parse_args()
t = rs.Restorer()
r = t.r
try:
    T_grasp = r.frame_pose("lara5_flange", "world")
    if a.descend:
        T_grasp[2, 3] -= a.descend
        r.move_pose(T_grasp)
    if a.staged:
        hj = r.hand([0.0, rs.FINGERS_CLOSED, rs.FINGERS_CLOSED, rs.FINGERS_CLOSED, rs.FINGERS_CLOSED, 0.0], 2.0)
        print("fingers", np.round(hj, 3).tolist())
        hj = r.hand([rs.THUMB_YAW, hj[1], hj[2], hj[3], hj[4], 0.0], 1.5)
        print("thumb yaw", np.round(hj, 3).tolist())
        hj = r.hand([hj[0], hj[1], hj[2], hj[3], hj[4], rs.THUMB_PITCH], 1.0)
    else:
        f = rs.FINGERS_CLOSED
        hj = r.hand([rs.THUMB_YAW, f, f, f, f, rs.THUMB_PITCH], 2.0)
    print("closed at", np.round(hj, 3).tolist())
    T = T_grasp.copy()
    T[2, 3] += a.lift
    r.move_pose(T)
    f = jetson.capture(jetson.FIXED, frames=1)
    cv2.imwrite("/run_data/lifted.jpg", f.color[..., ::-1])
    print("lifted; hand", np.round(r.hand_joints(), 3).tolist())
    if not a.keep:
        r.move_pose(T_grasp)
        t.release()
        T = T_grasp.copy()
        T[2, 3] += 0.15
        r.move_pose(T)
        print("released and up")
finally:
    t.close()
