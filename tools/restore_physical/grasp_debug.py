"""grasp_debug.py X Y [--up]: hover over the object nearest (X, Y), descend to the grasp pose with the hand
open and photograph both cameras (/run_data/gd_fixed.jpg, gd_wrist.jpg). --up: only go back up."""
import argparse
import json

import cv2
import numpy as np

import jetson
import restore as rs

ap = argparse.ArgumentParser()
ap.add_argument("x", type=float)
ap.add_argument("y", type=float)
ap.add_argument("--up", action="store_true")
ap.add_argument("--above", type=float, default=0.0, help="stop this far above the grasp height")
a = ap.parse_args()
t = rs.Restorer()
try:
    if a.up:
        rs.Restorer.observe(t)
        raise SystemExit
    t.observe()
    cur = rs.capture(t.scene, "gd")
    obj = min(cur["objects"], key=lambda o: np.hypot(o["position"][0] - a.x, o["position"][1] - a.y))
    others = [o for o in cur["objects"] if o is not obj]
    top = np.array([obj["position"][0], obj["position"][1], obj["top_z"]])
    print("object", rs.describe(obj), "top", np.round(top, 3).tolist())
    t.r.set_scene_boxes([rs.object_box(o) for o in others])
    pick = t.solve(top, t.cfg["hover_clearance_m"], obj["top_radius"], others, rs.table_z(cur, top))
    t.r.hand(t.cfg["open"], 1.0)
    t.r.move_joints(pick[0], max_speed=0.35)
    T = pick[2].copy()
    T[2, 3] -= t.cfg["hover_clearance_m"] - a.above
    t.r.move_pose(T, duration=4.0)
    H = t.r.frame_pose("right_hand_base_link", "world")
    p_obj_in_hand = np.linalg.inv(H) @ np.r_[top, 1.0]
    print("object top in hand frame", np.round(p_obj_in_hand[:3], 3).tolist())
    for serial, name in ((jetson.FIXED, "fixed"),):
        f = jetson.capture(serial, frames=2)
        cv2.imwrite(f"/run_data/gd_{name}.jpg", f.color[..., ::-1])
        if name == "wrist":
            np.save("/run_data/gd_wrist_depth.npy", f.depth)
            json.dump(f.intrinsics.to_dict(), open("/run_data/gd_wrist_intr.json", "w"))
    np.save("/run_data/gd_hand_pose.npy", H)
    print("stopped at the grasp pose with the hand open")
finally:
    t.close()
