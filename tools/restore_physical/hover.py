"""hover.py X Y [--go] [--clearance M]: put the open hand palm-down above the object nearest (X, Y)."""
import argparse

import numpy as np

import grasp as gr
import motion as mo
import robot as rb
import scene as sc

ap = argparse.ArgumentParser()
ap.add_argument("x", type=float)
ap.add_argument("y", type=float)
ap.add_argument("--go", action="store_true")
ap.add_argument("--clearance", type=float, default=None)
ap.add_argument("--speed", type=float, default=0.25)
a = ap.parse_args()

cfg = gr.settings()
clearance = a.clearance if a.clearance is not None else cfg["hover_clearance_m"]
s = sc.Scene()
result, frame, _ = s.capture("hover")
for o in result["objects"]:
    print(o["id"], np.round(o["position"], 3).tolist(), "top", round(o["top_z"], 3), "r", round(o["top_radius"], 3))
obj = min(result["objects"], key=lambda o: np.hypot(o["position"][0] - a.x, o["position"][1] - a.y))
top = np.array([obj["position"][0], obj["position"][1], obj["top_z"]])
print("target", obj["id"], "top", np.round(top, 3).tolist())

r = rb.start("restore_hover")
T_flange_hand = r.frame_pose("right_hand_base_link", "lara5_flange")
current = r.arm_joints()
sol = mo.solve_hand(r, top, clearance, T_flange_hand, current, radius=obj["top_radius"])
if sol is None:
    raise SystemExit("no IK solution")
q, yaw, T_flange = sol
print("heading", round(np.degrees(yaw - gr.radial_yaw(top)), 1), "joints", np.round(q, 3).tolist(),
      "change", np.round(q - current, 2).tolist())
if a.go:
    r.hand(cfg["open"], 1.5)
    err = r.move_joints(q, max_speed=a.speed)
    print("arrived, joint error", round(err, 4), "flange", np.round(r.frame_pose("lara5_flange", "world")[:3, 3], 3).tolist())
r.close()
