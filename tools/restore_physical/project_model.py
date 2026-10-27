"""project_model.py [OUT.jpg]: draw model frames (flange, hand, fingertips, link origins) on a fresh fixed-camera image."""
import sys

import cv2
import numpy as np

import jetson
import robot as rb
import scene as sc

s = sc.Scene()
frame = jetson.capture(jetson.FIXED, frames=1)
img = frame.color[..., ::-1].copy()
r = rb.start("restore_project")
K = frame.intrinsics.matrix()
frames = ["lara5_link1", "lara5_link2", "lara5_link3", "lara5_link4", "lara5_link5", "lara5_link6", "lara5_flange",
          "right_palm_force_sensor", "right_index_force_sensor_3", "right_pinky_force_sensor_3", "camera_2_link"]
for i, f in enumerate(frames):
    p = r.frame_pose(f, "world")[:3, 3]
    uv = s.project(p, K)
    print(f"{f:28s} world {np.round(p, 3).tolist()} -> px {None if uv is None else np.round(uv).astype(int).tolist()}")
    if uv is not None:
        c = (0, 0, 255) if "link" in f else (0, 255, 0)
        cv2.circle(img, (int(uv[0]), int(uv[1])), 6, c, -1)
        cv2.putText(img, f.replace("lara5_", "").replace("right_", ""), (int(uv[0]) + 8, int(uv[1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, c, 1, cv2.LINE_AA)
out = sys.argv[1] if len(sys.argv) > 1 else "/run_data/project_model.jpg"
cv2.imwrite(out, img)
print("joints", np.round(r.arm_joints(), 4).tolist())
r.close()
