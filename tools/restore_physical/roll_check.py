"""roll_check.py: close the fingers in free air, draw the model's fingertip/palm frames (TF) on the fixed image."""
import time

import cv2
import numpy as np

import jetson
import robot as rb
import scene as sc

s = sc.Scene()
r = rb.start("restore_roll_check")
r.hand([0.0, 1.3, 1.3, 1.3, 1.3, 0.0], 2.0)
time.sleep(0.5)
f = jetson.capture(jetson.FIXED, frames=1)
img = f.color[..., ::-1].copy()
K = f.intrinsics.matrix()
for name, c in (("right_index_force_sensor_3", (0, 255, 0)), ("right_middle_force_sensor_3", (0, 255, 0)),
                ("right_pinky_force_sensor_3", (0, 255, 0)), ("right_palm_force_sensor", (255, 0, 255)),
                ("right_hand_base_link", (0, 0, 255)), ("right_thumb_force_sensor_4", (255, 255, 0))):
    p = r.tf(name, "world")[:3, 3]
    uv = s.project(p, K)
    cv2.circle(img, (int(uv[0]), int(uv[1])), 6, c, -1)
    cv2.putText(img, name.replace("right_", "").replace("_force_sensor", ""), (int(uv[0]) + 8, int(uv[1])),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, c, 1, cv2.LINE_AA)
H = r.tf("right_hand_base_link", "world")
print("hand y (palm normal) in world", np.round(H[:3, 1], 3).tolist(), "fingers", np.round(H[:3, 2], 3).tolist())
# palm normal arrow: 8 cm from the palm sensor
p0 = r.tf("right_palm_force_sensor", "world")[:3, 3]
a, b = s.project(p0, K), s.project(p0 + 0.08 * H[:3, 1], K)
cv2.arrowedLine(img, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])), (255, 0, 255), 2)
cv2.imwrite("/run_data/roll_check.jpg", img)
r.hand([0.0] * 6, 1.5)
r.close()
