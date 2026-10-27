"""thumb_probe.py: set hand poses in free air and print thumb/finger tip positions in the hand frame."""
import time

import numpy as np

import robot as rb

r = rb.start("restore_thumb_probe")
tips = ["right_thumb_force_sensor_4", "right_index_force_sensor_3", "right_middle_force_sensor_3",
        "right_pinky_force_sensor_3"]


def show(label):
    time.sleep(0.5)
    print(label, "hand", np.round(r.hand_joints(), 2).tolist())
    for f in tips:
        print("   ", f"{f:28s}", np.round(r.tf(f, "right_hand_base_link")[:3, 3], 3).tolist())


show("open")
for pose in ([1.16, 0, 0, 0, 0, 0], [1.16, 0, 0, 0, 0, 0.5], [0.0, 1.3, 1.3, 1.3, 1.3, 0.0], [0.0, 0.8, 0.8, 0.8, 0.8, 0.0]):
    r.hand(pose, 1.5)
    show(str(pose))
r.hand([0, 0, 0, 0, 0, 0], 1.5)
show("open again")
r.close()
