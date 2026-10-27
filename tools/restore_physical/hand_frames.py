"""Print hand link origins in the right_hand_base_link frame (open hand, current joints)."""
import numpy as np

import robot as rb

r = rb.start("restore_hand_frames")
base = np.linalg.inv(r.frame_pose("right_hand_base_link", "world"))
for f in ["lara5_flange", "right_palm_force_sensor", "right_thumb_yaw_link", "right_thumb_distal_link",
          "right_thumb_force_sensor_4", "right_index_proximal_link", "right_index_distal_link",
          "right_index_force_sensor_3", "right_middle_force_sensor_3", "right_pinky_proximal_link",
          "right_pinky_force_sensor_3", "right_wrist_camera_holder", "camera_2_link", "lara5_gripper_tcp"]:
    try:
        T = base @ r.frame_pose(f, "world")
        print(f"{f:34s}", np.round(T[:3, 3], 4).tolist())
    except Exception as e:
        print(f, "?", e)
r.close()
