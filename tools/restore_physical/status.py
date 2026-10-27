"""Print the measured arm joints, flange pose in world and hand joints."""
import numpy as np
from scipy.spatial.transform import Rotation

import robot as rb

r = rb.start("restore_status")
q = r.arm_joints()
T = r.frame_pose("lara5_flange", "world")
print("arm joints [rad]:", np.round(q, 4).tolist())
print("flange xyz [m]:", np.round(T[:3, 3], 4).tolist(), "rpy [deg]:",
      np.round(Rotation.from_matrix(T[:3, :3]).as_euler("xyz", degrees=True), 1).tolist())
H = r.frame_pose("right_hand_base_link", "world")
print("hand base xyz:", np.round(H[:3, 3], 4).tolist(), "z axis:", np.round(H[:3, 2], 3).tolist())
print("hand joints:", np.round(r.hand_joints(), 3).tolist())
r.close()
