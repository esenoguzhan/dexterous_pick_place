"""Move the 2026-10-26 hand-eye calibration into main's robot model.

The calibration was solved with the old URDF, whose joint-1 origin had no yaw. Main adds
joint1_zero_yaw (-50 deg, lara5_macro.xacro). For the same measured joints the flange is then
    F_new = M @ F_old,   M = T_world_link0 @ Rz(joint1_zero_yaw) @ inv(T_world_link0),
and every pose solved from flange poses moves the same way: the fixed camera and the board on the
table (world frame) become M @ T. The hand camera is relative to the flange and does not change.

Usage (in the container, controllers running):  convert_extrinsics.py OLD.yaml NEW.yaml
"""

import sys
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

import robot as rb

JOINT1_ZERO_YAW = -0.8726646259971648  # lara5_macro.xacro default in main


def matrix(t, q):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(q).as_matrix()
    T[:3, 3] = t
    return T


def entry(T):
    return ([round(float(v), 6) for v in T[:3, 3]],
            [round(float(v), 8) for v in Rotation.from_matrix(T[:3, :3]).as_quat()])


def main(old_path, new_path):
    data = yaml.safe_load(Path(old_path).read_text())
    r = rb.start("restore_convert_extrinsics")
    T_world_link0 = r.frame_pose("lara5_link0", "world")
    r.close()
    Rz = np.eye(4)
    Rz[:3, :3] = Rotation.from_euler("z", JOINT1_ZERO_YAW).as_matrix()
    M = T_world_link0 @ Rz @ np.linalg.inv(T_world_link0)
    print("arm base in world:", np.round(T_world_link0[:3, 3], 4).tolist())

    def convert(item):
        T = M @ matrix(item["translation"], item["rotation_xyzw"])
        item["translation"], item["rotation_xyzw"] = entry(T)

    if data.get("board_pose", {}).get("parent_frame") == "world":
        convert(data["board_pose"])
    for serial, cam in data["cameras"].items():
        if cam.get("parent_frame") == "world":
            convert(cam)
            print(serial, cam["name"], "->", cam["translation"])
    data["converted"] = (f"from {Path(old_path).name}: world poses multiplied by M = T_world_link0 Rz({JOINT1_ZERO_YAW}) "
                         f"T_world_link0^-1 for main's joint-1 zero")
    Path(new_path).write_text(yaml.safe_dump(data, sort_keys=False))
    print("written", new_path)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
