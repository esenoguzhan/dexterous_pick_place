"""Keep the fixed camera calibrated when the robot and its table move together under the camera.

    board_shift.py before    board pose in world (robot frame) from the current calibration
    board_shift.py after     new camera pose = saved board pose (it moved with the robot) * the board
                             seen from the camera now; writes config/extrinsics.yaml (old one kept)

The ChArUco board must lie on the robot's table and not slide while it moves; the camera stand
must not move.
"""

import json
import shutil
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

import jetson

HERE = Path(__file__).resolve().parent
EXTRINSICS = HERE / "config" / "extrinsics.yaml"
SAVED = Path("/run_data/board_before.json")


def board_in_camera(cal):
    b = cal["board"]
    frame = jetson.capture(jetson.FIXED, frames=3)
    gray = cv2.cvtColor(frame.color, cv2.COLOR_RGB2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, b["dictionary"]))
    board = cv2.aruco.CharucoBoard_create(b["squares_x"], b["squares_y"], b["square_mm"] / 1000.0,
                                          b["marker_mm"] / 1000.0, dictionary)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    if ids is None:
        sys.exit("no board markers in the fixed camera")
    n, ch, chid = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
    if not n or n < 10:
        sys.exit(f"only {n} board corners: is the board visible and not covered?")
    obj = board.chessboardCorners[chid.ravel()].astype(np.float64)
    K = frame.intrinsics.matrix()
    ok, rvec, tvec = cv2.solvePnP(obj, ch.reshape(-1, 2), K, np.zeros(5), flags=cv2.SOLVEPNP_IPPE)
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, np.zeros(5))
    rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - ch.reshape(-1, 2)) ** 2, axis=1))))
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(rvec)[0]
    T[:3, 3] = tvec.ravel()
    return T, int(n), rms


def matrix(t, q):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(q).as_matrix()
    T[:3, 3] = t
    return T


def main(step):
    cal = yaml.safe_load(EXTRINSICS.read_text())
    cam = cal["cameras"][jetson.FIXED]
    T_world_cam = matrix(cam["translation"], cam["rotation_xyzw"])
    T_cam_board, n, rms = board_in_camera(cal)
    print(f"{n} corners, {rms:.2f} px")
    if step == "before":
        T_world_board = T_world_cam @ T_cam_board
        SAVED.write_text(json.dumps({"T_world_board": T_world_board.tolist(), "stamp": time.time()}))
        print("board in world", np.round(T_world_board[:3, 3], 4).tolist(), "saved")
        return
    T_world_board = np.array(json.loads(SAVED.read_text())["T_world_board"])
    T_new = T_world_board @ np.linalg.inv(T_cam_board)
    shift = T_new[:3, 3] - T_world_cam[:3, 3]
    turn = np.degrees(Rotation.from_matrix(T_new[:3, :3] @ T_world_cam[:3, :3].T).magnitude())
    print(f"camera moved {1000 * np.linalg.norm(shift):.0f} mm {np.round(shift, 4).tolist()} and {turn:.2f} deg "
          f"relative to the robot")
    backup = EXTRINSICS.with_name(f"extrinsics_before_shift_{time.strftime('%Y%m%d_%H%M%S')}.yaml")
    shutil.copy(EXTRINSICS, backup)
    cam["translation"] = [round(float(v), 6) for v in T_new[:3, 3]]
    cam["rotation_xyzw"] = [round(float(v), 8) for v in Rotation.from_matrix(T_new[:3, :3]).as_quat()]
    cal["shifted"] = f"{time.strftime('%Y-%m-%d %H:%M')}: robot and table moved; camera re-posed from the board"
    EXTRINSICS.write_text(yaml.safe_dump(cal, sort_keys=False))
    print("written", EXTRINSICS, "(old one in", backup.name + ")")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "before")
