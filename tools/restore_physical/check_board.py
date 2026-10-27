"""Check the fixed camera's calibration against the ChArUco board lying on the table.

Detects the board in a fresh fixed-camera image, moves its centre and normal into world with the
calibration file, and compares them with the board pose stored in the same file. A camera or board
that moved since the calibration shows up as a large difference.

Usage: check_board.py [config/extrinsics.yaml]
"""

import sys
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

import jetson

path = Path(sys.argv[1] if len(sys.argv) > 1 else "config/extrinsics.yaml")
cal = yaml.safe_load(path.read_text())
b = cal["board"]
cam = cal["cameras"][jetson.FIXED]


def matrix(t, q):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(q).as_matrix()
    T[:3, 3] = t
    return T


T_world_cam = matrix(cam["translation"], cam["rotation_xyzw"])
T_world_board = matrix(cal["board_pose"]["translation"], cal["board_pose"]["rotation_xyzw"])

frame = jetson.capture(jetson.FIXED, frames=3)
gray = cv2.cvtColor(frame.color, cv2.COLOR_RGB2GRAY)
dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, b["dictionary"]))
board = cv2.aruco.CharucoBoard_create(b["squares_x"], b["squares_y"], b["square_mm"] / 1000.0,
                                      b["marker_mm"] / 1000.0, dictionary)
corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
if ids is None:
    sys.exit("no markers found")
n, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
if not n or n < 6:
    sys.exit(f"only {n} ChArUco corners")
K = frame.intrinsics.matrix()
obj = board.chessboardCorners[ch_ids.ravel()].astype(np.float64)
ok, rvec, tvec = cv2.solvePnP(obj, ch_corners.reshape(-1, 2), K, np.zeros(5), flags=cv2.SOLVEPNP_IPPE)
proj, _ = cv2.projectPoints(obj, rvec, tvec, K, np.zeros(5))
rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - ch_corners.reshape(-1, 2)) ** 2, axis=1))))
T_cam_board = np.eye(4)
T_cam_board[:3, :3] = cv2.Rodrigues(rvec)[0]
T_cam_board[:3, 3] = tvec.ravel()

# Board centre and normal: independent of which corner the board frame starts at.
all_corners = np.c_[board.chessboardCorners, np.ones(len(board.chessboardCorners))]
centre_now = (T_world_cam @ T_cam_board @ all_corners.T)[:3].mean(axis=1)
normal_now = (T_world_cam @ T_cam_board)[:3, 2]
w, h = b["squares_x"] * b["square_mm"] / 1000.0, b["squares_y"] * b["square_mm"] / 1000.0
candidates = [T_world_board @ np.array([sx * w / 2, sy * h / 2, 0, 1]) for sx in (1, -1) for sy in (1, -1)]
centre_cal = min(candidates, key=lambda c: np.linalg.norm(c[:3] - centre_now))[:3]
angle = np.degrees(np.arccos(abs(float(normal_now @ T_world_board[:3, 2]))))

print(f"{n} corners, reprojection rms {rms:.2f} px")
print("board centre now [m]:", np.round(centre_now, 4).tolist())
print("board centre in calibration [m]:", np.round(centre_cal, 4).tolist())
print(f"difference {1000 * np.linalg.norm(centre_now - centre_cal):.1f} mm, normals {angle:.2f} deg apart, "
      f"normal now {np.round(normal_now, 3).tolist()}")
