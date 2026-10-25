"""Synthetic solver regressions; requires the perception app's Python dependencies."""

from dataclasses import asdict
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import calibration


class SolverMotionTests(unittest.TestCase):
    def session(self, stationary=False, translation_only=False):
        board = calibration.BoardConfig()
        points = board.build().getChessboardCorners().astype(np.float64)
        intrinsics = dict(fx=600, fy=600, cx=320, cy=240, coeffs=[0] * 5)
        K, dist = calibration.camera_matrix(intrinsics)
        board_pose = calibration.to_matrix([0.05, -0.03, 0.02], [0.02, -0.04, 1.0])
        camera_pose = calibration.to_matrix([0.04, 0.02, -0.03], [0.02, 0, 0.1])
        rotations = [[0, 0, 0], [.12, 0, 0], [0, .14, 0], [0, 0, .13],
                     [-.13, .1, 0], [.06, -.15, .05], [.15, .07, -.05], [0, .05, -.1]]
        samples = []
        for i, rot in enumerate(rotations):
            flange = calibration.to_matrix([0, 0, 0] if stationary or translation_only else rot,
                                          [0, 0, 0] if stationary else [i * .005, (i % 3) * .01, 0])
            observed = calibration.inverse(flange @ camera_pose) @ board_pose
            rvec = cv2.Rodrigues(observed[:3, :3])[0].ravel()
            tvec = observed[:3, 3]
            corners = cv2.projectPoints(points, rvec, tvec, K, dist)[0].reshape(-1, 2)
            det = dict(ids=list(range(len(points))), corners=corners.tolist(), rvec=rvec.tolist(),
                       tvec=tvec.tolist(), reproj_px=0.0, markers=10, intrinsics=intrinsics)
            samples.append(dict(id=i + 1, robot=dict(parent="world", child="lara5_flange",
                                translation=flange[:3, 3].tolist(),
                                rotation_xyzw=Rotation.from_matrix(flange[:3, :3]).as_quat().tolist()),
                                cameras={"test_hand_camera": det}))
        return dict(board=asdict(board), setup=dict(board_location="table", mounts={"test_hand_camera": "hand"}),
                    samples=samples), camera_pose

    def test_stationary_session_is_rejected(self):
        data, _ = self.session(stationary=True)
        with self.assertRaisesRegex(ValueError, "orientations barely change"):
            calibration.solve(data)

    def test_translation_without_rotation_is_rejected(self):
        data, _ = self.session(translation_only=True)
        with self.assertRaisesRegex(ValueError, "orientations barely change"):
            calibration.solve(data)

    def test_varied_poses_recover_known_camera_transform(self):
        data, expected = self.session()
        result = calibration.solve(data)["cameras"]["test_hand_camera"]
        measured = calibration.from_pose(result["translation"], result["rotation_xyzw"])
        np.testing.assert_allclose(measured, expected, atol=1e-5)
        self.assertLess(result["reprojection_rms_px"], 0.01)


if __name__ == "__main__":
    unittest.main()
