"""Camera-to-robot calibration, docs/perception_architecture.md section 3.4.

A ChArUco board either lies on the table or is fixed to the hand, and every camera is either fixed in
the room or mounted on the hand. For every robot pose, the robot side (calibrate_cameras.py on the
Ubuntu PC) sends the flange pose F_i from TF and this module finds the board in each camera:

    T_cam_board_i = inverse(W_cam_i) * W_board_i
    W_board_i = T_world_board            (board on the table)   or  F_i * T_flange_board  (on the hand)
    W_cam_i   = T_world_cam              (camera fixed)         or  F_i * T_flange_cam    (on the hand)

The unknowns are the board pose and one pose per camera. A camera on the other side from the board
(hand camera and table board, or fixed camera and hand board) is solved in closed form with OpenCV
(Shah's method); a fixed camera that sees a table board follows from the board pose. All unknowns are
then refined together by minimising the pixel error of every board corner in every camera.
"""

from __future__ import annotations

import io
import json
import logging
import math
import os
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml
from PIL import Image
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

log = logging.getLogger("calibration")

MIN_CORNERS = 8  # ChArUco corners needed for a board pose
MIN_SAMPLES = 6  # samples per camera before solving
OUTLIER_FLOOR_PX = 2.0  # samples above max(this, 3 x median RMS) are dropped once
A4_MM = (210.0, 297.0)
BOARD_LOCATIONS = ("table", "hand")
CAMERA_MOUNTS = ("fixed", "hand")


def _env_map(variable, defaults):
    """serial=value pairs from an environment variable, on top of defaults."""
    values = dict(defaults)
    for item in filter(None, os.environ.get(variable, "").split(",")):
        serial, _, value = item.partition("=")
        values[serial.strip()] = value.strip()
    return values


# Serial to ROS camera name; defaults follow compose.realsense.yaml.
CAMERA_NAMES = _env_map("CAMERA_NAMES", {"263522073705": "camera_1", "262722075125": "camera_2"})
# Serial to mount, e.g. CAMERA_MOUNTS=262722075125=hand. Unlisted cameras are fixed.
DEFAULT_MOUNTS = _env_map("CAMERA_MOUNTS", {})


# ---------- geometry helpers ----------

def to_matrix(rotation, translation):
    """4x4 transform from a 3x3 rotation matrix or a Rodrigues vector, and a translation."""
    rotation = np.asarray(rotation, np.float64)
    if rotation.shape != (3, 3):
        rotation = cv2.Rodrigues(rotation.reshape(3))[0]
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = np.asarray(translation, np.float64).reshape(3)
    return T


def from_pose(translation, rotation_xyzw):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(rotation_xyzw).as_matrix()
    T[:3, 3] = translation
    return T


def to_pose(T):
    """Translation and xyzw quaternion, as plain lists for JSON and YAML."""
    return {
        "translation": [round(float(v), 6) for v in T[:3, 3]],
        "rotation_xyzw": [round(float(v), 8) for v in Rotation.from_matrix(T[:3, :3]).as_quat()],
    }


def inverse(T):
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def mean_transform(transforms):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_matrix(np.stack([t[:3, :3] for t in transforms])).mean().as_matrix()
    T[:3, 3] = np.mean([t[:3, 3] for t in transforms], axis=0)
    return T


def _params(T):
    return np.concatenate([cv2.Rodrigues(T[:3, :3])[0].ravel(), T[:3, 3]])


def _matrix(params):
    return to_matrix(params[:3], params[3:6])


def camera_matrix(intrinsics):
    """Pinhole matrix and OpenCV distortion. RealSense's inverse Brown-Conrady model is not OpenCV's;
    its coefficients are zero on the D435 colour stream, so it is treated as no distortion."""
    K = np.array([[intrinsics["fx"], 0.0, intrinsics["cx"]],
                  [0.0, intrinsics["fy"], intrinsics["cy"]],
                  [0.0, 0.0, 1.0]])
    coeffs = np.array(intrinsics.get("coeffs") or [0.0] * 5, np.float64)
    if "inverse" in str(intrinsics.get("model", "")):
        if np.any(coeffs):
            log.warning("Ignoring inverse Brown-Conrady coefficients %s", coeffs.tolist())
        coeffs = np.zeros(5)
    return K, coeffs


# ---------- board ----------

@dataclass
class BoardConfig:
    squares_x: int = 5
    squares_y: int = 7
    square_mm: float = 36.0  # measured on the printed board
    marker_mm: float = 27.0
    dictionary: str = "DICT_4X4_50"

    def build(self):
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.dictionary))
        return cv2.aruco.CharucoBoard((self.squares_x, self.squares_y), self.square_mm / 1000.0,
                                      self.marker_mm / 1000.0, dictionary)

    def validate(self):
        if not hasattr(cv2.aruco, self.dictionary):
            raise ValueError(f"Unknown ArUco dictionary {self.dictionary}")
        if not (3 <= self.squares_x <= 20 and 3 <= self.squares_y <= 20):
            raise ValueError("The board needs 3 to 20 squares per side")
        if not 0 < self.marker_mm < self.square_mm:
            raise ValueError("Markers must be smaller than the squares")


def printable_pdf(config, dpi=600):
    """The board centred on an A4 page at its exact size. Print it at 100 % (actual size)."""
    width_mm, height_mm = config.squares_x * config.square_mm, config.squares_y * config.square_mm
    page_mm = A4_MM if height_mm >= width_mm else A4_MM[::-1]
    if width_mm > page_mm[0] - 20 or height_mm > page_mm[1] - 30:
        raise ValueError(f"A {width_mm:.0f} x {height_mm:.0f} mm board does not fit on A4 with margins")

    def px(mm):
        return int(round(mm / 25.4 * dpi))

    board = config.build().generateImage((px(width_mm), px(height_mm)), marginSize=0, borderBits=1)
    page = np.full((px(page_mm[1]), px(page_mm[0])), 255, np.uint8)
    top, left = (page.shape[0] - board.shape[0]) // 2, (page.shape[1] - board.shape[1]) // 2
    page[top:top + board.shape[0], left:left + board.shape[1]] = board
    label = (f"ChArUco {config.squares_x}x{config.squares_y}, square {config.square_mm:g} mm, "
             f"marker {config.marker_mm:g} mm, {config.dictionary}. Print at 100 %, then measure a square.")
    scale = dpi / 300.0
    cv2.putText(page, label, (left, top + board.shape[0] + px(6)), cv2.FONT_HERSHEY_SIMPLEX,
                0.9 * scale, 0, max(1, round(2 * scale)), cv2.LINE_AA)
    image = Image.fromarray(page).point(lambda v: 255 if v > 127 else 0, mode="1")
    buf = io.BytesIO()
    image.save(buf, "PDF", resolution=dpi)  # bilevel pages are stored losslessly (CCITT G4)
    return buf.getvalue()


# ---------- detection ----------

class Detector:
    def __init__(self, config):
        self.config = config
        self.board = config.build()
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._detector = cv2.aruco.CharucoDetector(self.board, cv2.aruco.CharucoParameters(), params)

    def detect(self, color_rgb, intrinsics):
        """Board pose in the camera from one colour image. Returns (detection or None, raw markers).

        Any part of the board works: every ChArUco corner is identified by its neighbouring markers,
        so a hand covering part of the view only removes corners."""
        gray = cv2.cvtColor(color_rgb, cv2.COLOR_RGB2GRAY)
        corners, ids, marker_corners, marker_ids = self._detector.detectBoard(gray)
        raw = (corners, ids, marker_corners, marker_ids)
        if ids is None or len(ids) < MIN_CORNERS:
            return None, raw
        obj, img = self.board.matchImagePoints(corners, ids)
        K, dist = camera_matrix(intrinsics)
        ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            return None, raw
        rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, dist, rvec, tvec)
        projected, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        err = np.linalg.norm(projected.reshape(-1, 2) - img.reshape(-1, 2), axis=1)
        return {
            "ids": ids.ravel().astype(int).tolist(),
            "corners": [[round(float(u), 3), round(float(v), 3)] for u, v in corners.reshape(-1, 2)],
            "rvec": rvec.ravel().tolist(),
            "tvec": tvec.ravel().tolist(),
            "reproj_px": round(float(np.sqrt(np.mean(err ** 2))), 3),
            "markers": 0 if marker_ids is None else len(marker_ids),
            "intrinsics": intrinsics,
        }, raw

    def overlay(self, color_rgb, intrinsics):
        """JPEG of the frame with markers, corners and the board axes drawn, plus a short summary."""
        found, (corners, ids, marker_corners, marker_ids) = self.detect(color_rgb, intrinsics)
        image = cv2.cvtColor(color_rgb, cv2.COLOR_RGB2BGR)
        if marker_ids is not None:
            cv2.aruco.drawDetectedMarkers(image, marker_corners, marker_ids)
        if ids is not None:
            cv2.aruco.drawDetectedCornersCharuco(image, corners, ids, (0, 255, 0))
        total = (self.config.squares_x - 1) * (self.config.squares_y - 1)
        if found:
            K, dist = camera_matrix(intrinsics)
            cv2.drawFrameAxes(image, K, dist, np.array(found["rvec"]), np.array(found["tvec"]),
                              2 * self.config.square_mm / 1000.0, 3)
            distance = float(np.linalg.norm(found["tvec"]))
            text = f"board: {len(found['ids'])}/{total} corners, {found['reproj_px']:.2f} px, {distance:.2f} m"
            summary = {"found": True, "corners": len(found["ids"]), "reproj_px": found["reproj_px"],
                       "distance_m": round(distance, 3)}
        else:
            seen = 0 if ids is None else len(ids)
            text = f"board not found ({seen}/{total} corners, need {MIN_CORNERS})"
            summary = {"found": False, "corners": seen}
        cv2.putText(image, text, (16, 36), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 5, cv2.LINE_AA)
        cv2.putText(image, text, (16, 36), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (80, 220, 120) if found else (90, 90, 245), 2, cv2.LINE_AA)
        ok, buf = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return buf.tobytes(), summary


# ---------- session ----------

class _PlainDumper(yaml.SafeDumper):
    """Writes shared objects (such as one camera's intrinsics) out in full instead of as &aliases."""

    def ignore_aliases(self, data):
        return True


def _check_setup(board_location, mounts):
    if board_location not in BOARD_LOCATIONS:
        raise ValueError(f"board_location must be one of {BOARD_LOCATIONS}")
    bad = {k: v for k, v in mounts.items() if v not in CAMERA_MOUNTS}
    if bad:
        raise ValueError(f"Camera mounts must be one of {CAMERA_MOUNTS}, got {bad}")


class Session:
    """Samples of the current calibration run, kept in <root>/session.json across restarts.

    Samples hold only what was measured (flange pose and board detections), so the board location
    and the camera mounts can be changed at any time; they are used when solving."""

    def __init__(self, root):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.path = self.root / "session.json"
        self.lock = threading.Lock()
        self.result = None
        data = json.loads(self.path.read_text()) if self.path.exists() else {}
        self._load(data)

    def _load(self, data):
        board = BoardConfig(**data.get("board", {}))
        board.validate()
        setup = data.get("setup", {})
        location = setup.get("board_location", "table")
        mounts = {**DEFAULT_MOUNTS, **setup.get("mounts", {})}
        _check_setup(location, mounts)
        self.board, self.board_location, self.mounts = board, location, mounts
        self.samples = list(data.get("samples", []))
        self.detector = Detector(self.board)
        self.result = None

    def _save(self):
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.export()))
        tmp.replace(self.path)

    def export(self):
        return {"board": asdict(self.board),
                "setup": {"board_location": self.board_location, "mounts": self.mounts},
                "samples": self.samples}

    def replace(self, data):
        with self.lock:
            self._load(data)
            self._save()

    def set_board(self, config):
        config.validate()
        with self.lock:
            if self.samples:
                raise ValueError("Clear the samples before changing the board")
            self.board, self.detector, self.result = config, Detector(config), None
            self._save()

    def set_setup(self, board_location, mounts):
        _check_setup(board_location, mounts)
        with self.lock:
            self.board_location, self.mounts, self.result = board_location, dict(mounts), None
            self._save()

    def mount(self, serial):
        return self.mounts.get(serial, "fixed")

    def add(self, robot, detections):
        with self.lock:
            sample = {"id": max((s["id"] for s in self.samples), default=0) + 1,
                      "stamp": time.time(), "robot": robot, "cameras": detections}
            self.samples.append(sample)
            self.result = None
            self._save()
            return sample

    def remove(self, sample_id):
        with self.lock:
            before = len(self.samples)
            self.samples = [s for s in self.samples if s["id"] != sample_id]
            self.result = None
            self._save()
            return len(self.samples) < before

    def clear(self):
        with self.lock:
            self.samples, self.result = [], None
            self._save()

    def summary(self):
        with self.lock:
            return [{
                "id": s["id"],
                "stamp": s["stamp"],
                "flange": [round(v, 4) for v in s["robot"]["translation"]],
                "cameras": {serial: {"corners": len(d["ids"]), "reproj_px": d["reproj_px"],
                                     "distance_m": round(float(np.linalg.norm(d["tvec"])), 3)}
                            for serial, d in s["cameras"].items()},
            } for s in self.samples]

    def solve(self):
        with self.lock:
            data = self.export()
        result = solve(data)
        with self.lock:
            self.result = result
        return result

    def save_extrinsics(self):
        """Write extrinsics.yaml (the file perception loads) and a timestamped copy."""
        with self.lock:
            result = self.result
        if not result or not result["cameras"]:
            raise ValueError("Solve first; there is no calibrated camera to save")
        stamp = time.localtime(result["solved_at"])
        header = ("# Camera poses from the hand-eye calibration (docs/perception_architecture.md, section 3.4).\n"
                  "# Each camera entry is the TF transform parent_frame -> frame_id, the optical frame's pose in\n"
                  "# parent_frame: the robot's world frame for a fixed camera, the flange for a camera on the hand.\n")
        body = {
            "created": time.strftime("%Y-%m-%dT%H:%M:%S", stamp),
            "world_frame": result["world_frame"],
            "flange_frame": result["flange_frame"],
            "board": result["board"],
            "board_pose": result["board_pose"],
            "cameras": {serial: {k: cam[k] for k in (
                "name", "mount", "parent_frame", "frame_id", "translation", "rotation_xyzw", "samples_used",
                "samples_total", "reprojection_rms_px", "board_position_rms_mm", "board_rotation_rms_deg",
                "intrinsics")}
                for serial, cam in result["cameras"].items()},
        }
        text = header + yaml.dump(body, Dumper=_PlainDumper, sort_keys=False, default_flow_style=None)
        (self.root / "extrinsics.yaml").write_text(text)
        (self.root / f"extrinsics_{time.strftime('%Y%m%d-%H%M%S', stamp)}.yaml").write_text(text)
        return text


# ---------- solver ----------

def _closed_form(obs, camera_on_hand):
    """OpenCV's robot-world/hand-eye solve, AX = ZB with A = T_cam_board.

    Camera on the hand, board on the table: B = T_flange_world, X = T_board_world, Z = T_cam_flange.
    Camera fixed, board on the hand: B = T_world_flange, X = T_board_flange, Z = T_cam_world.
    Returns (board pose, camera pose) as (T_world_board, T_flange_cam) or (T_flange_board, T_world_cam).
    """
    robot = [inverse(T_wf) if camera_on_hand else T_wf for _, T_wf, _ in obs]
    R_x, t_x, R_z, t_z = cv2.calibrateRobotWorldHandEye(
        [cv2.Rodrigues(np.array(d["rvec"]))[0] for _, _, d in obs], [np.array(d["tvec"]) for _, _, d in obs],
        [T[:3, :3] for T in robot], [T[:3, 3] for T in robot], method=cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH)
    return inverse(to_matrix(R_x, t_x)), inverse(to_matrix(R_z, t_z))


def solve(data, min_samples=MIN_SAMPLES):
    """Camera poses from a session export. Cameras that cannot be solved are listed under skipped."""
    config = BoardConfig(**data["board"])
    setup = data.get("setup", {})
    location = setup.get("board_location", "table")
    mounts = {**DEFAULT_MOUNTS, **setup.get("mounts", {})}
    _check_setup(location, mounts)
    board_points = config.build().getChessboardCorners().astype(np.float64)
    samples = data["samples"]
    frames = {(s["robot"]["parent"], s["robot"]["child"]) for s in samples}
    if len(frames) > 1:
        raise ValueError(f"Samples mix robot frames {sorted(frames)}; clear them and record again")
    world, flange = frames.pop() if frames else ("world", "lara5_flange")

    # Per camera: (sample id, T_world_flange, detection)
    observations = {}
    for s in samples:
        T_wf = from_pose(s["robot"]["translation"], s["robot"]["rotation_xyzw"])
        for serial, det in s["cameras"].items():
            observations.setdefault(serial, []).append((s["id"], T_wf, det))

    skipped = {}
    for serial in list(observations):
        mount = mounts.get(serial, "fixed")
        if len(observations[serial]) < min_samples:
            skipped[serial] = f"{len(observations[serial])} samples, need {min_samples}"
        elif location == "hand" and mount == "hand":
            skipped[serial] = "it is on the hand like the board, so it sees the board in the same place at every pose"
        else:
            continue
        del observations[serial]
    # Cameras on the other side from the board anchor the solve; with the board on the table a fixed
    # camera only adds itself once the board has been located by a camera on the hand.
    anchors = [s for s in observations if (mounts.get(s, "fixed") == "hand") == (location == "table")]
    if not anchors:
        for serial in observations:
            skipped[serial] = ("with the board on the table, a fixed camera needs a camera on the hand "
                               "to locate the board" if location == "table" else "no anchor camera")
        observations = {}
    if not observations:
        raise ValueError("Nothing to solve: " + ("; ".join(f"{k}: {v}" for k, v in skipped.items()) or "no samples"))

    serials = sorted(observations)
    reference = max(anchors, key=lambda k: len(observations[k]))
    B0, _ = _closed_form(observations[reference], location == "table")
    initial = {}
    for serial in serials:
        on_hand = mounts.get(serial, "fixed") == "hand"
        if serial in anchors:
            initial[serial] = _closed_form(observations[serial], on_hand)[1]
        else:  # fixed camera, table board: T_world_cam = T_world_board * T_board_cam
            initial[serial] = mean_transform([B0 @ inverse(to_matrix(d["rvec"], d["tvec"]))
                                              for _, _, d in observations[serial]])

    def predicted(T_wf, B, C, serial):
        """Board pose in the camera for flange pose T_wf."""
        W_board = B if location == "table" else T_wf @ B
        W_cam = T_wf @ C if mounts.get(serial, "fixed") == "hand" else C
        return inverse(W_cam) @ W_board

    def unpack(x):
        return _matrix(x[:6]), {s: _matrix(x[6 + 6 * c:12 + 6 * c]) for c, s in enumerate(serials)}

    def residuals(x, active):
        B, C = unpack(x)
        return np.concatenate([_reprojection(predicted(T_wf, B, C[serial], serial), det, board_points).ravel()
                               for serial in serials for sid, T_wf, det in observations[serial]
                               if sid in active[serial]])

    def per_sample_rms(x):
        B, C = unpack(x)
        return {serial: {sid: float(np.sqrt(np.mean(np.sum(
                    _reprojection(predicted(T_wf, B, C[serial], serial), det, board_points) ** 2, axis=1))))
                         for sid, T_wf, det in observations[serial]} for serial in serials}

    def refine(x0, active):
        return least_squares(residuals, x0, args=(active,), loss="huber", f_scale=1.0, x_scale="jac").x

    x = np.concatenate([_params(B0)] + [_params(initial[s]) for s in serials])
    active = {serial: {sid for sid, _, _ in observations[serial]} for serial in serials}
    x = refine(x, active)

    # Drop gross outliers once (a wrong corner, a robot that had not settled) and refine again.
    rms = per_sample_rms(x)
    dropped = {serial: [] for serial in serials}
    for serial in serials:
        values = rms[serial]
        limit = max(OUTLIER_FLOOR_PX, 3.0 * float(np.median(list(values.values()))))
        bad = [sid for sid, v in values.items() if v > limit]
        if bad and len(values) - len(bad) >= min_samples:
            dropped[serial] = sorted(bad)
            active[serial] -= set(bad)
    if any(dropped.values()):
        x = refine(x, active)
        rms = per_sample_rms(x)

    B, C = unpack(x)
    # Errors are measured at the board centre, not at its corner origin.
    centre = np.array([config.squares_x * config.square_mm / 2000.0,
                       config.squares_y * config.square_mm / 2000.0, 0.0, 1.0])
    cameras = {}
    for serial in serials:
        mount = mounts.get(serial, "fixed")
        per_sample, pos_err, rot_err, pixels = [], [], [], []
        for sample_id, T_wf, det in observations[serial]:
            pred = predicted(T_wf, B, C[serial], serial)
            measured = to_matrix(det["rvec"], det["tvec"])
            d_mm = 1000.0 * float(np.linalg.norm((pred @ centre)[:3] - (measured @ centre)[:3]))
            d_deg = math.degrees(Rotation.from_matrix(pred[:3, :3].T @ measured[:3, :3]).magnitude())
            used = sample_id in active[serial]
            per_sample.append({"id": sample_id, "rms_px": round(rms[serial][sample_id], 3),
                               "position_mm": round(d_mm, 2), "rotation_deg": round(d_deg, 3), "used": used})
            if used:
                pos_err.append(d_mm)
                rot_err.append(d_deg)
                pixels.append(_reprojection(pred, det, board_points))
        pixels = np.concatenate(pixels)
        name = CAMERA_NAMES.get(serial, f"camera_{serial}")
        cameras[serial] = {
            "name": name,
            "mount": mount,
            "parent_frame": flange if mount == "hand" else world,
            "frame_id": f"{name}_color_optical_frame",
            **to_pose(C[serial]),
            "samples_used": len(active[serial]),
            "samples_total": len(observations[serial]),
            "dropped": dropped[serial],
            "reprojection_rms_px": round(float(np.sqrt(np.mean(np.sum(pixels ** 2, axis=1)))), 3),
            "board_position_rms_mm": round(float(np.sqrt(np.mean(np.square(pos_err)))), 2),
            "board_rotation_rms_deg": round(float(np.sqrt(np.mean(np.square(rot_err)))), 3),
            "distance_from_parent_origin_m": round(float(np.linalg.norm(C[serial][:3, 3])), 3),
            "intrinsics": observations[serial][0][2]["intrinsics"],
            "per_sample": per_sample,
        }
    return {
        "solved_at": time.time(),
        "world_frame": world,
        "flange_frame": flange,
        "board": asdict(config),
        "board_pose": {"location": location, "parent_frame": world if location == "table" else flange,
                       **to_pose(B)},
        "cameras": cameras,
        "skipped": skipped,
    }


def _reprojection(T_cam_board, det, board_points):
    """Pixel error of every detected corner when the board is at T_cam_board."""
    K, dist = camera_matrix(det["intrinsics"])
    rvec = cv2.Rodrigues(T_cam_board[:3, :3])[0]
    projected, _ = cv2.projectPoints(board_points[det["ids"]], rvec, T_cam_board[:3, 3], K, dist)
    return projected.reshape(-1, 2) - np.asarray(det["corners"], np.float64)
