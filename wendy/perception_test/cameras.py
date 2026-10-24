"""RealSense D435 capture (colour plus depth aligned to colour), with a plain V4L2 colour fallback."""

from __future__ import annotations

import glob
import logging
import os
import re
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np

log = logging.getLogger("cameras")

# Section 3.3 of the architecture: 1280x720 colour, 848x480 depth aligned to colour, laser power 250.
COLOR_SIZE = (int(os.environ.get("COLOR_WIDTH", "1280")), int(os.environ.get("COLOR_HEIGHT", "720")))
DEPTH_SIZE = (848, 480)
FPS = int(os.environ.get("CAMERA_FPS", "15"))
LASER_POWER = float(os.environ.get("LASER_POWER", "250"))
DEPTH_VIEW_MAX_M = 2.0

try:
    import pyrealsense2 as rs
except ImportError as exc:
    rs = None
    log.warning("pyrealsense2 is unavailable: %s", exc)


@dataclass
class Frame:
    color: np.ndarray  # RGB, uint8
    depth: np.ndarray | None  # raw depth units, uint16, on the colour pixel grid
    stamp: float
    seq: int


def colorize_depth(depth, scale):
    metres = depth.astype(np.float32) * (scale or 0.001)
    scaled = np.clip(metres / DEPTH_VIEW_MAX_M, 0.0, 1.0)
    image = cv2.applyColorMap((255 * (1.0 - scaled)).astype(np.uint8), cv2.COLORMAP_TURBO)
    image[depth == 0] = 0
    return image


class Camera:
    """Latest-frame buffer and JPEG cache. Subclasses fill it from their own capture thread."""

    kind = ""

    def __init__(self, cam_id, name):
        self.id = cam_id
        self.name = name
        self.online = False
        self.error = None
        self.depth_scale = None
        self.intrinsics = None
        self.details = {}
        self._frame = None
        self._seq = 0
        self._fps = 0.0
        self._jpeg = {}
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, daemon=True, name=f"camera-{cam_id}")

    def start(self):
        self._thread.start()

    def _run(self):
        raise NotImplementedError

    def _publish(self, color, depth):
        now = time.time()
        with self._lock:
            if self._frame is not None and now > self._frame.stamp:
                self._fps = 0.9 * self._fps + 0.1 / (now - self._frame.stamp)
            self._seq += 1
            self._frame = Frame(color, depth, now, self._seq)

    def latest(self):
        with self._lock:
            return self._frame

    def jpeg(self, view="color"):
        frame = self.latest()
        if frame is None:
            return None, 0
        cached = self._jpeg.get(view)
        if cached and cached[0] == frame.seq:
            return cached[1], frame.seq
        if view == "depth" and frame.depth is not None:
            image = colorize_depth(frame.depth, self.depth_scale)
        else:
            image = cv2.cvtColor(frame.color, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        data = buf.tobytes()
        self._jpeg[view] = (frame.seq, data)
        return data, frame.seq

    def describe(self):
        frame = self.latest()
        return {
            "id": self.id,
            "name": self.name,
            "kind": self.kind,
            "online": self.online,
            "error": self.error,
            "fps": round(self._fps, 1),
            "has_depth": frame is not None and frame.depth is not None,
            "width": frame.color.shape[1] if frame else None,
            "height": frame.color.shape[0] if frame else None,
            **self.details,
        }


class RealSenseCamera(Camera):
    kind = "realsense"

    def __init__(self, serial, name, usb, firmware):
        super().__init__(serial, name)
        self.details = {"serial": serial, "usb": usb, "firmware": firmware}

    def _open(self):
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(self.id)
        config.enable_stream(rs.stream.color, COLOR_SIZE[0], COLOR_SIZE[1], rs.format.rgb8, FPS)
        config.enable_stream(rs.stream.depth, DEPTH_SIZE[0], DEPTH_SIZE[1], rs.format.z16, FPS)
        profile = pipeline.start(config)
        sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = sensor.get_depth_scale()
        if sensor.supports(rs.option.laser_power):
            limits = sensor.get_option_range(rs.option.laser_power)
            sensor.set_option(rs.option.laser_power, min(LASER_POWER, limits.max))
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.intrinsics = {"fx": intr.fx, "fy": intr.fy, "cx": intr.ppx, "cy": intr.ppy,
                           "width": intr.width, "height": intr.height}
        return pipeline, rs.align(rs.stream.color)

    def _run(self):
        while True:
            try:
                pipeline, align = self._open()
            except Exception as exc:
                self.online, self.error = False, f"open failed: {exc}"
                log.warning("RealSense %s: %s", self.id, self.error)
                time.sleep(3)
                continue
            self.online, self.error = True, None
            log.info("RealSense %s streaming %dx%d colour at %d fps", self.id, *COLOR_SIZE, FPS)
            try:
                while True:
                    frames = align.process(pipeline.wait_for_frames(5000))
                    color, depth = frames.get_color_frame(), frames.get_depth_frame()
                    if color and depth:
                        self._publish(np.asanyarray(color.get_data()).copy(),
                                      np.asanyarray(depth.get_data()).copy())
            except Exception as exc:
                self.online, self.error = False, f"stream stopped: {exc}"
                log.warning("RealSense %s: %s", self.id, self.error)
            finally:
                try:
                    pipeline.stop()
                except Exception:
                    pass
            time.sleep(1)


class V4L2Camera(Camera):
    """Colour only, for when the RealSense SDK cannot see the camera."""

    kind = "v4l2"

    def __init__(self, path):
        super().__init__(os.path.basename(path), f"V4L2 {path}")
        self.path = path
        self.details = {"path": path}

    def _run(self):
        while True:
            cap = cv2.VideoCapture(self.path, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, COLOR_SIZE[0])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, COLOR_SIZE[1])
            cap.set(cv2.CAP_PROP_FPS, FPS)
            failures = 0
            while cap.isOpened() and failures < 30:
                ok, frame = cap.read()
                if ok and frame is not None and frame.ndim == 3:
                    self.online, self.error, failures = True, None, 0
                    self._publish(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), None)
                else:
                    failures += 1
                    time.sleep(0.05)
            cap.release()
            self.online, self.error = False, "no frames"
            time.sleep(2)


def _video_index(path):
    match = re.search(r"(\d+)$", path)
    return int(match.group(1)) if match else 0


def _find_v4l2_color():
    for path in sorted(glob.glob("/dev/video*"), key=_video_index):
        cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
        try:
            ok, frame = cap.read()
        finally:
            cap.release()
        if ok and frame is not None and frame.ndim == 3 and frame.shape[2] == 3 and frame.std() > 1.0:
            return path
    return None


class CameraManager:
    def __init__(self):
        self._cameras = {}
        self._lock = threading.Lock()
        self.error = None

    def rescan(self):
        found = []
        if rs is None:
            self.error = "pyrealsense2 is not installed"
        else:
            try:
                for dev in rs.context().query_devices():
                    def info(key):
                        return dev.get_info(key) if dev.supports(key) else ""

                    found.append((info(rs.camera_info.serial_number), info(rs.camera_info.name),
                                  info(rs.camera_info.usb_type_descriptor), info(rs.camera_info.firmware_version)))
                self.error = None if found else "RealSense SDK found no cameras"
            except Exception as exc:
                self.error = f"RealSense query failed: {exc}"
        if self.error:
            log.warning(self.error)

        with self._lock:
            for serial, name, usb, firmware in found:
                if serial and serial not in self._cameras:
                    camera = RealSenseCamera(serial, name, usb, firmware)
                    self._cameras[serial] = camera
                    camera.start()
            if not self._cameras and os.environ.get("V4L2_FALLBACK", "1") == "1":
                path = _find_v4l2_color()
                if path:
                    log.info("Falling back to V4L2 colour on %s", path)
                    camera = V4L2Camera(path)
                    self._cameras[camera.id] = camera
                    camera.start()
        return self.list()

    def list(self):
        with self._lock:
            return [c.describe() for c in self._cameras.values()]

    def get(self, cam_id):
        with self._lock:
            return self._cameras.get(cam_id)

    def first_online(self):
        with self._lock:
            return next((c for c in self._cameras.values() if c.online), None)
