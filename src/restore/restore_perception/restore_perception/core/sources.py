"""Frame sources (section 3.2): the real D435s and saved captures. RosTopicSource lives in node.py.

`capture(frames)` returns one CameraFrame per camera with the median depth of the next `frames`
framesets (5.1). Poses are not set here: the caller adds them from the calibration file and TF.
"""

from __future__ import annotations

import logging
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

from . import snapshot_io
from .capture import clip_depth, median_depth
from .types import CameraFrame, Intrinsics, PerceptionError

log = logging.getLogger("restore_perception.sources")


class FrameSource:
    def capture(self, frames):
        raise NotImplementedError

    def close(self):
        pass

    def status(self):
        """Camera name -> short state, for diagnostics."""
        return {}


class _RealSenseCamera:
    """Streams one D435 and aligns depth to colour only for the framesets a capture asks for (3.3)."""

    def __init__(self, rs, serial, name, cfg):
        self.rs, self.serial, self.name, self.cfg = rs, serial, name, cfg
        self.online = False
        self.error = None
        self.depth_scale = 0.001
        self.intrinsics = None
        self._want = 0
        self._got = []
        self._cond = threading.Condition()
        self._thread = threading.Thread(target=self._run, daemon=True, name=f"realsense-{serial}")
        self._stop = threading.Event()

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()

    def _open(self):
        rs, cfg = self.rs, self.cfg
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(self.serial)
        (cw, ch), (dw, dh), fps = cfg["color_size"], cfg["depth_size"], int(cfg["fps"])
        config.enable_stream(rs.stream.color, cw, ch, rs.format.rgb8, fps)
        config.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, fps)
        profile = pipeline.start(config)
        sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = sensor.get_depth_scale()
        if sensor.supports(rs.option.laser_power):
            limits = sensor.get_option_range(rs.option.laser_power)
            sensor.set_option(rs.option.laser_power, min(float(cfg["laser_power"]), limits.max))
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.intrinsics = Intrinsics(intr.fx, intr.fy, intr.ppx, intr.ppy, intr.width, intr.height,
                                     str(intr.model), [float(c) for c in intr.coeffs])
        return pipeline, rs.align(rs.stream.color)

    def _run(self):
        while not self._stop.is_set():
            try:
                pipeline, align = self._open()
            except Exception as exc:
                self.online, self.error = False, f"open failed: {exc}"
                log.warning("RealSense %s: %s", self.serial, self.error)
                time.sleep(3)
                continue
            self.online, self.error = True, None
            log.info("RealSense %s (%s) streaming", self.serial, self.name)
            try:
                while not self._stop.is_set():
                    frames = pipeline.wait_for_frames(5000)
                    with self._cond:
                        if self._want <= 0:
                            continue
                    frames = align.process(frames)
                    color, depth = frames.get_color_frame(), frames.get_depth_frame()
                    if not (color and depth):
                        continue
                    item = (np.asanyarray(color.get_data()).copy(), np.asanyarray(depth.get_data()).copy())
                    with self._cond:
                        if self._want > 0:
                            self._got.append(item)
                            self._want -= 1
                            self._cond.notify_all()
            except Exception as exc:
                self.online, self.error = False, f"stream stopped: {exc}"
                log.warning("RealSense %s: %s", self.serial, self.error)
                with self._cond:
                    self._cond.notify_all()
            finally:
                try:
                    pipeline.stop()
                except Exception:
                    pass
            time.sleep(1)

    def collect(self, n, timeout):
        with self._cond:
            if not self.online:
                raise PerceptionError(f"{self.name} ({self.serial}) is offline: {self.error}")
            self._got, self._want = [], n
            done = self._cond.wait_for(lambda: self._want <= 0 or not self.online, timeout)
            got, self._want = self._got, 0
        if not done or len(got) < n:
            raise PerceptionError(f"{self.name}: only {len(got)} of {n} framesets within {timeout:.0f} s")
        return got


class RealSenseSource(FrameSource):
    """Both D435s, read directly with pyrealsense2 on the Jetson."""

    def __init__(self, config):
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise PerceptionError(f"pyrealsense2 is not installed: {exc}") from exc
        self.cfg = config["capture"]
        names = config["cameras"]
        self.cameras = []
        for dev in rs.context().query_devices():
            serial = dev.get_info(rs.camera_info.serial_number)
            camera = _RealSenseCamera(rs, serial, names.get(serial, f"camera_{serial}"), self.cfg)
            camera.start()
            self.cameras.append(camera)
        if not self.cameras:
            raise PerceptionError("the RealSense SDK found no cameras")
        self._pool = ThreadPoolExecutor(max_workers=len(self.cameras))

    def wait_online(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not all(c.online for c in self.cameras):
            time.sleep(0.1)
        return [c.name for c in self.cameras if not c.online]

    def status(self):
        return {c.name: "online" if c.online else (c.error or "starting") for c in self.cameras}

    def capture(self, frames):
        timeout = float(self.cfg["timeout_s"])
        online = [c for c in self.cameras if c.online]
        if not online:
            raise PerceptionError("no camera is streaming: " + ", ".join(f"{k}: {v}" for k, v in self.status().items()))
        stamp = time.time()
        jobs = [self._pool.submit(c.collect, frames, timeout) for c in online]  # both cameras in parallel
        result = []
        for camera, job in zip(online, jobs):
            sets = job.result()
            depth = median_depth([d for _, d in sets], self.cfg["min_valid_fraction"]) * camera.depth_scale
            depth = clip_depth(depth, self.cfg["min_depth_m"], self.cfg["max_depth_m"])
            result.append(CameraFrame(camera.name, camera.serial, sets[-1][0], depth, camera.intrinsics,
                                      stamp, frames=frames))
        return result

    def close(self):
        for camera in self.cameras:
            camera.stop()
        self._pool.shutdown(wait=False)


class FileSource(FrameSource):
    """Saved captures: a snapshot folder (8.3), a test bench download, or a folder of those.

    Each capture() returns the next capture in name order and starts over after the last one.
    """

    def __init__(self, path, config):
        self.root = Path(path)
        self.names = config["cameras"]
        if snapshot_io.is_capture(self.root):
            self.captures = [self.root]
        elif self.root.is_dir():
            self.captures = sorted(p for p in self.root.iterdir() if snapshot_io.is_capture(p))
        else:
            self.captures = []
        if not self.captures:
            raise PerceptionError(f"no saved capture under {self.root}")
        self._next = 0
        self.last_path = None

    def status(self):
        return {"file": f"{len(self.captures)} capture(s) under {self.root}"}

    def capture(self, frames=None):
        path = self.captures[self._next % len(self.captures)]
        self._next += 1
        self.last_path = path
        return snapshot_io.load_frames(path, self.names)
