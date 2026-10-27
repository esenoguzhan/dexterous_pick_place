"""Camera frames from the Jetson over HTTP (wendy/perception_test app, port 8090).

The Jetson drops UDP on the USB-C link, so DDS cannot cross it, but the app's declared HTTP port
can. The app streams both D435s and returns, per capture, the newest colour image and the
per-pixel median of N depth frames aligned to colour, which is what restore_perception expects.
"""

from __future__ import annotations

import io
import json
import os
import time
import urllib.request
import zipfile

import cv2
import numpy as np

from restore_perception.core.capture import clip_depth
from restore_perception.core.types import CameraFrame, Intrinsics, PerceptionError

JETSON = os.environ.get("RESTORE_JETSON", "http://169.254.211.202:8090")
FIXED = "263522073705"  # camera_1, on the stand
WRIST = "262722075125"  # camera_2, on the hand
NAMES = {FIXED: "camera_1", WRIST: "camera_2"}


def _request(path, body=None, timeout=60.0):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(JETSON + path, data=data,
                                 headers={"Content-Type": "application/json"} if data else {})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as res:
            return res.read()
    except Exception as exc:
        raise PerceptionError(f"Jetson {path}: {exc}") from exc


def status():
    return json.loads(_request("/api/status", timeout=10.0))


def capture(serial=FIXED, frames=10, min_depth=0.15, max_depth=2.0, attempts=4) -> CameraFrame:
    """One capture: newest colour, median depth [m] aligned to colour, intrinsics. No pose.

    The wrist camera runs on USB 2 and sometimes stops after a reopen; a rescan brings it back."""
    for attempt in range(attempts):
        try:
            meta = json.loads(_request("/api/capture", {"camera": serial, "frames": int(frames)}))
            break
        except PerceptionError:
            if attempt == attempts - 1:
                raise
            try:
                _request("/api/cameras/rescan", {}, timeout=40.0)
            except PerceptionError:
                pass
            time.sleep(3.0)
    stamp = time.time()
    archive = zipfile.ZipFile(io.BytesIO(_request(f"/api/captures/{meta['id']}/capture.zip")))
    color = cv2.imdecode(np.frombuffer(archive.read("color.png"), np.uint8), cv2.IMREAD_COLOR)[..., ::-1]
    depth_mm = cv2.imdecode(np.frombuffer(archive.read("depth.png"), np.uint8), cv2.IMREAD_UNCHANGED)
    info = json.loads(archive.read("intrinsics.json"))
    i = info["intrinsics"]
    intr = Intrinsics(float(i["fx"]), float(i["fy"]), float(i.get("ppx", i.get("cx"))), float(i.get("ppy", i.get("cy"))),
                      int(info["width"]), int(info["height"]), str(i.get("model", "")),
                      list(i.get("coeffs") or [0.0] * 5))
    depth = clip_depth(depth_mm.astype(np.float32) / 1000.0, min_depth, max_depth)
    return CameraFrame(NAMES.get(serial, f"camera_{serial}"), serial, np.ascontiguousarray(color), depth, intr,
                       stamp, frames=int(meta.get("frames", frames)))
