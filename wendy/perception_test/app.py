"""Browser test bench for the SAM 2.1 and DINOv2 stages of docs/perception_architecture.md."""

from __future__ import annotations

import asyncio
import base64
import io
import itertools
import json
import logging
import os
import threading
import time
import zipfile
from collections import OrderedDict
from dataclasses import asdict, dataclass, field
from pathlib import Path

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, Response, StreamingResponse
from pydantic import BaseModel

import calibration
from cameras import MIN_VALID_FRACTION, CameraManager, median_depth
from vision import Gallery, Models, mask_bbox

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
log = logging.getLogger("app")

PORT = int(os.environ.get("PORT", "8090"))
DATA_DIR = Path(os.environ.get("DATA_DIR", "/data"))
STATIC_DIR = Path(__file__).parent / "static"
STREAM_FPS = 15
MAX_CAPTURES = 4
MAX_AUTO_MASKS = 64
PALETTE = [
    (79, 157, 255), (62, 207, 142), (245, 165, 36), (244, 91, 105), (167, 139, 250),
    (45, 212, 191), (250, 204, 21), (236, 72, 153), (132, 204, 22), (251, 146, 60),
    (56, 189, 248), (192, 132, 252),
]

app = FastAPI(title="Perception test")
models = Models()
cameras = CameraManager()
gallery = Gallery(DATA_DIR / "gallery")
calib = calibration.Session(DATA_DIR / "calibration")
_board_pdf = {}  # printable board per board config


@dataclass
class MaskItem:
    mask: np.ndarray
    embedding: np.ndarray
    crop: np.ndarray


@dataclass
class Capture:
    id: str
    camera: str
    color: np.ndarray  # RGB
    depth_m: np.ndarray | None  # metres, aligned to colour, 0 where invalid
    intrinsics: dict | None
    stamp: float  # arrival time of the newest frame
    frames: int  # depth frames in the per-pixel median (section 5.1)
    prompt: list = field(default_factory=list)
    auto: list = field(default_factory=list)


_captures: OrderedDict[str, Capture] = OrderedDict()
_captures_lock = threading.Lock()
_capture_ids = itertools.count(1)


def _get_capture(capture_id):
    with _captures_lock:
        cap = _captures.get(capture_id)
    if cap is None:
        raise HTTPException(404, f"Capture {capture_id} is gone; capture a new frame")
    return cap


def _require_models():
    if models.state != "ready":
        raise HTTPException(409, f"Models are {models.state}" + (f": {models.error}" if models.error else ""))


def _png_data_url(rgba):
    ok, buf = cv2.imencode(".png", cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGRA))
    return "data:image/png;base64," + base64.b64encode(buf.tobytes()).decode()


def _overlay(masks, colors, alpha=110):
    """Transparent PNG with filled masks and solid outlines, the size of the capture."""
    h, w = masks[0].shape
    rgba = np.zeros((h, w, 4), np.uint8)
    for mask, color in zip(masks, colors):
        rgba[mask] = (*color, alpha)
    for mask, color in zip(masks, colors):
        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(rgba, contours, -1, (*color, 255), 2)
    return _png_data_url(rgba)


def _describe(cap, mask):
    area = int(mask.sum())
    info = {"area": area, "bbox": list(mask_bbox(mask))}
    if cap.depth_m is not None:
        z = cap.depth_m[mask]
        z = z[z > 0]
        info["depth_m"] = round(float(np.median(z)), 3) if z.size else None
        info["depth_valid"] = round(z.size / area, 2)
    return info


def _matches(items, assign):
    if not items:
        return []
    return gallery.match(np.stack([it.embedding for it in items]), assign=assign)


def _download_name(cap, suffix):
    """Unique across restarts, unlike the capture id: local time of the capture plus camera serial."""
    return f"{time.strftime('%Y%m%d-%H%M%S', time.localtime(cap.stamp))}_{cap.camera}_{suffix}"


def _attachment(cap, suffix):
    return {"Content-Disposition": f'attachment; filename="{_download_name(cap, suffix)}"'}


def _color_png(cap):
    ok, buf = cv2.imencode(".png", cv2.cvtColor(cap.color, cv2.COLOR_RGB2BGR))
    return buf.tobytes()


def _depth_png(cap):
    """Section 8.3 format: 16-bit PNG in millimetres on the colour pixel grid, 0 = no data."""
    millimetres = np.clip(np.round(cap.depth_m * 1000.0), 0, 65535).astype(np.uint16)
    ok, buf = cv2.imencode(".png", millimetres)
    return buf.tobytes()


def _capture_meta(cap):
    """Everything needed to turn depth.png back into 3D points offline (section 5.2)."""
    return {
        "capture": cap.id,
        "camera_serial": cap.camera,
        "stamp": cap.stamp,
        "width": int(cap.color.shape[1]),
        "height": int(cap.color.shape[0]),
        "intrinsics": cap.intrinsics,
        "depth": None if cap.depth_m is None else {
            "file": "depth.png",
            "units": "millimetres",
            "aligned_to": "color",
            "invalid_value": 0,
            "median_frames": cap.frames,
            "min_valid_fraction": MIN_VALID_FRACTION,
        },
    }


class CaptureRequest(BaseModel):
    camera: str
    frames: int = 10  # section 5.1: depth frames in the per-pixel median; 1 = a single frame


class SegmentRequest(BaseModel):
    capture: str
    points: list[list[float]] = []  # [x, y, label]; label 1 = object, 0 = background
    box: list[float] | None = None  # [x0, y0, x1, y1]
    multimask: bool = True


class AutoMaskRequest(BaseModel):
    capture: str
    points_per_side: int = 24
    max_area_frac: float = 0.2


class AddRequest(BaseModel):
    capture: str
    source: str  # "prompt" or "auto"
    index: int
    name: str
    up_side: str = ""


class CaptureRef(BaseModel):
    capture: str


@app.get("/")
def index():
    return FileResponse(STATIC_DIR / "index.html")


@app.get("/api/status")
def status():
    with _captures_lock:
        capture_ids = list(_captures)
    return {
        "models": models.info(),
        "cameras": cameras.list(),
        "camera_error": cameras.error,
        "gallery": gallery.list(),
        "captures": capture_ids,
    }


@app.post("/api/cameras/rescan")
def rescan():
    return {"cameras": cameras.rescan(), "camera_error": cameras.error}


@app.get("/stream/{cam_id}")
async def stream(cam_id: str, request: Request, view: str = "color"):
    camera = cameras.get(cam_id)
    if camera is None:
        raise HTTPException(404, f"Unknown camera {cam_id}")

    async def frames():
        last = -1
        while not await request.is_disconnected():
            data, seq = await asyncio.to_thread(camera.jpeg, view)
            if data is not None and seq != last:
                last = seq
                yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                       + str(len(data)).encode() + b"\r\n\r\n" + data + b"\r\n")
            await asyncio.sleep(1.0 / STREAM_FPS)

    return StreamingResponse(frames(), media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/frame.jpg")
def frame_jpg():
    """Lets `wendy device camera view` share the feed while this app holds the camera."""
    camera = cameras.first_online()
    frame = camera.latest() if camera else None
    if frame is None:
        raise HTTPException(503, "No frame yet")
    data, _ = camera.jpeg("color")
    return Response(data, media_type="image/jpeg",
                    headers={"X-Captured-At-Unix-Ns": str(int(frame.stamp * 1e9))})


@app.post("/api/capture")
def capture(req: CaptureRequest):
    camera = cameras.get(req.camera)
    if camera is None:
        raise HTTPException(404, f"Unknown camera {req.camera}")
    if camera.latest() is None:
        raise HTTPException(503, "The camera has not delivered a frame yet")
    try:
        frame, depths = camera.collect(req.frames)
    except ValueError as exc:
        raise HTTPException(400, str(exc))
    except TimeoutError as exc:
        raise HTTPException(503, f"Camera {camera.id}: {exc}")

    depth_m = median_ms = depth_valid = None
    if depths:
        start = time.monotonic()
        depth_m = median_depth(depths) * (camera.depth_scale or 0.001)
        median_ms = round((time.monotonic() - start) * 1000.0, 1)
        depth_valid = round(np.count_nonzero(depth_m) / depth_m.size, 3)
    cap = Capture(f"c{next(_capture_ids)}", camera.id, frame.color, depth_m, camera.intrinsics,
                  frame.stamp, len(depths))
    with _captures_lock:
        _captures[cap.id] = cap
        while len(_captures) > MAX_CAPTURES:
            _captures.popitem(last=False)
    set_image_ms = models.prepare(cap.id, cap.color) if models.state == "ready" else None
    return {
        "id": cap.id,
        "camera": cap.camera,
        "width": int(cap.color.shape[1]),
        "height": int(cap.color.shape[0]),
        "has_depth": depth_m is not None,
        "intrinsics": cap.intrinsics,
        "frames": cap.frames,
        "median_ms": median_ms,
        "depth_valid": depth_valid,
        "set_image_ms": set_image_ms,
    }


@app.get("/api/captures/{capture_id}/color.jpg")
def capture_jpg(capture_id: str):
    cap = _get_capture(capture_id)
    ok, buf = cv2.imencode(".jpg", cv2.cvtColor(cap.color, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 92])
    return Response(buf.tobytes(), media_type="image/jpeg")


@app.get("/api/captures/{capture_id}/color.png")
def capture_png(capture_id: str):
    cap = _get_capture(capture_id)
    return Response(_color_png(cap), media_type="image/png", headers=_attachment(cap, "color.png"))


@app.get("/api/captures/{capture_id}/depth.png")
def capture_depth(capture_id: str):
    cap = _get_capture(capture_id)
    if cap.depth_m is None:
        raise HTTPException(404, "This capture has no depth")
    return Response(_depth_png(cap), media_type="image/png", headers=_attachment(cap, "depth.png"))


@app.get("/api/captures/{capture_id}/intrinsics.json")
def capture_intrinsics(capture_id: str):
    cap = _get_capture(capture_id)
    return Response(json.dumps(_capture_meta(cap), indent=2), media_type="application/json",
                    headers=_attachment(cap, "intrinsics.json"))


@app.get("/api/captures/{capture_id}/capture.zip")
def capture_zip(capture_id: str):
    """colour.png, depth.png and intrinsics.json together, the unit an offline test loads."""
    cap = _get_capture(capture_id)
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_STORED) as archive:  # PNGs are already compressed
        archive.writestr("color.png", _color_png(cap))
        if cap.depth_m is not None:
            archive.writestr("depth.png", _depth_png(cap))
        archive.writestr("intrinsics.json", json.dumps(_capture_meta(cap), indent=2))
    return Response(buf.getvalue(), media_type="application/zip", headers=_attachment(cap, "capture.zip"))


@app.post("/api/segment")
def segment(req: SegmentRequest):
    cap = _get_capture(req.capture)
    _require_models()
    points = labels = box = None
    if req.points:
        points = np.array([p[:2] for p in req.points], np.float32)
        labels = np.array([int(p[2]) for p in req.points], np.int32)
    if req.box:
        box = np.array(req.box, np.float32)
    if points is None and box is None:
        raise HTTPException(400, "Give at least one point or a box")

    masks, scores, timing = models.segment(cap.id, cap.color, points, labels, box, req.multimask)
    keep = [i for i, m in enumerate(masks) if m.any()]
    masks, scores = [masks[i] for i in keep], [float(scores[i]) for i in keep]
    embeddings, crops, timing["embed_ms"] = models.embed(cap.color, masks)
    cap.prompt = [MaskItem(m, e, c) for m, e, c in zip(masks, embeddings, crops)]
    matches = _matches(cap.prompt, assign=False)

    color = PALETTE[0]
    return {
        "masks": [
            {"index": i, "sam_score": round(s, 3), "match": match, "overlay": _overlay([m], [color]),
             **_describe(cap, m)}
            for i, (m, s, match) in enumerate(zip(masks, scores, matches))
        ],
        "timing": timing,
    }


@app.post("/api/automask")
def automask(req: AutoMaskRequest):
    cap = _get_capture(req.capture)
    _require_models()
    if not 8 <= req.points_per_side <= 64:
        raise HTTPException(400, "points_per_side must be between 8 and 64")

    records, automask_ms = models.automask(cap.color, req.points_per_side, req.max_area_frac)
    records = records[:MAX_AUTO_MASKS]
    masks = [r["segmentation"] for r in records]
    embeddings, crops, embed_ms = models.embed(cap.color, masks)
    cap.auto = [MaskItem(m, e, c) for m, e, c in zip(masks, embeddings, crops)]
    matches = _matches(cap.auto, assign=True)

    colors = [PALETTE[i % len(PALETTE)] for i in range(len(masks))]
    return {
        "overlay": _overlay(masks, colors) if masks else None,
        "masks": [
            {"index": i, "color": "#%02x%02x%02x" % colors[i], "predicted_iou": round(float(r["predicted_iou"]), 3),
             "stability": round(float(r["stability_score"]), 3), "match": matches[i], **_describe(cap, masks[i])}
            for i, r in enumerate(records)
        ],
        "timing": {"automask_ms": automask_ms, "embed_ms": embed_ms, "count": len(masks)},
    }


@app.post("/api/rematch")
def rematch(req: CaptureRef):
    cap = _get_capture(req.capture)
    return {"prompt": _matches(cap.prompt, assign=False), "auto": _matches(cap.auto, assign=True)}


@app.post("/api/gallery/add")
def gallery_add(req: AddRequest):
    cap = _get_capture(req.capture)
    items = cap.prompt if req.source == "prompt" else cap.auto
    if not 0 <= req.index < len(items):
        raise HTTPException(400, "That mask is gone; segment again")
    item = items[req.index]
    try:
        name, views = gallery.add(req.name, item.crop, item.embedding, req.up_side.strip())
    except ValueError as exc:
        raise HTTPException(400, str(exc))
    return {"name": name, "views": views, "gallery": gallery.list()}


@app.get("/api/gallery/{name}/thumb.png")
def gallery_thumb(name: str):
    path = gallery.thumbnail(name)
    if path is None:
        raise HTTPException(404, "No such object")
    return FileResponse(path, media_type="image/png")


@app.delete("/api/gallery/{name}")
def gallery_delete(name: str):
    if not gallery.remove(name):
        raise HTTPException(404, "No such object")
    return {"gallery": gallery.list()}


# ---------- camera calibration (docs/perception_architecture.md, section 3.4) ----------

class BoardRequest(BaseModel):
    squares_x: int = 5
    squares_y: int = 7
    square_mm: float = 36.0
    marker_mm: float = 27.0
    dictionary: str = "DICT_4X4_50"


class SetupRequest(BaseModel):
    board_location: str = "table"  # "table" or "hand"
    mounts: dict[str, str] = {}  # camera serial -> "fixed" or "hand"


class RobotPose(BaseModel):
    parent: str = "world"
    child: str = "lara5_flange"
    translation: list[float]  # flange position in parent [m]
    rotation_xyzw: list[float]
    joints: dict[str, float] = {}


class SampleRequest(BaseModel):
    robot: RobotPose
    frames: int = 3  # fresh frames to wait for, so the image is from after the robot stopped


@app.get("/calibration")
def calibration_page():
    return FileResponse(STATIC_DIR / "calibration.html")


@app.get("/api/calib/status")
def calib_status():
    saved = calib.root / "extrinsics.yaml"
    board = calib.board
    samples = calib.summary()
    online = {c.id for c in cameras.online_with_intrinsics()}
    known = sorted(online | {serial for s in samples for serial in s["cameras"]} | set(calib.mounts))
    return {
        "board": asdict(board),
        "board_size_mm": [board.squares_x * board.square_mm, board.squares_y * board.square_mm],
        "setup": {"board_location": calib.board_location, "mounts": calib.mounts},
        "samples": samples,
        "cameras": [{"id": serial, "name": calibration.CAMERA_NAMES.get(serial, serial),
                     "mount": calib.mount(serial), "online": serial in online} for serial in known],
        "result": calib.result,
        "saved": {"exists": saved.exists(), "modified": saved.stat().st_mtime if saved.exists() else None},
        "min_samples": calibration.MIN_SAMPLES,
    }


@app.post("/api/calib/board")
def calib_board(req: BoardRequest):
    try:
        calib.set_board(calibration.BoardConfig(**req.model_dump()))
    except (ValueError, AttributeError) as exc:
        raise HTTPException(409, str(exc))
    return calib_status()


@app.post("/api/calib/setup")
def calib_setup(req: SetupRequest):
    """Where the board is and which cameras ride on the hand. Used at solve time only."""
    try:
        calib.set_setup(req.board_location, req.mounts)
    except ValueError as exc:
        raise HTTPException(400, str(exc))
    return calib_status()


@app.get("/api/calib/board.pdf")
def calib_board_pdf():
    board = calib.board
    key = tuple(asdict(board).values())
    if key not in _board_pdf:
        try:
            _board_pdf[key] = calibration.printable_pdf(board)
        except ValueError as exc:
            raise HTTPException(409, str(exc))
    name = f"charuco_{board.squares_x}x{board.squares_y}_{board.square_mm:g}mm.pdf"
    return Response(_board_pdf[key], media_type="application/pdf",
                    headers={"Content-Disposition": f'attachment; filename="{name}"'})


@app.get("/api/calib/overlay/{cam_id}.jpg")
def calib_overlay(cam_id: str):
    camera = cameras.get(cam_id)
    frame = camera.latest() if camera else None
    if frame is None or not camera.intrinsics:
        raise HTTPException(404, f"No RealSense frame from {cam_id}")
    data, _ = calib.detector.overlay(frame.color, camera.intrinsics)
    return Response(data, media_type="image/jpeg", headers={"Cache-Control": "no-store"})


@app.post("/api/calib/sample")
def calib_sample(req: SampleRequest):
    """Record the board in every camera at the robot pose the robot side sends."""
    robot = req.robot.model_dump()
    if len(robot["translation"]) != 3 or len(robot["rotation_xyzw"]) != 4:
        raise HTTPException(400, "translation needs 3 values and rotation_xyzw 4")
    norm = float(np.linalg.norm(robot["rotation_xyzw"]))
    if not 0.9 < norm < 1.1:
        raise HTTPException(400, f"rotation_xyzw is not a unit quaternion (norm {norm:.3f})")
    robot["rotation_xyzw"] = [v / norm for v in robot["rotation_xyzw"]]
    online = cameras.online_with_intrinsics()
    if not online:
        raise HTTPException(503, "No RealSense camera is streaming")

    detections, report = {}, {}
    for camera in online:
        try:
            frame, _ = camera.collect(req.frames)
        except (TimeoutError, ValueError) as exc:
            report[camera.id] = {"found": False, "error": str(exc)}
            continue
        det, raw = calib.detector.detect(frame.color, camera.intrinsics)
        if det:
            detections[camera.id] = det
            report[camera.id] = {"found": True, "corners": len(det["ids"]), "reproj_px": det["reproj_px"],
                                 "distance_m": round(float(np.linalg.norm(det["tvec"])), 3)}
        else:
            report[camera.id] = {"found": False, "corners": 0 if raw[1] is None else len(raw[1])}
    if not detections:
        raise HTTPException(422, {"message": "No camera sees the board; sample not recorded", "cameras": report})
    sample = calib.add(robot, detections)
    return {"id": sample["id"], "cameras": report, "samples": len(calib.samples)}


@app.delete("/api/calib/samples/{sample_id}")
def calib_delete_sample(sample_id: int):
    if not calib.remove(sample_id):
        raise HTTPException(404, f"No sample {sample_id}")
    return {"samples": calib.summary()}


@app.delete("/api/calib/samples")
def calib_clear():
    calib.clear()
    return {"samples": []}


@app.get("/api/calib/session.json")
def calib_export():
    return Response(json.dumps(calib.export(), indent=1), media_type="application/json",
                    headers={"Content-Disposition": 'attachment; filename="calibration_session.json"'})


@app.post("/api/calib/session")
def calib_import(data: dict):
    """Replace the samples with an exported session, for example to solve again after a restart."""
    try:
        calib.replace(data)
    except (ValueError, TypeError, KeyError, AttributeError) as exc:
        raise HTTPException(400, f"Not a calibration session: {exc}")
    return {"samples": calib.summary()}


@app.post("/api/calib/solve")
def calib_solve():
    try:
        return calib.solve()
    except ValueError as exc:
        raise HTTPException(409, str(exc))


@app.post("/api/calib/save")
def calib_save():
    try:
        text = calib.save_extrinsics()
    except ValueError as exc:
        raise HTTPException(409, str(exc))
    return {"path": str(calib.root / "extrinsics.yaml"), "yaml": text}


@app.get("/api/calib/extrinsics.yaml")
def calib_extrinsics():
    path = calib.root / "extrinsics.yaml"
    if not path.exists():
        raise HTTPException(404, "No calibration saved yet")
    return FileResponse(path, media_type="application/x-yaml", filename="extrinsics.yaml")


if __name__ == "__main__":
    cameras.rescan()
    threading.Thread(target=models.load, daemon=True, name="model-loader").start()
    log.info("Serving on port %d", PORT)
    uvicorn.run(app, host="0.0.0.0", port=PORT, log_level="warning")
