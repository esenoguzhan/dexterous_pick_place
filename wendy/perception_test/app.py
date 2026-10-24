"""Browser test bench for the SAM 2.1 and DINOv2 stages of docs/perception_architecture.md."""

from __future__ import annotations

import asyncio
import base64
import itertools
import logging
import os
import threading
from collections import OrderedDict
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, Response, StreamingResponse
from pydantic import BaseModel

from cameras import CameraManager
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


class CaptureRequest(BaseModel):
    camera: str


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
    frame = camera.latest()
    if frame is None:
        raise HTTPException(503, "The camera has not delivered a frame yet")
    depth_m = None
    if frame.depth is not None:
        depth_m = frame.depth.astype(np.float32) * (camera.depth_scale or 0.001)
    cap = Capture(f"c{next(_capture_ids)}", camera.id, frame.color, depth_m, camera.intrinsics)
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
    ok, buf = cv2.imencode(".png", cv2.cvtColor(cap.color, cv2.COLOR_RGB2BGR))
    return Response(buf.tobytes(), media_type="image/png",
                    headers={"Content-Disposition": f'attachment; filename="{cap.id}_color.png"'})


@app.get("/api/captures/{capture_id}/depth.png")
def capture_depth(capture_id: str):
    """Section 8.3 format: 16-bit PNG in millimetres on the colour pixel grid."""
    cap = _get_capture(capture_id)
    if cap.depth_m is None:
        raise HTTPException(404, "This capture has no depth")
    millimetres = np.clip(cap.depth_m * 1000.0, 0, 65535).astype(np.uint16)
    ok, buf = cv2.imencode(".png", millimetres)
    return Response(buf.tobytes(), media_type="image/png",
                    headers={"Content-Disposition": f'attachment; filename="{cap.id}_depth.png"'})


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


if __name__ == "__main__":
    cameras.rescan()
    threading.Thread(target=models.load, daemon=True, name="model-loader").start()
    log.info("Serving on port %d", PORT)
    uvicorn.run(app, host="0.0.0.0", port=PORT, log_level="warning")
