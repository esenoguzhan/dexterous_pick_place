"""SAM 2.1 and DINOv2, used as docs/perception_architecture.md sections 5.5 and 5.7 describe."""

from __future__ import annotations

import contextlib
import glob
import json
import logging
import os
import re
import shutil
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.nn.functional as F

log = logging.getLogger("vision")

SAM2_ROOT = os.environ.get("SAM2_ROOT", "/opt/sam2")
SAM2_CKPT = os.environ.get("SAM2_CKPT", "/models/sam2.1_hiera_small.pt")
SAM2_CONFIG = "configs/sam2.1/sam2.1_hiera_s.yaml"
DINO_DIR = os.environ.get("DINO_DIR", "/models/dinov2-small")

# Section 5.7: crop padding, background grey and input size for DINOv2.
CROP_PAD = 0.10
CROP_GREY = 127
DINO_SIZE = 224
EMBED_DIM = 384
IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], np.float32)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], np.float32)

# Section 5.7: gallery acceptance thresholds.
MIN_SCORE = 0.6
MIN_MARGIN = 0.05


def _import_sam2():
    """Make the SAM 2 source importable. The image ships it as source, not as a pip package."""
    try:
        import sam2  # noqa: F401

        return
    except ImportError:
        pass
    inits = glob.glob(os.path.join(SAM2_ROOT, "**", "sam2", "__init__.py"), recursive=True)
    if not inits:
        raise ImportError(f"SAM 2 source not found under {SAM2_ROOT}")
    sys.path.insert(0, os.path.dirname(os.path.dirname(min(inits, key=len))))


def _ms(start):
    return round((time.monotonic() - start) * 1000.0, 1)


def mask_bbox(mask):
    """Tight box of a boolean mask as [x0, y0, x1, y1), in pixels."""
    ys, xs = np.nonzero(mask)
    return int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1


def masked_crop(image, mask):
    """Section 5.7: mask bounding box plus 10 % padding, grey outside the mask, resized to 224x224."""
    h, w = mask.shape
    x0, y0, x1, y1 = mask_bbox(mask)
    pad_x, pad_y = round((x1 - x0) * CROP_PAD), round((y1 - y0) * CROP_PAD)
    x0, y0 = max(0, x0 - pad_x), max(0, y0 - pad_y)
    x1, y1 = min(w, x1 + pad_x), min(h, y1 + pad_y)
    crop = image[y0:y1, x0:x1].copy()
    crop[~mask[y0:y1, x0:x1]] = CROP_GREY
    return cv2.resize(crop, (DINO_SIZE, DINO_SIZE), interpolation=cv2.INTER_AREA)


class Models:
    """Loads both models once, then serialises GPU work behind one lock."""

    def __init__(self):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.state = "loading"
        self.error = None
        self.load_seconds = None
        self.lock = threading.Lock()
        self._sam = None
        self._predictor = None
        self._dino = None
        self._image_key = None

    def load(self):
        start = time.monotonic()
        try:
            _import_sam2()
            from sam2.build_sam import build_sam2
            from sam2.sam2_image_predictor import SAM2ImagePredictor
            from transformers import Dinov2Model

            if self.device == "cuda":
                torch.backends.cuda.matmul.allow_tf32 = True
                torch.backends.cudnn.allow_tf32 = True
            self._sam = build_sam2(SAM2_CONFIG, SAM2_CKPT, device=self.device)
            self._predictor = SAM2ImagePredictor(self._sam)
            self._dino = Dinov2Model.from_pretrained(DINO_DIR).to(self.device).eval()

            # Section 8.2: warm up once so the first capture is not slow.
            dummy = np.full((480, 640, 3), CROP_GREY, np.uint8)
            with self.lock:
                self._set_image("warmup", dummy)
                self._predict(np.array([[320.0, 240.0]], np.float32), np.array([1]), None, True)
                self._embed([cv2.resize(dummy, (DINO_SIZE, DINO_SIZE))])
                self._image_key = None
            self.load_seconds = round(time.monotonic() - start, 1)
            self.state = "ready"
            log.info("Models ready in %.1f s on %s", self.load_seconds, self.device)
        except Exception as exc:
            log.exception("Model loading failed")
            self.error = f"{type(exc).__name__}: {exc}"
            self.state = "error"

    def info(self):
        info = {
            "state": self.state,
            "error": self.error,
            "device": self.device,
            "load_seconds": self.load_seconds,
            "torch": torch.__version__,
            "torch_cuda": torch.version.cuda,
            "sam2": "sam2.1_hiera_small",
            "dinov2": "dinov2-small",
        }
        if self.device == "cuda":
            try:
                info["gpu"] = torch.cuda.get_device_name(0)
                free, total = torch.cuda.mem_get_info()
                info["gpu_mem_free_gb"] = round(free / 1e9, 2)
                info["gpu_mem_total_gb"] = round(total / 1e9, 2)
                info["torch_allocated_gb"] = round(torch.cuda.memory_allocated() / 1e9, 2)
            except Exception as exc:
                info["gpu_error"] = str(exc)
        return info

    def _autocast(self):
        if self.device == "cuda":
            return torch.autocast("cuda", dtype=torch.bfloat16)
        return contextlib.nullcontext()

    def _sync(self):
        if self.device == "cuda":
            torch.cuda.synchronize()

    def _set_image(self, key, image):
        """Section 5.5: compute the image embedding once per capture. Returns milliseconds, 0 if cached."""
        if key == self._image_key:
            return 0.0
        start = time.monotonic()
        with torch.inference_mode(), self._autocast():
            self._predictor.set_image(image)
        self._sync()
        self._image_key = key
        return _ms(start)

    def _predict(self, points, labels, box, multimask):
        with torch.inference_mode(), self._autocast():
            masks, scores, _ = self._predictor.predict(
                point_coords=points,
                point_labels=labels,
                box=box,
                multimask_output=multimask,
            )
        return masks > 0.5, scores

    def _embed(self, crops):
        """Section 5.7: CLS token of DINOv2, L2-normalised. Runs in float32 so stored views stay comparable."""
        batch = (np.stack(crops).astype(np.float32) / 255.0 - IMAGENET_MEAN) / IMAGENET_STD
        pixels = torch.from_numpy(batch).permute(0, 3, 1, 2).contiguous().to(self.device)
        with torch.inference_mode():
            cls = self._dino(pixel_values=pixels).pooler_output
        return F.normalize(cls.float(), dim=-1).cpu().numpy()

    def prepare(self, key, image):
        with self.lock:
            return self._set_image(key, image)

    def segment(self, key, image, points, labels, box, multimask):
        """Section 5.5: box- and point-prompted masks, best SAM score first."""
        with self.lock:
            set_ms = self._set_image(key, image)
            start = time.monotonic()
            masks, scores = self._predict(points, labels, box, multimask)
            predict_ms = _ms(start)
        order = np.argsort(-scores)
        return masks[order], scores[order], {"set_image_ms": set_ms, "predict_ms": predict_ms}

    def automask(self, image, points_per_side, max_area_frac):
        """Section 5.5 (objects with bad depth): SAM2AutomaticMaskGenerator over the whole image."""
        from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator

        generator = SAM2AutomaticMaskGenerator(
            self._sam,
            points_per_side=points_per_side,
            points_per_batch=32,  # the Orin Nano shares 8 GB between CPU and GPU
            pred_iou_thresh=0.8,
            stability_score_thresh=0.92,
            min_mask_region_area=100,
        )
        with self.lock:
            start = time.monotonic()
            with torch.inference_mode(), self._autocast():
                records = generator.generate(image)
            elapsed = _ms(start)
        limit = max_area_frac * image.shape[0] * image.shape[1]
        records = [r for r in records if r["area"] <= limit]
        records.sort(key=lambda r: r["area"], reverse=True)
        return records, elapsed

    def embed(self, image, masks):
        crops = [masked_crop(image, m) for m in masks]
        if not crops:
            return np.zeros((0, EMBED_DIM), np.float32), crops, 0.0
        with self.lock:
            start = time.monotonic()
            embeddings = self._embed(crops)
            return embeddings, crops, _ms(start)


_NAME_RE = re.compile(r"[^a-z0-9_-]+")


def clean_name(name):
    return _NAME_RE.sub("_", name.strip().lower()).strip("_")


class Gallery:
    """Sections 7.1 and 8.3: gallery/<name>/view_XX.png, embeddings.npy and meta.json."""

    def __init__(self, root):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.lock = threading.Lock()
        self._objects = {}
        for path in sorted(self.root.glob("*/embeddings.npy")):
            self._objects[path.parent.name] = np.load(path)

    def _meta(self, name):
        path = self.root / name / "meta.json"
        return json.loads(path.read_text()) if path.exists() else {"views": []}

    def list(self):
        with self.lock:
            items = []
            for name, embeddings in sorted(self._objects.items()):
                sides = {v.get("up_side", "") for v in self._meta(name)["views"]} - {""}
                items.append({"name": name, "views": len(embeddings), "up_sides": sorted(sides)})
            return items

    def add(self, name, crop, embedding, up_side=""):
        name = clean_name(name)
        if not name:
            raise ValueError("Object name is empty")
        with self.lock:
            folder = self.root / name
            folder.mkdir(parents=True, exist_ok=True)
            previous = self._objects.get(name)
            embeddings = embedding[None] if previous is None else np.vstack([previous, embedding[None]])
            filename = f"view_{len(embeddings) - 1:02d}.png"
            cv2.imwrite(str(folder / filename), cv2.cvtColor(crop, cv2.COLOR_RGB2BGR))
            np.save(folder / "embeddings.npy", embeddings)
            meta = self._meta(name)
            meta["views"].append({"file": filename, "up_side": up_side, "added": time.time()})
            (folder / "meta.json").write_text(json.dumps(meta, indent=2))
            self._objects[name] = embeddings
            return name, len(embeddings)

    def remove(self, name):
        with self.lock:
            if name not in self._objects:
                return False
            shutil.rmtree(self.root / name, ignore_errors=True)
            del self._objects[name]
            return True

    def thumbnail(self, name):
        path = self.root / clean_name(name) / "view_00.png"
        return path if path.exists() else None

    def match(self, embeddings, assign):
        """Section 5.7: best cosine similarity per gallery object, then accept or call it unknown.

        With assign=True, each gallery object is used at most once (Hungarian on 1 - score).
        """
        with self.lock:
            names = sorted(self._objects)
            if not names or len(embeddings) == 0:
                return [{"label": "unknown", "candidate": None, "score": None, "margin": None, "top": []}
                        for _ in range(len(embeddings))]
            scores = np.stack([(embeddings @ self._objects[n].T).max(axis=1) for n in names], axis=1)

        assigned = {}
        if assign:
            from scipy.optimize import linear_sum_assignment

            rows, cols = linear_sum_assignment(1.0 - scores)
            assigned = dict(zip(rows.tolist(), cols.tolist()))

        results = []
        for i, row in enumerate(scores):
            order = np.argsort(-row)
            best = assigned.get(i, int(order[0]))
            score = float(row[best])
            second = max((float(row[k]) for k in range(len(names)) if k != best), default=0.0)
            margin = score - second
            accepted = score >= MIN_SCORE and margin >= MIN_MARGIN and (not assign or i in assigned)
            results.append({
                "label": names[best] if accepted else "unknown",
                "candidate": names[best],
                "score": round(score, 3),
                "margin": round(margin, 3),
                "top": [{"name": names[k], "score": round(float(row[k]), 3)} for k in order[:3]],
            })
        return results
