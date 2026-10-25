"""Section 5.7: masked crops, DINOv2 embeddings and HSV colour histograms.

The pipeline talks to any embedder with `embed(crops) -> n x d float32, L2-normalised`.
"""

from __future__ import annotations

import logging
import time

import cv2
import numpy as np

log = logging.getLogger("restore_perception.embed")

CROP_PAD = 0.10
CROP_GREY = 127
DINO_SIZE = 224
IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], np.float32)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], np.float32)
HIST_BINS = (16, 4)  # hue, saturation; brightness is left out so lighting matters less


def mask_bbox(mask):
    ys, xs = np.nonzero(mask)
    return int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1


def masked_crop(image, mask):
    """Mask bounding box plus 10 % padding, grey outside the mask, padded to a grey square and resized
    to 224x224, so the object keeps its aspect ratio at every rotation. Same as the test bench."""
    h, w = mask.shape
    x0, y0, x1, y1 = mask_bbox(mask)
    pad_x, pad_y = round((x1 - x0) * CROP_PAD), round((y1 - y0) * CROP_PAD)
    x0, y0 = max(0, x0 - pad_x), max(0, y0 - pad_y)
    x1, y1 = min(w, x1 + pad_x), min(h, y1 + pad_y)
    crop = image[y0:y1, x0:x1].copy()
    crop[~mask[y0:y1, x0:x1]] = CROP_GREY
    ch, cw = crop.shape[:2]
    side = max(ch, cw)
    square = np.full((side, side, 3), CROP_GREY, np.uint8)
    top, left = (side - ch) // 2, (side - cw) // 2
    square[top:top + ch, left:left + cw] = crop
    interpolation = cv2.INTER_AREA if side >= DINO_SIZE else cv2.INTER_LINEAR
    return cv2.resize(square, (DINO_SIZE, DINO_SIZE), interpolation=interpolation)


def hsv_histogram(image, mask):
    """Normalised hue-saturation histogram of the masked pixels (the 5.7 tie-break)."""
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    hist = cv2.calcHist([hsv], [0, 1], mask.astype(np.uint8), list(HIST_BINS), [0, 180, 0, 256]).ravel()
    total = hist.sum()
    return (hist / total if total else hist).astype(np.float32)


def crop_histogram(crop):
    """Histogram of a stored gallery crop: every pixel that is not the grey background."""
    return hsv_histogram(crop, np.any(crop != CROP_GREY, axis=-1))


def histogram_similarity(a, b):
    """Bhattacharyya coefficient: 1 for identical histograms, 0 for disjoint ones."""
    return float(np.sum(np.sqrt(np.clip(a, 0, None) * np.clip(b, 0, None))))


class Dinov2Embedder:
    """DINOv2-small CLS token, L2-normalised, in float32 so stored views stay comparable."""

    def __init__(self, models_cfg):
        self.cfg = models_cfg
        self._model = None
        self.device = None

    def load(self):
        import torch
        from transformers import Dinov2Model

        start = time.monotonic()
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self._model = Dinov2Model.from_pretrained(self.cfg["dino_dir"]).to(self.device).eval()
        self.embed([np.full((DINO_SIZE, DINO_SIZE, 3), CROP_GREY, np.uint8)])  # warm-up
        log.info("DINOv2 ready in %.1f s on %s", time.monotonic() - start, self.device)

    def embed(self, crops):
        import torch
        import torch.nn.functional as F

        if not crops:
            return np.zeros((0, 384), np.float32)
        batch = (np.stack(crops).astype(np.float32) / 255.0 - IMAGENET_MEAN) / IMAGENET_STD
        pixels = torch.from_numpy(batch).permute(0, 3, 1, 2).contiguous().to(self.device)
        with torch.inference_mode():
            cls = self._model(pixel_values=pixels).pooler_output
        return F.normalize(cls.float(), dim=-1).cpu().numpy()
