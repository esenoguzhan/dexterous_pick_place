"""Section 5.5: refine candidates with SAM 2.1, split touching objects, find depth-poor objects.

The pipeline talks to any segmenter with this interface, so tests can pass a fake one:

    set_image(key, image)                                   once per camera image
    predict(box=None, points=None, labels=None, multimask=True) -> masks (k x H x W bool), scores (k)
    automask(image, points_per_side) -> [(mask, score)]
"""

from __future__ import annotations

import contextlib
import glob
import logging
import os
import sys
import time

import cv2
import numpy as np

log = logging.getLogger("restore_perception.segment")


def iou(a, b):
    union = np.count_nonzero(a | b)
    return np.count_nonzero(a & b) / union if union else 0.0


def refine(segmenter, target, box, cfg):
    """Masks of the objects in one candidate as one camera sees it: [(mask, score)], the box mask first.

    target: the candidate's pixels in this camera (H x W bool, gaps closed). The box mask is the SAM
    output that best matches target. If it covers less than split_coverage of target, the candidate
    probably holds more than one object: prompt SAM with a point in the middle of the uncovered
    pixels and keep the new mask, until coverage is reached or no new mask appears.
    """
    masks, scores = segmenter.predict(box=np.asarray(box, np.float32), multimask=True)
    if len(masks) == 0:
        return []
    overlaps = [iou(m, target) for m in masks]
    best = int(np.argmax(overlaps))
    if overlaps[best] == 0.0:
        return []
    chosen = [(masks[best], float(scores[best]))]
    union = masks[best].copy()
    total = np.count_nonzero(target)
    kernel = np.ones((5, 5), np.uint8)
    for _ in range(int(cfg["max_splits"])):
        if np.count_nonzero(union & target) >= cfg["split_coverage"] * total:
            break
        uncovered = cv2.morphologyEx((target & ~union).astype(np.uint8), cv2.MORPH_OPEN, kernel)
        if np.count_nonzero(uncovered) < cfg["min_split_px"]:
            break
        dist = cv2.distanceTransform(uncovered, cv2.DIST_L2, 5)
        v, u = np.unravel_index(int(np.argmax(dist)), dist.shape)
        new_masks, new_scores = segmenter.predict(points=np.array([[u, v]], np.float32), labels=np.array([1]),
                                                  multimask=True)
        found = None
        for mask, score in sorted(zip(new_masks, new_scores), key=lambda item: -item[1]):
            area = np.count_nonzero(mask)
            if (mask[v, u] and np.count_nonzero(mask & union) <= 0.5 * area
                    and np.count_nonzero(mask & target) >= 0.3 * area):
                found = (mask, float(score))
                break
        if found is None:
            break
        chosen.append(found)
        union |= found[0]
    claimed = np.zeros_like(target)
    result = []
    for mask, score in chosen:  # an overlapping pixel stays with the earlier mask
        mask = mask & ~claimed
        claimed |= mask
        result.append((mask, score))
    return result


def depth_poor_masks(segmenter, image, existing, cfg):
    """Automatic masks of plausible size that overlap no object found by geometry: [(mask, score)]."""
    records = segmenter.automask(image, int(cfg["points_per_side"]))
    pixels = image.shape[0] * image.shape[1]
    taken = np.zeros(image.shape[:2], bool)
    for mask in existing:
        taken |= mask
    result = []
    for mask, score in records:
        area = np.count_nonzero(mask)
        if not cfg["min_area_frac"] * pixels <= area <= cfg["max_area_frac"] * pixels:
            continue
        if np.count_nonzero(mask & taken) > cfg["max_overlap"] * area:
            continue
        taken |= mask
        result.append((mask, float(score)))
    return result


def _import_sam2(root):
    """Make the SAM 2 source importable; the Jetson image ships it as source, not as a pip package."""
    try:
        import sam2  # noqa: F401
        return
    except ImportError:
        pass
    inits = glob.glob(os.path.join(root, "**", "sam2", "__init__.py"), recursive=True)
    if not inits:
        raise ImportError(f"SAM 2 source not found under {root}")
    sys.path.insert(0, os.path.dirname(os.path.dirname(min(inits, key=len))))


class Sam2Segmenter:
    """SAM 2.1 hiera-small, as the test bench runs it (wendy/perception_test/vision.py)."""

    def __init__(self, models_cfg):
        self.cfg = models_cfg
        self._predictor = None
        self._sam = None
        self._key = None
        self.device = None

    def load(self):
        import torch

        _import_sam2(self.cfg["sam2_root"])
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor

        start = time.monotonic()
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        if self.device == "cuda":
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True
        self._sam = build_sam2(self.cfg["sam2_config"], self.cfg["sam2_checkpoint"], device=self.device)
        self._predictor = SAM2ImagePredictor(self._sam)
        dummy = np.full((480, 640, 3), 127, np.uint8)  # section 8.2: warm up once
        self.set_image("warmup", dummy)
        self.predict(points=np.array([[320.0, 240.0]], np.float32), labels=np.array([1]))
        self._key = None
        log.info("SAM 2.1 ready in %.1f s on %s", time.monotonic() - start, self.device)

    def _autocast(self):
        import torch

        if self.device == "cuda":
            return torch.autocast("cuda", dtype=torch.bfloat16)
        return contextlib.nullcontext()

    def set_image(self, key, image):
        import torch

        if key == self._key:
            return
        with torch.inference_mode(), self._autocast():
            self._predictor.set_image(image)
        self._key = key

    def predict(self, box=None, points=None, labels=None, multimask=True):
        import torch

        with torch.inference_mode(), self._autocast():
            masks, scores, _ = self._predictor.predict(point_coords=points, point_labels=labels, box=box,
                                                       multimask_output=multimask)
        return masks > 0.5, np.asarray(scores, np.float32)

    def automask(self, image, points_per_side):
        import torch
        from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator

        generator = SAM2AutomaticMaskGenerator(
            self._sam, points_per_side=points_per_side,
            points_per_batch=32,  # the Orin Nano shares 8 GB between CPU and GPU
            pred_iou_thresh=0.8, stability_score_thresh=0.92, min_mask_region_area=100)
        with torch.inference_mode(), self._autocast():
            records = generator.generate(image)  # has its own predictor; set_image's cache stays valid
        return [(r["segmentation"].astype(bool), float(r["predicted_iou"])) for r in records]
