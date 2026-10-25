"""Debug overlays (section 8.2): masks, ids and labels drawn on a camera's colour image."""

from __future__ import annotations

import cv2
import numpy as np

PALETTE = [(230, 25, 75), (60, 180, 75), (255, 225, 25), (0, 130, 200), (245, 130, 48), (145, 30, 180),
           (70, 240, 240), (240, 50, 230), (210, 245, 60), (250, 190, 212), (0, 128, 128), (170, 110, 40)]


def color_of(index):
    return PALETTE[index % len(PALETTE)]


def draw(frame, objects, alpha=0.45):
    """RGB image with each object's mask tinted and outlined, and 'id label score' next to it."""
    image = frame.color.copy()
    for i, obj in enumerate(objects):
        mask = obj.masks.get(frame.name)
        if mask is None or not mask.any():
            continue
        color = np.array(color_of(i), np.float32)
        image[mask] = (image[mask] * (1 - alpha) + color * alpha).astype(np.uint8)
        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(image, contours, -1, color_of(i), 2)
        ys, xs = np.nonzero(mask)
        text = f"{obj.id} {obj.label}" + (f" {obj.label_score:.2f}" if obj.label != "unknown" else "")
        org = (int(xs.min()), max(int(ys.min()) - 6, 14))
        cv2.putText(image, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(image, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return image
