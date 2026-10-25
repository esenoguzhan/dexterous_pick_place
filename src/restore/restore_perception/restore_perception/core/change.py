"""Section 6: pixels that changed between the reference capture and the current one.

The fixed camera does not move and the hand camera captures from the same observe pose, so the
two images can be compared pixel by pixel.
"""

from __future__ import annotations

import cv2
import numpy as np


def change_mask(reference, current, cfg):
    """H x W bool: depth moved by more than depth_threshold_m or colour by more than color_threshold
    (8-bit Lab distance), cleaned with an opening and with regions under min_region_px dropped."""
    if reference.color.shape != current.color.shape:
        raise ValueError(f"{current.name}: image size differs from the reference")
    both = (reference.depth > 0) & (current.depth > 0)
    depth_changed = both & (np.abs(reference.depth - current.depth) > cfg["depth_threshold_m"])
    lab_ref = cv2.cvtColor(reference.color, cv2.COLOR_RGB2LAB).astype(np.float32)
    lab_cur = cv2.cvtColor(current.color, cv2.COLOR_RGB2LAB).astype(np.float32)
    color_changed = np.linalg.norm(lab_ref - lab_cur, axis=-1) > cfg["color_threshold"]
    changed = cv2.morphologyEx((depth_changed | color_changed).astype(np.uint8), cv2.MORPH_OPEN,
                               np.ones((5, 5), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(changed, connectivity=8)
    keep = np.zeros(count, bool)
    keep[1:] = stats[1:, cv2.CC_STAT_AREA] >= cfg["min_region_px"]
    return keep[labels]
