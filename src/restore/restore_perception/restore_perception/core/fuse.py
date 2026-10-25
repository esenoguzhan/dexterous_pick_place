"""Section 5.6: associate the per-camera views of each object.

Partial views of one object (front from one camera, back from the other) have different 3D centroids,
but their footprints on the table overlap. A pair is accepted when the footprints overlap enough or
the centroids are close; pairs come from the Hungarian algorithm on 1 - footprint IoU plus the
centroid distance in metres.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment

_OFFSET = 1 << 20
_NEIGHBOURS = [(dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1)]


@dataclass
class View:
    """One camera's view of one object."""

    camera: str
    mask: np.ndarray  # H x W bool
    points: np.ndarray  # world, N x 3
    score: float | None  # SAM score, None without SAM
    cells: np.ndarray = field(default_factory=lambda: np.zeros(0, np.int64))  # footprint grid keys
    centre: np.ndarray = field(default_factory=lambda: np.zeros(2))  # footprint centre, table frame
    depth_poor: bool = False


def footprint_cells(xy, grid):
    """Occupied grid cells of 2D table-frame points, grown by one cell, as sorted int64 keys."""
    if len(xy) == 0:
        return np.zeros(0, np.int64)
    cells = np.unique(np.floor(np.asarray(xy) / grid).astype(np.int64), axis=0)
    grown = np.concatenate([cells + np.array(d) for d in _NEIGHBOURS])
    return np.unique((grown[:, 0] + _OFFSET) * (2 * _OFFSET) + (grown[:, 1] + _OFFSET))


def polygon_cells(xy, grid):
    """Grid cells covered by the convex hull of 2D table-frame points, grown by one cell."""
    xy = np.asarray(xy, np.float64)
    if len(xy) < 3:
        return footprint_cells(xy, grid)
    low = np.floor(xy.min(axis=0) / grid).astype(np.int64)
    pts = np.round(xy / grid - low).astype(np.int32)
    size = pts.max(axis=0) + 1
    raster = np.zeros((size[1], size[0]), np.uint8)
    cv2.fillPoly(raster, [cv2.convexHull(pts.reshape(-1, 1, 2))], 1)
    rows, cols = np.nonzero(raster)
    return footprint_cells((np.column_stack([cols, rows]) + low + 0.5) * grid, grid)


def cells_iou(a, b):
    if len(a) == 0 or len(b) == 0:
        return 0.0
    inter = len(np.intersect1d(a, b, assume_unique=True))
    return inter / (len(a) + len(b) - inter)


def associate(views_by_camera, cfg):
    """Groups of views, one view per camera at most, each group one object."""
    groups = []
    for views in views_by_camera:
        if not groups:
            groups = [[v] for v in views]
            continue
        if not views:
            continue
        group_cells = [np.unique(np.concatenate([v.cells for v in g])) for g in groups]
        group_centre = [np.mean([v.centre for v in g], axis=0) for g in groups]
        iou = np.array([[cells_iou(gc, v.cells) for v in views] for gc in group_cells])
        dist = np.array([[np.linalg.norm(c - v.centre) for v in views] for c in group_centre])
        rows, cols = linear_sum_assignment(1.0 - iou + dist)
        matched = set()
        for r, c in zip(rows, cols):
            if iou[r, c] >= cfg["min_iou"] or dist[r, c] <= cfg["max_distance_m"]:
                groups[r].append(views[c])
                matched.add(c)
        groups.extend([views[c]] for c in range(len(views)) if c not in matched)
    return groups
