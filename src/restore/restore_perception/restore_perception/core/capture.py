"""Section 5.1: per-pixel median depth over several frames."""

from __future__ import annotations

import math

import numpy as np


def median_depth(depths, min_valid=0.5):
    """Per-pixel median over frames, ignoring zeros (no data).

    A pixel keeps a value only when at least min_valid of its samples are valid; otherwise it is 0.
    Returns the input units as float32.
    """
    stack = np.stack(depths)
    n = stack.shape[0]
    valid = np.count_nonzero(stack, axis=0)
    ordered = np.sort(stack, axis=0)  # zeros sort first
    zeros = n - valid
    lo = np.minimum(zeros + (valid - 1) // 2, n - 1)
    hi = np.minimum(zeros + valid // 2, n - 1)
    lower = np.take_along_axis(ordered, lo[None], axis=0)[0].astype(np.float32)
    upper = np.take_along_axis(ordered, hi[None], axis=0)[0].astype(np.float32)
    median = 0.5 * (lower + upper)
    median[valid < max(1, math.ceil(min_valid * n))] = 0.0
    return median


def clip_depth(depth_m, min_m, max_m):
    """Zero out depth outside the camera's useful range."""
    depth_m = depth_m.astype(np.float32, copy=True)
    depth_m[(depth_m < min_m) | (depth_m > max_m)] = 0.0
    return depth_m
