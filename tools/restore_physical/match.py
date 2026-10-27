"""Which current object is which reference object, and which ones were moved.

Without DINOv2 the identity comes from what the fixed camera measures well: the hue-saturation
histogram of the mask, the object's height and the radius of its top surface. The assignment is
a minimum-cost matching (Hungarian), so two similar objects cannot both claim the same reference.
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import linear_sum_assignment

MAX_COST = 3.0


def appearance(a, b):
    """0 = same colours, 1 = disjoint histograms."""
    ha, hb = np.asarray(a["hist"]), np.asarray(b["hist"])
    return 1.0 - float(np.minimum(ha, hb).sum())


def cost(ref, cur):
    dh = abs(ref["size"][2] - cur["size"][2]) / 0.02
    dr = abs(ref["top_radius"] - cur["top_radius"]) / 0.01
    dv = abs(ref["val"] - cur["val"]) / 0.15 + abs(ref["sat"] - cur["sat"]) / 0.15
    dxy = np.hypot(ref["position"][0] - cur["position"][0], ref["position"][1] - cur["position"][1]) / 0.5
    return 2.0 * appearance(ref, cur) + dh + 0.5 * dr + 0.5 * dv + 0.2 * dxy


def match(reference, current):
    """[(ref, cur, cost)], unmatched reference (missing), unmatched current (new)."""
    if not reference or not current:
        return [], list(reference), list(current)
    C = np.array([[cost(r, c) for c in current] for r in reference])
    rows, cols = linear_sum_assignment(C)
    pairs, used_r, used_c = [], set(), set()
    for i, j in zip(rows, cols):
        if C[i, j] <= MAX_COST:
            pairs.append((reference[i], current[j], float(C[i, j])))
            used_r.add(i)
            used_c.add(j)
    missing = [r for i, r in enumerate(reference) if i not in used_r]
    new = [c for j, c in enumerate(current) if j not in used_c]
    return pairs, missing, new


def displacement(ref, cur):
    return float(np.hypot(ref["position"][0] - cur["position"][0], ref["position"][1] - cur["position"][1]))
