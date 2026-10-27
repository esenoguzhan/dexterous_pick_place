"""Joint-space waypoints for palm-down hand poses: the IK seeds and the choice of hand heading.

A palm-down hand points its fingers horizontally, so the wrist sits behind the hand along the
finger direction. Which heading works depends on where the object is (pointing away from the
base folds the arm up for near objects), so every candidate heading is tried and the solution
closest to the current joints wins.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

import grasp as gr
import ik

SEEDS = Path(__file__).resolve().parent / "config" / "seeds.json"
HEADINGS_DEG = [0.0, 90.0, -90.0, 180.0, 45.0, -45.0, 135.0, -135.0]  # relative to the radial direction


def load_seeds():
    return [np.array(q) for q in json.loads(SEEDS.read_text())] if SEEDS.exists() else []


def save_seed(q, limit=40):
    seeds = load_seeds()
    if all(np.max(np.abs(q - s)) > 0.05 for s in seeds):
        seeds.insert(0, np.asarray(q))
        SEEDS.write_text(json.dumps([[round(float(v), 4) for v in s] for s in seeds[:limit]]))


def wrap_near(q, current):
    """Joints 4 and 6 turn +-2 pi: take the turn closest to the current joints."""
    q = np.array(q, float)
    for i in (3, 5):
        while q[i] - current[i] > math.pi and q[i] - 2 * math.pi >= -2 * math.pi:
            q[i] -= 2 * math.pi
        while current[i] - q[i] > math.pi and q[i] + 2 * math.pi <= 2 * math.pi:
            q[i] += 2 * math.pi
    return q


def solve_hand(r, top, clearance, T_flange_hand, current, radius=0.035, headings=None, extra_seeds=(),
               max_seeds=6, check=None, validate=None):
    """(q, heading_yaw, T_world_flange) for the open hand `clearance` above the object top.

    check(yaw) -> bool can reject a heading before any IK (e.g. the wrist camera would hit something);
    validate(q, T_flange) -> bool a solution (e.g. the straight descent from it must be plannable)."""
    radial = gr.radial_yaw(top)
    seeds = [np.asarray(current)] + [np.asarray(s) for s in extra_seeds] + load_seeds()[:max_seeds]
    best = None
    for offset in (headings if headings is not None else HEADINGS_DEG):
        yaw = radial + math.radians(offset)
        if check is not None and not check(yaw):
            continue
        T_flange = gr.flange_pose(gr.hand_pose(top, yaw, clearance, radius), T_flange_hand)
        for seed in seeds:
            q, ep, er = ik.solve(r, T_flange, seed)
            if ep < 2e-3 and er < 0.01:
                q = wrap_near(q, current)
                if validate is not None and not validate(q, T_flange):
                    continue
                cost = float(np.sum(np.abs(q - current)))
                if best is None or cost < best[3]:
                    best = (q, yaw, T_flange, cost)
                break
        if best is not None and offset == 0.0 and best[3] < 1.0:
            break
    if best is None:
        return None
    save_seed(best[0])
    return best[:3]
