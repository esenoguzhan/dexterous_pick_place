"""Section 7.6: hand shapes and grasp checks from finger positions only (no force sensing).

Joint order: thumb rotation, thumb bend, index, middle, ring, little finger. 0 is open; closing
increases every angle. A finger's "span" is the distance between the angle at which it touches an
object of the planned width and the squeeze target past it.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

HAND_JOINTS = ["right_thumb_finger_proximal_yaw", "right_thumb_finger_proximal_pitch",
               "right_index_finger_proximal", "right_middle_finger_proximal",
               "right_ring_finger_proximal", "right_pinky_finger_proximal"]
UPPER = np.array([1.164, 0.586, 1.438, 1.438, 1.438, 1.438])
CLOSING_JOINT = {"thumb": 1, "index": 2, "middle": 3, "ring": 4, "pinky": 5}
OPEN = np.zeros(6)


def angles_for_width(primitive, width):
    """Finger angles at which the primitive's shape touches an object of this width (measured table)."""
    rows = sorted(primitive["width_to_angles"], key=lambda r: r[0])
    widths = np.array([r[0] for r in rows], float)
    table = np.array([r[1] for r in rows], float)
    angles = np.array([np.interp(width, widths, table[:, j]) for j in range(6)])
    return np.clip(angles, 0.0, UPPER)


def preshape(primitive, width):
    return angles_for_width(primitive, width + primitive["open_margin_m"])


def squeeze_target(primitive, width):
    return angles_for_width(primitive, width - primitive["squeeze_m"])


@dataclass
class GraspCheck:
    state: str  # held, missed or unclear
    ratios: dict  # finger -> share of its span it stopped short of the target

    @property
    def held(self):
        return self.state == "held"


def _ratios(primitive, width, measured):
    contact, target = angles_for_width(primitive, width), squeeze_target(primitive, width)
    ratios = {}
    for finger in primitive["fingers"]:
        j = CLOSING_JOINT[finger]
        span = max(target[j] - contact[j], 1e-3)
        ratios[finger] = float((target[j] - measured[j]) / span)
    return ratios


def check_grasp(primitive, width, measured, cfg):
    """Fingers stopped well short of the squeeze target are held open by the object."""
    ratios = _ratios(primitive, width, np.asarray(measured, float))
    others = [r for f, r in ratios.items() if f != "thumb"]
    if all(r <= cfg["missed_ratio"] for r in ratios.values()):
        state = "missed"
    elif ratios.get("thumb", 1.0) >= cfg["held_ratio"] and any(r >= cfg["held_ratio"] for r in others):
        state = "held"
    else:
        state = "unclear"  # treated as missed
    return GraspCheck(state, {f: round(r, 2) for f, r in ratios.items()})


def dropped(primitive, width, held_positions, measured, cfg):
    """A held finger that closed further by drop_ratio of its span has lost the object."""
    held = _ratios(primitive, width, np.asarray(held_positions, float))
    now = _ratios(primitive, width, np.asarray(measured, float))
    return any(held[f] - now[f] >= cfg["drop_ratio"] for f in held)
