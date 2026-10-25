"""Section 5: which objects moved, and where each one belongs."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from restore_perception.core.types import SYM_CONTINUOUS, SYM_NONE
from scipy.optimize import linear_sum_assignment
from scipy.spatial import cKDTree

from .config import primitive_for
from .poses import Placement, rot_z, wrap
from .scene import yaw_of


@dataclass
class Task:
    """Put one object back. home.yaw is current.yaw plus the true rotation, so placing the object
    with the grasp's object-to-flange transform turns it by exactly dyaw (5.5)."""

    object_id: str
    reference_id: str
    label: str
    current: Placement
    home: Placement
    size: np.ndarray
    symmetry: int
    dxy: float
    dyaw: float
    primitive: str
    rolls: bool
    current_footprint: np.ndarray  # N x 2, world xy
    home_footprint: np.ndarray  # N x 2, the reference footprint
    current_support: str  # current object id or "table"
    home_support: str  # reference object id or "table"
    flags: list = field(default_factory=list)


@dataclass
class Refusal:
    object_id: str
    reference_id: str
    label: str
    reason: str


@dataclass
class Diff:
    tasks: list
    in_place: list  # (object id, reference id)
    refused: list
    missing: list  # reference ids without a current match
    new: list  # current ids without a reference match
    pairs: dict  # current id -> reference id

    def reference_of(self, object_id):
        return self.pairs.get(object_id)

    def current_of(self, reference_id):
        return next((c for c, r in self.pairs.items() if r == reference_id), None)


# ---------- 5.1 matching ----------

def _size_difference(a, b):
    a, b = np.sort(np.asarray(a))[::-1], np.sort(np.asarray(b))[::-1]
    return float(np.max(np.abs(a - b) / np.maximum(b, 1e-3)))


def match(reference, current, cfg):
    """current id -> reference id: by gallery name, then the Hungarian algorithm on embeddings and size."""
    def unique_labels(objects):
        counts = {}
        for o in objects:
            counts[o.label] = counts.get(o.label, 0) + 1
        return {o.label: o for o in objects if o.label != "unknown" and counts[o.label] == 1}

    ref_labels, cur_labels = unique_labels(reference), unique_labels(current)
    pairs = {cur_labels[k].id: ref_labels[k].id for k in ref_labels.keys() & cur_labels.keys()}
    refs = [o for o in reference if o.id not in pairs.values()]
    curs = [o for o in current if o.id not in pairs]
    if refs and curs:
        cost = np.zeros((len(curs), len(refs)))
        ok = np.zeros_like(cost, bool)
        for i, c in enumerate(curs):
            for j, r in enumerate(refs):
                size = _size_difference(c.size, r.size)
                distance = float(np.linalg.norm(c.position[:2] - r.position[:2]))
                if len(c.embedding) and len(r.embedding):
                    cosine = float(c.embedding @ r.embedding)
                    cost[i, j] = (1.0 - cosine) + size + 0.1 * distance
                    ok[i, j] = cosine >= cfg["unknown_min_cosine"] and size <= cfg["unknown_size_tolerance"]
                else:  # no DINOv2: size, then nearness, decides
                    cost[i, j] = 0.5 + size + 0.1 * distance
                    ok[i, j] = size <= cfg["unknown_size_tolerance"]
                if c.label != "unknown" and r.label != "unknown" and c.label != r.label:
                    ok[i, j] = False
        rows, cols = linear_sum_assignment(np.where(ok, cost, cost + 1e3))
        for i, j in zip(rows, cols):
            if ok[i, j]:
                pairs[curs[i].id] = refs[j].id
    return pairs


# ---------- 5.2 and 5.3 displacement ----------

def symmetric(dyaw, symmetry):
    """Yaw difference that matters for an object with this symmetry."""
    if symmetry == SYM_CONTINUOUS:
        return 0.0
    if symmetry >= 2:
        return wrap(dyaw, 2 * math.pi / symmetry)
    return wrap(dyaw)


def icp_yaw(source, target, centre_source, centre_target, yaw0, iterations):
    """Rotation about z (and translation) taking the source cloud onto the target, from yaw0.

    Returns the yaw and the mean distance of the trimmed nearest neighbours."""
    tree = cKDTree(target)
    yaw = yaw0
    t = centre_target - rot_z(yaw) @ centre_source
    residual = math.inf
    for _ in range(iterations):
        moved = source @ rot_z(yaw).T + t
        dist, idx = tree.query(moved)
        keep = dist <= 3.0 * np.median(dist) + 1e-9
        a, b = source[keep], target[idx[keep]]
        ca, cb = a.mean(axis=0), b.mean(axis=0)
        h = (a[:, :2] - ca[:2]).T @ (b[:, :2] - cb[:2])
        yaw = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
        t = cb - rot_z(yaw) @ ca
        residual = float(dist[keep].mean())
    return yaw, residual


def true_rotation(cur, ref, raw, cfg):
    """Section 5.3: for symmetry none, decide between raw and raw + 180 degrees with ICP.

    Returns (dyaw, ambiguous)."""
    if len(cur.points) < cfg["icp_min_points"] or len(ref.points) < cfg["icp_min_points"]:
        return wrap(raw, math.pi), True  # the smaller turn; flagged
    src, tgt = np.asarray(cur.points, float), np.asarray(ref.points, float)
    fits = [icp_yaw(src, tgt, cur.position, ref.position, start, cfg["icp_iterations"])
            for start in (raw, raw + math.pi)]
    (yaw_a, res_a), (yaw_b, res_b) = fits
    best, worst = (fits[0], res_b) if res_a <= res_b else (fits[1], res_a)
    if (worst - best[1]) / max(worst, 1e-9) < cfg["icp_ambiguity"]:
        return wrap(raw, math.pi), True  # both fit about as well: perception's yaw, the smaller turn
    return wrap(best[0]), False


# ---------- 5.4 and 5.5 ----------

def _in_area(xy, area):
    return bool(np.all(xy >= np.array(area["min"])) and np.all(xy <= np.array(area["max"])))


def diff(reference, current, objects, config):
    """reference, current: core.scene.Scene. objects: objects.yaml entries. Returns a Diff."""
    cfg = config["diff"]
    pairs = match(reference.objects, current.objects, cfg)
    refs, curs = reference.by_id(), current.by_id()
    tasks, in_place, refused = [], [], []
    moved_yaw = math.radians(cfg["moved_yaw_deg"])
    for cur_id, ref_id in sorted(pairs.items()):
        cur, ref = curs[cur_id], refs[ref_id]
        label = cur.label if cur.label != "unknown" else ref.label
        cur_yaw, ref_yaw = yaw_of(cur), yaw_of(ref)
        flags = []
        raw = ref_yaw - cur_yaw
        if cur.symmetry == SYM_NONE:
            dyaw, ambiguous = true_rotation(cur, ref, raw, cfg)
            if ambiguous:
                flags.append("ambiguous_yaw")
        else:
            dyaw = symmetric(raw, cur.symmetry)
        dxy = float(np.linalg.norm(cur.position[:2] - ref.position[:2]))
        cur_support_ref = pairs.get(cur.support_id, cur.support_id) if cur.support_id != "table" else "table"
        support_changed = cur_support_ref != ref.support_id
        if support_changed:
            flags.append("support_changed")
        moved = dxy > cfg["moved_distance_m"] or abs(dyaw) > moved_yaw or support_changed
        noise = (cur.changed is not None and cur.changed < cfg["noise_changed"] and not support_changed
                 and dxy < 2 * cfg["moved_distance_m"] and abs(dyaw) < 2 * moved_yaw)
        if not moved or noise:
            in_place.append((cur_id, ref_id))
            continue

        reason = None
        if cur.up_side_matters and cur.up_side and ref.up_side and cur.up_side != ref.up_side:
            reason = f"turned over ({ref.up_side} side was up, now {cur.up_side}): flipping is not supported"
        elif min(cur.confidence, ref.confidence) < cfg["min_confidence"]:
            reason = f"perception is unsure of it (confidence {min(cur.confidence, ref.confidence):.2f})"
        elif not _in_area(cur.position[:2], config["reach_area"]):
            reason = "its current place is outside the reachable area"
        elif not _in_area(ref.position[:2], config["reach_area"]):
            reason = "its home is outside the reachable area"
        if reason:
            refused.append(Refusal(cur_id, ref_id, label, reason))
            continue

        primitive, _ = primitive_for(label, objects, config)
        tasks.append(Task(
            object_id=cur_id, reference_id=ref_id, label=label,
            current=Placement(*cur.position, cur_yaw),
            home=Placement(*ref.position, cur_yaw + dyaw),
            size=np.asarray(cur.size, float), symmetry=cur.symmetry, dxy=dxy, dyaw=dyaw,
            primitive=primitive, rolls=bool((objects.get(label) or {}).get("rolls", False)),
            current_footprint=np.asarray(cur.footprint, float)[:, :2],
            home_footprint=np.asarray(ref.footprint, float)[:, :2],
            current_support=cur.support_id, home_support=ref.support_id, flags=flags))
    missing = sorted(set(refs) - set(pairs.values()))
    new = sorted(set(curs) - set(pairs))
    return Diff(tasks, in_place, refused, missing, new, pairs)
