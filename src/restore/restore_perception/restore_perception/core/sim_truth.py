"""Score perception estimates against the simulator's ground truth (docs/perception_simulation_eval.md).

Pure functions, no ROS: the truth is the JSON the simulator publishes on /sim/objects, the estimates
are dicts built from ObjectState messages (see sim_eval.estimate_from_msg).
"""

from __future__ import annotations

import math
from statistics import mean

SYM_CONTINUOUS = "continuous"


def yaw_from_quaternion(x, y, z, w):
    """Rotation about Z (ZYX convention); exact for yaw-only quaternions, fine for a near-level table."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(angle, period):
    """Fold an angle difference into [-period/2, period/2)."""
    return (angle + period / 2.0) % period - period / 2.0


def yaw_error_deg(estimate_yaw, truth_yaw, symmetry):
    """Absolute yaw error in degrees under the footprint symmetry; None for round objects.

    Perception defines yaw as the long side modulo 180 degrees, so a 2-fold object is compared
    modulo 180 and a square (4-fold) modulo 90."""
    if symmetry == SYM_CONTINUOUS:
        return None
    period = 2.0 * math.pi / float(symmetry)
    return abs(math.degrees(wrap(estimate_yaw - truth_yaw, period)))


def size_error_mm(estimate_size, truth_size):
    """Per-axis differences of the sorted dimensions (largest first), in millimetres."""
    e = sorted((float(v) for v in estimate_size), reverse=True)
    t = sorted((float(v) for v in truth_size), reverse=True)
    return [1000.0 * (a - b) for a, b in zip(e, t)]


def match(truth, estimates, radius):
    """Greedy one-to-one matching by horizontal distance: [(truth index, estimate index, distance)]."""
    pairs = []
    for i, t in enumerate(truth):
        for j, e in enumerate(estimates):
            d = math.hypot(e["position"][0] - t["position"][0], e["position"][1] - t["position"][1])
            if d <= radius:
                pairs.append((d, i, j))
    pairs.sort()
    used_t, used_e, result = set(), set(), []
    for d, i, j in pairs:
        if i in used_t or j in used_e:
            continue
        used_t.add(i)
        used_e.add(j)
        result.append((i, j, d))
    return result


def score_capture(truth, estimates, radius=0.08):
    """Rows of errors for the matched objects, plus the missed truths and the unmatched estimates."""
    matched = match(truth, estimates, radius)
    rows = []
    for i, j, _ in matched:
        t, e = truth[i], estimates[j]
        delta = [1000.0 * (e["position"][k] - t["position"][k]) for k in range(3)]
        rows.append({
            "name": t["name"],
            "estimate_id": e["id"],
            "label": e.get("label", ""),
            "dx_mm": delta[0], "dy_mm": delta[1], "dz_mm": delta[2],
            "dxy_mm": math.hypot(delta[0], delta[1]),
            "d3_mm": math.sqrt(sum(v * v for v in delta)),
            "yaw_error_deg": yaw_error_deg(e["yaw"], t["long_axis_yaw_rad"], t["yaw_symmetry"]),
            "size_error_mm": size_error_mm(e["size"], t["size_m"]),
            "size_estimate_mm": [1000.0 * v for v in e["size"]],
            "size_truth_mm": [1000.0 * v for v in t["size_m"]],
            "confidence": e.get("confidence"),
            "cameras_seen": e.get("cameras_seen"),
            "depth_reliable": e.get("depth_reliable"),
        })
    matched_t = {i for i, _, _ in matched}
    matched_e = {j for _, j, _ in matched}
    return {
        "rows": rows,
        "missed": [t["name"] for i, t in enumerate(truth) if i not in matched_t],
        "extra": [{"id": e["id"], "label": e.get("label", ""), "position": list(e["position"]),
                   "size_mm": [1000.0 * v for v in e["size"]]} for j, e in enumerate(estimates) if j not in matched_e],
    }


def summarize(captures, truth_names):
    """Per-object and overall statistics over several scored captures."""
    per_object = {}
    for name in truth_names:
        rows = [r for c in captures for r in c["rows"] if r["name"] == name]
        yaws = [r["yaw_error_deg"] for r in rows if r["yaw_error_deg"] is not None]
        per_object[name] = {
            "detected": len(rows),
            "captures": len(captures),
            "xy_mean_mm": mean(r["dxy_mm"] for r in rows) if rows else None,
            "xy_max_mm": max(r["dxy_mm"] for r in rows) if rows else None,
            "z_mean_mm": mean(r["dz_mm"] for r in rows) if rows else None,
            "yaw_mean_deg": mean(yaws) if yaws else None,
            "yaw_max_deg": max(yaws) if yaws else None,
            "size_mean_mm": [mean(r["size_error_mm"][k] for r in rows) for k in range(3)] if rows else None,
        }
    all_rows = [r for c in captures for r in c["rows"]]
    all_yaws = [r["yaw_error_deg"] for r in all_rows if r["yaw_error_deg"] is not None]
    overall = {
        "captures": len(captures),
        "expected": len(truth_names) * len(captures),
        "detected": len(all_rows),
        "extra": sum(len(c["extra"]) for c in captures),
        "xy_mean_mm": mean(r["dxy_mm"] for r in all_rows) if all_rows else None,
        "xy_max_mm": max(r["dxy_mm"] for r in all_rows) if all_rows else None,
        "z_mean_abs_mm": mean(abs(r["dz_mm"]) for r in all_rows) if all_rows else None,
        "yaw_mean_deg": mean(all_yaws) if all_yaws else None,
        "yaw_max_deg": max(all_yaws) if all_yaws else None,
    }
    return {"objects": per_object, "overall": overall}


def _fmt(value, digits=1):
    if value is None:
        return "-"
    if isinstance(value, (list, tuple)):
        return " / ".join(_fmt(v, digits) for v in value)
    return f"{value:+.{digits}f}" if isinstance(value, float) and digits else f"{value:.0f}"


def markdown_report(summary, captures, meta):
    o = summary["overall"]
    lines = [
        "# Perception in simulation: pose estimation against MuJoCo ground truth", "",
        f"Captures: {o['captures']}, objects expected: {o['expected']}, detected: {o['detected']}, "
        f"false positives: {o['extra']}. Match radius {1000 * meta['radius']:.0f} mm. "
        f"Cameras: {meta.get('cameras', '?')}. Models: {meta.get('models', 'off')}.", "",
        "## Per object (mean over captures)", "",
        "| Object | Detected | XY error mean / max [mm] | Z error mean [mm] | Yaw error mean / max [deg] | Size error L / W / H [mm] |",
        "|---|---|---|---|---|---|",
    ]
    for name, s in summary["objects"].items():
        lines.append(f"| {name} | {s['detected']}/{s['captures']} | {_fmt(s['xy_mean_mm'])} / {_fmt(s['xy_max_mm'])} | "
                     f"{_fmt(s['z_mean_mm'])} | {_fmt(s['yaw_mean_deg'])} / {_fmt(s['yaw_max_deg'])} | {_fmt(s['size_mean_mm'])} |")
    lines += ["", f"Overall: XY mean {_fmt(o['xy_mean_mm'])} mm, XY max {_fmt(o['xy_max_mm'])} mm, |Z| mean "
                  f"{_fmt(o['z_mean_abs_mm'])} mm, yaw mean {_fmt(o['yaw_mean_deg'])} deg, yaw max {_fmt(o['yaw_max_deg'])} deg.",
              "", "## Every capture", ""]
    for n, c in enumerate(captures):
        lines += [f"### Capture {n}: {c.get('message', '')}", "",
                  "| Object | Estimate | dx / dy / dz [mm] | XY [mm] | Yaw err [deg] | Size est [mm] | Size true [mm] | Conf | Cams | Depth ok |",
                  "|---|---|---|---|---|---|---|---|---|---|"]
        for r in c["rows"]:
            lines.append(f"| {r['name']} | {r['estimate_id']} ({r['label']}) | {_fmt([r['dx_mm'], r['dy_mm'], r['dz_mm']])} | "
                         f"{_fmt(r['dxy_mm'])} | {_fmt(r['yaw_error_deg'])} | {_fmt(r['size_estimate_mm'], 0)} | "
                         f"{_fmt(r['size_truth_mm'], 0)} | {_fmt(r['confidence'], 2)} | {r['cameras_seen']} | {r['depth_reliable']} |")
        if c["missed"]:
            lines.append(f"\nMissed: {', '.join(c['missed'])}")
        if c["extra"]:
            lines.append("\nUnmatched estimates (no true object within the match radius):\n")
            for e in c["extra"]:
                lines.append(f"- {e['id']} ({e['label']}) at x {e['position'][0]:+.3f} y {e['position'][1]:+.3f} "
                             f"z {e['position'][2]:.3f} m, size {_fmt(e['size_mm'], 0)} mm")
        lines.append("")
    return "\n".join(lines)
