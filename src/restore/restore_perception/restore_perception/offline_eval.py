"""Run the pipeline on saved captures and write overlays and a report (section 11.1). No ROS needed.

    python -m restore_perception.offline_eval CAPTURE [CAPTURE ...] --out DIR [options]
    ros2 run restore_perception offline_eval CAPTURE ... --out DIR [options]

A CAPTURE is a snapshot folder written by the node (save_to_disk), a test bench download (capture.zip
or its unpacked folder), a folder of bench downloads from different cameras, or a folder of any of
these. Camera poses come from the snapshot, else from --extrinsics (fixed cameras), else, with
--estimate-pose, from the largest plane in a single camera's depth: the table is then assumed level
at --table-z with the world origin where the optical axis meets it, which is enough to try the
geometry on a bench capture taken before calibration.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import yaml

from .core import overlay, snapshot_io
from .core.config import config_dir, load_config, load_objects
from .core.extrinsics import load_extrinsics
from .core.gallery import Gallery
from .core.geometry import fit_plane, pixels_to_points
from .core.pipeline import Pipeline
from .core.sources import FileSource
from .core.types import PerceptionError

def estimate_pose(frame, table_z, cfg):
    """World pose of a camera from the table plane in its own depth image (see the module docstring)."""
    valid = frame.depth > 0
    v, u = np.nonzero(valid[::4, ::4])
    points = pixels_to_points(frame.depth, frame.intrinsics.matrix(), v * 4, u * 4).astype(np.float64)
    if len(points) < 1000:
        raise PerceptionError(f"{frame.name}: too little depth to find the table")
    plane, inliers = fit_plane(points, cfg["table"]["distance_threshold_m"], 1000, up=-points.mean(axis=0))
    normal, d = plane[:3], plane[3]  # normal points towards the camera
    hit = -d / normal[2] * np.array([0.0, 0.0, 1.0])  # optical axis meets the table
    x = np.array([1.0, 0.0, 0.0]) - normal[0] * normal
    x /= np.linalg.norm(x)
    R_cw = np.column_stack([x, np.cross(normal, x), normal])  # world axes in camera coordinates
    T = np.eye(4)
    T[:3, :3] = R_cw.T
    T[:3, 3] = -R_cw.T @ hit + [0.0, 0.0, table_z]
    return T, float(inliers.mean())


def report_rows(snapshot):
    rows = []
    for o in snapshot.objects:
        rows.append(f"| {o.id} | {o.label} | {o.label_score:.2f} | {o.position[0]:+.3f} {o.position[1]:+.3f} "
                    f"{o.position[2]:.3f} | {np.degrees(o.yaw):+.0f} | {1000 * o.size[0]:.0f} x {1000 * o.size[1]:.0f}"
                    f" x {1000 * o.size[2]:.0f} | {o.cameras_seen} | {len(o.points)} | {o.support_id} | "
                    f"{'yes' if o.known_size else ''} | {o.confidence:.2f} |")
    return rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("captures", nargs="+", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=config_dir() / "perception.yaml")
    parser.add_argument("--objects", type=Path, default=config_dir() / "objects.yaml")
    parser.add_argument("--extrinsics", type=Path)
    parser.add_argument("--gallery", type=Path, help="gallery folder (needs --models)")
    parser.add_argument("--models", action="store_true", help="load SAM 2.1 and DINOv2 (paths from the config)")
    parser.add_argument("--reference", type=Path, help="snapshot folder to compute the change mask against")
    parser.add_argument("--estimate-pose", action="store_true", help="pose an uncalibrated camera from the table")
    parser.add_argument("--table-z", type=float, default=0.90)
    parser.add_argument("--set", action="append", default=[], metavar="GROUP.KEY=YAML",
                        help="override a parameter, e.g. --set workspace.pixel_stride=1")
    args = parser.parse_args(argv)

    overrides = {}
    for item in args.set:
        key, _, value = item.partition("=")
        *groups, leaf = key.split(".")
        node = overrides
        for g in groups:
            node = node.setdefault(g, {})
        node[leaf] = yaml.safe_load(value)
    cfg = load_config(args.config, overrides)
    extrinsics = load_extrinsics(args.extrinsics)
    segmenter = embedder = None
    if args.models:
        from .core.embed import Dinov2Embedder
        from .core.segment import Sam2Segmenter

        segmenter, embedder = Sam2Segmenter(cfg["models"]), Dinov2Embedder(cfg["models"])
        segmenter.load()
        embedder.load()
    gallery = Gallery(args.gallery) if args.gallery else None
    pipeline = Pipeline(cfg, load_objects(args.objects), segmenter, embedder, gallery)
    reference = snapshot_io.load_snapshot(args.reference, cfg["cameras"]) if args.reference else None

    captures = []
    for path in args.captures:
        source = FileSource(path, cfg)
        captures.extend(source.captures)
    args.out.mkdir(parents=True, exist_ok=True)
    lines = ["# Offline evaluation", "", f"{time.strftime('%Y-%m-%d %H:%M')}, config {args.config}", ""]
    failures = 0
    for path in captures:
        name = path.stem if path.is_file() else path.name
        lines += [f"## {name}", ""]
        try:
            frames = snapshot_io.load_frames(path, cfg["cameras"])
            for frame in frames:
                if frame.T_world_cam is not None:
                    continue
                cam = extrinsics.get(frame.serial)
                if cam is not None and not cam.on_hand:
                    frame.T_world_cam = cam.world_pose()
                elif args.estimate_pose and len(frames) == 1:
                    frame.T_world_cam, share = estimate_pose(frame, args.table_z, cfg)
                    lines.append(f"Pose estimated from the table plane ({share:.0%} of the depth on it).")
                else:
                    raise PerceptionError(f"{frame.name} ({frame.serial}) has no pose: pass --extrinsics, or "
                                          f"--estimate-pose for a single camera")
            snapshot = pipeline.run(frames, name, reference)
        except PerceptionError as exc:
            failures += 1
            lines += [f"Failed: {exc}", ""]
            print(f"{name}: failed: {exc}")
            continue
        overlays = {f.name: overlay.draw(f, snapshot.objects) for f in snapshot.frames}
        folder = snapshot_io.save_snapshot(snapshot, args.out, overlays)
        lines += [f"{len(snapshot.objects)} objects, table at {snapshot.table_height:.3f} m, "
                  f"{snapshot.timings['total']:.0f} ms ({json.dumps(snapshot.timings)})", ""]
        lines += [f"- {w}" for w in snapshot.warnings]
        lines += ["| id | label | score | position [m] | yaw [deg] | size [mm] | cameras | points | on | "
                  "known size | confidence |", "|---|---|---|---|---|---|---|---|---|---|---|"]
        lines += report_rows(snapshot) + ["", f"Overlays: `{folder.relative_to(args.out)}/<camera>/overlay.png`", ""]
        print(f"{name}: {len(snapshot.objects)} objects in {snapshot.timings['total']:.0f} ms -> {folder}")
    (args.out / "report.md").write_text("\n".join(lines) + "\n")
    print(f"report: {args.out / 'report.md'}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
