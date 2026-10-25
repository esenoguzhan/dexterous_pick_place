"""Reading and writing data/ (section 8.3), and loading test bench downloads for offline tests.

    data/snapshots/<snapshot_id>/
        snapshot.json        everything in SceneSnapshot except clouds, plus each camera's
                             intrinsics and pose, so the capture can be run again offline
        objects/<id>.ply     per-object point cloud, world frame
        camera_N/color.png, depth.png (16-bit millimetres), masks.png (object index + 1), overlay.png
"""

from __future__ import annotations

import json
import zipfile
from pathlib import Path

import cv2
import numpy as np

from .types import CameraFrame, Intrinsics, ObjectState, PerceptionError, Snapshot

BENCH_META = "intrinsics.json"


# ---------- images ----------

def _read_image(data_or_path, flags):
    if isinstance(data_or_path, (str, Path)):
        image = cv2.imread(str(data_or_path), flags)
    else:
        image = cv2.imdecode(np.frombuffer(data_or_path, np.uint8), flags)
    if image is None:
        raise PerceptionError(f"cannot read image {data_or_path if isinstance(data_or_path, (str, Path)) else ''}")
    return image


def read_color(data_or_path):
    return cv2.cvtColor(_read_image(data_or_path, cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)


def read_depth_mm(data_or_path):
    """16-bit millimetre PNG to float32 metres, 0 = no data."""
    depth = _read_image(data_or_path, cv2.IMREAD_UNCHANGED)
    if depth.dtype != np.uint16:
        raise PerceptionError("depth.png must be a 16-bit PNG in millimetres")
    return depth.astype(np.float32) / 1000.0


def write_color(path, rgb):
    cv2.imwrite(str(path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))


def write_depth_mm(path, depth_m):
    cv2.imwrite(str(path), np.clip(np.round(depth_m * 1000.0), 0, 65535).astype(np.uint16))


# ---------- point clouds ----------

def write_ply(path, points):
    points = np.asarray(points, np.float32).reshape(-1, 3)
    header = ("ply\nformat binary_little_endian 1.0\n"
              f"element vertex {len(points)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n")
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(points.astype("<f4").tobytes())


def read_ply(path):
    """Reads the binary xyz files write_ply writes."""
    data = Path(path).read_bytes()
    end = data.index(b"end_header\n") + len(b"end_header\n")
    count = int(next(line.split()[-1] for line in data[:end].decode("ascii").splitlines()
                     if line.startswith("element vertex")))
    return np.frombuffer(data[end:end + 12 * count], "<f4").reshape(count, 3).copy()


# ---------- captures on disk ----------

def _bench_items(folder):
    """Single-camera test bench downloads in a folder: sub-folders or zips holding intrinsics.json."""
    items = []
    for child in sorted(folder.iterdir()):
        if child.is_dir() and (child / BENCH_META).exists():
            items.append(child)
        elif child.suffix == ".zip" and _zip_has_meta(child):
            items.append(child)
    return items


def _zip_has_meta(path):
    try:
        with zipfile.ZipFile(path) as archive:
            return BENCH_META in archive.namelist()
    except zipfile.BadZipFile:
        return False


def _bench_serial(item):
    if item.is_dir():
        return json.loads((item / BENCH_META).read_text()).get("camera_serial")
    with zipfile.ZipFile(item) as archive:
        return json.loads(archive.read(BENCH_META)).get("camera_serial")


def is_capture(path):
    """A snapshot folder, one bench download (folder or zip), or bench downloads of distinct cameras."""
    path = Path(path)
    if path.is_file():
        return path.suffix == ".zip" and _zip_has_meta(path)
    if not path.is_dir():
        return False
    if (path / "snapshot.json").exists() or (path / BENCH_META).exists():
        return True
    items = _bench_items(path)
    serials = [_bench_serial(item) for item in items]
    return bool(items) and len(set(serials)) == len(serials)


def _bench_frame(item, names):
    if item.is_dir():
        meta = json.loads((item / BENCH_META).read_text())
        color, depth = read_color(item / "color.png"), read_depth_mm(item / "depth.png")
    else:
        with zipfile.ZipFile(item) as archive:
            meta = json.loads(archive.read(BENCH_META))
            if "depth.png" not in archive.namelist():
                raise PerceptionError(f"{item.name} has no depth.png (capture taken without depth)")
            color, depth = read_color(archive.read("color.png")), read_depth_mm(archive.read("depth.png"))
    serial = str(meta["camera_serial"])
    frames = int((meta.get("depth") or {}).get("median_frames", 1))
    return CameraFrame(names.get(serial, f"camera_{serial}"), serial, color, depth,
                       Intrinsics.from_dict(meta["intrinsics"]), float(meta.get("stamp", 0.0)), frames=frames)


def load_frames(path, names):
    """CameraFrames of a saved capture; poses are set only for snapshot folders."""
    path = Path(path)
    if path.is_dir() and (path / "snapshot.json").exists():
        meta = json.loads((path / "snapshot.json").read_text())
        frames = []
        for cam in meta["cameras"]:
            folder = path / cam["name"]
            frames.append(CameraFrame(
                cam["name"], cam["serial"], read_color(folder / "color.png"), read_depth_mm(folder / "depth.png"),
                Intrinsics.from_dict(cam["intrinsics"]), float(cam["stamp"]),
                np.array(cam["T_world_cam"]) if cam.get("T_world_cam") else None, int(cam.get("frames", 1)),
                cam.get("frame_id", "")))
        return frames
    if path.is_file() or (path / BENCH_META).exists():
        return [_bench_frame(path, names)]
    return [_bench_frame(item, names) for item in _bench_items(path)]


# ---------- snapshots ----------

def object_to_dict(obj):
    return {
        "id": obj.id, "label": obj.label, "label_score": round(float(obj.label_score), 4),
        "position": [round(float(v), 5) for v in obj.position],
        "orientation_xyzw": [round(float(v), 6) for v in obj.orientation_xyzw],
        "yaw_deg": round(float(np.degrees(obj.yaw)), 2),
        "size": [round(float(v), 4) for v in obj.size],
        "footprint": [[round(float(v), 4) for v in p] for p in obj.footprint],
        "symmetry": int(obj.symmetry), "up_side_matters": bool(obj.up_side_matters), "up_side": obj.up_side,
        "support_id": obj.support_id, "bottom_height": round(float(obj.bottom_height), 4),
        "cameras_seen": int(obj.cameras_seen), "depth_reliable": bool(obj.depth_reliable),
        "confidence": round(float(obj.confidence), 3),
        "sam_score": None if obj.sam_score is None else round(float(obj.sam_score), 3),
        "known_size": bool(obj.known_size), "depth_poor": bool(obj.depth_poor),
        "changed": None if obj.changed is None else round(float(obj.changed), 3),
        "points": int(len(obj.points)), "match": obj.match,
        "embedding": [round(float(v), 6) for v in obj.embedding],
    }


def object_from_dict(d, points=None):
    return ObjectState(
        id=d["id"], label=d["label"], label_score=d["label_score"], position=np.array(d["position"]),
        orientation_xyzw=np.array(d["orientation_xyzw"]), yaw=np.radians(d["yaw_deg"]), size=np.array(d["size"]),
        footprint=np.array(d["footprint"]).reshape(-1, 3), symmetry=d["symmetry"],
        up_side_matters=d["up_side_matters"], up_side=d["up_side"], support_id=d["support_id"],
        embedding=np.array(d["embedding"], np.float32),
        points=np.zeros((0, 3), np.float32) if points is None else points, cameras_seen=d["cameras_seen"],
        depth_reliable=d["depth_reliable"], confidence=d["confidence"], sam_score=d.get("sam_score"),
        known_size=d.get("known_size", False), depth_poor=d.get("depth_poor", False), match=d.get("match", {}),
        bottom_height=d.get("bottom_height", 0.0), changed=d.get("changed"))


def camera_to_dict(frame):
    return {"name": frame.name, "serial": frame.serial, "stamp": frame.stamp, "frames": frame.frames,
            "frame_id": frame.frame_id, "intrinsics": frame.intrinsics.to_dict(),
            "T_world_cam": None if frame.T_world_cam is None else np.round(frame.T_world_cam, 8).tolist()}


def label_image(snapshot, frame):
    """masks.png: object index + 1 per pixel, 0 = no object."""
    labels = np.zeros(frame.depth.shape, np.uint8)
    for i, obj in enumerate(snapshot.objects):
        mask = obj.masks.get(frame.name)
        if mask is not None:
            labels[mask] = min(i + 1, 255)
    return labels


def save_snapshot(snapshot, root, overlays=None):
    """Write the snapshot in the 8.3 layout; returns its folder."""
    folder = Path(root) / "snapshots" / snapshot.snapshot_id
    (folder / "objects").mkdir(parents=True, exist_ok=True)
    meta = {
        "snapshot_id": snapshot.snapshot_id, "stamp": snapshot.stamp,
        "table_plane": [float(v) for v in snapshot.table_plane], "table_height": snapshot.table_height,
        "warnings": snapshot.warnings, "timings_ms": snapshot.timings, "changed_fraction": snapshot.changed_fraction,
        "cameras": [camera_to_dict(f) for f in snapshot.frames],
        "objects": [object_to_dict(o) for o in snapshot.objects],
    }
    (folder / "snapshot.json").write_text(json.dumps(meta, indent=1))
    for obj in snapshot.objects:
        write_ply(folder / "objects" / f"{obj.id}.ply", obj.points)
    for frame in snapshot.frames:
        cam = folder / frame.name
        cam.mkdir(exist_ok=True)
        write_color(cam / "color.png", frame.color)
        write_depth_mm(cam / "depth.png", frame.depth)
        cv2.imwrite(str(cam / "masks.png"), label_image(snapshot, frame))
        if overlays and frame.name in overlays:
            write_color(cam / "overlay.png", overlays[frame.name])
    return folder


def load_snapshot(folder, names=None):
    """A saved snapshot with its frames and objects (point clouds from the .ply files)."""
    folder = Path(folder)
    meta = json.loads((folder / "snapshot.json").read_text())
    frames = load_frames(folder, names or {})
    objects = []
    for d in meta["objects"]:
        ply = folder / "objects" / f"{d['id']}.ply"
        objects.append(object_from_dict(d, read_ply(ply) if ply.exists() else None))
    return Snapshot(meta["snapshot_id"], meta["stamp"], np.array(meta["table_plane"]), objects, frames,
                    meta.get("warnings", []), meta.get("timings_ms", {}), meta.get("table_height", 0.0),
                    meta.get("changed_fraction", {}))
