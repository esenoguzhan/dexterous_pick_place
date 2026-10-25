"""The whole pipeline on ray-cast scenes seen by two cameras, against the ground truth."""

import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

import synthetic_scene as sim
from restore_perception.core import snapshot_io
from restore_perception.core.config import load_config, load_objects
from restore_perception.core.gallery import Gallery
from restore_perception.core.pipeline import Pipeline
from restore_perception.core.sources import FileSource
from restore_perception.core.types import PerceptionError
from test_geometry import CONFIG

CUBE = dict(kind="box", dims=(0.055, 0.055, 0.055), color=(200, 160, 90))
SPONGE = dict(kind="box", dims=(0.090, 0.060, 0.045), color=(235, 220, 40))
TUNA = dict(kind="cylinder", dims=(0.0425, 0.040), color=(60, 90, 200))
BOTTLE = dict(kind="cylinder", dims=(0.045, 0.140), color=(250, 200, 0), lying=True)
MUG = dict(kind="cylinder", dims=(0.040, 0.080), color=(170, 170, 180))


def scene():
    return [
        sim.on_table(x=0.15, y=-0.10, yaw_deg=30, **CUBE),
        sim.on_table(x=-0.15, y=-0.12, yaw_deg=-20, **SPONGE),
        sim.on_table(x=0.10, y=0.15, **TUNA),
        sim.on_table(x=-0.20, y=0.12, yaw_deg=60, **BOTTLE),
    ]


def match_truth(objects, shapes):
    """Each shape's nearest detected object, with the position error in metres."""
    pairs = []
    for s in shapes:
        dists = [np.linalg.norm(o.position[:2] - np.array(s.centre[:2])) for o in objects]
        k = int(np.argmin(dists))
        pairs.append((s, objects[k], dists[k]))
    return pairs


def yaw_error_deg(est, truth_deg, period_deg):
    d = (math.degrees(est) - truth_deg) % period_deg
    return min(d, period_deg - d)


@pytest.fixture(scope="module")
def config():
    return load_config(CONFIG / "perception.yaml")


def test_geometry_only(config):
    shapes = scene()
    frames, _ = sim.frames(shapes, sim.CAMERAS)
    snap = Pipeline(config).run(frames, "geometry")
    assert len(snap.objects) == 4, [(o.id, o.position.round(3).tolist()) for o in snap.objects]
    assert abs(snap.table_height - sim.TABLE_Z) < 0.002
    for s, obj, dist in match_truth(snap.objects, shapes):
        assert dist < 0.01, (s, obj.position)
        assert abs(obj.position[2] - s.centre[2]) < 0.01
        assert np.allclose(obj.size, s.resting_size, atol=0.012), (s.resting_size, obj.size)
        assert obj.cameras_seen == 3 and obj.support_id == "table" and obj.label == "unknown"
    cube, sponge, _, bottle = [obj for _, obj, _ in match_truth(snap.objects, shapes)]
    assert yaw_error_deg(cube.yaw, 30, 90) < 3
    assert yaw_error_deg(sponge.yaw, -20, 180) < 3
    assert yaw_error_deg(bottle.yaw, 60, 180) < 3
    q = Rotation.from_quat(cube.orientation_xyzw)
    assert abs(q.apply([0, 0, 1])[2] - 1) < 1e-3  # upright: only yaw
    assert snap.timings["total"] > 0


def test_touching_pair_is_split_by_sam_prompts(config):
    shapes = [sim.on_table(x=0.0, y=0.0, **SPONGE),
              sim.on_table(x=0.0, y=0.0302 + 0.0276, yaw_deg=0, **CUBE)]  # 2 mm gap: one cluster
    frames, labels = sim.frames(shapes, sim.CAMERAS)
    geometry = Pipeline(config).run(frames)
    assert len(geometry.objects) == 1  # merged without SAM
    seg = sim.FakeSegmenter(labels)
    snap = Pipeline(config, segmenter=seg).run(frames)
    assert len(snap.objects) == 2 and seg.calls["point"] >= 1
    for s, obj, dist in match_truth(snap.objects, shapes):
        assert dist < 0.012, (s.centre, obj.position)


def test_register_identify_and_known_size_fallback(config, tmp_path):
    cfg = load_config(overrides={"segment": {"depth_poor": {"enabled": True}}})
    objects = load_objects(CONFIG / "objects.yaml")
    gallery = Gallery(tmp_path / "gallery")
    names = {"wooden_cube": CUBE, "sponge": SPONGE, "tuna_can": TUNA, "steel_mug": MUG}
    for i, (name, spec) in enumerate(names.items()):
        shape = sim.on_table(x=0.05, y=0.0, yaw_deg=15 * i, no_depth=(name == "steel_mug"), **spec)
        frames, labels = sim.frames([shape], sim.CAMERAS)
        pipe = Pipeline(cfg, objects, sim.FakeSegmenter(labels, frames), sim.FakeEmbedder(), gallery)
        stored, total, added, _ = pipe.register(frames, name, up_side="yellow" if name == "sponge" else "")
        assert stored == name and added == 2 and total == 2

    shapes = [sim.on_table(x=0.15, y=-0.10, yaw_deg=40, **CUBE),
              sim.on_table(x=-0.15, y=-0.12, yaw_deg=-20, **SPONGE),
              sim.on_table(x=0.10, y=0.15, **TUNA),
              sim.on_table(x=-0.15, y=0.15, no_depth=True, **MUG)]
    frames, labels = sim.frames(shapes, sim.CAMERAS)
    snap = Pipeline(cfg, objects, sim.FakeSegmenter(labels, frames), sim.FakeEmbedder(), gallery).run(frames)
    by_label = {o.label: o for o in snap.objects}
    assert set(by_label) == set(names), [(o.label, o.match) for o in snap.objects]
    for s, obj, dist in match_truth(snap.objects, shapes):
        assert dist < 0.015, (obj.label, s.centre, obj.position)
    mug = by_label["steel_mug"]
    assert mug.known_size and mug.depth_poor and not mug.depth_reliable
    assert np.allclose(mug.size, objects["steel_mug"]["size_m"])
    assert by_label["tuna_can"].known_size  # depth_reliable: false in objects.yaml
    assert not by_label["wooden_cube"].known_size and by_label["wooden_cube"].symmetry == 4
    assert by_label["sponge"].up_side == "yellow" and by_label["sponge"].up_side_matters
    assert all(len(o.embedding) and abs(np.linalg.norm(o.embedding) - 1) < 1e-5 for o in snap.objects)


def test_stacked_object_rests_on_the_other(config):
    base = sim.on_table(x=0.0, y=0.0, **SPONGE)
    top = sim.Shape("box", (0.0, 0.0, sim.TABLE_Z + 0.045 + 0.0275), CUBE["dims"], CUBE["color"], 10)
    frames, labels = sim.frames([base, top], sim.CAMERAS)
    snap = Pipeline(config, segmenter=sim.FakeSegmenter(labels)).run(frames)
    assert len(snap.objects) == 2
    lower, upper = sorted(snap.objects, key=lambda o: o.position[2])
    assert lower.support_id == "table" and upper.support_id == lower.id
    assert abs(upper.bottom_height - 0.045) < 0.01 and abs(upper.size[2] - 0.055) < 0.012


def test_save_load_replay_and_change_mask(config, tmp_path):
    shapes = scene()
    frames, labels = sim.frames(shapes, sim.CAMERAS)
    pipe = Pipeline(config, segmenter=sim.FakeSegmenter(labels))
    reference = pipe.run(frames, "reference")
    folder = snapshot_io.save_snapshot(reference, tmp_path)
    assert (folder / "camera_1" / "masks.png").exists() and len(list((folder / "objects").glob("*.ply"))) == 4

    loaded = snapshot_io.load_snapshot(folder, config["cameras"])
    assert [o.id for o in loaded.objects] == [o.id for o in reference.objects]
    assert np.allclose(loaded.objects[0].points, reference.objects[0].points)
    replay = Pipeline(config).run(FileSource(folder, config).capture(), "replay")
    assert len(replay.objects) == 4

    moved = scene()
    moved[0] = sim.on_table(x=0.25, y=-0.05, yaw_deg=30, **CUBE)
    frames2, labels2 = sim.frames(moved, sim.CAMERAS, rng=np.random.default_rng(5))
    current = Pipeline(config, segmenter=sim.FakeSegmenter(labels2)).run(frames2, "current", reference=loaded)
    cube = match_truth(current.objects, moved[:1])[0][1]
    others = [o for o in current.objects if o is not cube]
    assert cube.changed > 0.5 and all(o.changed < 0.1 for o in others), [o.changed for o in current.objects]
    assert all(v < 0.05 for v in current.changed_fraction.values()) and not current.warnings


def test_missing_pose_and_moved_table_are_reported(config):
    frames, _ = sim.frames(scene(), sim.CAMERAS)
    pipe = Pipeline(config)
    pipe.run(frames)
    frames[0].T_world_cam = None
    with pytest.raises(PerceptionError, match="no pose"):
        pipe.run(frames)
    frames, _ = sim.frames(scene(), sim.CAMERAS)
    for f in frames:  # every camera 2 cm lower than calibrated: the table appears 2 cm higher
        f.T_world_cam = f.T_world_cam.copy()
        f.T_world_cam[2, 3] += 0.02
    snap = pipe.run(frames)
    assert any("table height changed" in w for w in snap.warnings)
