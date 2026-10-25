"""The orchestrator core on hand-made scenes: diff, ordering, grasp and place poses, hand checks."""

import math
from pathlib import Path

import numpy as np
import pytest

import scenes as sc
from restore_task.core import grasp as gr
from restore_task.core import hand
from restore_task.core.config import DEFAULTS, load_config, primitive_for
from restore_task.core.diff import diff, match
from restore_task.core.order import plan
from restore_task.core.poses import Placement, from_xyz_rpy, inverse, world_yaw, wrap
from restore_task.core.report import Report
from restore_perception.core.config import load_objects

CONFIG = Path(__file__).resolve().parents[1] / "config" / "task.yaml"
OBJECTS = Path(__file__).resolve().parents[2] / "restore_perception" / "config" / "objects.yaml"


@pytest.fixture(scope="module")
def cfg():
    return load_config(CONFIG)


@pytest.fixture(scope="module")
def objects():
    return load_objects(OBJECTS)


# ---------- configuration ----------

def test_task_yaml_matches_defaults_and_typos_fail(cfg):
    assert cfg["diff"] == DEFAULTS["diff"] and cfg["pick_place"] == DEFAULTS["pick_place"]
    with pytest.raises(ValueError, match="diff.moved_distance"):
        load_config(overrides={"diff": {"moved_distance": 0.02}})
    merged = load_config(overrides={"primitives": {"pinch_wide": {"squeeze_m": 0.002}}})
    assert merged["primitives"]["pinch_wide"]["squeeze_m"] == 0.002
    assert merged["primitives"]["pinch_wide"]["fingers"] == ["thumb", "index", "middle"]


def test_primitive_per_object(cfg, objects):
    assert primitive_for("wooden_cube", objects, cfg)[0] == "pinch_wide"
    name, params = primitive_for("sponge", objects, cfg)
    assert name == "pinch_wide" and params["squeeze_m"] == 0.004
    assert primitive_for("something_new", objects, cfg)[0] == "wrap_top"


# ---------- 5 diff ----------

def test_matching_by_name_embedding_and_size():
    ref = [sc.obj("r0", "wooden_cube", 0.0, 0.0), sc.obj("r1", "sponge", 0.2, 0.0, labelled=False),
           sc.obj("r2", "tuna_can", -0.2, 0.0, labelled=False, embedding=False)]
    cur = [sc.obj("c0", "tuna_can", -0.1, 0.1, labelled=False, embedding=False),
           sc.obj("c1", "sponge", 0.25, 0.05, labelled=False), sc.obj("c2", "wooden_cube", 0.1, 0.1),
           sc.obj("c3", "mouse", 0.3, 0.2)]
    assert match(ref, cur, DEFAULTS["diff"]) == {"c2": "r0", "c1": "r1", "c0": "r2"}


def test_symmetry_aware_displacement(cfg, objects):
    ref = sc.scene("ref", sc.obj("r0", "wooden_cube", 0.10, 0.10, 0), sc.obj("r1", "sponge", -0.10, 0.10, 0),
                   sc.obj("r2", "tuna_can", 0.0, -0.15, 0), sc.obj("r3", "soup_can", 0.25, -0.1, 0))
    cur = sc.scene("cur", sc.obj("c0", "wooden_cube", 0.10, 0.10, 95),  # 95 deg = 5 deg for a cube
                   sc.obj("c1", "sponge", -0.10, 0.10, 160),  # 160 deg = -20 deg for a 2-fold box
                   sc.obj("c2", "tuna_can", 0.0, -0.15, 70),  # round: yaw does not matter
                   sc.obj("c3", "soup_can", 0.05, -0.1, 0))  # moved 20 cm
    d = diff(ref, cur, objects, cfg)
    tasks = {t.label: t for t in d.tasks}
    assert set(tasks) == {"sponge", "soup_can"}
    assert sorted(cid for cid, _ in d.in_place) == ["c0", "c2"]
    sponge = tasks["sponge"]
    assert math.isclose(math.degrees(sponge.dyaw), -160 + 180 - 0, abs_tol=0.5) or \
        math.isclose(math.degrees(sponge.dyaw), 20, abs_tol=0.5)
    assert abs(wrap(sponge.home.yaw - sponge.current.yaw - sponge.dyaw)) < 1e-9
    assert math.isclose(tasks["soup_can"].dxy, 0.20, abs_tol=1e-6)
    assert np.allclose(tasks["soup_can"].home.xy, [0.25, -0.1])


def test_icp_finds_the_real_turn_of_an_asymmetric_object(cfg, objects):
    # The mouse turned by 150 degrees; perception reports its yaw modulo 180 degrees (-30).
    ref = sc.scene("ref", sc.obj("r0", "mouse", 0.0, 0.0, 0))
    cur = sc.scene("cur", sc.obj("c0", "mouse", 0.10, 0.05, 150))
    task = diff(ref, cur, objects, cfg).tasks[0]
    assert math.isclose(math.degrees(task.dyaw), -150, abs_tol=2.0), math.degrees(task.dyaw)
    assert "ambiguous_yaw" not in task.flags
    # a shape that looks the same both ways round (a plain box) is ambiguous: the smaller turn, flagged
    box = sc.obj("c0", "mouse", 0.10, 0.05, 150)
    box.points = sc.box_cloud(box.size, seed=1) @ np.array([[math.cos(2.618), -math.sin(2.618), 0],
                                                     [math.sin(2.618), math.cos(2.618), 0], [0, 0, 1]]).T
    box.points = (box.points + box.position).astype(np.float32)
    ref_box = sc.obj("r0", "mouse", 0.0, 0.0, 0)
    ref_box.points = (sc.box_cloud(ref_box.size) + ref_box.position).astype(np.float32)
    task = diff(sc.scene("ref", ref_box), sc.scene("cur", box), objects, cfg).tasks[0]
    assert "ambiguous_yaw" in task.flags and math.isclose(math.degrees(task.dyaw), 30, abs_tol=0.5)
    # without clouds, likewise
    cur.objects[0].points = np.zeros((0, 3), np.float32)
    task = diff(ref, cur, objects, cfg).tasks[0]
    assert "ambiguous_yaw" in task.flags and abs(math.degrees(task.dyaw)) <= 90


def test_noise_guard_refusals_missing_and_new(cfg, objects):
    ref = sc.scene("ref", sc.obj("r0", "wooden_cube", 0.0, 0.0), sc.obj("r1", "sponge", 0.2, 0.0, up_side="yellow"),
                   sc.obj("r2", "tuna_can", -0.2, 0.0), sc.obj("r3", "soup_can", 0.0, 0.2),
                   sc.obj("r4", "wooden_egg", 0.3, 0.3))
    cur = sc.scene("cur", sc.obj("c0", "wooden_cube", 0.02, 0.0, changed=0.02),  # 2 cm but pixels unchanged
                   sc.obj("c1", "sponge", 0.25, 0.0, up_side="dark"),
                   sc.obj("c2", "tuna_can", -0.2, 0.1, confidence=0.3),
                   sc.obj("c3", "soup_can", 0.7, 0.2),  # out of reach
                   sc.obj("c5", "mouse", -0.3, -0.2))
    d = diff(ref, cur, objects, cfg)
    assert d.tasks == [] and d.in_place == [("c0", "r0")]
    reasons = {r.label: r.reason for r in d.refused}
    assert "flipping" in reasons["sponge"] and "unsure" in reasons["tuna_can"] and "reachable" in reasons["soup_can"]
    assert d.missing == ["r4"] and d.new == ["c5"]


# ---------- 6 ordering ----------

def test_object_on_another_home_moves_first(cfg, objects):
    ref = sc.scene("ref", sc.obj("r0", "wooden_cube", 0.0, 0.0), sc.obj("r1", "sponge", 0.2, 0.1))
    cur = sc.scene("cur", sc.obj("c0", "wooden_cube", 0.2, 0.1),  # on the sponge's home
                   sc.obj("c1", "sponge", -0.2, -0.1))
    d = diff(ref, cur, objects, cfg)
    p = plan(d, cur, cfg, hand_xy=(-0.2, -0.1))  # the hand is next to the sponge
    assert [s.task.label for s in p.steps] == ["wooden_cube", "sponge"]
    assert p.dependencies["c1"] == ["c0"]


def test_swap_cycle_parks_one_object(cfg, objects):
    ref = sc.scene("ref", sc.obj("r0", "wooden_cube", 0.0, 0.0), sc.obj("r1", "tuna_can", 0.2, 0.0))
    cur = sc.scene("cur", sc.obj("c0", "wooden_cube", 0.2, 0.0), sc.obj("c1", "tuna_can", 0.0, 0.0))
    d = diff(ref, cur, objects, cfg)
    p = plan(d, cur, cfg, hand_xy=(0.0, 0.0))
    assert [(s.task.label, s.park) for s in p.steps] == [("tuna_can", True), ("wooden_cube", False),
                                                         ("tuna_can", False)]
    spot = p.steps[0].target
    for o in cur.objects + ref.objects:  # the spot is clear of every footprint and home
        assert np.linalg.norm(spot.xy - o.position[:2]) > 0.07
    assert math.isclose(spot.z, sc.TABLE_Z + 0.02)


def test_stacks_and_blocked_homes(cfg, objects):
    # The cube was put on the moved sponge: the cube goes first.
    ref = sc.scene("ref", sc.obj("r0", "wooden_cube", -0.2, 0.0), sc.obj("r1", "sponge", 0.2, 0.0),
                   sc.obj("r2", "tuna_can", 0.0, 0.25))
    cur = sc.scene("cur", sc.obj("c1", "sponge", 0.0, -0.2),
                   sc.obj("c0", "wooden_cube", 0.0, -0.2, support="c1", z=sc.TABLE_Z + 0.045 + 0.0275),
                   sc.obj("c2", "tuna_can", 0.0, 0.25),  # in place
                   sc.obj("c9", "mouse", 0.2, 0.0))  # new object on the sponge's home
    d = diff(ref, cur, objects, cfg)
    p = plan(d, cur, cfg)
    assert [s.task.label for s in p.steps] == ["wooden_cube"]
    assert [r.label for r in p.refused] == ["sponge"] and "c9" in p.refused[0].reason


# ---------- 7 grasp and place ----------

T_FLANGE_TCP = from_xyz_rpy([0, 0, 0], [0, 0, -math.pi / 2])


def make_task(cfg, objects, label, yaw_deg, home_yaw_deg=None):
    ref = sc.scene("ref", sc.obj("r0", label, 0.2, 0.1, home_yaw_deg if home_yaw_deg is not None else yaw_deg))
    cur = sc.scene("cur", sc.obj("c0", label, -0.1, -0.1, yaw_deg))
    return diff(ref, cur, objects, cfg).tasks[0]


def test_candidate_yaws_follow_symmetry(cfg):
    p = cfg["primitives"]
    assert len(gr.candidate_yaws(p["pinch_wide"], 4)) == 4
    assert len(gr.candidate_yaws(p["pinch_wide"], 2)) == 2
    assert len(gr.candidate_yaws(p["wrap_top"], 0)) == 12
    assert sorted(round(math.degrees(y)) for y in gr.candidate_yaws(p["wrap_body"], 1)) == [-90, 90]


def test_place_puts_the_object_on_its_home(cfg, objects):
    task = make_task(cfg, objects, "sponge", 30, home_yaw_deg=-50)
    name, primitive = primitive_for(task.label, objects, cfg)
    grasps = gr.candidates(task, primitive, T_FLANGE_TCP, flange_yaw=0.0, neighbours=[], cfg=cfg["pick_place"])
    assert [g.wrist_turn for g in grasps] == sorted(g.wrist_turn for g in grasps)
    for g in grasps:
        # the grasp is above the object, palm down, and the object is where the hand expects it
        assert np.allclose(gr.object_in_hand(g.grasp, g)[:3, 3], [task.current.x, task.current.y, task.current.z])
        assert g.pre_grasp[2, 3] - g.grasp[2, 3] == pytest.approx(0.10)
        poses = gr.place_poses(task.home, g, primitive, task.rolls, cfg["pick_place"])
        placed = gr.object_in_hand(poses.place, g)
        assert np.allclose(placed[:3, 3], [task.home.x, task.home.y, task.home.z + 0.005])
        assert math.isclose(wrap(world_yaw(placed[:3, :3]) - task.home.yaw), 0.0, abs_tol=1e-9)
        assert poses.pre_place[2, 3] - poses.place[2, 3] == pytest.approx(0.10)
    # the sponge's width is across its narrow side at grasp yaw 0
    assert math.isclose(grasps[0].width, 0.060) or math.isclose(grasps[0].width, 0.090)


def test_rolling_objects_are_lowered_to_their_height(cfg, objects):
    task = make_task(cfg, objects, "soup_can", 0)
    _, primitive = primitive_for(task.label, objects, cfg)
    g = gr.candidates(task, primitive, T_FLANGE_TCP, 0.0, [], cfg["pick_place"])[0]
    placed = gr.object_in_hand(gr.place_poses(task.home, g, primitive, True, cfg["pick_place"]).place, g)
    assert math.isclose(placed[2, 3], task.home.z)


def test_side_grasp_faces_the_object_and_backs_out(cfg):
    task = make_task(cfg, {"steel_mug": {"grasp": "wrap_body"}}, "tuna_can", 0)
    primitive = cfg["primitives"]["wrap_body"]
    for g in gr.candidates(task, primitive, T_FLANGE_TCP, 0.0, [], cfg["pick_place"]):
        tcp = task.current.matrix() @ g.T_object_tcp
        to_centre = task.current.matrix()[:3, 3] - tcp[:3, 3]
        assert np.allclose(tcp[:3, 2], to_centre / np.linalg.norm(to_centre), atol=1e-6)  # palm z at the centre
        poses = gr.place_poses(task.home, g, primitive, False, cfg["pick_place"])
        assert math.isclose(poses.retreat[2, 3], poses.place[2, 3], abs_tol=1e-6)  # backs out horizontally
        assert np.linalg.norm(poses.retreat[:2, 3] - poses.place[:2, 3]) == pytest.approx(0.10)


def test_grasps_next_to_a_neighbour_rank_last(cfg, objects):
    task = make_task(cfg, objects, "wooden_cube", 0)
    _, primitive = primitive_for(task.label, objects, cfg)
    neighbour = np.array([[task.current.x + 0.04, task.current.y]])
    grasps = gr.candidates(task, primitive, T_FLANGE_TCP, 0.0, [neighbour], cfg["pick_place"])
    assert all(g.tight for g in grasps)  # the neighbour is inside the hand radius for every yaw
    assert not any(g.tight for g in gr.candidates(task, primitive, T_FLANGE_TCP, 0.0, [neighbour + 0.2],
                                                  cfg["pick_place"]))


# ---------- 7.6 hand ----------

def test_hand_targets_and_checks(cfg):
    p, check = cfg["primitives"]["pinch_wide"], cfg["grasp_check"]
    width = 0.055
    contact, target = hand.angles_for_width(p, width), hand.squeeze_target(p, width)
    assert np.all(target[1:4] > contact[1:4]) and np.all(hand.preshape(p, width)[1:4] < contact[1:4])
    assert np.isclose(target[4], 1.43)  # curled fingers stay curled
    assert hand.check_grasp(p, width, contact, check).state == "held"
    assert hand.check_grasp(p, width, target, check).state == "missed"
    one_finger = target.copy()
    one_finger[2] = contact[2]
    assert hand.check_grasp(p, width, one_finger, check).state == "unclear"
    assert hand.dropped(p, width, contact, target, check)
    assert not hand.dropped(p, width, contact, contact + 0.002, check)


def test_report(cfg, objects):
    ref = sc.scene("ref", sc.obj("r0", "wooden_cube", 0.0, 0.0), sc.obj("r1", "sponge", 0.2, 0.0),
                   sc.obj("r2", "wooden_egg", -0.2, 0.0))
    cur = sc.scene("cur", sc.obj("c0", "wooden_cube", 0.0, 0.0), sc.obj("c1", "sponge", 0.25, 0.05))
    report = Report()
    report.attempt("r0", "wooden_cube")
    report.finish(diff(ref, cur, objects, cfg), ref)
    assert report.counts() == {"restored": 1, "not restored": 1, "missing": 1}
