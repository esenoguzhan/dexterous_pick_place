"""Ground-truth object poses for perception tests, without ROS or rendering."""

import json
import math

import mujoco
import numpy as np
import pytest

from lara5_mujoco_sim import ground_truth as gt

OBJECTS = [
    {"name": "cube", "type": "box", "size": [0.025, 0.025, 0.025], "pos": [0.1, 0.2, 0.3], "yaw_deg": 20.0},
    {"name": "bar", "type": "box", "size": [0.02, 0.05, 0.015], "pos": [0.4, 0.5, 0.6], "yaw_deg": -35.0},
    {"name": "can", "type": "cylinder", "size": [0.04, 0.02], "pos": [0.7, 0.8, 0.9]},
    {"name": "ball", "type": "sphere", "size": [0.03], "pos": [1.0, 1.1, 1.2]},
]


def model_with(objects):
    bodies = "".join(
        f'<body name="{o["name"]}" pos="{" ".join(map(str, o["pos"]))}" '
        f'quat="{" ".join(map(str, gt.body_quaternion_wxyz(o)))}"><freejoint/>'
        f'<geom type="{o["type"]}" size="{" ".join(map(str, o["size"]))}"/></body>' for o in objects)
    model = mujoco.MjModel.from_xml_string(f"<mujoco><worldbody>{bodies}</worldbody></mujoco>")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def test_dimensions_and_symmetry():
    assert gt.object_dimensions(OBJECTS[0]) == pytest.approx([0.05, 0.05, 0.05])
    assert gt.object_dimensions(OBJECTS[1]) == pytest.approx([0.04, 0.10, 0.03])
    assert gt.object_dimensions(OBJECTS[2]) == pytest.approx([0.08, 0.08, 0.04])
    assert gt.object_dimensions(OBJECTS[3]) == pytest.approx([0.06, 0.06, 0.06])
    assert [gt.yaw_symmetry(o) for o in OBJECTS] == [4, 2, "continuous", "continuous"]


def test_ground_truth_reports_true_pose_and_long_side_yaw():
    model, data = model_with(OBJECTS)
    truth = json.loads(gt.ground_truth_json(model, data, OBJECTS, stamp=1.5))
    assert truth["frame_id"] == "world" and truth["stamp"] == 1.5
    by_name = {o["name"]: o for o in truth["objects"]}
    assert by_name["cube"]["position"] == pytest.approx([0.1, 0.2, 0.3])
    assert math.degrees(by_name["cube"]["yaw_rad"]) == pytest.approx(20.0)
    # The bar is longer along its local Y, so the long side is 90 degrees from the body yaw.
    assert math.degrees(by_name["bar"]["yaw_rad"]) == pytest.approx(-35.0)
    assert math.degrees(by_name["bar"]["long_axis_yaw_rad"]) == pytest.approx(55.0)
    q = by_name["cube"]["orientation_xyzw"]
    assert np.linalg.norm(q) == pytest.approx(1.0) and q[3] == pytest.approx(math.cos(math.radians(10)))


def test_unknown_bodies_are_skipped():
    model, data = model_with(OBJECTS[:1])
    truth = gt.ground_truth(model, data, OBJECTS)
    assert [o["name"] for o in truth["objects"]] == ["cube"]
