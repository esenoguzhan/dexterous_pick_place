"""Check mount extraction, transform direction and the open-hand preset."""

import math
from pathlib import Path

import numpy as np
import pytest
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from lara5_object_approach.grasp_geometry import HandModel, from_rpy, grasp_targets, pose_components, transform


CONFIG = Path(__file__).resolve().parents[1]/"config/grasp_preview.yaml"


@pytest.fixture
def geometry():
    p = yaml.safe_load(CONFIG.read_text())["grasp_preview"]["ros__parameters"]
    share = Path(get_package_share_directory("lara5_inspire_hand_description"))
    urdf = xacro.process_file(str(share/"urdf/lara5_inspire_hand.urdf.xacro")).toxml()
    hand = HandModel(urdf, p["flange_link"], p["hand_link"],
                     p["open_joint_names"], p["open_joint_positions"])
    return p, hand, urdf


def test_mount_is_extracted_from_actual_hand_urdf(geometry):
    _, hand, _ = geometry
    np.testing.assert_allclose(hand.flange_hand, from_rpy([0, 0, 0], [0, 0, -math.pi/2]), atol=1e-12)
    assert len(hand.hand_links) > 20
    assert all(angle == 0 for angle in hand.resolved_positions.values())
    assert len(list(hand.visuals(np.eye(4)))) == len(hand.hand_links)


@pytest.mark.parametrize("xyz,yaw", [([.25, -.2, .95], 0), ([.1, .2, .95], math.pi/2),
                                    ([-.3, 0, 1.1], math.pi), ([0, 0, .95], -math.pi/2)])
def test_flange_places_grasp_frame_on_desired_object_frame(geometry, xyz, yaw):
    p, hand, _ = geometry
    obj = from_rpy(xyz, [0, 0, yaw])
    hand_grasp = from_rpy(p["hand_to_grasp_xyz"], p["hand_to_grasp_rpy"])
    object_grasp = from_rpy(p["object_to_grasp_xyz"], p["object_to_grasp_rpy"])
    flange, grasp, world_hand = grasp_targets(obj, hand.flange_hand, hand_grasp, object_grasp)
    np.testing.assert_allclose(flange @ hand.flange_hand @ hand_grasp, obj @ object_grasp, atol=1e-12)
    np.testing.assert_allclose(grasp, world_hand @ hand_grasp, atol=1e-12)
    # Palm normal and defined approach +Z both point down for this preset.
    np.testing.assert_allclose(world_hand[:3, 1], [0, 0, -1], atol=1e-12)
    np.testing.assert_allclose(grasp[:3, 2], [0, 0, -1], atol=1e-12)
    if yaw == 0:
        np.testing.assert_allclose(flange[:3, 3], [.275, -.35, 1.035], atol=1e-12)


@pytest.mark.parametrize("rpy", [[0, 0, 0], [math.pi, 0, 0], [0, math.pi, 0], [.4, -.2, 2.1]])
def test_quaternion_roundtrip_at_half_turns(rpy):
    original = from_rpy([1, 2, 3], rpy)
    np.testing.assert_allclose(transform(*pose_components(original)), original, atol=1e-12)


def test_object_relative_height_is_applied_once(geometry):
    p, hand, _ = geometry
    flange, grasp, _ = grasp_targets(from_rpy([.25, -.2, .95], [0, 0, 0]), hand.flange_hand,
        from_rpy(p["hand_to_grasp_xyz"], p["hand_to_grasp_rpy"]),
        from_rpy([0, 0, .02], p["object_to_grasp_rpy"]))
    assert grasp[2, 3] == pytest.approx(.97)
    assert flange[2, 3] == pytest.approx(1.055)


def test_missing_posture_or_wrong_mount_are_rejected(geometry):
    p, _, urdf = geometry
    with pytest.raises(ValueError, match="Missing open posture"):
        HandModel(urdf, p["flange_link"], p["hand_link"], [], [])
    with pytest.raises(ValueError, match="fixed joint chain"):
        HandModel(urdf, "lara5_root_link", p["hand_link"], p["open_joint_names"], p["open_joint_positions"])
    with pytest.raises(ValueError, match="exceeds limits"):
        HandModel(urdf, p["flange_link"], p["hand_link"], p["open_joint_names"], [9.0]*6)


@pytest.mark.parametrize("q", [[0, 0, 0, 0], [math.nan, 0, 0, 1], [0, 0, 0, 2]])
def test_invalid_quaternions_rejected(q):
    with pytest.raises(ValueError):
        transform([0, 0, 0], q)
