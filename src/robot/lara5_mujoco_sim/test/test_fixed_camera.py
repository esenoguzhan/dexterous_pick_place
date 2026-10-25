"""Check the measured base offset and downward optical direction together."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory


def description(**mappings):
    share = Path(get_package_share_directory("lara5_inspire_hand_description"))
    return ET.fromstring(xacro.process_file(
        str(share / "urdf/lara5_inspire_hand.urdf.xacro"), mappings=mappings).toxml())


def joint_pose(joint):
    origin = joint.find("origin")
    xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
    roll, pitch, yaw = [float(v) for v in origin.get("rpy", "0 0 0").split()]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    result = np.eye(4)
    result[:3, :3] = np.array([
        [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
        [-sp, cp*sr, cp*cr],
    ])
    result[:3, 3] = xyz
    return result


def test_measured_mount_and_optical_forward_use_arm_base_axes():
    root = description()
    mount = root.find("joint[@name='camera_1_mount_joint']")
    optical = root.find("joint[@name='camera_1_color_optical_joint']")
    assert mount.find("parent").get("link") == "lara5_root_link"
    body_pose = joint_pose(mount)
    np.testing.assert_allclose(body_pose[:3, 3], [-.555, -.450, .900])
    base_optical = body_pose @ joint_pose(optical)
    s = math.sqrt(.5)
    # Optical Z looks forward and down, with no yaw or roll of the housing.
    np.testing.assert_allclose(base_optical[:3, 2], [s, 0, -s], atol=1e-12)
    np.testing.assert_allclose(base_optical[:3, 0], [0, -1, 0], atol=1e-12)
    np.testing.assert_allclose(base_optical[:3, 3], [-.555, -.450, .900])
    # The pedestal's rotated mounting plate changes world X/Y, not just Z.
    world_base = joint_pose(root.find("joint[@name='mount_frame_neg_y_joint']"))
    world_camera = world_base @ base_optical
    np.testing.assert_allclose(world_camera[2, 3], 1.815, atol=1e-12)
    assert .25 < world_camera[0, 3] < .27
    assert world_camera[1, 3] < -.4


def test_simulation_inherits_the_urdf_optical_pose_without_another_offset():
    config = yaml.safe_load((Path(__file__).resolve().parents[1] /
                             "config/mujoco_sim.yaml").read_text())["external_camera"]
    assert config["parent_frame"] == "camera_1_color_optical_frame"
    np.testing.assert_allclose(config["position"], [0, 0, 0])
    np.testing.assert_allclose(config["orientation_xyzw"], [0, 0, 0, 1])
    assert description().find("link[@name='camera_1_color_optical_frame']") is not None


def test_measured_mount_can_be_disabled_for_the_calibration_broadcaster():
    root = description(fixed_camera="false")
    assert root.find("link[@name='camera_1_link']") is None
    assert root.find("link[@name='camera_1_color_optical_frame']") is None
