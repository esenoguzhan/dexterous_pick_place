"""Actual collision meshes at the computed pose; forward kinematics only."""

import copy
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from lara5_mujoco_sim.mjcf_builder import build_scene
from lara5_object_approach.grasp_geometry import HandModel, from_rpy, grasp_targets
from lara5_object_approach.planning_geometry import pregrasp_target


@pytest.mark.parametrize("yaw", [0, math.pi/2, math.pi, -math.pi/2])
@pytest.mark.parametrize("mode", ["final_grasp_preview", "aligned_pregrasp"])
def test_open_hand_meshes_clear_object_and_table(yaw, mode):
    p = yaml.safe_load((Path(__file__).resolve().parents[1]/"config/grasp_preview.yaml").read_text())[
        "grasp_preview"]["ros__parameters"]
    share = Path(get_package_share_directory("lara5_inspire_hand_description"))
    urdf = xacro.process_file(str(share/"urdf/lara5_inspire_hand.urdf.xacro")).toxml()
    hand = HandModel(urdf, p["flange_link"], p["hand_link"], p["open_joint_names"], p["open_joint_positions"])
    world_object = from_rpy([.25, -.2, .95], [0, 0, yaw])
    flange, grasp, world_hand = grasp_targets(world_object, hand.flange_hand,
        from_rpy(p["hand_to_grasp_xyz"], p["hand_to_grasp_rpy"]),
        from_rpy(p["object_to_grasp_xyz"], p["object_to_grasp_rpy"]))
    if mode == "aligned_pregrasp":
        approach = yaml.safe_load((Path(__file__).resolve().parents[1]/"config/approach.yaml").read_text())[
            "object_approach"]["ros__parameters"]
        flange = pregrasp_target(world_object, flange, grasp, approach["pregrasp_flange_clearance"])
        world_hand = flange @ hand.flange_hand
        assert flange[2, 3] == pytest.approx(1.25)
    root = ET.Element("robot", name="static_grasp_test")
    ET.SubElement(root, "link", name="world")
    ET.SubElement(root, "link", name=p["flange_link"])
    joint = ET.SubElement(root, "joint", name="world_flange", type="fixed")
    ET.SubElement(joint, "parent", link="world")
    ET.SubElement(joint, "child", link=p["flange_link"])
    r = flange[:3, :3]
    rpy = [math.atan2(r[2, 1], r[2, 2]), math.asin(float(-r[2, 0])), math.atan2(r[1, 0], r[0, 0])]
    ET.SubElement(joint, "origin", xyz=" ".join(map(str, flange[:3, 3])), rpy=" ".join(map(str, rpy)))
    source = ET.fromstring(urdf)
    for link in source.findall("link"):
        if link.get("name") in hand.hand_links:
            root.append(copy.deepcopy(link))
    for joint in source.findall("joint"):
        if joint.find("child").get("link") in hand.hand_links:
            root.append(copy.deepcopy(joint))
    config = {"objects": [dict(name="cylinder", type="cylinder", size=[p["cylinder_radius"], p["cylinder_height"]/2],
                              pos=[.25, -.2, .95], mass=.15)]}
    scene = build_scene(ET.tostring(root, encoding="unicode"), config)
    model = mujoco.MjModel.from_xml_string(scene.mjcf)
    data = mujoco.MjData(model)
    for name, angle in hand.resolved_positions.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid >= 0:
            data.qpos[model.jnt_qposadr[jid]] = angle
    mujoco.mj_forward(model, data)  # No integration, control or motion commands.
    hand_id = model.body(p["hand_link"]).id
    np.testing.assert_allclose(data.xpos[hand_id], world_hand[:3, 3], atol=1e-10)
    np.testing.assert_allclose(data.xmat[hand_id].reshape(3, 3), world_hand[:3, :3], atol=1e-10)
    assert not any(contact.dist < -1e-6 for contact in data.contact)
    lowest = math.inf
    gaps = dict(thumb=[], index=[])
    cylinder = model.geom("cylinder_geom").id
    for i in range(model.ngeom):
        name = model.body(model.geom_bodyid[i]).name
        if name not in hand.hand_links or model.geom_contype[i] == 0:
            continue
        mesh = model.geom_dataid[i]
        vertices = model.mesh_vert[model.mesh_vertadr[mesh]:model.mesh_vertadr[mesh]+model.mesh_vertnum[mesh]]
        world_vertices = vertices @ data.geom_xmat[i].reshape(3, 3).T + data.geom_xpos[i]
        lowest = min(lowest, float(world_vertices[:, 2].min()))
        for key in gaps:
            if key in name:
                gaps[key].append(mujoco.mj_geomDistance(model, data, i, cylinder, .3, None))
    assert lowest - p["table_height"] > .08
    # Both opposing regions are close but clear in the specified OPEN posture.
    if mode == "final_grasp_preview":
        assert 0 < min(gaps["thumb"]) < .03
        assert 0 < min(gaps["index"]) < .03
