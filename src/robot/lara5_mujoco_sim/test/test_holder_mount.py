"""CAD datums, actual mounting pair and coherent housing/optical placement."""

import json
from pathlib import Path
import struct
import subprocess
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

from lara5_mujoco_sim.camera import camera_pose
from lara5_mujoco_sim.mjcf_builder import build_scene


def inputs():
    description = Path(get_package_share_directory("lara5_inspire_hand_description"))
    assets = description/"meshes/camera_holder"
    cad = json.loads((assets/"source.json").read_text())
    config = yaml.safe_load((Path(__file__).resolve().parents[1]/"config/mujoco_sim.yaml").read_text())
    return description, assets, cad, config


def test_supplied_mesh_is_in_mm_and_keeps_original_dimensions():
    _, assets, cad, _ = inputs()
    data = (assets/"tcp_cam_holder_v2.stl").read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    assert count == cad["mesh_triangles"]
    assert len(data) == 84+count*50
    faces = np.frombuffer(data, offset=84, dtype=np.dtype([
        ("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")]))
    vertices = faces["vertices"].reshape(-1, 3)
    np.testing.assert_allclose([vertices.min(0), vertices.max(0)], cad["mesh_bounds_mm"], atol=1e-4)


def test_optical_pose_and_housing_rear_follow_outer_pair_not_center_screw():
    _, _, cad, config = inputs()
    c = config["wrist_camera"]
    holes = np.array(cad["camera_screw_pair_cad_mm"])/1000
    assert len(holes) == 2
    np.testing.assert_allclose(holes[1]-holes[0], [.045, 0, 0], atol=1e-12)
    rear = holes.mean(0)
    normal = np.asarray(cad["platform_normal_cad"])
    expected = rear + normal*(.02505-.0043) + [.0175, 0, 0]
    position, optical = camera_pose(c)
    np.testing.assert_allclose(position, expected, atol=1e-12)
    np.testing.assert_allclose(optical[:, 2], normal, atol=1e-12)
    np.testing.assert_allclose(optical[:, 0], [-1, 0, 0], atol=1e-12)
    center = np.asarray(c["housing_center_optical"])
    np.testing.assert_allclose(position + optical @ (center-[0, 0, .012525]), rear, atol=1e-12)
    np.testing.assert_allclose(position + optical @ [0, 0, .0043],
                               rear+normal*.02505+[.0175, 0, 0], atol=1e-12)


def test_shared_urdf_holder_frame_and_rendered_optical_frame_agree():
    description, _, cad, config = inputs()
    urdf = subprocess.check_output([
        "xacro", str(description/"urdf/lara5_inspire_hand.urdf.xacro")], text=True)
    root = ET.fromstring(urdf)
    holder = root.find("link[@name='right_wrist_camera_holder']")
    assert holder is not None
    assert holder.find("collision") is None
    np.testing.assert_allclose([float(v) for v in holder.find("visual/geometry/mesh").get("scale").split()], [.001]*3)
    joint = root.find("joint[@name='right_wrist_camera_holder_mount_joint']")
    assert joint.find("parent").get("link") == "right_hand_base_link"
    np.testing.assert_allclose([float(v) for v in joint.find("origin").get("xyz").split()], cad["holder_to_hand_xyz_m"])
    model = mujoco.MjModel.from_xml_string(build_scene(urdf, config).mjcf)
    data = mujoco.MjData(model)
    # Several joint poses, kinematics only: camera and bracket move together.
    for angle in (0., .4, -.7):
        jid = model.joint("lara5_joint6").id
        data.qpos[model.jnt_qposadr[jid]] = angle
        mujoco.mj_forward(model, data)
        parent = model.body(config["wrist_camera"]["parent_frame"]).id
        sensor = model.camera("wrist_camera").id
        position, optical = camera_pose(config["wrist_camera"])
        rotation = data.xmat[parent].reshape(3, 3)
        np.testing.assert_allclose(data.cam_xpos[sensor], data.xpos[parent]+rotation@position, atol=1e-12)
        np.testing.assert_allclose(data.cam_xmat[sensor].reshape(3, 3)@np.diag([1, -1, -1]), rotation@optical, atol=1e-12)
