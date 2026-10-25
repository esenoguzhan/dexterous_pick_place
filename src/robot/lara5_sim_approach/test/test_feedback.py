"""Run inside the built Humble image: python3 -m pytest <this file>."""

from pathlib import Path
import subprocess

import cv2
import mujoco
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import yaml

from lara5_mujoco_sim.camera import camera_axes, camera_pose, intrinsics, optical_rotation
from lara5_mujoco_sim.mjcf_builder import apply_initial_state, build_scene
from lara5_sim_approach.trajectory import (duration, interpolation_samples, retime_cartesian,
                                         seconds, validate_trajectory)
from lara5_sim_approach.vision import detect_objects, image_array


def test_camera_conventions():
    position, target = [-1, 1, 2], [0, 0, 1]
    axes = camera_axes(position, target)
    assert np.allclose(axes.T @ axes, np.eye(3))
    assert np.linalg.det(axes) == pytest.approx(1)
    optical = optical_rotation(position, target)
    assert np.allclose(optical[:, 2], (np.array(target)-position) / np.linalg.norm(np.array(target)-position))
    assert np.allclose(optical[:, 1], -axes[:, 1])
    with pytest.raises(ValueError):
        camera_axes([0, 0, 0], [0, 0, 0])


def test_image_stride_and_endianness():
    msg = Image(width=2, height=2, encoding="32FC1", is_bigendian=True, step=12)
    msg.data = np.array([[1., 2., 0.], [3., 4., 0.]], dtype=">f4").tobytes()
    assert np.array_equal(image_array(msg), [[1, 2], [3, 4]])
    msg.step = 4
    with pytest.raises(ValueError):
        image_array(msg)


def test_cartesian_timing_and_jump_rejection():
    names = [f"j{i}" for i in range(6)]
    original = JointTrajectory(joint_names=names, points=[
        JointTrajectoryPoint(positions=[value]*6) for value in [0., .02, .04]])
    result = retime_cartesian(original)
    validate_trajectory(result, names)
    interval = seconds(result.points[1].time_from_start)
    assert 1.5 * .02 / interval <= .12 + 1e-9
    assert 6 * .02 / interval**2 <= .25 + 1e-9
    for position, velocity, acceleration in interpolation_samples(result):
        assert np.all(position >= -1e-12) and np.all(position <= .04 + 1e-12)
        assert np.max(np.abs(velocity)) <= .12 + 1e-9
        assert np.max(np.abs(acceleration)) <= .25 + 1e-9
    original.points[1].positions = [1.]*6
    with pytest.raises(ValueError, match="Discontinuous"):
        retime_cartesian(original)
    result.points[1].time_from_start = duration(0)
    with pytest.raises(ValueError, match="timestamps"):
        validate_trajectory(result, names)


@pytest.fixture(scope="module")
def rendered_scene():
    sim_share = Path(get_package_share_directory("lara5_mujoco_sim"))
    config = yaml.safe_load((sim_share/"config/mujoco_sim.yaml").read_text())
    robot_share = Path(get_package_share_directory("lara5_controller_manager"))
    urdf = subprocess.check_output(["xacro", str(robot_share/"urdf/lara5_inspire_hand_ros2_control.urdf.xacro"),
                                    "deployment_mode:=simulation"], text=True)
    scene = build_scene(urdf, config)
    model = mujoco.MjModel.from_xml_string(scene.mjcf)
    data = mujoco.MjData(model)
    apply_initial_state(model, data, scene)
    for _ in range(1000):
        mujoco.mj_step(model, data)
    camera = config["external_camera"]
    option = mujoco.MjvOption()
    option.geomgroup[3] = 0
    with mujoco.Renderer(model, width=camera["width"], height=camera["height"]) as renderer:
        renderer.update_scene(data, camera="external_camera", scene_option=option)
        rgb = renderer.render().copy()
        renderer.enable_depth_rendering()
        depth = renderer.render().copy()
    cv2.imwrite("/tmp/lara5_external_camera.png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    return config, model, data, rgb, depth


def test_rendered_rgb_depth_localization(rendered_scene):
    config, model, data, rgb, depth = rendered_scene
    camera = config["external_camera"]
    k = intrinsics(camera["width"], camera["height"], camera["fovy"])
    # Main's calibrated mount is relative to a URDF optical frame. Compose
    # that parent pose instead of treating camera.position as world XYZ.
    local_position, local_rotation = camera_pose(camera)
    parent = model.body(camera.get("parent_frame", "world")).id
    parent_rotation = data.xmat[parent].reshape(3, 3)
    position = data.xpos[parent] + parent_rotation @ local_position
    rotation = parent_rotation @ local_rotation
    detections = detect_objects(rgb, depth, k, rotation, position, config["objects"])
    assert set(detections) == {obj["name"] for obj in config["objects"]}
    for name, detection in detections.items():
        # Ground truth is used only as an independent test oracle, never by detector.
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        assert np.linalg.norm(detection.center - data.xpos[body]) < 0.025, (name, detection.center, data.xpos[body])
    missing = detect_objects(rgb, np.full_like(depth, np.nan), k,
                             rotation, position, config["objects"])
    assert not missing
