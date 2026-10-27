"""Static model/ROS-message checks: no physics steps, renderer or ROS nodes."""

import copy
from pathlib import Path
import subprocess
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

from lara5_mujoco_sim.camera import camera_axes, camera_pose, intrinsics, optical_rotation
from lara5_mujoco_sim.camera_publisher import RgbdCameraPublisher
from lara5_mujoco_sim.mjcf_builder import _add_cameras, build_scene


CONFIG = Path(__file__).resolve().parents[1] / "config" / "mujoco_sim.yaml"


class WristCameraTests(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load(CONFIG.read_text())

    def model(self, config=None):
        root = ET.fromstring('''<mujoco><worldbody>
          <body name="camera_1_color_optical_frame" pos="-0.9 0.4 1.815"/>
          <body name="right_hand_base_link" pos="0.2 -0.1 1">
            <joint name="wrist" type="hinge" axis="0 0 1"/>
            <geom type="sphere" size="0.01" mass="1"/>
            <body name="right_wrist_camera_holder" pos="0 0 -0.0115"/>
          </body></worldbody></mujoco>''')
        _add_cameras(root, self.config if config is None else config)
        return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))

    def test_optical_roll_and_invalid_up(self):
        c = dict(position=[0, .08, -.08], target=[-.025, .085, .15], up=[0, 1, 0])
        axes = camera_axes(c["position"], c["target"], c["up"])
        optical = optical_rotation(c["position"], c["target"], c["up"])
        forward = np.array(c["target"]) - c["position"]
        np.testing.assert_allclose(optical[:, 2], forward / np.linalg.norm(forward))
        np.testing.assert_allclose(optical.T @ optical, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(optical), 1.0)
        np.testing.assert_allclose(optical[:, 1], -axes[:, 1])
        for up in ([0, 0, 0], forward, [float("nan"), 0, 1]):
            with self.assertRaises(ValueError):
                camera_axes(c["position"], c["target"], up)

    def test_wrist_follows_parent_and_external_stays_fixed(self):
        model = self.model()
        data = mujoco.MjData(model)
        wrist = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "wrist_camera")
        external = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "external_camera")
        parent = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, self.config["wrist_camera"]["parent_frame"])
        self.assertEqual(model.cam_bodyid[wrist], parent)
        fixed_parent = model.body(self.config["external_camera"]["parent_frame"]).id
        self.assertEqual(model.cam_bodyid[external], fixed_parent)
        poses = []
        for angle in (0.0, 0.7):
            data.qpos[0] = angle
            mujoco.mj_forward(model, data)  # Kinematics only; no time integration.
            parent_rotation = data.xmat[parent].reshape(3, 3)
            c = self.config["wrist_camera"]
            np.testing.assert_allclose(data.cam_xpos[wrist],
                                       data.xpos[parent] + parent_rotation @ c["position"])
            world_optical = (data.cam_xmat[wrist].reshape(3, 3)
                             @ np.diag([1., -1., -1.]))
            np.testing.assert_allclose(world_optical, parent_rotation @ camera_pose(c)[1], atol=1e-12)
            poses.append(data.cam_xpos.copy())
        self.assertGreater(np.linalg.norm(poses[1][wrist] - poses[0][wrist]), 0.01)
        np.testing.assert_allclose(poses[0][external], poses[1][external])

    def test_missing_parent_is_rejected_and_camera_can_be_disabled(self):
        config = copy.deepcopy(self.config)
        config["wrist_camera"]["parent_frame"] = "missing_link"
        with self.assertRaisesRegex(ValueError, "missing_link"):
            self.model(config)
        config["wrist_camera"]["enabled"] = False
        self.assertEqual(self.model(config).ncam, 1)

    def test_publisher_topics_and_mount_tf_match_model(self):
        # Capture real ROS messages without creating a Node or DDS context.
        from builtin_interfaces.msg import Time

        class Sink:
            def __init__(self):
                self.transforms = []

            def sendTransform(self, transform):
                self.transforms.append(transform)

        class FakeNode:
            config = self.config

            def __init__(self):
                self.topics = []

            def create_publisher(self, message_type, topic, depth):
                self.topics.append(topic)

            def get_clock(self):
                return self

            def now(self):
                return self

            def to_msg(self):
                return Time(sec=123)

            def get_logger(self):
                return self

            def info(self, message):
                pass

        node, sink = FakeNode(), Sink()
        for name in ("external_camera", "wrist_camera"):
            publisher = RgbdCameraPublisher(node, name, sink)
            transform = sink.transforms[-1]
            c = self.config[name]
            self.assertEqual(transform.header.frame_id, c["parent_frame"])
            self.assertEqual(transform.child_frame_id, c["frame_id"])
            rotation = transform.transform.rotation
            matrix = np.empty(9)
            mujoco.mju_quat2Mat(matrix, [rotation.w, rotation.x, rotation.y, rotation.z])
            np.testing.assert_allclose(matrix.reshape(3, 3), camera_pose(c)[1], atol=1e-12)
            for suffix in ("color/image_raw", "depth/image_raw", "camera_info"):
                self.assertIn(f"/sim/{name}/{suffix}", node.topics)
            msg = publisher._image(np.zeros((2, 3, 3), dtype=np.uint8), "rgb8", Time(sec=123))
            self.assertEqual(msg.header.frame_id, c["frame_id"])
            self.assertEqual(msg.header.stamp.sec, 123)
            self.assertEqual(msg.step, 9)

    def test_actual_robot_accepts_mount(self):
        robot = Path(get_package_share_directory("lara5_controller_manager"))
        urdf = subprocess.check_output([
            "xacro", str(robot / "urdf/lara5_inspire_hand_ros2_control.urdf.xacro"),
            "deployment_mode:=simulation"], text=True)
        scene = build_scene(urdf, self.config)
        model = mujoco.MjModel.from_xml_string(scene.mjcf)
        camera = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "wrist_camera")
        self.assertGreaterEqual(camera, 0)
        parent = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, self.config["wrist_camera"]["parent_frame"])
        self.assertEqual(model.cam_bodyid[camera], parent)
        case = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "wrist_camera_case")
        if self.config["wrist_camera"].get("show_body", False):
            self.assertGreaterEqual(case, 0)
            self.assertEqual(model.geom_contype[case], 0)
            self.assertEqual(model.geom_conaffinity[case], 0)
        else:
            self.assertEqual(case, -1)
        # The D435 body comes from the URDF (camera_2_link, d435_camera.xacro), visual only.
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "camera_2_link")
        self.assertGreaterEqual(body, 0)
        self.assertGreaterEqual(model.body_geomnum[body], 1)
        for geom in range(model.body_geomadr[body], model.body_geomadr[body] + model.body_geomnum[body]):
            self.assertEqual(model.geom_contype[geom], 0)
            self.assertEqual(model.geom_conaffinity[geom], 0)

    def test_cad_mount_frames_grasp_region_but_exposes_short_depth_distance(self):
        config = self.config["wrist_camera"]
        position, rotation = camera_pose(config)
        target = np.asarray(config["target"])
        optical_target = rotation.T @ (target - position)
        self.assertGreater(optical_target[2], 0)
        self.assertLess(optical_target[2], config["depth_min_m"])
        k = intrinsics(config["width"], config["height"], config["fovy"])
        pixel = k @ optical_target
        pixel = pixel[:2] / pixel[2]
        # Respect CAD orientation: do not re-aim the sensor to center a QA marker.
        self.assertTrue(0 <= pixel[0] < config["width"] and 0 <= pixel[1] < config["height"])
        calibrated = copy.deepcopy(self.config)
        calibrated["wrist_camera"]["orientation_xyzw"] = [0., 0., 0., 1.]
        model = self.model(calibrated)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        camera = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "wrist_camera")
        world_optical = data.cam_xmat[camera].reshape(3, 3) @ np.diag([1., -1., -1.])
        np.testing.assert_allclose(world_optical, np.eye(3), atol=1e-12)


if __name__ == "__main__":
    unittest.main()
