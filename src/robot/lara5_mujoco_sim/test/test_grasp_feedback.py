"""Verify contact-feedback frames/serialization using static FK only."""

from pathlib import Path
import subprocess
import threading
import unittest

import mujoco
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time

from lara5_mujoco_sim.grasp_feedback import GraspFeedbackPublisher
from lara5_mujoco_sim.mjcf_builder import build_scene, apply_initial_state


class GraspFeedbackTests(unittest.TestCase):
    def test_object_frames_match_fk_without_physics_or_attachment(self):
        config = yaml.safe_load((Path(__file__).resolve().parents[1]/"config/mujoco_sim.yaml").read_text())
        robot = Path(get_package_share_directory("lara5_controller_manager"))
        urdf = subprocess.check_output(["xacro", str(robot/"urdf/lara5_inspire_hand_ros2_control.urdf.xacro"),
                                        "deployment_mode:=simulation"], text=True)
        scene = build_scene(urdf, config)

        class FakeNode:
            def __init__(self):
                self.config = config
                self.model = mujoco.MjModel.from_xml_string(scene.mjcf)
                self.data = mujoco.MjData(self.model)
                apply_initial_state(self.model, self.data, scene)
                self.lock = threading.Lock()
                self.messages = []

            def create_publisher(self, message_type, topic, depth):
                self.topic = topic
                return self

            def get_clock(self):
                return self

            def now(self):
                return self

            def to_msg(self):
                return Time(sec=123)

            def publish(self, message):
                self.messages.append(message)

        node = FakeNode()
        publisher = GraspFeedbackPublisher(node)
        qpos = node.data.qpos.copy()
        publisher.publish()
        self.assertEqual(node.topic, "/sim/grasp_feedback")
        self.assertEqual({msg.object_name for msg in node.messages}, {obj["name"] for obj in config["objects"]})
        flange_r = node.data.xmat[publisher.flange].reshape(3, 3)
        for message in node.messages:
            self.assertEqual(message.header.frame_id, "world")
            self.assertEqual(message.header.stamp.sec, 123)
            relative = message.object_in_flange.position
            world = message.object_pose.position
            np.testing.assert_allclose(node.data.xpos[publisher.flange]+flange_r@[relative.x, relative.y, relative.z],
                                       [world.x, world.y, world.z], atol=1e-12)
            self.assertFalse(message.opposing_contacts)
            self.assertEqual(len(message.contact_links), len(message.contact_normal_forces))
            self.assertTrue(all(force >= 0 for force in message.contact_normal_forces))
        np.testing.assert_array_equal(qpos, node.data.qpos)  # Feedback never moves an object.

    def test_reports_real_normal_force_on_a_supporting_finger(self):
        from types import SimpleNamespace
        model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
          <body name="lara5_flange"/>
          <body name="right_index_distal_link"><geom type="box" size=".1 .1 .01"/></body>
          <body name="target" pos="0 0 .06"><freejoint/>
            <geom type="sphere" size=".03" mass=".1"/></body>
        </worldbody></mujoco>''')
        data = mujoco.MjData(model)
        for _ in range(500):
            mujoco.mj_step(model, data)
        messages = []
        clock = SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=123)))
        node = SimpleNamespace(model=model, data=data, config={'objects': [{'name': 'target'}]},
            lock=threading.Lock(), create_publisher=lambda *args: SimpleNamespace(publish=messages.append),
            get_clock=lambda: clock)
        GraspFeedbackPublisher(node).publish()
        self.assertEqual(messages[0].contact_links, ['right_index_distal_link'])
        self.assertAlmostEqual(sum(messages[0].contact_normal_forces), .981, delta=.03)


if __name__ == "__main__":
    unittest.main()
