"""Opt-in static native IK integration: no MuJoCo, controllers or motion commands.

LARA5_NATIVE_GRASP_TEST=1 python3 <this file>
Synthetic settled object poses are test oracles, never runtime perception inputs.
Run in an isolated container/domain: this starts only the native safety server.
"""

import math
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import yaml

from lara5_sim_approach.approach import ARM
from lara5_sim_approach.grasping import HAND, hand_offset_in_flange
from lara5_sim_approach.pick_place import PickPlace
from lara5_sim_approach.routing import RouteRejected
from lara5_sim_approach.vision import Detection, dimensions


@unittest.skipUnless(os.environ.get("LARA5_NATIVE_GRASP_TEST") == "1", "Opt-in native IK only")
class NativeGraspPreviewTest(unittest.TestCase):
    def test_full_cycle_has_a_native_ik_preview(self):
        robot = Path(get_package_share_directory("lara5_controller_manager"))
        safety = Path(get_package_share_directory("lara5_safety_filter"))
        urdf = subprocess.check_output(["xacro", str(robot/"urdf/lara5_inspire_hand_ros2_control.urdf.xacro"),
                                        "deployment_mode:=simulation"], text=True)
        params = yaml.safe_load((safety/"config/safety_filter.yaml").read_text())["safety_filter"]["ros__parameters"]
        params.update(robot_description=urdf, allow_scene_updates=True, allow_grasp_updates=True,
                      obstacles_file=str(safety/"config/obstacles.yaml"))
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory)/"params.yaml"
            config.write_text(yaml.safe_dump({"safety_filter": {"ros__parameters": params}}))
            log = open(Path(directory)/"server.log", "w+")
            server = subprocess.Popen(["ros2", "run", "lara5_safety_filter", "safety_filter_node",
                                       "--ros-args", "--params-file", str(config)], stdout=log, stderr=log,
                                      start_new_session=True)
            node = None
            rclpy.init()
            try:
                node = PickPlace()
                source_config = Path(__file__).resolve().parents[2]/"lara5_mujoco_sim/config/mujoco_sim.yaml"
                node.config = yaml.safe_load(source_config.read_text())
                node.on_description(String(data=urdf))
                node.joints = JointState(name=ARM+HAND,
                    position=[0., 0., math.pi/2, 0., math.pi/2, 0.]+[0.]*6)
                node.touch_links = [link.get("name") for link in ET.fromstring(urdf).findall("link")
                                    if link.get("name", "").startswith("right_")]
                node.open_hand = np.zeros(6)
                node.closed_hand = np.asarray(node.p("close_positions"))
                node.object_config = next(obj for obj in node.config["objects"] if obj["name"] == node.target)
                node.extent = dimensions(node.object_config)
                node.offset = hand_offset_in_flange(node.p("grasp_offset_hand"))
                node.destination = np.asarray(node.p("place_position"), dtype=float)
                if node.destination[2] < 0:
                    node.destination[2] = .90+node.extent[2]/2
                node.baseline = {}
                for obj in node.config["objects"]:
                    extent = dimensions(obj)
                    center = np.array([*obj["pos"][:2], .90+extent[2]/2])
                    node.baseline[obj["name"]] = Detection(obj["name"], center, extent+.03,
                                                           center+[0, 0, extent[2]/2], 100)
                failures = []
                for yaw in (0., math.pi/2, math.pi, -math.pi/2):
                    try:
                        preview = node.preview(node.baseline[node.target].center, yaw)
                        self.assertEqual([name for name, _ in preview["carry"]],
                                         ["test_lift", "lift", "transfer", "lower_for_release"])
                        state = node.endpoint_state(preview["approach"])
                        payload = node.nominal_payload(preview["rotation"])
                        node.configure_scene(node.closed_hand, node.baseline, attached=payload)
                        with self.assertRaises(RouteRejected):
                            node.validate(node.stationary(state))  # Resting payload needs its scoped exception.
                        node.configure_scene(node.closed_hand, node.baseline, attached=payload, support_contact=True)
                        node.validate(node.stationary(state))
                        lifted = node.endpoint_state(preview["carry"][0][1])
                        node.configure_scene(node.closed_hand, node.baseline, attached=payload)
                        node.validate(node.stationary(lifted))  # Full margin restored after lift.
                        print(f"Native grasp preview accepted yaw={yaw}, route={preview['route']}")
                        break
                    except RouteRejected as error:
                        failures.append(str(error))
                else:
                    self.fail("No full native preview: " + " | ".join(failures))
            except BaseException:
                log.flush(); log.seek(0)
                print(log.read()[-6000:])
                raise
            finally:
                if node is not None:
                    node.destroy_node()
                rclpy.shutdown()
                import signal
                os.killpg(server.pid, signal.SIGINT)
                try:
                    server.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(server.pid, signal.SIGKILL)
                    server.wait(timeout=5)
                log.close()


if __name__ == "__main__":
    unittest.main()
