"""The restore_task node end to end against the fake robot (needs ROS 2; skipped without rclpy)."""

import json
import math
import threading
import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.action import ActionClient  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from visualization_msgs.msg import MarkerArray  # noqa: E402

from fake_robot import FakeRobot, Thing  # noqa: E402
from restore_interfaces.action import Restore  # noqa: E402
from restore_perception.core.config import load_objects  # noqa: E402
from restore_task.core.config import load_config, objects_file  # noqa: E402
from restore_task.core.diff import symmetric  # noqa: E402
from restore_task.node import RestoreNode  # noqa: E402

TIDY = [Thing("wooden_cube", 0.15, -0.10, math.radians(30)), Thing("sponge", -0.15, -0.12, math.radians(-20)),
        Thing("tuna_can", 0.10, 0.15, 0.0)]
SYMMETRY = {"wooden_cube": 4, "sponge": 2, "tuna_can": 0}


class Harness:
    def __init__(self, tmp_path):
        rclpy.init()
        self.cfg, self.objects = load_config(), load_objects(objects_file())
        self.things = [Thing(t.label, t.x, t.y, t.yaw) for t in TIDY]
        self.robot = FakeRobot(self.things, self.objects, self.cfg)
        self.client_node = Node("restore_test_client")
        self.client = ActionClient(self.client_node, Restore, "/restore/run")
        self.plans = []
        self.client_node.create_subscription(
            MarkerArray, "/restore/plan", self.plans.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.executor = MultiThreadedExecutor(num_threads=8)
        self.executor.add_node(self.robot)
        self.executor.add_node(self.client_node)
        self.node = None
        self.start_node(tmp_path, "move_to_pose")
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def start_node(self, tmp_path, mode):
        if self.node is not None:
            self.executor.remove_node(self.node)
            self.node.destroy_node()
        self.node = RestoreNode(parameter_overrides=[Parameter("data_dir", value=str(tmp_path)),
                                                     Parameter("approach_mode", value=mode)])
        self.executor.add_node(self.node)
        assert self.client.wait_for_server(timeout_sec=10)

    def run(self, mode, dry_run=False, timeout=120):
        feedback = []
        future = self.client.send_goal_async(Restore.Goal(mode=mode, dry_run=dry_run),
                                             feedback_callback=lambda f: feedback.append(f.feedback.phase))
        handle = self._wait(future, 10)
        assert handle.accepted
        result = self._wait(handle.get_result_async(), timeout).result
        return result, feedback

    @staticmethod
    def _wait(future, timeout):
        end = time.monotonic() + timeout
        while not future.done():
            assert time.monotonic() < end, "timed out"
            time.sleep(0.01)
        return future.result()

    def close(self):
        self.executor.shutdown()
        rclpy.shutdown()


@pytest.fixture(scope="module")
def h(tmp_path_factory):
    harness = Harness(tmp_path_factory.mktemp("data"))
    yield harness
    harness.close()


def place(h, label, x, y, yaw_deg):
    thing = next(t for t in h.things if t.label == label)
    thing.x, thing.y, thing.yaw = x, y, math.radians(yaw_deg)


def assert_tidy(h):
    for thing, tidy in zip(h.things, TIDY):
        dxy = math.hypot(thing.x - tidy.x, thing.y - tidy.y)
        dyaw = math.degrees(symmetric(thing.yaw - tidy.yaw, SYMMETRY[thing.label]))
        assert dxy < 0.001 and abs(dyaw) < 0.5, (thing.label, dxy, dyaw)


def test_reference(h):
    result, feedback = h.run("reference")
    assert result.success, result.message
    assert "3 objects" in result.message and "init" in feedback


def test_dry_run_plans_without_moving(h):
    place(h, "wooden_cube", 0.30, 0.20, 75)
    place(h, "sponge", -0.05, 0.05, 40)
    moves = h.robot.moves
    result, _ = h.run("restore", dry_run=True)
    assert result.success, result.message
    plan = json.loads(result.report_json)["plan"]
    assert sorted(s["label"] for s in plan) == ["sponge", "wooden_cube"]
    assert h.robot.moves == moves
    time.sleep(0.3)
    arrows = [m for m in h.plans[-1].markers if m.ns == "steps"]
    assert len(arrows) == 2


def test_restore_with_a_refused_motion_and_a_missed_grasp(h):
    h.robot.reject_moves, h.robot.miss_grasps = 1, 1
    result, feedback = h.run("restore")
    assert result.success, result.message + "\nfake robot log:\n" + "\n".join(h.robot.log)
    report = json.loads(result.report_json)["objects"]
    outcomes = sorted((e["label"], e["outcome"], e["attempts"]) for e in report.values())
    assert [o[:2] for o in outcomes] == [("sponge", "restored"), ("tuna_can", "in place"), ("wooden_cube", "restored")]
    assert sum(o[2] for o in outcomes) == 3  # one object needed a second attempt after the missed grasp
    assert {"pick: approach", "pick: close", "place: release", "place: retreat"} <= set(feedback)
    assert h.robot.holding is None and h.robot.hand[2] == 0.0  # hand empty and open
    assert_tidy(h)


def test_unreachable_object_is_refused(h):
    place(h, "tuna_can", 0.62, 0.15, 0)  # beyond reach_area
    result, _ = h.run("restore")
    assert not result.success
    report = json.loads(result.report_json)["objects"]
    tuna = next(e for e in report.values() if e["label"] == "tuna_can")
    assert "reachable" in tuna["outcome"]
    place(h, "tuna_can", 0.10, 0.15, 0)


def test_restore_through_the_approach_node(h, tmp_path_factory):
    h.start_node(tmp_path_factory.mktemp("data2"), "approach_node")
    result, _ = h.run("reference")
    assert result.success, result.message
    place(h, "wooden_cube", -0.30, 0.20, 10)
    approaches = h.robot.approaches
    result, _ = h.run("restore")
    assert result.success, result.message
    assert h.robot.approaches - approaches == 2  # above the object, above its home
    assert_tidy(h)
