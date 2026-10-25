"""Real ROS transport with an isolated fake action backend; no robot commands."""

import json
import math
import threading
import time
import uuid

import pytest

pytest.importorskip("rclpy")
from geometry_msgs.msg import PoseStamped
from lara5_safety_msgs.action import MoveToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from std_msgs.msg import String

from lara5_object_approach.node import ObjectApproach


def wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("ROS workflow timed out")


class Harness:
    def __init__(self, with_server=True):
        # Unique names prevent these tests from ever reaching a real arm backend.
        prefix = "/approach_test_" + uuid.uuid4().hex
        self.context = Context()
        self.context.init()
        self.peer = Node("test_peer", context=self.context)
        self.approach = ObjectApproach(context=self.context, cli_args=[
            "--ros-args", "-r", "/safety/arm/move_to_pose:=" + prefix + "/motion",
            "-r", "/arm/approach/target:=" + prefix + "/target",
            "-r", "/arm/approach/status:=" + prefix + "/status"])
        self.events = []
        self.goals = []
        self.accept = True
        self.success = True
        self.release = threading.Event()
        self.pending = []
        self.tick = self.peer.create_timer(0.01, self.complete_pending)
        qos = QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.peer.create_subscription(
            String, prefix + "/status", lambda msg: self.events.append(json.loads(msg.data)), qos)
        self.publisher = self.peer.create_publisher(PoseStamped, prefix + "/target", 1)
        self.server = None
        if with_server:
            self.server = ActionServer(
                self.peer, MoveToPose, prefix + "/motion", execute_callback=self.execute,
                goal_callback=lambda _: GoalResponse.ACCEPT if self.accept else GoalResponse.REJECT,
                cancel_callback=lambda _: CancelResponse.ACCEPT,
                callback_group=ReentrantCallbackGroup())
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.peer)
        self.executor.add_node(self.approach)
        self.thread = threading.Thread(target=self.executor.spin)
        self.thread.start()
        try:
            wait_for(lambda: self.publisher.get_subscription_count() == 1)
            if with_server:
                wait_for(self.approach.motion.server_is_ready)
        except BaseException:
            self.close()
            raise

    async def execute(self, handle):
        self.goals.append(handle.request)
        ready = Future()
        self.pending.append((handle, ready))
        await ready
        if handle.is_cancel_requested:
            handle.canceled()
            return MoveToPose.Result(success=False, message="Canceled by test")
        if self.success:
            handle.succeed()
        else:
            handle.abort()
        return MoveToPose.Result(success=self.success, message="Fake backend result")

    def complete_pending(self):
        for handle, ready in list(self.pending):
            if self.release.is_set() or handle.is_cancel_requested:
                self.pending.remove((handle, ready))
                ready.set_result(None)

    def send(self, frame="world", valid=True):
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = 0.2, -0.1, 0.95
        pose.pose.orientation.z = math.sqrt(0.5) if valid else 0.0
        pose.pose.orientation.w = math.sqrt(0.5) if valid else 0.0
        self.publisher.publish(pose)

    def event(self, state):
        wait_for(lambda: any(event["state"] == state for event in self.events))
        return next(event for event in reversed(self.events) if event["state"] == state)

    def close(self):
        self.release.set()
        self.executor.shutdown(timeout_sec=5)
        self.thread.join(timeout=5)
        if self.server is not None:
            self.server.destroy()
        self.approach.destroy_node()
        self.peer.destroy_node()
        self.context.shutdown()


@pytest.fixture
def harness():
    instance = Harness()
    try:
        yield instance
    finally:
        instance.close()


def test_pose_topic_reaches_absolute_native_goal_and_completes(harness):
    harness.release.set()
    harness.send()
    done = harness.event("succeeded")
    assert done["active_request_id"] is None
    assert done["request_id"] == harness.event("planning")["request_id"]
    goal = harness.goals[0]
    assert not goal.relative
    assert goal.duration == 0.0
    assert goal.target.header.frame_id == "world"
    assert goal.target.pose.position.x == pytest.approx(0.2)
    assert goal.target.pose.position.y == pytest.approx(-0.1)
    assert goal.target.pose.position.z == pytest.approx(1.15)
    assert goal.target.pose.orientation.x == pytest.approx(-math.sqrt(0.5))
    assert goal.target.pose.orientation.y == pytest.approx(0.0, abs=1e-12)
    assert goal.target.pose.orientation.z == pytest.approx(math.sqrt(0.5))
    assert goal.target.pose.orientation.w == pytest.approx(0.0, abs=1e-12)


def test_busy_command_is_discarded_and_next_command_can_run(harness):
    harness.send()
    active = harness.event("executing")["request_id"]
    harness.send()
    rejected = harness.event("rejected_busy")
    assert rejected["active_request_id"] == active
    assert rejected["request_id"] != active
    assert len(harness.goals) == 1
    harness.release.set()
    harness.event("succeeded")
    harness.send()
    wait_for(lambda: sum(e["state"] == "succeeded" for e in harness.events) == 2)
    assert len(harness.goals) == 2


@pytest.mark.parametrize("frame,valid", [("camera_optical", True), ("", True), ("world", False)])
def test_invalid_pose_never_dispatches(harness, frame, valid):
    harness.send(frame, valid)
    harness.event("rejected_invalid")
    assert not harness.goals
    assert harness.approach.active_request is None


def test_native_rejection_releases_task(harness):
    harness.accept = False
    harness.send()
    harness.event("failed")
    assert harness.approach.active_request is None
    assert not harness.goals


def test_native_execution_failure_is_not_reported_as_success(harness):
    harness.success = False
    harness.release.set()
    harness.send()
    harness.event("failed")
    assert not any(e["state"] == "succeeded" for e in harness.events)
    assert harness.approach.active_request is None


def test_shutdown_cancels_active_motion(harness):
    harness.send()
    harness.event("executing")
    harness.approach.cancel_active()
    harness.event("canceled")
    assert harness.approach.active_request is None


def test_missing_backend_does_not_queue_command():
    harness = Harness(with_server=False)
    try:
        harness.send()
        harness.event("unavailable")
        assert harness.approach.active_request is None
    finally:
        harness.close()
