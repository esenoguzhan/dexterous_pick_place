"""Actual ROS topics and an isolated fake hand action; no arm or hardware calls."""

import json
import threading
import time
import uuid

import pytest

from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from std_msgs.msg import Empty, String

from lara5_grasp.configuration import HAND_JOINTS
from lara5_grasp.node import HandGrasp


def wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("Hand topic workflow timed out")


class Harness:
    def __init__(self, with_server=True, parameters=None):
        prefix = "/grasp_test_" + uuid.uuid4().hex
        self.context = Context()
        self.context.init()
        self.peer = Node("fake_hand_peer", context=self.context)
        self.grasp = HandGrasp(context=self.context, parameter_overrides=parameters or [], cli_args=[
            "--ros-args", "-r", "/hand_controller/follow_joint_trajectory:=" + prefix + "/hand",
            "-r", "/arm/grasp/target:=" + prefix + "/target",
            "-r", "/arm/grasp/status:=" + prefix + "/status"])
        self.status_topic = prefix + "/status"
        self.events, self.goals, self.pending = [], [], []
        self.accept = True
        self.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.abort = False
        self.release = threading.Event()
        self.tick = self.peer.create_timer(0.01, self.complete_pending)
        self.status = self.peer.create_subscription(
            String, self.status_topic, lambda msg: self.events.append(json.loads(msg.data)),
            QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.publisher = self.peer.create_publisher(Empty, prefix + "/target", 1)
        self.server = None
        if with_server:
            self.server = ActionServer(
                self.peer, FollowJointTrajectory, prefix + "/hand", execute_callback=self.execute,
                goal_callback=lambda _: GoalResponse.ACCEPT if self.accept else GoalResponse.REJECT,
                cancel_callback=lambda _: CancelResponse.ACCEPT,
                callback_group=ReentrantCallbackGroup())
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.peer)
        self.executor.add_node(self.grasp)
        self.thread = threading.Thread(target=self.executor.spin)
        self.thread.start()
        try:
            wait_for(lambda: self.publisher.get_subscription_count() == 1)
            if with_server:
                wait_for(self.grasp.hand.server_is_ready)
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
        elif self.abort:
            handle.abort()
        else:
            handle.succeed()
        return FollowJointTrajectory.Result(error_code=self.error_code, error_string="Fake hand result")

    def complete_pending(self):
        for handle, ready in list(self.pending):
            if self.release.is_set() or handle.is_cancel_requested:
                self.pending.remove((handle, ready))
                ready.set_result(None)

    def event(self, state):
        wait_for(lambda: any(event["state"] == state for event in self.events))
        return next(event for event in reversed(self.events) if event["state"] == state)

    def send(self):
        self.publisher.publish(Empty())

    def close(self):
        self.release.set()
        self.executor.shutdown(timeout_sec=5)
        self.thread.join(timeout=5)
        if self.server is not None:
            self.server.destroy()
        self.grasp.destroy_node()
        self.peer.destroy_node()
        self.context.shutdown()


@pytest.fixture
def harness():
    instance = Harness()
    try:
        yield instance
    finally:
        instance.close()


def test_empty_command_sends_only_hand_joints_and_reports_motion_completion(harness):
    harness.release.set()
    harness.send()
    done = harness.event("succeeded")
    assert done["active_request_id"] is None
    assert done["request_id"] == harness.event("requested")["request_id"]
    assert "retention not verified" in done["message"]
    assert len(harness.goals) == 1
    path = harness.goals[0].trajectory
    assert list(path.joint_names) == list(HAND_JOINTS)
    assert not any(name.startswith("lara5_joint") for name in path.joint_names)
    assert len(path.points) == 1
    assert list(path.points[0].positions) == [1.1, 1.3, 1.3, 1.3, 1.3, 0.5]
    assert path.points[0].time_from_start.sec == 3


def test_busy_commands_are_discarded_and_a_later_command_can_run(harness):
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
    wait_for(lambda: sum(event["state"] == "succeeded" for event in harness.events) == 2)
    assert len(harness.goals) == 2


def test_controller_rejection_releases_task(harness):
    harness.accept = False
    harness.send()
    harness.event("failed")
    assert not harness.goals
    assert harness.grasp.active_request is None


@pytest.mark.parametrize("abort,error_code", [(True, 0), (False, -5)])
def test_controller_errors_are_not_reported_as_success(harness, abort, error_code):
    harness.abort, harness.error_code = abort, error_code
    harness.release.set()
    harness.send()
    harness.event("failed")
    assert not any(event["state"] == "succeeded" for event in harness.events)
    assert harness.grasp.active_request is None


def test_shutdown_cancels_closure(harness):
    harness.send()
    harness.event("executing")
    harness.grasp.cancel_active()
    harness.event("canceled")
    assert harness.grasp.active_request is None


def test_result_is_retained_for_a_late_status_subscriber(harness):
    harness.release.set()
    harness.send()
    harness.event("succeeded")
    late = []
    subscription = harness.peer.create_subscription(
        String, harness.status_topic, lambda msg: late.append(json.loads(msg.data)),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    wait_for(lambda: bool(late))
    assert late[-1]["state"] == "succeeded"
    harness.peer.destroy_subscription(subscription)


def test_missing_controller_discards_command():
    harness = Harness(with_server=False)
    try:
        harness.send()
        harness.event("unavailable")
        assert harness.grasp.active_request is None
        assert not harness.goals
    finally:
        harness.close()


def test_custom_angles_and_fractional_duration_reach_controller():
    positions = [0.7, 0.9, 0.9, 0.9, 0.9, 0.35]
    harness = Harness(parameters=[Parameter("close_positions", value=positions),
                                  Parameter("duration", value=2.75)])
    try:
        harness.release.set()
        harness.send()
        harness.event("succeeded")
        point = harness.goals[0].trajectory.points[0]
        assert list(point.positions) == positions
        assert point.time_from_start.sec == 2
        assert point.time_from_start.nanosec == 750_000_000
    finally:
        harness.close()
