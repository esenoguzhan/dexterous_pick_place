"""Real ROS transport with fake read-only services; no controllers or actions."""

import copy
import json
import threading
import time
import uuid

import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped
from lara5_safety_msgs.srv import CheckTrajectory, GetFramePose, PlanCartesianPath
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from lara5_object_approach.grasp_geometry import from_rpy
from lara5_object_approach.planning_geometry import duration
from lara5_object_approach.preview import GraspPreview, ros_pose


def wait_for(predicate, timeout=5):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if predicate(): return
        time.sleep(.01)
    raise AssertionError("planning ROS test timed out")


class ToyModel:
    """A transparent FK oracle for testing the ROS protocol, not robot IK."""
    names = [f"lara5_joint{i}" for i in range(1, 7)]
    velocity = np.ones(6)
    def check(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all() or np.max(np.abs(q)) > 5:
            raise ValueError("invalid toy arm state")
        return q
    def fk(self, q):
        return from_rpy(self.check(q)[:3], [-np.pi/2, np.pi/2, 0])


class Harness:
    def __init__(self, services=True, node_class=GraspPreview, parameters=None):
        self.prefix = "/plan_test_"+uuid.uuid4().hex
        self.context = Context(); self.context.init()
        self.peer = Node("planner_test_peer", context=self.context)
        remaps = []
        for suffix in ("target", "status", "markers", "flange_pose", "grasp_pose", "plan_status",
                       "pregrasp_trajectory", "final_trajectory", "pregrasp_pose", "path_markers"):
            remaps.extend(["-r", f"/arm/approach/preview/{suffix}:={self.prefix}/{suffix}"])
        remaps.extend(["-r", "/joint_states:="+self.prefix+"/state"])
        for suffix in ("get_frame_pose", "plan_cartesian_path", "check_trajectory"):
            remaps.extend(["-r", f"/safety/arm/{suffix}:={self.prefix}/{suffix}"])
        for source, suffix in (("/arm/approach/target", "target"), ("/arm/approach/status", "plan_status"),
                               ("/arm_trajectory_controller/follow_joint_trajectory", "motion")):
            remaps.extend(["-r", source+":="+self.prefix+"/"+suffix])
        self.preview = node_class(context=self.context, cli_args=["--ros-args", *remaps],
            parameter_overrides=[Parameter("plan_trajectories", value=True),
                                 Parameter("planning_service_timeout", value=.5), *(parameters or [])])
        self.preview.planner.model = ToyModel()
        self.q = [.1, -.1, 1.3, 0., 0., 0.]
        self.hand = [0.]*6
        self.send_state = True
        self.stamp_offset = 0.
        self.hold = False
        self.pending = []
        self.plan_requests, self.check_requests, self.events = [], [], []
        self.fault = None
        self.paths = [[], []]
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions = [self.peer.create_subscription(String, self.prefix+"/plan_status",
            lambda s: self.events.append(json.loads(s.data)), qos)]
        for index, suffix in enumerate(("pregrasp_trajectory", "final_trajectory")):
            self.subscriptions.append(self.peer.create_subscription(JointTrajectory, self.prefix+"/"+suffix,
                lambda p, index=index: self.paths[index].append(p), qos))
        self.publisher = self.peer.create_publisher(PoseStamped, self.prefix+"/target", 1)
        self.state_publisher = self.peer.create_publisher(JointState, self.prefix+"/state", 10)
        self.services = []
        group = ReentrantCallbackGroup()
        if services:
            for suffix, service, callback in (("get_frame_pose", GetFramePose, self.fk),
                ("plan_cartesian_path", PlanCartesianPath, self.plan),
                ("check_trajectory", CheckTrajectory, self.check)):
                self.services.append(self.peer.create_service(service, self.prefix+"/"+suffix, callback,
                                                              callback_group=group))
        self.timer = self.peer.create_timer(.02, self.tick, callback_group=group)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.peer); self.executor.add_node(self.preview)
        self.thread = threading.Thread(target=self.executor.spin); self.thread.start()
        try:
            wait_for(lambda: self.publisher.get_subscription_count() == 1)
            wait_for(lambda: len(self.preview.planner.samples) == 12)
            if services:
                wait_for(lambda: all(c.service_is_ready() for c in (self.preview.planner.plan_client,
                    self.preview.planner.fk_client, self.preview.planner.check_client)))
        except BaseException:
            self.close(); raise

    def tick(self):
        if self.send_state:
            state = JointState(name=ToyModel.names+self.preview.planner.open_names, position=self.q+self.hand)
            now = self.peer.get_clock().now().nanoseconds-round(self.stamp_offset*1e9)
            state.header.stamp.sec, state.header.stamp.nanosec = now//1_000_000_000, now % 1_000_000_000
            self.state_publisher.publish(state)
        if not self.hold:
            for future in self.pending:
                if not future.done(): future.set_result(None)
            self.pending.clear()

    def fk(self, request, response):
        response.success = True
        response.pose.header.frame_id = "world"
        response.pose.pose = ros_pose(ToyModel().fk(request.start_state.position))
        if self.fault == "fk" and self.plan_requests:
            response.pose.pose.position.x += .01
        return response

    async def plan(self, request, response):
        self.plan_requests.append(copy.deepcopy(request))
        if self.hold:
            future = Future(); self.pending.append(future); await future
        if self.fault == "unreachable" or (self.fault == "second_phase" and len(self.plan_requests) == 2):
            response.success, response.message = False, "unreachable native IK target"
            return response
        response.success = True
        response.trajectory.joint_names = list(reversed(ToyModel.names))
        if self.fault == "empty":
            return response
        start = np.array(request.start_state.position)
        p = request.target.pose.position
        end = start.copy(); end[:3] = [p.x, p.y, p.z]
        if self.fault == "incomplete": end[2] += .05
        for index, u in enumerate(np.linspace(.1, 1, 10)):
            response.trajectory.points.append(JointTrajectoryPoint(
                positions=(start+(end-start)*u)[::-1].tolist(), time_from_start=duration((index+1)*.1)))
        if self.fault == "jump": response.trajectory.points[0].positions[0] += .4
        return response

    def check(self, request, response):
        self.check_requests.append(copy.deepcopy(request))
        response.valid = self.fault != "collision"
        response.message = "collision" if not response.valid else "clear"
        response.clearance = float("inf") if self.fault == "unbounded_clearance" else .05
        return response

    def send(self, frame="world"):
        message = PoseStamped()
        message.header.frame_id = frame
        message.pose.position.x, message.pose.position.y, message.pose.position.z = .25, -.2, .95
        message.pose.orientation.w = 1.
        self.publisher.publish(message)

    def event(self, state):
        wait_for(lambda: any(e["state"] == state for e in self.events))
        return next(e for e in reversed(self.events) if e["state"] == state)

    def close(self):
        self.hold = False
        self.executor.shutdown(timeout_sec=5); self.thread.join(timeout=5)
        for service in self.services: self.peer.destroy_service(service)
        self.preview.destroy_node(); self.peer.destroy_node(); self.context.shutdown()


@pytest.fixture
def harness():
    h = Harness()
    try: yield h
    finally: h.close()


def test_two_phases_are_connected_validated_and_published_only_for_preview(harness):
    h = harness; h.send(); ready = h.event("plan_ready")
    assert ready["executable"] is False
    assert ready["pregrasp_xyz"] == pytest.approx([.275, -.35, 1.25])
    assert ready["final_xyz"] == pytest.approx([.275, -.35, 1.035])
    assert len(h.plan_requests) == len(h.check_requests) == 2
    assert h.plan_requests[0].start_state.position == pytest.approx(h.q)
    wait_for(lambda: all(any(p.points for p in ps) for ps in h.paths))
    a, b = h.paths[0][-1], h.paths[1][-1]
    assert a.points[-1].positions == b.points[0].positions
    assert h.plan_requests[1].start_state.position == a.points[-1].positions
    assert a.header.stamp == b.header.stamp
    assert a.joint_names == b.joint_names == ToyModel.names
    assert h.check_requests[1].start_state.position == a.points[-1].positions
    clients = h.peer.get_client_names_and_types_by_node(h.preview.get_name(), h.preview.get_namespace())
    assert {name for name, _ in clients} == {h.prefix+"/"+s for s in
        ("get_frame_pose", "plan_cartesian_path", "check_trajectory")}


@pytest.mark.parametrize("fault", ["unreachable", "incomplete", "jump", "collision", "fk", "second_phase", "empty"])
def test_invalid_native_result_never_publishes_partial_plan(harness, fault):
    h = harness; h.fault = fault; h.send(); h.event("plan_failed")
    assert h.preview.planner.valid is None
    assert not any(p.points for phase in h.paths for p in phase)


def test_closed_hand_and_stale_timestamp_rejected_before_planning(harness):
    h = harness; h.hand = [.2]*6
    wait_for(lambda: h.preview.planner.samples[h.preview.planner.open_names[0]][0] == .2)
    h.send(); assert "open posture" in h.event("rejected_state")["message"]
    assert not h.plan_requests
    h.hand = [0.]*6; h.stamp_offset = 5.
    wait_for(lambda: h.peer.get_clock().now().nanoseconds/1e9-h.preview.planner.samples[ToyModel.names[0]][2] > 4)
    h.send(); wait_for(lambda: sum(e["state"] == "rejected_state" for e in h.events) >= 2)
    assert "timestamp" in h.events[-1]["message"]
    assert not h.plan_requests


def test_busy_request_discarded_and_timeout_clears_outputs(harness):
    h = harness; h.hold = True; h.send()
    wait_for(lambda: len(h.plan_requests) == 1)
    h.send(); h.event("rejected_busy")
    assert len(h.plan_requests) == 1
    assert "timed out" in h.event("plan_failed")["message"]
    assert not any(p.points for phase in h.paths for p in phase)
    # A late response from that abandoned request cannot contaminate a new plan.
    h.hold = False
    wait_for(lambda: not h.pending)
    h.send(); h.event("plan_ready")


def test_arm_motion_while_planning_abandons_request(harness):
    h = harness; h.hold = True; h.send()
    wait_for(lambda: len(h.plan_requests) == 1)
    h.q[0] += .05
    assert "changed" in h.event("plan_failed")["message"]
    assert not any(p.points for phase in h.paths for p in phase)


def test_missing_joint_feedback_rejected_before_dispatch(harness):
    h = harness; h.send_state = False
    h.preview.planner.samples.pop(ToyModel.names[0], None)
    h.send(); assert "missing" in h.event("rejected_state")["message"]
    assert not h.plan_requests


def test_robot_motion_invalidates_previously_ready_plan(harness):
    h = harness; h.send(); h.event("plan_ready")
    h.q[0] += .05
    h.event("plan_invalidated")
    wait_for(lambda: all(ps and not ps[-1].points for ps in h.paths))
    assert h.preview.planner.valid is None


def test_invalid_target_clears_previous_plan(harness):
    h = harness; h.send(); h.event("plan_ready"); h.send("camera")
    h.event("rejected_invalid")
    wait_for(lambda: all(ps and not ps[-1].points for ps in h.paths))


def test_valid_backend_check_without_finite_distance_is_reported_as_null(harness):
    h = harness; h.fault = "unbounded_clearance"; h.send()
    ready = h.event("plan_ready")
    assert ready["native_clearance_m"] == [None, None]


def test_missing_services_report_unavailable():
    h = Harness(services=False)
    try:
        h.send(); h.event("unavailable")
        assert not h.preview.planner.busy
    finally: h.close()
