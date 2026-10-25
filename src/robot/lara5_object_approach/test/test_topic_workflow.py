"""Two-phase command transport against isolated fake services/controllers."""

import copy
import time

import pytest
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.parameter import Parameter
from rclpy.task import Future

from lara5_object_approach.node import ObjectApproach
from test_planning_workflow import Harness as PlanningHarness, wait_for


class Harness(PlanningHarness):
    def __init__(self, server=True, services=True):
        self.goals, self.executing = [], []
        self.accept = True
        self.release = True
        self.success = True
        self.move_to_endpoint = True
        self.cancel_requests = 0
        self.complete_cancellation = True
        self.endpoint_fault = False
        self.fault_phase = 1
        self.freeze_on_result = False
        self.measured_fk_failure = False
        self.late_accept = False
        self.acceptance = None
        super().__init__(services=services, node_class=ObjectApproach, parameters=[
            Parameter("execution_settle_time", value=.05),
            Parameter("execution_verification_timeout", value=.7),
            Parameter("execution_goal_response_timeout", value=.3),
            Parameter("execution_cancel_timeout", value=.3)])
        self.server = None
        if server:
            self.server = ActionServer(self.peer, FollowJointTrajectory, self.prefix+"/motion",
                execute_callback=self.execute, goal_callback=self.on_goal,
                cancel_callback=self.cancel, callback_group=ReentrantCallbackGroup())
            wait_for(self.preview.planner.motion.server_is_ready)

    async def on_goal(self, goal):
        if self.late_accept:
            self.acceptance = Future()
            await self.acceptance
        return GoalResponse.ACCEPT if self.accept else GoalResponse.REJECT

    def cancel(self, handle):
        self.cancel_requests += 1
        return CancelResponse.ACCEPT

    async def execute(self, handle):
        self.goals.append(copy.deepcopy(handle.request))
        future = Future()
        self.executing.append((handle, future, time.monotonic()))
        await future
        if handle.is_cancel_requested:
            handle.canceled()
            return FollowJointTrajectory.Result(error_code=-1, error_string="Fake controller canceled")
        if self.move_to_endpoint:
            self.q = list(handle.request.trajectory.points[-1].positions)
            if self.endpoint_fault and len(self.goals) == self.fault_phase:
                self.q[0] += .015  # Joint tolerance exceeded; phase B must never run.
        if self.freeze_on_result:
            self.send_state = False
        if self.success:
            handle.succeed()
        else:
            handle.abort()
        return FollowJointTrajectory.Result(error_code=0 if self.success else -1,
                                            error_string="Fake controller result")

    def fk(self, request, response):
        response = super().fk(request, response)
        if self.measured_fk_failure and self.goals:
            response.success, response.message = False, "Measured native FK unavailable"
        return response

    def tick(self):
        super().tick()
        for handle, future, start in list(self.executing):
            canceled = handle.is_cancel_requested and self.complete_cancellation
            if canceled or (self.release and time.monotonic()-start > .08):
                self.executing.remove((handle, future, start))
                if not future.done(): future.set_result(None)

    def close(self):
        if self.acceptance and not self.acceptance.done(): self.acceptance.set_result(None)
        self.release = True
        self.complete_cancellation = True
        wait_for(lambda: not self.executing)
        super().close()
        if self.server is not None: self.server.destroy()


@pytest.fixture
def harness():
    h = Harness()
    try: yield h
    finally: h.close()


def test_command_executes_exact_checked_phases_and_verifies_open_hand(harness):
    h = harness; h.send(); done = h.event("succeeded")
    assert done["active_request_id"] is None
    assert done["request_id"] == h.event("planning")["request_id"]
    assert done["ready_for_closure"] is True
    assert done["grasp_error_m_rad"] == pytest.approx([0., 0.])
    assert len(h.goals) == len(h.plan_requests) == 2
    assert h.goals[0].trajectory.points[-1].positions == h.goals[1].trajectory.points[0].positions
    for i, goal in enumerate(h.goals):
        assert goal.trajectory.header.stamp.sec == goal.trajectory.header.stamp.nanosec == 0
        assert goal.trajectory.points == h.check_requests[i].trajectory.points
        assert goal.goal_tolerance[0].position == .01
    states = [e["state"] for e in h.events]
    assert states.index("verifying_pregrasp") < states.index("moving_to_grasp") < states.index("succeeded")
    # No finger action or motion publisher exists on the approach node.
    clients = h.peer.get_client_names_and_types_by_node(h.preview.get_name(), h.preview.get_namespace())
    assert not any("hand_controller" in name or "move_to_pose" in name for name, _ in clients)
    publishers = h.peer.get_publisher_names_and_types_by_node(h.preview.get_name(), h.preview.get_namespace())
    assert not any("grasp/target" in name or "joint_trajectory" in name for name, _ in publishers)


@pytest.mark.parametrize("fault", ["second_phase", "collision", "jump", "fk", "incomplete"])
def test_both_phases_must_validate_before_any_arm_motion(harness, fault):
    h = harness; h.fault = fault; h.send(); h.event("failed")
    assert not h.goals
    assert h.preview.active_request is None


def test_busy_target_discarded_without_retargeting(harness):
    h = harness; h.release = False; h.send()
    first = h.event("moving_to_pregrasp")["request_id"]
    wait_for(lambda: len(h.goals) == 1)
    h.send(); rejected = h.event("rejected_busy")
    assert rejected["active_request_id"] == first
    assert rejected["request_id"] != first
    assert len(h.plan_requests) == 2 and len(h.goals) == 1
    h.release = True; h.event("succeeded")


@pytest.mark.parametrize("frame", ["camera", ""])
def test_bad_frame_never_plans_or_moves(harness, frame):
    h = harness; h.send(frame); h.event("rejected_invalid")
    assert not h.goals and not h.plan_requests


@pytest.mark.parametrize("condition", ["closed", "stale", "missing"])
def test_initial_feedback_preconditions(harness, condition):
    h = harness
    if condition == "closed":
        h.hand = [.2]*6
        wait_for(lambda: h.preview.planner.samples[h.preview.planner.open_names[0]][0] == .2)
    elif condition == "stale":
        h.stamp_offset = 5.
        wait_for(lambda: h.peer.get_clock().now().nanoseconds/1e9-h.preview.planner.samples[h.preview.planner.names[0]][2] > 4)
    else:
        h.send_state = False; h.preview.planner.samples.pop(h.preview.planner.names[0])
    h.send(); h.event("rejected_state")
    assert not h.plan_requests and not h.goals


@pytest.mark.parametrize("fault", ["not_at_endpoint", "joint_error", "controller_abort"])
def test_controller_result_alone_does_not_allow_second_phase(harness, fault):
    h = harness
    if fault == "not_at_endpoint": h.move_to_endpoint = False
    if fault == "joint_error": h.endpoint_fault = True
    if fault == "controller_abort": h.success = False
    h.send(); h.event("failed")
    assert len(h.goals) == 1
    assert not any(e["state"] == "succeeded" for e in h.events)


def test_fresh_open_hand_feedback_is_required_during_motion(harness):
    h = harness; h.release = False; h.send(); wait_for(lambda: len(h.goals) == 1)
    h.hand = [.2]*6
    assert "open posture" in h.event("failed")["message"]
    assert h.cancel_requests == 1 and len(h.goals) == 1


def test_feedback_loss_cancels_and_blocks_final_phase(harness):
    h = harness; h.release = False; h.send(); wait_for(lambda: len(h.goals) == 1)
    h.send_state = False
    assert "timestamp" in h.event("failed")["message"]
    assert h.cancel_requests == 1 and len(h.goals) == 1


def test_post_result_feedback_is_required_even_after_controller_success(harness):
    h = harness; h.freeze_on_result = True; h.send()
    h.event("verifying_pregrasp"); h.event("failed")
    assert len(h.goals) == 1 and h.cancel_requests == 0


def test_measured_native_fk_failure_blocks_final_phase(harness):
    h = harness; h.measured_fk_failure = True; h.send(); h.event("failed")
    assert len(h.goals) == 1


def test_final_endpoint_mismatch_is_not_positioning_success(harness):
    h = harness; h.endpoint_fault = True; h.fault_phase = 2
    h.send(); h.event("failed")
    assert len(h.goals) == 2
    assert not any(e["state"] == "succeeded" for e in h.events)


def test_motion_deadline_requests_cancellation(harness):
    h = harness; h.release = False; h.send()
    wait_for(lambda: len(h.goals) == 1 and h.preview.planner.execution["state"] == "moving")
    h.preview.planner.execution["deadline"] = 0.
    assert "timed out" in h.event("failed")["message"]
    assert len(h.goals) == 1 and h.cancel_requests == 1


def test_shutdown_during_planning_sends_no_motion(harness):
    h = harness; h.hold = True; h.send(); wait_for(lambda: len(h.plan_requests) == 1)
    h.preview.cancel_active(); h.event("canceled")
    assert not h.goals and h.preview.active_request is None


def test_unknown_goal_transport_outcome_keeps_command_locked(harness):
    h = harness
    def fail_send(_):
        raise RuntimeError("transport outcome unknown")
    h.preview.planner.motion.send_goal_async = fail_send
    h.send(); h.event("error")
    assert h.preview.active_request is not None
    h.send(); h.event("rejected_busy")
    assert not h.goals


def test_controller_rejection_releases_command(harness):
    h = harness; h.accept = False; h.send(); h.event("failed")
    assert not h.goals and h.preview.active_request is None


def test_shutdown_cancels_active_phase(harness):
    h = harness; h.release = False; h.send(); wait_for(lambda: len(h.goals) == 1)
    h.preview.cancel_active(); h.event("canceled")
    assert h.cancel_requests == 1 and len(h.goals) == 1
    assert h.preview.active_request is None


def test_unconfirmed_stop_keeps_command_locked(harness):
    h = harness; h.release = False; h.complete_cancellation = False
    h.send(); wait_for(lambda: len(h.goals) == 1)
    h.preview.cancel_active(); h.event("error")
    assert h.preview.active_request is not None
    h.send(); h.event("rejected_busy")
    assert len(h.goals) == 1
    h.complete_cancellation = True; h.event("canceled")


def test_late_goal_acceptance_after_timeout_is_canceled(harness):
    h = harness; h.release = False; h.late_accept = True; h.send()
    wait_for(lambda: h.acceptance is not None)
    h.event("canceling")
    assert h.preview.active_request is not None
    h.send(); h.event("rejected_busy")
    h.acceptance.set_result(None)
    assert "timed out" in h.event("failed")["message"]
    assert h.cancel_requests == 1 and len(h.goals) == 1


def test_missing_action_or_services_discard_command():
    for options in ({"server": False}, {"services": False}):
        h = Harness(**options)
        try:
            h.send(); h.event("unavailable")
            assert not h.goals and h.preview.active_request is None
        finally: h.close()
