"""Execute the planner's two checked phases, with measured-state barriers."""

import copy
import json
import time
import uuid

import numpy as np
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from lara5_safety_msgs.srv import GetFramePose
from rclpy.action import ActionClient
from std_msgs.msg import String

from .planning import TrajectoryPreview
from .planning_geometry import duration, pose_error, seconds


class TrajectoryExecutor(TrajectoryPreview):
    """Own an entire command until planning or confirmed controller termination.

    Review topics are visualization only. Execution uses the in-memory plan and
    the safety-filtered action, never retained trajectory messages.
    """

    def __init__(self, node, description, flange_link, open_names, open_positions):
        self.execution = None
        self.stopping = False
        super().__init__(node, description, flange_link, open_names, open_positions,
                         status_topic="/arm/approach/status")
        def param(name, default):
            return node.declare_parameter(name, default).value
        self.joint_tolerance = param("execution_joint_tolerance", .01)
        self.position_error = param("execution_position_tolerance", .003)
        self.angle_error = param("execution_angle_tolerance", .02)
        self.settle_time = param("execution_settle_time", .2)
        self.verify_timeout = param("execution_verification_timeout", 3.)
        self.response_timeout = param("execution_goal_response_timeout", 10.)
        self.timeout_margin = param("execution_timeout_margin", 10.)
        self.cancel_timeout = param("execution_cancel_timeout", 5.)
        self.tracking_tolerance = param("execution_tracking_tolerance", .1)
        limits = [self.joint_tolerance, self.position_error, self.angle_error, self.settle_time,
                  self.verify_timeout, self.response_timeout, self.timeout_margin,
                  self.cancel_timeout, self.tracking_tolerance]
        if not np.isfinite(limits).all() or min(limits) <= 0 or self.settle_time >= self.verify_timeout:
            raise ValueError("execution limits must be positive, with settling shorter than verification")
        action = node.resolve_topic_name("/arm_trajectory_controller/follow_joint_trajectory")
        self.motion = ActionClient(node, FollowJointTrajectory, action)

    @property
    def busy(self):
        return self.stopping or self.active is not None or self.execution is not None

    @property
    def active_request(self):
        context = self.execution or self.active
        return context["id"] if context else None

    def report(self, state, message, **values):
        # Planning and execution share one command ID; rejected busy inputs have
        # their own ID and cannot replace the active command.
        request_id = str(uuid.uuid4()) if state == "rejected_busy" else self.last_id
        if state == "plan_failed":
            state = "canceled" if message == "operator cancellation" else "failed"
        if state == "idle":
            message = "Waiting for a manual object pose; hand must already be open"
        event = dict(state=state, message=message, request_id=request_id,
                     active_request_id=self.active_request, execution_enabled=True,
                     ready_for_closure=False, holder_camera_collision_checked=False,
                     manual_object_collision_checked=False)
        event.update(values)
        self.status.publish(String(data=json.dumps(event)))
        self.node.get_logger().info(f"{state}: {message}")

    def request(self, message, obj, final, grasp):
        if self.busy:
            self.report("rejected_busy", "Approach active or stopping; new target discarded")
            return
        if not self.motion.server_is_ready():
            self.last_id = str(uuid.uuid4())
            self.clear_outputs()
            self.report("unavailable", "Safety-filtered arm action unavailable; target discarded")
            return
        super().request(message, obj, final, grasp)

    def complete_plan(self, context, stamp):
        self.valid = None
        context.update(phase=0, state="planned", handle=None, cancel_reason=None,
                       cancel_sent=False, stop_error_reported=False)
        self.execution = context
        self.report("planned", "Both phases validated; starting measured-state-checked execution",
                    pregrasp_xyz=context["targets"][0][:3, 3].tolist(),
                    final_xyz=context["targets"][1][:3, 3].tolist(),
                    durations_s=[seconds(p.points[-1].time_from_start) for p in context["phases"]])
        self.send_phase(context)

    def send_phase(self, context):
        try:
            phase = context["phase"]
            path = copy.deepcopy(context["phases"][phase])
            measured = self.measured()
            if np.max(np.abs(measured-np.asarray(path.points[0].positions))) > self.drift:
                raise ValueError("Measured arm does not match the validated phase start")
            if not self.motion.server_is_ready():
                raise ValueError("Safety-filtered arm action is unavailable")
            # A preview timestamp is historical. Zero requests controller start
            # on receipt, retaining every checked position/velocity and duration.
            path.header.stamp.sec = path.header.stamp.nanosec = 0
            goal = FollowJointTrajectory.Goal(trajectory=path)
            goal.path_tolerance = [JointTolerance(name=n, position=self.tracking_tolerance) for n in self.names]
            goal.goal_tolerance = [JointTolerance(name=n, position=self.joint_tolerance) for n in self.names]
            goal.goal_time_tolerance = duration(self.verify_timeout)
        except Exception as error:
            self.finish("failed", str(error))
            return
        context.update(state="sending", handle=None, deadline=time.monotonic()+self.response_timeout)
        self.report(self.moving_state(context), "Sending validated phase to the safety-filtered arm action")
        # Mark acceptance as unknown before calling transport. Exceptions do
        # not prove that a goal was never delivered.
        try:
            self.motion.send_goal_async(goal).add_done_callback(lambda future: self.on_goal(context, future))
        except Exception as error:
            self.stop(context, f"Cannot dispatch arm goal; acceptance unknown: {error}")

    @staticmethod
    def moving_state(context):
        return "moving_to_pregrasp" if context["phase"] == 0 else "moving_to_grasp"

    def on_goal(self, context, future):
        if self.execution is not context:
            return
        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                self.finish("canceled" if context["cancel_reason"] == "operator cancellation" else "failed",
                            context["cancel_reason"] or "Safety-filtered arm action rejected the phase")
                return
            context["handle"] = handle
            handle.get_result_async().add_done_callback(lambda result: self.on_result(context, result))
            if context["cancel_reason"]:
                self.send_cancel(context)
                return
            context.update(state="moving", deadline=time.monotonic()+
                           seconds(context["phases"][context["phase"]].points[-1].time_from_start)+self.timeout_margin)
            self.report(self.moving_state(context), "Controller accepted phase; monitoring fresh open-hand feedback")
        except Exception as error:
            self.stop(context, f"Cannot obtain arm goal response: {error}")

    def on_result(self, context, future):
        if self.execution is not context:
            return
        try:
            outcome = future.result()
        except Exception as error:
            self.stop(context, f"Cannot obtain arm result; stop unconfirmed: {error}")
            return
        # Only terminal action results release ownership. Cancellation ACK alone
        # is insufficient. An unexpected nonterminal result stays locked.
        if outcome is None or outcome.status not in (GoalStatus.STATUS_SUCCEEDED,
                GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
            self.stop(context, "Arm result did not confirm controller termination")
            return
        context["handle"] = None
        if context["cancel_reason"]:
            self.finish("canceled" if context["cancel_reason"] == "operator cancellation" else "failed",
                        context["cancel_reason"]+"; controller termination confirmed")
        elif outcome.result is None:
            self.finish("failed", "Terminal arm result had no controller result payload")
        elif outcome.status != GoalStatus.STATUS_SUCCEEDED or outcome.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.finish("canceled" if outcome.status == GoalStatus.STATUS_CANCELED else "failed",
                        outcome.result.error_string or "Arm controller did not complete the phase successfully")
        else:
            now = time.monotonic()
            context.update(state="verifying", result_time=now, deadline=now+self.verify_timeout,
                           settled_since=None, verification_error="Waiting for post-result measured feedback")
            self.report("verifying_pregrasp" if context["phase"] == 0 else "verifying_grasp",
                        "Controller completed; checking fresh measured joint and grasp-frame pose")

    def endpoint_errors(self, context, measured):
        endpoint = np.asarray(context["phases"][context["phase"]].points[-1].positions)
        joint_error = float(np.max(np.abs(measured-endpoint)))
        actual = self.model.fk(measured)
        target = context["targets"][context["phase"]]
        flange_error = pose_error(actual, target)
        grasp_error = pose_error(actual @ self.node.flange_grasp, target @ self.node.flange_grasp)
        if joint_error > self.joint_tolerance or any(e[0] > self.position_error or e[1] > self.angle_error
                                                   for e in (flange_error, grasp_error)):
            raise ValueError(f"Measured endpoint mismatch: joint {joint_error:.6f} rad, "
                             f"flange {flange_error}, grasp frame {grasp_error}")
        return dict(joint_error_rad=joint_error, flange_error_m_rad=list(flange_error),
                    grasp_error_m_rad=list(grasp_error), measured_flange_xyz=actual[:3, 3].tolist())

    def verify(self, context):
        measured = self.measured()
        if any(self.samples[n][1] < context["result_time"] for n in self.names+self.open_names):
            return
        try:
            self.endpoint_errors(context, measured)
        except ValueError as error:
            context["verification_error"] = str(error)
            context["settled_since"] = None
            return
        now = time.monotonic()
        if context["settled_since"] is None:
            context["settled_since"] = now
        if now-context["settled_since"] < self.settle_time:
            return
        context.update(state="verifying_fk", verify_q=measured.copy(), deadline=now+self.service_timeout)
        request = GetFramePose.Request(start_state=self.state_message(measured),
                                      frame=self.flange_link, reference_frame=self.node.world_frame)
        try:
            future = self.fk_client.call_async(request)
            context["verify_future"] = future
            future.add_done_callback(lambda result: self.on_measured_fk(context, result))
        except Exception as error:
            self.finish("failed", f"Cannot verify measured native FK: {error}")

    def on_measured_fk(self, context, future):
        if self.execution is not context or context["state"] != "verifying_fk":
            return
        try:
            measured = self.measured()
            if np.max(np.abs(measured-context["verify_q"])) > self.drift:
                raise ValueError("Measured arm changed during endpoint FK verification")
            self.checked_pose(future.result(), self.model.fk(context["verify_q"]))
            metrics = self.endpoint_errors(context, measured)
            if context["phase"] == 0:
                context.update(phase=1, state="planned")
                self.send_phase(context)
            else:
                self.finish("succeeded", "Positioning verified; hand open and ready for explicit closure",
                            ready_for_closure=True, **metrics)
        except Exception as error:
            self.finish("failed", f"Measured endpoint verification failed: {error}")

    def stop(self, context, reason):
        if self.execution is not context:
            return
        if context["state"] in ("planned", "verifying", "verifying_fk"):
            self.finish("canceled" if reason == "operator cancellation" else "failed", reason)
            return
        if context["cancel_reason"] is None:
            context.update(cancel_reason=reason, state="canceling", deadline=time.monotonic()+self.cancel_timeout)
            self.report("canceling", reason+"; requesting controller cancellation")
        self.send_cancel(context)

    def send_cancel(self, context):
        if context["handle"] is not None and not context["cancel_sent"]:
            context["cancel_sent"] = True
            try:
                context["handle"].cancel_goal_async()
            except Exception as error:
                self.report("error", f"Cancellation transport failed; controller stop unconfirmed: {error}")

    def cancel_active(self):
        self.stopping = True
        if self.active:
            self.fail("operator cancellation")
        if self.execution:
            self.stop(self.execution, "operator cancellation")

    def finish(self, state, message, **values):
        context = self.execution
        self.execution = None
        if context and context.get("verify_future") and not context["verify_future"].done():
            self.fk_client.remove_pending_request(context["verify_future"])
            context["verify_future"].cancel()
        if state != "succeeded":
            self.clear_outputs()
        self.report(state, message, **values)

    def on_timer(self):
        if self.active:
            super().on_timer()
        context = self.execution
        if context is None:
            return
        now = time.monotonic()
        if context["state"] == "canceling":
            if now > context["deadline"] and not context["stop_error_reported"]:
                context["stop_error_reported"] = True
                self.report("error", "Controller stop not confirmed; approach remains locked against new commands")
            return
        try:
            self.measured()  # Feedback and open-hand requirements apply throughout.
            if now > context["deadline"]:
                reason = context.get("verification_error", "Arm phase timed out") if context["state"] == "verifying" else "Arm phase timed out"
                self.stop(context, reason)
            elif context["state"] == "verifying":
                self.verify(context)
        except ValueError as error:
            self.stop(context, str(error))
