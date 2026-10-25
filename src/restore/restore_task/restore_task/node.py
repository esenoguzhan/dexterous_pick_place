"""restore_task node (docs/task_architecture.md, section 8): the state machine behind /restore/run.

    action      /restore/run      restore_interfaces/action/Restore (mode reference or restore, dry_run)
    publishes   /restore/status   std_msgs/String, JSON, transient local
                /restore/plan     visualization_msgs/MarkerArray, transient local
"""

from __future__ import annotations

import json
import math
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Vector3
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, Header, String
from visualization_msgs.msg import Marker, MarkerArray

from restore_interfaces.action import Restore
from restore_perception.core.config import load_objects

from .clients import MotionRejected, Robot, TaskCancelled, TaskError
from .core import grasp as gr
from .core import hand
from .core import scene as sc
from .core.config import config_dir, load_config, objects_file, primitive_for
from .core.diff import Refusal, diff
from .core.order import plan
from .core.poses import Placement, lifted, world_yaw
from .core.report import Report

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


class GraspFailed(TaskError):
    """The hand closed on nothing, or lost the object: re-plan."""


class RestoreNode(Node):
    def __init__(self, **kwargs):
        super().__init__("restore_task", **kwargs)
        self.declare_parameter("config_file", str(config_dir() / "task.yaml"))
        self.declare_parameter("objects_file", str(objects_file()))
        self.declare_parameter("data_dir", "/data")
        self.declare_parameter("approach_mode", "")  # overrides pick_place.approach_mode when set
        p = {k: self.get_parameter(k).value for k in ("config_file", "objects_file", "data_dir", "approach_mode")}
        overrides = {"pick_place": {"approach_mode": p["approach_mode"]}} if p["approach_mode"] else None
        self.cfg = load_config(p["config_file"], overrides)
        self.objects = load_objects(p["objects_file"])
        self.data_dir = Path(p["data_dir"])
        self.reference = None
        self.hand_checked = False
        self._busy = threading.Lock()

        self.robot = Robot(self, self.cfg, ReentrantCallbackGroup())
        self.status_pub = self.create_publisher(String, "/restore/status", LATCHED)
        self.plan_pub = self.create_publisher(MarkerArray, "/restore/plan", LATCHED)
        # Reentrant, so a cancel request is handled while a goal runs; _busy keeps goals one at a time.
        self.server = ActionServer(self, Restore, "/restore/run", execute_callback=self._execute,
                                   goal_callback=self._on_goal, cancel_callback=self._on_cancel,
                                   callback_group=ReentrantCallbackGroup())
        self._status("idle", "waiting for a Restore goal")

    # ---------- action plumbing ----------

    def _on_goal(self, goal):
        if goal.mode not in (Restore.Goal.MODE_REFERENCE, Restore.Goal.MODE_RESTORE):
            return GoalResponse.REJECT
        return GoalResponse.REJECT if self._busy.locked() else GoalResponse.ACCEPT

    def _on_cancel(self, _):
        self.robot.cancel()
        return CancelResponse.ACCEPT

    def _status(self, phase, detail="", **fields):
        self.status_pub.publish(String(data=json.dumps({"phase": phase, "detail": detail, **fields})))
        self.get_logger().info(f"{phase}: {detail}" if detail else phase)

    def _feedback(self, handle, phase, label="", object_id="", index=0, remaining=0, detail=""):
        handle.publish_feedback(Restore.Feedback(phase=phase, object_id=object_id, label=label, index=index,
                                                 remaining=remaining, detail=detail))
        self._status(phase, detail, label=label, object_id=object_id, index=index, remaining=remaining)

    def _execute(self, handle):
        goal = handle.request
        with self._busy:
            self.robot.cancelled.clear()
            try:
                if goal.mode == Restore.Goal.MODE_REFERENCE:
                    result = self._reference(handle)
                else:
                    result = self._restore(handle, goal.dry_run)
            except TaskCancelled:
                handle.canceled()
                self._status("idle", "cancelled")
                return Restore.Result(success=False, message="cancelled")
            except TaskError as exc:
                handle.abort()
                self._status("idle", f"aborted: {exc}")
                return Restore.Result(success=False, message=str(exc))
            except Exception as exc:  # a bug: report it instead of leaving the goal hanging
                self.get_logger().error(f"restore failed: {exc!r}")
                handle.abort()
                return Restore.Result(success=False, message=f"{type(exc).__name__}: {exc}")
        handle.succeed()
        self._status("idle", result.message)
        return result

    # ---------- Init and Reference ----------

    def _init(self, handle):
        self._feedback(handle, "init", detail="checking interfaces")
        deadline = time.monotonic() + 10.0
        missing = self.robot.missing()
        while missing and time.monotonic() < deadline:
            time.sleep(0.2)
            missing = self.robot.missing()
        if missing:
            raise TaskError("not available: " + ", ".join(missing))
        if not self.hand_checked:  # 7.6: every finger must reach an empty pinch
            self._feedback(handle, "init", detail="hand check")
            primitive = self.cfg["primitives"]["pinch_wide"]
            self.robot.hand_to(hand.squeeze_target(primitive, 0.05), self.cfg["pick_place"]["hand_close_s"])
            check = hand.check_grasp(primitive, 0.05, self.robot.hand_positions(self.cfg["grasp_check"]["settle_s"]),
                                     self.cfg["grasp_check"])
            self.robot.hand_to(hand.OPEN, self.cfg["pick_place"]["hand_open_s"])
            if check.state != "missed":
                raise TaskError(f"hand check: fingers stop early in free air ({check.ratios}); power-cycle the hand")
            self.hand_checked = True
        self.robot.observe()

    def _reference(self, handle):
        self._init(handle)
        self._feedback(handle, "reference", detail="capturing the tidy scene")
        self.reference = sc.from_msg(self.robot.capture("reference", True))
        labels = ", ".join(o.label for o in self.reference.objects)
        return Restore.Result(success=True, message=f"reference: {len(self.reference.objects)} objects ({labels})",
                              report_json=json.dumps({"reference": [o.label for o in self.reference.objects]}))

    def _load_reference(self):
        if self.reference is None:
            folder = self.data_dir / "snapshots" / "reference"
            if not (folder / "snapshot.json").exists():
                raise TaskError("no reference: run mode reference first")
            self.reference = sc.from_disk(folder)
        return self.reference

    # ---------- Restore ----------

    def _capture(self):
        last = None
        for _ in range(int(self.cfg["capture_retries"]) + 1):
            try:
                return sc.from_msg(self.robot.capture("", False))
            except TaskCancelled:
                raise
            except TaskError as exc:
                last = exc
        raise TaskError(f"capture failed twice: {last}")

    def _restore(self, handle, dry_run):
        reference = self._load_reference()
        if not dry_run:
            self._init(handle)
        pp = self.cfg["pick_place"]
        report, start, index, parks, last = Report(), time.monotonic(), 0, set(), None
        while True:
            if time.monotonic() - start > self.cfg["budget_s"]:
                report.events.append("time budget used up")
                if last is not None:
                    report.finish(last, reference)
                break
            if not dry_run:
                self._feedback(handle, "observe", index=index)
                self.robot.observe()
            self._feedback(handle, "capture", index=index)
            current = self._capture()
            d = diff(reference, current, self.objects, self.cfg)
            for task in list(d.tasks):  # 7.7: at most max_attempts placements per object
                if report.attempts(task.reference_id) >= pp["max_attempts"]:
                    d.tasks.remove(task)
                    d.refused.append(Refusal(task.object_id, task.reference_id, task.label,
                                             f"not restored after {pp['max_attempts']} attempts"))
            flange = self.robot.flange_pose()
            p = plan(d, current, self.cfg, hand_xy=flange[:2, 3] if flange is not None else (0.0, 0.0))
            d.refused.extend(p.refused)
            last = d
            self.plan_pub.publish(self._markers(p, d, current))
            if dry_run:
                report.finish(d, reference)
                steps = [{"label": s.task.label, "park": s.park, "from": s.task.current.as_dict(),
                          "to": s.target.as_dict()} for s in p.steps]
                return Restore.Result(success=True, message=f"dry run: {len(p.steps)} step(s) planned; "
                                      f"{report.summary()}",
                                      report_json=json.dumps({"plan": steps, **json.loads(report.to_json())}))
            steps = [s for s in p.steps if not (s.park and s.task.object_id in parks)]
            if not steps:
                report.finish(d, reference)
                break
            step = steps[0]
            index += 1
            task = step.task
            if step.park:
                parks.add(task.object_id)
            else:
                report.attempt(task.reference_id, task.label)
            try:
                self._run_step(handle, step, current, index, len(steps))
                report.note(task.reference_id, task.label, "parked" if step.park else "placed")
            except (MotionRejected, GraspFailed) as exc:
                report.note(task.reference_id, task.label, str(exc))
        counts = report.counts()
        success = not counts.get("not restored") and not counts.get("refused")
        return Restore.Result(success=success, message=report.summary(), report_json=report.to_json())

    # ---------- one pick and place (7.4, 7.5) ----------

    def _run_step(self, handle, step, scene, index, remaining):
        task = step.task
        _, primitive = primitive_for(task.label, self.objects, self.cfg)
        pp, check_cfg = self.cfg["pick_place"], self.cfg["grasp_check"]

        def fb(phase, detail=""):
            self._feedback(handle, phase, task.label, task.object_id, index, remaining, detail)

        flange = self.robot.flange_pose()
        flange_yaw = world_yaw(flange[:3, :3]) if flange is not None else 0.0
        neighbours = [np.asarray(o.footprint)[:, :2] for o in scene.objects if o.id != task.object_id]
        grasps = gr.candidates(task, primitive, self.robot.flange_to_tcp(), flange_yaw, neighbours, pp)
        reasons = []
        for g in grasps:
            fb("pick: pre-shape", f"grasp yaw {math.degrees(g.yaw):.0f} deg")
            self.robot.hand_to(hand.preshape(primitive, g.width), pp["hand_open_s"])
            try:
                fb("pick: approach")
                if self.robot.use_approach:
                    self.robot.approach(Placement(task.current.x, task.current.y, task.current.z,
                                                  task.current.yaw + g.yaw).matrix())
                self.robot.move_to(g.pre_grasp, what="pre-grasp")
                fb("pick: descend")
                self.robot.move_to(g.grasp, pp["descend_duration_s"], what="descend")
            except MotionRejected as exc:
                reasons.append(str(exc))
                continue  # the next grasp yaw
            fb("pick: close")
            self.robot.hand_to(hand.squeeze_target(primitive, g.width), pp["hand_close_s"])
            held = self.robot.hand_positions(check_cfg["settle_s"])
            check = hand.check_grasp(primitive, g.width, held, check_cfg)
            if not check.held:
                self.robot.hand_to(hand.preshape(primitive, g.width), pp["hand_open_s"])
                self.robot.move_to(g.pre_grasp, what="back off")
                raise GraspFailed(f"grasp {check.state} {check.ratios}")
            fb("pick: lift")
            self._move_holding(g.lift, None, task, g, primitive, held, scene, "lift")
            place = gr.place_poses(step.target, g, primitive, task.rolls, pp)
            fb("place: approach", "parking spot" if step.park else "home")
            if self.robot.use_approach:
                try:
                    self.robot.approach(Placement(step.target.x, step.target.y, step.target.z,
                                                  step.target.yaw + g.yaw).matrix())
                except MotionRejected as exc:
                    self._set_down(g, task, scene, primitive)
                    raise exc
            self._move_holding(place.pre_place, None, task, g, primitive, held, scene, "pre-place")
            fb("place: descend")
            self._move_holding(place.place, pp["descend_duration_s"], task, g, primitive, held, scene, "place")
            fb("place: release")
            self.robot.hand_to(hand.preshape(primitive, g.width), pp["slow_open_s"] if task.rolls else pp["hand_open_s"])
            fb("place: retreat")
            self.robot.move_to(place.retreat, what="retreat")
            self.robot.hand_to(hand.OPEN, pp["hand_open_s"])
            return
        raise MotionRejected("no reachable grasp: " + "; ".join(reasons))

    def _move_holding(self, T, duration, task, g, primitive, held, scene, what):
        """A move with the object in the hand: on a refusal, set it down; afterwards, check for a drop."""
        try:
            self.robot.move_to(T, duration or 0.0, what=what)
        except MotionRejected:
            self._set_down(g, task, scene, primitive)
            raise
        now = self.robot.hand_positions(0.0)
        if hand.dropped(primitive, g.width, held, now, self.cfg["grasp_check"]):
            self.robot.hand_to(hand.OPEN, self.cfg["pick_place"]["hand_open_s"])
            raise GraspFailed(f"dropped during {what}")

    def _set_down(self, g, task, scene, primitive):
        """Recover: lower the held object straight down onto the table, release, back off."""
        pp = self.cfg["pick_place"]
        flange = self.robot.flange_pose()
        if flange is None:
            raise TaskError("cannot set the object down: no flange pose in TF")
        obj = gr.object_in_hand(flange, g)
        x, y = obj[0, 3], obj[1, 3]
        dz = scene.table_z(x, y) + float(task.size[2]) / 2.0 + pp["release_height_m"] - obj[2, 3]
        down = lifted(flange, dz)
        self.robot.move_to(down, pp["descend_duration_s"], what="set down")
        self.robot.hand_to(hand.preshape(primitive, g.width), pp["slow_open_s"])
        self.robot.move_to(lifted(down, pp["pre_distance_m"]), what="back off")
        self.robot.hand_to(hand.OPEN, pp["hand_open_s"])

    # ---------- markers (9.2) ----------

    def _markers(self, p, d, current):
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.cfg["world_frame"])
        out = MarkerArray(markers=[Marker(header=header, action=Marker.DELETEALL)])
        for i, step in enumerate(p.steps):
            t, color = step.task, ColorRGBA(r=1.0, g=0.6, b=0.0, a=1.0) if step.park else ColorRGBA(g=0.8, b=0.3, a=1.0)
            top = float(t.size[2]) / 2.0 + 0.02
            arrow = Marker(header=header, ns="steps", id=i, type=Marker.ARROW, action=Marker.ADD, color=color,
                           scale=Vector3(x=0.01, y=0.02, z=0.03),
                           points=[Point(x=t.current.x, y=t.current.y, z=t.current.z + top),
                                   Point(x=step.target.x, y=step.target.y, z=step.target.z + top)])
            arrow.pose.orientation.w = 1.0
            text = Marker(header=header, ns="labels", id=i, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD,
                          color=color, scale=Vector3(z=0.025), text=f"{i + 1} {t.label}" + (" (park)" if step.park else ""))
            text.pose.position = Point(x=t.current.x, y=t.current.y, z=t.current.z + top + 0.03)
            text.pose.orientation.w = 1.0
            out.markers.extend([arrow, text])
        by_id = current.by_id()
        for j, r in enumerate(d.refused):
            o = by_id.get(r.object_id)
            if o is None:
                continue
            text = Marker(header=header, ns="refused", id=j, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD,
                          color=ColorRGBA(r=0.6, g=0.6, b=0.6, a=1.0), scale=Vector3(z=0.02),
                          text=f"{r.label}: {r.reason}")
            text.pose.position = Point(x=float(o.position[0]), y=float(o.position[1]), z=float(o.position[2]) + 0.08)
            text.pose.orientation.w = 1.0
            out.markers.append(text)
        return out


def main(args=None):
    rclpy.init(args=args)
    node = RestoreNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.robot.cancel()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
