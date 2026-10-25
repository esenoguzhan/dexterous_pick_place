#!/usr/bin/python3

"""Robot side of the camera calibration (docs/perception_architecture.md, section 3.4).

The cameras hang on the Jetson, which detects the ChArUco board (on the table, or fixed to the hand)
and solves. This script runs next to the robot stack: at every calibration pose it waits for the arm
to stand still, reads the flange pose from TF and asks the Jetson to record the board in each camera
at that pose. Set the board location and the camera mounts, watch the detection and solve on the
Jetson's page http://<jetson>:8090/calibration.

    manual  move the arm yourself (for example with move_cartesian.py), press Enter to record
    auto    replay robot.joints from calibration_session.json through the safety filter,
            record each one and return to the starting joints; --random uses generated Cartesian poses
    solve   solve on the Jetson and print the result; add --save to write extrinsics.yaml there

    ros2 run lara5_controller_manager calibrate_cameras.py --jetson http://192.168.2.101:8090 auto --session /ws/calibration_session.json
    ros2 run lara5_controller_manager calibrate_cameras.py --jetson http://192.168.2.101:8090 auto --session /ws/calibration_session.json --dry-run
    ros2 run lara5_controller_manager calibrate_cameras.py --jetson http://192.168.2.101:8090 auto --random --pivot 0.4
    ros2 run lara5_controller_manager calibrate_cameras.py --jetson http://192.168.2.101:8090 manual
    ros2 run lara5_controller_manager calibrate_cameras.py --jetson http://192.168.2.101:8090 solve --save

--pivot tilts the flange about a point that far along its z axis instead of about the flange itself,
so a camera on the hand keeps looking at the same spot. The safety filter knows neither the board nor
a camera on the hand: start auto with plenty of room around the hand and keep the e-stop in reach.
"""

import argparse
import json
import math
from pathlib import Path
import random
import sys
import threading
import time
import urllib.error
import urllib.request

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from geometry_msgs.msg import PoseStamped
from lara5_safety_msgs.action import MoveToPose
from rclpy.action import ActionClient
from rclpy.action.graph import get_action_server_names_and_types_by_node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = [f"lara5_joint{i}" for i in range(1, 7)]
POSITION_TOLERANCE_M = 0.005
ROTATION_TOLERANCE_RAD = math.radians(2.0)
JOINT_TOLERANCE_RAD = 0.01
JOINT_ACTION = "/arm_trajectory_controller/follow_joint_trajectory"


class MotionVerificationError(RuntimeError):
    """The controller reported success without fresh feedback at the target."""


# ---------- quaternions (x, y, z, w) ----------

def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_from_axis_angle(axis, angle):
    s = math.sin(angle / 2) / math.sqrt(sum(v * v for v in axis))
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2))


def quat_rotate(q, v):
    x, y, z, _ = quat_multiply(quat_multiply(q, (v[0], v[1], v[2], 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return (x, y, z)


def orbit_pose(start_p, start_q, offset, delta_q, pivot):
    """Start pose tilted by delta_q (in the parent frame) about a pivot point, then shifted by offset.

    pivot is a vector in the flange frame from the flange to the pivot point."""
    centre = [a + b for a, b in zip(start_p, quat_rotate(start_q, pivot))]
    arm = quat_rotate(delta_q, [a - b for a, b in zip(start_p, centre)])
    position = [c + a + o for c, a, o in zip(centre, arm, offset)]
    return position, quat_multiply(delta_q, start_q)


def pose_errors(position, orientation, measured_position, measured_orientation):
    distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(position, measured_position)))
    norm = math.sqrt(sum(v * v for v in orientation) * sum(v * v for v in measured_orientation))
    if norm == 0 or not math.isfinite(norm) or not math.isfinite(distance):
        raise MotionVerificationError("Invalid measured or requested flange pose")
    dot = abs(sum(a * b for a, b in zip(orientation, measured_orientation))) / norm
    return distance, 2 * math.acos(min(1.0, dot))


def load_joint_waypoints(path):
    """Read all targets before moving. Joint names, not dictionary order, determine the six values."""
    try:
        data = json.loads(Path(path).read_text())
    except (OSError, ValueError) as exc:
        raise ValueError(f"Cannot read calibration session {path}: {exc}") from exc
    samples = data.get("samples") if isinstance(data, dict) else None
    if not isinstance(samples, list) or not samples:
        raise ValueError("Calibration session needs a non-empty samples list")
    waypoints = []
    for index, sample in enumerate(samples):
        if not isinstance(sample, dict):
            raise ValueError(f"Session entry {index + 1} is not a sample object")
        sample_id = sample.get("id", index + 1)
        robot = sample.get("robot")
        joints = robot.get("joints") if isinstance(robot, dict) else None
        if not isinstance(joints, dict):
            raise ValueError(f"Sample {sample_id}: robot.joints is missing or is not a joint-name map")
        values = []
        for name in ARM_JOINTS:
            value = joints.get(name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"Sample {sample_id}: {name} needs a finite position in radians")
            values.append(float(value))
        waypoints.append({"sample_id": sample_id, "positions": values})
    return waypoints


def joint_trajectory(start, target, duration=0.0, speed=0.2):
    """A joint-space line with zero endpoint velocity/acceleration and a bounded quintic peak speed."""
    if len(start) != len(ARM_JOINTS) or len(target) != len(ARM_JOINTS) or \
            not all(math.isfinite(v) for v in list(start) + list(target)):
        raise ValueError("Joint trajectories require six finite start and target positions")
    if not math.isfinite(speed) or not 0 < speed <= 0.5:
        raise ValueError("Replay joint speed must be greater than 0 and at most 0.5 rad/s")
    if not math.isfinite(duration) or duration < 0:
        raise ValueError("Move duration must be finite and nonnegative")
    seconds = max(3.0, duration, 1.875 * max(abs(b - a) for a, b in zip(start, target)) / speed)
    trajectory = JointTrajectory()
    trajectory.joint_names = list(ARM_JOINTS)
    for positions, stamp in [(start, 0.0), (target, seconds)]:
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        point.velocities = [0.0] * len(ARM_JOINTS)
        point.accelerations = [0.0] * len(ARM_JOINTS)
        point.time_from_start = Duration(seconds=stamp).to_msg()
        trajectory.points.append(point)
    return trajectory


# ---------- Jetson HTTP ----------

def jetson(base, path, body=None, method=None, timeout=30):
    """JSON request to the Jetson's perception app. Returns (ok, body)."""
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(base.rstrip("/") + path, data=data, method=method or ("POST" if data else "GET"),
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as res:
            return True, json.loads(res.read() or b"{}")
    except urllib.error.HTTPError as err:
        try:
            return False, json.loads(err.read()).get("detail")
        except ValueError:
            return False, f"HTTP {err.code}"
    except urllib.error.URLError as err:
        return False, f"Jetson not reachable at {base}: {err.reason}"
    except TimeoutError:
        return False, f"Jetson request {path} timed out after {timeout} s; check the calibration page"


def describe_cameras(cameras):
    parts = []
    for serial, c in sorted(cameras.items()):
        if c.get("found"):
            parts.append(f"{serial}: {c['corners']} corners, {c['reproj_px']:.2f} px, {c['distance_m']:.2f} m")
        else:
            parts.append(f"{serial}: no board" + (f" ({c['error']})" if c.get("error") else f" ({c.get('corners', 0)} corners)"))
    return "; ".join(parts)


# ---------- ROS ----------

class Sampler(Node):
    def __init__(self, args):
        super().__init__("calibrate_cameras")
        self.args = args
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.positions = {}
        self.arm_updates = 0
        self.last_arm_update = 0.0
        self.lock = threading.Lock()
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.move_client = ActionClient(self, MoveToPose, "/safety/arm/move_to_pose")
        self.joint_client = ActionClient(self, FollowJointTrajectory, JOINT_ACTION)

    def on_joints(self, msg):
        with self.lock:
            self.positions.update(zip(msg.name, msg.position))
            if set(ARM_JOINTS).issubset(msg.name):
                self.arm_updates += 1
                self.last_arm_update = time.monotonic()

    def arm_positions(self):
        with self.lock:
            return {j: self.positions[j] for j in ARM_JOINTS if j in self.positions}

    def arm_snapshot(self):
        with self.lock:
            return ({j: self.positions[j] for j in ARM_JOINTS if j in self.positions},
                    self.arm_updates, self.last_arm_update)

    def wait_still(self, timeout=15.0):
        """Wait until no arm joint moves more than 0.0005 rad in 0.3 s, then settle."""
        deadline = time.monotonic() + timeout
        before, previous_update, _ = self.arm_snapshot()
        while time.monotonic() < deadline:
            time.sleep(0.3)
            now, update, received = self.arm_snapshot()
            if update > previous_update and time.monotonic() - received < 0.5 and \
                    len(now) == len(ARM_JOINTS) and len(before) == len(ARM_JOINTS) and \
                    all(math.isfinite(now[j]) and math.isfinite(before[j]) and
                        abs(now[j] - before[j]) < 0.0005 for j in ARM_JOINTS):
                time.sleep(self.args.settle)
                settled, settled_update, received = self.arm_snapshot()
                if settled_update >= update and time.monotonic() - received < 0.5 and \
                        all(math.isfinite(settled[j]) and abs(settled[j] - now[j]) < 0.0005 for j in ARM_JOINTS):
                    return True
            before, previous_update = now, update
        return False

    def verify_target(self, position, orientation):
        if not self.wait_still():
            raise MotionVerificationError("No fresh, stationary joint feedback after the move")
        try:
            measured_position, measured_orientation = self.flange_pose()
        except Exception as exc:
            raise MotionVerificationError(f"Cannot read the measured flange pose: {exc}") from exc
        distance, angle = pose_errors(position, orientation, measured_position, measured_orientation)
        if distance > POSITION_TOLERANCE_M or angle > ROTATION_TOLERANCE_RAD:
            raise MotionVerificationError(
                f"Controller reported success, but measured flange is {distance * 1000:.1f} mm / "
                f"{math.degrees(angle):.1f} degrees from the requested pose "
                f"(limits {POSITION_TOLERANCE_M * 1000:g} mm / {math.degrees(ROTATION_TOLERANCE_RAD):g} degrees). "
                "Check robot drive/control status and controller logs.")
        print(f"  Measured target error: {distance * 1000:.1f} mm / {math.degrees(angle):.2f} degrees")

    def joint_server_ready(self):
        if not self.joint_client.wait_for_server(timeout_sec=5.0):
            return False
        providers = []
        for name, namespace in self.get_node_names_and_namespaces():
            actions = get_action_server_names_and_types_by_node(self, name, namespace)
            if any(action == JOINT_ACTION for action, _ in actions):
                providers.append(namespace.rstrip("/") + "/" + name)
        # With safety:=off the trajectory controller owns this same public action.
        return providers == ["/safety_filter"]

    def verify_joints(self, target):
        if not self.wait_still():
            raise MotionVerificationError("No fresh, stationary joint feedback after the joint move")
        measured = self.arm_positions()
        errors = {name: abs(measured[name] - value) for name, value in zip(ARM_JOINTS, target)}
        worst = max(errors, key=errors.get)
        if not all(math.isfinite(v) for v in errors.values()) or errors[worst] > JOINT_TOLERANCE_RAD:
            raise MotionVerificationError(
                f"Measured {worst} is {errors[worst]:.4f} rad from the recorded target "
                f"(limit {JOINT_TOLERANCE_RAD:g} rad). Check robot drive/control status and controller logs.")
        print(f"  Measured maximum joint error: {errors[worst]:.5f} rad")

    def move_to_joints(self, target):
        if not self.wait_still():
            raise MotionVerificationError("No fresh, stationary joint feedback before the joint move")
        start = self.arm_positions()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_trajectory([start[j] for j in ARM_JOINTS], target,
                                           self.args.duration, self.args.joint_speed)
        goal.path_tolerance = [JointTolerance(name=j, position=0.1) for j in ARM_JOINTS]
        goal.goal_tolerance = [JointTolerance(name=j, position=JOINT_TOLERANCE_RAD) for j in ARM_JOINTS]
        goal.goal_time_tolerance = Duration(seconds=3.0).to_msg()
        send = self.joint_client.send_goal_async(goal)
        # Keep the acceptance future until it resolves, so a late accepted goal can be canceled.
        try:
            handle = wait(send, 10.0)
        except KeyboardInterrupt:
            send.add_done_callback(lambda future: future.result().cancel_goal_async()
                                   if future.result() is not None and future.result().accepted else None)
            raise
        if handle is None:
            send.add_done_callback(lambda future: future.result().cancel_goal_async()
                                   if future.result() is not None and future.result().accepted else None)
            raise MotionVerificationError("No joint-goal response within 10 s; cancellation requested if accepted")
        if not handle.accepted:
            return False, "joint path rejected by the safety filter (see /safety/status)"
        stamp = goal.trajectory.points[-1].time_from_start
        timeout = max(120.0, stamp.sec + stamp.nanosec / 1e9 + 15.0)
        try:
            done = wait(handle.get_result_async(), timeout)
        except KeyboardInterrupt:
            wait(handle.cancel_goal_async(), 5.0)
            raise
        if done is None:
            wait(handle.cancel_goal_async(), 5.0)
            raise MotionVerificationError("Joint motion timed out; cancellation requested")
        if done.status != GoalStatus.STATUS_SUCCEEDED or \
                done.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise MotionVerificationError(f"Joint controller did not complete the move: {done.result.error_string}")
        self.verify_joints(target)
        return True, "recorded joint target reached"

    def flange_pose(self):
        t = self.tf.lookup_transform(self.args.parent, self.args.child, Time(), timeout=Duration(seconds=2.0))
        p, q = t.transform.translation, t.transform.rotation
        return [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]

    def record(self):
        if not self.wait_still():
            print("  The arm did not come to rest within 15 s; not recorded.")
            return None
        try:
            translation, rotation = self.flange_pose()
        except Exception as exc:  # tf2 raises several exception types
            print(f"  No transform {self.args.parent} -> {self.args.child}: {exc}")
            return None
        ok, body = jetson(self.args.jetson, "/api/calib/sample", {"robot": {
            "parent": self.args.parent, "child": self.args.child,
            "translation": translation, "rotation_xyzw": rotation, "joints": self.arm_positions()}})
        if ok:
            print(f"  Sample {body['id']} recorded ({body['samples']} in total): {describe_cameras(body['cameras'])}")
            return body["id"]
        if isinstance(body, dict):
            print(f"  Not recorded: {body.get('message')}. {describe_cameras(body.get('cameras', {}))}")
        else:
            print(f"  Not recorded: {body}")
        return None

    def move_to(self, position, orientation):
        """Absolute flange pose in the parent frame through the safety filter. Returns (ok, message)."""
        goal = MoveToPose.Goal()
        goal.relative = False
        goal.duration = self.args.duration
        goal.target = PoseStamped()
        goal.target.header.frame_id = self.args.parent
        goal.target.pose.position.x, goal.target.pose.position.y, goal.target.pose.position.z = position
        o = goal.target.pose.orientation
        o.x, o.y, o.z, o.w = orientation
        handle = wait(self.move_client.send_goal_async(goal), 10.0)
        if handle is None or not handle.accepted:
            return False, "goal rejected by the safety filter (see /safety/status)"
        done = wait(handle.get_result_async(), 120.0)
        if done is None:
            wait(handle.cancel_goal_async(), 5.0)
            raise MotionVerificationError("No controller result within 120 s; cancellation requested")
        return done.result.success, done.result.message


def wait(future, timeout):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.02)
    return future.result() if future.done() else None


def run_manual(node):
    print("Move the arm so the board faces the cameras, then press Enter to record.")
    print("Commands: Enter = record, u = delete the last sample, s = solve, q = quit")
    recorded = []
    while True:
        try:
            command = input("> ").strip().lower()
        except EOFError:
            return 0
        if command == "q":
            return 0
        if command == "u":
            if not recorded:
                print("  Nothing to delete.")
                continue
            sample_id = recorded.pop()
            ok, body = jetson(node.args.jetson, f"/api/calib/samples/{sample_id}", method="DELETE")
            print(f"  Sample {sample_id} deleted." if ok else f"  Delete failed: {body}")
        elif command == "s":
            print_solution(node.args.jetson, save=False)
        elif command == "":
            sample_id = node.record()
            if sample_id is not None:
                recorded.append(sample_id)
        else:
            print("  Unknown command.")


def run_recorded_auto(node):
    args = node.args
    if not node.joint_server_ready():
        print(f"{JOINT_ACTION} must have exactly one server, /safety_filter. "
              "Start the controller launch with safety:=on.")
        return 1
    if not node.wait_still():
        print("No fresh, stationary arm feedback; start replay from a resting pose.")
        return 1
    start = node.arm_positions()
    start_joints = [start[j] for j in ARM_JOINTS]
    print(f"Replay {len(args.waypoints)} recorded joint poses from {Path(args.session).resolve()}.")
    print("Sample order: " + ", ".join(str(w['sample_id']) for w in args.waypoints))
    print("Joint-space paths between the recorded endpoints are checked by the safety filter.")
    print("The safety filter knows neither the board nor a camera on the hand. Keep the e-stop in reach.")
    print("After the last sample, return to the starting joints.")
    if not args.yes and input("Start? [y/N] ").strip().lower() != "y":
        return 0
    recorded = failed = 0
    try:
        for index, waypoint in enumerate(args.waypoints, 1):
            print(f"Pose {index}/{len(args.waypoints)}: recorded sample {waypoint['sample_id']}, joints [rad] "
                  + ", ".join(f"{v:.5f}" for v in waypoint["positions"]))
            ok, message = node.move_to_joints(waypoint["positions"])
            if not ok:
                print(f"  Skipped: {message}")
                failed += 1
                continue
            if node.record() is not None:
                recorded += 1
            else:
                failed += 1
        ok, message = node.move_to_joints(start_joints)
        if not ok:
            print(f"Could not return to the starting joints: {message}")
            return 1
    except MotionVerificationError as exc:
        print(f"\nReplay stopped: {exc}")
        print("No further motion commands, samples, or solve requests will be sent. Check the robot before retrying.")
        return 1
    except KeyboardInterrupt:
        print("\nReplay interrupted. No return-to-start motion will be requested.")
        return 130
    print(f"Back at the starting joints. Recorded {recorded} samples, {failed} poses skipped.")
    return 0 if recorded and print_solution(args.jetson, save=False) else 1


def run_auto(node):
    if not node.args.random:
        return run_recorded_auto(node)
    args = node.args
    if not node.move_client.wait_for_server(timeout_sec=5.0):
        print("/safety/arm/move_to_pose is not available. Is the controller launch running with safety:=on?")
        return 1
    if not node.wait_still():
        print("The arm is moving; start auto from a resting pose.")
        return 1
    start_p, start_q = node.flange_pose()
    sign = -1.0 if args.pivot_axis.startswith("-") else 1.0
    pivot = [sign * args.pivot if args.pivot_axis.endswith(a) else 0.0 for a in "xyz"]
    print(f"Start pose in {args.parent}: ({start_p[0]:.3f}, {start_p[1]:.3f}, {start_p[2]:.3f}).")
    print(f"{args.count} poses within ±{args.reach * 100:.0f} cm and {args.min_tilt:.0f} to {args.tilt:.0f}° of tilt "
          + (f"about a point {args.pivot:.2f} m along the flange's {args.pivot_axis} axis, " if args.pivot else "")
          + "then back to the start pose.")
    print("The safety filter knows neither the board nor a camera on the hand. Keep the e-stop in reach.")
    if not args.yes and input("Start? [y/N] ").strip().lower() != "y":
        return 0

    rng = random.Random(args.seed)
    recorded = failed = 0
    try:
        for k in range(args.count):
            offset = [rng.uniform(-args.reach, args.reach) for _ in range(3)]
            axis = [rng.gauss(0, 1) for _ in range(3)]
            angle = math.radians(rng.uniform(args.min_tilt, args.tilt))
            position, orientation = orbit_pose(start_p, start_q, offset, quat_from_axis_angle(axis, angle), pivot)
            print(f"Pose {k + 1}/{args.count}: offset ({offset[0]:+.3f}, {offset[1]:+.3f}, {offset[2]:+.3f}) m, "
                  f"tilt {math.degrees(angle):.0f}°")
            ok, message = node.move_to(position, orientation)
            if not ok:
                print(f"  Skipped: {message}")
                failed += 1
                continue
            node.verify_target(position, orientation)
            if node.record() is not None:
                recorded += 1
            else:
                failed += 1
    except MotionVerificationError as exc:
        print(f"\nAuto stopped: {exc}")
        print("No further motion commands, samples, or solve requests will be sent. Check the robot before retrying.")
        return 1
    except KeyboardInterrupt:
        print("\nStopped. Returning to the start pose.")
    try:
        ok, message = node.move_to(start_p, start_q)
        if ok:
            node.verify_target(start_p, start_q)
    except MotionVerificationError as exc:
        print(f"Could not verify the return to the start pose: {exc}")
        return 1
    print("Back at the start pose." if ok else f"Could not return to the start pose: {message}")
    print(f"Recorded {recorded} samples, {failed} poses skipped.")
    if recorded:
        print_solution(args.jetson, save=False)
    return 0


def print_solution(base, save):
    ok, result = jetson(base, "/api/calib/solve", {}, timeout=120)
    if not ok:
        print(f"  Not solved: {result}")
        return False
    for serial, c in sorted(result["cameras"].items()):
        p = c["translation"]
        print(f"  {c['name']} ({serial}, {c['mount']}): position ({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f}) m in {c['parent_frame']}, "
              f"{c['samples_used']}/{c['samples_total']} samples, reprojection {c['reprojection_rms_px']:.2f} px, "
              f"board {c['board_position_rms_mm']:.1f} mm / {c['board_rotation_rms_deg']:.2f}° RMS"
              + (f", dropped {c['dropped']}" if c["dropped"] else ""))
    for serial, reason in result.get("skipped", {}).items():
        print(f"  {serial} not solved: {reason}")
    if save:
        ok, body = jetson(base, "/api/calib/save", {})
        print(f"  Saved {body['path']} on the Jetson." if ok else f"  Not saved: {body}")
    return ok


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--jetson", required=True, help="perception app on the Jetson, e.g. http://192.168.2.101:8090")
    parser.add_argument("--parent", default="world", help="robot frame the camera poses are expressed in")
    parser.add_argument("--child", default="lara5_flange", help="robot link the board is fixed to")
    parser.add_argument("--settle", type=float, default=1.0, help="wait after the arm stops [s]")
    parser.add_argument("--duration", type=float, default=0.0, help="auto: time per move [s], 0 = speed limits")
    sub = parser.add_subparsers(dest="mode", required=True)
    sub.add_parser("manual", help="record while you move the arm")
    auto = sub.add_parser("auto", help="replay recorded session joints; --random generates Cartesian poses")
    source = auto.add_mutually_exclusive_group()
    source.add_argument("--session", type=Path, default=Path("calibration_session.json"),
                        help="exported calibration session supplying robot.joints (default: ./calibration_session.json)")
    source.add_argument("--random", action="store_true", help="generate Cartesian poses instead of replaying joints")
    auto.add_argument("--dry-run", action="store_true", help="list recorded joints and exit without HTTP or ROS connections")
    auto.add_argument("--joint-speed", type=float, default=0.2,
                      help="replay peak joint speed [rad/s], greater than 0 and at most 0.5 (default: 0.2)")
    auto.add_argument("--count", type=int, help="limit replay to the first N samples; random mode defaults to 20")
    auto.add_argument("--reach", type=float, default=0.08, help="max offset per axis [m]")
    auto.add_argument("--min-tilt", type=float, default=8.0, help="min tilt [deg]")
    auto.add_argument("--tilt", type=float, default=25.0, help="max tilt [deg]")
    auto.add_argument("--pivot", type=float, default=0.0,
                      help="tilt about a point this far along --pivot-axis from the flange [m]; about the "
                           "camera-to-board distance for a camera on the hand, 0 = about the flange")
    auto.add_argument("--pivot-axis", default="z", choices=["x", "y", "z", "-x", "-y", "-z"],
                      help="flange axis pointing from the hand towards the board")
    auto.add_argument("--seed", type=int, default=1)
    auto.add_argument("--yes", action="store_true", help="do not ask before moving")
    solve = sub.add_parser("solve", help="solve on the Jetson and print the result")
    solve.add_argument("--save", action="store_true", help="also write extrinsics.yaml on the Jetson")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    if args.mode == "auto":
        if args.count is not None and args.count <= 0:
            parser.error("--count must be positive")
        if not math.isfinite(args.duration) or args.duration < 0:
            parser.error("--duration must be finite and nonnegative")
        if args.random:
            args.count = 20 if args.count is None else args.count
            if args.dry_run:
                parser.error("--dry-run is for recorded session replay; omit --random")
        else:
            if not math.isfinite(args.joint_speed) or not 0 < args.joint_speed <= 0.5:
                parser.error("--joint-speed must be greater than 0 and at most 0.5 rad/s")
            try:
                args.waypoints = load_joint_waypoints(args.session)
            except ValueError as exc:
                parser.error(str(exc))
            if args.count is not None:
                args.waypoints = args.waypoints[:args.count]
            if args.dry_run:
                print(f"{len(args.waypoints)} joint targets from {args.session.resolve()}; no connections or motion.")
                for waypoint in args.waypoints:
                    print(f"Sample {waypoint['sample_id']}: " + ", ".join(
                        f"{name}={value:.9f}" for name, value in zip(ARM_JOINTS, waypoint["positions"])))
                return 0

    if args.mode == "solve":
        return 0 if print_solution(args.jetson, args.save) else 1

    ok, status = jetson(args.jetson, "/api/calib/status")
    if not ok:
        print(status)
        return 1
    board = status["board"]
    print(f"Jetson: {len(status['cameras'])} camera(s), {len(status['samples'])} sample(s) so far, board "
          f"{board['squares_x']}x{board['squares_y']} with {board['square_mm']:g} mm squares.")

    rclpy.init()
    node = Sampler(args)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        time.sleep(1.0)  # let TF and /joint_states arrive
        return run_manual(node) if args.mode == "manual" else run_auto(node)
    finally:
        executor.shutdown()
        spin_thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
