#!/usr/bin/python3

"""Robot side of the camera calibration (docs/perception_architecture.md, section 3.4).

The cameras hang on the Jetson, which detects the ChArUco board (on the table, or fixed to the hand)
and solves. This script runs next to the robot stack: at every calibration pose it waits for the arm
to stand still, reads the flange pose from TF and asks the Jetson to record the board in each camera
at that pose. Set the board location and the camera mounts, watch the detection and solve on the
Jetson's page http://<jetson>:8090/calibration.

    manual  move the arm yourself (for example with move_cartesian.py), press Enter to record
    auto    visit --count poses around the current pose through the safety filter's MoveToPose,
            record each one and return to the start pose
    solve   solve on the Jetson and print the result; add --save to write extrinsics.yaml there

    # board on the table, camera on the hand looking at it from about 0.4 m
    ros2 run lara5_controller_manager calibrate_cameras.py --jetson http://192.168.2.101:8090 auto --pivot 0.4
    ros2 run lara5_controller_manager calibrate_cameras.py --jetson http://192.168.2.101:8090 manual
    ros2 run lara5_controller_manager calibrate_cameras.py --jetson http://192.168.2.101:8090 solve --save

--pivot tilts the flange about a point that far along its z axis instead of about the flange itself,
so a camera on the hand keeps looking at the same spot. The safety filter knows neither the board nor
a camera on the hand: start auto with plenty of room around the hand and keep the e-stop in reach.
"""

import argparse
import json
import math
import random
import sys
import threading
import time
import urllib.error
import urllib.request

import rclpy
from geometry_msgs.msg import PoseStamped
from lara5_safety_msgs.action import MoveToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

ARM_JOINTS = [f"lara5_joint{i}" for i in range(1, 7)]


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


# ---------- Jetson HTTP ----------

def jetson(base, path, body=None, method=None):
    """JSON request to the Jetson's perception app. Returns (ok, body)."""
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(base.rstrip("/") + path, data=data, method=method or ("POST" if data else "GET"),
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=30) as res:
            return True, json.loads(res.read() or b"{}")
    except urllib.error.HTTPError as err:
        try:
            return False, json.loads(err.read()).get("detail")
        except ValueError:
            return False, f"HTTP {err.code}"
    except urllib.error.URLError as err:
        return False, f"Jetson not reachable at {base}: {err.reason}"


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
        self.lock = threading.Lock()
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.move_client = ActionClient(self, MoveToPose, "/safety/arm/move_to_pose")

    def on_joints(self, msg):
        with self.lock:
            self.positions.update(zip(msg.name, msg.position))

    def arm_positions(self):
        with self.lock:
            return {j: self.positions[j] for j in ARM_JOINTS if j in self.positions}

    def wait_still(self, timeout=15.0):
        """Wait until no arm joint moves more than 0.0005 rad in 0.3 s, then settle."""
        deadline = time.monotonic() + timeout
        before = self.arm_positions()
        while time.monotonic() < deadline:
            time.sleep(0.3)
            now = self.arm_positions()
            if len(now) == len(ARM_JOINTS) and len(before) == len(ARM_JOINTS) and \
                    max(abs(now[j] - before[j]) for j in ARM_JOINTS) < 0.0005:
                time.sleep(self.args.settle)
                return True
            before = now
        return False

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
            return False, "no result within 120 s"
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


def run_auto(node):
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
            if node.record() is not None:
                recorded += 1
            else:
                failed += 1
    except KeyboardInterrupt:
        print("\nStopped. Returning to the start pose.")
    ok, message = node.move_to(start_p, start_q)
    print("Back at the start pose." if ok else f"Could not return to the start pose: {message}")
    print(f"Recorded {recorded} samples, {failed} poses skipped.")
    if recorded:
        print_solution(args.jetson, save=False)
    return 0


def print_solution(base, save):
    ok, result = jetson(base, "/api/calib/solve", {})
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
    auto = sub.add_parser("auto", help="visit poses around the current pose")
    auto.add_argument("--count", type=int, default=20)
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
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        time.sleep(1.0)  # let TF and /joint_states arrive
        return run_manual(node) if args.mode == "manual" else run_auto(node)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
