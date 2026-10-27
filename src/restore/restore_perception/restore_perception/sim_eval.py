"""Measure the pose estimation against MuJoCo's ground truth (docs/perception_simulation_eval.md).

    ros2 run restore_perception sim_eval [--captures 5] [--out /tmp/perception_sim/eval] [--radius 0.08]

Needs the simulation with its cameras (ground truth on /sim/objects) and scene_perception_node from
perception_sim.launch.py. Each capture calls /perception/capture_scene, matches the returned objects
to the true ones by horizontal distance and records position, yaw and size errors. Writes report.md,
results.csv, truth.json and the overlay image of each camera per capture.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from restore_interfaces.srv import CaptureScene

from .core import sim_truth

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
CAMERAS = ("camera_1", "camera_2")
ARM_JOINTS = [f"lara5_joint{i}" for i in range(1, 7)]
ARM_ACTION = "/arm_trajectory_controller/follow_joint_trajectory"  # checked by the safety filter


def estimate_from_msg(obj):
    q = obj.pose.orientation
    return {
        "id": obj.id, "label": obj.label,
        "position": [obj.pose.position.x, obj.pose.position.y, obj.pose.position.z],
        "yaw": sim_truth.yaw_from_quaternion(q.x, q.y, q.z, q.w),
        "size": [obj.size.x, obj.size.y, obj.size.z],
        "confidence": float(obj.confidence), "cameras_seen": int(obj.cameras_seen),
        "depth_reliable": bool(obj.depth_reliable),
    }


class SimEval(Node):
    def __init__(self, args):
        super().__init__("perception_sim_eval")
        self.args = args
        self.truth = None
        self.overlays = {}
        self.joints = {}
        self.create_subscription(String, args.objects_topic, self._on_truth, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        for camera in CAMERAS:
            self.create_subscription(Image, f"/perception/debug/{camera}/overlay",
                                     lambda m, c=camera: self.overlays.__setitem__(c, m), LATCHED)
        self.client = self.create_client(CaptureScene, "/perception/capture_scene")

    def _on_truth(self, msg):
        self.truth = json.loads(msg.data)

    def _on_joints(self, msg):
        self.joints.update(zip(msg.name, msg.position))

    def move_arm(self, positions, seconds):
        """Send the arm to a joint configuration through the safety filter and wait until it is there."""
        client = ActionClient(self, FollowJointTrajectory, ARM_ACTION)
        if not client.wait_for_server(timeout_sec=self.args.timeout):
            raise TimeoutError(f"{ARM_ACTION} is not available (is the controller launch running with safety:=on?)")
        # The safety filter rejects goals until the joint state broadcaster runs, which starts a
        # few seconds after the simulator's topics appear.
        self.wait_until(lambda: all(n in self.joints for n in ARM_JOINTS), self.args.timeout, "arm /joint_states")
        time.sleep(1.5)
        point = JointTrajectoryPoint(positions=[float(p) for p in positions],
                                     time_from_start=Duration(seconds=seconds).to_msg())
        goal = FollowJointTrajectory.Goal(trajectory=JointTrajectory(joint_names=ARM_JOINTS, points=[point]))
        handle_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, handle_future, timeout_sec=self.args.timeout)
        handle = handle_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("the observe move was rejected (safety filter or controller)")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.args.timeout + seconds)
        if not result_future.done():
            raise TimeoutError("the observe move did not finish")
        code = result_future.result().result.error_code
        if code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f"the observe move failed with error code {code}: {result_future.result().result.error_string}")
        self.wait_until(lambda: all(abs(self.joints.get(n, 1e9) - p) < 0.02 for n, p in zip(ARM_JOINTS, positions)),
                        10.0, "the arm to settle at the observe pose")
        time.sleep(0.5)

    def wait_until(self, predicate, timeout, what):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return
        raise TimeoutError(f"timed out after {timeout:.0f} s waiting for {what}")

    def capture(self, name):
        future = self.client.call_async(CaptureScene.Request(name=name, save_to_disk=False))
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.args.timeout)
        if not future.done():
            raise TimeoutError(f"capture {name} did not return within {self.args.timeout:.0f} s")
        return future.result()

    def save_overlays(self, out, index):
        end = time.monotonic() + 1.0  # the latched overlays arrive right after the response
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        try:
            import cv2
            import numpy as np
        except ImportError:
            return
        for camera, msg in self.overlays.items():
            image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            cv2.imwrite(str(out / f"capture_{index:02d}_{camera}.png"), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        self.overlays.clear()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--captures", type=int, default=5)
    parser.add_argument("--out", default="/tmp/perception_sim/eval")
    parser.add_argument("--radius", type=float, default=0.08, help="match radius in the table plane [m]")
    parser.add_argument("--timeout", type=float, default=60.0, help="per capture and for the services [s]")
    parser.add_argument("--objects-topic", default="/sim/objects")
    parser.add_argument("--pause", type=float, default=0.5, help="seconds between captures")
    parser.add_argument("--observe-joints", type=float, nargs=6, metavar="RAD", default=None,
                        help="move the arm to these joint angles first, e.g. 0 0 0 0 0 0 (arm straight up, "
                             "out of the fixed camera's view and above perception's workspace box)")
    parser.add_argument("--observe-duration", type=float, default=6.0, help="seconds for the observe move")
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init(args=None)
    node = SimEval(args)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    try:
        node.wait_until(lambda: node.truth is not None, args.timeout,
                        f"ground truth on {args.objects_topic} (is the simulation running with objects?)")
        node.wait_until(lambda: node.client.service_is_ready(), args.timeout,
                        "/perception/capture_scene (is perception_sim.launch.py running?)")
        if args.observe_joints is not None:
            node.get_logger().info(f"moving the arm to the observe pose {args.observe_joints}")
            node.move_arm(args.observe_joints, args.observe_duration)
        truth_names = [o["name"] for o in node.truth["objects"]]
        (out / "truth.json").write_text(json.dumps(node.truth, indent=1))
        node.get_logger().info(f"{len(truth_names)} true objects: {', '.join(truth_names)}")
        captures, cameras_seen = [], set()
        for i in range(args.captures):
            truth = node.truth["objects"]
            response = node.capture(f"sim_eval_{i:02d}")
            if not response.success:
                node.get_logger().error(f"capture {i}: {response.message}")
                captures.append({"rows": [], "missed": truth_names, "extra": [], "message": response.message})
                continue
            estimates = [estimate_from_msg(o) for o in response.snapshot.objects]
            cameras_seen.update(e["cameras_seen"] for e in estimates)
            scored = sim_truth.score_capture(truth, estimates, args.radius)
            scored["message"] = response.message
            captures.append(scored)
            node.save_overlays(out, i)
            for r in scored["rows"]:
                yaw = "-" if r["yaw_error_deg"] is None else f"{r['yaw_error_deg']:.1f} deg"
                node.get_logger().info(f"capture {i} {r['name']:15s} <- {r['estimate_id']}: xy {r['dxy_mm']:5.1f} mm, "
                                       f"z {r['dz_mm']:+5.1f} mm, yaw {yaw}, size "
                                       f"{'/'.join(f'{v:+.0f}' for v in r['size_error_mm'])} mm")
            if scored["missed"]:
                node.get_logger().warning(f"capture {i} missed: {', '.join(scored['missed'])}")
            for e in scored["extra"]:
                node.get_logger().warning(f"capture {i} unmatched estimate {e['id']} at x {e['position'][0]:+.3f} "
                                          f"y {e['position'][1]:+.3f} z {e['position'][2]:.3f} m, size "
                                          f"{'/'.join(f'{v:.0f}' for v in e['size_mm'])} mm")
            time.sleep(args.pause)
        summary = sim_truth.summarize(captures, truth_names)
        meta = {"radius": args.radius, "cameras": sorted(cameras_seen) or "none", "models": "off"}
        (out / "report.md").write_text(sim_truth.markdown_report(summary, captures, meta))
        with (out / "results.csv").open("w", newline="") as f:
            fields = ["capture", "name", "estimate_id", "label", "dx_mm", "dy_mm", "dz_mm", "dxy_mm", "d3_mm",
                      "yaw_error_deg", "size_error_mm", "size_estimate_mm", "size_truth_mm", "confidence",
                      "cameras_seen", "depth_reliable"]
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for n, c in enumerate(captures):
                for r in c["rows"]:
                    writer.writerow({"capture": n, **{k: r[k] for k in fields if k != "capture"}})
        o = summary["overall"]
        node.get_logger().info(
            f"done: {o['detected']}/{o['expected']} detections, {o['extra']} false positives, XY mean "
            f"{o['xy_mean_mm'] if o['xy_mean_mm'] is None else round(o['xy_mean_mm'], 1)} mm, yaw mean "
            f"{o['yaw_mean_deg'] if o['yaw_mean_deg'] is None else round(o['yaw_mean_deg'], 1)} deg -> {out / 'report.md'}")
        return 0
    except (TimeoutError, RuntimeError) as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
