"""Run with the robot workspace sourced; tests never connect to or move a robot."""

import contextlib
import importlib.util
import io
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
from tempfile import TemporaryDirectory
from unittest.mock import Mock, patch


path = Path(__file__).resolve().parents[1] / "scripts" / "calibrate_cameras.py"
spec = importlib.util.spec_from_file_location("calibrate_cameras", path)
calib = importlib.util.module_from_spec(spec)
spec.loader.exec_module(calib)


class CalibrationMotionTests(unittest.TestCase):
    def node(self, measured_position=(0.0, 0.0, 0.0), measured_orientation=(0.0, 0.0, 0.0, 1.0)):
        node = SimpleNamespace(
            args=SimpleNamespace(count=3, reach=0.08, min_tilt=8.0, tilt=25.0,
                                 pivot=0.4, pivot_axis="z", seed=1, yes=True, parent="world", random=True),
            move_client=SimpleNamespace(wait_for_server=Mock(return_value=True)),
            wait_still=Mock(return_value=True),
            flange_pose=Mock(return_value=(measured_position, measured_orientation)),
            move_to=Mock(return_value=(True, "controller reported success")),
            record=Mock(return_value=1),
        )
        node.verify_target = lambda p, q: calib.Sampler.verify_target(node, p, q)
        return node

    def test_stationary_robot_with_successful_action_aborts_before_recording(self):
        node = self.node()
        with patch.object(calib, "print_solution") as solve, contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(calib.run_auto(node), 1)
        self.assertEqual(node.move_to.call_count, 1)
        node.record.assert_not_called()
        solve.assert_not_called()

    def test_measured_motion_allows_samples_and_return(self):
        node = self.node()

        def move(position, orientation):
            node.flange_pose.return_value = (position, orientation)
            return True, "finished"

        node.move_to.side_effect = move
        node.args.jetson = "http://unused"
        with patch.object(calib, "print_solution") as solve, contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(calib.run_auto(node), 0)
        self.assertEqual(node.record.call_count, 3)
        self.assertEqual(node.move_to.call_count, 4)
        solve.assert_called_once()

    def test_orientation_only_failure_is_rejected(self):
        node = self.node()
        target = calib.quat_from_axis_angle((1, 0, 0), math.radians(20))
        with self.assertRaises(calib.MotionVerificationError):
            node.verify_target((0, 0, 0), target)

    def test_quaternion_sign_does_not_cause_false_failure(self):
        node = self.node(measured_orientation=(0, 0, 0, -1))
        with contextlib.redirect_stdout(io.StringIO()):
            node.verify_target((0, 0, 0), (0, 0, 0, 1))

    def test_missing_fresh_joint_feedback_is_rejected(self):
        node = self.node()
        node.wait_still.return_value = False
        with self.assertRaises(calib.MotionVerificationError):
            node.verify_target((0, 0, 0), (0, 0, 0, 1))

    def test_stale_joint_messages_are_not_treated_as_rest(self):
        clock = SimpleNamespace(now=0.0)
        node = SimpleNamespace(args=SimpleNamespace(settle=1.0), arm_snapshot=Mock(
            return_value=({j: 0.0 for j in calib.ARM_JOINTS}, 1, 0.0)))

        def sleep(seconds):
            clock.now += seconds

        with patch.object(calib.time, "monotonic", side_effect=lambda: clock.now), \
                patch.object(calib.time, "sleep", side_effect=sleep):
            self.assertFalse(calib.Sampler.wait_still(node, timeout=1.0))

    def test_http_timeout_is_reported_instead_of_raising(self):
        with patch.object(calib.urllib.request, "urlopen", side_effect=TimeoutError):
            ok, message = calib.jetson("http://unused", "/api/calib/solve", {}, timeout=120)
        self.assertFalse(ok)
        self.assertIn("timed out", message)


class RecordedJointTests(unittest.TestCase):
    def session(self):
        return {"samples": [
            {"id": 19, "robot": {"joints": {j: (i + 1) / 10 for i, j in reversed(list(enumerate(calib.ARM_JOINTS)))}}},
            {"id": 12, "robot": {"joints": {j: -(i + 1) / 10 for i, j in enumerate(calib.ARM_JOINTS)}}},
        ]}

    def write_session(self, directory, data=None):
        path = Path(directory) / "session.json"
        path.write_text(json.dumps(self.session() if data is None else data))
        return path

    def replay_node(self):
        node = CalibrationMotionTests().node()
        node.args.random = False
        node.args.session = Path("recorded.json")
        node.args.jetson = "http://unused"
        node.args.waypoints = [dict(sample_id=19, positions=[0.1] * 6),
                               dict(sample_id=12, positions=[-0.2] * 6)]
        node.joint_server_ready = Mock(return_value=True)
        node.arm_positions = Mock(return_value={j: 0.0 for j in calib.ARM_JOINTS})
        node.move_to_joints = Mock(return_value=(True, "recorded target reached"))
        return node

    def test_file_order_and_exact_named_joint_values_are_preserved(self):
        with TemporaryDirectory() as directory:
            path = self.write_session(directory)
            before = path.read_bytes()
            waypoints = calib.load_joint_waypoints(path)
            self.assertEqual(path.read_bytes(), before)
        self.assertEqual([w["sample_id"] for w in waypoints], [19, 12])
        self.assertEqual(waypoints[0]["positions"], [.1, .2, .3, .4, .5, .6])
        self.assertEqual(waypoints[1]["positions"], [-.1, -.2, -.3, -.4, -.5, -.6])

    def test_incomplete_or_nonfinite_session_is_rejected_before_motion(self):
        for value in [None, True, "0.1", float("nan"), float("inf")]:
            with self.subTest(value=value), TemporaryDirectory() as directory:
                data = self.session()
                data["samples"][1]["robot"]["joints"][calib.ARM_JOINTS[0]] = value
                with self.assertRaisesRegex(ValueError, "Sample 12"):
                    calib.load_joint_waypoints(self.write_session(directory, data))
        with TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "non-empty"):
                calib.load_joint_waypoints(self.write_session(directory, {"samples": []}))

    def test_replay_visits_saved_joints_in_order_then_returns_to_start(self):
        node = self.replay_node()
        with patch.object(calib, "print_solution", return_value=True), contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(calib.run_auto(node), 0)
        self.assertEqual([c.args[0] for c in node.move_to_joints.call_args_list], [[.1] * 6, [-.2] * 6, [0.] * 6])
        self.assertEqual(node.record.call_count, 2)
        node.move_to.assert_not_called()

    def test_replay_stops_without_record_or_return_if_measured_joints_do_not_match(self):
        node = self.replay_node()
        node.move_to_joints.side_effect = calib.MotionVerificationError("robot did not move")
        with patch.object(calib, "print_solution") as solve, contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(calib.run_auto(node), 1)
        self.assertEqual(node.move_to_joints.call_count, 1)
        node.record.assert_not_called()
        solve.assert_not_called()

    def test_rejected_joint_path_is_not_recorded(self):
        node = self.replay_node()
        node.move_to_joints.side_effect = [(False, "collision"), (True, "reached"), (True, "returned")]
        with patch.object(calib, "print_solution", return_value=True), contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(calib.run_auto(node), 0)
        self.assertEqual(node.record.call_count, 1)

    def test_ctrl_c_does_not_request_a_return_motion(self):
        node = self.replay_node()
        node.move_to_joints.side_effect = KeyboardInterrupt
        with patch.object(calib, "print_solution") as solve, contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(calib.run_auto(node), 130)
        self.assertEqual(node.move_to_joints.call_count, 1)
        node.record.assert_not_called()
        solve.assert_not_called()

    def test_replay_requires_the_safety_filter(self):
        for names, expected in [([("safety_filter", "/")], True),
                                ([("arm_trajectory_controller", "/")], False),
                                ([("safety_filter", "/"), ("arm_trajectory_controller", "/")], False)]:
            node = SimpleNamespace(joint_client=SimpleNamespace(wait_for_server=Mock(return_value=True)),
                                   get_node_names_and_namespaces=Mock(return_value=names))
            with patch.object(calib, "get_action_server_names_and_types_by_node", return_value=[(calib.JOINT_ACTION, [])]):
                self.assertEqual(calib.Sampler.joint_server_ready(node), expected)

    def test_quintic_trajectory_obeys_peak_speed_and_stops_at_endpoints(self):
        target = [0., -.5, 1., 0., 0., 0.]
        trajectory = calib.joint_trajectory([0.] * 6, target, speed=.2)
        end = trajectory.points[-1]
        seconds = end.time_from_start.sec + end.time_from_start.nanosec / 1e9
        self.assertLessEqual(1.875 * max(abs(v) for v in target) / seconds, .2 + 1e-9)
        self.assertEqual(trajectory.joint_names, calib.ARM_JOINTS)
        self.assertEqual(list(end.positions), target)
        for point in trajectory.points:
            self.assertEqual(list(point.velocities), [0.] * 6)
            self.assertEqual(list(point.accelerations), [0.] * 6)

    def test_successful_controller_result_still_requires_measured_joint_target(self):
        node = self.replay_node()
        node.args.duration = 0.
        node.args.joint_speed = .2
        node.verify_joints = lambda target: calib.Sampler.verify_joints(node, target)
        handle = Mock(accepted=True)
        node.joint_client = SimpleNamespace(send_goal_async=Mock())
        result = SimpleNamespace(status=calib.GoalStatus.STATUS_SUCCEEDED,
                                 result=SimpleNamespace(error_code=0, error_string=""))
        with patch.object(calib, "wait", side_effect=[handle, result]):
            with self.assertRaises(calib.MotionVerificationError):
                calib.Sampler.move_to_joints(node, [.1] * 6)
        goal = node.joint_client.send_goal_async.call_args.args[0]
        self.assertEqual([v.position for v in goal.goal_tolerance], [.01] * 6)
        self.assertEqual([v.position for v in goal.path_tolerance], [.1] * 6)

    def test_timeout_cancels_the_joint_action(self):
        node = self.replay_node()
        node.args.duration = 0.
        node.args.joint_speed = .2
        node.joint_client = SimpleNamespace(send_goal_async=Mock())
        handle = Mock(accepted=True)
        with patch.object(calib, "wait", side_effect=[handle, None, None]):
            with self.assertRaisesRegex(calib.MotionVerificationError, "timed out"):
                calib.Sampler.move_to_joints(node, [.1] * 6)
        handle.cancel_goal_async.assert_called_once()

    def test_dry_run_limits_count_and_never_connects(self):
        with TemporaryDirectory() as directory:
            path = self.write_session(directory)
            args = ["calibrate_cameras.py", "--jetson", "http://unused", "auto", "--session", str(path),
                    "--count", "1", "--dry-run"]
            output = io.StringIO()
            with patch.object(sys, "argv", args), patch.object(calib.rclpy, "init") as init, \
                    patch.object(calib, "jetson") as http, contextlib.redirect_stdout(output):
                self.assertEqual(calib.main(), 0)
            init.assert_not_called()
            http.assert_not_called()
            self.assertIn("Sample 19:", output.getvalue())
            self.assertNotIn("Sample 12:", output.getvalue())


if __name__ == "__main__":
    unittest.main()
