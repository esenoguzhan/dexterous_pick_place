"""Pick/place geometry and failure sequencing; no physics, rendering or ROS nodes."""

import math
import time
import unittest
from unittest.mock import Mock

import numpy as np
from geometry_msgs.msg import Pose
from lara5_safety_msgs.msg import SimGraspFeedback
from rclpy.time import Time
from rclpy.clock import ClockType
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from lara5_mujoco_sim.grasp_feedback import opposing_contacts
from lara5_sim_approach.approach import ARM, rotation_matrix
from lara5_sim_approach.grasping import (flange_for_object, grip_slipped, hand_offset_in_flange,
    palm_down_rotation, quaternion_xyzw, validate_destination)
from lara5_sim_approach.pick_place import PickPlace, make_pose
from lara5_sim_approach.routing import RouteRejected
from lara5_sim_approach.trajectory import duration
from lara5_sim_approach.vision import Detection


class GraspGeometryTests(unittest.TestCase):
    def test_grasp_frame_and_inverse_object_placement(self):
        hand_mount = np.array([[0., 1., 0], [-1., 0, 0], [0, 0, 1.]])
        offset = hand_offset_in_flange([-.025, .07, .15])
        center = np.array([-.35, .30, .925])
        for yaw in (0, math.pi/2, math.pi, -math.pi/2):
            rotation = palm_down_rotation(yaw)
            np.testing.assert_allclose((rotation @ hand_mount)[:, 1], [0, 0, -1], atol=1e-12)
            pose = make_pose(flange_for_object(center, rotation, offset), rotation)
            np.testing.assert_allclose(rotation_matrix(pose.orientation), rotation, atol=1e-12)
            np.testing.assert_allclose([pose.position.x, pose.position.y, pose.position.z]+rotation@offset,
                                       center, atol=1e-12)

    def test_placement_requires_table_support_and_free_space(self):
        valid = [.1, .2, .925]
        np.testing.assert_allclose(validate_destination(valid, [.05]*3, []), valid)
        for destination in ([.59, 0, .925], [.1, .2, .95], [float("nan"), 0, .925]):
            with self.assertRaises(ValueError):
                validate_destination(destination, [.05]*3, [])
        with self.assertRaisesRegex(ValueError, "green"):
            validate_destination(valid, [.05]*3, [("green", [.12, .2, .95], [.06, .06, .10])])

    def test_loaded_opposing_digits_are_distinct_from_same_side_contacts(self):
        links = ["right_thumb_force_sensor_3", "right_index_force_sensor_2"]
        self.assertFalse(opposing_contacts([links[0], "right_thumb_force_sensor_4"]))
        self.assertFalse(opposing_contacts(["right_palm_force_sensor", links[1]]))
        self.assertFalse(opposing_contacts(links, {links[0]: [np.array([1, 0, 0])],
                                                   links[1]: [np.array([1, 0, 0])]}))
        self.assertTrue(opposing_contacts(links, {links[0]: [np.array([1, 0, 0])],
                                                  links[1]: [np.array([-1, 0, 0])]}))

    def test_relative_slip_ignores_quaternion_sign(self):
        reference = (np.array([.05, .02, .15]), np.array([0., 0, 0, 1.]))
        self.assertFalse(grip_slipped(reference, (reference[0], -reference[1])))
        self.assertTrue(grip_slipped(reference, (reference[0]+[.03, 0, 0], reference[1])))
        rotated = quaternion_xyzw(palm_down_rotation(0))
        self.assertTrue(grip_slipped(reference, (reference[0], rotated)))


class PickPlaceSequencingTests(unittest.TestCase):
    def test_invalid_grasp_feedback_is_rejected(self):
        harness = Mock()
        harness.target = "red_cube"
        harness.p = lambda name: 1.0
        harness.get_clock.return_value.now.return_value = Time(seconds=123, clock_type=ClockType.ROS_TIME)
        sample = SimGraspFeedback(object_name="red_cube")
        sample.header.stamp = Time(seconds=123).to_msg()
        sample.header.frame_id = "world"
        sample.object_pose.orientation.w = sample.object_in_flange.orientation.w = 1.0
        harness.feedback = {"red_cube": (sample, time.monotonic())}
        self.assertIs(PickPlace.latest_feedback(harness), sample)
        sample.object_pose.orientation.w = 0.0
        with self.assertRaisesRegex(RuntimeError, "invalid pose"):
            PickPlace.latest_feedback(harness)
        sample.object_pose.orientation.w = 1.0
        sample.linear_speed = float("nan")
        with self.assertRaisesRegex(RuntimeError, "invalid frame or speed"):
            PickPlace.latest_feedback(harness)

    def harness(self, execute):
        # Call the real state machine on a plain object: Node/DDS never initialize.
        harness = Mock()
        harness.p = lambda name: {"execute": execute}[name]
        harness.target = "red_cube"
        harness.motion_started = False
        harness.context_dirty = True
        harness.report = {"mode": "planning", "phases": []}
        item = Detection("red_cube", np.array([-.35, .30, .925]), np.array([.08]*3),
                         np.array([-.35, .30, .95]), 100)
        harness.baseline = {"red_cube": item}
        harness.open_hand, harness.closed_hand = np.zeros(6), np.array([1, 1, 1, 1, 1, .5])
        harness.offset = hand_offset_in_flange([-.025, .07, .15])
        harness.grip_reference = None
        path = JointTrajectory(joint_names=ARM, points=[JointTrajectoryPoint(
            positions=[0.]*6, velocities=[0.]*6, time_from_start=duration(t)) for t in (0., 1.)])
        harness.preview.return_value = {"rotation": palm_down_rotation(0), "grasp": Pose(),
            "pregrasp": path, "approach": path, "carry": [("test_lift", path)],
            "retreat": path, "route": "direct"}
        harness.measured_hand.return_value = np.zeros(6)
        harness.line.return_value = path
        harness.wrist_refine.return_value = item.center
        return harness

    def test_plan_only_never_dispatches_either_controller_and_restores_context(self):
        harness = self.harness(False)
        PickPlace.run(harness)
        harness.execute_path.assert_not_called()
        harness.execute_hand.assert_not_called()
        harness.configure_scene.assert_called_once_with(None, harness.baseline)
        self.assertEqual(harness.report["mode"], "plan_only")

    def test_all_candidates_blocked_does_not_leave_a_fictitious_payload(self):
        harness = self.harness(True)
        harness.preview.side_effect = RouteRejected("blocked")
        with self.assertRaises(RouteRejected):
            PickPlace.run(harness)
        self.assertEqual(harness.preview.call_count, 4)
        harness.execute_path.assert_not_called()
        harness.execute_hand.assert_not_called()
        harness.configure_scene.assert_called_once_with(None, harness.baseline)

    def test_wrist_failure_stops_before_contact_approach_or_finger_closure(self):
        harness = self.harness(True)
        harness.wrist_refine.side_effect = RuntimeError("not visible")
        with self.assertRaisesRegex(RuntimeError, "not visible"):
            PickPlace.run(harness)
        self.assertEqual(harness.execute_path.call_count, 1)  # Only pre-grasp ran.
        harness.execute_hand.assert_not_called()
        harness.carry_paths.assert_not_called()

    def test_failed_grasp_never_sends_lift_or_transfer(self):
        harness = self.harness(True)
        harness.stable_grip.side_effect = RuntimeError("no opposing grip")
        with self.assertRaisesRegex(RuntimeError, "no opposing grip"):
            PickPlace.run(harness)
        self.assertEqual(harness.execute_path.call_count, 2)  # Pre-grasp + contact approach only.
        harness.execute_hand.assert_called_once()
        harness.carry_paths.assert_not_called()
        self.assertEqual(harness.report["mode"], "failed")


if __name__ == "__main__":
    unittest.main()
