"""Opt-in tests against an isolated camera_approach launch; moves simulated arm.

LARA5_LIVE_TEST=1 python3 -m pytest -q <this file>
"""

import os
from pathlib import Path
import runpy
import copy

import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Pose
from lara5_safety_msgs.msg import SceneBox
from lara5_safety_msgs.srv import CheckTrajectory, PlanCartesianPath, SetSceneBoxes

pytestmark = pytest.mark.skipif(os.environ.get("LARA5_LIVE_TEST") != "1",
                                reason="Requires an isolated simulation; moves simulated arm")


@pytest.fixture
def node():
    path = Path(get_package_prefix("lara5_sim_approach"))/"lib/lara5_sim_approach/approach_object.py"
    base = runpy.run_path(str(path))["Approach"]

    class DropoutApproach(base):
        drop_frames = False

        def on_frames(self, *frames):
            if not self.drop_frames:
                super().on_frames(*frames)

    rclpy.init(args=["--ros-args", "-p", "object_name:=blue_ball", "-p", "execute:=true",
                    "-p", "output_file:=/tmp/lara5_feedback_test.json"])
    instance = DropoutApproach()
    try:
        yield instance
    finally:
        instance.cancel()
        instance.destroy_node()
        instance.run_lock.close()
        rclpy.shutdown()


def test_blocked_goal_rejected(node):
    node.wait(lambda: bool(node.description and node.frames and node.joints is not None), 20, "not ready")
    node.assert_simulation()
    detections = node.observations()
    node.apply_scene(detections)
    point = detections[node.target].top.copy()
    point[2] += 0.45
    request = SetSceneBoxes.Request(frame_id="world")
    for detection in detections.values():
        box = SceneBox(name=f"demo_{detection.name}")
        box.center.x, box.center.y, box.center.z = map(float, detection.center)
        box.size.x, box.size.y, box.size.z = map(float, detection.dimensions)
        request.boxes.append(box)
    obstacle = SceneBox(name="integration_goal_blocker")
    obstacle.center.x, obstacle.center.y, obstacle.center.z = map(float, point)
    obstacle.size.x = obstacle.size.y = obstacle.size.z = 0.15
    request.boxes.append(obstacle)
    assert node.call(node.safety_scene, request).success
    try:
        with pytest.raises(RuntimeError, match="NEURA IK routing failed"):
            node.global_plan(point)
        assert node.active_goal is None
    finally:
        node.apply_scene(detections)


def test_native_cartesian_preview_does_not_move_arm(node):
    node.wait(lambda: bool(node.description and node.frames and node.joints is not None), 20, "not ready")
    node.apply_scene(node.observations())
    start = node.state()
    before = node.flange_pose(start).position
    request = PlanCartesianPath.Request(relative=True, start_state=node.arm_state(start))
    request.target.header.frame_id = "world"
    request.target.pose.orientation.w = 1.0
    request.target.pose.position.z = 0.01
    response = node.call(node.cart_client, request)
    assert response.success, response.message
    assert response.trajectory.points
    after = node.flange_pose(node.endpoint_state(response.trajectory)).position
    assert np.linalg.norm(np.array([after.x-before.x, after.y-before.y, after.z-before.z])
                          - [0, 0, 0.01]) < 0.004
    assert np.max(np.abs(np.array(node.arm_state(node.state()).position)
                         - node.arm_state(start).position)) < 0.005
    assert node.active_goal is None


def test_native_scene_update_is_atomic_and_replaceable(node):
    node.wait(lambda: bool(node.description and node.frames and node.joints is not None), 20, "not ready")
    detections = node.observations()
    node.apply_scene(detections)
    check = CheckTrajectory.Request(start_state=node.arm_state(node.state()))
    assert node.call(node.safety_check, check).valid
    flange = node.flange_pose(node.state()).position
    blocker = SceneBox(name="test_blocker", center=copy.deepcopy(flange))
    blocker.size.x = blocker.size.y = blocker.size.z = 0.5
    update = SetSceneBoxes.Request(frame_id="world", boxes=[blocker])
    try:
        assert node.call(node.safety_scene, update).success
        assert not node.call(node.safety_check, check).valid
        # Invalid input must leave the blocking scene intact.
        update.boxes[0].size.x = -1.0
        assert not node.call(node.safety_scene, update).success
        assert not node.call(node.safety_check, check).valid
    finally:
        node.apply_scene(detections)
    assert node.call(node.safety_check, check).valid
    check.start_state.name[0] = check.start_state.name[1]
    assert not node.call(node.safety_check, check).valid


def test_camera_dropout_cancels_active_trajectory(node):
    updates = []

    def drop_after_dispatch():
        if node.active_goal is not None and node.active_goal.accepted:
            if not updates:
                updates.append(node.safety_scene.call_async(SetSceneBoxes.Request(frame_id="world")))
            node.drop_frames = True

    timer = node.create_timer(0.1, drop_after_dispatch)
    try:
        with pytest.raises(RuntimeError, match="camera sample is stale"):
            node.run()
        node.cancel()
        assert node.active_result is not None and node.active_result.done()
        assert node.active_result.result().status == 5  # action_msgs STATUS_CANCELED
        assert updates and updates[0].done()
        assert not updates[0].result().success
        assert "active arm actions" in updates[0].result().message
    finally:
        node.destroy_timer(timer)
