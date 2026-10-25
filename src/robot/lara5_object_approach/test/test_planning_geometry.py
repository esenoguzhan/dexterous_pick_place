"""Validate real-model FK, pregrasp semantics and bounded cubic trajectories."""

import math
from pathlib import Path

import mujoco
import numpy as np
import pytest
import xacro
from ament_index_python.packages import get_package_share_directory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from lara5_mujoco_sim.mjcf_builder import build_scene
from lara5_object_approach.grasp_geometry import from_rpy
from lara5_object_approach.planning_geometry import ArmModel, duration, path_poses, pregrasp_target, retime, seconds


@pytest.fixture
def model():
    share = Path(get_package_share_directory("lara5_inspire_hand_description"))
    urdf = xacro.process_file(str(share/"urdf/lara5_inspire_hand.urdf.xacro")).toxml()
    names = [f"lara5_joint{i}" for i in range(1, 7)]
    return ArmModel(urdf, "world", "lara5_flange", names), urdf


@pytest.mark.parametrize("q", [[0, 0, 1.5708, 0, 1.5708, 0], [.3, -.2, 1.1, .2, 1.3, -.4],
                              [-.5, .5, -.6, .1, .7, 1.]])
def test_local_fk_agrees_with_actual_mujoco_model(model, q):
    arm, urdf = model
    native = mujoco.MjModel.from_xml_string(build_scene(urdf, {}).mjcf)
    data = mujoco.MjData(native)
    for name, value in zip(arm.names, q):
        data.qpos[native.jnt_qposadr[native.joint(name).id]] = value
    mujoco.mj_forward(native, data)
    flange = native.body("lara5_flange").id
    fk = arm.fk(q)
    # MuJoCo's intermediate URDF->MJCF serialization rounds some transforms.
    # One micrometre is still 1000x tighter than the preview endpoint tolerance.
    np.testing.assert_allclose(fk[:3, 3], data.xpos[flange], atol=1e-6, rtol=0)
    np.testing.assert_allclose(fk[:3, :3], data.xmat[flange].reshape(3, 3), atol=1e-6, rtol=0)


@pytest.mark.parametrize("yaw", [0, math.pi/2, math.pi, -math.pi/2])
def test_30cm_is_flange_height_not_final_descent_distance(yaw):
    obj = from_rpy([.25, -.2, .95], [0, 0, yaw])
    final = obj @ from_rpy([.025, -.15, .085], [-math.pi/2, math.pi/2, 0])
    grasp = obj @ from_rpy([0, 0, 0], [math.pi, 0, 0])
    pre = pregrasp_target(obj, final, grasp, .3)
    assert pre[2, 3] == pytest.approx(1.25)
    assert pre[2, 3]-final[2, 3] == pytest.approx(.215)
    np.testing.assert_allclose(pre[:2, 3], final[:2, 3], atol=1e-12)
    np.testing.assert_allclose(pre[:3, :3], final[:3, :3])


@pytest.mark.parametrize("clearance", [-.1, 0, math.nan, .01])
def test_invalid_pregrasp_clearance_rejected(clearance):
    with pytest.raises(ValueError):
        pregrasp_target(np.eye(4), from_rpy([0, 0, .085], [0, 0, 0]),
                        from_rpy([0, 0, 0], [math.pi, 0, 0]), clearance)


def raw_path(names):
    return JointTrajectory(joint_names=list(reversed(names)), points=[
        JointTrajectoryPoint(positions=[.05]*6, time_from_start=duration(.1)),
        JointTrajectoryPoint(positions=[.10]*6, time_from_start=duration(.2))])


def test_retiming_preserves_start_limits_and_cubic_peak_bounds(model):
    arm, _ = model
    path = retime(raw_path(arm.names), np.zeros(6), arm, .2, .25, .5)
    assert path.joint_names == arm.names
    np.testing.assert_allclose(path.points[0].positions, np.zeros(6))
    assert seconds(path.points[0].time_from_start) == 0
    for left, right in zip(path.points, path.points[1:]):
        dt = seconds(right.time_from_start)-seconds(left.time_from_start)
        delta = np.abs(np.array(right.positions)-left.positions)
        assert np.max(1.5*delta/dt) <= .25+1e-8
        assert np.max(6*delta/dt**2) <= .5+1e-8
        assert list(left.velocities) == list(right.velocities) == [0.]*6


@pytest.mark.parametrize("fault", ["duplicate_joint", "short_positions", "nan", "time", "jump", "limit", "velocity"])
def test_incomplete_or_discontinuous_native_paths_rejected(model, fault):
    arm, _ = model
    path = raw_path(arm.names)
    if fault == "duplicate_joint": path.joint_names[0] = path.joint_names[1]
    if fault == "short_positions": path.points[0].positions = [0.]
    if fault == "nan": path.points[0].positions[0] = math.nan
    if fault == "time": path.points[0].time_from_start = duration(0.)
    if fault == "jump": path.points[0].positions = [.4]*6
    if fault == "limit": path.points[0].positions = [20.]*6
    if fault == "velocity": path.points[0].velocities = [math.inf]*6
    with pytest.raises(ValueError):
        retime(path, np.zeros(6), arm, .2, .25, .5)


def test_already_at_target_gets_stationary_phase_subject_to_later_fk_check(model):
    arm, _ = model
    path = retime(JointTrajectory(joint_names=arm.names), np.zeros(6), arm, .2, .25, .5)
    assert len(path.points) == 2
    assert path.points[0].positions == path.points[1].positions
    assert seconds(path.points[-1].time_from_start) == 1.


def test_cartesian_interpolation_deviation_is_rejected(model):
    arm, _ = model
    path = retime(raw_path(arm.names), np.zeros(6), arm, .2, .25, .5)
    with pytest.raises(ValueError, match="Cartesian segment"):
        path_poses(path, arm, arm.fk(np.zeros(6)), from_rpy([5, 5, 5], [0, 0, 0]))
