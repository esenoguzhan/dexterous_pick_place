"""Unit tests of the geometry helpers, the median and the configuration files."""

from pathlib import Path

import numpy as np
import pytest

from restore_perception.core import geometry as g
from restore_perception.core.capture import median_depth
from restore_perception.core.config import DEFAULTS, load_config, load_objects
from restore_perception.core.describe import rectangle
from restore_perception.core.fuse import cells_iou, footprint_cells, polygon_cells
from restore_perception.core.gallery import size_fits
from restore_perception.core.types import SYM_CONTINUOUS, SYM_NONE, PerceptionError

CONFIG = Path(__file__).resolve().parents[1] / "config"


def test_median_ignores_zeros_and_needs_half_valid():
    frames = [np.array([[1000, 0, 0]], np.uint16), np.array([[1002, 5, 0]], np.uint16),
              np.array([[0, 0, 7]], np.uint16), np.array([[1004, 0, 0]], np.uint16)]
    m = median_depth(frames, 0.5)
    assert m[0, 0] == 1002.0  # median of 1000, 1002, 1004
    assert m[0, 1] == 0.0 and m[0, 2] == 0.0  # 1 of 4 valid


def test_plane_fit_with_clutter():
    rng = np.random.default_rng(1)
    xy = rng.uniform(-0.5, 0.5, (5000, 2))
    table = np.column_stack([xy, 0.9 + 0.02 * xy[:, 0] + rng.normal(0, 0.002, 5000)])
    clutter = rng.uniform([-0.5, -0.5, 0.9], [0.5, 0.5, 1.2], (1500, 3))
    plane, inliers = g.fit_plane(np.vstack([table, clutter]), 0.008, 500)
    expected = np.array([-0.02, 0.0, 1.0]) / np.linalg.norm([-0.02, 0.0, 1.0])
    assert np.allclose(plane[:3], expected, atol=2e-3)
    assert abs(g.plane_height_at(plane, 0.0, 0.0) - 0.9) < 1e-3
    assert inliers[:5000].mean() > 0.95


def test_plane_needs_points():
    with pytest.raises(PerceptionError):
        g.fit_plane(np.zeros((2, 3)), 0.01, 10)


def test_dbscan_separates_blobs_and_marks_noise():
    rng = np.random.default_rng(2)
    a = rng.normal([0, 0, 0], 0.005, (300, 3))
    b = rng.normal([0.1, 0, 0], 0.005, (300, 3))
    noise = np.array([[0.05, 0.3, 0.0], [0.3, 0.3, 0.3]])
    labels = g.dbscan(np.vstack([a, b, noise]), 0.01, 10)
    assert len(set(labels[:300])) == 1 and len(set(labels[300:600])) == 1
    assert labels[0] != labels[300]
    assert (labels[600:] == -1).all()


def test_voxel_downsample_centroids():
    pts = np.array([[0.001, 0.001, 0.001], [0.003, 0.003, 0.003], [0.011, 0.0, 0.0]])
    centroids, ids = g.voxel_downsample(pts, 0.005)
    assert len(centroids) == 2 and ids[0] == ids[1] != ids[2]
    assert np.allclose(centroids[ids[0]], [0.002, 0.002, 0.002])


def test_table_frame_and_rays():
    plane = np.array([0.0, 0.0, 1.0, -0.9])
    table = g.TableFrame(plane, (0.1, 0.2))
    assert np.allclose(table.to_table([[0.1, 0.2, 0.95]]), [[0, 0, 0.05]])
    q = table.orientation(np.radians(30))
    from scipy.spatial.transform import Rotation
    assert abs(Rotation.from_quat(q).as_euler("zyx", degrees=True)[0] - 30) < 1e-6
    T = np.eye(4)
    T[:3, :3] = np.diag([1.0, -1.0, -1.0])  # looking straight down
    T[:3, 3] = [0.0, 0.0, 1.5]
    K = np.array([[500.0, 0, 320], [0, 500.0, 240], [0, 0, 1]])
    hit = g.ray_plane([[320, 240], [420, 240]], K, T, plane)
    assert np.allclose(hit, [[0, 0, 0.9], [0.12, 0, 0.9]])


def test_rectangle_yaw_of_long_side():
    rng = np.random.default_rng(3)
    local = rng.uniform([-0.05, -0.02], [0.05, 0.02], (2000, 2))
    yaw = np.radians(70)
    R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    centre, (length, width), est = rectangle(local @ R.T + [0.3, -0.1])
    assert np.allclose(centre, [0.3, -0.1], atol=1e-3)
    assert abs(length - 0.1) < 2e-3 and abs(width - 0.04) < 2e-3
    assert abs(np.degrees(est) - 70) < 1.0


def test_footprint_cells():
    a = footprint_cells(np.array([[0.0, 0.0], [0.004, 0.004]]), 0.005)
    assert cells_iou(a, a) == 1.0
    assert cells_iou(a, footprint_cells(np.array([[1.0, 1.0]]), 0.005)) == 0.0
    square = polygon_cells(np.array([[0, 0], [0.05, 0], [0.05, 0.05], [0, 0.05]]), 0.005)
    assert 100 <= len(square) <= 200


def test_size_check_any_axis_order():
    assert size_fits([0.06, 0.09, 0.045], [0.09, 0.06, 0.045], 0.3)
    assert not size_fits([0.14, 0.09, 0.09], [0.055, 0.055, 0.055], 0.3)


def test_config_files_load_and_typos_fail():
    config = load_config(CONFIG / "perception.yaml")
    assert config["cluster"]["eps_m"] == DEFAULTS["cluster"]["eps_m"]
    with pytest.raises(ValueError, match="cluster.epsilon"):
        load_config(overrides={"cluster": {"epsilon": 0.01}})
    assert load_config(overrides={"observe": {"joints": {"lara5_joint1": 0.5}}})["observe"]["joints"]


def test_objects_yaml():
    objects = load_objects(CONFIG / "objects.yaml")
    assert len(objects) == 8
    assert objects["tuna_can"]["symmetry"] == SYM_CONTINUOUS
    assert objects["mouse"]["symmetry"] == SYM_NONE
    assert objects["wooden_cube"]["symmetry"] == 4
    assert objects["steel_mug"]["depth_reliable"] is False
    assert objects["sponge"]["up_side_matters"] is True
