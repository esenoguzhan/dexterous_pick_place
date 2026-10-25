"""Back-projection, table plane, clustering and table-frame helpers (sections 5.2 to 5.4).

NumPy and SciPy only. Open3D would cover the same steps, but its aarch64 wheel is a large extra
dependency on the Jetson for a handful of functions.
"""

from __future__ import annotations

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from .types import PerceptionError

UP = np.array([0.0, 0.0, 1.0])


def back_project(depth, K):
    """Section 5.2: H x W x 3 points in the camera's optical frame; depth 0 gives (0, 0, 0)."""
    h, w = depth.shape
    u = np.arange(w, dtype=np.float32)
    v = np.arange(h, dtype=np.float32)
    z = depth.astype(np.float32)
    x = (u[None, :] - K[0, 2]) * z / K[0, 0]
    y = (v[:, None] - K[1, 2]) * z / K[1, 1]
    return np.stack([x, y, z], axis=-1)


def pixels_to_points(depth, K, v, u):
    """Optical-frame points of the pixels (v, u), N x 3 float32."""
    z = depth[v, u].astype(np.float32)
    x = (u.astype(np.float32) - K[0, 2]) * z / K[0, 0]
    y = (v.astype(np.float32) - K[1, 2]) * z / K[1, 1]
    return np.stack([x, y, z], axis=1).astype(np.float32)


def transform(T, points):
    return points @ T[:3, :3].T.astype(points.dtype) + T[:3, 3].astype(points.dtype)


def inverse(T):
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def pose_matrix(translation, rotation_xyzw):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(rotation_xyzw).as_matrix()
    T[:3, 3] = translation
    return T


def voxel_index(points, voxel):
    """Voxel id of every point (0 .. n - 1) and the number of occupied voxels."""
    cells = np.floor(points / voxel).astype(np.int64)
    cells -= cells.min(axis=0)
    dims = cells.max(axis=0) + 1
    keys = (cells[:, 0] * dims[1] + cells[:, 1]) * dims[2] + cells[:, 2]
    _, inverse_ids = np.unique(keys, return_inverse=True)
    inverse_ids = inverse_ids.reshape(-1)
    return inverse_ids, int(inverse_ids.max()) + 1 if len(inverse_ids) else 0


def voxel_downsample(points, voxel):
    """Centroid of every occupied voxel, plus each point's voxel id."""
    if len(points) == 0:
        return np.zeros((0, 3)), np.zeros(0, np.int64)
    ids, n = voxel_index(points, voxel)
    counts = np.bincount(ids, minlength=n).astype(np.float64)
    centroids = np.stack([np.bincount(ids, points[:, k], minlength=n) for k in range(3)], axis=1)
    return centroids / counts[:, None], ids


def fit_plane(points, threshold, iterations, rng=None, up=UP):
    """Section 5.3: RANSAC plane, refined by least squares on its inliers.

    Returns (a, b, c, d) with the unit normal (a, b, c) pointing to the same side as `up`, and the
    inlier mask.
    """
    points = np.asarray(points, np.float64)
    if len(points) < 3:
        raise PerceptionError("table not found: too few points in the workspace")
    rng = rng or np.random.default_rng(0)
    idx = rng.integers(0, len(points), size=(iterations, 3))
    p0, p1, p2 = points[idx[:, 0]], points[idx[:, 1]], points[idx[:, 2]]
    normals = np.cross(p1 - p0, p2 - p0)
    norms = np.linalg.norm(normals, axis=1)
    keep = norms > 1e-9
    normals, p0 = normals[keep] / norms[keep, None], p0[keep]
    offsets = -np.einsum("ij,ij->i", normals, p0)
    # Hypotheses are scored on a random subset; the winner is refined on all points below.
    sample = points if len(points) <= 5000 else points[rng.choice(len(points), 5000, replace=False)]
    counts = np.concatenate([
        (np.abs(sample @ normals[i:i + 64].T + offsets[i:i + 64]) < threshold).sum(axis=0)
        for i in range(0, len(normals), 64)
    ])
    best = int(np.argmax(counts))
    inliers = np.abs(points @ normals[best] + offsets[best]) < threshold
    for _ in range(2):
        centroid = points[inliers].mean(axis=0)
        normal = np.linalg.svd(points[inliers] - centroid, full_matrices=False)[2][2]
        if normal @ up < 0:
            normal = -normal
        offset = -normal @ centroid
        inliers = np.abs(points @ normal + offset) < threshold
    return np.array([*normal, offset]), inliers


def plane_heights(points, plane):
    """Signed distance of every point above the plane."""
    return points @ plane[:3].astype(points.dtype) + plane[3]


def plane_height_at(plane, x, y):
    """z of the plane at (x, y)."""
    a, b, c, d = plane
    return -(a * x + b * y + d) / c


def tilt_deg(plane, up=UP):
    return float(np.degrees(np.arccos(np.clip(plane[:3] @ up, -1.0, 1.0))))


def dbscan(points, eps, min_points):
    """Section 5.4: DBSCAN labels (-1 = noise). A point is a core point when it has at least
    min_points points within eps, itself included; clusters are connected core points plus the
    border points next to them."""
    n = len(points)
    if n == 0:
        return np.zeros(0, np.int64)
    pairs = cKDTree(points).query_pairs(eps, output_type="ndarray")
    degree = np.bincount(pairs.ravel(), minlength=n) + 1
    core = degree >= min_points
    both = core[pairs[:, 0]] & core[pairs[:, 1]]
    edges = pairs[both]
    graph = coo_matrix((np.ones(len(edges)), (edges[:, 0], edges[:, 1])), shape=(n, n))
    _, components = connected_components(graph, directed=False)
    labels = np.full(n, -1, np.int64)
    labels[core] = components[core]
    a, b = pairs[core[pairs[:, 0]] ^ core[pairs[:, 1]]].T
    a_core = core[a]
    border, centre = np.where(a_core, b, a), np.where(a_core, a, b)
    labels[border] = labels[centre]  # a border point next to two clusters joins either one
    used = np.unique(labels[labels >= 0])
    remap = np.full(labels.max() + 1 if len(used) else 1, -1, np.int64)
    remap[used] = np.arange(len(used))
    labels[labels >= 0] = remap[labels[labels >= 0]]
    return labels


def statistical_outliers(points, neighbors, std_ratio):
    """Section 5.6: True for points whose mean neighbour distance is above mean + std_ratio * std."""
    if len(points) <= neighbors:
        return np.zeros(len(points), bool)
    distances, _ = cKDTree(points).query(points, neighbors + 1)
    mean = distances[:, 1:].mean(axis=1)
    return mean > mean.mean() + std_ratio * mean.std()


class TableFrame:
    """A frame on the table plane: x is world x projected onto the plane, z is the plane normal.

    Footprints, yaw and heights (section 5.8) are computed in this frame.
    """

    def __init__(self, plane, centre_xy):
        normal = plane[:3] / np.linalg.norm(plane[:3])
        x = np.array([1.0, 0.0, 0.0]) - normal[0] * normal
        if np.linalg.norm(x) < 1e-6:
            x = np.array([0.0, 1.0, 0.0]) - normal[1] * normal
        x /= np.linalg.norm(x)
        self.R = np.column_stack([x, np.cross(normal, x), normal])
        cx, cy = centre_xy
        self.origin = np.array([cx, cy, plane_height_at(plane, cx, cy)])

    def to_table(self, points):
        return (np.asarray(points, np.float64) - self.origin) @ self.R

    def to_world(self, points):
        return np.asarray(points, np.float64) @ self.R.T + self.origin

    def orientation(self, yaw):
        """World rotation of an object turned by yaw about the table normal, as an xyzw quaternion."""
        R = self.R @ Rotation.from_euler("z", yaw).as_matrix()
        return Rotation.from_matrix(R).as_quat()


def pixel_rays(pixels_uv, K, T_world_cam):
    """World-frame origin and unit directions of the rays through pixel centres (u, v)."""
    uv = np.asarray(pixels_uv, np.float64).reshape(-1, 2)
    d = np.column_stack([(uv[:, 0] - K[0, 2]) / K[0, 0], (uv[:, 1] - K[1, 2]) / K[1, 1], np.ones(len(uv))])
    d = d @ T_world_cam[:3, :3].T
    return T_world_cam[:3, 3], d / np.linalg.norm(d, axis=1, keepdims=True)


def ray_plane(pixels_uv, K, T_world_cam, plane, height=0.0):
    """World points where pixel rays meet the plane raised by `height`; NaN for rays that miss."""
    origin, d = pixel_rays(pixels_uv, K, T_world_cam)
    denom = d @ plane[:3]
    with np.errstate(divide="ignore", invalid="ignore"):
        t = -(origin @ plane[:3] + plane[3] - height) / denom
    t[(np.abs(denom) < 1e-9) | (t <= 0)] = np.nan
    return origin + t[:, None] * d
