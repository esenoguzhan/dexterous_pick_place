"""Small RGB-D adapter for the known, distinctly colored simulation props.

Uses rendered pixels and metric depth, never configured object positions. This
deliberately limited detector can later be replaced by the perception team's node.
"""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Detection:
    name: str
    center: np.ndarray
    dimensions: np.ndarray
    top: np.ndarray
    pixels: int


def dimensions(obj):
    size = obj["size"]
    if obj["type"] == "box":
        return 2 * np.asarray(size, dtype=float)
    if obj["type"] == "cylinder":
        return np.array([2 * size[0], 2 * size[0], 2 * size[1]])
    if obj["type"] == "sphere":
        return np.full(3, 2 * size[0], dtype=float)
    raise ValueError(f"Unsupported demo prop shape: {obj['type']}")


def detect_objects(rgb, depth, k, rotation, translation, objects, min_pixels=20):
    """Return observed upright props in world. Missing/ambiguous props are omitted.

    Known size/color are allowed priors; position and height come from RGB-D.
    Conservative boxes include size priors plus a margin for visible-surface bias.
    """
    rgb = np.asarray(rgb)
    depth, k = np.asarray(depth), np.asarray(k, dtype=float).reshape(3, 3)
    rotation, translation = np.asarray(rotation), np.asarray(translation)
    if rgb.shape != (*depth.shape, 3) or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError("RGB/depth dimensions or intrinsics are invalid")
    colors = rgb.astype(float) / 255.0
    chroma = colors / np.maximum(colors.sum(axis=2, keepdims=True), 1e-8)
    saturation = colors.max(axis=2) - colors.min(axis=2)
    valid_depth = np.isfinite(depth) & (depth > 0.05) & (depth < 5.0)
    result = {}
    for obj in objects:
        expected = np.asarray(obj["rgba"][:3], dtype=float)
        expected /= expected.sum()
        mask = ((np.linalg.norm(chroma - expected, axis=2) < 0.14)
                & (saturation > 0.15) & valid_depth).astype(np.uint8)
        # RGB antialiasing blends silhouette colors while depth has one surface
        # per pixel. Exclude that boundary so a robot/background depth cannot be
        # mistaken for a high point on a colored prop.
        mask = cv2.erode(mask, np.ones((3, 3), dtype=np.uint8), iterations=1)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        components = sorted(range(1, count), key=lambda i: stats[i, cv2.CC_STAT_AREA], reverse=True)
        if not components or stats[components[0], cv2.CC_STAT_AREA] < min_pixels:
            continue
        if len(components) > 1 and stats[components[1], cv2.CC_STAT_AREA] > max(
                min_pixels, 0.3 * stats[components[0], cv2.CC_STAT_AREA]):
            continue  # Multiple similarly colored components: do not guess identity.
        rows, cols = np.nonzero(labels == components[0])
        z = depth[rows, cols]
        camera = np.column_stack(((cols - k[0, 2]) * z / k[0, 0],
                                  (rows - k[1, 2]) * z / k[1, 1], z))
        world = camera @ rotation.T + translation
        top_z = np.percentile(world[:, 2], 95)
        top_points = world[world[:, 2] > top_z - 0.005]
        if len(top_points) < 3:
            continue
        xy = (np.percentile(top_points[:, :2], 5, axis=0)
              + np.percentile(top_points[:, :2], 95, axis=0)) / 2
        top = np.r_[xy, top_z]
        extent = dimensions(obj)
        center = top - np.array([0.0, 0.0, extent[2] / 2])
        result[obj["name"]] = Detection(obj["name"], center, extent + 0.03, top, len(rows))
    return result


def image_array(msg, depth_scale=0.001):
    """Decode ROS Image row stride and byte order without cv_bridge."""
    formats = {"rgb8": (np.uint8, 3), "bgr8": (np.uint8, 3),
               "16UC1": (np.dtype(">u2" if msg.is_bigendian else "<u2"), 1),
               "32FC1": (np.dtype(">f4" if msg.is_bigendian else "<f4"), 1)}
    if msg.encoding not in formats:
        raise ValueError(f"Unsupported RGB-D image encoding: {msg.encoding}")
    dtype, channels = formats[msg.encoding]
    dtype = np.dtype(dtype)
    row_bytes = msg.width * channels * dtype.itemsize
    if msg.step < row_bytes or len(msg.data) != msg.height * msg.step:
        raise ValueError("Malformed image buffer/stride")
    rows = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
    pixels = np.ascontiguousarray(rows[:, :row_bytes]).view(dtype)
    shape = (msg.height, msg.width, channels) if channels > 1 else (msg.height, msg.width)
    result = pixels.reshape(shape)
    if msg.encoding == '16UC1':
        if not np.isfinite(depth_scale) or depth_scale <= 0:
            raise ValueError('depth_scale must be positive metres per integer unit')
        return result.astype(np.float32)*depth_scale
    if msg.encoding == 'bgr8':
        return result[:, :, ::-1].copy()
    return result
