"""scene_perception_node (docs/perception_architecture.md, section 8.2): ROS 2 around the core.

    services    /perception/capture_scene     restore_interfaces/CaptureScene
                /perception/register_object   restore_interfaces/RegisterObject
    publishes   /tf_static                    camera poses from the calibration file
                /perception/snapshot          restore_interfaces/SceneSnapshot, transient local
                /perception/markers           boxes, labels and footprints for Foxglove
                /perception/debug/<camera>/overlay   masks and labels on colour, once per capture
    subscribes  /joint_states                 observe-pose guard (5.0)
                /cameras/<camera>/...         only with source:=ros (simulation, 3.2)
"""

from __future__ import annotations

import array
import re
import shutil
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Quaternion, TransformStamped, Vector3
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2, PointField
from std_msgs.msg import ColorRGBA, Header
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from restore_interfaces.msg import ObjectState as ObjectStateMsg
from restore_interfaces.msg import SceneSnapshot as SceneSnapshotMsg
from restore_interfaces.srv import CaptureScene, RegisterObject

from .core import overlay, snapshot_io
from .core.capture import clip_depth, median_depth
from .core.config import config_dir, load_config, load_objects
from .core.extrinsics import load_extrinsics
from .core.gallery import Gallery
from .core.geometry import voxel_downsample
from .core.pipeline import Pipeline
from .core.sources import FileSource, FrameSource, RealSenseSource
from .core.types import CameraFrame, Intrinsics, PerceptionError

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
SNAPSHOT_NAME = re.compile(r"[A-Za-z0-9_][A-Za-z0-9_.-]{0,63}")  # also a folder name under snapshots/


# ---------- conversions (no cv_bridge, so pip's OpenCV never meets the apt one) ----------

def image_to_numpy(msg):
    """rgb8, bgr8, 16UC1 (millimetres) or 32FC1 (metres) to RGB uint8 or float32 metres."""
    if msg.encoding in ("rgb8", "bgr8"):
        rows = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
        image = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        return np.ascontiguousarray(image[..., ::-1] if msg.encoding == "bgr8" else image)
    if msg.encoding in ("16UC1", "mono16"):
        dtype = np.dtype(">u2" if msg.is_bigendian else "<u2")
        rows = np.frombuffer(bytes(msg.data), dtype).reshape(msg.height, msg.step // 2)
        return rows[:, :msg.width].astype(np.float32) / 1000.0
    if msg.encoding == "32FC1":
        dtype = np.dtype(">f4" if msg.is_bigendian else "<f4")
        rows = np.frombuffer(bytes(msg.data), dtype).reshape(msg.height, msg.step // 4)
        depth = np.nan_to_num(rows[:, :msg.width].astype(np.float32), nan=0.0, posinf=0.0, neginf=0.0)
        return depth
    raise PerceptionError(f"unsupported image encoding {msg.encoding}")


def rgb_image_msg(rgb, header):
    msg = Image(header=header, height=rgb.shape[0], width=rgb.shape[1], encoding="rgb8", is_bigendian=0,
                step=rgb.shape[1] * 3)
    msg.data = np.ascontiguousarray(rgb).tobytes()
    return msg


def cloud_msg(points, header):
    points = np.ascontiguousarray(points, np.float32).reshape(-1, 3)
    fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate("xyz")]
    return PointCloud2(header=header, height=1, width=len(points), fields=fields, is_bigendian=False,
                       point_step=12, row_step=12 * len(points), data=points.tobytes(), is_dense=True)


def transform_to_matrix(t):
    T = np.eye(4)
    q = t.transform.rotation
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    return T


def object_msg(obj, header, cloud_voxel):
    m = ObjectStateMsg()
    m.id, m.label, m.label_score = obj.id, obj.label, float(obj.label_score)
    m.pose.position = Point(x=float(obj.position[0]), y=float(obj.position[1]), z=float(obj.position[2]))
    qx, qy, qz, qw = (float(v) for v in obj.orientation_xyzw)
    m.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    m.size = Vector3(x=float(obj.size[0]), y=float(obj.size[1]), z=float(obj.size[2]))
    m.footprint = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in obj.footprint]
    m.symmetry = int(min(obj.symmetry, 255))
    m.up_side_matters, m.up_side, m.support_id = bool(obj.up_side_matters), obj.up_side, obj.support_id
    m.embedding = array.array("f", [float(v) for v in obj.embedding])
    points = voxel_downsample(obj.points.astype(np.float64), cloud_voxel)[0] if len(obj.points) else obj.points
    m.cloud = cloud_msg(points, header)
    m.cameras_seen = int(obj.cameras_seen) & 0xFF
    m.depth_reliable, m.confidence = bool(obj.depth_reliable), float(obj.confidence)
    return m


def markers_msg(snapshot, header):
    markers = MarkerArray()
    markers.markers.append(Marker(header=header, action=Marker.DELETEALL))
    for i, obj in enumerate(snapshot.objects):
        r, g, b = (c / 255.0 for c in overlay.color_of(i))
        qx, qy, qz, qw = (float(v) for v in obj.orientation_xyzw)
        box = Marker(header=header, ns="objects", id=i, type=Marker.CUBE, action=Marker.ADD)
        box.pose.position = Point(x=float(obj.position[0]), y=float(obj.position[1]), z=float(obj.position[2]))
        box.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        box.scale = Vector3(x=max(float(obj.size[0]), 0.005), y=max(float(obj.size[1]), 0.005),
                            z=max(float(obj.size[2]), 0.005))
        box.color = ColorRGBA(r=r, g=g, b=b, a=0.5)
        text = Marker(header=header, ns="labels", id=i, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
        text.pose.position = Point(x=box.pose.position.x, y=box.pose.position.y,
                                   z=box.pose.position.z + float(obj.size[2]) / 2.0 + 0.03)
        text.pose.orientation.w = 1.0
        text.scale.z = 0.025
        text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        text.text = f"{obj.id} {obj.label}" + (f" {obj.label_score:.2f}" if obj.label != "unknown" else "")
        outline = Marker(header=header, ns="footprints", id=i, type=Marker.LINE_STRIP, action=Marker.ADD)
        outline.pose.orientation.w = 1.0
        outline.scale.x = 0.003
        outline.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
        outline.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]) + 0.001)
                          for p in list(obj.footprint) + list(obj.footprint[:1])]
        markers.markers.extend([box, text, outline])
    return markers


# ---------- simulation source ----------

class RosTopicSource(FrameSource):
    """Section 3.2: simulated cameras on /cameras/<name>/..., posed from TF by the image frame_id."""

    def __init__(self, node, config, group):
        self.cfg = config["capture"]
        self.names = sorted(set(config["cameras"].values()))
        self._cond = threading.Condition()
        self._info, self._color, self._depths, self._want = {}, {}, {}, 0
        for name in self.names:
            base = f"/cameras/{name}"
            node.create_subscription(Image, f"{base}/color/image_raw", lambda m, n=name: self._on_color(n, m),
                                     qos_profile_sensor_data, callback_group=group)
            node.create_subscription(Image, f"{base}/aligned_depth_to_color/image_raw",
                                     lambda m, n=name: self._on_depth(n, m), qos_profile_sensor_data,
                                     callback_group=group)
            node.create_subscription(CameraInfo, f"{base}/color/camera_info", lambda m, n=name: self._on_info(n, m),
                                     qos_profile_sensor_data, callback_group=group)

    def _on_info(self, name, msg):
        with self._cond:
            self._info[name] = msg

    def _on_color(self, name, msg):
        with self._cond:
            if self._want:
                self._color[name] = msg

    def _on_depth(self, name, msg):
        with self._cond:
            if self._want and len(self._depths.setdefault(name, [])) < self._want:
                self._depths[name].append(msg)
                self._cond.notify_all()

    def status(self):
        with self._cond:
            return {n: "camera_info received" if n in self._info else "no camera_info yet" for n in self.names}

    def capture(self, frames):
        timeout = float(self.cfg["timeout_s"])
        with self._cond:
            active = [n for n in self.names if n in self._info]
            if not active:
                raise PerceptionError("no simulated camera has published camera_info on /cameras/<name>/...")
            self._color, self._depths, self._want = {}, {}, frames

            def ready():
                return all(len(self._depths.get(n, [])) >= frames and n in self._color for n in active)

            done = self._cond.wait_for(ready, timeout)
            colors, depths, infos = dict(self._color), dict(self._depths), dict(self._info)
            self._want = 0
        if not done:
            missing = [n for n in active if len(depths.get(n, [])) < frames or n not in colors]
            raise PerceptionError(f"cameras {missing} did not deliver {frames} frames within {timeout:.0f} s")
        result = []
        for name in active:
            info, color_msg = infos[name], colors[name]
            depth = median_depth([image_to_numpy(m) for m in depths[name]], self.cfg["min_valid_fraction"])
            depth = clip_depth(depth, self.cfg["min_depth_m"], self.cfg["max_depth_m"])
            k = info.k
            intr = Intrinsics(k[0], k[4], k[2], k[5], info.width, info.height, info.distortion_model,
                              list(info.d) or [0.0] * 5)
            stamp = Time.from_msg(color_msg.header.stamp).nanoseconds * 1e-9
            result.append(CameraFrame(name, name, image_to_numpy(color_msg), depth, intr, stamp, frames=frames,
                                      optical_frame=color_msg.header.frame_id or info.header.frame_id))
        return result


# ---------- the node ----------

class ScenePerceptionNode(Node):
    def __init__(self):
        super().__init__("scene_perception_node")
        self.declare_parameter("config_file", str(config_dir() / "perception.yaml"))
        self.declare_parameter("objects_file", str(config_dir() / "objects.yaml"))
        self.declare_parameter("data_dir", "/data")
        self.declare_parameter("extrinsics_file", "")  # default: <data_dir>/calibration/extrinsics.yaml
        self.declare_parameter("source", "realsense")  # realsense, ros (simulation) or file
        self.declare_parameter("file_source_path", "")
        self.declare_parameter("load_models", True)
        self.declare_parameter("publish_overlays", True)

        p = {name: self.get_parameter(name).value for name in (
            "config_file", "objects_file", "data_dir", "extrinsics_file", "source", "file_source_path",
            "load_models", "publish_overlays")}
        self.cfg = load_config(p["config_file"])
        self.objects = load_objects(p["objects_file"])
        self.data_dir = Path(p["data_dir"])
        self.extrinsics = load_extrinsics(p["extrinsics_file"] or self.data_dir / "calibration" / "extrinsics.yaml")
        self.publish_overlays = bool(p["publish_overlays"])
        self.world = self.cfg["world_frame"]
        self.flange = self.cfg["flange_frame"]

        self.sensor_group = ReentrantCallbackGroup()
        self.service_group = MutuallyExclusiveCallbackGroup()  # one capture at a time
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_tf = StaticTransformBroadcaster(self)
        self._publish_camera_tf()

        self._joints = {}  # name -> (position, arrival time)
        self._last_motion = 0.0
        self._joint_lock = threading.Lock()
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, qos_profile_sensor_data,
                                 callback_group=self.sensor_group)

        self.model_error = None
        segmenter = embedder = None
        if p["load_models"]:
            segmenter, embedder = self._load_models()
        self.gallery = Gallery(self.data_dir / "gallery")
        self.pipeline = Pipeline(self.cfg, self.objects, segmenter, embedder, self.gallery)

        self.source_kind, self.file_source_path = p["source"], p["file_source_path"]
        self.source = None
        self.source_error = None
        self._open_source()
        self._reference = (None, None)  # (mtime, Snapshot)

        self.snapshot_pub = self.create_publisher(SceneSnapshotMsg, "/perception/snapshot", LATCHED)
        self.marker_pub = self.create_publisher(MarkerArray, "/perception/markers", LATCHED)
        self.overlay_pubs = {}
        self.create_service(CaptureScene, "/perception/capture_scene", self._capture,
                            callback_group=self.service_group)
        self.create_service(RegisterObject, "/perception/register_object", self._register,
                            callback_group=self.service_group)
        self.get_logger().info(
            f"ready: source {self.source_kind} ({self.source_error or 'ok'}), "
            f"{len(self.extrinsics)} calibrated camera(s), gallery {self.gallery.names() or 'empty'}, "
            f"models {'off' if not p['load_models'] else self.model_error or 'loaded'}")

    # ---------- start-up ----------

    def _load_models(self):
        """SAM 2.1 and DINOv2, each optional: a model that fails to load is reported in every reply."""
        from .core.embed import Dinov2Embedder
        from .core.segment import Sam2Segmenter

        loaded, errors = {}, []
        for key, enabled, cls in (("sam", self.cfg["segment"]["enabled"], Sam2Segmenter),
                                  ("dino", self.cfg["identify"]["enabled"], Dinov2Embedder)):
            if not enabled:
                continue
            try:
                model = cls(self.cfg["models"])
                model.load()
                loaded[key] = model
            except Exception as exc:
                errors.append(f"{cls.__name__}: {type(exc).__name__}: {exc}")
                self.get_logger().error(f"model loading failed: {errors[-1]}")
        self.model_error = "; ".join(errors) or None
        return loaded.get("sam"), loaded.get("dino")

    def _open_source(self):
        try:
            if self.source_kind == "realsense":
                self.source = RealSenseSource(self.cfg)
                offline = self.source.wait_online(10.0)
                if offline:
                    self.get_logger().warning(f"cameras not streaming yet: {offline}")
            elif self.source_kind == "ros":
                self.source = RosTopicSource(self, self.cfg, self.sensor_group)
            elif self.source_kind == "file":
                self.source = FileSource(self.file_source_path, self.cfg)
            else:
                raise PerceptionError(f"unknown source {self.source_kind!r}: use realsense, ros or file")
            self.source_error = None
        except PerceptionError as exc:
            self.source, self.source_error = None, str(exc)
            self.get_logger().error(f"frame source: {exc}")

    def _publish_camera_tf(self):
        """Section 3.4: fixed cameras from world, hand cameras from the flange."""
        stamp = self.get_clock().now().to_msg()
        transforms = []
        for cam in self.extrinsics.values():
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, cam.parent_frame, cam.frame_id
            x, y, z = (float(v) for v in cam.T_parent_cam[:3, 3])
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
            qx, qy, qz, qw = (float(v) for v in Rotation.from_matrix(cam.T_parent_cam[:3, :3]).as_quat())
            t.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)
            transforms.append(t)
            if cam.name not in self.cfg["cameras"].values():
                self.get_logger().warning(f"calibrated camera {cam.serial} is named {cam.name}, which is not in "
                                          f"the cameras list of the config")
        if transforms:
            self.static_tf.sendTransform(transforms)

    # ---------- 5.0 observe-pose guard ----------

    def _on_joint_states(self, msg):
        """Arrival time, not the header stamp, so a clock offset to the Ubuntu PC cannot block captures."""
        now = time.monotonic()
        watched = self.cfg["observe"]["joints"]
        speed_limit = self.cfg["observe"]["max_speed_rad_s"]
        with self._joint_lock:
            for name, position in zip(msg.name, msg.position):
                previous = self._joints.get(name)
                if (name in watched and previous and now > previous[1]
                        and abs(position - previous[0]) / (now - previous[1]) > speed_limit):
                    self._last_motion = now
                self._joints[name] = (position, now)

    def _observe_problem(self):
        cfg = self.cfg["observe"]
        if not cfg["require_observe_pose"]:
            return None
        if not cfg["joints"]:
            return "observe.require_observe_pose is set but observe.joints is empty"
        now = time.monotonic()
        with self._joint_lock:
            for name, target in cfg["joints"].items():
                if name not in self._joints:
                    return f"no /joint_states for {name}"
                position, arrived = self._joints[name]
                if now - arrived > cfg["max_age_s"]:
                    return f"/joint_states is {now - arrived:.1f} s old"
                if abs(position - target) > cfg["tolerance_rad"]:
                    return f"the arm is not at the observe pose: {name} is {position - target:+.3f} rad off"
            if now - self._last_motion < cfg["settle_s"]:
                return "the arm is still moving or settling"
        return None

    # ---------- poses ----------

    def _lookup(self, parent, child):
        try:
            return transform_to_matrix(self.tf_buffer.lookup_transform(parent, child, Time(),
                                                                       timeout=Duration(seconds=1.0)))
        except Exception as exc:
            raise PerceptionError(f"no TF from {parent} to {child}: {exc}") from exc

    def _pose(self, frame):
        """Calibrated camera: the file (and the flange from TF for a hand camera). Else TF of its frame."""
        cam = self.extrinsics.get(frame.serial)
        if cam is not None:
            return cam.world_pose(self._lookup(self.world, self.flange) if cam.on_hand else None)
        return self._lookup(self.world, frame.frame_id)

    def _frames(self):
        if self.source is None:
            self._open_source()
            if self.source is None:
                raise PerceptionError(f"no frame source: {self.source_error}")
        frames = self.source.capture(int(self.cfg["capture"]["frames"]))
        for frame in frames:
            if frame.T_world_cam is None:
                frame.T_world_cam = self._pose(frame)
        return frames

    def _reference_snapshot(self):
        folder = self.data_dir / "snapshots" / "reference"
        if not (folder / "snapshot.json").exists():
            return None
        mtime = (folder / "snapshot.json").stat().st_mtime
        if self._reference[0] != mtime:
            self._reference = (mtime, snapshot_io.load_snapshot(folder, self.cfg["cameras"]))
        return self._reference[1]

    # ---------- services ----------

    def _notes(self, snapshot=None):
        notes = list(snapshot.warnings) if snapshot else []
        if self.model_error:
            notes.append(f"models unavailable ({self.model_error})")
        return notes

    def _capture(self, request, response):
        start = time.monotonic()
        problem = self._observe_problem()
        if problem:
            response.success, response.message = False, problem
            return response
        name = request.name.strip() or time.strftime("%Y%m%d-%H%M%S")
        if not SNAPSHOT_NAME.fullmatch(name):
            response.success, response.message = False, "name may only contain letters, digits, '_', '-' and '.'"
            return response
        try:
            frames = self._frames()
            reference = None if name == "reference" else self._reference_snapshot()
            snapshot = self.pipeline.run(frames, name, reference)
        except PerceptionError as exc:
            response.success, response.message = False, str(exc)
            return response
        except Exception as exc:
            self.get_logger().error(f"capture failed: {exc!r}")
            response.success, response.message = False, f"{type(exc).__name__}: {exc}"
            return response

        overlays = {f.name: overlay.draw(f, snapshot.objects) for f in snapshot.frames}
        if request.save_to_disk:
            shutil.rmtree(self.data_dir / "snapshots" / name, ignore_errors=True)
            snapshot_io.save_snapshot(snapshot, self.data_dir, overlays)
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.world)
        msg = SceneSnapshotMsg(header=header, snapshot_id=snapshot.snapshot_id,
                               table_plane=[float(v) for v in snapshot.table_plane])
        msg.objects = [object_msg(o, header, self.cfg["describe"]["cloud_voxel_m"]) for o in snapshot.objects]
        self.snapshot_pub.publish(msg)
        self.marker_pub.publish(markers_msg(snapshot, header))
        if self.publish_overlays:
            for camera, image in overlays.items():
                if camera not in self.overlay_pubs:
                    self.overlay_pubs[camera] = self.create_publisher(
                        Image, f"/perception/debug/{camera}/overlay", LATCHED)
                self.overlay_pubs[camera].publish(rgb_image_msg(image, Header(stamp=header.stamp,
                                                                              frame_id=f"{camera}_color_optical_frame")))
        labels = ", ".join(f"{o.id} {o.label}" for o in snapshot.objects) or "none"
        notes = self._notes(snapshot)
        response.success = True
        response.message = (f"{len(snapshot.objects)} objects ({labels}) from {len(frames)} camera(s) in "
                            f"{time.monotonic() - start:.1f} s" + ("; " + "; ".join(notes) if notes else ""))
        response.snapshot = msg
        self.get_logger().info(f"snapshot {name}: {response.message}")
        return response

    def _register(self, request, response):
        problem = self._observe_problem()
        if problem:
            response.success, response.message = False, problem
            return response
        try:
            stored, total, added, _ = self.pipeline.register(self._frames(), request.name, request.up_side)
        except (PerceptionError, ValueError) as exc:
            response.success, response.message = False, str(exc)
            return response
        except Exception as exc:
            self.get_logger().error(f"registration failed: {exc!r}")
            response.success, response.message = False, f"{type(exc).__name__}: {exc}"
            return response
        response.success, response.views_stored = True, int(total)
        response.message = f"stored {added} view(s) of {stored}, {total} in total"
        self.get_logger().info(response.message)
        return response

    def shutdown(self):
        if self.source is not None:
            self.source.close()


def main(args=None):
    rclpy.init(args=args)
    node = ScenePerceptionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
