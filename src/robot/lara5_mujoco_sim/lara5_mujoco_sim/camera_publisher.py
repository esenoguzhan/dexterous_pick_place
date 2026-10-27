"""Render coherent RGB/depth samples off the physics thread."""

import threading
import time

import mujoco
import numpy as np
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image

from lara5_mujoco_sim.camera import camera_pose, depth_limits, intrinsics, mask_depth


class StaticTransformSet:
    """Latched static transforms that accumulate.

    Humble's StaticTransformBroadcaster publishes each sendTransform() call as its own message on
    a depth-1 transient-local topic, so a node that starts later only receives the last call. Every
    camera's optical transform is therefore re-sent together whenever one is added."""

    def __init__(self, node):
        from tf2_ros import StaticTransformBroadcaster
        self._broadcaster = StaticTransformBroadcaster(node)
        self.transforms = []

    def sendTransform(self, transform):  # noqa: N802 - tf2 naming
        self.transforms.extend(transform if isinstance(transform, list) else [transform])
        self._broadcaster.sendTransform(list(self.transforms))


class RgbdCameraPublisher:
    """A fixed optical mount relative to world or a moving robot TF link."""

    def __init__(self, node, name, broadcaster):
        self.node = node
        self.name = name
        self.config = node.config[name]
        depth_limits(self.config)
        self.stop_event = threading.Event()
        self.thread = None
        self.frame = self.config["frame_id"]
        prefix = f"/sim/{name}"
        self.rgb_pub = node.create_publisher(Image, f"{prefix}/color/image_raw", 2)
        self.depth_pub = node.create_publisher(Image, f"{prefix}/depth/image_raw", 2)
        self.info_pub = node.create_publisher(CameraInfo, f"{prefix}/camera_info", 2)
        self.tf = broadcaster
        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        # Only the mount is static. robot_state_publisher supplies the changing
        # world -> hand transform; never publish a fixed world -> wrist pose.
        transform.header.frame_id = self.config.get("parent_frame", "world")
        transform.child_frame_id = self.frame
        xyz, optical = camera_pose(self.config)
        transform.transform.translation.x = float(xyz[0])
        transform.transform.translation.y = float(xyz[1])
        transform.transform.translation.z = float(xyz[2])
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, optical.flatten())
        rotation = transform.transform.rotation
        rotation.w, rotation.x, rotation.y, rotation.z = map(float, quat)
        self.tf.sendTransform(transform)
        node.get_logger().info(
            f"{name}: ideal RGB-D on {prefix}; optical TF "
            f"{transform.header.frame_id} -> {self.frame}")

    def start(self):
        if self.config["rate"] <= 0:
            raise ValueError(f"{self.name} rate must be positive")
        self.thread = threading.Thread(target=self._run, name=self.name, daemon=True)
        self.thread.start()

    def close(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join()

    def _image(self, values, encoding, stamp):
        msg = Image()
        msg.header.stamp, msg.header.frame_id = stamp, self.frame
        msg.height, msg.width = values.shape[:2]
        msg.encoding = encoding
        msg.is_bigendian = False
        msg.step = int(values.strides[0])
        msg.data = values.tobytes()
        return msg

    def _run(self):
        renderer = None
        try:
            width, height = self.config["width"], self.config["height"]
            renderer = mujoco.Renderer(self.node.model, height=height, width=width)
            snapshot = mujoco.MjData(self.node.model)
            option = mujoco.MjvOption()
            option.geomgroup[3] = 0  # Hide robot collision hulls; retain visual meshes.
            k = intrinsics(width, height, self.config["fovy"])
            period = 1.0 / self.config["rate"]
            while not self.stop_event.is_set():
                start = time.monotonic()
                with self.node.lock:
                    snapshot.qpos[:] = self.node.data.qpos
                    snapshot.qvel[:] = self.node.data.qvel
                    stamp = self.node.get_clock().now().to_msg()
                mujoco.mj_forward(self.node.model, snapshot)
                renderer.update_scene(snapshot, camera=self.name, scene_option=option)
                rgb = renderer.render().copy()
                renderer.enable_depth_rendering()
                depth = mask_depth(renderer.render(), self.config)
                renderer.disable_depth_rendering()
                info = CameraInfo()
                info.header.stamp, info.header.frame_id = stamp, self.frame
                info.width, info.height = width, height
                info.distortion_model = "plumb_bob"
                info.d = [0.0] * 5
                info.k = k.flatten().tolist()
                info.r = np.eye(3).flatten().tolist()
                info.p = np.column_stack((k, np.zeros(3))).flatten().tolist()
                self.info_pub.publish(info)
                self.rgb_pub.publish(self._image(rgb, "rgb8", stamp))
                self.depth_pub.publish(self._image(depth, "32FC1", stamp))
                self.stop_event.wait(max(0.0, period - (time.monotonic() - start)))
        except Exception as error:
            self.node.get_logger().error(
                f"{self.name} stopped: {error}. Headless rendering needs OpenGL; "
                "use sim_camera_backend:=egl with the viewer, or osmesa for headless operation. "
                "Physics remains running."
            )
        finally:
            if renderer is not None:
                renderer.close()
