#!/usr/bin/env python3
"""Save a rendered RGB frame without a GUI (useful in headless Docker)."""

from pathlib import Path
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from lara5_sim_approach.vision import image_array


def main():
    rclpy.init()
    node = Node("save_sim_external_camera")
    node.declare_parameter("output_file", "/tmp/external_camera.png")
    frames = []
    node.create_subscription(Image, "/sim/external_camera/color/image_raw", frames.append, 2)
    deadline = time.monotonic() + 15
    code = 0
    try:
        while rclpy.ok() and not frames and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not frames:
            raise RuntimeError("No simulated camera image received within 15 seconds")
        output = Path(node.get_parameter("output_file").value)
        output.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(output), cv2.cvtColor(image_array(frames[-1]), cv2.COLOR_RGB2BGR)):
            raise RuntimeError(f"Could not save image to {output}")
        node.get_logger().info(f"Saved {output}")
    except Exception as error:
        node.get_logger().error(str(error))
        code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
