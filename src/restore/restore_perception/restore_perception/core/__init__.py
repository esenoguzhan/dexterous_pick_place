"""The perception pipeline without ROS (docs/perception_architecture.md, section 9).

Nothing here imports rclpy, and pyrealsense2, torch, SAM 2 and transformers are only imported when a
camera or a model is opened, so the core also runs offline on saved captures.
"""
