"""Camera-guided simulation with fixed tabletop fixtures; no hardware mode."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("lara5_sim_approach")
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("rviz", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("sim_config", default_value=os.path.join(
            get_package_share_directory("lara5_mujoco_sim"), "config", "mujoco_sim.yaml")),
        DeclareLaunchArgument("obstacles_file", default_value=os.path.join(share, "config", "obstacle_demo.yaml")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "camera_approach.launch.py")),
            launch_arguments={
                "headless": LaunchConfiguration("headless"),
                "rviz": LaunchConfiguration("rviz"),
                "sim_config": LaunchConfiguration("sim_config"),
                "obstacles_file": LaunchConfiguration("obstacles_file"),
            }.items(),
        ),
    ])
