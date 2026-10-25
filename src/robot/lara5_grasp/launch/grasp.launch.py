"""Start only the hand-closure task against an existing hand controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory("lara5_grasp"), "config", "grasp.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=config),
        Node(package="lara5_grasp", executable="grasp_node.py", name="hand_grasp",
             output="screen", parameters=[LaunchConfiguration("config_file")]),
    ])
