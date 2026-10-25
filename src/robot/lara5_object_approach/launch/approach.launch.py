"""Start only the approach task; use an already running native motion backend."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory("lara5_object_approach"),
                          "config", "approach.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=config),
        DeclareLaunchArgument("rviz", default_value="false"),
        Node(package="lara5_object_approach", executable="approach_node.py",
             name="object_approach", output="screen",
             parameters=[LaunchConfiguration("config_file")]),
        Node(package="rviz2", executable="rviz2", name="approach_execution_rviz",
             arguments=["-d", os.path.join(get_package_share_directory("lara5_object_approach"),
                                          "rviz", "grasp_planning.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz")), output="screen"),
    ])
