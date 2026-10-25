"""Simulation + external RGB-D camera + native NEURA trajectory services."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup(context):
    share = get_package_share_directory("lara5_controller_manager")
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "controller_manager.launch.py")),
        launch_arguments={
            "deployment_mode": "simulation", "sim_headless": LaunchConfiguration("headless"),
            "sim_config": LaunchConfiguration("sim_config"), "rviz": LaunchConfiguration("rviz"),
            "safety": "on", "allow_scene_updates": "true",
            "allow_grasp_updates": LaunchConfiguration("allow_grasp_updates"),
            "obstacles_file": LaunchConfiguration("obstacles_file"),
            "sim_obstacles_file": LaunchConfiguration("obstacles_file"),
        }.items(),
    )
    return [bringup]


def generate_launch_description():
    default = os.path.join(get_package_share_directory("lara5_mujoco_sim"), "config", "mujoco_sim.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("rviz", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("sim_config", default_value=default),
        DeclareLaunchArgument("allow_grasp_updates", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("obstacles_file", default_value=os.path.join(
            get_package_share_directory("lara5_safety_filter"), "config", "obstacles.yaml")),
        OpaqueFunction(function=setup),
    ])
