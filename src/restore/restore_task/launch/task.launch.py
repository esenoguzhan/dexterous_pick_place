"""The orchestrator (docs/task_architecture.md). Needs perception, the safety filter, the arm and hand
controllers and, with approach_mode:=approach_node, lara5_object_approach.

    ros2 launch restore_task task.launch.py
    ros2 action send_goal /restore/run restore_interfaces/action/Restore "{mode: reference}"
    ros2 action send_goal --feedback /restore/run restore_interfaces/action/Restore "{mode: restore, dry_run: true}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument("config_file",
                              default_value=PathJoinSubstitution([FindPackageShare("restore_task"), "config", "task.yaml"])),
        DeclareLaunchArgument("objects_file", default_value=PathJoinSubstitution(
            [FindPackageShare("restore_perception"), "config", "objects.yaml"])),
        DeclareLaunchArgument("data_dir", default_value="/data", description="where perception saves snapshots"),
        DeclareLaunchArgument("approach_mode", default_value="",
                              description="move_to_pose or approach_node; empty = the config file's value"),
    ]
    node = Node(package="restore_task", executable="restore_task", name="restore_task", output="screen",
                parameters=[{k: LaunchConfiguration(k) for k in ("config_file", "objects_file", "data_dir",
                                                                 "approach_mode")}])
    return LaunchDescription(args + [node])
