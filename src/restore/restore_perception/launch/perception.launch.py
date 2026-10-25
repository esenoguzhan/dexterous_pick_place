"""scene_perception_node with its configuration (docs/perception_architecture.md, section 8.2).

    ros2 launch restore_perception perception.launch.py                       # D435s on the Jetson
    ros2 launch restore_perception perception.launch.py source:=ros           # simulated cameras
    ros2 launch restore_perception perception.launch.py source:=file file_source_path:=/data/snapshots
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare("restore_perception"), "config"])
    args = [
        DeclareLaunchArgument("source", default_value="realsense", description="realsense, ros or file"),
        DeclareLaunchArgument("config_file", default_value=PathJoinSubstitution([config, "perception.yaml"])),
        DeclareLaunchArgument("objects_file", default_value=PathJoinSubstitution([config, "objects.yaml"])),
        DeclareLaunchArgument("data_dir", default_value="/data",
                              description="gallery/, snapshots/ and calibration/extrinsics.yaml"),
        DeclareLaunchArgument("extrinsics_file", default_value="",
                              description="default: <data_dir>/calibration/extrinsics.yaml"),
        DeclareLaunchArgument("file_source_path", default_value="", description="for source:=file"),
        DeclareLaunchArgument("load_models", default_value="true", description="SAM 2.1 and DINOv2"),
    ]
    node = Node(
        package="restore_perception",
        executable="scene_perception_node",
        name="scene_perception_node",
        output="screen",
        parameters=[{
            name: LaunchConfiguration(name)
            for name in ("source", "config_file", "objects_file", "data_dir", "extrinsics_file", "file_source_path")
        } | {"load_models": ParameterValue(LaunchConfiguration("load_models"), value_type=bool)}],
    )
    return LaunchDescription(args + [node])
