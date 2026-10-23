"""Launch two serial-selected D435 cameras in separate namespaces."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = str(
        Path(get_package_share_directory('neurahackathon_cameras'))
        / 'config' / 'streams.yaml'
    )
    declarations = [
        DeclareLaunchArgument('camera_1_serial', default_value='263522073705',
                              description='Serial number of camera_1'),
        DeclareLaunchArgument('camera_2_serial', default_value='262722075125',
                              description='Serial number of camera_2'),
        DeclareLaunchArgument('camera_namespace', default_value='cameras'),
        DeclareLaunchArgument('config_file', default_value=config,
                              description='Shared ROS parameter YAML file'),
    ]
    cameras = [
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name=name,
            namespace=LaunchConfiguration('camera_namespace'),
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'camera_name': name,
                    # Keep numeric serials as strings without relying on CLI parsing.
                    'serial_no': ParameterValue(
                        LaunchConfiguration(f'{name}_serial'), value_type=str
                    ),
                },
            ],
            output='screen',
        )
        for name in ('camera_1', 'camera_2')
    ]
    return LaunchDescription(declarations + cameras)
