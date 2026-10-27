"""Read measured hand state only; starts no backend, controller or motion task."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('lara5_grasp')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=os.path.join(share, 'config', 'force_check.yaml')),
        # The hardware YAML whose hand_grasp force settings will be compared.
        DeclareLaunchArgument('grasp_config_file', default_value=''),
        Node(package='lara5_grasp', executable='hand_force_check.py', name='hand_force_check',
             output='screen', parameters=[LaunchConfiguration('config_file'),
                                          {'grasp_config_file': LaunchConfiguration('grasp_config_file')}]),
    ])
