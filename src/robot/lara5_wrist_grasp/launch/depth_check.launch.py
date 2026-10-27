"""Read camera data only; starts no robot backend or motion task."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('lara5_wrist_grasp')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=os.path.join(share, 'config', 'depth_check.yaml')),
        DeclareLaunchArgument('approach_config_file', default_value=os.path.join(share, 'config', 'hardware.yaml')),
        Node(package='lara5_wrist_grasp', executable='wrist_depth_check.py',
             name='wrist_depth_check', output='screen', parameters=[
                 LaunchConfiguration('config_file'),
                 {'approach_config_file': LaunchConfiguration('approach_config_file')}]),
    ])
