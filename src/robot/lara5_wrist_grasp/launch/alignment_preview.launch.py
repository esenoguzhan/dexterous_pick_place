"""Read-only geometry inspection; no backend, planner, arm or hand task starts."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('lara5_wrist_grasp')
    return LaunchDescription([
        DeclareLaunchArgument('approach_config_file', default_value=os.path.join(share, 'config', 'hardware.yaml')),
        Node(package='lara5_wrist_grasp', executable='wrist_alignment_preview.py',
             name='wrist_alignment_preview', output='screen', parameters=[
                 {'approach_config_file': LaunchConfiguration('approach_config_file')}]),
    ])
