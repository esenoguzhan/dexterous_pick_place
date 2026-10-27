"""Wrist-only hardware tasks. Camera/backend startup is explicit and optional."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('lara5_wrist_grasp')
    backend = get_package_share_directory('lara5_controller_manager')
    config = LaunchConfiguration('config_file')
    enabled = ParameterValue(LaunchConfiguration('execution_enabled'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=os.path.join(share, 'config', 'hardware.yaml')),
        DeclareLaunchArgument('execution_enabled', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('start_backend', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('robot_ip', default_value='192.168.2.13'),
        DeclareLaunchArgument('hand_ip', default_value='192.168.2.75'),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        GroupAction(condition=IfCondition(LaunchConfiguration('start_backend')), scoped=True, actions=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
                backend, 'launch', 'controller_manager.launch.py')), launch_arguments={
                    'deployment_mode': 'robot', 'hands': 'on', 'safety': 'on',
                    'allow_scene_updates': 'false', 'allow_grasp_updates': 'false',
                    'robot_ip': LaunchConfiguration('robot_ip'), 'hand_ip': LaunchConfiguration('hand_ip'),
                    'rviz': LaunchConfiguration('rviz'),
                }.items()),
        ]),
        Node(package='lara5_wrist_grasp', executable='wrist_approach.py', name='wrist_approach',
             output='screen', parameters=[config, {'execution_enabled': enabled}]),
        Node(package='lara5_grasp', executable='grasp_node.py', name='hand_grasp',
             output='screen', parameters=[config, {'feedback_mode': 'hardware',
                'hardware_execution_enabled': enabled}]),
    ])
