"""Start the complete manual-pose grasp simulation; no motion is triggered."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    controller = get_package_share_directory('lara5_controller_manager')
    approach = get_package_share_directory('lara5_object_approach')
    grasp = get_package_share_directory('lara5_grasp')
    lift = get_package_share_directory('lara5_object_lift')
    bringup = get_package_share_directory('lara5_grasp_bringup')
    simulation = get_package_share_directory('lara5_mujoco_sim')

    def include(share, filename, arguments):
        # Both task launches call their YAML argument config_file. Keep each
        # include's settings local, including its disabled child RViz option.
        return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', filename)),
            launch_arguments=arguments.items())])

    return LaunchDescription([
        DeclareLaunchArgument('sim_headless', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('sim_camera_backend', default_value='egl', choices=['egl', 'osmesa']),
        DeclareLaunchArgument('sim_initial_positions', default_value='0 0 1.5708 0 1.5708 0'),
        DeclareLaunchArgument('sim_config', default_value=os.path.join(simulation, 'config', 'mujoco_sim.yaml')),
        DeclareLaunchArgument('approach_config', default_value=os.path.join(approach, 'config', 'approach.yaml')),
        DeclareLaunchArgument('grasp_config', default_value=os.path.join(grasp, 'config', 'grasp.yaml')),
        DeclareLaunchArgument('lift_config', default_value=os.path.join(lift, 'config', 'lift.yaml')),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(bringup, 'rviz', 'grasp_lift.rviz')),
        include(controller, 'controller_manager.launch.py', {
            'deployment_mode': 'simulation', 'hands': 'on', 'safety': 'on', 'rviz': 'false',
            'allow_scene_updates': 'true', 'allow_grasp_updates': 'true',
            'sim_headless': LaunchConfiguration('sim_headless'),
            'sim_config': LaunchConfiguration('sim_config'),
            'sim_initial_positions': LaunchConfiguration('sim_initial_positions'),
            'sim_camera_backend': LaunchConfiguration('sim_camera_backend'),
        }),
        include(approach, 'approach.launch.py', {
            'config_file': LaunchConfiguration('approach_config'), 'rviz': 'false',
        }),
        include(grasp, 'grasp.launch.py', {'config_file': LaunchConfiguration('grasp_config')}),
        include(lift, 'lift.launch.py', {'config_file': LaunchConfiguration('lift_config')}),
        Node(package='rviz2', executable='rviz2', name='grasp_bringup_rviz', output='screen',
             arguments=['-d', LaunchConfiguration('rviz_config')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
