"""Static hand overlay with optional RViz. No robot motion node is started."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("lara5_object_approach")
    def rviz(context):
        planning = LaunchConfiguration("plan_trajectories").perform(context).lower() in ("true", "1")
        config = "grasp_planning.rviz" if planning else "grasp_preview.rviz"
        return [Node(package="rviz2", executable="rviz2", name="grasp_preview_rviz",
                     arguments=["-d", os.path.join(share, "rviz", config)],
                     condition=IfCondition(LaunchConfiguration("rviz")), output="screen")]
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=os.path.join(share, "config/grasp_preview.yaml")),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("plan_trajectories", default_value="false",
                              description="Enable two-phase planning preview; never execute motion"),
        Node(package="lara5_object_approach", executable="grasp_preview_node.py",
             name="grasp_preview", output="screen", parameters=[LaunchConfiguration("config_file"),
                 {"plan_trajectories": ParameterValue(LaunchConfiguration("plan_trajectories"), value_type=bool)}]),
        OpaqueFunction(function=rviz),
    ])
