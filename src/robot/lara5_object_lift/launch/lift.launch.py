"""Start only the explicit lift task; the native backend must already be running."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config=os.path.join(get_package_share_directory('lara5_object_lift'),'config','lift.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config_file',default_value=config),
        Node(package='lara5_object_lift',executable='lift_node.py',name='object_lift',
             parameters=[LaunchConfiguration('config_file')],output='screen'),
    ])
