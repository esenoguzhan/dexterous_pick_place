"""scene_perception_node on the MuJoCo cameras (docs/perception_simulation_eval.md).

The simulator publishes /sim/external_camera/... and /sim/wrist_camera/...; the node's ROS source
expects /cameras/camera_N/... with the RealSense topic names, so this launch remaps them:

    ros2 launch restore_perception perception_sim.launch.py                # fixed camera only
    ros2 launch restore_perception perception_sim.launch.py wrist:=true    # plus the wrist camera

Camera poses come from TF (world -> sim_*_optical_frame), models are off by default (every label
is "unknown", masks come from depth), which is what a pose-accuracy test needs.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

EXTERNAL = [
    ("/cameras/camera_1/color/image_raw", "/sim/external_camera/color/image_raw"),
    ("/cameras/camera_1/aligned_depth_to_color/image_raw", "/sim/external_camera/depth/image_raw"),
    ("/cameras/camera_1/color/camera_info", "/sim/external_camera/camera_info"),
]
WRIST = [
    ("/cameras/camera_2/color/image_raw", "/sim/wrist_camera/color/image_raw"),
    ("/cameras/camera_2/aligned_depth_to_color/image_raw", "/sim/wrist_camera/depth/image_raw"),
    ("/cameras/camera_2/color/camera_info", "/sim/wrist_camera/camera_info"),
]


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare("restore_perception"), "config"])
    args = [
        DeclareLaunchArgument("wrist", default_value="false", description="also read the wrist camera"),
        DeclareLaunchArgument("config_file", default_value=PathJoinSubstitution([config, "perception_sim.yaml"])),
        DeclareLaunchArgument("objects_file", default_value=PathJoinSubstitution([config, "objects.yaml"])),
        DeclareLaunchArgument("data_dir", default_value="/tmp/perception_sim",
                              description="gallery/, snapshots/ and calibration/extrinsics.yaml"),
        DeclareLaunchArgument("extrinsics_file", default_value="",
                              description="normally empty in simulation: camera poses come from TF"),
        DeclareLaunchArgument("load_models", default_value="false", description="SAM 2.1 and DINOv2"),
    ]
    parameters = [{
        "source": "ros",
        **{name: LaunchConfiguration(name) for name in ("config_file", "objects_file", "data_dir", "extrinsics_file")},
        "load_models": ParameterValue(LaunchConfiguration("load_models"), value_type=bool),
    }]
    common = dict(package="restore_perception", executable="scene_perception_node", name="scene_perception_node",
                  output="screen", parameters=parameters)
    return LaunchDescription(args + [
        Node(**common, remappings=EXTERNAL, condition=UnlessCondition(LaunchConfiguration("wrist"))),
        Node(**common, remappings=EXTERNAL + WRIST, condition=IfCondition(LaunchConfiguration("wrist"))),
    ])
