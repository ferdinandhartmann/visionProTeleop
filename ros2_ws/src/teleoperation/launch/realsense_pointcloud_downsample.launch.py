import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch",
                "rs_launch.py",
            )
        ),
        launch_arguments={
            "depth_module.depth_profile": "1280x720x30",
            "pointcloud.enable": "true",
        }.items(),
    )

    target_frame_arg = DeclareLaunchArgument(
        "target_frame", default_value="mycobot_base"
    )
    downsample_factor_arg = DeclareLaunchArgument(
        "downsample_factor", default_value="4"
    )
    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate_hz", default_value="5.0"
    )
    input_topic_arg = DeclareLaunchArgument(
        "input_topic", default_value="/camera/depth/color/points"
    )
    output_topic_arg = DeclareLaunchArgument(
        "output_topic", default_value="/camera/depth/color/points_downsampled"
    )

    downsample_node = Node(
        package="teleoperation",
        executable="rgb_pointcloud_downsampler_node",
        name="rgb_pointcloud_downsampler",
        output="screen",
        parameters=[
            {
                "input_topic": LaunchConfiguration("input_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "target_frame": LaunchConfiguration("target_frame"),
                "publish_rate_hz": ParameterValue(
                    LaunchConfiguration("publish_rate_hz"), value_type=float
                ),
                "downsample_factor": ParameterValue(
                    LaunchConfiguration("downsample_factor"), value_type=int
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            target_frame_arg,
            downsample_factor_arg,
            publish_rate_arg,
            input_topic_arg,
            output_topic_arg,
            realsense_launch,
            downsample_node,
        ]
    )
