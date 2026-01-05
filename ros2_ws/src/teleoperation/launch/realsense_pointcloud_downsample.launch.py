import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


rviz_config_path = os.path.join(
    get_package_share_directory("teleoperation"),
    "rviz",
    "realsense_pointcloud.rviz",
)
    
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
            "enable_depth": "true",
            "enable_color": "true",

            "depth_module.depth_profile": "1280x720x30",
            "rgb_camera.color_profile": "640x480x30",

            "align_depth.enable": "true",

            "pointcloud.enable": "true",
            # "pointcloud.stream_filter": "1",
            # "pointcloud.stream_index_filter": "0",  # usually 0

            # Reduce internal buffering
            # "queue_size": "1",
            # "wait_for_device_timeout": "5.0",
            # "enable_sync": "true",
        }.items(),
    )


    downsample_node = Node(
        package="teleoperation",
        executable="rgb_pointcloud_downsampler_node",
        name="rgb_pointcloud_downsampler",
        output="screen",
        parameters=[
            {
                "input_topic": "/camera/camera/depth/color/points",
                "output_topic": "/points_downsampled",
                "target_frame": "camera_link",
                "publish_rate_hz": 10.0,
                "downsample_factor": 70,
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
    )

    return LaunchDescription([
            realsense_launch,
            downsample_node,
            rviz_node,
        ]
    )
