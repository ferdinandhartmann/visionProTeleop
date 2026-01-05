from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import sys
from launch.actions import DeclareLaunchArgument
import launch
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import TimerAction

    
def generate_launch_description():

    config_folder = os.path.join(get_package_share_directory("teleoperation"), "config")
    
    rviz_folder = os.path.join(get_package_share_directory("teleoperation"), "rviz")
    
    model_launch_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(
            get_package_share_directory("robot_description"),
            "urdf/mycobot_280_m5/mycobot_280_m5_camera_adaptive_gripper.urdf"
        )
    )
    
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)


    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )
        
    static_transform_map_mycobot_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_map_mycobot_base",
        arguments=[
            "0", "0", str(0.032 - 0.025),  # translation x y z
            "0", "0", "0",  # rotation roll pitch yaw (radians)
            "map",
            "mycobot_base"
        ],
        output="screen"
    )
    
    run_sim_node = Node(
        package="teleoperation",
        executable="run_sim.py",
        name="run_sim",
        output="screen",
    )

    depth_to_cloud_node = TimerAction(
        period=1.0,
        actions=[
            Node(
            package="teleoperation",
            executable="depth_to_cloud_node",
            name="depth_to_cloud",
            # remappings=[
            #     ("image_rect", "/camera/depth/image_raw"),
            # ("camera_info", "/camera/camera_info"),
            # ("points", "/camera/depth/points"),
            # ],
            parameters=[
                {"use_sim_time": True},
                # depth_to_cloud node parameters (defaults mirrored from source)
                {"depth_topic": "/camera/depth/image_raw"},
                {"rgb_topic": "/camera/color/image_raw"},
                {"info_topic": "/camera/camera_info"},
                {"output_topic": "/camera/depth/points"},
                {"target_frame": "mycobot_base"},
                {"downsample_step": 7},
                {"max_range_m": 2.0},
                {"publish_rate_hz": 2.0},
                {"bottom_crop_percent": 0.28},
                {"publish_single_frame": True},
                {"single_frame_topic": "/rgb_map/cloud_frame"},
                {"cropped_rgb_topic": "/rgb_map/cropped_rgb"},
                {"accumulate": True},
                {"max_points": 50000},
            ],
            output="screen",
            )
        ]
    )
    
    rgbd_sync_node = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name="rgbd_sync",
        remappings=[
            ("rgb/image", "/camera/color/image_raw"),
            ("depth/image", "/camera/depth/image_raw"),
            ("rgb/camera_info", "/camera/color/camera_info"),
            ("rgbd_image", "/rgbd_image"),
        ],
        parameters=[
            {"approx_sync": True},
            {"use_sim_time": True},
            {"decimation": 6},
        ],
        output="screen",
    )

    rtabmap_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        parameters=[
            {"frame_id": "camera"},
            {"map_frame_id": "map"},
            {"subscribe_rgbd": True},

            # 🔑 IMPORTANT
            {"use_tf_odom": True},
            {"odom_frame_id": "map"},
            {"publish_tf": False},

            # Mapping parameters
            {"Grid/FromDepth": True},
            {"Vis/MinInliers": "10"},
            {"RGBD/OptimizeFromGraphEnd": "0"},

            {"use_sim_time": True},
        ],
        remappings=[
            ("rgbd_image", "/rgbd_image"),
        ],
        output="screen",
    )



    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            os.path.join(rviz_folder, "mapping.rviz"),
        ],
    )

    nodes = [
        
        run_sim_node,

        model_launch_arg,
        
        robot_state_publisher_node,

        static_transform_map_mycobot_base,
        
        depth_to_cloud_node,
        
        # rgbd_sync_node,
        # rtabmap_node,
        
        rviz2_node,
        
    ]

    return LaunchDescription(nodes)
