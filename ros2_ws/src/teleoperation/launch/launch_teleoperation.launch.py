import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

    
def generate_launch_description():

    config_folder = os.path.join(get_package_share_directory("teleoperation"), "config")
    teleop_config = os.path.join(config_folder, "teleoperation.yaml")
    rviz_folder = os.path.join(get_package_share_directory("teleoperation"), "rviz")
    
    model = os.path.join(
        get_package_share_directory("robot_description"),
        "urdf/mycobot_280_m5/mycobot_280_m5_camera_adaptive_gripper.urdf"
    )
    
    #####################################################################################
    ############################### Set Launch Arguments ################################
    #####################################################################################
    serial_port = "/dev/ttyACM0"
    baud_rate = "115200"
    
    enable_camera = True
    camera_mode = "both" # robot, realsense, both
    enable_pointcloud = True
    enable_audio = True


    use_realsense = camera_mode in ("realsense", "both")
    use_realsense_condition = IfCondition("true" if use_realsense else "false")

    
    robot_description = ParameterValue(Command(['xacro ', model]), value_type=str)


    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )
    
    # Hardware access is now handled directly inside teleop_control,
    # which also publishes /joint_states. We therefore do not launch
    # the mycobot_280 listen_real node here to avoid double-opening
    # the same serial port.
    # listen_real_node = Node(
    #     package="mycobot_280",
    #     executable="listen_real",
    #     name="listen_real",
    #     parameters=[
    #         {'port': LaunchConfiguration('port')},
    #         {'baud': LaunchConfiguration('baud')}
    #     ],
    #     output="screen"
    # )
    
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
    
    vp_transform_publisher_node = Node(
        package="teleoperation",
        executable="vp_transform_publisher.py",
        name="vp_transform_publisher",
        output="screen",
        emulate_tty=True,
        additional_env={"PYTHONUNBUFFERED": "1"},        
        parameters=[
            teleop_config,
        ],
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch",
                "rs_launch.py",
            )
        ),
        launch_arguments={
            "enable_depth": "true" if enable_pointcloud else "false",
            "enable_color": "true",
            "depth_module.depth_profile": "640x480x15",
            "rgb_camera.color_profile": "640x480x15",
            "align_depth.enable": "false",
            "pointcloud.enable": "true" if enable_pointcloud else "false",
            "enable_infra": "false",
            "enable_infra1": "false",
            "enable_infra2": "false"
        }.items(),
        condition=use_realsense_condition,
    )
    
            
    realsense_description = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="realsense_state_publisher",
        namespace="realsense",
        output="screen",
        parameters=[{
            "robot_description": ParameterValue(
                Command([
                    "xacro ",
                    os.path.join(
                        get_package_share_directory("realsense2_description"),
                        "urdf",
                        "test_d435_camera.urdf.xacro",
                    ),
                    # " base_frame:=realsense_base_link"
                ]),
                value_type=str,
            )
        }],
        condition=use_realsense_condition,
    )

    downsample_node = Node(
        package="teleoperation",
        executable="rgb_pointcloud_downsampler_node",
        name="rgb_pointcloud_downsampler",
        output="screen",
        parameters=[
            teleop_config,
        ],
        condition=IfCondition("true" if enable_pointcloud else "false"),
    )
    
    static_transform_map_vp_base_origin = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_map_vp_base",
        arguments=[
            "0", "0", "0",  # translation x y z
            "0", "0", "0",  # rotation roll pitch yaw (radians)
            "map",
            "vp_base_origin"
        ],
        output="screen"
    )
    
    static_transform_camera_lens_realsense = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_camera_lens_realsense",
        output="screen",
        arguments=[
            # "0", "0.02", "0.0", # camera_lens
            # "1.57", "-1.57", "0.0", # camera_lens
            "-0.14", "0.28", "0.46",     # mycobot_base
            "-0.78", "0.78", "0.0",     # mycobot_base
            # "camera_lens",
            "mycobot_base",
            "base_link", #(from realsense, i dont know how to rename it)
        ],
        condition=use_realsense_condition,
    )
    
    teleop_control_cpp_node = Node(
        package="teleoperation",
        executable="teleop_control_cpp",
        name="teleop_control_cpp",
        output="screen",
        parameters=[
            teleop_config,
        ],
    )
    
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            os.path.join(rviz_folder, "visionpro.rviz"),
        ],
    )

    joint_state_to_mycobot_node = Node(
        package="teleoperation",
        executable="joint_state_to_mycobot.py",
        name="joint_state_to_mycobot",
        output="screen",
        parameters=[teleop_config],    
    )
    
    inverse_kinematics_node = Node(
        package="teleoperation",
        executable="inverse_kinematics_node",
        name="inverse_kinematics_node",
        output="screen",
        parameters=[teleop_config],
    )
    
    camera_streamer_node = Node(
        package="teleoperation",
        executable="camera_streamer.py",
        name="camera_streamer",
        output="screen",
        emulate_tty=True,
        additional_env={"PYTHONUNBUFFERED": "1"},        
        parameters=[teleop_config],
    )

    vp_streamer_node = Node(
        package="teleoperation",
        executable="vp_streamer.py",
        name="vp_streamer",
        output="screen",
        emulate_tty=True,
        additional_env={"PYTHONUNBUFFERED": "1"},        
        parameters=[
            teleop_config,
            {"viewer": "ar"}, # Options: "None", "ar", "mujoco"
            {"enable_camera": enable_camera},
            {"enable_audio": enable_audio},
            {"camera_mode": camera_mode}, # robot, realsense, both
            {"enable_pointcloud": enable_pointcloud}
        ],
    )
    
    dummy_pointcloud_publisher_node = Node(
        package="teleoperation",
        executable="dummy_pointcloud_publisher.py",
        name="dummy_pointcloud_publisher",
        output="screen",
    )


    nodes = [
        static_transform_camera_lens_realsense,
        realsense_launch,
        realsense_description,
        downsample_node,
        # dummy_pointcloud_publisher_node, 
        
        vp_streamer_node,


        robot_state_publisher_node,
        # listen_real_node,  # disabled: teleop_control now owns the serial port and publishes /joint_states

        static_transform_map_mycobot_base,
        vp_transform_publisher_node,
        static_transform_map_vp_base_origin,

        teleop_control_cpp_node,
        inverse_kinematics_node,
        
        rviz2_node,
                        
        joint_state_to_mycobot_node,
    ]

    return LaunchDescription(nodes)
