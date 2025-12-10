from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import sys
from launch.actions import DeclareLaunchArgument
import launch
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

    
def generate_launch_description():

    config_folder = os.path.join(
        get_package_share_directory("inverse_kinematics"), "config"
    )
    
    model_launch_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(
            get_package_share_directory("robot_description"),
            "urdf/mycobot_280_m5/mycobot_280_m5_camera_flange_adaptive_gripper.urdf"
        )
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port to use'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud',
        default_value='115200',
        description='Baud rate to use'
    )
    
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)


    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )
    
    gripper_joint_state = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="gripper_joint_state_publisher",
        parameters=[{
            "publish_default_positions": True,
            "rate": 30.0,
            "use_gui": False,
        }]
    )

    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
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
    
    static_transform_map_g_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_map_g_base",
        arguments=[
            "0", "0", "0",  # translation x y z
            "0", "0", "0",  # rotation roll pitch yaw (radians)
            "map",
            "g_base"
        ],
        output="screen"
    )
    
    transform_publisher_node = Node(
        package="teleoperation",
        executable="transform_publisher",
        name="transform_publisher",
        output="screen",
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

    teleop_control_node = Node(
        package="teleoperation",
        executable="teleop_control",
        name="teleop_control",
        output="screen",
        parameters=[
            {'port': LaunchConfiguration('port')},
            {'baud': LaunchConfiguration('baud')},
        ],
    )

    teleop_bridge_node = Node(
        package="teleoperation",
        executable="teleop_coords_bridge",
        name="teleop_coords_bridge",
        output="screen"
    )
    
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            os.path.join(config_folder, "inverse_kin.rviz"),
        ],
    )
    
    inverse_kinematics_node = Node(
        package="inverse_kinematics",
        executable="inverse_kinematics_node",
        name="inverse_kinematics_node",
        output="screen",
    )
    
    keyboard_ee_teleop_node = Node(
        package="teleoperation",
        executable="keyboard_ee_teleop",
        name="keyboard_ee_teleop",
        output="screen",
    )


    nodes = [
        model_launch_arg,
        # serial_port_arg,
        # baud_rate_arg,
        
        robot_state_publisher_node,
        # gripper_joint_state,
        # joint_state_publisher_node,
        # listen_real_node,  # disabled: teleop_control now owns the serial port and publishes /joint_states

        static_transform_map_g_base,
        # transform_publisher_node,
        # static_transform_map_vp_base_origin,

        # teleop_control_node,
        # teleop_bridge_node,

        rviz2_node,
        
        inverse_kinematics_node
        # keyboard_ee_teleop_node
    ]

    return LaunchDescription(nodes)
