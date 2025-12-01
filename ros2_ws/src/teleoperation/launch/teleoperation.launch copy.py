from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import sys
from launch.actions import DeclareLaunchArgument
import launch
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

model_launch_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            get_package_share_directory("mycobot_description"),
            "urdf/mycobot_280_m5/mycobot_280_m5.urdf"
        )
    )

rvizconfig_launch_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=os.path.join(
            get_package_share_directory("mycobot_280"),
            "config/mycobot.rviz"
        )
    )

robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    
def generate_launch_description():

    # ======================================================
    SIMULATION_MODE = "rviz"    # ← CHANGE THIS LINE ONLY
    # ======================================================

    config_folder = os.path.join(
        get_package_share_directory("teleoperation"), "config"
    )

    transform_publisher_node = Node(
        package="teleoperation",
        executable="transform_publisher",
        name="transform_publisher",
        output="screen",
    )

    mycobot_adapter_node = Node(
        package="teleoperation",
        executable="mycobot_adapter",
        name="mycobot_adapter",
        output="screen"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            os.path.join(config_folder, "visionpro.rviz"),
        ],
    )


    mycobot_sim_rviz = None

    if SIMULATION_MODE == "rviz":
        # mycobot_sim_rviz = Node(
        #     package="mycobot_ros2",
        #     executable="mycobot_rviz",
        #     name="mycobot_rviz",
        #     output="screen"
        # )

        robot_state_publisher_node = Node(
            name="robot_state_publisher",
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{'robot_description': robot_description}]
        )    
    


    # Build node list dynamically
    nodes = [
        rviz2_node,
        transform_publisher_node,
        mycobot_adapter_node,
        model_launch_arg,
        rvizconfig_launch_arg, 
    ]

    if mycobot_sim_rviz:
        nodes.append(mycobot_sim_rviz)
        nodes.append(robot_state_publisher_node)


    return LaunchDescription(nodes)
