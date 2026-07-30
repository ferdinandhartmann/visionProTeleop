"""Launch Vision Pro Cartesian teleoperation for an FR3 with a Robotiq 2F-85."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    teleop_config = PathJoinSubstitution(
        [FindPackageShare("teleoperation"), "config", "franka_teleoperation.yaml"]
    )
    model_path = PathJoinSubstitution(
        [
            FindPackageShare("robot_description"),
            "franka_mujoco",
            "fr3_robotiq_2f85.xml",
        ]
    )

    franka_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("factr_utils"),
                    "launch",
                    "franka_robotiqgripperoption.launch.py",
                ]
            )
        ),
        launch_arguments={
            "arm_id": "fr3",
            "robot_ip": LaunchConfiguration("robot_ip"),
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "load_gripper": "true",
            "ee_id": "robotiq_gripper_2f85",
            "tcp_xyz": "0 0 0.174",
            "rpy_ee": "0 0 0",
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_robot")),
    )

    cartesian_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="cartesian_impedance_controller_spawner",
        arguments=[
            "cartesian_impedance_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_controller")),
    )

    robotiq_gripper = Node(
        package="factr_teleop",
        executable="robotiq_gripper_node",
        name="robotiq_gripper_node",
        output="screen",
        emulate_tty=True,
        parameters=[teleop_config],
        condition=IfCondition(LaunchConfiguration("start_gripper")),
    )

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("cameras"), "launch", "launch_realsense.launch.py"]
            )
        ),
        launch_arguments={
            "camera_names": "top",
            "camera_serial_numbers": LaunchConfiguration("camera_serial"),
            "color_profiles": LaunchConfiguration("camera_profile"),
            "depth_profiles": LaunchConfiguration("camera_profile"),
            "depth": "true",
            "align_depth": "true",
            "enable_sync": "true",
            "pointcloud": "true",
            "decimation_filter_value": "4",
            "tf_sliders": "false",
            "rviz": "false",
            "franka": "false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_camera")),
    )

    vp_base_calibration = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="vp_base_calibration",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "-2.4434609528",
            "--frame-id",
            "vp_base_origin",
            "--child-frame-id",
            "vp_base",
        ],
        output="screen",
    )

    vp_origin = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="fr3_to_vp_base_origin",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "fr3_link0",
            "--child-frame-id",
            "vp_base_origin",
        ],
        output="screen",
    )

    vp_transforms = Node(
        package="teleoperation",
        executable="vp_transform_publisher.py",
        name="vp_transform_publisher",
        output="screen",
        emulate_tty=True,
        additional_env={"PYTHONUNBUFFERED": "1"},
        parameters=[
            teleop_config,
            {"visionpro_ip": LaunchConfiguration("visionpro_ip")},
        ],
    )

    teleop = Node(
        package="teleoperation",
        executable="franka_visionpro_teleop",
        name="franka_visionpro_teleop",
        output="screen",
        parameters=[teleop_config],
    )

    vp_streamer = Node(
        package="teleoperation",
        executable="vp_streamer.py",
        name="vp_streamer",
        output="screen",
        emulate_tty=True,
        additional_env={"PYTHONUNBUFFERED": "1"},
        parameters=[
            teleop_config,
            {
                "viewer": "ar",
                "visionpro_ip": LaunchConfiguration("visionpro_ip"),
                "xml_path": model_path,
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="franka_visionpro_rviz",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("hand_teleop"), "rviz", "hand_teleop.rviz"]
            ),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_ip",
                default_value="172.168.0.1",
                description="Franka robot hostname or IP address.",
            ),
            DeclareLaunchArgument(
                "visionpro_ip",
                default_value="192.168.50.153",
                description="Vision Pro IP address.",
            ),
            DeclareLaunchArgument(
                "camera_serial",
                default_value="950122070179",
                description="Top RealSense serial number.",
            ),
            DeclareLaunchArgument(
                "camera_profile",
                default_value="1280x720x30",
                description="RealSense color and depth profile.",
            ),
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="false",
                description="Use Franka fake hardware.",
            ),
            DeclareLaunchArgument("start_robot", default_value="true"),
            DeclareLaunchArgument("start_controller", default_value="true"),
            DeclareLaunchArgument("start_gripper", default_value="true"),
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            franka_bringup,
            cartesian_controller,
            robotiq_gripper,
            realsense,
            vp_origin,
            vp_base_calibration,
            vp_transforms,
            teleop,
            vp_streamer,
            rviz,
        ]
    )
