from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    pkg_dir = Path(get_package_share_directory("teleoperation"))
    tunnel_script = pkg_dir / "scripts" / "start_sam_tunnel.sh"

    # Start SSH tunnel first
    sam_tunnel = ExecuteProcess(
        cmd=["bash", str(tunnel_script)],
        output="screen"
    )

    sam_client = TimerAction(
        period=1.5,
        actions=[
            Node(
                package="teleoperation",
                executable="sam_client.py",
                name="sam_client",
                output="screen",
                parameters=[str(pkg_dir / "config" / "teleoperation.yaml")],
            )
        ]
    )

    return LaunchDescription([
        sam_tunnel, 
        sam_client
    ])
