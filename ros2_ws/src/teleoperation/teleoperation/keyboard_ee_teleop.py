#!/usr/bin/env python3

import sys
import select
import termios
import tty
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster


HELP_MESSAGE = """\nKeyboard EE teleop for /teleop/ee_target
-------------------------------------------------
Position:
  w/s : +x / -x
  a/d : +y / -y
  r/f : +z / -z

Orientation (roll-pitch-yaw):
  u/o : +roll / -roll
  i/k : +pitch / -pitch
  j/l : +yaw / -yaw

Other:
  space : reset pose
  h     : print this help
  q     : quit
"""


def get_key(timeout=0.1):
    """Read a single keypress from stdin with timeout (seconds)."""
    if select.select([sys.stdin], [], [], timeout)[0]:
        return sys.stdin.read(1)
    return None


class KeyboardEeTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_ee_teleop")

        # Parameters
        # Parameters (loaded from teleoperation/config/teleoperation.yaml or overrides)
        # Frame in which ee_target is expressed; use mycobot_base by default
        self.declare_parameter("frame_id", "mycobot_base")
        self.declare_parameter("step_linear", 0.05)
        self.declare_parameter("step_angular", 0.25)

        self.frame_id = self.get_parameter("frame_id").value
        self.step_linear = float(self.get_parameter("step_linear").value)
        self.step_angular = float(self.get_parameter("step_angular").value)

        self.publisher = self.create_publisher(PoseStamped, "/teleop/ee_target", 10)

        # TF broadcaster for ee_target in map frame
        self.tf_broadcaster = TransformBroadcaster(self)
        self.ee_frame = "ee_target"

        # Internal pose state (meters, radians)
        self.x = 0.15
        self.y = 0.0
        self.z = 0.3
        self.roll = -1.4
        self.pitch = 1.0
        self.yaw = -1.0

        self.get_logger().info("Keyboard EE teleop started. Press 'h' for help.")

    def reset_pose(self):
        self.x = 0.15
        self.y = 0.0
        self.z = 0.25
        self.roll = 1.0
        self.pitch = 1.0
        self.yaw = 1.0
        self.get_logger().info("Pose reset.")

    def handle_key(self, key: str) -> bool:
        """Update internal state from key. Return False to request shutdown."""
        if key is None:
            return True

        if key == "q" or key == "\x03":  # 'q' or Ctrl-C
            return False
        elif key == "h":
            print(HELP_MESSAGE)
        elif key == " ":
            self.reset_pose()
        # Position
        elif key == "w":
            self.x += self.step_linear
        elif key == "s":
            self.x -= self.step_linear
        elif key == "a":
            self.y += self.step_linear
        elif key == "d":
            self.y -= self.step_linear
        elif key == "r":
            self.z += self.step_linear
        elif key == "f":
            self.z -= self.step_linear
        # Orientation (roll, pitch, yaw)
        elif key == "u":
            self.roll += self.step_angular
        elif key == "o":
            self.roll -= self.step_angular
        elif key == "i":
            self.pitch += self.step_angular
        elif key == "k":
            self.pitch -= self.step_angular
        elif key == "j":
            self.yaw += self.step_angular
        elif key == "l":
            self.yaw -= self.step_angular
            
        print(f"Current pose: pos=({self.x:.2f}, {self.y:.2f}, {self.z:.2f}) "
              f"orient=({self.roll:.2f}, {self.pitch:.2f}, {self.yaw:.2f})")

        return True

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.pose.position.x = float(self.x)
        msg.pose.position.y = float(self.y)
        msg.pose.position.z = float(self.z)

        # Convert roll, pitch, yaw (XYZ convention) to quaternion
        q = R.from_euler("xyz", [self.roll, self.pitch, self.yaw]).as_quat()
        msg.pose.orientation.x = float(q[0])
        msg.pose.orientation.y = float(q[1])
        msg.pose.orientation.z = float(q[2])
        msg.pose.orientation.w = float(q[3])

        self.publisher.publish(msg)

        # Also publish as TF in the same frame (typically 'map')
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.ee_frame
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation.x = msg.pose.orientation.x
        t.transform.rotation.y = msg.pose.orientation.y
        t.transform.rotation.z = msg.pose.orientation.z
        t.transform.rotation.w = msg.pose.orientation.w

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    # Try to save terminal settings only if stdin is a TTY.
    settings = None
    if sys.stdin.isatty():
        settings = termios.tcgetattr(sys.stdin.fileno())
    else:
        # When launched via ros2 launch, stdin is often not a TTY.
        # Keyboard input will then not work; log a hint.
        print(
            "[keyboard_ee_teleop] stdin is not a TTY. "
            "For keyboard control, run in a normal terminal, e.g. "
            "`ros2 run teleoperation keyboard_ee_teleop`.",
            file=sys.stderr,
        )

    rclpy.init(args=args)
    node = KeyboardEeTeleop()

    try:
        print(HELP_MESSAGE)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = get_key(timeout=0.1)
            if not node.handle_key(key):
                break
            node.publish_pose()
    finally:
        if settings is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
