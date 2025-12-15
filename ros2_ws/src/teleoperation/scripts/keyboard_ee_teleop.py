#!/usr/bin/env python3

import sys
import select
import termios
import tty
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster
from teleoperation.msg import TeleopTarget


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

Gripper (0-100%):
    z/x : close / open

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

        self.frame_id = "mycobot_base"
        self.step_linear = 0.025
        self.step_angular = 0.25

        # Gripper state in percent (0 = closed, 100 = open)
        self.gripper_percent = 100

        # Publisher for combined pose + gripper target
        self.publisher = self.create_publisher(TeleopTarget, "/teleop/ee_target", 10)

        # TF broadcaster for ee_target in map frame
        self.tf_broadcaster = TransformBroadcaster(self)
        self.ee_frame = "ee_target"

        # Internal pose state (meters, radians)
        self.x = 0.2
        self.y = -0.05
        self.z = 0.35
        self.roll = -0.15
        self.pitch = 0.0
        self.yaw = 0.0

        self.get_logger().info("Keyboard EE teleop started. Press 'h' for help.")

    def reset_pose(self):
        self.x = 0.2
        self.y = -0.05
        self.z = 0.35
        self.roll = -0.15
        self.pitch = 0.0
        self.yaw = 0.0
        self.gripper_percent = 100
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
        # Gripper control
        elif key == "z":
            # Close gripper
            self.gripper_percent = max(0, self.gripper_percent - 10)
        elif key == "x":
            # Open gripper
            self.gripper_percent = min(100, self.gripper_percent + 10)
            
        print(f"Current pose: pos=({self.x:.2f}, {self.y:.2f}, {self.z:.2f}) "
              f"orient=({self.roll:.2f}, {self.pitch:.2f}, {self.yaw:.2f}), "
              f"gripper={self.gripper_percent:d}%")

        return True

    def publish_pose(self):
        msg = TeleopTarget()
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.header.frame_id = self.frame_id

        msg.pose.pose.position.x = float(self.x)
        msg.pose.pose.position.y = float(self.y)
        msg.pose.pose.position.z = float(self.z)

        # Convert roll, pitch, yaw (XYZ convention) to quaternion
        q = R.from_euler("xyz", [self.roll, self.pitch, self.yaw]).as_quat()
        msg.pose.pose.orientation.x = float(q[0])
        msg.pose.pose.orientation.y = float(q[1])
        msg.pose.pose.orientation.z = float(q[2])
        msg.pose.pose.orientation.w = float(q[3])

        # Gripper command in percent
        msg.gripper = int(self.gripper_percent)

        self.publisher.publish(msg)

        # Also publish as TF in the same frame (typically 'map')
        t = TransformStamped()
        t.header.stamp = msg.pose.header.stamp
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.ee_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

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
