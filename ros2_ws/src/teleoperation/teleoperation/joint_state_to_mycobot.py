#!/usr/bin/env python3

import math
import time
import fcntl
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from pymycobot import MyCobot280
from pymycobot.error import MyCobot280DataException


LOCK_FILE = "/tmp/mycobot_lock"


class JointStateToMyCobot(Node):
    """Bridge node: /joint_states -> MyCobot280 send_angles + gripper."""

    def __init__(self):
        super().__init__("joint_state_to_mycobot")

        # Parameters for hardware connection
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("speed", 40)  # joint speed 1-100
        self.declare_parameter("gripper_speed", 75)  # gripper speed 1-100

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.joint_state_topic = (
            self.get_parameter("joint_state_topic").get_parameter_value().string_value
        )
        self.speed = int(self.get_parameter("speed").value)
        self.gripper_speed = int(self.get_parameter("gripper_speed").value)

        self.get_logger().info(f"Connecting to MyCobot on {port} @ {baud} baud")

        # Connect to MyCobot
        self.mc = MyCobot280(port, baud)
        time.sleep(0.1)
        try:
            self.mc.set_fresh_mode(1)
        except Exception as e:
            self.get_logger().warn(f"Failed to set fresh mode: {e}")

        # Gripper joint limits from inverse_kinematics.cpp
        self.gripper_lower_limit = -0.74  # closed
        self.gripper_upper_limit = 0.15   # open

        # Subscribe to joint states
        self.subscription = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10,
        )

        self.get_logger().info(
            f"Subscribed to '{self.joint_state_topic}' and ready to drive MyCobot."
        )

    # ----- Shared lock helpers (match teleop_control.py) -----
    def acquire_lock(self):
        fd = os.open(LOCK_FILE, os.O_RDWR | os.O_CREAT)
        fcntl.flock(fd, fcntl.LOCK_EX)
        return fd

    def release_lock(self, fd):
        fcntl.flock(fd, fcntl.LOCK_UN)
        os.close(fd)

    # ----- Core callback -----
    def joint_state_callback(self, msg: JointState):
        # Need at least 6 joints for arm
        if not msg.position or len(msg.position) < 6:
            self.get_logger().warn(
                f"Received JointState with insufficient positions: {len(msg.position) if msg.position else 0}"
            )
            return

        # First 6 positions: arm joints in radians -> degrees
        arm_positions_rad = list(msg.position[:6])
        angles_deg = [math.degrees(p) for p in arm_positions_rad]

        # Optional 7th value: gripper joint (radians)
        gripper_percent = None
        if len(msg.position) >= 7:
            gripper_joint = msg.position[6]
            gripper_percent = self.gripper_joint_to_percent(gripper_joint)

        try:
            fd = self.acquire_lock()
            try:
                # Send arm joint angles
                try:
                    self.mc.send_angles(angles_deg, self.speed)
                except MyCobot280DataException as e:
                    self.get_logger().warn(f"send_angles error: {e}")

                # Send gripper if present
                if gripper_percent is not None:
                    clamped = max(0, min(100, int(round(gripper_percent))))
                    try:
                        self.mc.set_gripper_value(clamped, self.gripper_speed)
                    except MyCobot280DataException as e:
                        self.get_logger().warn(f"set_gripper_value error: {e}")
            finally:
                self.release_lock(fd)
        except Exception as e:
            self.get_logger().error(f"Failed to acquire MyCobot lock: {e}")

    def gripper_joint_to_percent(self, joint_value: float) -> float:
        """Map gripper joint angle [lower, upper] -> [0, 100]."""
        lo = self.gripper_lower_limit
        hi = self.gripper_upper_limit
        if hi == lo:
            return 0.0
        # Linear mapping: lo -> 0, hi -> 100
        ratio = (joint_value - lo) / (hi - lo)
        percent = ratio * 100.0
        # Clamp to [0, 100]
        return max(0.0, min(100.0, percent))


def main(args=None):
    rclpy.init(args=args)
    node = JointStateToMyCobot()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
