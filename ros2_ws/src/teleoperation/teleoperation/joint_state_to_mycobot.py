#!/usr/bin/env python3

import math
import time
import fcntl
import os

import rclpy
from rclpy.node import Node
from rclpy.parameter import ParameterValue
from rcl_interfaces.msg import ParameterType
from sensor_msgs.msg import JointState

from pymycobot import MyCobot280
from pymycobot.error import MyCobot280DataException


LOCK_FILE = "/tmp/mycobot_lock"


class JointStateToMyCobot(Node):
    """Bridge node: /joint_states -> MyCobot280 send_angles + gripper."""

    def __init__(self):
        super().__init__("joint_state_to_mycobot")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("joint_state_topic", "/joint_states_mycobot")
        self.declare_parameter("speed", 40)  # joint speed 1-100
        self.declare_parameter("gripper_speed", 75)  # gripper speed 1-100
        # 6 joint angles in degrees; explicitly declare as INTEGER_ARRAY
        self.declare_parameter(
            "initial_angles_deg",
            ParameterValue(
                type=ParameterType.PARAMETER_INTEGER_ARRAY,
                integer_array_value=[],
            ),
        )
        # Speed used only for initial move on startup
        self.declare_parameter("initial_move_speed", 15)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.joint_state_topic = (self.get_parameter("joint_state_topic").get_parameter_value().string_value)
        self.speed = int(self.get_parameter("speed").value)
        self.gripper_speed = int(self.get_parameter("gripper_speed").value)
        self.initial_angles_deg = list(self.get_parameter("initial_angles_deg").value)
        self.initial_move_speed = int(self.get_parameter("initial_move_speed").value)

        self.get_logger().info(f"Connecting to MyCobot on {port} @ {baud} baud")

        # Connect to MyCobot
        self.mc = MyCobot280(port, baud)
        time.sleep(0.1)
        try:
            self.mc.set_fresh_mode(1)
        except Exception as e:
            self.get_logger().warn(f"Failed to set fresh mode: {e}")

        # If an initial pose is configured, move to it once on startup
        if self.initial_angles_deg:
            if len(self.initial_angles_deg) != 6:
                self.get_logger().warn(
                    "Parameter 'initial_angles_deg' must contain exactly 6 values; "
                    f"got {len(self.initial_angles_deg)}. Skipping initial move."
                )
            else:
                self.get_logger().info(
                    "Moving MyCobot to initial joint angles from parameters: "
                    f"{self.initial_angles_deg} at speed {self.initial_move_speed}"
                )
                try:
                    fd = self.acquire_lock()
                    try:
                        try:
                            self.mc.send_angles(
                                self.initial_angles_deg,
                                max(1, min(100, self.initial_move_speed)),
                            )
                        except MyCobot280DataException as e:
                            self.get_logger().warn(
                                f"send_angles error during initial move: {e}"
                            )
                    finally:
                        self.release_lock(fd)
                except Exception as e:
                    self.get_logger().error(
                        f"Failed to perform initial move to configured pose: {e}"
                    )

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
            
        self.get_logger().info(f"Trying to send angles: {angles_deg} and gripper percent: {gripper_percent}")

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
                        self.get_logger().debug(f"Sent gripper value: {clamped} at speed {self.gripper_speed}")
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
