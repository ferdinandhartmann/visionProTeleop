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
        # Adaptive speed settings
        self.declare_parameter("adaptive_speed", False)
        self.declare_parameter("adaptive_speed_low_speed", 20)
        self.declare_parameter("adaptive_speed_high_speed", 80)
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
        self.adaptive_speed = bool(self.get_parameter("adaptive_speed").value)
        self.adaptive_speed_low = int(self.get_parameter("adaptive_speed_low_speed").value)
        self.adaptive_speed_high = int(self.get_parameter("adaptive_speed_high_speed").value)
        self.initial_angles_deg = list(self.get_parameter("initial_angles_deg").value)
        self.initial_move_speed = int(self.get_parameter("initial_move_speed").value)

        self.get_logger().info(f"Connecting to MyCobot on {port} @ {baud} baud")

        self.prev_angles_deg = None
        self.prev_gripper_percent = None

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
        gripper_percent = msg.position[6]
            
        # self.get_logger().info(f"Trying to send angles: {[round(a, 2) for a in angles_deg]} and gripper percent: {round(gripper_percent, 2) if gripper_percent is not None else gripper_percent}")
        

        angles_changed = (
            self.prev_angles_deg is None or
            any(abs(a - b) > 1e-2 for a, b in zip(angles_deg, self.prev_angles_deg))
        )
        gripper_changed = (
            self.prev_gripper_percent is None or
            abs(gripper_percent - self.prev_gripper_percent) > 1e-2
        )

        send_speed = self.speed
        if angles_changed:
            # Choose speed based on angle change if adaptive speed is enabled
            if self.adaptive_speed and self.prev_angles_deg is not None:
                max_delta = max(abs(a - b) for a, b in zip(angles_deg, self.prev_angles_deg))
                # Interpolate speed between adaptive_speed_low and adaptive_speed_high
                angle_change_upper_bouond = 4.0  # degrees
                angle_change_lower_bound = 2.0  # degrees
                if max_delta <= angle_change_upper_bouond:
                    send_speed = self.adaptive_speed_low
                elif max_delta >= angle_change_lower_bound:
                    send_speed = self.adaptive_speed_high
                else:
                    # Linear interpolation between low and high speed
                    ratio = (max_delta - angle_change_lower_bound) / (angle_change_upper_bouond - angle_change_lower_bound)
                    send_speed = int(self.adaptive_speed_low + ratio * (self.adaptive_speed_high - self.adaptive_speed_low))
            self.mc.send_angles(angles_deg, send_speed, _async=True)
            self.prev_angles_deg = angles_deg.copy()
        if gripper_changed:
            self.mc.set_gripper_value(int(gripper_percent), self.gripper_speed)
            self.prev_gripper_percent = gripper_percent
        if angles_changed or gripper_changed:
            self.get_logger().info(
                f"Sent gripper and/or joint values. Gripper: {gripper_percent} at speed {self.gripper_speed}, angle speed: {send_speed}"
            )


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
