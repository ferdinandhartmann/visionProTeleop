#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Float64MultiArray


class MyCobotAdapter(Node):
    def __init__(self):
        super().__init__('mycobot_adapter')

        self.ee_sub = self.create_subscription(
            PoseStamped, '/teleop/ee_target', self.ee_cb, 10
        )
        self.grip_sub = self.create_subscription(
            Float64, '/teleop/gripper_target', self.grip_cb, 10
        )

        # Generic command topics – adjust to your mycobot_ros2 driver
        self.coords_pub = self.create_publisher(
            Float64MultiArray, '/mycobot/coords_cmd', 10
        )
        self.grip_pub = self.create_publisher(
            Float64, '/mycobot/gripper_cmd', 10
        )

        self.get_logger().info('MyCobotAdapter started. Connect /mycobot/coords_cmd to your driver.')

    def quat_to_rpy(self, x, y, z, w):
        # Standard quaternion → roll, pitch, yaw (radians)
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def ee_cb(self, msg: PoseStamped):
        # Convert Pose [m, quaternion] → [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
        x_m = msg.pose.position.x
        y_m = msg.pose.position.y
        z_m = msg.pose.position.z

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        roll, pitch, yaw = self.quat_to_rpy(qx, qy, qz, qw)

        x_mm = x_m * 1000.0
        y_mm = y_m * 1000.0
        z_mm = z_m * 1000.0

        rx_deg = math.degrees(roll)
        ry_deg = math.degrees(pitch)
        rz_deg = math.degrees(yaw)

        arr = Float64MultiArray()
        arr.data = [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]

        self.coords_pub.publish(arr)

    def grip_cb(self, msg: Float64):
        # msg.data is 0.0 (closed) to 1.0 (open)
        # You may remap this to servo angles, etc., in your MyCobot driver.
        self.grip_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MyCobotAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
