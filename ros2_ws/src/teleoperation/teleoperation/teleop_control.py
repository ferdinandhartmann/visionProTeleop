#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Float64

from tf2_ros import Buffer, TransformListener, TransformBroadcaster

# from tf_transformations import quaternion_multiply, quaternion_from_euler
from scipy.spatial.transform import Rotation as R

def quat_multiply(q1, q2):
    return (R.from_quat(q1) * R.from_quat(q2)).as_quat()
    
class VisionProTeleop(Node):
    def __init__(self):
        super().__init__('visionpro_teleop')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishes target pose and gripper command
        self.ee_pub = self.create_publisher(PoseStamped, '/teleop/ee_target', 10)
        self.grip_pub = self.create_publisher(Float64, '/teleop/gripper_target', 10)

        # TF broadcaster for target pose
        self.tf_broadcaster = TransformBroadcaster(self)

        # Parameters
        self.declare_parameter('base_frame', 'map')
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        # pinch thresholds (meters)
        self.PINCH_THRESHOLD = 0.02   # left pinch < ON → start teleop

        # right pinch mapping to gripper 0–1 (meters)
        self.RIGHT_PINCH_MIN = 0.015   # considered fully closed
        self.RIGHT_PINCH_MAX = 0.150   # considered fully open

        self.teleop_enabled = False

        self.timer = self.create_timer(0.01, self.update)

    def get_pos(self, child_frame):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                child_frame,
                Time()
            )
            p = tf.transform.translation
            return np.array([p.x, p.y, p.z], dtype=float)
        except Exception:
            return None

    def get_pose(self, child_frame):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                child_frame,
                Time()
            )
            p = tf.transform.translation
            q = tf.transform.rotation
            pos = np.array([p.x, p.y, p.z], dtype=float)
            ori = np.array([q.x, q.y, q.z, q.w], dtype=float)
            return pos, ori
        except Exception:
            return None, None

    def distance(self, a, b):
        return float(np.linalg.norm(a - b))

    def clamp(self, x, lo, hi):
        return max(lo, min(hi, x))
    

    

    def update(self):
        # ----- LEFT PINCH: teleop mode -----
        left_thumb = self.get_pos('visionpro/left/thumb_4')
        left_index = self.get_pos('visionpro/left/index_4')

        if left_thumb is None or left_index is None:
            # no left hand → disable teleop
            self.teleop_enabled = False
            return

        left_pinch = self.distance(left_thumb, left_index)

        # state machine with hysteresis
        if not self.teleop_enabled and left_pinch < self.PINCH_THRESHOLD:
            self.teleop_enabled = True
            self.get_logger().info('Teleop ENABLED by left pinch')

        elif self.teleop_enabled and left_pinch > self.PINCH_THRESHOLD:
            self.teleop_enabled = False
            self.get_logger().info('Teleop DISABLED by left pinch')

        if not self.teleop_enabled:
            return

        # ----- RIGHT HAND: EE pose and gripper -----
        right_thumb = self.get_pos('visionpro/right/thumb_4')
        right_index = self.get_pos('visionpro/right/index_4')
        wrist_pos, wrist_ori = self.get_pose('visionpro/right/wrist')

        if right_thumb is None or right_index is None or wrist_pos is None:
            return

        # EE position = midpoint of thumb/index
        ee_pos = (right_thumb + right_index) / 2.0

        # Use the wrist orientation directly for the end-effector orientation
        ee_ori = wrist_ori
        
        # 180-degree rotation around the X-axis
        q_flip = R.from_euler('z', 220, degrees=True).as_quat()

        ee_ori = quat_multiply(wrist_ori, q_flip)

        # # Publish EE pose
        # msg = PoseStamped()
        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.header.frame_id = self.base_frame
        # msg.pose.position.x = float(ee_pos[0])
        # msg.pose.position.y = float(ee_pos[1])
        # msg.pose.position.z = float(ee_pos[2])
        # msg.pose.orientation.x = float(ee_ori[0])
        # msg.pose.orientation.y = float(ee_ori[1])
        # msg.pose.orientation.z = float(ee_ori[2])
        # msg.pose.orientation.w = float(ee_ori[3])

        # self.ee_pub.publish(msg)

        # Publish TF for EE pose
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = 'ee_target'
        t.transform.translation.x = float(ee_pos[0])
        t.transform.translation.y = float(ee_pos[1])
        t.transform.translation.z = float(ee_pos[2])
        t.transform.rotation.x = float(ee_ori[0])
        t.transform.rotation.y = float(ee_ori[1])
        t.transform.rotation.z = float(ee_ori[2])
        t.transform.rotation.w = float(ee_ori[3])

        self.tf_broadcaster.sendTransform(t)
        
        # Right pinch → gripper
        right_pinch = self.distance(right_thumb, right_index)
        d_clamped = self.clamp(right_pinch, self.RIGHT_PINCH_MIN, self.RIGHT_PINCH_MAX)
        open_ratio = (d_clamped - self.RIGHT_PINCH_MIN) / (self.RIGHT_PINCH_MAX - self.RIGHT_PINCH_MIN)

        grip_msg = Float64()
        grip_msg.data = float(open_ratio)
        self.grip_pub.publish(grip_msg)
        
        if not hasattr(self, '_last_log_time') or (self.get_clock().now() - self._last_log_time).nanoseconds * 1e-9 > 0.1:
            self.get_logger().info(f'Gripper command: {grip_msg.data:.3f}')
            self._last_log_time = self.get_clock().now()
        else:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = VisionProTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
