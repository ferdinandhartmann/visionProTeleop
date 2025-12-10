#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

from tf2_ros import Buffer, TransformListener, TransformBroadcaster

# from tf_transformations import quaternion_multiply, quaternion_from_euler
from scipy.spatial.transform import Rotation as R

from tf2_ros import StaticTransformBroadcaster

from pathlib import Path
import fcntl
import os


def mat_from_tf(tf):
    t = tf.transform.translation
    q = tf.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w

    R = np.array([
        [1-2*y*y-2*z*z, 2*x*y-2*z*w,   2*x*z+2*y*w],
        [2*x*y+2*z*w,   1-2*x*x-2*z*z, 2*y*z-2*x*w],
        [2*x*z-2*y*w,   2*y*z+2*x*w,   1-2*x*x-2*y*y],
    ])

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def quat_multiply(q1, q2):
    return (R.from_quat(q1) * R.from_quat(q2)).as_quat()
    
class VisionProTeleop(Node):
    def __init__(self):
        super().__init__('visionpro_teleop')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishes target pose and gripper command
        self.ee_target_pub = self.create_publisher(PoseStamped, '/teleop/ee_target', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # TF broadcaster for target pose
        self.tf_broadcaster = TransformBroadcaster(self)

        # Parameters
        self.declare_parameter('vp_base_frame', 'vp_base')
        self.declare_parameter('gripper_base_frame', 'gripper_base')
        
        self.vp_base_frame = self.get_parameter('vp_base_frame').get_parameter_value().string_value
        self.gripper_base_frame = self.get_parameter('gripper_base_frame').get_parameter_value().string_value
        
        # pinch thresholds (meters) as parameters
        self.declare_parameter('pinch_threshold', 0.02)   # left pinch < ON → start teleop
        self.declare_parameter('right_pinch_min', 0.015)  # considered fully closed
        self.declare_parameter('right_pinch_max', 0.150)  # considered fully open

        self.pinch_threshold = self.get_parameter('pinch_threshold').get_parameter_value().double_value
        self.right_pinch_min = self.get_parameter('right_pinch_min').get_parameter_value().double_value
        self.right_pinch_max = self.get_parameter('right_pinch_max').get_parameter_value().double_value

        self.teleop_enabled = False

        # -------- Offset + state --------
        self.offset_T = None
        self.ee_pos_offset = np.zeros(3)
        self.ee_ori_offset = np.zeros(4)
        self.offset_pos = None
        self.offset_rot = None
        
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        self.publish_vp_base_calibration(np.eye(4))


    def get_pos(self, child_frame):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.vp_base_frame,
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
                self.vp_base_frame,
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
    
    
    def publish_vp_base_calibration(self, T):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "vp_base_origin"
        t.child_frame_id = "vp_base"

        t.transform.translation.x = float(T[0, 3])
        t.transform.translation.y = float(T[1, 3])
        t.transform.translation.z = float(T[2, 3])

        q = R.from_matrix(T[:3, :3]).as_quat()
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.static_tf_broadcaster.sendTransform(t)
        self.get_logger().info('Published vp_base calibration transform')
        
    def publish_ee_target_tf(self, ee_pos, ee_ori, child_frame='ee_target', header_frame=None):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = header_frame if header_frame is not None else self.vp_base_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = float(ee_pos[0])
        t.transform.translation.y = float(ee_pos[1])
        t.transform.translation.z = float(ee_pos[2])
        t.transform.rotation.x = float(ee_ori[0])
        t.transform.rotation.y = float(ee_ori[1])
        t.transform.rotation.z = float(ee_ori[2])
        t.transform.rotation.w = float(ee_ori[3])

        self.tf_broadcaster.sendTransform(t)
    

    def update(self):
        
        # ----- LEFT PINCH: teleop mode -----
        left_thumb = self.get_pos('visionpro/left/thumb_4')
        left_index = self.get_pos('visionpro/left/index_4')

        if left_thumb is None or left_index is None:
            # no left hand → disable teleop
            self.teleop_enabled = False
            return

        left_pinch = self.distance(left_thumb, left_index)

        # ---------- Teleop state ----------
        just_enabled = False
        just_disabled = False

        if not self.teleop_enabled and left_pinch < self.PINCH_THRESHOLD:
            self.teleop_enabled = True
            just_enabled = True
            self.get_logger().info('Teleop ENABLED (left pinch)')

        elif self.teleop_enabled and left_pinch > self.PINCH_THRESHOLD:
            self.teleop_enabled = False
            just_disabled = True
            self.publish_vp_base_calibration(np.eye(4))
            self.get_logger().info('Teleop DISABLED (left pinch)')

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

        # Publish TF for EE pose
        self.publish_ee_target_tf(ee_pos, ee_ori, child_frame='ee_target')
    
        # Right pinch → gripper
        right_pinch = self.distance(right_thumb, right_index)
        d_clamped = self.clamp(right_pinch, self.RIGHT_PINCH_MIN, self.RIGHT_PINCH_MAX)
        open_ratio = int((d_clamped - self.RIGHT_PINCH_MIN) / (self.RIGHT_PINCH_MAX - self.RIGHT_PINCH_MIN) * 100)
        
        if not hasattr(self, '_last_log_time') or (self.get_clock().now() - self._last_log_time).nanoseconds * 1e-9 > 0.5:
            self.get_logger().info(f'Gripper command: {open_ratio}')
            self._last_log_time = self.get_clock().now()
        
        
        # ---- Build HAND transform in MAP frame ----
        try:
            tf_h = self.tf_buffer.lookup_transform(
                "map",
                "ee_target",
                Time()
            )
            T_hand_map = mat_from_tf(tf_h)
        except Exception:
            return

        # ---- Capture offset on teleop enable ----
        if just_enabled:
            try:
                tf_g = self.tf_buffer.lookup_transform(
                    "map",
                    self.gripper_base_frame,
                    Time()
                )
                T_gripper = mat_from_tf(tf_g)

                # offset = hand⁻¹ × gripper
                T_hand_map_inv = np.linalg.inv(T_hand_map)
                self.offset_T = T_hand_map_inv @ T_gripper
                
                self.offset_pos = T_gripper[:3, 3] - T_hand_map[:3, 3]
                R_hand = R.from_matrix(T_hand_map[:3, :3])
                R_grip = R.from_matrix(T_gripper[:3, :3])
                self.offset_rot = R_hand.inv() * R_grip
                
                self.get_logger().info(f"Teleop offset captured (gripper frame: {self.gripper_base_frame})")
            except Exception as e:
                self.offset_T = None
                self.get_logger().warn(f"Offset capture failed for frame '{self.gripper_base_frame}': {e}")


        # ---- Apply offset while teleop active ----
        if not hasattr(self, "_last_5hz_time"):
            self._last_5hz_time = self.get_clock().now()

        elapsed = (self.get_clock().now() - self._last_5hz_time).nanoseconds * 1e-9

        if elapsed >= 0.5:
            if self.teleop_enabled and self.offset_T is not None:
                                
                T_target_pos = T_hand_map[:3, 3] + self.offset_pos
                T_target_rot = R.from_matrix(T_hand_map[:3, :3]) * self.offset_rot
                T_target = np.eye(4)
                T_target[:3, 3] = T_target_pos
                T_target[:3, :3] = T_target_rot.as_matrix()
                
                ee_pos_offset = T_target[:3, 3]
                rot_offset = R.from_matrix(T_target[:3, :3])
                # Use a valid quaternion (identity or true offset orientation).
                # Here we publish the actual offset orientation.
                ee_ori_offset = rot_offset.as_quat()
                # ee_ori_offset = np.array([0.0, 0.0, 0.0, 1.0])

                # Transform ee_target_offset (in map) to mycobot_base frame
                try:
                    tf_gbase = self.tf_buffer.lookup_transform(
                        "mycobot_base",
                        "map",
                        Time()
                    )
                    T_map_to_gbase = mat_from_tf(tf_gbase)
                    T_ee_target_offset_map = np.eye(4)
                    T_ee_target_offset_map[:3, :3] = R.from_quat(ee_ori_offset).as_matrix()
                    T_ee_target_offset_map[:3, 3] = ee_pos_offset

                    # Transform to gripper_base frame
                    T_ee_target_offset_gbase = T_map_to_gbase @ T_ee_target_offset_map
                    ee_pos_gbase = T_ee_target_offset_gbase[:3, 3]
                    ee_ori_gbase = R.from_matrix(T_ee_target_offset_gbase[:3, :3]).as_quat()

                    self.publish_ee_target_tf(ee_pos_gbase, ee_ori_gbase, child_frame='ee_target_offset_gbase', header_frame='mycobot_base')
                    
                    # Publish ee_target_pub in mycobot_base frame
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = 'mycobot_base'
                    pose_msg.pose.position.x = float(ee_pos_gbase[0])
                    pose_msg.pose.position.y = float(ee_pos_gbase[1])
                    pose_msg.pose.position.z = float(ee_pos_gbase[2])
                    pose_msg.pose.orientation.x = float(ee_ori_gbase[0])
                    pose_msg.pose.orientation.y = float(ee_ori_gbase[1])
                    pose_msg.pose.orientation.z = float(ee_ori_gbase[2])
                    pose_msg.pose.orientation.w = float(ee_ori_gbase[3])
                    self.ee_target_pub.publish(pose_msg)
                    
                    self.get_logger().info(f"Published ee_target_offset in mycobot_base frame")
                
                except Exception as e:
                    self.get_logger().warn(f"Failed to transform ee_target_offset to mycobot_base: {e}")
                
                
            if just_disabled:
                self.offset_T = None
                self.get_logger().info("Teleop offset cleared")

            self._last_5hz_time = self.get_clock().now()



def main(args=None):
    rclpy.init(args=args)
    node = VisionProTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
