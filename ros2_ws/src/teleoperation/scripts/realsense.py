#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import pyrealsense2 as rs
import numpy as np
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from sensor_msgs.msg import CameraInfo


class RealSensePublisher(Node):
    def __init__(self):
        super().__init__('realsense_publisher')

        self.bridge = CvBridge()

        self.color_pub = self.create_publisher(
            Image, '/camera/color/image_raw', 10
        )
        self.depth_pub = self.create_publisher(
            Image, '/camera/depth/image_raw', 10
        )
        
        self.depth_vis_pub = self.create_publisher(
            Image, '/camera/depth/image_vis', 10
        )
        
        self.color_info_pub = self.create_publisher(
            CameraInfo, '/camera/color/camera_info', 10
        )

        self.depth_info_pub = self.create_publisher(
            CameraInfo, '/camera/depth/camera_info', 10
        )



        # Configure RealSense
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)

        self.timer = self.create_timer(1.0 / 30.0, self.publish_frames)
        
        profile = self.pipeline.get_active_profile()

        color_stream = profile.get_stream(rs.stream.color)
        depth_stream = profile.get_stream(rs.stream.depth)

        self.color_intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        self.depth_intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()


        self.get_logger().info("RealSense image publisher started")

    def publish_frames(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        color_msg = self.bridge.cv2_to_imgmsg(
            color_image, encoding='bgr8'
        )
        depth_msg = self.bridge.cv2_to_imgmsg(
            depth_image, encoding='16UC1'
        )

        color_msg.header.stamp = self.get_clock().now().to_msg()
        depth_msg.header.stamp = color_msg.header.stamp
        color_msg.header.frame_id = 'camera_lens'
        depth_msg.header.frame_id = 'camera_lens'
        
        
        
        depth_image = np.asanyarray(depth_frame.get_data())

        depth_m = depth_image.astype(np.float32) * 0.001  # mm → meters
        depth_m = np.clip(depth_m, 0.3, 5.0)

        depth_vis = cv2.normalize(
            depth_m, None, 0, 255, cv2.NORM_MINMAX
        ).astype(np.uint8)

        depth_vis_msg = self.bridge.cv2_to_imgmsg(
            depth_vis, encoding='mono8'
        )

        depth_vis_msg.header = depth_msg.header
        self.depth_vis_pub.publish(depth_vis_msg)
        

        self.color_pub.publish(color_msg)
        self.depth_pub.publish(depth_msg)
        


        
        
        color_info_msg = self.make_camera_info(
            self.color_intrinsics, 'camera_lens'
        )

        depth_info_msg = self.make_camera_info(
            self.depth_intrinsics, 'camera_lens'
        )
        
        color_info_msg.header.stamp = color_msg.header.stamp
        depth_info_msg.header.stamp = depth_msg.header.stamp

        self.color_info_pub.publish(color_info_msg)
        self.depth_info_pub.publish(depth_info_msg)

        
    def make_camera_info(self, intrinsics, frame_id):
        msg = CameraInfo()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        msg.width = intrinsics.width
        msg.height = intrinsics.height

        fx = intrinsics.fx
        fy = intrinsics.fy
        cx = intrinsics.ppx
        cy = intrinsics.ppy

        msg.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0
        ]

        msg.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        msg.d = list(intrinsics.coeffs)
        msg.distortion_model = "plumb_bob"

        return msg



def main(args=None):
    rclpy.init(args=args)
    node = RealSensePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline.stop()
        node.destroy_node()


if __name__ == '__main__':
    main()
