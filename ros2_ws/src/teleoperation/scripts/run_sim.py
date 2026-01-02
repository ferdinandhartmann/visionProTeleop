#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import mujoco
import numpy as np
import cv2

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class MujocoCameraNode(Node):
    def __init__(self):
        super().__init__('mujoco_camera_node')

        self.bridge = CvBridge()

        self.rgb_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        # Load model
        self.model = mujoco.MjModel.from_xml_path('/home/ferdinand/visionpro_teleop_project/visionProTeleop/ros2_ws/src/robot_description/mycobot_mujoco/scene_mycobot.xml')
        self.data = mujoco.MjData(self.model)

        # Renderer
        self.width = 640
        self.height = 480

        # Create and bind an offscreen GL context (required before mjr_* calls)
        self.gl = mujoco.GLContext(self.width, self.height)
        self.gl.make_current()

        # MuJoCo visualization structs
        self.scene = mujoco.MjvScene(self.model, maxgeom=10000)
        self.vopt = mujoco.MjvOption()
        self.cam = mujoco.MjvCamera()
        self.pert = mujoco.MjvPerturb()
        self.con = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150)

        # Bind to a named fixed camera in your XML ("depth_cam")
        cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "depth_cam")
        if cam_id < 0:
            raise RuntimeError("Camera 'depth_cam' not found in MJCF.")
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        self.cam.fixedcamid = cam_id

        # Viewport / pixel buffers
        self.viewport = mujoco.MjrRect(0, 0, self.width, self.height)
        self.rgb_buf = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        self.depth_buf = np.zeros((self.height, self.width), dtype=np.float32)

        self.get_logger().info("MuJoCo camera node started (mjr_readPixels pipeline)")


        self.timer = self.create_timer(0.033, self.step)  # ~30 Hz
        
        self.get_logger().info('MuJoCo camera node started')

    def step(self):
        mujoco.mj_step(self.model, self.data)

        # Ensure GL context is current in this thread
        self.gl.make_current()

        # Update scene for rendering
        mujoco.mjv_updateScene(
            self.model,
            self.data,
            self.vopt,
            self.pert,
            self.cam,
            mujoco.mjtCatBit.mjCAT_ALL,
            self.scene
        )

        # Render to the current framebuffer
        mujoco.mjr_render(self.viewport, self.scene, self.con)

        # Read pixels (rgb uint8, depth float32 normalized)
        mujoco.mjr_readPixels(self.rgb_buf, self.depth_buf, self.viewport, self.con)

        # MuJoCo's readPixels is typically upside-down for image coordinates
        rgb = np.flipud(self.rgb_buf)
        depth = np.flipud(self.depth_buf)

        # Convert depth buffer to meters using global znear/zfar (from <visual><map>)
        znear = float(self.model.vis.map.znear)
        zfar = float(self.model.vis.map.zfar)
        depth_m = znear / (1.0 - depth * (1.0 - znear / zfar))

        # Publish RGB
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
        rgb_msg.header.stamp = self.get_clock().now().to_msg()
        rgb_msg.header.frame_id = 'camera_link'
        self.rgb_pub.publish(rgb_msg)

        # Publish depth (32FC1 meters)
        depth_msg = self.bridge.cv2_to_imgmsg(depth_m.astype(np.float32), encoding='32FC1')
        depth_msg.header = rgb_msg.header
        self.depth_pub.publish(depth_msg)

        # CameraInfo (still your static K; ideally compute from fovy)
        info = CameraInfo()
        info.header = rgb_msg.header
        info.width = self.width
        info.height = self.height
        info.k = [554.0, 0.0, self.width/2,
                0.0, 554.0, self.height/2,
                0.0, 0.0, 1.0]
        self.info_pub.publish(info)




def main():
    rclpy.init()
    node = MujocoCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
