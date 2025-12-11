#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from avp_stream import VisionProStreamer

# Optional: Vision Pro streaming
VISIONPRO_IP = "192.168.50.153"
USE_VISIONPRO = True

if USE_VISIONPRO:
    from avp_stream import VisionProStreamer

class CameraStreamer(Node):
    def __init__(self):
        super().__init__("camera_streamer")
        self.publisher = self.create_publisher(Image, "/webcam/image_raw", 10)
        self.bridge = CvBridge()

        # Open webcam
        # self.cap = cv2.VideoCapture(0) # webcam
        self.cap = cv2.VideoCapture(4) # robot usb camera        
        if not self.cap.isOpened():
            raise RuntimeError("Could not open /dev/video0")

        # Start Vision Pro streaming
        if USE_VISIONPRO:
            self.streamer = VisionProStreamer(ip=VISIONPRO_IP)
            self.streamer.start_streaming(
                device=None, format=None, size="1280x720", fps=25 # 640x480, 320x240
            )
            self.get_logger().info("Vision Pro streaming enabled")

        # Timer for ~30 FPS
        self.timer = self.create_timer(0.033, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # Publish to ROS2
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher.publish(img_msg)

        # Optional VisionPro streaming
        if USE_VISIONPRO:
            self.streamer.update_frame(frame)

        # Optional local OpenCV preview
        cv2.imshow("Webcam", frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = CameraStreamer()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()