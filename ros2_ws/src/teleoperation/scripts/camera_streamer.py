#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from avp_stream import VisionProStreamer

# Optional: Vision Pro streaming
USE_VISIONPRO = True

if USE_VISIONPRO:
    from avp_stream import VisionProStreamer

class CameraStreamer(Node):
    def __init__(self):
        super().__init__("camera_streamer")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('visionpro_ip', '192.168.11.99'),
                ('resolution', '1280x720'),
                ('camera_input', '/dev/video0')
                ('format', 'v4l2'),
                ('fps', 25),
            ]
        )

        self.visionpro_ip = self.get_parameter('visionpro_ip').get_parameter_value().string_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().string_value
        self.camera_input = self.get_parameter('camera_input').get_parameter_value().string_value
        self.format = self.get_parameter('format').get_parameter_value().string_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        

        self.publisher = self.create_publisher(Image, "/webcam/image_raw", 10)
        self.bridge = CvBridge()

        # Open webcam
        self.cap = cv2.VideoCapture(self.camera_input)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open {self.camera_input}")

        # Start Vision Pro streaming
        if USE_VISIONPRO:
            self.streamer = VisionProStreamer(ip=self.visionpro_ip)
            self.streamer.configure_video(
                device=self.camera_input, format=self.format, size=self.resolution, fps=self.fps
            )
            self.streamer.start_webrtc(port=9999)
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

        # # Optional local OpenCV preview
        cv2.imshow("Webcam", frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = CameraStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()