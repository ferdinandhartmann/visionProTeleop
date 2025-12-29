#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from avp_stream import VisionProStreamer
import threading
import time

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
                ('visionpro_ip', '192.168.50.153'),
                ('resolution', '640x480'),
                ('camera_input', '/dev/video4'),
                ('format', 'v4l2'),
                ('fps', 30),
            ]
        )

        self.visionpro_ip = self.get_parameter('visionpro_ip').get_parameter_value().string_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().string_value
        self.camera_input = self.get_parameter('camera_input').get_parameter_value().string_value
        self.format = self.get_parameter('format').get_parameter_value().string_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        

        self.publisher = self.create_publisher(Image, "/webcam/image_raw", 10)
        self.bridge = CvBridge()
        self._stop_event = threading.Event()

        # Start Vision Pro streaming
        if USE_VISIONPRO:
            self.streamer = VisionProStreamer(ip=self.visionpro_ip, record=False)
            self.streamer.configure_video(device=None, format=self.format, size=self.resolution, fps=self.fps)
            self.streamer.start_webrtc(port=9999)
            self.get_logger().info("Vision Pro streaming enabled")

            
                # Open webcam
        self.cap = cv2.VideoCapture(self.camera_input)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera! {self.camera_input}")
        
        # Parse resolution parameter (expects format "WIDTHxHEIGHT", e.g., "1280x720")
        width, height = map(int, self.resolution.lower().split('x'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        self._camera_period = 1.0 / self.fps if self.fps > 0 else 0.0
        self._camera_thread = threading.Thread(target=self._camera_loop, name="camera_streamer", daemon=True)
        self._camera_thread.start()

    def _camera_loop(self):
        next_time = time.perf_counter()
        while not self._stop_event.is_set():
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(self._camera_period or 0.01)
                continue

            # Publish to ROS2
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher.publish(img_msg)

            # Optional VisionPro streaming
            if USE_VISIONPRO:
                self.streamer.update_frame(frame)

            # # Optional local OpenCV preview
            cv2.imshow("Webcam", frame)
            cv2.waitKey(1)

            if self._camera_period > 0:
                next_time += self._camera_period
                sleep_time = next_time - time.perf_counter()
                if sleep_time > 0:
                    time.sleep(sleep_time)

    def destroy_node(self):
        self._stop_event.set()
        if getattr(self, "_camera_thread", None):
            self._camera_thread.join(timeout=1.0)
        if getattr(self, "cap", None):
            self.cap.release()
        return super().destroy_node()


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



if __name__ == "__main__":
    main()
