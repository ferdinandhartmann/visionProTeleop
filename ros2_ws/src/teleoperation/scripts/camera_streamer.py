#!/usr/bin/env python3
import importlib.util
import threading
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

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
                ('camera_mode', 'both'),  # robot, realsense, both
            ]
        )

        self.visionpro_ip = self.get_parameter('visionpro_ip').get_parameter_value().string_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().string_value
        self.camera_input = self.get_parameter('camera_input').get_parameter_value().string_value
        self.format = self.get_parameter('format').get_parameter_value().string_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.camera_mode = self.get_parameter('camera_mode').get_parameter_value().string_value.lower()
        
        if self.camera_mode not in ("robot", "realsense", "both"):
            self.get_logger().warning(f"Unknown camera_mode '{self.camera_mode}', defaulting to robot")
            self.camera_mode = "robot"

        width, height = map(int, self.resolution.lower().split('x'))
        self._frame_size = (width, height)

        self.publisher = self.create_publisher(Image, "/webcam/image_raw", 10)
        self.combined_publisher = self.create_publisher(Image, "/camera/combined_image_raw", 10)
        self.bridge = CvBridge()
        self._stop_event = threading.Event()
        self._realsense_pipeline = None
        self._realsense_config = None

        # Start Vision Pro streaming
        if USE_VISIONPRO:
            self.streamer = VisionProStreamer(ip=self.visionpro_ip, record=False)
            self.streamer.configure_video(device=None, format=self.format, size=self.resolution, fps=self.fps)
            self.streamer.start_webrtc()
            self.streamer.register_frame_callback(lambda frame: frame)            

            self.get_logger().info("Vision Pro streaming enabled")

        self._use_robot_camera = self.camera_mode in ("robot", "both")
        self._use_realsense = self.camera_mode in ("realsense", "both")
            
        self.cap = None
        if self._use_robot_camera:
            self.cap = cv2.VideoCapture(self.camera_input)
            if not self.cap.isOpened():
                raise RuntimeError(f"Could not open camera! {self.camera_input}")
            
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        if self._use_realsense:
            self._init_realsense(width, height)

        self._camera_period = 1.0 / self.fps if self.fps > 0 else 0.0
        self._camera_thread = threading.Thread(target=self._camera_loop, name="camera_streamer", daemon=True)
        self._camera_thread.start()

    def _camera_loop(self):
        next_time = time.perf_counter()
        while not self._stop_event.is_set():
            robot_frame = self._read_robot_frame()
            realsense_frame = self._read_realsense_frame()

            combined_frame = self._compose_frame(robot_frame, realsense_frame)
            if combined_frame is None:
                time.sleep(self._camera_period or 0.01)
                continue

            if robot_frame is not None:
                robot_msg = self.bridge.cv2_to_imgmsg(robot_frame, encoding="bgr8")
                self.publisher.publish(robot_msg)

            combined_msg = self.bridge.cv2_to_imgmsg(combined_frame, encoding="bgr8")
            self.combined_publisher.publish(combined_msg)

            if USE_VISIONPRO:
                self.streamer.update_frame(combined_frame)

            cv2.imshow("Webcam", combined_frame)
            cv2.waitKey(1)

            if self._camera_period > 0:
                next_time += self._camera_period
                sleep_time = next_time - time.perf_counter()
                if sleep_time > 0:
                    time.sleep(sleep_time)

    def _read_robot_frame(self) -> Optional[np.ndarray]:
        if not self._use_robot_camera or self.cap is None:
            return None
        ret, frame = self.cap.read()
        if not ret:
            return None
        return cv2.resize(frame, self._frame_size)

    def _init_realsense(self, width: int, height: int) -> None:
        spec = importlib.util.find_spec("pyrealsense2")
        if spec is None:
            raise RuntimeError("camera_mode includes RealSense but pyrealsense2 is not installed")

        import pyrealsense2 as rs  # noqa: WPS433

        self._realsense = rs
        self._realsense_pipeline = rs.pipeline()
        self._realsense_config = rs.config()
        self._realsense_config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, self.fps)
        self._realsense_pipeline.start(self._realsense_config)
        self.get_logger().info("RealSense camera initialized")

    def _read_realsense_frame(self) -> Optional[np.ndarray]:
        if not self._use_realsense or self._realsense_pipeline is None:
            return None

        try:
            frames = self._realsense_pipeline.wait_for_frames(timeout_ms=200)
        except RuntimeError:
            # No frame this cycle — totally normal
            return None

        color_frame = frames.get_color_frame()
        if not color_frame:
            return None

        frame = np.asanyarray(color_frame.get_data())
        if frame.size == 0:
            return None

        return cv2.resize(frame, self._frame_size)


    def _compose_frame(self, robot_frame: Optional[np.ndarray], realsense_frame: Optional[np.ndarray]) -> Optional[np.ndarray]:
        if self.camera_mode == "robot":
            return robot_frame
        if self.camera_mode == "realsense":
            return realsense_frame
        if robot_frame is None or realsense_frame is None:
            return None
        return np.vstack([realsense_frame, robot_frame])

    def destroy_node(self):
        self._stop_event.set()
        if getattr(self, "_camera_thread", None):
            self._camera_thread.join(timeout=1.0)
        if getattr(self, "cap", None):
            self.cap.release()
        if getattr(self, "_realsense_pipeline", None):
            self._realsense_pipeline.stop()
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
