#!/usr/bin/env python3
"""Publish a synthetic RGB point cloud matching the depth_to_cloud.cpp format."""

from __future__ import annotations

import math
import struct
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
import colorsys

class DummyPointCloudPublisher(Node):
    """Publish a simple colored grid as PointCloud2 (x, y, z, rgb fields)."""

    def __init__(self) -> None:
        super().__init__("dummy_pointcloud_publisher")

        self.declare_parameter("frame_id", "camera_lens")
        self.declare_parameter("topic", "/rgb_map/dummy_cloud")
        self.declare_parameter("rate_hz", 2.0)
        self.declare_parameter("grid_size", 20)
        self.declare_parameter("grid_spacing", 0.01)

        self._frame_id: str = self.get_parameter("frame_id").get_parameter_value().string_value
        self._topic: str = self.get_parameter("topic").get_parameter_value().string_value
        self._rate_hz: float = self.get_parameter("rate_hz").get_parameter_value().double_value
        self._grid_size: int = int(self.get_parameter("grid_size").get_parameter_value().integer_value)
        self._grid_spacing: float = self.get_parameter("grid_spacing").get_parameter_value().double_value

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(PointCloud2, self._topic, qos)

        period = 1.0 / self._rate_hz if self._rate_hz > 0.0 else 0.5
        self._timer = self.create_timer(period, self._publish_cloud)
        self.get_logger().info(f"Publishing dummy RGB cloud on {self._topic} at {self._rate_hz} Hz")

    def _make_points(self) -> Tuple[List[Tuple[float, float, float]], List[float]]:
        points: List[Tuple[float, float, float]] = []
        rgb_floats: List[float] = []

        half = self._grid_size // 2
        for iy in range(-half, half):
            for ix in range(-half, half):
                x = ix * self._grid_spacing
                y = iy * self._grid_spacing
                z = 0.5 + 0.05 * math.sin((ix + iy) * 0.2)
                points.append((x, y, z))


                h = (ix * 0.05 + iy * 0.05) % 1.0
                s = 0.8
                v = 0.9  # never dark

                r_f, g_f, b_f = colorsys.hsv_to_rgb(h, s, v)

                r = int(r_f * 255)
                g = int(g_f * 255)
                b = int(b_f * 255)

                rgb_uint = (r << 16) | (g << 8) | b
                rgb_float = struct.unpack("f", struct.pack("I", rgb_uint))[0]
                rgb_floats.append(rgb_float)

        return points, rgb_floats

    def _publish_cloud(self) -> None:
        if self._pub.get_subscription_count() == 0:
            return

        pts, rgbs = self._make_points()
        if not pts:
            return

        data = np.zeros((len(pts), 4), dtype=np.float32)
        data[:, 0] = [p[0] for p in pts]
        data[:, 1] = [p[1] for p in pts]
        data[:, 2] = [p[2] for p in pts]
        data[:, 3] = rgbs

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.height = 1
        msg.width = data.shape[0]

        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = data.tobytes()

        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DummyPointCloudPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
