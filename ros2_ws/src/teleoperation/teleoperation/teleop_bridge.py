#!/usr/bin/env python3
import os
import fcntl
import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
import pymycobot
from packaging import version

MIN_REQUIRE_VERSION = '3.6.1'
if version.parse(pymycobot.__version__) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError(f'pymycobot >= {MIN_REQUIRE_VERSION} required')
from pymycobot import MyCobot280

def acquire(lock_file):
    fd = os.open(lock_file, os.O_RDWR | os.O_CREAT | os.O_TRUNC)
    timeout, start = 50.0, time.time()
    while time.time() - start < timeout:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            return fd
        except (IOError, OSError):
            time.sleep(0.1)
    os.close(fd)
    return None

def release(fd):
    try:
        fcntl.flock(fd, fcntl.LOCK_UN)
        os.close(fd)
    except OSError:
        pass

class TeleOpCoordsbridge(Node):
    def __init__(self):
        super().__init__('teleop_coords_bridge')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baud', 1000000)
        self.declare_parameter('base_frame', 'map')
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        
        self.mc = MyCobot280(port, baud)
        time.sleep(0.05)
        self.mc.set_fresh_mode(1)
        time.sleep(0.05)
        
        self.speed = 50
        self.model = 0
        self.timer = self.create_timer(0.05, self.update)  # 20Hz
        
        self.get_logger().info(f'Teleop bridge started on {port} @ {baud}')

    def update(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, 'ee_target', Time())
            p = tf.transform.translation
            q = tf.transform.rotation
            
            # Convert to Cartesian (mm) and RPY (degrees)
            x = p.x * 1000.0
            y = p.y * 1000.0
            z = p.z * 1000.0
            
            r = R.from_quat([q.x, q.y, q.z, q.w])
            rx, ry, rz = r.as_euler('xyz', degrees=True)
            
            c_value = [x, y, z, rx, ry, rz]
            
            lock = acquire("/tmp/mycobot_lock")
            if lock:
                self.mc.send_coords(c_value, self.speed, self.model)
                release(lock)
            
            self.get_logger().debug(f'Sent: x={x:.1f}, y={y:.1f}, z={z:.1f}')
        except Exception as e:
            self.get_logger().debug(f'TF lookup failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TeleOpCoordsbridge())

if __name__ == '__main__':
    main()