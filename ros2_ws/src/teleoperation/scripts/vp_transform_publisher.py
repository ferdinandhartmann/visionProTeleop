#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped, Point
from avp_stream import VisionProStreamer
import numpy as np
from tf2_ros import TransformBroadcaster

JOINT_NAMES = [
    "wrist",  # 0
    # Thumb (5 joints)
    "thumb_0",  # 1
    "thumb_1",  # 2
    "thumb_2",  # 3
    "thumb_3",  # 4
    "thumb_4",  # 5  ← extra thumb joint (VisionPro uses 5)
    # Index (5 joints)
    "index_0",  # 6
    "index_1",  # 7
    "index_2",  # 8
    "index_3",  # 9
    "index_4",  # 10
    # Middle (5 joints)
    "middle_0",  # 11
    "middle_1",  # 12
    "middle_2",  # 13
    "middle_3",  # 14
    "middle_4",  # 15
    # Ring (5 joints)
    "ring_0",  # 16
    "ring_1",  # 17
    "ring_2",  # 18
    "ring_3",  # 19
    "ring_4",  # 20
    # Little (5 joints)
    "little_0",  # 21
    "little_1",  # 22
    "little_2",  # 23
    "little_3",  # 24
    "little_4",  # 25
]

PARENT_INDEX = [
    -1,     # 0 wrist → parent = head

    # thumb chain (5)
    0, 1, 2, 3, 4,

    # index chain (5)
    0, 6, 7, 8, 9,

    # middle chain (5)
    0, 11, 12, 13, 14,

    # ring chain (5)
    0, 16, 17, 18, 19,

    # little chain (5)
    0, 21, 22, 23, 24,
]

HAND_CONNECTIONS = [
    # Thumb (1–5)
    (0, 1),
    (1, 2),
    (2, 3),
    (3, 4),
    (4, 5),

    # Index (6–10)
    (0, 6),
    (6, 7),
    (7, 8),
    (8, 9),
    (9, 10),

    # Middle (11–15)
    (0, 11),
    (11, 12),
    (12, 13),
    (13, 14),
    (14, 15),

    # Ring (16–20)
    (0, 16),
    (16, 17),
    (17, 18),
    (18, 19),
    (19, 20),

    # Little (21–25)
    (0, 21),
    (21, 22),
    (22, 23),
    (23, 24),
    (24, 25),
]

def is_valid_transform(T, eps=1e-6):
    if T.shape != (4, 4):
        return False
    R = T[:3, :3]
    if not np.isfinite(R).all():
        return False
    return abs(np.linalg.det(R)) > eps


def quat_from_matrix(mat):
    m = mat
    t = m[0, 0] + m[1, 1] + m[2, 2]

    if t > 0:
        r = np.sqrt(1.0 + t) * 2
        w = 0.25 * r
        x = (m[2, 1] - m[1, 2]) / r
        y = (m[0, 2] - m[2, 0]) / r
        z = (m[1, 0] - m[0, 1]) / r
    elif (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
        r = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / r
        x = 0.25 * r
        y = (m[0, 1] + m[1, 0]) / r
        z = (m[0, 2] + m[2, 0]) / r
    elif m[1, 1] > m[2, 2]:
        r = np.sqrt(1.0 - m[0, 0] + m[1, 1] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / r
        x = (m[0, 1] + m[1, 0]) / r
        y = 0.25 * r
        z = (m[1, 2] + m[2, 1]) / r
    else:
        r = np.sqrt(1.0 - m[0, 0] - m[1, 1] + m[2, 2]) * 2
        w = (m[1, 0] - m[0, 1]) / r
        x = (m[0, 2] + m[2, 0]) / r
        y = (m[1, 2] + m[2, 1]) / r
        z = 0.25 * r

    return (x, y, z, w)


# ======================================================
# ROS2 Node
# ======================================================

class VPTransformPublisher(Node):
    def __init__(self):
        super().__init__("vp_transform_publisher")

        self.declare_parameter("visionpro_ip", "192.168.50.153")
        self.declare_parameter("pinch_threshold", 0.02)

        visionpro_ip = self.get_parameter("visionpro_ip").get_parameter_value().string_value
        self.pinch_threshold = self.get_parameter("pinch_threshold").get_parameter_value().double_value
        
        self.streamer = VisionProStreamer(ip=visionpro_ip)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(MarkerArray, "/visionpro/hand_markers", 10)
        self.timer = self.create_timer(0.01, self.update)
        
        self.get_logger().info("VP Transform Publisher started.")

    def publish_tf(self, parent, child, mat):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child

        t.transform.translation.x = mat[0][3]
        t.transform.translation.y = mat[1][3]
        t.transform.translation.z = mat[2][3]

        q = quat_from_matrix(mat)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)
        
    
    def update(self):
        data = self.streamer.latest
        if data is None:
            return

        markers = MarkerArray()

        pinch_threshold = self.pinch_threshold

        # Publish head in vp_base
        head_mat = data["head"][0]
        self.publish_tf("vp_base", "visionpro/head", head_mat)

        for side, wrist_mat, fingers, pinch_dist in [
            ("left", data["left_wrist"][0], data["left_fingers"], data["left_pinch_distance"]),
            ("right", data["right_wrist"][0], data["right_fingers"], data["right_pinch_distance"])
        ]:
            self.publish_tf("vp_base", f"visionpro/{side}/wrist", wrist_mat)

            # --- TF publishing for joints ---
            joints = [np.eye(4)] + list(fingers)  # joint[0] is wrist (identity relative to wrist)

            for i, _ in enumerate(joints):
                if i == 0:
                    continue  # skip wrist (already published)

                joint_name = JOINT_NAMES[i]
                parent_idx = PARENT_INDEX[i]
                parent_name = f"visionpro/{side}/{JOINT_NAMES[parent_idx]}"
                child_name = f"visionpro/{side}/{joint_name}"

                # Get transforms relative to wrist
                T_parent = joints[parent_idx]
                T_child = joints[i]
                
                if not is_valid_transform(T_parent):
                    # Skip this joint for this frame
                    continue

                T_rel = np.linalg.inv(T_parent) @ T_child  # relative transform

                self.publish_tf(parent_name, child_name, T_rel)

            # --- Visual Markers: wrist + thumb tip + index tip ---
            # Visual markers for wrist, thumb_4, index_4
            tip_ids = [0, 5, 10]  # wrist, thumb tip, index tip

            for j in tip_ids:
                joint_name = JOINT_NAMES[j]
                frame_id = f"visionpro/{side}/{joint_name}"

                m = Marker()
                m.header.frame_id = frame_id
                m.id = 1000 + j + (0 if side == "left" else 100)
                m.type = Marker.SPHERE
                m.scale.x = m.scale.y = m.scale.z = 0.03

                # Color logic
                if j == 0:
                    m.color.r, m.color.g, m.color.b = 1.0, 0.5, 0.0  # wrist always orange
                else:
                    if pinch_dist < pinch_threshold:
                        m.color.r, m.color.g, m.color.b = 0.0, 1.0, 0.0  # green on pinch
                    else:
                        m.color.r, m.color.g, m.color.b = 1.0, 0.5, 0.0  # orange default
                m.color.a = 1.0

                # Origin of its own frame
                m.pose.position.x = 0.0
                m.pose.position.y = 0.0
                m.pose.position.z = 0.0

                markers.markers.append(m)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = VPTransformPublisher() 
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == "__main__":
    main()
