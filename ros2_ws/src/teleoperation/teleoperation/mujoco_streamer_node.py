"""ROS 2 node that streams the MuJoCo myCobot scene to Vision Pro or a local viewer."""

from __future__ import annotations

import threading
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import ParameterDescriptor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState


class MujocoStreamerNode(Node):
    """Bridge ROS joint states into the MuJoCo scene and stream it to Vision Pro."""

    def __init__(self) -> None:
        super().__init__("mujoco_streamer")

        # Import MuJoCo lazily so the node still declares parameters when MuJoCo is missing.
        import mujoco

        self.declare_parameter(
            "viewer",
            "ar",
            descriptor=ParameterDescriptor(
                description="Viewer type: 'ar' to stream to Vision Pro, 'mujoco' for local preview.",
            ),
        )
        self.declare_parameter("ip", "192.168.50.153")
        self.declare_parameter("port", 50051)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter(
            "attach_to",
            [0.2, 1.0, 0.7, -90.0],
            descriptor=ParameterDescriptor(
                description="AR attachment pose [x, y, z, yaw_degrees] used by VisionProStreamer.",
            ),
        )
        default_xml = Path(__file__).resolve().parents[3] / "robot_description/mycobot_mujoco/scene_mycobot.xml"
        self.declare_parameter(
            "xml_path",
            str(default_xml),
            descriptor=ParameterDescriptor(description="MuJoCo scene to stream."),
        )
        self.declare_parameter("update_rate_hz", 60.0)

        params = self._load_params()

        self.model = mujoco.MjModel.from_xml_path(params["xml_path"])
        self.data = mujoco.MjData(self.model)
        self.joint_name_to_qpos = self._build_joint_mapping(mujoco)

        # Vision Pro streaming setup
        self.streamer = None
        self.viewer_handle = None
        if params["viewer"] == "ar":
            from avp_stream import VisionProStreamer

            self.streamer = VisionProStreamer(ip=params["ip"], record=False)
            self.streamer.configure_sim(
                xml_path=params["xml_path"],
                model=self.model,
                data=self.data,
                relative_to=params["attach_to"],
                grpc_port=params["port"],
                force_reload=True,
            )
            self.streamer.start_webrtc()
            self.get_logger().info("Streaming MuJoCo scene to Vision Pro")
        else:
            try:
                from mujoco import viewer as mj_viewer

                self.viewer_handle = mj_viewer.launch_passive(self.model, self.data)
                self.get_logger().info("Launched local MuJoCo viewer")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"Failed to start local MuJoCo viewer: {exc}")

        qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE)
        joint_topic = params["joint_state_topic"]
        self.joint_sub = self.create_subscription(JointState, joint_topic, self._joint_state_cb, qos)
        self.get_logger().info(f"Listening for joint states on {joint_topic}")

        self._latest_joint_state: Dict[str, float] = {}
        self._joint_state_lock = threading.Lock()

        update_period = 1.0 / params["update_rate_hz"] if params["update_rate_hz"] > 0 else 0.016
        self.timer = self.create_timer(update_period, self._update_scene)

    def _load_params(self) -> Dict[str, object]:
        viewer = self.get_parameter("viewer").get_parameter_value().string_value
        ip = self.get_parameter("ip").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value
        joint_topic = self.get_parameter("joint_state_topic").get_parameter_value().string_value
        attach_to = list(self.get_parameter("attach_to").get_parameter_value().double_array_value)
        xml_path = self.get_parameter("xml_path").get_parameter_value().string_value
        rate = self.get_parameter("update_rate_hz").get_parameter_value().double_value
        return {
            "viewer": viewer,
            "ip": ip,
            "port": port,
            "joint_state_topic": joint_topic,
            "attach_to": attach_to,
            "xml_path": xml_path,
            "update_rate_hz": rate,
        }

    def _build_joint_mapping(self, mujoco) -> Dict[str, int]:
        mapping: Dict[str, int] = {}
        for joint_id in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            if name:
                mapping[name] = self.model.jnt_qposadr[joint_id]
        expected = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "gripper_controller",
        ]
        missing = [name for name in expected if name not in mapping]
        if missing:
            self.get_logger().warn(f"MuJoCo model missing joints referenced by IK: {missing}")
        return mapping

    def _joint_state_cb(self, msg: JointState) -> None:
        with self._joint_state_lock:
            for name, position in zip(msg.name, msg.position):
                self._latest_joint_state[name] = position

    def _apply_joint_state(self) -> None:
        if not self._latest_joint_state:
            return

        def clamp(val: float, min_val: float, max_val: float) -> float:
            return max(min(val, max_val), min_val)

        with self._joint_state_lock:
            # copy to avoid holding the lock while writing into MuJoCo buffers
            joint_copy = dict(self._latest_joint_state)

        for name, position in joint_copy.items():
            idx = self.joint_name_to_qpos.get(name)
            if idx is None:
                continue
            self.data.qpos[idx] = position

            if name == "gripper_controller":
                left_val = clamp(position, -0.5, 0.5)
                right_val = clamp(-position, -0.5, 0.5)

                mimic_map = {
                    "gripper_left3_to_gripper_left1": left_val,
                    "gripper_base_to_gripper_left2": left_val,
                    "gripper_base_to_gripper_right3": right_val,
                    "gripper_right3_to_gripper_right1": right_val,
                    "gripper_base_to_gripper_right2": right_val,
                }
                for mimic_name, mimic_pos in mimic_map.items():
                    m_idx = self.joint_name_to_qpos.get(mimic_name)
                    if m_idx is not None:
                        self.data.qpos[m_idx] = mimic_pos

        # Ensure derived values (sites, tendons) stay in sync
        import mujoco

        mujoco.mj_forward(self.model, self.data)

    def _update_scene(self) -> None:
        self._apply_joint_state()

        if self.streamer is not None:
            self.streamer.update_sim()
        elif self.viewer_handle is not None and self.viewer_handle.is_running():
            self.viewer_handle.sync()
        else:
            # If the viewer was closed, stop the timer to avoid spamming logs
            self.timer.cancel()
            self.get_logger().info("Viewer closed; stopping MujocoStreamerNode timer")


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = MujocoStreamerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
