#!/usr/bin/env python3
"""ROS 2 node that streams the MuJoCo myCobot scene to Vision Pro or a local viewer."""

from __future__ import annotations

import threading
from pathlib import Path
from typing import Dict, List, Optional

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import mujoco

import soundfile as sf

import time

from avp_stream import VisionProStreamer

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class VPStreamer(Node):
    """Bridge ROS joint states into the MuJoCo scene and stream it to Vision Pro."""

    def __init__(self) -> None:
        super().__init__("vp_streamer")

        self.declare_parameter(
            "viewer",
            "ar",
            descriptor=ParameterDescriptor(description="Viewer type: 'ar' to stream to Vision Pro, 'mujoco' for local preview.",),
        )
        self.declare_parameter("visionpro_ip", "192.168.11.99")
        self.declare_parameter("port", 50051)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter(
            "attach_to",
            [0.2, 1.0, 0.7, -90.0],
            descriptor=ParameterDescriptor(description="AR attachment pose [x, y, z, yaw_degrees] used by VisionProStreamer.",),
        )
        self.declare_parameter("force_reload", False)
        self.declare_parameter("camera_device", "/dev/video0")
        self.declare_parameter("camera_resolution", "1280x720")
        self.declare_parameter("camera_fps", 25)
        self.declare_parameter("format", "v4l2")
        self.declare_parameter("enable_camera", True)
        self.declare_parameter("enable_audio", True)
        
        # Resolve the default MuJoCo scene from the robot_description package.
        robot_description_share = Path("/home/ferdinand/visionpro_teleop_project/visionProTeleop/ros2_ws/src/robot_description")
        default_xml = robot_description_share / "mycobot_mujoco/scene_mycobot.xml"
        self.declare_parameter(
            "xml_path",
            str(default_xml),
            descriptor=ParameterDescriptor(description="MuJoCo scene to stream."),
        )
        self.declare_parameter("update_simulation_hz", 60.0)

        params = self._load_params()
        
        self._teleop_enabled = False
        self._teleop_enabled_sub = self.create_subscription(Bool, '/teleop/teleop_enabled', self._teleop_enabled_cb, 10)
        
        self._contact_active = False
        self.latest_joint_vel = None
        self._motor_start_delay = 0.2   
        self._motor_ramp_time = 0.3    # seconds to full volume
        self._teleop_enabled_time = None
        self.motor_gain = 0.0
        self._enable_idx = 0
        self._disable_idx = 0
        self.enable_sound = load_wav_mono("/home/ferdinand/visionpro_teleop_project/visionProTeleop/ros2_ws/src/teleoperation/sounds/enabled.wav")
        self.disable_sound = load_wav_mono("/home/ferdinand/visionpro_teleop_project/visionProTeleop/ros2_ws/src/teleoperation/sounds/disabled.wav")

        self.enable_audio = params["enable_audio"]
        self.enable_camera = params["enable_camera"]
        
        
        self.bridge = CvBridge()
        if self.enable_camera:
            self.cap = cv2.VideoCapture(params["camera_device"])
            if not self.cap.isOpened():
                raise RuntimeError(f"Could not open camera {params['camera_device']}")
            
            camera_period = 1.0 / params["camera_fps"]
            
            width, height = map(int, params["camera_resolution"].split('x'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, params["camera_fps"])
            
            self.camera_timer = self.create_timer(camera_period, self._camera_cb)
            self.publisher = self.create_publisher(Image, "/camera_raw", 10)
            
            self.get_logger().info("Camera initialized")


        self.model = mujoco.MjModel.from_xml_path(params["xml_path"])
        self.data = mujoco.MjData(self.model)
        self.joint_name_to_qpos = self._build_joint_mapping(mujoco)
        

        self.streamer = VisionProStreamer(ip=params["visionpro_ip"], record=False)
        self.viewer_handle = None
        
        if params["viewer"] == "ar":

            self.streamer.configure_mujoco(
                xml_path=params["xml_path"],
                model=self.model,
                data=self.data,
                relative_to=params["attach_to"],
                grpc_port=params["port"],
                force_reload=params["force_reload"],
            )
        
        elif params["viewer"] == "mujoco":
            try:
                from mujoco import viewer as mj_viewer

                self.viewer_handle = mj_viewer.launch_passive(self.model, self.data)
                self.get_logger().info("Launched local MuJoCo viewer")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"Failed to start local MuJoCo viewer: {exc}")
                    
        if self.enable_camera:
            self.streamer.configure_video(
                device=None, # Set frames manually to also be able to publish to ROS2
                format=params["format"],
                size=params["camera_resolution"],
                fps=params["camera_fps"],
            )
            self.get_logger().info("Vision Pro camera streaming enabled")
            
        if self.enable_audio:
            self.motor_audio = MotorSoundModel()
            self.streamer.configure_audio(sample_rate=48000)
            self.streamer.register_audio_callback(self._audio_callback)
            self.get_logger().info("Vision Pro audio streaming enabled")

        self.streamer.start_webrtc()
        self.get_logger().info("Streaming MuJoCo scene to Vision Pro")

        self.get_logger().info(f"viewer: {params['viewer']}")
        if params["viewer"] != "None":
            update_period = 1.0 / params["update_simulation_hz"] if params["update_simulation_hz"] > 0 else 0.016
            self.timer = self.create_timer(update_period, self._update_scene)
                
        qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE)
        joint_topic = params["joint_state_topic"]
        self.joint_sub = self.create_subscription(JointState, joint_topic, self._joint_state_cb, qos)
        self.get_logger().info(f"Listening for joint states on {joint_topic}")

        self._latest_joint_state: Dict[str, float] = {}
        self._joint_state_lock = threading.Lock()
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.ee_fk_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "ee_fk_frame"
        )

        self.ee_target_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "ee_target_frame"
        )



    def _load_params(self) -> Dict[str, object]:
        viewer = self.get_parameter("viewer").get_parameter_value().string_value
        visionpro_ip = self.get_parameter("visionpro_ip").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value
        joint_topic = self.get_parameter("joint_state_topic").get_parameter_value().string_value
        attach_to = list(self.get_parameter("attach_to").get_parameter_value().double_array_value)
        xml_path = self.get_parameter("xml_path").get_parameter_value().string_value
        rate = self.get_parameter("update_simulation_hz").get_parameter_value().double_value
        force_reload = self.get_parameter("force_reload").get_parameter_value().bool_value
        camera_device = self.get_parameter("camera_device").value
        camera_resolution = self.get_parameter("camera_resolution").value
        camera_fps = self.get_parameter("camera_fps").value
        enable_camera = self.get_parameter("enable_camera").value
        format = self.get_parameter("format").value
        enable_audio = self.get_parameter("enable_audio").value
        return {
            "viewer": viewer,
            "visionpro_ip": visionpro_ip,
            "port": port,
            "joint_state_topic": joint_topic,
            "attach_to": attach_to,
            "xml_path": xml_path,
            "update_simulation_hz": rate,
            "force_reload": force_reload,
            "camera_device": camera_device,
            "camera_resolution": camera_resolution,
            "camera_fps": camera_fps,
            "enable_camera": enable_camera,
            "format": format,
            "enable_audio": enable_audio,
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
                
            self.latest_joint_vel = list(msg.velocity)


    def _apply_joint_state(self) -> None:
        if not self._latest_joint_state:
            return

        def clamp(val: float, min_val: float, max_val: float) -> float:
            return max(min(val, max_val), min_val)

        with self._joint_state_lock:
            # copy to avoid holding the lock while writing into MuJoCo buffers
            joint_copy = dict(self._latest_joint_state)

        ################# Apply joint states into MuJoCo buffers #################
        for name, position in joint_copy.items():
            idx = self.joint_name_to_qpos.get(name)
            if idx is None:
                    continue
            if name == "gripper_controller":
                gripper_lower_limit = -0.25
                gripper_upper_limit = 0.8
                self.data.qpos[idx] = (gripper_lower_limit + (gripper_upper_limit - gripper_lower_limit) * ((100 - position) / 100.0))
                self.data.qpos[idx+1] = (gripper_lower_limit + (gripper_upper_limit - gripper_lower_limit) * ((100 - position) / 100.0))
                gripper_lower_limit = 0.0
                gripper_upper_limit = 0.8
                self.data.qpos[idx+2] = (gripper_lower_limit + (gripper_upper_limit - gripper_lower_limit) * ((100 - position) / 100.0))
                self.data.qpos[idx+3] = (gripper_lower_limit + (gripper_upper_limit - gripper_lower_limit) * ((100 - position) / 100.0))
            else:
                self.data.qpos[idx] = position

        # Ensure derived values (sites, tendons) stay in sync
        mujoco.mj_forward(self.model, self.data)


    def _update_scene(self) -> None:
        self._apply_joint_state()
        
        self._update_target_frames()
        
        mujoco.mj_step(self.model, self.data)

        if self.streamer is not None:
            self.streamer.update_sim()
        elif self.viewer_handle is not None and self.viewer_handle.is_running():
            self.viewer_handle.sync()
        else:
            # If the viewer was closed, stop the timer to avoid spamming logs
            self.timer.cancel()
            self.get_logger().info("Viewer closed; stopping VPStreamer timer")
            
        self._contact_active = self._detect_impact_contact()


    def _set_mocap_from_tf(self, body_id, tf):
        mocap_id = self.model.body_mocapid[body_id]
        if mocap_id < 0:
            return

        # --- position (rotate 180deg about world Z) ---
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        z = tf.transform.translation.z
        self.data.mocap_pos[mocap_id, 0] = -x
        self.data.mocap_pos[mocap_id, 1] = -y
        self.data.mocap_pos[mocap_id, 2] =  z

        # --- orientation (premultiply by 180deg about world Z) ---
        r = tf.transform.rotation
        q_tf = np.array([r.w, r.x, r.y, r.z], dtype=np.float64)

        q_corr = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)  # yaw=pi about Z

        q_out = np.zeros(4, dtype=np.float64)
        mujoco.mju_mulQuat(q_out, q_corr, q_tf)  # world-frame correction

        self.data.mocap_quat[mocap_id, :] = q_out


    def _update_target_frames(self):
        try:
            latest_time = rclpy.time.Time(seconds=0)

            # FK pose (from IK node) — latest available
            tf_fk = self.tf_buffer.lookup_transform(
                "mycobot_base",
                "gripper_ee",
                latest_time
            )
            self._set_mocap_from_tf(self.ee_fk_body_id, tf_fk)

            # Teleop target pose — latest available
            tf_target = self.tf_buffer.lookup_transform(
                "mycobot_base",
                "ee_target_offset_mycobot_base_vis",
                latest_time
            )
            self._set_mocap_from_tf(self.ee_target_body_id, tf_target)

            mujoco.mj_forward(self.model, self.data)

        except (LookupException, ConnectivityException, ExtrapolationException):
            pass


            
    def _camera_cb(self) -> None:
        if self.streamer is None:
            return

        ret, frame = self.cap.read()
        if not ret:
            return
        
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher.publish(img_msg)

        # Send frame to Vision Pro
        self.streamer.update_frame(frame)
        
        # # Optional local OpenCV preview
        # cv2.imshow("Webcam", frame)
        # cv2.waitKey(1)

    def _geom_linvel(self, geom_id: int) -> np.ndarray:
        """Return world-frame linear velocity of a geom (3,)."""
        # mj_objectVelocity returns 6D spatial velocity (angular+linear) in requested frame.
        # In many bindings: out[0:3]=angular, out[3:6]=linear.
        v6 = np.zeros(6, dtype=np.float64)
        mujoco.mj_objectVelocity(
            self.model,
            self.data,
            mujoco.mjtObj.mjOBJ_GEOM,
            geom_id,
            v6,
            0,  # 0 = world frame
        )
        return v6[3:6].copy()

        
    def _detect_impact_contact(self,
                            force_thresh=15.0,
                            relvel_thresh=0.30) -> bool:

        for i in range(self.data.ncon):
            c = self.data.contact[i]
            if c.efc_address < 0:
                continue

            force = float(abs(self.data.efc_force[c.efc_address]))

            v1 = self._geom_linvel(int(c.geom1))
            v2 = self._geom_linvel(int(c.geom2))
            rel_vel = float(np.linalg.norm(v1 - v2))

            if force > force_thresh and rel_vel > relvel_thresh:
                return True

        return False


        
    def _audio_callback(self, audio_frame):

        sample_rate = 48000
        num_samples = audio_frame.samples
        t = np.arange(num_samples) / sample_rate

        output = np.zeros(num_samples, dtype=np.float32)
        
        if not self._teleop_enabled:
            self._motor_gain = 0.0

        # ENABLE SOUND 
        if self._enable_idx < len(self.enable_sound):
            n = min(num_samples, len(self.enable_sound) - self._enable_idx)
            output[:n] += self.enable_sound[self._enable_idx:self._enable_idx + n]
            self._enable_idx += n

        # DISABLE SOUND 
        if self._disable_idx < len(self.disable_sound):
            n = min(num_samples, len(self.disable_sound) - self._disable_idx)
            output[:n] += self.disable_sound[self._disable_idx:self._disable_idx + n]
            self._disable_idx += n
                

        # MOTOR SOUND (delayed + ramped)
        if self._teleop_enabled and self.latest_joint_vel is not None:
            now = time.time()
            dt = now - self._teleop_enabled_time if self._teleop_enabled_time else 0.0

            if dt > self._motor_start_delay:
                ramp = 1.0 - np.exp(-3.0 * (dt - self._motor_start_delay))
                self._motor_gain = np.clip(ramp, 0.0, 1.0)
            else:
                self._motor_gain = 0.0

            speed = float(np.mean(np.abs(self.latest_joint_vel[:6])))
            motor = self.motor_audio.generate(speed, num_samples)

            output += self._motor_gain * motor

            if not hasattr(self, "_last_motor_log") or (self.get_clock().now().nanoseconds - getattr(self, "_last_motor_log", 0)) > 3e8:
                self.get_logger().info(f"Motor sound generated at speed: {speed}")
                self._last_motor_log = self.get_clock().now().nanoseconds


        # ==============================
        # 3. CONTACT SOUND (click/buzz)
        # ==============================
        # if self._contact_active:
        #     freq = 1200.0
        #     amp = 0.35
        #     output += amp * np.sign(np.sin(2 * np.pi * freq * t))


        output = np.clip(output, -1.0, 1.0)

        audio = (output * 32767).astype(np.int16).tobytes()

        for plane in audio_frame.planes:
            plane.update(audio)

        return audio_frame

    def _teleop_enabled_cb(self, msg: Bool):
        prev = self._teleop_enabled
        self._teleop_enabled = msg.data

        if self._teleop_enabled and not prev:
            self._enable_idx = 0
            self._disable_idx = len(self.disable_sound)  # stop disable
            self._teleop_enabled_time = time.time()
            self._motor_gain = 0.0

        elif not self._teleop_enabled and prev:
            self._disable_idx = 0
            self._enable_idx = len(self.enable_sound)    # stop enable
            self._teleop_enabled_time = None
            self._motor_gain = 0.0


def load_wav_mono(path):
    data, sr = sf.read(path, dtype="float32")
    if sr != 48000:
        raise RuntimeError(f"{path} must be 48kHz")
    if data.ndim > 1:
        data = data[:, 0]
    return data

class MotorSoundModel:
    def __init__(self, sample_rate=48000):
        self.fs = sample_rate
        self.phase = 0.0
        self.freq_smooth = 0.0
        self.amp_smooth = 0.0
        
        # --- Hall / space ---
        self.delay_samples = int(0.002 * sample_rate)  # 50 ms
        self.reverb_buf = np.zeros(self.delay_samples, dtype=np.float32)
        self.reverb_idx = 0
        
    def generate(self, speed, n):
        MAX_SPEED = 4.0  # rad/s
        speed = float(np.clip(speed / MAX_SPEED, 0.0, 1.0))
        speed = np.power(speed, 0.2)  # gentler perceptual curve compression

        # Target frequency & amplitude
        target_freq = 45.0 + 80.0 * speed
        target_amp  = 0.1 + 0.2 * speed

        # SMOOTH THEM (this is the key)
        alpha_freq = 0.55 # low = fast response
        alpha_amp  = 0.55
        self.freq_smooth = (1-alpha_freq) * self.freq_smooth + alpha_freq * target_freq
        self.amp_smooth  = (1-alpha_amp) * self.amp_smooth  + alpha_amp * target_amp

        if self.amp_smooth < 0.002:
            return np.zeros(n, dtype=np.float32)

        # Phase-accurate oscillator
        phase_inc = 3 * np.pi * self.freq_smooth / self.fs
        phases = self.phase + phase_inc * np.arange(n)

        # Base motor signal
        dry = self.amp_smooth * np.sin(phases)

        # --- SIMPLE HALL (feedback delay) ---
        wet = np.zeros_like(dry)
        for i in range(len(dry)):
            d = self.reverb_buf[self.reverb_idx]
            wet[i] = d
            self.reverb_buf[self.reverb_idx] = 0.92 * d + 0.35 * dry[i]
            self.reverb_idx = (self.reverb_idx + 1) % self.delay_samples

        signal = dry + wet * 0.3

        self.phase = phases[-1] + phase_inc
        return signal



def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = VPStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
