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
from teleoperation.msg import TeleopTarget

    # import avp_stream, inspect
    # print("USING avp_stream from:", avp_stream.__file__, flush=True)

class VPStreamer(Node):
    """Bridge ROS joint states into the MuJoCo scene and stream it to Vision Pro."""

    def __init__(self) -> None:
        super().__init__("vp_streamer")

        self._last_log_times = {}
        
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
        self.declare_parameter(
            "initial_joint_positions_deg",
            [0.0, 30.0, -90.0, 0.0, 0.0, 45.0],
            descriptor=ParameterDescriptor(
                description="Initial joint angles (degrees) used when issuing a reset."
            ),
        )
        self.declare_parameter(
            "initial_gripper_percent",
            100.0,
            descriptor=ParameterDescriptor(
                description="Initial gripper percentage used when issuing a reset."
            ),
        )

        # Parameters for publishing a TeleopTarget when MuJoCo is reset
        self.declare_parameter("ee_target_on_reset_position", [0.109, -0.063, 0.314])
        self.declare_parameter("ee_target_on_reset_orientation_xyzw", [-0.002, 0.500, -0.004, 0.866])
        self.declare_parameter("ee_target_on_reset_gripper", 100)

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

        # Listen for reset events from the VisionProStreamer so we can re-publish joint states.
        self.streamer.register_reset_callback(self._on_streamer_reset)
        self.get_logger().info(f"Registered reset callback with streamer id={id(self.streamer)}")
        self.get_logger().info("VPStreamer initialized and listening for reset events.")

        # Publisher for TeleopTarget messages (pose + gripper)
        self.ee_target_pub = self.create_publisher(TeleopTarget, "/teleop/ee_target", 10)

        self._reset_requested = False
        self._reset_lock = threading.Lock()
        self._pending_model = None
        self._pending_data = None
        self._reset_state = "idle"
        self._skip_joint_apply_frames = 0

        

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
        ee_target_on_reset_position = list(self.get_parameter("ee_target_on_reset_position").get_parameter_value().double_array_value)
        ee_target_on_reset_orientation_xyzw = list(self.get_parameter("ee_target_on_reset_orientation_xyzw").get_parameter_value().double_array_value)
        ee_target_on_reset_gripper = int(self.get_parameter("ee_target_on_reset_gripper").get_parameter_value().integer_value)
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
            "ee_target_on_reset_position": ee_target_on_reset_position,
            "ee_target_on_reset_orientation_xyzw": ee_target_on_reset_orientation_xyzw,
            "ee_target_on_reset_gripper": ee_target_on_reset_gripper,
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

        with self._joint_state_lock:
            # copy to avoid holding the lock while writing into MuJoCo buffers
            joint_copy = dict(self._latest_joint_state)
            for name, position in joint_copy.items():
                if not np.isfinite(position):
                    self.get_logger().warn(f"Skipping non-finite joint '{name}': {position}")
                    continue

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
        with self._reset_lock:
            # If a final reset has been requested (model/data available),
            # handle swap in the sim thread.
            if self._reset_state == "requested":
                self.get_logger().info("Handling MuJoCo reset now...")
                self._reset_state = "handling"

                self.get_logger().info("Performing MuJoCo reset in sim thread.")

                if self._pending_model is not None:
                    self.model = self._pending_model
                if self._pending_data is not None:
                    self.data = self._pending_data
                    
               # After swapping self.model/self.data
                if self.data.qpos.shape[0] != self.model.nq or self.data.qvel.shape[0] != self.model.nv:
                    self.get_logger().error(
                        f"Model/Data mismatch after reload: "
                        f"len(qpos)={self.data.qpos.shape[0]} vs model.nq={self.model.nq}, "
                        f"len(qvel)={self.data.qvel.shape[0]} vs model.nv={self.model.nv}"
                    )
                    self._pending_model = None
                    self._pending_data = None
                    self._reset_state = "idle"
                    return

                self.joint_name_to_qpos = self._build_joint_mapping(mujoco)

                self.ee_fk_body_id = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_BODY, "ee_fk_frame"
                )
                self.ee_target_body_id = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_BODY, "ee_target_frame"
                )
                
                self._publish_ee_target_on_reset()


                self._hard_reset_mujoco_state()

                self._publish_ee_target_on_reset()
                self._publish_ee_target_on_reset()
                self._publish_ee_target_on_reset()
                self._publish_ee_target_on_reset()

                self._pending_model = None
                self._pending_data = None
                self._reset_state = "idle"
                self._skip_joint_apply_frames = 30
                
                with self._joint_state_lock:
                    self._latest_joint_state.clear()
                    self.get_logger().info("Cleared joint state buffer on reset.")


                return 

            # If we were notified that a reset is starting but the new model/data
            # have not yet been provided, enter paused mode: stop applying
            # incoming joint states and skip stepping until final notify arrives.
            if self._reset_state == "paused":
                self.get_logger().info("MuJoCo reset pending; waiting for new model/data...")
                
                # While paused do nothing (skip apply/step). This keeps the
                # simulation thread idle until the streamer provides the new
                # model/data via the second notify.
                return

        # 🚫 If we ever get here, NO reset is pending
        if self._skip_joint_apply_frames > 0:
            self._skip_joint_apply_frames -= 1
        else:
            self._apply_joint_state()        
        self._update_target_frames()
        mujoco.mj_step(self.model, self.data)

        if self.streamer:
            if self.streamer.is_sim_channel_open():
            # if True:
                self.streamer.update_sim()
                # Periodic logging (use helper so multiple messages/intervals can be used)
                self._periodic_log("update_scene", 2.0, "Updated MuJoCo scene...")
            else:
                self._periodic_log("webrtc", 1.5, "Sim channel not open, skipping update")
        # self._contact_active = self._detect_impact_contact()


    def _hard_reset_mujoco_state(self):
        # 1) reset dynamic state
        mujoco.mj_resetData(self.model, self.data)

        # 2) put robot in a known good configuration (critical)
        with self._joint_state_lock:
            joint_init = {
                "joint1": 0.0,
                "joint2": 0.0,
                "joint3": 0.0,
                "joint4": 0.0,
                "joint5": 0.0,
                "joint6": 0.0,
            }
            gripper = 100.0

        for name, position in joint_init.items():
            idx = self.joint_name_to_qpos.get(name)
            if idx is None:
                continue
            self.data.qpos[idx] = float(position)

        # gripper mapping (use your existing mapping)
        idx = self.joint_name_to_qpos.get("gripper_controller")
        if idx is not None:
            gl0, gu0 = -0.25, 0.8
            v0 = gl0 + (gu0 - gl0) * ((100.0 - gripper) / 100.0)
            self.data.qpos[idx]   = v0
            self.data.qpos[idx+1] = v0

            gl1, gu1 = 0.0, 0.8
            v1 = gl1 + (gu1 - gl1) * ((100.0 - gripper) / 100.0)
            self.data.qpos[idx+2] = v1
            self.data.qpos[idx+3] = v1

        # 3) zero velocities/actuators/controls
        if self.model.nv:
            self.data.qvel[:] = 0.0
        if self.model.na:
            self.data.act[:] = 0.0
        if self.model.nu:
            self.data.ctrl[:] = 0.0

        self.data.time = 0.0

        # 4) rebuild derived quantities once (critical before stepping)
        mujoco.mj_forward(self.model, self.data)


    def _periodic_log(self, key: str, interval: float, message: str, level: str = "info") -> None:
        """Log `message` no more often than `interval` seconds.

        - `key` is an identifier for this message stream (so multiple
          periodic logs can coexist independently).
        - `level` supports: "info", "warn"/"warning", "error", "debug".
        """
        now = time.time()
        if not hasattr(self, "_last_log_times"):
            self._last_log_times = {}
        last = float(self._last_log_times.get(key, 0.0))
        if last is None or now - last > float(interval):
            if level == "info":
                self.get_logger().info(message)
            elif level in ("warn", "warning"):
                # existing code uses warn in places; preserve that
                self.get_logger().warn(message)
            elif level == "error":
                self.get_logger().error(message)
            elif level == "debug":
                self.get_logger().debug(message)
            else:
                self.get_logger().info(message)
            self._last_log_times[key] = now



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


    def _on_streamer_reset(self, model, data) -> None:
        import threading

        with self._reset_lock:
            # Debug trace to help diagnose why callbacks may not be
            # observed by the ROS node. Log the types and calling thread.
            try:
                mtype = type(model).__name__ if model is not None else 'None'
            except Exception:
                mtype = str(model)
            try:
                dtype = type(data).__name__ if data is not None else 'None'
            except Exception:
                dtype = str(data)
            self.get_logger().info(
                f"_on_streamer_reset invoked (model={mtype}, data={dtype}, thread={threading.current_thread().name})"
            )
            # Two-phase notification protocol:
            #  - First call: (model, data) == (None, None) => reset STARTED
            #    -> pause simulation updates until final notify.
            #  - Second call: model/data provided => perform pending swap.

            if model is None and data is None:
                # Start phase
                if self._reset_state != "idle":
                    self.get_logger().info("Reset already in progress; ignoring duplicate start notification.")
                    return
                self.get_logger().info("MuJoCo reset start received; pausing simulation updates.")
                self._pending_model = None
                self._pending_data = None
                self._reset_state = "paused"
                return

            # Final phase: model/data provided
            # Accept final notify even if we are in paused state.
            if self._reset_state not in {"idle", "paused"}:
                self.get_logger().info("Reset already in progress; ignoring duplicate VisionOS reset.")
                return

            self.get_logger().info("MuJoCo reset requested.")
            self._pending_model = model
            self._pending_data = data
            self._reset_state = "requested"


    def _publish_ee_target_on_reset(self):
        msg = TeleopTarget()

        # Header
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.header.frame_id = "mycobot_base"

        # Pose
        pos = self.get_parameter("ee_target_on_reset_position").value
        quat = self.get_parameter("ee_target_on_reset_orientation_xyzw").value

        msg.pose.pose.position.x = float(pos[0])
        msg.pose.pose.position.y = float(pos[1])
        msg.pose.pose.position.z = float(pos[2])

        msg.pose.pose.orientation.x = float(quat[0])
        msg.pose.pose.orientation.y = float(quat[1])
        msg.pose.pose.orientation.z = float(quat[2])
        msg.pose.pose.orientation.w = float(quat[3])

        # Gripper
        msg.gripper = int(self.get_parameter("ee_target_on_reset_gripper").value)

        self.ee_target_pub.publish(msg)

        self.get_logger().info("Published ee_target reset pose on /teleop/ee_target")

            
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

            # if not hasattr(self, "_last_motor_log") or (self.get_clock().now().nanoseconds - getattr(self, "_last_motor_log", 0)) > 3e8:
            #     self.get_logger().info(f"Motor sound generated at speed: {speed}")
            #     self._last_motor_log = self.get_clock().now().nanoseconds


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
