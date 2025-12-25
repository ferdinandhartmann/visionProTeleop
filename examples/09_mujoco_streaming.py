"""Stream MuJoCo simulation to Vision Pro, replaying pre-recorded robot trajectories."""

import numpy as np
from tqdm import trange
import time
from pathlib import Path
import argparse

# Get the directory containing demo assets
ASSETS_DIR = Path(__file__).resolve().parent.parent / "assets" / "mujoco_demos"


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from avp_stream import VisionProStreamer



def main(args):
    import mujoco
    # Load the scene
    # Original demo path (kept for reference):
    # xml_path = str(ASSETS_DIR / "scenes" / "franka_emika_panda" / "scene_blockpush.xml")
    # Use custom mycobot scene instead:
    xml_path = "/home/ferdinand/visionpro_teleop_project/visionProTeleop/ros2_ws/src/robot_description/mycobot_mujoco/scene_mycobot.xml"
    # xml_path = "/home/ferdinand/visionpro_teleop_project/visionProTeleop/ros2_ws/src/robot_description/ufactory_xarm7/scene.xml"
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    streamer = None
    viewer_handle = None
    
    STREAM_CAMERA = True


    if args.viewer == "ar":
        # Vision Pro AR streaming via VisionProStreamer
        from avp_stream import VisionProStreamer
        streamer = VisionProStreamer(ip=args.ip, record=False)

        # attach_to format: [x, y, z, yaw_degrees]
        attach_to = [0.0, 0.6, 0.55, -90.0]

        streamer.configure_mujoco(
            xml_path=xml_path,
            model=model,
            data=data,
            relative_to=attach_to,
            grpc_port=args.port,
            force_reload=False,
        )
        
        if STREAM_CAMERA:
            from avp_stream import VisionProStreamer
            streamer.configure_video(device="/dev/video0", format="v4l2", size="1280x720", fps=25)
            print("Vision Pro camera streaming enabled")
            
        streamer.start_webrtc(port=9999)

    else:
        # Local MuJoCo viewer (no streaming)
        from mujoco import viewer as mj_viewer
        viewer_handle = mj_viewer.launch_passive(model, data)


    fixed_pos = True  # Set to True to move all joints to zero position

    movement = True  # Set to True to have joints move back and forth using motors


    logs_dir = ASSETS_DIR / "logs"
    episodes = sorted(logs_dir.glob("ep*.npz"))
    if fixed_pos == True:
        print("🔧 Moving joints")
        data.qpos[:] = 0.0
        data.qvel[:] = 0.0
        data.ctrl[:] = 0.0
        data.qacc_warmstart[:] = 1.0
        mujoco.mj_forward(model, data)
        sign = 1.0
        joints_angle = 0.0
        joint_angle_max = 0.8
        dt = model.opt.timestep  # e.g. 0.002 → 500 Hz

        hold_steps = 100  # Number of steps to hold at each end
        hold_counter = 0
        while True:
            if movement is True:
                if (joints_angle > joint_angle_max and sign > 0) or (joints_angle < -joint_angle_max and sign < 0):
                    if hold_counter < hold_steps:
                        hold_counter += 1
                    else:
                        sign *= -joint_angle_max
                        hold_counter = 0
                else:
                    joints_angle += 0.004 * sign
                    

            # Arm Joints
            data.ctrl[:-4] = joints_angle
            

            # Gripper
            gripper_lower_limit = -0.25
            gripper_upper_limit = 0.8
            data.ctrl[-4:-2] = (gripper_lower_limit + (gripper_upper_limit - gripper_lower_limit)) * (abs(joints_angle) / joint_angle_max)
            
            gripper_lower_limit = 0.0
            gripper_upper_limit = 0.8
            data.ctrl[-2:] = (gripper_lower_limit + (gripper_upper_limit - gripper_lower_limit)) * (abs(joints_angle) / joint_angle_max)
            

            # Advance physics one step
            mujoco.mj_step(model, data)

            # Sync to viewer / AR streamer
            if args.viewer == "ar" and streamer is not None:
                streamer.update_sim()
            elif viewer_handle is not None and viewer_handle.is_running():
                viewer_handle.sync()
            else:
                break
            
            time.sleep(dt)
    else:
        try:
            for ep_idx in range(1, min(5, len(episodes) + 1)):
                traj_path = logs_dir / f"ep{ep_idx}.npz"
                if not traj_path.exists():
                    continue

                print(f"📼 Playing episode {ep_idx}...")
                traj = np.load(traj_path)

                qpos_log = traj["qpos"]
                qvel_log = traj["qvel"]
                ctrl_log = traj["ctrl"]
                mocap_log = traj["mocap"]

                T = qpos_log.shape[0]

                # Initialize state
                data.qpos[:] = qpos_log[0]
                data.qvel[:] = qvel_log[0]

                if mocap_log.shape[0] > 0:
                    data.mocap_pos[1] = mocap_log[0, :3]
                    data.mocap_quat[1] = mocap_log[0, 3:]

                data.qacc_warmstart[:] = 0.0
                mujoco.mj_forward(model, data)

                # Replay trajectory using position targets (first nu joints)
                nu = model.nu
                for t in trange(T, desc=f"  Episode {ep_idx}", leave=False):
                    # Use recorded joint positions as desired angles for the servos
                    data.ctrl[:] = qpos_log[t, :nu]
                    mujoco.mj_step(model, data)
                    if args.viewer == "ar" and streamer is not None:
                        streamer.update_sim()
                    elif viewer_handle is not None and viewer_handle.is_running():
                        viewer_handle.sync()
                    else:
                        break
                    time.sleep(1 / 1000.0)

                print(f"  ✅ Episode {ep_idx} complete ({T} steps)")
            

        except KeyboardInterrupt:
            print(f"\n\n🛑 Stopped by user")

    print(f"\n✨ Demo complete!")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Stream MuJoCo simulation replay to Vision Pro"
    )
    parser.add_argument(
        "--viewer",
        default="ar",
        choices=["mujoco", "ar"],
        help="Viewer type: 'mujoco' for local preview, 'ar' for Vision Pro streaming",
    )
    parser.add_argument(
        "--ip",
        # default="192.168.50.153",
        # default="192.168.10.137",
        default="192.168.10.113",
        # default="192.168.11.99",
        # default="192.168.10.191",
        help="Vision Pro IP address (only used with --viewer ar)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=50051,
        help="MuJoCo gRPC port for USDZ transfer (default: 50051)",
    )
    args = parser.parse_args()

    main(args)




