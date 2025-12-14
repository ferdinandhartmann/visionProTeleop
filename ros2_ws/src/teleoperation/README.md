# Teleoperation


## Data flow and key nodes

1. **Vision Pro transforms** – `vp_transform_publisher.py` listens to the Vision Pro stream and publishes TF frames such as `visionpro/right/wrist` and fingertip frames under the configurable `vp_base` tree.
2. **Target computation** – `teleop_control_cpp` reads the TF frames, applies calibration offsets, converts the wrist pose into an end-effector goal, and derives a gripper percentage from the right-hand pinch distance. It publishes `teleoperation/TeleopTarget` on `/teleop/ee_target_` and the frame `ee_target_offset_mycobot_base`.
3. **Inverse kinematics** – The `inverse_kinematics` package subscribes to `/teleop/ee_target`, solves for joint angles, and republishes joint targets (see its README for details).
4. **Send to Robot** – `joint_state_to_mycobot.py` gets the pose and gripper command from inverse_kinematics node and sends it to the robot. It is subscribed with a timer and sends joint commands when the angle change is bigger than some limit.
5. **Keyboard fallback** – `keyboard_ee_teleop.py` publishes targets without any Vision Pro and robot.

## Check the images in the ros2_ws folder to see the tf and the node graph

## Custom message: `teleoperation/TeleopTarget`

```
geometry_msgs/PoseStamped pose
int32 gripper  # Desired gripper opening in percent (0 = closed, 100 = open)
```

- The pose is expressed in the `mycobot_base` frame so the IK solver can work directly in the robot base.
- `teleop_control_cpp` maps the right-hand pinch distance to `gripper` (0–100%).

## Launch options

### Full Vision Pro + robot stack

```bash
ros2 launch teleoperation launch_teleoperation.launch.py
```

### No Vision Pro and no Robot (keyboard-only)

```bash
ros2 launch teleoperation keyboard_launch_teleoperation.launch.py

ros2 run teleoperation keyboard_ee_teleop.py
```

This brings up RViz, the C++ teleop node, the IK solver, and the keyboard controller. You can drive the end-effector pose with the keyboard and still publish `TeleopTarget` messages that include a gripper percentage, so downstream nodes behave the same way as with Vision Pro input.

## Configuration

- Teleop parameters live in `config/teleoperation.yaml` (frames, pinch thresholds, update rates).
- Static transforms for `map` → `mycobot_base` and `map` → `vp_base_origin` are set in the launch files. Adjust them if your physical setup differs.
- The launch files accept URDF/model overrides through the `model` argument, making it easy to swap robot variants.