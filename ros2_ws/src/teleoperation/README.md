# teleoperation

The teleoperation package bridges Apple Vision Pro tracking, keyboard-based controls, and the MyCobot arm. It converts Vision Pro hand poses (or keyboard commands) into end-effector targets, computes gripper intent, and forwards everything to inverse kinematics and robot drivers.

## Data flow and key nodes

1. **Vision Pro transforms** – `vp_transform_publisher.py` listens to the Vision Pro stream and publishes TF frames such as `visionpro/right/wrist` and fingertip frames under the configurable `vp_base` tree.
2. **Target computation** – `teleop_control_cpp` reads the TF frames, applies calibration offsets, converts the wrist pose into an end-effector goal, and derives a gripper percentage from the right-hand pinch distance. It publishes `teleoperation/TeleopTarget` on `/teleop/ee_target`.
3. **Inverse kinematics** – The `inverse_kinematics` package subscribes to `/teleop/ee_target`, solves for joint angles, and republishes joint targets (see its README for details).
4. **Robot command fan-out** – Optional helper nodes in this package translate joint targets for specific hardware (`joint_state_to_mycobot.py`) or publish visualization data (RViz, camera overlay).
5. **Keyboard fallback** – `keyboard_ee_teleop.py` publishes targets without any Vision Pro hardware. Launch it via the keyboard launch file when you want to drive RViz or the IK solver without a headset or robot attached.

## Custom message: `teleoperation/TeleopTarget`

```
geometry_msgs/PoseStamped pose
int32 gripper  # Desired gripper opening in percent (0 = closed, 100 = open)
```

- The pose is expressed in the `mycobot_base` frame so the IK solver can work directly in the robot base.
- `teleop_control_cpp` maps the right-hand pinch distance to `gripper` (0–100%). If Vision Pro is unavailable, `keyboard_ee_teleop.py` fills this field from key presses instead.

## Launch options

### Full Vision Pro + robot stack
Use the main launch file when you have both the Vision Pro stream and a connected MyCobot:

```bash
ros2 launch teleoperation launch_teleoperation.launch.py
```

This starts TF publishers, the C++ teleop node, the inverse kinematics node, RViz, and the MyCobot interface. The end-effector pose and gripper percentage travel together in the `TeleopTarget` message from teleop to IK and on to the cobot node.

### Vision Pro without robot hardware
Run the same launch file but disable or disconnect the hardware side; RViz will still show the streamed hand targets and IK results. Because the teleop node owns the serial port, no additional changes are required to omit the cobot driver.

### No Vision Pro (keyboard-only)
When you want to test without a headset (and optionally without a robot), use the keyboard launch file:

```bash
ros2 launch teleoperation keyboard_launch_teleoperation.launch.py
```

This brings up RViz, the C++ teleop node, the IK solver, and the keyboard controller. You can drive the end-effector pose with the keyboard and still publish `TeleopTarget` messages that include a gripper percentage, so downstream nodes behave the same way as with Vision Pro input.

## Configuration

- Teleop parameters live in `config/teleoperation.yaml` (frames, pinch thresholds, update rates).
- Static transforms for `map` → `mycobot_base` and `map` → `vp_base_origin` are set in the launch files. Adjust them if your physical setup differs.
- The launch files accept URDF/model overrides through the `model` argument, making it easy to swap robot variants.

## How everything ties together

1. Vision Pro (or keyboard) produces an end-effector target and gripper percentage as `TeleopTarget`.
2. The inverse kinematics node consumes that message, solves for joint angles, and publishes `/joint_states` plus cobot-specific joint commands.
3. If a real robot is present, `joint_state_to_mycobot.py` sends the joint values to hardware; otherwise, RViz visualizes the same stream, so you can test with or without the robot attached.
