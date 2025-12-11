# inverse_kinematics

This package provides a damped-least-squares inverse kinematics node for the six-axis MyCobot arm (with an adaptive gripper). It consumes end-effector targets from teleoperation, enforces joint limits, and outputs joint states for visualization and hardware drivers.

## Node: `inverse_kinematics_node`

- **Subscription:** `/teleop/ee_target` (`teleoperation/TeleopTarget`)
- **Publishers:**
  - `/joint_states` (`sensor_msgs/JointState`) – continuous stream for RViz and other ROS 2 tools
  - `/joint_states_mycobot` (`sensor_msgs/JointState`) – intended for the downstream cobot driver

## Input message contract

`teleoperation/TeleopTarget` bundles the end-effector pose and a gripper command:

```
geometry_msgs/PoseStamped pose  # expressed in the mycobot_base frame
int32 gripper                    # 0 = closed, 100 = fully open
```

The node converts `pose` into joint angles using the provided Denavit–Hartenberg table. The `gripper` field is mapped to the final gripper joint position so the cobot driver receives coordinated arm + gripper commands from a single message.

## Configuration and parameters

Key parameters live in `config/inverse_kinematics.yaml`:
- `target_topic` and `joint_target_topic` let you retarget the subscription/publisher topics.
- `position_tolerance`, `orientation_tolerance`, `max_iterations`, `damping`, and `step_tolerance` tune the solver.
- `joint_lower_limits_deg` / `joint_upper_limits_deg` set safety bounds for the six arm joints.
- `gripper_percent` provides a fallback opening percentage before any teleop messages arrive.

Launch `inverse_kinematics_node` directly or via the teleoperation launch files; it will automatically solve incoming `TeleopTarget` messages and republish joint targets to the cobot node.
