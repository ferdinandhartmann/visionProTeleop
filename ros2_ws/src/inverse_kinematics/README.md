# inverse_kinematics

This package provides a simple damped-least-squares inverse kinematics node for the six-axis arm described by the provided Denavit–Hartenberg parameters. The node listens for end-effector pose targets and publishes the resulting joint positions.

## Node: `inverse_kinematics_node`

- **Subscription:** `target_topic` (`geometry_msgs/PoseStamped`, default: `/teleop/ee_target`)
- **Publisher:** `joint_target_topic` (`sensor_msgs/JointState`, default: `/teleop/joint_targets`)

The solver uses the supplied DH table (units converted from millimeters to meters) and iteratively solves for a joint configuration that matches both position and orientation. Adjust the parameters on startup to point at different topics if needed.
