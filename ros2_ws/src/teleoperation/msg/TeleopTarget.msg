# Combined teleoperation target consisting of an end-effector pose and gripper command.
geometry_msgs/PoseStamped pose
int32 gripper  # Desired gripper opening in percent (0 = closed, 100 = open)
