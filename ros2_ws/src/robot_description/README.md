### Load the xml in Mujoco


```bash
python3 -m mujoco.viewer

```
Then drag and drop the robot_description/mycobot_mujoco/xml/scene_mycobot.xml inside and you can edit the robot model or the scene and just klick reload

The Franka Vision Pro scene is `franka_mujoco/fr3_robotiq_2f85.xml`. Its
`${franka_description}` mesh token is resolved by `vp_streamer.py` at runtime,
so the rendered arm and Robotiq meshes come from the same description package
used by ROS.
