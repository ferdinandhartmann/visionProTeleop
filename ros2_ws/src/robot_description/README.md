### Load the xml in Mujoco

```bash
cd ~/visionpro_teleop_project/visionProTeleop/ros2_ws/src/robot_description/mycobot_mujoco
```
```bash
# Load the mycobot_280jn_mujoco.xml model
python3 - <<'EOF'
import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("xml/mycobot_280jn_mujoco.xml")
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
EOF


# Load the scene.xml scene
python3 - <<'EOF'
import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("scene_mycobot.xml")
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
EOF
```

```bash
cd ~/visionpro_teleop_project/visionProTeleop/ros2_ws/src/robot_description/mujoco_models
```

```bash
python3 - <<'EOF'
import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("mycobot_280.xml")
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
EOF
```
