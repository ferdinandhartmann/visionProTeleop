### Load the xml in Mujoco

```bash
cd ~/visionpro_teleop_project/visionProTeleop/ros2_ws/src/robot_description/mycobot_mujoco
```
```bash
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
^CTraceback (most recent call last):
  File "<stdin>", line 10, in <module>
  File "/home/ferdinand/.local/lib/python3.10/site-packages/mujoco/viewer.py", line 236, in sync
    sim.sync(state_only)  # locks internally
KeyboardInterrupt

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
```bash
