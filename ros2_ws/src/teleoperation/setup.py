from setuptools import find_packages, setup 
from glob import glob

package_name = 'teleoperation'

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/config", glob("config/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ferdinand",
    maintainer_email="ferdinand.hartmann@web.de",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_streamer = teleoperation.camera_streamer:main",
            "vp_transform_publisher = teleoperation.vp_transform_publisher:main",
            "keyboard_ee_teleop = teleoperation.keyboard_ee_teleop:main",
            "joint_state_to_mycobot = teleoperation.joint_state_to_mycobot:main",
            "mujoco_streamer_node = teleoperation.mujoco_streamer_node:main",
        ],
    },
)
