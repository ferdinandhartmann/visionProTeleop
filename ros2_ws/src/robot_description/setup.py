import os
from setuptools import setup, __version__ as setuptools_version
from packaging.version import Version
from glob import glob

package_name = 'robot_description'

# Checking the setuptools version
use_dash_separated_options = Version(setuptools_version) < Version("58.0.0")


# Dynamically generate setup.cfg content
setup_cfg_content = """
[develop]
{script_option}=$base/lib/{package_name}

[install]
{install_scripts_option}=$base/lib/{package_name}
""".format(
    package_name=package_name,
    script_option='script-dir' if use_dash_separated_options else 'script_dir',
    install_scripts_option='install-scripts' if use_dash_separated_options else 'install_scripts'
)

# Write the contents to setup.cfg
with open("setup.cfg", "w") as f:
    f.write(setup_cfg_content)

# Helper function to include all files in a directory
def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install MyCobot URDFs
        ('share/' + package_name + '/urdf/mycobot_280_m5', glob('urdf/mycobot_280_m5/*')),
        ('share/' + package_name + '/urdf/adaptive_gripper', glob('urdf/adaptive_gripper/*')),
        ('share/' + package_name + '/mycobot_mujoco', package_files('mycobot_mujoco')),
        # ('share/' + package_name + '/mycobot_mujoco', glob('mycobot_mujoco/')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='u2',
    maintainer_email='u2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
