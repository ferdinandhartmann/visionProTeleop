cmake_minimum_required(VERSION 3.8)
project(teleoperation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(teleop_control src/teleop_control.cpp)
target_include_directories(teleop_control PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(teleop_control
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS teleop_control
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  teleoperation/camera_streamer.py
  teleoperation/joint_state_to_mycobot.py
  teleoperation/keyboard_ee_teleop.py
  teleoperation/vp_transform_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
