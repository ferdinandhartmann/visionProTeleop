cmake_minimum_required(VERSION 3.8)
project(teleoperation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TeleopTarget.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

add_executable(teleop_control_cpp src/teleop_control_cpp.cpp)
target_include_directories(teleop_control_cpp PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(teleop_control_cpp
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Link against the generated C++ typesupport for TeleopTarget
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(teleop_control_cpp
  "${cpp_typesupport_target}")

install(TARGETS teleop_control_cpp
  DESTINATION lib/${PROJECT_NAME})

# ament_python_install_package(${PROJECT_NAME})

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

ament_export_dependencies(rosidl_default_runtime)

ament_package()
