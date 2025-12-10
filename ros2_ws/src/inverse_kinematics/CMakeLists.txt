cmake_minimum_required(VERSION 3.8)
project(inverse_kinematics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(inverse_kinematics_node src/inverse_kinematics_node.cpp)
ament_target_dependencies(inverse_kinematics_node
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_link_libraries(inverse_kinematics_node Eigen3::Eigen)

install(TARGETS inverse_kinematics_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
