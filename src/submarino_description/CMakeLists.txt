cmake_minimum_required(VERSION 3.8)
project(submarino_description)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ros_gz REQUIRED)

install(
  DIRECTORY urdf meshes worlds launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()