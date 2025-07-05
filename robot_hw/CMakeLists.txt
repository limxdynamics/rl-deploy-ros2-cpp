# Copyright information
#
# © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

# Set the minimum required CMake version
cmake_minimum_required(VERSION 3.10)

project(robot_hw)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(limxsdk_lowlevel REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(pointfoot_node 
  src/HardwareBase.cpp
  src/PointfootHardware.cpp
  src/HardwareLoop.cpp
  src/PointfootHardwareNode.cpp
)
ament_target_dependencies(pointfoot_node
  rclcpp
  hardware_interface
  controller_manager
  controller_manager_msgs
  urdf
  limxsdk_lowlevel
)

install(TARGETS pointfoot_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
