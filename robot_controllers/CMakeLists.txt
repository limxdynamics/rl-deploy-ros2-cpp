# Copyright information
#
# © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

# Set the minimum required CMake version
cmake_minimum_required(VERSION 3.10)

# Define the project name
project(robot_controllers)

# Enable generation of compile commands database (used by some code editors)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdf REQUIRED)
find_package(limxsdk_lowlevel REQUIRED)

# Add a shared library target
add_library(${PROJECT_NAME} SHARED
  src/ControllerBase.cpp
  src/PointfootController.cpp
  src/SolefootController.cpp
  src/WheelfootController.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
  include 
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  geometry_msgs
  Eigen3
  urdf
  limxsdk_lowlevel
)

# Link with onnxruntime library
target_link_libraries(${PROJECT_NAME}
  onnxruntime          
)

pluginlib_export_plugin_description_file(controller_interface robot_controllers_plugin.xml)

# install
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(
  ament_index_cpp
  controller_interface
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  geometry_msgs
  Eigen3
  limxsdk_lowlevel
)

ament_package()