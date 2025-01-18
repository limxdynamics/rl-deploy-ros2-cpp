// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#include "robot_controllers/ControllerBase.h"

#include "controller_interface/helpers.hpp"

namespace robot_controllers {

controller_interface::CallbackReturn ControllerBase::on_init() {
  // Retrieve the robot type from the environment variable "ROBOT_TYPE"
  const char* value = ::getenv("ROBOT_TYPE");
  if (value && strlen(value) > 0) {
    robot_type_ = std::string(value);
  } else {
    RCLCPP_FATAL(rclcpp::get_logger("ControllerBase"), "Error: Please set the ROBOT_TYPE using 'export ROBOT_TYPE=<robot_type>'.");
    abort();
  }

  // Determine the specific robot configuration based on the robot type
  is_point_foot_ = (robot_type_.find("PF") != std::string::npos);
  is_wheel_foot_ = (robot_type_.find("WF") != std::string::npos);
  is_sole_foot_  = (robot_type_.find("SF") != std::string::npos);

  // Declare parameters
  if (!get_node()->has_parameter("ControllerCfg.joint_names")) {
    get_node()->declare_parameter<std::vector<std::string>>("ControllerCfg.joint_names", jointNames_);
  }

  if (!get_node()->get_parameter("ControllerCfg.joint_names", jointNames_)) {
    RCLCPP_FATAL(rclcpp::get_logger("ControllerBase"), "Failed to get 'ControllerCfg.joint_names' parameter");
    
    return controller_interface::CallbackReturn::ERROR;
  }

  std::string jointNamesPrint = "";
  for (const auto &joint_name : jointNames_) {
    jointNamesPrint += joint_name + ", ";
  }

  RCLCPP_WARN(rclcpp::get_logger("ControllerBase"), "jointNames: [%s]", jointNamesPrint.c_str());

  return (this->onInit() ? controller_interface::CallbackReturn::SUCCESS : controller_interface::CallbackReturn::ERROR);
  
}

controller_interface::return_type ControllerBase::update(const rclcpp::Time& , const rclcpp::Duration&) {
  this->onUpdate();
  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration ControllerBase::command_interface_configuration() const {
  const std::vector<std::string> joint_command_interface_types = {"position",
                                                                  "velocity",
                                                                  "effort",
                                                                  "kp",
                                                                  "kd",
                                                                  "mode"};
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : jointNames_) {
    for (const auto &interface_type : joint_command_interface_types) {
      config.names.push_back(joint_name + "/" + interface_type);
    }
  }
  return config;
}

controller_interface::InterfaceConfiguration ControllerBase::state_interface_configuration() const {
  const std::vector<std::string> joint_state_interface_types = {"position",
                                                                "velocity",
                                                                "effort"};

  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto &joint_name : jointNames_) {
    for (const auto &interface_type : joint_state_interface_types) {
      config.names.push_back(joint_name + "/" + interface_type);
    }
  }
  config.names.push_back("imu/orientation_x");
  config.names.push_back("imu/orientation_y");
  config.names.push_back("imu/orientation_z");
  config.names.push_back("imu/orientation_w");
  config.names.push_back("imu/angular_velocity_x");
  config.names.push_back("imu/angular_velocity_y");
  config.names.push_back("imu/angular_velocity_z");
  config.names.push_back("imu/linear_acceleration_x");
  config.names.push_back("imu/linear_acceleration_y");
  config.names.push_back("imu/linear_acceleration_z");

  return config;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ControllerBase::on_activate(
    const rclcpp_lifecycle::State &previous_state) {
  controller_interface::ControllerInterface::on_activate(previous_state);
  command_interfaces_map_.clear();
  state_interfaces_map_.clear();
  for (auto &interface : command_interfaces_) {
    command_interfaces_map_[interface.get_full_name()] = &interface;
  }
  for (auto &interface : state_interfaces_) {
    state_interfaces_map_[interface.get_full_name()] = &interface;
  }

  this->onStart();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ControllerBase::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {

  this->onStop();
  
  controller_interface::ControllerInterface::on_deactivate(previous_state);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void ControllerBase::setJointCommandValue(const std::string &joint_name, const std::string &type, double value) {
  if (command_interfaces_map_.count(joint_name + "/" + type)) {
    command_interfaces_map_[joint_name + "/" + type]->set_value(value);
  } else {
    RCLCPP_FATAL(rclcpp::get_logger("ControllerBase"),
                  "setJointCommandValue jointName: %s, type: %s failed!", joint_name.c_str(), type.c_str());
    abort();
  }
}

double ControllerBase::getJointStateValue(const std::string &joint_name, const std::string &type) {
  if (state_interfaces_map_.count(joint_name + "/" + type)) {
    return state_interfaces_map_[joint_name + "/" + type]->get_value();
  } else {
    RCLCPP_FATAL(rclcpp::get_logger("ControllerBase"),
                  "getJointStateValue jointName: %s, type: %s failed!", joint_name.c_str(), type.c_str());
    abort();
  }
  return 0.0;
}

double ControllerBase::getSensorValue(const std::string &sensor_name, const std::string &type) {
  if (state_interfaces_map_.count(sensor_name + "/" + type)) {
    return state_interfaces_map_[sensor_name + "/" + type]->get_value();
  } else {
    RCLCPP_FATAL(rclcpp::get_logger("ControllerBase"),
                  "getSensorValue sensorName: %s, type: %s failed!", sensor_name.c_str(), type.c_str());
    abort();
  }
  return 0.0;
}

} // namespace
