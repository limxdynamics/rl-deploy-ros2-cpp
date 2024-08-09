// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#include "robot_hw/HardwareBase.h"

namespace robot_hw {

/**
 * @class HardwareBase
 * @brief A class to interface with the robot hardware, manage state and command interfaces,
 *        and handle the communication with the underlying robot SDK.
 */

/**
 * @brief Constructor for the HardwareBase class.
 * 
 * Initializes the hardware base, sets up buffers, subscribes to state and IMU data.
 * 
 * @param robot A pointer to the robot SDK API base class.
 */
HardwareBase::HardwareBase(limxsdk::ApiBase* robot) : robot_(robot) {
  jointData_.resize(robot_->getMotorNumber());

  // Initializing robot command instance, state buffer and imu buffer
  robotCmd_ = limxsdk::RobotCmd(robot_->getMotorNumber());
  robotStateBuffer_.writeFromNonRT(limxsdk::RobotState(robot_->getMotorNumber()));
  imuDataBuffer_.writeFromNonRT(limxsdk::ImuData());

  // Subscribing to robot state
  robot_->subscribeRobotState([this](const limxsdk::RobotStateConstPtr& msg) {
    robotStateBuffer_.writeFromNonRT(*msg);
  });

  // Subscribing to robot imu
  robot_->subscribeImuData([this](const limxsdk::ImuDataConstPtr& msg) {
    imuDataBuffer_.writeFromNonRT(*msg);
  });
}

/**
 * @brief Exports the state interfaces for the hardware.
 * 
 * Provides interfaces for the robot's joint and IMU states.
 * 
 * @return A vector of hardware_interface::StateInterface containing the state interfaces.
 */
std::vector<hardware_interface::StateInterface> HardwareBase::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Export motor state interfaces
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    int idx = parseJointIndex(info_.joints[i].name);
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, "position", &jointData_[idx].pos_));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, "velocity", &jointData_[idx].vel_));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, "effort", &jointData_[idx].tau_));
  }

  // Export IMU state interfaces
  state_interfaces.emplace_back(
      hardware_interface::StateInterface("imu", "orientation_x", &imuData_.orientation_x_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface("imu", "orientation_y", &imuData_.orientation_y_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface("imu", "orientation_z", &imuData_.orientation_z_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface("imu", "orientation_w", &imuData_.orientation_w_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface("imu", "angular_velocity_x", &imuData_.angular_velocity_x_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface("imu", "angular_velocity_y", &imuData_.angular_velocity_y_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface("imu", "angular_velocity_z", &imuData_.angular_velocity_z_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface("imu", "linear_acceleration_x", &imuData_.linear_acceleration_x_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface("imu", "linear_acceleration_y", &imuData_.linear_acceleration_y_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface("imu", "linear_acceleration_z", &imuData_.linear_acceleration_z_));

  return state_interfaces;
}

/**
 * @brief Exports the command interfaces for the hardware.
 * 
 * Provides interfaces for the robot's joint commands.
 * 
 * @return A vector of hardware_interface::CommandInterface containing the command interfaces.
 */
std::vector<hardware_interface::CommandInterface> HardwareBase::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Export motor command interfaces
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    int idx = parseJointIndex(info_.joints[i].name);
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(info_.joints[i].name, "position", &jointData_[idx].posDes_));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(info_.joints[i].name, "velocity", &jointData_[idx].velDes_));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(info_.joints[i].name, "effort", &jointData_[idx].tau_ff_));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(info_.joints[i].name, "kp", &jointData_[idx].kp_));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(info_.joints[i].name, "kd", &jointData_[idx].kd_));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(info_.joints[i].name, "mode", &jointData_[idx].mode_));
  }

  return command_interfaces;
}

/**
 * @brief Gets the name of the hardware.
 * 
 * @return A string containing the name of the hardware.
 */
std::string HardwareBase::get_name() const {
  return "HardwareBase";
}

hardware_interface::return_type HardwareBase::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  // Reading robot state
  limxsdk::RobotState robotState = *robotStateBuffer_.readFromRT();
  for (int i = 0; i < robot_->getMotorNumber(); ++i) {
    jointData_[i].pos_ = robotState.q[i];
    jointData_[i].vel_ = robotState.dq[i];
    jointData_[i].tau_ = robotState.tau[i];
  }
  // Reading imu data
  limxsdk::ImuData imuData = *imuDataBuffer_.readFromRT();
  imuData_.orientation_x_ = imuData.quat[1];
  imuData_.orientation_y_ = imuData.quat[2];
  imuData_.orientation_z_ = imuData.quat[3];
  imuData_.orientation_w_ = imuData.quat[0];
  imuData_.angular_velocity_x_ = imuData.gyro[0];
  imuData_.angular_velocity_y_ = imuData.gyro[1];
  imuData_.angular_velocity_z_ = imuData.gyro[2];
  imuData_.linear_acceleration_x_ = imuData.acc[0];
  imuData_.linear_acceleration_y_ = imuData.acc[1];
  imuData_.linear_acceleration_z_ = imuData.acc[2];

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HardwareBase::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  // Writing commands to robot
  if (can_publish_cmd_) {
    for (int i = 0; i < robot_->getMotorNumber(); ++i) {
      robotCmd_.q[i] = static_cast<float>(jointData_[i].posDes_);
      robotCmd_.dq[i] = static_cast<float>(jointData_[i].velDes_);
      robotCmd_.Kp[i] = static_cast<float>(jointData_[i].kp_);
      robotCmd_.Kd[i] = static_cast<float>(jointData_[i].kd_);
      robotCmd_.tau[i] = static_cast<float>(jointData_[i].tau_ff_);
      robotCmd_.mode[i] = static_cast<float>(jointData_[i].mode_);
    }
    robotCmd_.stamp = rclcpp::Clock().now().nanoseconds();

    // Publishing robot commands
    robot_->publishRobotCmd(robotCmd_);
  }

  return hardware_interface::return_type::OK;
}

/**
 * @brief Configures the hardware with the given information.
 * 
 * @param info The hardware information to configure.
 * @return hardware_interface::return_type indicating the result of the configure operation.
 */
hardware_interface::return_type HardwareBase::configure(const hardware_interface::HardwareInfo& info) {
  info_ = info;
  return hardware_interface::return_type::OK;
}

/**
 * @brief Starts the hardware.
 * 
 * @return hardware_interface::return_type indicating the result of the start operation.
 */
hardware_interface::return_type HardwareBase::start() {
  return hardware_interface::return_type::OK;
}

}  // namespace robot_hw
