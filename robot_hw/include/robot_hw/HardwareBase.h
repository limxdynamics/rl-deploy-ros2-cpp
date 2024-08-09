// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#ifndef _LIMX_HARDWARE_BASE_H_
#define _LIMX_HARDWARE_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "robot_hw/RobotData.h"
#include "limxsdk/apibase.h"

namespace robot_hw {

/**
 * @brief A base class for robot hardware interface.
 */
class HardwareBase : public hardware_interface::SystemInterface {
public:
  /**
   * @brief Constructor for HardwareBase.
   * 
   * @param robot Pointer to the robot API base.
   */
  HardwareBase(limxsdk::ApiBase* robot);

  /**
   * @brief Pure virtual function to parse the joint index from the joint name.
   * 
   * @param jointName The name of the joint.
   * @return The index of the joint.
   */
  virtual int parseJointIndex(const std::string& jointName) = 0;

  /**
   * @brief Exports the state interfaces of the hardware.
   * 
   * @return A vector of state interfaces.
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * @brief Exports the command interfaces of the hardware.
   * 
   * @return A vector of command interfaces.
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * @brief Gets the name of the hardware.
   * 
   * @return The name of the hardware.
   */
  std::string get_name() const override;

  hardware_interface::return_type start();
  hardware_interface::return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;
  hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo& info);

  /**
   * @brief Retrieves the robot instance.
   * 
   * This function returns a pointer to the `ApiBase` instance representing the robot.
   * 
   * @return A pointer to the `limxsdk::ApiBase` instance representing the robot.
   */
  limxsdk::ApiBase* getRobot() { return robot_; }

  /**
   * @brief Enables or disables the ability to publish robot commands.
   * 
   * @param enable True to enable publishing of robot commands, False to disable.
   */
  void setRobotCmdEnable(bool enable) { can_publish_cmd_ = enable; }

protected:
  hardware_interface::HardwareInfo info_; ///< Hardware information.
  std::vector<MotorData> jointData_; ///< Data for each joint.
  ImuData imuData_{}; ///< IMU data.
  
  realtime_tools::RealtimeBuffer<limxsdk::RobotState> robotStateBuffer_; ///< Buffer for robot state.
  realtime_tools::RealtimeBuffer<limxsdk::ImuData> imuDataBuffer_; ///< Buffer for IMU data.
  limxsdk::RobotCmd robotCmd_; ///< Robot command structure.
  std::map<std::string, int> joystickBtnMap_; ///< Map for joystick button configuration.
  std::map<std::string, int> joystickAxesMap_; ///< Map for joystick axes configuration.

  limxsdk::ApiBase* robot_; ///< Pointer to the robot API base.
  bool can_publish_cmd_{true}; ///< Flag indicating if publishing robot commands is enabled.
};
}  // namespace robot_hw

#endif // _LIMX_HARDWARE_BASE_H_
