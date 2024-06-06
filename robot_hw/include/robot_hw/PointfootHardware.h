// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#ifndef _LIMX_POINTFOOT_HARDWARE_H_
#define _LIMX_POINTFOOT_HARDWARE_H_

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "robot_hw/HardwareBase.h"
#include "robot_hw/RobotData.h"
#include "limxsdk/pointfoot.h"

namespace robot_hw {

/**
 * @brief Class representing the hardware interface for Pointfoot robots.
 */
class PointfootHardware: public robot_hw::HardwareBase {
public:
  /**
   * @brief Constructor for PointfootHardware.
   */
  PointfootHardware();

  /**
   * @brief Parses the joint index from the joint name.
   * 
   * @param jointName The name of the joint.
   * @return The index of the joint.
   */
  int parseJointIndex(const std::string& jointName) override;
};

}  // namespace robot_hw

#endif // _LIMX_POINTFOOT_HARDWARE_H_
