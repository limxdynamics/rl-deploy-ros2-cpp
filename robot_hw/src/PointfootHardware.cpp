// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#include "robot_hw/PointfootHardware.h"

namespace robot_hw {

// Constructor for the PointfootHardware class
// Initializes the base hardware with the PointFoot instance from limxsdk
PointfootHardware::PointfootHardware()
  : robot_hw::HardwareBase(limxsdk::PointFoot::getInstance()) { }

/**
 * @brief Parses the joint index based on the joint name
 * 
 * This function determines the leg and joint indices from the joint name
 * and returns a unique index for the joint. The index is calculated based
 * on the leg (left or right) and the specific joint (abad, hip, knee).
 * 
 * @param jointName Name of the joint (e.g., "L_abad", "R_hip", "L_knee")
 * @return int Unique joint index or -1 if parsing fails
 */
int PointfootHardware::parseJointIndex(const std::string& jointName) {
  int leg_index;
  int joint_index;

  // Determine the leg index based on the joint name prefix
  if (jointName.find("L_") != std::string::npos) {
    leg_index = 0; // Left leg
  } else if (jointName.find("R_") != std::string::npos) {
    leg_index = 1; // Right leg
  } else {
    RCLCPP_FATAL(rclcpp::get_logger("PointfootHardware"), "parseJointIndex fail!"); // Log fatal error if leg cannot be determined
    return -1;
  }

  // Determine the joint index based on the joint name suffix
  if (jointName.find("abad") != std::string::npos) {
    joint_index = 0; // Abad joint
  } else if (jointName.find("hip") != std::string::npos) {
    joint_index = 1; // Hip joint
  } else if (jointName.find("knee") != std::string::npos) {
    joint_index = 2; // Knee joint
  } else if (jointName.find("wheel") != std::string::npos) {
    joint_index = 3; // wheel joint
  } else if (jointName.find("ankle") != std::string::npos) {
    joint_index = 3; // ankle joint
  } else {
    RCLCPP_FATAL(rclcpp::get_logger("PointfootHardware"), "parseJointIndex fail!"); // Log fatal error if joint cannot be determined
    return -1;
  }

  // Calculate and return the unique joint index
  return leg_index * robot_->getMotorNumber() / 2 + joint_index;
}

}  // namespace robot_hw
