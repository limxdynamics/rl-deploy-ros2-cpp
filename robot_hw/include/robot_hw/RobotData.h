// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#ifndef _LIMX_ROBOT_DATA_H_
#define _LIMX_ROBOT_DATA_H_

#include <mutex>
#include <sstream>
#include <string>

namespace robot_hw {

/**
 * @brief Structure to store data related to a motor.
 */
struct MotorData {
  /**
   * @brief Constructor to initialize motor command variables.
   */
  MotorData() {
    // Initialize state variables
    pos_ = vel_ = tau_ = 0.0;
    // Initialize command variables
    posDes_ = velDes_ = kp_ = kd_ = tau_ff_ = 0.0;
    mode_ = 0;
  }

  // State variables
  double pos_;  ///< Motor position
  double vel_;  ///< Motor velocity
  double tau_;  ///< Motor torque

  // Command variables
  double posDes_;  ///< Desired position
  double velDes_;  ///< Desired velocity
  double kp_;      ///< Proportional gain
  double kd_;      ///< Derivative gain
  double tau_ff_;  ///< Feedforward torque
  double mode_;    ///< Control mode
};

/**
 * @brief Structure to store IMU data.
 */
struct ImuData {
  // Orientation
  double orientation_x_{0.0}; ///< Orientation in the x-axis
  double orientation_y_{0.0}; ///< Orientation in the y-axis
  double orientation_z_{0.0}; ///< Orientation in the z-axis
  double orientation_w_{0.0}; ///< Orientation in the w component (quaternion)

  // Angular velocity
  double angular_velocity_x_{0.0}; ///< Angular velocity in the x-axis
  double angular_velocity_y_{0.0}; ///< Angular velocity in the y-axis
  double angular_velocity_z_{0.0}; ///< Angular velocity in the z-axis

  // Linear acceleration
  double linear_acceleration_x_{0.0}; ///< Linear acceleration in the x-axis
  double linear_acceleration_y_{0.0}; ///< Linear acceleration in the y-axis
  double linear_acceleration_z_{1000.0}; ///< Linear acceleration in the z-axis
};

} // namespace robot_hw

#endif // _LIMX_ROBOT_DATA_H_
