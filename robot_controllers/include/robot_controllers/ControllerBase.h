// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#ifndef _LIMX_CONTROLLER_BASE_H_
#define _LIMX_CONTROLLER_BASE_H_

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <map>
#include <functional>
#include <Eigen/Geometry>
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/duration.hpp"
#include "urdf/model.h"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace robot_controllers {

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @brief Base class for robot controllers.
 */
class CONTROLLER_INTERFACE_PUBLIC ControllerBase : public controller_interface::ControllerInterface {
public:
  /**
   * @brief Constructor for ControllerBase.
   */
  ControllerBase() {}

  /**
   * @brief Virtual function to initialize the controller.
   * 
   * @return True if initialization is successful, false otherwise.
   */
  virtual bool onInit() = 0;

  /**
   * @brief Virtual function to update the controller.
   */
  virtual void onUpdate() = 0;

  /**
   * @brief Virtual function to start the controller.
   */
  virtual void onStart() = 0;

  /**
   * @brief Virtual function to stop the controller.
   */
  virtual void onStop() = 0;

public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  /**
   * @brief Sets the command value for a joint.
   * 
   * @param joint_name Name of the joint.
   * @param type Type of command (e.g., position, velocity).
   * @param value Command value to set.
   */
  void setJointCommandValue(const std::string & joint_name, const std::string & type, double value);

  /**
   * @brief Gets the state value of a joint.
   * 
   * @param joint_name Name of the joint.
   * @param type Type of state (e.g., position, velocity).
   * @return The state value of the joint.
   */
  double getJointStateValue(const std::string & joint_name, const std::string & type);

  /**
   * @brief Gets the value of a sensor.
   * 
   * @param sensor_name Name of the sensor.
   * @param type Type of sensor data (e.g., orientation, acceleration).
   * @return The sensor value.
   */
  double getSensorValue(const std::string & sensor_name, const std::string & type);

  /**
   * @brief Declares and checks a parameter.
   * 
   * @param param_name Name of the parameter.
   * @param param_value Value of the parameter.
   * @return 0 if the parameter is successfully declared and retrieved, 1 otherwise.
   */
  template<typename T>
  int declareAndCheckParameter(const std::string & param_name, T & param_value) {
    if (!get_node()->has_parameter(param_name)){
      get_node()->declare_parameter<T>(param_name, param_value);
    }
    return static_cast<int>(!get_node()->get_parameter(param_name, param_value));
  }

  std::map<std::string, hardware_interface::LoanedStateInterface*> state_interfaces_map_; ///< Map of state interfaces.
  std::map<std::string, hardware_interface::LoanedCommandInterface*> command_interfaces_map_; ///< Map of command interfaces.
  std::vector<std::string> jointNames_; ///< Vector of joint names.

  std::string robot_type_;      // Type of the robot (e.g., point foot, wheel foot, sole foot)
  bool is_point_foot_{false};   // Indicates if the robot has a point foot configuration
  bool is_wheel_foot_{false};   // Indicates if the robot has a wheel foot configuration
  bool is_sole_foot_{false};    // Indicates if the robot has a sole foot configuration
};

/**
 * @brief Computes the square of a value.
 * 
 * @tparam T Type of the value.
 * @param a The value to be squared.
 * @return The square of the value.
 */
template <typename T>
T square(T a) {
  return a * a;
}

/**
 * @brief Converts a quaternion to ZYX Euler angles.
 * 
 * @tparam SCALAR_T Type of the scalar values.
 * @param q The quaternion to be converted.
 * @return A vector of ZYX Euler angles.
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T> &q) {
  Eigen::Matrix<SCALAR_T, 3, 1> zyx;

  SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
  zyx(0) = std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
  zyx(1) = std::asin(as);
  zyx(2) = std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
  return zyx;
}

/**
 * @brief Computes the rotation matrix from ZYX Euler angles.
 * 
 * @tparam SCALAR_T Type of the scalar values.
 * @param eulerAngles The ZYX Euler angles.
 * @return The rotation matrix.
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> getRotationMatrixFromZyxEulerAngles(const Eigen::Matrix<SCALAR_T, 3, 1> &eulerAngles) {
  const SCALAR_T z = eulerAngles(0);
  const SCALAR_T y = eulerAngles(1);
  const SCALAR_T x = eulerAngles(2);

  const SCALAR_T c1 = cos(z);
  const SCALAR_T c2 = cos(y);
  const SCALAR_T c3 = cos(x);
  const SCALAR_T s1 = sin(z);
  const SCALAR_T s2 = sin(y);
  const SCALAR_T s3 = sin(x);

  const SCALAR_T s2s3 = s2 * s3;
  const SCALAR_T s2c3 = s2 * c3;

  // Construct rotation matrix
  Eigen::Matrix<SCALAR_T, 3, 3> rotationMatrix;
  rotationMatrix << c1 * c2,      c1 * s2s3 - s1 * c3,       c1 * s2c3 + s1 * s3,
                    s1 * c2,      s1 * s2s3 + c1 * c3,       s1 * s2c3 - c1 * s3,
                    -s2,          c2 * s3,                   c2 * c3;
  return rotationMatrix;
}

} // namespace robot_controllers

#endif // _LIMX_CONTROLLER_BASE_H_
