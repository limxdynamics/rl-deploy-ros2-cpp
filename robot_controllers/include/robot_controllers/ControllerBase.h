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
  controller_interface::return_type init(const std::string & controller_name) override;
  controller_interface::return_type update() override;
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

// Define scalar and vector types for Eigen
using vector3_t = Eigen::Matrix<double, 3, 1>;
using vector_t = Eigen::Matrix<double, Eigen::Dynamic, 1>;
using matrix_t = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>; // Type alias for matrices
using tensor_element_t = float; // Type alias for tensor elements

// Utility class to measure time intervals
class TicToc {
public:
  TicToc() {
    tic();
  }

  void tic() {
    start = std::chrono::system_clock::now();
  }

  double toc() {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    return elapsed_seconds.count() * 1000;
  }

private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
};

// Structure to hold robot configuration settings
struct RobotCfg {
  // Control configuration settings
  struct ControlCfg {
    double stiffness{0.0};            // Stiffness parameter
    double damping{0.0};              // Damping parameter
    double action_scale_pos{0.0};     // Scaling factor for position action
    double action_scale_vel{0.0};     // Scaling factor for velocity action
    int decimation{0};                // Decimation factor
    double user_torque_limit{0.0};    // User-defined torque limit

    // Print control configuration settings
    void print() {
      RCLCPP_INFO(rclcpp::get_logger("RobotCfg"), "=======start ControlCfg========");
      RCLCPP_INFO(rclcpp::get_logger("RobotCfg"), "stiffness: %f", stiffness);
      RCLCPP_INFO(rclcpp::get_logger("RobotCfg"), "damping: %f", damping);
      RCLCPP_INFO(rclcpp::get_logger("RobotCfg"), "action_scale_pos: %f", action_scale_pos);
      RCLCPP_INFO(rclcpp::get_logger("RobotCfg"), "action_scale_vel: %f", action_scale_vel);
      RCLCPP_INFO(rclcpp::get_logger("RobotCfg"), "decimation: %d", decimation);
      RCLCPP_INFO(rclcpp::get_logger("RobotCfg"), "user_torque_limit: %f", user_torque_limit);
      RCLCPP_INFO(rclcpp::get_logger("RobotCfg"), "=======end ControlCfg========\n");
    }
  };

  // Reinforcement learning configuration settings
  struct RlCfg {
    // Observation scaling parameters
    struct ObsScales {
      double linVel{0.0};            // Linear velocity scaling
      double angVel{0.0};            // Angular velocity scaling
      double dofPos{0.0};            // Degree of freedom position scaling
      double dofVel{0.0};            // Degree of freedom velocity scaling

      // Print observation scaling parameters
      void print() {
        RCLCPP_INFO(rclcpp::get_logger("RobotCfg"), "=======start ObsScales========");
        RCLCPP_INFO(rclcpp::get_logger("RobotCfg"), "linVel: %f", linVel);
        RCLCPP_INFO(rclcpp::get_logger("RobotCfg"), "angVel: %f", angVel);
        RCLCPP_INFO(rclcpp::get_logger("RobotCfg"), "dofPos: %f", dofPos);
        RCLCPP_INFO(rclcpp::get_logger("RobotCfg"), "dofVel: %f", dofVel);
        RCLCPP_INFO(rclcpp::get_logger("RobotCfg"), "=======end ObsScales========\n");
      }
    };

    double clipActions{0.0};       // Action clipping parameter
    double clipObs{0.0};           // Observation clipping parameter
    ObsScales obsScales;           // Observation scaling settings
  };

  // User command configuration settings
  struct UserCmdCfg {
    double linVel_x{0.0}; 
    double linVel_y{0.0}; 
    double angVel_yaw{0.0}; 

    // Print user command scaling parameters
    void print() {
      RCLCPP_INFO(rclcpp::get_logger("RobotCfg"), "======= Start User Cmd Scales========");
      RCLCPP_INFO(rclcpp::get_logger("RobotCfg"), "lin_vel_x: %f", linVel_x);
      RCLCPP_INFO(rclcpp::get_logger("RobotCfg"), "lin_vel_y: %f", linVel_y);
      RCLCPP_INFO(rclcpp::get_logger("RobotCfg"), "ang_vel_yaw: %f", angVel_yaw);
      RCLCPP_INFO(rclcpp::get_logger("RobotCfg"), "=======End User Cmd Scales========\n");
    }
  };

  // gait settings
  struct GaitCfg
  {
    double frequencies{0.0};
    double swing_height{0.0};
    void print()
    {
      RCLCPP_INFO(rclcpp::get_logger("RobotCfg"), "=======Start GaitCfg========");
      RCLCPP_INFO(rclcpp::get_logger("RobotCfg"), "frequencies: %f", frequencies);
      RCLCPP_INFO(rclcpp::get_logger("RobotCfg"), "swing_height: %f", swing_height);
      RCLCPP_INFO(rclcpp::get_logger("RobotCfg"), "=======end GaitCfg========\n");
    }
  };

  RlCfg rlCfg;                   // RL configuration settings
  UserCmdCfg userCmdCfg;         // User command configuration settings
  GaitCfg gaitCfg;               // Gait configuration settings
  std::map<std::string, double> initState;  // Initial state settings
  ControlCfg controlCfg;         // Control configuration settings

  // Print robot configuration settings
  void print() {
    rlCfg.obsScales.print();
    controlCfg.print();
    userCmdCfg.print();
    gaitCfg.print();
    RCLCPP_INFO(rclcpp::get_logger("RobotCfg"), "clipActions: %f", rlCfg.clipActions);
    RCLCPP_INFO(rclcpp::get_logger("RobotCfg"), "clipObs: %f", rlCfg.clipObs);
  }
};
} // namespace robot_controllers

#endif // _LIMX_CONTROLLER_BASE_H_
