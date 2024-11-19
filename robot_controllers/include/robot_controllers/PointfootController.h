// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#ifndef _LIMX_POINTFOOT_CONTROLLER_H_
#define _LIMX_POINTFOOT_CONTROLLER_H_

#include <onnxruntime_cxx_api.h>
#include <vector>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_controllers/ControllerBase.h"
#include "limxsdk/pointfoot.h"

namespace robot_controllers {

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
      RCLCPP_INFO(rclcpp::get_logger("PointfootController"), "=======start ControlCfg========");
      RCLCPP_INFO(rclcpp::get_logger("PointfootController"), "stiffness: %f", stiffness);
      RCLCPP_INFO(rclcpp::get_logger("PointfootController"), "damping: %f", damping);
      RCLCPP_INFO(rclcpp::get_logger("PointfootController"), "action_scale_pos: %f", action_scale_pos);
      RCLCPP_INFO(rclcpp::get_logger("PointfootController"), "action_scale_vel: %f", action_scale_vel);
      RCLCPP_INFO(rclcpp::get_logger("PointfootController"), "decimation: %d", decimation);
      RCLCPP_INFO(rclcpp::get_logger("PointfootController"), "user_torque_limit: %f", user_torque_limit);
      RCLCPP_INFO(rclcpp::get_logger("PointfootController"), "=======end ControlCfg========\n");
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
        RCLCPP_INFO(rclcpp::get_logger("PointfootController"), "=======start ObsScales========");
        RCLCPP_INFO(rclcpp::get_logger("PointfootController"), "linVel: %f", linVel);
        RCLCPP_INFO(rclcpp::get_logger("PointfootController"), "angVel: %f", angVel);
        RCLCPP_INFO(rclcpp::get_logger("PointfootController"), "dofPos: %f", dofPos);
        RCLCPP_INFO(rclcpp::get_logger("PointfootController"), "dofVel: %f", dofVel);
        RCLCPP_INFO(rclcpp::get_logger("PointfootController"), "=======end ObsScales========\n");
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
      RCLCPP_INFO(rclcpp::get_logger("PointfootController"), "======= Start User Cmd Scales========");
      RCLCPP_INFO(rclcpp::get_logger("PointfootController"), "lin_vel_x: %f", linVel_x);
      RCLCPP_INFO(rclcpp::get_logger("PointfootController"), "lin_vel_y: %f", linVel_y);
      RCLCPP_INFO(rclcpp::get_logger("PointfootController"), "ang_vel_yaw: %f", angVel_yaw);
      RCLCPP_INFO(rclcpp::get_logger("PointfootController"), "=======End User Cmd Scales========\n");
    }
  };


  RlCfg rlCfg;                   // RL configuration settings
  UserCmdCfg userCmdCfg;         // User command configuration settings
  std::map<std::string, double> initState;  // Initial state settings
  ControlCfg controlCfg;         // Control configuration settings

  // Print robot configuration settings
  void print() {
    rlCfg.obsScales.print();
    controlCfg.print();
    userCmdCfg.print();
    RCLCPP_INFO(rclcpp::get_logger("PointfootController"), "clipActions: %f", rlCfg.clipActions);
    RCLCPP_INFO(rclcpp::get_logger("PointfootController"), "clipObs: %f", rlCfg.clipObs);
  }
};

/**
 * @brief Class representing the PointfootController.
 */
class CONTROLLER_INTERFACE_PUBLIC PointfootController : public robot_controllers::ControllerBase {
public:
  PointfootController();

  bool onInit() override;
  void onUpdate() override;
  void onStart() override;
  void onStop() override;

  // Enumeration for controller modes
  enum class Mode : uint8_t {
    STAND,  // Stand mode
    WALK,   // Walk mode
  };

private:
  // Load RL configuration settings
  bool loadRLCfg();

  // Load the model for the controller
  bool loadModel();

  // Compute actions for the controller
  void computeActions();

  // Compute observations for the controller
  void computeObservation();

  // Handle walk mode
  void handleWalkMode();

  // Handle stand mode
  void handleStandMode();

  // Callback function for command velocity
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // Get the robot configuration
  RobotCfg &getRobotCfg() { return robotCfg_; }

private:
  double loopFrequency_; // Control Frequency

  int64_t loopCount_;       // Loop count
  vector3_t commands_;      // Command vector
  vector3_t scaled_commands_; // Scaled command vector

  RobotCfg robotCfg_; // Robot configuration

  Mode mode_; // Controller mode

  // File path for policy model
  std::string policyFilePath_;

  std::shared_ptr<Ort::Env> onnxEnvPrt_; // Shared pointer to ONNX environment

  // ONNX session pointers
  std::unique_ptr<Ort::Session> policySessionPtr_;

  // Names and shapes of inputs and outputs for ONNX sessions
  std::vector<std::vector<int64_t>> policyInputShapes_;
  std::vector<std::vector<int64_t>> policyOutputShapes_;
  std::vector<const char *> policyInputNames_;
  std::vector<const char *> policyOutputNames_;

  vector3_t baseLinVel_; // Base linear velocity
  vector3_t basePosition_; // Base position
  vector_t lastActions_; // Last actions

  int actionsSize_; // Size of actions
  int observationSize_; // Size of observations
  double imuOrientationOffset_[3]; // IMU orientation offset

  std::vector<tensor_element_t> actions_; // Actions
  std::vector<tensor_element_t> observations_; // Observations

  vector_t defaultJointAngles_; // Default joint angles
  vector_t initJointAngles_;    // Initial joint angles in standard standing pose

  double standPercent_;       // Standing percent
  double standDuration_;      // Standing duration

  // Declaration of the subscription to the geometry_msgs::msg::Twist topic.
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  std::shared_ptr<rclcpp::Node> cmd_vel_node_;

  double wheelJointDamping_, wheelJointTorqueLimit_;
  std::vector<int> jointPosIdxs_;
};

} // namespace robot_controllers

#endif //_LIMX_POINTFOOT_CONTROLLER_H_
