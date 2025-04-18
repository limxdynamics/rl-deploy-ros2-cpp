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
/**
 * @brief Class representing the SolefootController.
 */
class CONTROLLER_INTERFACE_PUBLIC SolefootController : public robot_controllers::ControllerBase {
public:
  SolefootController();

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

  // Compute encoder for the controller
  void computeEncoder();

  // Handle walk mode
  void handleWalkMode();

  // Handle stand mode
  void handleStandMode();

  // Callback function for command velocity
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // compute gait
  vector_t handleGait();

  // compute gait clock
  vector_t handleGaitClock(vector_t &curr_gait);

  // Get the robot configuration
  RobotCfg &getRobotCfg() { return robotCfg_; }

private:
  double loopFrequency_; // Control Frequency

  int64_t loopCount_;       // Loop count

  RobotCfg robotCfg_; // Robot configuration

  Mode mode_; // Controller mode

  // File path for policy model
  std::string policyFilePath_;

  std::shared_ptr<Ort::Env> onnxEnvPrt_; // Shared pointer to ONNX environment

  // ONNX session pointers
  std::unique_ptr<Ort::Session> policySessionPtr_;
  std::unique_ptr<Ort::Session> encoderSessionPtr_;

  // Names and shapes of inputs and outputs for ONNX sessions
  std::vector<std::vector<int64_t>> policyInputShapes_;
  std::vector<std::vector<int64_t>> policyOutputShapes_;
  std::vector<const char *> policyInputNames_;
  std::vector<const char *> policyOutputNames_;

  std::vector<std::vector<int64_t>> encoderInputShapes_;
  std::vector<std::vector<int64_t>> encoderOutputShapes_;
  std::vector<const char *> encoderInputNames_;
  std::vector<const char *> encoderOutputNames_;

  std::vector<tensor_element_t> proprioHistoryVector_;
  Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1> proprioHistoryBuffer_;

  vector3_t baseLinVel_; // Base linear velocity
  vector3_t basePosition_; // Base position
  vector_t lastActions_; // Last actions
  int commandSize_;
  vector_t commands_; // command for solefoot(size=5)
  vector_t scaled_commands_; // scaled command for solefoot(size=5)

  int actionsSize_; // Size of actions
  int observationSize_; // Size of observations
  int obsHistoryLength_; // Size of history observations
  int encoderInputSize_, encoderOutputSize_; // Input and output size of encoder
  double imuOrientationOffset_[3]; // IMU orientation offset

  std::vector<tensor_element_t> actions_; // Actions
  std::vector<tensor_element_t> observations_; // Observations
  std::vector<tensor_element_t> encoderOut_;  // Encoder
  
  double gait_index_{0.0};
  bool isfirstRecObs_{true};

  vector_t defaultJointAngles_; // Default joint angles
  vector_t initJointAngles_;    // Initial joint angles in standard standing pose

  double standPercent_;       // Standing percent
  double standDuration_;      // Standing duration

  // Declaration of the subscription to the geometry_msgs::msg::Twist topic.
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  std::shared_ptr<rclcpp::Node> cmd_vel_node_;

  double wheelJointDamping_, wheelJointTorqueLimit_, ankleJointDamping_, ankleJointTorqueLimit_;
  std::vector<int> jointPosIdxs_;
};

} // namespace robot_controllers

#endif //_LIMX_POINTFOOT_CONTROLLER_H_
