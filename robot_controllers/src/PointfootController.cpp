// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#include "robot_controllers/PointfootController.h"

#include "controller_interface/helpers.hpp"

namespace robot_controllers {
PointfootController::PointfootController()
{
}

bool PointfootController::onInit() {
  if (!loadRLCfg()) {
    RCLCPP_ERROR(rclcpp::get_logger("PointfootController"), "Error in loadRLCfg");
    return false;
  }
  if (!loadModel()) {
    RCLCPP_ERROR(rclcpp::get_logger("PointfootController"), "Error in loadModel");
    return false;
  }

  auto &initState = robotCfg_.initState;

  initJointAngles_.resize(initState.size());

  for (int i = 0; i < jointNames_.size(); i++) {
    initJointAngles_(i) = initState[jointNames_[i]];
  }

  defaultJointAngles_.resize(jointNames_.size());

  // Create a shared pointer to a ROS 2 node named "cmd_vel_node".
  cmd_vel_node_ = std::make_shared<rclcpp::Node>("cmd_vel_node");

  // Create a subscription to the geometry_msgs::msg::Twist topic ("/cmd_vel").
  cmd_vel_sub_ = cmd_vel_node_->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&PointfootController::cmdVelCallback, this, std::placeholders::_1));

  // Spin the cmd_vel_node_ in a separate thread to handle message callbacks.
  std::thread cmd_vel_node_spin_thid([this]() {
      rclcpp::spin(cmd_vel_node_);
  });
  cmd_vel_node_spin_thid.detach();

  return true;
}

void PointfootController::onStart() {
  standPercent_ += 1 / (standDuration_ * loopFrequency_);

  loopCount_ = 0;

  mode_ = Mode::STAND;
}

void PointfootController::onUpdate() {
  switch (mode_)
  {
  case Mode::STAND:
    handleStandMode();
    break;
  case Mode::WALK:
    handleWalkMode();
    break;
  }

  loopCount_++;
}

void PointfootController::onStop() {
}

// Handle walking mode
void PointfootController::handleWalkMode() {
  TicToc dida;
  // Compute observation & actions
  if (robotCfg_.controlCfg.decimation == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("PointfootController"), "Error robotCfg_.controlCfg.decimation");
    return;
  }

  if (loopCount_ % robotCfg_.controlCfg.decimation == 0) {
    computeObservation();
    computeActions();

    // Limit action range
    double actionMin = -robotCfg_.rlCfg.clipActions;
    double actionMax = robotCfg_.rlCfg.clipActions;
    std::transform(actions_.begin(), actions_.end(), actions_.begin(),
                    [actionMin, actionMax](double x)
                    { return std::max(actionMin, std::min(actionMax, x)); });
  }

  // Set action
  vector_t jointPos(jointNames_.size()), jointVel(jointNames_.size());
  for (size_t i = 0; i < jointNames_.size(); i++) {
    jointPos(i) = this->getJointStateValue(jointNames_[i], "position");
    jointVel(i) = this->getJointStateValue(jointNames_[i], "velocity");
  }

  for (int i = 0; i < jointNames_.size(); i++) {
    double actionMin =
        jointPos(i) - initJointAngles_(i, 0) +
        (robotCfg_.controlCfg.damping * jointVel(i) - robotCfg_.controlCfg.user_torque_limit) / robotCfg_.controlCfg.stiffness;
    double actionMax =
        jointPos(i) - initJointAngles_(i, 0) +
        (robotCfg_.controlCfg.damping * jointVel(i) + robotCfg_.controlCfg.user_torque_limit) / robotCfg_.controlCfg.stiffness;
    actions_[i] = std::max(actionMin / robotCfg_.controlCfg.action_scale_pos,
                            std::min(actionMax / robotCfg_.controlCfg.action_scale_pos, (double)actions_[i]));
    double pos_des = actions_[i] * robotCfg_.controlCfg.action_scale_pos + initJointAngles_(i, 0);

    this->setJointCommandValue(jointNames_[i], "position", pos_des);
    this->setJointCommandValue(jointNames_[i], "velocity", 0);
    this->setJointCommandValue(jointNames_[i], "kp", robotCfg_.controlCfg.stiffness);
    this->setJointCommandValue(jointNames_[i], "kd", robotCfg_.controlCfg.damping);
    this->setJointCommandValue(jointNames_[i], "effort", 0);
    this->setJointCommandValue(jointNames_[i], "mode", 2);

    lastActions_(i, 0) = actions_[i];
  }
}

// Handle standing mode
void PointfootController::handleStandMode() {
  if (standPercent_ < 1) {
    for (int i = 0; i < jointNames_.size(); i++)
    {
      double pos_des = defaultJointAngles_[i] * (1 - standPercent_) + initJointAngles_[i] * standPercent_;
      this->setJointCommandValue(jointNames_[i], "position", pos_des);
      this->setJointCommandValue(jointNames_[i], "velocity", 0);
      this->setJointCommandValue(jointNames_[i], "kp", robotCfg_.controlCfg.stiffness);
      this->setJointCommandValue(jointNames_[i], "kd", robotCfg_.controlCfg.damping);
      this->setJointCommandValue(jointNames_[i], "effort", 0);
      this->setJointCommandValue(jointNames_[i], "mode", 2);
    }
    standPercent_ += 1 / (standDuration_ * loopFrequency_);
  } else {
    mode_ = Mode::WALK;
  }
}

bool PointfootController::loadModel() {
  // Load ONNX models for policy, encoder, and gait generator.

  std::string policyModelPath;

  if (declareAndCheckParameter<std::string>("robot_controllers_model_file", policyModelPath)) {
    RCLCPP_ERROR(rclcpp::get_logger("PointfootController"), "Failed to retrieve policy path from the parameter server!");
    return false;
  }

  // Create ONNX environment
  onnxEnvPrt_.reset(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "PointFootOnnxController"));

  // Create session options
  Ort::SessionOptions sessionOptions;
  sessionOptions.SetIntraOpNumThreads(1);
  sessionOptions.SetInterOpNumThreads(1);

  Ort::AllocatorWithDefaultOptions allocator;

  // Policy session
  RCLCPP_INFO(rclcpp::get_logger("PointfootController"), "Loading policy from: %s", policyModelPath.c_str());
  policySessionPtr_ = std::make_unique<Ort::Session>(*onnxEnvPrt_, policyModelPath.c_str(), sessionOptions);
  policyInputNames_.clear();
  policyOutputNames_.clear();
  policyInputShapes_.clear();
  policyOutputShapes_.clear();
  for (int i = 0; i < policySessionPtr_->GetInputCount(); i++) {
    policyInputNames_.push_back(policySessionPtr_->GetInputName(i, allocator));
    policyInputShapes_.push_back(policySessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
    RCLCPP_INFO(rclcpp::get_logger("PointfootController"), "GetInputName: %s", policySessionPtr_->GetInputName(i, allocator));
    std::vector<int64_t> shape = policySessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
    std::string shapeString;
    for (size_t j = 0; j < shape.size(); ++j) {
      shapeString += std::to_string(shape[j]);
      if (j != shape.size() - 1) {
        shapeString += ", ";
      }
    }
    RCLCPP_INFO(rclcpp::get_logger("PointfootController"), "Shape: [%s]", shapeString.c_str());
  }
  for (int i = 0; i < policySessionPtr_->GetOutputCount(); i++) {
    policyOutputNames_.push_back(policySessionPtr_->GetOutputName(i, allocator));
    RCLCPP_INFO(rclcpp::get_logger("PointfootController"), "GetOutputName: %s", policySessionPtr_->GetOutputName(i, allocator));
    policyOutputShapes_.push_back(policySessionPtr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
    std::vector<int64_t> shape = policySessionPtr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
    std::string shapeString;
    for (size_t j = 0; j < shape.size(); ++j) {
      shapeString += std::to_string(shape[j]);
      if (j != shape.size() - 1) {
        shapeString += ", ";
      }
    }
    RCLCPP_INFO(rclcpp::get_logger("PointfootController"), "Shape: [%s]", shapeString.c_str());
  }

  RCLCPP_INFO(rclcpp::get_logger("PointfootController"), "Successfully loaded ONNX models!");
  return true;
}

// Loads the reinforcement learning configuration.
bool PointfootController::loadRLCfg() {
  auto &initState = robotCfg_.initState;
  RobotCfg::ControlCfg &controlCfg = robotCfg_.controlCfg;
  RobotCfg::RlCfg::ObsScales &obsScales = robotCfg_.rlCfg.obsScales;

  try {
    // Declare and check parameters
    int error = 0;
    error += declareAndCheckParameter<double>("ControllerCfg.init_state.default_joint_angle.abad_L_Joint", initState["abad_L_Joint"]);
    error += declareAndCheckParameter<double>("ControllerCfg.init_state.default_joint_angle.hip_L_Joint", initState["hip_L_Joint"]);
    error += declareAndCheckParameter<double>("ControllerCfg.init_state.default_joint_angle.knee_L_Joint", initState["knee_L_Joint"]);
    error += declareAndCheckParameter<double>("ControllerCfg.init_state.default_joint_angle.abad_R_Joint", initState["abad_R_Joint"]);
    error += declareAndCheckParameter<double>("ControllerCfg.init_state.default_joint_angle.hip_R_Joint", initState["hip_R_Joint"]);
    error += declareAndCheckParameter<double>("ControllerCfg.init_state.default_joint_angle.knee_R_Joint", initState["knee_R_Joint"]);
    error += declareAndCheckParameter<double>("ControllerCfg.stand_mode.stand_duration", standDuration_);
    error += declareAndCheckParameter<double>("robot_hw.loop_frequency", loopFrequency_);
    error += declareAndCheckParameter<double>("ControllerCfg.control.stiffness", controlCfg.stiffness);
    error += declareAndCheckParameter<double>("ControllerCfg.control.damping", controlCfg.damping);
    error += declareAndCheckParameter<double>("ControllerCfg.control.action_scale_pos", controlCfg.action_scale_pos);
    error += declareAndCheckParameter<int>("ControllerCfg.control.decimation", controlCfg.decimation);
    error += declareAndCheckParameter<double>("ControllerCfg.control.user_torque_limit", controlCfg.user_torque_limit);
    error += declareAndCheckParameter<double>("ControllerCfg.normalization.clip_scales.clip_observations", robotCfg_.rlCfg.clipObs);
    error += declareAndCheckParameter<double>("ControllerCfg.normalization.clip_scales.clip_actions", robotCfg_.rlCfg.clipActions);
    error += declareAndCheckParameter<double>("ControllerCfg.normalization.obs_scales.lin_vel", obsScales.linVel);
    error += declareAndCheckParameter<double>("ControllerCfg.normalization.obs_scales.ang_vel", obsScales.angVel);
    error += declareAndCheckParameter<double>("ControllerCfg.normalization.obs_scales.dof_pos", obsScales.dofPos);
    error += declareAndCheckParameter<double>("ControllerCfg.normalization.obs_scales.dof_vel", obsScales.dofVel);
    error += declareAndCheckParameter<int>("ControllerCfg.size.actions_size", actionsSize_);
    error += declareAndCheckParameter<int>("ControllerCfg.size.observations_size", observationSize_);
    error += declareAndCheckParameter<double>("ControllerCfg.imu_orientation_offset.yaw", imuOrientationOffset_[0]);
    error += declareAndCheckParameter<double>("ControllerCfg.imu_orientation_offset.pitch", imuOrientationOffset_[1]);
    error += declareAndCheckParameter<double>("ControllerCfg.imu_orientation_offset.roll", imuOrientationOffset_[2]);

    // Log the result of parameter fetching
    if (error != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("PointfootController"), "Some parameters could not be retrieved. Number of errors: %d", error);
    } else {
      RCLCPP_INFO(rclcpp::get_logger("PointfootController"), "All parameters retrieved successfully.");
    }

    robotCfg_.print();

    // Resize vectors.
    actions_.resize(actionsSize_);
    observations_.resize(observationSize_);
    lastActions_.resize(actionsSize_);

    // Initialize vectors.
    lastActions_.setZero();
    commands_.setZero();
    scaled_commands_.setZero();
    baseLinVel_.setZero();
    basePosition_.setZero();
  } catch (const std::exception &e) {
    // Error handling.
    RCLCPP_ERROR(rclcpp::get_logger("PointfootController"), "Error in the ControllerCfg: %s", e.what());
    return false;
  }

  return true;
}

// Computes actions using the policy model.
void PointfootController::computeActions() {
  // Create input tensor object
  Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator,
                                                          OrtMemType::OrtMemTypeDefault);
  std::vector<Ort::Value> inputValues;
  std::vector<tensor_element_t> combined_obs;
  for (const auto &item : observations_) {
    combined_obs.push_back(item);
  }
  inputValues.push_back(
      Ort::Value::CreateTensor<tensor_element_t>(memoryInfo, combined_obs.data(), combined_obs.size(),
                                                  policyInputShapes_[0].data(), policyInputShapes_[0].size()));
  // Run inference
  Ort::RunOptions runOptions;
  std::vector<Ort::Value> outputValues = policySessionPtr_->Run(runOptions, policyInputNames_.data(),
                                                                inputValues.data(), 1, policyOutputNames_.data(),
                                                                1);

  for (int i = 0; i < actionsSize_; i++) {
    actions_[i] = *(outputValues[0].GetTensorMutableData<tensor_element_t>() + i);
  }
}

void PointfootController::computeObservation()
{
  // Get IMU orientation
  Eigen::Quaterniond q_wi;
  q_wi.coeffs()(0) = this->getSensorValue("imu", "orientation_x");
  q_wi.coeffs()(1) = this->getSensorValue("imu", "orientation_y");
  q_wi.coeffs()(2) = this->getSensorValue("imu", "orientation_z");
  q_wi.coeffs()(3) = this->getSensorValue("imu", "orientation_w");

  // Convert quaternion to ZYX Euler angles and calculate inverse rotation matrix
  vector3_t zyx = quatToZyx(q_wi);
  matrix_t inverseRot = getRotationMatrixFromZyxEulerAngles(zyx).inverse();

  // Define gravity vector and project it to the body frame
  vector3_t gravityVector(0, 0, -1);
  vector3_t projectedGravity(inverseRot * gravityVector);

  // Get base angular velocity and apply orientation offset
  vector3_t baseAngVel(this->getSensorValue("imu", "angular_velocity_x"),
                        this->getSensorValue("imu", "angular_velocity_y"),
                        this->getSensorValue("imu", "angular_velocity_z"));

  vector3_t _zyx(imuOrientationOffset_[0], imuOrientationOffset_[1], imuOrientationOffset_[2]);
  matrix_t rot = getRotationMatrixFromZyxEulerAngles(_zyx);
  baseAngVel = rot * baseAngVel;
  projectedGravity = rot * projectedGravity;

  // Get initial state of joints
  auto &initState = robotCfg_.initState;
  vector_t jointPos(initState.size());
  vector_t jointVel(initState.size());
  for (size_t i = 0; i < jointNames_.size(); ++i) {
    jointPos(i) = this->getJointStateValue(jointNames_[i], "position");
    jointVel(i) = this->getJointStateValue(jointNames_[i], "velocity");
  }

  vector_t actions(lastActions_);

  // Define command scaler and observation vector
  matrix_t commandScaler = Eigen::DiagonalMatrix<double, 3>(robotCfg_.rlCfg.obsScales.linVel,
                                                            robotCfg_.rlCfg.obsScales.linVel,
                                                            robotCfg_.rlCfg.obsScales.linVel);

  vector_t obs(observationSize_);
  vector3_t scaled_commands = commandScaler * commands_;
  // Populate observation vector
  obs << baseAngVel * robotCfg_.rlCfg.obsScales.angVel,
      projectedGravity,
      (jointPos - initJointAngles_) * robotCfg_.rlCfg.obsScales.dofPos,
      jointVel * robotCfg_.rlCfg.obsScales.dofVel,
      actions,
      scaled_commands;

  // Update observation, scaled commands, and proprioceptive history vector
  for (size_t i = 0; i < obs.size(); i++) {
    observations_[i] = static_cast<tensor_element_t>(obs(i));
  }
  for (size_t i = 0; i < scaled_commands_.size(); i++) {
    scaled_commands_[i] = static_cast<tensor_element_t>(scaled_commands(i));
  }

  // Limit observation range
  double obsMin = -robotCfg_.rlCfg.clipObs;
  double obsMax = robotCfg_.rlCfg.clipObs;
  std::transform(observations_.begin(), observations_.end(), observations_.begin(),
                  [obsMin, obsMax](double x)
                  { return std::max(obsMin, std::min(obsMax, x)); });
}

void PointfootController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  // Update the commands with the linear and angular velocities from the Twist message.

  // Set linear x velocity.
  commands_(0) = msg->linear.x;

  // Set linear y velocity.
  commands_(1) = msg->linear.y;

  // Set angular z velocity.
  commands_(2) = msg->angular.z;
}

} // namespace

#include "pluginlib/class_list_macros.hpp"

// Export the class as a plugin.
PLUGINLIB_EXPORT_CLASS(robot_controllers::PointfootController, controller_interface::ControllerInterface)
