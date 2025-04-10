// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#include "robot_hw/HardwareLoop.h"

namespace robot_hw {

/**
 * @class HardwareLoop
 * @brief A class to manage the hardware loop for the robot, including initialization, 
 * controller management, and periodic updates.
 */

/**
 * @brief Constructor for the HardwareLoop class.
 * 
 * Initializes the hardware loop, sets up the controller manager, and loads necessary parameters.
 * 
 * @param hardware A unique pointer to the base hardware class.
 */
HardwareLoop::HardwareLoop(std::unique_ptr<robot_hw::HardwareBase> &hardware, const std::string& controller_name)
    : hardware_(hardware.get()), elapsedTime_(0, 0) {
  try {
    // Create ResourceManager
    auto resource_manager = std::make_unique<hardware_interface::ResourceManager>();
    hardware_interface::ResourceManager* resourceManagerPtr = resource_manager.get();

    // Create MultiThreadedExecutor
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Create ControllerManager
    controller_manager_ = std::make_shared<controller_manager::ControllerManager>(
        std::move(resource_manager), executor_, "controller_manager");

    // Declare parameters ControllerCfg.joint_names
    if (!controller_manager_->has_parameter("ControllerCfg.joint_names")) {
      controller_manager_->declare_parameter<std::vector<std::string>>("ControllerCfg.joint_names", jointNames_);
    }

    // Attempt to retrieve the 'ControllerCfg.joint_names' parameter
    if (!controller_manager_->get_parameter("ControllerCfg.joint_names", jointNames_)) {
      RCLCPP_FATAL(rclcpp::get_logger("HardwareLoop"), "Failed to get 'ControllerCfg.joint_names' parameter");
      return;
    }

    // Configure hardware information
    std::string jointNamesPrint;
    hardware_interface::HardwareInfo hardwareinfo;
    for (const auto &joint_name : jointNames_) {
      hardware_interface::ComponentInfo joint;
      joint.name = joint_name;
      hardwareinfo.joints.push_back(joint);
      jointNamesPrint += joint_name + ", ";
    }

    RCLCPP_WARN(rclcpp::get_logger("HardwareLoop"), "jointNames: [%s]", jointNamesPrint.c_str());
    hardware->configure(hardwareinfo);
    hardware->start();

    resourceManagerPtr->import_component(std::move(hardware), hardwareinfo);
    
    auto components_status_map = resourceManagerPtr->get_components_status();
    for (auto & component: components_status_map) {
      rclcpp_lifecycle::State state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, 
                                      hardware_interface::lifecycle_state_names::ACTIVE);
      resourceManagerPtr->set_component_state(component.first, state);
    }

    // Declare parameters robot_controllers
    std::vector<std::string> controllers;
    if (!controller_manager_->has_parameter("robot_controllers")) {
      controller_manager_->declare_parameter<std::vector<std::string>>("robot_controllers", controllers);
    }

    // Attempt to retrieve the 'robot_controllers' parameter
    if (!controller_manager_->get_parameter("robot_controllers", controllers)) {
      RCLCPP_FATAL(rclcpp::get_logger("HardwareLoop"), "Failed to get 'robot_controllers' parameter");
      return;
    }

    // Load controllers
    for (const auto &controller : controllers) {
      std::size_t pos = controller.rfind("/");
      if (pos != std::string::npos) {
        if (controller.substr(pos + 1) != controller_name) {
          continue;
        }
        if (controller_manager_->load_controller(controller_name, controller) == nullptr) {
          RCLCPP_FATAL(rclcpp::get_logger("HardwareLoop"), "Failed to load controller name: %s, type: %s", controller_name.c_str(), controller.c_str());
          rclcpp::shutdown();
          return;
        } else {
          RCLCPP_INFO(rclcpp::get_logger("HardwareLoop"), "Loaded controller name: %s, type: %s", controller_name.c_str(), controller.c_str());
          controller_manager_->configure_controller(controller_name);
          robot_controllers_[controller_name] = controller;
        }
      }
    }
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("HardwareLoop"), "Failed to load hardware interface: %s", e.what());
    throw e;
  }
}

/**
 * @brief Starts a specified controller.
 * 
 * @param controller_name The name of the controller to start.
 * @return true if the controller is successfully started, false otherwise.
 */
bool HardwareLoop::startController(const std::string &controller_name) {
  // Check if the controller is already active
  auto loaded_controllers = controller_manager_->get_loaded_controllers();
  for (auto &controller : loaded_controllers) {
    std::string current_state_label = controller.c->get_state().label();
    if (controller.info.name == controller_name && current_state_label == "active") {
      RCLCPP_INFO(rclcpp::get_logger("HardwareLoop"),
                  "Controller '%s' is already active. Skipping start.", controller_name.c_str());
      return false;
    }
  }

  // Switch the controllers
  std::vector<std::string> start_controllers = {controller_name};
  std::vector<std::string> stop_controllers = {};
  RCLCPP_INFO(rclcpp::get_logger("HardwareLoop"), "Switching controllers...");
  controller_manager_->switch_controller(
      start_controllers,
      stop_controllers,
      controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT,
      false,
      rclcpp::Duration(3, 0));
  RCLCPP_INFO(rclcpp::get_logger("HardwareLoop"), "Controller '%s' started.", controller_name.c_str());
  return true;
}

/**
 * @brief Stops a specified controller.
 * 
 * @param controller_name The name of the controller to stop.
 * @return true if the controller is successfully stopped, false otherwise.
 */
bool HardwareLoop::stopController(const std::string &controller_name) {
  // Check if the controller is already active
  auto loaded_controllers = controller_manager_->get_loaded_controllers();
  for (auto &controller : loaded_controllers) {
    std::string current_state_label = controller.c->get_state().label();
    if (controller.info.name == controller_name && current_state_label == "active") {
      std::vector<std::string> start_controllers = {};
      std::vector<std::string> stop_controllers = {controller_name};
      controller_manager_->switch_controller(
          start_controllers,
          stop_controllers,
          controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT,
          false,
          rclcpp::Duration(3, 0));
      RCLCPP_INFO(rclcpp::get_logger("HardwareLoop"), "Controller '%s' stopped.", controller_name.c_str());
    }
  }
  return true;
}

/**
 * @brief Enables or disables the ability to publish robot commands.
 * 
 * @param enable A boolean indicating whether to enable or disable robot commands.
 */
void HardwareLoop::setRobotCmdEnable(bool enable) {
  if (hardware_) {
    hardware_->setRobotCmdEnable(enable);
  }
}

/**
 * @brief Starts the hardware loop.
 * 
 * Loads parameters, sets up the update loop, and starts the loop thread.
 */
void HardwareLoop::start() {
  // Load parameters
  if (!controller_manager_->has_parameter("robot_hw.loop_frequency")) {
    controller_manager_->declare_parameter<double>("robot_hw.loop_frequency", loopHz_);
  }
  if (!controller_manager_->has_parameter("robot_hw.cycle_time_error_threshold")) {
    controller_manager_->declare_parameter<double>("robot_hw.cycle_time_error_threshold", cycleThreshold_);
  }
  if (!controller_manager_->get_parameter("robot_hw.loop_frequency", loopHz_)) {
    loopHz_ = 500.0;
  }
  if (!controller_manager_->get_parameter("robot_hw.cycle_time_error_threshold", cycleThreshold_)) {
    cycleThreshold_ = 0.002;
  }

  // Print loaded parameters
  RCLCPP_INFO(rclcpp::get_logger("HardwareLoop"), "Loaded parameters - loop_frequency: %f, cycle_time_error_threshold: %f", loopHz_, cycleThreshold_);

  // Get current time for use with first update
  lastTime_ = std::chrono::high_resolution_clock::now();

  // Setup loop thread
  loopRunning_ = true;

  loopThread_ = std::thread([this]() {
    while (loopRunning_) {
      update();
    }
  });
}

/**
 * @brief Destructor for the HardwareLoop class.
 * 
 * Stops the loop thread if it's running.
 */
HardwareLoop::~HardwareLoop() {
  loopRunning_ = false;
  if (loopThread_.joinable()) {
    loopThread_.join();
  }
}

/**
 * @brief Updates the hardware and controllers.
 * 
 * This function is called periodically to read from hardware, update the controllers,
 * write commands to the hardware, and manage the timing of the loop.
 */
void HardwareLoop::update() {
  const auto currentTime = std::chrono::high_resolution_clock::now();

  // Compute desired duration rounded to clock decimation
  const std::chrono::duration<double> desiredDuration(1.0 / loopHz_);

  // Get change in time
  std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - lastTime_);
  elapsedTime_ = rclcpp::Duration::from_seconds(time_span.count());
  lastTime_ = currentTime;

  // Check cycle time for excess delay
  const double cycle_time_error = (elapsedTime_ - rclcpp::Duration::from_seconds(desiredDuration.count())).seconds();
  if (cycle_time_error > cycleThreshold_) {
    RCLCPP_WARN(rclcpp::get_logger("HardwareLoop"), "Cycle time exceeded error threshold by: %fs, cycle time: %fs, threshold: %fs",
                cycle_time_error - cycleThreshold_, elapsedTime_.seconds(), cycleThreshold_);
  }

  rclcpp::Time time = rclcpp::Time(0, 0, controller_manager_->get_node_clock_interface()->get_clock()->get_clock_type());
  rclcpp::Duration period = rclcpp::Duration::from_seconds(0.01);

  // Input: Get the hardware's state
  controller_manager_->read(time, period);

  // Control: Let the controller compute the new command (via the controller manager)
  controller_manager_->update(time, period);

  // Output: Send the new command to hardware
  controller_manager_->write(time, period);

  // Sleep until the next update
  const auto sleepTill = currentTime + std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(desiredDuration);
  std::this_thread::sleep_until(sleepTill);
}

}  // namespace robot_hw
