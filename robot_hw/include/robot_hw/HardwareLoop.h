// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#ifndef _LIMX_HARDWARE_LOOP_H_
#define _LIMX_HARDWARE_LOOP_H_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include "urdf/model.h"
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "controller_manager/controller_manager.hpp"
#include "robot_hw/PointfootHardware.h"
#include "limxsdk/pointfoot.h"

namespace robot_hw {

/**
 * @brief Class responsible for managing the hardware loop of the robot.
 */
class HardwareLoop {
public:
  /**
   * @brief Constructor for HardwareLoop.
   * 
   * @param hardware Unique pointer to the robot hardware base.
   * @param controller_name Name of the controller to load.
   */
  HardwareLoop(std::unique_ptr<robot_hw::HardwareBase>& hardware, const std::string& controller_name);

  /**
   * @brief Destructor for HardwareLoop.
   */
  ~HardwareLoop();

  /**
   * @brief Starts the hardware loop and controller manager.
   * 
   * @return void.
   */
  void start();

  /**
   * @brief Starts the specified controller.
   * 
   * @param controller_name Name of the controller to start.
   * @return True if the controller was successfully started or is already active.
   */
  bool startController(const std::string& controller_name);

  /**
   * @brief Stops the specified controller if it is active.
   * 
   * @param controller_name Name of the controller to stop.
   * @return True if the controller was successfully stopped.
   */
  bool stopController(const std::string& controller_name);

  /**
   * @brief Enables or disables the ability to publish robot commands.
   * 
   * @param enable True to enable publishing of robot commands, False to disable.
   */
  void setRobotCmdEnable(bool enable);

  /**
   * @brief Enables or disables the ability to publish robot commands.
   * 
   * @param enable True to enable publishing of robot commands, False to disable.
   */
  std::shared_ptr<controller_manager::ControllerManager> getNode() const { return controller_manager_; }

private:
  /**
   * @brief The main update loop for the hardware.
   */
  void update();

  std::vector<std::string> jointNames_; ///< Names of the robot joints.
  robot_hw::HardwareBase* hardware_; ///< Pointer to the robot hardware.
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_; ///< Executor for managing ROS2 callbacks.
  double cycleThreshold_, loopHz_; ///< Loop cycle threshold and frequency.
  std::thread loopThread_; ///< Thread for running the hardware loop.
  std::atomic_bool loopRunning_; ///< Atomic boolean to control the loop execution.
  rclcpp::Duration elapsedTime_; ///< Elapsed time between loop iterations.
  std::chrono::high_resolution_clock::time_point lastTime_; ///< Time point of the last loop iteration.
  std::shared_ptr<controller_manager::ControllerManager> controller_manager_; ///< Controller manager for handling controllers.
  std::map<std::string, std::string> robot_controllers_; ///< Map of robot controllers.
};

} // namespace robot_hw

#endif // _LIMX_HARDWARE_LOOP_H_
