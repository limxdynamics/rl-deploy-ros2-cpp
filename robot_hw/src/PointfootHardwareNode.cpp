/**
 * @file PointfootHWNode.cpp
 *
 * © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_hw/PointfootHardware.h"
#include "robot_hw/HardwareLoop.h"

// Controller name
static std::string controller_name_ = "";

// Publisher for sending velocity commands to the robot
static rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_ = nullptr;

// Shared pointer to the hardware loop managing the robot's control loop
static std::shared_ptr<robot_hw::HardwareLoop> hw_loop_ = nullptr;

// Pointer to the PointFoot robot instance
static limxsdk::PointFoot* robot_ = nullptr;

// Variable to keep track of the calibration state of the robot
static int calibration_state_ = -1;

/**
 * @brief Declare and check a parameter
 * 
 * Template function to declare a parameter if it does not exist, and then retrieve its value.
 * 
 * @tparam T Type of the parameter
 * @param param_name Name of the parameter
 * @param param_value Reference to the variable to store the parameter value
 * @return true if the parameter was successfully retrieved, false otherwise
 */
template<typename T>
bool declareAndCheckParameter(const std::string & param_name, T & param_value) {
  if (!hw_loop_->getNode()->has_parameter(param_name)){
    hw_loop_->getNode()->declare_parameter<T>(param_name, param_value);
  }
  return hw_loop_->getNode()->get_parameter(param_name, param_value);
}

/**
 * @brief Callback function for handling diagnostic messages
 * 
 * This function processes diagnostic messages to monitor the calibration state and handle errors
 * related to EtherCAT and IMU.
 * 
 * @param msg Shared pointer to the received diagnostic message
 */
static void subscribeDiagnosticValueCallback(const limxsdk::DiagnosticValueConstPtr& msg) {
  // Check if the diagnostic message pertains to calibration
  if (msg->name == "calibration") {
    RCLCPP_WARN(rclcpp::get_logger("PointfootHardwareNode"), "Calibration state: %d, msg: %s", msg->code, msg->message.c_str());
    calibration_state_ = msg->code;
  }
  
  // Check if the diagnostic message pertains to EtherCAT
  if (msg->name == "ethercat" && msg->level == limxsdk::DiagnosticValue::ERROR) {
    RCLCPP_FATAL(rclcpp::get_logger("PointfootHardwareNode"), "Ethercat code: %d, msg: %s", msg->code, msg->message.c_str());
    hw_loop_->stopController(controller_name_);
    abort();
  }

  // Check if the diagnostic message pertains to IMU
  if (msg->name == "imu" && msg->level == limxsdk::DiagnosticValue::ERROR) {
    RCLCPP_FATAL(rclcpp::get_logger("PointfootHardwareNode"), "IMU code: %d, msg: %s", msg->code, msg->message.c_str());
    hw_loop_->stopController(controller_name_);
    abort();
  }
}

/**
 * @brief Callback function for handling joystick input
 * 
 * This function processes joystick input to start and stop the robot controller and publish
 * velocity commands based on the joystick axes.
 * 
 * @param msg Shared pointer to the received joystick message
 */
static void subscribeSensorJoyCallback(const limxsdk::SensorJoyConstPtr& msg) {
  // Logic for starting controller
  int BTN_L1, BTN_Y, BTN_X;
  if (declareAndCheckParameter("joystick_buttons.L1", BTN_L1) && declareAndCheckParameter("joystick_buttons.Y", BTN_Y)) {
    if (msg->buttons[BTN_L1] == 1 && msg->buttons[BTN_Y] == 1) {
      hw_loop_->startController(controller_name_);
    }
  }

  // Logic for stopping controller
  if (declareAndCheckParameter("joystick_buttons.L1", BTN_L1) && declareAndCheckParameter("joystick_buttons.X", BTN_X)) {
    if (msg->buttons[BTN_L1] == 1 && msg->buttons[BTN_X] == 1) {
      RCLCPP_FATAL(rclcpp::get_logger("PointfootHardwareNode"), "L1 + X stopping controller!");
      hw_loop_->stopController(controller_name_);
      abort();
    }
  }

  // Publishing cmd_vel based on joystick input
  int left_horizon, left_vertical, right_horizon, right_vertical;
  if (declareAndCheckParameter("joystick_axes.left_horizon", left_horizon) 
    && declareAndCheckParameter("joystick_axes.left_vertical", left_vertical)
    && declareAndCheckParameter("joystick_axes.right_horizon", right_horizon)
    && declareAndCheckParameter("joystick_axes.right_vertical", right_vertical)) {
    static rclcpp::Time lastpub;
    rclcpp::Time now = rclcpp::Clock().now();
    if (fabs((now - lastpub).seconds()) >= (1.0 / 30)) {
      geometry_msgs::msg::Twist twist;
      twist.linear.x = msg->axes[left_vertical] * 0.5;
      twist.linear.y = msg->axes[left_horizon] * 0.5;
      twist.angular.z = msg->axes[right_horizon] * 0.5;
      cmd_vel_pub_->publish(twist);
      lastpub = now;
    }
  }
}

/**
 * @brief Main function
 * 
 * Initializes the ROS 2 node, sets up the hardware loop and subscribers, and starts the node
 * to process callbacks.
 * 
 * @param argc Number of command-line arguments
 * @param argv Array of command-line arguments
 * @return int Exit status
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  // Default robot IP address, can be overridden by command-line argument
  std::string robot_ip = "127.0.0.1";
  if (argc > 1) {
    robot_ip = argv[1];
  }

  // Retrieve the robot type from the environment variable "ROBOT_TYPE"
  const char* value = ::getenv("ROBOT_TYPE");
  if (value && strlen(value) > 0) {
    // Determine the specific robot configuration based on the robot type
    std::string robot_type = std::string(value);
    if(robot_type.find("PF") != std::string::npos) {
      controller_name_ = "PointfootController";
    } else if (robot_type.find("WF") != std::string::npos) {
      controller_name_ = "WheelfootController";
    } else if (robot_type.find("SF") != std::string::npos) {
      controller_name_ = "SolefootController";
    } else {
      RCLCPP_FATAL(rclcpp::get_logger("PointfootHardwareNode"), "Error: ROBOT_TYPE.");
      abort();
    }
    RCLCPP_WARN(rclcpp::get_logger("PointfootHardwareNode"), "Controller Name: %s", controller_name_.c_str());
  } else {
    RCLCPP_FATAL(rclcpp::get_logger("PointfootHardwareNode"), "Error: Please set the ROBOT_TYPE using 'export ROBOT_TYPE=<robot_type>'.");
    abort();
  }


  // Get an instance of the PointFoot robot
  robot_ = limxsdk::PointFoot::getInstance();

  // Initialize the PointFoot instance with the robot IP
  if (!robot_->init(robot_ip)) {
    RCLCPP_ERROR(rclcpp::get_logger("PointfootHardwareNode"), "Failed to connect to the robot: %s", robot_ip.c_str());
    abort();
  } else {
    RCLCPP_WARN(rclcpp::get_logger("PointfootHardwareNode"), "Connected to the robot: %s", robot_ip.c_str());
  }
  
  // Create a unique pointer to the PointfootHardware object
  std::unique_ptr<robot_hw::HardwareBase> hw = std::make_unique<robot_hw::PointfootHardware>();

  // Create a HardwareLoop object with the node and hardware
  hw_loop_ = std::make_shared<robot_hw::HardwareLoop>(hw, controller_name_);

  // Start the hardware loop
  hw_loop_->start();

  // Retrieve the value of the "use_gazebo" parameter
  bool use_gazebo = false;
  if (!declareAndCheckParameter("use_gazebo", use_gazebo)) {
    use_gazebo = false;
  }

  // Create a publisher for velocity commands
  cmd_vel_pub_ = hw_loop_->getNode()->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Subscribe to joystick and diagnostic messages
  robot_->subscribeSensorJoy(&subscribeSensorJoyCallback);
  robot_->subscribeDiagnosticValue(&subscribeDiagnosticValueCallback);

  // Start the controller if "use_gazebo" parameter is true
  if (use_gazebo) {
    std::thread controller_thread([]() {
      std::this_thread::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(6.0 * 1e9)));
      hw_loop_->startController(controller_name_);
    });
    controller_thread.detach();
  }

  // Spin the node to process callbacks and keep it alive
  rclcpp::spin(hw_loop_->getNode());

  // Shutdown the ROS 2 system
  rclcpp::shutdown();
  return 0;
}
