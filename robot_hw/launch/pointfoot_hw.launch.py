from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    robot_controllers_model_file = PathJoinSubstitution([FindPackageShare("robot_controllers"), "config/pointfoot/policy/policy.onnx"])
    robot_controllers_pointfoot_params_file = PathJoinSubstitution([FindPackageShare("robot_controllers"), "config/pointfoot/params.yaml"])
    robot_controllers_file = PathJoinSubstitution([FindPackageShare("robot_controllers"), "config/robot_controllers.yaml"])
    robot_hw_joystick_file = PathJoinSubstitution([FindPackageShare("robot_hw"), "config/joystick.yaml"])
    robot_hw_file = PathJoinSubstitution([FindPackageShare("robot_hw"), "config/robot_hw.yaml"])

    return LaunchDescription([
        Node(
            package='robot_hw',
            executable='pointfoot_node',
            name='robot_hw_node',
            output='screen',
            arguments=["10.192.1.2"],
            parameters=[
                {
                    "robot_controllers_model_file": robot_controllers_model_file,
                },
                robot_controllers_pointfoot_params_file, 
                robot_controllers_file,
                robot_hw_joystick_file,
                robot_hw_file,
            ],
        ),
    ])