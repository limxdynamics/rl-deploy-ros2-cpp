from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
import sys

def generate_launch_description():
    robot_type = os.getenv("ROBOT_TYPE")

    # Check if the ROBOT_TYPE environment variable is set, otherwise exit with an error
    if not robot_type:
        print("Error: Please set the ROBOT_TYPE using 'export ROBOT_TYPE=<robot_type>'.")
        sys.exit(1)

    pointfoot_gazebo_launch_file = PathJoinSubstitution([FindPackageShare("pointfoot_gazebo"), "launch/empty_world.launch.py"])
    robot_controllers_model_file = PathJoinSubstitution([FindPackageShare("robot_controllers"), "config/pointfoot/"+ robot_type +"/policy/policy.onnx"])
    robot_controllers_pointfoot_params_file = PathJoinSubstitution([FindPackageShare("robot_controllers"), "config/pointfoot/"+ robot_type +"/params.yaml"])
    robot_controllers_file = PathJoinSubstitution([FindPackageShare("robot_controllers"), "config/robot_controllers.yaml"])
    robot_hw_joystick_file = PathJoinSubstitution([FindPackageShare("robot_hw"), "config/joystick.yaml"])
    robot_hw_file = PathJoinSubstitution([FindPackageShare("robot_hw"), "config/robot_hw.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_gazebo",
            default_value="true",
            description="Boolean flag to indicate whether to use Gazebo for simulation.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pointfoot_gazebo_launch_file)
        ),
        Node(
            package='robot_hw',
            executable='pointfoot_node',
            name='robot_hw_node',
            output='screen',
            arguments=["127.0.0.1"],
            parameters=[
                {
                    "robot_controllers_model_file": robot_controllers_model_file, 
                    "use_gazebo": LaunchConfiguration("use_gazebo"),
                },
                robot_controllers_pointfoot_params_file, 
                robot_controllers_file,
                robot_hw_joystick_file,
                robot_hw_file,
            ],
        ),
        Node(
            package='rqt_robot_steering',
            executable='rqt_robot_steering',
            output='screen',
        ),
    ])