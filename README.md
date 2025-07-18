# English | [中文](README_cn.md)
# Deployment of Training Results



## 1. Deployment Environment Configuration

- Install ROS 2 Iron: Set up a ROS 2 Iron-based algorithm Development Environment on the Ubuntu 20.04 operating system. For installation, please refer to the documentation: https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html, and choose "ros-iron-desktop" for installation. After the installation of ROS 2 Iron is completed, enter the following Shell commands in the Bash end point to install the libraries required by the Development Environment:

    ```bash
    sudo apt update
    sudo apt install ros-iron-urdf \
                ros-iron-urdfdom \
                ros-iron-urdfdom-headers \
                ros-iron-kdl-parser \
                ros-iron-hardware-interface \
                ros-iron-controller-manager \
                ros-iron-controller-interface \
                ros-iron-controller-manager-msgs \
                ros-iron-control-msgs \
                ros-iron-controller-interface \
                ros-iron-gazebo-* \
                ros-iron-rviz* \
                ros-iron-rqt-gui \
                ros-iron-rqt-robot-steering \
                ros-iron-plotjuggler* \
                ros-iron-control-toolbox \
                ros-iron-ros2-control \
                ros-iron-ros2-controllers \
                ros-dev-tools \
                cmake build-essential libpcl-dev libeigen3-dev libopencv-dev libmatio-dev \
                python3-pip libboost-all-dev libtbb-dev liburdfdom-dev liborocos-kdl-dev -y
    ```

    

- Install the onnxruntime dependency, download link:https://github.com/microsoft/onnxruntime/releases/tag/v1.10.0. Please choose the appropriate version to download according to your operating system and platform. For example, on Ubuntu 20.04 x86_64, please follow the steps below for installation:
  
    ```Bash
    wget https://github.com/microsoft/onnxruntime/releases/download/v1.10.0/onnxruntime-linux-x64-1.10.0.tgz
    
    tar xvf onnxruntime-linux-x64-1.10.0.tgz
    
    sudo cp -a onnxruntime-linux-x64-1.10.0/include/* /usr/include
    sudo cp -a onnxruntime-linux-x64-1.10.0/lib/* /usr/lib
    ```



## 2. Create Workspace

You can create an RL deployment development workspace by following these steps:
- Open a Bash end point.
- Create a new directory to store the workspace. For example, you can create a directory named "limx_ws" under the user's home directory:
    ```Bash
    mkdir -p ~/limx_ws/src
    ```
    
- Download the Motion Control Development Interface:
    ```Bash
    cd ~/limx_ws/src
    git clone https://github.com/limxdynamics/limxsdk-lowlevel.git
    ```
    
- Download Gazebo Simulator:
    ```Bash
    cd ~/limx_ws/src
    git clone https://github.com/limxdynamics/tron1-gazebo-ros2.git
    ```
    
- Download the robot model description file
    ```Bash
    cd ~/limx_ws/src
    git clone https://github.com/limxdynamics/robot-description.git
    ```
    
- Download Visualization Tool
    ```Bash
    cd ~/limx_ws/src
    git clone https://github.com/limxdynamics/robot-visualization.git
    ```
    
- Download RL deployment source code:
    ```Bash
    cd ~/limx_ws/src
    git clone https://github.com/limxdynamics/tron1-rl-deploy-ros2.git
    ```
    
- Compile Project:
    ```Bash
    cd ~/limx_ws
    source /opt/ros/iron/setup.bash
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

- Select robot type

  - List available robot types via the Shell command tree -L 1 src/robot-description/pointfoot : 
  
    ```
    src/robot-description/pointfoot
    ├── PF_P441A
    ├── PF_P441B
    ├── PF_P441C
    ├── PF_P441C2
    ├── PF_TRON1A
    ├── SF_TRON1A
    └── WF_TRON1A
    ```
  
  - TakingPF_P441C (please replace it according to the actual robot type) as an example, set the robot model type:
  
    ```
    echo 'export ROBOT_TYPE=PF_P441C' >> ~/.bashrc && source ~/.bashrc
    ```
  
- Run Simulation
  
  ```
  source /opt/ros/iron/setup.bash
  source install/setup.bash
  ros2 launch robot_hw pointfoot_hw_sim.launch.py
  ```
  ![](doc/simulator.gif)

