# 中文 | [English](README.md)
# 训练结果部署



## 1. 部署环境配置

- 安装ROS 2 Humble：在 Ubuntu 22.04 操作系统上建立基于 ROS 2 Humble 的算法开发环境。 安装请参考文档： https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html ，选择“ros-humble-desktop”进行安装。ROS 2 Humble 安装完成后，Bash 终端输入以下 Shell 命令，安装开发环境所依赖的库：

    ```
    sudo apt update
    sudo apt install ros-humble-urdf \
                        ros-humble-urdfdom \
                        ros-humble-urdfdom-headers \
                        ros-humble-kdl-parser \
                        ros-humble-hardware-interface \
                        ros-humble-controller-manager \
                        ros-humble-controller-interface \
                        ros-humble-controller-manager-msgs \
                        ros-humble-control-msgs \
                        ros-humble-controller-interface \
                        ros-humble-gazebo-* \
                        ros-humble-rviz* \
                        ros-humble-rqt-gui \
                        ros-humble-rqt-robot-steering \
                        ros-humble-plotjuggler* \
                        ros-humble-control-toolbox \
                        ros-humble-ros2-control \
                        ros-humble-ros2-controllers \
                        ros-humble-robot-controllers \
                        ros-humble-robot-controllers-interface \
                        ros-humble-robot-controllers-msgs \
                        ros-dev-tools \
                        cmake build-essential libpcl-dev libeigen3-dev libopencv-dev libmatio-dev \
                        python3-pip libboost-all-dev libtbb-dev liburdfdom-dev liborocos-kdl-dev -y
    ```

    

- 安装onnxruntime依赖，下载连接：https://github.com/microsoft/onnxruntime/releases/tag/v1.10.0  。请您根据自己的操作系统和平台选择合适版本下载。如在Ubuntu 22.04 x86_64，请按下面步骤进行安装：
  
    ```Bash
    wget https://github.com/microsoft/onnxruntime/releases/download/v1.10.0/onnxruntime-linux-x64-1.10.0.tgz
    
    tar xvf onnxruntime-linux-x64-1.10.0.tgz
    
    sudo cp -a onnxruntime-linux-x64-1.10.0/include/* /usr/include
    sudo cp -a onnxruntime-linux-x64-1.10.0/lib/* /usr/lib
    ```



## 2. 创建工作空间

可以按照以下步骤，创建一个RL部署开发工作空间：

- 打开一个Bash终端。
- 创建一个新目录来存放工作空间。例如，可以在用户的主目录下创建一个名为“limx_ws”的目录：
    ```Bash
    mkdir -p ~/limx_ws/src
    ```
- 下载机器人模型描述文件
    ```Bash
    cd ~/limx_ws/src
    git clone https://github.com/limxdynamics/robot-description.git
    ```
- 下载运动控制开发接口：

    ```Bash
    cd ~/limx_ws/src
    git clone https://github.com/limxdynamics/limxsdk-lowlevel.git
    ```
- 下载可视化工具
    ```Bash
    cd ~/limx_ws/src
    git clone https://github.com/limxdynamics/robot-visualization.git
    ```
- 下载Gazebo仿真器：
    ```Bash
    cd ~/limx_ws/src
    git clone -b feature/humble https://github.com/limxdynamics/tron1-gazebo-ros2.git
    ```
- 下载RL部署源码：
    ```Bash
    cd ~/limx_ws/src
    git clone -b feature/humble https://github.com/limxdynamics/tron1-rl-deploy-ros2.git
    ```
- 编译工程：
    ```Bash
    cd ~/limx_ws
    source /opt/ros/humble/setup.bash
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

- 选择机器人类型

  - 通过 Shell 命令 `tree -L 1 src/robot-description/pointfoot ` 列出可用的机器人类型：
  
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
  
  - 以`PF_P441C`（请根据实际机器人类型进行替换）为例，设置机器人型号类型：
  
    ```
    echo 'export ROBOT_TYPE=PF_P441C' >> ~/.bashrc && source ~/.bashrc
    ```
  
- 运行仿真

  ```
  source /opt/ros/humble/setup.bash
  source /usr/share/gazebo/setup.bash
  source install/setup.bash
  ros2 launch robot_hw pointfoot_hw_sim.launch.py
  ```
  ![](doc/simulator.gif)

