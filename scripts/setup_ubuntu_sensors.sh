#!/bin/bash
# Ubuntu 传感器驱动安装脚本
# 在 Ubuntu 22.04 + ROS2 Humble 机器上运行
set -e

echo "=== 1. 安装 ROS2 Humble（如果还没装）==="
if ! command -v ros2 &> /dev/null; then
    sudo apt update && sudo apt install -y software-properties-common
    sudo add-apt-repository universe
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update && sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions
fi
source /opt/ros/humble/setup.bash

echo "=== 2. 安装编译依赖 ==="
sudo apt install -y \
    ros-humble-sensor-msgs ros-humble-std-msgs ros-humble-geometry-msgs \
    ros-humble-rosidl-default-generators ros-humble-ament-cmake \
    python3-colcon-common-extensions cmake build-essential \
    libpcap-dev libeigen3-dev

echo "=== 3. 安装 Orbbec Gemini 336L 驱动 ==="
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
if [ ! -d "OrbbecSDK_ROS2" ]; then
    git clone -b v2-main https://github.com/orbbec/OrbbecSDK_ROS2.git
    cd OrbbecSDK_ROS2 && git submodule update --init --recursive
    cd ~/ros2_ws/src
fi

echo "=== 4. 安装 Fairy LiDAR 驱动 ==="
if [ ! -d "rslidar_msg" ]; then
    git clone https://github.com/RoboSense-LiDAR/rslidar_msg.git
fi
if [ ! -d "rslidar_sdk" ]; then
    git clone -b v1.5.19 https://github.com/RoboSense-LiDAR/rslidar_sdk.git
fi

echo "=== 5. 编译 ==="
cd ~/ros2_ws
colcon build --packages-select rslidar_msg
source install/setup.bash
colcon build --packages-select rslidar_sdk orbbec_camera orbbec_description
source install/setup.bash

echo "=== 6. 配置 DDS 网络发现（允许局域网通信）==="
# FastDDS 默认用多播发现，同局域网即可
# 如果多播不通，可以用环境变量指定对端 IP
cat >> ~/.bashrc << 'EOF'

# ROS2 局域网通信
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
# 如果用 CycloneDDS（可选），取消下面注释：
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
EOF

source ~/.bashrc

echo ""
echo "=== 安装完成 ==="
echo ""
echo "启动传感器驱动："
echo "  # 终端1: Fairy LiDAR"
echo "  source ~/ros2_ws/install/setup.bash"
echo "  ros2 launch rslidar_sdk std_RSFairY.launch.py"
echo ""
echo "  # 终端2: Orbbec 相机"
echo "  source ~/ros2_ws/install/setup.bash"
echo "  ros2 launch orbbec_camera gemini_330_series.launch.py"
echo ""
echo "验证话题发布："
echo "  ros2 topic list"
echo "  ros2 topic hz /fairy/points"
echo "  ros2 topic hz /camera/color/image_raw"
