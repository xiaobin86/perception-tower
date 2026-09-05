# perception_tower

基于 ROS2 Humble 的感知塔控制与转盘扫描拼合包（RoboSense Fairy LiDAR + Orbbec Gemini 336L）。

## 架构

```
Ubuntu 机器（传感器）              macOS/任意机器（本包）
┌──────────────────────────┐      ┌──────────────────────────────┐
│ rslidar_sdk              │──┐   │  perception_tower            │
│   → /rslidar_points      │  │   │    ┌─ FairyBuffer (订阅)     │
│                          │  └──►│    ├─ CameraGrabber (订阅)    │
│ orbbec_camera            │──┐   │    ├─ ServoClient  (串口)     │
│   → /camera/color/image_raw│ └──►│    ├─ TowerFSM    (状态机)    │
│   → /camera/depth/image_raw│     │    └─ Stitcher     (拼合)     │
└──────────────────────────┘      └──────────────────────────────┘
        DDS 发现（同局域网）
```

## 功能

- `/perception_tower/command` 服务：
  - `CMD_INIT=1`：转盘回零 + 移到 90° 待命
  - `CMD_SCAN=2`：拍照 → 30°→150° 扫描 → 逐点时间补偿拼合 → 输出点云
- `/perception_tower/status`：实时状态（transient_local）
- 输出：`/perception_tower/stitched_points`、`/perception_tower/photo_color`、`/perception_tower/photo_depth`
- 支持 `mock_hardware:=true` 无硬件仿真

---

## 快速开始（新电脑 clone 后）

### 1. macOS 开发机

#### 1.1 安装 Miniforge（如果没有）

```bash
# 下载安装 Miniforge（https://github.com/conda-forge/miniforge）
curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-MacOSX-x86_64.sh"
bash Miniforge3-MacOSX-x86_64.sh
```

#### 1.2 创建 conda 环境

```bash
conda create -n tower -c robostack-humble -c conda-forge --override-channels \
  python=3.11 \
  ros-humble-desktop \
  ros-humble-cv-bridge \
  ros-humble-rosidl-default-generators \
  ros-humble-ament-cmake-python \
  colcon-common-extensions \
  pytest numpy pyserial opencv \
  c-compiler cxx-compiler

conda activate tower
pip install empy==3.3.4
```

#### 1.3 配置环境变量

conda 激活时自动设置（已预置在 conda 环境中，无需手动操作）：
```bash
# 以下变量在 conda activate tower 时自动生效：
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

如果自动配置未生效，手动执行：
```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export PATH="$CONDA_PREFIX/bin:$PATH"
```

#### 1.4 Clone & 构建

```bash
git clone git@github.com:xiaobin86/perception-tower.git
cd perception-tower

# 编译（每次 clone 后必须执行一次）
colcon build --symlink-install --cmake-args -DPython_EXECUTABLE=$(which python)
```

> **注意**：如果 `colcon` 命令找不到，确保 `conda activate tower` 已执行。

#### 1.5 启动

```bash
# Mock 模式（无需硬件/传感器，验证流程）
ros2 launch perception_tower tower.launch.py mock_hardware:=true

# 真实模式（需要串口舵机 + Ubuntu 传感器机器在同一局域网）
ros2 launch perception_tower tower.launch.py
```

#### 1.6 调用服务

```bash
# 新终端
conda activate tower

# 初始化（回零 + 移到 90°）
ros2 service call /perception_tower/command perception_tower_interfaces/srv/TowerCommand "{command: 1}"

# 扫描
ros2 service call /perception_tower/command perception_tower_interfaces/srv/TowerCommand "{command: 2}"

# 查看状态
ros2 topic echo /perception_tower/status --no-daemon
```

> **提示**：如果 `ros2` 命令卡住，先执行 `kill $(pgrep -f ros2_daemon) 2>/dev/null` 杀掉残留进程，或使用 `--no-daemon` 参数。

#### 1.7 可视化（rviz2）

```bash
conda activate tower
rviz2
# 添加 PointCloud2 display → 话题选 /perception_tower/stitched_points
```

---

### 2. Ubuntu 传感器机

#### 2.1 安装 ROS2 Humble

```bash
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions
```

#### 2.2 安装 Fairy LiDAR 驱动

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/RoboSense-LiDAR/rslidar_msg.git
git clone -b v1.5.19 https://github.com/RoboSense-LiDAR/rslidar_sdk.git
cd ~/ros2_ws
colcon build --packages-select rslidar_msg rslidar_sdk
source install/setup.bash
```

rslidar_sdk 配置（编辑 `~/ros2_ws/src/rslidar_sdk/config/ros_rsfairy.yaml`）：
```yaml
common:
  msg_source: 1
  send_point_cloud_ros: true
lidar:
  - driver:
      lidar_type: RSFAIRY
      msop_port: 6699
    ros:
      ros_frame_id: lidar_link
      ros_send_point_cloud_topic: /rslidar_points
```

确保 `timestamp_type` 为 `host`（逐点补偿依赖此配置）。

启动：
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch rslidar_sdk std_RSFairY.launch.py
```

验证：
```bash
ros2 topic hz /rslidar_points  # 应看到 ~10Hz
```

#### 2.3 安装 Orbbec Gemini 336L 驱动

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/src
git clone -b v2-main https://github.com/orbbec/OrbbecSDK_ROS2.git
cd OrbbecSDK_ROS2 && git submodule update --init --recursive
cd ~/ros2_ws
colcon build --packages-select orbbec_camera orbbec_description
source install/setup.bash
```

启动：
```bash
ros2 launch orbbec_camera gemini_330_series.launch.py
```

输出话题：
- `/camera/color/image_raw`（1280×720@30）
- `/camera/depth/image_raw`（848×480@30）

#### 2.4 配置 DDS 网络发现

```bash
# 加入容器内 ~/.bashrc（Docker 容器内必须设置，否则跨机 DDS 发现不通）
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><Peers><Peer address="macOS机器IP"/></Peers></Discovery></Domain></CycloneDDS>'
source /opt/ros/humble/setup.bash
```

> **重要**：如果 ROS2 跑在 Docker 容器中（即使 `--network host`），容器内的传感器进程启动时必须带上 `CYCLONEDDS_URI`，否则远端机器无法发现 topic。最简单的做法是把上面的 export 写入容器的 `/root/.bashrc`。

#### 2.5 一键安装脚本

```bash
bash scripts/setup_ubuntu_sensors.sh
```

---

### 3. 局域网通信配置

两台机器必须在同一局域网，且 `ROS_DOMAIN_ID` 一致（默认 0）。

**DDS 发现不通时的排查：**

```bash
# 1. 确认两台机器能互相 ping 通
ping 192.168.x.x

# 2. 确认 ROS_DOMAIN_ID 和 ROS_LOCALHOST_ONLY 一致
echo $ROS_DOMAIN_ID     # 两台都应为 0
echo $ROS_LOCALHOST_ONLY # 两台都应为 0

# 3. 确认 Ubuntu 端能看到自己的 topic
ros2 topic list  # 应看到 /rslidar_points 等

# 4. macOS 端查看远端 topic
ros2 topic list --no-daemon
```

**如果 FastDDS 多播发现失败（macOS 常见），使用 CycloneDDS：**

```bash
# 安装
conda activate tower
/Users/acelan/miniforge3/bin/mamba install -y -n tower -c conda-forge rmw_cyclonedds_cpp

# 设置（加入 conda activate.d 或手动执行）
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>auto</NetworkInterfaceAddress><Peers><Peer address="Ubuntu机器IP"/></Peers></General></Domain></CycloneDDS>'

# 重启 ros2 daemon
kill $(pgrep -f ros2_daemon) 2>/dev/null
ros2 daemon start
ros2 topic list --no-daemon
```

---

## 常见问题

| 问题 | 解决 |
|------|------|
| `ros2 topic list` 看不到远端 topic | 确认两台 `ROS_DOMAIN_ID=0`、`ROS_LOCALHOST_ONLY=0`，ping 通。**如果传感器在 Docker 容器内，容器里的 DDS 进程也必须设 `CYCLONEDDS_URI` peer 指向远端机器，双向 peer 才能发现** |
| `ros2 topic list` 超时 | `kill $(pgrep -f ros2_daemon)` 或加 `--no-daemon` |
| `Package 'perception_tower' not found` | 执行 `colcon build --symlink-install --cmake-args -DPython_EXECUTABLE=$(which python)` |
| `source install/setup.bash` 报错 | 改用 `source install/local_setup.bash` 或不 source，conda 激活脚本已自动配置 |
| `rviz2` 找不到 | `conda install -c robostack-humble -c conda-forge ros-humble-desktop` |
| `ModuleNotFoundError: rclpy` | 确认 `conda activate tower`，不要用系统 Python |
| `The passed service type is invalid` | 先 `ros2 daemon stop`，再 `ros2 service list --no-daemon` 确认服务存在 |

## 输出目录

默认 `/tmp/perception_tower/YYYYMMDD_HHMMSS/`：
- `color.png`：彩色图
- `depth.png`：16-bit 深度图（毫米）
- `stitched.pcd`：拼合点云
- `angle_log.csv`：100Hz 转盘角度日志

## 关键参数

见 `config/tower_params.yaml`：
- `serial_port` / `serial_baud`：转盘串口
- `ready_deg` / `scan_start_deg` / `scan_end_deg` / `sweep_speed_deg_s`：扫描范围与速度
- `mount_rpy_deg` / `mount_offset_xyz`：LiDAR 横装外参与偏心距
- `voxel_leaf_m`：体素下采样叶节点大小（0 关闭）
- `fairy_topic` / `color_topic` / `depth_topic`：远程传感器 topic 名

## 单元测试

```bash
conda activate tower
cd perception_tower
python -m pytest test/ --ignore=test/test_tower_node.py -v
```
