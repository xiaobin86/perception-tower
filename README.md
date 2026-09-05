# perception_tower

基于 ROS2 Humble 的感知塔控制与转盘扫描拼合包。

## 功能

- 提供 `/perception_tower/command` 服务，支持两条指令：
  - `CMD_INIT=1`：控制转盘 RST 回零并移动到 90° 待命。
  - `CMD_SCAN=2`：在 90° 拍摄 336L 彩色/深度图，然后扫描 30°→150°，逐点时间补偿拼合 Fairy 点云，输出结果 topic 与文件。
- 发布状态 `/perception_tower/status`（transient_local，晚加入也可收到当前状态）。
- 输出：
  - `/perception_tower/stitched_points`：拼合后的 world-frame 点云
  - `/perception_tower/photo_color`、`/perception_tower/photo_depth`：90° 照片
- 支持 `mock_hardware:=true` 无硬件跑通全流程。

## 依赖安装（空机器 / macOS conda）

```bash
conda create -n tower -c robostack-humble -c conda-forge --override-channels \
  python=3.11 ros-humble-ros-base ros-humble-cv-bridge \
  ros-humble-rosidl-default-generators ros-humble-ament-cmake-python \
  colcon-common-extensions pytest numpy pyserial opencv c-compiler cxx-compiler
conda activate tower
# 修复 empy 版本兼容性（rosidl 需要 3.x）
pip install empy==3.3.4
# macOS 上 cmake 需要用 conda 环境的版本（避免 homebrew 的旧版 cmake）
export PATH="$CONDA_PREFIX/bin:$PATH"
```

## 构建

```bash
cd /Users/acelan/workspace/perception_tower
source /opt/ros/humble/setup.bash  # Linux 生产机
conda activate tower               # macOS 开发机
export PATH="$CONDA_PREFIX/bin:$PATH"  # macOS 确保用 conda cmake
colcon build --symlink-install --cmake-args -DPython_EXECUTABLE=$(which python)
source install/setup.bash
```

## 传感器驱动安装

> **注意**：两个驱动都是 Linux C++ ROS2 包（rslidar_sdk 使用 Linux 专有 `recvmmsg` API，orbbec_camera 依赖 libusb），**只能在 Linux 生产机上编译安装**，macOS 开发机不支持。macOS 上请使用 `mock_hardware:=true` 模式。

### RoboSense Fairy LiDAR

在 **Linux 生产机**上安装 [rslidar_sdk](https://github.com/RoboSense-LiDAR/rslidar_sdk)（≥v1.5.19，支持 RSFAIRY，ROS2 Humble）：

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/src
git clone https://github.com/RoboSense-LiDAR/rslidar_msg.git
git clone -b v1.5.19 https://github.com/RoboSense-LiDAR/rslidar_sdk.git
cd ~/ros2_ws
colcon build --packages-select rslidar_msg rslidar_sdk
source install/setup.bash
```

关键配置（在 rslidar_sdk 的 yaml 中）：
```yaml
common:
  msg_source: 1                  # 1=点云来自真实雷达
  send_point_cloud_ros: true
lidar:
  - driver:
      lidar_type: RSFAIRY        # Fairy 型号
    ros:
      ros_frame_id: lidar_link
      ros_send_point_cloud_topic: /fairy/points
```

**重要**：确保 `timestamp_type` 为 `host` 时间基准（逐点补偿依赖此配置）。

### Orbbec Gemini 336L

在 **Linux 生产机**上安装 [OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2)（`v2-main` 分支）：

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/src
git clone -b v2-main https://github.com/orbbec/OrbbecSDK_ROS2.git
cd OrbbecSDK_ROS2 && git submodule update --init --recursive
cd ~/ros2_ws
colcon build --packages-select orbbec_camera orbbec_description
source install/setup.bash
```

启动相机驱动：
```bash
ros2 launch orbbec_camera gemini_330_series.launch.py
```

输出话题：
- `/camera/color/image_raw`（1280×720@30，sensor_msgs/Image）
- `/camera/depth/image_raw`（848×480@30，sensor_msgs/Image）

## 使用

### 硬件准备

1. 确认步进电机转盘的 USB 串口已接入：
   - Linux：`/dev/ttyUSB0`
   - macOS：`/dev/tty.usbserial-*`
2. 启动 Fairy 驱动（rslidar_sdk，配置 `timestamp_type` 为 host 时间基准）。
3. 启动 Orbbec Gemini 336L 驱动（参考 `pallet_vision_lidar/launch/gemini_336l_custom.launch.py`）。

### 启动本节点

```bash
ros2 launch perception_tower tower.launch.py
```

### 服务调用

```bash
# 初始化
ros2 service call /perception_tower/command perception_tower_interfaces/srv/TowerCommand "{command: 1}"

# 启动扫描
ros2 service call /perception_tower/command perception_tower_interfaces/srv/TowerCommand "{command: 2}"
```

调用返回 `accepted=true` 表示已受理；通过状态 topic 查看进度：

```bash
ros2 topic echo /perception_tower/status
```

### Mock 模式（无硬件）

```bash
ros2 launch perception_tower tower.launch.py mock_hardware:=true
ros2 service call /perception_tower/command perception_tower_interfaces/srv/TowerCommand "{command: 1}"
ros2 service call /perception_tower/command perception_tower_interfaces/srv/TowerCommand "{command: 2}"
```

### macOS 原生桥接（无 rslidar_sdk / orbbec_camera）

macOS 无法编译 Linux 专有驱动，本包提供两个纯 Python 桥接节点：

**Fairy LiDAR UDP 桥接** — 直接接收 Fairy MSOP UDP 包，发布 PointCloud2：
```bash
# 启动桥接（监听 UDP 6699 端口）
ros2 run perception_tower fairy_udp_bridge

# 或在 launch 中同时启动
ros2 launch perception_tower tower.launch.py use_fairy_bridge:=true
```

**Orbbec 相机桥接** — 需要 `pyorbbecsdk`（ARM64 macOS）或 `opencv-python`（fallback）：
```bash
# ARM64 macOS（pip install pyorbbecsdk）
ros2 run perception_tower orbbec_bridge

# x86_64 macOS（自动 fallback 到 OpenCV）
pip install opencv-python
ros2 run perception_tower orbbec_bridge
```

> **注意**：pyorbbecsdk 的 x86_64 轮子有打包 bug（.so 是 arm64），x86_64 macOS 只能用 OpenCV fallback（无深度数据）。完整 Orbbec 支持需要 ARM64 macOS 或从源码编译 OrbbecSDK_v2。

## 输出目录

默认 `/tmp/perception_tower/YYYYMMDD_HHMMSS/` 下包含：
- `color.png`：彩色图
- `depth.png`：16-bit 深度图（毫米）
- `stitched.pcd`：拼合点云（save_cloud=true 时）
- `angle_log.csv`：100Hz 转盘角度日志

## 关键参数

见 `config/tower_params.yaml`：
- `serial_port` / `serial_baud`：转盘串口
- `ready_deg` / `scan_start_deg` / `scan_end_deg` / `sweep_speed_deg_s`：扫描范围与速度
- `mount_rpy_deg` / `mount_offset_xyz`：LiDAR 横装外参与偏心距
- `voxel_leaf_m`：体素下采样叶节点大小（0 关闭）
- `fairy_time_field`：是否启用逐点时间补偿
