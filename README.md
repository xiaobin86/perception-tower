# perception_tower

基于 ROS2 Humble 的感知塔控制与转盘扫描拼合包。

## 架构

```
Ubuntu 机器（传感器）          macOS/任意机器（本包）
┌─────────────────────┐      ┌──────────────────────────────┐
│ rslidar_sdk         │──┐   │  perception_tower            │
│   → /fairy/points   │  │   │    ┌─ FairyBuffer (订阅)     │
│                     │  └──►│    ├─ CameraGrabber (订阅)    │
│ orbbec_camera       │──┐   │    ├─ ServoClient  (串口)     │
│   → /camera/color   │  └──►│    ├─ TowerFSM    (状态机)    │
│   → /camera/depth   │      │    └─ Stitcher     (拼合)     │
└─────────────────────┘      └──────────────────────────────┘
       DDS 发现（同局域网自动发现）
```

传感器驱动安装在 Ubuntu 机器上，本包通过 ROS2 DDS 订阅远程 topic 获取数据。
只需两台机器在同一局域网，设置相同的 `ROS_DOMAIN_ID` 即可。

## 功能

- 提供 `/perception_tower/command` 服务，支持两条指令：
  - `CMD_INIT=1`：控制转盘 RST 回零并移动到 90° 待命。
  - `CMD_SCAN=2`：在 90° 拍摄 336L 彩色/深度图，然后扫描 30°→150°，逐点时间补偿拼合 Fairy 点云，输出结果 topic 与文件。
- 发布状态 `/perception_tower/status`（transient_local，晚加入也可收到当前状态）。
- 输出：
  - `/perception_tower/stitched_points`：拼合后的 world-frame 点云
  - `/perception_tower/photo_color`、`/perception_tower/photo_depth`：90° 照片
- 支持 `mock_hardware:=true` 无硬件跑通全流程。

## 依赖安装（macOS conda）

```bash
conda create -n tower -c robostack-humble -c conda-forge --override-channels \
  python=3.11 ros-humble-ros-base ros-humble-cv-bridge \
  ros-humble-rosidl-default-generators ros-humble-ament-cmake-python \
  colcon-common-extensions pytest numpy pyserial opencv c-compiler cxx-compiler
conda activate tower
pip install empy==3.3.4
export PATH="$CONDA_PREFIX/bin:$PATH"
```

## 构建

```bash
cd /Users/acelan/workspace/perception_tower
source /opt/ros/humble/setup.bash  # Linux 生产机
conda activate tower               # macOS 开发机
export PATH="$CONDA_PREFIX/bin:$PATH"
colcon build --symlink-install --cmake-args -DPython_EXECUTABLE=$(which python)
source install/setup.bash
```

## 传感器驱动安装（Ubuntu 机器）

在 Ubuntu 机器上安装驱动，让它发布 ROS2 topic。

### RoboSense Fairy LiDAR

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/src
git clone https://github.com/RoboSense-LiDAR/rslidar_msg.git
git clone -b v1.5.19 https://github.com/RoboSense-LiDAR/rslidar_sdk.git
cd ~/ros2_ws
colcon build --packages-select rslidar_msg rslidar_sdk
source install/setup.bash
```

rslidar_sdk 配置（yaml）：
```yaml
common:
  msg_source: 1
  send_point_cloud_ros: true
lidar:
  - driver:
      lidar_type: RSFAIRY
    ros:
      ros_frame_id: lidar_link
      ros_send_point_cloud_topic: /fairy/points
```

确保 `timestamp_type` 为 `host`（逐点补偿依赖此配置）。

### Orbbec Gemini 336L

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

### 一键安装脚本

```bash
bash scripts/setup_ubuntu_sensors.sh
```

## 局域网配置

两台机器在同一局域网，设置相同 domain ID：

```bash
# 两台机器都执行
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

验证发现：
```bash
# 在 macOS 上查看远程 topic
ros2 topic list
# 应能看到 /fairy/points, /camera/color/image_raw, /camera/depth/image_raw
```

## 使用

### 硬件准备

1. 确认步进电机转盘的 USB 串口已接入（macOS: `/dev/tty.usbserial-*`，Linux: `/dev/ttyUSB0`）
2. Ubuntu 机器启动 Fairy 驱动 + Orbbec 相机驱动
3. 两台机器在同一局域网，`ROS_DOMAIN_ID` 一致

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

### Mock 模式（无硬件）

```bash
ros2 launch perception_tower tower.launch.py mock_hardware:=true
```

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
- `fairy_topic` / `color_topic` / `depth_topic`：远程传感器 topic 名
