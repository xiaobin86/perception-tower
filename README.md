# perception_tower

基于 ROS2 Humble 的感知塔控制与转盘扫描拼合包（RoboSense Fairy LiDAR + Orbbec Gemini 336L）。

## 架构

```
Ubuntu 机器（Docker: perception_tower_sensor_env）
┌──────────────────────────────────────────────┐
│  perception_tower 容器 (--network host)       │
│  ┌─────────────────┐  ┌──────────────────┐  │
│  │ rslidar_sdk      │  │ turntable_node   │  │
│  │  → /rslidar_points│  │  → /turntable/status│
│  │                  │  │  ← /turntable/command│
│  │ orbbec_camera    │  │  (串口 → STM32)   │  │
│  │  → /camera/*     │  └──────────────────┘  │
│  └─────────────────┘                         │
└───────────────────────│──────────────────────┘
                        │ CycloneDDS
                        ▼
macOS 机器（本包）
┌──────────────────────────────────────────────┐
│  perception_tower                            │
│    ├─ FairyBuffer   (订阅 /rslidar_points)   │
│    ├─ CameraGrabber (订阅 /camera/*)         │
│    ├─ TurntableCmd  (调用 /turntable/command) │
│    ├─ AngleLogger   (订阅 /turntable/status)  │
│    ├─ TowerFSM      (状态机)                  │
│    ├─ Stitcher      (逐点时间补偿拼合)        │
│    └─ TowerGUI      (tkinter 控制面板)        │
└──────────────────────────────────────────────┘
```

## 功能

- `/perception_tower/command` 服务：
  - `CMD_INIT=1`：转盘回零 + 移到 90° 待命
  - `CMD_SCAN=2`：拍照 → 30°→150° 扫描 → 逐点时间补偿拼合 → 输出点云
- `/perception_tower/status`：实时状态（transient_local）
- 输出：`/perception_tower/stitched_points`、`/perception_tower/photo_color`、`/perception_tower/photo_depth`
- 支持 `mock_hardware:=true` 无硬件仿真
- 支持 `gui:=true` tkinter 可视控制面板

---

## 快速开始（新电脑 clone 后）

### 1. macOS 开发机

#### 1.1 安装 Miniforge（如果没有）

```bash
# 下载安装 Miniforge（https://github.com/conda-forge/miniforge）
curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-MacOSX-x86_64.sh"
bash Miniforge3-MacOSX-x86_64.sh
```

#### 1.2 创建并配置 conda 环境

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
pip install cmake==3.28.3
```

> 注意：ROS2 Humble 与 cmake 4.x 不兼容，必须安装 cmake 3.28.x。homebrew 的 cmake 4.x 会导致 `Could NOT find Python` 错误。

配置 conda 自动激活脚本（只需执行一次）：

```bash
mkdir -p "$CONDA_PREFIX/etc/conda/activate.d" "$CONDA_PREFIX/etc/conda/deactivate.d"

cat > "$CONDA_PREFIX/etc/conda/activate.d/env_vars.sh" <<'EOF'
#!/bin/bash
# Auto-configure environment when conda env 'tower' is activated.
export PATH="$CONDA_PREFIX/bin:$PATH"
if [ -z "${_TOWER_OLD_PATH+x}" ]; then
    export _TOWER_OLD_PATH="$PATH"
fi
if [ -f "$CONDA_PREFIX/setup.bash" ]; then
    pushd "$CONDA_PREFIX" > /dev/null
    source setup.bash
    popd > /dev/null
fi
EOF

cat > "$CONDA_PREFIX/etc/conda/deactivate.d/env_vars.sh" <<'EOF'
#!/bin/bash
if [ -n "${_TOWER_OLD_PATH+x}" ]; then
    export PATH="$_TOWER_OLD_PATH"
    unset _TOWER_OLD_PATH
fi
EOF

chmod +x "$CONDA_PREFIX/etc/conda/activate.d/env_vars.sh" "$CONDA_PREFIX/etc/conda/deactivate.d/env_vars.sh"
```

之后每次 `conda activate tower` 会自动：
- 把 conda bin 放到 PATH 最前面（使用 cmake 3.28.3）
- source ROS2 Humble setup

#### 1.3 环境变量

conda 激活时自动设置（无需手动操作）：
```bash
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

# 编译
rm -rf build install log
colcon build --packages-select perception_tower_interfaces perception_tower_sensor_interfaces perception_tower
```

> **注意**：如果 `colcon` 命令找不到，确保 `conda activate tower` 已执行。

#### 1.5 启动

```bash
# Mock 模式（无需硬件/传感器，验证流程）
ros2 launch perception_tower tower.launch.py mock_hardware:=true

# 真实模式（需要 Ubuntu 传感器机器在同一局域网）
ros2 launch perception_tower tower.launch.py

# 启动 GUI 控制面板
ros2 launch perception_tower tower.launch.py gui:=true

# Mock + GUI
ros2 launch perception_tower tower.launch.py mock_hardware:=true gui:=true
```

#### 1.6 GUI 控制面板

启动 `gui:=true` 后弹出 tkinter 窗口：

- **CMD_INIT (Reset & Home)**：转盘回零 + 移到 90° 待命位置
- **CMD_SCAN (Scan)**：拍照 → 30°~150° 扫描 → 点云拼合
- **Status 区域**：实时显示状态（颜色编码）+ 进度条
- **Log 区域**：深色主题滚动日志，带时间戳

状态颜色：IDLE 灰 | INITING 黄 | READY 绿 | SCANNING 蓝 | PROCESSING 紫 | ERROR 红

#### 1.7 命令行调用服务

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

#### 1.8 可视化（rviz2）

```bash
conda activate tower
rviz2
# 添加 PointCloud2 display → 话题选 /perception_tower/stitched_points
```

---

### 2. Ubuntu 传感器机（Docker 部署）

传感器驱动通过 Docker 容器部署，仓库：`git@github.com:xiaobin86/perception_tower_sensor_env.git`

#### 2.1 克隆传感器环境仓库

```bash
git clone git@github.com:xiaobin86/perception_tower_sensor_env.git
cd perception_tower_sensor_env
```

#### 2.2 构建 Docker 镜像

```bash
docker compose -f docker/docker-compose.yml build
```

> 首次构建约 10-15 分钟（编译 Orbbec SDK + rslidar_sdk）。

#### 2.3 启动容器

```bash
docker compose -f docker/docker-compose.yml up -d
docker exec -it perception_tower bash
```

容器内环境已自动配置（`.bashrc`）：
- `ROS_DOMAIN_ID=0`、`ROS_LOCALHOST_ONLY=0`
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- CycloneDDS peer 指向 macOS 机器 IP
- 工作空间：`/opt/orbbec_ws`（相机）、`/opt/fairy_ws`（LiDAR）

#### 2.4 构建 turntable 包

```bash
# 容器内
cd /workspace
colcon build --packages-select perception_tower_sensor_interfaces perception_tower_sensor
source install/setup.bash
```

#### 2.5 一键启动所有硬件

```bash
# 容器内 - 一键启动 LiDAR + 相机 + 转盘
ros2 launch perception_tower_sensor sensor_env.launch.py

# 指定转盘串口
ros2 launch perception_tower_sensor sensor_env.launch.py turntable_port:=/dev/ttyUSB1
```

#### 2.6 验证话题发布

```bash
ros2 topic list
# 应看到：
#   /rslidar_points          (PointCloud2, ~10Hz)
#   /camera/color/image_raw  (Image, ~30Hz)
#   /camera/depth/image_raw  (Image, ~30Hz)
#   /turntable/status        (TurntableStatus, ~50Hz)

ros2 topic hz /turntable/status    # ~50Hz
ros2 topic echo /turntable/status --no-daemon

# 测试转盘命令
ros2 service call /turntable/command perception_tower_sensor_interfaces/srv/TurntableCommand "{command: 1, target_deg: 90.0, duration_s: 0.0}"
```

---

### 3. 局域网通信配置

两台机器必须在同一局域网，且 `ROS_DOMAIN_ID` 一致（默认 0）。

```
macOS (192.168.3.187)  ←── WiFi ──→  Ubuntu (192.168.3.162)
                                          │
                                     Docker 容器（--network host）
                                          │
                          ┌───────────────┴───────────────┐
                     Fairy LiDAR        Orbbec 相机     转盘 STM32
                   (Ethernet, UDP)      (USB 直通)     (USB 串口)
```

**DDS 发现不通时的排查：**

```bash
# 1. 确认两台机器能互相 ping 通
ping 192.168.x.x

# 2. 确认 ROS_DOMAIN_ID 和 ROS_LOCALHOST_ONLY 一致
echo $ROS_DOMAIN_ID     # 两台都应为 0
echo $ROS_LOCALHOST_ONLY # 两台都应为 0

# 3. 确认 Ubuntu 容器内能看到自己的 topic
ros2 topic list  # 应看到 /rslidar_points, /turntable/status 等

# 4. macOS 端查看远端 topic
ros2 topic list --no-daemon
```

**如果 CycloneDDS 发现失败（macOS 常见）：**

```bash
# macOS 端设置（加入 conda activate.d 或手动执行）
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///path/to/perception_tower/config/cyclone.xml

# 重启 ros2 daemon
kill $(pgrep -f ros2_daemon) 2>/dev/null
ros2 daemon start
ros2 topic list --no-daemon
```

> **注意**：CycloneDDS XML 配置必须放在文件中（`file://` URI），内联 XML 会因 shell 转义问题失败。配置文件见 `config/cyclone.xml`。

---

## ROS2 接口总览

| 类型 | 名称 | 方向 | 说明 |
|------|------|------|------|
| **Service** | `/perception_tower/command` | 外部 → node | CMD_INIT / CMD_SCAN |
| **Publisher** | `/perception_tower/status` | node → 外部 | FSM 状态 + 进度 |
| **Publisher** | `/perception_tower/stitched_points` | node → 外部 | 拼合点云 |
| **Publisher** | `/perception_tower/photo_color` | node → 外部 | 彩色照片 |
| **Publisher** | `/perception_tower/photo_depth` | node → 外部 | 深度照片 |
| **Subscriber** | `/turntable/status` | sensor_env → node | 转盘位置 (50Hz) |
| **Service Client** | `/turntable/command` | node → sensor_env | 转盘控制 |

---

## 常见问题

| 问题 | 解决 |
|------|------|
| `ros2 topic list` 看不到远端 topic | 确认两台 `ROS_DOMAIN_ID=0`、`ROS_LOCALHOST_ONLY=0`，ping 通。**容器内 DDS 也必须设 `CYCLONEDDS_URI` peer 指向远端机器** |
| `ros2 topic list` 超时 | `kill $(pgrep -f ros2_daemon)` 或加 `--no-daemon` |
| `Package 'perception_tower' not found` | 执行 `colcon build --packages-select perception_tower_interfaces perception_tower_sensor_interfaces perception_tower` |
| `Could NOT find Python` | conda env 中 `pip install cmake==3.28.3`，避免使用 homebrew cmake 4.x |
| `source install/setup.bash` 报错 | 改用 `source install/local_setup.bash` 或不 source，conda 激活脚本已自动配置 |
| `rviz2` 找不到 | `conda install -c robostack-humble -c conda-forge ros-humble-desktop` |
| `ModuleNotFoundError: rclpy` | 确认 `conda activate tower`，不要用系统 Python |
| `The passed service type is invalid` | 先 `ros2 daemon stop`，再 `ros2 service list --no-daemon` 确认服务存在 |
| Docker 容器内 USB 设备不可见 | 确认 `docker-compose.yml` 中 `privileged: true` 且 `/dev:/dev` 已挂载 |
| Orbbec 相机无图像输出 | 重启 orbbec_camera 节点；确认 SDK 版本为 v2.8.6（有 USB PAL 支持） |
| 转盘串口连不上 | 确认 `turntable_port` 参数指向正确的 `/dev/ttyUSB*`，容器内 `ls /dev/ttyUSB*` 查看 |

## 输出目录

默认 `/tmp/perception_tower/YYYYMMDD_HHMMSS/`：
- `color.png`：彩色图
- `depth.png`：16-bit 深度图（毫米）
- `stitched.pcd`：拼合点云
- `angle_log.csv`：50Hz 转盘角度日志

## 关键参数

见 `config/tower_params.yaml`：
- `turntable_cmd_service` / `turntable_status_topic`：转盘 ROS2 接口名
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
