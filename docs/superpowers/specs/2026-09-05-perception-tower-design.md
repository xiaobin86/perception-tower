# 感知塔控制与转盘扫描拼合包 设计文档

- 日期：2026-09-05
- 项目：`perception_tower`（新项目，位于 `/Users/acelan/workspace/perception_tower/`）
- 目标环境：ROS2 Humble（生产 Linux / 开发 macOS conda RoboStack）
- 状态：已与需求方逐条确认（见 §12 决策记录）

---

## 1. 项目定位

提供可被其他项目安装（colcon build + 依赖声明）的 ROS2 Humble 包 `perception_tower`，控制一台**硬件感知塔**并对外提供**服务接口**：

- 感知塔 = 步进电机转盘 + RoboSense Fairy LiDAR（横装）+ Orbbec Gemini 336L 深度相机（正装），三设备**同心**，随转盘绕世界 Z 轴旋转。
- 转盘由已有固件 `servo-control`（STM32F103，串口 115200 8N1）控制，串口已接好。
- 本包作为 ROS2 service 服务端，接收两类指令：**初始化**、**启动扫描**；扫描输出 30°~150° 拼合 3D 点云与 336L 照片。

### 1.1 不做什么（YAGNI）

- 不包含 Fairy / 336L 驱动本身（外部安装 rslidar_sdk、orbbec_camera；本包只订阅其话题）。
- 不做相机-LiDAR 联合标定工具（外参由参数文件给定）。
- 不做 ROS2 Action 接口（需求方确认用 Service + Status topic）。
- 不做多转盘 / 多设备 ID（固件 ID 固定 000，参数化但默认 000）。

---

## 2. 硬件与固件契约（来源：`servo-driver/servo-control` 源码，非猜测）

### 2.1 串口协议（115200 8N1，指令以 `#` 起 `!` 止）

| 动作 | 指令 | 固件回复 |
|------|------|----------|
| 移动 | `#000P{pos}T{time}!` | 无（**非阻塞**，固件立即返回继续接收） |
| 读实时位置 | `#000PRAD!` | `#000P{pos}!\r\n`（**运动中返回实时位置**） |
| 停止 | `#000PDST!` | `#OK!` |
| 回零/复位 | `#000PRST!` | 回零全程**阻塞固件**，完成后回 `#OK!`（期间其他指令被拒） |

- 位置范围 500~18000，`0.02°/脉冲`；**角度 = (pos − 500) × 0.02**，原点 500 = 0°（限位开关处）。
  - 90° → pos 5000；30° → pos 2000；150° → pos 8000。
- time 参数最多 5 位（≤99999ms）；位置最多 6 位。
- **固件会在同一串口混发调试串**：`BOOT:` / `DBG:` / `MOV:` 等行，解析端必须容错过滤。
- 梯形加减速导致角度-时间非线性 → 必须 100Hz 轮询位置记录而非按指令时间推算。

### 2.2 传感器与安装

| 设备 | 驱动 | 安装 | 输出 |
|------|------|------|------|
| RoboSense Fairy | rslidar_sdk（≥v1.5.19 支持 RSFAIRY，ROS2 Humble） | 横装：绕自身 X 轴 +90°，360° 扫描面变**竖直扇面**；与转盘同心 | `sensor_msgs/PointCloud2`，帧内每点带 `time` 字段（逐点补偿依赖） |
| Orbbec Gemini 336L | Orbbec SDK ROS2（`orbbec_camera`，参考 `pallet_vision_lidar/launch/gemini_336l_custom.launch.py`：color 1280×720@30 + depth 848×480@30） | 正装 | `/camera/color/image_raw`、`/camera/depth/image_raw` 等 |

坐标系（沿袭 `pallet_vision_lidar` turntable 约定）：
- **{W} 世界系**：原点在转盘轴心，Z 朝上（旋转轴），X = 转盘处于 0°（原点）时塔的正前方向。
- **{L} 雷达系**：RoboSense 出厂系（X 前、Y 左、Z 上）。
- 每帧变换：`P_world = Rz(θ_p) · (R_mount · P_lidar + T_mount)`，其中 `R_mount` 默认 `Rx(+90°)`、`T_mount` 默认 `[0,0,0]`（同心），均可参数化（供标定后修正）。
- 位置→角度方向约定：**pos 增大 = 绕 +Z 逆时针**（右手定则）。若实际安装方向相反，用参数 `angle_sign = -1` 翻转，不改固件。

---

## 3. 包结构与接口

```
perception_tower/                          # 仓库根
├── perception_tower_interfaces/           # ament_cmake + rosidl（接口包）
│   ├── msg/TowerStatus.msg
│   └── srv/TowerCommand.srv
└── perception_tower/                      # ament_python（主包）
    ├── config/tower_params.yaml           # 全部参数
    ├── launch/tower.launch.py
    ├── perception_tower/
    │   ├── tower_node.py                  # 主节点：service + 状态机调度 + 话题收发
    │   ├── fsm.py                         # INIT/SCAN 流程（后台线程）
    │   ├── servo_client.py                # 串口协议客户端
    │   ├── angle_logger.py                # 100Hz 位置日志 + 时间插值
    │   ├── camera_grabber.py              # 336L color+depth 抓拍
    │   ├── fairy_buffer.py                # Fairy 帧缓存
    │   ├── stitcher.py                    # 逐点补偿 + 拼合 + PointCloud2 构建
    │   ├── geometry.py                    # 变换数学（移植自 turntable_stitcher/transform.py）
    │   ├── pc2_utils.py                   # PointCloud2 ↔ numpy 读写（含 PCD 导出）
    │   └── mock.py                        # mock 硬件（无串口/无传感器环境）
    └── test/                              # pytest 单元 + mock 集成
```

### 3.1 Service：`/perception_tower/command`（TowerCommand.srv）

```
uint8 CMD_INIT=1    # 初始化：RST 回零 → 移到 90° 待命
uint8 CMD_SCAN=2    # 启动扫描
uint8 command
---
bool accepted       # 拒绝时 false + message
string message
```

- 服务回调**非阻塞**：校验状态后立即返回 `accepted`；实际动作由后台线程执行。
- 忙状态（INITING/SCANNING/PROCESSING）下新指令拒绝：`accepted=false, message="busy: <state>"`。
- 可接受新指令的状态：`IDLE / READY / ERROR`。

### 3.2 Status：`/perception_tower/status`（TowerStatus.msg）

```
uint8 IDLE=0
uint8 INITING=1
uint8 READY=2
uint8 SCANNING=3
uint8 PROCESSING=4
uint8 ERROR=5
uint8 state
uint8 progress_pct      # 阶段内进度 0~100
string message
```

- 发布频率 ~2Hz + 状态变化立即发布；QoS：reliable + **transient_local**（调用方晚加入也能收到当前状态）。

### 3.3 输出话题

| 话题 | 类型 | frame_id | 说明 |
|------|------|----------|------|
| `/perception_tower/stitched_points` | sensor_msgs/PointCloud2 | `world` | 30°~150° 拼合点云（xyz + intensity 若源带） |
| `/perception_tower/photo_color` | sensor_msgs/Image | 相机驱动原 frame_id | 90° 拍摄彩色图 |
| `/perception_tower/photo_depth` | sensor_msgs/Image | 同上 | 90° 拍摄深度图（16UC1 mm） |

话题名均可参数化覆盖。

---

## 4. 核心流程（状态机，后台线程执行）

### 4.1 INIT

```
state=INITING
1. servo RST：发 #000PRST!，等待 #OK!（回零阻塞期固件不回包；超时 30s → ERROR）
2. 移动到 90°：#000P5000T2000! → 100Hz 轮询至 |pos−5000|≤0.1° 连续 5 次
state=READY（等待后续指令）
```

### 4.2 SCAN

```
state=SCANNING
1. 复位状态检查：PRAD 读位置；|pos−5000|≤0.1°？否 → 执行 RST + 移到 90°（同 INIT 1~2 步）
2. 90° 抓拍：取最新 color+depth（两者时间差 <200ms；等待新帧超时 5s → ERROR）
   → 保存 PNG 到输出目录 → 发布 photo_color / photo_depth
3. 启动采集：100Hz 角度日志线程 + Fairy 帧缓存开始
4. 移动到 30°：#000P2000T1000!，轮询到位
5. 连续扫描：#000P8000T3000!（30°→150° @40°/s，固件非阻塞）
   期间持续 100Hz 记录 + 缓存 Fairy 帧
6. 到位后继续采集 200ms（余量），停止采集
   采集有效性检查：Fairy 帧数 >0 且角度日志覆盖扫描窗口，否则 ERROR
state=PROCESSING
7. 后台线程拼合（见 §5），完成后发布 stitched_points、可选存 PCD
8. 转盘回 90°（直接移动，不再回零）→ state=READY，message 带结果目录路径
```

进度映射（progress_pct）：复位检查 0~10 / 抓拍 10~20 / 移到30° 20~30 / 扫描 30~70（按实际角度线性）/ 拼合 70~95 / 回90° 95~100。

### 4.3 错误处理

- 任一步失败 → `state=ERROR, message=<原因>`；串口/传感器超时不重试、不自动恢复，由调用方重新下发指令。
- 串口打开失败/写超时/回读超时 → ERROR，message 包含具体阶段与错误。
- ERROR 状态下允许重新 INIT / SCAN（SCAN 自带复位检查，可自恢复）。

---

## 5. 拼合算法（PROCESSING）

### 5.1 时间同步与逐点补偿

- 角度日志：100Hz `(host_ros_time, pos)` 序列（扫描窗口内）。
- Fairy 每点采集时刻：`t_p = frame_header.stamp + point.time`（`time` 为帧内相对偏移，秒；要求 Fairy 驱动 `timestamp_type` 配置为 host 时间基准，见 §10 集成验证项）。
- 每点转角：`θ_p = interp(angle_log, t_p)`，线性插值；超出日志范围时钳位到最近端点。
- 角度换算：原始角度（绝对位置角）`θ_raw = (pos − 500) × 0.02`（恒为正，用于扫描窗口裁剪）；旋转方向角 `θ = angle_sign × θ_raw`（仅决定 Rz 方向）。**裁剪窗口 [30°, 150°] 作用于 θ_raw**，与 `angle_sign` 无关。

### 5.2 点云变换与输出

- 固定外参：`R_mount = Rz(yaw)·Ry(pitch)·Rx(roll)`，参数 `mount_rpy_deg = [90, 0, 0]`；`T_mount = mount_offset_xyz = [0,0,0]`。
- `P_world = Rz(θ_p) · (R_mount · P_lidar + T_mount)`，逐点执行（Rz 按点向量化的 2D 旋转，避免逐点 3×3 乘法）。
- 过滤：NaN/Inf 点剔除；角度窗口裁剪 [30°, 150°]（θ_p 在窗口内才保留，天然按补偿后角度裁剪）。
- 可选 voxel 下采样（`voxel_leaf_m`，默认 0.01，0 = 关闭），实现：numpy 网格哈希取均值点。
- 输出 PointCloud2（字段 x,y,z + intensity 若源存在）并发布；`save_cloud=true` 时写二进制 PCD（自带 PCD writer，不引 open3d）。

### 5.3 数据规模预估

40°/s × 120° = 3s 扫描，Fairy 10Hz ≈ 30 帧；~7 万点/帧 ≈ 200 万点 → numpy float32 拼合内存 ~50MB 量级，可接受。帧率若改为 20Hz 翻倍仍可接受。

---

## 6. 串口客户端（servo_client.py）

- 设备参数化：`serial_port`（macOS 形如 `/dev/tty.usbserial-*`，Linux `/dev/ttyUSB0`）、`baud=115200`。
- 流式解析：字节流扫描 `#000P<数字>!`（位置回复）、`#OK!`；其余字符（`BOOT:/DBG:/MOV:` 调试串、`\r\n`）丢弃。
- 互斥模型：
  - 单一后台读线程持续收流，解析结果投递线程安全队列。
  - 命令-响应用同步请求（带超时，如 RST 等 `#OK!`）；**同一时刻只允许一条在途指令**，回复按发送顺序匹配，避免跨指令串扰。
  - 扫描期间 100Hz 轮询线程独占发送 PRAD；外部移动指令在扫描流程内部串行发出，不并发。
- `wait_until_reached(pos_target, tol_deg, timeout)`：连续 5 次读数落在容差内判定位。
- 断线处理：读/写异常抛出并置 ERROR；不做自动重连（fail fast，调用方重新走 INIT）。
- `serial_port` 为空且 `mock_hardware=false`：不打开串口，任何依赖舵机的指令（INIT/SCAN）直接拒绝并返回明确 message（"serial_port not configured"），不影响节点启动与话题订阅链路。

---

## 7. 相机抓拍（camera_grabber.py）

- 订阅 color + depth 话题，缓存各自最新一帧及到达时刻。
- 触发抓拍时：两帧均为新帧（缓存时刻距 now <300ms）且两帧时间差 <200ms → 成对取出；否则等待新帧，总超时 5s → ERROR。
- 保存：`<save_dir>/<YYYYMMDD_HHMMSS>/color.png`（8UC3）、`depth.png`（16UC1 毫米值，cv2.imwrite PNG16）。
- 发布 photo_color / photo_depth（同时保留 header 时间戳）。
- 深度图内容与彩色图分辨率可不同（848×480 vs 1280×720），各自原样保存发布，不做对齐。

---

## 8. 参数（config/tower_params.yaml，全部可覆盖）

```yaml
tower_node:
  ros__parameters:
    # 串口
    serial_port: ""            # 空 = 不连接真实舵机（配合 mock_hardware，或仅测试订阅/拼合链路）
    serial_baud: 115200
    poll_hz: 100.0             # 扫描期间位置轮询频率
    pos_tol_deg: 0.1           # 到位容差
    pos_stable_count: 5        # 连续 N 次在容差内判到位
    # 塔几何
    pos_origin: 500            # 0° 对应位置
    deg_per_pos: 0.02
    angle_sign: 1              # +1: pos 增大 = 绕 +Z 逆时针
    ready_deg: 90.0            # 复位/待命角度
    scan_start_deg: 30.0
    scan_end_deg: 150.0
    sweep_speed_deg_s: 40.0    # 扫描角速度（决定 MOVE 的 T 参数）
    home_timeout_s: 30.0       # RST 回零超时
    # Fairy 拼合
    fairy_topic: /fairy/points
    fairy_time_field: true     # 逐点补偿开关（依赖驱动每点 time）
    mount_rpy_deg: [90.0, 0.0, 0.0]
    mount_offset_xyz: [0.0, 0.0, 0.0]
    voxel_leaf_m: 0.01         # 0 = 关闭下采样
    world_frame_id: world
    # 相机
    color_topic: /camera/color/image_raw
    depth_topic: /camera/depth/image_raw
    # 输出
    output_dir: /tmp/perception_tower
    save_cloud: true
    # 输出话题
    stitched_topic: /perception_tower/stitched_points
    photo_color_topic: /perception_tower/photo_color
    photo_depth_topic: /perception_tower/photo_depth
    # 调试
    mock_hardware: false       # true: FakeServo/FakeFairy/FakeCamera
```

launch/tower.launch.py 参数：`params_file`、`mock_hardware`。传感器驱动外部启动（生产 Linux 上 `ros2 launch` rslidar_sdk 与 orbbec_camera；本包只订阅）。

---

## 9. 测试策略

| 层级 | 环境 | 内容 |
|------|------|------|
| 单元测试（pytest） | macOS conda / CI | 串口解析（含调试串脏数据混流、半包粘包）、角度插值边界、geometry 已知用例（θ=0/90/180 旋转正确性）、PCD writer 回读、相机配对逻辑 |
| FSM mock 集成 | macOS conda | `mock_hardware:=true`：FakeServo（按速度模型模拟移动）+ FakeFairy（合成旋转立方体场景）+ FakeCamera，跑 INIT→SCAN 全流程，断言拼合点云几何（合成目标真实位置 vs 拼合结果均值误差 < voxel 量级） |
| 真机集成（本机 macOS） | 真实硬件 | 真转盘串口走 INIT/SCAN；Fairy/336L 驱动若可在 macOS 构建则直连，否则 rosbag 回放真实数据过拼合管线；逐项核对：到位精度、照片保存、点云发布、状态流转 |
| 生产部署 | Linux Humble | rslidar_sdk（`timestamp_type` host）+ orbbec_camera 真机联调 |

依赖（perception_tower 包）：`rclpy`、`sensor_msgs`、`std_msgs`、`perception_tower_interfaces`、`cv_bridge`、`python3-numpy`、`python3-serial`（pyserial）。不引入 open3d / PCL。

---

## 10. 集成期验证项（真机联调时确认）

1. **Fairy 每点 time**：确认 rslidar_sdk 输出 PointCloud2 含 `time` 字段且 `timestamp_type` 为 host 时间基准；若字段缺失，`fairy_time_field=false` 降级为帧级匹配（帧中心角度）。
2. **Fairy 发布频率**：`publish_freq` 默认 10Hz（40°/s 时帧间 4°，有逐点补偿，可行）；如需更密可配 20Hz。
3. **转向一致性**：首次真机扫描用已知目标验证 `angle_sign` 与扫描方向；错误则翻参数。
4. **串口设备名**：macOS `/dev/tty.usbserial-*`（接入后 `ls /dev/tty.usb*` 确认）。
5. **336L 话题名/分辨率**：与 `gemini_336l_custom.launch.py` 实际输出对齐（color 1280×720、depth 848×480）。
6. **Orbbec SDK macOS 支持**：若不可用则走 rosbag 回放路径。

---

## 11. 里程碑（供实施计划展开）

1. 包骨架 + 接口包（msg/srv）可编译
2. geometry + pc2_utils + 单元测试
3. servo_client + 单元测试（脏数据解析）
4. angle_logger + stitcher + 单元测试
5. fsm + tower_node 串联（mock 模式端到端）
6. launch + 参数文件 + README
7. 真机联调（macOS 串口 + 传感器数据）

---

## 12. 决策记录（与需求方确认）

| # | 问题 | 结论 |
|---|------|------|
| 1 | 扫描范围不一致（150 vs 120） | 扫到 150°，拼合 30°~150°（120 为笔误） |
| 2 | 拍照内容/时机 | color+depth；**复位位置 = 90°**（复位动作 = RST → 移到 90°），照片在 90°（30~150 中点）拍 |
| 3 | 传感器驱动 | 336L = Orbbec Gemini 336L（驱动参考 pallet_vision_lidar）；Fairy = rslidar_sdk 官方 ROS2 驱动 |
| 4 | 语言 | 混合：Python 为主体，性能热点（拼合）必要时后换 C++ |
| 5 | 服务交互 | 非阻塞 + status topic（不用 Action） |
| 6 | 扫描速度 | 40°/s（120° 约 3s） |
| 7 | 帧内畸变 | 采用**逐点时间补偿**（rslidar_sdk 每点 time + 100Hz 日志插值） |
| 8 | 开发/验证环境 | macOS 当前即接真实硬件：conda (RoboStack humble) 跑真实数据联调；生产 Linux Humble |
| 9 | 项目命名 | `perception_tower` |
| 10 | 节点架构 | 方案 A：单节点内聚 + 模块化 + 后台线程拼合（拼合数学移植自 turntable_stitcher） |
