# Turntable Control Migration to sensor_env

## Overview

Move turntable (STM32 stepper motor) control from `perception_tower` (Mac) to `perception_tower_sensor_env` (Ubuntu), so all hardware connections reside in one project. The two projects communicate via ROS2 topics and services over CycloneDDS.

## Architecture

```
┌─────────────────────────────────────────────┐
│  Ubuntu / sensor_env (Docker)               │
│                                             │
│  rslidar_sdk  →  /rslidar_points            │
│  orbbec_camera → /camera/*                  │
│  turntable_node (new)                       │
│    - serial → STM32                         │
│    - 100Hz internal polling                 │
│    - 50Hz /turntable/status publisher       │
│    - /turntable/command service             │
└─────────────────┬───────────────────────────┘
                  │ CycloneDDS
┌─────────────────┴───────────────────────────┐
│  Mac / perception_tower                     │
│                                             │
│  订阅 /turntable/status → AngleLogger       │
│  订阅 /rslidar_points → FairyBuffer        │
│  订阅 /camera/* → CameraGrabber            │
│  调用 /turntable/command                    │
│  FSM + stitching + 文件保存                 │
└─────────────────────────────────────────────┘
```

## sensor_env New: turntable_node

### Responsibilities
- Serial communication with STM32 (115200 8N1, `#...!` protocol)
- Internal polling loop at 100Hz for position reading
- ROS2 interface exposure for remote control

### ROS2 Interface

**Service: `/turntable/command`**

```srv
# TurntableCommand.srv
uint8 CMD_HOME = 1
uint8 CMD_MOVE = 2
uint8 CMD_STOP = 3

uint8 command
float32 target_deg      # target angle in degrees (used for CMD_MOVE)
float32 duration_s      # move duration in seconds (used for CMD_MOVE)
---
bool success
string message
```

**Publisher: `/turntable/status`**

```msg
# TurntableStatus.msg
float32 position        # raw position value (500-18000)
float32 angle_deg       # converted angle in degrees
uint8 state             # 0=IDLE, 1=HOMING, 2=MOVING, 3=ERROR
```

- Publish rate: **50 Hz**
- QoS: Best Effort, depth 10

### Parameters

```yaml
turntable_node:
  ros__parameters:
    serial_port: /dev/ttyUSB0        # adjust for Ubuntu
    serial_baud: 115200
    poll_hz: 100.0                   # internal polling rate
    pub_hz: 50.0                     # topic publish rate
    pos_origin: 500
    deg_per_pos: 0.02
    angle_sign: 1
    home_timeout_s: 30.0
```

### Files to migrate from perception_tower

| Source file | Destination | Notes |
|-------------|-------------|-------|
| `servo_client.py` (ServoClient, ProtocolParser) | sensor_env/turntable/ | Serial protocol parser + client |
| `mock.py` (FakeServo only) | sensor_env/turntable/ | Modified to publish topic + respond to service |

### Files to remove from perception_tower

- `servo_client.py` — entire file (serial control moved to sensor_env)

## perception_tower Changes

### angle_logger.py

- **Before**: Polls `ServoClient.read_position()` at 100Hz in a thread
- **After**: Subscribes to `/turntable/status` at 50Hz via ROS2 subscription
- **Keep**: `angles_at(ts)` interpolation interface unchanged
- **Time source**: Use ROS message header timestamp (from sensor_env)

### fsm.py

- **Before**: Directly calls `servo.reset()`, `servo.move_to()`, `servo.read_position()`
- **After**: Calls `/turntable/command` service for init/move/stop
- **Polling**: Uses `AngleLogger` (which subscribes to topic) to check position reached

### tower_node.py

- Remove serial parameters (`serial_port`, `serial_baud`, etc.)
- Add turntable topic/service name parameters
- Remove `ServoClient` instantiation from `_build_components()`
- Pass turntable command client to FSM

### mock.py

- `FakeServo` → Modified to publish `/turntable/status` topic and respond to `/turntable/command` service
- `MockFairy`, `MockCamera` — unchanged

### stitcher.py, pc2_utils.py, geometry.py, fairy_buffer.py, camera_grabber.py

- **No changes** — these modules are hardware-agnostic

## Time Synchronization

- LiDAR timestamps: `use_lidar_clock: true` (already configured)
- Turntable position timestamps: `turntable_node` uses `node.get_clock().now()` as ROS timestamp
- Camera timestamps: Orbbec SDK provides synchronized timestamps
- Mac-side `AngleLogger` uses ROS message timestamps for `angles_at(ts)` interpolation

## Parameters to Remove from perception_tower

```yaml
# These move to sensor_env/turntable_node parameters:
serial_port
serial_baud
pos_origin
deg_per_pos
angle_sign
home_timeout_s
```

## Parameters to Add to perception_tower

```yaml
turntable_cmd_service: /turntable/command
turntable_status_topic: /turntable/status
```

## Launch Changes

### sensor_env

Add `turntable_node` to the Docker container's launch sequence:
```bash
ros2 run perception_tower_sensor_env turntable_node --ros-args --params-file config/turntable_params.yaml
```

### perception_tower

No structural changes — just parameter updates. The tower launch file continues to work, but now relies on remote turntable node.
