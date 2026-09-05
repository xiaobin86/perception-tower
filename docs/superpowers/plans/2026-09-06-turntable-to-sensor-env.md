# Turntable Migration to sensor_env — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move turntable (STM32 stepper motor) serial control from `perception_tower` (Mac) to `perception_tower_sensor_env` (Ubuntu), exposing it via ROS2 topics/services. Add a one-click launch file in sensor_env to start LiDAR, camera, and turntable together.

**Architecture:** sensor_env becomes the hardware HAL — owns all serial/Ethernet/USB connections. perception_tower becomes pure business logic — subscribes to topics, calls services, runs FSM + stitching. Communication is via CycloneDDS.

**Tech Stack:** ROS2 Humble, Python 3, ament_cmake (interfaces), ament_python (turntable node), pyserial, CycloneDDS

**Spec:** `docs/superpowers/specs/2026-09-06-turntable-to-sensor-env-design.md`

## Global Constraints

- ROS2 Humble (conda RoboStack on Mac, Docker on Ubuntu)
- CycloneDDS for cross-machine communication
- `ROS_DOMAIN_ID=0`, `ROS_LOCALHOST_ONLY=0`
- Python 3.10+
- No open3d/PCL dependencies — PCD is hand-written
- Existing test framework: pytest

## File Structure

### sensor_env (new files)

| File | Responsibility |
|------|---------------|
| `perception_tower_sensor_interfaces/CMakeLists.txt` | ament_cmake package for msg/srv generation |
| `perception_tower_sensor_interfaces/package.xml` | Package manifest |
| `perception_tower_sensor_interfaces/msg/TurntableStatus.msg` | Position + angle + state |
| `perception_tower_sensor_interfaces/srv/TurntableCommand.srv` | home/move/stop commands |
| `perception_tower_sensor/turntable_node.py` | Serial protocol + ROS2 node |
| `perception_tower_sensor/package.xml` | ament_python package manifest |
| `perception_tower_sensor/setup.py` | Python package setup |
| `perception_tower_sensor/setup.cfg` | Entry points |
| `perception_tower_sensor/config/turntable_params.yaml` | Turntable parameters |
| `perception_tower_sensor/launch/sensor_env.launch.py` | One-click launch (LiDAR + camera + turntable) |

### perception_tower (modified files)

| File | Change |
|------|--------|
| `perception_tower/angle_logger.py` | Remove serial dependency; accept `record_cb` + `last_angle()` |
| `perception_tower/fsm.py` | Use turntable command fn + position reader instead of servo object |
| `perception_tower/tower_node.py` | Remove serial params; add turntable topic/service; wire new components |
| `perception_tower/mock.py` | FakeServo publishes /turntable/status + responds to /turntable/command |
| `perception_tower/config/tower_params.yaml` | Remove serial params; add turntable topic/service names |
| `perception_tower/test/test_angle_logger.py` | Update for new callback interface |
| `perception_tower/test/test_tower_node.py` | Remove serial_port override; adapt for new mock |

### perception_tower (deleted files)

| File | Reason |
|------|--------|
| `perception_tower/servo_client.py` | Serial protocol moved to sensor_env |

---

## Task 1: Create interface package (perception_tower_sensor_interfaces)

**Files:**
- Create: `perception_tower_sensor_interfaces/CMakeLists.txt`
- Create: `perception_tower_sensor_interfaces/package.xml`
- Create: `perception_tower_sensor_interfaces/msg/TurntableStatus.msg`
- Create: `perception_tower_sensor_interfaces/srv/TowerCommand.srv`

**Interfaces:**
- Produces: `TurntableStatus` message, `TurntableCommand` service (used by Tasks 2, 5, 6)

- [ ] **Step 1: Create msg/srv directories**

```bash
mkdir -p perception_tower_sensor_interfaces/msg perception_tower_sensor_interfaces/srv
```

- [ ] **Step 2: Write TurntableStatus.msg**

```msg
uint8 STATE_IDLE=0
uint8 STATE_HOMING=1
uint8 STATE_MOVING=2
uint8 STATE_ERROR=3
float32 position
float32 angle_deg
uint8 state
```

- [ ] **Step 3: Write TurntableCommand.srv**

```srv
uint8 CMD_HOME=1
uint8 CMD_MOVE=2
uint8 CMD_STOP=3
uint8 command
float32 target_deg
float32 duration_s
---
bool success
string message
```

- [ ] **Step 4: Write package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>perception_tower_sensor_interfaces</name>
  <version>0.1.0</version>
  <description>ROS2 message/service definitions for perception tower sensor environment</description>
  <maintainer email="dev@example.com">perception_tower</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <depend>std_msgs</depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 5: Write CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.8)
project(perception_tower_sensor_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TurntableStatus.msg"
  "srv/TurntableCommand.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

- [ ] **Step 6: Build and verify**

```bash
cd /Users/acelan/workspace/perception_tower_sensor_env
colcon build --packages-select perception_tower_sensor_interfaces
source install/setup.bash
ros2 interface show perception_tower_sensor_interfaces/msg/TurntableStatus
ros2 interface show perception_tower_sensor_interfaces/srv/TurntableCommand
```

Expected: msg/srv definitions displayed correctly.

- [ ] **Step 7: Commit**

```bash
git add perception_tower_sensor_interfaces/
git commit -m "feat(sensor_env): add TurntableStatus msg and TurntableCommand srv interfaces"
```

---

## Task 2: Create turntable ROS2 node (sensor_env)

**Files:**
- Create: `perception_tower_sensor/turntable_node.py`
- Create: `perception_tower_sensor/package.xml`
- Create: `perception_tower_sensor/setup.py`
- Create: `perception_tower_sensor/setup.cfg`
- Create: `perception_tower_sensor/__init__.py`
- Create: `perception_tower_sensor/config/turntable_params.yaml`

**Interfaces:**
- Consumes: `TurntableCommand.srv` (from Task 1)
- Produces: `/turntable/status` (TurntableStatus, 50Hz), `/turntable/command` (TurntableCommand service)

- [ ] **Step 1: Create package structure**

```bash
mkdir -p perception_tower_sensor/perception_tower_sensor
mkdir -p perception_tower_sensor/config
mkdir -p perception_tower_sensor/launch
touch perception_tower_sensor/perception_tower_sensor/__init__.py
```

- [ ] **Step 2: Write package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>perception_tower_sensor</name>
  <version>0.1.0</version>
  <description>Turntable control node for perception tower sensor environment</description>
  <maintainer email="dev@example.com">perception_tower</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <depend>rclpy</depend>
  <depend>perception_tower_sensor_interfaces</depend>
  <exec_depend>python3-serial</exec_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 3: Write setup.py**

```python
from setuptools import find_packages, setup

package_name = "perception_tower_sensor"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/turntable_params.yaml"]),
        ("share/" + package_name + "/launch", ["launch/sensor_env.launch.py"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="perception_tower",
    maintainer_email="dev@example.com",
    description="Turntable control node",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "turntable_node = perception_tower_sensor.turntable_node:main",
        ],
    },
)
```

- [ ] **Step 4: Write setup.cfg**

```cfg
[develop]
script_dir=$base/lib/perception_tower_sensor
[install]
install_scripts=$base/lib/perception_tower_sensor
```

- [ ] **Step 5: Create resource directory**

```bash
mkdir -p perception_tower_sensor/resource
touch perception_tower_sensor/resource/perception_tower_sensor
```

- [ ] **Step 6: Write turntable_params.yaml**

```yaml
turntable_node:
  ros__parameters:
    serial_port: /dev/ttyUSB0
    serial_baud: 115200
    poll_hz: 100.0
    pub_hz: 50.0
    pos_origin: 500
    deg_per_pos: 0.02
    angle_sign: 1
    home_timeout_s: 30.0
```

- [ ] **Step 7: Write turntable_node.py**

```python
"""Turntable control ROS2 node.

Serial protocol (115200 8N1, #...! delimiters):
    MOVE : #000P{pos}T{time_ms}!
    READ : #000PRAD!           -> #000P{pos}!
    STOP : #000PDST!           -> #OK!
    RST  : #000PRST!           -> #OK!

Publishes /turntable/status at 50 Hz.
Provides /turntable/command service.
"""

from __future__ import annotations

import queue
import re
import threading
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from perception_tower_sensor_interfaces.msg import TurntableStatus
from perception_tower_sensor_interfaces.srv import TurntableCommand


# --- Protocol parser ---

_OK_EVENT = ("ok",)
_POSITION_RE = re.compile(rb"^(\d{3})P(\d+)$")


class ProtocolParser:
    def __init__(self, servo_id: int = 0):
        self._id = servo_id
        self._buf = bytearray()
        self._id_bytes = f"{servo_id:03d}".encode()

    def feed(self, data: bytes) -> List[tuple]:
        self._buf.extend(data)
        events: List[tuple] = []
        while True:
            start = self._buf.find(b"#")
            if start < 0:
                self._buf.clear()
                break
            if start > 0:
                del self._buf[:start]
            end = self._buf.find(b"!")
            if end < 0:
                break
            chunk = bytes(self._buf[1:end])
            del self._buf[: end + 1]
            if chunk == b"OK":
                events.append(_OK_EVENT)
            else:
                m = _POSITION_RE.match(chunk)
                if m and m.group(1) == self._id_bytes:
                    events.append(("pos", int(m.group(2))))
        return events


# --- Servo errors ---

class ServoError(RuntimeError):
    pass


# --- Serial client ---

class ServoClient:
    def __init__(self, port: str, baud: int = 115200, servo_id: int = 0,
                 pos_origin: int = 500, deg_per_pos: float = 0.02):
        self._port = port
        self._baud = baud
        self._servo_id = servo_id
        self._origin = pos_origin
        self._dpp = deg_per_pos
        self._ser = None
        self._parser = ProtocolParser(servo_id)
        self._reply_q: "queue.Queue[tuple]" = queue.SimpleQueue()
        self._write_lock = threading.Lock()
        self._reader_thread: Optional[threading.Thread] = None
        self._running = False

    def open(self):
        import serial
        if not self._port:
            raise ServoError("serial_port not configured")
        self._ser = serial.Serial(self._port, self._baud, timeout=0.05)
        self._running = True
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()

    def close(self):
        self._running = False
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=0.5)

    def _read_loop(self):
        while self._running:
            try:
                data = self._ser.read(256)
            except Exception:
                break
            if data:
                events = self._parser.feed(data)
                for ev in events:
                    self._reply_q.put(ev)

    def _send(self, payload: bytes):
        with self._write_lock:
            if self._ser is None:
                raise ServoError("serial not open")
            self._ser.write(payload)

    def _wait_event(self, kinds: tuple, timeout_s: float):
        deadline = time.monotonic() + timeout_s
        while True:
            remain = deadline - time.monotonic()
            if remain <= 0:
                raise ServoError(f"timeout waiting for {kinds}")
            try:
                ev = self._reply_q.get(timeout=min(remain, 0.1))
            except queue.Empty:
                continue
            if ev[0] in kinds:
                return ev

    def _flush_replies(self):
        while True:
            try:
                self._reply_q.get_nowait()
            except queue.Empty:
                break

    def move_to(self, pos: int, time_ms: int):
        self._send(f"#{self._servo_id:03d}P{pos}T{time_ms}!".encode())

    def stop(self):
        self._send(f"#{self._servo_id:03d}PDST!".encode())
        self._wait_event(("ok",), 0.5)

    def read_position(self, timeout_s: float = 0.2) -> int:
        self._flush_replies()
        self._send(f"#{self._servo_id:03d}PRAD!".encode())
        ev = self._wait_event(("pos",), timeout_s)
        return int(ev[1])

    def reset(self, timeout_s: float = 30.0):
        self._send(f"#{self._servo_id:03d}PRST!".encode())
        self._wait_event(("ok",), timeout_s)

    def pos_to_deg(self, pos: int) -> float:
        return (pos - self._origin) * self._dpp

    def deg_to_pos(self, deg: float) -> int:
        return int(round(self._origin + deg / self._dpp))


# --- ROS2 node ---

class TurntableNode(Node):
    def __init__(self):
        super().__init__("turntable_node")
        self._declare_params()
        self._load_params()

        self._servo = ServoClient(
            port=self._port,
            baud=self._baud,
            pos_origin=self._origin,
            deg_per_pos=self._dpp,
        )
        try:
            self._servo.open()
            self.get_logger().info(f"serial opened: {self._port}")
        except Exception as exc:
            self.get_logger().error(f"failed to open serial: {exc}")
            raise

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._status_pub = self.create_publisher(TurntableStatus, "/turntable/status", qos)
        self._srv = self.create_service(TurntableCommand, "/turntable/command", self._on_command)

        self._state = TurntableStatus.STATE_IDLE
        self._lock = threading.Lock()

        period = 1.0 / self._pub_hz
        self._pub_timer = self.create_timer(period, self._publish_status)

        self._poll_period = 1.0 / self._poll_hz
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()

    def _declare_params(self):
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("serial_baud", 115200)
        self.declare_parameter("poll_hz", 100.0)
        self.declare_parameter("pub_hz", 50.0)
        self.declare_parameter("pos_origin", 500)
        self.declare_parameter("deg_per_pos", 0.02)
        self.declare_parameter("angle_sign", 1)
        self.declare_parameter("home_timeout_s", 30.0)

    def _load_params(self):
        self._port = self.get_parameter("serial_port").value
        self._baud = self.get_parameter("serial_baud").value
        self._poll_hz = self.get_parameter("poll_hz").value
        self._pub_hz = self.get_parameter("pub_hz").value
        self._origin = self.get_parameter("pos_origin").value
        self._dpp = self.get_parameter("deg_per_pos").value
        self._angle_sign = self.get_parameter("angle_sign").value
        self._home_timeout = self.get_parameter("home_timeout_s").value

    def _poll_loop(self):
        while rclpy.ok():
            t0 = time.monotonic()
            try:
                pos = self._servo.read_position(timeout_s=self._poll_period * 2.0)
                with self._lock:
                    self._last_pos = pos
            except Exception:
                pass
            elapsed = time.monotonic() - t0
            if elapsed < self._poll_period:
                time.sleep(self._poll_period - elapsed)

    def _publish_status(self):
        msg = TurntableStatus()
        with self._lock:
            pos = getattr(self, "_last_pos", self._origin)
        msg.position = float(pos)
        msg.angle_deg = self._servo.pos_to_deg(pos)
        msg.state = self._state
        self._status_pub.publish(msg)

    def _on_command(self, request, response):
        cmd = request.command
        if cmd == TurntableCommand.Request.CMD_HOME:
            self._state = TurntableStatus.STATE_HOMING
            try:
                self._servo.reset(timeout_s=self._home_timeout)
                target = self._servo.deg_to_pos(request.target_deg if request.target_deg else 90.0)
                time_ms = max(200, int(abs(request.target_deg if request.target_deg else 90.0) / 40.0 * 1000))
                self._servo.move_to(target, time_ms)
                self._state = TurntableStatus.STATE_IDLE
                response.success = True
                response.message = "homed"
            except Exception as exc:
                self._state = TurntableStatus.STATE_ERROR
                response.success = False
                response.message = f"home failed: {exc}"

        elif cmd == TurntableCommand.Request.CMD_MOVE:
            self._state = TurntableStatus.STATE_MOVING
            try:
                pos = self._servo.deg_to_pos(request.target_deg)
                time_ms = max(200, int(request.duration_s * 1000)) if request.duration_s > 0 else 2000
                self._servo.move_to(pos, time_ms)
                response.success = True
                response.message = f"moving to {request.target_deg:.1f} deg"
            except Exception as exc:
                self._state = TurntableStatus.STATE_ERROR
                response.success = False
                response.message = f"move failed: {exc}"

        elif cmd == TurntableCommand.Request.CMD_STOP:
            try:
                self._servo.stop()
                self._state = TurntableStatus.STATE_IDLE
                response.success = True
                response.message = "stopped"
            except Exception as exc:
                self._state = TurntableStatus.STATE_ERROR
                response.success = False
                response.message = f"stop failed: {exc}"

        else:
            response.success = False
            response.message = f"unknown command {cmd}"

        return response

    def destroy_node(self):
        try:
            self._servo.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TurntableNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 8: Build and verify node starts**

```bash
cd /Users/acelan/workspace/perception_tower_sensor_env
colcon build --packages-select perception_tower_sensor_interfaces perception_tower_sensor
source install/setup.bash
ros2 run perception_tower_sensor turntable_node --ros-args -p serial_port:=/dev/null
```

Note: will fail to open serial on Mac, but node structure should load. Verify with `ros2 node list` and `ros2 service list` before it errors.

- [ ] **Step 9: Commit**

```bash
git add perception_tower_sensor/
git commit -m "feat(sensor_env): add turntable_node with serial protocol + ROS2 interface"
```

---

## Task 3: Modify angle_logger.py — callback-based recording

**Files:**
- Modify: `perception_tower/perception_tower/angle_logger.py`
- Modify: `perception_tower/test/test_angle_logger.py`

**Interfaces:**
- Consumes: (none — standalone module)
- Produces: `AngleLogger` class with `record_sample(ts, angle_deg)`, `last_angle()`, `start()`, `stop()`, `angles_at(ts)`, `coverage()`

- [ ] **Step 1: Rewrite angle_logger.py**

Replace the entire file with:

```python
"""Turntable angle logger with time-based interpolation.

Records (timestamp, angle_deg) samples fed via record_sample(),
then provides linear interpolation via angles_at(). Out-of-range
queries are clamped to the nearest endpoint.

Kept free of ROS imports: the caller feeds samples.
"""

from __future__ import annotations

import threading
from typing import List, Optional, Tuple

import numpy as np


class AngleLogger:
    def __init__(self):
        self._samples: List[Tuple[float, float]] = []
        self._lock = threading.Lock()
        self.error: Optional[Exception] = None

    def start(self):
        with self._lock:
            self._samples = []
        self.error = None

    def stop(self):
        pass

    def record_sample(self, ts: float, angle_deg: float):
        with self._lock:
            self._samples.append((ts, angle_deg))

    def last_angle(self) -> Optional[float]:
        with self._lock:
            if not self._samples:
                return None
            return self._samples[-1][1]

    def _get_samples(self) -> List[Tuple[float, float]]:
        with self._lock:
            return list(self._samples)

    def coverage(self) -> Optional[Tuple[float, float]]:
        samples = self._get_samples()
        if len(samples) < 2:
            return None
        return (samples[0][0], samples[-1][0])

    def angles_at(self, ts: np.ndarray) -> np.ndarray:
        samples = self._get_samples()
        if len(samples) < 2:
            if len(samples) == 1:
                return np.full(np.asarray(ts).shape, samples[0][1], dtype=np.float64)
            return np.zeros(np.asarray(ts).shape, dtype=np.float64)
        arr = np.asarray(samples, dtype=np.float64)
        return np.interp(np.asarray(ts, dtype=np.float64), arr[:, 0], arr[:, 1],
                         left=arr[0, 1], right=arr[-1, 1])
```

- [ ] **Step 2: Update test_angle_logger.py**

Replace the entire file with:

```python
import numpy as np
import pytest
from perception_tower.angle_logger import AngleLogger


def test_interpolation_and_clamping():
    logger = AngleLogger()
    logger.start()
    logger.record_sample(0.0, 0.0)
    logger.record_sample(0.01, 10.0)
    logger.record_sample(0.02, 20.0)
    logger.record_sample(0.03, 30.0)
    logger.stop()

    ts = np.array([-0.1, 0.015, 0.025, 0.05])
    out = logger.angles_at(ts)
    assert out[0] == 0.0
    assert out[-1] == 30.0
    assert np.isclose(out[1], 15.0)
    assert np.isclose(out[2], 25.0)


def test_last_angle():
    logger = AngleLogger()
    logger.start()
    assert logger.last_angle() is None
    logger.record_sample(0.0, 45.0)
    assert logger.last_angle() == 45.0
    logger.record_sample(0.01, 90.0)
    assert logger.last_angle() == 90.0
    logger.stop()


def test_coverage():
    logger = AngleLogger()
    logger.start()
    assert logger.coverage() is None
    logger.record_sample(1.0, 0.0)
    assert logger.coverage() is None
    logger.record_sample(2.0, 90.0)
    cov = logger.coverage()
    assert cov == (1.0, 2.0)
    logger.stop()


def test_empty_interpolation():
    logger = AngleLogger()
    logger.start()
    ts = np.array([0.0, 1.0])
    out = logger.angles_at(ts)
    assert np.all(out == 0.0)
    logger.stop()
```

- [ ] **Step 3: Run tests**

```bash
cd /Users/acelan/workspace/perception_tower
pytest perception_tower/test/test_angle_logger.py -v
```

Expected: all tests PASS.

- [ ] **Step 4: Commit**

```bash
git add perception_tower/perception_tower/angle_logger.py perception_tower/test/test_angle_logger.py
git commit -m "refactor: angle_logger uses callback-based recording, removes serial dependency"
```

---

## Task 4: Modify fsm.py — use turntable command function

**Files:**
- Modify: `perception_tower/perception_tower/fsm.py`

**Interfaces:**
- Consumes: `turntable_cmd(command, target_deg, duration_s)` fn, `read_position()` fn, `pos_to_deg(pos)` fn, `deg_to_pos(deg)` fn
- Produces: `TowerFSM` class (same public API: `request_init()`, `request_scan()`, `state`)

- [ ] **Step 1: Rewrite fsm.py**

Replace the entire file with:

```python
"""Tower state machine (INIT / SCAN).

Runs the turntable + angle-logger + Fairy buffer + camera pipeline.
Free of rclpy so it can be exercised by plain pytest with mock hardware.

The turntable is controlled via a command callback:
    turntable_cmd(command, target_deg, duration_s) -> (success, message)

Position is read via read_position() -> int (raw pos value).
"""

from __future__ import annotations

import csv
import os
import threading
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Callable, Optional, Tuple

import numpy as np

from .camera_grabber import PhotoPair
from .stitcher import StitchParams, stitch


class State(IntEnum):
    IDLE = 0
    INITING = 1
    READY = 2
    SCANNING = 3
    PROCESSING = 4
    ERROR = 5


@dataclass
class SaveConfig:
    output_dir: str
    save_cloud: bool = True


class TowerFSM:
    def __init__(
        self,
        turntable_cmd: Callable[[int, float, float], Tuple[bool, str]],
        read_position: Callable[[], int],
        pos_to_deg: Callable[[int], float],
        deg_to_pos: Callable[[float], int],
        camera,
        fairy_buffer,
        stitch_params: StitchParams,
        save_cfg: dict,
        status_cb: Callable[["State", int, str], None],
        photo_cb: Callable[[PhotoPair], None],
        cloud_cb: Callable[[np.ndarray, Optional[np.ndarray], float], None],
        clock_now: Callable[[], float],
        log_cb: Callable[[str], None] = print,
        config: Optional[dict] = None,
    ):
        self._cmd = turntable_cmd
        self._read_pos = read_position
        self._pos_to_deg = pos_to_deg
        self._deg_to_pos = deg_to_pos
        self._camera = camera
        self._fairy_buffer = fairy_buffer
        self._stitch_params = stitch_params
        self._save_cfg = SaveConfig(**save_cfg)
        self._status_cb = status_cb
        self._photo_cb = photo_cb
        self._cloud_cb = cloud_cb
        self._clock = clock_now
        self._log = log_cb
        self._cfg = config or {}
        self._state = State.IDLE
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None

    @property
    def state(self) -> "State":
        return self._state

    def _set_state(self, state: "State", progress: int = 0, message: str = ""):
        with self._lock:
            self._state = state
            self._status_cb(state, progress, message)

    def _busy_states(self):
        return {State.INITING, State.SCANNING, State.PROCESSING}

    def request_init(self) -> tuple:
        with self._lock:
            if self._state in self._busy_states():
                return False, f"busy: {self._state.name}"
            self._state = State.INITING
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._thread = threading.Thread(target=self._run_init, daemon=True)
        self._thread.start()
        return True, "init started"

    def request_scan(self) -> tuple:
        with self._lock:
            if self._state in self._busy_states():
                return False, f"busy: {self._state.name}"
            self._state = State.SCANNING
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._thread = threading.Thread(target=self._run_scan, daemon=True)
        self._thread.start()
        return True, "scan started"

    def _move_to_deg(self, deg: float, speed_deg_s: float,
                     progress_base: int, progress_span: int, label: str):
        pos = self._deg_to_pos(deg)
        current_pos = self._read_pos()
        current_deg = self._pos_to_deg(current_pos)
        delta = abs(deg - current_deg)
        time_s = max(0.2, delta / speed_deg_s)
        self._cmd(2, deg, time_s)  # CMD_MOVE = 2

        tol_deg = self._cfg.get("pos_tol_deg", 0.1)
        stable_count = self._cfg.get("pos_stable_count", 5)
        timeout_s = self._cfg.get("move_timeout_s", 30.0)
        poll_hz = self._cfg.get("poll_hz", 100.0)
        period = 1.0 / poll_hz

        start_deg = current_deg
        count = 0
        start_t = time.monotonic()
        n = 0
        while True:
            t0 = time.monotonic()
            if t0 - start_t > timeout_s:
                raise RuntimeError("position timeout")
            pos = self._read_pos()
            d = self._pos_to_deg(pos)
            if abs(d - deg) <= tol_deg:
                count += 1
            else:
                count = 0
            n += 1
            if n % 10 == 0:
                total = abs(deg - start_deg)
                remain = max(0.0, abs(deg - d))
                pct = 0.0 if total <= 0 else 1.0 - remain / total
                self._set_state(self._state, int(progress_base + min(0.999, pct) * progress_span), label)
            if count >= stable_count:
                self._set_state(self._state, int(progress_base + progress_span), label)
                return
            elapsed = time.monotonic() - t0
            if elapsed < period:
                time.sleep(period - elapsed)

    def _run_init(self):
        try:
            self._set_state(State.INITING, 0, "homing")
            self._cmd(1, 0.0, 0.0)  # CMD_HOME = 1
            time.sleep(0.5)
            self._set_state(State.INITING, 50, "moving to ready")
            self._move_to_deg(
                self._cfg.get("ready_deg", 90.0),
                self._cfg.get("sweep_speed_deg_s", 40.0),
                50, 50, "moving to ready",
            )
            self._set_state(State.READY, 100, "ready")
        except Exception as exc:
            self._set_state(State.ERROR, 0, f"init failed: {exc}")

    def _ensure_ready(self):
        pos = self._read_pos()
        deg = self._pos_to_deg(pos)
        if abs(deg - self._cfg.get("ready_deg", 90.0)) > self._cfg.get("pos_tol_deg", 0.1):
            self._set_state(State.SCANNING, 0, "re-homing")
            self._cmd(1, 0.0, 0.0)  # CMD_HOME
            time.sleep(0.5)
            self._move_to_deg(
                self._cfg.get("ready_deg", 90.0),
                self._cfg.get("sweep_speed_deg_s", 40.0),
                0, 10, "re-homing",
            )

    def _run_scan(self):
        try:
            self._set_state(State.SCANNING, 0, "scan start")
            self._ensure_ready()
            self._set_state(State.SCANNING, 10, "capture photo")
            out_dir = os.path.join(self._save_cfg.output_dir, time.strftime("%Y%m%d_%H%M%S"))
            os.makedirs(out_dir, exist_ok=True)
            try:
                pair = self._camera.capture(timeout_s=self._cfg.get("photo_timeout_s", 5.0))
                from .camera_grabber import save_photos
                save_photos(pair, out_dir)
                self._photo_cb(pair)
            except RuntimeError:
                self._log("camera not available, skipping photo")

            self._set_state(State.SCANNING, 20, "move to scan start")
            from .angle_logger import AngleLogger
            logger = AngleLogger()
            self._fairy_buffer.start()
            logger.start()
            self._move_to_deg(
                self._cfg.get("scan_start_deg", 30.0),
                self._cfg.get("sweep_speed_deg_s", 40.0),
                20, 10, "move to start",
            )

            self._set_state(State.SCANNING, 30, "sweep")
            self._move_to_deg(
                self._cfg.get("scan_end_deg", 150.0),
                self._cfg.get("sweep_speed_deg_s", 40.0),
                30, 40, "sweeping",
            )
            time.sleep(self._cfg.get("move_settle_s", 0.2))
            logger.stop()
            self._fairy_buffer.stop()

            frames = self._fairy_buffer.frames()
            if not frames:
                raise RuntimeError("no fairy frames captured")
            cov = logger.coverage()
            expected = (self._cfg.get("scan_end_deg", 150.0) - self._cfg.get("scan_start_deg", 30.0)) / self._cfg.get("sweep_speed_deg_s", 40.0)
            if cov is None or (cov[1] - cov[0]) < expected * 0.5:
                raise RuntimeError("insufficient angle log coverage")

            with open(os.path.join(out_dir, "angle_log.csv"), "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t", "theta_raw_deg"])
                for t, deg in logger._get_samples():
                    w.writerow([f"{t:.6f}", f"{deg:.6f}"])

            self._set_state(State.PROCESSING, 70, "stitching")
            result = stitch(frames, logger.angles_at, self._stitch_params)
            stamp = self._clock()
            self._cloud_cb(result.xyz, result.intensity, stamp)
            if self._save_cfg.save_cloud:
                from .pc2_utils import save_pcd_binary
                save_pcd_binary(os.path.join(out_dir, "stitched.pcd"), result.xyz, result.intensity)

            self._set_state(State.PROCESSING, 95, "return to ready")
            self._move_to_deg(
                self._cfg.get("ready_deg", 90.0),
                self._cfg.get("sweep_speed_deg_s", 40.0),
                95, 5, "return to ready",
            )
            self._set_state(State.READY, 100, f"scan done: {out_dir}")
        except Exception as exc:
            self._set_state(State.ERROR, 0, f"scan failed: {exc}")
```

- [ ] **Step 2: Verify import structure**

```bash
cd /Users/acelan/workspace/perception_tower
python -c "from perception_tower.fsm import TowerFSM, State; print('OK')"
```

Expected: OK

- [ ] **Step 3: Commit**

```bash
git add perception_tower/perception_tower/fsm.py
git commit -m "refactor: fsm uses command callback + position reader instead of servo object"
```

---

## Task 5: Modify tower_node.py — wire new components

**Files:**
- Modify: `perception_tower/perception_tower/tower_node.py`

**Interfaces:**
- Consumes: `TurntableCommand.srv` (service client), `TurntableStatus.msg` (subscriber)
- Produces: `TowerNode` (same ROS2 node interface)

- [ ] **Step 1: Rewrite tower_node.py**

Replace the entire file with:

```python
"""perception_tower ROS2 node.

Wires the TowerFSM to ROS interfaces:
  * service  /perception_tower/command  (TowerCommand)
  * status   /perception_tower/status   (TowerStatus, reliable+transient_local)
  * topics   stitched / photo_color / photo_depth

Subscribes to /turntable/status (from sensor_env turntable_node)
and /turntable/command service for turntable control.
"""

from __future__ import annotations

import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from .angle_logger import AngleLogger
from .camera_grabber import CameraGrabber, PhotoPair
from .fairy_buffer import FairyBuffer
from .fsm import State, TowerFSM
from .pc2_utils import make_cloud_msg
from .stitcher import StitchParams

try:
    from builtin_interfaces.msg import Time
    from sensor_msgs.msg import Image, PointCloud2
    from perception_tower_interfaces.msg import TowerStatus
    from perception_tower_interfaces.srv import TowerCommand
    from perception_tower_sensor_interfaces.msg import TurntableStatus
    from perception_tower_sensor_interfaces.srv import TurntableCommand

    _ROS_AVAILABLE = True
except Exception:  # pragma: no cover
    Time = Image = PointCloud2 = TowerStatus = TowerCommand = None
    TurntableStatus = TurntableCommand = None
    _ROS_AVAILABLE = False


class TowerNode(Node):
    def __init__(self, **kwargs):
        super().__init__("tower_node", **kwargs)
        self._declare_params()
        self._load_params()

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(TowerStatus, "/perception_tower/status", status_qos)
        self._cloud_pub = self.create_publisher(PointCloud2, self._stitched_topic, 10)
        self._photo_color_pub = self.create_publisher(Image, self._photo_color_topic, 10)
        self._photo_depth_pub = self.create_publisher(Image, self._photo_depth_topic, 10)

        self._srv = self.create_service(TowerCommand, "/perception_tower/command", self._on_command)
        self._status_timer = self.create_timer(0.5, self._publish_status)
        self._last_status = (State.IDLE, 0, "initialized")

        self._angle_logger = AngleLogger()
        self._angle_logger.start()

        self._build_components()

    def _declare_params(self):
        p = [
            ("turntable_cmd_service", "/turntable/command"),
            ("turntable_status_topic", "/turntable/status"),
            ("pos_tol_deg", 0.1),
            ("pos_stable_count", 5),
            ("pos_origin", 500),
            ("deg_per_pos", 0.02),
            ("angle_sign", 1),
            ("ready_deg", 90.0),
            ("scan_start_deg", 30.0),
            ("scan_end_deg", 150.0),
            ("sweep_speed_deg_s", 40.0),
            ("home_timeout_s", 30.0),
            ("fairy_topic", "/rslidar_points"),
            ("fairy_time_field", True),
            ("mount_rpy_deg", [90.0, 0.0, 0.0]),
            ("mount_offset_xyz", [0.0, 0.0, 0.0]),
            ("voxel_leaf_m", 0.01),
            ("world_frame_id", "world"),
            ("color_topic", "/camera/color/image_raw"),
            ("depth_topic", "/camera/depth/image_raw"),
            ("output_dir", "/tmp/perception_tower"),
            ("save_cloud", True),
            ("stitched_topic", "/perception_tower/stitched_points"),
            ("photo_color_topic", "/perception_tower/photo_color"),
            ("photo_depth_topic", "/perception_tower/photo_depth"),
            ("mock_hardware", False),
            ("photo_timeout_s", 5.0),
            ("move_settle_s", 0.2),
            ("move_timeout_s", 30.0),
            ("poll_hz", 100.0),
        ]
        for name, value in p:
            self.declare_parameter(name, value)

    def _load_params(self):
        self._stitched_topic = self.get_parameter("stitched_topic").value
        self._photo_color_topic = self.get_parameter("photo_color_topic").value
        self._photo_depth_topic = self.get_parameter("photo_depth_topic").value
        self._mock = self.get_parameter("mock_hardware").value
        self._world_frame_id = self.get_parameter("world_frame_id").value
        self._output_dir = self.get_parameter("output_dir").value
        self._origin = self.get_parameter("pos_origin").value
        self._dpp = self.get_parameter("deg_per_pos").value

    def _build_components(self):
        cfg = {
            "pos_tol_deg": self.get_parameter("pos_tol_deg").value,
            "pos_stable_count": self.get_parameter("pos_stable_count").value,
            "poll_hz": self.get_parameter("poll_hz").value,
            "ready_deg": self.get_parameter("ready_deg").value,
            "scan_start_deg": self.get_parameter("scan_start_deg").value,
            "scan_end_deg": self.get_parameter("scan_end_deg").value,
            "sweep_speed_deg_s": self.get_parameter("sweep_speed_deg_s").value,
            "home_timeout_s": self.get_parameter("home_timeout_s").value,
            "photo_timeout_s": self.get_parameter("photo_timeout_s").value,
            "move_settle_s": self.get_parameter("move_settle_s").value,
            "move_timeout_s": self.get_parameter("move_timeout_s").value,
        }

        if self._mock:
            from .mock import FakeServo, MockTurntableService
            servo = FakeServo(origin=self._origin, deg_per_pos=self._dpp,
                              speed_deg_s=cfg["sweep_speed_deg_s"])
            self._mock_tt = MockTurntableService(self, servo, self._origin, self._dpp)
            turntable_cmd = self._mock_tt.command_fn
            read_position = self._mock_tt.read_position
        else:
            turntable_cmd = self._create_turntable_client()
            read_position = lambda: self._angle_logger.last_angle()
            if read_position() is None:
                read_position = lambda: self._origin

        pos_to_deg = lambda pos: (pos - self._origin) * self._dpp
        deg_to_pos = lambda deg: int(round(self._origin + deg / self._dpp))

        camera = CameraGrabber(now_fn=lambda: self.get_clock().now().nanoseconds * 1e-9)
        self.create_subscription(
            Image, self.get_parameter("color_topic").value, camera.on_color, 10)
        self.create_subscription(
            Image, self.get_parameter("depth_topic").value, camera.on_depth, 10)
        if self._mock:
            from .mock import MockCamera
            MockCamera(self, self.get_parameter("color_topic").value,
                       self.get_parameter("depth_topic").value)

        fairy_buffer = FairyBuffer(use_time_field=self.get_parameter("fairy_time_field").value)
        self.create_subscription(
            PointCloud2, self.get_parameter("fairy_topic").value,
            lambda msg: fairy_buffer.on_cloud(msg, self.get_clock().now().nanoseconds * 1e-9), 10)
        if self._mock:
            from .mock import MockFairy
            MockFairy(self, self.get_parameter("fairy_topic").value, servo)

        if not self._mock:
            status_topic = self.get_parameter("turntable_status_topic").value
            self.create_subscription(
                TurntableStatus, status_topic,
                self._on_turntable_status, 10)

        stitch_params = StitchParams(
            mount_rpy_deg=self.get_parameter("mount_rpy_deg").value,
            mount_offset_xyz=self.get_parameter("mount_offset_xyz").value,
            scan_start_deg=cfg["scan_start_deg"],
            scan_end_deg=cfg["scan_end_deg"],
            voxel_leaf_m=self.get_parameter("voxel_leaf_m").value,
            per_point_time=self.get_parameter("fairy_time_field").value,
            angle_sign=self.get_parameter("angle_sign").value,
        )

        self._fsm = TowerFSM(
            turntable_cmd=turntable_cmd,
            read_position=read_position,
            pos_to_deg=pos_to_deg,
            deg_to_pos=deg_to_pos,
            camera=camera,
            fairy_buffer=fairy_buffer,
            stitch_params=stitch_params,
            save_cfg={"output_dir": self._output_dir, "save_cloud": self.get_parameter("save_cloud").value},
            status_cb=self._on_fsm_status,
            photo_cb=self._on_photo,
            cloud_cb=self._on_cloud,
            clock_now=lambda: self.get_clock().now().nanoseconds * 1e-9,
            log_cb=lambda m: self.get_logger().info(m),
            config=cfg,
        )

    def _on_turntable_status(self, msg: "TurntableStatus"):
        ts = self.get_clock().now().nanoseconds * 1e-9
        self._angle_logger.record_sample(ts, msg.angle_deg)

    def _create_turntable_client(self):
        svc_name = self.get_parameter("turntable_cmd_service").value
        client = self.create_client(TurntableCommand, svc_name)
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(f"turntable service not available: {svc_name}")

        def turntable_cmd(command, target_deg, duration_s):
            req = TurntableCommand.Request()
            req.command = command
            req.target_deg = target_deg
            req.duration_s = duration_s
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            result = future.result()
            return result.success, result.message

        return turntable_cmd

    def _on_fsm_status(self, state: "State", progress: int, message: str):
        self._last_status = (state, progress, message)
        self._publish_status()

    def _publish_status(self):
        state, progress, message = self._last_status
        msg = TowerStatus()
        msg.state = int(state)
        msg.progress_pct = progress
        msg.message = message
        self._status_pub.publish(msg)

    def _on_photo(self, pair: "PhotoPair"):
        self._photo_color_pub.publish(pair.color)
        self._photo_depth_pub.publish(pair.depth)

    def _on_cloud(self, xyz, intensity, stamp_sec: float):
        stamp = Time(sec=int(stamp_sec), nanosec=int((stamp_sec % 1) * 1e9))
        msg = make_cloud_msg(xyz, intensity, self._world_frame_id, stamp)
        self._cloud_pub.publish(msg)

    def _on_command(self, request, response):
        if request.command == TowerCommand.Request.CMD_INIT:
            accepted, message = self._fsm.request_init()
        elif request.command == TowerCommand.Request.CMD_SCAN:
            accepted, message = self._fsm.request_scan()
        else:
            accepted, message = False, f"unknown command {request.command}"
        response.accepted = accepted
        response.message = message
        return response

    def destroy_node(self):
        try:
            self._angle_logger.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Verify import structure**

```bash
cd /Users/acelan/workspace/perception_tower
python -c "from perception_tower.tower_node import TowerNode; print('OK')"
```

Expected: OK

- [ ] **Step 3: Commit**

```bash
git add perception_tower/perception_tower/tower_node.py
git commit -m "refactor: tower_node subscribes to turntable topic, uses service client"
```

---

## Task 6: Modify mock.py — FakeServo publishes topic + responds to service

**Files:**
- Modify: `perception_tower/perception_tower/mock.py`

**Interfaces:**
- Produces: `FakeServo` (same interface), `MockTurntableService` (new: publishes /turntable/status + responds to /turntable/command)

- [ ] **Step 1: Rewrite mock.py**

Replace the entire file with:

```python
"""Mock hardware for mock_hardware:=true and offline tests.

FakeServo simulates the turntable firmware with a linear speed model.
MockTurntableService wraps FakeServo as a ROS2 service + publisher.
MockFairy / MockCamera emit synthetic sensor data.
"""

from __future__ import annotations

import threading
import time

import numpy as np

from .geometry import mount_rotation


class FakeServo:
    def __init__(self, origin=500, deg_per_pos=0.02, speed_deg_s=40.0):
        self._origin = origin
        self._dpp = deg_per_pos
        self._speed = speed_deg_s
        self._target_deg = 0.0
        self._start_deg = 0.0
        self._start_t = time.monotonic()
        self._duration = 0.0
        self._lock = threading.RLock()

    def _current_deg(self) -> float:
        with self._lock:
            if self._duration <= 0:
                return self._target_deg
            elapsed = time.monotonic() - self._start_t
            p = min(1.0, elapsed / self._duration)
            return self._start_deg + (self._target_deg - self._start_deg) * p

    def open(self):
        pass

    def close(self):
        pass

    def move_to(self, pos: int, time_ms: int):
        with self._lock:
            self._start_deg = self._current_deg()
            self._target_deg = (pos - self._origin) * self._dpp
            self._start_t = time.monotonic()
            self._duration = time_ms / 1000.0

    def stop(self):
        with self._lock:
            self._start_deg = self._current_deg()
            self._target_deg = self._start_deg
            self._duration = 0.0

    def reset(self, timeout_s: float = 30.0):
        with self._lock:
            self._target_deg = 0.0
            self._start_deg = 0.0
            self._duration = 0.0

    def read_position(self, timeout_s: float = 0.2) -> int:
        return int(round(self._origin + self._current_deg() / self._dpp))

    def pos_to_deg(self, pos: int) -> float:
        return (pos - self._origin) * self._dpp

    def deg_to_pos(self, deg: float) -> int:
        return int(round(self._origin + deg / self._dpp))


try:
    from builtin_interfaces.msg import Time
    from sensor_msgs.msg import Image, PointCloud2
    from perception_tower_sensor_interfaces.msg import TurntableStatus
    from perception_tower_sensor_interfaces.srv import TurntableCommand
    _ROS_AVAILABLE = True
except Exception:  # pragma: no cover
    _ROS_AVAILABLE = False


if _ROS_AVAILABLE:

    class MockTurntableService:
        def __init__(self, node, servo, origin, deg_per_pos):
            self._node = node
            self._servo = servo
            self._origin = origin
            self._dpp = deg_per_pos

            from rclpy.qos import QoSProfile, ReliabilityPolicy
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self._pub = node.create_publisher(TurntableStatus, "/turntable/status", qos)
            self._srv = node.create_service(TurntableCommand, "/turntable/command", self._on_command)
            self._timer = node.create_timer(0.02, self._publish_status)  # 50 Hz

        def _publish_status(self):
            msg = TurntableStatus()
            pos = self._servo.read_position()
            msg.position = float(pos)
            msg.angle_deg = self._servo.pos_to_deg(pos)
            msg.state = TurntableStatus.STATE_IDLE
            self._pub.publish(msg)

        def _on_command(self, request, response):
            cmd = request.command
            if cmd == TurntableCommand.Request.CMD_HOME:
                self._servo.reset()
                target = self._servo.deg_to_pos(request.target_deg if request.target_deg else 90.0)
                time_ms = max(200, int(abs(request.target_deg if request.target_deg else 90.0) / 40.0 * 1000))
                self._servo.move_to(target, time_ms)
                response.success = True
                response.message = "homed"
            elif cmd == TurntableCommand.Request.CMD_MOVE:
                pos = self._servo.deg_to_pos(request.target_deg)
                time_ms = max(200, int(request.duration_s * 1000)) if request.duration_s > 0 else 2000
                self._servo.move_to(pos, time_ms)
                response.success = True
                response.message = f"moving to {request.target_deg:.1f}"
            elif cmd == TurntableCommand.Request.CMD_STOP:
                self._servo.stop()
                response.success = True
                response.message = "stopped"
            else:
                response.success = False
                response.message = f"unknown command {cmd}"
            return response

        def command_fn(self, command, target_deg, duration_s):
            req = TurntableCommand.Request()
            req.command = command
            req.target_deg = target_deg
            req.duration_s = duration_s
            resp = TurntableCommand.Response()
            self._on_command(req, resp)
            return resp.success, resp.message

        def read_position(self):
            return self._servo.read_position()

    class MockFairy:
        def __init__(self, node, topic: str, servo, period: float = 0.1):
            self._node = node
            self._pub = node.create_publisher(PointCloud2, topic, 10)
            self._servo = servo
            self._period = period
            self._timer = node.create_timer(period, self._publish)
            self._seq = 0
            self._r_mount = mount_rotation([90.0, 0.0, 0.0])

        def _scene_world(self):
            yy, zz = np.meshgrid(np.linspace(-0.5, 0.5, 10), np.linspace(0, 1, 10))
            xx = np.full_like(yy, 2.0)
            return np.stack([xx.ravel(), yy.ravel(), zz.ravel()], axis=1).astype(np.float32)

        def _publish(self):
            from .geometry import rotation_z_deg
            from .pc2_utils import make_cloud_msg

            now = self._node.get_clock().now()
            theta_deg = self._servo.pos_to_deg(self._servo.read_position())
            world = self._scene_world()
            theta_rad = np.deg2rad(-theta_deg)
            c, s = np.cos(theta_rad), np.sin(theta_rad)
            xz = world[:, 0].copy()
            yz = world[:, 1].copy()
            rotated = np.empty_like(world, dtype=np.float64)
            rotated[:, 0] = xz * c - yz * s
            rotated[:, 1] = xz * s + yz * c
            rotated[:, 2] = world[:, 2]
            lidar = (rotated @ np.linalg.inv(self._r_mount).T).astype(np.float32)
            n = lidar.shape[0]
            point_time = np.linspace(0.0, self._period, n, dtype=np.float64)
            msg = make_cloud_msg(lidar, None, "lidar_link", now.to_msg(), point_time=point_time)
            self._pub.publish(msg)
            self._seq += 1

    class MockCamera:
        def __init__(self, node, color_topic: str, depth_topic: str, period: float = 0.1):
            self._node = node
            self._color_pub = node.create_publisher(Image, color_topic, 10)
            self._depth_pub = node.create_publisher(Image, depth_topic, 10)
            self._timer = node.create_timer(period, self._publish)
            self._seq = 0

        def _publish(self):
            stamp = self._node.get_clock().now().to_msg()
            h, w = 4, 4
            color = Image()
            color.header.stamp = stamp
            color.header.frame_id = "camera_color_optical_frame"
            color.height = h
            color.width = w
            color.encoding = "bgr8"
            color.step = w * 3
            color.data = (np.full((h, w, 3), self._seq % 256, dtype=np.uint8)).tobytes()

            depth = Image()
            depth.header.stamp = stamp
            depth.header.frame_id = "camera_color_optical_frame"
            depth.height = h
            depth.width = w
            depth.encoding = "16UC1"
            depth.step = w * 2
            depth.data = (np.full((h, w), 1000 + self._seq, dtype=np.uint16)).tobytes()

            self._color_pub.publish(color)
            self._depth_pub.publish(depth)
            self._seq += 1
```

- [ ] **Step 2: Verify import**

```bash
cd /Users/acelan/workspace/perception_tower
python -c "from perception_tower.mock import FakeServo, MockTurntableService; print('OK')"
```

Expected: OK

- [ ] **Step 3: Commit**

```bash
git add perception_tower/perception_tower/mock.py
git commit -m "feat: MockTurntableService publishes /turntable/status + responds to command"
```

---

## Task 7: Delete servo_client.py, update configs and launch

**Files:**
- Delete: `perception_tower/perception_tower/servo_client.py`
- Modify: `perception_tower/config/tower_params.yaml`
- Modify: `perception_tower/test/test_servo_client.py` (remove or rewrite for sensor_env)
- Modify: `perception_tower/test/test_tower_node.py`

**Interfaces:**
- (cleanup task — no new interfaces)

- [ ] **Step 1: Delete servo_client.py**

```bash
rm perception_tower/perception_tower/servo_client.py
```

- [ ] **Step 2: Rewrite tower_params.yaml**

```yaml
tower_node:
  ros__parameters:
    # Turntable (remote via sensor_env)
    turntable_cmd_service: /turntable/command
    turntable_status_topic: /turntable/status
    pos_tol_deg: 0.1
    pos_stable_count: 5
    pos_origin: 500
    deg_per_pos: 0.02
    angle_sign: 1
    ready_deg: 90.0
    scan_start_deg: 30.0
    scan_end_deg: 150.0
    sweep_speed_deg_s: 40.0
    home_timeout_s: 30.0
    poll_hz: 100.0
    # Fairy
    fairy_topic: /rslidar_points
    fairy_time_field: true
    mount_rpy_deg: [90.0, 0.0, 0.0]
    mount_offset_xyz: [0.0, 0.0, 0.0]
    voxel_leaf_m: 0.01
    world_frame_id: world
    # Camera
    color_topic: /camera/color/image_raw
    depth_topic: /camera/depth/image_raw
    # Output
    output_dir: /tmp/perception_tower
    save_cloud: true
    stitched_topic: /perception_tower/stitched_points
    photo_color_topic: /perception_tower/photo_color
    photo_depth_topic: /perception_tower/photo_depth
    # Debug
    mock_hardware: false
    photo_timeout_s: 5.0
    move_settle_s: 0.2
    move_timeout_s: 30.0
```

- [ ] **Step 3: Delete test_servo_client.py**

This test was for the serial protocol which now lives in sensor_env.

```bash
rm perception_tower/test/test_servo_client.py
```

- [ ] **Step 4: Update test_tower_node.py**

Replace the entire file with:

```python
import os
import tempfile
import time
import pytest
import rclpy
from rclpy.node import Node
from perception_tower_interfaces.srv import TowerCommand
from perception_tower_interfaces.msg import TowerStatus
from sensor_msgs.msg import PointCloud2, Image


@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def wait_for_status(node, state_value, timeout_s=10.0):
    received = []
    sub = node.create_subscription(TowerStatus, '/perception_tower/status', lambda m: received.append(m.state), 10)
    start = time.time()
    while time.time() - start < timeout_s:
        rclpy.spin_once(node, timeout_sec=0.1)
        if received and received[-1] == state_value:
            sub.destroy()
            return True
    sub.destroy()
    return False


def test_mock_scan_end_to_end(ros_context):
    from perception_tower.tower_node import TowerNode
    out = tempfile.mkdtemp()
    node = TowerNode(
        parameter_overrides=[
            ('mock_hardware', True),
            ('output_dir', out),
        ]
    )
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    import threading
    t = threading.Thread(target=executor.spin, daemon=True)
    t.start()

    client_node = rclpy.create_node('test_client')
    cli = client_node.create_client(TowerCommand, '/perception_tower/command')
    assert cli.wait_for_service(timeout_sec=3.0)

    def call(cmd):
        req = TowerCommand.Request()
        req.command = cmd
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(client_node, fut, timeout_sec=2.0)
        return fut.result()

    resp = call(TowerCommand.Request.CMD_INIT)
    assert resp.accepted
    assert wait_for_status(node, TowerStatus.READY, timeout_s=10.0)

    cloud_received = []
    node.create_subscription(PointCloud2, '/perception_tower/stitched_points', lambda m: cloud_received.append(m), 10)
    img_received = []
    node.create_subscription(Image, '/perception_tower/photo_color', lambda m: img_received.append(m), 10)

    resp = call(TowerCommand.Request.CMD_SCAN)
    assert resp.accepted
    assert wait_for_status(node, TowerStatus.READY, timeout_s=30.0)

    assert len(cloud_received) >= 1
    assert len(img_received) >= 1
    assert os.path.exists(os.path.join(out, os.listdir(out)[0], 'color.png'))

    executor.shutdown()
    node.destroy_node()
    client_node.destroy_node()
```

- [ ] **Step 5: Run all tests**

```bash
cd /Users/acelan/workspace/perception_tower
pytest perception_tower/test/ -v
```

Expected: all tests PASS.

- [ ] **Step 6: Commit**

```bash
git add -A perception_tower/
git commit -m "refactor: remove servo_client.py, update configs and tests for remote turntable"
```

---

## Task 8: Create sensor_env launch file (one-click start)

**Files:**
- Create: `perception_tower_sensor/launch/sensor_env.launch.py`

**Interfaces:**
- Consumes: rslidar_sdk, orbbec_camera, turntable_node (from Task 2)

- [ ] **Step 1: Write sensor_env.launch.py**

```python
"""One-click launch for sensor_env: LiDAR + camera + turntable.

Usage:
    ros2 launch perception_tower_sensor sensor_env.launch.py
    ros2 launch perception_tower_sensor sensor_env.launch.py turntable_port:=/dev/ttyUSB1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    turntable_port = LaunchConfiguration("turntable_port")
    params_file = LaunchConfiguration("params_file")

    default_params = PathJoinSubstitution(
        [FindPackageShare("perception_tower_sensor"), "config", "turntable_params.yaml"]
    )

    # LiDAR launch
    rslidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("rslidar_sdk"), "launch", "start.py"])
        )
    )

    # Camera launch
    orbbec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("orbbec_camera"), "launch", "gemini_330_series.launch.py"])
        )
    )

    # Turntable node
    turntable_node = Node(
        package="perception_tower_sensor",
        executable="turntable_node",
        name="turntable_node",
        output="screen",
        parameters=[params_file, {"serial_port": turntable_port}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "turntable_port",
            default_value="/dev/ttyUSB0",
            description="Serial port for turntable STM32.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to turntable parameter YAML.",
        ),
        rslidar_launch,
        orbbec_launch,
        turntable_node,
    ])
```

- [ ] **Step 2: Verify launch file parses**

```bash
cd /Users/acelan/workspace/perception_tower_sensor_env
colcon build --packages-select perception_tower_sensor
source install/setup.bash
ros2 launch perception_tower_sensor sensor_env.launch.py --show-args
```

Expected: shows 2 arguments (turntable_port, params_file).

- [ ] **Step 3: Commit**

```bash
git add perception_tower_sensor/launch/
git commit -m "feat(sensor_env): add one-click launch for LiDAR + camera + turntable"
```

---

## Task 9: Integration test on Ubuntu

**Files:**
- (no new files — verification only)

- [ ] **Step 1: Build both packages in sensor_env**

```bash
cd /Users/acelan/workspace/perception_tower_sensor_env
colcon build
source install/setup.bash
```

- [ ] **Step 2: Launch sensor_env with real hardware**

```bash
ros2 launch perception_tower_sensor sensor_env.launch.py
```

Verify:
- `/turntable/status` topic appears: `ros2 topic list | grep turntable`
- Status publishing at ~50Hz: `ros2 topic hz /turntable/status`
- Service available: `ros2 service list | grep turntable`

- [ ] **Step 3: Test turntable command**

```bash
ros2 service call /turntable/command perception_tower_sensor_interfaces/srv/TurntableCommand "{command: 1, target_deg: 90.0, duration_s: 0.0}"
```

Expected: turntable homes and moves to 90 deg. response.success = true.

- [ ] **Step 4: Build perception_tower on Mac**

```bash
cd /Users/acelan/workspace/perception_tower
colcon build --packages-select perception_tower_interfaces perception_tower
source install/setup.bash
```

- [ ] **Step 5: Launch perception_tower on Mac**

```bash
ros2 launch perception_tower tower.launch.py
```

- [ ] **Step 6: Test full pipeline**

```bash
# Init
ros2 service call /perception_tower/command perception_tower_interfaces/srv/TowerCommand "{command: 1}"
# Wait for READY status, then scan
ros2 service call /perception_tower/command perception_tower_interfaces/srv/TowerCommand "{command: 2}"
```

Verify: stitched point cloud published, PCD file saved, photos saved.

- [ ] **Step 7: Final commit**

```bash
git add -A
git commit -m "chore: integration test passed, turntable fully migrated to sensor_env"
```
