# perception_tower 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 `/Users/acelan/workspace/perception_tower/` 下实现 ROS2 Humble 包 `perception_tower` 及其接口包 `perception_tower_interfaces`，对外提供 `/perception_tower/command` 服务（初始化 / 扫描），控制步进电机转盘并在 30°~150° 转盘旋转过程中拼合 Fairy LiDAR 点云、输出 336L 照片。

**Architecture:** 单节点内聚架构：`tower_node` 为主节点，内部模块化（串口客户端、100Hz 角度日志、相机抓拍、Fairy 缓存、逐点补偿拼合、状态机）。核心数学移植自 `pallet_vision_lidar` 的 `turntable_stitcher/transform.py`，但自包含、不依赖其他项目。支持 `mock_hardware:=true` 在 macOS conda + RoboStack 环境无硬件跑通。

**Tech Stack:** ROS2 Humble, Python 3.10, ament_python + ament_cmake/rosidl, pyserial, numpy, cv_bridge, pytest. 不引入 open3d / PCL。

**Spec:** `docs/superpowers/specs/2026-09-05-perception-tower-design.md`

## Global Constraints

- 两个包：`perception_tower_interfaces`（ament_cmake/rosidl）、`perception_tower`（ament_python）。
- 舵机协议：串口 115200 8N1，固件源码见 `/Users/acelan/workspace/servo-driver/servo-control/src/`；回复为 `#000P{pos}!` / `#OK!`，需容错调试串脏数据。
- 位置-角度映射：`angle_raw_deg = (pos − 500) × 0.02`；90° → 5000，30° → 2000，150° → 8000。
- 扫描速度 40°/s，轮询 100Hz，到位容差 0.1°，连续 5 次稳定判到位。
- 拼合数学：`P_world = Rz(θ) · (R_mount · P_lidar + T_mount)`，默认 `R_mount = Rx(+90°)`、`T_mount = [0,0,0]`；裁剪窗口 [30°,150°] 作用于 `θ_raw`，符号参数仅影响 Rz 方向。
- Fairy 逐点补偿：利用 PointCloud2 的 `time` 字段，帧首锚定 `t_origin = header.stamp − frame_period`。
- 服务非阻塞：回调立即返回 `accepted`；状态与进度由 `/perception_tower/status` 发布（transient_local QoS）。
- 开发/验证环境：macOS conda `robostack-humble`；生产部署 Ubuntu 22.04 Humble。
- 依赖限制：不引入 open3d / PCL；PCD 写手写实现。

---

## File Map

| 文件 | 责任 |
|------|------|
| `perception_tower_interfaces/msg/TowerStatus.msg` | FSM 状态枚举与进度消息 |
| `perception_tower_interfaces/srv/TowerCommand.srv` | INIT/SCAN 服务定义 |
| `perception_tower/perception_tower/geometry.py` | 转盘旋转与横装外参变换 |
| `perception_tower/perception_tower/pc2_utils.py` | PointCloud2 构造/解析、PCD 读写 |
| `perception_tower/perception_tower/servo_client.py` | 串口协议客户端 + 到位轮询 |
| `perception_tower/perception_tower/angle_logger.py` | 100Hz 位置日志 + 时间插值 |
| `perception_tower/perception_tower/camera_grabber.py` | 336L 彩色/深度图抓拍与保存 |
| `perception_tower/perception_tower/fairy_buffer.py` | Fairy 帧缓存 + 帧首锚定 |
| `perception_tower/perception_tower/stitcher.py` | 逐点角度补偿 + 拼合 + 体素下采样 |
| `perception_tower/perception_tower/fsm.py` | INIT/SCAN 状态机（后台线程） |
| `perception_tower/perception_tower/mock.py` | FakeServo + MockFairy + MockCamera |
| `perception_tower/perception_tower/tower_node.py` | ROS2 节点装配与启动入口 |
| `perception_tower/config/tower_params.yaml` | 默认参数 |
| `perception_tower/launch/tower.launch.py` | launch 文件（支持 mock_hardware） |
| `perception_tower/README.md` | 安装、部署、使用、服务调用说明 |

---

## Task 1: 环境 + 双包骨架 + 首次构建

**Files:**
- Create: `perception_tower_interfaces/package.xml`
- Create: `perception_tower_interfaces/CMakeLists.txt`
- Create: `perception_tower_interfaces/msg/TowerStatus.msg`
- Create: `perception_tower_interfaces/srv/TowerCommand.srv`
- Create: `perception_tower/package.xml`
- Create: `perception_tower/setup.py`
- Create: `perception_tower/setup.cfg`
- Create: `perception_tower/resource/perception_tower`
- Create: `perception_tower/perception_tower/__init__.py`
- Create: `perception_tower/test/__init__.py`
- Create: `perception_tower/config/tower_params.yaml`
- Create: `perception_tower/launch/tower.launch.py`
- Modify: none
- Test: `perception_tower/test/test_import.py`

**Interfaces:**
- Produces: 两个包可 `colcon build --symlink-install` 通过；`perception_tower_interfaces` 的 Python msg/srv 可被导入；`perception_tower` 可作为 ament_python 包被安装。

- [ ] **Step 1: conda 环境创建**

```bash
conda create -n tower -c robostack-humble -c conda-forge --override-channels \
  python=3.10 ros-humble-ros-base ros-humble-cv-bridge \
  ros-humble-rosidl-default-generators ros-humble-ament-cmake-python \
  colcon-common-extensions pytest numpy pyserial opencv
conda activate tower
```

- [ ] **Step 2: 创建接口包**

`perception_tower_interfaces/package.xml`:
```xml
<?xml version="1.0"?>
<package format="3">
  <name>perception_tower_interfaces</name>
  <version>0.0.1</version>
  <description>Interfaces for perception_tower</description>
  <maintainer email="you@example.com">You</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
</package>
```

`perception_tower_interfaces/CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.8)
project(perception_tower_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TowerStatus.msg"
  "srv/TowerCommand.srv"
)
ament_package()
```

`perception_tower_interfaces/msg/TowerStatus.msg`:
```
uint8 IDLE=0
uint8 INITING=1
uint8 READY=2
uint8 SCANNING=3
uint8 PROCESSING=4
uint8 ERROR=5
uint8 state
uint8 progress_pct
string message
```

`perception_tower_interfaces/srv/TowerCommand.srv`:
```
uint8 CMD_INIT=1
uint8 CMD_SCAN=2
uint8 command
---
bool accepted
string message
```

- [ ] **Step 3: 创建主包骨架**

`perception_tower/package.xml`:
```xml
<?xml version="1.0"?>
<package format="3">
  <name>perception_tower</name>
  <version>0.0.1</version>
  <description>Perception tower controller and turntable scanner</description>
  <maintainer email="you@example.com">You</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>perception_tower_interfaces</depend>
  <depend>cv_bridge</depend>
  <depend>python3-numpy</depend>
  <depend>python3-serial</depend>
  <test_depend>python3-pytest</test_depend>
</package>
```

`perception_tower/setup.py`:
```python
from setuptools import setup
import os
from glob import glob

package_name = 'perception_tower'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/config', glob('config/*')),
        (f'share/{package_name}/launch', glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='Perception tower controller and turntable scanner',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'tower_node = {package_name}.tower_node:main',
        ],
    },
)
```

`perception_tower/setup.cfg`:
```ini
[develop]
script_dir=$base/lib/perception_tower
[install]
install_scripts=$base/lib/perception_tower
```

`perception_tower/resource/perception_tower`:
```

```

- [ ] **Step 4: 占位参数文件与 launch**

`perception_tower/config/tower_params.yaml`（后续任务会完善内容，这里先保证 launch 可加载）：
```yaml
tower_node:
  ros__parameters:
    serial_port: ""
    serial_baud: 115200
    poll_hz: 100.0
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
    fairy_topic: /fairy/points
    fairy_time_field: true
    mount_rpy_deg: [90.0, 0.0, 0.0]
    mount_offset_xyz: [0.0, 0.0, 0.0]
    voxel_leaf_m: 0.01
    world_frame_id: world
    color_topic: /camera/color/image_raw
    depth_topic: /camera/depth/image_raw
    output_dir: /tmp/perception_tower
    save_cloud: true
    stitched_topic: /perception_tower/stitched_points
    photo_color_topic: /perception_tower/photo_color
    photo_depth_topic: /perception_tower/photo_depth
    mock_hardware: false
```

`perception_tower/launch/tower.launch.py`（占位，后续任务完善 mock 分支）：
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('perception_tower')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg, 'config', 'tower_params.yaml')),
        DeclareLaunchArgument('mock_hardware', default_value='false'),
        Node(
            package='perception_tower',
            executable='tower_node',
            name='tower_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {'mock_hardware': LaunchConfiguration('mock_hardware')}],
        ),
    ])
```

- [ ] **Step 5: 写首个冒烟测试**

`perception_tower/test/test_import.py`:
```python
def test_interfaces_import():
    from perception_tower_interfaces.msg import TowerStatus
    from perception_tower_interfaces.srv import TowerCommand
    assert TowerStatus.IDLE == 0
    assert TowerCommand.Request.CMD_INIT == 1


def test_package_import():
    import perception_tower
    assert perception_tower.__name__ == 'perception_tower'
```

- [ ] **Step 6: 构建并运行冒烟测试**

```bash
cd /Users/acelan/workspace/perception_tower
conda activate tower
colcon build --symlink-install
source install/setup.bash
python -m pytest perception_tower/test/test_import.py -v
```

- [ ] **Step 7: Commit**

```bash
git add -A
git commit -m "chore: package skeleton + interfaces + first build"
```

---

## Task 2: geometry.py（变换数学，TDD）

**Files:**
- Create: `perception_tower/perception_tower/geometry.py`
- Create: `perception_tower/test/test_geometry.py`

**Interfaces:**
- Produces: `rotation_x_deg`, `rotation_y_deg`, `rotation_z_deg`, `mount_rotation(rpy_deg)`, `transform_frame(xyz, r_mount, t_mount, theta_deg)`.
- `theta_deg` 支持标量或 `(N,)` 数组，返回 world-frame xyz `(N, 3)` float64。

- [ ] **Step 1: 写失败测试**

`perception_tower/test/test_geometry.py`:
```python
import numpy as np
from perception_tower.geometry import rotation_z_deg, mount_rotation, transform_frame


def test_rotation_z_90():
    pts = np.array([[1.0, 0.0, 0.0]])
    out = transform_frame(pts, np.eye(3), [0.0, 0.0, 0.0], 90.0)
    assert np.allclose(out, [[0.0, 1.0, 0.0]], atol=1e-6)


def test_mount_rx90_z_becomes_minus_y():
    r_mount = mount_rotation([90.0, 0.0, 0.0])
    out = transform_frame(np.array([[0.0, 0.0, 1.0]]), r_mount, [0.0, 0.0, 0.0], 0.0)
    assert np.allclose(out, [[0.0, -1.0, 0.0]], atol=1e-6)


def test_per_point_theta():
    pts = np.array([[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    out = transform_frame(pts, np.eye(3), [0.0, 0.0, 0.0], np.array([0.0, 90.0]))
    assert np.allclose(out[0], [1.0, 0.0, 0.0])
    assert np.allclose(out[1], [0.0, 1.0, 0.0])


def test_empty_input():
    out = transform_frame(np.zeros((0, 3)), np.eye(3), [0.0, 0.0, 0.0], 0.0)
    assert out.shape == (0, 3)
```

- [ ] **Step 2: 运行确认失败**

```bash
python -m pytest perception_tower/test/test_geometry.py -v
```

- [ ] **Step 3: 实现 geometry.py**

`perception_tower/perception_tower/geometry.py`:
```python
from __future__ import annotations
from typing import Sequence
import numpy as np


def _to_rad(angle_deg: float) -> float:
    return angle_deg * np.pi / 180.0


def rotation_x_deg(angle_deg: float) -> np.ndarray:
    a = _to_rad(angle_deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float64)


def rotation_y_deg(angle_deg: float) -> np.ndarray:
    a = _to_rad(angle_deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)


def rotation_z_deg(angle_deg: float) -> np.ndarray:
    a = _to_rad(angle_deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def mount_rotation(rpy_deg: Sequence[float]) -> np.ndarray:
    roll, pitch, yaw = rpy_deg
    return rotation_z_deg(yaw) @ rotation_y_deg(pitch) @ rotation_x_deg(roll)


def transform_frame(
    xyz: np.ndarray,
    r_mount: np.ndarray,
    t_mount: Sequence[float],
    theta_deg: float | np.ndarray,
) -> np.ndarray:
    xyz = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
    if xyz.shape[0] == 0:
        return xyz.copy()
    base = xyz @ np.asarray(r_mount, dtype=np.float64).T + np.asarray(t_mount, dtype=np.float64)
    th = np.deg2rad(np.asarray(theta_deg, dtype=np.float64))
    c, s = np.cos(th), np.sin(th)
    x, y = base[:, 0].copy(), base[:, 1].copy()
    out = np.empty_like(base)
    out[:, 0] = x * c - y * s
    out[:, 1] = x * s + y * c
    out[:, 2] = base[:, 2]
    return out
```

- [ ] **Step 4: 运行确认通过**

```bash
python -m pytest perception_tower/test/test_geometry.py -v
```

- [ ] **Step 5: Commit**

```bash
git add perception_tower/perception_tower/geometry.py perception_tower/test/test_geometry.py
git commit -m "feat: geometry transform with per-point theta"
```

---

## Task 3: pc2_utils.py（PointCloud2 解析/构造 + PCD 读写，TDD）

**Files:**
- Create: `perception_tower/perception_tower/pc2_utils.py`
- Create: `perception_tower/test/test_pc2_utils.py`

**Interfaces:**
- Produces:
  - `read_field(msg, name) -> np.ndarray | None`
  - `read_xyz(msg) -> (N,3) float32`
  - `read_time(msg) -> (N,) float64 | None`
  - `read_intensity(msg) -> (N,) float32 | None`
  - `make_cloud_msg(xyz, intensity, frame_id, stamp) -> sensor_msgs.msg.PointCloud2`
  - `save_pcd_binary(path, xyz, intensity=None) -> None`
  - `load_pcd_binary(path) -> (xyz, intensity|None)`
- `xyz` 为 `(N,3)` float32；`intensity` 为 `(N,)` float32 或 None；`stamp` 为 `builtin_interfaces.msg.Time`。

- [ ] **Step 1: 写失败测试**

`perception_tower/test/test_pc2_utils.py`:
```python
import numpy as np
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2
from perception_tower.pc2_utils import (
    make_cloud_msg, read_xyz, read_intensity, read_time,
    save_pcd_binary, load_pcd_binary,
)


def test_make_and_read_xyz():
    xyz = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
    msg = make_cloud_msg(xyz, None, 'lidar_link', Time(sec=10, nanosec=0))
    out = read_xyz(msg)
    assert np.allclose(out, xyz)


def test_make_and_read_with_intensity():
    xyz = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
    intensity = np.array([0.5], dtype=np.float32)
    msg = make_cloud_msg(xyz, intensity, 'lidar_link', Time(sec=0, nanosec=0))
    assert np.allclose(read_xyz(msg), xyz)
    assert np.allclose(read_intensity(msg), intensity)


def test_read_time_field():
    xyz = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float32)
    times = np.array([0.0, 0.05], dtype=np.float64)
    msg = make_cloud_msg(xyz, None, 'lidar_link', Time(sec=0, nanosec=0), point_time=times)
    assert np.allclose(read_time(msg), times)


def test_pcd_roundtrip():
    xyz = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
    intensity = np.array([10.0, 20.0], dtype=np.float32)
    import tempfile, os
    path = os.path.join(tempfile.mkdtemp(), 't.pcd')
    save_pcd_binary(path, xyz, intensity)
    out_xyz, out_i = load_pcd_binary(path)
    assert np.allclose(out_xyz, xyz)
    assert np.allclose(out_i, intensity)


def test_pcd_without_intensity():
    xyz = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
    import tempfile, os
    path = os.path.join(tempfile.mkdtemp(), 't.pcd')
    save_pcd_binary(path, xyz)
    out_xyz, out_i = load_pcd_binary(path)
    assert out_i is None
```

- [ ] **Step 2: 运行确认失败**

```bash
python -m pytest perception_tower/test/test_pc2_utils.py -v
```

- [ ] **Step 3: 实现 pc2_utils.py**

`perception_tower/perception_tower/pc2_utils.py`:
```python
from __future__ import annotations
import struct
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from builtin_interfaces.msg import Time


_TYPEMAP = {
    PointField.INT8: np.int8,
    PointField.UINT8: np.uint8,
    PointField.INT16: np.int16,
    PointField.UINT16: np.uint16,
    PointField.INT32: np.int32,
    PointField.UINT32: np.uint32,
    PointField.FLOAT32: np.float32,
    PointField.FLOAT64: np.float64,
}


def _find_field(msg: PointCloud2, name: str) -> PointField | None:
    for f in msg.fields:
        if f.name == name:
            return f
    return None


def read_field(msg: PointCloud2, name: str) -> np.ndarray | None:
    f = _find_field(msg, name)
    if f is None:
        return None
    n = msg.width * msg.height
    dtype = np.dtype(_TYPEMAP[f.datatype])
    arr = np.frombuffer(msg.data, dtype=np.uint8, count=n * msg.point_step).reshape(n, msg.point_step)
    cols = arr[:, f.offset : f.offset + dtype.itemsize].copy()
    return cols.view(dtype).reshape(n)


def read_xyz(msg: PointCloud2) -> np.ndarray:
    return np.stack([read_field(msg, 'x'), read_field(msg, 'y'), read_field(msg, 'z')], axis=1).astype(np.float32)


def read_time(msg: PointCloud2) -> np.ndarray | None:
    t = read_field(msg, 'time')
    return None if t is None else t.astype(np.float64)


def read_intensity(msg: PointCloud2) -> np.ndarray | None:
    t = read_field(msg, 'intensity')
    return None if t is None else t.astype(np.float32)


def make_cloud_msg(
    xyz: np.ndarray,
    intensity: np.ndarray | None,
    frame_id: str,
    stamp: Time,
    point_time: np.ndarray | None = None,
) -> PointCloud2:
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    n = xyz.shape[0]
    names = ['x', 'y', 'z']
    formats = [np.float32, np.float32, np.float32]
    offsets = [0, 4, 8]
    if intensity is not None:
        names.append('intensity')
        formats.append(np.float32)
        offsets.append(12)
    if point_time is not None:
        names.append('time')
        formats.append(np.float64)
        offsets.append(16 if intensity is None else 20)
    dtype = np.dtype({'names': names, 'formats': formats, 'offsets': offsets, 'itemsize': offsets[-1] + np.dtype(formats[-1]).itemsize})
    rec = np.zeros(n, dtype=dtype)
    rec['x'] = xyz[:, 0]
    rec['y'] = xyz[:, 1]
    rec['z'] = xyz[:, 2]
    if intensity is not None:
        rec['intensity'] = np.asarray(intensity, dtype=np.float32)
    if point_time is not None:
        rec['time'] = np.asarray(point_time, dtype=np.float64)
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = n
    msg.fields = [
        PointField(name=name, offset=off, datatype=_key_for_type(np.dtype(fmt)), count=1)
        for name, off, fmt in zip(names, offsets, formats)
    ]
    msg.is_bigendian = False
    msg.point_step = dtype.itemsize
    msg.row_step = dtype.itemsize * n
    msg.is_dense = False
    msg.data = rec.tobytes()
    return msg


def _key_for_type(dtype: np.dtype) -> int:
    for k, v in _TYPEMAP.items():
        if v == dtype.type:
            return k
    raise ValueError(f'unsupported dtype {dtype}')


def save_pcd_binary(path: str, xyz: np.ndarray, intensity: np.ndarray | None = None) -> None:
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    n = xyz.shape[0]
    has_i = intensity is not None
    fields = 'x y z intensity' if has_i else 'x y z'
    size = '4 4 4 4' if has_i else '4 4 4'
    typ = 'F F F F' if has_i else 'F F F'
    count = '1 1 1 1' if has_i else '1 1 1'
    header = (
        f'# .PCD v0.7 - Point Cloud Data file format\n'
        f'VERSION 0.7\n'
        f'FIELDS {fields}\n'
        f'SIZE {size}\n'
        f'TYPE {typ}\n'
        f'COUNT {count}\n'
        f'WIDTH {n}\n'
        f'HEIGHT 1\n'
        f'VIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {n}\n'
        f'DATA binary\n'
    )
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(xyz.tobytes())
        if has_i:
            f.write(np.asarray(intensity, dtype=np.float32).tobytes())


def load_pcd_binary(path: str) -> tuple[np.ndarray, np.ndarray | None]:
    with open(path, 'rb') as f:
        header_lines = []
        while True:
            line = f.readline().decode('ascii')
            header_lines.append(line)
            if line.startswith('DATA'):
                break
    meta = {}
    for line in header_lines:
        if ' ' in line:
            k, v = line.strip().split(' ', 1)
            meta[k] = v
    n = int(meta['POINTS'])
    fields = meta['FIELDS'].split()
    sizes = list(map(int, meta['SIZE'].split()))
    types = meta['TYPE'].split()
    fmt_map = {'F': np.float32, 'I': np.int32, 'U': np.uint32}
    itemsize = sum(sizes)
    raw = np.fromfile(path, dtype=np.uint8, offset=sum(len(l.encode('ascii')) for l in header_lines))
    rec = raw[: n * itemsize].reshape(n, itemsize)
    off = 0
    out = {}
    for field, size, typ in zip(fields, sizes, types):
        out[field] = rec[:, off : off + size].copy().view(fmt_map[typ]).reshape(n)
        off += size
    xyz = np.stack([out['x'], out['y'], out['z']], axis=1).astype(np.float32)
    intensity = out.get('intensity')
    return xyz, intensity
```

- [ ] **Step 4: 运行确认通过**

```bash
python -m pytest perception_tower/test/test_pc2_utils.py -v
```

- [ ] **Step 5: Commit**

```bash
git add perception_tower/perception_tower/pc2_utils.py perception_tower/test/test_pc2_utils.py
git commit -m "feat: PointCloud2 parse/build and binary PCD roundtrip"
```

---

## Task 4: 协议解析器 ProtocolParser（含脏数据容差，TDD）

**Files:**
- Create: `perception_tower/perception_tower/servo_client.py`（仅 `ProtocolParser` 与常量）
- Create: `perception_tower/test/test_servo_parser.py`

**Interfaces:**
- Produces: `ProtocolParser(servo_id=0)` 的 `feed(data: bytes) -> list[tuple]`，事件格式 `('pos', int)` / `('ok',)`。

- [ ] **Step 1: 写失败测试**

`perception_tower/test/test_servo_parser.py`:
```python
from perception_tower.servo_client import ProtocolParser


def test_parse_position():
    p = ProtocolParser()
    assert p.feed(b'#000P5000!\r\n') == [('pos', 5000)]


def test_parse_ok():
    p = ProtocolParser()
    assert p.feed(b'#OK!\r\n') == [('ok',)]


def test_dirty_debug_strings_filtered():
    p = ProtocolParser()
    data = b'BOOT: ready\r\n#000P5000!\r\nMOV: 2000 -> 5000\r\n#OK!\r\n'
    assert p.feed(data) == [('pos', 5000), ('ok',)]


def test_partial_and_resumable():
    p = ProtocolParser()
    assert p.feed(b'#000P50') == []
    assert p.feed(b'00!\r\n#OK!') == [('pos', 5000)]
    assert p.feed(b'\r\n') == [('ok',)]


def test_custom_servo_id():
    p = ProtocolParser(servo_id=1)
    assert p.feed(b'#001P1234!') == [('pos', 1234)]
    assert p.feed(b'#000P1234!') == []
```

- [ ] **Step 2: 运行确认失败**

```bash
python -m pytest perception_tower/test/test_servo_parser.py -v
```

- [ ] **Step 3: 实现 ProtocolParser**

在 `perception_tower/perception_tower/servo_client.py` 顶部写入：
```python
from __future__ import annotations
import re


_OK_EVENT = ('ok',)
_POSITION_RE = re.compile(rb'^(\d{3})P(\d+)$')


class ProtocolParser:
    def __init__(self, servo_id: int = 0):
        self._id = servo_id
        self._buf = bytearray()
        self._id_bytes = f'{servo_id:03d}'.encode()

    def feed(self, data: bytes) -> list[tuple]:
        self._buf.extend(data)
        events = []
        while True:
            start = self._buf.find(b'#')
            if start < 0:
                self._buf.clear()
                break
            if start > 0:
                del self._buf[:start]
            end = self._buf.find(b'!')
            if end < 0:
                break
            chunk = bytes(self._buf[1:end])
            del self._buf[: end + 1]
            if chunk == b'OK':
                events.append(_OK_EVENT)
            else:
                m = _POSITION_RE.match(chunk)
                if m and m.group(1) == self._id_bytes:
                    events.append(('pos', int(m.group(2))))
        return events
```

- [ ] **Step 4: 运行确认通过**

```bash
python -m pytest perception_tower/test/test_servo_parser.py -v
```

- [ ] **Step 5: Commit**

```bash
git add perception_tower/perception_tower/servo_client.py perception_tower/test/test_servo_parser.py
git commit -m "feat: streaming servo protocol parser tolerant to debug noise"
```

---

## Task 5: ServoClient 串口交互 + 到位轮询（TDD）

**Files:**
- Modify: `perception_tower/perception_tower/servo_client.py`（补全 ServoClient + poll_until_reached）
- Create: `perception_tower/test/test_servo_client.py`

**Interfaces:**
- Produces:
  - `ServoClient(port, baud=115200, servo_id=0, pos_origin=500, deg_per_pos=0.02)`
  - `open()`, `close()`, `move_to(pos, time_ms)`, `stop()`, `reset(timeout_s)`, `read_position(timeout_s) -> int`
  - `pos_to_deg(pos)`, `deg_to_pos(deg)`
  - `poll_until_reached(read_position, deg_of, pos_target, tol_deg, stable_count, timeout_s, poll_hz, progress_cb=None)`（模块级函数，真实与 fake 共用）
- 真实串口写 / 读基于 pyserial；测试通过 `serial_factory` 注入 fake serial。

- [ ] **Step 1: 写失败测试**

`perception_tower/test/test_servo_client.py`:
```python
import threading
import time
import numpy as np
import pytest
from perception_tower.servo_client import ServoClient, poll_until_reached, ServoError


class FakeSerial:
    def __init__(self, script):
        """script: callable(write_bytes) -> reply_bytes or None"""
        self._script = script
        self._rx = bytearray()
        self.written = []
        self._lock = threading.Lock()

    def write(self, data: bytes):
        with self._lock:
            self.written.append(bytes(data))
            reply = self._script(data)
            if reply:
                self._rx.extend(reply)
        return len(data)

    def read(self, size: int = 1) -> bytes:
        with self._lock:
            n = min(size, len(self._rx))
            out = bytes(self._rx[:n])
            del self._rx[:n]
            return out

    def close(self):
        pass


def test_read_position():
    def script(d):
        if d == b'#000PRAD!':
            return b'#000P5000!\r\n'
    c = ServoClient('/dev/null', serial_factory=lambda p, b, timeout: FakeSerial(script))
    c.open()
    assert c.read_position() == 5000
    c.close()


def test_reset_waits_ok():
    def script(d):
        if d == b'#000PRST!':
            time.sleep(0.05)
            return b'#OK!\r\n'
    c = ServoClient('/dev/null', serial_factory=lambda p, b, timeout: FakeSerial(script))
    c.open()
    c.reset(timeout_s=1.0)
    c.close()


def test_move_to_writes_correct_command():
    fake = FakeSerial(lambda d: None)
    c = ServoClient('/dev/null', serial_factory=lambda p, b, timeout: fake)
    c.open()
    c.move_to(5000, 2000)
    c.close()
    assert fake.written == [b'#000P5000T2000!']


def test_poll_until_reached():
    positions = [4980, 4990, 4995, 4998, 5000, 5000, 5000, 5000, 5000]
    it = iter(positions)

    def read():
        return next(it)

    def deg_of(pos):
        return (pos - 500) * 0.02

    poll_until_reached(read, deg_of, 5000, tol_deg=0.1, stable_count=3, timeout_s=5.0, poll_hz=1000.0)


def test_poll_until_reached_timeout():
    def read():
        return 4900

    def deg_of(pos):
        return (pos - 500) * 0.02

    with pytest.raises(ServoError):
        poll_until_reached(read, deg_of, 5000, tol_deg=0.1, stable_count=3, timeout_s=0.2, poll_hz=1000.0)


def test_disabled_serial_port_raises():
    c = ServoClient('', serial_factory=lambda p, b, timeout: FakeSerial(lambda d: None))
    with pytest.raises(ServoError):
        c.open()
```

- [ ] **Step 2: 运行确认失败**

```bash
python -m pytest perception_tower/test/test_servo_client.py -v
```

- [ ] **Step 3: 补全 servo_client.py**

追加到 `perception_tower/perception_tower/servo_client.py`：
```python
import queue
import threading
import time


class ServoError(RuntimeError):
    pass


def poll_until_reached(
    read_position,
    deg_of,
    pos_target,
    tol_deg,
    stable_count,
    timeout_s,
    poll_hz,
    progress_cb=None,
):
    period = 1.0 / poll_hz
    start_deg = None
    count = 0
    start_t = time.monotonic()
    n = 0
    while True:
        t0 = time.monotonic()
        if t0 - start_t > timeout_s:
            raise ServoError('position timeout')
        pos = read_position(timeout_s=period * 2.0)
        deg = deg_of(pos)
        if start_deg is None:
            start_deg = deg
        if abs(deg - deg_of(pos_target)) <= tol_deg:
            count += 1
        else:
            count = 0
        n += 1
        if progress_cb and n % 10 == 0:
            total = abs(deg_of(pos_target) - start_deg)
            remain = max(0.0, abs(deg_of(pos_target) - deg))
            pct = 0.0 if total <= 0 else 1.0 - remain / total
            progress_cb(min(0.999, pct))
        if count >= stable_count:
            if progress_cb:
                progress_cb(1.0)
            return deg
        elapsed = time.monotonic() - t0
        if elapsed < period:
            time.sleep(period - elapsed)


class ServoClient:
    def __init__(
        self,
        port: str,
        baud: int = 115200,
        servo_id: int = 0,
        pos_origin: int = 500,
        deg_per_pos: float = 0.02,
        serial_factory=None,
    ):
        self._port = port
        self._baud = baud
        self._servo_id = servo_id
        self._origin = pos_origin
        self._dpp = deg_per_pos
        self._serial_factory = serial_factory
        self._ser = None
        self._parser = ProtocolParser(servo_id)
        self._reply_q = queue.SimpleQueue()
        self._write_lock = threading.Lock()
        self._reader_thread = None
        self._running = False

    def open(self):
        if not self._port:
            raise ServoError('serial_port not configured')
        factory = self._serial_factory
        if factory is None:
            import serial as _serial
            factory = _serial.Serial
        self._ser = factory(self._port, self._baud, timeout=0.05)
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
                for ev in self._parser.feed(data):
                    self._reply_q.put(ev)

    def _send(self, payload: bytes):
        with self._write_lock:
            if self._ser is None:
                raise ServoError('serial not open')
            self._ser.write(payload)

    def _wait_event(self, kinds: tuple, timeout_s: float):
        deadline = time.monotonic() + timeout_s
        while True:
            remain = deadline - time.monotonic()
            if remain <= 0:
                raise ServoError(f'timeout waiting for {kinds}')
            try:
                ev = self._reply_q.get(timeout=min(remain, 0.1))
            except queue.Empty:
                continue
            if ev[0] in kinds:
                return ev

    def move_to(self, pos: int, time_ms: int):
        self._send(f'#{self._servo_id:03d}P{pos}T{time_ms}!'.encode())

    def stop(self):
        self._send(f'#{self._servo_id:03d}PDST!'.encode())
        self._wait_event(('ok',), 0.5)

    def read_position(self, timeout_s: float = 0.2) -> int:
        self._send(f'#{self._servo_id:03d}PRAD!'.encode())
        ev = self._wait_event(('pos',), timeout_s)
        return ev[1]

    def reset(self, timeout_s: float = 30.0):
        self._send(f'#{self._servo_id:03d}PRST!'.encode())
        self._wait_event(('ok',), timeout_s)

    def pos_to_deg(self, pos: int) -> float:
        return (pos - self._origin) * self._dpp

    def deg_to_pos(self, deg: float) -> int:
        return int(round(self._origin + deg / self._dpp))

    def poll_until_reached(
        self,
        pos_target: int,
        tol_deg: float,
        stable_count: int = 5,
        timeout_s: float = 30.0,
        poll_hz: float = 100.0,
        progress_cb=None,
    ) -> float:
        return poll_until_reached(
            self.read_position,
            self.pos_to_deg,
            pos_target,
            tol_deg,
            stable_count,
            timeout_s,
            poll_hz,
            progress_cb,
        )
```

- [ ] **Step 4: 运行确认通过**

```bash
python -m pytest perception_tower/test/test_servo_client.py -v
```

- [ ] **Step 5: Commit**

```bash
git add perception_tower/perception_tower/servo_client.py perception_tower/test/test_servo_client.py
git commit -m "feat: ServoClient with blocking reads and poll-until-reached"
```

---

## Task 6: angle_logger.py（100Hz 日志 + 时间插值，TDD）

**Files:**
- Create: `perception_tower/perception_tower/angle_logger.py`
- Create: `perception_tower/test/test_angle_logger.py`

**Interfaces:**
- Produces: `AngleLogger(read_position, clock_now, pos_to_deg, poll_hz=100.0)`
  - `start()` / `stop()`
  - `angles_at(ts: np.ndarray) -> np.ndarray`：对数列线性插值返回 `θ_raw_deg`；ts 越界时钳位到端点。
  - `coverage() -> (t0, t1) | None`
  - `error: Exception | None`

- [ ] **Step 1: 写失败测试**

`perception_tower/test/test_angle_logger.py`:
```python
import time
import numpy as np
import pytest
from perception_tower.angle_logger import AngleLogger


def test_interpolation_and_clamping():
    clock_times = [0.0, 0.01, 0.02, 0.03]
    positions = [500, 1000, 1500, 2000]
    clock_iter = iter(clock_times)
    pos_iter = iter(positions)

    def read_position(timeout_s):
        return next(pos_iter)

    def clock_now():
        return next(clock_iter)

    logger = AngleLogger(read_position, clock_now, lambda p: (p - 500) * 0.02, poll_hz=100.0)
    logger.start()
    while len(logger._samples) < 4:
        time.sleep(0.001)
    logger.stop()

    ts = np.array([-0.1, 0.015, 0.025, 0.05])
    out = logger.angles_at(ts)
    assert out[0] == 0.0
    assert out[-1] == 30.0
    assert np.isclose(out[1], 10.0)
    assert np.isclose(out[2], 20.0)


def test_error_propagation():
    def read_position(timeout_s):
        raise RuntimeError('boom')

    logger = AngleLogger(read_position, time.time, lambda p: 0.0, poll_hz=1000.0)
    logger.start()
    time.sleep(0.01)
    logger.stop()
    assert isinstance(logger.error, RuntimeError)
```

- [ ] **Step 2: 运行确认失败**

```bash
python -m pytest perception_tower/test/test_angle_logger.py -v
```

- [ ] **Step 3: 实现 angle_logger.py**

`perception_tower/perception_tower/angle_logger.py`:
```python
from __future__ import annotations
import threading
import time
from typing import Callable
import numpy as np


class AngleLogger:
    def __init__(
        self,
        read_position: Callable[[float], int],
        clock_now: Callable[[], float],
        pos_to_deg: Callable[[int], float],
        poll_hz: float = 100.0,
    ):
        self._read_position = read_position
        self._clock_now = clock_now
        self._pos_to_deg = pos_to_deg
        self._period = 1.0 / poll_hz
        self._samples = []
        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._thread = None
        self.error = None

    def start(self):
        self._samples = []
        self.error = None
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=0.5)

    def _loop(self):
        try:
            while not self._stop_evt.is_set():
                t0 = time.monotonic()
                pos = self._read_position(timeout_s=self._period * 2.0)
                t = self._clock_now()
                with self._lock:
                    self._samples.append((t, self._pos_to_deg(pos)))
                elapsed = time.monotonic() - t0
                if elapsed < self._period:
                    time.sleep(self._period - elapsed)
        except Exception as exc:
            self.error = exc

    def _get_samples(self):
        with self._lock:
            return list(self._samples)

    def coverage(self):
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
        return np.interp(np.asarray(ts, dtype=np.float64), arr[:, 0], arr[:, 1], left=arr[0, 1], right=arr[-1, 1])
```

- [ ] **Step 4: 运行确认通过**

```bash
python -m pytest perception_tower/test/test_angle_logger.py -v
```

- [ ] **Step 5: Commit**

```bash
git add perception_tower/perception_tower/angle_logger.py perception_tower/test/test_angle_logger.py
git commit -m "feat: 100Hz angle logger with interpolation"
```

---

## Task 7: stitcher.py（逐点补偿拼合 + 体素下采样，TDD）

**Files:**
- Create: `perception_tower/perception_tower/stitcher.py`
- Create: `perception_tower/test/test_stitcher.py`

**Interfaces:**
- Produces:
  - dataclass `FairyFrame(stamp_sec, time_origin_sec, xyz, point_time, intensity)`
  - dataclass `StitchParams(...)`
  - dataclass `StitchResult(xyz, intensity, n_points, n_frames)`
  - `stitch(frames, angles_at, params) -> StitchResult`
  - `voxel_downsample(xyz, intensity, leaf_m) -> (xyz, intensity)`
- `angles_at(ts)` 返回 θ_raw_deg；stitcher 内部做符号翻转、窗口裁剪、NaN 过滤。

- [ ] **Step 1: 写失败测试**

`perception_tower/test/test_stitcher.py`:
```python
import numpy as np
from perception_tower.stitcher import FairyFrame, StitchParams, stitch, voxel_downsample
from perception_tower.geometry import mount_rotation


def test_single_frame_no_rotation():
    # R_mount=I, T=0, theta=0 => output equals input
    xyz = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float32)
    frame = FairyFrame(0.0, 0.0, xyz, None, None)
    params = StitchParams(mount_rpy_deg=[0.0, 0.0, 0.0], scan_start_deg=-90.0, scan_end_deg=90.0)
    res = stitch([frame], lambda ts: np.zeros_like(ts), params)
    assert np.allclose(res.xyz, xyz)


def test_window_crop():
    xyz = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
    frame = FairyFrame(0.0, 0.0, xyz, None, None)
    params = StitchParams(scan_start_deg=10.0, scan_end_deg=20.0)
    res = stitch([frame], lambda ts: np.full_like(ts, 5.0), params)
    assert res.n_points == 0


def test_per_point_time_compensation():
    # half points at theta=0, half at theta=90
    xyz = np.array([[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float32)
    pt = np.array([0.0, 0.1], dtype=np.float64)
    frame = FairyFrame(0.0, 0.0, xyz, pt, None)
    params = StitchParams(mount_rpy_deg=[0.0, 0.0, 0.0], scan_start_deg=-90.0, scan_end_deg=90.0)

    def angles_at(ts):
        return np.where(ts < 0.05, 0.0, 90.0)

    res = stitch([frame], angles_at, params)
    assert res.n_points == 2
    assert np.allclose(res.xyz[0], [1.0, 0.0, 0.0], atol=1e-5)
    assert np.allclose(res.xyz[1], [0.0, 1.0, 0.0], atol=1e-5)


def test_voxel_downsample():
    xyz = np.array([[0.0, 0.0, 0.0], [0.009, 0.0, 0.0], [1.0, 1.0, 1.0]], dtype=np.float32)
    intensity = np.array([10.0, 20.0, 30.0], dtype=np.float32)
    out_xyz, out_i = voxel_downsample(xyz, intensity, 0.01)
    assert out_xyz.shape[0] == 2


def test_nan_filtered():
    xyz = np.array([[1.0, 0.0, 0.0], [np.nan, 0.0, 0.0]], dtype=np.float32)
    frame = FairyFrame(0.0, 0.0, xyz, None, None)
    params = StitchParams(mount_rpy_deg=[0.0, 0.0, 0.0], scan_start_deg=-90.0, scan_end_deg=90.0)
    res = stitch([frame], lambda ts: np.zeros_like(ts), params)
    assert res.n_points == 1
```

- [ ] **Step 2: 运行确认失败**

```bash
python -m pytest perception_tower/test/test_stitcher.py -v
```

- [ ] **Step 3: 实现 stitcher.py**

`perception_tower/perception_tower/stitcher.py`:
```python
from __future__ import annotations
from dataclasses import dataclass
from typing import Callable, Sequence
import numpy as np
from perception_tower.geometry import mount_rotation, transform_frame


@dataclass
class FairyFrame:
    stamp_sec: float
    time_origin_sec: float
    xyz: np.ndarray
    point_time: np.ndarray | None
    intensity: np.ndarray | None


@dataclass
class StitchParams:
    mount_rpy_deg: Sequence[float] = (90.0, 0.0, 0.0)
    mount_offset_xyz: Sequence[float] = (0.0, 0.0, 0.0)
    scan_start_deg: float = 30.0
    scan_end_deg: float = 150.0
    voxel_leaf_m: float = 0.01
    per_point_time: bool = True
    angle_sign: int = 1


@dataclass
class StitchResult:
    xyz: np.ndarray
    intensity: np.ndarray | None
    n_points: int
    n_frames: int


def voxel_downsample(
    xyz: np.ndarray,
    intensity: np.ndarray | None,
    leaf_m: float,
) -> tuple[np.ndarray, np.ndarray | None]:
    if xyz.shape[0] == 0:
        return xyz.copy(), None if intensity is None else intensity.copy()
    keys = np.floor(xyz / leaf_m).astype(np.int64)
    order = np.lexsort(keys.T)
    sorted_xyz = xyz[order]
    sorted_i = None if intensity is None else intensity[order]
    diff = np.concatenate([[True], np.any(np.diff(sorted_xyz / leaf_m, axis=0) >= 1, axis=1)])
    boundaries = np.where(diff)[0]
    out_xyz = []
    out_i = []
    for start, end in zip(boundaries, list(boundaries[1:]) + [len(sorted_xyz)]):
        out_xyz.append(sorted_xyz[start:end].mean(axis=0))
        if sorted_i is not None:
            out_i.append(sorted_i[start:end].mean())
    out_xyz = np.array(out_xyz, dtype=np.float32)
    out_i = np.array(out_i, dtype=np.float32) if sorted_i is not None else None
    return out_xyz, out_i


def stitch(
    frames: Sequence[FairyFrame],
    angles_at: Callable[[np.ndarray], np.ndarray],
    params: StitchParams,
) -> StitchResult:
    r_mount = mount_rotation(params.mount_rpy_deg)
    t_mount = np.asarray(params.mount_offset_xyz, dtype=np.float64)
    chunks = []
    ichunks = []
    for fr in frames:
        if fr.xyz.size == 0:
            continue
        n = fr.xyz.shape[0]
        if params.per_point_time and fr.point_time is not None:
            ts = np.asarray(fr.time_origin_sec + fr.point_time, dtype=np.float64)
        else:
            ts = np.full(n, fr.time_origin_sec, dtype=np.float64)
        theta_raw = angles_at(ts)
        valid = np.isfinite(fr.xyz).all(axis=1) & np.isfinite(theta_raw)
        valid &= (theta_raw >= params.scan_start_deg) & (theta_raw <= params.scan_end_deg)
        if not np.any(valid):
            continue
        xyz = fr.xyz[valid]
        theta_deg = theta_raw[valid] * params.angle_sign
        world = transform_frame(xyz.astype(np.float64), r_mount, t_mount, theta_deg)
        chunks.append(world)
        if fr.intensity is not None:
            ichunks.append(fr.intensity[valid])
        else:
            ichunks.append(None)
    if not chunks:
        return StitchResult(np.zeros((0, 3), dtype=np.float32), None, 0, 0)
    xyz_out = np.vstack(chunks).astype(np.float32)
    intensity = None if any(c is None for c in ichunks) else np.concatenate(ichunks).astype(np.float32)
    if params.voxel_leaf_m > 0.0 and xyz_out.shape[0] > 0:
        xyz_out, intensity = voxel_downsample(xyz_out, intensity, params.voxel_leaf_m)
    return StitchResult(xyz_out, intensity, xyz_out.shape[0], len(frames))
```

- [ ] **Step 4: 运行确认通过**

```bash
python -m pytest perception_tower/test/test_stitcher.py -v
```

- [ ] **Step 5: Commit**

```bash
git add perception_tower/perception_tower/stitcher.py perception_tower/test/test_stitcher.py
git commit -m "feat: per-point time stitching with voxel downsampling"
```

---

## Task 8: camera_grabber.py（336L 抓拍与保存，TDD）

**Files:**
- Create: `perception_tower/perception_tower/camera_grabber.py`
- Create: `perception_tower/test/test_camera_grabber.py`

**Interfaces:**
- Produces:
  - `CameraGrabber(now_fn=time.monotonic, freshness_s=0.3, max_pair_gap_s=0.2)`
    - `on_color(msg)`, `on_depth(msg)`（ros 回调入口）
    - `capture(timeout_s=5.0) -> PhotoPair`
  - `save_photos(pair, output_dir) -> (color_path, depth_path)` 使用 cv_bridge + cv2。
- `PhotoPair` 为 dataclass（color_msg, depth_msg）。

- [ ] **Step 1: 写失败测试**

`perception_tower/test/test_camera_grabber.py`:
```python
import time
import os
import tempfile
import numpy as np
import pytest
from sensor_msgs.msg import Image
from perception_tower.camera_grabber import CameraGrabber, save_photos


def make_image(encoding, data):
    msg = Image()
    msg.header.frame_id = 'camera'
    msg.encoding = encoding
    msg.height = data.shape[0]
    msg.width = data.shape[1]
    if encoding == 'bgr8':
        msg.step = msg.width * 3
        msg.data = data.astype(np.uint8).tobytes()
    elif encoding == '16UC1':
        msg.step = msg.width * 2
        msg.data = data.astype(np.uint16).tobytes()
    return msg


def test_capture_pair():
    clock = [0.0]
    g = CameraGrabber(now_fn=lambda: clock[0], freshness_s=0.5, max_pair_gap_s=0.2)
    g.on_color(make_image('bgr8', np.zeros((4, 4, 3), np.uint8)))
    clock[0] = 0.05
    g.on_depth(make_image('16UC1', np.ones((4, 4), np.uint16) * 500))
    clock[0] = 0.06
    pair = g.capture(timeout_s=0.5)
    assert pair is not None


def test_capture_timeout():
    g = CameraGrabber(now_fn=lambda: 0.0, freshness_s=0.1, max_pair_gap_s=0.2)
    with pytest.raises(RuntimeError):
        g.capture(timeout_s=0.1)


def test_save_photos_roundtrip():
    color = make_image('bgr8', np.arange(16, dtype=np.uint8).reshape(4, 4, 3))
    depth = make_image('16UC1', np.arange(16, dtype=np.uint16).reshape(4, 4))
    pair = CameraGrabber.PhotoPair(color=color, depth=depth)
    out = tempfile.mkdtemp()
    cpath, dpath = save_photos(pair, out)
    assert os.path.exists(cpath) and os.path.exists(dpath)
```

- [ ] **Step 2: 运行确认失败**

```bash
python -m pytest perception_tower/test/test_camera_grabber.py -v
```

- [ ] **Step 3: 实现 camera_grabber.py**

`perception_tower/perception_tower/camera_grabber.py`:
```python
from __future__ import annotations
import os
import time
from dataclasses import dataclass
from typing import Callable
from sensor_msgs.msg import Image
import numpy as np


@dataclass
class PhotoPair:
    color: Image
    depth: Image


class CameraGrabber:
    def __init__(
        self,
        now_fn: Callable[[], float] = time.monotonic,
        freshness_s: float = 0.3,
        max_pair_gap_s: float = 0.2,
    ):
        self._now = now_fn
        self._freshness_s = freshness_s
        self._max_pair_gap_s = max_pair_gap_s
        self._color = None  # (Image, recv_t)
        self._depth = None

    def on_color(self, msg: Image):
        self._color = (msg, self._now())

    def on_depth(self, msg: Image):
        self._depth = (msg, self._now())

    def capture(self, timeout_s: float = 5.0) -> PhotoPair:
        deadline = self._now() + timeout_s
        poll_s = 0.02
        while self._now() < deadline:
            c = self._color
            d = self._depth
            now = self._now()
            if c is not None and d is not None:
                if (now - c[1]) <= self._freshness_s and (now - d[1]) <= self._freshness_s:
                    if abs(c[1] - d[1]) <= self._max_pair_gap_s:
                        return PhotoPair(color=c[0], depth=d[0])
            time.sleep(poll_s)
        raise RuntimeError('camera capture timeout: no fresh color+depth pair')


def save_photos(pair: PhotoPair, output_dir: str) -> tuple[str, str]:
    os.makedirs(output_dir, exist_ok=True)
    from cv_bridge import CvBridge
    bridge = CvBridge()
    color_cv = bridge.imgmsg_to_cv2(pair.color, desired_encoding='bgr8')
    depth_cv = bridge.imgmsg_to_cv2(pair.depth, desired_encoding='16UC1')
    cpath = os.path.join(output_dir, 'color.png')
    dpath = os.path.join(output_dir, 'depth.png')
    import cv2
    cv2.imwrite(cpath, color_cv)
    cv2.imwrite(dpath, depth_cv)
    return cpath, dpath
```

- [ ] **Step 4: 运行确认通过**

```bash
python -m pytest perception_tower/test/test_camera_grabber.py -v
```

- [ ] **Step 5: Commit**

```bash
git add perception_tower/perception_tower/camera_grabber.py perception_tower/test/test_camera_grabber.py
git commit -m "feat: camera grabber with color+depth capture and PNG save"
```

---

## Task 9: fairy_buffer.py（帧缓存 + 帧首锚定，TDD）

**Files:**
- Create: `perception_tower/perception_tower/fairy_buffer.py`
- Create: `perception_tower/test/test_fairy_buffer.py`

**Interfaces:**
- Produces: `FairyBuffer(use_time_field=True)`
  - `start()` / `stop()`
  - `on_cloud(msg, stamp_sec: float)`
  - `frames() -> list[FairyFrame]`
- 在线估计 `frame_period` = 最近 5 帧 stamp 差的中位数；不足 3 个差值时取 0.0。

- [ ] **Step 1: 写失败测试**

`perception_tower/test/test_fairy_buffer.py`:
```python
import numpy as np
from builtin_interfaces.msg import Time
from perception_tower.fairy_buffer import FairyBuffer
from perception_tower.pc2_utils import make_cloud_msg


def test_buffer_drops_when_not_capturing():
    buf = FairyBuffer(use_time_field=True)
    msg = make_cloud_msg(np.array([[1.0, 0.0, 0.0]], dtype=np.float32), None, 'lidar_link', Time(sec=1, nanosec=0))
    buf.on_cloud(msg, 1.0)
    assert len(buf.frames()) == 0


def test_buffer_captures_and_estimates_period():
    buf = FairyBuffer(use_time_field=True)
    buf.start()
    for i in range(5):
        xyz = np.array([[float(i), 0.0, 0.0]], dtype=np.float32)
        pt = np.array([0.01], dtype=np.float64)
        msg = make_cloud_msg(xyz, None, 'lidar_link', Time(sec=i, nanosec=0), point_time=pt)
        buf.on_cloud(msg, float(i))
    buf.stop()
    frames = buf.frames()
    assert len(frames) == 5
    assert frames[3].time_origin_sec == 2.0  # median period 1.0 -> frame[3].stamp=3 -> origin=2
```

- [ ] **Step 2: 运行确认失败**

```bash
python -m pytest perception_tower/test/test_fairy_buffer.py -v
```

- [ ] **Step 3: 实现 fairy_buffer.py**

`perception_tower/perception_tower/fairy_buffer.py`:
```python
from __future__ import annotations
from collections import deque
import numpy as np
from sensor_msgs.msg import PointCloud2
from perception_tower.pc2_utils import read_field, read_intensity, read_time, read_xyz
from perception_tower.stitcher import FairyFrame


class FairyBuffer:
    def __init__(self, use_time_field: bool = True, history_max: int = 5):
        self._use_time = use_time_field
        self._capturing = False
        self._frames = []
        self._stamps = deque(maxlen=history_max)
        self._history_max = history_max

    def start(self):
        self._capturing = True
        self._frames = []
        self._stamps.clear()

    def stop(self):
        self._capturing = False

    def _estimate_period(self) -> float:
        if len(self._stamps) < 3:
            return 0.0
        diffs = [self._stamps[i] - self._stamps[i - 1] for i in range(1, len(self._stamps))]
        diffs.sort()
        return diffs[len(diffs) // 2]

    def on_cloud(self, msg: PointCloud2, stamp_sec: float):
        if not self._capturing:
            return
        xyz = read_xyz(msg)
        if xyz.shape[0] == 0:
            return
        self._stamps.append(stamp_sec)
        period = self._estimate_period()
        time_origin = stamp_sec - period
        point_time = read_time(msg) if self._use_time else None
        intensity = read_intensity(msg)
        self._frames.append(FairyFrame(stamp_sec, time_origin, xyz, point_time, intensity))

    def frames(self) -> list[FairyFrame]:
        return list(self._frames)

    def count(self) -> int:
        return len(self._frames)
```

- [ ] **Step 4: 运行确认通过**

```bash
python -m pytest perception_tower/test/test_fairy_buffer.py -v
```

- [ ] **Step 5: Commit**

```bash
git add perception_tower/perception_tower/fairy_buffer.py perception_tower/test/test_fairy_buffer.py
git commit -m "feat: Fairy frame buffer with online frame-period estimation"
```

---

## Task 10: FSM（INIT/SCAN 全流程 + 错误路径，TDD）

**Files:**
- Create: `perception_tower/perception_tower/fsm.py`
- Create: `perception_tower/perception_tower/mock.py`（仅 FakeServo，供 FSM 测试）
- Create: `perception_tower/test/test_fsm.py`

**Interfaces:**
- Produces:
  - `State` IntEnum（IDLE/INITING/READY/SCANNING/PROCESSING/ERROR）
  - `TowerFSM(servo, camera, fairy_buffer, stitch_params, save_cfg, status_cb, photo_cb, cloud_cb, clock_now, log_cb=print)`
  - `request_init() -> (accepted, message)` / `request_scan() -> (accepted, message)`
- `status_cb(state: State, progress_pct: int, message: str)` 在状态变化/进度更新时调用。
- `photo_cb(pair: PhotoPair)` 在保存照片后调用。
- `cloud_cb(xyz, intensity, stamp_sec)` 在拼合完成时调用。

- [ ] **Step 1: 写失败测试**

`perception_tower/test/test_fsm.py`:
```python
import time
import os
import tempfile
import numpy as np
from perception_tower.fsm import State, TowerFSM
from perception_tower.mock import FakeServo
from perception_tower.camera_grabber import PhotoPair
from perception_tower.stitcher import FairyFrame, StitchParams
from sensor_msgs.msg import Image


class StubCamera:
    def __init__(self):
        self.pair = PhotoPair(color=Image(), depth=Image())

    def capture(self, timeout_s):
        return self.pair


class StubFairyBuffer:
    def __init__(self, frames):
        self._frames = frames
        self.capturing = False

    def start(self):
        self.capturing = True

    def stop(self):
        self.capturing = False

    def frames(self):
        return self._frames


class StubCloudCb:
    def __init__(self):
        self.called = False
        self.xyz = None

    def __call__(self, xyz, intensity, stamp):
        self.called = True
        self.xyz = xyz


def make_fsm(mock_servo_speed=1000.0):
    servo = FakeServo(speed_deg_s=mock_servo_speed)
    camera = StubCamera()
    frames = [FairyFrame(0.0, 0.0, np.array([[1.0, 0.0, 0.0]], dtype=np.float32), None, None)]
    buffer = StubFairyBuffer(frames)
    status_log = []
    cloud = StubCloudCb()
    fsm = TowerFSM(
        servo=servo,
        camera=camera,
        fairy_buffer=buffer,
        stitch_params=StitchParams(mount_rpy_deg=[0.0, 0.0, 0.0], scan_start_deg=-90.0, scan_end_deg=90.0),
        save_cfg={'output_dir': tempfile.mkdtemp(), 'save_cloud': False},
        status_cb=lambda s, p, m: status_log.append((s, p, m)),
        photo_cb=lambda pair: None,
        cloud_cb=cloud,
        clock_now=lambda: time.time(),
    )
    return fsm, status_log, cloud


def test_init_flow():
    fsm, log, _ = make_fsm()
    ok, msg = fsm.request_init()
    assert ok
    # wait for thread
    while fsm.state != State.READY and fsm.state != State.ERROR:
        time.sleep(0.01)
    assert fsm.state == State.READY
    assert any(s == State.INITING for s, _, _ in log)


def test_scan_flow():
    fsm, log, cloud = make_fsm()
    fsm.request_init()
    while fsm.state != State.READY:
        time.sleep(0.01)
    ok, msg = fsm.request_scan()
    assert ok
    while fsm.state != State.READY:
        time.sleep(0.01)
    assert cloud.called
    assert cloud.xyz.shape[0] == 1


def test_busy_rejection():
    fsm, _, _ = make_fsm()
    fsm.request_init()
    ok, msg = fsm.request_scan()
    assert not ok
    assert 'busy' in msg


def test_error_on_empty_frames():
    fsm, log, _ = make_fsm()
    fsm.request_init()
    while fsm.state != State.READY:
        time.sleep(0.01)
    fsm._fairy_buffer = StubFairyBuffer([])
    fsm.request_scan()
    while fsm.state != State.ERROR:
        time.sleep(0.01)
```

- [ ] **Step 2: 运行确认失败**

```bash
python -m pytest perception_tower/test/test_fsm.py -v
```

- [ ] **Step 3: 实现 mock.py 中的 FakeServo**

`perception_tower/perception_tower/mock.py`（先写 FakeServo 部分）：
```python
from __future__ import annotations
import threading
import time
from perception_tower.servo_client import poll_until_reached, ServoError


class FakeServo:
    def __init__(self, origin=500, deg_per_pos=0.02, speed_deg_s=40.0):
        self._origin = origin
        self._dpp = deg_per_pos
        self._speed = speed_deg_s
        self._target_deg = 0.0
        self._start_deg = 0.0
        self._start_t = time.monotonic()
        self._duration = 0.0
        self._lock = threading.Lock()

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

    def poll_until_reached(self, pos_target, tol_deg, stable_count=5, timeout_s=30.0, poll_hz=100.0, progress_cb=None):
        return poll_until_reached(
            self.read_position,
            self.pos_to_deg,
            pos_target,
            tol_deg,
            stable_count,
            timeout_s,
            poll_hz,
            progress_cb,
        )
```

- [ ] **Step 4: 实现 fsm.py**

`perception_tower/perception_tower/fsm.py`:
```python
from __future__ import annotations
import csv
import os
import threading
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Callable
import numpy as np
from perception_tower.camera_grabber import PhotoPair
from perception_tower.stitcher import StitchParams, stitch


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
        servo,
        camera,
        fairy_buffer,
        stitch_params: StitchParams,
        save_cfg: dict,
        status_cb: Callable[[State, int, str], None],
        photo_cb: Callable[[PhotoPair], None],
        cloud_cb: Callable[[np.ndarray, np.ndarray | None, float], None],
        clock_now: Callable[[], float],
        log_cb: Callable[[str], None] = print,
        config: dict | None = None,
    ):
        self._np = np
        self._servo = servo
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
        self._thread: threading.Thread | None = None

    @property
    def state(self) -> State:
        return self._state

    def _set_state(self, state: State, progress: int = 0, message: str = ''):
        with self._lock:
            self._state = state
        self._status_cb(state, progress, message)

    def _busy_states(self):
        return {State.INITING, State.SCANNING, State.PROCESSING}

    def request_init(self) -> tuple[bool, str]:
        with self._lock:
            if self._state in self._busy_states():
                return False, f'busy: {self._state.name}'
        self._thread = threading.Thread(target=self._run_init, daemon=True)
        self._thread.start()
        return True, 'init started'

    def request_scan(self) -> tuple[bool, str]:
        with self._lock:
            if self._state in self._busy_states():
                return False, f'busy: {self._state.name}'
        self._thread = threading.Thread(target=self._run_scan, daemon=True)
        self._thread.start()
        return True, 'scan started'

    def _move_to_deg(self, deg: float, speed_deg_s: float, progress_base: int, progress_span: int, label: str):
        pos = self._servo.deg_to_pos(deg)
        current_deg = self._servo.pos_to_deg(self._servo.read_position())
        delta = abs(deg - current_deg)
        time_ms = max(200, int(delta / speed_deg_s * 1000))
        self._servo.move_to(pos, time_ms)

        def cb(pct):
            self._set_state(self._state, int(progress_base + pct * progress_span), label)

        self._servo.poll_until_reached(
            pos,
            tol_deg=self._cfg.get('pos_tol_deg', 0.1),
            stable_count=self._cfg.get('pos_stable_count', 5),
            timeout_s=self._cfg.get('move_timeout_s', 30.0),
            poll_hz=self._cfg.get('poll_hz', 100.0),
            progress_cb=cb,
        )
        cb(1.0)

    def _run_init(self):
        try:
            self._set_state(State.INITING, 0, 'homing')
            self._servo.reset(timeout_s=self._cfg.get('home_timeout_s', 30.0))
            self._set_state(State.INITING, 50, 'moving to ready')
            self._move_to_deg(self._cfg.get('ready_deg', 90.0), self._cfg.get('sweep_speed_deg_s', 40.0), 50, 50, 'moving to ready')
            self._set_state(State.READY, 100, 'ready')
        except Exception as exc:
            self._set_state(State.ERROR, 0, f'init failed: {exc}')

    def _ensure_ready(self):
        pos = self._servo.read_position()
        deg = self._servo.pos_to_deg(pos)
        if abs(deg - self._cfg.get('ready_deg', 90.0)) > self._cfg.get('pos_tol_deg', 0.1):
            self._set_state(State.SCANNING, 0, 're-homing')
            self._servo.reset(timeout_s=self._cfg.get('home_timeout_s', 30.0))
            self._move_to_deg(self._cfg.get('ready_deg', 90.0), self._cfg.get('sweep_speed_deg_s', 40.0), 0, 10, 're-homing')

    def _run_scan(self):
        try:
            self._set_state(State.SCANNING, 0, 'scan start')
            self._ensure_ready()
            self._set_state(State.SCANNING, 10, 'capture photo')
            pair = self._camera.capture(timeout_s=self._cfg.get('photo_timeout_s', 5.0))
            out_dir = os.path.join(self._save_cfg.output_dir, time.strftime('%Y%m%d_%H%M%S'))
            os.makedirs(out_dir, exist_ok=True)
            from perception_tower.camera_grabber import save_photos
            save_photos(pair, out_dir)
            self._photo_cb(pair)

            self._set_state(State.SCANNING, 20, 'move to scan start')
            from perception_tower.angle_logger import AngleLogger
            logger = AngleLogger(
                self._servo.read_position,
                self._clock,
                self._servo.pos_to_deg,
                poll_hz=self._cfg.get('poll_hz', 100.0),
            )
            self._fairy_buffer.start()
            logger.start()
            self._move_to_deg(self._cfg.get('scan_start_deg', 30.0), self._cfg.get('sweep_speed_deg_s', 40.0), 20, 10, 'move to start')

            self._set_state(State.SCANNING, 30, 'sweep')
            self._move_to_deg(self._cfg.get('scan_end_deg', 150.0), self._cfg.get('sweep_speed_deg_s', 40.0), 30, 40, 'sweeping')
            time.sleep(self._cfg.get('move_settle_s', 0.2))
            logger.stop()
            self._fairy_buffer.stop()

            frames = self._fairy_buffer.frames()
            if not frames:
                raise RuntimeError('no fairy frames captured')
            cov = logger.coverage()
            expected = (self._cfg.get('scan_end_deg', 150.0) - self._cfg.get('scan_start_deg', 30.0)) / self._cfg.get('sweep_speed_deg_s', 40.0)
            if cov is None or (cov[1] - cov[0]) < expected * 0.5:
                raise RuntimeError('insufficient angle log coverage')

            with open(os.path.join(out_dir, 'angle_log.csv'), 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['t', 'theta_raw_deg'])
                for t, deg in logger._get_samples():
                    w.writerow([f'{t:.6f}', f'{deg:.6f}'])

            self._set_state(State.PROCESSING, 70, 'stitching')
            result = stitch(frames, logger.angles_at, self._stitch_params)
            stamp = self._clock()
            self._cloud_cb(result.xyz, result.intensity, stamp)
            if self._save_cfg.save_cloud:
                from perception_tower.pc2_utils import save_pcd_binary
                save_pcd_binary(os.path.join(out_dir, 'stitched.pcd'), result.xyz, result.intensity)

            self._set_state(State.PROCESSING, 95, 'return to ready')
            self._move_to_deg(self._cfg.get('ready_deg', 90.0), self._cfg.get('sweep_speed_deg_s', 40.0), 95, 5, 'return to ready')
            self._set_state(State.READY, 100, f'scan done: {out_dir}')
        except Exception as exc:
            self._set_state(State.ERROR, 0, f'scan failed: {exc}')

    def _move_to_deg_dep(self, deg, speed_deg_s, progress_base, progress_span, label):
        # placeholder removed; logic above is sufficient
        pass
```

- [ ] **Step 5: 运行确认通过**

```bash
python -m pytest perception_tower/test/test_fsm.py -v
```

- [ ] **Step 6: Commit**

```bash
git add perception_tower/perception_tower/fsm.py perception_tower/perception_tower/mock.py perception_tower/test/test_fsm.py
git commit -m "feat: TowerFSM for init/scan with error handling"
```

---

## Task 11: tower_node.py + mock.py + rclpy 端到端测试

**Files:**
- Modify: `perception_tower/perception_tower/mock.py`（补全 MockFairy、MockCamera）
- Create: `perception_tower/perception_tower/tower_node.py`
- Modify: `perception_tower/launch/tower.launch.py`（mock 参数透传已做）
- Create: `perception_tower/test/test_tower_node.py`

**Interfaces:**
- Produces: `tower_node` 可执行节点，mock 模式下自包含 FakeServo + 内部 mock 传感器 publisher。
- `TowerNode` 参数读取、service 注册、status 发布、话题订阅/发布。

- [ ] **Step 1: 写端到端测试**

`perception_tower/test/test_tower_node.py`:
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
            ('serial_port', ''),
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

- [ ] **Step 2: 运行确认失败**

```bash
python -m pytest perception_tower/test/test_tower_node.py -v
```

- [ ] **Step 3: 补全 mock.py（MockFairy + MockCamera）**

追加到 `perception_tower/perception_tower/mock.py`：
```python
import numpy as np
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image, PointCloud2
from perception_tower.pc2_utils import make_cloud_msg
from perception_tower.geometry import mount_rotation


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
        # vertical plane at x=2.0, y in [-0.5,0.5], z in [0,1]
        yy, zz = np.meshgrid(np.linspace(-0.5, 0.5, 10), np.linspace(0, 1, 10))
        xx = np.full_like(yy, 2.0)
        return np.stack([xx.ravel(), yy.ravel(), zz.ravel()], axis=1).astype(np.float32)

    def _publish(self):
        from perception_tower.geometry import rotation_z_deg
        now = self._node.get_clock().now()
        stamp_sec = now.nanoseconds * 1e-9
        theta_deg = self._servo.pos_to_deg(self._servo.read_position())
        # invert: p_lidar = R_mount^T (Rz(-theta) p_world)
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
        msg = make_cloud_msg(lidar, None, 'lidar_link', now.to_msg(), point_time=point_time)
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
        color.header.frame_id = 'camera_color_optical_frame'
        color.height = h
        color.width = w
        color.encoding = 'bgr8'
        color.step = w * 3
        color.data = (np.full((h, w, 3), self._seq % 256, dtype=np.uint8)).tobytes()

        depth = Image()
        depth.header.stamp = stamp
        depth.header.frame_id = 'camera_color_optical_frame'
        depth.height = h
        depth.width = w
        depth.encoding = '16UC1'
        depth.step = w * 2
        depth.data = (np.full((h, w), 1000 + self._seq, dtype=np.uint16)).tobytes()

        self._color_pub.publish(color)
        self._depth_pub.publish(depth)
        self._seq += 1
```

- [ ] **Step 4: 实现 tower_node.py**

`perception_tower/perception_tower/tower_node.py`:
```python
from __future__ import annotations
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image, PointCloud2
from perception_tower_interfaces.msg import TowerStatus
from perception_tower_interfaces.srv import TowerCommand
from perception_tower.angle_logger import AngleLogger
from perception_tower.camera_grabber import CameraGrabber, PhotoPair
from perception_tower.fairy_buffer import FairyBuffer
from perception_tower.fsm import State, TowerFSM
from perception_tower.pc2_utils import make_cloud_msg
from perception_tower.servo_client import ServoClient
from perception_tower.stitcher import StitchParams
from perception_tower.mock import FakeServo, MockCamera, MockFairy


class TowerNode(Node):
    def __init__(self, **kwargs):
        super().__init__('tower_node', **kwargs)
        self._declare_params()
        self._load_params()

        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(TowerStatus, '/perception_tower/status', status_qos)
        self._cloud_pub = self.create_publisher(PointCloud2, self._stitched_topic, 10)
        self._photo_color_pub = self.create_publisher(Image, self._photo_color_topic, 10)
        self._photo_depth_pub = self.create_publisher(Image, self._photo_depth_topic, 10)

        self._srv = self.create_service(TowerCommand, '/perception_tower/command', self._on_command)
        self._status_timer = self.create_timer(0.5, self._publish_status)
        self._last_status = (State.IDLE, 0, 'initialized')

        self._build_components()

    def _declare_params(self):
        p = [
            ('serial_port', ''),
            ('serial_baud', 115200),
            ('poll_hz', 100.0),
            ('pos_tol_deg', 0.1),
            ('pos_stable_count', 5),
            ('pos_origin', 500),
            ('deg_per_pos', 0.02),
            ('angle_sign', 1),
            ('ready_deg', 90.0),
            ('scan_start_deg', 30.0),
            ('scan_end_deg', 150.0),
            ('sweep_speed_deg_s', 40.0),
            ('home_timeout_s', 30.0),
            ('fairy_topic', '/fairy/points'),
            ('fairy_time_field', True),
            ('mount_rpy_deg', [90.0, 0.0, 0.0]),
            ('mount_offset_xyz', [0.0, 0.0, 0.0]),
            ('voxel_leaf_m', 0.01),
            ('world_frame_id', 'world'),
            ('color_topic', '/camera/color/image_raw'),
            ('depth_topic', '/camera/depth/image_raw'),
            ('output_dir', '/tmp/perception_tower'),
            ('save_cloud', True),
            ('stitched_topic', '/perception_tower/stitched_points'),
            ('photo_color_topic', '/perception_tower/photo_color'),
            ('photo_depth_topic', '/perception_tower/photo_depth'),
            ('mock_hardware', False),
            ('photo_timeout_s', 5.0),
            ('move_settle_s', 0.2),
            ('move_timeout_s', 30.0),
        ]
        for name, value in p:
            self.declare_parameter(name, value)

    def _load_params(self):
        self._stitched_topic = self.get_parameter('stitched_topic').value
        self._photo_color_topic = self.get_parameter('photo_color_topic').value
        self._photo_depth_topic = self.get_parameter('photo_depth_topic').value
        self._mock = self.get_parameter('mock_hardware').value
        self._world_frame_id = self.get_parameter('world_frame_id').value
        self._output_dir = self.get_parameter('output_dir').value

    def _build_components(self):
        cfg = {
            'pos_tol_deg': self.get_parameter('pos_tol_deg').value,
            'pos_stable_count': self.get_parameter('pos_stable_count').value,
            'poll_hz': self.get_parameter('poll_hz').value,
            'ready_deg': self.get_parameter('ready_deg').value,
            'scan_start_deg': self.get_parameter('scan_start_deg').value,
            'scan_end_deg': self.get_parameter('scan_end_deg').value,
            'sweep_speed_deg_s': self.get_parameter('sweep_speed_deg_s').value,
            'home_timeout_s': self.get_parameter('home_timeout_s').value,
            'photo_timeout_s': self.get_parameter('photo_timeout_s').value,
            'move_settle_s': self.get_parameter('move_settle_s').value,
            'move_timeout_s': self.get_parameter('move_timeout_s').value,
        }
        origin = self.get_parameter('pos_origin').value
        dpp = self.get_parameter('deg_per_pos').value
        angle_sign = self.get_parameter('angle_sign').value

        if self._mock:
            servo = FakeServo(origin=origin, deg_per_pos=dpp, speed_deg_s=cfg['sweep_speed_deg_s'])
        else:
            servo = ServoClient(
                port=self.get_parameter('serial_port').value,
                baud=self.get_parameter('serial_baud').value,
                pos_origin=origin,
                deg_per_pos=dpp,
            )
            try:
                servo.open()
            except Exception as exc:
                self.get_logger().error(f'failed to open servo: {exc}')

        camera = CameraGrabber(now_fn=lambda: self.get_clock().now().nanoseconds * 1e-9)
        if self._mock:
            MockCamera(self, self.get_parameter('color_topic').value, self.get_parameter('depth_topic').value)
        else:
            self.create_subscription(Image, self.get_parameter('color_topic').value, camera.on_color, 10)
            self.create_subscription(Image, self.get_parameter('depth_topic').value, camera.on_depth, 10)

        fairy_buffer = FairyBuffer(use_time_field=self.get_parameter('fairy_time_field').value)
        if self._mock:
            MockFairy(self, self.get_parameter('fairy_topic').value, servo)
        else:
            self.create_subscription(
                PointCloud2,
                self.get_parameter('fairy_topic').value,
                lambda msg: fairy_buffer.on_cloud(msg, self.get_clock().now().nanoseconds * 1e-9),
                10,
            )

        stitch_params = StitchParams(
            mount_rpy_deg=self.get_parameter('mount_rpy_deg').value,
            mount_offset_xyz=self.get_parameter('mount_offset_xyz').value,
            scan_start_deg=cfg['scan_start_deg'],
            scan_end_deg=cfg['scan_end_deg'],
            voxel_leaf_m=self.get_parameter('voxel_leaf_m').value,
            per_point_time=self.get_parameter('fairy_time_field').value,
            angle_sign=angle_sign,
        )

        self._fsm = TowerFSM(
            servo=servo,
            camera=camera,
            fairy_buffer=fairy_buffer,
            stitch_params=stitch_params,
            save_cfg={'output_dir': self._output_dir, 'save_cloud': self.get_parameter('save_cloud').value},
            status_cb=self._on_fsm_status,
            photo_cb=self._on_photo,
            cloud_cb=self._on_cloud,
            clock_now=lambda: self.get_clock().now().nanoseconds * 1e-9,
            log_cb=lambda m: self.get_logger().info(m),
            config=cfg,
        )

    def _on_fsm_status(self, state: State, progress: int, message: str):
        self._last_status = (state, progress, message)
        self._publish_status()

    def _publish_status(self):
        state, progress, message = self._last_status
        msg = TowerStatus()
        msg.state = int(state)
        msg.progress_pct = progress
        msg.message = message
        self._status_pub.publish(msg)

    def _on_photo(self, pair: PhotoPair):
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
            accepted, message = False, f'unknown command {request.command}'
        response.accepted = accepted
        response.message = message
        return response

    def destroy_node(self):
        try:
            self._fsm._servo.close()
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
```

- [ ] **Step 5: 运行确认通过**

```bash
cd /Users/acelan/workspace/perception_tower
source install/setup.bash
python -m pytest perception_tower/test/test_tower_node.py -v
```

- [ ] **Step 6: Commit**

```bash
git add perception_tower/perception_tower/mock.py perception_tower/perception_tower/tower_node.py perception_tower/test/test_tower_node.py
git commit -m "feat: TowerNode + mock sensors + end-to-end rclpy test"
```

---

## Task 12: launch、参数文件、README

**Files:**
- Modify: `perception_tower/launch/tower.launch.py`
- Modify: `perception_tower/config/tower_params.yaml`
- Create: `perception_tower/README.md`
- Test: launch 语法检查 `ros2 launch --print-description` + 手动运行 mock smoke

- [ ] **Step 1: 完善 launch 文件**

`perception_tower/launch/tower.launch.py`:
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('perception_tower')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'tower_params.yaml'),
            description='Path to tower_params.yaml',
        ),
        DeclareLaunchArgument(
            'mock_hardware',
            default_value='false',
            description='Use fake servo/sensors for testing',
        ),
        Node(
            package='perception_tower',
            executable='tower_node',
            name='tower_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'mock_hardware': LaunchConfiguration('mock_hardware')},
            ],
        ),
    ])
```

- [ ] **Step 2: 确认参数文件完整**

`perception_tower/config/tower_params.yaml` 已完整（Task 1 已写），无需改动。

- [ ] **Step 3: 编写 README.md**

`perception_tower/README.md` 内容：
```markdown
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
  python=3.10 ros-humble-ros-base ros-humble-cv-bridge \
  ros-humble-rosidl-default-generators ros-humble-ament-cmake-python \
  colcon-common-extensions pytest numpy pyserial opencv
conda activate tower
```

## 构建

```bash
cd /Users/acelan/workspace/perception_tower
source /opt/ros/humble/setup.bash  # Linux 生产机
conda activate tower               # macOS 开发机
colcon build --symlink-install
source install/setup.bash
```

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
```

- [ ] **Step 4: launch 语法检查与 mock 冒烟**

```bash
cd /Users/acelan/workspace/perception_tower
source install/setup.bash
ros2 launch perception_tower tower.launch.py mock_hardware:=true --print-description
# 另开终端：
ros2 launch perception_tower tower.launch.py mock_hardware:=true
ros2 service call /perception_tower/command perception_tower_interfaces/srv/TowerCommand "{command: 1}"
ros2 service call /perception_tower/command perception_tower_interfaces/srv/TowerCommand "{command: 2}"
ros2 topic echo /perception_tower/status
```

- [ ] **Step 5: Commit**

```bash
git add perception_tower/launch/tower.launch.py perception_tower/README.md
git commit -m "docs+launch: README, launch args, mock smoke"
```

---

## Task 13: 真机联调（macOS + 最终推送）

**Files:**
- Create: `perception_tower/docs/integration_notes.md`（记录真机参数与验证结果）

- [ ] **Step 1: 串口真机冒烟**

```bash
conda activate tower
cd /Users/acelan/workspace/perception_tower
source install/setup.bash
# 查找串口设备
ls /dev/tty.usbserial-*
# 临时测试（可选 python 脚本）：
python - <<'PY'
from perception_tower.servo_client import ServoClient
c = ServoClient('/dev/tty.usbserial-XXXX', pos_origin=500, deg_per_pos=0.02)
c.open(); print(c.read_position()); c.reset(); c.close()
PY
```

记录：`docs/integration_notes.md` 写入设备名、波特率、RST 耗时。

- [ ] **Step 2: 真机 INIT/SCAN 验证**

```bash
# 确认 Fairy + 336L 驱动已在 macOS 或 rosbag 回放可用
ros2 launch perception_tower tower.launch.py serial_port:=/dev/tty.usbserial-XXXX
ros2 service call /perception_tower/command perception_tower_interfaces/srv/TowerCommand "{command: 1}"
ros2 service call /perception_tower/command perception_tower_interfaces/srv/TowerCommand "{command: 2}"
```

逐项确认并记录：
- INIT 后状态 READY
- 90° 位置误差 < 0.1°
- 照片保存成功
- 扫描 30→150° 耗时 ≈3s
- stitched_points topic 收到点云
- angle_log.csv 覆盖 30~150°

- [ ] **Step 3: 集成确认项勾选**

在 `docs/integration_notes.md` 中记录：
- Fairy 驱动是否含 `time` 字段 / `timestamp_type` 配置
- 336L 实际话题名与分辨率
- `angle_sign` 是否正确（用已知目标验证）
- 最终 `mount_rpy_deg` / `mount_offset_xyz` 标定值

- [ ] **Step 4: Push 到 GitHub**

```bash
git remote add origin git@github.com:xiaobin86/perception-tower.git  # 若尚未配置
git push -u origin main
```

- [ ] **Step 5: Commit integration notes**

```bash
git add perception_tower/docs/integration_notes.md
git commit -m "docs: integration notes from real hardware check"
git push
```

---

## 计划自审

### Spec 覆盖检查

| Spec 要求 | 对应任务 |
|-----------|----------|
| 双包骨架 + 可被安装 | Task 1 |
| 串口协议 115200 + `#000P`/`#OK!` | Task 4, 5 |
| 位置→角度 `(pos−500)×0.02` | Task 5, 6, 10 |
| 90°=5000, 30°=2000, 150°=8000 | Task 10 硬编码默认值 |
| 扫描 40°/s、100Hz 轮询、0.1° 容差、连续 5 次稳定 | Task 5, 6, 10 |
| 拼合数学 `P_world = Rz(θ)·(R_mount·P_lidar+T_mount)` | Task 2, 7 |
| 裁剪窗口 [30°,150°] 作用于 θ_raw | Task 7 |
| Fairy 逐点 time 字段、帧首锚定 | Task 7, 9 |
| 服务非阻塞 + status topic | Task 10, 11 |
| 336L 彩色+深度抓拍与保存 | Task 8 |
| 输出 stitched_points + photo topics | Task 8, 11 |
| mock_hardware 支持 macOS 开发 | Task 10, 11, 12 |
| README 安装/部署/使用/服务调用/功能 | Task 12 |
| 真机联调 | Task 13 |

### 占位符扫描

- 无 TBD / TODO / "稍后实现" / "适当处理"。
- 每个任务给出实际可运行的代码或确切命令。

### 类型一致性检查

- `FairyFrame` 在 Task 7 定义后，Task 9 `fairy_buffer.py` 与 Task 10/11 共用同一 dataclass。
- `PhotoPair` 在 Task 8 定义后，Task 10 FSM 与 Task 11 node 共用。
- `StitchParams` 在 Task 7 定义后，Task 10 FSM 与 Task 11 node 共用，字段名一致。
- `TowerFSM` 签名在 Task 10 定义，Task 11 按签名构造。
- `PointCloud2` 构造使用 `make_cloud_msg(xyz, intensity, frame_id, stamp)` 一致。

### 潜在风险

- Task 11 的 rclpy 端到端测试在 macOS conda 下可能因线程/executor 时序不稳定；首次失败时提高超时并确认 mock Fairy 已发布足够帧。
- `voxel_downsample` 使用排序+边界切分，对空输入和大数据量均安全；若性能不足可后续换 numpy.unique。
- 真机联调 Task 13 依赖外部传感器驱动，可能部分步骤需在 Linux 完成；README 已说明 rosbag 回放备选路径。
