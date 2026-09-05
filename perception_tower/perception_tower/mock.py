"""Mock hardware for ``mock_hardware:=true`` and offline integration tests.

FakeServo simulates the turntable firmware with a linear speed model. MockFairy
/ MockCamera are rclpy publishers that emit synthetic sensor data in mock mode
so the full INIT -> SCAN pipeline can run without real drivers.
"""

from __future__ import annotations

import threading
import time
from typing import List, Optional

import numpy as np

from .geometry import mount_rotation
from .servo_client import poll_until_reached, ServoError


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


try:
    from builtin_interfaces.msg import Time
    from sensor_msgs.msg import Image, PointCloud2

    _ROS_AVAILABLE = True
except Exception:  # pragma: no cover
    _ROS_AVAILABLE = False


if _ROS_AVAILABLE:

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
            from .pc2_utils import make_cloud_msg

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
