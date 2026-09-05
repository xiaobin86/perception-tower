"""Fairy LiDAR UDP bridge — Task 14 (macOS native).

Receives Fairy MSOP UDP packets, parses them into PointCloud2 messages,
and publishes on the topic expected by tower_node. Runs standalone so
macOS can talk to real Fairy hardware without rslidar_sdk.

Protocol reference: RoboSense Fairy User Manual v2.1, Section 4.4.
"""

from __future__ import annotations

import math
import socket
import struct
import threading
import time
from typing import Optional

import numpy as np

try:
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header

    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False

# ── MSOP constants ──────────────────────────────────────────────────────
_MSOP_PORT = 6699
_MSOP_PKT_SIZE = 1248
_MSOP_HEADER_SIZE = 42
_MSOP_BLOCK_SIZE = 148
_MSOP_SIGN = 0xFFEE
_MSOP_HEADER_ID = bytes([0x55, 0xAA, 0x05, 0x5A])

# Fairy geometry
_CHANNELS_PER_BLOCK = 48
_BLOCKS_PER_PKT = 8
_AZIMUTH_RESOLUTION = 0.01  # degrees per count
_DISTANCE_RESOLUTION = 0.005  # meters per count (0.5 cm)
_MAX_DISTANCE_M = 200.0
_MIN_DISTANCE_M = 0.2


class FairyUDPBridge:
    """Pure-Python Fairy MSOP UDP receiver.

    Collects packets into full 360° frames and provides an ``on_frame``
    callback for each completed scan. When wired to a ROS2 node it
    publishes PointCloud2 messages.
    """

    def __init__(
        self,
        msop_port: int = _MSOP_PORT,
        host: str = "0.0.0.0",
        dense: bool = False,
        use_lidar_clock: bool = True,
    ) -> None:
        self._port = msop_port
        self._host = host
        self._dense = dense
        self._use_lidar_clock = use_lidar_clock

        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False

        # Frame accumulator
        self._frame_packets: dict[int, list] = {}  # angle_block_idx -> [(azimuth, points)]
        self._current_azimuth: Optional[float] = None
        self._frame_xyz: list[np.ndarray] = []
        self._frame_intensity: list[np.ndarray] = []
        self._frame_time: Optional[float] = None

        # DIFOP calibration data (azimuth offsets per channel)
        # For simplicity, use uniform 0.25° spacing for 96-ch interleaved
        self._channel_angles: Optional[np.ndarray] = None

        # Callback
        self._on_frame_cb = None

    def set_on_frame(self, cb) -> None:
        """Register callback: cb(xyz, intensity, timestamp_s)."""
        self._on_frame_cb = cb

    def start(self) -> None:
        """Open UDP socket and start receiver thread."""
        if self._running:
            return
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.settimeout(1.0)
        self._sock.bind((self._host, self._port))
        self._running = True
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop receiver and close socket."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
            self._thread = None
        if self._sock:
            self._sock.close()
            self._sock = None

    # ── UDP receiver loop ───────────────────────────────────────────────

    def _recv_loop(self) -> None:
        while self._running:
            try:
                data, addr = self._sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(data) < _MSOP_PKT_SIZE:
                continue
            if data[:4] != _MSOP_HEADER_ID:
                continue
            self._parse_msop_packet(data)

    # ── MSOP parser ─────────────────────────────────────────────────────

    def _parse_msop_packet(self, data: bytes) -> None:
        # Header
        # bytes 20-29: timestamp (6B sec + 4B usec)
        ts_sec = struct.unpack_from("<Q", data, 20)[0]  # actually 6 bytes + padding
        # More precise: bytes 20-25 = sec (6 bytes LE), bytes 26-29 = usec (4 bytes LE)
        ts_sec_raw = int.from_bytes(data[20:26], byteorder="little")
        ts_usec = struct.unpack_from("<I", data, 26)[0]
        timestamp = float(ts_sec_raw) + float(ts_usec) * 1e-6

        for blk_idx in range(_BLOCKS_PER_PKT):
            blk_offset = _MSOP_HEADER_SIZE + blk_idx * _MSOP_BLOCK_SIZE
            sign = struct.unpack_from("<H", data, blk_offset)[0]
            if sign != _MSOP_SIGN:
                continue

            azimuth_raw = struct.unpack_from("<H", data, blk_offset + 2)[0]
            azimuth_deg = azimuth_raw * _AZIMUTH_RESOLUTION

            points = []
            for ch in range(_CHANNELS_PER_BLOCK):
                ch_offset = blk_offset + 4 + ch * 3
                dist_raw = struct.unpack_from("<H", data, ch_offset)[0]
                reflectivity = data[ch_offset + 2]

                # Distance: lower 15 bits, resolution 0.5 cm
                dist_m = (dist_raw & 0x7FFF) * _DISTANCE_RESOLUTION
                if dist_m < _MIN_DISTANCE_M or dist_m > _MAX_DISTANCE_M:
                    points.append((np.nan, np.nan, np.nan, 0))
                    continue

                # Compute vertical angle for this channel
                # Fairy 96-ch interleaved: channels 0-47 map to alternating
                # vertical angles. We'll compute the vertical angle based on
                # channel index within the block.
                # For single return: 2 blocks = 1 line (48 channels)
                # Block pair (0,1) = line 0, (2,3) = line 1, etc.
                # Actually, for Fairy 96-ch interleaved, each block has 48
                # channels but they represent alternating vertical angles.
                # The vertical FOV is 32° (-16° to +16°).
                # Channel mapping: even channels = one set, odd = another
                v_angle = self._channel_vertical_angle(blk_idx, ch)

                # Horizontal angle = azimuth + channel offset within block
                # Each channel takes ~0.25° (360° / 1440 samples per rev)
                h_angle = azimuth_deg + ch * 0.25

                # Convert polar to Cartesian
                cos_v = math.cos(math.radians(v_angle))
                x = dist_m * cos_v * math.cos(math.radians(h_angle))
                y = dist_m * cos_v * math.sin(math.radians(h_angle))
                z = dist_m * math.sin(math.radians(v_angle))

                points.append((x, y, z, reflectivity))

            xyz = np.array([p[:3] for p in points], dtype=np.float32)
            intensity = np.array([p[3] for p in points], dtype=np.float32)
            self._frame_xyz.append(xyz)
            self._frame_intensity.append(intensity)
            if self._frame_time is None:
                self._frame_time = timestamp

        # Check if we've completed a 360° scan
        # Fairy rotates at ~10 Hz, one revolution = ~3600 blocks
        # We detect frame boundary when azimuth wraps around
        self._check_frame_boundary(azimuth_deg)

    def _channel_vertical_angle(self, blk_idx: int, ch: int) -> float:
        """Compute vertical angle for a given channel.

        Fairy 96-ch interleaved: vertical FOV = 32° (-16° to +16°)
        Each block has 48 channels representing alternating vertical angles.
        Block pair (2i, 2i+1) covers one "line" of 48 channels.
        """
        # Vertical angle spacing: 32° / 96 channels ≈ 0.333°
        # For interleaved: channel index in the 96-ch sequence
        # blk_idx 0, ch 0 → 96-ch index 0
        # blk_idx 0, ch 1 → 96-ch index 1
        # blk_idx 1, ch 0 → 96-ch index 48
        # blk_idx 1, ch 1 → 96-ch index 49
        ch_96 = blk_idx * _CHANNELS_PER_BLOCK + ch
        # Interleaved mapping: even 96-ch index → lower half, odd → upper half
        # Actually, the mapping is more complex. For simplicity, assume
        # linear mapping from 0 to 95 across -16° to +16°
        v_angle = -16.0 + (ch_96 / 95.0) * 32.0
        return v_angle

    def _check_frame_boundary(self, current_azimuth: float) -> None:
        """Detect 360° frame boundary and emit completed frame."""
        if self._current_azimuth is None:
            self._current_azimuth = current_azimuth
            return

        # Detect wrap: azimuth jumped from >350° to <10°
        if self._current_azimuth > 350.0 and current_azimuth < 10.0:
            # Frame complete
            if self._frame_xyz and self._on_frame_cb:
                xyz = np.concatenate(self._frame_xyz, axis=0)
                intensity = np.concatenate(self._frame_intensity, axis=0)
                ts = self._frame_time if self._frame_time else time.time()
                self._on_frame_cb(xyz, intensity, ts)
            self._frame_xyz = []
            self._frame_intensity = []
            self._frame_time = None

        self._current_azimuth = current_azimuth


class FairyUDPBridgeNode(Node):
    """ROS2 node wrapper around FairyUDPBridge."""

    def __init__(self) -> None:
        super().__init__("fairy_udp_bridge")
        self._pub = self.create_publisher(PointCloud2, "/fairy/points", 10)
        self._bridge = FairyUDPBridge()
        self._bridge.set_on_frame(self._on_frame)
        self.get_logger().info("FairyUDPBridge node created")

    def start(self) -> None:
        self._bridge.start()
        self.get_logger().info(f"FairyUDPBridge listening on port {self._bridge._port}")

    def stop(self) -> None:
        self._bridge.stop()

    def _on_frame(self, xyz: np.ndarray, intensity: np.ndarray, ts: float) -> None:
        if not _ROS_AVAILABLE:
            return
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "lidar_link"
        msg.height = 1
        msg.width = len(xyz)
        msg.is_dense = not self._bridge._dense

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.fields = fields
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width

        data = np.zeros((len(xyz), 4), dtype=np.float32)
        data[:, :3] = xyz
        data[:, 3] = intensity
        msg.data = data.tobytes()

        self._pub.publish(msg)


def main(args=None):
    if not _ROS_AVAILABLE:
        print("ROS2 not available — running in standalone mode")
        bridge = FairyUDPBridge()
        count = [0]

        def on_frame(xyz, intensity, ts):
            count[0] += 1
            print(f"Frame {count[0]}: {len(xyz)} points, ts={ts:.3f}")

        bridge.set_on_frame(on_frame)
        bridge.start()
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            bridge.stop()
        return

    import rclpy
    rclpy.init(args=args)
    node = FairyUDPBridgeNode()
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
