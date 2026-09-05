"""Orbbec camera bridge — Task 14 (macOS native).

Uses pyorbbecsdk to capture color + depth frames from Orbbec Gemini 336L
and publishes as ROS2 Image messages. Falls back to OpenCV for testing
when Orbbec SDK is not available.

Requires: pip install pyorbbecsdk (ARM64 macOS) or opencv-python (fallback).
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np

try:
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import Header

    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False


class OrbbecCameraBridge:
    """Orbbec camera capture using pyorbbecsdk or OpenCV fallback."""

    def __init__(
        self,
        color_width: int = 1280,
        color_height: int = 720,
        color_fps: int = 30,
        depth_width: int = 848,
        depth_height: int = 480,
        depth_fps: int = 30,
    ) -> None:
        self._color_w = color_width
        self._color_h = color_height
        self._color_fps = color_fps
        self._depth_w = depth_width
        self._depth_h = depth_height
        self._depth_fps = depth_fps

        self._pipeline = None
        self._cap = None
        self._running = False
        self._thread: Optional[threading.Thread] = None

        self._color_cb = None
        self._depth_cb = None

        # Try pyorbbecsdk first
        self._backend = self._init_orbbec()
        if self._backend is None:
            self._backend = "opencv"
            self._init_opencv()

    def _init_orbbec(self) -> Optional[str]:
        try:
            from pyorbbecsdk import Config, Pipeline, OBSensorType, OBFormat

            config = Config()
            # Enable color stream
            config.enable_stream(
                OBSensorType.COLOR_STREAM,
                self._color_w,
                self._color_h,
                self._color_fps,
                OBFormat.RGB888,
            )
            # Enable depth stream
            config.enable_stream(
                OBSensorType.DEPTH_STREAM,
                self._depth_w,
                self._depth_h,
                self._depth_fps,
                OBFormat.U16_MM,
            )

            self._pipeline = Pipeline()
            self._pipeline.start(config)
            return "pyorbbecsdk"
        except Exception as e:
            print(f"pyorbbecsdk not available: {e}")
            return None

    def _init_opencv(self) -> None:
        try:
            import cv2

            self._cap = cv2.VideoCapture(0)
            if self._cap.isOpened():
                self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._color_w)
                self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._color_h)
                self._cap.set(cv2.CAP_PROP_FPS, self._color_fps)
                print("OpenCV fallback initialized")
            else:
                print("Warning: No camera available")
                self._cap = None
        except ImportError:
            print("Warning: Neither pyorbbecsdk nor opencv available")

    def set_callbacks(self, color_cb, depth_cb) -> None:
        """Register callbacks: cb(np.ndarray, encoding_str, timestamp)."""
        self._color_cb = color_cb
        self._depth_cb = depth_cb

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
            self._thread = None
        if self._pipeline:
            self._pipeline.stop()
            self._pipeline = None
        if self._cap:
            self._cap.release()
            self._cap = None

    def _capture_loop(self) -> None:
        while self._running:
            if self._backend == "pyorbbecsdk":
                self._capture_orbbec()
            else:
                self._capture_opencv()
            time.sleep(0.01)

    def _capture_orbbec(self) -> None:
        try:
            from pyorbbecsdk import OBSensorType

            frames = self._pipeline.wait_for_frames(100)
            if frames is None:
                return

            ts = time.time()

            color_frame = frames.get_color_frame()
            if color_frame and self._color_cb:
                data = color_frame.get_data()
                w = color_frame.get_width()
                h = color_frame.get_height()
                img = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
                self._color_cb(img, "rgb8", ts)

            depth_frame = frames.get_depth_frame()
            if depth_frame and self._depth_cb:
                data = depth_frame.get_data()
                w = depth_frame.get_width()
                h = depth_frame.get_height()
                img = np.frombuffer(data, dtype=np.uint16).reshape(h, w)
                self._depth_cb(img, "16UC1", ts)
        except Exception as e:
            print(f"Orbbec capture error: {e}")

    def _capture_opencv(self) -> None:
        if self._cap is None:
            time.sleep(1)
            return
        try:
            import cv2

            ret, frame = self._cap.read()
            if ret and self._color_cb:
                ts = time.time()
                # OpenCV returns BGR, convert to RGB
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                self._color_cb(rgb, "rgb8", ts)
        except Exception as e:
            print(f"OpenCV capture error: {e}")


class OrbbecBridgeNode(Node):
    """ROS2 node wrapper around OrbbecCameraBridge."""

    def __init__(self) -> None:
        super().__init__("orbbec_bridge")
        self._color_pub = self.create_publisher(Image, "/camera/color/image_raw", 10)
        self._depth_pub = self.create_publisher(Image, "/camera/depth/image_raw", 10)
        self._bridge = OrbbecCameraBridge()
        self._bridge.set_callbacks(self._on_color, self._on_depth)
        self.get_logger().info(
            f"OrbbecBridge node created (backend: {self._bridge._backend})"
        )

    def start(self) -> None:
        self._bridge.start()
        self.get_logger().info("OrbbecBridge capturing started")

    def stop(self) -> None:
        self._bridge.stop()

    def _on_color(self, img: np.ndarray, encoding: str, ts: float) -> None:
        if not _ROS_AVAILABLE:
            return
        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_color_optical_frame"
        msg.height, msg.width = img.shape[:2]
        msg.encoding = encoding
        msg.is_bigendian = False
        msg.step = img.shape[1] * 3
        msg.data = img.tobytes()
        self._color_pub.publish(msg)

    def _on_depth(self, img: np.ndarray, encoding: str, ts: float) -> None:
        if not _ROS_AVAILABLE:
            return
        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_depth_optical_frame"
        msg.height, msg.width = img.shape[:2]
        msg.encoding = encoding
        msg.is_bigendian = False
        msg.step = img.shape[1] * 2
        msg.data = img.tobytes()
        self._depth_pub.publish(msg)


def main(args=None):
    if not _ROS_AVAILABLE:
        print("ROS2 not available — running in standalone mode")
        bridge = OrbbecCameraBridge()

        def on_color(img, enc, ts):
            print(f"Color: {img.shape} {enc} ts={ts:.3f}")

        def on_depth(img, enc, ts):
            print(f"Depth: {img.shape} {enc} ts={ts:.3f}")

        bridge.set_callbacks(on_color, on_depth)
        bridge.start()
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            bridge.stop()
        return

    import rclpy
    rclpy.init(args=args)
    node = OrbbecBridgeNode()
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
