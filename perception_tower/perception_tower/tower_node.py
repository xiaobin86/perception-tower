"""perception_tower ROS2 node (Task 11).

Wires the :class:`TowerFSM` to ROS interfaces:
  * service  ``/perception_tower/command``  (TowerCommand)
  * status   ``/perception_tower/status``    (TowerStatus, reliable+transient_local)
  * topics   ``<stitched_topic>`` / ``<photo_color_topic>`` / ``<photo_depth_topic>``

In real mode the Fairy / 336L subscriptions feed the buffers and the real
serial servo is used. In ``mock_hardware:=true`` the buffers are fed by the
same subscriptions but the data is produced by ``MockFairy`` / ``MockCamera``
publishers, and a ``FakeServo`` replaces the serial client.
"""

from __future__ import annotations

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from .angle_logger import AngleLogger
from .camera_grabber import CameraGrabber, PhotoPair
from .fairy_buffer import FairyBuffer
from .fsm import State, TowerFSM
from .pc2_utils import make_cloud_msg
from .servo_client import ServoClient
from .stitcher import StitchParams

try:
    from builtin_interfaces.msg import Time
    from sensor_msgs.msg import Image, PointCloud2
    from perception_tower_interfaces.msg import TowerStatus
    from perception_tower_interfaces.srv import TowerCommand

    _ROS_AVAILABLE = True
except Exception:  # pragma: no cover - only when built
    Time = Image = PointCloud2 = TowerStatus = TowerCommand = None
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

        self._build_components()

    def _declare_params(self):
        p = [
            ("serial_port", ""),
            ("serial_baud", 115200),
            ("poll_hz", 100.0),
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
            ("fairy_topic", "/fairy/points"),
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
        origin = self.get_parameter("pos_origin").value
        dpp = self.get_parameter("deg_per_pos").value
        angle_sign = self.get_parameter("angle_sign").value

        if self._mock:
            from .mock import FakeServo

            servo = FakeServo(origin=origin, deg_per_pos=dpp, speed_deg_s=cfg["sweep_speed_deg_s"])
        else:
            servo = ServoClient(
                port=self.get_parameter("serial_port").value,
                baud=self.get_parameter("serial_baud").value,
                pos_origin=origin,
                deg_per_pos=dpp,
            )
            try:
                servo.open()
            except Exception as exc:
                self.get_logger().error(f"failed to open servo: {exc}")

        camera = CameraGrabber(now_fn=lambda: self.get_clock().now().nanoseconds * 1e-9)
        # Always subscribe so the buffers are fed (in mock mode the data is
        # produced by MockCamera / MockFairy below).
        self.create_subscription(
            Image,
            self.get_parameter("color_topic").value,
            camera.on_color,
            10,
        )
        self.create_subscription(
            Image,
            self.get_parameter("depth_topic").value,
            camera.on_depth,
            10,
        )
        if self._mock:
            from .mock import MockCamera

            MockCamera(self, self.get_parameter("color_topic").value, self.get_parameter("depth_topic").value)

        fairy_buffer = FairyBuffer(use_time_field=self.get_parameter("fairy_time_field").value)
        self.create_subscription(
            PointCloud2,
            self.get_parameter("fairy_topic").value,
            lambda msg: fairy_buffer.on_cloud(msg, self.get_clock().now().nanoseconds * 1e-9),
            10,
        )
        if self._mock:
            from .mock import MockFairy

            MockFairy(self, self.get_parameter("fairy_topic").value, servo)

        stitch_params = StitchParams(
            mount_rpy_deg=self.get_parameter("mount_rpy_deg").value,
            mount_offset_xyz=self.get_parameter("mount_offset_xyz").value,
            scan_start_deg=cfg["scan_start_deg"],
            scan_end_deg=cfg["scan_end_deg"],
            voxel_leaf_m=self.get_parameter("voxel_leaf_m").value,
            per_point_time=self.get_parameter("fairy_time_field").value,
            angle_sign=angle_sign,
        )

        self._fsm = TowerFSM(
            servo=servo,
            camera=camera,
            fairy_buffer=fairy_buffer,
            stitch_params=stitch_params,
            save_cfg={
                "output_dir": self._output_dir,
                "save_cloud": self.get_parameter("save_cloud").value,
            },
            status_cb=self._on_fsm_status,
            photo_cb=self._on_photo,
            cloud_cb=self._on_cloud,
            clock_now=lambda: self.get_clock().now().nanoseconds * 1e-9,
            log_cb=lambda m: self.get_logger().info(m),
            config=cfg,
        )

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
            self._fsm._servo.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
