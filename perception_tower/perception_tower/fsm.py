"""Tower state machine (INIT / SCAN) — Task 10.

Runs the servo + angle-logger + Fairy buffer + camera pipeline. Free of rclpy
so it can be exercised by plain pytest with mock hardware. Status / result
events are surfaced through callbacks the node wires to ROS publishers.
"""

from __future__ import annotations

import csv
import os
import threading
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Callable, Optional

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
        servo,
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
        self._thread = threading.Thread(target=self._run_init, daemon=True)
        self._thread.start()
        return True, "init started"

    def request_scan(self) -> tuple:
        with self._lock:
            if self._state in self._busy_states():
                return False, f"busy: {self._state.name}"
        self._thread = threading.Thread(target=self._run_scan, daemon=True)
        self._thread.start()
        return True, "scan started"

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
            tol_deg=self._cfg.get("pos_tol_deg", 0.1),
            stable_count=self._cfg.get("pos_stable_count", 5),
            timeout_s=self._cfg.get("move_timeout_s", 30.0),
            poll_hz=self._cfg.get("poll_hz", 100.0),
            progress_cb=cb,
        )
        cb(1.0)

    def _run_init(self):
        try:
            self._set_state(State.INITING, 0, "homing")
            self._servo.reset(timeout_s=self._cfg.get("home_timeout_s", 30.0))
            self._set_state(State.INITING, 50, "moving to ready")
            self._move_to_deg(
                self._cfg.get("ready_deg", 90.0),
                self._cfg.get("sweep_speed_deg_s", 40.0),
                50,
                50,
                "moving to ready",
            )
            self._set_state(State.READY, 100, "ready")
        except Exception as exc:  # noqa: BLE001
            self._set_state(State.ERROR, 0, f"init failed: {exc}")

    def _ensure_ready(self):
        pos = self._servo.read_position()
        deg = self._servo.pos_to_deg(pos)
        if abs(deg - self._cfg.get("ready_deg", 90.0)) > self._cfg.get("pos_tol_deg", 0.1):
            self._set_state(State.SCANNING, 0, "re-homing")
            self._servo.reset(timeout_s=self._cfg.get("home_timeout_s", 30.0))
            self._move_to_deg(
                self._cfg.get("ready_deg", 90.0),
                self._cfg.get("sweep_speed_deg_s", 40.0),
                0,
                10,
                "re-homing",
            )

    def _run_scan(self):
        try:
            self._set_state(State.SCANNING, 0, "scan start")
            self._ensure_ready()
            self._set_state(State.SCANNING, 10, "capture photo")
            pair = self._camera.capture(timeout_s=self._cfg.get("photo_timeout_s", 5.0))
            out_dir = os.path.join(self._save_cfg.output_dir, time.strftime("%Y%m%d_%H%M%S"))
            os.makedirs(out_dir, exist_ok=True)
            from .camera_grabber import save_photos

            save_photos(pair, out_dir)
            self._photo_cb(pair)

            self._set_state(State.SCANNING, 20, "move to scan start")
            from .angle_logger import AngleLogger

            logger = AngleLogger(
                self._servo.read_position,
                self._clock,
                self._servo.pos_to_deg,
                poll_hz=self._cfg.get("poll_hz", 100.0),
            )
            self._fairy_buffer.start()
            logger.start()
            self._move_to_deg(
                self._cfg.get("scan_start_deg", 30.0),
                self._cfg.get("sweep_speed_deg_s", 40.0),
                20,
                10,
                "move to start",
            )

            self._set_state(State.SCANNING, 30, "sweep")
            self._move_to_deg(
                self._cfg.get("scan_end_deg", 150.0),
                self._cfg.get("sweep_speed_deg_s", 40.0),
                30,
                40,
                "sweeping",
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
                95,
                5,
                "return to ready",
            )
            self._set_state(State.READY, 100, f"scan done: {out_dir}")
        except Exception as exc:  # noqa: BLE001
            self._set_state(State.ERROR, 0, f"scan failed: {exc}")
