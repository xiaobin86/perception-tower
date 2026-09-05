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
        angle_logger,
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
        self._angle_logger = angle_logger
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
                self._set_state(self._state,
                                int(progress_base + min(0.999, pct) * progress_span), label)
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
        except Exception as exc:  # noqa: BLE001
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
            logger = self._angle_logger
            logger.start()
            self._fairy_buffer.start()
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
        except Exception as exc:  # noqa: BLE001
            self._set_state(State.ERROR, 0, f"scan failed: {exc}")
