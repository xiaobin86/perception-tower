"""100 Hz turntable angle logger with time-based interpolation (Task 6).

Records ``(host_time, raw_angle_deg)`` while the turntable sweeps, then
provides linear interpolation of the raw absolute angle (``angles_at``).
Out-of-range queries are clamped to the nearest endpoint.

Kept free of ROS imports: time source and position reader are injected.
"""

from __future__ import annotations

import threading
import time
from typing import Callable, List, Optional, Tuple

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
        self._samples: List[Tuple[float, float]] = []
        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.error: Optional[Exception] = None

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
        except Exception as exc:  # noqa: BLE001
            self.error = exc

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
        return np.interp(np.asarray(ts, dtype=np.float64), arr[:, 0], arr[:, 1], left=arr[0, 1], right=arr[-1, 1])
