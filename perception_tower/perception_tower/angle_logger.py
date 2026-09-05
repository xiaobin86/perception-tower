"""Turntable angle logger with callback-based recording.

Records (timestamp, angle_deg) samples fed via record_sample(),
then provides linear interpolation via angles_at(). Out-of-range
queries are clamped to the nearest endpoint.

Kept free of ROS imports: the caller feeds samples.
"""

from __future__ import annotations

import threading
from typing import Callable, List, Optional, Tuple

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
