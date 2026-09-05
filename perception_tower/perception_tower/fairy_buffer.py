"""Fairy LiDAR frame buffer with online frame-period estimation (Task 9).

Caches Fairy ``PointCloud2`` frames during a sweep. The per-frame
``time_origin`` is anchored to ``stamp - period``, where ``period`` is the
median of recent successive header-stamp deltas (0.0 when insufficient
samples). The stitcher consumes these frames together with the angle log.
"""

from __future__ import annotations

from collections import deque

import numpy as np

from .pc2_utils import read_field, read_intensity, read_time, read_xyz
from .stitcher import FairyFrame


class FairyBuffer:
    def __init__(self, use_time_field: bool = True, history_max: int = 5):
        self._use_time = use_time_field
        self._capturing = False
        self._frames: list = []
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

    def on_cloud(self, msg, stamp_sec: float):
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

    def frames(self) -> list:
        return list(self._frames)

    def count(self) -> int:
        return len(self._frames)
