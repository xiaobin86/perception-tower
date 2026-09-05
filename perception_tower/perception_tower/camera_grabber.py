"""Orbbec Gemini 336L camera grabber (Task 8).

Caches the latest color and depth frames (and their arrival times), then grabs
a synchronized pair for the 90-degree snapshot. ``save_photos`` requires
``cv_bridge`` + ``cv2`` (opencv) for PNG output.
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

import numpy as np

try:  # only needed for save_photos
    from sensor_msgs.msg import Image

    _ROS_AVAILABLE = True
except Exception:  # pragma: no cover
    _ROS_AVAILABLE = False


@dataclass
class PhotoPair:
    color: "Image"
    depth: "Image"


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
        self._color: Optional[Tuple["Image", float]] = None
        self._depth: Optional[Tuple["Image", float]] = None

    def on_color(self, msg: "Image"):
        self._color = (msg, self._now())

    def on_depth(self, msg: "Image"):
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
        raise RuntimeError("camera capture timeout: no fresh color+depth pair")


def save_photos(pair: PhotoPair, output_dir: str) -> Tuple[str, str]:
    os.makedirs(output_dir, exist_ok=True)
    from cv_bridge import CvBridge

    bridge = CvBridge()
    color_cv = bridge.imgmsg_to_cv2(pair.color, desired_encoding="bgr8")
    depth_cv = bridge.imgmsg_to_cv2(pair.depth, desired_encoding="16UC1")
    cpath = os.path.join(output_dir, "color.png")
    dpath = os.path.join(output_dir, "depth.png")
    import cv2

    cv2.imwrite(cpath, color_cv)
    cv2.imwrite(dpath, depth_cv)
    return cpath, dpath
