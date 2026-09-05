"""Turntable stitching transform math (per plan Task 2).

Coordinate convention (world frame {W}):
  - Z axis points up (turntable rotation axis).
  - The LiDAR is mounted with a fixed extrinsic ``R_mount`` (an RPY rotation
    about the LiDAR body frame) and an optional offset ``T_mount`` relative to
    the turntable rotation axis.

Per-frame transform (point p at turntable angle theta):
    P_world = Rz(theta) * (R_mount * P_lidar + T_mount)
"""

from __future__ import annotations

from typing import Sequence

import numpy as np


def _to_rad(angle_deg: float) -> float:
    return angle_deg * np.pi / 180.0


def rotation_x_deg(angle_deg: float) -> np.ndarray:
    a = _to_rad(angle_deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float64)


def rotation_y_deg(angle_deg: float) -> np.ndarray:
    a = _to_rad(angle_deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)


def rotation_z_deg(angle_deg: float) -> np.ndarray:
    a = _to_rad(angle_deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def mount_rotation(rpy_deg: Sequence[float]) -> np.ndarray:
    roll, pitch, yaw = rpy_deg
    return rotation_z_deg(yaw) @ rotation_y_deg(pitch) @ rotation_x_deg(roll)


def transform_frame(
    xyz: np.ndarray,
    r_mount: np.ndarray,
    t_mount: Sequence[float],
    theta_deg: float | np.ndarray,
) -> np.ndarray:
    xyz = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
    if xyz.shape[0] == 0:
        return xyz.copy()
    base = xyz @ np.asarray(r_mount, dtype=np.float64).T + np.asarray(t_mount, dtype=np.float64)
    th = np.deg2rad(np.asarray(theta_deg, dtype=np.float64))
    c, s = np.cos(th), np.sin(th)
    x, y = base[:, 0].copy(), base[:, 1].copy()
    out = np.empty_like(base)
    out[:, 0] = x * c - y * s
    out[:, 1] = x * s + y * c
    out[:, 2] = base[:, 2]
    return out
