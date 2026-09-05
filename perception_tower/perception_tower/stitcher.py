"""Point-cloud stitching with per-point temporal compensation (Task 7).

For each Fairy frame:
  * Per-point absolute time ``t_p = time_origin + point_time`` (frame-level
    ``time_origin`` when the per-point ``time`` field is unavailable).
  * Raw absolute angle ``theta_raw = angles_at(t_p)`` (deg) via the angle log.
  * Rotation angle ``theta = angle_sign * theta_raw`` (deg) -> applied as Rz.
  * Crop window [scan_start, scan_end] applies to ``theta_raw`` (independent of
    ``angle_sign``). NaN/Inf points filtered. Optional voxel downsampling.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Sequence, Tuple

import numpy as np

from .geometry import mount_rotation, transform_frame


@dataclass
class FairyFrame:
    stamp_sec: float
    time_origin_sec: float
    xyz: np.ndarray
    point_time: Optional[np.ndarray]
    intensity: Optional[np.ndarray]


@dataclass
class StitchParams:
    mount_rpy_deg: Sequence[float] = (90.0, 0.0, 0.0)
    mount_offset_xyz: Sequence[float] = (0.0, 0.0, 0.0)
    scan_start_deg: float = 30.0
    scan_end_deg: float = 150.0
    voxel_leaf_m: float = 0.01
    per_point_time: bool = True
    angle_sign: int = 1


@dataclass
class StitchResult:
    xyz: np.ndarray
    intensity: Optional[np.ndarray]
    n_points: int
    n_frames: int


def voxel_downsample(
    xyz: np.ndarray,
    intensity: Optional[np.ndarray],
    leaf_m: float,
) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    if xyz.shape[0] == 0:
        return xyz.copy(), None if intensity is None else intensity.copy()
    keys = np.floor(xyz / leaf_m).astype(np.int64)
    order = np.lexsort(keys.T)
    sorted_xyz = xyz[order]
    sorted_i = None if intensity is None else intensity[order]
    diff = np.concatenate([[True], np.any(np.diff(sorted_xyz / leaf_m, axis=0) >= 1, axis=1)])
    boundaries = np.where(diff)[0]
    out_xyz = []
    out_i = []
    for start, end in zip(boundaries, list(boundaries[1:]) + [len(sorted_xyz)]):
        out_xyz.append(sorted_xyz[start:end].mean(axis=0))
        if sorted_i is not None:
            out_i.append(sorted_i[start:end].mean())
    out_xyz = np.array(out_xyz, dtype=np.float32)
    out_i = np.array(out_i, dtype=np.float32) if sorted_i is not None else None
    return out_xyz, out_i


def stitch(
    frames: Sequence[FairyFrame],
    angles_at: Callable[[np.ndarray], np.ndarray],
    params: StitchParams,
) -> StitchResult:
    r_mount = mount_rotation(params.mount_rpy_deg)
    t_mount = np.asarray(params.mount_offset_xyz, dtype=np.float64)
    chunks = []
    ichunks = []
    for fr in frames:
        if fr.xyz.size == 0:
            continue
        n = fr.xyz.shape[0]
        if params.per_point_time and fr.point_time is not None:
            ts = np.asarray(fr.time_origin_sec + fr.point_time, dtype=np.float64)
        else:
            ts = np.full(n, fr.time_origin_sec, dtype=np.float64)
        theta_raw = angles_at(ts)
        valid = np.isfinite(fr.xyz).all(axis=1) & np.isfinite(theta_raw)
        valid &= (theta_raw >= params.scan_start_deg) & (theta_raw <= params.scan_end_deg)
        if not np.any(valid):
            continue
        xyz = fr.xyz[valid]
        theta_deg = theta_raw[valid] * params.angle_sign
        world = transform_frame(xyz.astype(np.float64), r_mount, t_mount, theta_deg)
        chunks.append(world)
        if fr.intensity is not None:
            ichunks.append(fr.intensity[valid])
        else:
            ichunks.append(None)
    if not chunks:
        return StitchResult(np.zeros((0, 3), dtype=np.float32), None, 0, 0)
    xyz_out = np.vstack(chunks).astype(np.float32)
    intensity = None if any(c is None for c in ichunks) else np.concatenate(ichunks).astype(np.float32)
    if params.voxel_leaf_m > 0.0 and xyz_out.shape[0] > 0:
        xyz_out, intensity = voxel_downsample(xyz_out, intensity, params.voxel_leaf_m)
    return StitchResult(xyz_out, intensity, xyz_out.shape[0], len(frames))
