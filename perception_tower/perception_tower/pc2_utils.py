"""PointCloud2 parse/build helpers and a hand-written binary PCD reader/writer.

Kept ROS-dependent message construction lazy so the pure-PCD functions remain
usable in plain Python environments; the point-cloud message helpers require
``sensor_msgs`` (ROS2 runtime).
"""

from __future__ import annotations

import struct
from typing import Optional, Tuple

import numpy as np

try:
    from sensor_msgs.msg import PointCloud2, PointField
    from builtin_interfaces.msg import Time

    _ROS_AVAILABLE = True
except Exception:  # pragma: no cover - exercised only without ROS
    _ROS_AVAILABLE = False


_TYPEMAP = {
    PointField.INT8: np.int8,
    PointField.UINT8: np.uint8,
    PointField.INT16: np.int16,
    PointField.UINT16: np.uint16,
    PointField.INT32: np.int32,
    PointField.UINT32: np.uint32,
    PointField.FLOAT32: np.float32,
    PointField.FLOAT64: np.float64,
} if _ROS_AVAILABLE else {}


def _find_field(msg: "PointCloud2", name: str) -> Optional["PointField"]:
    for f in msg.fields:
        if f.name == name:
            return f
    return None


def read_field(msg: "PointCloud2", name: str) -> Optional[np.ndarray]:
    f = _find_field(msg, name)
    if f is None:
        return None
    n = msg.width * msg.height
    dtype = np.dtype(_TYPEMAP[f.datatype])
    arr = np.frombuffer(msg.data, dtype=np.uint8, count=n * msg.point_step).reshape(n, msg.point_step)
    cols = arr[:, f.offset : f.offset + dtype.itemsize].copy()
    return cols.view(dtype).reshape(n)


def read_xyz(msg: "PointCloud2") -> np.ndarray:
    return np.stack([read_field(msg, "x"), read_field(msg, "y"), read_field(msg, "z")], axis=1).astype(np.float32)


def read_time(msg: "PointCloud2") -> Optional[np.ndarray]:
    t = read_field(msg, "time")
    return None if t is None else t.astype(np.float64)


def read_intensity(msg: "PointCloud2") -> Optional[np.ndarray]:
    t = read_field(msg, "intensity")
    return None if t is None else t.astype(np.float32)


def _key_for_type(dtype: np.dtype) -> int:
    for k, v in _TYPEMAP.items():
        if v == dtype.type:
            return k
    raise ValueError(f"unsupported dtype {dtype}")


def make_cloud_msg(
    xyz: np.ndarray,
    intensity: Optional[np.ndarray],
    frame_id: str,
    stamp: "Time",
    point_time: Optional[np.ndarray] = None,
) -> "PointCloud2":
    if not _ROS_AVAILABLE:  # pragma: no cover
        raise RuntimeError("sensor_msgs not available")
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    n = xyz.shape[0]
    names = ["x", "y", "z"]
    formats = [np.float32, np.float32, np.float32]
    offsets = [0, 4, 8]
    if intensity is not None:
        names.append("intensity")
        formats.append(np.float32)
        offsets.append(12)
    if point_time is not None:
        names.append("time")
        formats.append(np.float64)
        offsets.append(16 if intensity is None else 20)
    dtype = np.dtype(
        {
            "names": names,
            "formats": formats,
            "offsets": offsets,
            "itemsize": offsets[-1] + np.dtype(formats[-1]).itemsize,
        }
    )
    rec = np.zeros(n, dtype=dtype)
    rec["x"] = xyz[:, 0]
    rec["y"] = xyz[:, 1]
    rec["z"] = xyz[:, 2]
    if intensity is not None:
        rec["intensity"] = np.asarray(intensity, dtype=np.float32)
    if point_time is not None:
        rec["time"] = np.asarray(point_time, dtype=np.float64)
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = n
    msg.fields = [
        PointField(name=name, offset=off, datatype=_key_for_type(np.dtype(fmt)), count=1)
        for name, off, fmt in zip(names, offsets, formats)
    ]
    msg.is_bigendian = False
    msg.point_step = dtype.itemsize
    msg.row_step = dtype.itemsize * n
    msg.is_dense = False
    msg.data = rec.tobytes()
    return msg


def save_pcd_binary(path: str, xyz: np.ndarray, intensity: Optional[np.ndarray] = None) -> None:
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    n = xyz.shape[0]
    has_i = intensity is not None
    fields = "x y z intensity" if has_i else "x y z"
    size = "4 4 4 4" if has_i else "4 4 4"
    typ = "F F F F" if has_i else "F F F"
    count = "1 1 1 1" if has_i else "1 1 1"
    header = (
        f"# .PCD v0.7 - Point Cloud Data file format\n"
        f"VERSION 0.7\n"
        f"FIELDS {fields}\n"
        f"SIZE {size}\n"
        f"TYPE {typ}\n"
        f"COUNT {count}\n"
        f"WIDTH {n}\n"
        f"HEIGHT 1\n"
        f"VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        f"DATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        if has_i:
            i_arr = np.asarray(intensity, dtype=np.float32).reshape(-1)
            rec = np.column_stack([xyz, i_arr]).astype(np.float32)
            f.write(rec.tobytes())
        else:
            f.write(xyz.tobytes())


def load_pcd_binary(path: str) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    with open(path, "rb") as f:
        header_lines = []
        while True:
            line = f.readline().decode("ascii")
            header_lines.append(line)
            if line.startswith("DATA"):
                break
    meta = {}
    for line in header_lines:
        if " " in line:
            k, v = line.strip().split(" ", 1)
            meta[k] = v
    n = int(meta["POINTS"])
    fields = meta["FIELDS"].split()
    sizes = list(map(int, meta["SIZE"].split()))
    types = meta["TYPE"].split()
    fmt_map = {"F": np.float32, "I": np.int32, "U": np.uint32}
    itemsize = sum(sizes)
    raw = np.fromfile(path, dtype=np.uint8, offset=sum(len(l.encode("ascii")) for l in header_lines))
    rec = raw[: n * itemsize].reshape(n, itemsize)
    off = 0
    out = {}
    for field, size, typ in zip(fields, sizes, types):
        out[field] = rec[:, off : off + size].copy().view(fmt_map[typ]).reshape(n)
        off += size
    xyz = np.stack([out["x"], out["y"], out["z"]], axis=1).astype(np.float32)
    intensity = out.get("intensity")
    return xyz, intensity
