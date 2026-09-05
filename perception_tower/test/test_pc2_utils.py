import struct
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from builtin_interfaces.msg import Time

from perception_tower.pc2_utils import (
    read_field, read_xyz, read_time, read_intensity,
    make_cloud_msg, save_pcd_binary, load_pcd_binary,
)


def test_make_and_read_xyz():
    xyz = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
    msg = make_cloud_msg(xyz, None, 'test_frame', Time(sec=0, nanosec=0))
    out = read_xyz(msg)
    assert np.allclose(out, xyz)


def test_make_and_read_with_intensity():
    xyz = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
    intensity = np.array([10.0], dtype=np.float32)
    msg = make_cloud_msg(xyz, intensity, 'f', Time(sec=0, nanosec=0))
    assert np.allclose(read_xyz(msg), xyz)
    assert np.allclose(read_intensity(msg), intensity)


def test_make_and_read_with_point_time():
    xyz = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
    pt = np.array([0.5], dtype=np.float64)
    msg = make_cloud_msg(xyz, None, 'f', Time(sec=0, nanosec=0), point_time=pt)
    assert np.allclose(read_time(msg), pt)


def test_pcd_binary_roundtrip(tmp_path):
    xyz = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
    intensity = np.array([10.0, 20.0], dtype=np.float32)
    path = str(tmp_path / 'test.pcd')
    save_pcd_binary(path, xyz, intensity)
    xyz2, int2 = load_pcd_binary(path)
    assert np.allclose(xyz2, xyz)
    assert np.allclose(int2, intensity)


def test_pcd_binary_no_intensity(tmp_path):
    xyz = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
    path = str(tmp_path / 'test.pcd')
    save_pcd_binary(path, xyz)
    xyz2, int2 = load_pcd_binary(path)
    assert np.allclose(xyz2, xyz)
    assert int2 is None
