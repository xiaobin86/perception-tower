import numpy as np
from perception_tower.stitcher import FairyFrame, StitchParams, stitch, voxel_downsample
from perception_tower.geometry import mount_rotation


def test_single_frame_no_rotation():
    # R_mount=I, T=0, theta=0 => output equals input
    xyz = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float32)
    frame = FairyFrame(0.0, 0.0, xyz, None, None)
    params = StitchParams(mount_rpy_deg=[0.0, 0.0, 0.0], scan_start_deg=-90.0, scan_end_deg=90.0)
    res = stitch([frame], lambda ts: np.zeros_like(ts), params)
    assert np.allclose(res.xyz, xyz)


def test_window_crop():
    xyz = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
    frame = FairyFrame(0.0, 0.0, xyz, None, None)
    params = StitchParams(scan_start_deg=10.0, scan_end_deg=20.0)
    res = stitch([frame], lambda ts: np.full_like(ts, 5.0), params)
    assert res.n_points == 0


def test_per_point_time_compensation():
    # half points at theta=0, half at theta=90
    xyz = np.array([[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float32)
    pt = np.array([0.0, 0.1], dtype=np.float64)
    frame = FairyFrame(0.0, 0.0, xyz, pt, None)
    params = StitchParams(mount_rpy_deg=[0.0, 0.0, 0.0], scan_start_deg=-90.0, scan_end_deg=90.0)

    def angles_at(ts):
        return np.where(ts < 0.05, 0.0, 90.0)

    res = stitch([frame], angles_at, params)
    assert res.n_points == 2
    assert np.allclose(res.xyz[0], [1.0, 0.0, 0.0], atol=1e-5)
    assert np.allclose(res.xyz[1], [0.0, 1.0, 0.0], atol=1e-5)


def test_voxel_downsample():
    xyz = np.array([[0.0, 0.0, 0.0], [0.009, 0.0, 0.0], [1.0, 1.0, 1.0]], dtype=np.float32)
    intensity = np.array([10.0, 20.0, 30.0], dtype=np.float32)
    out_xyz, out_i = voxel_downsample(xyz, intensity, 0.01)
    assert out_xyz.shape[0] == 2


def test_nan_filtered():
    xyz = np.array([[1.0, 0.0, 0.0], [np.nan, 0.0, 0.0]], dtype=np.float32)
    frame = FairyFrame(0.0, 0.0, xyz, None, None)
    params = StitchParams(mount_rpy_deg=[0.0, 0.0, 0.0], scan_start_deg=-90.0, scan_end_deg=90.0)
    res = stitch([frame], lambda ts: np.zeros_like(ts), params)
    assert res.n_points == 1
