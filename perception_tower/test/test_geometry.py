import numpy as np
from perception_tower.geometry import rotation_z_deg, mount_rotation, transform_frame


def test_rotation_z_90():
    pts = np.array([[1.0, 0.0, 0.0]])
    out = transform_frame(pts, np.eye(3), [0.0, 0.0, 0.0], 90.0)
    assert np.allclose(out, [[0.0, 1.0, 0.0]], atol=1e-6)


def test_mount_rx90_z_becomes_minus_y():
    r_mount = mount_rotation([90.0, 0.0, 0.0])
    out = transform_frame(np.array([[0.0, 0.0, 1.0]]), r_mount, [0.0, 0.0, 0.0], 0.0)
    assert np.allclose(out, [[0.0, -1.0, 0.0]], atol=1e-6)


def test_per_point_theta():
    pts = np.array([[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    out = transform_frame(pts, np.eye(3), [0.0, 0.0, 0.0], np.array([0.0, 90.0]))
    assert np.allclose(out[0], [1.0, 0.0, 0.0])
    assert np.allclose(out[1], [0.0, 1.0, 0.0])


def test_empty_input():
    out = transform_frame(np.zeros((0, 3)), np.eye(3), [0.0, 0.0, 0.0], 0.0)
    assert out.shape == (0, 3)
