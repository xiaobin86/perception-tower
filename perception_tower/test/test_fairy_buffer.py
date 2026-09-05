import numpy as np
from builtin_interfaces.msg import Time
from perception_tower.fairy_buffer import FairyBuffer
from perception_tower.pc2_utils import make_cloud_msg


def test_buffer_drops_when_not_capturing():
    buf = FairyBuffer(use_time_field=True)
    msg = make_cloud_msg(np.array([[1.0, 0.0, 0.0]], dtype=np.float32), None, 'lidar_link', Time(sec=1, nanosec=0))
    buf.on_cloud(msg, 1.0)
    assert len(buf.frames()) == 0


def test_buffer_captures_and_estimates_period():
    buf = FairyBuffer(use_time_field=True)
    buf.start()
    for i in range(5):
        xyz = np.array([[float(i), 0.0, 0.0]], dtype=np.float32)
        pt = np.array([0.01], dtype=np.float64)
        msg = make_cloud_msg(xyz, None, 'lidar_link', Time(sec=i, nanosec=0), point_time=pt)
        buf.on_cloud(msg, float(i))
    buf.stop()
    frames = buf.frames()
    assert len(frames) == 5
    assert frames[3].time_origin_sec == 2.0  # median period 1.0 -> frame[3].stamp=3 -> origin=2
