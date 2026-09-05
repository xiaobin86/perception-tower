import time
import os
import tempfile
import numpy as np
import pytest
from sensor_msgs.msg import Image
from perception_tower.camera_grabber import CameraGrabber, PhotoPair

try:
    import cv2
    _CV2_AVAILABLE = True
except ImportError:
    _CV2_AVAILABLE = False


def make_image(encoding, data):
    msg = Image()
    msg.header.frame_id = 'camera'
    msg.encoding = encoding
    msg.height = data.shape[0]
    msg.width = data.shape[1]
    if encoding == 'bgr8':
        msg.step = msg.width * 3
        msg.data = data.astype(np.uint8).tobytes()
    elif encoding == '16UC1':
        msg.step = msg.width * 2
        msg.data = data.astype(np.uint16).tobytes()
    return msg


def test_capture_pair():
    clock = [0.0]
    g = CameraGrabber(now_fn=lambda: clock[0], freshness_s=0.5, max_pair_gap_s=0.2)
    g.on_color(make_image('bgr8', np.zeros((4, 4, 3), np.uint8)))
    clock[0] = 0.05
    g.on_depth(make_image('16UC1', np.ones((4, 4), np.uint16) * 500))
    clock[0] = 0.06
    pair = g.capture(timeout_s=0.5)
    assert pair is not None


def test_capture_timeout():
    import time as _time
    start = _time.monotonic()
    g = CameraGrabber(now_fn=_time.monotonic, freshness_s=0.01, max_pair_gap_s=0.01)
    with pytest.raises(RuntimeError):
        g.capture(timeout_s=0.1)


@pytest.mark.skipif(not _CV2_AVAILABLE, reason="cv2 not installed")
def test_save_photos_roundtrip():
    from perception_tower.camera_grabber import save_photos
    color = make_image('bgr8', np.arange(48, dtype=np.uint8).reshape(4, 4, 3))
    depth = make_image('16UC1', np.arange(16, dtype=np.uint16).reshape(4, 4))
    pair = PhotoPair(color=color, depth=depth)
    out = tempfile.mkdtemp()
    cpath, dpath = save_photos(pair, out)
    assert os.path.exists(cpath) and os.path.exists(dpath)
