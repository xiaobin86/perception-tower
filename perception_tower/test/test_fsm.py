import time
import os
import tempfile
import numpy as np
from perception_tower.fsm import State, TowerFSM
from perception_tower.mock import FakeServo
from perception_tower.camera_grabber import PhotoPair
from perception_tower.stitcher import FairyFrame, StitchParams
from sensor_msgs.msg import Image


class StubCamera:
    def __init__(self):
        self.pair = PhotoPair(color=Image(), depth=Image())

    def capture(self, timeout_s):
        return self.pair


class StubFairyBuffer:
    def __init__(self, frames):
        self._frames = frames
        self.capturing = False

    def start(self):
        self.capturing = True

    def stop(self):
        self.capturing = False

    def frames(self):
        return self._frames


class StubCloudCb:
    def __init__(self):
        self.called = False
        self.xyz = None

    def __call__(self, xyz, intensity, stamp):
        self.called = True
        self.xyz = xyz


def make_fsm(mock_servo_speed=1000.0):
    servo = FakeServo(speed_deg_s=mock_servo_speed)
    camera = StubCamera()
    frames = [FairyFrame(0.0, 0.0, np.array([[1.0, 0.0, 0.0]], dtype=np.float32), None, None)]
    buffer = StubFairyBuffer(frames)
    status_log = []
    cloud = StubCloudCb()
    fsm = TowerFSM(
        servo=servo,
        camera=camera,
        fairy_buffer=buffer,
        stitch_params=StitchParams(mount_rpy_deg=[0.0, 0.0, 0.0], scan_start_deg=-90.0, scan_end_deg=90.0),
        save_cfg={'output_dir': tempfile.mkdtemp(), 'save_cloud': False},
        status_cb=lambda s, p, m: status_log.append((s, p, m)),
        photo_cb=lambda pair: None,
        cloud_cb=cloud,
        clock_now=lambda: time.time(),
    )
    return fsm, status_log, cloud


def test_init_flow():
    fsm, log, _ = make_fsm()
    ok, msg = fsm.request_init()
    assert ok
    # wait for thread
    while fsm.state != State.READY and fsm.state != State.ERROR:
        time.sleep(0.01)
    assert fsm.state == State.READY
    assert any(s == State.INITING for s, _, _ in log)


def test_scan_flow():
    from unittest.mock import patch
    fsm, log, cloud = make_fsm()
    fsm.request_init()
    while fsm.state != State.READY:
        time.sleep(0.01)
    with patch('perception_tower.camera_grabber.save_photos', return_value=('/tmp/fake.png', '/tmp/fake.png')):
        ok, msg = fsm.request_scan()
        assert ok
        while fsm.state != State.READY:
            time.sleep(0.01)
    assert cloud.called
    assert cloud.xyz.shape[0] == 1


def test_busy_rejection():
    fsm, _, _ = make_fsm()
    fsm.request_init()
    ok, msg = fsm.request_scan()
    assert not ok
    assert 'busy' in msg


def test_error_on_empty_frames():
    fsm, log, _ = make_fsm()
    fsm.request_init()
    while fsm.state != State.READY:
        time.sleep(0.01)
    fsm._fairy_buffer = StubFairyBuffer([])
    fsm.request_scan()
    while fsm.state != State.ERROR:
        time.sleep(0.01)
