import threading
import time
import numpy as np
import pytest
from perception_tower.servo_client import ServoClient, poll_until_reached, ServoError


class FakeSerial:
    def __init__(self, script):
        """script: callable(write_bytes) -> reply_bytes or None"""
        self._script = script
        self._rx = bytearray()
        self.written = []
        self._lock = threading.Lock()

    def write(self, data: bytes):
        with self._lock:
            self.written.append(bytes(data))
            reply = self._script(data)
            if reply:
                self._rx.extend(reply)
        return len(data)

    def read(self, size: int = 1) -> bytes:
        with self._lock:
            n = min(size, len(self._rx))
            out = bytes(self._rx[:n])
            del self._rx[:n]
            return out

    def close(self):
        pass


def test_read_position():
    def script(d):
        if d == b'#000PRAD!':
            return b'#000P5000!\r\n'
    c = ServoClient('/dev/null', serial_factory=lambda p, b, timeout: FakeSerial(script))
    c.open()
    assert c.read_position() == 5000
    c.close()


def test_reset_waits_ok():
    def script(d):
        if d == b'#000PRST!':
            time.sleep(0.05)
            return b'#OK!\r\n'
    c = ServoClient('/dev/null', serial_factory=lambda p, b, timeout: FakeSerial(script))
    c.open()
    c.reset(timeout_s=1.0)
    c.close()


def test_move_to_writes_correct_command():
    fake = FakeSerial(lambda d: None)
    c = ServoClient('/dev/null', serial_factory=lambda p, b, timeout: fake)
    c.open()
    c.move_to(5000, 2000)
    c.close()
    assert fake.written == [b'#000P5000T2000!']


def test_poll_until_reached():
    positions = [4980, 4990, 4995, 4998, 5000, 5000, 5000, 5000, 5000]
    it = iter(positions)

    def read(timeout_s=0.2):
        return next(it)

    def deg_of(pos):
        return (pos - 500) * 0.02

    poll_until_reached(read, deg_of, 5000, tol_deg=0.1, stable_count=3, timeout_s=5.0, poll_hz=1000.0)


def test_poll_until_reached_timeout():
    def read(timeout_s=0.2):
        return 4900

    def deg_of(pos):
        return (pos - 500) * 0.02

    with pytest.raises(ServoError):
        poll_until_reached(read, deg_of, 5000, tol_deg=0.1, stable_count=3, timeout_s=0.2, poll_hz=1000.0)


def test_disabled_serial_port_raises():
    c = ServoClient('', serial_factory=lambda p, b, timeout: FakeSerial(lambda d: None))
    with pytest.raises(ServoError):
        c.open()
