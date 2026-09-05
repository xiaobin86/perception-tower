import time
import numpy as np
import pytest
from perception_tower.angle_logger import AngleLogger


def test_interpolation_and_clamping():
    clock_times = [0.0, 0.01, 0.02, 0.03]
    positions = [500, 1000, 1500, 2000]
    clock_iter = iter(clock_times)
    pos_iter = iter(positions)

    def read_position(timeout_s):
        return next(pos_iter)

    def clock_now():
        return next(clock_iter)

    logger = AngleLogger(read_position, clock_now, lambda p: (p - 500) * 0.02, poll_hz=100.0)
    logger.start()
    while len(logger._samples) < 4:
        time.sleep(0.001)
    logger.stop()

    ts = np.array([-0.1, 0.015, 0.025, 0.05])
    out = logger.angles_at(ts)
    assert out[0] == 0.0
    assert out[-1] == 30.0
    assert np.isclose(out[1], 15.0)
    assert np.isclose(out[2], 25.0)


def test_error_propagation():
    def read_position(timeout_s):
        raise RuntimeError('boom')

    logger = AngleLogger(read_position, time.time, lambda p: 0.0, poll_hz=1000.0)
    logger.start()
    time.sleep(0.01)
    logger.stop()
    assert isinstance(logger.error, RuntimeError)
