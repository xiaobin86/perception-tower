import numpy as np
import pytest
from perception_tower.angle_logger import AngleLogger


def test_interpolation_and_clamping():
    logger = AngleLogger()
    logger.start()
    logger.record_sample(0.0, 0.0)
    logger.record_sample(0.01, 10.0)
    logger.record_sample(0.02, 20.0)
    logger.record_sample(0.03, 30.0)
    logger.stop()

    ts = np.array([-0.1, 0.015, 0.025, 0.05])
    out = logger.angles_at(ts)
    assert out[0] == 0.0
    assert out[-1] == 30.0
    assert np.isclose(out[1], 15.0)
    assert np.isclose(out[2], 25.0)


def test_last_angle():
    logger = AngleLogger()
    logger.start()
    assert logger.last_angle() is None
    logger.record_sample(0.0, 45.0)
    assert logger.last_angle() == 45.0
    logger.record_sample(0.01, 90.0)
    assert logger.last_angle() == 90.0
    logger.stop()


def test_coverage():
    logger = AngleLogger()
    logger.start()
    assert logger.coverage() is None
    logger.record_sample(1.0, 0.0)
    assert logger.coverage() is None
    logger.record_sample(2.0, 90.0)
    cov = logger.coverage()
    assert cov == (1.0, 2.0)
    logger.stop()


def test_empty_interpolation():
    logger = AngleLogger()
    logger.start()
    ts = np.array([0.0, 1.0])
    out = logger.angles_at(ts)
    assert np.all(out == 0.0)
    logger.stop()
