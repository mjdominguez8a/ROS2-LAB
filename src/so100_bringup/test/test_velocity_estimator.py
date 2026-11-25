import math

from so100_bringup.safety_monitor import VelocityEstimator

STEP = 2 * math.pi / 4096  # one encoder step, rad


def run(samples, rate=50.0):
    """Feed one joint's positions at `rate` Hz; return the velocity estimate after each sample."""
    est = VelocityEstimator()
    return [est.update(i / rate, {'j': q})['j'] for i, q in enumerate(samples)]


def test_holding_still_with_encoder_noise_stays_slow():
    # The arm holding position: the reading flickers by one encoder step.
    v = run([0.0, STEP, 0.0, STEP, STEP, 0.0, STEP, 0.0, 0.0, STEP] * 5)
    assert max(abs(x) for x in v) < 0.1


def test_steady_motion_is_measured():
    v = run([0.5 * i / 50 for i in range(50)])  # 0.5 rad/s
    assert abs(v[-1] - 0.5) < 1e-6


def test_fast_motion_exceeds_limit():
    v = run([3.0 * i / 50 for i in range(10)])  # 3 rad/s
    assert v[-1] > 1.5


def test_backwards_drift_is_small_and_negative():
    # What feetech_ros2_driver 0.2.2 reported as +50.34 rad/s: a slow drift backwards.
    v = run([-0.075 * i / 50 for i in range(20)])
    assert -0.1 < v[-1] < 0


def test_gap_restarts_estimate():
    est = VelocityEstimator()
    est.update(0.0, {'j': 0.0})
    est.update(0.1, {'j': 0.1})
    # Robot restarted 10 s later in a different pose: no huge fake velocity.
    assert est.update(10.0, {'j': 2.0})['j'] == 0.0
