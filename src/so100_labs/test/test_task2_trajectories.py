import numpy as np
import pytest

from so100_labs.task2_trajectories import minimum_duration, quintic, sample_trajectory

Q0 = [0.0, -1.0, 1.2, -0.5, 1.57]
Q1 = [0.8, -0.3, 0.6, -1.2, 1.57]


def test_quintic_boundaries():
    p, v = quintic(Q0, Q1, 2.0, 0.0)
    assert p == pytest.approx(Q0) and v == pytest.approx([0] * 5)
    p, v = quintic(Q0, Q1, 2.0, 2.0)
    assert p == pytest.approx(Q1) and v == pytest.approx([0] * 5, abs=1e-12)


def test_quintic_midpoint_and_clamping():
    p, v = quintic(Q0, Q1, 2.0, 1.0)
    assert p == pytest.approx((np.array(Q0) + Q1) / 2)
    assert v == pytest.approx((np.array(Q1) - Q0) * 15 / 8 / 2.0)
    assert quintic(Q0, Q1, 2.0, 5.0)[0] == pytest.approx(Q1)
    assert quintic(Q0, Q1, 2.0, -1.0)[0] == pytest.approx(Q0)


def test_minimum_duration():
    # largest change is 0.8 rad -> T = 15/8 * 0.8 / 1.0
    assert minimum_duration(Q0, Q1, 1.0) == pytest.approx(1.5)
    assert minimum_duration(Q0, Q1, 0.5) == pytest.approx(3.0)


def test_sample_trajectory_shape_and_limits():
    waypoints = [Q0, Q1, Q0]
    times, pos, vel = sample_trajectory(waypoints, max_velocity=1.0, dt=0.1)
    assert len(times) == len(pos) == len(vel)
    assert times[0] == pytest.approx(0.1)
    assert np.all(np.diff(times) > 0)
    assert np.allclose(np.diff(times), 0.1)
    assert times[-1] == pytest.approx(3.0)  # 2 segments x 1.5 s
    assert pos[-1] == pytest.approx(Q0)
    assert np.max(np.abs(vel)) <= 1.0 + 1e-9
    # stops at the intermediate waypoint
    i = int(np.argmin(np.abs(np.array(times) - 1.5)))
    assert pos[i] == pytest.approx(Q1) and vel[i] == pytest.approx([0] * 5, abs=1e-9)


def test_sample_trajectory_rounds_up_duration():
    times, _, _ = sample_trajectory([Q0, [q + 0.1 for q in Q0]], max_velocity=1.0, dt=0.1)
    # 15/8 * 0.1 = 0.1875 -> 0.2 s
    assert times[-1] == pytest.approx(0.2)
