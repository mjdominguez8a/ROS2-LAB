import math

import numpy as np
import pytest

from so100_labs.robot_model import LOWER, POSES, UPPER
from so100_labs.task4_inverse_kinematics import (TARGETS, UnreachableError, inverse_kinematics,
                                                 numerical_jacobian, task_vector)


def test_jacobian_shape_and_values():
    q = POSES['ready']
    jac = numerical_jacobian(q)
    assert np.asarray(jac).shape == (4, 4)
    # Compare with central differences
    for i in range(4):
        dq = np.zeros(5)
        dq[i] = 1e-5
        col = (task_vector(np.add(q, dq)) - task_vector(np.subtract(q, dq))) / 2e-5
        assert np.asarray(jac)[:, i] == pytest.approx(col, abs=1e-3)


@pytest.mark.parametrize('xyz, pitch', TARGETS)
def test_ik_reaches_targets(xyz, pitch):
    q = inverse_kinematics(xyz, pitch, POSES['home'])
    assert len(q) == 5
    assert task_vector(q)[:3] == pytest.approx(xyz, abs=5e-4)
    assert task_vector(q)[3] == pytest.approx(pitch, abs=5e-3)
    assert all(lo - 1e-9 <= v <= hi + 1e-9 for v, lo, hi in zip(q, LOWER, UPPER))
    assert q[4] == pytest.approx(POSES['home'][4])  # wrist roll untouched


def test_ik_round_trip_random():
    rng = np.random.default_rng(0)
    for _ in range(10):
        q_true = list(rng.uniform([-1.0, -0.8, -0.2, -1.4, 1.57], [1.0, 0.4, 1.2, -0.2, 1.57]))
        x = task_vector(q_true)
        q = inverse_kinematics(x[:3], x[3], POSES['home'])
        assert task_vector(q)[:3] == pytest.approx(x[:3], abs=5e-4)


def test_ik_unreachable():
    with pytest.raises(UnreachableError):
        inverse_kinematics((0.0, -1.0, 0.1), math.radians(90), POSES['home'])
