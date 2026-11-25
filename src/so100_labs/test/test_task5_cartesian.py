import math

import numpy as np
import pytest

from so100_labs.robot_model import POSES
from so100_labs.task3_forward_kinematics import tcp_position
from so100_labs.task5_cartesian import (DOWN, cartesian_path, linear_interpolation,
                                        square_corners, time_parametrize)


def test_linear_interpolation():
    pts = np.array(linear_interpolation((0, 0, 0), (0.1, 0, 0), step=0.01))
    assert len(pts) == 11
    assert pts[0] == pytest.approx([0, 0, 0]) and pts[-1] == pytest.approx([0.1, 0, 0])
    gaps = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    assert gaps == pytest.approx(np.full(10, 0.01))


def test_linear_interpolation_uneven_length():
    pts = np.array(linear_interpolation((0, 0, 0), (0, 0.023, 0.0), step=0.005))
    gaps = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    assert np.all(gaps <= 0.005 + 1e-12)
    assert np.allclose(gaps, gaps[0])
    assert len(pts) == 6


def test_cartesian_path_stays_on_square():
    corners = square_corners((0.0, -0.24, 0.05), 0.08)
    pts = []
    for a, b in zip(corners[:-1], corners[1:]):
        pts += linear_interpolation(a, b, 0.01)[1:]
    path = cartesian_path(pts, DOWN, POSES['home'])
    assert len(path) == len(pts)
    for q, p in zip(path, pts):
        assert tcp_position(q) == pytest.approx(p, abs=1e-3)
    # continuity: no big jumps between neighbouring IK solutions
    assert np.max(np.abs(np.diff(np.array(path), axis=0))) < 0.2


def test_time_parametrize():
    q0 = [0.0] * 5
    path = [[0.1, 0, 0, 0, 0], [0.1, 0.3, 0, 0, 0], [0.1, 0.3, 0, 0, 0.001]]
    times = time_parametrize(path, q0, max_velocity=0.5, min_dt=0.05)
    assert times == pytest.approx([0.2, 0.8, 0.85])
