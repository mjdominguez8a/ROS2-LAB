import math

import numpy as np
import pytest

from so100_labs.task3_forward_kinematics import (approach_pitch, forward_kinematics, homogeneous,
                                                 rotation_about_axis, rotation_from_rpy,
                                                 tcp_position)

# Reference TCP positions measured with TF (robot_state_publisher) on the URDF.
REFERENCE = [
    ([0.0, 0.0, 0.0, 0.0, 0.0], [-0.0085, -0.2387, 0.0787], 90.0),
    ([0.0, -1.0, 1.2, -0.5, 1.57], [0.0, -0.1843, 0.0332], 72.8),
    ([0.0, -0.3, 0.6, -1.2, 1.57], [0.0, -0.3288, 0.1083], 38.4),
    ([0.8, -0.3, 0.6, -1.2, 1.57], [-0.2035, -0.2428, 0.1083], 38.4),
    ([0.3, 0.5, -0.7, -0.9, 0.4], [-0.1206, -0.4085, 0.1675], 27.0),
]


def test_rotation_from_rpy():
    assert rotation_from_rpy(0, 0, 0) == pytest.approx(np.eye(3))
    rz = rotation_from_rpy(0, 0, math.pi / 2)
    assert rz @ [1, 0, 0] == pytest.approx([0, 1, 0], abs=1e-12)
    rx = rotation_from_rpy(math.pi / 2, 0, 0)
    assert rx @ [0, 1, 0] == pytest.approx([0, 0, 1], abs=1e-12)
    r = rotation_from_rpy(0.3, -0.2, 0.7)
    assert r @ r.T == pytest.approx(np.eye(3))
    # order: yaw applied last
    expected = rotation_from_rpy(0, 0, 0.7) @ rotation_from_rpy(0, -0.2, 0) @ rotation_from_rpy(0.3, 0, 0)
    assert r == pytest.approx(expected)


def test_rotation_about_axis():
    r = rotation_about_axis((0, 0, 2.0), math.pi / 2)  # not normalised on purpose
    assert r @ [1, 0, 0] == pytest.approx([0, 1, 0], abs=1e-12)
    assert rotation_about_axis((1, 0, 0), 0.4) == pytest.approx(rotation_from_rpy(0.4, 0, 0))
    assert rotation_about_axis((0, -1, 0), 0.4) == pytest.approx(rotation_from_rpy(0, -0.4, 0))


def test_homogeneous():
    t = homogeneous(np.eye(3), [1, 2, 3])
    assert t.shape == (4, 4)
    assert t @ [0, 0, 0, 1] == pytest.approx([1, 2, 3, 1])
    assert t[3] == pytest.approx([0, 0, 0, 1])


@pytest.mark.parametrize('q, xyz, pitch_deg', REFERENCE)
def test_forward_kinematics_matches_tf(q, xyz, pitch_deg):
    t = forward_kinematics(q)
    assert t.shape == (4, 4)
    assert tcp_position(q) == pytest.approx(xyz, abs=2e-4)
    assert math.degrees(approach_pitch(q)) == pytest.approx(pitch_deg, abs=0.2)
    assert t[:3, :3] @ t[:3, :3].T == pytest.approx(np.eye(3))
