import pytest

from so100_labs.robot_model import ARM_JOINTS, LOWER, UPPER
from so100_labs.task1_joint_commands import clamp_to_limits, interpolate_steps, order_joints


def test_order_joints():
    named = {name: float(i) for i, name in enumerate(reversed(ARM_JOINTS))}
    assert order_joints(named) == [4.0, 3.0, 2.0, 1.0, 0.0]


def test_order_joints_missing_joint():
    with pytest.raises(KeyError):
        order_joints({'shoulder_pan_joint': 0.0})


def test_clamp_inside_is_unchanged():
    q = [0.1, -0.2, 0.3, -0.4, 0.5]
    assert clamp_to_limits(q) == pytest.approx(q)


def test_clamp_outside():
    assert clamp_to_limits([10.0] * 5) == pytest.approx(UPPER)
    assert clamp_to_limits([-10.0] * 5) == pytest.approx(LOWER)


def test_clamp_does_not_modify_input():
    q = [10.0] * 5
    clamp_to_limits(q)
    assert q == [10.0] * 5


def test_interpolate_steps():
    start, goal = [0.0] * 5, [0.1, -0.05, 0.0, 0.0, 0.02]
    steps = interpolate_steps(start, goal, max_step=0.02)
    assert len(steps) == 5
    assert steps[-1] == pytest.approx(goal)
    prev = start
    for s in steps:
        assert max(abs(a - b) for a, b in zip(s, prev)) <= 0.02 + 1e-9
        prev = s
    assert steps[0] == pytest.approx([0.02, -0.01, 0.0, 0.0, 0.004])


def test_interpolate_no_motion_gives_one_step():
    assert interpolate_steps([0.3] * 5, [0.3] * 5) == [pytest.approx([0.3] * 5)]
