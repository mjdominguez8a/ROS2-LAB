"""Tests for the circle exercise. These FAIL until the TODO is completed."""
import math

from lab_starter.turtle_circle import compute_circle_command


def test_moves_forward():
    linear, _ = compute_circle_command(radius=2.0, speed=1.0)
    assert math.isclose(linear, 1.0)


def test_turns_at_correct_rate():
    _, angular = compute_circle_command(radius=2.0, speed=1.0)
    assert math.isclose(angular, 0.5)


def test_smaller_circle_turns_faster():
    _, small = compute_circle_command(radius=1.0, speed=1.0)
    _, large = compute_circle_command(radius=4.0, speed=1.0)
    assert small > large
