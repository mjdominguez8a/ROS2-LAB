"""Tests for the provided square example (these pass out of the box)."""
from lab_starter.turtle_square import SIDE_TICKS, TURN_TICKS, compute_command


def test_starts_driving_forward():
    linear, angular = compute_command(0)
    assert linear > 0.0
    assert angular == 0.0


def test_turns_after_a_side():
    linear, angular = compute_command(SIDE_TICKS)
    assert linear == 0.0
    assert angular > 0.0


def test_pattern_repeats():
    assert compute_command(0) == compute_command(SIDE_TICKS + TURN_TICKS)
