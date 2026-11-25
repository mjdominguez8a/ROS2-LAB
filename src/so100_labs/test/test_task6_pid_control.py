import numpy as np
import pytest

from so100_labs import servo_model
from so100_labs.robot_model import ARM_JOINTS, LOWER, POSES, UPPER
from so100_labs.task6_pid_control import (GAINS, PID, TARGETS, corrected_command, make_pids,
                                          simulate, tcp_error_mm)


# ---------------------------------------------------------------- the provided servo model
def test_model_sags_like_the_real_arm():
    q = POSES['ready']
    sag = servo_model.steady_state(q) - np.asarray(q)
    assert sag[0] == pytest.approx(0.0, abs=1e-6)            # pan axis is vertical: no gravity
    assert 0.005 < abs(sag[1]) < 0.05 and 0.02 < abs(sag[2]) < 0.08   # 1-3 deg, as on arm3
    assert tcp_error_mm(q, servo_model.steady_state(q)) > 10.0


def test_model_without_command_change_stays_sagged():
    q = POSES['ready']
    sim = servo_model.ServoArmSim(q, quantize=False)
    for _ in range(100):
        measured = sim.step(q, 0.05)
    assert measured == pytest.approx(servo_model.steady_state(q), abs=1e-4)


# ---------------------------------------------------------------------- PID building blocks
def test_proportional_only():
    pid = PID(kp=2.0, ki=0.0, kd=0.0, output_limit=10.0)
    assert pid.update(1.0, 0.25, 0.1) == pytest.approx(1.5)
    assert pid.update(0.0, 0.5, 0.1) == pytest.approx(-1.0)


def test_integral_accumulates_error_times_dt():
    pid = PID(kp=0.0, ki=2.0, kd=0.0, output_limit=10.0, integral_limit=10.0)
    assert pid.update(1.0, 0.0, 0.1) == pytest.approx(0.2)
    assert pid.update(1.0, 0.0, 0.1) == pytest.approx(0.4)
    assert pid.update(1.0, 0.5, 0.2) == pytest.approx(0.6)  # error 0.5 for 0.2 s


def test_integral_anti_windup():
    pid = PID(kp=0.0, ki=10.0, kd=0.0, output_limit=10.0, integral_limit=0.1)
    for _ in range(100):
        out = pid.update(1.0, 0.0, 0.05)
    assert out == pytest.approx(0.1)
    # Error changes sign: the bounded integral must come back quickly, not after seconds
    # (without the bound the integral would be +50 and the output would still push the wrong way)
    assert pid.update(0.0, 1.0, 0.05) < 0.0


def test_output_limit():
    pid = PID(kp=100.0, ki=0.0, kd=0.0, output_limit=0.15)
    assert pid.update(1.0, 0.0, 0.05) == pytest.approx(0.15)
    assert pid.update(-1.0, 0.0, 0.05) == pytest.approx(-0.15)


def test_derivative_on_measurement_no_kick():
    pid = PID(kp=0.0, ki=0.0, kd=1.0, output_limit=10.0)
    assert pid.update(0.0, 0.0, 0.1) == pytest.approx(0.0)    # first call: no previous value
    assert pid.update(5.0, 0.0, 0.1) == pytest.approx(0.0)    # setpoint jump: no kick
    assert pid.update(5.0, 0.2, 0.1) == pytest.approx(-2.0)   # measurement rises 2 units/s


def test_reset():
    pid = PID(kp=0.0, ki=1.0, kd=1.0, output_limit=10.0, integral_limit=10.0)
    pid.update(1.0, 0.0, 1.0)
    pid.update(1.0, 0.5, 1.0)
    pid.reset()
    assert pid.update(1.0, 0.9, 1.0) == pytest.approx(0.1)    # no old integral, no derivative


def test_make_pids_one_per_joint():
    pids = make_pids()
    assert len(pids) == len(ARM_JOINTS)
    assert all(isinstance(p, PID) for p in pids)


def test_corrected_command():
    pids = [PID(kp=1.0, ki=0.0, kd=0.0, output_limit=1.0) for _ in ARM_JOINTS]
    target = POSES['ready']
    measured = [t - 0.05 for t in target]
    cmd = corrected_command(target, measured, pids, 0.05)
    assert cmd == pytest.approx([t + 0.05 for t in target])
    # never outside the joint limits, even if the correction would push it there
    cmd = corrected_command(UPPER, [u - 0.5 for u in UPPER], pids, 0.05)
    assert np.all(np.array(cmd) <= np.array(UPPER) + 1e-9)
    assert np.all(np.array(cmd) >= np.array(LOWER) - 1e-9)


# --------------------------------------------------------- your gains on the servo model
def test_gains_defined_for_every_joint():
    assert set(GAINS) == set(ARM_JOINTS)
    for g in GAINS.values():
        assert set(g) == {'kp', 'ki', 'kd'}


@pytest.mark.parametrize('name', list(TARGETS))
def test_closed_loop_removes_the_sag(name):
    target = np.asarray(TARGETS[name])
    open_loop_mm = tcp_error_mm(target, servo_model.steady_state(target))
    times, errors = simulate(target=target, duration=3.0)
    after_2s = np.abs(errors[times >= 2.0]).max()
    assert after_2s < 0.003, f'joint error after 2 s is {after_2s:.4f} rad (need < 0.003)'
    initial_sign = np.sign(errors[0])
    overshoot = np.max(np.maximum(0.0, -initial_sign * errors))
    assert overshoot < 0.02, f'overshoot {overshoot:.3f} rad: gains too aggressive'
    final_mm = tcp_error_mm(target, target - errors[-1])
    assert final_mm < 3.0 < open_loop_mm


def test_proportional_alone_is_not_enough():
    gains = {j: {'kp': GAINS[j]['kp'], 'ki': 0.0, 'kd': GAINS[j]['kd']} for j in ARM_JOINTS}
    _, errors = simulate(gains=gains, duration=3.0)
    assert np.abs(errors[-1]).max() > 0.01  # the integral term is what removes the sag
