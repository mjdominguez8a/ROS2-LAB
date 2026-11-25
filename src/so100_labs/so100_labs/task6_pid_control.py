# ======  MEDICAL ROBOTICS AND TECHNOLOGIES  ======
# ======  FOR COMPUTER AIDED SURGERY LABORATORY  ======
#
#         Hands-on ROS2 - SO-100 arm - Task 6: Closed-loop joint control (PID)
#                                   2026-2027
#
# Complete every line marked with  # TODO : replace the `...` with your code.
# Each TODO is ONE short line of Python. The hints tell you which Python tool to use.
#
# Goal: the servos sag under gravity, so the arm never quite reaches its target.
# Measure the joints, and correct the command with a PID until the error is zero.
#
# Run (simulation that sags like the real arm, terminal 1):
#     ros2 launch so100_bringup arm.launch.py sag:=true
# Run your code (terminal 2):
#     ros2 run so100_labs task6_pid_control
# Tune on the servo model (no ROS needed):
#     ros2 run so100_labs task6_pid_tune
# Test:
#     pytest src/so100_labs/test/test_task6_pid_control.py
#
# Background - each servo is a P controller:  torque = K * (q_cmd - q).
# Holding the arm up needs torque, so it needs an error. The joint settles at
#     q = q_cmd + tau_g(q) / K          <- steady-state error ("sag")
# Your PID adds a correction u on top of the target:   q_cmd = q_d + u
#     e = q_d - q                                   (q = MEASURED joint)
#     u = kp * e  +  ki * integral(e dt)  -  kd * dq/dt
# The I term keeps growing while there is an error: that is what removes the sag.
#
# ------------------------------------------------------------------------------
# PYTHON QUICK REFERENCE for this task                 MATLAB  ->  Python
# ------------------------------------------------------------------------------
#  object attribute          obj.x                    ->  self.x   (inside the class)
#  "no value yet"            isempty(x)               ->  x is None
#  clamp a number            min(max(x, lo), hi)      ->  float(np.clip(x, lo, hi))
#  clamp a vector            -                        ->  np.clip(v, LOWER, UPPER)
#  loop over 3 lists         for i = 1:n              ->  for a, b, c in zip(A, B, C):
#  append to a list          v(end+1) = x             ->  v.append(x)
#  elapsed time              tic / toc                ->  t0 = time.monotonic(); time.monotonic() - t0
#  numpy array -> list       -                        ->  a.tolist()
# ------------------------------------------------------------------------------
"""Task 6 - Closed-loop joint control with a PID."""
import math
import time

import numpy as np
from std_msgs.msg import Float64MultiArray

from so100_labs import servo_model
from so100_labs.arm import run
from so100_labs.robot_model import ARM_JOINTS, LOWER, POSES, UPPER

OUTPUT_LIMIT = 0.15    # rad, the PID may shift a command by at most this much
INTEGRAL_LIMIT = 0.15  # rad, anti-windup: bound on the integral term
RATE = 20.0            # Hz, control loop rate
PASS_MM = 5.0          # main_sequence succeeds if the TCP ends closer than this to every target

TARGETS = {
    'ready': POSES['ready'],
    'reach': [0.3, 0.2, 0.3, -1.4, 1.57],
}


# %% EX1 - One PID controller (one joint)
class PID:
    def __init__(self, kp, ki, kd, output_limit=OUTPUT_LIMIT, integral_limit=INTEGRAL_LIMIT):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.output_limit, self.integral_limit = output_limit, integral_limit
        self.reset()

    def reset(self):
        """Forget the past: no integral, no previous measurement."""
        self.integral = 0.0
        self.prev_measurement = None

    def update(self, setpoint, measurement, dt):
        """Return the correction u for this time step (a number, in rad)."""
        # 1) Error
        error = ...  # TODO

        # 2) Integral term: add ki * error * dt, then clamp to +-integral_limit (anti-windup)
        self.integral = ...  # TODO
        self.integral = ...  # TODO (hint: float(np.clip(...)))

        # 3) Derivative term ON THE MEASUREMENT: -kd * (measurement - prev) / dt
        #    (0 on the first call, when there is no previous measurement yet)
        derivative = 0.0
        if self.prev_measurement is not None and dt > 0:
            derivative = ...  # TODO
        self.prev_measurement = ...  # TODO remember it for the next call

        # 4) Sum of the three terms, clamped to +-output_limit
        u = ...  # TODO P + I + D
        u = ...  # TODO
        return u

        # QUESTION: why is the derivative taken on the measurement and not on the error?
        # (hint: what happens to d(error)/dt when the setpoint jumps?)


# %% EX2 - Correct the command of every joint
def corrected_command(target, measured, pids, dt):
    """q_cmd = target + u for every joint, clipped to the joint limits. Returns a list."""
    cmd = []
    for t, m, pid in zip(target, measured, pids):
        u = ...  # TODO call this joint's PID
        ...      # TODO add t + u at the end of cmd
    cmd = ...    # TODO clip to [LOWER, UPPER] (hint: np.clip)
    return cmd.tolist()


# %% EX3 - The control loop
def hold_with_pid(arm, target, pids, duration=3.0, rate=RATE):
    """Hold `target` with the PID for `duration` seconds. Returns [(t, error), ...]."""
    for pid in pids:
        pid.reset()
    period = 1.0 / rate
    history = []
    start = last = time.monotonic()
    next_tick = start
    while time.monotonic() - start < duration:
        now = time.monotonic()
        # 1) Real time since the last step (a loop never runs exactly on time)
        dt = (now - last) if history else period
        last = now

        q = ...    # TODO measured joints (hint: arm.joint_positions())
        cmd = ...  # TODO your corrected_command()
        ...        # TODO send it: arm.stream_position(cmd, period)

        error = ...  # TODO list of target - q for every joint (hint: zip)
        history.append((now - start, error))
        report_error(arm, error)  # for rqt_plot

        # 2) Wait until the next tick (fixed rate)
        next_tick += period
        arm.sleep(max(0.0, next_tick - time.monotonic()))
    return history


# %% EX4 - Tune the gains
# 1) Only kp: the error shrinks but never reaches 0.  2) Add ki: the error goes to 0.
# Check with:  ros2 run so100_labs task6_pid_tune   and the tests
# (error < 0.003 rad after 2 s, overshoot < 0.02 rad).
KP = 0.3
KI = ...   # TODO start at 1.0 and raise it until the tests pass
KD = 0.0

GAINS = {name: {'kp': KP, 'ki': KI, 'kd': KD} for name in ARM_JOINTS}

# QUESTION: gravity sag is predictable (servo_model.gravity_torques). Could you add it to the
# command BEFORE the error appears (feed-forward)? What would the PID still be needed for?


# %% PROVIDED -------------------------------------------------------------------
def make_pids(gains=GAINS):
    return [PID(**gains[name]) for name in ARM_JOINTS]


def tcp_error_mm(q_target, q_measured):
    d = servo_model.tcp_position(q_measured) - servo_model.tcp_position(q_target)
    return 1000.0 * float(np.linalg.norm(d))


def report_error(arm, error):
    if not hasattr(arm, '_pid_error_pub'):
        arm._pid_error_pub = arm.create_publisher(Float64MultiArray, '/so100_labs/pid_error', 10)
    arm._pid_error_pub.publish(Float64MultiArray(data=[float(e) for e in error]))


def simulate(gains=GAINS, target=POSES['ready'], duration=3.0, rate=RATE, start=None):
    """Run your PID on the servo model (no ROS). Returns (times, errors)."""
    sim = servo_model.ServoArmSim(servo_model.steady_state(target) if start is None else start)
    pids = make_pids(gains)
    dt = 1.0 / rate
    q = sim.measured()
    times, errors = [], []
    for k in range(int(round(duration * rate))):
        q = sim.step(corrected_command(target, q, pids, dt), dt)
        times.append((k + 1) * dt)
        errors.append(np.asarray(target) - q)
    return np.array(times), np.array(errors)


def tune():
    times, errors = simulate()
    print('Closed-loop response on the servo model (target: ready, starting sagged)')
    header = '  '.join(f'{j.split("_")[0][:8]:>8s}' for j in ARM_JOINTS)
    print(f'   t [s]  {header}   TCP [mm]')
    target = POSES['ready']
    for t, e in zip(times, errors):
        if abs(t * 10 - round(t * 10)) < 1e-6 or t == times[0]:
            err_mm = tcp_error_mm(target, np.asarray(target) - e)
            print(f'  {t:5.2f}   ' + '  '.join(f'{v:+8.4f}' for v in e) + f'   {err_mm:7.1f}')


def log_pose_error(arm, name, label, target):
    q = arm.joint_positions()
    err = [t - m for t, m in zip(target, q)]
    worst = max(err, key=abs)
    mm = tcp_error_mm(target, q)
    arm.get_logger().info(f'{name:6s} {label:10s} joint error {np.round(err, 3)} '
                          f'(worst {math.degrees(worst):+.1f} deg) -> TCP off by {mm:.1f} mm')
    return mm


def save_history(history, path):
    """Write (t, errors) to a CSV file you can plot."""
    with open(path, 'w') as f:
        f.write('t,' + ','.join(ARM_JOINTS) + '\n')
        for t, err in history:
            f.write(f'{t:.3f},' + ','.join(f'{e:.5f}' for e in err) + '\n')


def main_sequence(arm):
    """For each target: move open loop, measure, hold with your PID, measure again."""
    ok = True
    for name, target in TARGETS.items():
        arm.move_to(target)
        arm.sleep(1.0)
        open_loop = log_pose_error(arm, name, 'open loop', target)
        history = hold_with_pid(arm, target, make_pids(), duration=3.0)
        closed_loop = log_pose_error(arm, name, 'with PID', target)
        save_history(history, f'/tmp/task6_{name}.csv')
        if open_loop < 1.0:
            arm.get_logger().warn('No error without the PID: is the arm sagging? '
                                  'Launch arm.launch.py sag:=true (or use the real robot).')
        ok = ok and closed_loop < PASS_MM
    arm.get_logger().info('Error history saved in /tmp/task6_*.csv')
    return ok and arm.move_to(POSES['home'])


def main():
    run(main_sequence)


if __name__ == '__main__':
    main()
