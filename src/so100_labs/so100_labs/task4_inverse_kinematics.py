# ======  MEDICAL ROBOTICS AND TECHNOLOGIES  ======
# ======  FOR COMPUTER AIDED SURGERY LABORATORY  ======
#
#      Hands-on ROS2 - SO-100 arm - Task 4: End-effector commands with IK
#                                   2026-2027
#
# Complete every line marked with  # TODO : replace the `...` with your code.
# Each TODO is ONE short line of Python. The hints tell you which Python tool to use.
#
# Goal: command the arm in Cartesian space: "put the gripper at (x, y, z) with
# this approach pitch". The SO-100 has only 5 joints, so an arbitrary 6-D pose
# (position + full orientation) is NOT reachable. We control 4 things with the
# first 4 joints:
#     task vector f(q) = [x, y, z, pitch]
# and keep wrist_roll at whatever value the seed has.
#
# Method - damped least squares (Levenberg-Marquardt):
#     repeat:
#         e   = target - f(q)
#         J   = d f / d q[0:4]          (4x4, by finite differences)
#         dq  = J^T (J J^T + lambda^2 I)^-1 e
#         q[0:4] += dq, then clamp q[0:4] into the joint limits
#     until |e_position| < tol (and |e_pitch| < 10*tol), else raise UnreachableError
#
# Run (simulation, terminal 1):  ros2 launch so100_bringup arm.launch.py
# Run your code (terminal 2):    ros2 run so100_labs task4_inverse_kinematics
# Test:  pytest src/so100_labs/test/test_task4_inverse_kinematics.py
# Requires Task 3 (forward kinematics).
#
# ------------------------------------------------------------------------------
# PYTHON QUICK REFERENCE for this task                 MATLAB  ->  Python
# ------------------------------------------------------------------------------
#  COPY an array             b = a  (MATLAB copies)   ->  b = a.copy()
#                            In Python  b = a  does NOT copy: b and a are the SAME
#                            array, changing b(1) would also change a!
#  zeros                     zeros(4, 4)              ->  np.zeros((4, 4))   (double brackets)
#  column i                  J(:, i)                  ->  J[:, i]
#  first 4 elements          q(1:4)                   ->  q[:4]       (indices 0, 1, 2, 3)
#  4th element               e(4)                     ->  e[3]
#  transpose                 J'                       ->  J.T
#  solve A x = b             x = A \ b                ->  x = np.linalg.solve(A, b)
#  logical and / or / not    &&  ||  ~                ->  and  or  not
#  clamp element-wise        min(max(x, lo), hi)      ->  np.clip(x, lo, hi)
#  exit a loop               break                    ->  break
#  raise an error            error('msg')             ->  raise UnreachableError('msg')
# ------------------------------------------------------------------------------
"""Task 4 - End-effector commands with inverse kinematics (IK)."""
import math

import numpy as np

from so100_labs.arm import run
from so100_labs.robot_model import LOWER, POSES, UPPER  # noqa: F401
from so100_labs.task3_forward_kinematics import approach_pitch, tcp_position


class UnreachableError(ValueError):
    pass


def task_vector(q):
    """Provided: [x, y, z, pitch] of the TCP for joint vector q."""
    return np.array([*tcp_position(q), approach_pitch(q)])


# %% EX1 - Numerical Jacobian
def numerical_jacobian(q, eps=1e-6):
    """4x4 Jacobian of task_vector with respect to the first 4 joints (forward differences)."""
    # Column i of J tells how [x, y, z, pitch] change when ONLY joint i moves:
    #     J[:, i] = ( f(q + eps * e_i) - f(q) ) / eps
    q = np.asarray(q, dtype=float)
    f0 = task_vector(q)                 # f(q), computed once

    J = np.zeros((4, 4))
    for i in range(4):                  # i = 0, 1, 2, 3  -> the first 4 joints
        q_pert = ...     # TODO a COPY of q (not q itself!)
        q_pert[i] = ...  # TODO move only joint i by eps
        f1 = ...         # TODO task vector at q_pert
        J[:, i] = ...    # TODO forward difference -> column i
    return J


# %% EX2 - Damped least squares IK
def inverse_kinematics(target_xyz, pitch, q_seed, tol=1e-4, max_iterations=200, damping=0.01):
    """Joint vector (list of 5) that puts the TCP at target_xyz with the given pitch.

    Start from q_seed (5 values; q_seed[4] = wrist roll is kept unchanged).
    Tip: weight the pitch row lower than the position rows (e.g. 0.1), since 1 rad of
    pitch is a much bigger number than a few centimetres of position.
    Raise UnreachableError if it does not converge within max_iterations.
    """
    q = np.array(q_seed, dtype=float)   # 5 joints (a new array); only q[:4] will be changed
    x, y, z = target_xyz                # unpack the target position

    target = ...  # TODO numpy array [x, y, z, pitch]
    W = np.array([1.0, 1.0, 1.0, 0.1])  # weights of the rows [x, y, z, pitch]

    converged = False
    for iteration in range(max_iterations):

        # 1) Error and convergence check
        e = ...               # TODO task-space error
        position_error = ...  # TODO norm of the first 3 entries of e
        pitch_error = ...     # TODO absolute value of the pitch entry (index 3)
        if position_error < tol and pitch_error < 10 * tol:
            converged = True
            break                       # leave the for loop

        # 2) Weighted Jacobian and error: row i multiplied by W[i]
        #    W[:, None] turns W into a 4x1 column, so J * W[:, None] scales each ROW
        J = numerical_jacobian(q)
        J_w = ...  # TODO
        e_w = ...  # TODO element-wise product of e and W

        # 3) Damped least squares step  dq = J_w^T (J_w J_w^T + lambda^2 I)^-1 e_w
        A = ...    # TODO J_w J_w^T + damping^2 I   (use @, .T, **, np.eye)
        sol = ...  # TODO solve A sol = e_w  (np.linalg.solve, NOT inv)
        dq = ...   # TODO

        # 4) Update ONLY the first 4 joints and clamp them into the limits
        q[:4] = ...  # TODO
        q[:4] = ...  # TODO (hint: np.clip with LOWER[:4] and UPPER[:4])

    if not converged:
        raise UnreachableError(f'IK did not converge for {target_xyz}, pitch {pitch:.2f}')

    return q.tolist()                   # numpy array -> list

    # QUESTION: what happens with damping = 0 near a singular configuration?
    # And with a very large damping?


TARGETS = [
    # x, y, z [m] in base_link, pitch [rad]
    ((0.0, -0.25, 0.10), math.radians(60)),
    ((0.10, -0.22, 0.05), math.radians(90)),
    ((-0.10, -0.22, 0.05), math.radians(90)),
    ((0.0, -0.30, 0.15), math.radians(30)),
]


# %% Main sequence (PROVIDED)
def main_sequence(arm):
    """Visit each Cartesian target and report the error measured with TF."""
    q = arm.joint_positions()
    arm.show_points([t for t, _ in TARGETS])
    for xyz, pitch in TARGETS:
        try:                                    # try/except = MATLAB try/catch
            q = inverse_kinematics(xyz, pitch, q)
        except UnreachableError as e:
            arm.get_logger().error(str(e))
            return False
        if not arm.move_to(q):
            return False
        arm.sleep(0.3)
        err = np.linalg.norm(arm.tcp_position() - np.array(xyz))
        arm.get_logger().info(f'target {xyz}: reached with error {err * 1000:.1f} mm')
    return arm.move_to(POSES['home'])


def main():
    run(main_sequence)


if __name__ == '__main__':
    main()
