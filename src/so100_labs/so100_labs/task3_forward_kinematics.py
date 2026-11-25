# ======  MEDICAL ROBOTICS AND TECHNOLOGIES  ======
# ======  FOR COMPUTER AIDED SURGERY LABORATORY  ======
#
#             Hands-on ROS2 - SO-100 arm - Task 3: Forward kinematics
#                                   2026-2027
#
# Complete every line marked with  # TODO : replace the `...` with your code.
# Each TODO is ONE short line of Python. The hints tell you which Python tool to use.
#
# Goal: compute where the tool centre point (TCP) is, given the 5 joint angles,
# using homogeneous transforms built from the URDF (robot_model.CHAIN).
#
# Run (simulation, terminal 1):
#     ros2 launch so100_bringup arm.launch.py
# Run your code (terminal 2) - it compares your FK with TF from robot_state_publisher:
#     ros2 run so100_labs task3_forward_kinematics
# Test:
#     pytest src/so100_labs/test/test_task3_forward_kinematics.py
#
# URDF conventions (this is exactly what robot_state_publisher does):
#   * For each joint: T = T @ Trans(xyz) @ Rot(rpy) @ Rot(axis, q)
#   * Rot(rpy) = Rz(yaw) @ Ry(pitch) @ Rx(roll)      (fixed-axis roll, pitch, yaw)
#   * Rot(axis, q): rotation by angle q about the unit vector `axis` (Rodrigues)
#   * After the last joint: T = T @ Trans(TCP_XYZ)
#
# ------------------------------------------------------------------------------
# PYTHON QUICK REFERENCE for this task                 MATLAB  ->  Python
# ------------------------------------------------------------------------------
#  matrix (list of ROWS)     [1 0; 0 1]               ->  np.array([[1, 0], [0, 1]])
#  identity, zeros           eye(4), zeros(3,3)       ->  np.eye(4), np.zeros((3, 3))
#  MATRIX PRODUCT            A * B                    ->  A @ B
#                            (in numpy  A * B  is ELEMENT-WISE, like MATLAB A .* B !)
#  transpose                 A'                       ->  A.T
#  sub-matrix                T(1:3, 1:3)              ->  T[0:3, 0:3]  or  T[:3, :3]
#  column                    T(1:3, 4)                ->  T[:3, 3]     (end index EXCLUDED)
#  norm                      norm(v)                  ->  np.linalg.norm(v)
#  cos, sin                  cos(a)                   ->  math.cos(a)
#  unpack several values     a = v(1); b = v(2);      ->  a, b, c = v
#  call a function you wrote                          ->  R = rotation_from_rpy(r, p, y)
# ------------------------------------------------------------------------------
"""Task 3 - Forward kinematics."""
import math

import numpy as np

from so100_labs.arm import run
from so100_labs.robot_model import APPROACH_AXIS, CHAIN, POSES, TCP_XYZ  # noqa: F401


# %% EX1 - Rotation from roll, pitch, yaw
def rotation_from_rpy(roll, pitch, yaw):
    """3x3 rotation matrix Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    # Rx is GIVEN as an example of the numpy syntax (one inner list per row):
    rx = np.array([[1, 0, 0],
                   [0, cr, -sr],
                   [0, sr, cr]])
    ry = ...  # TODO elementary rotation about y (use cp, sp)
    rz = ...  # TODO elementary rotation about z (use cy, sy)

    R = ...  # TODO compose them in the right order, with the MATRIX product @
    return R


# %% EX2 - Rotation about a generic axis (Rodrigues' formula)
def rotation_about_axis(axis, angle):
    """3x3 rotation of `angle` rad about `axis` (normalise it first). Rodrigues' formula:
    R = I + sin(a) K + (1 - cos(a)) K^2, with K the skew-symmetric matrix of the axis."""
    k = np.asarray(axis, dtype=float)       # list -> numpy array
    k = ...  # TODO make it a unit vector (hint: np.linalg.norm)
    kx, ky, kz = k                          # unpack the 3 components

    # Skew-symmetric matrix of k:
    #     [  0  -kz   ky ]
    #     [  kz   0  -kx ]
    #     [ -ky  kx    0 ]
    K = ...  # TODO write it with np.array([[...], [...], [...]])

    identity = np.eye(3)
    # NB: K^2 means K @ K.  K ** 2 would square every ELEMENT: wrong!
    R = ...  # TODO Rodrigues' formula (math.sin, math.cos, @)
    return R


# %% EX3 - Homogeneous transform
def homogeneous(rotation, translation):
    """4x4 homogeneous transform from a 3x3 rotation and a 3-vector translation."""
    T = np.eye(4)
    T[:3, :3] = ...  # TODO top-left 3x3 block
    T[:3, 3] = ...   # TODO first 3 rows of the last column
    return T


# %% EX4 - Forward kinematics
def forward_kinematics(q):
    """4x4 pose of tcp_link in base_link for joint vector q (5 values, ARM_JOINTS order)."""
    # CHAIN is a list with one element per joint, from the base to the wrist.
    # Each element is a TUPLE of 4 values:  (joint_name, xyz, rpy, axis)
    #   xyz  : translation of the joint origin w.r.t. the previous link [m]
    #   rpy  : (roll, pitch, yaw) of the joint origin w.r.t. the previous link [rad]
    #   axis : rotation axis of the joint, in the joint frame
    # NB: homogeneous(R, p) = Trans(p) @ Rot(R), so Trans(xyz) @ Rot(rpy) is ONE matrix.

    T = np.eye(4)
    for joint, qi in zip(CHAIN, q):         # joint = CHAIN[i], qi = q[i]
        name, xyz, rpy, axis = joint        # unpack the 4 fields of this joint
        roll, pitch, yaw = rpy              # unpack the 3 angles

        R_origin = ...  # TODO rotation of the joint origin (your EX1 function)
        T_origin = ...  # TODO fixed transform (your EX3 function)
        R_joint = ...   # TODO rotation of qi about the joint axis (EX2)
        T_joint = ...   # TODO as a 4x4 matrix, translation [0.0, 0.0, 0.0]
        T = ...         # TODO accumulate: post-multiply with @

    T_tcp = ...  # TODO fixed offset to the TCP: no rotation (np.eye(3)), translation TCP_XYZ
    T = ...      # TODO
    return T

    # QUESTION: why do we POST-multiply (T @ T_i) and not pre-multiply?


# ---- Provided helpers built on your forward_kinematics -----------------------------
def tcp_position(q):
    """TCP position [x, y, z] in base_link."""
    return forward_kinematics(q)[:3, 3]


def approach_pitch(q):
    """Angle [rad] of the gripper's approach direction below the horizontal plane.
    pi/2 = pointing straight down, 0 = horizontal."""
    a = forward_kinematics(q)[:3, :3] @ np.asarray(APPROACH_AXIS)
    return math.atan2(-a[2], math.hypot(a[0], a[1]))


# %% Main sequence (PROVIDED)
def main_sequence(arm):
    """Move through poses and compare your FK with TF."""
    worst = 0.0
    for name in ['home', 'ready', 'left', 'right']:
        arm.move_to(POSES[name])
        arm.sleep(0.5)
        q = arm.joint_positions()
        mine, tf = tcp_position(q), arm.tcp_position()
        error = float(np.linalg.norm(mine - tf))
        worst = max(worst, error)
        arm.get_logger().info(f'{name:6s} FK={np.round(mine, 4)} TF={np.round(tf, 4)} '
                              f'error={error * 1000:.2f} mm')
    arm.move_to(POSES['home'])
    return worst < 0.001


def main():
    run(main_sequence)


if __name__ == '__main__':
    main()
