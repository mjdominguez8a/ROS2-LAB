# ======  MEDICAL ROBOTICS AND TECHNOLOGIES  ======
# ======  FOR COMPUTER AIDED SURGERY LABORATORY  ======
#
#   Hands-on ROS2 - SO-100 arm - Task 5: Cartesian paths, square, pick & place
#                                   2026-2027
#
# Complete every line marked with  # TODO : replace the `...` with your code.
# Each TODO is ONE short line of Python. The hints tell you which Python tool to use.
#
# Goal: move the TCP along STRAIGHT LINES in Cartesian space (not joint space)
# by running IK on many closely spaced points, and combine this with the gripper.
#
# Run (Gazebo has a table and a cube; terminal 1):
#     ros2 launch so100_bringup arm.launch.py backend:=gazebo
# Run your code (terminal 2):
#     ros2 run so100_labs task5_cartesian
# Test:  pytest src/so100_labs/test/test_task5_cartesian.py
# Requires Tasks 3 and 4.
#
# ------------------------------------------------------------------------------
# PYTHON QUICK REFERENCE for this task                 MATLAB  ->  Python
# ------------------------------------------------------------------------------
#  call a function from another file  (on the path)  ->  from module import function
#  tuple (fixed list)        p = [x y z]              ->  p = (x, y, z)   p[2] is z
#  numpy array -> list       -                        ->  a.tolist()
#  distance                  norm(p1 - p0)            ->  np.linalg.norm(p1 - p0)
#  range including n         0:n                      ->  range(n + 1)
#  logical not               ~ok                      ->  not ok
#  function that returns True/False:   if not f(x): return False
#  function inside a function (local helper)          ->  def above(p): ...
# ------------------------------------------------------------------------------
"""Task 5 - Cartesian paths: draw a square, then pick and place."""
import math

import numpy as np  # noqa: F401

from so100_labs.arm import run
from so100_labs.robot_model import GRIPPER_CLOSED, GRIPPER_OPEN, MAX_VELOCITY, POSES  # noqa: F401
from so100_labs.task4_inverse_kinematics import inverse_kinematics

DOWN = math.radians(90)  # gripper pointing straight down


# %% EX1 - Points on a straight segment
def linear_interpolation(p0, p1, step=0.005):
    """Points on the straight segment p0 -> p1, INCLUDING both ends, spaced at most `step` m
    apart and evenly spaced. Return a list of [x, y, z] lists."""
    p0 = np.asarray(p0, dtype=float)
    p1 = ...  # TODO same for p1

    length = ...  # TODO length of the segment (hint: np.linalg.norm)
    n = ...       # TODO number of intervals: length / step rounded UP (math.ceil)
    n = ...       # TODO at least 1 (hint: max())

    points = []
    for k in range(...):  # TODO k must go 0, 1, ..., n  (both ends included!)
        p_k = ...         # TODO point k:  p0 + (p1 - p0) * k / n
        points.append(...)  # TODO add it AS A LIST (hint: .tolist())
    return points


def square_corners(center, side):
    """Provided: 5 corners of a horizontal square (the first corner is repeated at the end)."""
    cx, cy, cz = center
    h = side / 2
    return [(cx - h, cy - h, cz), (cx + h, cy - h, cz), (cx + h, cy + h, cz),
            (cx - h, cy + h, cz), (cx - h, cy - h, cz)]


# %% EX2 - From Cartesian points to joint vectors
def cartesian_path(points, pitch, q_seed):
    """Joint vectors for every Cartesian point (same pitch for all).
    Seed each IK call with the previous solution so the arm moves continuously."""
    # inverse_kinematics(target_xyz, pitch, q_seed) is your function from Task 4
    path = []
    q = q_seed
    for p in points:
        q = ...  # TODO IK for point p, seeded with the PREVIOUS solution q
        ...      # TODO add q at the end of path
    return path

    # QUESTION: what could happen if you always used q_seed as seed instead?


# %% EX3 - Time parametrisation
def time_parametrize(joint_path, q_start, max_velocity=MAX_VELOCITY * 0.5, min_dt=0.05):
    """Times (seconds from start) to reach each joint vector of joint_path, starting from q_start.
    Each step lasts max(min_dt, largest joint change / max_velocity)."""
    times = []
    t = 0.0
    q_prev = q_start
    for q in joint_path:
        # largest joint change between q_prev and q
        delta = 0.0
        for a, b in zip(q, q_prev):
            change = ...  # TODO absolute change of this joint
            delta = ...   # TODO keep the largest one (hint: max())

        step_time = ...   # TODO duration of this step (see docstring)
        t = ...           # TODO
        times.append(t)
        q_prev = ...      # TODO the next step starts from q
    return times


def follow_cartesian(arm, points, pitch):
    """Provided: move the TCP through `points` along the straight lines between them."""
    q0 = arm.joint_positions()
    path = cartesian_path(points, pitch, q0)
    return arm.follow_trajectory(time_parametrize(path, q0), path)


def draw_square(arm, center=(0.0, -0.24, 0.05), side=0.08):
    """Provided: draw a square with the TCP pointing down."""
    corners = square_corners(center, side)
    arm.show_points(corners, ns='square')
    if not arm.move_to(inverse_kinematics(corners[0], DOWN, arm.joint_positions())):
        return False
    points = []
    for a, b in zip(corners[:-1], corners[1:]):
        points += linear_interpolation(a, b)[1:]
    return follow_cartesian(arm, points, DOWN)


# %% EX4 - Pick and place
def pick_and_place(arm, pick=(0.08, -0.24, 0.013), place=(-0.08, -0.22, 0.013), hover=0.06):
    """Pick an object at `pick` and put it down at `place`.

    Sequence (every step must succeed, i.e. return True):
      1. open the gripper
      2. move (joint space) above the pick point
      3. straight line down to the pick point
      4. close the gripper
      5. straight line up, then over to above the place point, then down
      6. open the gripper
      7. straight line back up
    "Above" means `hover` metres higher in z. Return True if everything succeeded.
    """
    arm.show_points([pick, place], ns='pick_place', color=(0.9, 0.2, 0.2))

    def above(p):
        """Point `hover` metres above p (p is a tuple (x, y, z))."""
        return (p[0], p[1], p[2] + hover)

    # Useful calls (each returns True if it worked):
    #   arm.gripper(GRIPPER_OPEN)  /  arm.gripper(GRIPPER_CLOSED)
    #   follow_cartesian(arm, linear_interpolation(start_point, end_point), DOWN)
    # Steps 1-3 are GIVEN as examples. Steps 4-7: write the call after `if not`.

    # 1. Open the gripper
    if not arm.gripper(GRIPPER_OPEN):
        return False

    # 2. Joint-space motion above the pick point
    if not arm.move_to(inverse_kinematics(above(pick), DOWN, arm.joint_positions())):
        return False

    # 3. Straight line down to the pick point
    if not follow_cartesian(arm, linear_interpolation(above(pick), pick), DOWN):
        return False

    # 4. Close the gripper
    if not ...:  # TODO
        return False

    # 5a. Straight line up (from pick to above the pick point)
    if not ...:  # TODO
        return False

    # 5b. Straight line from above the pick point to above the place point
    if not ...:  # TODO
        return False

    # 5c. Straight line down to the place point
    if not ...:  # TODO
        return False

    # 6. Open the gripper (release the object)
    if not ...:  # TODO
        return False

    # 7. Straight line back up
    if not ...:  # TODO
        return False

    return True

    # QUESTION: why do we go down/up with follow_cartesian and not with move_to?


# %% Main sequence (PROVIDED)
def main_sequence(arm):
    ok = draw_square(arm) and pick_and_place(arm)
    return ok and arm.move_to(POSES['home'])


def main():
    run(main_sequence)


if __name__ == '__main__':
    main()
