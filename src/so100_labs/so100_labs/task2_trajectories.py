# ======  MEDICAL ROBOTICS AND TECHNOLOGIES  ======
# ======  FOR COMPUTER AIDED SURGERY LABORATORY  ======
#
#             Hands-on ROS2 - SO-100 arm - Task 2: Joint trajectories
#                                   2026-2027
#
# Complete every line marked with  # TODO : replace the `...` with your code.
# Each TODO is ONE short line of Python. The hints tell you which Python tool to use.
#
# Goal: generate a smooth, time-parametrised trajectory through several poses and
# let `joint_trajectory_controller` execute it (FollowJointTrajectory action).
#
# Run (simulation, terminal 1):
#     ros2 launch so100_bringup arm.launch.py
# Run your code (terminal 2):
#     ros2 run so100_labs task2_trajectories
# Test:
#     pytest src/so100_labs/test/test_task2_trajectories.py
#
# Background - quintic (5th order) blend with zero velocity and acceleration at both ends:
#     s = t / T                       (0 <= s <= 1)
#     b(s) = 10 s^3 - 15 s^4 + 6 s^5
#     q(t) = q0 + (q1 - q0) * b(s)
#     q'(t) = (q1 - q0) * b'(s) / T,   b'(s) = 30 s^2 - 60 s^3 + 30 s^4
# The peak of b'(s) is 15/8 (at s = 0.5).
#
# ------------------------------------------------------------------------------
# PYTHON QUICK REFERENCE for this task                 MATLAB  ->  Python
# ------------------------------------------------------------------------------
#  POWER                     s^3                      ->  s ** 3     (^ is NOT power!)
#  numpy                     (built in)               ->  import numpy as np
#  list -> numpy array       -                        ->  a = np.asarray(q, dtype=float)
#  numpy array -> list       -                        ->  a.tolist()
#  element-wise math         a .* b, a + b            ->  a * b, a + b   (numpy arrays)
#  max(abs(a - b))           max(abs(a - b))          ->  np.max(np.abs(a - b))
#  two outputs               [p, v] = f(x)            ->  p, v = f(x)
#                            function [p, v] = f(x)   ->  def f(x): ... return p, v
#  slicing                   W(1:end-1), W(2:end)     ->  W[:-1], W[1:]
#  round to nearest integer  round(x)                 ->  round(x)
#
#  WARNING - with plain Python LISTS math does not work element-wise:
#      [1, 2] + [3, 4]  ->  [1, 2, 3, 4]     (concatenation!)
#      2 * [1, 2]       ->  [1, 2, 1, 2]     (repetition!)
#  Convert lists to numpy arrays (np.asarray) before doing math on joint vectors.
# ------------------------------------------------------------------------------
"""Task 2 - Joint trajectories."""
import math  # noqa: F401

import numpy as np  # noqa: F401

from so100_labs.arm import run
from so100_labs.robot_model import MAX_VELOCITY, POSES

PEAK_VELOCITY_FACTOR = 15.0 / 8.0


# %% EX1 - Quintic blend
def quintic(q0, q1, duration, t):
    """Position and velocity of the quintic blend from q0 to q1 at time t.

    q0, q1: joint vectors (lists or arrays). Returns (position, velocity) as numpy arrays.
    For t < 0 or t > duration, clamp t into [0, duration].
    """
    q0 = np.asarray(q0, dtype=float)    # list -> numpy array (so that math is element-wise)
    q1 = ...  # TODO same for q1

    # 1) Normalised time, clamped into [0, 1]
    s = ...  # TODO s = t / T
    s = ...  # TODO clamp into [0, 1] (hint: min(max(s, 0.0), 1.0))

    # 2) Blend b(s) and its derivative b'(s)  (formulas in the header, use ** for powers)
    b = ...   # TODO
    db = ...  # TODO

    # 3) Position and velocity of all joints at once (q0, q1 are numpy arrays)
    position = ...  # TODO q(t)
    velocity = ...  # TODO q'(t), do not forget the 1/T
    return position, velocity           # two outputs


# %% EX2 - Shortest duration that respects the velocity limit
def minimum_duration(q0, q1, max_velocity=MAX_VELOCITY):
    """Shortest duration T so that no joint exceeds max_velocity during quintic(q0, q1, T, .).

    Hint: the joint that moves the furthest is the limiting one.
    """
    # The peak velocity of a joint moving by `delta` in time T is
    #     v_peak = delta * PEAK_VELOCITY_FACTOR / T
    # Impose v_peak = max_velocity for the joint with the LARGEST delta and solve for T.
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)

    delta = ...  # TODO largest absolute change (hint: np.max, np.abs)
    T = ...      # TODO
    return T


# %% EX3 - Chain the segments and sample them
def sample_trajectory(waypoints, max_velocity=MAX_VELOCITY, dt=0.1):
    """Chain quintic segments between consecutive waypoints and sample them every dt seconds.

    Returns (times, positions, velocities):
      times      : increasing list of seconds from the start, first element = dt
      positions  : list of joint vectors (lists), one per time
      velocities : list of joint velocity vectors (lists), one per time
    Each segment lasts minimum_duration(...) rounded UP to a multiple of dt (at least dt),
    and the robot stops (zero velocity) at every waypoint. The last sample is exactly the
    last waypoint.
    """
    times = []
    positions = []
    velocities = []
    t_offset = 0.0      # time at which the current segment starts

    # waypoints[:-1] = all but the last,  waypoints[1:] = all but the first.
    # zip pairs them: (W0, W1), (W1, W2), (W2, W3), ...  -> one pair per segment
    for q0, q1 in zip(waypoints[:-1], waypoints[1:]):

        # 1) Duration of this segment
        duration = ...  # TODO call your minimum_duration()
        duration = ...  # TODO at least dt (hint: max())
        duration = ...  # TODO round UP to a multiple of dt (math.ceil)
        n = round(duration / dt)    # number of samples (an integer, needed by range)

        # 2) Sample at t = dt, 2 dt, ..., n dt   (t = 0 is the end of the previous segment)
        for k in range(1, n + 1):
            t_local = ...           # TODO time inside this segment
            q, v = ...              # TODO call quintic() (it returns TWO values)
            times.append(...)       # TODO ABSOLUTE time (remember t_offset)
            positions.append(q.tolist())        # numpy array -> list
            velocities.append(...)  # TODO same for v

        # 3) The next segment starts where this one ends
        t_offset = ...  # TODO

    return times, positions, velocities

    # QUESTION: why is the velocity zero at every waypoint? What would you
    # change to pass through the intermediate poses without stopping?


# %% Main sequence (PROVIDED)
def main_sequence(arm):
    """Plan through several poses and execute in one trajectory."""
    names = ['home', 'ready', 'left', 'right', 'ready', 'home']
    waypoints = [arm.joint_positions()] + [POSES[n] for n in names]
    times, positions, velocities = sample_trajectory(waypoints)
    arm.get_logger().info(f'Sending {len(times)} points, {times[-1]:.1f} s')
    return arm.follow_trajectory(times, positions, velocities)


def main():
    run(main_sequence)


if __name__ == '__main__':
    main()
