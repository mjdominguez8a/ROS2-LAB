# ======  MEDICAL ROBOTICS AND TECHNOLOGIES  ======
# ======  FOR COMPUTER AIDED SURGERY LABORATORY  ======
#
#             Hands-on ROS2 - SO-100 arm - Task 1: Joint-space commands
#                                   2026-2027
#
# Complete every line marked with  # TODO : replace the `...` with your code.
# Each TODO is ONE short line of Python. The hints tell you which Python tool to use.
#
# Goal: drive the arm through a sequence of named poses by sending joint
# positions directly to `forward_position_controller`.
#
# Run (simulation, terminal 1):
#     ros2 launch so100_bringup arm.launch.py controller:=position
# Run your code (terminal 2):
#     ros2 run so100_labs task1_joint_commands
# Test:
#     pytest src/so100_labs/test/test_task1_joint_commands.py
#
# forward_position_controller applies every command immediately, so a big jump
# would make the arm snap to the target. That is why you send many small steps
# (`interpolate_steps`) at RATE_HZ. In Task 2 a trajectory controller will do
# this for you.
#
# ------------------------------------------------------------------------------
# PYTHON QUICK REFERENCE for this task                 MATLAB  ->  Python
# ------------------------------------------------------------------------------
#  list of numbers           q = [1 2 3]              ->  q = [1, 2, 3]
#  indexing STARTS AT 0      q(1), q(end)             ->  q[0], q[-1]
#  empty list, add at end    q = []; q(end+1) = v     ->  q = []   then   q.append(v)
#  length                    numel(q)                 ->  len(q)
#  for loop                  for k = 1:n ... end      ->  for k in range(1, n + 1):
#                            range(a, b) gives a, a+1, ..., b-1  (b is EXCLUDED)
#  blocks                    if/for ... end           ->  ':' then INDENTED lines, no 'end'
#  if / elseif / else        if a < b ... elseif      ->  if a < b:  /  elif ...:  /  else:
#  abs, max, min             abs(x), max(a,b)         ->  abs(x), max(a, b), min(a, b)
#  round up                  ceil(x)                  ->  math.ceil(x)   (needs `import math`)
#  division                  7 / 2 = 3.5              ->  7 / 2 = 3.5    (7 // 2 = 3 is integer)
#  struct / dictionary       s.shoulder               ->  d['shoulder']
#  convert to real number    double(x)                ->  float(x)
#  loop over a list          for i=1:numel(v), x=v(i) ->  for x in v:
#  loop over 2 lists at once                          ->  for a, b in zip(list_a, list_b):
# ------------------------------------------------------------------------------
"""Task 1 - Joint-space commands."""
import math  # noqa: F401  (you will need it)

from so100_labs.arm import run
from so100_labs.robot_model import ARM_JOINTS, LOWER, POSES, UPPER  # noqa: F401

RATE_HZ = 50.0
MAX_STEP = 0.02  # rad per command -> at most 1 rad/s at 50 Hz


# %% EX1 - Order the joints
def order_joints(named_positions):
    """Convert {joint_name: position} into a list ordered like ARM_JOINTS.

    Example: {'wrist_roll_joint': 1.0, 'shoulder_pan_joint': 0.5, ...}
             -> [0.5, ..., 1.0]
    Values must be floats. If a joint is missing, raise KeyError.
    """
    # named_positions is a DICTIONARY: a set of key -> value pairs, e.g.
    #     named_positions = {'shoulder_pan_joint': 0.5, 'wrist_roll_joint': 1.0, ...}
    #     named_positions['shoulder_pan_joint']   ->  0.5
    # ARM_JOINTS is a LIST of joint names, in the order the controller wants.
    # NB: reading a key that is not in the dictionary already raises KeyError.

    q = []                          # empty list (MATLAB: q = [])
    for name in ARM_JOINTS:         # `name` takes the value of each joint name in turn
        value = ...  # TODO read the value stored under the key `name`
        value = ...  # TODO convert it to a real number (hint: float())
        ...          # TODO add `value` at the end of the list q
    return q


# %% EX2 - Respect the joint limits
def clamp_to_limits(q):
    """Return a copy of `q` where every joint is clamped into [LOWER[i], UPPER[i]]."""
    # LOWER and UPPER are lists with the lower / upper limit of each joint [rad].
    # zip(q, LOWER, UPPER) walks the three lists TOGETHER: at every iteration you get
    # value = q[i], low = LOWER[i], high = UPPER[i] for the same joint i.

    q_clamped = []
    for value, low, high in zip(q, LOWER, UPPER):
        if ...:              # TODO is value below the lower limit?
            value = ...      # TODO
        elif ...:            # TODO is value above the upper limit?
            value = ...      # TODO
        q_clamped.append(value)
    return q_clamped

    # EXTRA: the whole if/elif can be written in one line with min() and max(). How?


# %% EX3 - Split a motion into small steps
def interpolate_steps(q_start, q_goal, max_step=MAX_STEP):
    """Split the motion q_start -> q_goal into small, evenly spaced steps.

    Return a list of joint vectors. Rules:
      * no joint may change by more than `max_step` between consecutive vectors
        (the first vector is compared to q_start),
      * use as few steps as possible (n = ceil(largest joint change / max_step), at least 1),
      * all joints move simultaneously (linear interpolation),
      * the last vector equals q_goal; q_start itself is NOT included.
    """
    # 1) Largest change among all the joints
    delta = 0.0
    for start, goal in zip(q_start, q_goal):
        change = ...  # TODO absolute change of this joint (hint: abs())
        if change > delta:
            delta = change

    # 2) Number of steps: delta / max_step rounded UP, and at least 1
    n = ...  # TODO round up (hint: math.ceil())
    n = ...  # TODO make sure n >= 1 (hint: max())

    # 3) Linear interpolation: at step k (k = 1, ..., n) every joint has done
    #    the fraction k/n of its motion:   value = start + (goal - start) * k / n
    steps = []                          # list of lists (one joint vector per step)
    for k in range(1, n + 1):           # k = 1, 2, ..., n
        fraction = ...  # TODO
        q_k = []                        # joint vector of step k
        for start, goal in zip(q_start, q_goal):
            value = ...  # TODO position of this joint at step k
            q_k.append(value)
        ...  # TODO add q_k at the end of steps
    return steps

    # QUESTION: with MAX_STEP = 0.02 rad and RATE_HZ = 50 Hz, what is the
    # maximum joint velocity? Why do all joints reach the goal at the same time?


# %% Main sequence (PROVIDED - read it!)
def main_sequence(arm):
    """Visit home -> ready -> left -> right -> home."""
    sequence = ['home', 'ready', 'left', 'right', 'home']
    q = arm.joint_positions()
    for name in sequence:
        goal = clamp_to_limits(POSES[name])
        arm.get_logger().info(f'Moving to {name}: {goal}')   # f-string: {x} inserts x
        for step in interpolate_steps(q, goal):
            arm.send_joint_positions(step)
            arm.sleep(1.0 / RATE_HZ)
        arm.sleep(0.5)
        q = goal
    return True


def main():
    run(main_sequence)


if __name__ == '__main__':
    main()
