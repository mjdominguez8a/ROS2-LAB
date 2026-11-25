# Task 4: End-effector commands (inverse kinematics)

**File:** `src/so100_labs/so100_labs/task4_inverse_kinematics.py`

Inverse kinematics (IK) answers the opposite question: _which joint angles put the gripper here?_

The SO-100 has only **5 joints**, so it cannot reach an arbitrary position **and** orientation (6
numbers). We ask for 4 things: the TCP position `(x, y, z)` and the **approach pitch** (0 =
horizontal, 90° = pointing down), and solve for the first 4 joints. `wrist_roll` stays as it is.

## What to implement

1. `numerical_jacobian(q)`: 4×4 matrix of ∂[x, y, z, pitch]/∂q, from finite differences of your FK.
2. `inverse_kinematics(target_xyz, pitch, q_seed)`: damped least squares iterations (algorithm in
   the file). Raise `UnreachableError` if it doesn't converge.

## Run it

```bash
ros2 launch so100_bringup arm.launch.py           # terminal 1
ros2 run so100_labs task4_inverse_kinematics      # terminal 2: visits 4 targets (green dots in RViz)
pytest src/so100_labs/test/test_task4_inverse_kinematics.py
```

## Think about it

- Try a target 1 m away. What does your solver do? What should it do?
- Set `damping=0` and send the arm to a fully stretched pose. What happens near a singularity?
- Two different joint solutions can reach the same point ("elbow up" and "elbow down"). Which one
  does your solver find, and what decides it?
