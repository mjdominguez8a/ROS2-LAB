# Task 2: Joint trajectories

**File:** `src/so100_labs/so100_labs/task2_trajectories.py`

Instead of streaming positions yourself, send a whole **trajectory** (positions + velocities +
timestamps) to `joint_trajectory_controller` through the `FollowJointTrajectory` **action**. The
controller interpolates between your points at 50 Hz and reports success or failure when it's done.

## What to implement

1. `quintic(q0, q1, T, t)`: smooth blend with zero velocity and acceleration at both ends (the
   formula is in the file).
2. `minimum_duration(q0, q1, vmax)`: the fastest duration that keeps every joint under `vmax`. The
   quintic's peak velocity is 15/8 × Δq / T.
3. `sample_trajectory(waypoints, vmax, dt)`: chain segments, stop at each waypoint, sample every
   `dt`.

## Run it

```bash
ros2 launch so100_bringup arm.launch.py           # terminal 1
ros2 run so100_labs task2_trajectories            # terminal 2
pytest src/so100_labs/test/test_task2_trajectories.py
```

## Think about it

- Plot one joint's position and velocity (e.g. with matplotlib) for a cubic vs a quintic blend.
  Which one has a jump in acceleration?
- `Arm.follow_trajectory` rejects trajectories faster than `MAX_VELOCITY`. Find the check in
  `arm.py`.
- Watch `ros2 action list` and `ros2 topic echo /joint_trajectory_controller/controller_state` while
  it runs.
