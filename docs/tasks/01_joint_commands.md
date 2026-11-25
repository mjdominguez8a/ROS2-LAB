# Task 1: Joint-space commands

**File:** `src/so100_labs/so100_labs/task1_joint_commands.py`

The SO-100 has 5 arm joints (`shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`,
`wrist_roll`) and a gripper. The simplest way to move it is to send the 5 target angles to
`forward_position_controller` on the topic `/forward_position_controller/commands`
(`std_msgs/Float64MultiArray`). The controller applies them **immediately**.

## What to implement

1. `order_joints(named_positions)`: dict to list in `ARM_JOINTS` order. Topics carry plain arrays,
   so the order matters.
2. `clamp_to_limits(q)`: keep every joint inside its URDF limits (`LOWER`, `UPPER` in
   `robot_model.py`).
3. `interpolate_steps(q_start, q_goal, max_step)`: break a big motion into many small steps so the
   arm moves at most `max_step` rad per command. `main_sequence` sends one step every 1/50 s.

## Run it

```bash
ros2 launch so100_bringup arm.launch.py controller:=position    # terminal 1
ros2 run so100_labs task1_joint_commands                        # terminal 2
pytest src/so100_labs/test/test_task1_joint_commands.py
```

## Think about it

- Change `MAX_STEP` to `1.0` and run in Gazebo (`backend:=gazebo controller:=position`). What
  happens, and why would this be dangerous on a real robot?
- `/joint_states` lists the joints in a different order than `ARM_JOINTS`. Check with
  `ros2 topic echo /joint_states --once`.

**Real robot:** Task 1 is simulation-only. Sending raw positions straight to real servos is how
robots get broken. Task 2 does the same motion safely.
