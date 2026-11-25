# SO-100 arm labs

Six tasks, from joint commands to pick-and-place and closed-loop control. Write your code in
`src/so100_labs/so100_labs/taskN_*.py`, wherever you see `TODO`.

| #   | Task                               | You learn                                     | Guide                          |
| --- | ---------------------------------- | --------------------------------------------- | ------------------------------ |
| 1   | Joint-space commands               | topics, joint order, limits                   | [01](01_joint_commands.md)     |
| 2   | Joint trajectories                 | actions, quintic polynomials                  | [02](02_trajectories.md)       |
| 3   | Forward kinematics                 | homogeneous transforms, URDF, TF              | [03](03_forward_kinematics.md) |
| 4   | Inverse kinematics                 | Jacobian, damped least squares, reachability  | [04](04_inverse_kinematics.md) |
| 5   | Cartesian paths and pick-and-place | straight-line motion, gripper, task sequences | [05](05_cartesian.md)          |
| ★   | MoveIt                             | planning, orientation constraints             | [MoveIt](moveit.md)            |
| 6   | PID joint control                  | feedback, steady-state error, PID, tuning     | [06](06_pid_control.md)        |

## Workflow (every task)

```bash
# Terminal 1: simulated arm (RViz in the Desktop tab)
ros2 launch so100_bringup arm.launch.py

# Terminal 2
cb                                                    # build
pytest src/so100_labs/test/test_task2_trajectories.py # check your functions
ros2 run so100_labs task2_trajectories                # watch it move in RViz
```

When you push, GitHub runs the tests and shows "Task N ✓/✗" next to your commit (Actions tab).

**Simulators:** `arm.launch.py` (default) is a perfect, instant robot. `arm.launch.py backend:=gazebo`
adds physics, a table and a cube (Task 5). `arm.launch.py sag:=true` sags under gravity like the
real arm (Task 6). Real robot: [REAL_ROBOT.md](../REAL_ROBOT.md).

## Coordinate frame

Positions are in **`base_link`**, in metres, **z up**. At `shoulder_pan = 0` the arm points
towards **−y**. You control **`tcp_link`**, the point between the fingertips.

## Useful commands

```bash
ros2 topic echo /joint_states --once             # current joint angles
ros2 control list_controllers                    # active controllers
ros2 run tf2_ros tf2_echo base_link tcp_link     # where is the gripper?
```
