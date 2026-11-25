# Bonus: MoveIt and orientation constraints

MoveIt plans, solves IK and times trajectories: everything you built by hand in Tasks 2–5.

```bash
ros2 launch so100_bringup arm.launch.py rviz:=false      # terminal 1
ros2 launch so100_bringup moveit.launch.py               # terminal 2: MoveIt + RViz
ros2 run so100_labs bonus_moveit_goals                   # terminal 3
```

In RViz, drag the marker at the gripper and click **Plan & Execute**. `bonus_moveit_goals.py` sends
three goals for the same point:

| Goal | Constraints                         | Result                         |
| ---- | ----------------------------------- | ------------------------------ |
| 1    | position only                       | ✅ reached, gripper tilted     |
| 2    | position + pointing down            | ✅ reached, gripper vertical   |
| 3    | position + pointing sideways        | ❌ fails: impossible, 5 joints |

With 5 joints the gripper can only point within the vertical plane chosen by `shoulder_pan`. That's
why Task 4 solves for **xyz + pitch** only.

```python
oc = OrientationConstraint()
oc.header.frame_id = 'base_link'
oc.link_name = 'tcp_link'
oc.orientation = Quaternion(x=0.7071, y=0.0, z=0.0, w=0.7071)  # pointing down
oc.absolute_x_axis_tolerance = 0.1
oc.absolute_y_axis_tolerance = 0.1
oc.absolute_z_axis_tolerance = math.pi                         # free spin about base z
oc.parameterization = OrientationConstraint.ROTATION_VECTOR
oc.weight = 1.0
```

> ⚠️ The tolerances are about the axes of **`base_link`**, not the gripper's own axes.

## Try

- Tighten goal 2's tolerances to 0.02 rad. What happens to planning time?
- Redo Task 5's pick-and-place with MoveIt goals.

## Is the gripper really there?

`bonus_moveit_goals` also prints how far the measured TCP is from the target. Run it again with
`arm.launch.py rviz:=false sag:=true` (a mock arm that sags like the real one): MoveIt still says
`SUCCESS`, but the gripper is 1–3 cm off. MoveIt assumes _commanded = actual_. Task 6 fixes that:
[06_pid_control.md](06_pid_control.md).
