# Task 5: Cartesian paths and pick-and-place

**File:** `src/so100_labs/so100_labs/task5_cartesian.py`

Moving between two IK solutions in joint space makes the gripper follow a **curve**. To draw a
straight line, or to go straight down onto an object, you need many closely spaced Cartesian points
with IK for each.

## What to implement

1. `linear_interpolation(p0, p1, step)`: evenly spaced points on a segment.
2. `cartesian_path(points, pitch, q_seed)`: IK for every point, each seeded with the previous
   solution.
3. `time_parametrize(joint_path, q_start)`: timestamps that respect a velocity limit.
4. `pick_and_place(arm, pick, place)`: open, approach from above, descend, grasp, lift, carry,
   descend, release, retreat.

## Run it

```bash
ros2 launch so100_bringup arm.launch.py backend:=gazebo       # terminal 1: table + red cube
ros2 run so100_labs task5_cartesian                           # terminal 2
pytest src/so100_labs/test/test_task5_cartesian.py
```

In Gazebo the cube really gets picked up, so a sloppy approach knocks it over. Reset the cube:

```bash
gz service -s /world/so100_table/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
  --req 'name: "cube", position: {x: 0.08, y: -0.22, z: 0.7625}' --timeout 2000
```

## Think about it

- Replace the straight-line descent with a single `arm.move_to(...)`. Watch the fingertips in RViz:
  do they still go straight down?
- Why does the pick point have z = 1.3 cm and not 0?
- Your path sends positions only. The controller then joins the points with straight lines, so
  the joint velocities jump at every point: in RViz it looks fine, on the real robot you feel it as
  vibration. How could you also send `velocities` to `arm.follow_trajectory` (as in Task 2)?
