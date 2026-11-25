# Task 3: Forward kinematics

**File:** `src/so100_labs/so100_labs/task3_forward_kinematics.py`

Forward kinematics (FK) answers: _given the joint angles, where is the gripper?_ The URDF describes
the arm as a chain. Each joint has a fixed offset from its parent (`xyz`, `rpy`) and a rotation
`axis`. `robot_model.CHAIN` contains these numbers copied from the URDF.

```
T_base_tcp = Π_i [ Trans(xyz_i) · Rot(rpy_i) · Rot(axis_i, q_i) ] · Trans(TCP_XYZ)
```

## What to implement

`rotation_from_rpy`, `rotation_about_axis` (Rodrigues), `homogeneous`, `forward_kinematics`.

## Run it

```bash
ros2 launch so100_bringup arm.launch.py           # terminal 1
ros2 run so100_labs task3_forward_kinematics      # terminal 2: prints your FK vs TF
pytest src/so100_labs/test/test_task3_forward_kinematics.py
```

`robot_state_publisher` computes exactly the same thing from the URDF and publishes it as TF, so
your result must match `ros2 run tf2_ros tf2_echo base_link tcp_link` to well under 1 mm.

## Think about it

- Run Task 3 against Gazebo. The FK error is still about 0, but the TCP is a few mm lower than where
  you asked it to be in Task 4. Why? (Hint: what does FK assume about the links?)
