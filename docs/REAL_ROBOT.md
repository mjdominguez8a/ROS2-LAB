# Running your code on the real SO-100

The real arm is connected to the instructor's MacBook. Your code does not change, only the launch
command.

> ⚠️ **Safety rules**
>
> 1. Nothing within 40 cm of the arm, including your face.
> 2. Keep one hand on the **power switch** while the arm moves. It is the emergency stop.
> 3. Your code must pass its tests and work in simulation first (`run_on_robot.sh` checks this).
> 4. Start slow (`speed 0.3`).

## Steps

1. Open your repository on the MacBook in the **default** dev container. RViz is on port 6080.
2. Plug in the robot (USB + power). In a **Mac** terminal, in the repository folder, start the bridge
   and leave it running:

   ```bash
   uv run --with pyserial scripts/mac_serial_bridge.py
   ```

3. In the container:

   ```bash
   bash scripts/connect_robot.sh                               # creates /dev/so100
   ros2 launch so100_bringup arm.launch.py backend:=real robot:=arm3   # terminal 1
   bash scripts/run_on_robot.sh task2                          # terminal 2 (tasks 2–6, speed 0.3)
   bash scripts/run_on_robot.sh task4 0.5                      # faster once you trust it
   ```

4. When done: `Ctrl+C` in terminal 1, **hold the arm** (it goes limp), switch the power off.

Task 1 is simulation only. If the bridge stops, restart it and run `connect_robot.sh` again.

## If something goes wrong

| Situation                         | Action                                                                     |
| --------------------------------- | -------------------------------------------------------------------------- |
| Arm heading for a collision       | **Power switch.**                                                          |
| Stop now, arm safe to drop        | `ros2 run so100_bringup estop` (arm falls, hold it). Resume: `--resume`    |
| "SAFETY STOP" in terminal 1       | A joint left its safe range or moved too fast. Check your targets.         |
| "Trajectory too fast" / "outside" | `Arm` refused your command. Fix your code.                                 |
| `/dev/so100` not found            | Check USB/power, the bridge, and run `connect_robot.sh` again              |
| RViz pose ≠ real pose             | Recalibrate (below)                                                        |

## Calibration (once per robot)

With nothing else using the robot: `ros2 run so100_bringup calibrate --robot-id arm3`. Move the arm
to the zero pose (all joints at 0 in the mock RViz), press Enter, then `cb`. Commit
`src/so100_bringup/config/robots/arm3.yaml`.
