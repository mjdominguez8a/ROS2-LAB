# ROS 2 Lab: SO-100 robot arm

Program a simulated **SO-100 arm** in ROS 2 Jazzy, all in your browser. At the end you run the same
code on the real arm.

**You need:** a GitHub account and a modern browser. Tip: the free
[GitHub Student Pack](https://education.github.com/pack) gives you more Codespaces hours.

## 1. Setup

1. **Use this template → Create a new repository**.
2. In your repository: **Code → Codespaces → Create codespace on main**. The first start takes a few
   minutes.
3. When the terminal says `Setup complete`, open a **new terminal** and run:

   ```bash
   bash scripts/check_setup.sh
   ```

4. Open the desktop (RViz, Gazebo): **PORTS** tab → port **6080** → globe icon → **Connect**,
   password `vscode`.

## 2. Warm-up (Task 0)

Complete the TODO in `src/lab_starter/lab_starter/turtle_circle.py`, then:

```bash
cb                                     # build (colcon build + source)
ros2 run turtlesim turtlesim_node      # terminal 1: turtle window in the Desktop tab
ros2 run lab_starter turtle_circle     # terminal 2
colcon test --packages-select lab_starter && colcon test-result --verbose
```

## 3. Arm labs (Tasks 1–6)

Follow **[docs/tasks/README.md](docs/tasks/README.md)**. When your code works in simulation, run it
on the real arm: **[docs/REAL_ROBOT.md](docs/REAL_ROBOT.md)**.

Your code lives in `src/so100_labs/so100_labs/`. You don't need to change anything else.

## Save your work

- **Commit and push** before you leave (Source Control icon → message → Commit → Sync).
- **Stop your codespace** when done (github.com/codespaces → ⋯ → Stop).

## Troubleshooting

| Problem                                          | Fix                                                                       |
| ------------------------------------------------ | ------------------------------------------------------------------------- |
| `ros2: command not found` / package not found    | Open a **new** terminal, or run `cb`                                      |
| Port 6080 shows nothing                          | Wait 30 s and refresh                                                     |
| `arm.launch.py`: controllers never become active | Another copy is still running: `Ctrl+C` in all terminals, then relaunch   |
| "No /joint_states received"                      | Start `ros2 launch so100_bringup arm.launch.py` in another terminal first |
| Something is badly broken                        | Push your code, delete the codespace, create a new one                    |
