#!/usr/bin/env bash
# Run one of your SO-100 tasks on the REAL robot (see docs/REAL_ROBOT.md), slowed down.
#   bash scripts/run_on_robot.sh task2 [speed]      speed: 0.1 .. 1.0 (default 0.3)
# Refuses to run unless that task's unit tests pass ("simulation first").
set -e
TASK="$1"; SPEED="${2:-0.3}"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
declare -A EXE=(
  [task2]=task2_trajectories [task3]=task3_forward_kinematics
  [task4]=task4_inverse_kinematics [task5]=task5_cartesian
  [task6]=task6_pid_control
)
if [ "$TASK" = task1 ]; then
  echo "Task 1 uses forward_position_controller, which is disabled on the real robot"
  echo "(it would jump straight to each command). Run tasks 2-6 on the robot."; exit 1
fi
[ -n "${EXE[$TASK]}" ] || { echo "usage: $0 task2|task3|task4|task5|task6 [speed]"; exit 1; }
awk -v s="$SPEED" 'BEGIN{exit !(s>=0.1 && s<=1.0)}' || { echo "speed must be in [0.1, 1.0]"; exit 1; }

source "$WS/install/setup.bash"
echo ">>> 1/3 Checking your code passes the unit tests for $TASK ..."
TESTS=$(ls "$WS"/src/so100_labs/test/test_${TASK}_*.py)
# Later tasks build on earlier ones, so their tests must pass too.
case $TASK in task4) TESTS="$TESTS $(ls "$WS"/src/so100_labs/test/test_task3_*.py)";;
              task5) TESTS="$TESTS $(ls "$WS"/src/so100_labs/test/test_task[34]_*.py)";; esac
if ! (cd /tmp && python3 -m pytest -q -p no:cacheprovider $TESTS); then
  echo; echo "Unit tests fail. Fix your code in simulation first."; exit 1
fi

echo ">>> 2/3 Checking the real robot is running ..."
if ! ros2 control list_hardware_components 2>/dev/null | grep -q feetech; then
  echo "The real robot is not running. In another terminal start it with:"
  echo "    ros2 launch so100_bringup arm.launch.py backend:=real robot:=<your robot id>"
  exit 1
fi

echo ">>> 3/3 Running ${EXE[$TASK]} at speed ${SPEED}. Keep a hand near the power switch."
echo "    Emergency (torque off, arm drops!): ros2 run so100_bringup estop"
read -r -p "    Workspace clear? Press Enter to start, Ctrl+C to abort. "
ros2 run so100_labs "${EXE[$TASK]}" --ros-args -p speed_scale:="$SPEED"
