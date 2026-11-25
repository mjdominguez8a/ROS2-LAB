#!/usr/bin/env bash
# Runs once when a codespace is created: installs dependencies and builds the workspace.
set -e

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
echo ">>> Setting up ROS 2 workspace in $WS"

source /opt/ros/jazzy/setup.bash

sudo apt-get update -qq || true
rosdep update --rosdistro jazzy || echo "rosdep update failed (network?), continuing"
rosdep install --from-paths "$WS/src" --ignore-src -y --rosdistro jazzy \
  --skip-keys "mujoco_ros2_control" || \
  echo "Some rosdep dependencies could not be installed, continuing"

cd "$WS"
colcon build --symlink-install

MARKER="# >>> ros2-student-lab >>>"
if ! grep -q "$MARKER" ~/.bashrc; then
  cat >> ~/.bashrc <<BASHRC

$MARKER
source /opt/ros/jazzy/setup.bash
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
alias cb='cd "$WS" && colcon build --symlink-install && source install/setup.bash'
# <<< ros2-student-lab <<<
BASHRC
fi

echo ">>> Setup complete. Open a NEW terminal, then run: bash scripts/check_setup.sh"
