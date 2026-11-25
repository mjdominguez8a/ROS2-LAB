#!/usr/bin/env bash
# Self-check: confirms ROS 2, the workspace and the desktop are working.
ok()   { echo "  [OK]   $1"; }
fail() { echo "  [FAIL] $1"; FAILED=1; }
FAILED=0

echo "Checking your ROS 2 lab environment..."

[ "$ROS_DISTRO" = "jazzy" ] && ok "ROS 2 Jazzy is sourced" \
  || fail "ROS 2 not sourced. Open a new terminal or run: source /opt/ros/jazzy/setup.bash"

command -v ros2 >/dev/null && ok "ros2 command available" || fail "ros2 command not found"

for pkg in lab_starter so100_bringup so100_labs; do
  ros2 pkg list 2>/dev/null | grep -q "^$pkg\$" && ok "$pkg package is built" \
    || fail "$pkg not found. Run: cb   (or: colcon build && source install/setup.bash)"
done

[ -n "$DISPLAY" ] && ok "DISPLAY is set to $DISPLAY" || fail "DISPLAY is not set"

if command -v curl >/dev/null && curl -s -o /dev/null http://localhost:6080; then
  ok "Desktop (noVNC) is running on port 6080"
else
  fail "Desktop not reachable on port 6080. Check the PORTS tab."
fi

if [ -e /dev/so100 ]; then
  ok "Robot found at /dev/so100"
else
  echo "  [INFO] No robot at /dev/so100 (fine for simulation; see docs/REAL_ROBOT.md)"
fi

echo
if [ "$FAILED" = 0 ]; then
  echo "All good! Open the PORTS tab, click the globe next to port 6080, and log in with password: vscode"
else
  echo "Some checks failed. Ask your instructor or see README.md > Troubleshooting."
fi
