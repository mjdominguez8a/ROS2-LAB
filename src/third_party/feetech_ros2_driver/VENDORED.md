# Vendored: feetech_ros2_driver

- Source: https://github.com/JafarAbdi/feetech_ros2_driver
- Tag: 0.2.2, commit e424839f0fb23cc0b0ade8d6e5ea166e30811356 (same version as the
  `ros-jazzy-feetech-ros2-driver` apt package in the image, which this package overrides)
- License: BSD (see LICENSE)
- Left out: `.git`, `.github`, pixi and linter configuration files

Built from source because of these course patches, each marked `[so100 lab patch]`:

1. **Velocity sign** (`common.hpp`, `src/feetech_ros2_driver.cpp`, `communication_protocol.cpp`):
   the servos report speed as a magnitude with bit 15 as the sign, but the driver read it as an
   unsigned number. A joint drifting at -0.08 rad/s showed up in `/joint_states` as +50.3 rad/s
   and tripped the safety monitor. Added `decode_signed_value` and used it for both speed reads.
2. **Read timeout 10 ms -> 50 ms** (`serial_port.hpp`): through the MacBook serial bridge (Docker
   Desktop) about one reply in 1000 takes 10-13 ms, and a single late byte makes ros2_control
   deactivate the robot. A disconnected robot is still detected within 50 ms per byte.
3. **Survive an occasional missed read** (`communication_protocol.hpp`, `src/feetech_ros2_driver.cpp`):
   even with 50 ms, the bridge stalls for ~80 ms every few minutes. `read()` now keeps the last
   state for up to 2 failed reads in a row (logged as a warning) and deactivates the robot only on
   the 3rd. To make that safe, `sync_read` first discards stale input (late bytes of a previous
   reply) and rejects a reply whose servo ID is not the one expected.
4. **Servo settings as parameters** (`src/feetech_ros2_driver.cpp`, header): the hard-coded
   `acceleration` 50 and `max_speed` 2400 sent with every position command are now hardware
   parameters, and on startup the driver can also write `return_delay_time`, `max_torque_limit`,
   `protection_current` and `overload_torque` per joint (RAM only, nothing is stored in the servo).
   `so100_bringup` sets LeRobot's values; acceleration 254 instead of 50 was what made the arm lag
   behind its trajectories and move in surges.
5. **Release build by default** (`CMakeLists.txt`): colcon builds without optimisation otherwise.

Students should not edit this package. To update: copy a newer release, re-apply the patches (or
drop them if fixed upstream), and update the commit above.
