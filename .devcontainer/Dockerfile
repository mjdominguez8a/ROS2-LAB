# ROS 2 Jazzy + Gazebo Harmonic + SO-100 arm stack, for GitHub Codespaces.
# A prebuilt copy (including the desktop) is published to ghcr.io by .github/workflows/image.yml
# and used as a build cache, so codespaces start in ~1-2 minutes instead of ~10.
# Keep it independent of src/ so the cache stays valid while students edit code.
FROM osrf/ros:jazzy-desktop-full

# Links the published image to this repository on GHCR (lets the workflow write to it).
LABEL org.opencontainers.image.source="https://github.com/NEARLab-MRS/medrob-workshop-ros2"

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
        sudo git nano less python3-pip python3-pytest python3-numpy python3-serial \
        ros-jazzy-turtlesim ros-jazzy-ros-gz ros-jazzy-rqt-graph \
        # ros2_control + controllers + Gazebo bridge
        ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control \
        ros-jazzy-rqt-joint-trajectory-controller ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-xacro ros-jazzy-nav2-common ros-jazzy-launch-testing-ros \
        # MoveIt (bonus task)
        ros-jazzy-moveit \
        # Real SO-100 servos (Feetech STS3215)
        ros-jazzy-feetech-ros2-driver \
    && rm -rf /var/lib/apt/lists/*

# The Ubuntu 24.04 base image ships a non-root "ubuntu" user (uid 1000).
# Passwordless sudo lets students install extra packages; dialout gives serial (robot) access.
RUN echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu \
    && chmod 0440 /etc/sudoers.d/ubuntu \
    && usermod -aG dialout ubuntu
