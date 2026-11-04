#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source /opt/ros/jazzy/setup.bash
# The host runs command/state clients; Cyclo runs only in robot containers.
MAKEFLAGS="${MAKEFLAGS:--j2}" colcon build --symlink-install --executor sequential \
  --base-paths src/ai_worker/ffw_description src/ai_worker/ffw_bringup \
  src/realsense-ros/realsense2_camera_msgs src/realsense-ros/realsense2_description \
  src/robotis_interfaces src/cyclo_control/cyclo_motion_controller_models \
  src/ai_worker/ffw_swerve_drive_controller \
  src/coop_carry_hardware src/coop_carry_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
