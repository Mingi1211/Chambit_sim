#!/usr/bin/env bash
internship_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash || return 1
if [[ -f "$internship_root/install/setup.bash" ]]; then
  source "$internship_root/install/setup.bash" || return 1
fi
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=71
export GZ_PARTITION=internship_dual
export ROS_LOG_DIR=/tmp/internship_ros_logs
unset internship_root
