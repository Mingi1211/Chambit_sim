#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /opt/internship_ws/install/setup.bash
case "${1:-}" in
  router) exec /opt/ros/jazzy/lib/rmw_zenoh_cpp/rmw_zenohd ;;
  gazebo)
    rm -f /run/coop/ready
    exec ros2 launch coop_carry_bringup gazebo.launch.py runtime_dir:=/run/coop gui:="${COOP_GUI:-false}" ;;
  robot1|robot2)
    robot="$1"
    for ((attempt=0; attempt<120; attempt++)); do
      if [[ -s /run/coop/ready && -s "/run/coop/$robot/control.urdf" ]]; then
        exec ros2 launch coop_carry_bringup robot.launch.py robot:="$robot" runtime_dir:=/run/coop
      fi
      sleep 1
    done
    echo 'Gazebo descriptions did not become ready' >&2
    exit 1 ;;
  *) exec "$@" ;;
esac
