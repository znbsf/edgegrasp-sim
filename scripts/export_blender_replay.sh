#!/usr/bin/env bash
# Offline deserialization only: no ROS nodes, publishers, simulation, or controllers.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/edgegrasp/ros2_ws/install/setup.bash
replay_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
export PYTHONPATH="$replay_root/src:${PYTHONPATH:-}"
exec /usr/bin/python3 "$replay_root/scripts/export_blender_replay.py" "$@"
