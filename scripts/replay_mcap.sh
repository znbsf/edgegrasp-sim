#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  printf 'usage: %s BAG_DIRECTORY\n' "$0" >&2
  exit 2
fi

if ! command -v ros2 >/dev/null 2>&1; then
  printf 'error: ros2 is not on PATH; source the intended ROS workspace first\n' >&2
  exit 127
fi

clock_info="$(ros2 topic info /clock 2>/dev/null || true)"
publisher_count="$(
  printf '%s\n' "$clock_info" |
    awk -F: '/^[[:space:]]*Publisher count:/ {
      gsub(/[^0-9]/, "", $2)
      print $2
      exit
    }'
)"
if [[ -z "$publisher_count" ]]; then
  if [[ -z "$clock_info" ]] || printf '%s\n' "$clock_info" | grep -Eiq 'not found|unknown topic'; then
    publisher_count=0
  else
    printf 'error: could not determine the existing /clock publisher count; refusing replay\n' >&2
    exit 4
  fi
fi
if ! [[ "$publisher_count" =~ ^[0-9]+$ ]]; then
  printf 'error: invalid /clock publisher count: %s\n' "$publisher_count" >&2
  exit 4
fi
if (( publisher_count > 0 )); then
  printf 'error: refusing replay; /clock already has %s publisher(s)\n' "$publisher_count" >&2
  printf 'stop Gazebo/other clock publishers; rosbag --clock must be the sole /clock authority\n' >&2
  exit 3
fi

printf 'Replay contract: this script does not launch edgegrasp_replay; start it separately.\n' >&2
printf 'rosbag --clock is the sole /clock authority; do not run Gazebo concurrently.\n' >&2

RAW_REPLAY_TOPICS=(
  /joint_states
  /edgegrasp/target_3d
  /edgegrasp/tracked_target
  /camera_head/color/image_raw
  /camera_head/depth/image_rect_raw
  /camera_head/depth/camera_info
)

ros2 bag play "$1" --clock --topics "${RAW_REPLAY_TOPICS[@]}"
