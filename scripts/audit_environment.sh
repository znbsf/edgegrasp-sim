#!/usr/bin/env bash
set -u

if [[ -r /etc/os-release ]]; then
  sed -n 's/^\(NAME\|VERSION\|VERSION_ID\)=/\1=/p' /etc/os-release
fi
uname -a

for command_name in python3 cmake docker colcon ros2 gz gazebo rviz2; do
  if command -v "$command_name" >/dev/null 2>&1; then
    printf '%s: %s\n' "$command_name" "$(command -v "$command_name")"
  else
    printf '%s: unavailable\n' "$command_name"
  fi
done

df -h .
printf 'DISPLAY=%s\n' "${DISPLAY:-unset}"
printf 'WAYLAND_DISPLAY=%s\n' "${WAYLAND_DISPLAY:-unset}"
