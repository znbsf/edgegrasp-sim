#!/usr/bin/env bash
# Enumerate or terminate only Linux processes whose /proc environment contains
# one exact ROS_DOMAIN_ID. This avoids broad pkill patterns during disposable
# simulation evidence runs.

set -eo pipefail

if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
  echo "usage: bash scripts/cleanup_ros_domain.sh ROS_DOMAIN_ID [--terminate]" >&2
  exit 2
fi

cleanup_domain_id=$1
cleanup_mode=${2:-}
case "$cleanup_domain_id" in
  ''|*[!0-9]*) echo "ROS_DOMAIN_ID must be an integer in [0,232]" >&2; exit 2 ;;
esac
if [ "$cleanup_domain_id" -gt 232 ]; then
  echo "ROS_DOMAIN_ID must be an integer in [0,232]" >&2
  exit 2
fi
if [ -n "$cleanup_mode" ] && [ "$cleanup_mode" != "--terminate" ]; then
  echo "second argument must be --terminate" >&2
  exit 2
fi

domain_pids=()
echo "DOMAIN_${cleanup_domain_id}_PROCESSES_BEFORE:"
for cleanup_proc_dir in /proc/[0-9]*; do
  [ -r "$cleanup_proc_dir/environ" ] || continue
  if { tr '\0' '\n' < "$cleanup_proc_dir/environ"; } 2>/dev/null \
    | grep -qx "ROS_DOMAIN_ID=$cleanup_domain_id"; then
    cleanup_pid=${cleanup_proc_dir#/proc/}
    if [ "$cleanup_pid" = "$$" ] || [ "$cleanup_pid" = "$PPID" ]; then
      continue
    fi
    cleanup_cmd=$(tr '\0' ' ' < "$cleanup_proc_dir/cmdline" 2>/dev/null || true)
    printf '%s %s\n' "$cleanup_pid" "$cleanup_cmd"
    domain_pids+=("$cleanup_pid")
  fi
done

if [ "$cleanup_mode" != "--terminate" ]; then
  echo "READ_ONLY_COUNT=${#domain_pids[@]}"
  exit 0
fi

for cleanup_pid in "${domain_pids[@]}"; do
  kill -TERM "$cleanup_pid" 2>/dev/null || true
done
for _ in $(seq 1 10); do
  cleanup_alive=0
  for cleanup_pid in "${domain_pids[@]}"; do
    kill -0 "$cleanup_pid" 2>/dev/null && cleanup_alive=$((cleanup_alive + 1))
  done
  [ "$cleanup_alive" -eq 0 ] && break
  sleep 1
done
for cleanup_pid in "${domain_pids[@]}"; do
  if kill -0 "$cleanup_pid" 2>/dev/null; then
    printf 'force_kill_pid=%s\n' "$cleanup_pid"
    kill -KILL "$cleanup_pid" 2>/dev/null || true
  fi
done
sleep 2

cleanup_remaining=0
echo "DOMAIN_${cleanup_domain_id}_PROCESSES_AFTER:"
for cleanup_proc_dir in /proc/[0-9]*; do
  [ -r "$cleanup_proc_dir/environ" ] || continue
  if { tr '\0' '\n' < "$cleanup_proc_dir/environ"; } 2>/dev/null \
    | grep -qx "ROS_DOMAIN_ID=$cleanup_domain_id"; then
    cleanup_pid=${cleanup_proc_dir#/proc/}
    if [ "$cleanup_pid" = "$$" ] || [ "$cleanup_pid" = "$PPID" ]; then
      continue
    fi
    cleanup_cmd=$(tr '\0' ' ' < "$cleanup_proc_dir/cmdline" 2>/dev/null || true)
    printf '%s %s\n' "$cleanup_pid" "$cleanup_cmd"
    cleanup_remaining=$((cleanup_remaining + 1))
  fi
done
echo "REMAINING_COUNT=$cleanup_remaining"
if [ "$cleanup_remaining" -ne 0 ]; then
  exit 4
fi

source /opt/ros/jazzy/setup.bash
echo "CLOCK_AFTER_EXACT_CLEANUP:"
ROS_DOMAIN_ID=$cleanup_domain_id ROS_LOCALHOST_ONLY=1 \
  timeout 8s ros2 topic info /clock 2>&1 || true
ROS_DOMAIN_ID=$cleanup_domain_id ROS_LOCALHOST_ONLY=1 \
  ros2 daemon stop >/dev/null 2>&1 || true
