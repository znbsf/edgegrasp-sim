#!/usr/bin/env bash
set -euo pipefail

usage() {
  printf 'usage: %s [BAG_DIRECTORY] [ros_sim|system]\n' "$0" >&2
  printf '  ros_sim (default; aliases: gazebo): wait for /clock and use --use-sim-time\n' >&2
  printf '  system (alias: mock): use system receive time without --use-sim-time\n' >&2
}

if (( $# > 2 )); then
  usage
  exit 2
fi

output_dir="${1:-$HOME/.ros/edgegrasp_bags/edgegrasp_$(date +%Y%m%d_%H%M%S)}"
profile="${2:-ros_sim}"

case "$profile" in
  ros_sim|gazebo)
    use_sim_time=true
    ;;
  system|mock)
    use_sim_time=false
    ;;
  *)
    printf 'error: unsupported recording profile: %s\n' "$profile" >&2
    usage
    exit 2
    ;;
esac

if ! command -v ros2 >/dev/null 2>&1; then
  printf 'error: ros2 is not on PATH; source the intended ROS workspace first\n' >&2
  exit 127
fi

mkdir -p "$(dirname "$output_dir")"

record_topics=(
  /clock
  /joint_states
  /gripper_controller/controller_state
  /edgegrasp/target_3d
  /edgegrasp/tracked_target
  /edgegrasp/motion_allowed
  /edgegrasp/safety_status
  /edgegrasp/arm_joint_trajectory_request
  /edgegrasp/gripper_joint_trajectory_request
  /edgegrasp/trajectory_gate_status
  /edgegrasp/interface_ready
  /edgegrasp/interface_status
  /edgegrasp/target_cube_pose
  /edgegrasp/target_cube_contacts
  /edgegrasp/table_contacts
  /edgegrasp/planning_scene_status
  /edgegrasp/grasp_timeline
  /edgegrasp/grasp_sequence_status
  /edgegrasp/grasp_sequence_terminal
  /edgegrasp/grasp_physics_status
  /camera_head/color/image_raw
  /camera_head/depth/image_rect_raw
  /camera_head/depth/camera_info
)

if [[ "$use_sim_time" == true ]]; then
  printf 'Waiting for one /clock sample before ros_sim recording...\n' >&2
  ros2 topic echo /clock --once >/dev/null
  ros2 bag record -s mcap --use-sim-time --output "$output_dir" \
    --topics "${record_topics[@]}"
else
  ros2 bag record -s mcap --output "$output_dir" \
    --topics "${record_topics[@]}"
fi
