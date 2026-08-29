#!/usr/bin/env bash
# Run a chained MoveIt planning probe without launching any EdgeGrasp motion
# node.  Returned trajectories are validated and discarded; no controller goal
# is sent.  This is planning evidence only.

set -eo pipefail

if [ "$#" -lt 4 ] || [ "$#" -gt 11 ]; then
  echo "usage: bash scripts/run_grasp_candidate_plan_only.sh ROS_DOMAIN_ID ARTIFACT_DIR LABEL GRASP_GEOMETRY_FILENAME [ATTEMPTS] [TARGET_CENTER_X_OFFSET_M] [PAD_CONTACT_MATERIAL_PROFILE] [MOVING_PAD_DISTAL_EXTENSION_M] [CANDIDATE_FILENAME] [SCENE_CONFIG_FILENAME] [WORLD_FILENAME]" >&2
  exit 2
fi

probe_domain_id=$1
probe_artifact_dir=$2
probe_label=$3
probe_geometry_filename=$4
probe_attempts=${5:-10}
probe_target_center_x_offset_m=${6:-0.0}
probe_pad_contact_material_profile=${7:-implicit_default}
probe_moving_pad_distal_extension_m=${8:-0.0}
probe_candidate_filename=${9:-so101_side_grasp_candidate.json}
probe_scene_config_filename=${10:-scene.json}
probe_world_filename=${11:-table_cube.sdf}

case "$probe_domain_id" in
  ''|*[!0-9]*) echo "ROS_DOMAIN_ID must be a non-negative integer" >&2; exit 2 ;;
esac
case "$probe_artifact_dir" in
  /home/edgegrasp/ros2_ws/test_results/*) ;;
  *) echo "ARTIFACT_DIR must stay under /home/edgegrasp/ros2_ws/test_results" >&2; exit 2 ;;
esac
case "$probe_label" in
  ''|*[!a-zA-Z0-9._-]*) echo "LABEL contains unsupported characters" >&2; exit 2 ;;
esac
case "$probe_geometry_filename" in
  ''|*/*|*\\*) echo "GRASP_GEOMETRY_FILENAME must be one JSON basename" >&2; exit 2 ;;
  *.json) ;;
  *) echo "GRASP_GEOMETRY_FILENAME must be one JSON basename" >&2; exit 2 ;;
esac
case "$probe_candidate_filename" in
  ''|*/*|*\\*) echo "CANDIDATE_FILENAME must be one JSON basename" >&2; exit 2 ;;
  *.json) ;;
  *) echo "CANDIDATE_FILENAME must be one JSON basename" >&2; exit 2 ;;
esac
case "$probe_scene_config_filename" in
  ''|*/*|*\\*) echo "SCENE_CONFIG_FILENAME must be one JSON basename" >&2; exit 2 ;;
  *.json) ;;
  *) echo "SCENE_CONFIG_FILENAME must be one JSON basename" >&2; exit 2 ;;
esac
case "$probe_world_filename" in
  ''|*/*|*\\*) echo "WORLD_FILENAME must be one SDF basename" >&2; exit 2 ;;
  *.sdf) ;;
  *) echo "WORLD_FILENAME must be one SDF basename" >&2; exit 2 ;;
esac
case "$probe_attempts" in
  ''|*[!0-9]*) echo "ATTEMPTS must be an integer in [1, 100]" >&2; exit 2 ;;
esac
if [ "$probe_attempts" -lt 1 ] || [ "$probe_attempts" -gt 100 ]; then
  echo "ATTEMPTS must be an integer in [1, 100]" >&2
  exit 2
fi
python3 -c 'import math,sys; value=float(sys.argv[1]); assert math.isfinite(value)' \
  "$probe_target_center_x_offset_m" 2>/dev/null || {
  echo "TARGET_CENTER_X_OFFSET_M must be finite" >&2
  exit 2
}
case "$probe_pad_contact_material_profile" in
  ''|*[!a-zA-Z0-9._-]*) echo "PAD_CONTACT_MATERIAL_PROFILE contains unsupported characters" >&2; exit 2 ;;
esac
python3 -c 'import math,sys; value=float(sys.argv[1]); assert math.isfinite(value) and 0.0 <= value <= 0.015' \
  "$probe_moving_pad_distal_extension_m" 2>/dev/null || {
  echo "MOVING_PAD_DISTAL_EXTENSION_M must be finite and in [0, 0.015]" >&2
  exit 2
}

export ROS_DOMAIN_ID=$probe_domain_id
export ROS_LOCALHOST_ONLY=1
source /opt/ros/jazzy/setup.bash
source /home/edgegrasp/ros2_ws/install/setup.bash
cd /home/edgegrasp/ros2_ws/src/edgegrasp-sim

installed_profile="/home/edgegrasp/ros2_ws/install/edgegrasp_ros/share/edgegrasp_ros/config/${probe_geometry_filename}"
installed_candidate="/home/edgegrasp/ros2_ws/install/edgegrasp_ros/share/edgegrasp_ros/config/${probe_candidate_filename}"
installed_scene="/home/edgegrasp/ros2_ws/install/edgegrasp_ros/share/edgegrasp_ros/config/${probe_scene_config_filename}"
installed_world="/home/edgegrasp/ros2_ws/install/edgegrasp_ros/share/edgegrasp_ros/worlds/${probe_world_filename}"
if [ ! -f "$installed_profile" ]; then
  echo "installed grasp geometry profile is missing: $installed_profile" >&2
  exit 2
fi
if [ ! -f "$installed_candidate" ]; then
  echo "installed candidate config is missing: $installed_candidate" >&2
  exit 2
fi
if [ ! -f "$installed_scene" ] || [ ! -f "$installed_world" ]; then
  echo "installed scene config/world is missing" >&2
  exit 2
fi
probe_scene_values=$(python3 -c '
import json, sys
from edgegrasp.scene import load_scene_contract, render_gazebo_sdf
scene = load_scene_contract(__import__("pathlib").Path(sys.argv[1]))
cube = next(item for item in scene.objects if item.object_id == "target_cube")
actual = __import__("pathlib").Path(sys.argv[2]).read_text(encoding="utf-8")
if actual != render_gazebo_sdf(scene):
    raise ValueError("installed scene world drift")
if scene.name != "edgegrasp_table_cube":
    raise ValueError("unsupported Gazebo world name")
print("\t".join((*(format(value, ".17g") for value in cube.pose_world.position_m), scene.digest)))
' "$installed_scene" "$installed_world")
IFS=$'\t' read -r probe_target_x_m probe_target_y_m probe_target_z_m \
  probe_scene_digest <<< "$probe_scene_values"
if [ -e "$probe_artifact_dir" ]; then
  echo "ARTIFACT_DIR already exists; use a fresh path" >&2
  exit 2
fi

mkdir -p "$probe_artifact_dir"
{
  printf 'started_at=%s\n' "$(date --iso-8601=ns)"
  printf 'ROS_DOMAIN_ID=%s\n' "$ROS_DOMAIN_ID"
  printf 'LABEL=%s\n' "$probe_label"
  printf 'GRASP_GEOMETRY_FILENAME=%s\n' "$probe_geometry_filename"
  printf 'ATTEMPTS=%s\n' "$probe_attempts"
  printf 'TARGET_CENTER_X_OFFSET_M=%s\n' "$probe_target_center_x_offset_m"
  printf 'PAD_CONTACT_MATERIAL_PROFILE=%s\n' "$probe_pad_contact_material_profile"
  printf 'MOVING_PAD_DISTAL_EXTENSION_M=%s\n' "$probe_moving_pad_distal_extension_m"
  printf 'CANDIDATE_FILENAME=%s\n' "$probe_candidate_filename"
  printf 'SCENE_CONFIG_FILENAME=%s\n' "$probe_scene_config_filename"
  printf 'WORLD_FILENAME=%s\n' "$probe_world_filename"
  printf 'SCENE_CONTRACT_SHA256=%s\n' "$probe_scene_digest"
  printf 'TARGET_POSITION_M=[%s,%s,%s]\n' "$probe_target_x_m" "$probe_target_y_m" "$probe_target_z_m"
  printf '%s\n' 'CAPABILITY=GetMotionPlan service only; zero trajectory publication/action execution'
} > "$probe_artifact_dir/start.txt"

probe_pids=()

cleanup_probe() {
  probe_exit_code=$?
  {
    printf 'cleanup_start=%s\n' "$(date --iso-8601=ns)"
    printf 'original_exit=%s\n' "$probe_exit_code"
    for probe_pid in "${probe_pids[@]}"; do
      if kill -0 "$probe_pid" 2>/dev/null; then
        probe_pgid=$(ps -o pgid= -p "$probe_pid" 2>/dev/null | tr -d ' ' || true)
        printf 'term_pid=%s pgid=%s\n' "$probe_pid" "$probe_pgid"
        if [ -n "$probe_pgid" ] && [ "$probe_pgid" = "$probe_pid" ]; then
          kill -TERM -- "-$probe_pid" 2>/dev/null || true
        else
          kill -TERM "$probe_pid" 2>/dev/null || true
        fi
      fi
    done
    for _ in $(seq 1 15); do
      probe_alive=0
      for probe_pid in "${probe_pids[@]}"; do
        kill -0 "$probe_pid" 2>/dev/null && probe_alive=$((probe_alive + 1))
      done
      [ "$probe_alive" -eq 0 ] && break
      sleep 1
    done
    for probe_pid in "${probe_pids[@]}"; do
      if kill -0 "$probe_pid" 2>/dev/null; then
        probe_pgid=$(ps -o pgid= -p "$probe_pid" 2>/dev/null | tr -d ' ' || true)
        printf 'kill_pid=%s pgid=%s\n' "$probe_pid" "$probe_pgid"
        if [ -n "$probe_pgid" ] && [ "$probe_pgid" = "$probe_pid" ]; then
          kill -KILL -- "-$probe_pid" 2>/dev/null || true
        else
          kill -KILL "$probe_pid" 2>/dev/null || true
        fi
      fi
    done
    bash scripts/cleanup_ros_domain.sh "$ROS_DOMAIN_ID" --terminate || true
    sleep 2
    printf '%s\n' 'matching_processes_after_cleanup:'
    ps -eo pid=,ppid=,args= \
      | grep -E '(^|/)(gz sim|gzserver|move_group|controller_manager|planning_scene_loader)( |$)' \
      | grep -v grep || true
    printf '%s\n' 'clock_after_cleanup:'
    timeout 8s ros2 topic info /clock 2>&1 || true
    printf 'cleanup_end=%s\n' "$(date --iso-8601=ns)"
  } >> "$probe_artifact_dir/cleanup.log" 2>&1
  return "$probe_exit_code"
}

trap cleanup_probe EXIT INT TERM

launch_probe_group() {
  probe_log=$1
  shift
  setsid "$@" > "$probe_artifact_dir/$probe_log" 2>&1 &
  probe_pid=$!
  probe_pids+=("$probe_pid")
  printf '%s %s\n' "$probe_pid" "$*" >> "$probe_artifact_dir/launch_parents.log"
}

ros2 daemon stop >/dev/null 2>&1 || true
preexisting_processes=$(ps -eo pid=,ppid=,args= \
  | grep -E '(^|/)(gz sim|gzserver|move_group|controller_manager|trajectory_gate|grasp_sequence|moveit_adapter|planning_scene_loader)( |$)' \
  | grep -v grep || true)
printf '%s\n' "$preexisting_processes" > "$probe_artifact_dir/preexisting_processes.log"
if [ -n "$preexisting_processes" ]; then
  echo "pre-existing ROS/Gazebo process matched; refusing to start" >&2
  exit 3
fi
if timeout 8s ros2 topic info /clock > "$probe_artifact_dir/clock_preflight.log" 2>&1; then
  echo "pre-existing /clock graph detected; refusing to start" >&2
  exit 3
fi

echo "STAGE launch_gazebo_plan_only"
launch_probe_group gazebo.log ros2 launch edgegrasp_ros \
  edgegrasp_proxy_gazebo.launch.py use_camera:=false \
  world_filename:="$probe_world_filename" \
  scene_config_filename:="$probe_scene_config_filename" \
  pad_contact_material_profile:="$probe_pad_contact_material_profile" \
  moving_pad_distal_extension_m:="$probe_moving_pad_distal_extension_m" \
  launch_edgegrasp_nodes:=false launch_physics_observer:=false \
  target_id:=target_cube \
  target_x_m:="$probe_target_x_m" \
  target_y_m:="$probe_target_y_m" target_z_m:="$probe_target_z_m"

probe_ready=0
for _ in $(seq 1 150); do
  probe_clock=$(timeout 5s ros2 topic info /clock 2>/dev/null || true)
  probe_controllers=$(timeout 5s ros2 control list_controllers \
    -c /controller_manager 2>/dev/null || true)
  if printf '%s' "$probe_clock" | grep -q 'Publisher count: 1' \
    && [ "$(printf '%s\n' "$probe_controllers" | grep -c ' active')" -ge 3 ]; then
    probe_ready=1
    break
  fi
  sleep 1
done
printf '%s\n' "$probe_clock" > "$probe_artifact_dir/clock_before.log"
printf '%s\n' "$probe_controllers" > "$probe_artifact_dir/controllers_before.log"
if [ "$probe_ready" -ne 1 ]; then
  echo "Gazebo/controller readiness timeout" >&2
  exit 20
fi

gazebo_server_log=$(find /home/edgegrasp/.gz/sim/log -mindepth 1 -maxdepth 1 \
  -type d -printf '%T@ %p\n' | sort -nr | sed -n '1s/^[^ ]* //p')/server_console.log
printf '%s\n' "$gazebo_server_log" > "$probe_artifact_dir/gazebo_server_log_path.txt"

echo "STAGE launch_moveit_and_scene_only"
launch_probe_group move_group.log ros2 launch edgegrasp_ros \
  edgegrasp_proxy_move_group.launch.py robot_name:=so101 use_camera:=false \
  use_gazebo:=true use_sim_time:=true use_rviz:=false \
  moving_pad_distal_extension_m:="$probe_moving_pad_distal_extension_m"
launch_probe_group planning_scene.log ros2 launch edgegrasp_ros \
  planning_scene.launch.py use_sim_time:=true clock_domain:=ros_sim \
  clock_epoch:=0 target_frame:=base_link include_optional_cube:=true \
  scene_config:="$installed_scene"

probe_ready=0
for _ in $(seq 1 150); do
  probe_services=$(timeout 5s ros2 service list -t 2>/dev/null || true)
  if printf '%s\n' "$probe_services" \
      | grep -qx '/plan_kinematic_path \[moveit_msgs/srv/GetMotionPlan\]' \
    && printf '%s\n' "$probe_services" \
      | grep -qx '/edgegrasp/set_target_pad_contacts \[std_srvs/srv/SetBool\]'; then
    probe_ready=1
    break
  fi
  sleep 1
done
printf '%s\n' "$probe_services" > "$probe_artifact_dir/services_before.log"
if [ "$probe_ready" -ne 1 ]; then
  echo "MoveIt/planning-scene service readiness timeout" >&2
  exit 21
fi

probe_scene_base_ready=0
for _ in $(seq 1 60); do
  probe_scene_status=$(timeout 8s ros2 topic echo --once --full-length \
    /edgegrasp/planning_scene_status 2>/dev/null || true)
  if printf '%s' "$probe_scene_status" | grep -q '"ready": true' \
    && printf '%s' "$probe_scene_status" | grep -q '"reason": "confirmed"'; then
    probe_scene_base_ready=1
    break
  fi
  sleep 1
done
printf '%s\n' "$probe_scene_status" > "$probe_artifact_dir/planning_scene_status_before_acm.log"
if [ "$probe_scene_base_ready" -ne 1 ]; then
  echo "planning-scene base confirmation timeout" >&2
  exit 22
fi

if ! printf '%s' "$probe_scene_status" \
    | grep -q '"allow_target_pad_contacts": false'; then
  echo "plan-only graph must begin with target-pad contacts disabled" >&2
  exit 22
fi
printf '%s\n' "$probe_scene_status" > "$probe_artifact_dir/planning_scene_status.log"

timeout 10s ros2 node list > "$probe_artifact_dir/nodes_before.log" 2>&1
timeout 10s ros2 action list -t > "$probe_artifact_dir/actions_before.log" 2>&1
if grep -Eq '/edgegrasp/(execute_trajectory|plan_target|grasp_sequence)' \
    "$probe_artifact_dir/actions_before.log"; then
  echo "EdgeGrasp motion action unexpectedly present in plan-only graph" >&2
  exit 23
fi

echo "STAGE chained_get_motion_plan_no_execute"
set +e
timeout 180s python3 scripts/probe_grasp_candidate_plan_only.py \
  --label "$probe_label" \
  --candidate-filename "$probe_candidate_filename" \
  --scene-config-filename "$probe_scene_config_filename" \
  --grasp-geometry-filename "$probe_geometry_filename" \
  --target-center-x-offset-m "$probe_target_center_x_offset_m" \
  --attempts "$probe_attempts" \
  --pipeline-id pilz_industrial_motion_planner --planner-id PTP \
  --velocity-scaling 0.1 --acceleration-scaling 0.1 \
  --allowed-planning-time-s 2.0 \
  --ros-args -p use_sim_time:=true \
  > "$probe_artifact_dir/plan_only_result.json" \
  2> "$probe_artifact_dir/plan_only_stderr.log"
probe_result_code=$?
set -e
printf '%s\n' "$probe_result_code" > "$probe_artifact_dir/plan_only_exit_code.txt"

timeout 10s ros2 node list > "$probe_artifact_dir/nodes_after.log" 2>&1 || true
timeout 10s ros2 action list -t > "$probe_artifact_dir/actions_after.log" 2>&1 || true
timeout 10s ros2 control list_controllers -c /controller_manager \
  > "$probe_artifact_dir/controllers_after.log" 2>&1 || true
timeout 10s ros2 topic echo --once /joint_states \
  > "$probe_artifact_dir/joint_states_after.log" 2>&1 || true
timeout 10s gz model -m so101 -p > "$probe_artifact_dir/robot_pose_after.log" 2>&1 || true
timeout 10s gz model -m target_cube -p > "$probe_artifact_dir/cube_pose_after.log" 2>&1 || true
{
  printf 'gazebo_server_log=%s\n' "$gazebo_server_log"
  printf 'dart_mesh_diagnostic_count='
  grep -c 'Mesh construction from an SDF has not been implemented yet for dartsim' \
    "$gazebo_server_log" 2>/dev/null || true
  printf 'geometry_creation_failure_count='
  grep -c "geometry.*couldn't be created" "$gazebo_server_log" 2>/dev/null || true
} > "$probe_artifact_dir/gazebo_diagnostics.log"
printf 'finished_at=%s\n' "$(date --iso-8601=ns)" > "$probe_artifact_dir/finish.txt"
sha256sum "$probe_artifact_dir"/*.log "$probe_artifact_dir"/*.txt \
  "$probe_artifact_dir"/*.json > "$probe_artifact_dir/sha256sums.txt"

if [ "$probe_result_code" -ne 0 ]; then
  exit "$probe_result_code"
fi
