#!/usr/bin/env bash
# Run the fixed candidate005 grasp sequence with read-only contact-quality
# evidence. This is an evidence harness, not a claim of grasp success. It starts
# one clean ROS domain, uses only EdgeGrasp's typed motion boundary, accepts
# client exit 11 (sequence complete, physics unverified) as an observed outcome,
# and cleans only the exact process groups it created.

set -eo pipefail

if [ "$#" -lt 3 ] || [ "$#" -gt 13 ]; then
  echo "usage: bash scripts/run_candidate005_contact_quality.sh ROS_DOMAIN_ID ARTIFACT_DIR TASK_ID [GRASP_GEOMETRY_FILENAME] [GRIPPER_CLOSED_RAD] [DERIVE_DESCEND_LIFT_FROM_PROFILE] [PAD_CONTACT_MATERIAL_PROFILE] [MOVING_PAD_DISTAL_EXTENSION_M] [GRIPPER_CONTROL_PROFILE] [GRIPPER_FEEDFORWARD_EFFORT_NM] [CANDIDATE_FILENAME] [SCENE_CONFIG_FILENAME] [WORLD_FILENAME]" >&2
  exit 2
fi

trial_domain_id=$1
trial_artifact_dir=$2
trial_task_id=$3
trial_grasp_geometry_filename=${4:-so101_grasp_geometry.json}
trial_gripper_closed_rad=${5:-0.60}
trial_derive_descend_lift=${6:-false}
trial_pad_contact_material_profile=${7:-implicit_default}
trial_moving_pad_distal_extension_m=${8:-0.0}
trial_gripper_control_profile=${9:-position_only}
trial_gripper_feedforward_effort_nm=${10:-0.0}
trial_candidate_filename=${11:-so101_side_grasp_candidate.json}
trial_scene_config_filename=${12:-scene.json}
trial_world_filename=${13:-table_cube.sdf}
trial_observation_timeout_s=${EDGEGRASP_OBSERVATION_TIMEOUT_S:-30.0}

case "$trial_domain_id" in
  ''|*[!0-9]*) echo "ROS_DOMAIN_ID must be a non-negative integer" >&2; exit 2 ;;
esac
case "$trial_artifact_dir" in
  /home/edgegrasp/ros2_ws/test_results/*) ;;
  *) echo "ARTIFACT_DIR must stay under /home/edgegrasp/ros2_ws/test_results" >&2; exit 2 ;;
esac
case "$trial_task_id" in
  ''|*[!a-zA-Z0-9._-]*) echo "TASK_ID contains unsupported characters" >&2; exit 2 ;;
esac
case "$trial_grasp_geometry_filename" in
  ''|*/*|*\\*|*.json.json) echo "GRASP_GEOMETRY_FILENAME must be one JSON basename" >&2; exit 2 ;;
  *.json) ;;
  *) echo "GRASP_GEOMETRY_FILENAME must be one JSON basename" >&2; exit 2 ;;
esac
case "$trial_candidate_filename" in
  ''|*/*|*\\*) echo "CANDIDATE_FILENAME must be one JSON basename" >&2; exit 2 ;;
  *.json) ;;
  *) echo "CANDIDATE_FILENAME must be one JSON basename" >&2; exit 2 ;;
esac
case "$trial_scene_config_filename" in
  ''|*/*|*\\*) echo "SCENE_CONFIG_FILENAME must be one JSON basename" >&2; exit 2 ;;
  *.json) ;;
  *) echo "SCENE_CONFIG_FILENAME must be one JSON basename" >&2; exit 2 ;;
esac
case "$trial_world_filename" in
  ''|*/*|*\\*) echo "WORLD_FILENAME must be one SDF basename" >&2; exit 2 ;;
  *.sdf) ;;
  *) echo "WORLD_FILENAME must be one SDF basename" >&2; exit 2 ;;
esac
python3 -c 'import math,sys; value=float(sys.argv[1]); assert math.isfinite(value) and value > 0.0' \
  "$trial_gripper_closed_rad" 2>/dev/null || {
  echo "GRIPPER_CLOSED_RAD must be finite and positive" >&2
  exit 2
}
case "$trial_derive_descend_lift" in
  true|false) ;;
  *) echo "DERIVE_DESCEND_LIFT_FROM_PROFILE must be true or false" >&2; exit 2 ;;
esac
case "$trial_pad_contact_material_profile" in
  ''|*[!a-zA-Z0-9._-]*) echo "PAD_CONTACT_MATERIAL_PROFILE contains unsupported characters" >&2; exit 2 ;;
esac
python3 -c 'import math,sys; value=float(sys.argv[1]); assert math.isfinite(value) and 0.0 <= value <= 0.015' \
  "$trial_moving_pad_distal_extension_m" 2>/dev/null || {
  echo "MOVING_PAD_DISTAL_EXTENSION_M must be finite and in [0, 0.015]" >&2
  exit 2
}
case "$trial_gripper_control_profile" in
  position_only|effort_pid_preload) ;;
  position_effort_preload)
    echo "position_effort_preload is a retained negative-evidence profile: gz_ros2_control prioritizes position and shadows effort; use effort_pid_preload" >&2
    exit 2
    ;;
  *) echo "GRIPPER_CONTROL_PROFILE is unsupported" >&2; exit 2 ;;
esac
python3 -c 'import math,sys; value=float(sys.argv[1]); assert math.isfinite(value) and abs(value) <= 0.5' \
  "$trial_gripper_feedforward_effort_nm" 2>/dev/null || {
  echo "GRIPPER_FEEDFORWARD_EFFORT_NM must be finite and within +/-0.5" >&2
  exit 2
}
if [ "$trial_gripper_control_profile" = "position_only" ] \
  && [ "$trial_gripper_feedforward_effort_nm" != "0.0" ] \
  && [ "$trial_gripper_feedforward_effort_nm" != "0" ]; then
  echo "position_only requires zero gripper feed-forward effort" >&2
  exit 2
fi
if [ "$trial_gripper_control_profile" = "effort_pid_preload" ] \
  && { [ "$trial_gripper_feedforward_effort_nm" = "0.0" ] \
    || [ "$trial_gripper_feedforward_effort_nm" = "0" ]; }; then
  echo "effort_pid_preload requires one non-zero effort" >&2
  exit 2
fi
python3 -c 'import math,sys; value=float(sys.argv[1]); assert math.isfinite(value) and 0.0 < value <= 30.0' \
  "$trial_observation_timeout_s" 2>/dev/null || {
  echo "EDGEGRASP_OBSERVATION_TIMEOUT_S must be in (0, 30]" >&2
  exit 2
}
trial_observation_timeout_s=$(python3 -c \
  'import sys; print(repr(float(sys.argv[1])))' \
  "$trial_observation_timeout_s")

export ROS_DOMAIN_ID=$trial_domain_id
export ROS_LOCALHOST_ONLY=1
source /opt/ros/jazzy/setup.bash
source /home/edgegrasp/ros2_ws/install/setup.bash
trial_source_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$trial_source_root"
export PYTHONPATH="$trial_source_root/src:$trial_source_root/ros_ws/src/edgegrasp_ros:$trial_source_root/ros_ws/src/edgegrasp_grasp_sequence:$trial_source_root/ros_ws/src/edgegrasp_moveit_adapter:${PYTHONPATH:-}"
trial_velocity_scaling=${EDGEGRASP_RGBD_VELOCITY_SCALING:-0.1}
case "$trial_velocity_scaling" in 0.1|0.15) ;; *) echo "unsupported velocity scaling" >&2; exit 2 ;; esac
trial_rotation_bound=${EDGEGRASP_RGBD_ROTATION_BOUND_DEG:-0.0}
case "$trial_rotation_bound" in 0.0|5.0|20.0|40.0) ;; *) echo "unsupported rotation bound" >&2; exit 2 ;; esac
trial_camera=false
trial_camera_view=${EDGEGRASP_RGBD_CAMERA_VIEW:-upstream}
trial_camera_resolution=${EDGEGRASP_RGBD_CAMERA_RESOLUTION:-upstream}
trial_camera_rate=${EDGEGRASP_RGBD_CAMERA_HZ:-50}
case "$trial_camera_rate" in
  20|30|50) ;;
  *) echo "RGB-D camera Hz must be 20, 30 or 50" >&2; exit 2 ;;
esac
trial_gazebo_launch=(edgegrasp_ros edgegrasp_proxy_gazebo.launch.py)
trial_camera_args=()
trial_observer_wall_factor=2.0
trial_motion_mode=static
if [ "${EDGEGRASP_RGBD_TRIAL:-0}" = "1" ]; then
  test -f "${EDGEGRASP_RGBD_PLAN_RESULT:-}" || { echo "RGB-D plan-only result required" >&2; exit 2; }
  trial_camera=true
  trial_observer_wall_factor=6.0
  trial_motion_mode=${EDGEGRASP_RGBD_MOTION_MODE:-rigid_lift}
  case "$trial_motion_mode" in
    rigid_lift|planned_lift_region|measured_pad) ;;
    *) echo "Unsupported RGB-D motion mode: $trial_motion_mode" >&2; exit 2 ;;
  esac
  trial_gazebo_launch=("$trial_source_root/ros_ws/src/edgegrasp_ros/launch/edgegrasp_proxy_gazebo.launch.py")
  trial_camera_args=(camera_update_rate_hz:="$trial_camera_rate" camera_view:="$trial_camera_view" camera_resolution:="$trial_camera_resolution" launch_mock_target:=false)
fi
export GZ_PARTITION="edgegrasp_trial_${trial_domain_id}_${trial_task_id}"

trial_installed_profile="/home/edgegrasp/ros2_ws/install/edgegrasp_ros/share/edgegrasp_ros/config/${trial_grasp_geometry_filename}"
trial_installed_candidate="/home/edgegrasp/ros2_ws/install/edgegrasp_ros/share/edgegrasp_ros/config/${trial_candidate_filename}"
trial_installed_scene="/home/edgegrasp/ros2_ws/install/edgegrasp_ros/share/edgegrasp_ros/config/${trial_scene_config_filename}"
trial_installed_world="/home/edgegrasp/ros2_ws/install/edgegrasp_ros/share/edgegrasp_ros/worlds/${trial_world_filename}"
trial_installed_material_contract="/home/edgegrasp/ros2_ws/install/edgegrasp_ros/share/edgegrasp_ros/config/so101_pad_contact_materials.json"
trial_installed_xacro="/home/edgegrasp/ros2_ws/install/so101_description/share/so101_description/urdf/robots/so101.urdf.xacro"
if [ "$trial_camera" = "true" ]; then
  python3 -c '
import hashlib,json,sys,os
from pathlib import Path
r=json.loads(Path(sys.argv[1]).read_text())
assert r["status"] == "PLAN_ONLY_PASS" and not r["execution_attempted"]
expected_segments = 5 if os.environ.get("EDGEGRASP_RELEASE_CYCLE") == "1" else 3
assert r["config"]["recorded_rgbd_observation"] and len(r["segments"]) == expected_segments
assert r["config"].get("bounded_control_pan", False) == (os.environ.get("EDGEGRASP_BOUNDED_CONTROL_PAN") == "1"), "bounded pan planning/runtime mismatch"
if expected_segments == 5:
    assert r["config"].get("release_cycle") is True
    assert r["config"].get("carried_scene") is True
    assert not os.environ.get("EDGEGRASP_CARRIED_PLAN_EVIDENCE"), "recorded geometry forbidden in execution"
assert all(r[key] == 0 for key in ("trajectory_publication_count", "execute_trajectory_goal_count", "fjt_goal_count"))
assert all(row["validator_accepted"] for row in r["segments"])
assert r["config"].get("velocity_scaling", 0.1) == float(os.environ.get("EDGEGRASP_RGBD_VELOCITY_SCALING", "0.1")), "velocity scaling mismatch"
assert r["config"].get("acceleration_scaling", 0.1) == 0.1
assert r["config"]["recorded_rgbd_observation"]["camera_view"] == os.environ.get("EDGEGRASP_RGBD_CAMERA_VIEW", "upstream"), "camera view mismatch"
for key,path in zip(("candidate_sha256", "grasp_geometry_sha256", "scene_config_sha256"), sys.argv[2:]):
    assert r["config"][key] == hashlib.sha256(Path(path).read_bytes()).hexdigest(), key
' "$EDGEGRASP_RGBD_PLAN_RESULT" "$trial_installed_candidate" "$trial_installed_profile" "$trial_installed_scene"
fi
if [ ! -f "$trial_installed_profile" ]; then
  echo "installed grasp geometry profile is missing: $trial_installed_profile" >&2
  exit 2
fi
if [ ! -f "$trial_installed_candidate" ]; then
  echo "installed candidate config is missing: $trial_installed_candidate" >&2
  exit 2
fi
if [ ! -f "$trial_installed_scene" ] || [ ! -f "$trial_installed_world" ]; then
  echo "installed scene config/world is missing" >&2
  exit 2
fi
if [ ! -f "$trial_installed_material_contract" ]; then
  echo "installed pad contact-material contract is missing: $trial_installed_material_contract" >&2
  exit 2
fi
if [ ! -f "$trial_installed_xacro" ]; then
  echo "installed SO-101 xacro is missing: $trial_installed_xacro" >&2
  exit 2
fi

trial_candidate_values=$(python3 -c '
import json, math, sys
from pathlib import Path
from edgegrasp.scene import load_scene_contract, render_gazebo_sdf
payload = json.load(open(sys.argv[1], encoding="utf-8"))
scene = load_scene_contract(Path(sys.argv[2]))
if Path(sys.argv[3]).read_text(encoding="utf-8") != render_gazebo_sdf(scene):
    raise ValueError("installed scene world drift")
if scene.name != "edgegrasp_table_cube":
    raise ValueError("unsupported Gazebo world name")
cube = next(item for item in scene.objects if item.object_id == "target_cube")
if payload.get("planning_frame") != "base_link" or payload.get("planning_group") != "arm":
    raise ValueError("candidate planning contract mismatch")
if payload.get("target", {}).get("id") != "target_cube":
    raise ValueError("candidate target mismatch")
configured_center = payload.get("target", {}).get("center_m")
if configured_center is not None and tuple(float(item) for item in configured_center) != cube.pose_world.position_m:
    raise ValueError("candidate and scene target centers differ")
values = (
    payload.get("stages", {}).get("approach", {}).get("position_m"),
    payload.get("orientations_xyzw", {}).get("approach"),
    payload.get("orientations_xyzw", {}).get("grasp"),
)
for value, length in zip(values, (3, 4, 4), strict=True):
    if not isinstance(value, list) or len(value) != length:
        raise ValueError("candidate vector length mismatch")
    if any(isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(float(item)) for item in value):
        raise ValueError("candidate vector must be finite")
print("\t".join((
    *(json.dumps(value, separators=(",", ":")) for value in values),
    *(format(value, ".17g") for value in cube.pose_world.position_m),
    scene.digest,
)))
' "$trial_installed_candidate" "$trial_installed_scene" "$trial_installed_world")
IFS=$'\t' read -r trial_approach_position trial_approach_orientation \
  trial_grasp_orientation trial_target_x_m trial_target_y_m trial_target_z_m \
  trial_scene_digest <<< "$trial_candidate_values"

if [ -e "$trial_artifact_dir" ]; then
  echo "ARTIFACT_DIR already exists; use a fresh path" >&2
  exit 2
fi

mkdir -p "$trial_artifact_dir"
if [ "$trial_camera" = "true" ]; then
  python3 -c '
import hashlib,importlib,inspect,json,sys
from pathlib import Path
root=Path(sys.argv[1]).resolve()
rows=[]
for name in ("edgegrasp.action_readiness", "edgegrasp.grasp_geometry", "edgegrasp.camera", "edgegrasp.rgbd", "edgegrasp.target_motion", "edgegrasp.contact_material", "edgegrasp.release_cycle", "edgegrasp.carried_scene", "edgegrasp_ros.carried_scene_policy", "edgegrasp_ros.grasp_physics_observer", "edgegrasp_ros.planning_scene_loader", "edgegrasp_ros.trajectory_gate", "edgegrasp_moveit_adapter.adapter_node", "edgegrasp_ros.rgbd_target_publisher", "edgegrasp_grasp_sequence.node", "edgegrasp_grasp_sequence.trial_client"):
    path=Path(inspect.getfile(importlib.import_module(name))).resolve()
    assert path.is_relative_to(root), str(path)
    rows.append(dict(module=name,path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
print(json.dumps(rows,indent=2))
' "$trial_source_root" > "$trial_artifact_dir/imported_rgbd_sources.json"
fi
printf '%s\n' "$(date --iso-8601=ns)" > "$trial_artifact_dir/start.txt"
printf 'ROS_DOMAIN_ID=%s\n' "$ROS_DOMAIN_ID" >> "$trial_artifact_dir/start.txt"
printf 'RGBD_CAMERA_RESOLUTION=%s\n' "$trial_camera_resolution" >> "$trial_artifact_dir/start.txt"
printf 'VELOCITY_SCALING=%s\n' "$trial_velocity_scaling" >> "$trial_artifact_dir/start.txt"
printf 'RGBD_ROTATION_BOUND_DEG=%s\n' "$trial_rotation_bound" >> "$trial_artifact_dir/start.txt"
printf 'OBSERVED_TARGET_MOTION=%s\n' "$trial_motion_mode" >> "$trial_artifact_dir/start.txt"
printf 'OBSERVER_WALL_FACTOR=%s\n' "$trial_observer_wall_factor" >> "$trial_artifact_dir/start.txt"
printf 'RGBD_CAMERA_HZ=%s\n' "$trial_camera_rate" >> "$trial_artifact_dir/start.txt"
printf 'RGBD_CAMERA_VIEW=%s\n' "$trial_camera_view" >> "$trial_artifact_dir/start.txt"
printf 'GRASP_GEOMETRY_FILENAME=%s\n' "$trial_grasp_geometry_filename" >> "$trial_artifact_dir/start.txt"
printf 'GRIPPER_CLOSED_RAD=%s\n' "$trial_gripper_closed_rad" >> "$trial_artifact_dir/start.txt"
printf 'DERIVE_DESCEND_LIFT_FROM_PROFILE=%s\n' "$trial_derive_descend_lift" >> "$trial_artifact_dir/start.txt"
printf 'PAD_CONTACT_MATERIAL_PROFILE=%s\n' "$trial_pad_contact_material_profile" >> "$trial_artifact_dir/start.txt"
printf 'MOVING_PAD_DISTAL_EXTENSION_M=%s\n' "$trial_moving_pad_distal_extension_m" >> "$trial_artifact_dir/start.txt"
printf 'GRIPPER_CONTROL_PROFILE=%s\n' "$trial_gripper_control_profile" >> "$trial_artifact_dir/start.txt"
printf 'GRIPPER_FEEDFORWARD_EFFORT_NM=%s\n' "$trial_gripper_feedforward_effort_nm" >> "$trial_artifact_dir/start.txt"
printf 'CANDIDATE_FILENAME=%s\n' "$trial_candidate_filename" >> "$trial_artifact_dir/start.txt"
printf 'SCENE_CONFIG_FILENAME=%s\n' "$trial_scene_config_filename" >> "$trial_artifact_dir/start.txt"
printf 'WORLD_FILENAME=%s\n' "$trial_world_filename" >> "$trial_artifact_dir/start.txt"
printf 'SCENE_CONTRACT_SHA256=%s\n' "$trial_scene_digest" >> "$trial_artifact_dir/start.txt"
printf 'TARGET_POSITION_M=[%s,%s,%s]\n' "$trial_target_x_m" "$trial_target_y_m" "$trial_target_z_m" >> "$trial_artifact_dir/start.txt"
printf 'APPROACH_POSITION_M=%s\n' "$trial_approach_position" >> "$trial_artifact_dir/start.txt"
printf 'APPROACH_ORIENTATION_XYZW=%s\n' "$trial_approach_orientation" >> "$trial_artifact_dir/start.txt"
printf 'GRASP_ORIENTATION_XYZW=%s\n' "$trial_grasp_orientation" >> "$trial_artifact_dir/start.txt"
printf 'OBSERVATION_TIMEOUT_S=%s\n' "$trial_observation_timeout_s" >> "$trial_artifact_dir/start.txt"
sha256sum "$trial_installed_material_contract" > "$trial_artifact_dir/pad_contact_material_contract.sha256"
if [ "${EDGEGRASP_EXPERIMENT_ID:-}" = "candidate012_pad_friction" ]; then
  python3 scripts/validate_candidate012_experiment.py \
    --profile "$trial_pad_contact_material_profile" \
    --experiment /home/edgegrasp/ros2_ws/install/edgegrasp_ros/share/edgegrasp_ros/config/candidate012_pad_friction_experiment.json \
    --config-root /home/edgegrasp/ros2_ws/install/edgegrasp_ros/share/edgegrasp_ros/config \
    --world-root /home/edgegrasp/ros2_ws/install/edgegrasp_ros/share/edgegrasp_ros/worlds \
    --output "$trial_artifact_dir/candidate012_contract_validation.json" \
    > "$trial_artifact_dir/candidate012_contract_validation_stdout.json"
fi

trial_pids=()

cleanup_trial() {
  trial_exit_code=$?
  {
    printf 'cleanup_start=%s\n' "$(date --iso-8601=ns)"
    printf 'original_exit=%s\n' "$trial_exit_code"
    for trial_pid in "${trial_pids[@]}"; do
      if kill -0 "$trial_pid" 2>/dev/null; then
        trial_pgid=$(ps -o pgid= -p "$trial_pid" 2>/dev/null | tr -d ' ' || true)
        printf 'term_pid=%s pgid=%s\n' "$trial_pid" "$trial_pgid"
        if [ -n "$trial_pgid" ] && [ "$trial_pgid" = "$trial_pid" ]; then
          kill -TERM -- "-$trial_pid" 2>/dev/null || true
        else
          kill -TERM "$trial_pid" 2>/dev/null || true
        fi
      fi
    done
    for _ in $(seq 1 15); do
      trial_alive=0
      for trial_pid in "${trial_pids[@]}"; do
        kill -0 "$trial_pid" 2>/dev/null && trial_alive=$((trial_alive + 1))
      done
      [ "$trial_alive" -eq 0 ] && break
      sleep 1
    done
    for trial_pid in "${trial_pids[@]}"; do
      if kill -0 "$trial_pid" 2>/dev/null; then
        trial_pgid=$(ps -o pgid= -p "$trial_pid" 2>/dev/null | tr -d ' ' || true)
        printf 'kill_pid=%s pgid=%s\n' "$trial_pid" "$trial_pgid"
        if [ -n "$trial_pgid" ] && [ "$trial_pgid" = "$trial_pid" ]; then
          kill -KILL -- "-$trial_pid" 2>/dev/null || true
        else
          kill -KILL "$trial_pid" 2>/dev/null || true
        fi
      fi
    done
    # ros2 launch children may create their own process groups. Enumerate the
    # exact ROS domain through /proc so no reparented child can survive merely
    # because its launch parent exited first.
    bash scripts/cleanup_ros_domain.sh "$ROS_DOMAIN_ID" --terminate || true
    sleep 2
    printf '%s\n' 'matching_processes_after_cleanup:'
    ps -eo pid=,ppid=,args= \
      | grep -E '(^|/)(gz sim|gzserver|move_group|controller_manager|trajectory_gate|grasp_physics_observer|grasp_sequence|moveit_adapter|planning_scene_loader)( |$)' \
      | grep -v grep || true
    printf '%s\n' 'clock_after_cleanup:'
    timeout 8s ros2 topic info /clock 2>&1 || true
    ros2 daemon stop >/dev/null 2>&1 || true
    printf 'cleanup_end=%s\n' "$(date --iso-8601=ns)"
  } >> "$trial_artifact_dir/cleanup.log" 2>&1
  return "$trial_exit_code"
}

trap cleanup_trial EXIT INT TERM

launch_trial_group() {
  trial_log=$1
  shift
  setsid "$@" > "$trial_artifact_dir/$trial_log" 2>&1 &
  trial_pid=$!
  trial_pids+=("$trial_pid")
  printf '%s %s\n' "$trial_pid" "$*" >> "$trial_artifact_dir/launch_parents.log"
}

ros2 daemon stop >/dev/null 2>&1 || true
preexisting_processes=$(ps -eo pid=,ppid=,args= \
  | grep -E '(^|/)(gz sim|gzserver|move_group|controller_manager|trajectory_gate|grasp_physics_observer|grasp_sequence|moveit_adapter|planning_scene_loader)( |$)' \
  | grep -v grep || true)
printf '%s\n' "$preexisting_processes" > "$trial_artifact_dir/preexisting_processes.log"
if [ -n "$preexisting_processes" ]; then
  echo "pre-existing ROS/Gazebo process matched; refusing to start" >&2
  exit 3
fi
if timeout 8s ros2 topic info /clock > "$trial_artifact_dir/clock_preflight.log" 2>&1; then
  echo "pre-existing /clock graph detected; refusing to start" >&2
  exit 3
fi

trial_cycle_count=${EDGEGRASP_RELEASE_CYCLE_COUNT:-1}
if [[ "$trial_cycle_count" != 1 && "$trial_cycle_count" != 3 ]]; then
  echo "cycle count must be predeclared as 1 or 3" >&2
  exit 2
fi
if [[ "$trial_cycle_count" == 3 && ( "${EDGEGRASP_RELEASE_CYCLE:-0}" != 1 || "$trial_gripper_control_profile" != effort_pid_preload ) ]]; then
  echo "three cycles require the complete release-cycle contract" >&2
  exit 2
fi

echo "STAGE validate_pad_contact_material_mapping"
python3 scripts/generate_collision_proxy_urdf.py \
  --xacro "$trial_installed_xacro" \
  --contract "/home/edgegrasp/ros2_ws/install/edgegrasp_ros/share/edgegrasp_ros/config/so101_collision_proxies.json" \
  --material-contract "$trial_installed_material_contract" \
  --pad-contact-material-profile "$trial_pad_contact_material_profile" \
  --moving-pad-distal-extension-m "$trial_moving_pad_distal_extension_m" \
  --gripper-control-profile "$trial_gripper_control_profile" \
  --use-camera "$trial_camera" --camera-view "$trial_camera_view" --camera-resolution "$trial_camera_resolution" --camera-update-rate-hz "$trial_camera_rate" \
  --output "$trial_artifact_dir/generated_proxy.urdf" \
  --report "$trial_artifact_dir/generated_proxy_report.json" \
  > "$trial_artifact_dir/generated_proxy_stdout.json"
gz sdf -p "$trial_artifact_dir/generated_proxy.urdf" \
  > "$trial_artifact_dir/generated_proxy.sdf" \
  2> "$trial_artifact_dir/gz_sdf_stderr.log"
python3 scripts/validate_pad_contact_material_sdf.py \
  --sdf "$trial_artifact_dir/generated_proxy.sdf" \
  --contract "$trial_installed_material_contract" \
  --profile "$trial_pad_contact_material_profile" \
  --output "$trial_artifact_dir/pad_contact_material_mapping.json" \
  > "$trial_artifact_dir/pad_contact_material_mapping_stdout.json"

echo "STAGE launch_gazebo"
launch_trial_group gazebo.log ros2 launch "${trial_gazebo_launch[@]}" observation_wall_factor:="$trial_observer_wall_factor" \
  use_camera:="$trial_camera" "${trial_camera_args[@]}" \
  world_filename:="$trial_world_filename" \
  scene_config_filename:="$trial_scene_config_filename" \
  pad_contact_material_profile:="$trial_pad_contact_material_profile" \
  moving_pad_distal_extension_m:="$trial_moving_pad_distal_extension_m" \
  gripper_control_profile:="$trial_gripper_control_profile" \
  future_skew_tolerance_ms:=1000.0 target_publisher_start_delay_s:=3.0 \
  launch_edgegrasp_nodes:=true launch_physics_observer:=true \
  target_id:=target_cube \
  target_x_m:="$trial_target_x_m" \
  target_y_m:="$trial_target_y_m" target_z_m:="$trial_target_z_m"
if [ "$trial_camera" = "true" ]; then
  launch_trial_group perception.log python3 -m edgegrasp_ros.rgbd_target_publisher \
    --ros-args -p use_sim_time:=true -p yaw_rad:=0.6500077341171558 -p max_rotation_deg:="$trial_rotation_bound"
fi

trial_ready=0
for _ in $(seq 1 150); do
  trial_clock=$(timeout 5s ros2 topic info /clock 2>/dev/null || true)
  trial_controllers=$(timeout 5s ros2 control list_controllers \
    -c /controller_manager 2>/dev/null || true)
  if printf '%s' "$trial_clock" | grep -q 'Publisher count: 1' \
    && [ "$(printf '%s\n' "$trial_controllers" | grep -c ' active')" -ge 3 ]; then
    trial_ready=1
    break
  fi
  sleep 1
done
printf '%s\n' "$trial_clock" > "$trial_artifact_dir/clock_before.log"
printf '%s\n' "$trial_controllers" > "$trial_artifact_dir/controllers_before.log"
if [ "$trial_ready" -ne 1 ]; then
  echo "Gazebo/controller readiness timeout" >&2
  exit 20
fi
timeout 10s ros2 control list_hardware_interfaces -c /controller_manager \
  > "$trial_artifact_dir/hardware_interfaces.log" 2>&1
timeout 10s ros2 param dump /gripper_controller \
  > "$trial_artifact_dir/gripper_controller_parameters.yaml" 2>&1
timeout 10s ros2 topic info --verbose /gripper_controller/controller_state \
  > "$trial_artifact_dir/gripper_controller_state_topic.log" 2>&1

gazebo_server_log=$(find /home/edgegrasp/.gz/sim/log -mindepth 1 -maxdepth 1 \
  -type d -printf '%T@ %p\n' | sort -nr | sed -n '1s/^[^ ]* //p')/server_console.log
printf '%s\n' "$gazebo_server_log" > "$trial_artifact_dir/gazebo_server_log_path.txt"

echo "STAGE launch_moveit_edgegrasp"
launch_trial_group move_group.log ros2 launch "$trial_source_root/ros_ws/src/edgegrasp_ros/launch/edgegrasp_proxy_move_group.launch.py" robot_name:=so101 use_camera:="$trial_camera" camera_view:="$trial_camera_view" camera_resolution:="$trial_camera_resolution" \
  use_gazebo:=true use_sim_time:=true use_rviz:=false \
  moving_pad_distal_extension_m:="$trial_moving_pad_distal_extension_m"
launch_trial_group planning_scene.log ros2 launch edgegrasp_ros \
  planning_scene.launch.py use_sim_time:=true clock_domain:=ros_sim \
  clock_epoch:=0 target_frame:=base_link include_optional_cube:=true \
  scene_config:="$trial_installed_scene"
launch_trial_group adapter.log ros2 launch edgegrasp_moveit_adapter \
  moveit_adapter.launch.py use_sim_time:=true clock_domain:=ros_sim \
  planning_frame:=base_link future_skew_tolerance_ms:=1000.0
launch_trial_group sequence.log ros2 launch \
  "$trial_source_root/ros_ws/src/edgegrasp_grasp_sequence/launch/grasp_sequence.launch.py" \
  use_sim_time:=true clock_domain:=ros_sim observed_target_motion:="$trial_motion_mode" \
  observed_cube_geometry_profile:="/home/edgegrasp/ros2_ws/install/edgegrasp_ros/share/edgegrasp_ros/config/$trial_grasp_geometry_filename" \
  clock_epoch:=0 target_frame:=base_link future_skew_tolerance_ms:=1000.0 \
  gripper_feedforward_effort_nm:="$trial_gripper_feedforward_effort_nm"

trial_ready=0
for _ in $(seq 1 150); do
  trial_actions=$(timeout 5s ros2 action list 2>/dev/null || true)
  trial_services=$(timeout 5s ros2 service list 2>/dev/null || true)
  if printf '%s\n' "$trial_actions" | grep -qx '/edgegrasp/execute_trajectory' \
    && printf '%s\n' "$trial_actions" | grep -qx '/edgegrasp/plan_target' \
    && printf '%s\n' "$trial_actions" | grep -qx '/edgegrasp/grasp_sequence' \
    && printf '%s\n' "$trial_actions" | grep -qx '/edgegrasp/grasp_physics_evidence' \
    && printf '%s\n' "$trial_services" | grep -qx '/edgegrasp/set_target_pad_contacts'; then
    trial_ready=1
    break
  fi
  sleep 1
done
printf '%s\n' "$trial_actions" > "$trial_artifact_dir/actions_before.log"
if [ "$trial_ready" -ne 1 ]; then
  echo "EdgeGrasp/MoveIt readiness timeout" >&2
  exit 21
fi

trial_scene_base_ready=0
for _ in $(seq 1 60); do
  trial_scene_status=$(timeout 8s ros2 topic echo --once --full-length \
    /edgegrasp/planning_scene_status 2>/dev/null || true)
  if printf '%s' "$trial_scene_status" | grep -q '"ready": true' \
    && printf '%s' "$trial_scene_status" | grep -q '"reason": "confirmed"' \
    && printf '%s' "$trial_scene_status" \
      | grep -q '"allow_target_pad_contacts": false'; then
    trial_scene_base_ready=1
    break
  fi
  sleep 1
done
printf '%s\n' "$trial_scene_status" > "$trial_artifact_dir/planning_scene_status_before_acm.log"
if [ "$trial_scene_base_ready" -ne 1 ]; then
  echo "planning scene must begin confirmed with target-pad contacts disabled" >&2
  exit 22
fi
printf '%s\n' "$trial_scene_status" > "$trial_artifact_dir/planning_scene_status.log"
printf '%s\n' \
  "MoveIt adapter owns stage-scoped target-pad ACM transitions." \
  > "$trial_artifact_dir/contact_policy_owner.txt"

echo "STAGE preflight_topics"
trial_permission_ready=0
for _ in $(seq 1 60); do
  trial_motion=$(timeout 5s ros2 topic echo --once \
    /edgegrasp/motion_allowed 2>&1 || true)
  trial_interface=$(timeout 5s ros2 topic echo --once \
    /edgegrasp/interface_ready 2>&1 || true)
  if printf '%s' "$trial_motion" | grep -q 'data: true' \
    && printf '%s' "$trial_interface" | grep -q 'data: true'; then
    trial_permission_ready=1
    break
  fi
  timeout 5s ros2 topic echo --once /edgegrasp/safety_status \
    >> "$trial_artifact_dir/safety_status_wait.log" 2>&1 || true
  sleep 1
done
printf '%s\n' "$trial_motion" > "$trial_artifact_dir/motion_allowed.log"
printf '%s\n' "$trial_interface" > "$trial_artifact_dir/interface_ready.log"
if [ "$trial_permission_ready" -ne 1 ]; then
  echo "motion permission/interface readiness timeout" >&2
  exit 22
fi

echo "STAGE start_single_clock_mcap"
launch_trial_group mcap.log bash scripts/record_mcap.sh \
  "$trial_artifact_dir/mcap" ros_sim
trial_mcap_pid=$trial_pid
trial_mcap_ready=0
for _ in $(seq 1 30); do
  if timeout 5s ros2 node list 2>/dev/null \
    | grep -Eq '^/rosbag2_recorder(_[0-9]+)?$'; then
    trial_mcap_ready=1
    break
  fi
  sleep 1
done
if [ "$trial_mcap_ready" -ne 1 ]; then
  echo "MCAP recorder readiness timeout" >&2
  exit 23
fi

echo "STAGE typed_gripper_prep"
trial_prepare_effort_args=()
if [ "$trial_gripper_control_profile" = "effort_pid_preload" ]; then
  trial_prepare_effort_args=(--gripper-effort 0.0)
fi
python3 scripts/prepare_so101_trial.py --gripper-only --gripper 1.5 \
  "${trial_prepare_effort_args[@]}" \
  --target-id target_cube --task-id "${trial_task_id}-prep" --timeout-s 20 \
  > "$trial_artifact_dir/gripper_prep.log" 2>&1

trial_base_task_id=$trial_task_id
trial_fixed_control_geometry=false
trial_bounded_control_pan=false
if [[ "${EDGEGRASP_BOUNDED_CONTROL_PAN:-0}" == 1 ]]; then
  [[ "${EDGEGRASP_RELEASE_CYCLE:-0}" == 1 ]] || exit 2
  trial_bounded_control_pan=true
fi
if [[ "${EDGEGRASP_RELEASE_CYCLE:-0}" == 1 ]]; then
  trial_fixed_control_geometry=true
fi
trial_completed_count=0
trial_attempted_count=0
for trial_cycle_index in $(seq 1 "$trial_cycle_count"); do
  trial_cycle_dir=$trial_artifact_dir
  if [[ "$trial_cycle_count" == 3 ]]; then
    trial_cycle_dir="$trial_artifact_dir/cycle_$trial_cycle_index"
    mkdir "$trial_cycle_dir"
    trial_task_id="${trial_base_task_id}-cycle-${trial_cycle_index}"
  fi
  trial_attempted_count=$trial_cycle_index
echo "STAGE contact_quality_timing_trial"
set +e
timeout 260s ros2 run edgegrasp_grasp_sequence grasp_trial_client --ros-args \
  -p use_sim_time:=true -p clock_domain:=ros_sim -p clock_epoch:=0 \
  -p task_id:="$trial_task_id" -p target_id:=target_cube \
  -p approach_position_m:="$trial_approach_position" \
  -p descend_position_m:="[0.2462364349184608,0.15125054509958977,0.21671404504175756]" \
  -p lift_position_m:="[0.2462364349184608,0.15125054509958977,0.25671404504175754]" \
  -p approach_orientation_xyzw:="$trial_approach_orientation" \
  -p grasp_orientation_xyzw:="$trial_grasp_orientation" \
  -p grasp_geometry_filename:="$trial_grasp_geometry_filename" \
  -p scene_config_filename:="$trial_scene_config_filename" \
  -p derive_descend_and_lift_from_profile:="$trial_derive_descend_lift" \
  -p require_measured_geometry:="$trial_camera" \
  -p fixed_control_stage_geometry:="$trial_fixed_control_geometry" \
  -p bounded_control_pan:="$trial_bounded_control_pan" \
  -p gripper_closed_position_rad:="$trial_gripper_closed_rad" \
  -p pipeline_id:=pilz_industrial_motion_planner -p planner_id:=PTP \
  -p planning_timeout_s:=2.0 -p sequence_timeout_s:=90.0 \
  -p physics_timeout_s:=100.0 \
  -p observation_timeout_s:="$trial_observation_timeout_s" \
  -p velocity_scaling:="$trial_velocity_scaling" -p acceleration_scaling:=0.1 \
  > "$trial_cycle_dir/trial.log" 2>&1
trial_result_code=$?
set -e
printf '%s\n' "$trial_result_code" > "$trial_cycle_dir/trial_exit_code.txt"
printf 'TRIAL_RC=%s\n' "$trial_result_code"

trial_release_code=0
trial_release_fallback_code=0
if [ "$trial_gripper_control_profile" = "effort_pid_preload" ]; then
  echo "STAGE bounded_zero_effort_release"
  set +e
  if [ "${EDGEGRASP_RELEASE_CYCLE:-0}" = "1" ]; then
    if [ "$trial_result_code" -eq 0 ]; then
      timeout 160s python3 scripts/complete_release_cycle.py \
        --plan "$EDGEGRASP_RGBD_PLAN_RESULT" --task-id "$trial_task_id" \
        --output "$trial_cycle_dir/release_cycle.jsonl" \
        > "$trial_cycle_dir/gripper_release.log" 2>&1
      trial_release_code=$?
    else
      printf '%s\n' 'grasp_not_independently_verified;no_place_or_release_command' \
        > "$trial_cycle_dir/gripper_release.log"
      trial_release_code=2
    fi
  else
  python3 scripts/prepare_so101_trial.py --gripper-only --gripper 1.5 \
    --gripper-effort 0.0 --target-id target_cube \
    --task-id "${trial_task_id}-release" --timeout-s 20 \
    > "$trial_cycle_dir/gripper_release.log" 2>&1
  trial_release_code=$?
  fi
  set -e
  printf '%s\n' "$trial_release_code" \
    > "$trial_cycle_dir/gripper_release_exit_code.txt"
  if [ "$trial_release_code" -ne 0 ]; then
    # A downstream FJT failure deliberately latches the typed gate. Do not
    # weaken that latch merely to send another motion goal. Instead stop the
    # exact simulated gripper controller and prove its effort interface is no
    # longer claimed before process cleanup.
    echo "STAGE emergency_gripper_controller_deactivate"
    set +e
    timeout 15s ros2 control switch_controllers \
      --deactivate gripper_controller --strict \
      -c /controller_manager \
      > "$trial_cycle_dir/gripper_deactivate.log" 2>&1
    trial_release_fallback_code=$?
    timeout 10s ros2 control list_controllers -c /controller_manager \
      > "$trial_cycle_dir/controllers_after_gripper_deactivate.log" 2>&1
    trial_controller_state_code=$?
    timeout 10s ros2 control list_hardware_interfaces -c /controller_manager \
      > "$trial_cycle_dir/hardware_interfaces_after_gripper_deactivate.log" \
      2>&1
    trial_interface_state_code=$?
    set -e
    if [ "$trial_release_fallback_code" -eq 0 ] \
      && [ "$trial_controller_state_code" -eq 0 ] \
      && [ "$trial_interface_state_code" -eq 0 ] \
      && grep -Eq '^gripper_controller[[:space:]]+joint_trajectory_controller/JointTrajectoryController[[:space:]]+inactive$' \
        "$trial_cycle_dir/controllers_after_gripper_deactivate.log" \
      && grep -Eq 'gripper/effort[[:space:]]+\[available\][[:space:]]+\[unclaimed\]' \
        "$trial_cycle_dir/hardware_interfaces_after_gripper_deactivate.log"; then
      printf '%s\n' 'CONTROLLER_DEACTIVATED_AND_EFFORT_UNCLAIMED' \
        > "$trial_cycle_dir/gripper_release_fallback_status.txt"
    else
      trial_release_fallback_code=1
      printf '%s\n' 'GRIPPER_RELEASE_FALLBACK_UNCONFIRMED' \
        > "$trial_cycle_dir/gripper_release_fallback_status.txt"
    fi
    printf '%s\n' "$trial_release_fallback_code" \
      > "$trial_cycle_dir/gripper_release_fallback_exit_code.txt"
  fi
fi


  if [[ "$trial_result_code" != 0 || "$trial_release_code" != 0 ]]; then
    break
  fi
  trial_completed_count=$((trial_completed_count + 1))
done
trial_task_id=$trial_base_task_id
python3 - "$trial_artifact_dir/cycle_summary.json" "$trial_cycle_count" "$trial_attempted_count" "$trial_completed_count" <<'PYCYCLES'
import json,sys
from pathlib import Path
requested,attempted,completed=map(int,sys.argv[2:])
root=Path(sys.argv[1]).parent
renewals=[]
for index in range(1,attempted+1):
    log=(root/f"cycle_{index}" if requested==3 else root)/"trial.log"
    count=0
    for line in log.read_text().splitlines():
        try:
            count += json.loads(line).get("type") == "physics_baseline_restart"
        except (ValueError, AttributeError):
            pass
    renewals.append(count)
with Path(sys.argv[1]).open('x') as stream:
    json.dump(dict(requested=requested,attempted=attempted,completed=completed,
                   complete=requested==completed,cycle_retry_count=0,
                   physics_baseline_request_renewals=renewals,
                   controller_restart_between_cycles=False),stream,indent=2)
PYCYCLES

echo "STAGE finalize_single_clock_mcap"
trial_mcap_pgid=$(ps -o pgid= -p "$trial_mcap_pid" 2>/dev/null \
  | tr -d ' ' || true)
if [ -z "$trial_mcap_pgid" ] || [ "$trial_mcap_pgid" != "$trial_mcap_pid" ]; then
  echo "MCAP recorder process-group identity is unavailable" >&2
  exit 26
fi
if kill -0 -- "-$trial_mcap_pgid" 2>/dev/null; then
  kill -INT -- "-$trial_mcap_pgid" 2>/dev/null || true
  for _ in $(seq 1 20); do
    kill -0 -- "-$trial_mcap_pgid" 2>/dev/null || break
    sleep 0.25
  done
fi
if kill -0 -- "-$trial_mcap_pgid" 2>/dev/null; then
  echo "MCAP recorder ignored SIGINT; sending bounded SIGTERM" \
    >> "$trial_artifact_dir/mcap.log"
  kill -TERM -- "-$trial_mcap_pgid" 2>/dev/null || true
  for _ in $(seq 1 40); do
    kill -0 -- "-$trial_mcap_pgid" 2>/dev/null || break
    sleep 0.25
  done
fi
if kill -0 -- "-$trial_mcap_pgid" 2>/dev/null; then
  echo "MCAP recorder did not terminate after SIGTERM" >&2
  exit 26
fi
wait "$trial_mcap_pid" 2>/dev/null || true
timeout 15s ros2 bag info "$trial_artifact_dir/mcap" \
  > "$trial_artifact_dir/mcap_info.log" 2>&1
if [ -d "$trial_artifact_dir/mcap" ]; then
  find "$trial_artifact_dir/mcap" -type f -print0 \
    | sort -z \
    | xargs -0 -r sha256sum \
    > "$trial_artifact_dir/mcap_sha256sums.txt"
fi

for trial_cycle_index in $(seq 1 "$trial_attempted_count"); do
  trial_cycle_dir=$trial_artifact_dir
  trial_analysis_task_id=$trial_base_task_id
  if [[ "$trial_cycle_count" == 3 ]]; then
    trial_cycle_dir="$trial_artifact_dir/cycle_$trial_cycle_index"
    trial_analysis_task_id="${trial_base_task_id}-cycle-${trial_cycle_index}"
  fi
echo "STAGE analyze_single_clock_timeline"
set +e
python3 scripts/analyze_grasp_mcap.py \
  --bag "$trial_artifact_dir/mcap" \
  --task-id "$trial_analysis_task_id" \
  --output "$trial_cycle_dir/grasp_timeline_summary.json" \
  --csv "$trial_cycle_dir/grasp_timeline.csv" \
  > "$trial_cycle_dir/grasp_timeline_stdout.json" \
  2> "$trial_cycle_dir/grasp_timeline_stderr.log"
trial_timeline_analysis_code=$?
set -e
printf '%s\n' "$trial_timeline_analysis_code" \
  > "$trial_cycle_dir/grasp_timeline_exit_code.txt"

done

echo "STAGE final_capture"
timeout 10s ros2 control list_controllers -c /controller_manager \
  > "$trial_artifact_dir/controllers_final.log" 2>&1 || true
timeout 10s ros2 action list -t > "$trial_artifact_dir/actions_final.log" 2>&1 || true
timeout 10s ros2 topic info /clock -v \
  > "$trial_artifact_dir/clock_info_final.log" 2>&1 || true
timeout 10s ros2 topic echo --once /joint_states \
  > "$trial_artifact_dir/joint_states_final.log" 2>&1 || true
timeout 10s gz model -m so101 -p > "$trial_artifact_dir/robot_pose_final.log" 2>&1 || true
timeout 10s gz model -m target_cube -p > "$trial_artifact_dir/cube_pose_final.log" 2>&1 || true
timeout 10s gz topic -l | grep edgegrasp \
  > "$trial_artifact_dir/contact_topics_final.log" 2>&1 || true
{
  printf 'gazebo_server_log=%s\n' "$gazebo_server_log"
  printf 'dart_mesh_diagnostic_count='
  grep -c 'Mesh construction from an SDF has not been implemented yet for dartsim' \
    "$gazebo_server_log" 2>/dev/null || true
  printf 'geometry_creation_failure_count='
  grep -c "geometry.*couldn't be created" "$gazebo_server_log" 2>/dev/null || true
} > "$trial_artifact_dir/gazebo_diagnostics.log"
sha256sum "$trial_artifact_dir"/*.json "$trial_artifact_dir"/*.log \
  "$trial_artifact_dir"/*.sdf "$trial_artifact_dir"/*.txt \
  "$trial_artifact_dir"/*.urdf \
  > "$trial_artifact_dir/sha256sums.txt"
printf '%s\n' "$(date --iso-8601=ns)" > "$trial_artifact_dir/capture_end.txt"

if [ "$trial_result_code" -ne 0 ] && [ "$trial_result_code" -ne 11 ]; then
  exit "$trial_result_code"
fi
if [ "$trial_release_code" -ne 0 ]; then
  exit 24
fi
if [ "$trial_timeline_analysis_code" -ne 0 ]; then
  exit 25
fi
