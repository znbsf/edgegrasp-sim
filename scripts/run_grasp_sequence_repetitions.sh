#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  bash scripts/run_grasp_sequence_repetitions.sh \
    RUNS ARTIFACT_DIR TASK_PREFIX \
    APPROACH_POSITION DESCEND_POSITION LIFT_POSITION \
    APPROACH_ORIENTATION GRASP_ORIENTATION GRIPPER_POSITION_RAD

Example array syntax:
  '[0.39, 0.0, 0.23]' '[0.39, 0.0, 0.22]' \
  '[0.39, 0.0, 0.24]' '[0.0, 0.0, 0.0, 1.0]' \
  '[0.0, 0.707, 0.0, 0.707]'

Preconditions: the sourced Jazzy graph already provides fresh EdgeGrasp
inputs, PlanningScene readiness, /edgegrasp/plan_target, and
/edgegrasp/execute_trajectory. No GraspSequence server may already be running.
This script does not prove object contact or grasp physics.
EOF
}

if [[ $# -ne 9 ]]; then
  usage >&2
  exit 2
fi

runs="$1"
artifact_dir="$2"
task_prefix="$3"
approach_position="$4"
descend_position="$5"
lift_position="$6"
approach_orientation="$7"
grasp_orientation="$8"
gripper_position_rad="$9"

if ! [[ "$runs" =~ ^[1-9][0-9]*$ ]]; then
  echo "RUNS must be a positive integer" >&2
  exit 2
fi
if [[ -z "$task_prefix" || "$task_prefix" == *[[:space:]]* ]]; then
  echo "TASK_PREFIX must be nonempty and contain no whitespace" >&2
  exit 2
fi

mkdir -p "$artifact_dir"
artifact_dir="$(cd "$artifact_dir" && pwd)"
summary_path="$artifact_dir/summary.tsv"
printf 'run\ttask_id\taction_name\tclient_exit\tsequence_completed\tshutdown_clean\n' \
  > "$summary_path"

sequence_pid=""
script_pid="$$"

cleanup_sequence() {
  if [[ -z "$sequence_pid" ]]; then
    return 0
  fi
  if kill -0 "$sequence_pid" 2>/dev/null; then
    kill -INT "$sequence_pid" 2>/dev/null || true
    for _ in {1..100}; do
      if ! kill -0 "$sequence_pid" 2>/dev/null; then
        break
      fi
      sleep 0.05
    done
  fi
  if kill -0 "$sequence_pid" 2>/dev/null; then
    kill -TERM "$sequence_pid" 2>/dev/null || true
    for _ in {1..100}; do
      if ! kill -0 "$sequence_pid" 2>/dev/null; then
        break
      fi
      sleep 0.05
    done
  fi
  if kill -0 "$sequence_pid" 2>/dev/null; then
    echo "GraspSequence PID $sequence_pid did not stop; refusing broad cleanup" >&2
    return 1
  fi
  wait "$sequence_pid" 2>/dev/null || true
  sequence_pid=""
}

trap 'cleanup_sequence || true' EXIT INT TERM

action_has_server() {
  local action_name="$1"
  [[ "$(action_server_count "$action_name")" -gt 0 ]]
}

action_server_count() {
  local action_name="$1"
  local action_info
  action_info="$(ros2 action info "$action_name" 2>/dev/null || true)"
  awk '/^Action servers: [0-9]+$/ {print $3; found=1} END {if (!found) print 0}' \
    <<<"$action_info"
}

for required_action in \
  /edgegrasp/plan_target \
  /edgegrasp/execute_trajectory; do
  if ! action_has_server "$required_action"; then
    echo "required action server unavailable: $required_action" >&2
    exit 3
  fi
done

if action_has_server /edgegrasp/grasp_sequence; then
  echo "an existing GraspSequence server is active; refusing ambiguous ownership" >&2
  exit 3
fi

sequence_prefix="$(ros2 pkg prefix edgegrasp_grasp_sequence 2>/dev/null || true)"
sequence_executable="${sequence_prefix}/lib/edgegrasp_grasp_sequence/grasp_sequence"
client_executable="${sequence_prefix}/lib/edgegrasp_grasp_sequence/grasp_sequence_client"
if [[ -z "$sequence_prefix" || ! -x "$sequence_executable" || ! -x "$client_executable" ]]; then
  echo "installed GraspSequence executables are unavailable; source the colcon install" >&2
  exit 3
fi

successes=0
failures=0

for run_index in $(seq 1 "$runs"); do
  run_label="$(printf '%02d' "$run_index")"
  task_id="${task_prefix}-${run_label}"
  sequence_action="/edgegrasp/grasp_sequence_repetitions/pid_${script_pid}/run_${run_label}"
  sequence_node_name="edgegrasp_grasp_sequence_repeat_${script_pid}_${run_label}"
  client_node_name="edgegrasp_grasp_sequence_client_repeat_${script_pid}_${run_label}"
  node_log="$artifact_dir/${run_label}-sequence-node.log"
  client_log="$artifact_dir/${run_label}-client.jsonl"

  if [[ "$(action_server_count "$sequence_action")" -ne 0 ]]; then
    echo "run $run_label: unique action endpoint is unexpectedly occupied: $sequence_action" >&2
    printf '%s\t%s\t%s\t%s\t%s\t%s\n' \
      "$run_label" "$task_id" "$sequence_action" 126 false false \
      >> "$summary_path"
    failures=$((failures + 1))
    continue
  fi

  # Run the installed entry point directly.  Backgrounding `ros2 run` tracks
  # the CLI wrapper rather than its child node and can report a clean shutdown
  # while the actual action server is still alive.
  "$sequence_executable" --ros-args \
    -p use_sim_time:=true -p clock_domain:=ros_sim -p clock_epoch:=0 \
    -p target_frame:=base_link -p sequence_action:="$sequence_action" \
    -r __node:="$sequence_node_name" \
    >"$node_log" 2>&1 &
  sequence_pid="$!"

  ready=false
  for _ in {1..50}; do
    if ! kill -0 "$sequence_pid" 2>/dev/null; then
      break
    fi
    if [[ "$(action_server_count "$sequence_action")" -eq 1 ]]; then
      ready=true
      break
    fi
    sleep 0.2
  done
  if [[ "$ready" != true ]]; then
    echo "run $run_label: GraspSequence server did not become ready" >&2
    ros2 action info "$sequence_action" >>"$node_log" 2>&1 || true
    tail -40 "$node_log" >&2 || true
    printf '%s\t%s\t%s\t%s\t%s\t%s\n' \
      "$run_label" "$task_id" "$sequence_action" 125 false false \
      >> "$summary_path"
    failures=$((failures + 1))
    cleanup_sequence
    sleep 0.5
    continue
  fi

  set +e
  "$client_executable" --ros-args \
    -p use_sim_time:=true -p clock_domain:=ros_sim -p clock_epoch:=0 \
    -p sequence_action:="$sequence_action" \
    -p task_id:="$task_id" -p target_id:=ros-target \
    -p approach_position_m:="$approach_position" \
    -p descend_position_m:="$descend_position" \
    -p lift_position_m:="$lift_position" \
    -p approach_orientation_xyzw:="$approach_orientation" \
    -p grasp_orientation_xyzw:="$grasp_orientation" \
    -p gripper_closed_position_rad:="$gripper_position_rad" \
    -p planning_timeout_s:=1.0 -p result_timeout_s:=60.0 \
    -p velocity_scaling:=0.1 -p acceleration_scaling:=0.1 \
    -r __node:="$client_node_name" \
    | tee "$client_log"
  client_exit="${PIPESTATUS[0]}"
  set -e

  completed=false
  if [[ "$client_exit" -eq 0 ]] \
    && grep -Fq '"sequence_completed": true' "$client_log"; then
    completed=true
  fi

  shutdown_clean=true
  if ! cleanup_sequence; then
    shutdown_clean=false
  fi
  if grep -Eq \
    'cannot use Destroyable|exception was never retrieved|Traceback \(most recent call last\)' \
    "$node_log"; then
    shutdown_clean=false
  fi

  if [[ "$completed" == true && "$shutdown_clean" == true ]]; then
    successes=$((successes + 1))
  else
    failures=$((failures + 1))
  fi
  printf '%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$run_label" "$task_id" "$sequence_action" "$client_exit" \
    "$completed" "$shutdown_clean" >> "$summary_path"

  if [[ "$shutdown_clean" != true ]]; then
    echo "run $run_label: sequence node shutdown was not clean; refusing another run" >&2
    exit 6
  fi
  sleep 0.2
done

printf 'RESULT runs=%s successes=%s failures=%s artifacts=%s\n' \
  "$runs" "$successes" "$failures" "$artifact_dir"

if [[ "$failures" -ne 0 ]]; then
  exit 4
fi
