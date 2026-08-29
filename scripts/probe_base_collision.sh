#!/usr/bin/env bash
set -euo pipefail

world_name="${1:-edgegrasp_table_cube}"
probe_file="${2:-$(ros2 pkg prefix edgegrasp_ros)/share/edgegrasp_ros/worlds/base_proxy_probe.sdf}"
probe_name="${3:-edgegrasp_base_proxy_probe}"
probe_x="0.0"
probe_y="0.0"
probe_z="0.16"
expected_settled_z="0.0776"
settled_tolerance="0.0030"
retention_tolerance="0.0010"

extract_z() {
  awk '
    /Pose \[ XYZ \(m\) \]/ {
      getline
      gsub(/\[/, "")
      gsub(/\]/, "")
      print $3
      exit
    }
  '
}

if ! gz service -l | grep -Fxq "/world/${world_name}/create"; then
  printf 'error: Gazebo Transport create service is unavailable for world %s\n' \
    "$world_name" >&2
  exit 2
fi

printf 'PROBE_SCOPE=one_base_proxy_contact_only\n'
printf 'PROBE_FILE=%s\n' "$probe_file"
printf 'PROBE_WORLD=%s\n' "$world_name"
printf 'PROBE_SPAWN_XYZ=%s,%s,%s\n' "$probe_x" "$probe_y" "$probe_z"
ros2 run ros_gz_sim create \
  -world "$world_name" \
  -file "$probe_file" \
  -name "$probe_name" \
  -x "$probe_x" \
  -y "$probe_y" \
  -z "$probe_z" \
  -allow_renaming false

printf 'INITIAL_POSE\n'
gz model -m "$probe_name" -p
sleep 3
printf 'SETTLED_POSE_T_PLUS_3S\n'
settled_output="$(gz model -m "$probe_name" -p)"
printf '%s\n' "$settled_output"
settled_z="$(printf '%s\n' "$settled_output" | extract_z)"
sleep 2
printf 'RETENTION_POSE_T_PLUS_5S\n'
retention_output="$(gz model -m "$probe_name" -p)"
printf '%s\n' "$retention_output"
retention_z="$(printf '%s\n' "$retention_output" | extract_z)"
if [[ -z "$settled_z" || -z "$retention_z" ]]; then
  printf 'error: could not parse probe Z poses\n' >&2
  exit 5
fi
printf 'SETTLED_Z=%s\n' "$settled_z"
printf 'RETENTION_Z=%s\n' "$retention_z"
if ! awk \
  -v settled="$settled_z" \
  -v retained="$retention_z" \
  -v expected="$expected_settled_z" \
  -v settled_tol="$settled_tolerance" \
  -v retention_tol="$retention_tolerance" '
    BEGIN {
      settled_error = settled - expected
      if (settled_error < 0) settled_error = -settled_error
      retention_error = retained - settled
      if (retention_error < 0) retention_error = -retention_error
      exit !(settled_error <= settled_tol && retention_error <= retention_tol)
    }
  '; then
  printf '%s\n' \
    "error: final pose is not stable on the expected base proxy" \
    "expected_z=${expected_settled_z},settled_tolerance=${settled_tolerance},retention_tolerance=${retention_tolerance}" >&2
  exit 6
fi
printf 'CONTACT_PAIR\n'
if ! contact_output="$(timeout 5 gz topic \
  -e \
  -t /edgegrasp/base_proxy_probe_contacts \
  -n 1 \
  --json-output)"; then
  printf 'error: no base-proxy contact evidence received\n' >&2
  exit 7
fi
printf '%s\n' "$contact_output"
if ! grep -Fq 'probe_collision' <<<"$contact_output" \
  || ! grep -Fq 'so101::base_link::' <<<"$contact_output" \
  || ! grep -Fq 'edgegrasp_proxy_base_link_collision_1' \
    <<<"$contact_output"; then
  printf 'error: contact pair does not identify the expected base proxy\n' >&2
  exit 8
fi
printf '%s\n' \
  'EXPECTED=final_z_near_0.0776m and a probe-to-base_link collision pair' \
  'FAILURE_SIGNATURE=final_z_near_0.0075m indicates the probe reached the ground' \
  'RESULT=SPECIFIC_BASE_PROXY_CONTACT_PASS' \
  'LIMITATION=this verifies one base proxy only, not every arm proxy, MoveIt collision, or grasp physics'
