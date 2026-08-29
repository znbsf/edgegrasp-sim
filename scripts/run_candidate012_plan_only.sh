#!/usr/bin/env bash
# Candidate012 no-execution admission wrapper.  Material is present in the
# Gazebo robot description, but MoveIt still validates only the fixed geometry.

set -eo pipefail

if [ "$#" -ne 4 ]; then
  echo "usage: bash scripts/run_candidate012_plan_only.sh ROS_DOMAIN_ID ARTIFACT_DIR LABEL PAD_CONTACT_MATERIAL_PROFILE" >&2
  exit 2
fi

candidate012_profile=$4
case "$candidate012_profile" in
  implicit_default|candidate012_control_mu1p0|candidate012_treatment_mu1p5) ;;
  *) echo "PAD_CONTACT_MATERIAL_PROFILE is not in the Candidate012 matrix" >&2; exit 2 ;;
esac

exec bash scripts/run_grasp_candidate_plan_only.sh \
  "$1" "$2" "$3" \
  so101_grasp_geometry_candidate011_q0p40.json \
  10 0.0 "$candidate012_profile"
