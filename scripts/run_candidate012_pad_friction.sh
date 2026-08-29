#!/usr/bin/env bash
# Run one fixed Candidate012 friction row.  The generic harness still owns all
# process lifecycle and evidence capture; this wrapper freezes every non-material
# input to the Candidate011 q=0.40 baseline.

set -eo pipefail

if [ "$#" -ne 4 ]; then
  echo "usage: bash scripts/run_candidate012_pad_friction.sh ROS_DOMAIN_ID ARTIFACT_DIR TASK_ID PAD_CONTACT_MATERIAL_PROFILE" >&2
  exit 2
fi

candidate012_profile=$4
case "$candidate012_profile" in
  implicit_default|candidate012_control_mu1p0|candidate012_treatment_mu1p5) ;;
  *) echo "PAD_CONTACT_MATERIAL_PROFILE is not in the Candidate012 matrix" >&2; exit 2 ;;
esac

export EDGEGRASP_EXPERIMENT_ID=candidate012_pad_friction
exec bash scripts/run_candidate005_contact_quality.sh \
  "$1" "$2" "$3" \
  so101_grasp_geometry_candidate011_q0p40.json \
  0.40 true "$candidate012_profile"
