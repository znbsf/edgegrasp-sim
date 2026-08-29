#!/usr/bin/env bash
set -euo pipefail

# Safe runtime evidence for cancellation and goal-response timeout handling.
# This runner uses the existing in-process fake MoveGroup ActionServer.  It does
# not pause, signal, inspect, or terminate an operating-system process; it does
# not connect to a physical robot or a remote host.

if [ "$#" -ne 1 ]; then
  printf 'Usage: %s ARTIFACT_DIR\n' "$0" >&2
  exit 2
fi

artifact_dir=$1
if [ -e "$artifact_dir" ]; then
  printf 'Refusing to overwrite existing artifact path: %s\n' "$artifact_dir" >&2
  exit 2
fi

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
project_root=$(CDPATH= cd -- "$script_dir/.." && pwd)
ros_workspace=${EDGEGRASP_ROS_WS:-/home/edgegrasp/ros2_ws}
test_file="$project_root/ros_ws/src/edgegrasp_moveit_adapter/test/test_moveit_adapter_integration.py"

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  printf 'ROS Jazzy setup is missing.\n' >&2
  exit 2
fi
if [ ! -f "$ros_workspace/install/setup.bash" ]; then
  printf 'Workspace install setup is missing: %s\n' "$ros_workspace/install/setup.bash" >&2
  exit 2
fi
if [ ! -f "$test_file" ]; then
  printf 'Integration test source is missing: %s\n' "$test_file" >&2
  exit 2
fi

mkdir -p -- "$artifact_dir"
artifact_dir=$(CDPATH= cd -- "$artifact_dir" && pwd)

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$ros_workspace/install/setup.bash"
set -u

started_at=$(date --iso-8601=ns)
printf 'started_at=%s\n' "$started_at" > "$artifact_dir/start.txt"
printf 'evidence_class=INJECTED_CANCEL_TIMEOUT\n' >> "$artifact_dir/start.txt"
printf 'real_move_group_process=false\n' >> "$artifact_dir/start.txt"
printf 'os_process_signal_injection=false\n' >> "$artifact_dir/start.txt"

set +e
python3 -m pytest -q \
  "$test_file::test_injected_planning_failures_cancel_and_never_publish" \
  "$test_file::test_injected_explicit_cancel_cannot_publish_old_result" \
  --junitxml="$artifact_dir/junit.xml" \
  > "$artifact_dir/pytest.log" 2>&1
scenario_rc=$?
set -e

ended_at=$(date --iso-8601=ns)
printf 'ended_at=%s\n' "$ended_at" > "$artifact_dir/end.txt"

python3 - \
  "$artifact_dir" \
  "$scenario_rc" \
  "$started_at" \
  "$ended_at" <<'PY'
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


artifact = Path(sys.argv[1])
scenario_rc = int(sys.argv[2])
started_at = sys.argv[3]
ended_at = sys.argv[4]
junit_path = artifact / "junit.xml"
pytest_log = artifact / "pytest.log"

tests = failures = errors = skipped = 0
if junit_path.is_file():
    root = ET.parse(junit_path).getroot()
    suites = [root] if root.tag == "testsuite" else list(root.findall("testsuite"))
    tests = sum(int(suite.attrib.get("tests", 0)) for suite in suites)
    failures = sum(int(suite.attrib.get("failures", 0)) for suite in suites)
    errors = sum(int(suite.attrib.get("errors", 0)) for suite in suites)
    skipped = sum(int(suite.attrib.get("skipped", 0)) for suite in suites)


def sha256(path: Path) -> str | None:
    if not path.is_file():
        return None
    return hashlib.sha256(path.read_bytes()).hexdigest()


passed = scenario_rc == 0 and tests == 2 and failures == errors == skipped == 0
summary = {
    "schema_version": 1,
    "evidence_class": "INJECTED_CANCEL_TIMEOUT",
    "status": "PASS" if passed else "FAIL",
    "overall_pass": passed,
    "scenario_exit_code": scenario_rc,
    "started_at": started_at,
    "ended_at": ended_at,
    "runtime": {
        "in_process_dependency_injection": True,
        "fake_move_group_action_server": True,
        "real_move_group_process": False,
        "os_process_signal_injection": False,
        "remote_host_or_network_target": False,
        "physical_robot": False,
    },
    "verified": {
        "explicit_plan_target_cancel_is_fail_closed": passed,
        "goal_response_timeout_reason_is_move_group_timeout": passed,
        "late_accepted_fake_move_group_goal_is_cancelled": passed,
        "trajectory_publication_count": 0 if passed else None,
        "fake_gate_goal_count": 0 if passed else None,
        "accepted_goal_result_future_timeout": False,
    },
    "injection": {
        "goal_response_delay_s": 0.35,
        "adapter_goal_response_timeout_s": 0.05,
        "mechanism": "dedicated in-process fake MoveGroup ActionServer goal callback delay",
    },
    "pytest": {
        "tests": tests,
        "failures": failures,
        "errors": errors,
        "skipped": skipped,
        "junit_sha256": sha256(junit_path),
        "log_sha256": sha256(pytest_log),
    },
    "claim_boundary": {
        "injected_runtime_only": True,
        "real_move_group_runtime_health": False,
        "real_controller_motion_observation": False,
        "accepted_goal_result_future_timeout_verified": False,
        "hardware_evidence": False,
    },
}
(artifact / "summary.json").write_text(
    json.dumps(summary, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
)
print(json.dumps(summary, indent=2, sort_keys=True))
PY

exit "$scenario_rc"
