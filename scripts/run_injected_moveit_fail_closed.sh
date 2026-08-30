#!/usr/bin/env bash
set -euo pipefail

# Safe runtime evidence for cancellation and goal-response timeout handling.
# This runner uses an in-process fake MoveGroup/trajectory-gate dependency. It
# never starts a real MoveGroup process, contacts a remote host, or controls a
# robot. The JSONL ledger is opt-in through EDGEGRASP_INJECTED_EVENT_LOG and is
# validated against JUnit before a PASS can be reported.

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
runner_file="$script_dir/$(basename -- "$0")"
test_file="$project_root/ros_ws/src/edgegrasp_moveit_adapter/test/test_moveit_adapter_integration.py"
adapter_file="$project_root/ros_ws/src/edgegrasp_moveit_adapter/edgegrasp_moveit_adapter/adapter_node.py"

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
if [ ! -f "$adapter_file" ]; then
  printf 'Adapter source is missing: %s\n' "$adapter_file" >&2
  exit 2
fi

mkdir -p -- "$artifact_dir"
artifact_dir=$(CDPATH= cd -- "$artifact_dir" && pwd)
event_log="$artifact_dir/scenarios.jsonl"
export EDGEGRASP_INJECTED_EVENT_LOG="$event_log"

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$ros_workspace/install/setup.bash"
set -u

started_at=$(date --iso-8601=ns)
{
  printf 'started_at=%s\n' "$started_at"
  printf 'schema_version=2\n'
  printf 'evidence_class=INJECTED_CANCEL_TIMEOUT\n'
  printf 'runtime_scope=in_process_fake\n'
  printf 'real_move_group=false\n'
  printf 'controller=false\n'
  printf 'simulation_physics=false\n'
  printf 'hardware=false\n'
  printf 'event_log=%s\n' "$event_log"
} > "$artifact_dir/start.txt"

set +e
python3 -m pytest -q \
  "$test_file::test_injected_move_group_goal_response_timeout_cancels_late_accepted_goal" \
  "$test_file::test_injected_move_group_accepted_result_timeout_is_correlated_and_fail_closed" \
  "$test_file::test_move_group_result_future_request_exception_is_fail_closed" \
  "$test_file::test_move_group_result_future_none_is_fail_closed" \
  "$test_file::test_injected_gate_goal_response_timeout_cancels_late_accepted_gate_goal" \
  "$test_file::test_gate_result_future_none_is_fail_closed_and_cancels_exact_goal" \
  "$test_file::test_gate_result_exception_is_fail_closed_and_cancels_exact_goal" \
  "$test_file::test_move_group_result_timeout_with_unconfirmed_terminal_stays_fail_closed" \
  "$test_file::test_move_group_cancel_rejected_then_late_success_never_reaches_gate" \
  "$test_file::test_move_group_late_result_from_old_generation_cannot_dispatch_same_id_retry" \
  "$test_file::test_move_group_late_success_from_old_generation_cannot_dispatch_same_id_retry" \
  "$test_file::test_injected_explicit_cancel_cannot_publish_old_result" \
  --junitxml="$artifact_dir/junit.xml" \
  > "$artifact_dir/pytest.log" 2>&1
scenario_rc=$?
set -e

ended_at=$(date --iso-8601=ns)
printf 'ended_at=%s\n' "$ended_at" > "$artifact_dir/end.txt"

set +e
python3 - \
  "$artifact_dir" \
  "$scenario_rc" \
  "$started_at" \
  "$ended_at" \
  "$runner_file" \
  "$test_file" \
  "$adapter_file" <<'PY'
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET


artifact = Path(sys.argv[1])
scenario_rc = int(sys.argv[2])
started_at = sys.argv[3]
ended_at = sys.argv[4]
source_paths = {
    "runner": Path(sys.argv[5]),
    "integration_test": Path(sys.argv[6]),
    "adapter": Path(sys.argv[7]),
}
junit_path = artifact / "junit.xml"
pytest_log = artifact / "pytest.log"
event_log = artifact / "scenarios.jsonl"

EXPECTED_TESTS = {
    "test_injected_move_group_goal_response_timeout_cancels_late_accepted_goal",
    "test_injected_move_group_accepted_result_timeout_is_correlated_and_fail_closed",
    "test_move_group_result_future_request_exception_is_fail_closed",
    "test_move_group_result_future_none_is_fail_closed",
    "test_injected_gate_goal_response_timeout_cancels_late_accepted_gate_goal",
    "test_gate_result_future_none_is_fail_closed_and_cancels_exact_goal",
    "test_gate_result_exception_is_fail_closed_and_cancels_exact_goal",
    "test_move_group_result_timeout_with_unconfirmed_terminal_stays_fail_closed",
    "test_move_group_cancel_rejected_then_late_success_never_reaches_gate",
    "test_move_group_late_result_from_old_generation_cannot_dispatch_same_id_retry",
    "test_move_group_late_success_from_old_generation_cannot_dispatch_same_id_retry",
    "test_injected_explicit_cancel_cannot_publish_old_result",
}
TEST_BY_BASE_SCENARIO = {
    "move_group_goal_response_timeout": "test_injected_move_group_goal_response_timeout_cancels_late_accepted_goal",
    "late_move_group_send": "test_injected_move_group_goal_response_timeout_cancels_late_accepted_goal",
    "move_group_accepted_result_timeout": "test_injected_move_group_accepted_result_timeout_is_correlated_and_fail_closed",
    "move_group_result_future_exception": "test_move_group_result_future_request_exception_is_fail_closed",
    "move_group_result_future_unavailable": "test_move_group_result_future_none_is_fail_closed",
    "move_group_terminal_confirmed": "test_injected_move_group_accepted_result_timeout_is_correlated_and_fail_closed",
    "move_group_terminal_unconfirmed": "test_move_group_result_timeout_with_unconfirmed_terminal_stays_fail_closed",
    "success_after_cancel_race": "test_move_group_cancel_rejected_then_late_success_never_reaches_gate",
    "old_generation_isolation": "test_move_group_late_result_from_old_generation_cannot_dispatch_same_id_retry",
    "old_generation_late_success_isolation": "test_move_group_late_success_from_old_generation_cannot_dispatch_same_id_retry",
    "gate_goal_response_timeout": "test_injected_gate_goal_response_timeout_cancels_late_accepted_gate_goal",
    "late_gate_response": "test_injected_gate_goal_response_timeout_cancels_late_accepted_gate_goal",
    "gate_result_future_unavailable": "test_gate_result_future_none_is_fail_closed_and_cancels_exact_goal",
    "gate_result_exception": "test_gate_result_exception_is_fail_closed_and_cancels_exact_goal",
    "explicit_cancel": "test_injected_explicit_cancel_cannot_publish_old_result",
}
SCENARIO_ALIASES = {
    "move_group_goal_response_timeout": {
        "move_group_goal_response_timeout",
        "goal_response_timeout",
        "move_group_late_goal_response",
    },
    "late_move_group_send": {
        "late_move_group_send",
        "late_accepted_move_group_goal",
        "move_group_send_future_timeout",
    },
    "move_group_accepted_result_timeout": {
        "move_group_accepted_result_timeout",
        "accepted_goal_result_timeout",
        "accepted_goal_result_future_timeout",
    },
    "move_group_result_future_exception": {
        "move_group_result_future_exception",
        "result_future_request_exception",
        "move_group_result_request_exception",
    },
    "move_group_result_future_unavailable": {
        "move_group_result_future_unavailable",
        "move_group_result_future_none",
        "accepted_goal_has_no_result_future",
    },
    "move_group_terminal_confirmed": {
        "move_group_terminal_confirmed",
        "result_timeout_terminal_confirmed",
        "accepted_result_terminal_confirmed",
    },
    "move_group_terminal_unconfirmed": {
        "move_group_terminal_unconfirmed",
        "result_timeout_terminal_unconfirmed",
        "accepted_result_terminal_unconfirmed",
    },
    "success_after_cancel_race": {
        "success_after_cancel_race",
        "result_success_after_cancel",
        "move_group_success_after_cancel",
    },
    "old_generation_isolation": {
        "old_generation_isolation",
        "stale_generation_isolation",
        "late_result_old_generation",
    },
    "old_generation_late_success_isolation": {
        "old_generation_late_success_isolation",
        "late_success_old_generation_isolation",
        "old_generation_late_success",
    },
    "gate_goal_response_timeout": {
        "gate_goal_response_timeout",
        "trajectory_gate_goal_response_timeout",
        "gate_delayed_goal_response_timeout",
    },
    "late_gate_response": {
        "late_gate_response",
        "late_accepted_gate_goal",
        "late_gate_goal_response",
    },
    "gate_result_future_unavailable": {
        "gate_result_future_unavailable",
        "gate_result_future_none",
        "trajectory_gate_result_future_unavailable",
    },
    "gate_result_exception": {
        "gate_result_exception",
        "trajectory_gate_result_exception",
        "gate_result_future_result_exception",
    },
    "explicit_cancel": {
        "explicit_cancel",
        "explicit_cancel_old_result",
    },
}
REQUIRED_SCENARIOS = tuple(SCENARIO_ALIASES)
errors: list[str] = []


def sha256(path: Path) -> str | None:
    if not path.is_file():
        return None
    return hashlib.sha256(path.read_bytes()).hexdigest()


def line_count(path: Path) -> int | None:
    if not path.is_file():
        return None
    return path.read_bytes().count(b"\n")


def get(value: object, *paths: str) -> object:
    """Read the first present dotted path, preserving false and zero values."""
    missing = object()
    for dotted in paths:
        current: object = value
        for part in dotted.split("."):
            if not isinstance(current, dict) or part not in current:
                current = missing
                break
            current = current[part]
        if current is not missing:
            return current
    return None


def bool_at(value: object, *paths: str) -> bool | None:
    item = get(value, *paths)
    return item if isinstance(item, bool) else None


def int_at(value: object, *paths: str) -> int | None:
    item = get(value, *paths)
    return item if isinstance(item, int) and not isinstance(item, bool) else None


def text_at(value: object, *paths: str) -> str | None:
    item = get(value, *paths)
    return item.strip() if isinstance(item, str) and item.strip() else None


def canonical_scenario(value: object) -> str | None:
    if not isinstance(value, str):
        return None
    for canonical, aliases in SCENARIO_ALIASES.items():
        if value == canonical or value in aliases:
            return canonical
    return None


def record_test_name(record: dict[str, object]) -> str | None:
    value = text_at(record, "test_name", "test", "pytest_test", "pytest_nodeid")
    if value and "::" in value:
        value = value.rsplit("::", 1)[-1]
    return value


def flatten_records(record: dict[str, object]) -> list[dict[str, object]]:
    """Flatten a structured ledger, inheriting test/scope metadata."""
    children: list[dict[str, object]] = []
    for key in (
        "scenario_records",
        "scenarios",
        "ledger",
        "records",
        "entries",
    ):
        value = record.get(key)
        if not isinstance(value, list):
            continue
        for child in value:
            if not isinstance(child, dict):
                errors.append(f"JSONL {key} contains a non-object record")
                continue
            merged = dict(record)
            merged.pop(key, None)
            merged.update(child)
            children.extend(flatten_records(merged))
    return children or [record]


def parse_junit() -> tuple[dict[str, int], list[str], bool]:
    counts = {"tests": 0, "failures": 0, "errors": 0, "skipped": 0}
    names: list[str] = []
    if not junit_path.is_file():
        errors.append("junit.xml is missing")
        return counts, names, False
    try:
        root = ET.parse(junit_path).getroot()
    except (ET.ParseError, OSError) as exc:
        errors.append(f"junit.xml cannot be parsed: {exc}")
        return counts, names, False
    suites = [root] if root.tag == "testsuite" else list(root.iter("testsuite"))
    for suite in suites:
        for key in counts:
            try:
                counts[key] += int(suite.attrib.get(key, "0"))
            except ValueError:
                errors.append(f"JUnit invalid {key}={suite.attrib.get(key)!r}")
    for testcase in root.iter("testcase"):
        name = testcase.attrib.get("name")
        if not name:
            errors.append("JUnit testcase has no name")
            continue
        names.append(name)
        for tag in ("failure", "error", "skipped"):
            if testcase.find(tag) is not None:
                errors.append(f"JUnit {tag}: {name}")
    expected = {
        "tests": len(EXPECTED_TESTS),
        "failures": 0,
        "errors": 0,
        "skipped": 0,
    }
    ok = (
        counts == expected
        and len(names) == len(EXPECTED_TESTS)
        and len(set(names)) == len(EXPECTED_TESTS)
        and set(names) == EXPECTED_TESTS
    )
    if counts["tests"] != len(names):
        errors.append(f"JUnit tests={counts['tests']} != testcase count={len(names)}")
        ok = False
    if not ok:
        errors.append(
            "JUnit must contain exactly the requested injected tests; "
            f"observed={names!r}"
        )
    return counts, names, ok


def parse_jsonl() -> tuple[list[dict[str, object]], bool]:
    if not event_log.is_file():
        errors.append("scenarios.jsonl is missing")
        return [], False
    try:
        lines = event_log.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        errors.append(f"scenarios.jsonl cannot be read: {exc}")
        return [], False
    if not lines:
        errors.append("scenarios.jsonl is empty")
        return [], False
    records: list[dict[str, object]] = []
    for line_no, line in enumerate(lines, 1):
        if not line.strip():
            errors.append(f"scenarios.jsonl:{line_no}: blank line")
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError as exc:
            errors.append(f"scenarios.jsonl:{line_no}: invalid JSON: {exc}")
            continue
        if not isinstance(value, dict):
            errors.append(f"scenarios.jsonl:{line_no}: record is not an object")
            continue
        records.extend(flatten_records(value))
    if not records:
        errors.append("scenarios.jsonl contains no scenario records")
        return records, False
    return records, True


def iter_dicts(value: object):
    if isinstance(value, dict):
        yield value
        for child in value.values():
            yield from iter_dicts(child)
    elif isinstance(value, list):
        for child in value:
            yield from iter_dicts(child)


def event_items(record: dict[str, object]) -> list[dict[str, object]]:
    items: list[dict[str, object]] = []
    for key in (
        "events",
        "adapter_status_events",
        "status_events",
        "terminal",
        "actions",
        "attempts",
        "move_group_records",
        "gate_records",
        "move_group",
        "gate",
    ):
        value = record.get(key)
        if isinstance(value, (dict, list)):
            items.extend(iter_dicts(value))
    return items


def has_true(record: dict[str, object], *paths: str) -> bool:
    return any(bool_at(record, path) is True for path in paths)


def has_stage(record: dict[str, object], *stages: str) -> bool:
    wanted = {stage.lower() for stage in stages}
    return any(
        isinstance(item.get("stage"), str)
        and item["stage"].lower() in wanted
        for item in event_items(record)
    )


def status_text(record: dict[str, object]) -> str:
    return json.dumps(record, sort_keys=True, default=str).lower()


def action_entries(record: dict[str, object], kind: str) -> list[dict[str, object]]:
    entries: list[dict[str, object]] = []
    roots = (
        "terminal.move_group",
        "terminal.gate",
        "move_group_records",
        "gate_records",
        "actions.move_group",
        "actions.gate",
        "attempts.move_group",
        "attempts.gate",
        "move_group",
        "gate",
    )
    for root in roots:
        if kind not in root:
            continue
        value = get(record, root)
        entries.extend(iter_dicts(value))
    # Avoid duplicate summary projections caused by nested ``goals``.  A
    # strong action join requires the source-side ledger record itself: both
    # UUID endpoints, the request id, and the captured attempt generation.
    unique: list[dict[str, object]] = []
    seen: set[str] = set()
    for entry in entries:
        if not all(
            key in entry
            for key in (
                "server_goal_id",
                "client_goal_id",
                "request_id",
                "attempt_generation",
            )
        ):
            continue
        marker = json.dumps(entry, sort_keys=True, default=str)
        if marker not in seen:
            seen.add(marker)
            unique.append(entry)
    return unique


def is_uuid16(value: object) -> bool:
    return isinstance(value, str) and re.fullmatch(r"[0-9a-f]{32}", value) is not None


def uuid_from(entry: dict[str, object], *paths: str) -> object:
    return get(entry, *paths)


def status_records(
    record: dict[str, object],
    stages: tuple[str, ...],
    request_id: str,
    generation: int,
) -> list[dict[str, object]]:
    wanted = {stage.lower() for stage in stages}
    return [
        item
        for item in event_items(record)
        if isinstance(item.get("stage"), str)
        and item["stage"].lower() in wanted
        and item.get("request_id") == request_id
        and item.get("attempt_generation") == generation
    ]


def one_status(
    record: dict[str, object],
    label: str,
    stages: tuple[str, ...],
    request_id: str,
    generation: int,
) -> dict[str, object] | None:
    matches = status_records(record, stages, request_id, generation)
    if len(matches) != 1:
        errors.append(
            f"{record.get('scenario', '<unknown>')}: {label} expected one "
            f"status for request/generation, observed {len(matches)}"
        )
        return None
    return matches[0]


def pending_is(status: dict[str, object], key: str, value: object) -> bool:
    return status.get(key) is value


def action_triple(
    record: dict[str, object],
    kind: str,
    request_id: str,
    generation: int,
    goal_id: object,
    *,
    require_constraint: bool,
    expected_terminal_status_name: str | None = None,
) -> bool:
    """Join one action record to request/generation and both UUID endpoints."""
    candidates = []
    for entry in action_entries(record, kind):
        if entry.get("request_id") != request_id:
            continue
        if entry.get("attempt_generation") != generation:
            continue
        if require_constraint and entry.get("constraint_name") != f"edgegrasp:{request_id}":
            continue
        candidates.append(entry)
    if len(candidates) != 1:
        errors.append(
            f"{record.get('scenario', '<unknown>')}: {kind} action triple is "
            f"missing or ambiguous for request/generation (count={len(candidates)})"
        )
        return False
    entry = candidates[0]
    client_id = uuid_from(entry, "client_goal_id", f"{kind}_client_goal_id")
    server_id = uuid_from(entry, "server_goal_id", f"{kind}_server_goal_id")
    if not is_uuid16(client_id) or not is_uuid16(server_id) or client_id != server_id:
        errors.append(
            f"{record.get('scenario', '<unknown>')}: {kind} client/server UUID "
            "is not a matching canonical lowercase 16-byte value"
        )
        return False
    if not is_uuid16(goal_id) or goal_id != client_id or goal_id != server_id:
        errors.append(
            f"{record.get('scenario', '<unknown>')}: {kind} status UUID does not "
            "match the action triple"
        )
        return False
    if expected_terminal_status_name is not None:
        terminal_codes = {
            "SUCCEEDED": 4,
            "CANCELED": 5,
            "ABORTED": 6,
        }
        if (
            entry.get("terminal_status_name") != expected_terminal_status_name
            or entry.get("terminal_status")
            != terminal_codes[expected_terminal_status_name]
        ):
            errors.append(
                f"{record.get('scenario', '<unknown>')}: {kind} action "
                f"terminal is not {expected_terminal_status_name}"
            )
            return False
    return True


def move_group_chain(
    record: dict[str, object],
    *,
    terminal: str,
    cancel_accepted: bool,
) -> bool:
    """Check every MG status in one request/generation, not just list heads."""
    request_id = text_at(record, "request_id")
    generation = int_at(record, "attempt_generation")
    adapter_goal_id = get(record, "adapter_goal_id", "plan_target_client_goal_id")
    if request_id is None or generation is None or generation < 1 or not is_uuid16(adapter_goal_id):
        return False
    sent = one_status(
        record,
        "MoveGroup sent",
        ("move_group_goal_sent",),
        request_id,
        generation,
    )
    if sent is None:
        return False
    if sent.get("move_group_goal_id") is not None:
        errors.append("MoveGroup sent status must not contain a goal UUID")
        return False
    if sent.get("goal_response_future_pending_at_timeout") is not True:
        errors.append("MoveGroup sent status must mark goal-response pending")
        return False
    accepted = one_status(
        record,
        "MoveGroup accepted",
        ("move_group_goal_accepted",),
        request_id,
        generation,
    )
    if accepted is None or not is_uuid16(accepted.get("move_group_goal_id")):
        errors.append("MoveGroup accepted status lacks canonical goal UUID")
        return False
    if accepted.get("reason") != "result_pending":
        errors.append("MoveGroup accepted status must report result_pending")
        return False
    if (
        accepted.get("result_future_pending_at_timeout") is not None
        or accepted.get("goal_response_future_pending_at_timeout") is not None
    ):
        errors.append("MoveGroup accepted status must not report a timeout-pending future")
        return False
    goal_id = accepted["move_group_goal_id"]
    if not action_triple(
        record,
        "move_group",
        request_id,
        generation,
        goal_id,
        require_constraint=True,
        expected_terminal_status_name="CANCELED",
    ):
        return False
    timeout = one_status(
        record,
        "MoveGroup result timeout",
        ("move_group_result_timeout",),
        request_id,
        generation,
    )
    if timeout is None or timeout.get("move_group_goal_id") != goal_id:
        return False
    if timeout.get("result_future_pending_at_timeout") is not True:
        errors.append("MoveGroup result timeout must mark result pending")
        return False
    if timeout.get("goal_response_future_pending_at_timeout") is not None:
        errors.append("MoveGroup result timeout must not mark goal-response pending")
        return False
    cancelled = one_status(
        record,
        "MoveGroup cancel",
        ("cancelled_move_group",),
        request_id,
        generation,
    )
    if cancelled is None or cancelled.get("move_group_goal_id") != goal_id:
        return False
    if cancelled.get("move_group_cancel_requested") is not True:
        return False
    if cancelled.get("cancel_response_accepted") is not cancel_accepted:
        return False
    if terminal == "confirmed":
        terminal_status = one_status(
            record,
            "MoveGroup terminal observed",
            ("move_group_terminal_observed",),
            request_id,
            generation,
        )
        if terminal_status is None or terminal_status.get("move_group_goal_id") != goal_id:
            return False
        if terminal_status.get("move_group_terminal_observed") is not True:
            return False
        if terminal_status.get("move_group_terminal_status") not in (5, "CANCELED"):
            return False
        if terminal_status.get("action_goal_status") not in (5, "CANCELED"):
            return False
    else:
        terminal_status = one_status(
            record,
            "MoveGroup terminal unconfirmed",
            ("move_group_terminal_unconfirmed",),
            request_id,
            generation,
        )
        if terminal_status is None or terminal_status.get("move_group_goal_id") != goal_id:
            return False
        if terminal_status.get("move_group_terminal_observed") is not False:
            return False
    wrapper_status = int_at(record, "wrapper.status", "wrapper_status")
    wrapper_status_name = text_at(record, "wrapper.status_name")
    wrapper_reason = text_at(record, "wrapper.reason", "wrapper_reason")
    expected_reason = (
        "move_group_result_timeout"
        if terminal == "confirmed"
        else "move_group_result_timeout:move_group_terminal_unconfirmed"
    )
    return (
        wrapper_status == 6
        and wrapper_status_name == "ABORTED"
        and wrapper_reason == expected_reason
        and get(record, "wrapper.trajectory_dispatched", "trajectory_dispatched")
        is False
        and bool_at(record, "wrapper.gate_accepted") is False
        and bool_at(record, "wrapper.gate_terminal") is False
        and bool_at(record, "wrapper.downstream_terminal_observed") is False
        and count_at(record, "counters.gate_goals", "gate_goal_count") == 0
        and count_at(
            record,
            "counters.published_trajectories",
            "trajectory_publication_count",
        )
        == 0
    )


def validate_result_future_exception(record: dict[str, object]) -> bool:
    """Validate a result-future request exception without inventing terminal evidence."""
    request_id = text_at(record, "request_id")
    generation = int_at(record, "attempt_generation")
    adapter_goal_id = get(record, "adapter_goal_id", "plan_target_client_goal_id")
    if request_id is None or generation is None or generation < 1 or not is_uuid16(adapter_goal_id):
        return False
    accepted = one_status(
        record,
        "MoveGroup accepted before result-future exception",
        ("move_group_goal_accepted",),
        request_id,
        generation,
    )
    if accepted is None:
        return False
    goal_id = accepted.get("move_group_goal_id")
    if (
        not is_uuid16(goal_id)
        or accepted.get("reason") != "result_pending"
        or accepted.get("result_future_pending_at_timeout") is not None
        or accepted.get("goal_response_future_pending_at_timeout") is not None
        or not action_triple(
            record,
            "move_group",
            request_id,
            generation,
            goal_id,
            require_constraint=True,
            expected_terminal_status_name="CANCELED",
        )
    ):
        return False
    cancelled = one_status(
        record,
        "MoveGroup cancel after result-future exception",
        ("cancelled_move_group",),
        request_id,
        generation,
    )
    terminal = one_status(
        record,
        "MoveGroup terminal unconfirmed after result-future exception",
        ("move_group_terminal_unconfirmed",),
        request_id,
        generation,
    )
    if cancelled is None or terminal is None:
        return False
    wrapper_status = int_at(record, "wrapper.status", "wrapper_status")
    wrapper_status_name = text_at(record, "wrapper.status_name")
    wrapper_reason = text_at(record, "wrapper.reason", "wrapper_reason")
    if (
        wrapper_status != 6
        or wrapper_status_name != "ABORTED"
        or wrapper_reason != "move_group_result_request_exception"
        or get(record, "wrapper.trajectory_dispatched", "trajectory_dispatched") is not False
        or bool_at(record, "wrapper.gate_accepted") is not False
        or bool_at(record, "wrapper.gate_terminal") is not False
        or bool_at(record, "wrapper.downstream_terminal_observed") is not False
    ):
        return False
    if (
        cancelled.get("move_group_goal_id") != goal_id
        or cancelled.get("reason") != "result_request_exception"
        or cancelled.get("move_group_cancel_requested") is not True
        or cancelled.get("cancel_response_accepted") is not True
    ):
        return False
    if cancelled.get("result_future_pending_at_timeout") is not None:
        return False
    if (
        terminal.get("move_group_goal_id") != goal_id
        or terminal.get("reason") != "result_future_unavailable"
        or terminal.get("move_group_cancel_requested") is not True
        or terminal.get("move_group_terminal_observed") is not False
        or terminal.get("result_future_pending_at_timeout") is not None
        or terminal.get("move_group_terminal_status") is not None
        or terminal.get("action_goal_status") is not None
    ):
        return False
    if status_records(
        record,
        ("move_group_result_timeout",),
        request_id,
        generation,
    ):
        return False
    return (
        count_at(record, "counters.cancel_requests", "cancel.move_group_requests") is not None
        and count_at(record, "counters.cancel_requests", "cancel.move_group_requests") >= 1
        and count_at(record, "counters.gate_goals", "gate_goal_count") == 0
        and count_at(
            record,
            "counters.published_trajectories",
            "trajectory_publication_count",
        )
        == 0
    )


def validate_move_group_result_future_unavailable(record: dict[str, object]) -> bool:
    """Require an accepted MoveGroup goal with no result future to fail closed."""
    request_id = text_at(record, "request_id")
    generation = int_at(record, "attempt_generation")
    adapter_goal_id = get(record, "adapter_goal_id", "plan_target_client_goal_id")
    if request_id is None or generation is None or generation < 1 or not is_uuid16(adapter_goal_id):
        return False
    sent = one_status(
        record,
        "MoveGroup sent before missing result future",
        ("move_group_goal_sent",),
        request_id,
        generation,
    )
    accepted = one_status(
        record,
        "MoveGroup accepted before missing result future",
        ("move_group_goal_accepted",),
        request_id,
        generation,
    )
    unavailable = one_status(
        record,
        "MoveGroup result future unavailable",
        ("move_group_result_future_unavailable",),
        request_id,
        generation,
    )
    cancelled = one_status(
        record,
        "MoveGroup cancel after missing result future",
        ("cancelled_move_group",),
        request_id,
        generation,
    )
    terminal = one_status(
        record,
        "MoveGroup terminal unconfirmed after missing result future",
        ("move_group_terminal_unconfirmed",),
        request_id,
        generation,
    )
    if None in (sent, accepted, unavailable, cancelled, terminal):
        return False
    assert sent is not None and accepted is not None
    assert unavailable is not None and cancelled is not None and terminal is not None
    goal_id = accepted.get("move_group_goal_id")
    if (
        sent.get("move_group_goal_id") is not None
        or sent.get("goal_response_future_pending_at_timeout") is not True
        or accepted.get("reason") != "result_pending"
        or accepted.get("result_future_pending_at_timeout") is not None
        or accepted.get("goal_response_future_pending_at_timeout") is not None
        or not is_uuid16(goal_id)
        or not action_triple(
            record,
            "move_group",
            request_id,
            generation,
            goal_id,
            require_constraint=True,
            expected_terminal_status_name="CANCELED",
        )
    ):
        return False
    if (
        unavailable.get("move_group_goal_id") != goal_id
        or unavailable.get("reason") != "accepted_goal_has_no_result_future"
        or unavailable.get("result_future_pending_at_timeout") is not None
        or unavailable.get("goal_response_future_pending_at_timeout") is not None
        or unavailable.get("move_group_cancel_requested") is not True
        or unavailable.get("move_group_terminal_observed") is not False
        or unavailable.get("move_group_terminal_status") is not None
        or unavailable.get("action_goal_status") is not None
        or cancelled.get("move_group_goal_id") != goal_id
        or cancelled.get("reason") != "result_future_unavailable"
        or cancelled.get("result_future_pending_at_timeout") is not None
        or cancelled.get("move_group_cancel_requested") is not True
        or cancelled.get("cancel_response_accepted") is not True
        or terminal.get("move_group_goal_id") != goal_id
        or terminal.get("reason") != "result_future_unavailable"
        or terminal.get("result_future_pending_at_timeout") is not None
        or terminal.get("move_group_cancel_requested") is not True
        or terminal.get("move_group_terminal_observed") is not False
        or terminal.get("move_group_terminal_status") is not None
        or terminal.get("action_goal_status") is not None
    ):
        return False
    if status_records(record, ("move_group_result_timeout",), request_id, generation):
        return False
    return (
        int_at(record, "wrapper.status", "wrapper_status") == 6
        and text_at(record, "wrapper.status_name") == "ABORTED"
        and text_at(record, "wrapper.reason", "wrapper_reason")
        == "move_group_result_future_unavailable"
        and get(record, "wrapper.trajectory_dispatched", "trajectory_dispatched")
        is False
        and bool_at(record, "wrapper.gate_accepted") is False
        and bool_at(record, "wrapper.gate_terminal") is False
        and bool_at(record, "wrapper.downstream_terminal_observed") is False
        and count_at(record, "counters.cancel_requests", "cancel.move_group_requests")
        == 1
        and count_at(record, "counters.gate_goals", "gate_goal_count") == 0
        and count_at(
            record,
            "counters.published_trajectories",
            "trajectory_publication_count",
        )
        == 0
        and len(action_entries(record, "gate")) == 0
    )


def goal_response_chain(record: dict[str, object], kind: str) -> bool:
    """Validate sent/timeout/late-accept/cancel/terminal for one action."""
    request_id = text_at(record, "request_id")
    generation = int_at(record, "attempt_generation")
    adapter_goal_id = get(record, "adapter_goal_id", "plan_target_client_goal_id")
    if request_id is None or generation is None or generation < 1 or not is_uuid16(adapter_goal_id):
        return False
    if kind == "move_group":
        sent_stage = ("move_group_goal_sent",)
        timeout_stages = ("move_group_goal_response_timeout",)
        late_stage = ("late_move_group_goal_response",)
        cancel_stage = ("late_move_group_goal_cancel_response",)
        terminal_stage = ("late_move_group_goal_terminal",)
        pending_key = "goal_response_future_pending_at_timeout"
        goal_key = "move_group_goal_id"
    else:
        sent_stage = ("trajectory_gate_goal_sent",)
        timeout_stages = ("trajectory_gate_goal_response_timeout",)
        late_stage = ("late_gate_goal_response",)
        cancel_stage = ("late_gate_goal_cancel_response",)
        terminal_stage = ("late_gate_goal_terminal",)
        pending_key = "gate_goal_response_future_pending_at_timeout"
        goal_key = "gate_goal_id"
    sent = one_status(record, f"{kind} sent", sent_stage, request_id, generation)
    timeout = one_status(
        record, f"{kind} goal-response timeout", timeout_stages, request_id, generation
    )
    late = one_status(record, f"late {kind} response", late_stage, request_id, generation)
    cancel = one_status(record, f"late {kind} cancel response", cancel_stage, request_id, generation)
    terminal = one_status(record, f"late {kind} terminal", terminal_stage, request_id, generation)
    if None in (sent, timeout, late, cancel, terminal):
        return False
    assert sent is not None and timeout is not None and late is not None
    assert cancel is not None and terminal is not None
    if sent.get(goal_key) is not None or timeout.get(goal_key) is not None:
        return False
    if sent.get(pending_key) is not True or timeout.get(pending_key) is not True:
        return False
    for item in (sent, timeout):
        if item.get("result_future_pending_at_timeout") is not None:
            return False
    goal_id = late.get(goal_key)
    if not is_uuid16(goal_id) or late.get(pending_key) is not True:
        return False
    if late.get("result_future_pending_at_timeout") is not None:
        return False
    cancel_requested_key = (
        "move_group_cancel_requested" if kind == "move_group" else "gate_cancel_requested"
    )
    if late.get(cancel_requested_key) is not True:
        return False
    if cancel.get(goal_key) != goal_id or cancel.get("cancel_response_accepted") is not True:
        return False
    if terminal.get(goal_key) != goal_id:
        return False
    observed_key = (
        "move_group_terminal_observed" if kind == "move_group" else "gate_terminal_observed"
    )
    status_key = (
        "move_group_terminal_status" if kind == "move_group" else "gate_terminal_status"
    )
    wrapper_status = int_at(record, "wrapper.status", "wrapper_status")
    wrapper_status_name = text_at(record, "wrapper.status_name")
    wrapper_reason = text_at(record, "wrapper.reason", "wrapper_reason")
    wrapper_fail_closed = (
        wrapper_status == 6
        and wrapper_status_name == "ABORTED"
        and get(record, "wrapper.trajectory_dispatched", "trajectory_dispatched")
        is False
        and bool_at(record, "wrapper.gate_accepted") is False
        and bool_at(record, "wrapper.gate_terminal") is False
        and bool_at(record, "wrapper.downstream_terminal_observed") is False
    )
    if kind == "move_group":
        wrapper_fail_closed = bool(
            wrapper_fail_closed
            and wrapper_reason == "move_group_timeout"
            and count_at(record, "counters.gate_goals", "gate_goal_count") == 0
            and count_at(
                record,
                "counters.published_trajectories",
                "trajectory_publication_count",
            )
            == 0
        )
    else:
        # The fake gate records the one typed trajectory goal before delaying
        # its goal response.  This is injected action-server evidence only;
        # the PlanTarget wrapper must still report no admitted dispatch.
        wrapper_fail_closed = bool(
            wrapper_fail_closed
            and wrapper_reason == "trajectory_gate_goal_response_timeout"
            and count_at(record, "counters.gate_goals", "gate_goal_count") == 1
            and count_at(
                record,
                "counters.published_trajectories",
                "trajectory_publication_count",
            )
            == 1
        )
    return (
        terminal.get(observed_key) is True
        and terminal.get(status_key) in (5, "CANCELED")
        and terminal.get("action_goal_status") in (5, "CANCELED")
        and wrapper_fail_closed
        and action_triple(
            record,
            kind,
            request_id,
            generation,
            goal_id,
            require_constraint=kind == "move_group",
            expected_terminal_status_name="CANCELED",
        )
    )


def explicit_cancel_chain(record: dict[str, object]) -> bool:
    request_id = text_at(record, "request_id")
    generation = int_at(record, "attempt_generation")
    adapter_goal_id = get(record, "adapter_goal_id", "plan_target_client_goal_id")
    if request_id is None or generation is None or not is_uuid16(adapter_goal_id):
        return False
    sent = one_status(
        record,
        "MoveGroup sent",
        ("move_group_goal_sent",),
        request_id,
        generation,
    )
    accepted = one_status(
        record,
        "MoveGroup accepted",
        ("move_group_goal_accepted",),
        request_id,
        generation,
    )
    if sent is None or accepted is None:
        return False
    goal_id = accepted.get("move_group_goal_id")
    if not is_uuid16(goal_id) or not action_triple(
        record,
        "move_group",
        request_id,
        generation,
        goal_id,
        require_constraint=True,
        expected_terminal_status_name="CANCELED",
    ):
        return False
    cancelled = one_status(
        record,
        "explicit MoveGroup cancel",
        ("cancelled_move_group",),
        request_id,
        generation,
    )
    terminal = one_status(
        record,
        "explicit MoveGroup terminal",
        ("move_group_terminal_observed",),
        request_id,
        generation,
    )
    if cancelled is None or terminal is None:
        return False
    return (
        cancelled.get("reason") == "plan_target_cancel_requested"
        and cancelled.get("move_group_goal_id") == goal_id
        and cancelled.get("move_group_cancel_requested") is True
        and cancelled.get("cancel_response_accepted") is True
        and terminal.get("move_group_goal_id") == goal_id
        and terminal.get("move_group_terminal_observed") is True
        and terminal.get("move_group_terminal_status") in (5, "CANCELED")
        and terminal.get("action_goal_status") in (5, "CANCELED")
    )


def validate_gate_result_future_unavailable(record: dict[str, object]) -> bool:
    """Require exact gate cancellation when an accepted result future is absent."""
    request_id = text_at(record, "request_id")
    generation = int_at(record, "attempt_generation")
    adapter_goal_id = get(record, "adapter_goal_id", "plan_target_client_goal_id")
    if (
        request_id is None
        or generation is None
        or generation < 1
        or not is_uuid16(adapter_goal_id)
    ):
        return False
    accepted = one_status(
        record,
        "gate accepted before missing result future",
        ("gate_goal_accepted",),
        request_id,
        generation,
    )
    if accepted is None:
        return False
    goal_id = accepted.get("gate_goal_id")
    if (
        not is_uuid16(goal_id)
        or accepted.get("reason") != "result_pending"
        or accepted.get("result_future_pending_at_timeout") is not None
        or not action_triple(
            record,
            "gate",
            request_id,
            generation,
            goal_id,
            require_constraint=False,
            expected_terminal_status_name="CANCELED",
        )
    ):
        return False
    cancelled = one_status(
        record,
        "gate cancel after missing result future",
        ("cancelled_gate_command",),
        request_id,
        generation,
    )
    terminal = one_status(
        record,
        "gate terminal unconfirmed after missing result future",
        ("gate_terminal_unconfirmed",),
        request_id,
        generation,
    )
    if cancelled is None or terminal is None:
        return False
    if (
        cancelled.get("gate_goal_id") != goal_id
        or cancelled.get("reason") != "accepted"
        or cancelled.get("gate_cancel_requested") is not True
        or cancelled.get("cancel_response_accepted") is not True
        or terminal.get("gate_goal_id") != goal_id
        or terminal.get("reason") != "result_future_unavailable"
        or terminal.get("gate_cancel_requested") is not True
        or terminal.get("gate_terminal_observed") is not False
        or terminal.get("gate_terminal_status") is not None
        or terminal.get("action_goal_status") is not None
        or terminal.get("result_future_pending_at_timeout") is not None
    ):
        return False
    return (
        int_at(record, "wrapper.status", "wrapper_status") == 6
        and text_at(record, "wrapper.status_name") == "ABORTED"
        and text_at(record, "wrapper.reason", "wrapper_reason")
        == (
            "trajectory_gate_result_future_unavailable:"
            "gate_terminal_unconfirmed"
        )
        and get(record, "wrapper.trajectory_dispatched", "trajectory_dispatched")
        is True
        and bool_at(record, "wrapper.gate_accepted") is True
        and bool_at(record, "wrapper.gate_terminal") is False
        and bool_at(record, "wrapper.downstream_terminal_observed") is False
        and bool_at(record, "wrapper.cancel_requested") is True
        and count_at(record, "counters.gate_goals", "gate_goal_count") == 1
        and count_at(
            record,
            "counters.published_trajectories",
            "trajectory_publication_count",
        )
        == 1
        and count_at(
            record,
            "counters.gate_cancel_requests",
            "cancel.gate_requests",
        )
        == 1
    )


def validate_gate_result_exception(record: dict[str, object]) -> bool:
    """Require exact cancellation after an accepted gate result decode failure."""
    request_id = text_at(record, "request_id")
    generation = int_at(record, "attempt_generation")
    adapter_goal_id = get(record, "adapter_goal_id", "plan_target_client_goal_id")
    if (
        request_id is None
        or generation is None
        or generation < 1
        or not is_uuid16(adapter_goal_id)
    ):
        return False
    accepted = one_status(
        record,
        "gate accepted before result exception",
        ("gate_goal_accepted",),
        request_id,
        generation,
    )
    if accepted is None:
        return False
    goal_id = accepted.get("gate_goal_id")
    if (
        not is_uuid16(goal_id)
        or accepted.get("reason") != "result_pending"
        or not action_triple(
            record,
            "gate",
            request_id,
            generation,
            goal_id,
            require_constraint=False,
            expected_terminal_status_name="CANCELED",
        )
    ):
        return False
    cancelled = one_status(
        record,
        "gate cancel after result exception",
        ("cancelled_gate_command",),
        request_id,
        generation,
    )
    terminal = one_status(
        record,
        "gate terminal unconfirmed after result exception",
        ("gate_terminal_unconfirmed",),
        request_id,
        generation,
    )
    if cancelled is None or terminal is None:
        return False
    if (
        cancelled.get("gate_goal_id") != goal_id
        or cancelled.get("reason") != "accepted"
        or cancelled.get("gate_cancel_requested") is not True
        or cancelled.get("cancel_response_accepted") is not True
        or terminal.get("gate_goal_id") != goal_id
        or terminal.get("reason") != "result_exception:RuntimeError"
        or terminal.get("gate_cancel_requested") is not True
        or terminal.get("gate_terminal_observed") is not False
        or terminal.get("gate_terminal_status") is not None
        or terminal.get("action_goal_status") is not None
    ):
        return False
    return (
        int_at(record, "wrapper.status", "wrapper_status") == 6
        and text_at(record, "wrapper.status_name") == "ABORTED"
        and text_at(record, "wrapper.reason", "wrapper_reason")
        == "trajectory_gate_result_exception:gate_terminal_unconfirmed"
        and get(record, "wrapper.trajectory_dispatched", "trajectory_dispatched")
        is True
        and bool_at(record, "wrapper.gate_accepted") is True
        and bool_at(record, "wrapper.gate_terminal") is False
        and bool_at(record, "wrapper.downstream_terminal_observed") is False
        and bool_at(record, "wrapper.cancel_requested") is True
        and count_at(record, "counters.gate_goals", "gate_goal_count") == 1
        and count_at(
            record,
            "counters.published_trajectories",
            "trajectory_publication_count",
        )
        == 1
        and count_at(
            record,
            "counters.gate_cancel_requests",
            "cancel.gate_requests",
        )
        == 1
    )


def strong_request_join(record: dict[str, object]) -> bool:
    """Require a complete per-attempt status/action identity join."""
    request_id = text_at(record, "request_id")
    adapter_goal_id = get(record, "adapter_goal_id", "plan_target_client_goal_id")
    generation = int_at(record, "attempt_generation")
    if request_id is None or generation is None or not is_uuid16(adapter_goal_id):
        return False
    return move_group_chain(record, terminal="confirmed", cancel_accepted=True)


def scope_ok(record: dict[str, object]) -> bool:
    scope = get(record, "scope")
    if not isinstance(scope, dict):
        return False
    expected = {
        "injected_runtime_only": True,
        "in_process_fake": True,
        "real_move_group": False,
        "controller": False,
        "simulation_physics": False,
        "hardware": False,
    }
    return all(scope.get(key) is value for key, value in expected.items())


def feature_present(record: dict[str, object], scenario: str) -> bool:
    if canonical_scenario(get(record, "scenario")) == scenario:
        return True
    paths = {
        "move_group_goal_response_timeout": (
            "checks.move_group_goal_response_timed_out",
            "move_group_goal_response_timed_out",
        ),
        "late_move_group_send": (
            "checks.late_accepted_goal_cancelled",
            "late_accepted_goal_cancelled",
        ),
        "move_group_accepted_result_timeout": (
            "checks.accepted_goal_result_timeout",
            "checks.accepted_goal_result_future_timeout",
            "accepted_goal_result_timeout",
            "accepted_goal_result_future_timeout",
        ),
        "move_group_result_future_unavailable": (
            "checks.move_group_result_future_unavailable",
            "checks.move_group_result_future_none",
            "move_group_result_future_unavailable",
            "move_group_result_future_none",
        ),
        "move_group_terminal_confirmed": (
            "checks.move_group_terminal_confirmed",
            "checks.terminal_confirmed",
            "move_group_terminal_confirmed",
        ),
        "move_group_terminal_unconfirmed": (
            "checks.move_group_terminal_unconfirmed",
            "checks.terminal_unconfirmed",
            "move_group_terminal_unconfirmed",
        ),
        "success_after_cancel_race": (
            "checks.success_after_cancel_race",
            "success_after_cancel_race",
        ),
        "old_generation_isolation": (
            "checks.old_generation_isolation",
            "old_generation_isolation",
        ),
        "old_generation_late_success_isolation": (
            "checks.old_generation_late_success_isolation",
            "old_generation_late_success_isolation",
        ),
        "gate_goal_response_timeout": (
            "checks.gate_goal_response_timed_out",
            "gate_goal_response_timed_out",
        ),
        "late_gate_response": (
            "checks.late_accepted_gate_goal_cancelled",
            "late_accepted_gate_goal_cancelled",
        ),
        "explicit_cancel": (
            "checks.explicit_cancel_requested",
            "explicit_cancel_requested",
        ),
    }
    if has_true(record, *paths.get(scenario, ())):
        return True
    if scenario == "late_move_group_send" and has_stage(
        record, "late_move_group_goal_response"
    ):
        return True
    if scenario == "late_gate_response" and has_stage(
        record, "late_gate_goal_response"
    ):
        return True
    if scenario == "move_group_result_future_unavailable" and has_stage(
        record, "move_group_result_future_unavailable"
    ):
        return True
    return False


def count_at(record: dict[str, object], *paths: str) -> int | None:
    return int_at(record, *paths)


def validate_success_after_cancel(record: dict[str, object]) -> bool:
    """Require protocol SUCCEEDED after a rejected cancel, not fake payload text."""
    request_id = text_at(record, "request_id")
    generation = int_at(record, "attempt_generation")
    if request_id is None or generation is None:
        return False
    accepted = one_status(
        record,
        "MoveGroup accepted before success-after-cancel",
        ("move_group_goal_accepted",),
        request_id,
        generation,
    )
    timeout = one_status(
        record,
        "MoveGroup timeout before success-after-cancel",
        ("move_group_result_timeout",),
        request_id,
        generation,
    )
    cancelled = one_status(
        record,
        "MoveGroup rejected cancel before late success",
        ("cancelled_move_group",),
        request_id,
        generation,
    )
    terminal = one_status(
        record,
        "MoveGroup success observed during cancel terminal wait",
        ("move_group_terminal_observed",),
        request_id,
        generation,
    )
    if None in (accepted, timeout, cancelled, terminal):
        return False
    goal_id = accepted.get("move_group_goal_id")
    if not is_uuid16(goal_id):
        return False
    if any(
        item.get("move_group_goal_id") != goal_id
        for item in (timeout, cancelled, terminal)
    ):
        return False
    if (
        accepted.get("reason") != "result_pending"
        or timeout.get("reason") != "move_group_result_timeout"
        or timeout.get("result_future_pending_at_timeout") is not True
        or cancelled.get("reason") != "move_group_result_timeout"
        or cancelled.get("move_group_cancel_requested") is not True
        or cancelled.get("cancel_response_accepted") is not False
        or terminal.get("reason") != "success_after_cancel"
        or terminal.get("move_group_terminal_observed") is not True
        or terminal.get("move_group_terminal_status") not in (4, "SUCCEEDED")
        or terminal.get("action_goal_status") not in (4, "SUCCEEDED")
    ):
        return False
    if not action_triple(
        record,
        "move_group",
        request_id,
        generation,
        goal_id,
        require_constraint=True,
        expected_terminal_status_name="SUCCEEDED",
    ):
        return False
    entries = action_entries(record, "move_group")
    if not any(
        item.get("request_id") == request_id
        and item.get("attempt_generation") == generation
        and item.get("terminal_status_name") == "SUCCEEDED"
        and item.get("result_success_payload") is True
        for item in entries
    ):
        return False
    wrapper_status = int_at(record, "wrapper.status", "wrapper_status")
    wrapper_reason = text_at(record, "wrapper.reason", "wrapper_reason") or ""
    return (
        wrapper_status == 6
        and "move_group_result_timeout" in wrapper_reason
        and "move_group_success_after_cancel" in wrapper_reason
        and get(record, "wrapper.trajectory_dispatched", "trajectory_dispatched") is False
        and not any(
            item.get("request_id") == request_id
            and item.get("attempt_generation") == generation
            and item.get("terminal_status_name") == "CANCELED"
            and item.get("result_success_payload") is True
            for item in entries
        )
        and count_at(record, "counters.gate_goals", "gate_goal_count") == 0
        and count_at(record, "counters.published_trajectories", "trajectory_publication_count") == 0
    )


def validate_old_generation(record: dict[str, object]) -> bool:
    request_id = text_at(record, "request_id")
    adapter_goal_id = get(record, "adapter_goal_id", "plan_target_client_goal_id")
    if request_id is None or not is_uuid16(adapter_goal_id):
        return False
    accepted = [
        item
        for item in event_items(record)
        if item.get("stage") == "move_group_goal_accepted"
        and item.get("request_id") == request_id
        and isinstance(item.get("attempt_generation"), int)
    ]
    accepted_generations = [
        int(item["attempt_generation"]) for item in accepted
    ]
    if (
        len(accepted_generations) != 2
        or any(generation < 1 for generation in accepted_generations)
        or any(
            later <= earlier
            for earlier, later in zip(
                accepted_generations, accepted_generations[1:]
            )
        )
    ):
        return False
    goal_ids = {
        item.get("move_group_goal_id")
        for item in accepted
        if isinstance(item.get("move_group_goal_id"), str)
    }
    if len(goal_ids) < 2 or not all(is_uuid16(value) for value in goal_ids):
        return False
    if not (has_true(record, "checks.old_generation_isolation", "old_generation_isolation")
            or "late_result_ignored" in status_text(record)):
        return False
    oldest_generation = accepted_generations[0]
    newest_generation = accepted_generations[-1]
    if int_at(record, "attempt_generation") != newest_generation:
        return False
    for generation in accepted_generations:
        statuses = [
            item
            for item in accepted
            if item.get("attempt_generation") == generation
        ]
        entries = [
            item
            for item in action_entries(record, "move_group")
            if item.get("request_id") == request_id
            and item.get("attempt_generation") == generation
            and item.get("constraint_name") == f"edgegrasp:{request_id}"
        ]
        if len(statuses) != 1 or len(entries) != 1:
            return False
        entry = entries[0]
        status_goal_id = statuses[0].get("move_group_goal_id")
        if not (
            is_uuid16(status_goal_id)
            and is_uuid16(entry.get("client_goal_id"))
            and is_uuid16(entry.get("server_goal_id"))
            and entry.get("client_goal_id") == entry.get("server_goal_id")
            and entry.get("client_goal_id") == status_goal_id
        ):
            return False
        expected_terminal = (
            "CANCELED" if generation == oldest_generation else "SUCCEEDED"
        )
        expected_terminal_code = 5 if generation == oldest_generation else 4
        if (
            entry.get("terminal_status_name") != expected_terminal
            or entry.get("terminal_status") != expected_terminal_code
        ):
            return False
        terminal_stage = (
            "late_move_group_goal_terminal"
            if generation == oldest_generation
            else "move_group_terminal"
        )
        terminal_status = one_status(
            record,
            f"generation {generation} MoveGroup terminal",
            (terminal_stage,),
            request_id,
            generation,
        )
        if (
            terminal_status is None
            or terminal_status.get("move_group_goal_id") != status_goal_id
            or terminal_status.get("move_group_terminal_observed") is not True
            or terminal_status.get("move_group_terminal_status")
            != expected_terminal_code
            or terminal_status.get("action_goal_status")
            != expected_terminal_code
        ):
            return False
    all_gate_entries = action_entries(record, "gate")
    if len(all_gate_entries) != 1:
        return False
    gate_entries = [
        item
        for item in all_gate_entries
        if item.get("request_id") == request_id
        and item.get("attempt_generation") == newest_generation
    ]
    if len(gate_entries) != 1:
        return False
    gate_entry = gate_entries[0]
    gate_client_id = gate_entry.get("client_goal_id")
    gate_server_id = gate_entry.get("server_goal_id")
    if not (
        is_uuid16(gate_client_id)
        and gate_client_id == gate_server_id
        and gate_entry.get("terminal_status") == 4
        and gate_entry.get("terminal_status_name") == "SUCCEEDED"
    ):
        return False
    gate_accepted = one_status(
        record,
        "new-generation gate accepted",
        ("gate_goal_accepted",),
        request_id,
        newest_generation,
    )
    gate_terminals = [
        item
        for item in status_records(
            record,
            ("gate_terminal",),
            request_id,
            newest_generation,
        )
        if item.get("gate_goal_id") == gate_client_id
    ]
    gate_terminal = gate_terminals[0] if len(gate_terminals) == 1 else None
    if (
        gate_accepted is None
        or gate_terminal is None
        or gate_accepted.get("gate_goal_id") != gate_client_id
        or gate_terminal.get("gate_goal_id") != gate_client_id
        or gate_terminal.get("gate_terminal_observed") is not True
        or gate_terminal.get("gate_terminal_status") not in (4, "SUCCEEDED")
    ):
        return False
    gate_count = count_at(record, "counters.gate_goals", "gate_goal_count")
    trajectory_count = count_at(record, "counters.published_trajectories", "trajectory_publication_count")
    # The retry may legitimately dispatch one gate goal; the stale generation
    # must not dispatch an additional one. The ledger must state that mapping.
    return (
        gate_count == 1
        and trajectory_count == 1
        and int_at(record, "wrapper.status", "wrapper_status") == 4
        and text_at(record, "wrapper.status_name") == "SUCCEEDED"
        and text_at(record, "wrapper.reason", "wrapper_reason")
        == "trajectory_executed_through_gate"
        and get(record, "wrapper.trajectory_dispatched", "trajectory_dispatched")
        is True
        and bool_at(record, "wrapper.gate_accepted") is True
        and bool_at(record, "wrapper.gate_terminal") is True
        and bool_at(record, "wrapper.downstream_terminal_observed") is True
    )


def validate_old_generation_late_success(record: dict[str, object]) -> bool:
    """Require a late old-generation success to stay isolated from one retry."""
    request_id = text_at(record, "request_id")
    adapter_goal_id = get(record, "adapter_goal_id", "plan_target_client_goal_id")
    newest_generation = int_at(record, "attempt_generation")
    if (
        request_id is None
        or not is_uuid16(adapter_goal_id)
        or newest_generation is None
        or newest_generation < 1
        or not has_true(
            record,
            "checks.old_generation_late_success_isolation",
            "old_generation_late_success_isolation",
        )
    ):
        return False
    accepted = [
        item
        for item in event_items(record)
        if item.get("stage") == "move_group_goal_accepted"
        and item.get("request_id") == request_id
        and isinstance(item.get("attempt_generation"), int)
    ]
    generations = [int(item["attempt_generation"]) for item in accepted]
    if (
        len(generations) != 2
        or any(generation < 1 for generation in generations)
        or any(
            later <= earlier for earlier, later in zip(generations, generations[1:])
        )
        or generations[-1] != newest_generation
        or len(set(generations)) != 2
    ):
        return False
    oldest_generation = generations[0]
    move_group_entries = action_entries(record, "move_group")
    if len(move_group_entries) != 2:
        return False

    goal_ids: dict[int, str] = {}
    for generation, accepted_status in zip(generations, accepted):
        sent = one_status(
            record,
            f"generation {generation} MoveGroup sent",
            ("move_group_goal_sent",),
            request_id,
            generation,
        )
        accepted_again = one_status(
            record,
            f"generation {generation} MoveGroup accepted",
            ("move_group_goal_accepted",),
            request_id,
            generation,
        )
        goal_id = accepted_status.get("move_group_goal_id")
        if (
            sent is None
            or accepted_again is None
            or accepted_again != accepted_status
            or sent.get("move_group_goal_id") is not None
            or sent.get("goal_response_future_pending_at_timeout") is not True
            or accepted_again.get("reason") != "result_pending"
            or accepted_again.get("result_future_pending_at_timeout") is not None
            or accepted_again.get("goal_response_future_pending_at_timeout") is not None
            or not is_uuid16(goal_id)
            or not action_triple(
                record,
                "move_group",
                request_id,
                generation,
                goal_id,
                require_constraint=True,
                expected_terminal_status_name="SUCCEEDED",
            )
        ):
            return False
        goal_ids[generation] = goal_id

    if len(set(goal_ids.values())) != 2:
        return False
    old_goal_id = goal_ids[oldest_generation]
    new_goal_id = goal_ids[newest_generation]
    old_entry = next(
        (
            item
            for item in move_group_entries
            if item.get("request_id") == request_id
            and item.get("attempt_generation") == oldest_generation
        ),
        None,
    )
    new_entry = next(
        (
            item
            for item in move_group_entries
            if item.get("request_id") == request_id
            and item.get("attempt_generation") == newest_generation
        ),
        None,
    )
    if (
        old_entry is None
        or new_entry is None
        or old_entry.get("result_success_payload") is not True
        or old_entry.get("cancel_api_requested") is not True
        or new_entry.get("result_success_payload") is not True
    ):
        return False

    old_timeout = one_status(
        record,
        "old-generation MoveGroup result timeout",
        ("move_group_result_timeout",),
        request_id,
        oldest_generation,
    )
    old_cancel = one_status(
        record,
        "old-generation MoveGroup cancel",
        ("cancelled_move_group",),
        request_id,
        oldest_generation,
    )
    old_unconfirmed = one_status(
        record,
        "old-generation MoveGroup terminal unconfirmed",
        ("move_group_terminal_unconfirmed",),
        request_id,
        oldest_generation,
    )
    old_late_terminal = one_status(
        record,
        "old-generation late MoveGroup success terminal",
        ("late_move_group_goal_terminal",),
        request_id,
        oldest_generation,
    )
    new_terminal = one_status(
        record,
        "new-generation MoveGroup terminal",
        ("move_group_terminal",),
        request_id,
        newest_generation,
    )
    if None in (old_timeout, old_cancel, old_unconfirmed, old_late_terminal, new_terminal):
        return False
    assert old_timeout is not None
    assert old_cancel is not None
    assert old_unconfirmed is not None
    assert old_late_terminal is not None
    assert new_terminal is not None
    if (
        old_timeout.get("move_group_goal_id") != old_goal_id
        or old_timeout.get("reason") != "move_group_result_timeout"
        or old_timeout.get("result_future_pending_at_timeout") is not True
        or old_timeout.get("goal_response_future_pending_at_timeout") is not None
        or old_cancel.get("move_group_goal_id") != old_goal_id
        or old_cancel.get("reason") != "move_group_result_timeout"
        or old_cancel.get("result_future_pending_at_timeout") is not True
        or old_cancel.get("move_group_cancel_requested") is not True
        or old_cancel.get("cancel_response_accepted") is not False
        or old_unconfirmed.get("move_group_goal_id") != old_goal_id
        or old_unconfirmed.get("reason") != "result_future_timeout"
        or old_unconfirmed.get("result_future_pending_at_timeout") is not True
        or old_unconfirmed.get("move_group_cancel_requested") is not True
        or old_unconfirmed.get("move_group_terminal_observed") is not False
        or old_unconfirmed.get("move_group_terminal_status") is not None
        or old_unconfirmed.get("action_goal_status") is not None
        or old_late_terminal.get("move_group_goal_id") != old_goal_id
        or old_late_terminal.get("reason") != "observed"
        or old_late_terminal.get("move_group_terminal_observed") is not True
        or old_late_terminal.get("move_group_terminal_status") != 4
        or old_late_terminal.get("action_goal_status") != 4
        or new_terminal.get("move_group_goal_id") != new_goal_id
        or new_terminal.get("reason") != "observed"
        or new_terminal.get("move_group_terminal_observed") is not True
        or new_terminal.get("move_group_terminal_status") != 4
        or new_terminal.get("action_goal_status") != 4
    ):
        return False
    if any(
        status_records(record, (stage,), request_id, newest_generation)
        for stage in (
            "move_group_result_timeout",
            "cancelled_move_group",
            "move_group_terminal_unconfirmed",
            "late_move_group_goal_terminal",
        )
    ):
        return False

    gate_entries = action_entries(record, "gate")
    if len(gate_entries) != 1:
        return False
    gate_accepted = one_status(
        record,
        "new-generation gate accepted after old success",
        ("gate_goal_accepted",),
        request_id,
        newest_generation,
    )
    gate_terminal = one_status(
        record,
        "new-generation gate terminal after old success",
        ("gate_terminal",),
        request_id,
        newest_generation,
    )
    if gate_accepted is None or gate_terminal is None:
        return False
    gate_goal_id = gate_accepted.get("gate_goal_id")
    if (
        not is_uuid16(gate_goal_id)
        or gate_terminal.get("gate_goal_id") != gate_goal_id
        or gate_accepted.get("reason") != "result_pending"
        or gate_terminal.get("reason") != "observed"
        or gate_terminal.get("gate_terminal_observed") is not True
        or gate_terminal.get("gate_terminal_status") != 4
        or gate_terminal.get("action_goal_status") is not None
        or not action_triple(
            record,
            "gate",
            request_id,
            newest_generation,
            gate_goal_id,
            require_constraint=False,
            expected_terminal_status_name="SUCCEEDED",
        )
        or any(
            status_records(record, (stage,), request_id, oldest_generation)
            for stage in ("gate_goal_accepted", "gate_terminal")
        )
    ):
        return False
    return (
        count_at(record, "counters.cancel_requests", "cancel.move_group_requests")
        == 1
        and count_at(record, "counters.gate_goals", "gate_goal_count") == 1
        and count_at(record, "counters.published_trajectories", "trajectory_publication_count")
        == 1
        and int_at(record, "wrapper.status", "wrapper_status") == 4
        and text_at(record, "wrapper.status_name") == "SUCCEEDED"
        and text_at(record, "wrapper.reason", "wrapper_reason")
        == "trajectory_executed_through_gate"
        and get(record, "wrapper.trajectory_dispatched", "trajectory_dispatched")
        is True
        and bool_at(record, "wrapper.gate_accepted") is True
        and bool_at(record, "wrapper.gate_terminal") is True
        and bool_at(record, "wrapper.downstream_terminal_observed") is True
    )


def validate_scenario(record: dict[str, object], scenario: str) -> bool:
    # Required runtime scenarios use the status chain above.  Every status
    # must carry the same request id and monotonic generation; every accepted
    # action must also carry its canonical client/server UUID join.
    if scenario in {"move_group_goal_response_timeout", "late_move_group_send"}:
        good = scope_ok(record) and goal_response_chain(record, "move_group")
        if good:
            entries = action_entries(record, "move_group")
            request_id = text_at(record, "request_id")
            generation = int_at(record, "attempt_generation")
            late = one_status(
                record,
                "late MoveGroup response",
                ("late_move_group_goal_response",),
                request_id or "",
                generation or 0,
            )
            goal_id = late.get("move_group_goal_id") if late else None
            good = bool(
                request_id
                and generation
                and len(entries) == 1
                and action_triple(
                    record,
                    "move_group",
                    request_id,
                    generation,
                    goal_id,
                    require_constraint=True,
                    expected_terminal_status_name="CANCELED",
                )
            )
        if not good:
            errors.append(f"{scenario}: required per-status/per-action ledger facts are incomplete")
        return good
    if scenario == "move_group_accepted_result_timeout":
        good = scope_ok(record) and move_group_chain(
            record, terminal="confirmed", cancel_accepted=True
        )
        if not good:
            errors.append(f"{scenario}: required per-status/per-action ledger facts are incomplete")
        return good
    if scenario == "move_group_result_future_exception":
        good = scope_ok(record) and validate_result_future_exception(record)
        if not good:
            errors.append(f"{scenario}: required result-future-exception ledger facts are incomplete")
        return good
    if scenario == "move_group_result_future_unavailable":
        good = scope_ok(record) and validate_move_group_result_future_unavailable(record)
        if not good:
            errors.append(
                f"{scenario}: required missing MoveGroup result-future ledger facts are incomplete"
            )
        return good
    if scenario == "move_group_terminal_confirmed":
        good = scope_ok(record) and move_group_chain(
            record, terminal="confirmed", cancel_accepted=True
        )
        if not good:
            errors.append(f"{scenario}: required terminal-confirmed ledger facts are incomplete")
        return good
    if scenario == "move_group_terminal_unconfirmed":
        good = scope_ok(record) and move_group_chain(
            record, terminal="unconfirmed", cancel_accepted=True
        )
        if not good:
            errors.append(f"{scenario}: required terminal-unconfirmed ledger facts are incomplete")
        return good
    if scenario == "success_after_cancel_race":
        good = scope_ok(record) and validate_success_after_cancel(record)
        if not good:
            errors.append(f"{scenario}: required success-after-cancel ledger facts are incomplete")
        return good
    if scenario == "old_generation_isolation":
        good = scope_ok(record) and validate_old_generation(record)
        if not good:
            errors.append(f"{scenario}: required old-generation ledger facts are incomplete")
        return good
    if scenario == "old_generation_late_success_isolation":
        good = scope_ok(record) and validate_old_generation_late_success(record)
        if not good:
            errors.append(
                f"{scenario}: required old-generation late-success ledger facts are incomplete"
            )
        return good
    if scenario in {"gate_goal_response_timeout", "late_gate_response"}:
        good = scope_ok(record) and goal_response_chain(record, "gate")
        if not good:
            errors.append(f"{scenario}: required gate per-status/per-action ledger facts are incomplete")
        return good
    if scenario == "gate_result_future_unavailable":
        good = scope_ok(record) and validate_gate_result_future_unavailable(record)
        if not good:
            errors.append(
                f"{scenario}: required missing gate-result-future ledger facts are incomplete"
            )
        return good
    if scenario == "gate_result_exception":
        good = scope_ok(record) and validate_gate_result_exception(record)
        if not good:
            errors.append(
                f"{scenario}: required gate-result-exception ledger facts are incomplete"
            )
        return good
    if scenario == "explicit_cancel":
        good = scope_ok(record)
        request_id = text_at(record, "request_id")
        generation = int_at(record, "attempt_generation")
        if request_id is None or generation is None:
            good = False
        if good:
            good = explicit_cancel_chain(record)
        good &= (
            has_true(record, "checks.explicit_cancel_requested", "explicit_cancel_requested")
            or "plan_target_cancel_requested" in status_text(record)
        )
        good &= (
            count_at(
                record,
                "counters.cancel_requests",
                "cancel.move_group_requests",
                "cancel_requests",
            )
            is not None
            and count_at(
                record,
                "counters.cancel_requests",
                "cancel.move_group_requests",
                "cancel_requests",
            )
            >= 1
        )
        good &= count_at(record, "counters.gate_goals", "gate_goal_count") == 0
        good &= count_at(
            record,
            "counters.published_trajectories",
            "trajectory_publication_count",
        ) == 0
        if not good:
            errors.append(f"{scenario}: required explicit-cancel ledger facts are incomplete")
        return good

    errors.append(f"{scenario}: unknown required scenario validator")
    return False


junit_counts, junit_names, junit_ok = parse_junit()
records, jsonl_ok = parse_jsonl()

observed_scenarios: set[str] = set()
records_by_test: dict[str, list[dict[str, object]]] = {}
for index, record in enumerate(records, 1):
    if record.get("schema_version") != 2:
        errors.append(f"JSONL record {index}: schema_version must be 2")
    scenario = canonical_scenario(get(record, "scenario"))
    if scenario is not None:
        observed_scenarios.add(scenario)
    test_name = record_test_name(record)
    if test_name is None and scenario in TEST_BY_BASE_SCENARIO:
        test_name = TEST_BY_BASE_SCENARIO[scenario]
    if test_name not in EXPECTED_TESTS:
        errors.append(f"JSONL record {index}: cannot map to one requested JUnit test")
    else:
        records_by_test.setdefault(test_name, []).append(record)
    if not scope_ok(record):
        errors.append(f"JSONL record {index}: scope is missing or outside injected fake")

scenario_results: dict[str, bool] = {}
for required in REQUIRED_SCENARIOS:
    candidates = [record for record in records if feature_present(record, required)]
    if not candidates:
        errors.append(f"required scenario is missing: {required}")
        scenario_results[required] = False
    else:
        scenario_results[required] = any(
            validate_scenario(record, required) for record in candidates
        )
        if scenario_results[required]:
            observed_scenarios.add(required)
for test_name in sorted(EXPECTED_TESTS):
    if not records_by_test.get(test_name):
        errors.append(f"JSONL has no record mapped to JUnit test: {test_name}")

result_records = [
    record
    for record in records
    if feature_present(record, "move_group_accepted_result_timeout")
]
strong_correlation = any(strong_request_join(record) for record in result_records)
accepted_result_timeout = bool(
    result_records and scenario_results.get("move_group_accepted_result_timeout", False)
)
gate_timeout = bool(
    scenario_results.get("gate_goal_response_timeout", False)
    and scenario_results.get("late_gate_response", False)
)
gate_result_future_unavailable = bool(
    scenario_results.get("gate_result_future_unavailable", False)
)
gate_result_exception = bool(
    scenario_results.get("gate_result_exception", False)
)
move_group_result_future_unavailable = bool(
    scenario_results.get("move_group_result_future_unavailable", False)
)
old_generation_late_success_isolation = bool(
    scenario_results.get("old_generation_late_success_isolation", False)
)

source = {
    name: {
        "absolute_path": str(path),
        "sha256": sha256(path),
        "line_count": line_count(path),
    }
    for name, path in source_paths.items()
}
source_ok = all(item["sha256"] is not None for item in source.values())
if not source_ok:
    errors.append("source hash collection is incomplete")

artifact_paths = {
    "junit.xml": junit_path,
    "pytest.log": pytest_log,
    "scenarios.jsonl": event_log,
    "start.txt": artifact / "start.txt",
    "end.txt": artifact / "end.txt",
}
artifact_hashes = {
    name: {"absolute_path": str(path), "sha256": sha256(path)}
    for name, path in artifact_paths.items()
}

overall_pass = (
    scenario_rc == 0
    and junit_ok
    and jsonl_ok
    and len(records_by_test) == len(EXPECTED_TESTS)
    and all(scenario_results.values())
    and accepted_result_timeout
    and strong_correlation
    and gate_timeout
    and move_group_result_future_unavailable
    and old_generation_late_success_isolation
    and source_ok
    and not errors
)

summary = {
    "schema_version": 2,
    "evidence_class": "INJECTED_CANCEL_TIMEOUT",
    "status": "PASS" if overall_pass else "FAIL",
    "overall_pass": overall_pass,
    "scenario_exit_code": scenario_rc,
    "started_at": started_at,
    "ended_at": ended_at,
    "runtime": {
        "mode": "in_process_fake",
        "in_process_fake": True,
        "injected_runtime_only": True,
        "fake_move_group_action_server": True,
        "real_move_group": False,
        "controller": False,
        "simulation_physics": False,
        "hardware": False,
        "remote_host_or_network_target": False,
        "os_process_signal_injection": False,
    },
    "scope": {
        "injected_runtime_only": True,
        "in_process_fake": True,
        "real_move_group": False,
        "controller": False,
        "simulation_physics": False,
        "hardware": False,
    },
    "source": source,
    "source_hashes": {name: value["sha256"] for name, value in source.items()},
    "artifact_hashes": artifact_hashes,
    "junit": {**junit_counts, "testcases": junit_names},
    "pytest": {
        **junit_counts,
        "junit_sha256": sha256(junit_path),
        "log_sha256": sha256(pytest_log),
        "event_log_sha256": sha256(event_log),
    },
    "scenario_count": len(observed_scenarios),
    "scenarios": sorted(observed_scenarios),
    "scenario_records": records,
    "required_scenarios": {
        name: {
            "present": any(feature_present(record, name) for record in records),
            "verified": scenario_results.get(name, False),
        }
        for name in REQUIRED_SCENARIOS
    },
    "verified": {
        # Unqualified fields intentionally remain false: this runner cannot
        # upgrade historical/real MoveGroup claims. The scoped fields below
        # are the only positive claims made by this artifact.
        "accepted_goal_result_timeout_verified": False,
        "accepted_goal_result_future_timeout_verified": False,
        "strong_move_group_request_id_correlation": False,
        "move_group_result_future_unavailable_verified": False,
        "old_generation_late_success_isolation_verified": False,
        "accepted_goal_result_timeout_verified_injected": accepted_result_timeout,
        "accepted_goal_result_future_timeout_verified_injected": accepted_result_timeout,
        "strong_move_group_request_id_correlation_verified_injected": strong_correlation,
        "move_group_result_future_unavailable_verified_injected": move_group_result_future_unavailable,
        "old_generation_late_success_isolation_verified_injected": old_generation_late_success_isolation,
        "gate_delayed_goal_response_cancel_verified_injected": gate_timeout,
        "gate_result_future_unavailable_verified_injected": gate_result_future_unavailable,
        "gate_result_exception_verified_injected": gate_result_exception,
        "accepted_goal_result_timeout_verified_real": False,
        "strong_move_group_request_id_correlation_verified_real": False,
        "move_group_result_future_unavailable_verified_real": False,
        "old_generation_late_success_isolation_verified_real": False,
        "gate_delayed_goal_response_cancel_verified_real": False,
    },
    "claim_boundary": {
        "injected_runtime_only": True,
        "in_process_fake": True,
        "real_move_group": False,
        "controller": False,
        "simulation_physics": False,
        "hardware": False,
        "accepted_goal_result_timeout_verified": False,
        "accepted_goal_result_future_timeout_verified": False,
        "strong_move_group_request_id_correlation": False,
        "move_group_result_future_unavailable_verified": False,
        "old_generation_late_success_isolation_verified": False,
        "accepted_goal_result_timeout_verified_injected": accepted_result_timeout,
        "accepted_goal_result_future_timeout_verified_injected": accepted_result_timeout,
        "strong_move_group_request_id_correlation_verified_injected": strong_correlation,
        "move_group_result_future_unavailable_verified_injected": move_group_result_future_unavailable,
        "old_generation_late_success_isolation_verified_injected": old_generation_late_success_isolation,
        "gate_delayed_goal_response_cancel_verified_injected": gate_timeout,
        "gate_result_future_unavailable_verified_injected": gate_result_future_unavailable,
        "gate_result_exception_verified_injected": gate_result_exception,
        "accepted_goal_result_timeout_verified_real": False,
        "strong_move_group_request_id_correlation_verified_real": False,
        "move_group_result_future_unavailable_verified_real": False,
        "old_generation_late_success_isolation_verified_real": False,
        "gate_delayed_goal_response_cancel_verified_real": False,
        "interpretation": (
            "Positive claims are limited to the in-process fake dependency ledger. "
            "This artifact does not establish real MoveGroup health, controller "
            "behavior, simulation physics, or hardware evidence."
        ),
    },
    "validation_errors": errors,
}
(artifact / "summary.json").write_text(
    json.dumps(summary, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
)
print(json.dumps(summary, indent=2, sort_keys=True))
sys.exit(0 if overall_pass else 1)
PY
validation_rc=$?
set -e

if [ "$scenario_rc" -ne 0 ] || [ "$validation_rc" -ne 0 ]; then
  exit 1
fi
exit 0
