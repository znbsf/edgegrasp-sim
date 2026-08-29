from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OBSERVATION = (
    ROOT
    / "docs"
    / "observations"
    / "2026-08-27-candidate007-contact-timing-runtime.json"
)
CANDIDATE = (
    ROOT
    / "ros_ws"
    / "src"
    / "edgegrasp_ros"
    / "config"
    / "so101_side_grasp_candidate.json"
)


def test_candidate007_records_transient_contact_without_physics_promotion() -> None:
    evidence = json.loads(OBSERVATION.read_text(encoding="utf-8"))
    run = evidence["successful_observation"]
    sequence = run["sequence_result"]
    physics = run["physics_result"]
    simultaneous = physics["simultaneous_contact_timing"]

    assert sequence["sequence_completed"] is True
    assert physics["physics_grasp_verified"] is False
    assert physics["lift_observed"] is False
    assert physics["retention_observed"] is False
    assert physics["peak_lift_m"] < physics["required_lift_m"]
    assert physics["simultaneous_gripper_contact_sample_count"] == 39
    assert simultaneous["sample_count_at_sequence_completion"] == 39
    assert simultaneous["last_source_timestamp_ns_at_sequence_completion"] == (
        simultaneous["last_source_timestamp_ns"]
    )
    assert simultaneous["last_source_timestamp_ns"] < sequence["source_timestamp_ns"]
    assert simultaneous["ended_before_sequence_completion_ns"] == (
        sequence["source_timestamp_ns"] - simultaneous["last_source_timestamp_ns"]
    )
    assert simultaneous["ended_before_sequence_completion_ns"] == 1_747_000_000
    assert abs(physics["gripper_effort"]["at_sequence_completion"]) < 0.001
    assert run["runtime_health"]["cleanup_exact_domain_processes_remaining"] == 0
    assert run["runtime_health"]["clock_topic_after_cleanup"] == "unknown"


def test_candidate_profile_references_the_same_candidate007_measurements() -> None:
    evidence = json.loads(OBSERVATION.read_text(encoding="utf-8"))
    candidate = json.loads(CANDIDATE.read_text(encoding="utf-8"))
    run = evidence["successful_observation"]
    observed = run["physics_result"]
    recorded = candidate["contact_timing_runtime_observation"]

    assert candidate["schema_version"] == 14
    assert recorded["path"] == OBSERVATION.relative_to(ROOT).as_posix()
    assert recorded["task_id"] == run["task_id"]
    assert recorded["evidence_digest"] == observed["evidence_digest"]
    assert recorded["sequence_completed_source_timestamp_ns"] == run[
        "sequence_result"
    ]["source_timestamp_ns"]
    assert recorded["simultaneous_gripper_contact_sample_count"] == observed[
        "simultaneous_gripper_contact_sample_count"
    ]
    assert recorded["simultaneous_contact_last_source_timestamp_ns"] == observed[
        "simultaneous_contact_timing"
    ]["last_source_timestamp_ns"]
    assert recorded["peak_lift_m"] == observed["peak_lift_m"]


def test_candidate007_preflight_failures_are_not_motion_trials() -> None:
    evidence = json.loads(OBSERVATION.read_text(encoding="utf-8"))
    permission_abort, harness_abort = evidence["preflight_attempts"]

    assert permission_abort["downstream_terminal_observed"] is False
    assert permission_abort["arm_motion_requested"] is False
    assert permission_abort["sequence_goal_sent"] is False
    assert "permission" in permission_abort["reason"]
    assert harness_abort["typed_goal_sent_to_edgegrasp_gate"] is False
    assert harness_abort["sequence_goal_sent"] is False
    assert "SIGPIPE" in harness_abort["reason"]
    assert permission_abort["cleanup_exact_domain_processes_remaining"] == 0
    assert harness_abort["cleanup_exact_domain_processes_remaining"] == 0
