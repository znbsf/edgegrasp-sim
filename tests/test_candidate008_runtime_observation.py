from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OBSERVATION = (
    ROOT
    / "docs"
    / "observations"
    / "2026-08-27-candidate008-q0p50-runtime.json"
)
CANDIDATE007 = (
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


def test_candidate008_records_improvement_without_physics_promotion() -> None:
    evidence = json.loads(OBSERVATION.read_text(encoding="utf-8"))
    sequence = evidence["sequence_result"]
    physics = evidence["physics_result"]
    timing = physics["simultaneous_contact_timing"]

    assert evidence["candidate"]["gripper_closed_position_rad"] == 0.5
    assert evidence["candidate"]["source_profile"].endswith(
        "so101_grasp_geometry_candidate008_q0p50.json"
    )
    assert sequence["sequence_completed"] is True
    assert sequence["last_terminal_command_id"].endswith("|lift|3")
    assert physics["physics_grasp_verified"] is False
    assert physics["lift_observed"] is False
    assert physics["retention_observed"] is False
    assert physics["peak_lift_m"] == 0.00266662576599772
    assert physics["peak_lift_m"] < physics["required_lift_m"]
    assert physics["simultaneous_gripper_contact_sample_count"] == 98
    assert timing["observed_span_ns"] == 252_000_000
    assert timing["ended_before_sequence_completion_ns"] == 1_788_000_000
    assert timing["last_source_timestamp_ns"] < sequence["source_timestamp_ns"]
    assert timing["ended_before_sequence_completion_ns"] == (
        sequence["source_timestamp_ns"] - timing["last_source_timestamp_ns"]
    )
    assert evidence["runtime_health"]["cleanup_exact_domain_processes_remaining"] == 0
    assert evidence["runtime_health"]["clock_topic_after_cleanup"] == "unknown"


def test_candidate008_is_a_stronger_negative_control_than_candidate007() -> None:
    current = json.loads(OBSERVATION.read_text(encoding="utf-8"))
    prior = json.loads(CANDIDATE007.read_text(encoding="utf-8"))
    current_physics = current["physics_result"]
    prior_physics = prior["successful_observation"]["physics_result"]

    assert current_physics["simultaneous_gripper_contact_sample_count"] > (
        prior_physics["simultaneous_gripper_contact_sample_count"]
    )
    assert current_physics["simultaneous_contact_timing"]["observed_span_ns"] > (
        prior_physics["simultaneous_contact_timing"]["observed_span_ns"]
    )
    assert current_physics["gripper_effort"]["peak_abs"] > prior_physics[
        "gripper_effort"
    ]["peak_abs"]
    assert current_physics["peak_lift_m"] > prior_physics["peak_lift_m"]
    assert current_physics["physics_grasp_verified"] is False
    assert prior_physics["physics_grasp_verified"] is False
    assert current["interpretation"]["next_controlled_variable"].startswith(
        "First improve pose/contact symmetry"
    )


def test_candidate_profile_references_candidate008_runtime_evidence() -> None:
    evidence = json.loads(OBSERVATION.read_text(encoding="utf-8"))
    candidate = json.loads(CANDIDATE.read_text(encoding="utf-8"))
    recorded = candidate["candidate008_tighter_closure_runtime_observation"]
    physics = evidence["physics_result"]

    assert candidate["schema_version"] == 14
    assert recorded["path"] == OBSERVATION.relative_to(ROOT).as_posix()
    assert recorded["task_id"] == evidence["invocation"]["task_id"]
    assert recorded["gripper_closed_position_rad"] == evidence["candidate"][
        "gripper_closed_position_rad"
    ]
    assert recorded["evidence_digest"] == physics["evidence_digest"]
    assert recorded["simultaneous_gripper_contact_sample_count"] == physics[
        "simultaneous_gripper_contact_sample_count"
    ]
    assert recorded["simultaneous_contact_observed_span_ns"] == physics[
        "simultaneous_contact_timing"
    ]["observed_span_ns"]
    assert recorded["peak_lift_m"] == physics["peak_lift_m"]
    assert recorded["physics_grasp_verified"] is False
