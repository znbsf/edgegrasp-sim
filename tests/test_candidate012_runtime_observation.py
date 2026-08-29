from __future__ import annotations

import json
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
OBSERVATION = (
    PROJECT_ROOT
    / "docs"
    / "observations"
    / "2026-08-28-candidate012-pad-friction-runtime.json"
)


def test_candidate012_is_a_strict_paired_negative_observation() -> None:
    observed = json.loads(OBSERVATION.read_text(encoding="utf-8"))
    experiment = observed["experiment"]
    control = observed["paired_runtime"]["control"]
    treatment = observed["paired_runtime"]["treatment"]
    comparison = observed["comparison"]

    assert experiment["single_changed_runtime_parameter"] == (
        "generated finger-pad isotropic mu and mu2"
    )
    assert experiment["control_coefficient"] == 1.0
    assert experiment["treatment_coefficient"] == 1.5
    assert control["sequence_completed"] is True
    assert treatment["sequence_completed"] is True
    assert control["physics_grasp_verified"] is False
    assert treatment["physics_grasp_verified"] is False
    assert control["retention_observed"] is False
    assert treatment["retention_observed"] is False
    assert control["peak_lift_m"] < comparison["required_lift_m"]
    assert treatment["peak_lift_m"] < comparison["required_lift_m"]
    assert comparison["both_physics_grasp_verified"] is False


def test_candidate012_comparison_is_derived_from_recorded_rows() -> None:
    observed = json.loads(OBSERVATION.read_text(encoding="utf-8"))
    control = observed["paired_runtime"]["control"]
    treatment = observed["paired_runtime"]["treatment"]
    comparison = observed["comparison"]

    assert comparison["peak_lift_delta_treatment_minus_control_m"] == (
        treatment["peak_lift_m"] - control["peak_lift_m"]
    )
    assert comparison["simultaneous_contact_sample_delta"] == (
        treatment["simultaneous_contact_sample_count"]
        - control["simultaneous_contact_sample_count"]
    )
    assert comparison["simultaneous_contact_span_delta_ns"] == (
        treatment["simultaneous_contact_span_ns"]
        - control["simultaneous_contact_span_ns"]
    )
    assert observed["claim_boundary"]["dart_friction_coefficient_effect_identified"] is (
        False
    )


def test_candidate012_preserves_plan_only_and_cleanup_boundaries() -> None:
    observed = json.loads(OBSERVATION.read_text(encoding="utf-8"))
    preflight = observed["plan_only_preflight"]
    health = observed["runtime_health"]

    for row in ("implicit_context", "explicit_control", "explicit_treatment"):
        assert preflight[row]["attempts_completed"] == 10
        assert preflight[row]["all_segments_accepted"] is True
    assert preflight["execution_attempted"] is False
    assert preflight["fjt_goal_count"] == 0
    assert health["cleanup_exact_domain_processes_remaining_both_rows"] == 0
    assert health["clock_topic_after_cleanup_both_rows"] == "unknown"


def test_candidate012_is_the_current_candidate_status_without_authorizing_motion() -> None:
    candidate_path = (
        PROJECT_ROOT
        / "ros_ws"
        / "src"
        / "edgegrasp_ros"
        / "config"
        / "so101_side_grasp_candidate.json"
    )
    candidate = json.loads(candidate_path.read_text(encoding="utf-8"))
    recorded = candidate["candidate012_pad_friction_runtime_observation"]

    assert candidate["runtime_execution_status"] == (
        "LATEST_CANDIDATE012_PAIRED_SEQUENCE_COMPLETED_NO_PHYSICS_GRASP"
    )
    assert recorded["both_sequence_completed"] is True
    assert recorded["physics_grasp_verified"] is False
    assert recorded["friction_search_stopped"] is True
    assert candidate["execution_authorized"] is False
