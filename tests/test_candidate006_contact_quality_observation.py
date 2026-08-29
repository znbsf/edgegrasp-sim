from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OBSERVATION = (
    ROOT
    / "docs"
    / "observations"
    / "2026-08-27-candidate006-contact-quality-runtime.json"
)
CANDIDATE = (
    ROOT
    / "ros_ws"
    / "src"
    / "edgegrasp_ros"
    / "config"
    / "so101_side_grasp_candidate.json"
)


def test_candidate006_records_quality_without_promoting_physics_success() -> None:
    evidence = json.loads(OBSERVATION.read_text(encoding="utf-8"))
    run = evidence["successful_observation"]
    physics = run["physics_result"]

    assert run["sequence_result"]["sequence_completed"] is True
    assert physics["physics_grasp_verified"] is False
    assert physics["lift_observed"] is False
    assert physics["retention_observed"] is False
    assert physics["peak_lift_m"] < physics["required_lift_m"]
    assert physics["simultaneous_gripper_contact_sample_count"] == 36
    assert physics["all_gripper_tokens_observed"] is True
    assert set(physics["per_pad_contact_quality"]) == {
        "edgegrasp_grasp_proxy_fixed_finger_pad",
        "edgegrasp_grasp_proxy_moving_finger_pad",
    }
    for row in physics["per_pad_contact_quality"].values():
        assert row["max_penetration_depth_m"] > 0.0
        assert row["max_force_magnitude_n"] > 0.0
        assert row["max_abs_normal_force_n"] > 0.0
    assert physics["gripper_effort"]["sample_count"] > 0
    assert physics["gripper_effort"]["peak_abs"] > 0.0
    assert run["runtime_health"]["cleanup_exact_domain_processes_remaining"] == 0
    assert run["runtime_health"]["clock_topic_after_cleanup"] == "unknown"
    assert evidence["observer_contract"][
        "contact_quality_fields_are_diagnostic_only"
    ]


def test_candidate_profile_references_the_same_candidate006_measurements() -> None:
    evidence = json.loads(OBSERVATION.read_text(encoding="utf-8"))
    candidate = json.loads(CANDIDATE.read_text(encoding="utf-8"))
    observed = evidence["successful_observation"]["physics_result"]
    recorded = candidate["contact_quality_runtime_observation"]

    assert recorded["path"] == OBSERVATION.relative_to(ROOT).as_posix()
    assert recorded["task_id"] == evidence["successful_observation"]["task_id"]
    assert recorded["evidence_digest"] == observed["evidence_digest"]
    assert recorded["gripper_contact_counts_by_token"] == observed[
        "gripper_contact_counts_by_token"
    ]
    assert recorded["same_sample_weak_side_maxima"] == observed[
        "same_sample_weak_side_maxima"
    ]
    assert recorded["peak_lift_m"] == observed["peak_lift_m"]


def test_aborted_preflights_are_not_misreported_as_motion_trials() -> None:
    evidence = json.loads(OBSERVATION.read_text(encoding="utf-8"))
    first, second = evidence["preflight_attempts"]

    assert first["gripper_command_sent"] is False
    assert first["sequence_goal_sent"] is False
    assert second["typed_gripper_preparation_succeeded"] is True
    assert second["sequence_goal_sent"] is False
    assert "target" in first["reason"].lower()
    assert "scene cube pose" in second["reason"]
