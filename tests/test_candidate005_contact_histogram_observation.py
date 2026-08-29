from __future__ import annotations

import json
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
OBSERVATION = (
    PROJECT_ROOT
    / "docs"
    / "observations"
    / "2026-08-27-candidate005-contact-histogram-runtime.json"
)


def test_candidate005_histogram_closes_token_coverage_not_grasp_claim() -> None:
    report = json.loads(OBSERVATION.read_text(encoding="utf-8"))
    assert report["evidence_status"] == (
        "RUNTIME_SEQUENCE_COMPLETE_BOTH_PAD_TOKENS_OBSERVED_NO_LIFT"
    )
    assert report["runtime_preconditions"]["motion_allowed"] is True
    assert report["runtime_preconditions"]["interface_ready"] is True
    assert report["runtime_preconditions"]["planning_scene_ready"] is True
    assert report["runtime_preconditions"]["target_cube_retained"] is True
    assert report["runtime_preconditions"]["forbidden_target_contact_links"] == [
        "gripper_link",
        "moving_jaw_so101_v1_link",
    ]
    planning_boundary = report["planning_model_boundary"]
    assert planning_boundary["edgegrasp_proxy_move_group_used"] is False
    assert planning_boundary["contact_measurements_remain_valid"] is True
    assert planning_boundary["planning_model_claim_valid"] is False
    assert report["superseded_for_planning_model_scope_by"] == (
        "candidate005-simultaneous-proxy-runtime-2026-08-27"
    )

    preparation = report["typed_gripper_preparation"]
    assert preparation["use_sim_time_default_verified"] is True
    assert preparation["direct_fjt_bypass"] is False
    assert preparation["wrapper_status"] == 4
    assert preparation["fjt_error_code"] == 0

    sequence = report["sequence_result"]
    assert sequence["process_exit_code"] == 11
    assert sequence["wrapper_status"] == 4
    assert sequence["sequence_completed"] is True
    assert sequence["terminal_phase"] == "COMPLETE"
    assert sequence["last_terminal_command_id"].endswith("|lift|3")

    physics = report["physics_result"]
    histogram = physics["gripper_contact_counts_by_token"]
    assert histogram == {
        "edgegrasp_grasp_proxy_fixed_finger_pad": 1159,
        "edgegrasp_grasp_proxy_moving_finger_pad": 195,
    }
    assert sum(histogram.values()) == physics["gripper_contact_count"] == 1354
    assert physics["both_pad_tokens_observed"] is True
    assert physics["simultaneous_bilateral_force_closure_verified"] is False
    assert physics["peak_lift_m"] < physics["required_lift_m"]
    assert physics["lift_observed"] is False
    assert physics["retention_observed"] is False
    assert physics["physics_grasp_verified"] is False

    startup = report["startup_observations"]
    assert startup["motion_started_only_after_single_scene_node_confirmed"] is True
    assert startup["planning_scene_runtime_parameters"]["query_timeout_ms"] == 5000.0

    health = report["runtime_health"]
    assert health["final_robot_pose_xyz_rpy"] == [0.0] * 6
    assert health["dart_mesh_construction_diagnostic_count"] == 0
    assert report["cleanup"]["matching_process_count_after_cleanup"] == 0
    assert report["cleanup"]["clock_topic_after_cleanup"] == "absent"
    assert report["cleanup"]["broad_kill_used"] is False
    assert "proxy MoveGroup" in report["claim_boundary"]["retracted"]
