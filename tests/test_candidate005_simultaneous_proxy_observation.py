from __future__ import annotations

import json
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
OBSERVATION = (
    PROJECT_ROOT
    / "docs"
    / "observations"
    / "2026-08-27-candidate005-simultaneous-proxy-runtime.json"
)


def test_clock_rollback_attempt_is_retained_as_fail_closed_evidence() -> None:
    report = json.loads(OBSERVATION.read_text(encoding="utf-8"))
    failed, succeeded = report["attempts"]
    assert failed["result"] == "SAFE_STOP_CLOCK_ROLLBACK"
    assert failed["sequence_completed"] is False
    assert failed["terminal_phase"] == "SAFE_STOP"
    assert failed["rollback_observed_ns"] < failed["rollback_previous_ns"]
    assert failed["clock_publisher_count"] == 1
    assert failed["clock_publisher_node"] == "ros_gz_bridge"
    assert "not weakened" in failed["recovery_policy"]
    assert succeeded["ros_domain_id"] != failed["ros_domain_id"]


def test_correct_proxy_overlay_closes_simultaneous_contact_gap_not_grasp() -> None:
    report = json.loads(OBSERVATION.read_text(encoding="utf-8"))
    assert report["evidence_status"] == (
        "RUNTIME_PROXY_MOVEIT_SEQUENCE_COMPLETE_SIMULTANEOUS_PAD_CONTACT_NO_LIFT"
    )
    planning = report["planning_model_contract"]
    assert planning["edgegrasp_proxy_move_group_used"] is True
    assert planning["direct_moveit_execution_action_present"] is False
    assert planning["move_group_robot_description_pad_links"] == [
        "edgegrasp_fixed_finger_pad_link",
        "edgegrasp_moving_finger_pad_link",
    ]
    assert planning["validate_solution_load_count"] == 3

    succeeded = report["attempts"][1]
    preflight = succeeded["preflight"]
    assert preflight["motion_allowed"] is True
    assert preflight["interface_ready"] is True
    assert preflight["planning_scene_reason"] == "confirmed"
    assert preflight["planning_scene_node_count"] == 1
    assert preflight["forbidden_target_contact_links"] == [
        "gripper_link",
        "moving_jaw_so101_v1_link",
    ]

    sequence = succeeded["sequence_result"]
    assert sequence["process_exit_code"] == 11
    assert sequence["wrapper_status"] == 4
    assert sequence["sequence_completed"] is True
    assert sequence["terminal_phase"] == "COMPLETE"
    assert sequence["last_terminal_command_id"].endswith("|lift|3")

    physics = succeeded["physics_result"]
    histogram = physics["gripper_contact_counts_by_token"]
    assert histogram == {
        "edgegrasp_grasp_proxy_fixed_finger_pad": 1125,
        "edgegrasp_grasp_proxy_moving_finger_pad": 188,
    }
    assert sum(histogram.values()) == physics["gripper_contact_count"] == 1313
    assert physics["all_gripper_tokens_observed"] is True
    assert physics["simultaneous_gripper_contact_sample_count"] == 35
    assert physics["simultaneous_contact_samples_observed"] is True
    assert physics["simultaneous_bilateral_force_closure_verified"] is False
    assert physics["peak_lift_m"] < physics["required_lift_m"]
    assert physics["lift_observed"] is False
    assert physics["retention_observed"] is False
    assert physics["physics_grasp_verified"] is False

    health = succeeded["runtime_health"]
    assert health["clock_publisher_count"] == 1
    assert health["final_robot_pose_xyz_rpy"] == [0.0] * 6
    assert health["dart_mesh_construction_diagnostic_count"] == 0
    assert health["geometry_could_not_be_created_count"] == 0
    cleanup = succeeded["cleanup"]
    assert cleanup["matching_process_count_after_cleanup"] == 0
    assert cleanup["clock_publisher_count_after_cleanup"] == 0
    assert cleanup["broad_kill_used"] is False
