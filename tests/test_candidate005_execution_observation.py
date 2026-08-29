from __future__ import annotations

import json
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
OBSERVATION = (
    PROJECT_ROOT
    / "docs"
    / "observations"
    / "2026-08-27-candidate005-typed-runtime.json"
)


def test_candidate005_runtime_separates_sequence_and_physics_evidence() -> None:
    report = json.loads(OBSERVATION.read_text(encoding="utf-8"))
    assert report["evidence_status"] == (
        "RUNTIME_SEQUENCE_COMPLETE_CONTACT_OBSERVED_PHYSICS_FAIL"
    )
    assert report["runtime_preconditions"]["motion_allowed"] is True
    assert report["runtime_preconditions"]["interface_ready"] is True
    assert report["planning_scene_policy"]["target_cube_retained"] is True
    assert report["planning_scene_policy"]["forbidden_target_contact_links"] == [
        "gripper_link",
        "moving_jaw_so101_v1_link",
    ]

    setup = report["typed_gripper_preparation"]
    assert len(setup["failed_attempts"]) == 2
    assert setup["successful_attempt"]["success"] is True
    assert setup["successful_attempt"]["fjt_error_code"] == 0
    assert setup["direct_fjt_bypass"] is False

    sequence = report["sequence_result"]
    assert sequence["process_exit_code"] == 11
    assert sequence["wrapper_status"] == 4
    assert sequence["sequence_completed"] is True
    assert sequence["terminal_phase"] == "COMPLETE"
    assert sequence["observed_stage_order"][-1] == "COMPLETE"

    physics = report["physics_result"]
    assert physics["gripper_contact_observed"] is True
    assert physics["gripper_contact_count"] == 1283
    assert "fixed_finger_pad" in physics["matched_contact_pair"][1]
    assert physics["per_pad_contact_histogram_captured"] is False
    assert physics["peak_lift_m"] < physics["required_lift_m"]
    assert physics["lift_observed"] is False
    assert physics["retention_observed"] is False
    assert physics["physics_grasp_verified"] is False

    assert report["runtime_health"]["dart_mesh_construction_diagnostic_count"] == 0
    assert report["runtime_health"]["final_robot_pose_xyz_rpy"] == [0.0] * 6
    assert report["cleanup"]["matching_process_count_after_cleanup"] == 0
    assert report["cleanup"]["clock_topic_after_cleanup"] == "absent"
    assert report["cleanup"]["broad_kill_used"] is False
