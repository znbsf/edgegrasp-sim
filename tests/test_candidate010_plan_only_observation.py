import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OBSERVATION = (
    ROOT
    / "docs"
    / "observations"
    / "2026-08-27-candidate010-q0p45-plan-only.json"
)


def test_candidate010_plan_only_pass_is_zero_execution_evidence() -> None:
    report = json.loads(OBSERVATION.read_text(encoding="utf-8"))
    assert report["evidence_status"] == "RUNTIME_PLAN_ONLY_PASS_ZERO_EXECUTION"
    assert report["candidate"]["gripper_contact_position_rad"] == 0.45
    assert report["candidate"]["target_center_x_offset_m"] == 0.0
    plan = report["plan_only"]
    assert plan["attempts_requested"] == 10
    assert plan["attempts_completed"] == 10
    assert plan["all_segments_accepted"] is True
    assert set(plan["segments"]) == {
        "home_to_approach",
        "approach_to_descend",
        "descend_to_lift_closed",
    }
    for segment in plan["segments"].values():
        assert segment["accepted"] == "10/10"
        assert segment["unique_trajectory_digest_count"] == 1
    boundary = report["motion_boundary"]
    assert boundary["edgegrasp_motion_nodes_launched"] is False
    assert boundary["trajectory_publication_count"] == 0
    assert boundary["execute_trajectory_goal_count"] == 0
    assert boundary["fjt_goal_count"] == 0
    assert boundary["execution_attempted"] is False
    assert report["cleanup"]["matching_domain_process_count_after_cleanup"] == 0
    assert report["claim_boundary"]["physics_grasp_verified"] is False
