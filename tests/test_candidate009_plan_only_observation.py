import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OBSERVATION = (
    ROOT
    / "docs"
    / "observations"
    / "2026-08-27-candidate009-plan-only-runtime.json"
)
CANDIDATE = (
    ROOT
    / "ros_ws"
    / "src"
    / "edgegrasp_ros"
    / "config"
    / "so101_side_grasp_candidate.json"
)


def test_candidate009_plan_only_rejection_is_zero_execution_evidence() -> None:
    observed = json.loads(OBSERVATION.read_text(encoding="utf-8"))
    boundary = observed["motion_boundary"]
    segments = observed["plan_only_segments"]

    assert observed["evidence_status"] == (
        "RUNTIME_PLAN_ONLY_REJECTED_NO_IK_ZERO_EXECUTION"
    )
    assert segments[0]["plan_accepted"] is True
    assert segments[1]["plan_accepted"] is False
    assert segments[1]["move_group_log_error_name"] == "NO_IK_SOLUTION"
    assert segments[1]["source_point_count"] == 0
    assert segments[2] == {
        "name": "descend_to_lift_closed",
        "attempted": False,
        "reason": "prior_segment_rejected",
    }
    assert boundary["edgegrasp_motion_nodes_launched"] is False
    assert boundary["trajectory_publication_count"] == 0
    assert boundary["execute_trajectory_goal_count"] == 0
    assert boundary["fjt_goal_count"] == 0
    assert boundary["execution_attempted"] is False
    assert observed["cleanup"]["matching_domain_process_count_after_cleanup"] == 0
    assert observed["claim_boundary"]["candidate009_execution_authorized"] is False
    assert observed["claim_boundary"]["physics_grasp_verified"] is False


def test_candidate_contract_links_candidate009_plan_only_rejection() -> None:
    candidate = json.loads(CANDIDATE.read_text(encoding="utf-8"))
    observation = candidate["candidate009_plan_only_runtime_observation"]
    assert candidate["schema_version"] == 14
    assert observation["path"].endswith("candidate009-plan-only-runtime.json")
    assert observation["home_to_approach_plan_accepted"] is True
    assert observation["approach_to_descend_plan_accepted"] is False
    assert observation["move_group_log_error_name"] == "NO_IK_SOLUTION"
    assert observation["execution_attempted"] is False
