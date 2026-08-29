import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PLAN = (
    ROOT
    / "docs"
    / "observations"
    / "2026-08-27-candidate011-q0p40-plan-only.json"
)
RUNTIME = (
    ROOT
    / "docs"
    / "observations"
    / "2026-08-27-candidate011-q0p40-runtime.json"
)
CANDIDATE = (
    ROOT
    / "ros_ws"
    / "src"
    / "edgegrasp_ros"
    / "config"
    / "so101_side_grasp_candidate.json"
)


def test_candidate011_plan_only_passes_without_execution() -> None:
    report = json.loads(PLAN.read_text(encoding="utf-8"))
    assert report["plan_only"]["attempts_completed"] == 10
    assert report["plan_only"]["all_segments_accepted"] is True
    for segment in report["plan_only"]["segments"].values():
        assert segment["accepted"] == "10/10"
        assert segment["unique_trajectory_digest_count"] == 1
    assert report["motion_boundary"]["trajectory_publication_count"] == 0
    assert report["motion_boundary"]["fjt_goal_count"] == 0
    assert report["motion_boundary"]["execution_attempted"] is False


def test_candidate011_sequence_is_not_promoted_to_physics_success() -> None:
    report = json.loads(RUNTIME.read_text(encoding="utf-8"))
    assert report["sequence_result"]["sequence_completed"] is True
    assert report["sequence_result"]["wrapper_status"] == 4
    physics = report["physics_result"]
    assert physics["physics_grasp_verified"] is False
    assert physics["simultaneous_gripper_contact_sample_count"] == 171
    assert physics["peak_lift_m"] < physics["required_lift_m"]
    assert physics["retention_observed"] is False
    assert report["decision"]["closure_angle_search_stopped"] is True
    assert report["runtime_health"]["cleanup_exact_domain_processes_remaining"] == 0
    assert report["claim_boundary"]["physics_grasp_verified"] is False


def test_candidate_contract_links_latest_negative_physics_result() -> None:
    candidate = json.loads(CANDIDATE.read_text(encoding="utf-8"))
    latest = candidate["candidate011_q0p40_runtime_observation"]
    assert candidate["schema_version"] == 14
    assert latest["sequence_completed"] is True
    assert latest["physics_grasp_verified"] is False
    assert latest["closure_angle_search_stopped"] is True
    assert latest["peak_lift_m"] < latest["required_lift_m"]
