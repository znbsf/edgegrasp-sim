import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OBSERVATION = (
    ROOT
    / "docs"
    / "observations"
    / "2026-08-27-candidate009-reachability-scan.json"
)


def test_reachability_scan_preserves_zero_execution_boundary() -> None:
    report = json.loads(OBSERVATION.read_text(encoding="utf-8"))
    cases = {case["offset_m"]: case for case in report["cases"]}
    assert report["evidence_status"] == (
        "RUNTIME_PLAN_ONLY_REACHABILITY_BOUNDARY_ZERO_EXECUTION"
    )
    assert cases[0.0]["status"] == "PLAN_ONLY_PASS"
    assert cases[0.0]["attempts_completed"] == 10
    assert cases[0.0]["accepted_segment_count"] == 30
    for offset in (-0.0001, -0.0002, -0.0003, -0.0005, -0.0007, -0.0008, -0.0009):
        assert cases[offset]["status"] == "PLAN_ONLY_REJECTED"
        assert cases[offset]["failed_segment"] == "approach_to_descend"
        assert cases[offset]["move_group_log_error_name"] == "NO_IK_SOLUTION"
    safety = report["safety_and_cleanup"]
    assert safety["all_graphs_motion_boundary_absent"] is True
    assert safety["all_graphs_trajectory_publication_count"] == 0
    assert safety["all_graphs_execute_trajectory_goal_count"] == 0
    assert safety["all_graphs_fjt_goal_count"] == 0
    assert safety["all_graphs_execution_attempted"] is False
    assert safety["all_graphs_domain_process_count_after_cleanup"] == 0
    assert safety["all_graphs_clock_absent_after_cleanup"] is True
    assert report["decision"]["candidate009_execution_authorized"] is False


def test_zero_offset_control_is_repeatable_for_every_segment() -> None:
    repeatability = json.loads(OBSERVATION.read_text(encoding="utf-8"))[
        "zero_offset_repeatability"
    ]
    assert set(repeatability) == {
        "home_to_approach",
        "approach_to_descend",
        "descend_to_lift_closed",
    }
    for segment in repeatability.values():
        assert segment["accepted"] == "10/10"
        assert segment["unique_trajectory_digest_count"] == 1
