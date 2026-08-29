from __future__ import annotations

import json
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
OBSERVATION = (
    PROJECT_ROOT
    / "docs"
    / "observations"
    / "2026-08-27-planning-scene-runtime.json"
)


def test_planning_scene_runtime_observation_is_machine_readable_and_bounded() -> None:
    report = json.loads(OBSERVATION.read_text(encoding="utf-8"))
    cases = {case["case"]: case for case in report["cases"]}

    assert report["scene"]["service_echo_verified"] is True
    assert report["scene"]["periodic_revalidation_sample"] == {
        "true_messages": 10,
        "false_messages_after_first_true": 0,
    }
    assert cases["A_free_reachable"]["trajectory_request_count"] == 1
    assert cases["A_free_reachable"]["fjt_error_code"] == 0
    for case_name in (
        "B_inside_table",
        "C_path_crossing_table",
        "D_unreachable",
        "E_epoch_mismatch",
        "E_stale_during_planning",
    ):
        assert cases[case_name]["trajectory_request_count"] == 0
        assert cases[case_name]["trajectory_dispatched"] is False

    boundary = report["claim_boundary"]
    assert boundary["shared_gazebo_moveit_scene_runtime_verified"] is True
    assert boundary["table_collision_rejection_runtime_verified"] is True
    assert boundary["optional_cube_moveit_runtime_verified"] is False
    assert boundary["minimum_clearance_measured"] is False
    assert boundary["all_robot_proxy_collisions_verified"] is False
    assert boundary["grasp_sequence_runtime_verified"] is False
    assert boundary["physics_grasp_verified"] is False
