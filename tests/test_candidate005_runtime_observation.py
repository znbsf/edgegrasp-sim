from __future__ import annotations

import json
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
OBSERVATION = (
    PROJECT_ROOT
    / "docs"
    / "observations"
    / "2026-08-27-candidate005-routed-plan-only.json"
)


def test_candidate005_observation_is_repeatable_plan_only_evidence() -> None:
    report = json.loads(OBSERVATION.read_text(encoding="utf-8"))
    assert report["runtime_preconditions"]["planning_scene_target_cube_retained"]
    assert report["runtime_preconditions"]["selective_pad_contact_policy_confirmed"]
    assert report["runtime_preconditions"]["edgegrasp_motion_nodes_launched"] is False

    lin = report["direct_path_baselines"]["pilz_lin"]
    assert lin["plan_accepted"] is False
    assert lin["collision_pair"] == ["edgegrasp_target_cube", "gripper_link"]
    assert lin["execution_attempted"] is False

    ompl = report["direct_path_baselines"]["ompl_summary"]
    assert ompl["edgegrasp_accepted"] == 6
    assert ompl["edgegrasp_rejected"] == 4
    assert ompl["deterministic"] is False

    routed = report["routed_pose_plan_only"]
    assert routed["all_segments_accepted"] is True
    assert routed["execution_attempted"] is False
    segments = routed["segments"]
    assert [item["name"] for item in segments] == [
        "home_to_approach_via",
        "approach_via_to_descend",
        "descend_to_lift_closed",
    ]
    assert all(item["attempts"] == item["accepted"] == 10 for item in segments)
    assert [item["source_point_count_each"] for item in segments] == [83, 32, 22]

    cleanup = report["cleanup"]
    assert cleanup["matching_process_count_after_exact_cleanup"] == 0
    assert cleanup["clock_topic_after_cleanup"] == "absent"
    assert "simulated grasp success" in report["claim_boundary"]["not_verified"]
