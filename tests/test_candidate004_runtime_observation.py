from __future__ import annotations

import json
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
OBSERVATION = (
    PROJECT_ROOT
    / "docs"
    / "observations"
    / "2026-08-27-candidate004-selective-acm-plan-only.json"
)


def test_candidate004_observation_is_fail_closed_and_plan_only() -> None:
    report = json.loads(OBSERVATION.read_text(encoding="utf-8"))
    scene = report["planning_scene"]
    assert scene["target_cube_retained"] is True
    assert scene["post_confirmation_ready"] is True
    assert scene["observed_target_row_permissions"] == {
        "edgegrasp_fixed_finger_pad_link": True,
        "edgegrasp_moving_finger_pad_link": True,
        "gripper_link": False,
        "moving_jaw_so101_v1_link": False,
    }

    cases = {case["case"]: case for case in report["plan_only_cases"]}
    for name in (
        "approach_to_descend_gripper_1_5",
        "approach_to_descend_gripper_0_6_control",
    ):
        case = cases[name]
        assert case["plan_accepted"] is False
        assert case["moveit_error_name"] == "INVALID_MOTION_PLAN"
        assert case["collision_pair"] == [
            "edgegrasp_target_cube",
            "gripper_link",
        ]
        assert case["execution_attempted"] is False

    lift = cases["descend_to_lift_gripper_0_6"]
    assert lift["plan_accepted"] is True
    assert lift["moveit_error_code"] == 1
    assert lift["execution_attempted"] is False

    cleanup = report["cleanup"]
    assert cleanup["matching_process_count_after_exact_cleanup"] == 0
    assert cleanup["clock_publisher_count_after_cleanup"] == 0
    assert "simulated grasp success" in report["claim_boundary"]["not_verified"]
