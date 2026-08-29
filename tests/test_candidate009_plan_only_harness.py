import json
from pathlib import Path
import subprocess
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = PROJECT_ROOT / "scripts"


def test_chained_candidate_probe_is_service_only_and_fail_closed(
    tmp_path: Path,
) -> None:
    source = (SCRIPTS / "probe_grasp_candidate_plan_only.py").read_text(
        encoding="utf-8"
    )
    compile(source, "probe_grasp_candidate_plan_only.py", "exec")
    for required in (
        "GetMotionPlan",
        'create_client(GetMotionPlan, "/plan_kinematic_path")',
        "validate_and_convert_robot_trajectory",
        "validate_routed_grasp_stage_geometry",
        "approach_and_descend_disallowed_lift_allowed_after_close",
        "allow_target_pad_contacts",
        "set_target_pad_contacts",
        "terminal_joint_positions_rad",
        "target-center-x-offset-m",
        "effective_target_center_in_frame_m",
        '"trajectory_publication_count": 0',
        '"execute_trajectory_goal_count": 0',
        '"fjt_goal_count": 0',
        '"execution_attempted": False',
        '"motion_boundary_absent_throughout"',
    ):
        assert required in source
    for forbidden in (
        "ActionClient(",
        "create_publisher(",
        "send_goal_async(",
        "FollowJointTrajectory",
        "ExecuteTrajectory.Goal",
        "PlanTarget.Goal",
    ):
        assert forbidden not in source

    matrix_path = (
        PROJECT_ROOT
        / "ros_ws"
        / "src"
        / "edgegrasp_ros"
        / "config"
        / "candidate024_target_matrix.json"
    )
    matrix = json.loads(matrix_path.read_text(encoding="utf-8"))
    assert matrix["mode"] == "moveit_get_motion_plan_zero_execution"
    assert matrix["required_case_counts"] == {
        "reachable_hypothesis": 10,
        "rejection_hypothesis": 10,
    }
    assert len({item["case_id"] for item in matrix["cases"]}) == 20

    matrix_runner = SCRIPTS / "run_candidate024_target_matrix_plan_only.py"
    runner_source = matrix_runner.read_text(encoding="utf-8")
    compile(runner_source, matrix_runner.name, "exec")
    for required in (
        "SCENE_CONTRACT_REJECTED",
        "MATERIALIZED_FOR_PLAN_ONLY",
        "motion_side_effect_violation_count",
        "trajectory_publication_count",
        "execute_trajectory_goal_count",
        "fjt_goal_count",
        "motion_boundary_absent_throughout",
    ):
        assert required in runner_source
    output_dir = tmp_path / "candidate024_matrix"
    completed = subprocess.run(
        [
            sys.executable,
            str(matrix_runner),
            "--matrix",
            str(matrix_path),
            "--artifact-dir",
            str(output_dir),
            "--materialize-only",
        ],
        cwd=PROJECT_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0, completed.stderr
    materialization = json.loads(
        (output_dir / "matrix_materialization.json").read_text(encoding="utf-8")
    )
    assert materialization["status"] == "MATERIALIZATION_PASS"
    assert materialization["case_count"] == 20
    assert materialization["runtime_case_count"] == 16
    assert materialization["scene_contract_rejection_count"] == 4
    assert materialization["unexpected_contract_result_count"] == 0
    assert materialization["zero_execution_contract"] is True


def test_plan_only_harness_launches_no_edgegrasp_motion_node() -> None:
    source = (SCRIPTS / "run_grasp_candidate_plan_only.sh").read_text(
        encoding="utf-8"
    )
    for required in (
        "launch_edgegrasp_nodes:=false",
        "launch_physics_observer:=false",
        "/plan_kinematic_path \\[moveit_msgs/srv/GetMotionPlan\\]",
        "/edgegrasp/set_target_pad_contacts",
        '"allow_target_pad_contacts": false',
        "probe_grasp_candidate_plan_only.py",
        "chained_get_motion_plan_no_execute",
        "TARGET_CENTER_X_OFFSET_M",
        "SCENE_CONFIG_FILENAME",
        '--target-center-x-offset-m "$probe_target_center_x_offset_m"',
        '--scene-config-filename "$probe_scene_config_filename"',
        'scene_config:="$installed_scene"',
        'if [ -e "$probe_artifact_dir" ]',
        'bash scripts/cleanup_ros_domain.sh "$ROS_DOMAIN_ID" --terminate',
    ):
        assert required in source
    for forbidden in (
        "ros2 launch edgegrasp_moveit_adapter",
        "ros2 launch edgegrasp_grasp_sequence",
        "prepare_so101_trial.py",
        "grasp_trial_client",
        "ros2 action send_goal",
    ):
        assert forbidden not in source
