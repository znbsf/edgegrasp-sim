from __future__ import annotations

import json
from pathlib import Path
import shutil
import subprocess
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONFIG = PROJECT_ROOT / "ros_ws" / "src" / "edgegrasp_ros" / "config"
WORLDS = PROJECT_ROOT / "ros_ws" / "src" / "edgegrasp_ros" / "worlds"
EXPERIMENT = CONFIG / "candidate012_pad_friction_experiment.json"
VALIDATOR = PROJECT_ROOT / "scripts" / "validate_candidate012_experiment.py"


def _validate(profile: str, *, config_root: Path = CONFIG, world_root: Path = WORLDS):
    return subprocess.run(
        [
            sys.executable,
            str(VALIDATOR),
            "--profile",
            profile,
            "--experiment",
            str(config_root / EXPERIMENT.name),
            "--config-root",
            str(config_root),
            "--world-root",
            str(world_root),
        ],
        check=False,
        capture_output=True,
        text=True,
    )


def test_candidate012_matrix_freezes_all_nonmaterial_inputs() -> None:
    reports = []
    for profile in (
        "implicit_default",
        "candidate012_control_mu1p0",
        "candidate012_treatment_mu1p5",
    ):
        completed = _validate(profile)
        assert completed.returncode == 0, completed.stderr
        reports.append(json.loads(completed.stdout))
    assert [item["configured_coefficient"] for item in reports] == [None, 1.0, 1.5]
    assert {json.dumps(item["hashes"], sort_keys=True) for item in reports} == {
        json.dumps(reports[0]["hashes"], sort_keys=True)
    }
    assert {
        json.dumps(item["fixed_runtime_inputs"], sort_keys=True) for item in reports
    } == {json.dumps(reports[0]["fixed_runtime_inputs"], sort_keys=True)}
    assert all(item["gripper_contact_position_rad"] == 0.4 for item in reports)


def test_candidate012_contract_fails_on_baseline_hash_drift(tmp_path: Path) -> None:
    config_copy = tmp_path / "config"
    world_copy = tmp_path / "worlds"
    config_copy.mkdir()
    world_copy.mkdir()
    experiment = json.loads(EXPERIMENT.read_text(encoding="utf-8"))
    required_config_files = (
        experiment["fixed_inputs"]["grasp_geometry_filename"],
        experiment["fixed_inputs"]["scene_filename"],
        experiment["fixed_inputs"]["collision_proxy_filename"],
        experiment["material_contract"]["filename"],
        EXPERIMENT.name,
    )
    for filename in required_config_files:
        shutil.copy2(CONFIG / filename, config_copy / filename)
    shutil.copy2(
        WORLDS / experiment["fixed_inputs"]["gazebo_world_filename"],
        world_copy / experiment["fixed_inputs"]["gazebo_world_filename"],
    )
    geometry_path = config_copy / experiment["fixed_inputs"][
        "grasp_geometry_filename"
    ]
    geometry_path.write_text(
        geometry_path.read_text(encoding="utf-8") + "\n", encoding="utf-8"
    )
    completed = _validate(
        "candidate012_control_mu1p0",
        config_root=config_copy,
        world_root=world_copy,
    )
    assert completed.returncode != 0
    assert "grasp geometry hash drift" in completed.stderr


def test_candidate012_wrapper_freezes_q0p40_and_runs_contract_validator() -> None:
    wrapper = (PROJECT_ROOT / "scripts" / "run_candidate012_pad_friction.sh").read_text(
        encoding="utf-8"
    )
    harness = (PROJECT_ROOT / "scripts" / "run_candidate005_contact_quality.sh").read_text(
        encoding="utf-8"
    )
    assert "so101_grasp_geometry_candidate011_q0p40.json" in wrapper
    assert "0.40 true" in wrapper
    assert "EDGEGRASP_EXPERIMENT_ID=candidate012_pad_friction" in wrapper
    assert "validate_candidate012_experiment.py" in harness
    assert "candidate012_contract_validation.json" in harness
    assert 'std_srvs/srv/SetBool "{data: true}"' not in harness
    assert "stage-scoped target-pad ACM transitions" in harness
    assert "print(repr(float(sys.argv[1])))" in harness
    assert harness.count("future_skew_tolerance_ms:=1000.0") == 3
    assert 'pad_contact_material_profile:="$trial_pad_contact_material_profile"' in harness
    assert "/arm_controller/follow_joint_trajectory" not in wrapper

    plan_wrapper = (
        PROJECT_ROOT / "scripts" / "run_candidate012_plan_only.sh"
    ).read_text(encoding="utf-8")
    generic_plan = (
        PROJECT_ROOT / "scripts" / "run_grasp_candidate_plan_only.sh"
    ).read_text(encoding="utf-8")
    assert "so101_grasp_geometry_candidate011_q0p40.json" in plan_wrapper
    assert "10 0.0" in plan_wrapper
    assert "PAD_CONTACT_MATERIAL_PROFILE" in generic_plan
    assert 'pad_contact_material_profile:="$probe_pad_contact_material_profile"' in (
        generic_plan
    )
    assert "/edgegrasp/(execute_trajectory|plan_target|grasp_sequence)" in generic_plan


def test_candidate012_future_skew_window_is_explicit_and_cross_layer() -> None:
    launch_paths = (
        PROJECT_ROOT
        / "ros_ws"
        / "src"
        / "edgegrasp_ros"
        / "launch"
        / "edgegrasp_proxy_gazebo.launch.py",
        PROJECT_ROOT
        / "ros_ws"
        / "src"
        / "edgegrasp_moveit_adapter"
        / "launch"
        / "moveit_adapter.launch.py",
        PROJECT_ROOT
        / "ros_ws"
        / "src"
        / "edgegrasp_grasp_sequence"
        / "launch"
        / "grasp_sequence.launch.py",
    )
    for path in launch_paths:
        source = path.read_text(encoding="utf-8")
        assert 'LaunchConfiguration("future_skew_tolerance_ms")' in source
        assert '"future_skew_tolerance_ms": future_skew_tolerance_ms' in source
        assert "DEFAULT_ROS_FUTURE_SKEW_TOLERANCE_MS" in source

    experiment = json.loads(EXPERIMENT.read_text(encoding="utf-8"))
    assert experiment["fixed_inputs"]["future_skew_tolerance_ms"] == 1000.0
    assert experiment["fixed_inputs"]["target_publisher_start_delay_s"] == 3.0
    assert experiment["preflight"][
        "future_samples_remain_denied_until_local_clock_catches_up"
    ] is True

    proxy_launch = launch_paths[0].read_text(encoding="utf-8")
    assert 'LaunchConfiguration(\n        "target_publisher_start_delay_s"' in (
        proxy_launch
    )
    assert '"target_publisher_start_delay_s"' in proxy_launch
    assert "period=target_publisher_start_delay_s" in proxy_launch

    harness = (
        PROJECT_ROOT / "scripts" / "run_candidate005_contact_quality.sh"
    ).read_text(encoding="utf-8")
    assert "target_publisher_start_delay_s:=3.0" in harness
