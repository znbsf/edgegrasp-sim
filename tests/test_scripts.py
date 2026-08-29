from pathlib import Path
import json
import shutil
import subprocess
import sys

import pytest


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = PROJECT_ROOT / "scripts"


def powershell_executable() -> str | None:
    return shutil.which("pwsh") or shutil.which("powershell")


def bash_executable() -> str | None:
    if sys.platform == "win32":
        git_bash = Path(r"C:\Program Files\Git\bin\bash.exe")
        if git_bash.is_file():
            return str(git_bash)
    return shutil.which("bash")


def test_powershell_scripts_parse() -> None:
    shell = powershell_executable()
    if shell is None:
        pytest.skip("PowerShell is unavailable")
    parser = (
        "& { param([string]$Path) "
        "[scriptblock]::Create([IO.File]::ReadAllText($Path)) | Out-Null }"
    )
    for path in sorted(SCRIPTS.glob("*.ps1")):
        result = subprocess.run(
            [shell, "-NoProfile", "-NonInteractive", "-Command", parser, str(path)],
            capture_output=True,
            text=True,
            timeout=30,
        )
        assert result.returncode == 0, f"{path.name}: {result.stdout}{result.stderr}"


def test_git_bash_or_host_bash_syntax_for_shell_scripts() -> None:
    bash = bash_executable()
    if bash is None:
        pytest.skip("bash is unavailable")
    scripts = [str(path) for path in sorted(SCRIPTS.glob("*.sh"))]
    result = subprocess.run(
        [bash, "-n", *scripts],
        cwd=PROJECT_ROOT,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr

    repeat_source = (SCRIPTS / "run_grasp_sequence_repetitions.sh").read_text(
        encoding="utf-8"
    )
    for forbidden in ("pkill", "killall", "kill -9"):
        assert forbidden not in repeat_source
    for required in (
        'kill -INT "$sequence_pid"',
        'kill -TERM "$sequence_pid"',
        "/edgegrasp/plan_target",
        "/edgegrasp/execute_trajectory",
        "/edgegrasp/grasp_sequence_repetitions/pid_${script_pid}/run_${run_label}",
        '-p sequence_action:="$sequence_action"',
        '-r __node:="$sequence_node_name"',
        '-r __node:="$client_node_name"',
        '"$(action_server_count "$sequence_action")" -eq 1',
        'sequence_executable="${sequence_prefix}/lib/edgegrasp_grasp_sequence/grasp_sequence"',
        '"$sequence_executable" --ros-args',
        '"$client_executable" --ros-args',
        '-p approach_orientation_xyzw:="$approach_orientation"',
        '-p grasp_orientation_xyzw:="$grasp_orientation"',
        "Backgrounding `ros2 run` tracks",
        "shutdown_clean",
        "cannot use Destroyable",
        '"sequence_completed": true',
        "does not prove object contact or grasp physics",
    ):
        assert required in repeat_source
    assert "ros2 run edgegrasp_grasp_sequence grasp_sequence " not in repeat_source


def test_wsl_audit_uses_bounded_exact_argument_invocation() -> None:
    source = (SCRIPTS / "audit_environment.ps1").read_text(encoding="utf-8")

    for expected in (
        '"--status"',
        '"--version"',
        '"--list", "--verbose"',
        '"--list", "--quiet"',
        '"-d"',
        '"cat /etc/os-release; uname -a"',
        "$WslDistribution",
        "UNVERIFIED_REQUESTED_DISTRIBUTION_NOT_REGISTERED",
        "ArgumentList.Add",
        "WaitForExit",
        "UNVERIFIED_TIMEOUT",
    ):
        assert expected in source
    assert "Invoke-Expression" not in source
    assert "Start-Process" not in source


def test_pinned_so101_mjcf_static_validator_runs_when_checkout_is_present() -> None:
    script = SCRIPTS / "validate_so101_mujoco.py"
    checkout = PROJECT_ROOT.parents[1] / "workspaces" / "forks" / "SO-ARM100"
    if not checkout.is_dir():
        assert "--static-only" in script.read_text(encoding="utf-8")
        return

    result = subprocess.run(
        [sys.executable, str(script), "--static-only"],
        cwd=PROJECT_ROOT,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    report = json.loads(result.stdout)
    assert report["static_original_asset_validation"] is True
    assert report["runtime"]["verified"] is False
    assert report["joint_names"] == [
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
        "gripper",
    ]


def test_fk_scan_is_explicitly_read_only_and_has_no_motion_client() -> None:
    source = (SCRIPTS / "scan_so101_fk.py").read_text(encoding="utf-8")
    compile(source, "scan_so101_fk.py", "exec")
    assert "GetPositionFK" in source
    assert "read_only_fk_service_scan_no_motion" in source
    for forbidden in (
        "ActionClient",
        "JointTrajectory",
        "from control_msgs",
        "ExecuteTrajectory",
        "PlanTarget",
    ):
        assert forbidden not in source


def test_ik_probe_is_explicitly_read_only_and_collision_aware() -> None:
    source = (SCRIPTS / "probe_so101_ik.py").read_text(encoding="utf-8")
    compile(source, "probe_so101_ik.py", "exec")
    assert "GetPositionIK" in source
    assert "ik.avoid_collisions = not args.allow_collisions" in source
    assert "read_only_collision_aware_ik_service_no_motion" in source
    for forbidden in (
        "ActionClient",
        "JointTrajectory",
        "FollowJointTrajectory",
        "ExecuteTrajectory",
        "PlanTarget",
        "create_publisher",
    ):
        assert forbidden not in source


def test_valid_fk_scan_uses_moveit_state_validity_without_motion_client() -> None:
    source = (SCRIPTS / "scan_so101_valid_fk.py").read_text(encoding="utf-8")
    compile(source, "scan_so101_valid_fk.py", "exec")
    assert "GetStateValidity" in source
    assert "GetPositionFK" in source
    assert "read_only_moveit_state_validity_plus_fk_no_motion" in source
    assert 'parser.add_argument("--roll"' in source
    assert 'parser.add_argument("--lift"' in source
    assert 'parser.add_argument("--elbow"' in source
    assert 'parser.add_argument("--wrist"' in source
    assert "outside the pinned URDF position limits" in source
    for forbidden in (
        "ActionClient",
        "JointTrajectory",
        "FollowJointTrajectory",
        "ExecuteTrajectory",
        "PlanTarget",
        "create_publisher",
    ):
        assert forbidden not in source


def test_trial_preparation_uses_only_typed_edgegrasp_gate() -> None:
    source = (SCRIPTS / "prepare_so101_trial.py").read_text(encoding="utf-8")
    compile(source, "prepare_so101_trial.py", "exec")
    assert "ExecuteTrajectory" in source
    assert '"/edgegrasp/execute_trajectory"' in source
    assert "downstream_terminal_observed" in source
    assert "fjt_error_code" in source
    assert '"--gripper-only"' in source
    assert '"arm_motion_requested"' in source
    assert 'Parameter("use_sim_time", value=True)' in source
    assert "trial preparation requires use_sim_time=true" in source
    for forbidden in (
        "FollowJointTrajectory",
        "/arm_controller/follow_joint_trajectory",
        "/gripper_controller/follow_joint_trajectory",
        "create_publisher",
    ):
        assert forbidden not in source


def test_contact_quality_harness_locks_geometry_profile_and_close_angle() -> None:
    source = (SCRIPTS / "run_candidate005_contact_quality.sh").read_text(
        encoding="utf-8"
    )
    for required in (
        "GRASP_GEOMETRY_FILENAME",
        "GRIPPER_CLOSED_RAD",
        "SCENE_CONFIG_FILENAME",
        "WORLD_FILENAME",
        "trial_grasp_geometry_filename=${4:-so101_grasp_geometry.json}",
        "trial_gripper_closed_rad=${5:-0.60}",
        "trial_derive_descend_lift=${6:-false}",
        '-p grasp_geometry_filename:="$trial_grasp_geometry_filename"',
        '-p gripper_closed_position_rad:="$trial_gripper_closed_rad"',
        '-p derive_descend_and_lift_from_profile:="$trial_derive_descend_lift"',
        '-p scene_config_filename:="$trial_scene_config_filename"',
        'scene_config:="$trial_installed_scene"',
        'world_filename:="$trial_world_filename"',
        'if [ -e "$trial_artifact_dir" ]',
        'bash scripts/cleanup_ros_domain.sh "$ROS_DOMAIN_ID" --terminate',
    ):
        assert required in source
