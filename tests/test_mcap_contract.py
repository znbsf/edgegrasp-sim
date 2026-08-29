"""Static checks for the MCAP single-clock, raw-input replay contract."""

from pathlib import Path
import re


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = PROJECT_ROOT / "scripts"

RAW_TOPICS = {
    "/joint_states",
    "/edgegrasp/target_3d",
    "/edgegrasp/tracked_target",
    "/camera_head/color/image_raw",
    "/camera_head/depth/image_rect_raw",
    "/camera_head/depth/camera_info",
}
FORBIDDEN_REPLAY_TOPICS = {
    "/clock",
    "/edgegrasp/motion_allowed",
    "/edgegrasp/safety_status",
    "/edgegrasp/arm_joint_trajectory_request",
    "/edgegrasp/gripper_joint_trajectory_request",
    "/edgegrasp/trajectory_gate_status",
    "/edgegrasp/interface_ready",
    "/edgegrasp/interface_status",
    "/edgegrasp/target_cube_pose",
    "/edgegrasp/target_cube_contacts",
    "/edgegrasp/table_contacts",
    "/edgegrasp/planning_scene_status",
    "/edgegrasp/grasp_sequence_status",
    "/edgegrasp/grasp_sequence_terminal",
    "/edgegrasp/grasp_physics_status",
}
PHYSICS_EVIDENCE_RECORD_TOPICS = {
    "/gripper_controller/controller_state",
    "/edgegrasp/target_cube_pose",
    "/edgegrasp/target_cube_contacts",
    "/edgegrasp/table_contacts",
    "/edgegrasp/planning_scene_status",
    "/edgegrasp/grasp_sequence_status",
    "/edgegrasp/grasp_sequence_terminal",
    "/edgegrasp/grasp_physics_status",
}


def _topic_block(source: str, powershell: bool) -> list[str]:
    if powershell:
        match = re.search(
            r"\$RawReplayTopics\s*=\s*@\(\s*(.*?)\s*\n\s*\)",
            source,
            flags=re.DOTALL,
        )
    else:
        match = re.search(
            r"RAW_REPLAY_TOPICS\s*=\s*\(\s*(.*?)\s*\n\s*\)",
            source,
            flags=re.DOTALL,
        )
    assert match is not None, "replay raw-topic allowlist is missing"
    topics = []
    for line in match.group(1).splitlines():
        value = line.strip().rstrip(",")
        if value:
            topics.append(value.strip('"'))
    return topics


def test_replay_scripts_have_only_the_raw_input_allowlist() -> None:
    for name in ("replay_mcap.sh", "replay_mcap.ps1"):
        source = (SCRIPTS / name).read_text(encoding="utf-8")
        topics = _topic_block(source, name.endswith(".ps1"))
        assert set(topics) == RAW_TOPICS
        assert not FORBIDDEN_REPLAY_TOPICS.intersection(topics)
        assert "--clock" in source
        assert "--topics" in source
        assert "bag play -s" not in source
        assert "topic info /clock" in source
        assert "sole /clock authority" in source
        assert "edgegrasp_replay" in source


def test_record_scripts_separate_sim_and_system_time_profiles() -> None:
    bash_source = (SCRIPTS / "record_mcap.sh").read_text(encoding="utf-8")
    bash_match = re.search(
        r'if \[\[ "\$use_sim_time" == true \]\]; then(?P<sim>.*?)else(?P<system>.*?)fi',
        bash_source,
        flags=re.DOTALL,
    )
    assert bash_match is not None
    assert "ros2 topic echo /clock --once" in bash_match.group("sim")
    assert "--use-sim-time" in bash_match.group("sim")
    assert "--use-sim-time" not in bash_match.group("system")

    powershell_source = (SCRIPTS / "record_mcap.ps1").read_text(encoding="utf-8")
    powershell_match = re.search(
        r"if \(\$simProfile\) \{(?P<sim>.*?)\}\s*else \{(?P<system>.*?)\}",
        powershell_source,
        flags=re.DOTALL,
    )
    assert powershell_match is not None
    assert "ros2 topic echo /clock --once" in powershell_match.group("sim")
    assert "--use-sim-time" in powershell_match.group("sim")
    assert "--use-sim-time" not in powershell_match.group("system")
    for topic in PHYSICS_EVIDENCE_RECORD_TOPICS:
        assert topic in bash_source
        assert topic in powershell_source


def test_shell_arguments_preserve_paths_with_spaces() -> None:
    bash_record = (SCRIPTS / "record_mcap.sh").read_text(encoding="utf-8")
    bash_replay = (SCRIPTS / "replay_mcap.sh").read_text(encoding="utf-8")
    assert 'dirname "$output_dir"' in bash_record
    assert '"$output_dir"' in bash_record
    assert '"$1"' in bash_replay

    powershell_record = (SCRIPTS / "record_mcap.ps1").read_text(encoding="utf-8")
    powershell_replay = (SCRIPTS / "replay_mcap.ps1").read_text(encoding="utf-8")
    assert '"$OutputDirectory"' in powershell_record
    assert '"$BagDirectory"' in powershell_replay


def test_ros_only_fixture_time_domain_validator_is_present_and_syntax_valid() -> None:
    source = (SCRIPTS / "validate_mcap_time_domain.py").read_text(
        encoding="utf-8"
    )
    compile(source, "validate_mcap_time_domain.py", "exec")
    assert "receive_timestamp_ns" in source
    assert "message.header.stamp" in source
    assert "/edgegrasp/target_3d" in source
    assert '"status": "PASS" if count > 0 and violations == 0' in source
