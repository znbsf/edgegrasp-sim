from pathlib import Path
import xml.etree.ElementTree as ET

from edgegrasp.config import DEFAULT_TARGET_FRAME, ROS_SIM_CLOCK_DOMAIN


PROJECT_ROOT = Path(__file__).resolve().parents[1]
ROS_WS = PROJECT_ROOT / "ros_ws" / "src"
INTERFACES = ROS_WS / "edgegrasp_interfaces"
PACKAGE = ROS_WS / "edgegrasp_grasp_sequence"


def test_grasp_sequence_action_keeps_sequence_and_physics_evidence_separate() -> None:
    interface = (INTERFACES / "action" / "GraspSequence.action").read_text(
        encoding="utf-8"
    )
    cmake = (INTERFACES / "CMakeLists.txt").read_text(encoding="utf-8")

    assert '"action/GraspSequence.action"' in cmake
    assert "edgegrasp_interfaces/TrackedTarget target" in interface
    for field in ("approach_position", "descend_position", "lift_position"):
        assert f"geometry_msgs/Point {field}" in interface
    assert "geometry_msgs/Quaternion approach_orientation" in interface
    assert "geometry_msgs/Quaternion grasp_orientation" in interface
    assert "bool sequence_completed" in interface
    assert "bool physics_grasp_verified" in interface
    assert "string active_command_id" in interface
    assert "string last_terminal_command_id" in interface
    assert "string last_trajectory_digest" in interface


def test_grasp_sequence_ros_package_metadata_and_entrypoint_are_complete() -> None:
    package = ET.parse(PACKAGE / "package.xml").getroot()
    assert package.findtext("name") == "edgegrasp_grasp_sequence"
    dependencies = {item.text for item in package.findall("exec_depend")}
    assert {
        "action_msgs",
        "ament_index_python",
        "edgegrasp_core",
        "edgegrasp_interfaces",
        "edgegrasp_ros",
        "rclpy",
        "sensor_msgs",
        "trajectory_msgs",
    } <= dependencies
    assert "python3-pytest" in {
        item.text for item in package.findall("test_depend")
    }
    setup = (PACKAGE / "setup.py").read_text(encoding="utf-8")
    assert 'tests_require=["pytest"]' in setup
    assert "edgegrasp_grasp_sequence.node:main" in setup
    assert "edgegrasp_grasp_sequence.client:main" in setup
    assert "edgegrasp_grasp_sequence.trial_client:main" in setup


def test_wrapper_uses_only_correlated_typed_action_boundaries() -> None:
    source = (
        PACKAGE / "edgegrasp_grasp_sequence" / "node.py"
    ).read_text(encoding="utf-8")

    assert "GraspSequenceController" in source
    assert "ActionClient" in source
    assert "PlanTarget" in source
    assert "ExecuteTrajectory" in source
    assert 'goal.planning_group = "arm"' in source
    assert "goal.plan_only = True" in source
    assert 'goal.controller = "gripper_controller"' in source
    assert "plan_action_goal_status=int(wrapped.status)" in source
    assert "gate_action_goal_status=int(wrapped.status)" in source
    assert "downstream_terminal_observed" in source
    assert "trajectory_digest" in source
    assert "physics_grasp_verified = False" in source
    for callback_group in (
        "_target_callback_group",
        "_permission_callback_group",
        "_interface_callback_group",
        "_joint_state_callback_group",
    ):
        assert f"callback_group=self.{callback_group}" in source
    assert "_input_callback_group" not in source
    for parameter, timeout_ms in (
        ("permission_timeout_ms", 100.0),
        ("interface_timeout_ms", 500.0),
        ("target_receive_timeout_ms", 200.0),
        ("joint_state_timeout_ms", 250.0),
    ):
        assert f'self.declare_parameter("{parameter}", {timeout_ms})' in source
        assert f"{parameter}=self._{parameter}" in source
    assert "DEFAULT_ROS_FUTURE_SKEW_TOLERANCE_MS" in source
    assert "ros_clock_uninitialized" in source
    assert 'if reason == "ros_clock_uninitialized"' in source
    assert "target_source_timestamp_future_exceeds_delivery_window" in source
    assert "target_source_clock_catchup_timeout" in source
    assert "while self.context.ok() and time.monotonic() < deadline" in source
    assert "MultiThreadedExecutor" in source
    assert "executor._executor.shutdown(wait=True, cancel_futures=False)" in source
    assert "error = task.exception()" in source
    assert "except Exception:" in source
    assert "if rclpy.ok():\n            raise" in source
    assert "rclpy.spin(node)" not in source
    assert "/edgegrasp/arm_joint_trajectory_request" not in source
    assert "/edgegrasp/gripper_joint_trajectory_request" not in source


def test_wrapper_launch_defaults_match_sim_clock_and_base_frame_contract() -> None:
    launch = (PACKAGE / "launch" / "grasp_sequence.launch.py").read_text(
        encoding="utf-8"
    )
    assert DEFAULT_TARGET_FRAME == "base_link"
    assert ROS_SIM_CLOCK_DOMAIN == "ros_sim"
    assert "DEFAULT_TARGET_FRAME" in launch
    assert "ROS_SIM_CLOCK_DOMAIN" in launch
    assert 'DeclareLaunchArgument("use_sim_time", default_value="true")' in launch
    assert '"use_sim_time": use_sim_time' in launch


def test_one_shot_client_snapshots_tracked_target_and_reports_evidence_boundary() -> None:
    source = (
        PACKAGE / "edgegrasp_grasp_sequence" / "client.py"
    ).read_text(encoding="utf-8")
    assert "TrackedTarget" in source
    assert '"/edgegrasp/tracked_target"' in source
    assert "goal.target = self._latest" in source
    assert '"approach_orientation_xyzw"' in source
    assert '"grasp_orientation_xyzw"' in source
    assert "goal.approach_orientation" in source
    assert "goal.grasp_orientation" in source
    assert '"sequence_completed"' in source
    assert '"physics_grasp_verified"' in source
    assert '"last_terminal_command_id"' in source
    assert '"last_trajectory_digest"' in source
    assert "cancel_goal_async" in source


def test_trial_client_binds_one_target_to_sequence_and_read_only_evidence() -> None:
    source = (
        PACKAGE / "edgegrasp_grasp_sequence" / "trial_client.py"
    ).read_text(encoding="utf-8")
    assert "GraspPhysicsEvidence" in source
    assert "GraspSequence" in source
    assert "goal.target = target" in source
    assert "goal.target_source_timestamp_ns = self._stamp_ns(" in source
    assert "target.observation.header.stamp" in source
    assert "make_trajectory_command_id(" in source
    assert 'task_id, "lift", 3' in source
    assert "physics_baseline_unavailable" in source
    assert "physics_grasp_verified" in source
    assert "load_grasp_geometry_profile" in source
    assert '"grasp_geometry_filename"' in source
    assert "_config_basename" in source
    assert '"grasp_geometry_filename": self._grasp_geometry_filename' in source
    assert "validate_routed_grasp_stage_geometry" in source
    assert "self._validate_physics_trial_geometry(target)" in source
    assert "self._physics.send_goal_async" in source
    assert "self._sequence.send_goal_async" in source
    assert "def _cancel_and_confirm(" in source
    assert '"terminal_observed"' in source
    assert '"cancel_accepted_terminal_unconfirmed"' in source
    assert "physics_handle, physics_result_future" in source
    assert '"max_baseline_restarts"' in source
    assert '"physics_baseline_target_stale"' in source
    assert '"physics_baseline_restart_terminal_unconfirmed"' in source
    assert source.index("self._physics.send_goal_async") < source.index(
        "self._sequence.send_goal_async"
    )
    assert source.index("self._validate_physics_trial_geometry(target)") < (
        source.index("self._physics.send_goal_async")
    )
    assert source.index("if self._target_age_is_recent(target_age_ns):") < source.index(
        "self._sequence.send_goal_async"
    )
    for forbidden in (
        "JointTrajectory",
        "FollowJointTrajectory",
        "/arm_controller",
        "/gripper_controller",
    ):
        assert forbidden not in source
