import ast
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from edgegrasp.config import (
    DEFAULT_TARGET_FRAME,
    ROS_SIM_CLOCK_DOMAIN,
    ROS_SYSTEM_CLOCK_DOMAIN,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
ROS_WS = PROJECT_ROOT / "ros_ws"
ROS_PACKAGE = ROS_WS / "src" / "edgegrasp_ros"
CORE_PACKAGE = ROS_WS / "src" / "edgegrasp_core"
INTERFACES_PACKAGE = ROS_WS / "src" / "edgegrasp_interfaces"
MOVEIT_ADAPTER_PACKAGE = ROS_WS / "src" / "edgegrasp_moveit_adapter"
GRASP_SEQUENCE_PACKAGE = ROS_WS / "src" / "edgegrasp_grasp_sequence"


def test_ros_python_sources_are_syntax_valid_without_importing_ros() -> None:
    python_files = sorted(ROS_WS.rglob("*.py"))
    assert python_files
    for path in python_files:
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


def test_ros_packages_and_world_are_well_formed_xml() -> None:
    ros_package = ET.parse(ROS_PACKAGE / "package.xml").getroot()
    core_package = ET.parse(CORE_PACKAGE / "package.xml").getroot()
    interfaces_package = ET.parse(INTERFACES_PACKAGE / "package.xml").getroot()
    adapter_package = ET.parse(MOVEIT_ADAPTER_PACKAGE / "package.xml").getroot()
    sequence_package = ET.parse(GRASP_SEQUENCE_PACKAGE / "package.xml").getroot()
    world = ET.parse(ROS_PACKAGE / "worlds" / "table_cube.sdf").getroot()

    assert ros_package.findtext("name") == "edgegrasp_ros"
    assert core_package.findtext("name") == "edgegrasp_core"
    assert interfaces_package.findtext("name") == "edgegrasp_interfaces"
    assert adapter_package.findtext("name") == "edgegrasp_moveit_adapter"
    assert sequence_package.findtext("name") == "edgegrasp_grasp_sequence"
    assert world.tag == "sdf"
    assert world.find("./world/model[@name='table']") is not None
    assert world.find("./world/model[@name='target_cube']") is not None


def test_scene_evidence_bridge_is_gazebo_to_ros_only() -> None:
    bridge = (ROS_PACKAGE / "config" / "scene_bridge.yaml").read_text(
        encoding="utf-8"
    )
    launch = (
        ROS_PACKAGE / "launch" / "edgegrasp_proxy_gazebo.launch.py"
    ).read_text(encoding="utf-8")
    package = ET.parse(ROS_PACKAGE / "package.xml").getroot()
    dependencies = {item.text for item in package.findall("exec_depend")}

    assert "/model/target_cube/pose" in bridge
    assert "/edgegrasp/target_cube_pose" in bridge
    assert "geometry_msgs/msg/PoseStamped" in bridge
    assert "gz.msgs.Pose" in bridge
    assert "dynamic_pose/info" not in bridge
    assert "tf2_msgs/msg/TFMessage" not in bridge
    assert "/edgegrasp/target_cube_contacts" in bridge
    assert "ros_gz_interfaces/msg/Contacts" in bridge
    assert "gz.msgs.Contacts" in bridge
    assert bridge.count("direction: GZ_TO_ROS") == 2
    assert "GZ_TO_ROS" in bridge and "ROS_TO_GZ" not in bridge
    assert 'name="edgegrasp_scene_evidence_bridge"' in launch
    assert '"scene_bridge.yaml"' in launch
    assert {"geometry_msgs", "ros_gz_bridge", "ros_gz_interfaces"} <= dependencies


def test_physics_observer_is_typed_read_only_and_launched() -> None:
    source = (
        ROS_PACKAGE / "edgegrasp_ros" / "grasp_physics_observer.py"
    ).read_text(encoding="utf-8")
    setup = (ROS_PACKAGE / "setup.py").read_text(encoding="utf-8")
    launch = (
        ROS_PACKAGE / "launch" / "edgegrasp_proxy_gazebo.launch.py"
    ).read_text(encoding="utf-8")
    cmake = (INTERFACES_PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "GraspPhysicsEvidence" in source
    assert "GraspSequenceTerminal" in source
    assert "ActionServer" in source
    assert "ActionClient" not in source
    assert "JointTrajectory" not in source
    assert "create_publisher(\n            String" in source
    assert "grasp_physics_observer =" in setup
    assert 'executable="grasp_physics_observer"' in launch
    assert 'DeclareLaunchArgument("launch_physics_observer"' in launch
    assert '"action/GraspPhysicsEvidence.action"' in cmake
    assert '"msg/GraspSequenceTerminal.msg"' in cmake
    assert '"fixed_finger_pad"' in source
    assert '"moving_finger_pad"' in source
    assert '"so101::gripper"' not in source
    assert '"so101::moving_jaw_so101_v1_link"' not in source
    assert "def _drain_pending_observations(self) -> None:" in source
    assert "_drain_pending_observations(now_ns)" not in source
    assert "now_ns = self._now_ns()\n                core = self._core" in source
    assert "result.pose_sample_count // 20" in source


def test_overlay_is_thin_and_does_not_duplicate_upstream_moveit() -> None:
    overlay = (
        ROS_PACKAGE / "launch" / "adoodevv_gazebo_overlay.launch.py"
    ).read_text(encoding="utf-8")

    assert "so101.gazebo.launch.py" in overlay
    assert "so101_gazebo_moveit.launch.py" not in overlay
    assert "move_group.launch.py" not in overlay
    assert "STATIC_PATH_AND_TEXT_VERIFIED" not in overlay
    assert not any(MOVEIT_ADAPTER_PACKAGE.rglob("*.srdf"))
    assert not any(MOVEIT_ADAPTER_PACKAGE.rglob("moveit_controllers.yaml"))


def test_moveit_adapter_is_plan_only_and_publishes_only_through_gate() -> None:
    source = (
        MOVEIT_ADAPTER_PACKAGE
        / "edgegrasp_moveit_adapter"
        / "adapter_node.py"
    ).read_text(encoding="utf-8")
    interface = (
        INTERFACES_PACKAGE / "action" / "PlanTarget.action"
    ).read_text(encoding="utf-8")

    assert "GetPositionIK" in source and '"/compute_ik"' in source
    assert "MoveGroup" in source and '"/move_action"' in source
    assert "planning_options.plan_only = True" in source
    assert "validate_and_convert_robot_trajectory" in source
    assert '"/edgegrasp/execute_trajectory"' in source
    assert "ExecuteTrajectory" in source
    assert "_dispatch_to_gate" in source
    assert "trajectory_gate_result_correlation_mismatch" in source
    assert '"gate_terminal_wall_guard_ms", 120000.0' in source
    assert "self._gate_terminal_wall_guard_s" in source
    assert "duration_ns / 1e9 + self._gate_result_margin_s" in source
    assert '"/edgegrasp/arm_joint_trajectory_request"' not in source
    execute_interface = (
        INTERFACES_PACKAGE / "action" / "ExecuteTrajectory.action"
    ).read_text(encoding="utf-8")
    assert "trajectory_msgs/JointTrajectory trajectory" in execute_interface
    assert "string command_id" in execute_interface
    assert "string trajectory_digest" in execute_interface
    assert execute_interface.count("int64 source_timestamp_ns") == 2
    assert "int8 action_goal_status" in execute_interface
    assert "int32 fjt_error_code" in execute_interface
    assert "/arm_controller/follow_joint_trajectory" not in source
    assert "geometry_msgs/PoseStamped target_pose" in interface
    assert "source_timestamp_ns" in interface
    assert "clock_domain" in interface and "clock_epoch" in interface
    observe_now = source[source.index("    def _observe_now") : source.index(
        "    def _latch_fault"
    )]
    assert observe_now.index("with self._state_lock") < observe_now.index(
        "self.get_clock().now().nanoseconds"
    )
    assert "executor._executor.shutdown(wait=True, cancel_futures=False)" in source
    assert "except Exception:" in source
    assert "if rclpy.ok():\n            raise" in source


def test_mock_and_gazebo_launch_clock_and_frame_defaults_are_consistent() -> None:
    mock = (ROS_PACKAGE / "launch" / "edgegrasp_mock.launch.py").read_text(
        encoding="utf-8"
    )
    gazebo = (
        ROS_PACKAGE / "launch" / "adoodevv_gazebo_overlay.launch.py"
    ).read_text(encoding="utf-8")

    assert DEFAULT_TARGET_FRAME == "base_link"
    assert "DEFAULT_TARGET_FRAME" in mock and "DEFAULT_TARGET_FRAME" in gazebo
    assert "ROS_SYSTEM_CLOCK_DOMAIN" in mock
    assert "ROS_SIM_CLOCK_DOMAIN" in gazebo
    assert f'ROS_SYSTEM_CLOCK_DOMAIN = "{ROS_SYSTEM_CLOCK_DOMAIN}"' not in mock
    assert f'ROS_SIM_CLOCK_DOMAIN = "{ROS_SIM_CLOCK_DOMAIN}"' not in gazebo
    assert '"frame_id": target_frame' in mock and '"target_frame": target_frame' in mock
    assert '"frame_id": target_frame' in gazebo and '"target_frame": target_frame' in gazebo
    assert 'DeclareLaunchArgument("use_sim_time", default_value="false")' in mock
    assert '"use_sim_time": use_sim_time' in mock
    assert '"use_sim_time": True' in gazebo
    assert 'DeclareLaunchArgument("use_camera", default_value="true")' in gazebo


def test_trajectory_gate_keeps_ros_deadline_separate_from_wall_guard() -> None:
    source = (
        ROS_PACKAGE / "edgegrasp_ros" / "trajectory_gate.py"
    ).read_text(encoding="utf-8")
    mock = (ROS_PACKAGE / "launch" / "edgegrasp_mock.launch.py").read_text(
        encoding="utf-8"
    )
    gazebo = (
        ROS_PACKAGE / "launch" / "edgegrasp_proxy_gazebo.launch.py"
    ).read_text(encoding="utf-8")

    assert '"typed_terminal_wall_guard_ms", 90000.0' in source
    assert "deadline = time.monotonic() + self._typed_terminal_wall_guard_s" in source
    assert "_goal_duration_ns.get(request.controller, 0) / 1e9" not in source
    for launch in (mock, gazebo):
        assert (
            'DeclareLaunchArgument(\n'
            '                "typed_terminal_wall_guard_ms", default_value="90000.0"'
        ) in launch
        assert '"typed_terminal_wall_guard_ms": (' in launch
    assert '"require_camera": require_camera' in gazebo
    assert 'executable="trajectory_gate"' in mock
    assert 'executable="trajectory_gate"' in gazebo


def test_mock_target_publishes_atomic_identity_and_clock_metadata() -> None:
    interface = (
        INTERFACES_PACKAGE / "msg" / "TrackedTarget.msg"
    ).read_text(encoding="utf-8")
    cmake = (INTERFACES_PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
    source = (
        ROS_PACKAGE / "edgegrasp_ros" / "mock_target_publisher.py"
    ).read_text(encoding="utf-8")

    assert "string target_id" in interface
    assert "geometry_msgs/PointStamped observation" in interface
    assert "string clock_domain" in interface
    assert "uint64 clock_epoch" in interface
    assert '"msg/TrackedTarget.msg"' in cmake
    assert "TrackedTarget" in source
    assert '"/edgegrasp/tracked_target"' in source
    assert "tracked.target_id = self._target_id" in source
    assert "tracked.clock_domain = self._clock_domain" in source
    assert "tracked.clock_epoch = self._clock_epoch" in source


def test_ros_nodes_reject_clock_configuration_drift() -> None:
    for name in (
        "mock_target_publisher.py",
        "safety_monitor.py",
        "trajectory_gate.py",
    ):
        source = (ROS_PACKAGE / "edgegrasp_ros" / name).read_text(encoding="utf-8")
        assert "validate_ros_clock_domain" in source
        assert "clock_domain" in source
        assert "clock_epoch" in source
        assert "clock" in source.lower()

    safety = (ROS_PACKAGE / "edgegrasp_ros" / "safety_monitor.py").read_text(
        encoding="utf-8"
    )
    gate = (ROS_PACKAGE / "edgegrasp_ros" / "trajectory_gate.py").read_text(
        encoding="utf-8"
    )
    assert "clock_rollback" in safety
    assert "MotionPermissionGate" in gate


def test_trajectory_gate_is_the_documented_motion_command_boundary() -> None:
    source = (
        ROS_PACKAGE / "edgegrasp_ros" / "trajectory_gate.py"
    ).read_text(encoding="utf-8")

    assert "MotionPermissionGate" in source
    assert "/edgegrasp/arm_joint_trajectory_request" in source
    assert "/edgegrasp/gripper_joint_trajectory_request" in source
    assert "SO101_ARM_ACTION" in source
    assert "SO101_GRIPPER_ACTION" in source
    assert "send_goal_async" in source
    assert "cancel_goal_async" in source
    assert "ExecuteTrajectory" in source
    assert "ActionServer" in source
    assert "MultiThreadedExecutor" in source
    assert "executor.spin()" in source
    assert "rclpy.spin(node)" not in source
    assert "FollowJointTrajectory.Result.SUCCESSFUL" in source
    assert "trajectory_digest" in source
    assert "validate_trajectory_command_id" in source
    assert "trajectory_frame_mismatch" in source
    assert "TYPED_STAGE_CONTROLLERS" in source
    assert "downstream_terminal_observed" in source
    assert "validate_trajectory_contract" in source
    assert "/edgegrasp/interface_ready" in source
    assert "/joint_states" in source


    assert "target_stream_timeout" in source
    assert "cancel_failed_stop_escalation_required" in source
    assert "Direct goals" in source


def test_ros_adapter_uses_fjt_for_both_pinned_controllers() -> None:
    gate = (ROS_PACKAGE / "edgegrasp_ros" / "trajectory_gate.py").read_text(
        encoding="utf-8"
    )
    probe = (ROS_PACKAGE / "edgegrasp_ros" / "interface_probe.py").read_text(
        encoding="utf-8"
    )
    interfaces = (ROS_PACKAGE / "config" / "interfaces.yaml").read_text(
        encoding="utf-8"
    )

    for source in (gate, probe):
        assert "FollowJointTrajectory" in source
        assert "GripperCommand" not in source
    assert "/arm_controller/follow_joint_trajectory" in interfaces
    assert "/gripper_controller/follow_joint_trajectory" in interfaces
    assert "/gripper_controller/commands" not in interfaces
    assert "/gripper_controller/gripper_cmd" not in interfaces


def test_future_ros_fake_action_integration_test_covers_both_boundaries() -> None:
    source = (
        ROS_PACKAGE / "test" / "test_trajectory_gate_integration.py"
    ).read_text(encoding="utf-8")

    assert "ActionServer" in source
    assert "/arm_controller/follow_joint_trajectory" in source
    assert "/gripper_controller/follow_joint_trajectory" in source
    assert "positions: list[float]" in source
    assert "Bool(data=False)" in source
    assert 'servers.cancels["arm_controller"]' in source
    assert 'servers.cancels["gripper_controller"]' in source
    assert "server_is_ready" in source
    assert "time.sleep(0.05)" not in source


def test_ros_core_dependency_and_console_entrypoints_are_self_consistent() -> None:
    package = ET.parse(ROS_PACKAGE / "package.xml").getroot()
    dependencies = {item.text for item in package.findall("exec_depend")}
    assert {"edgegrasp_core", "trajectory_msgs", "std_srvs"} <= dependencies
    test_dependencies = {item.text for item in package.findall("test_depend")}
    assert "python3-pytest" in test_dependencies

    setup_source = (ROS_PACKAGE / "setup.py").read_text(encoding="utf-8")
    assert 'tests_require=["pytest"]' in setup_source
    entrypoints = re.findall(
        r'"([a-z_]+) = edgegrasp_ros\.([a-z_]+):main"', setup_source
    )
    assert entrypoints
    for executable, module in entrypoints:
        assert executable == module
        assert (ROS_PACKAGE / "edgegrasp_ros" / f"{module}.py").is_file()

    core_setup = (CORE_PACKAGE / "setup.py").read_text(encoding="utf-8")
    assert 'name="edgegrasp_core"' in core_setup
    assert 'CORE_SRC = PROJECT_ROOT / "src"' in core_setup


def test_replay_launch_and_mcap_scripts_cover_motion_boundary_topics() -> None:
    replay_launch = ROS_PACKAGE / "launch" / "edgegrasp_replay.launch.py"
    assert replay_launch.is_file()
    launch_source = replay_launch.read_text(encoding="utf-8")
    assert "ROS_SIM_CLOCK_DOMAIN" in launch_source
    assert '"use_sim_time": True' in launch_source

    for name in ("record_mcap.sh", "record_mcap.ps1"):
        source = (PROJECT_ROOT / "scripts" / name).read_text(encoding="utf-8")
        for topic in (
            "/edgegrasp/arm_joint_trajectory_request",
            "/edgegrasp/gripper_joint_trajectory_request",
            "/edgegrasp/trajectory_gate_status",
            "/edgegrasp/interface_ready",
        ):
            assert topic in source
        assert "-s mcap" in source
    for name in ("replay_mcap.sh", "replay_mcap.ps1"):
        source = (PROJECT_ROOT / "scripts" / name).read_text(encoding="utf-8")
        assert "--clock" in source
        assert "bag play -s" not in source


def test_safety_monitor_invalid_and_out_of_order_targets_latch_false() -> None:
    source = (ROS_PACKAGE / "edgegrasp_ros" / "safety_monitor.py").read_text(
        encoding="utf-8"
    )
    assert "invalid_target:" in source
    assert "source_timestamp_not_monotonic:" in source
    assert "self._latch(" in source
    assert "self._publish(False, reason)" in source
