from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_adapter_uses_move_group_plan_only_tf2_and_gate_topic_only() -> None:
    source = (PACKAGE_ROOT / "edgegrasp_moveit_adapter" / "adapter_node.py").read_text(
        encoding="utf-8"
    )
    interface = (
        PACKAGE_ROOT.parent / "edgegrasp_interfaces" / "action" / "PlanTarget.action"
    ).read_text(encoding="utf-8")

    assert "ActionClient" in source and "MoveGroup" in source
    assert "planning_options.plan_only = True" in source
    assert "lookup_transform" in source and "Time.from_msg" in source
    assert "validate_and_convert_robot_trajectory" in source
    assert "/edgegrasp/execute_trajectory" in source
    assert "ExecuteTrajectory" in source
    assert "_dispatch_to_gate" in source
    assert "SO101_PLANNING_PIPELINES" in source
    assert '"pilz"' not in source
    assert "trajectory_gate_result_correlation_mismatch" in source
    assert "/edgegrasp/planning_scene_ready" in source
    assert "_planning_scene_permission" in source
    assert "/edgegrasp/arm_joint_trajectory_request" not in source
    execute_interface = (
        PACKAGE_ROOT.parent
        / "edgegrasp_interfaces"
        / "action"
        / "ExecuteTrajectory.action"
    ).read_text(encoding="utf-8")
    assert "trajectory_msgs/JointTrajectory trajectory" in execute_interface
    assert "string command_id" in execute_interface
    assert "string trajectory_digest" in execute_interface
    assert execute_interface.count("int64 source_timestamp_ns") == 2
    assert "/arm_controller/follow_joint_trajectory" not in source
    assert "geometry_msgs/PoseStamped target_pose" in interface
    assert "source_timestamp_ns" in interface
    assert "clock_domain" in interface and "clock_epoch" in interface
    assert "string task_id" in interface
    assert "string stage" in interface
    assert "uint64 sequence_no" in interface
    assert "bool trajectory_dispatched" in interface
    assert "bool gate_terminal" in interface
    assert "int32 fjt_error_code" in interface


def test_launch_exposes_bounded_ik_wall_guard_separately_from_freshness() -> None:
    launch = (PACKAGE_ROOT / "launch" / "moveit_adapter.launch.py").read_text(
        encoding="utf-8"
    )
    source = (PACKAGE_ROOT / "edgegrasp_moveit_adapter" / "adapter_node.py").read_text(
        encoding="utf-8"
    )

    assert 'DeclareLaunchArgument(\n                "ik_response_timeout_ms"' in launch
    assert 'default_value="5000.0"' in launch
    assert '"ik_response_timeout_ms": ik_response_timeout_ms' in launch
    assert 'declare_parameter("ik_response_timeout_ms", 5000.0)' in source
    assert 'default_value="120000.0"' in launch
    assert '"gate_terminal_wall_guard_ms": (' in launch
    assert 'declare_parameter("gate_terminal_wall_guard_ms", 120000.0)' in source
    assert "target_freshness_timeout_ms" in source
    assert "require_source_freshness" in source
