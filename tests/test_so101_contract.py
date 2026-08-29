import json
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest

from edgegrasp.so101_contract import (
    SO101_ARM_ACTION,
    SO101_ARM_JOINTS,
    SO101_CAMERA_COLOR_TOPIC,
    SO101_CAMERA_DEPTH_TOPIC,
    SO101_CAMERA_INFO_TOPIC,
    SO101_CONTRACT,
    SO101_GRIPPER_ACTION,
    SO101_GRIPPER_JOINTS,
    SO101_PLANNING_PIPELINES,
    TrajectoryContractReason,
    load_so101_contract,
    validate_trajectory_contract,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PROJECT_ROOT.parents[1]
LOCAL_CHECKOUT = REPOSITORY_ROOT / "workspaces" / "forks" / "so101_ros2"
PIN = "0305e03ab54e64aae9263fcbf339622e654012f3"


def point(
    positions: list[float],
    time_from_start_ns: int = 1_000_000_000,
    **optional: list[float],
) -> dict[str, object]:
    return {
        "positions": positions,
        "time_from_start_ns": time_from_start_ns,
        **optional,
    }


def test_pinned_contract_resource_is_machine_readable_and_exact() -> None:
    contract = load_so101_contract()
    assert contract is SO101_CONTRACT
    assert json.loads(json.dumps(contract))["commit"] == PIN
    assert contract["verification"] == {
        "observed_at": "2026-08-26T17:40:00+08:00",
        "local_checkout_relative_to_repository_root": "workspaces/forks/so101_ros2",
        "local_checkout_static_verified": True,
        "static_path_and_text_verified": True,
        "ros_build_verified": True,
        "runtime_verified": True,
        "runtime_full_stack_verified": False,
        "verified_runtime_scopes": [
            "Gazebo Harmonic empty.world controller activation and six-joint /joint_states",
            "arm and gripper FollowJointTrajectory action discovery and conservative gated execution",
            "MoveIt move_group startup and GetMotionPlan joint-space planning",
            "EdgeGrasp PlanTarget through IK, MoveGroup plan_only, trajectory gate, and arm controller",
            "simulated camera topic publication in pick_and_place.world",
        ],
        "status": "PARTIAL_RUNTIME_VERIFIED_COLLISION_GRASP_UNVERIFIED",
    }
    assert SO101_ARM_JOINTS == (
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
    )
    assert SO101_GRIPPER_JOINTS == ("gripper",)
    assert SO101_ARM_ACTION == "/arm_controller/follow_joint_trajectory"
    assert SO101_GRIPPER_ACTION == "/gripper_controller/follow_joint_trajectory"
    assert SO101_CAMERA_COLOR_TOPIC == "/camera_head/color/image_raw"
    assert SO101_CAMERA_DEPTH_TOPIC == "/camera_head/depth/image_rect_raw"
    assert SO101_CAMERA_INFO_TOPIC == "/camera_head/depth/camera_info"
    assert SO101_PLANNING_PIPELINES == (
        "ompl",
        "pilz_industrial_motion_planner",
        "stomp",
    )


def test_valid_arm_and_gripper_trajectories_match_pinned_contract() -> None:
    arm = validate_trajectory_contract(
        "arm_controller",
        SO101_ARM_JOINTS,
        [point([0.0, -0.1, 0.2, -0.1, 0.0])],
    )
    gripper = validate_trajectory_contract(
        "gripper_controller",
        SO101_GRIPPER_JOINTS,
        [point([0.2], velocities=[0.0], effort=[-0.25])],
    )
    assert arm.accepted and arm.reason is TrajectoryContractReason.VALID
    assert gripper.accepted and gripper.reason is TrajectoryContractReason.VALID


def test_conservative_admission_profile_is_explicitly_project_owned() -> None:
    profile = SO101_CONTRACT["edgegrasp_admission_profile"]
    assert profile["profile_kind"] == "conservative_project_gate_not_upstream_limits"
    assert profile["max_trajectory_duration_s"] == 10.0
    assert profile["max_segment_velocity_rad_s"] == 1.0
    assert profile["max_gripper_feedforward_effort_nm"] == 0.5
    assert profile["joint_position_bounds_rad"]["gripper"] == [0.0, 1.5]


@pytest.mark.parametrize(
    ("controller", "joint_names", "points", "reason"),
    [
        (
            "unknown_controller",
            SO101_ARM_JOINTS,
            [point([0.0] * 5)],
            TrajectoryContractReason.UNKNOWN_CONTROLLER,
        ),
        (
            "arm_controller",
            tuple(reversed(SO101_ARM_JOINTS)),
            [point([0.0] * 5)],
            TrajectoryContractReason.JOINT_ORDER_MISMATCH,
        ),
        (
            "gripper_controller",
            SO101_GRIPPER_JOINTS,
            [],
            TrajectoryContractReason.EMPTY_TRAJECTORY,
        ),
        (
            "arm_controller",
            SO101_ARM_JOINTS,
            [point([0.0] * 4)],
            TrajectoryContractReason.POSITION_LENGTH_MISMATCH,
        ),
        (
            "arm_controller",
            SO101_ARM_JOINTS,
            [point([0.0] * 5, velocities=[0.0] * 4)],
            TrajectoryContractReason.OPTIONAL_VECTOR_LENGTH_MISMATCH,
        ),
        (
            "arm_controller",
            SO101_ARM_JOINTS,
            [point([0.0, 0.0, math.nan, 0.0, 0.0])],
            TrajectoryContractReason.NONFINITE_VALUE,
        ),
        (
            "gripper_controller",
            SO101_GRIPPER_JOINTS,
            [point([0.2], time_from_start_ns=0)],
            TrajectoryContractReason.INVALID_TIME,
        ),
        (
            "gripper_controller",
            SO101_GRIPPER_JOINTS,
            [point([0.1], 2), point([0.2], 2)],
            TrajectoryContractReason.INVALID_TIME,
        ),
        (
            "arm_controller",
            SO101_ARM_JOINTS,
            [point([1.81, 0.0, 0.0, 0.0, 0.0])],
            TrajectoryContractReason.POSITION_LIMIT_EXCEEDED,
        ),
        (
            "gripper_controller",
            SO101_GRIPPER_JOINTS,
            [point([0.2], time_from_start_ns=10_000_000_001)],
            TrajectoryContractReason.TRAJECTORY_DURATION_EXCEEDED,
        ),
        (
            "gripper_controller",
            SO101_GRIPPER_JOINTS,
            [point([0.2], velocities=[1.01])],
            TrajectoryContractReason.VELOCITY_LIMIT_EXCEEDED,
        ),
        (
            "gripper_controller",
            SO101_GRIPPER_JOINTS,
            [point([0.2], accelerations=[2.51])],
            TrajectoryContractReason.ACCELERATION_LIMIT_EXCEEDED,
        ),
    ],
)
def test_invalid_trajectory_contracts_fail_closed(
    controller: str,
    joint_names: tuple[str, ...],
    points: list[dict[str, object]],
    reason: TrajectoryContractReason,
) -> None:
    decision = validate_trajectory_contract(controller, joint_names, points)
    assert not decision.accepted
    assert decision.reason is reason


def test_feedforward_effort_is_bounded_and_gripper_only() -> None:
    arm = validate_trajectory_contract(
        "arm_controller",
        SO101_ARM_JOINTS,
        [point([0.0] * 5, effort=[0.1] * 5)],
    )
    excessive = validate_trajectory_contract(
        "gripper_controller",
        SO101_GRIPPER_JOINTS,
        [point([0.2], effort=[0.500001])],
    )
    assert arm.reason is TrajectoryContractReason.EFFORT_NOT_ALLOWED
    assert excessive.reason is TrajectoryContractReason.EFFORT_LIMIT_EXCEEDED


def test_start_state_enables_conservative_implied_segment_velocity_gate() -> None:
    too_fast = validate_trajectory_contract(
        "arm_controller",
        SO101_ARM_JOINTS,
        [point([0.2, 0.0, 0.0, 0.0, 0.0], time_from_start_ns=100_000_000)],
        start_positions=[0.0] * 5,
    )
    assert not too_fast.accepted
    assert too_fast.reason is TrajectoryContractReason.VELOCITY_LIMIT_EXCEEDED

    valid = validate_trajectory_contract(
        "arm_controller",
        SO101_ARM_JOINTS,
        [point([0.2, 0.0, 0.0, 0.0, 0.0], time_from_start_ns=500_000_000)],
        start_positions=[0.0] * 5,
    )
    assert valid.accepted


def test_start_state_length_mismatch_fails_closed() -> None:
    decision = validate_trajectory_contract(
        "arm_controller",
        SO101_ARM_JOINTS,
        [point([0.0] * 5)],
        start_positions=[0.0] * 4,
    )
    assert not decision.accepted
    assert decision.reason is TrajectoryContractReason.START_STATE_LENGTH_MISMATCH


def test_local_pinned_checkout_matches_committed_contract_when_available() -> None:
    verification = SO101_CONTRACT["verification"]
    if not LOCAL_CHECKOUT.is_dir():
        assert verification["runtime_full_stack_verified"] is False
        return

    commit = subprocess.run(
        ["git", "-C", str(LOCAL_CHECKOUT), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
        timeout=10,
    ).stdout.strip()
    status = subprocess.run(
        ["git", "-C", str(LOCAL_CHECKOUT), "status", "--porcelain"],
        check=True,
        capture_output=True,
        text=True,
        timeout=10,
    ).stdout.strip()
    assert commit == PIN
    assert status == ""
    for relative_path in SO101_CONTRACT["source_paths"]:
        assert (LOCAL_CHECKOUT / relative_path).is_file(), relative_path

    srdf = ET.parse(
        LOCAL_CHECKOUT / "so101_moveit_config/config/so101/so101.srdf"
    ).getroot()
    groups = {
        group.attrib["name"]: tuple(
            joint.attrib["name"] for joint in group.findall("joint")
        )
        for group in srdf.findall("group")
    }
    assert groups["arm"] == SO101_ARM_JOINTS
    assert groups["gripper"] == SO101_GRIPPER_JOINTS
    assert srdf.find("./virtual_joint[@parent_frame='world'][@child_link='base_link']") is not None
    assert srdf.find("./end_effector[@parent_link='gripper_frame_link']") is not None

    moveit_mapping = (
        LOCAL_CHECKOUT
        / "so101_moveit_config/config/so101/moveit_controllers.yaml"
    ).read_text(encoding="utf-8")
    assert "action_ns: follow_joint_trajectory" in moveit_mapping
    assert "type: FollowJointTrajectory" in moveit_mapping
    assert moveit_mapping.index("arm_controller:") < moveit_mapping.index(
        "gripper_controller:"
    )

    controller_yaml = (
        LOCAL_CHECKOUT / "so101_moveit_config/config/so101/ros2_controllers.yaml"
    ).read_text(encoding="utf-8")
    assert "update_rate: 100" in controller_yaml
    assert controller_yaml.count(
        "type: joint_trajectory_controller/JointTrajectoryController"
    ) == 2

    control_xacro = ET.parse(
        LOCAL_CHECKOUT
        / "so101_description/urdf/control/so101_ros2_control.urdf.xacro"
    ).getroot()
    gripper = control_xacro.find(".//joint[@name='${prefix}gripper']")
    assert gripper is not None
    assert [item.attrib["name"] for item in gripper.findall("command_interface")] == [
        "position"
    ]
    assert [item.attrib["name"] for item in gripper.findall("state_interface")] == [
        "position",
        "velocity",
        "effort",
    ]

    load_launch = (
        LOCAL_CHECKOUT
        / "so101_moveit_config/launch/load_ros2_controllers.launch.py"
    ).read_text(encoding="utf-8")
    assert "default_value='10.0'" in load_launch
    assert "on_exit=[start_arm_controller_cmd]" in load_launch
    assert "on_exit=[start_gripper_controller_cmd]" in load_launch


def test_upstream_template_write_side_effect_is_explicitly_detected() -> None:
    if not LOCAL_CHECKOUT.is_dir():
        assert SO101_CONTRACT["verification"]["runtime_full_stack_verified"] is False
        return
    source = (
        LOCAL_CHECKOUT
        / "so101_description/launch/robot_state_publisher.launch.py"
    ).read_text(encoding="utf-8")
    assert "ros2_ws/src/so101_ros2/so101_moveit_config/config" in source
    assert "ros2_ws/install/so101_moveit_config/share" in source
    assert "with open(output_path, 'w'" in source
