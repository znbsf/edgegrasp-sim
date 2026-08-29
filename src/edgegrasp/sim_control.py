"""EdgeGrasp-owned Gazebo control overlays for one bounded grasp experiment.

The pinned SO-101 checkout stays untouched.  This module transforms only an
already-expanded, generated URDF and fails closed unless the observed upstream
ros2_control contract is exactly the one audited at the pinned commit.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET


POSITION_ONLY_PROFILE = "position_only"
POSITION_EFFORT_PRELOAD_PROFILE = "position_effort_preload"
EFFORT_PID_PRELOAD_PROFILE = "effort_pid_preload"
GRIPPER_PRELOAD_EFFORT_LIMIT_NM = 0.5


class SimControlOverlayError(ValueError):
    """Raised when a generated control description is unsafe or has drifted."""


@dataclass(frozen=True, slots=True)
class GripperControlOverlayReport:
    profile: str
    joint_name: str
    command_interfaces_before: tuple[str, ...]
    command_interfaces_after: tuple[str, ...]
    state_interfaces: tuple[str, ...]
    effort_limit_nm: float
    controller_config_path: str


def _validated_gripper_control(
    urdf_xml: str,
    *,
    controller_config_path: Path,
) -> tuple[
    ET.Element,
    ET.Element,
    tuple[str, ...],
    tuple[str, ...],
    ET.Element,
]:
    config_path = Path(controller_config_path)
    if not config_path.is_file():
        raise SimControlOverlayError(
            f"EdgeGrasp controller config is missing: {config_path}"
        )
    try:
        root = ET.fromstring(urdf_xml)
    except ET.ParseError as error:
        raise SimControlOverlayError("generated URDF is not valid XML") from error

    systems = [
        item
        for item in root.findall("./ros2_control")
        if item.get("type") == "system"
    ]
    if len(systems) != 1:
        raise SimControlOverlayError("expected exactly one ros2_control system")
    joints = [
        item for item in systems[0].findall("./joint") if item.get("name") == "gripper"
    ]
    if len(joints) != 1:
        raise SimControlOverlayError("expected exactly one gripper control joint")
    joint = joints[0]
    commands_before = tuple(
        item.get("name", "") for item in joint.findall("./command_interface")
    )
    states = tuple(
        item.get("name", "") for item in joint.findall("./state_interface")
    )
    if commands_before != ("position",):
        raise SimControlOverlayError(
            f"pinned gripper command-interface drift: {commands_before}"
        )
    if states != ("position", "velocity", "effort"):
        raise SimControlOverlayError(
            f"pinned gripper state-interface drift: {states}"
        )

    plugins = [
        item
        for item in root.findall("./gazebo/plugin")
        if item.get("name") == "gz_ros2_control::GazeboSimROS2ControlPlugin"
    ]
    if len(plugins) != 1:
        raise SimControlOverlayError(
            "expected exactly one gz_ros2_control Gazebo plugin"
        )
    parameters = plugins[0].findall("./parameters")
    if len(parameters) != 1 or not (parameters[0].text or "").strip():
        raise SimControlOverlayError("gz_ros2_control parameters path is unavailable")
    parameters[0].text = str(config_path)
    return root, joint, commands_before, states, parameters[0]


def _validated_effort_limit(effort_limit_nm: float) -> float:
    limit = float(effort_limit_nm)
    if not math.isfinite(limit) or not 0.0 < limit <= GRIPPER_PRELOAD_EFFORT_LIMIT_NM:
        raise SimControlOverlayError(
            "gripper effort limit must be finite and in (0, 0.5] N m"
        )
    return limit


def _effort_command_interface(limit: float) -> ET.Element:
    effort = ET.Element("command_interface", {"name": "effort"})
    ET.SubElement(effort, "param", {"name": "min"}).text = f"{-limit:.12g}"
    ET.SubElement(effort, "param", {"name": "max"}).text = f"{limit:.12g}"
    return effort


def apply_gripper_position_effort_overlay(
    urdf_xml: str,
    *,
    controller_config_path: Path,
    effort_limit_nm: float = GRIPPER_PRELOAD_EFFORT_LIMIT_NM,
) -> tuple[str, GripperControlOverlayReport]:
    """Add bounded position+effort control to the generated Gazebo gripper.

    JointTrajectoryController keeps the pinned FollowJointTrajectory endpoint.
    Its optional trajectory effort becomes a bounded feed-forward torque while
    position remains the commanded jaw angle.  Runtime evidence is still
    required; this static transform alone proves no holding force.
    """

    limit = _validated_effort_limit(effort_limit_nm)
    root, joint, commands_before, states, _ = _validated_gripper_control(
        urdf_xml, controller_config_path=controller_config_path
    )
    effort = _effort_command_interface(limit)
    command_nodes = joint.findall("./command_interface")
    joint.insert(list(joint).index(command_nodes[-1]) + 1, effort)

    commands_after = tuple(
        item.get("name", "") for item in joint.findall("./command_interface")
    )
    if commands_after != ("position", "effort"):
        raise SimControlOverlayError("generated gripper command interfaces are invalid")

    return ET.tostring(root, encoding="unicode"), GripperControlOverlayReport(
        profile=POSITION_EFFORT_PRELOAD_PROFILE,
        joint_name="gripper",
        command_interfaces_before=commands_before,
        command_interfaces_after=commands_after,
        state_interfaces=states,
        effort_limit_nm=limit,
        controller_config_path=str(Path(controller_config_path)),
    )


def apply_gripper_effort_pid_overlay(
    urdf_xml: str,
    *,
    controller_config_path: Path,
    effort_limit_nm: float = GRIPPER_PRELOAD_EFFORT_LIMIT_NM,
) -> tuple[str, GripperControlOverlayReport]:
    """Use an effort-only Gazebo gripper with JTC's position-error PID.

    ``gz_ros2_control`` Jazzy's default system prioritizes the position branch
    when position and effort are both claimed for one joint.  The previous
    position+effort experiment therefore could not prove an applied torque.
    This profile claims only effort, so JTC converts the desired gripper
    position into bounded PID effort while retaining the pinned
    FollowJointTrajectory action endpoint.
    """

    limit = _validated_effort_limit(effort_limit_nm)
    root, joint, commands_before, states, _ = _validated_gripper_control(
        urdf_xml, controller_config_path=controller_config_path
    )
    command_node = joint.findall("./command_interface")[0]
    command_index = list(joint).index(command_node)
    joint.remove(command_node)
    joint.insert(command_index, _effort_command_interface(limit))
    commands_after = tuple(
        item.get("name", "") for item in joint.findall("./command_interface")
    )
    if commands_after != ("effort",):
        raise SimControlOverlayError("generated effort-only command interface is invalid")

    return ET.tostring(root, encoding="unicode"), GripperControlOverlayReport(
        profile=EFFORT_PID_PRELOAD_PROFILE,
        joint_name="gripper",
        command_interfaces_before=commands_before,
        command_interfaces_after=commands_after,
        state_interfaces=states,
        effort_limit_nm=limit,
        controller_config_path=str(Path(controller_config_path)),
    )
