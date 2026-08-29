"""Pinned SO-101 ROS controller contract and dependency-free validation."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from functools import lru_cache
from importlib.resources import files
import json
import math
from collections.abc import Mapping, Sequence
from typing import Any


CONTRACT_RESOURCE = "so101_ros2_0305e03.json"


@lru_cache(maxsize=1)
def load_so101_contract() -> dict[str, Any]:
    resource = files("edgegrasp.contracts").joinpath(CONTRACT_RESOURCE)
    contract = json.loads(resource.read_text(encoding="utf-8"))
    if contract.get("schema_version") != 1:
        raise ValueError("unsupported SO-101 contract schema")
    commit = contract.get("commit")
    if commit != "0305e03ab54e64aae9263fcbf339622e654012f3":
        raise ValueError("SO-101 contract pin drift")
    return contract


SO101_CONTRACT = load_so101_contract()
SO101_ARM_JOINTS = tuple(
    SO101_CONTRACT["controllers"]["arm_controller"]["joints"]
)
SO101_GRIPPER_JOINTS = tuple(
    SO101_CONTRACT["controllers"]["gripper_controller"]["joints"]
)
SO101_ARM_ACTION = SO101_CONTRACT["controllers"]["arm_controller"]["action"]
SO101_GRIPPER_ACTION = SO101_CONTRACT["controllers"]["gripper_controller"]["action"]
SO101_CAMERA_COLOR_TOPIC = SO101_CONTRACT["robot"]["camera_topics"]["color_image"]
SO101_CAMERA_DEPTH_TOPIC = SO101_CONTRACT["robot"]["camera_topics"]["depth_image"]
SO101_CAMERA_INFO_TOPIC = SO101_CONTRACT["robot"]["camera_topics"]["camera_info"]
SO101_PLANNING_PIPELINES = tuple(SO101_CONTRACT["moveit"]["planning_pipelines"])


class TrajectoryContractReason(str, Enum):
    VALID = "valid"
    UNKNOWN_CONTROLLER = "unknown_controller"
    JOINT_ORDER_MISMATCH = "joint_order_mismatch"
    EMPTY_TRAJECTORY = "empty_trajectory"
    POSITION_LENGTH_MISMATCH = "position_length_mismatch"
    OPTIONAL_VECTOR_LENGTH_MISMATCH = "optional_vector_length_mismatch"
    NONFINITE_VALUE = "nonfinite_value"
    INVALID_TIME = "invalid_time_from_start"
    POSITION_LIMIT_EXCEEDED = "position_limit_exceeded"
    TRAJECTORY_DURATION_EXCEEDED = "trajectory_duration_exceeded"
    VELOCITY_LIMIT_EXCEEDED = "velocity_limit_exceeded"
    ACCELERATION_LIMIT_EXCEEDED = "acceleration_limit_exceeded"
    EFFORT_NOT_ALLOWED = "effort_not_allowed"
    EFFORT_LIMIT_EXCEEDED = "effort_limit_exceeded"
    START_STATE_LENGTH_MISMATCH = "start_state_length_mismatch"


@dataclass(frozen=True, slots=True)
class TrajectoryContractDecision:
    accepted: bool
    reason: TrajectoryContractReason
    detail: str


def _finite_vector(values: Sequence[object]) -> bool:
    return all(
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
        for value in values
    )


def validate_trajectory_contract(
    controller_name: str,
    joint_names: Sequence[str],
    points: Sequence[Mapping[str, object]],
    *,
    start_positions: Sequence[float] | None = None,
) -> TrajectoryContractDecision:
    """Validate a trajectory shape before the ROS action boundary.

    Each point mapping must contain ``positions`` and ``time_from_start_ns``.
    Optional velocity, acceleration, and effort vectors may be empty or match
    the exact pinned joint count.
    """

    controllers = SO101_CONTRACT["controllers"]
    if controller_name not in ("arm_controller", "gripper_controller"):
        return TrajectoryContractDecision(
            False,
            TrajectoryContractReason.UNKNOWN_CONTROLLER,
            controller_name,
        )
    expected = tuple(controllers[controller_name]["joints"])
    if tuple(joint_names) != expected:
        return TrajectoryContractDecision(
            False,
            TrajectoryContractReason.JOINT_ORDER_MISMATCH,
            f"expected={expected},actual={tuple(joint_names)}",
        )
    if not points:
        return TrajectoryContractDecision(
            False,
            TrajectoryContractReason.EMPTY_TRAJECTORY,
            "at least one point is required",
        )

    profile = SO101_CONTRACT["edgegrasp_admission_profile"]
    position_bounds = profile["joint_position_bounds_rad"]
    max_duration_ns = int(float(profile["max_trajectory_duration_s"]) * 1_000_000_000)
    max_velocity = float(profile["max_segment_velocity_rad_s"])
    max_acceleration = float(profile["max_commanded_acceleration_rad_s2"])
    max_gripper_effort = float(profile["max_gripper_feedforward_effort_nm"])
    if start_positions is not None:
        if len(start_positions) != len(expected) or not _finite_vector(start_positions):
            return TrajectoryContractDecision(
                False,
                TrajectoryContractReason.START_STATE_LENGTH_MISMATCH,
                f"expected={len(expected)},actual={len(start_positions)}",
            )
        previous_positions: tuple[float, ...] | None = tuple(
            float(value) for value in start_positions
        )
        previous_time_ns = 0
    else:
        previous_positions = None
        previous_time_ns = -1
    optional_fields = ("velocities", "accelerations", "effort")
    for index, point in enumerate(points):
        positions = point.get("positions")
        if not isinstance(positions, Sequence) or isinstance(positions, (str, bytes)):
            positions = ()
        if len(positions) != len(expected):
            return TrajectoryContractDecision(
                False,
                TrajectoryContractReason.POSITION_LENGTH_MISMATCH,
                f"point={index},expected={len(expected)},actual={len(positions)}",
            )
        if not _finite_vector(positions):
            return TrajectoryContractDecision(
                False,
                TrajectoryContractReason.NONFINITE_VALUE,
                f"point={index},field=positions",
            )
        for joint, value in zip(expected, positions, strict=True):
            lower, upper = position_bounds[joint]
            if not float(lower) <= float(value) <= float(upper):
                return TrajectoryContractDecision(
                    False,
                    TrajectoryContractReason.POSITION_LIMIT_EXCEEDED,
                    f"point={index},joint={joint},value={value},bounds=({lower},{upper})",
                )
        for field in optional_fields:
            values = point.get(field, ())
            if not isinstance(values, Sequence) or isinstance(values, (str, bytes)):
                values = ()
            if len(values) not in (0, len(expected)):
                return TrajectoryContractDecision(
                    False,
                    TrajectoryContractReason.OPTIONAL_VECTOR_LENGTH_MISMATCH,
                    f"point={index},field={field},actual={len(values)}",
                )
            if values and not _finite_vector(values):
                return TrajectoryContractDecision(
                    False,
                    TrajectoryContractReason.NONFINITE_VALUE,
                    f"point={index},field={field}",
                )
            if field == "velocities" and any(
                abs(float(value)) > max_velocity for value in values
            ):
                return TrajectoryContractDecision(
                    False,
                    TrajectoryContractReason.VELOCITY_LIMIT_EXCEEDED,
                    f"point={index},field=velocities,limit={max_velocity}",
                )
            if field == "accelerations" and any(
                abs(float(value)) > max_acceleration for value in values
            ):
                return TrajectoryContractDecision(
                    False,
                    TrajectoryContractReason.ACCELERATION_LIMIT_EXCEEDED,
                    f"point={index},field=accelerations,limit={max_acceleration}",
                )
            if field == "effort" and values:
                if controller_name != "gripper_controller":
                    return TrajectoryContractDecision(
                        False,
                        TrajectoryContractReason.EFFORT_NOT_ALLOWED,
                        f"point={index},controller={controller_name}",
                    )
                if any(
                    abs(float(value)) > max_gripper_effort for value in values
                ):
                    return TrajectoryContractDecision(
                        False,
                        TrajectoryContractReason.EFFORT_LIMIT_EXCEEDED,
                        f"point={index},limit={max_gripper_effort}",
                    )
        time_ns = point.get("time_from_start_ns")
        if (
            isinstance(time_ns, bool)
            or not isinstance(time_ns, int)
            or time_ns <= previous_time_ns
            or time_ns <= 0
        ):
            return TrajectoryContractDecision(
                False,
                TrajectoryContractReason.INVALID_TIME,
                f"point={index},value={time_ns!r},previous={previous_time_ns}",
            )
        segment_start_ns = max(previous_time_ns, 0)
        previous_time_ns = time_ns
        if time_ns > max_duration_ns:
            return TrajectoryContractDecision(
                False,
                TrajectoryContractReason.TRAJECTORY_DURATION_EXCEEDED,
                f"point={index},value={time_ns},limit={max_duration_ns}",
            )
        current_positions = tuple(float(value) for value in positions)
        if previous_positions is not None:
            segment_seconds = (time_ns - segment_start_ns) / 1e9
            for joint, previous, current in zip(
                expected, previous_positions, current_positions, strict=True
            ):
                implied_velocity = abs(current - previous) / segment_seconds
                if implied_velocity > max_velocity:
                    return TrajectoryContractDecision(
                        False,
                        TrajectoryContractReason.VELOCITY_LIMIT_EXCEEDED,
                        f"point={index},joint={joint},implied={implied_velocity},limit={max_velocity}",
                    )
        previous_positions = current_positions

    return TrajectoryContractDecision(
        True,
        TrajectoryContractReason.VALID,
        controller_name,
    )
