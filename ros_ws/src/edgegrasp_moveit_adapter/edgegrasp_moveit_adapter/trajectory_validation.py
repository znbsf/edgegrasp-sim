"""Validation/conversion for MoveIt joint-space trajectories.

This module intentionally does not use the Cartesian ``edgegrasp.MotionPlan``
model.  MoveIt returns a multi-point joint-space ``RobotTrajectory``; the
accepted arm portion is converted only to the ``JointTrajectory`` consumed by
the existing ROS command-boundary gate.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from enum import Enum
import math
from collections.abc import Sequence

from edgegrasp.so101_contract import (
    SO101_ARM_JOINTS,
    validate_trajectory_contract,
)
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory


class RobotTrajectoryReason(str, Enum):
    VALID = "valid"
    MULTI_DOF_NOT_EMPTY = "multi_dof_not_empty"
    JOINT_ORDER_MISMATCH = "joint_order_mismatch"
    EMPTY = "empty_trajectory"
    INVALID_START_POINT = "invalid_start_point"
    START_STATE_MISMATCH = "start_state_mismatch"
    NO_POSITIVE_TIME_POINT = "no_positive_time_point"
    CONTRACT_REJECTED = "joint_trajectory_contract_rejected"


@dataclass(frozen=True, slots=True)
class RobotTrajectoryDecision:
    accepted: bool
    reason: RobotTrajectoryReason
    detail: str
    joint_trajectory: JointTrajectory | None = None


def _time_ns(point) -> int:
    return point.time_from_start.sec * 1_000_000_000 + point.time_from_start.nanosec


def _finite(values: Sequence[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def _point_payload(point) -> dict[str, object]:
    return {
        "positions": point.positions,
        "velocities": point.velocities,
        "accelerations": point.accelerations,
        "effort": point.effort,
        "time_from_start_ns": _time_ns(point),
    }


def validate_and_convert_robot_trajectory(
    robot_trajectory: RobotTrajectory,
    *,
    fresh_start_positions: Sequence[float],
    start_tolerance_rad: float,
) -> RobotTrajectoryDecision:
    """Return a gate-ready arm JointTrajectory or a fail-closed reason."""

    if robot_trajectory.multi_dof_joint_trajectory.points:
        return RobotTrajectoryDecision(
            False,
            RobotTrajectoryReason.MULTI_DOF_NOT_EMPTY,
            "fixed-base SO-101 expects no multi-DOF trajectory points",
        )
    source = robot_trajectory.joint_trajectory
    if tuple(source.joint_names) != SO101_ARM_JOINTS:
        return RobotTrajectoryDecision(
            False,
            RobotTrajectoryReason.JOINT_ORDER_MISMATCH,
            f"expected={SO101_ARM_JOINTS},actual={tuple(source.joint_names)}",
        )
    if not source.points:
        return RobotTrajectoryDecision(
            False, RobotTrajectoryReason.EMPTY, "MoveIt returned no arm points"
        )
    if (
        len(fresh_start_positions) != len(SO101_ARM_JOINTS)
        or not _finite(fresh_start_positions)
        or not math.isfinite(start_tolerance_rad)
        or start_tolerance_rad <= 0.0
    ):
        return RobotTrajectoryDecision(
            False,
            RobotTrajectoryReason.INVALID_START_POINT,
            "fresh start state/tolerance is malformed",
        )

    first = source.points[0]
    if len(first.positions) != len(SO101_ARM_JOINTS) or not _finite(first.positions):
        return RobotTrajectoryDecision(
            False,
            RobotTrajectoryReason.INVALID_START_POINT,
            "first point positions are malformed",
        )
    mismatch = max(
        abs(float(actual) - float(expected))
        for actual, expected in zip(
            first.positions, fresh_start_positions, strict=True
        )
    )
    if mismatch > start_tolerance_rad:
        return RobotTrajectoryDecision(
            False,
            RobotTrajectoryReason.START_STATE_MISMATCH,
            f"max_abs_error={mismatch},tolerance={start_tolerance_rad}",
        )

    converted = deepcopy(source)
    # MoveIt time parameterization normally emits the explicit start state at
    # t=0.  It is evidence for the start-state comparison above, not a future
    # controller command.  Remove only this validated leading sample; all
    # points handed to the gate must have strictly positive increasing times.
    if _time_ns(converted.points[0]) == 0:
        converted.points = list(converted.points[1:])
    if not converted.points:
        return RobotTrajectoryDecision(
            False,
            RobotTrajectoryReason.NO_POSITIVE_TIME_POINT,
            "trajectory contained only the t=0 start sample",
        )

    contract = validate_trajectory_contract(
        "arm_controller",
        converted.joint_names,
        [_point_payload(point) for point in converted.points],
        start_positions=fresh_start_positions,
    )
    if not contract.accepted:
        return RobotTrajectoryDecision(
            False,
            RobotTrajectoryReason.CONTRACT_REJECTED,
            f"{contract.reason.value}:{contract.detail}",
        )
    return RobotTrajectoryDecision(
        True,
        RobotTrajectoryReason.VALID,
        "validated MoveIt arm trajectory",
        converted,
    )
