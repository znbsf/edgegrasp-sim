"""Stable identities for correlated SO-101 trajectory execution commands."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import hashlib
import json
import math
import re


_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")


def _validated_identifier(value: object, field: str) -> str:
    if not isinstance(value, str) or _IDENTIFIER.fullmatch(value) is None:
        raise ValueError(
            f"{field} must match {_IDENTIFIER.pattern!r}; got {value!r}"
        )
    return value


def make_trajectory_command_id(
    task_id: str, stage: str, sequence_no: int
) -> str:
    """Return the only accepted command-id encoding for a task stage.

    Restricting identifiers to delimiter-free tokens makes the binding
    unambiguous and lets every ROS node independently validate it.
    """

    task = _validated_identifier(task_id, "task_id")
    phase = _validated_identifier(stage, "stage")
    if isinstance(sequence_no, bool) or not isinstance(sequence_no, int):
        raise ValueError("sequence_no must be an integer")
    if sequence_no < 0:
        raise ValueError("sequence_no must be non-negative")
    return f"{task}|{phase}|{sequence_no}"


def validate_trajectory_command_id(
    command_id: str, task_id: str, stage: str, sequence_no: int
) -> None:
    """Reject a command id that is not bound to its immutable task fields."""

    expected = make_trajectory_command_id(task_id, stage, sequence_no)
    if command_id != expected:
        raise ValueError(
            f"command_id mismatch: expected {expected!r}, got {command_id!r}"
        )


def _finite_vector(values: object, field: str) -> list[float]:
    if not isinstance(values, Sequence) or isinstance(values, (str, bytes)):
        raise ValueError(f"{field} must be a numeric sequence")
    normalized: list[float] = []
    for value in values:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{field} contains a non-numeric value")
        number = float(value)
        if not math.isfinite(number):
            raise ValueError(f"{field} contains a non-finite value")
        normalized.append(number)
    return normalized


def trajectory_digest(
    controller: str,
    joint_names: Sequence[str],
    points: Sequence[Mapping[str, object]],
) -> str:
    """Hash the complete controller/joint/point trajectory payload.

    The digest deliberately excludes ROS transport headers.  It identifies
    the exact controller command: controller, ordered joints, every numeric
    point vector, and every ``time_from_start`` value.
    """

    controller_name = _validated_identifier(controller, "controller")
    joints = [_validated_identifier(name, "joint_name") for name in joint_names]
    if not joints or len(set(joints)) != len(joints):
        raise ValueError("joint_names must be non-empty and unique")
    normalized_points: list[dict[str, object]] = []
    if not points:
        raise ValueError("at least one trajectory point is required")
    for index, point in enumerate(points):
        if not isinstance(point, Mapping):
            raise ValueError(f"point {index} must be a mapping")
        time_ns = point.get("time_from_start_ns")
        if isinstance(time_ns, bool) or not isinstance(time_ns, int) or time_ns < 0:
            raise ValueError(f"point {index} has invalid time_from_start_ns")
        normalized_points.append(
            {
                "positions": _finite_vector(
                    point.get("positions", ()), f"point[{index}].positions"
                ),
                "velocities": _finite_vector(
                    point.get("velocities", ()), f"point[{index}].velocities"
                ),
                "accelerations": _finite_vector(
                    point.get("accelerations", ()),
                    f"point[{index}].accelerations",
                ),
                "effort": _finite_vector(
                    point.get("effort", ()), f"point[{index}].effort"
                ),
                "time_from_start_ns": time_ns,
            }
        )
    payload = {
        "controller": controller_name,
        "joint_names": joints,
        "points": normalized_points,
        "schema_version": 1,
    }
    encoded = json.dumps(
        payload,
        allow_nan=False,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
