"""Dependency-free SO-101 grasp-pose geometry and fail-closed preflight.

The MoveIt goal pose is the pose of ``gripper_frame_link`` rather than the
cube centre.  This module makes that offset explicit so a protocol-successful
sequence cannot accidentally be presented as a physically meaningful grasp
trial when the cube is outside the finger pads.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any


PINNED_SO101_COMMIT = "0305e03ab54e64aae9263fcbf339622e654012f3"


class GraspGeometryError(ValueError):
    """Raised when a grasp profile or requested stage geometry is unsafe."""


@dataclass(frozen=True, slots=True)
class AxisAlignedEnvelope:
    """Conservative collision envelope expressed in ``end_effector_frame``."""

    name: str
    minimum_m: tuple[float, float, float]
    maximum_m: tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class OrientedBoxEnvelope:
    """Exact primitive-box pose expressed in ``end_effector_frame``."""

    name: str
    center_m: tuple[float, float, float]
    size_m: tuple[float, float, float]
    rotation_rows: tuple[tuple[float, float, float], ...]


@dataclass(frozen=True, slots=True)
class PrecloseContactPolicy:
    """Static geometry contract for a contact-free descend before closing."""

    fixed_pad_name: str
    moving_pad_name: str
    minimum_fixed_pad_clearance_m: float
    maximum_fixed_pad_clearance_m: float
    minimum_moving_pad_closed_overlap_m: float
    enable_target_pad_contacts_after: str


@dataclass(frozen=True, slots=True)
class GraspGeometryProfile:
    target_id: str
    end_effector_frame: str
    cube_size_m: tuple[float, float, float]
    target_center_in_frame_at_descend_m: tuple[float, float, float]
    target_center_tolerance_m: tuple[float, float, float]
    approach_offset_end_effector_m: tuple[float, float, float]
    approach_offset_planning_m: tuple[float, float, float]
    lift_offset_planning_m: tuple[float, float, float]
    gripper_open_position_rad: float
    gripper_contact_position_rad: float
    gripper_position_tolerance_rad: float
    stage_position_tolerance_m: float
    noncontact_body_aabbs: tuple[AxisAlignedEnvelope, ...]
    required_contact_pad_aabbs: tuple[AxisAlignedEnvelope, ...]
    required_contact_pad_obbs: tuple[OrientedBoxEnvelope, ...]
    minimum_noncontact_clearance_m: float
    minimum_approach_clearance_m: float
    minimum_contact_overlap_m: float
    preclose_contact_policy: PrecloseContactPolicy | None


@dataclass(frozen=True, slots=True)
class GraspStageGeometry:
    approach_position_m: tuple[float, float, float]
    descend_position_m: tuple[float, float, float]
    lift_position_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    cube_center_in_frame_at_descend_m: tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class RoutedGraspStageGeometry:
    """A collision-routing approach plus a fixed-orientation grasp pair."""

    approach_position_m: tuple[float, float, float]
    descend_position_m: tuple[float, float, float]
    lift_position_m: tuple[float, float, float]
    approach_orientation_xyzw: tuple[float, float, float, float]
    grasp_orientation_xyzw: tuple[float, float, float, float]
    cube_center_in_frame_at_descend_m: tuple[float, float, float]


def _vector3(value: Any, label: str, *, positive: bool = False) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != 3:
        raise GraspGeometryError(f"{label} must contain three values")
    if any(
        isinstance(item, bool)
        or not isinstance(item, (int, float))
        or not math.isfinite(float(item))
        for item in value
    ):
        raise GraspGeometryError(f"{label} must contain finite numbers")
    parsed = tuple(float(item) for item in value)
    if positive and any(item <= 0.0 for item in parsed):
        raise GraspGeometryError(f"{label} must be positive")
    return parsed


def _positive_float(value: Any, label: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or float(value) <= 0.0
    ):
        raise GraspGeometryError(f"{label} must be finite and positive")
    return float(value)


def _axis_offset(
    value: Any,
    label: str,
    *,
    axis_index: int,
    required_sign: str,
) -> tuple[float, float, float]:
    parsed = _vector3(value, label)
    if any(parsed[axis] != 0.0 for axis in range(3) if axis != axis_index):
        raise GraspGeometryError(
            f"{label} must use only axis {axis_index} in this first-stage profile"
        )
    component = parsed[axis_index]
    if required_sign == "positive" and component <= 0.0:
        raise GraspGeometryError(f"{label} positive-axis component must be positive")
    if required_sign == "negative" and component >= 0.0:
        raise GraspGeometryError(f"{label} axis component must be negative")
    if required_sign == "nonnegative" and component < 0.0:
        raise GraspGeometryError(f"{label} axis component must be non-negative")
    if required_sign not in {"positive", "negative", "nonnegative"}:
        raise AssertionError(f"unsupported required_sign: {required_sign}")
    return parsed


def _aabb_list(value: Any, label: str) -> tuple[AxisAlignedEnvelope, ...]:
    if not isinstance(value, list) or not value:
        raise GraspGeometryError(f"{label} must be a non-empty list")
    envelopes: list[AxisAlignedEnvelope] = []
    names: set[str] = set()
    for index, item in enumerate(value):
        if not isinstance(item, dict):
            raise GraspGeometryError(f"{label}[{index}] must be an object")
        name = item.get("name")
        if not isinstance(name, str) or not name or name in names:
            raise GraspGeometryError(f"{label}[{index}] name must be unique")
        minimum = _vector3(item.get("minimum_m"), f"{label}[{index}].minimum_m")
        maximum = _vector3(item.get("maximum_m"), f"{label}[{index}].maximum_m")
        if any(low >= high for low, high in zip(minimum, maximum, strict=True)):
            raise GraspGeometryError(f"{label}[{index}] bounds must be ordered")
        envelopes.append(
            AxisAlignedEnvelope(
                name=name,
                minimum_m=minimum,
                maximum_m=maximum,
            )
        )
        names.add(name)
    return tuple(envelopes)


def _determinant3(rows: tuple[tuple[float, float, float], ...]) -> float:
    return (
        rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
        - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
        + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0])
    )


def _rotation_rows(value: Any, label: str) -> tuple[tuple[float, float, float], ...]:
    if not isinstance(value, list) or len(value) != 3:
        raise GraspGeometryError(f"{label} must contain three rows")
    rows = tuple(_vector3(row, f"{label}[{index}]") for index, row in enumerate(value))
    for first in range(3):
        for second in range(3):
            dot = sum(rows[first][axis] * rows[second][axis] for axis in range(3))
            expected = 1.0 if first == second else 0.0
            if not math.isclose(dot, expected, rel_tol=0.0, abs_tol=1e-6):
                raise GraspGeometryError(f"{label} must be orthonormal")
    if not math.isclose(_determinant3(rows), 1.0, rel_tol=0.0, abs_tol=1e-6):
        raise GraspGeometryError(f"{label} must be a proper rotation")
    return rows


def _obb_list(value: Any, label: str) -> tuple[OrientedBoxEnvelope, ...]:
    if not isinstance(value, list) or not value:
        raise GraspGeometryError(f"{label} must be a non-empty list")
    boxes: list[OrientedBoxEnvelope] = []
    names: set[str] = set()
    for index, item in enumerate(value):
        if not isinstance(item, dict):
            raise GraspGeometryError(f"{label}[{index}] must be an object")
        name = item.get("name")
        if not isinstance(name, str) or not name or name in names:
            raise GraspGeometryError(f"{label}[{index}] name must be unique")
        boxes.append(
            OrientedBoxEnvelope(
                name=name,
                center_m=_vector3(item.get("center_m"), f"{label}[{index}].center_m"),
                size_m=_vector3(
                    item.get("size_m"), f"{label}[{index}].size_m", positive=True
                ),
                rotation_rows=_rotation_rows(
                    item.get("rotation_rows"),
                    f"{label}[{index}].rotation_rows",
                ),
            )
        )
        names.add(name)
    return tuple(boxes)


def load_grasp_geometry_profile(path: Path) -> GraspGeometryProfile:
    payload = json.loads(path.read_text(encoding="utf-8"))
    schema_version = payload.get("schema_version")
    if schema_version not in {6, 7}:
        raise GraspGeometryError("unsupported grasp geometry schema")
    upstream = payload.get("upstream")
    if not isinstance(upstream, dict) or upstream.get("commit") != PINNED_SO101_COMMIT:
        raise GraspGeometryError("grasp geometry upstream pin drift")
    target_id = payload.get("target_id")
    end_effector_frame = payload.get("end_effector_frame")
    if not isinstance(target_id, str) or not target_id:
        raise GraspGeometryError("target_id must not be empty")
    if not isinstance(end_effector_frame, str) or not end_effector_frame:
        raise GraspGeometryError("end_effector_frame must not be empty")
    envelope = payload.get("gazebo_proxy_preflight")
    if not isinstance(envelope, dict):
        raise GraspGeometryError("gazebo_proxy_preflight must be an object")
    pad_aabbs = _aabb_list(
        envelope.get("required_contact_pad_aabbs"),
        "gazebo_proxy_preflight.required_contact_pad_aabbs",
    )
    pad_obbs = _obb_list(
        envelope.get("required_contact_pad_obbs"),
        "gazebo_proxy_preflight.required_contact_pad_obbs",
    )
    if tuple(item.name for item in pad_aabbs) != tuple(item.name for item in pad_obbs):
        raise GraspGeometryError("contact pad AABB/OBB identity mismatch")
    preclose_policy_payload = payload.get("preclose_contact_policy")
    preclose_policy: PrecloseContactPolicy | None = None
    if schema_version == 7:
        if not isinstance(preclose_policy_payload, dict):
            raise GraspGeometryError(
                "schema 7 requires preclose_contact_policy"
            )
        fixed_pad_name = preclose_policy_payload.get("fixed_pad_name")
        moving_pad_name = preclose_policy_payload.get("moving_pad_name")
        pad_names = {item.name for item in pad_obbs}
        if (
            not isinstance(fixed_pad_name, str)
            or not isinstance(moving_pad_name, str)
            or fixed_pad_name == moving_pad_name
            or {fixed_pad_name, moving_pad_name} != pad_names
        ):
            raise GraspGeometryError(
                "preclose_contact_policy must identify the two contact pads"
            )
        minimum_clearance = _positive_float(
            preclose_policy_payload.get("minimum_fixed_pad_clearance_m"),
            "preclose_contact_policy.minimum_fixed_pad_clearance_m",
        )
        maximum_clearance = _positive_float(
            preclose_policy_payload.get("maximum_fixed_pad_clearance_m"),
            "preclose_contact_policy.maximum_fixed_pad_clearance_m",
        )
        if minimum_clearance > maximum_clearance:
            raise GraspGeometryError(
                "preclose fixed-pad clearance bounds must be ordered"
            )
        enable_after = preclose_policy_payload.get(
            "enable_target_pad_contacts_after"
        )
        if enable_after != "descend_terminal":
            raise GraspGeometryError(
                "preclose contact policy must enable contacts after descend_terminal"
            )
        preclose_policy = PrecloseContactPolicy(
            fixed_pad_name=fixed_pad_name,
            moving_pad_name=moving_pad_name,
            minimum_fixed_pad_clearance_m=minimum_clearance,
            maximum_fixed_pad_clearance_m=maximum_clearance,
            minimum_moving_pad_closed_overlap_m=_positive_float(
                preclose_policy_payload.get(
                    "minimum_moving_pad_closed_overlap_m"
                ),
                "preclose_contact_policy.minimum_moving_pad_closed_overlap_m",
            ),
            enable_target_pad_contacts_after=enable_after,
        )
    elif preclose_policy_payload is not None:
        raise GraspGeometryError(
            "preclose_contact_policy requires grasp geometry schema 7"
        )
    return GraspGeometryProfile(
        target_id=target_id,
        end_effector_frame=end_effector_frame,
        cube_size_m=_vector3(payload.get("cube_size_m"), "cube_size_m", positive=True),
        target_center_in_frame_at_descend_m=_vector3(
            payload.get("target_center_in_frame_at_descend_m"),
            "target_center_in_frame_at_descend_m",
        ),
        target_center_tolerance_m=_vector3(
            payload.get("target_center_tolerance_m"),
            "target_center_tolerance_m",
            positive=True,
        ),
        approach_offset_end_effector_m=_axis_offset(
            payload.get("approach_offset_end_effector_m"),
            "approach_offset_end_effector_m",
            axis_index=2,
            required_sign="positive",
        ),
        approach_offset_planning_m=_axis_offset(
            payload.get("approach_offset_planning_m"),
            "approach_offset_planning_m",
            axis_index=2,
            required_sign="nonnegative",
        ),
        lift_offset_planning_m=_axis_offset(
            payload.get("lift_offset_planning_m"),
            "lift_offset_planning_m",
            axis_index=2,
            required_sign="positive",
        ),
        gripper_open_position_rad=_positive_float(
            payload.get("gripper_open_position_rad"),
            "gripper_open_position_rad",
        ),
        gripper_contact_position_rad=_positive_float(
            payload.get("gripper_contact_position_rad"),
            "gripper_contact_position_rad",
        ),
        gripper_position_tolerance_rad=_positive_float(
            payload.get("gripper_position_tolerance_rad"),
            "gripper_position_tolerance_rad",
        ),
        stage_position_tolerance_m=_positive_float(
            payload.get("stage_position_tolerance_m"),
            "stage_position_tolerance_m",
        ),
        noncontact_body_aabbs=_aabb_list(
            envelope.get("noncontact_body_aabbs"),
            "gazebo_proxy_preflight.noncontact_body_aabbs",
        ),
        required_contact_pad_aabbs=pad_aabbs,
        required_contact_pad_obbs=pad_obbs,
        minimum_noncontact_clearance_m=_positive_float(
            envelope.get("minimum_noncontact_clearance_m"),
            "gazebo_proxy_preflight.minimum_noncontact_clearance_m",
        ),
        minimum_approach_clearance_m=_positive_float(
            envelope.get("minimum_approach_clearance_m"),
            "gazebo_proxy_preflight.minimum_approach_clearance_m",
        ),
        minimum_contact_overlap_m=_positive_float(
            envelope.get("minimum_contact_overlap_m"),
            "gazebo_proxy_preflight.minimum_contact_overlap_m",
        ),
        preclose_contact_policy=preclose_policy,
    )


def _rotation_matrix(
    orientation_xyzw: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], ...]:
    if len(orientation_xyzw) != 4 or any(
        not math.isfinite(value) for value in orientation_xyzw
    ):
        raise GraspGeometryError("orientation must contain four finite values")
    x, y, z, w = orientation_xyzw
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if abs(norm - 1.0) > 1e-3:
        raise GraspGeometryError("orientation quaternion must be normalized")
    return (
        (
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ),
        (
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ),
        (
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ),
    )


def _rotate(
    rotation: tuple[tuple[float, float, float], ...],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    return tuple(
        sum(rotation[row][column] * vector[column] for column in range(3))
        for row in range(3)
    )


def _inverse_rotate(
    rotation: tuple[tuple[float, float, float], ...],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    return tuple(
        sum(rotation[row][column] * vector[row] for row in range(3))
        for column in range(3)
    )


def transform_point_from_frame(
    frame_position_m: tuple[float, float, float],
    orientation_xyzw: tuple[float, float, float, float],
    point_in_frame_m: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Transform one local point into the parent frame without ROS/tf2."""

    if (
        len(frame_position_m) != 3
        or len(point_in_frame_m) != 3
        or any(
            not math.isfinite(value)
            for value in (*frame_position_m, *point_in_frame_m)
        )
    ):
        raise GraspGeometryError("frame and local point must contain finite triples")
    offset = _rotate(_rotation_matrix(orientation_xyzw), point_in_frame_m)
    return tuple(
        frame_position_m[axis] + offset[axis] for axis in range(3)
    )


def _cube_aabb_in_frame(
    *,
    center_in_frame_m: tuple[float, float, float],
    cube_size_m: tuple[float, float, float],
    frame_orientation_xyzw: tuple[float, float, float, float],
    cube_orientation_xyzw: tuple[float, float, float, float] = (
        0.0,
        0.0,
        0.0,
        1.0,
    ),
) -> AxisAlignedEnvelope:
    """Conservatively project an oriented cube into the tool frame."""

    frame_to_world = _rotation_matrix(frame_orientation_xyzw)
    cube_to_world = _rotation_matrix(cube_orientation_xyzw)
    cube_to_frame = tuple(
        tuple(
            sum(
                frame_to_world[world_axis][frame_axis]
                * cube_to_world[world_axis][cube_axis]
                for world_axis in range(3)
            )
            for cube_axis in range(3)
        )
        for frame_axis in range(3)
    )
    half_extents = tuple(
        sum(
            abs(cube_to_frame[frame_axis][cube_axis])
            * cube_size_m[cube_axis]
            / 2.0
            for cube_axis in range(3)
        )
        for frame_axis in range(3)
    )
    return AxisAlignedEnvelope(
        name="target_cube",
        minimum_m=tuple(
            center_in_frame_m[axis] - half_extents[axis] for axis in range(3)
        ),
        maximum_m=tuple(
            center_in_frame_m[axis] + half_extents[axis] for axis in range(3)
        ),
    )


def _aabb_separation_m(
    first: AxisAlignedEnvelope, second: AxisAlignedEnvelope
) -> float:
    """Return a conservative separating-axis clearance; <=0 means overlap."""

    return max(
        max(
            first.minimum_m[axis] - second.maximum_m[axis],
            second.minimum_m[axis] - first.maximum_m[axis],
        )
        for axis in range(3)
    )


def _aabb_minimum_overlap_m(
    first: AxisAlignedEnvelope, second: AxisAlignedEnvelope
) -> float:
    return min(
        min(first.maximum_m[axis], second.maximum_m[axis])
        - max(first.minimum_m[axis], second.minimum_m[axis])
        for axis in range(3)
    )


def _matrix_columns(
    rows: tuple[tuple[float, float, float], ...],
) -> tuple[tuple[float, float, float], ...]:
    return tuple(tuple(rows[row][column] for row in range(3)) for column in range(3))


def _dot3(
    first: tuple[float, float, float], second: tuple[float, float, float]
) -> float:
    return sum(first[axis] * second[axis] for axis in range(3))


def oriented_box_overlap_margins(
    first: OrientedBoxEnvelope,
    second: OrientedBoxEnvelope,
) -> tuple[float, float]:
    """Return minimum normalized SAT face/cross-axis overlaps in metres.

    A negative value is a separating axis. Cross products are normalized so
    their margin remains a metric distance; nearly parallel axes are skipped
    because their corresponding face axes already cover that direction.
    """

    first_axes = _matrix_columns(first.rotation_rows)
    second_axes = _matrix_columns(second.rotation_rows)
    first_half = tuple(value / 2.0 for value in first.size_m)
    second_half = tuple(value / 2.0 for value in second.size_m)
    relative = tuple(
        second.center_m[axis] - first.center_m[axis] for axis in range(3)
    )
    rotation = tuple(
        tuple(_dot3(first_axes[row], second_axes[column]) for column in range(3))
        for row in range(3)
    )
    translated = tuple(_dot3(relative, axis) for axis in first_axes)
    absolute = tuple(
        tuple(abs(rotation[row][column]) + 1e-12 for column in range(3))
        for row in range(3)
    )
    face_margins: list[float] = []
    for row in range(3):
        face_margins.append(
            first_half[row]
            + sum(second_half[column] * absolute[row][column] for column in range(3))
            - abs(translated[row])
        )
    for column in range(3):
        face_margins.append(
            second_half[column]
            + sum(first_half[row] * absolute[row][column] for row in range(3))
            - abs(sum(translated[row] * rotation[row][column] for row in range(3)))
        )

    cross_margins: list[float] = []
    for row in range(3):
        for column in range(3):
            axis_norm = math.sqrt(max(0.0, 1.0 - rotation[row][column] ** 2))
            if axis_norm < 1e-8:
                continue
            next_row, last_row = (row + 1) % 3, (row + 2) % 3
            next_column, last_column = (column + 1) % 3, (column + 2) % 3
            first_radius = (
                first_half[next_row] * absolute[last_row][column]
                + first_half[last_row] * absolute[next_row][column]
            )
            second_radius = (
                second_half[next_column] * absolute[row][last_column]
                + second_half[last_column] * absolute[row][next_column]
            )
            distance = abs(
                translated[last_row] * rotation[next_row][column]
                - translated[next_row] * rotation[last_row][column]
            )
            cross_margins.append(
                (first_radius + second_radius - distance) / axis_norm
            )
    return min(face_margins), min(cross_margins, default=math.inf)


def cube_obb_in_frame(
    *,
    center_in_frame_m: tuple[float, float, float],
    cube_size_m: tuple[float, float, float],
    frame_orientation_xyzw: tuple[float, float, float, float],
    cube_orientation_xyzw: tuple[float, float, float, float] = (
        0.0,
        0.0,
        0.0,
        1.0,
    ),
) -> OrientedBoxEnvelope:
    tool_to_world = _rotation_matrix(frame_orientation_xyzw)
    world_to_tool = tuple(
        tuple(tool_to_world[column][row] for column in range(3)) for row in range(3)
    )
    cube_to_world = _rotation_matrix(cube_orientation_xyzw)
    cube_to_tool = tuple(
        tuple(
            sum(
                world_to_tool[row][world_axis]
                * cube_to_world[world_axis][column]
                for world_axis in range(3)
            )
            for column in range(3)
        )
        for row in range(3)
    )
    return OrientedBoxEnvelope(
        name="target_cube",
        center_m=center_in_frame_m,
        size_m=cube_size_m,
        rotation_rows=cube_to_tool,
    )


def validate_grasp_collision_envelope(
    *,
    cube_center_m: tuple[float, float, float],
    approach_position_m: tuple[float, float, float],
    descend_position_m: tuple[float, float, float],
    orientation_xyzw: tuple[float, float, float, float],
    profile: GraspGeometryProfile,
    approach_orientation_xyzw: tuple[float, float, float, float] | None = None,
    cube_orientation_xyzw: tuple[float, float, float, float] = (
        0.0,
        0.0,
        0.0,
        1.0,
    ),
) -> None:
    """Fail closed on the task039 palm-contact geometry before any ROS goal.

    The check is intentionally conservative and Gazebo-proxy-specific.  It is
    not a swept-volume, MoveIt, contact-dynamics, or real-hardware proof.
    """

    rotation = _rotation_matrix(orientation_xyzw)
    approach_orientation = approach_orientation_xyzw or orientation_xyzw
    approach_rotation = _rotation_matrix(approach_orientation)

    def cube_at(
        frame_position_m: tuple[float, float, float],
        frame_rotation: tuple[tuple[float, float, float], ...],
        frame_orientation: tuple[float, float, float, float],
    ) -> AxisAlignedEnvelope:
        delta = tuple(
            cube_center_m[axis] - frame_position_m[axis] for axis in range(3)
        )
        return _cube_aabb_in_frame(
            center_in_frame_m=_inverse_rotate(frame_rotation, delta),
            cube_size_m=profile.cube_size_m,
            frame_orientation_xyzw=frame_orientation,
            cube_orientation_xyzw=cube_orientation_xyzw,
        )

    approach_cube = cube_at(
        approach_position_m,
        approach_rotation,
        approach_orientation,
    )
    descend_cube = cube_at(descend_position_m, rotation, orientation_xyzw)
    descend_cube_obb = cube_obb_in_frame(
        center_in_frame_m=_inverse_rotate(
            rotation,
            tuple(
                cube_center_m[axis] - descend_position_m[axis]
                for axis in range(3)
            ),
        ),
        cube_size_m=profile.cube_size_m,
        frame_orientation_xyzw=orientation_xyzw,
        cube_orientation_xyzw=cube_orientation_xyzw,
    )
    all_boxes = (*profile.noncontact_body_aabbs, *profile.required_contact_pad_aabbs)
    for box in all_boxes:
        clearance = _aabb_separation_m(approach_cube, box)
        if clearance < profile.minimum_approach_clearance_m:
            raise GraspGeometryError(
                f"approach_proxy_clearance:{box.name}:{clearance:.9f}"
            )
    for body in profile.noncontact_body_aabbs:
        clearance = _aabb_separation_m(descend_cube, body)
        if clearance < profile.minimum_noncontact_clearance_m:
            raise GraspGeometryError(
                f"noncontact_body_clearance:{body.name}:{clearance:.9f}"
            )
    overlaps: dict[str, float] = {}
    for pad in profile.required_contact_pad_obbs:
        face_overlap, cross_overlap = oriented_box_overlap_margins(
            descend_cube_obb, pad
        )
        overlaps[pad.name] = min(face_overlap, cross_overlap)
    policy = profile.preclose_contact_policy
    if policy is None:
        for pad_name, overlap in overlaps.items():
            if overlap < profile.minimum_contact_overlap_m:
                raise GraspGeometryError(
                    f"required_pad_obb_overlap:{pad_name}:{overlap:.9f}"
                )
    else:
        fixed_clearance = -overlaps[policy.fixed_pad_name]
        if not (
            policy.minimum_fixed_pad_clearance_m
            <= fixed_clearance
            <= policy.maximum_fixed_pad_clearance_m
        ):
            raise GraspGeometryError(
                "preclose_fixed_pad_clearance:"
                f"{policy.fixed_pad_name}:{fixed_clearance:.9f}"
            )
        moving_overlap = overlaps[policy.moving_pad_name]
        if moving_overlap < policy.minimum_moving_pad_closed_overlap_m:
            raise GraspGeometryError(
                "preclose_moving_pad_closed_overlap:"
                f"{policy.moving_pad_name}:{moving_overlap:.9f}"
            )


def derive_grasp_stage_geometry(
    cube_center_m: tuple[float, float, float],
    orientation_xyzw: tuple[float, float, float, float],
    profile: GraspGeometryProfile,
) -> GraspStageGeometry:
    if len(cube_center_m) != 3 or any(
        not math.isfinite(value) for value in cube_center_m
    ):
        raise GraspGeometryError("cube_center_m must contain three finite values")
    rotation = _rotation_matrix(orientation_xyzw)
    relative_world = _rotate(rotation, profile.target_center_in_frame_at_descend_m)
    descend = tuple(
        cube_center_m[axis] - relative_world[axis] for axis in range(3)
    )
    approach_world = _rotate(rotation, profile.approach_offset_end_effector_m)
    approach = tuple(
        descend[axis]
        + approach_world[axis]
        + profile.approach_offset_planning_m[axis]
        for axis in range(3)
    )
    lift = tuple(
        descend[axis] + profile.lift_offset_planning_m[axis]
        for axis in range(3)
    )
    return GraspStageGeometry(
        approach_position_m=approach,
        descend_position_m=descend,
        lift_position_m=lift,
        orientation_xyzw=orientation_xyzw,
        cube_center_in_frame_at_descend_m=(
            profile.target_center_in_frame_at_descend_m
        ),
    )


def validate_grasp_stage_geometry(
    *,
    cube_center_m: tuple[float, float, float],
    approach_position_m: tuple[float, float, float],
    descend_position_m: tuple[float, float, float],
    lift_position_m: tuple[float, float, float],
    orientation_xyzw: tuple[float, float, float, float],
    gripper_position_rad: float,
    profile: GraspGeometryProfile,
    cube_orientation_xyzw: tuple[float, float, float, float] = (
        0.0,
        0.0,
        0.0,
        1.0,
    ),
) -> GraspStageGeometry:
    requested = (approach_position_m, descend_position_m, lift_position_m)
    if any(
        len(position) != 3 or any(not math.isfinite(value) for value in position)
        for position in requested
    ):
        raise GraspGeometryError("stage positions must contain finite triples")
    if not math.isfinite(gripper_position_rad):
        raise GraspGeometryError("gripper position must be finite")
    expected = derive_grasp_stage_geometry(cube_center_m, orientation_xyzw, profile)
    rotation = _rotation_matrix(orientation_xyzw)
    cube_delta = tuple(
        cube_center_m[axis] - descend_position_m[axis] for axis in range(3)
    )
    relative = _inverse_rotate(rotation, cube_delta)
    if any(
        abs(value - target) > tolerance
        for value, target, tolerance in zip(
            relative,
            profile.target_center_in_frame_at_descend_m,
            profile.target_center_tolerance_m,
            strict=True,
        )
    ):
        raise GraspGeometryError(
            "cube_center_outside_finger_contact_envelope: "
            f"observed={relative},expected="
            f"{profile.target_center_in_frame_at_descend_m}"
        )
    for label, actual, reference in (
        ("approach", approach_position_m, expected.approach_position_m),
        ("descend", descend_position_m, expected.descend_position_m),
        ("lift", lift_position_m, expected.lift_position_m),
    ):
        if any(
            abs(value - target) > profile.stage_position_tolerance_m
            for value, target in zip(actual, reference, strict=True)
        ):
            raise GraspGeometryError(f"{label}_position_drift")
    if (
        abs(gripper_position_rad - profile.gripper_contact_position_rad)
        > profile.gripper_position_tolerance_rad
    ):
        raise GraspGeometryError("gripper_contact_position_mismatch")
    validate_grasp_collision_envelope(
        cube_center_m=cube_center_m,
        approach_position_m=approach_position_m,
        descend_position_m=descend_position_m,
        orientation_xyzw=orientation_xyzw,
        profile=profile,
        cube_orientation_xyzw=cube_orientation_xyzw,
    )
    return GraspStageGeometry(
        approach_position_m=approach_position_m,
        descend_position_m=descend_position_m,
        lift_position_m=lift_position_m,
        orientation_xyzw=orientation_xyzw,
        cube_center_in_frame_at_descend_m=relative,
    )


def validate_routed_grasp_stage_geometry(
    *,
    cube_center_m: tuple[float, float, float],
    approach_position_m: tuple[float, float, float],
    descend_position_m: tuple[float, float, float],
    lift_position_m: tuple[float, float, float],
    approach_orientation_xyzw: tuple[float, float, float, float],
    grasp_orientation_xyzw: tuple[float, float, float, float],
    gripper_position_rad: float,
    profile: GraspGeometryProfile,
    cube_orientation_xyzw: tuple[float, float, float, float] = (
        0.0,
        0.0,
        0.0,
        1.0,
    ),
) -> RoutedGraspStageGeometry:
    """Validate a routed pre-grasp without weakening contact geometry.

    The approach pose is allowed to use a distinct orientation so MoveIt can
    route the palm around the retained cube. Descend and lift must still match
    the profile derived from one immutable grasp quaternion. This is a static
    primitive-envelope gate; MoveIt remains responsible for swept-path checks.
    """

    requested = (approach_position_m, descend_position_m, lift_position_m)
    if any(
        len(position) != 3 or any(not math.isfinite(value) for value in position)
        for position in requested
    ):
        raise GraspGeometryError("stage positions must contain finite triples")
    _rotation_matrix(approach_orientation_xyzw)
    grasp_rotation = _rotation_matrix(grasp_orientation_xyzw)
    if not math.isfinite(gripper_position_rad):
        raise GraspGeometryError("gripper position must be finite")

    expected = derive_grasp_stage_geometry(
        cube_center_m,
        grasp_orientation_xyzw,
        profile,
    )
    cube_delta = tuple(
        cube_center_m[axis] - descend_position_m[axis] for axis in range(3)
    )
    relative = _inverse_rotate(grasp_rotation, cube_delta)
    if any(
        abs(value - target) > tolerance
        for value, target, tolerance in zip(
            relative,
            profile.target_center_in_frame_at_descend_m,
            profile.target_center_tolerance_m,
            strict=True,
        )
    ):
        raise GraspGeometryError(
            "cube_center_outside_finger_contact_envelope: "
            f"observed={relative},expected="
            f"{profile.target_center_in_frame_at_descend_m}"
        )
    for label, actual, reference in (
        ("descend", descend_position_m, expected.descend_position_m),
        ("lift", lift_position_m, expected.lift_position_m),
    ):
        if any(
            abs(value - target) > profile.stage_position_tolerance_m
            for value, target in zip(actual, reference, strict=True)
        ):
            raise GraspGeometryError(f"{label}_position_drift")
    if (
        abs(gripper_position_rad - profile.gripper_contact_position_rad)
        > profile.gripper_position_tolerance_rad
    ):
        raise GraspGeometryError("gripper_contact_position_mismatch")
    validate_grasp_collision_envelope(
        cube_center_m=cube_center_m,
        approach_position_m=approach_position_m,
        descend_position_m=descend_position_m,
        orientation_xyzw=grasp_orientation_xyzw,
        approach_orientation_xyzw=approach_orientation_xyzw,
        profile=profile,
        cube_orientation_xyzw=cube_orientation_xyzw,
    )
    return RoutedGraspStageGeometry(
        approach_position_m=approach_position_m,
        descend_position_m=descend_position_m,
        lift_position_m=lift_position_m,
        approach_orientation_xyzw=approach_orientation_xyzw,
        grasp_orientation_xyzw=grasp_orientation_xyzw,
        cube_center_in_frame_at_descend_m=relative,
    )
