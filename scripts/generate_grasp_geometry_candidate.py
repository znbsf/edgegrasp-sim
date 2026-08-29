"""Render a gripper-angle-specific SO-101 static grasp geometry profile.

The calculation uses the fixed-SHA kinematic origins recorded in the collision
proxy contract and the moving distal-pad box derived from the pinned STL. It
does not call ROS, MoveIt, Gazebo, or any motion interface. Generated overlap is
still only a static OBB preflight and is not contact, force-closure, or grasp
evidence.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
from typing import Any

from edgegrasp.collision_proxy import (
    load_collision_proxy_contract,
    with_moving_pad_distal_extension,
)
from edgegrasp.grasp_geometry import load_grasp_geometry_profile


Matrix3 = tuple[tuple[float, float, float], ...]
Vector3 = tuple[float, float, float]


def _matrix_multiply(first: Matrix3, second: Matrix3) -> Matrix3:
    return tuple(
        tuple(
            sum(first[row][axis] * second[axis][column] for axis in range(3))
            for column in range(3)
        )
        for row in range(3)
    )


def _matrix_vector(matrix: Matrix3, vector: Vector3) -> Vector3:
    return tuple(
        sum(matrix[row][axis] * vector[axis] for axis in range(3))
        for row in range(3)
    )


def _transpose(matrix: Matrix3) -> Matrix3:
    return tuple(tuple(matrix[column][row] for column in range(3)) for row in range(3))


def _rpy_matrix(values: list[float]) -> Matrix3:
    roll, pitch, yaw = (float(value) for value in values)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _axis_angle(axis: list[float], angle: float) -> Matrix3:
    x, y, z = (float(value) for value in axis)
    norm = math.sqrt(x * x + y * y + z * z)
    if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-12):
        raise ValueError("gripper axis must be normalized")
    x, y, z = x / norm, y / norm, z / norm
    cosine, sine, one_minus = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return (
        (
            cosine + x * x * one_minus,
            x * y * one_minus - z * sine,
            x * z * one_minus + y * sine,
        ),
        (
            y * x * one_minus + z * sine,
            cosine + y * y * one_minus,
            y * z * one_minus - x * sine,
        ),
        (
            z * x * one_minus - y * sine,
            z * y * one_minus + x * sine,
            cosine + z * z * one_minus,
        ),
    )


def _vector(values: list[float]) -> Vector3:
    return tuple(float(value) for value in values)  # type: ignore[return-value]


def _moving_pad_envelope(
    contract: dict[str, Any], angle: float
) -> tuple[Vector3, Matrix3, Vector3, Vector3, Vector3]:
    kinematics = contract["gripper_contact_kinematics"]
    reference = kinematics["reference_joint"]
    moving_joint = kinematics["moving_joint"]
    lower, upper = float(moving_joint["lower_rad"]), float(moving_joint["upper_rad"])
    if not math.isfinite(angle) or not lower <= angle <= upper:
        raise ValueError(f"gripper angle outside pinned limits: {angle}")

    moving_proxy = next(
        item
        for item in contract["gazebo_gripper_contact_extensions"]
        if item["name"] == "moving_finger_pad"
    )
    parent_to_reference = _rpy_matrix(reference["origin_rpy"])
    parent_to_moving = _matrix_multiply(
        _rpy_matrix(moving_joint["origin_rpy"]),
        _axis_angle(moving_joint["axis_xyz"], angle),
    )
    reference_to_parent = _transpose(parent_to_reference)
    reference_to_moving = _matrix_multiply(reference_to_parent, parent_to_moving)
    parent_delta = tuple(
        value - reference_value
        for value, reference_value in zip(
            _vector(moving_joint["origin_xyz"]),
            _vector(reference["origin_xyz"]),
            strict=True,
        )
    )
    reference_to_moving_translation = _matrix_vector(reference_to_parent, parent_delta)
    moving_to_box = _rpy_matrix(moving_proxy["proxy_pose_rpy"])
    rotation = _matrix_multiply(reference_to_moving, moving_to_box)
    offset = _matrix_vector(reference_to_moving, _vector(moving_proxy["proxy_pose_xyz"]))
    center = tuple(
        reference_to_moving_translation[index] + offset[index] for index in range(3)
    )
    size = _vector(moving_proxy["proxy_size_m"])
    half_extent = tuple(
        sum(abs(rotation[row][column]) * size[column] / 2.0 for column in range(3))
        for row in range(3)
    )
    minimum = tuple(center[index] - half_extent[index] for index in range(3))
    maximum = tuple(center[index] + half_extent[index] for index in range(3))
    return center, rotation, size, minimum, maximum


def render_candidate_profile(
    source_profile: Path,
    proxy_contract: Path,
    *,
    gripper_position_rad: float,
    name: str,
    target_center_x_offset_m: float = 0.0,
    moving_pad_distal_extension_m: float = 0.0,
) -> dict[str, Any]:
    load_grasp_geometry_profile(source_profile)
    contract = load_collision_proxy_contract(proxy_contract)
    contract = with_moving_pad_distal_extension(
        contract, moving_pad_distal_extension_m
    )
    payload = deepcopy(json.loads(source_profile.read_text(encoding="utf-8")))
    center, rotation, size, minimum, maximum = _moving_pad_envelope(
        contract, gripper_position_rad
    )
    payload["name"] = name
    payload["gripper_contact_position_rad"] = gripper_position_rad
    center_offset = float(target_center_x_offset_m)
    if not math.isfinite(center_offset):
        raise ValueError("target center X offset must be finite")
    target_center = payload["target_center_in_frame_at_descend_m"]
    tolerance = float(payload["target_center_tolerance_m"][0])
    if abs(center_offset) > tolerance:
        raise ValueError("target center X offset exceeds profile tolerance")
    if center_offset != 0.0:
        original_center = list(target_center)
        target_center[0] = float(target_center[0]) + center_offset
    geometry_source = payload["geometry_source"]
    fixed_aabb = payload["gazebo_proxy_preflight"]["required_contact_pad_aabbs"][0]
    geometry_source["predicted_inner_gap_at_contact_position_m"] = (
        float(fixed_aabb["minimum_m"][0]) - maximum[0]
    )
    geometry_source["generated_gripper_position_rad"] = gripper_position_rad
    geometry_source["generator"] = "scripts/generate_grasp_geometry_candidate.py"
    if moving_pad_distal_extension_m != 0.0:
        geometry_source["moving_pad_distal_extension_m"] = float(
            moving_pad_distal_extension_m
        )
        geometry_source["geometry_role"] = "edgegrasp_simulation_attachment"
    if moving_pad_distal_extension_m == 0.0:
        geometry_source["derivation"] = (
            "Moving-pad OBB/AABB regenerated from the fixed-SHA gripper frame/joint "
            "origins and EdgeGrasp STL-derived distal-pad box. This is static geometry "
            "evidence only; runtime contact, lift, and retention remain required."
        )
    else:
        geometry_source["derivation"] = (
            "Moving-pad OBB/AABB regenerated from the fixed-SHA gripper frame/joint "
            "origins plus one bounded EdgeGrasp simulation attachment that extends "
            "only the distal local-Y contact length. Runtime contact, lift, and "
            "retention remain required."
        )
    preflight = payload["gazebo_proxy_preflight"]
    moving_aabb = preflight["required_contact_pad_aabbs"][1]
    moving_aabb["minimum_m"] = list(minimum)
    moving_aabb["maximum_m"] = list(maximum)
    moving_obb = preflight["required_contact_pad_obbs"][1]
    moving_obb["center_m"] = list(center)
    moving_obb["rotation_rows"] = [list(row) for row in rotation]
    moving_obb["size_m"] = list(size)
    preflight["source"] = (
        "Pinned URDF primitive boxes plus EdgeGrasp distal-pad proxies transformed "
        f"into gripper_frame_link; moving pad evaluated at gripper={gripper_position_rad:.6f} rad."
    )
    if moving_pad_distal_extension_m != 0.0:
        preflight["source"] += (
            " The moving pad includes an EdgeGrasp simulation attachment with "
            f"{moving_pad_distal_extension_m:.6f} m extra distal local-Y length."
        )
    contact_marker = " rad contact position"
    claims: list[str] = []
    for line in payload["claim_boundary"]:
        if line.startswith("The ") and contact_marker in line:
            suffix = line.split(contact_marker, maxsplit=1)[1]
            line = f"The {gripper_position_rad:.2f}{contact_marker}{suffix}"
        claims.append(line)
    payload["claim_boundary"] = claims
    if moving_pad_distal_extension_m != 0.0:
        payload["claim_boundary"].append(
            "The moving-pad distal extension is an EdgeGrasp simulation attachment, "
            "not pinned upstream mesh geometry or a verified hardware design."
        )
    if center_offset != 0.0:
        geometry_source["source_target_center_in_frame_m"] = original_center
        geometry_source["target_center_x_offset_m"] = center_offset
        geometry_source["target_center_selection_method"] = (
            "equalize_two_pad_minimum_normalized_sat_overlap"
        )
        payload["claim_boundary"].append(
            "The target-center offset is a static two-pad OBB symmetry choice; "
            "it is not MoveIt, dynamics, force-closure, or grasp evidence."
        )
    return payload


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-profile", type=Path, required=True)
    parser.add_argument("--proxy-contract", type=Path, required=True)
    parser.add_argument("--gripper-position-rad", type=float, required=True)
    parser.add_argument("--target-center-x-offset-m", type=float, default=0.0)
    parser.add_argument(
        "--moving-pad-distal-extension-m", type=float, default=0.0
    )
    parser.add_argument("--name", required=True)
    args = parser.parse_args()
    rendered = render_candidate_profile(
        args.source_profile,
        args.proxy_contract,
        gripper_position_rad=args.gripper_position_rad,
        name=args.name,
        target_center_x_offset_m=args.target_center_x_offset_m,
        moving_pad_distal_extension_m=args.moving_pad_distal_extension_m,
    )
    print(json.dumps(rendered, indent=2, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
