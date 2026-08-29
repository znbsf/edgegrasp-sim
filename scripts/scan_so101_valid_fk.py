#!/usr/bin/env python3
"""Read-only nearest collision-valid SO-101 FK state scan.

Each candidate is checked through MoveIt's ``GetStateValidity`` service before
its gripper-frame pose is obtained with ``GetPositionFK``.  No action client,
trajectory publisher, or controller command is created.
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass, replace
import json
import math
from pathlib import Path
import time

from edgegrasp.grasp_geometry import (
    GraspGeometryError,
    _inverse_rotate,
    _rotation_matrix,
    cube_obb_in_frame,
    derive_grasp_stage_geometry,
    load_grasp_geometry_profile,
    oriented_box_overlap_margins,
    transform_point_from_frame,
    validate_routed_grasp_stage_geometry,
)
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK, GetStateValidity
import rclpy
from rclpy.node import Node


ARM_JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)
WRIST_SCAN_LIMITS_RAD = (-1.60806, 1.60806)


@dataclass(frozen=True)
class Candidate:
    distance_m: float
    joints: tuple[float, ...]
    position: tuple[float, float, float]
    evaluated_point: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    tool_z_world: float
    cube_center_in_frame: tuple[float, float, float] | None = None
    fixed_pad_margin_m: float | None = None
    moving_pad_closed_margin_m: float | None = None


def _values(lower: float, upper: float, step: float) -> tuple[float, ...]:
    count = int(math.floor((upper - lower) / step))
    values = tuple(lower + index * step for index in range(count + 1))
    if values and math.isclose(values[-1], upper, rel_tol=0.0, abs_tol=1e-12):
        return values
    return (*values, upper)


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", nargs=3, type=float, required=True)
    parser.add_argument(
        "--point-in-frame",
        nargs=3,
        type=float,
        default=(0.0, 0.0, 0.0),
        help=(
            "local point whose parent-frame position is ranked against --target; "
            "defaults to the gripper-frame origin"
        ),
    )
    parser.add_argument("--step", type=float, default=0.55)
    parser.add_argument("--top", type=int, default=12)
    parser.add_argument("--gripper", type=float, default=1.5)
    parser.add_argument("--pan", type=float)
    parser.add_argument("--lift", type=float)
    parser.add_argument("--elbow", type=float)
    parser.add_argument("--wrist", type=float)
    parser.add_argument("--lift-bounds", nargs=2, type=float)
    parser.add_argument("--elbow-bounds", nargs=2, type=float)
    parser.add_argument("--wrist-bounds", nargs=2, type=float)
    parser.add_argument("--min-frame-above-point-m", type=float)
    parser.add_argument("--min-tool-z-world", type=float)
    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument("--roll-values", nargs="+", type=float)
    parser.add_argument(
        "--grasp-profile",
        type=Path,
        help=(
            "optional schema-7 profile; when provided, only candidates passing "
            "its target tolerance, palm clearance, and pre-close pad contract remain"
        ),
    )
    parser.add_argument("--approach-position", nargs=3, type=float)
    parser.add_argument("--approach-orientation", nargs=4, type=float)
    parser.add_argument(
        "--pitch-sum",
        type=float,
        help="hold shoulder_lift + elbow_flex + wrist_flex constant",
    )
    parser.add_argument("--timeout-s", type=float, default=45.0)
    return parser.parse_args()


class Scanner(Node):
    def __init__(self) -> None:
        super().__init__("edgegrasp_read_only_valid_fk_scan")
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        self.validity = self.create_client(
            GetStateValidity, "/check_state_validity"
        )

    @staticmethod
    def state(joints: tuple[float, ...], gripper: float) -> RobotState:
        state = RobotState()
        state.joint_state.header.frame_id = "base_link"
        state.joint_state.name = [*ARM_JOINTS, "gripper"]
        state.joint_state.position = [*joints, gripper]
        state.is_diff = False
        return state

    def _wait(self, future, timeout_s: float, label: str):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done():
            raise TimeoutError(f"{label} response timeout")
        response = future.result()
        if response is None:
            raise RuntimeError(f"{label} returned no response")
        return response

    def evaluate(
        self,
        joints: tuple[float, ...],
        gripper: float,
        timeout_s: float,
    ) -> tuple[Candidate | None, tuple[str, ...]]:
        state = self.state(joints, gripper)
        validity_request = GetStateValidity.Request()
        validity_request.robot_state = state
        validity_request.group_name = "arm"
        validity = self._wait(
            self.validity.call_async(validity_request), timeout_s, "state validity"
        )
        if not validity.valid:
            pairs = tuple(
                "|".join(sorted((item.contact_body_1, item.contact_body_2)))
                for item in validity.contacts
            )
            return None, pairs or ("invalid_without_contact_detail",)

        fk_request = GetPositionFK.Request()
        fk_request.header.frame_id = "base_link"
        fk_request.fk_link_names = ["gripper_frame_link"]
        fk_request.robot_state = state
        fk = self._wait(self.fk.call_async(fk_request), timeout_s, "FK")
        if int(fk.error_code.val) != 1 or len(fk.pose_stamped) != 1:
            return None, (f"fk_error:{int(fk.error_code.val)}",)
        pose = fk.pose_stamped[0].pose
        position = (pose.position.x, pose.position.y, pose.position.z)
        orientation = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        tool_z_tip = transform_point_from_frame(
            position, orientation, (0.0, 0.0, 1.0)
        )
        return (
            Candidate(
                distance_m=0.0,
                joints=joints,
                position=position,
                evaluated_point=position,
                orientation=orientation,
                tool_z_world=tool_z_tip[2] - position[2],
            ),
            (),
        )


def main() -> int:
    args = _arguments()
    if args.roll_values is not None and args.roll != 0.0:
        raise ValueError("roll and roll-values are mutually exclusive")
    geometry_args = (
        args.grasp_profile,
        args.approach_position,
        args.approach_orientation,
    )
    if any(value is not None for value in geometry_args) and not all(
        value is not None for value in geometry_args
    ):
        raise ValueError(
            "grasp-profile, approach-position, and approach-orientation "
            "must be provided together"
        )
    grasp_profile = (
        load_grasp_geometry_profile(args.grasp_profile)
        if args.grasp_profile is not None
        else None
    )
    roll_values = tuple(args.roll_values or (args.roll,))
    optional_joint_values = tuple(
        value
        for value in (args.pan, args.lift, args.elbow, args.wrist, args.roll)
        if value is not None
    )
    values = (
        *args.target,
        *args.point_in_frame,
        args.step,
        args.gripper,
        args.timeout_s,
        *optional_joint_values,
        *roll_values,
        *(args.approach_position or ()),
        *(args.approach_orientation or ()),
    )
    if any(not math.isfinite(value) for value in values):
        raise ValueError("scan arguments must be finite")
    if args.step <= 0.0 or args.timeout_s <= 0.0 or args.top <= 0:
        raise ValueError("step, timeout, and top must be positive")
    if (
        args.min_frame_above_point_m is not None
        and (
            not math.isfinite(args.min_frame_above_point_m)
            or args.min_frame_above_point_m < 0.0
        )
    ):
        raise ValueError("min-frame-above-point-m must be finite and non-negative")
    if (
        args.min_tool_z_world is not None
        and (
            not math.isfinite(args.min_tool_z_world)
            or not -1.0 <= args.min_tool_z_world <= 1.0
        )
    ):
        raise ValueError("min-tool-z-world must be finite and within [-1, 1]")
    for label, value, lower, upper in (
        ("pan", args.pan, -1.91986, 1.91986),
        ("lift", args.lift, -1.74533, 1.74533),
        ("elbow", args.elbow, -1.69, 1.69),
        ("wrist", args.wrist, -1.65806, 1.65806),
        ("roll", None if args.roll_values is not None else args.roll, -2.74385, 2.84121),
    ):
        if value is not None and not lower <= value <= upper:
            raise ValueError(f"{label} is outside the pinned URDF position limits")
    if any(not -2.74385 <= value <= 2.84121 for value in roll_values):
        raise ValueError("roll-values contain a value outside pinned URDF limits")
    bounds_contract = (
        ("lift", args.lift, args.lift_bounds, -1.74533, 1.74533),
        ("elbow", args.elbow, args.elbow_bounds, -1.69, 1.69),
        (
            "wrist",
            args.wrist,
            args.wrist_bounds,
            *WRIST_SCAN_LIMITS_RAD,
        ),
    )
    for label, fixed, bounds, lower, upper in bounds_contract:
        if fixed is not None and bounds is not None:
            raise ValueError(f"{label} and {label}-bounds are mutually exclusive")
        if bounds is None:
            continue
        low, high = bounds
        if (
            not math.isfinite(low)
            or not math.isfinite(high)
            or low > high
            or low < lower
            or high > upper
        ):
            raise ValueError(f"{label}-bounds are invalid")

    rclpy.init()
    scanner = Scanner()
    started = time.monotonic()
    candidates: list[Candidate] = []
    rejected = Counter()
    evaluated = 0
    try:
        for client, name in (
            (scanner.fk, "GetPositionFK"),
            (scanner.validity, "GetStateValidity"),
        ):
            if not client.wait_for_service(timeout_sec=5.0):
                raise TimeoutError(f"{name} unavailable")
        deadline = started + args.timeout_s
        pan_values = (
            (args.pan,) if args.pan is not None else _values(-0.35, 0.35, args.step)
        )
        lift_values = (
            (args.lift,)
            if args.lift is not None
            else _values(
                *(args.lift_bounds or (-1.65, 1.65)),
                args.step,
            )
        )
        elbow_values = (
            (args.elbow,)
            if args.elbow is not None
            else _values(
                *(args.elbow_bounds or (-1.6, 1.6)),
                args.step,
            )
        )
        for pan in pan_values:
            for lift in lift_values:
                for elbow in elbow_values:
                    wrist_values = (
                            (args.wrist,)
                            if args.wrist is not None
                            else (
                                (args.pitch_sum - lift - elbow,)
                                if args.pitch_sum is not None
                                else _values(
                                    *(args.wrist_bounds or WRIST_SCAN_LIMITS_RAD),
                                    args.step,
                                )
                            )
                    )
                    for wrist in wrist_values:
                        if not (
                            WRIST_SCAN_LIMITS_RAD[0]
                            <= wrist
                            <= WRIST_SCAN_LIMITS_RAD[1]
                        ):
                            continue
                        for roll in roll_values:
                            remaining = deadline - time.monotonic()
                            if remaining <= 0.0:
                                raise TimeoutError("bounded valid-FK scan expired")
                            result, reasons = scanner.evaluate(
                                (pan, lift, elbow, wrist, roll),
                                args.gripper,
                                min(remaining, 1.0),
                            )
                            evaluated += 1
                            if result is None:
                                rejected.update(reasons)
                                continue
                            evaluated_point = transform_point_from_frame(
                                result.position,
                                result.orientation,
                                tuple(args.point_in_frame),
                            )
                            if (
                                args.min_frame_above_point_m is not None
                                and result.position[2] - evaluated_point[2]
                                < args.min_frame_above_point_m
                            ):
                                rejected["frame_height_filter"] += 1
                                continue
                            if (
                                args.min_tool_z_world is not None
                                and result.tool_z_world < args.min_tool_z_world
                            ):
                                rejected["tool_z_filter"] += 1
                                continue

                            cube_center_in_frame = None
                            fixed_pad_margin = None
                            moving_pad_margin = None
                            if grasp_profile is not None:
                                cube_delta = tuple(
                                    args.target[axis] - result.position[axis]
                                    for axis in range(3)
                                )
                                cube_center_in_frame = _inverse_rotate(
                                    _rotation_matrix(result.orientation),
                                    cube_delta,
                                )
                                if any(
                                    abs(actual - expected) > tolerance
                                    for actual, expected, tolerance in zip(
                                        cube_center_in_frame,
                                        grasp_profile.target_center_in_frame_at_descend_m,
                                        grasp_profile.target_center_tolerance_m,
                                        strict=True,
                                    )
                                ):
                                    rejected["target_center_tolerance"] += 1
                                    continue
                                candidate_profile = replace(
                                    grasp_profile,
                                    target_center_in_frame_at_descend_m=(
                                        cube_center_in_frame
                                    ),
                                )
                                stages = derive_grasp_stage_geometry(
                                    tuple(args.target),
                                    result.orientation,
                                    candidate_profile,
                                )
                                try:
                                    validate_routed_grasp_stage_geometry(
                                        cube_center_m=tuple(args.target),
                                        approach_position_m=tuple(
                                            args.approach_position
                                        ),
                                        descend_position_m=stages.descend_position_m,
                                        lift_position_m=stages.lift_position_m,
                                        approach_orientation_xyzw=tuple(
                                            args.approach_orientation
                                        ),
                                        grasp_orientation_xyzw=result.orientation,
                                        gripper_position_rad=(
                                            candidate_profile.gripper_contact_position_rad
                                        ),
                                        profile=candidate_profile,
                                    )
                                except GraspGeometryError as error:
                                    rejected[f"geometry:{error}"] += 1
                                    continue
                                cube_box = cube_obb_in_frame(
                                    center_in_frame_m=cube_center_in_frame,
                                    cube_size_m=candidate_profile.cube_size_m,
                                    frame_orientation_xyzw=result.orientation,
                                )
                                fixed_pad_margin = min(
                                    oriented_box_overlap_margins(
                                        cube_box,
                                        candidate_profile.required_contact_pad_obbs[0],
                                    )
                                )
                                moving_pad_margin = min(
                                    oriented_box_overlap_margins(
                                        cube_box,
                                        candidate_profile.required_contact_pad_obbs[1],
                                    )
                                )
                            candidates.append(
                                Candidate(
                                    distance_m=math.dist(
                                        evaluated_point,
                                        args.target,
                                    ),
                                    joints=result.joints,
                                    position=result.position,
                                    evaluated_point=evaluated_point,
                                    orientation=result.orientation,
                                    tool_z_world=result.tool_z_world,
                                    cube_center_in_frame=cube_center_in_frame,
                                    fixed_pad_margin_m=fixed_pad_margin,
                                    moving_pad_closed_margin_m=moving_pad_margin,
                                )
                            )
        candidates.sort(key=lambda item: (item.distance_m, item.joints))
        print(
            json.dumps(
                {
                    "capability": "read_only_moveit_state_validity_plus_fk_no_motion",
                    "elapsed_s": time.monotonic() - started,
                    "evaluated": evaluated,
                    "valid": len(candidates),
                    "invalid": evaluated - len(candidates),
                    "top_rejection_pairs": rejected.most_common(10),
                    "target_m": args.target,
                    "point_in_frame_m": args.point_in_frame,
                    "grasp_profile": (
                        str(args.grasp_profile) if args.grasp_profile else None
                    ),
                    "geometry_gate_enabled": grasp_profile is not None,
                    "roll_values_rad": roll_values,
                    "wrist_scan_limits_rad": WRIST_SCAN_LIMITS_RAD,
                    "results": [
                        {
                            "distance_m": item.distance_m,
                            "joints": dict(zip(ARM_JOINTS, item.joints, strict=True)),
                            "position_m": item.position,
                            "evaluated_point_m": item.evaluated_point,
                            "orientation_xyzw": item.orientation,
                            "tool_z_world": item.tool_z_world,
                            "cube_center_in_frame_m": item.cube_center_in_frame,
                            "fixed_pad_margin_m": item.fixed_pad_margin_m,
                            "moving_pad_closed_margin_m": (
                                item.moving_pad_closed_margin_m
                            ),
                        }
                        for item in candidates[: args.top]
                    ],
                },
                sort_keys=True,
                indent=2,
            )
        )
        return 0
    finally:
        scanner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
