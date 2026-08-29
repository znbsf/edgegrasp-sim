"""Select one bounded tool-X offset that equalizes two static pad overlaps.

This is dependency-free, read-only geometry analysis. It neither sends motion
nor predicts dynamics, friction, force closure, or grasp success. The selected
offset must still pass MoveIt plan-only and a fresh-domain physics trial.
"""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
import math
from pathlib import Path

from edgegrasp.grasp_geometry import (
    GraspGeometryProfile,
    cube_obb_in_frame,
    load_grasp_geometry_profile,
    oriented_box_overlap_margins,
)


def _finite(value: float, label: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"{label} must be finite")
    return parsed


def _pad_overlaps(
    profile: GraspGeometryProfile,
    center_m: tuple[float, float, float],
    orientation_xyzw: tuple[float, float, float, float],
) -> tuple[float, float]:
    if len(profile.required_contact_pad_obbs) != 2:
        raise ValueError("symmetry selector requires exactly two contact pads")
    cube = cube_obb_in_frame(
        center_in_frame_m=center_m,
        cube_size_m=profile.cube_size_m,
        frame_orientation_xyzw=orientation_xyzw,
    )
    return tuple(
        min(oriented_box_overlap_margins(cube, pad))
        for pad in profile.required_contact_pad_obbs
    )  # type: ignore[return-value]


def select_offset(
    profile: GraspGeometryProfile,
    *,
    orientation_xyzw: tuple[float, float, float, float],
    minimum_offset_m: float,
    maximum_offset_m: float,
    step_m: float,
) -> dict[str, object]:
    minimum = _finite(minimum_offset_m, "minimum_offset_m")
    maximum = _finite(maximum_offset_m, "maximum_offset_m")
    step = _finite(step_m, "step_m")
    if minimum > 0.0 or maximum < 0.0 or minimum >= maximum:
        raise ValueError("offset range must be ordered and include zero")
    if step <= 0.0:
        raise ValueError("step_m must be positive")
    span_steps = (maximum - minimum) / step
    count = int(round(span_steps))
    if not math.isclose(span_steps, count, rel_tol=0.0, abs_tol=1e-9):
        raise ValueError("offset range must contain an integral number of steps")
    if count > 100_000:
        raise ValueError("offset scan exceeds 100000 steps")
    orientation = tuple(_finite(value, "orientation") for value in orientation_xyzw)
    if len(orientation) != 4:
        raise ValueError("orientation_xyzw must contain four values")
    norm = math.sqrt(sum(value * value for value in orientation))
    if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
        raise ValueError("orientation_xyzw must be normalized")

    base = profile.target_center_in_frame_at_descend_m
    candidates: list[tuple[tuple[float, float, float, float], dict[str, object]]] = []
    for index in range(count + 1):
        offset = round(minimum + index * step, 15)
        if abs(offset) > profile.target_center_tolerance_m[0]:
            continue
        center = (round(base[0] + offset, 15), base[1], base[2])
        overlaps = _pad_overlaps(profile, center, orientation)
        if min(overlaps) < profile.minimum_contact_overlap_m:
            continue
        difference = abs(overlaps[0] - overlaps[1])
        payload = {
            "offset_x_m": offset,
            "target_center_in_frame_m": center,
            "minimum_normalized_sat_overlap_m_by_pad": dict(
                zip(
                    (pad.name for pad in profile.required_contact_pad_obbs),
                    overlaps,
                    strict=True,
                )
            ),
            "absolute_overlap_difference_m": difference,
            "bottleneck_overlap_m": min(overlaps),
        }
        objective = (difference, -min(overlaps), abs(offset), offset)
        candidates.append((objective, payload))
    if not candidates:
        raise ValueError("no offset satisfies the static contact-overlap gate")

    base_overlaps = _pad_overlaps(profile, base, orientation)
    selected = min(candidates, key=lambda item: item[0])[1]
    return {
        "selection_method": "equalize_two_pad_minimum_normalized_sat_overlap",
        "claim_boundary": (
            "Static primitive OBB selection only; MoveIt swept-path validation, "
            "Gazebo contact symmetry, lift, retention, and hardware remain required."
        ),
        "scan": {
            "axis": "end_effector_frame_x",
            "minimum_offset_m": minimum,
            "maximum_offset_m": maximum,
            "step_m": step,
            "candidate_count": len(candidates),
            "minimum_required_overlap_m": profile.minimum_contact_overlap_m,
        },
        "baseline": {
            "offset_x_m": 0.0,
            "target_center_in_frame_m": base,
            "minimum_normalized_sat_overlap_m_by_pad": dict(
                zip(
                    (pad.name for pad in profile.required_contact_pad_obbs),
                    base_overlaps,
                    strict=True,
                )
            ),
            "absolute_overlap_difference_m": abs(base_overlaps[0] - base_overlaps[1]),
            "bottleneck_overlap_m": min(base_overlaps),
        },
        "selected": selected,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument(
        "--grasp-orientation-xyzw", type=float, nargs=4, required=True
    )
    parser.add_argument("--minimum-offset-m", type=float, default=-0.003)
    parser.add_argument("--maximum-offset-m", type=float, default=0.003)
    parser.add_argument("--step-m", type=float, default=0.0001)
    args = parser.parse_args()
    profile = load_grasp_geometry_profile(args.profile)
    result = select_offset(
        profile,
        orientation_xyzw=tuple(args.grasp_orientation_xyzw),
        minimum_offset_m=args.minimum_offset_m,
        maximum_offset_m=args.maximum_offset_m,
        step_m=args.step_m,
    )
    result["profile"] = args.profile.name
    result["profile_sha256"] = sha256(args.profile.read_bytes()).hexdigest()
    result["grasp_orientation_xyzw"] = tuple(args.grasp_orientation_xyzw)
    print(json.dumps(result, indent=2, sort_keys=False, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
