#!/usr/bin/env python3
"""Fail-closed validation for one Candidate012 matrix row."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

from edgegrasp.contact_material import (
    load_pad_contact_material_contract,
    select_pad_contact_material_profile,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONFIG_ROOT = PROJECT_ROOT / "ros_ws" / "src" / "edgegrasp_ros" / "config"
DEFAULT_EXPERIMENT = CONFIG_ROOT / "candidate012_pad_friction_experiment.json"


class Candidate012ContractError(ValueError):
    """Raised when Candidate012 fixed inputs or hashes drift."""


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _require_hash(path: Path, expected: str, *, label: str) -> str:
    observed = _sha256(path)
    if observed != expected:
        raise Candidate012ContractError(
            f"{label} hash drift: expected={expected},observed={observed}"
        )
    return observed


def validate_candidate012_experiment(
    *, experiment_path: Path, config_root: Path, world_root: Path, profile_name: str
) -> dict:
    experiment = json.loads(experiment_path.read_text(encoding="utf-8"))
    if experiment.get("schema_version") != 1:
        raise Candidate012ContractError("unsupported Candidate012 schema")
    if experiment.get("experiment_id") != "candidate012_pad_friction":
        raise Candidate012ContractError("unexpected Candidate012 experiment id")
    fixed = experiment["fixed_inputs"]
    material = experiment["material_contract"]
    paths = {
        "grasp_geometry": config_root / fixed["grasp_geometry_filename"],
        "scene": config_root / fixed["scene_filename"],
        "gazebo_world": world_root / fixed["gazebo_world_filename"],
        "collision_proxy": config_root / fixed["collision_proxy_filename"],
        "material_contract": config_root / material["filename"],
    }
    hashes = {
        "grasp_geometry": _require_hash(
            paths["grasp_geometry"],
            fixed["grasp_geometry_sha256"],
            label="grasp geometry",
        ),
        "scene": _require_hash(
            paths["scene"], fixed["scene_sha256"], label="scene"
        ),
        "gazebo_world": _require_hash(
            paths["gazebo_world"],
            fixed["gazebo_world_sha256"],
            label="Gazebo world",
        ),
        "collision_proxy": _require_hash(
            paths["collision_proxy"],
            fixed["collision_proxy_sha256"],
            label="collision proxy",
        ),
        "material_contract": _require_hash(
            paths["material_contract"],
            material["sha256"],
            label="material contract",
        ),
    }

    geometry = json.loads(paths["grasp_geometry"].read_text(encoding="utf-8"))
    contact_position = float(geometry["gripper_contact_position_rad"])
    if not math.isclose(
        contact_position,
        float(fixed["gripper_contact_position_rad"]),
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        raise Candidate012ContractError("gripper contact position drift")
    if geometry["upstream"]["commit"] != experiment["upstream"]["commit"]:
        raise Candidate012ContractError("geometry/upstream commit drift")

    rows = {row["profile"]: row for row in experiment["matrix"]}
    if profile_name not in rows:
        raise Candidate012ContractError(
            f"profile is not in the Candidate012 matrix: {profile_name}"
        )
    material_contract = load_pad_contact_material_contract(
        paths["material_contract"]
    )
    selected = select_pad_contact_material_profile(material_contract, profile_name)
    configured = rows[profile_name]["coefficient"]
    if configured is None:
        if selected.coefficient is not None:
            raise Candidate012ContractError("implicit row assigned a coefficient")
    elif not math.isclose(
        float(configured), float(selected.coefficient), rel_tol=0.0, abs_tol=1e-12
    ):
        raise Candidate012ContractError("matrix/material coefficient drift")

    return {
        "status": "PASS",
        "experiment_id": experiment["experiment_id"],
        "profile": profile_name,
        "matrix_row": rows[profile_name]["row"],
        "changed_parameter": experiment["changed_parameter"],
        "configured_coefficient": selected.coefficient,
        "gripper_contact_position_rad": contact_position,
        "hashes": hashes,
        "fixed_runtime_inputs": {
            field: fixed[field]
            for field in (
                "target_id",
                "planning_group",
                "planning_frame",
                "pipeline_id",
                "planner_id",
                "velocity_scaling",
                "acceleration_scaling",
                "use_sim_time",
                "clock_domain",
                "clock_epoch",
                "future_skew_tolerance_ms",
                "target_publisher_start_delay_s",
            )
        },
        "claim_boundary": experiment["claim_boundary"],
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", required=True)
    parser.add_argument("--experiment", type=Path, default=DEFAULT_EXPERIMENT)
    parser.add_argument("--config-root", type=Path, default=CONFIG_ROOT)
    parser.add_argument(
        "--world-root",
        type=Path,
        default=PROJECT_ROOT / "ros_ws" / "src" / "edgegrasp_ros" / "worlds",
    )
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = validate_candidate012_experiment(
        experiment_path=args.experiment,
        config_root=args.config_root,
        world_root=args.world_root,
        profile_name=args.profile,
    )
    payload = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload, encoding="utf-8")
    print(payload, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
