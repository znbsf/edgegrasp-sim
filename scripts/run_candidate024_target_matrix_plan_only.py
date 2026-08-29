#!/usr/bin/env python3
"""Materialize and run the Candidate024 zero-execution target matrix.

Each valid case moves the shared scene cube and every Candidate024 Cartesian
stage by the same world-frame translation.  The existing isolated
``GetMotionPlan`` harness validates and discards all returned trajectories.
Cases rejected by the scene contract never start ROS or Gazebo.
"""

from __future__ import annotations

import argparse
from collections import Counter
from copy import deepcopy
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PROJECT_ROOT / "src"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from edgegrasp.grasp_geometry import (  # noqa: E402
    derive_grasp_stage_geometry,
    load_grasp_geometry_profile,
    validate_routed_grasp_stage_geometry,
)
from edgegrasp.scene import (  # noqa: E402
    SceneContractError,
    load_scene_contract,
    render_gazebo_sdf,
)


CONFIG_DIR = PROJECT_ROOT / "ros_ws" / "src" / "edgegrasp_ros" / "config"
WORLD_DIR = PROJECT_ROOT / "ros_ws" / "src" / "edgegrasp_ros" / "worlds"
DEFAULT_MATRIX = CONFIG_DIR / "candidate024_target_matrix.json"
DEFAULT_INSTALL_SHARE = Path(
    "/home/edgegrasp/ros2_ws/install/edgegrasp_ros/share/edgegrasp_ros"
)
RUNTIME_ARTIFACT_ROOT = Path("/home/edgegrasp/ros2_ws/test_results")
SAFE_ID = re.compile(r"^[a-z][a-z0-9_]{2,63}$")
SHOULDER_PAN_ORIGIN_BASE_M = (0.0388353, -8.97657e-09, 0.0624)
SHOULDER_PAN_AXIS_BASE = (0.0, 0.0, -1.0)


class MatrixError(ValueError):
    """Raised when the matrix or its generated evidence is inconsistent."""


def _read_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise MatrixError(f"cannot read JSON {path}: {error}") from error
    if not isinstance(value, dict):
        raise MatrixError(f"JSON root must be an object: {path}")
    return value


def _write_object(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
        newline="\n",
    )


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _numbers(value: Any, *, field: str) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise MatrixError(f"{field} must contain three numbers")
    if any(
        not isinstance(item, (int, float))
        or isinstance(item, bool)
        or not math.isfinite(float(item))
        for item in value
    ):
        raise MatrixError(f"{field} contains an invalid number")
    return (float(value[0]), float(value[1]), float(value[2]))


def _translated(
    value: Any, delta: tuple[float, float, float], *, field: str
) -> list[float]:
    source = _numbers(value, field=field)
    return [source[index] + delta[index] for index in range(3)]


def _quaternion(value: Any, *, field: str) -> tuple[float, float, float, float]:
    if not isinstance(value, list) or len(value) != 4:
        raise MatrixError(f"{field} must contain four numbers")
    if any(
        not isinstance(item, (int, float))
        or isinstance(item, bool)
        or not math.isfinite(float(item))
        for item in value
    ):
        raise MatrixError(f"{field} contains an invalid number")
    result = tuple(float(item) for item in value)
    norm = math.sqrt(sum(item * item for item in result))
    if abs(norm - 1.0) > 1e-9:
        raise MatrixError(f"{field} must be normalized")
    return result


def _case_transform(item: dict[str, Any]) -> dict[str, Any]:
    has_translation = "translation_world_m" in item
    has_pan_delta = "shoulder_pan_delta_rad" in item
    if has_translation == has_pan_delta:
        raise MatrixError(
            f"{item.get('case_id')} must define exactly one supported transform"
        )
    if has_translation:
        delta = _numbers(
            item.get("translation_world_m"),
            field=f"cases.{item.get('case_id')}.translation_world_m",
        )
        if delta == (0.0, 0.0, 0.0):
            raise MatrixError(
                f"{item.get('case_id')} must not duplicate the Candidate024 baseline"
            )
        return {"type": "translation", "delta_world_m": list(delta)}
    angle = item.get("shoulder_pan_delta_rad")
    if (
        not isinstance(angle, (int, float))
        or isinstance(angle, bool)
        or not math.isfinite(float(angle))
        or float(angle) == 0.0
        or abs(float(angle)) > 0.1
    ):
        raise MatrixError(
            f"cases.{item.get('case_id')}.shoulder_pan_delta_rad must be in [-0.1, 0.1] and nonzero"
        )
    joint_delta = float(angle)
    return {
        "type": "shoulder_pan_symmetry",
        "joint_delta_rad": joint_delta,
        "world_yaw_rad": -joint_delta,
        "joint_origin_base_m": list(SHOULDER_PAN_ORIGIN_BASE_M),
        "joint_axis_base": list(SHOULDER_PAN_AXIS_BASE),
    }


def _transform_point(
    value: Any, transform: dict[str, Any], *, field: str
) -> list[float]:
    source = _numbers(value, field=field)
    if transform["type"] == "translation":
        return _translated(source, tuple(transform["delta_world_m"]), field=field)
    angle = float(transform["world_yaw_rad"])
    cosine = math.cos(angle)
    sine = math.sin(angle)
    pivot_x, pivot_y, _ = SHOULDER_PAN_ORIGIN_BASE_M
    relative_x = source[0] - pivot_x
    relative_y = source[1] - pivot_y
    return [
        pivot_x + cosine * relative_x - sine * relative_y,
        pivot_y + sine * relative_x + cosine * relative_y,
        source[2],
    ]


def _transform_quaternion(
    value: Any, transform: dict[str, Any], *, field: str
) -> list[float]:
    source = _quaternion(value, field=field)
    if transform["type"] == "translation":
        return list(source)
    half = float(transform["world_yaw_rad"]) / 2.0
    yaw = (0.0, 0.0, math.sin(half), math.cos(half))
    ax, ay, az, aw = yaw
    bx, by, bz, bw = source
    result = (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )
    norm = math.sqrt(sum(item * item for item in result))
    return [item / norm for item in result]


def _baseline_paths(matrix: dict[str, Any]) -> dict[str, Path]:
    baseline = matrix.get("baseline")
    if not isinstance(baseline, dict):
        raise MatrixError("baseline must be an object")
    result: dict[str, Path] = {}
    for key, directory, suffix in (
        ("scene", CONFIG_DIR, ".json"),
        ("world", WORLD_DIR, ".sdf"),
        ("candidate", CONFIG_DIR, ".json"),
        ("geometry", CONFIG_DIR, ".json"),
    ):
        row = baseline.get(key)
        if not isinstance(row, dict):
            raise MatrixError(f"baseline.{key} must be an object")
        filename = row.get("filename")
        expected_hash = row.get("sha256")
        if (
            not isinstance(filename, str)
            or Path(filename).name != filename
            or not filename.endswith(suffix)
        ):
            raise MatrixError(f"baseline.{key}.filename is not a safe basename")
        if not isinstance(expected_hash, str) or not re.fullmatch(
            r"[0-9a-f]{64}", expected_hash
        ):
            raise MatrixError(f"baseline.{key}.sha256 is invalid")
        path = directory / filename
        if not path.is_file():
            raise MatrixError(f"baseline file is missing: {path}")
        observed_hash = _sha256(path)
        if observed_hash != expected_hash:
            raise MatrixError(
                f"baseline hash drift for {key}: {observed_hash} != {expected_hash}"
            )
        result[key] = path
    return result


def _validate_matrix(matrix: dict[str, Any]) -> list[dict[str, Any]]:
    if matrix.get("schema_version") != 1:
        raise MatrixError("unsupported matrix schema")
    if matrix.get("experiment_id") != "candidate024_target_matrix_plan_only":
        raise MatrixError("unexpected experiment_id")
    if matrix.get("mode") != "moveit_get_motion_plan_zero_execution":
        raise MatrixError("matrix mode must be zero execution")
    planning = matrix.get("planning")
    if not isinstance(planning, dict) or planning.get("attempts_per_case") != 1:
        raise MatrixError("first matrix requires exactly one attempt per case")
    if planning.get("pipeline_id") != "pilz_industrial_motion_planner":
        raise MatrixError("matrix must pin the Pilz pipeline")
    if planning.get("planner_id") != "PTP":
        raise MatrixError("matrix must pin the PTP planner")
    generator = matrix.get("reachable_case_generator")
    if not isinstance(generator, dict):
        raise MatrixError("reachable_case_generator must be an object")
    if generator.get("type") != "shoulder_pan_symmetry":
        raise MatrixError("reachable cases must use shoulder_pan symmetry")
    if _numbers(
        generator.get("joint_origin_base_m"),
        field="reachable_case_generator.joint_origin_base_m",
    ) != SHOULDER_PAN_ORIGIN_BASE_M:
        raise MatrixError("shoulder_pan origin drifted from the pinned URDF")
    if _numbers(
        generator.get("joint_axis_base"),
        field="reachable_case_generator.joint_axis_base",
    ) != SHOULDER_PAN_AXIS_BASE:
        raise MatrixError("shoulder_pan axis drifted from the pinned URDF")

    required = matrix.get("required_case_counts")
    expected_counts = {"reachable_hypothesis": 10, "rejection_hypothesis": 10}
    if required != expected_counts:
        raise MatrixError("matrix must require exactly 10 reachable and 10 rejection cases")
    cases = matrix.get("cases")
    if not isinstance(cases, list):
        raise MatrixError("cases must be an array")
    counts: Counter[str] = Counter()
    identities: set[str] = set()
    result: list[dict[str, Any]] = []
    for index, item in enumerate(cases):
        if not isinstance(item, dict):
            raise MatrixError(f"cases[{index}] must be an object")
        case_id = item.get("case_id")
        if not isinstance(case_id, str) or SAFE_ID.fullmatch(case_id) is None:
            raise MatrixError(f"cases[{index}].case_id is invalid")
        if case_id in identities:
            raise MatrixError(f"duplicate case_id: {case_id}")
        identities.add(case_id)
        hypothesis = item.get("hypothesis")
        if hypothesis not in expected_counts:
            raise MatrixError(f"unsupported hypothesis for {case_id}")
        counts[str(hypothesis)] += 1
        transform = _case_transform(item)
        if (
            hypothesis == "reachable_hypothesis"
            and transform["type"] != "shoulder_pan_symmetry"
        ):
            raise MatrixError(
                f"{case_id} reachable hypothesis must use the base-yaw transform"
            )
        if hypothesis == "rejection_hypothesis" and transform["type"] != "translation":
            raise MatrixError(
                f"{case_id} rejection hypothesis must use an explicit translation"
            )
        layer = item.get("admission_layer")
        status = item.get("expected_status")
        valid_pair = (
            layer == "moveit_plan_only"
            and status in {"PLAN_ONLY_PASS", "PLAN_ONLY_REJECTED"}
        ) or (
            layer == "scene_contract"
            and status == "SCENE_CONTRACT_REJECTED"
            and isinstance(item.get("expected_reason"), str)
        )
        if not valid_pair:
            raise MatrixError(f"invalid admission/status contract for {case_id}")
        result.append(item)
    if dict(counts) != expected_counts:
        raise MatrixError(f"case count mismatch: {dict(counts)}")
    return result


def _target_row(scene: dict[str, Any]) -> dict[str, Any]:
    objects = scene.get("objects")
    if not isinstance(objects, list):
        raise MatrixError("scene objects are missing")
    rows = [item for item in objects if isinstance(item, dict) and item.get("id") == "target_cube"]
    if len(rows) != 1:
        raise MatrixError("scene must contain exactly one target_cube")
    return rows[0]


def _validate_candidate_scene_geometry(
    *,
    candidate: dict[str, Any],
    scene_path: Path,
    geometry_path: Path,
) -> dict[str, list[float]]:
    scene = load_scene_contract(scene_path)
    cube = next(item for item in scene.objects if item.object_id == "target_cube")
    target = candidate.get("target")
    if not isinstance(target, dict) or target.get("id") != "target_cube":
        raise MatrixError("candidate target identity is invalid")
    target_center = _numbers(target.get("center_m"), field="candidate.target.center_m")
    if target_center != cube.pose_world.position_m:
        raise MatrixError("candidate target center and scene cube center disagree")
    if candidate.get("planning_frame") != "base_link":
        raise MatrixError("candidate planning frame must be base_link")
    if candidate.get("planning_group") != "arm":
        raise MatrixError("candidate planning group must be arm")

    profile = load_grasp_geometry_profile(geometry_path)
    orientations = candidate.get("orientations_xyzw")
    stages = candidate.get("stages")
    if not isinstance(orientations, dict) or not isinstance(stages, dict):
        raise MatrixError("candidate stages or orientations are missing")
    approach = _numbers(
        stages.get("approach", {}).get("position_m"),
        field="candidate.stages.approach.position_m",
    )
    approach_orientation = tuple(float(item) for item in orientations.get("approach", []))
    grasp_orientation = tuple(float(item) for item in orientations.get("grasp", []))
    if len(approach_orientation) != 4 or len(grasp_orientation) != 4:
        raise MatrixError("candidate orientations must contain four values")
    derived = derive_grasp_stage_geometry(
        cube.pose_world.position_m,
        grasp_orientation,
        profile,
    )
    routed = validate_routed_grasp_stage_geometry(
        cube_center_m=cube.pose_world.position_m,
        approach_position_m=approach,
        descend_position_m=derived.descend_position_m,
        lift_position_m=derived.lift_position_m,
        approach_orientation_xyzw=approach_orientation,
        grasp_orientation_xyzw=grasp_orientation,
        gripper_position_rad=profile.gripper_contact_position_rad,
        profile=profile,
        cube_orientation_xyzw=cube.pose_world.quaternion_xyzw,
    )
    return {
        "target_center_m": list(cube.pose_world.position_m),
        "approach_position_m": list(routed.approach_position_m),
        "descend_position_m": list(routed.descend_position_m),
        "lift_position_m": list(routed.lift_position_m),
    }


def materialize(matrix_path: Path, artifact_dir: Path) -> dict[str, Any]:
    matrix = _read_object(matrix_path)
    cases = _validate_matrix(matrix)
    baseline_paths = _baseline_paths(matrix)
    baseline_scene = _read_object(baseline_paths["scene"])
    baseline_candidate = _read_object(baseline_paths["candidate"])
    baseline_contract = load_scene_contract(baseline_paths["scene"])
    if baseline_paths["world"].read_text(encoding="utf-8") != render_gazebo_sdf(
        baseline_contract
    ):
        raise MatrixError("Candidate024 baseline world has drifted from its scene")
    baseline_geometry = _validate_candidate_scene_geometry(
        candidate=baseline_candidate,
        scene_path=baseline_paths["scene"],
        geometry_path=baseline_paths["geometry"],
    )
    baseline_target_center = tuple(baseline_geometry["target_center_m"])

    token = hashlib.sha256(str(artifact_dir.resolve()).encode("utf-8")).hexdigest()[:8]
    materialized_dir = artifact_dir / "materialized"
    rows: list[dict[str, Any]] = []
    for item in cases:
        case_id = str(item["case_id"])
        transform = _case_transform(item)
        prefix = f"egm_{token}_{case_id}"
        scene_filename = f"{prefix}_scene.json"
        candidate_filename = f"{prefix}_candidate.json"
        world_filename = f"{prefix}_world.sdf"
        case_dir = materialized_dir / case_id
        scene_path = case_dir / scene_filename
        candidate_path = case_dir / candidate_filename
        world_path = case_dir / world_filename

        scene = deepcopy(baseline_scene)
        target = _target_row(scene)
        target_pose = target.get("pose_world")
        if not isinstance(target_pose, dict):
            raise MatrixError("target_cube pose_world is missing")
        target_pose["position_m"] = _transform_point(
            target_pose.get("position_m"),
            transform,
            field="scene.target_cube.position_m",
        )
        target_pose["quaternion_xyzw"] = _transform_quaternion(
            target_pose.get("quaternion_xyzw"),
            transform,
            field="scene.target_cube.quaternion_xyzw",
        )
        target_displacement = tuple(
            target_pose["position_m"][axis] - baseline_target_center[axis]
            for axis in range(3)
        )
        experiment = scene.setdefault("experiment", {})
        if not isinstance(experiment, dict):
            raise MatrixError("scene experiment must be an object")
        experiment["matrix_experiment_id"] = matrix["experiment_id"]
        experiment["matrix_case_id"] = case_id
        experiment["matrix_transform"] = transform
        experiment["matrix_target_displacement_world_m"] = list(
            target_displacement
        )
        prior_translation = _numbers(
            experiment.get("target_cube_translation_world_m"),
            field="scene.experiment.target_cube_translation_world_m",
        )
        total_translation = tuple(
            prior_translation[axis] + target_displacement[axis]
            for axis in range(3)
        )
        experiment["target_cube_translation_world_m"] = list(total_translation)
        experiment["translation_norm_m"] = math.hypot(
            total_translation[0], total_translation[1]
        )
        if transform["type"] == "shoulder_pan_symmetry":
            experiment["target_cube_yaw_rad"] = float(
                experiment["target_cube_yaw_rad"]
            ) + float(transform["world_yaw_rad"])
            experiment["target_cube_yaw_deg"] = math.degrees(
                experiment["target_cube_yaw_rad"]
            )
        experiment["claim_boundary"] = "Generated zero-execution matrix case; not grasp evidence."
        _write_object(scene_path, scene)

        row: dict[str, Any] = {
            "case_id": case_id,
            "hypothesis": item["hypothesis"],
            "admission_layer": item["admission_layer"],
            "transform": transform,
            "target_displacement_world_m": list(target_displacement),
            "expected_status": item["expected_status"],
            "rationale": item.get("rationale"),
            "scene_filename": scene_filename,
            "candidate_filename": candidate_filename,
            "world_filename": world_filename,
            "geometry_filename": baseline_paths["geometry"].name,
        }
        try:
            generated_contract = load_scene_contract(scene_path)
        except SceneContractError as error:
            observed_reason = str(error)
            row.update(
                {
                    "materialization_status": "SCENE_CONTRACT_REJECTED",
                    "observed_reason": observed_reason,
                    "expected_reason": item.get("expected_reason"),
                    "expected_outcome_matched": (
                        item["expected_status"] == "SCENE_CONTRACT_REJECTED"
                        and observed_reason == item.get("expected_reason")
                    ),
                    "runtime_required": False,
                    "scene_sha256": _sha256(scene_path),
                }
            )
            rows.append(row)
            continue

        if item["admission_layer"] == "scene_contract":
            raise MatrixError(f"{case_id} unexpectedly passed the scene contract")

        candidate = deepcopy(baseline_candidate)
        candidate["name"] = f"edgegrasp_{case_id}"
        candidate["scene_config_filename"] = scene_filename
        candidate["world_filename"] = world_filename
        candidate["target"]["center_m"] = _transform_point(
            candidate["target"].get("center_m"),
            transform,
            field="candidate.target.center_m",
        )
        candidate["stages"]["approach"]["position_m"] = _transform_point(
            candidate["stages"]["approach"].get("position_m"),
            transform,
            field="candidate.stages.approach.position_m",
        )
        for key in ("approach", "grasp"):
            candidate["orientations_xyzw"][key] = _transform_quaternion(
                candidate["orientations_xyzw"].get(key),
                transform,
                field=f"candidate.orientations_xyzw.{key}",
            )
        changed = candidate.setdefault("changed_variable", {})
        if not isinstance(changed, dict):
            raise MatrixError("candidate changed_variable must be an object")
        for key in ("derived_descend_position_m", "derived_lift_position_m"):
            changed[key] = _transform_point(
                changed.get(key), transform, field=f"candidate.{key}"
            )
        changed["matrix_experiment_id"] = matrix["experiment_id"]
        changed["matrix_case_id"] = case_id
        changed["matrix_transform"] = transform
        changed["matrix_target_displacement_world_m"] = list(
            target_displacement
        )
        prior_candidate_translation = _numbers(
            changed.get("target_cube_translation_world_m"),
            field="candidate.changed_variable.target_cube_translation_world_m",
        )
        total_candidate_translation = tuple(
            prior_candidate_translation[axis] + target_displacement[axis]
            for axis in range(3)
        )
        changed["target_cube_translation_world_m"] = list(
            total_candidate_translation
        )
        changed["target_cube_translation_norm_m"] = math.hypot(
            total_candidate_translation[0], total_candidate_translation[1]
        )
        if transform["type"] == "shoulder_pan_symmetry":
            changed["target_cube_yaw_deg"] = float(
                changed["target_cube_yaw_deg"]
            ) + math.degrees(float(transform["world_yaw_rad"]))
        claims = candidate.setdefault("claim_boundary", [])
        if not isinstance(claims, list):
            raise MatrixError("candidate claim_boundary must be an array")
        claims.append("This generated case is used only for zero-execution planning.")
        _write_object(candidate_path, candidate)
        world_path.write_text(
            render_gazebo_sdf(generated_contract),
            encoding="utf-8",
            newline="\n",
        )
        geometry = _validate_candidate_scene_geometry(
            candidate=candidate,
            scene_path=scene_path,
            geometry_path=baseline_paths["geometry"],
        )
        expected_geometry = {
            key: _transform_point(
                baseline_geometry[key], transform, field=f"baseline_geometry.{key}"
            )
            for key in baseline_geometry
        }
        for key, expected in expected_geometry.items():
            if any(
                abs(observed - wanted) > 1e-12
                for observed, wanted in zip(geometry[key], expected, strict=True)
            ):
                raise MatrixError(f"{case_id} did not transform {key} consistently")
        if world_path.read_text(encoding="utf-8") != render_gazebo_sdf(
            load_scene_contract(scene_path)
        ):
            raise MatrixError(f"{case_id} world generation drift")
        row.update(
            {
                "materialization_status": "MATERIALIZED_FOR_PLAN_ONLY",
                "expected_outcome_matched": None,
                "runtime_required": True,
                "geometry": geometry,
                "scene_sha256": _sha256(scene_path),
                "candidate_sha256": _sha256(candidate_path),
                "world_sha256": _sha256(world_path),
                "scene_path": str(scene_path),
                "candidate_path": str(candidate_path),
                "world_path": str(world_path),
            }
        )
        rows.append(row)

    unexpected_contract_results = [
        row
        for row in rows
        if row["materialization_status"] == "SCENE_CONTRACT_REJECTED"
        and not row["expected_outcome_matched"]
    ]
    summary: dict[str, Any] = {
        "schema_version": 1,
        "experiment_id": matrix["experiment_id"],
        "mode": matrix["mode"],
        "recorded_at": datetime.now().astimezone().isoformat(),
        "matrix_path": str(matrix_path),
        "matrix_sha256": _sha256(matrix_path),
        "baseline_sha256": {
            key: _sha256(path) for key, path in baseline_paths.items()
        },
        "case_count": len(rows),
        "runtime_case_count": sum(bool(row["runtime_required"]) for row in rows),
        "scene_contract_rejection_count": sum(
            row["materialization_status"] == "SCENE_CONTRACT_REJECTED"
            for row in rows
        ),
        "unexpected_contract_result_count": len(unexpected_contract_results),
        "cases": rows,
        "zero_execution_contract": True,
        "status": (
            "MATERIALIZATION_PASS"
            if not unexpected_contract_results
            else "MATERIALIZATION_FAILED"
        ),
        "claim_boundary": matrix.get("claim_boundary"),
    }
    _write_object(artifact_dir / "matrix_materialization.json", summary)
    return summary


def _ensure_runtime_artifact_path(path: Path) -> None:
    try:
        path.resolve().relative_to(RUNTIME_ARTIFACT_ROOT.resolve())
    except ValueError as error:
        raise MatrixError(
            f"runtime artifact directory must stay under {RUNTIME_ARTIFACT_ROOT}"
        ) from error


def _copy_runtime_file(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists() or destination.is_symlink():
        raise MatrixError(f"refusing to overwrite installed path: {destination}")
    shutil.copy2(source, destination)
    if _sha256(source) != _sha256(destination):
        raise MatrixError(f"installed copy hash mismatch: {destination}")


def _safe_unlink(path: Path, parent: Path) -> None:
    if path.parent.resolve() != parent.resolve():
        raise MatrixError(f"cleanup path escaped install directory: {path}")
    if path.exists() or path.is_symlink():
        path.unlink()


def _motion_contract(report: dict[str, Any]) -> tuple[bool, list[str]]:
    violations: list[str] = []
    for field in (
        "trajectory_publication_count",
        "execute_trajectory_goal_count",
        "fjt_goal_count",
    ):
        if report.get(field) != 0:
            violations.append(f"{field}={report.get(field)!r}")
    if report.get("execution_attempted") is not False:
        violations.append(f"execution_attempted={report.get('execution_attempted')!r}")
    if report.get("motion_boundary_absent_throughout") is not True:
        violations.append("motion_boundary_absent_throughout is not true")
    segments = report.get("segments")
    if not isinstance(segments, list):
        violations.append("segments are missing")
    else:
        for index, row in enumerate(segments):
            if not isinstance(row, dict):
                violations.append(f"segments[{index}] is malformed")
                continue
            for field in (
                "trajectory_published",
                "execute_trajectory_goal_sent",
                "fjt_goal_sent",
                "execution_attempted",
            ):
                if row.get(field) is not False:
                    violations.append(f"segments[{index}].{field}={row.get(field)!r}")
    return not violations, violations


def _percentile(values: list[float], quantile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, math.ceil(quantile * len(ordered)) - 1)
    return ordered[index]


def _aggregate(
    materialization: dict[str, Any], runtime_rows: list[dict[str, Any]]
) -> dict[str, Any]:
    by_id = {str(row["case_id"]): row for row in runtime_rows}
    cases: list[dict[str, Any]] = []
    planning_times: list[float] = []
    wall_times: list[float] = []
    reasons: Counter[str] = Counter()
    false_accepts = 0
    false_rejects = 0
    unverified = 0
    motion_violations = 0
    matched = 0
    for source in materialization["cases"]:
        if not source["runtime_required"]:
            row = dict(source)
            observed = row["materialization_status"]
            row["observed_status"] = observed
            if row.get("expected_outcome_matched"):
                matched += 1
            else:
                unverified += 1
            reasons[f"scene_contract:{row.get('observed_reason', 'unknown')}"] += 1
            cases.append(row)
            continue
        runtime = by_id.get(str(source["case_id"]))
        row = dict(source)
        if runtime is None:
            row["observed_status"] = "PLAN_ONLY_UNVERIFIED"
            row["expected_outcome_matched"] = False
            unverified += 1
            reasons["infrastructure:missing_runtime_result"] += 1
            cases.append(row)
            continue
        row.update(runtime)
        observed = runtime.get("observed_status")
        expected = source["expected_status"]
        outcome_matched = observed == expected
        row["expected_outcome_matched"] = outcome_matched
        if outcome_matched:
            matched += 1
        elif observed == "PLAN_ONLY_UNVERIFIED":
            unverified += 1
        elif source["hypothesis"] == "rejection_hypothesis" and observed == "PLAN_ONLY_PASS":
            false_accepts += 1
        elif source["hypothesis"] == "reachable_hypothesis" and observed == "PLAN_ONLY_REJECTED":
            false_rejects += 1
        if not runtime.get("zero_motion_contract_pass", False):
            motion_violations += 1
        report = runtime.get("report")
        if isinstance(report, dict):
            for segment in report.get("segments", []):
                if not isinstance(segment, dict):
                    continue
                planning_time = segment.get("planning_time_s")
                wall_time = segment.get("elapsed_wall_s")
                if isinstance(planning_time, (int, float)) and math.isfinite(float(planning_time)):
                    planning_times.append(float(planning_time))
                if isinstance(wall_time, (int, float)) and math.isfinite(float(wall_time)):
                    wall_times.append(float(wall_time))
                if not segment.get("plan_accepted", False):
                    reasons[
                        "moveit_code:"
                        f"{segment.get('moveit_error_code', 'unknown')}|"
                        f"validator:{segment.get('validator_reason', 'unknown')}"
                    ] += 1
        cases.append(row)

    if motion_violations:
        status = "PLAN_ONLY_MATRIX_SAFETY_CONTRACT_FAILED"
    elif unverified:
        status = "PLAN_ONLY_MATRIX_UNVERIFIED"
    elif false_accepts or false_rejects:
        status = "PLAN_ONLY_MATRIX_HYPOTHESIS_MISMATCH"
    else:
        status = "PLAN_ONLY_MATRIX_PASS"
    return {
        "schema_version": 1,
        "experiment_id": materialization["experiment_id"],
        "recorded_at": datetime.now().astimezone().isoformat(),
        "status": status,
        "case_count": len(cases),
        "matched_expected_outcome_count": matched,
        "false_accept_count": false_accepts,
        "false_reject_count": false_rejects,
        "unverified_count": unverified,
        "motion_side_effect_violation_count": motion_violations,
        "planning_time_s": {
            "sample_count": len(planning_times),
            "p50_nearest_rank": _percentile(planning_times, 0.50),
            "p95_nearest_rank": _percentile(planning_times, 0.95),
        },
        "plan_response_wall_s": {
            "sample_count": len(wall_times),
            "p50_nearest_rank": _percentile(wall_times, 0.50),
            "p95_nearest_rank": _percentile(wall_times, 0.95),
        },
        "rejection_reason_histogram": dict(sorted(reasons.items())),
        "cases": cases,
        "claim_boundary": (
            "This is zero-execution scene-contract and MoveIt planning evidence. "
            "It is not trajectory execution, contact, lift, retention, physics-grasp, "
            "or hardware evidence."
        ),
    }


def run_runtime(
    *,
    materialization: dict[str, Any],
    artifact_dir: Path,
    domain_start: int,
    install_share: Path,
) -> dict[str, Any]:
    _ensure_runtime_artifact_path(artifact_dir)
    runtime_sources = [
        row for row in materialization["cases"] if row["runtime_required"]
    ]
    if domain_start < 0 or domain_start + len(runtime_sources) - 1 > 232:
        raise MatrixError("ROS domain range must stay within [0, 232]")
    config_install = install_share / "config"
    world_install = install_share / "worlds"
    if not config_install.is_dir() or not world_install.is_dir():
        raise MatrixError(f"EdgeGrasp install share is missing: {install_share}")
    baseline_geometry = materialization["baseline_sha256"]["geometry"]
    geometry_filename = materialization["cases"][0]["geometry_filename"]
    installed_geometry = config_install / geometry_filename
    if not installed_geometry.is_file() or _sha256(installed_geometry) != baseline_geometry:
        raise MatrixError("installed Candidate024 geometry is missing or has drifted")

    runtime_rows: list[dict[str, Any]] = []
    for index, row in enumerate(runtime_sources):
        case_id = str(row["case_id"])
        case_artifact = artifact_dir / "runtime" / case_id
        generated = {
            config_install / str(row["scene_filename"]): Path(str(row["scene_path"])),
            config_install / str(row["candidate_filename"]): Path(str(row["candidate_path"])),
            world_install / str(row["world_filename"]): Path(str(row["world_path"])),
        }
        copied: list[Path] = []
        observed: dict[str, Any] = {
            "case_id": case_id,
            "ros_domain_id": domain_start + index,
            "observed_status": "PLAN_ONLY_UNVERIFIED",
            "harness_exit_code": None,
            "zero_motion_contract_pass": False,
            "motion_contract_violations": [],
        }
        try:
            for destination, source in generated.items():
                _copy_runtime_file(source, destination)
                copied.append(destination)
            command = [
                "bash",
                "scripts/run_grasp_candidate_plan_only.sh",
                str(domain_start + index),
                str(case_artifact),
                case_id,
                str(row["geometry_filename"]),
                "1",
                "0.0",
                "implicit_default",
                "0.0",
                str(row["candidate_filename"]),
                str(row["scene_filename"]),
                str(row["world_filename"]),
            ]
            print(f"MATRIX_CASE_START {case_id} domain={domain_start + index}", flush=True)
            completed = subprocess.run(command, cwd=PROJECT_ROOT, check=False)
            observed["harness_exit_code"] = completed.returncode
            report_path = case_artifact / "plan_only_result.json"
            if report_path.is_file():
                report = _read_object(report_path)
                observed["report"] = report
                observed["observed_status"] = report.get("status", "PLAN_ONLY_UNVERIFIED")
                motion_ok, violations = _motion_contract(report)
                observed["zero_motion_contract_pass"] = motion_ok
                observed["motion_contract_violations"] = violations
            else:
                observed["motion_contract_violations"] = ["plan_only_result.json missing"]
            expected_exit = {"PLAN_ONLY_PASS": 0, "PLAN_ONLY_REJECTED": 2}.get(
                observed["observed_status"]
            )
            if expected_exit is None or completed.returncode != expected_exit:
                observed["observed_status"] = "PLAN_ONLY_UNVERIFIED"
            print(
                f"MATRIX_CASE_END {case_id} status={observed['observed_status']} "
                f"zero_motion={observed['zero_motion_contract_pass']}",
                flush=True,
            )
        except Exception as error:  # fail closed and preserve the partial evidence
            observed["error"] = f"{type(error).__name__}:{error}"
        finally:
            cleanup_errors: list[str] = []
            for path in reversed(copied):
                try:
                    parent = config_install if path.parent == config_install else world_install
                    _safe_unlink(path, parent)
                except Exception as error:  # cleanup failure invalidates this case
                    cleanup_errors.append(f"{path}:{type(error).__name__}:{error}")
            observed["installed_file_cleanup_errors"] = cleanup_errors
            if cleanup_errors:
                observed["observed_status"] = "PLAN_ONLY_UNVERIFIED"
                observed["zero_motion_contract_pass"] = False
            runtime_rows.append(observed)
            partial = _aggregate(materialization, runtime_rows)
            _write_object(artifact_dir / "matrix_result.partial.json", partial)

    result = _aggregate(materialization, runtime_rows)
    _write_object(artifact_dir / "matrix_result.json", result)
    return result


def _arguments(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--matrix", type=Path, default=DEFAULT_MATRIX)
    parser.add_argument("--artifact-dir", type=Path, required=True)
    parser.add_argument("--materialize-only", action="store_true")
    parser.add_argument("--ros-domain-id-start", type=int)
    parser.add_argument("--install-share", type=Path, default=DEFAULT_INSTALL_SHARE)
    args = parser.parse_args(argv)
    if not args.materialize_only and args.ros_domain_id_start is None:
        parser.error("--ros-domain-id-start is required for a runtime matrix")
    if args.artifact_dir.exists():
        parser.error("--artifact-dir must be a fresh path")
    return args


def main(argv: list[str] | None = None) -> int:
    args = _arguments(sys.argv[1:] if argv is None else argv)
    args.artifact_dir.mkdir(parents=True)
    try:
        materialization = materialize(args.matrix.resolve(), args.artifact_dir.resolve())
        if materialization["status"] != "MATERIALIZATION_PASS":
            return 4
        if args.materialize_only:
            print(json.dumps(materialization, indent=2, sort_keys=True))
            return 0
        result = run_runtime(
            materialization=materialization,
            artifact_dir=args.artifact_dir.resolve(),
            domain_start=int(args.ros_domain_id_start),
            install_share=args.install_share.resolve(),
        )
        print(json.dumps(result, indent=2, sort_keys=True))
        return {
            "PLAN_ONLY_MATRIX_PASS": 0,
            "PLAN_ONLY_MATRIX_HYPOTHESIS_MISMATCH": 2,
            "PLAN_ONLY_MATRIX_UNVERIFIED": 3,
            "PLAN_ONLY_MATRIX_SAFETY_CONTRACT_FAILED": 4,
        }[str(result["status"])]
    except Exception as error:
        failure = {
            "schema_version": 1,
            "status": "PLAN_ONLY_MATRIX_UNVERIFIED",
            "error": f"{type(error).__name__}:{error}",
            "recorded_at": datetime.now().astimezone().isoformat(),
            "execution_attempted": False,
        }
        _write_object(args.artifact_dir / "matrix_failure.json", failure)
        print(json.dumps(failure, indent=2, sort_keys=True), file=sys.stderr)
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
