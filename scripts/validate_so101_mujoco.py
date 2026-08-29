"""Validate the pinned upstream SO-101 MJCF, optionally with MuJoCo stepping.

The dependency-free static mode checks the original XML and checkout pin. The
runtime mode additionally loads and steps the unchanged scene through MuJoCo.
Neither mode is an EdgeGrasp backend, ROS/Gazebo test, planning test, or grasp.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import math
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET


EXPECTED_COMMIT = "7629d2ad9853d10fb903093a33ef6114099d97e5"
EXPECTED_JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
CONTROL_TARGET = (0.20, -0.35, 0.40, -0.25, 0.15, 0.35)


def default_checkout() -> Path:
    project_root = Path(__file__).resolve().parents[1]
    repository_root = project_root.parents[1]
    return repository_root / "workspaces" / "forks" / "SO-ARM100"


def git_output(checkout: Path, *arguments: str) -> str:
    completed = subprocess.run(
        ["git", "-C", str(checkout), *arguments],
        check=True,
        capture_output=True,
        text=True,
        timeout=10,
    )
    return completed.stdout.strip()


def static_validate(checkout: Path, expected_commit: str) -> dict[str, Any]:
    checkout = checkout.resolve(strict=True)
    commit = git_output(checkout, "rev-parse", "HEAD")
    if commit != expected_commit:
        raise AssertionError(f"checkout commit {commit} does not match {expected_commit}")
    if git_output(checkout, "status", "--porcelain"):
        raise AssertionError("upstream checkout is not clean")

    asset_root = checkout / "Simulation" / "SO101"
    scene = asset_root / "scene.xml"
    scene_root = ET.parse(scene).getroot()
    include = scene_root.find("include")
    if include is None or include.get("file") != "so101_new_calib.xml":
        raise AssertionError("scene.xml does not include the pinned default MJCF")
    model_path = asset_root / include.get("file", "")
    model_root = ET.parse(model_path).getroot()

    joint_names = tuple(
        joint.get("name", "")
        for joint in model_root.findall(".//joint[@name]")
    )
    actuator_names = tuple(
        actuator.get("name", "")
        for actuator in model_root.findall("./actuator/position")
    )
    if joint_names != EXPECTED_JOINTS:
        raise AssertionError(f"unexpected MJCF joint order: {joint_names}")
    if actuator_names != EXPECTED_JOINTS:
        raise AssertionError(f"unexpected actuator order: {actuator_names}")

    meshes = [mesh.get("file", "") for mesh in model_root.findall("./asset/mesh")]
    if len(meshes) != 13:
        raise AssertionError(f"expected 13 mesh declarations, found {len(meshes)}")
    missing_meshes = [name for name in meshes if not (asset_root / "assets" / name).is_file()]
    if missing_meshes:
        raise AssertionError(f"missing mesh assets: {missing_meshes}")

    return {
        "schema_version": 1,
        "validated_at_utc": datetime.now(timezone.utc).isoformat(),
        "repository": "https://github.com/TheRobotStudio/SO-ARM100",
        "checkout": str(checkout),
        "commit": commit,
        "scene": str(scene),
        "model": str(model_path),
        "joint_names": list(joint_names),
        "actuator_names": list(actuator_names),
        "mesh_count": len(meshes),
        "static_original_asset_validation": True,
        "known_asset_limitations": [
            "SO101 base collision meshes are absent in this pinned upstream asset.",
            "LeRobot gripper 0-closed/100-open mapping is not represented by the MJCF joint range.",
            "Static XML checks do not validate dynamics, collision-safe planning, grasping, ROS, Gazebo, or MoveIt.",
        ],
    }


def object_names(mujoco: Any, model: Any, object_type: Any, count: int) -> list[str]:
    names: list[str] = []
    for index in range(count):
        name = mujoco.mj_id2name(model, object_type, index)
        if name is None:
            raise AssertionError(f"unnamed MuJoCo object at index {index}")
        names.append(name)
    return names


def state_digest(np: Any, data: Any) -> str:
    hasher = hashlib.sha256()
    for array in (data.qpos, data.qvel, data.ctrl, data.act, data.qacc_warmstart):
        stable = np.asarray(array, dtype="<f8")
        hasher.update(stable.shape[0].to_bytes(8, byteorder="little", signed=False))
        hasher.update(stable.tobytes(order="C"))
    hasher.update(np.asarray((data.time,), dtype="<f8").tobytes())
    return hasher.hexdigest()


def run_once(mujoco: Any, np: Any, model: Any, steps: int) -> dict[str, Any]:
    data = mujoco.MjData(model)
    data.ctrl[:] = np.asarray(CONTROL_TARGET, dtype=np.float64)
    mujoco.mj_forward(model, data)
    for _ in range(steps):
        mujoco.mj_step(model, data)
    qpos = np.asarray(data.qpos, dtype=np.float64).copy()
    qvel = np.asarray(data.qvel, dtype=np.float64).copy()
    if not all(math.isfinite(float(value)) for value in (*qpos, *qvel)):
        raise AssertionError("non-finite MuJoCo state after stepping")
    return {
        "digest": state_digest(np, data),
        "final_time_s": float(data.time),
        "final_qpos_rad": [float(value) for value in qpos],
        "final_qvel_rad_s": [float(value) for value in qvel],
        "contacts_at_final_step": int(data.ncon),
    }


def runtime_validate(report: dict[str, Any], steps: int, runs: int) -> dict[str, Any]:
    try:
        mujoco = importlib.import_module("mujoco")
        np = importlib.import_module("numpy")
    except (ImportError, OSError) as error:
        raise RuntimeError(f"MuJoCo runtime import unavailable: {error}") from error

    model = mujoco.MjModel.from_xml_path(report["scene"])
    joint_names = object_names(mujoco, model, mujoco.mjtObj.mjOBJ_JOINT, model.njnt)
    actuator_names = object_names(
        mujoco, model, mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu
    )
    if tuple(joint_names) != EXPECTED_JOINTS or tuple(actuator_names) != EXPECTED_JOINTS:
        raise AssertionError("MuJoCo compiled model contract differs from source XML")
    if (model.nq, model.nv, model.nu) != (6, 6, 6):
        raise AssertionError(
            f"unexpected dimensions nq={model.nq}, nv={model.nv}, nu={model.nu}"
        )
    control = np.asarray(CONTROL_TARGET, dtype=np.float64)
    ranges = np.asarray(model.actuator_ctrlrange, dtype=np.float64)
    if np.any(control < ranges[:, 0]) or np.any(control > ranges[:, 1]):
        raise AssertionError("smoke-test control is outside declared actuator ranges")

    results = [run_once(mujoco, np, model, steps) for _ in range(runs)]
    digests = {result["digest"] for result in results}
    if len(digests) != 1:
        raise AssertionError(f"same-process repeated runs diverged: {sorted(digests)}")

    report["scope"] = "pinned_upstream_mjcf_headless_runtime_smoke_not_ros_or_gazebo"
    report["runtime"] = {
        "verified": True,
        "mujoco_version": mujoco.__version__,
        "numpy_version": np.__version__,
        "nq": int(model.nq),
        "nv": int(model.nv),
        "nu": int(model.nu),
        "nbody": int(model.nbody),
        "ngeom": int(model.ngeom),
        "nmesh": int(model.nmesh),
        "timestep_s": float(model.opt.timestep),
        "steps_per_run": steps,
        "runs": runs,
        "deterministic_same_process": True,
        "state_digest_sha256": results[0]["digest"],
        "first_run": results[0],
    }
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkout", type=Path, default=default_checkout())
    parser.add_argument("--expected-commit", default=EXPECTED_COMMIT)
    parser.add_argument("--steps", type=int, default=1_000)
    parser.add_argument("--runs", type=int, default=2)
    parser.add_argument("--static-only", action="store_true")
    arguments = parser.parse_args()
    if arguments.steps <= 0:
        parser.error("--steps must be positive")
    if arguments.runs < 2:
        parser.error("--runs must be at least 2")
    return arguments


def main() -> int:
    arguments = parse_args()
    try:
        report = static_validate(arguments.checkout, arguments.expected_commit)
        if arguments.static_only:
            report["scope"] = "pinned_upstream_mjcf_static_original_asset_check"
            report["runtime"] = {"verified": False, "reason": "--static-only"}
        else:
            report = runtime_validate(report, arguments.steps, arguments.runs)
    except (
        AssertionError,
        FileNotFoundError,
        RuntimeError,
        subprocess.SubprocessError,
        ValueError,
        ET.ParseError,
    ) as error:
        print(f"SO-101 MuJoCo asset validation FAILED: {error}", file=sys.stderr)
        return 1
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
