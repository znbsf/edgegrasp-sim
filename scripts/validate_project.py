"""Dependency-free structural validation for CI and pre-ROS machines."""

from __future__ import annotations

import ast
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def validate_manifest() -> None:
    path = ROOT / "docs" / "upstream-manifest.json"
    data = json.loads(path.read_text(encoding="utf-8"))
    if len(data.get("upstreams", [])) != 3:
        raise ValueError("upstream manifest must contain exactly three audited repositories")
    for upstream in data["upstreams"]:
        commit = upstream["commit"]
        if re.fullmatch(r"[0-9a-f]{40}", commit) is None:
            raise ValueError(f"invalid commit for {upstream['repository']}")
        for label, url in upstream["evidence_urls"].items():
            if commit not in url:
                raise ValueError(f"floating evidence URL: {upstream['repository']}:{label}")
        for step in upstream.get("reproduction_steps", []):
            if not step.get("cwd") or not step.get("command"):
                raise ValueError(f"incomplete reproduction step: {upstream['repository']}")

    adoodevv = next(
        item for item in data["upstreams"] if item["repository"] == "adoodevv/so101_ros2"
    )
    adoodevv_status = adoodevv["local_verification_status"]
    if not adoodevv_status.get("local_checkout_static_verified"):
        raise ValueError("adoodevv local static verification status is missing")
    if adoodevv_status.get("runtime_full_stack_verified") is not False:
        raise ValueError("adoodevv full-runtime boundary must remain explicit")
    if not adoodevv_status.get("moveit_plan_runtime_verified"):
        raise ValueError("observed MoveIt plan runtime evidence is missing")
    if adoodevv_status.get("collision_and_grasp_runtime_verified") is not False:
        raise ValueError("collision/grasp boundary must remain unverified")


def validate_so101_contract() -> None:
    path = ROOT / "src" / "edgegrasp" / "contracts" / "so101_ros2_0305e03.json"
    contract = json.loads(path.read_text(encoding="utf-8"))
    if contract.get("commit") != "0305e03ab54e64aae9263fcbf339622e654012f3":
        raise ValueError("SO-101 adapter contract pin drift")
    controllers = contract["controllers"]
    if controllers["arm_controller"]["action"] != (
        "/arm_controller/follow_joint_trajectory"
    ):
        raise ValueError("arm action contract drift")
    if controllers["gripper_controller"]["action"] != (
        "/gripper_controller/follow_joint_trajectory"
    ):
        raise ValueError("gripper action contract drift")
    if contract["verification"]["runtime_verified"] is not True:
        raise ValueError("observed runtime evidence is missing from contract")
    if contract["verification"]["runtime_full_stack_verified"] is not False:
        raise ValueError("contract must not imply full-stack runtime verification")


def validate_ros_skeleton() -> None:
    workspace = ROOT / "ros_ws" / "src"
    package = workspace / "edgegrasp_ros"
    for path in sorted(workspace.rglob("*.py")):
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    ET.parse(package / "package.xml")
    ET.parse(workspace / "edgegrasp_core" / "package.xml")
    world = ET.parse(package / "worlds" / "table_cube.sdf").getroot()
    if world.find("./world/model[@name='target_cube']") is None:
        raise ValueError("table_cube.sdf has no target_cube model")


def main() -> int:
    validate_manifest()
    validate_so101_contract()
    validate_ros_skeleton()
    print("project structure: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
