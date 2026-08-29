from __future__ import annotations

import json
import math
import os
from pathlib import Path
import subprocess
import sys

import pytest

from edgegrasp.grasp_geometry import (
    derive_grasp_stage_geometry,
    load_grasp_geometry_profile,
    validate_routed_grasp_stage_geometry,
)
from edgegrasp.scene import load_scene_contract


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "ros_ws" / "src" / "edgegrasp_ros" / "config"
SOURCE = CONFIG / "so101_grasp_geometry_candidate008_q0p50.json"
PROFILE = CONFIG / "so101_grasp_geometry_candidate010_q0p45.json"
PROXY = CONFIG / "so101_collision_proxies.json"
CANDIDATE = CONFIG / "so101_side_grasp_candidate.json"
SCENE = CONFIG / "scene.json"


def _assert_generated_equal(actual: object, expected: object) -> None:
    if isinstance(actual, dict) and isinstance(expected, dict):
        assert actual.keys() == expected.keys()
        for key in actual:
            _assert_generated_equal(actual[key], expected[key])
        return
    if isinstance(actual, list) and isinstance(expected, list):
        assert len(actual) == len(expected)
        for left, right in zip(actual, expected, strict=True):
            _assert_generated_equal(left, right)
        return
    if (
        isinstance(actual, (int, float))
        and not isinstance(actual, bool)
        and isinstance(expected, (int, float))
        and not isinstance(expected, bool)
    ):
        assert float(actual) == pytest.approx(float(expected), rel=0.0, abs=1e-14)
        return
    assert actual == expected


def test_candidate010_profile_is_generator_reproducible() -> None:
    environment = os.environ.copy()
    source_path = str(ROOT / "src")
    environment["PYTHONPATH"] = os.pathsep.join(
        value
        for value in (source_path, environment.get("PYTHONPATH", ""))
        if value
    )
    result = subprocess.run(
        [
            sys.executable,
            "scripts/generate_grasp_geometry_candidate.py",
            "--source-profile",
            SOURCE.relative_to(ROOT).as_posix(),
            "--proxy-contract",
            PROXY.relative_to(ROOT).as_posix(),
            "--gripper-position-rad",
            "0.45",
            "--name",
            "edgegrasp_so101_target_cube_grasp_geometry_candidate010_q0p45",
        ],
        cwd=ROOT,
        env=environment,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    _assert_generated_equal(
        json.loads(result.stdout),
        json.loads(PROFILE.read_text(encoding="utf-8")),
    )


def test_candidate010_changes_only_close_geometry_and_passes_static_gate() -> None:
    source = load_grasp_geometry_profile(SOURCE)
    profile = load_grasp_geometry_profile(PROFILE)
    candidate = json.loads(CANDIDATE.read_text(encoding="utf-8"))
    scene = load_scene_contract(SCENE)
    cube = next(item for item in scene.objects if item.object_id == "target_cube")
    approach_orientation = tuple(candidate["orientations_xyzw"]["approach"])
    grasp_orientation = tuple(candidate["orientations_xyzw"]["grasp"])
    derived = derive_grasp_stage_geometry(
        cube.pose_world.position_m,
        grasp_orientation,
        profile,
    )
    validate_routed_grasp_stage_geometry(
        cube_center_m=cube.pose_world.position_m,
        approach_position_m=tuple(candidate["stages"]["approach"]["position_m"]),
        descend_position_m=derived.descend_position_m,
        lift_position_m=derived.lift_position_m,
        approach_orientation_xyzw=approach_orientation,
        grasp_orientation_xyzw=grasp_orientation,
        gripper_position_rad=0.45,
        profile=profile,
    )

    assert profile.target_center_in_frame_at_descend_m == (
        source.target_center_in_frame_at_descend_m
    )
    assert profile.gripper_contact_position_rad == 0.45
    assert source.gripper_contact_position_rad == 0.5
    assert profile.gripper_open_position_rad == source.gripper_open_position_rad
    assert profile.approach_offset_end_effector_m == (
        source.approach_offset_end_effector_m
    )
    assert derived.descend_position_m == pytest.approx(
        candidate["stages"]["descend"]["position_m"], abs=1e-12
    )
    source_gap = (
        source.required_contact_pad_aabbs[0].minimum_m[0]
        - source.required_contact_pad_aabbs[1].maximum_m[0]
    )
    profile_gap = (
        profile.required_contact_pad_aabbs[0].minimum_m[0]
        - profile.required_contact_pad_aabbs[1].maximum_m[0]
    )
    assert math.isfinite(source_gap) and math.isfinite(profile_gap)
    assert profile_gap < source_gap
    assert profile.gripper_contact_position_rad < source.gripper_contact_position_rad
    claims = json.loads(PROFILE.read_text(encoding="utf-8"))["claim_boundary"]
    assert any("The 0.45 rad contact position" in line for line in claims)
    assert not any("The 0.50 rad contact position" in line for line in claims)
