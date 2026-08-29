from __future__ import annotations

import json
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
SOURCE = CONFIG / "so101_grasp_geometry_candidate010_q0p45.json"
PROFILE = CONFIG / "so101_grasp_geometry_candidate011_q0p40.json"
EXTENDED_PROFILE = (
    CONFIG / "so101_grasp_geometry_candidate014_moving_pad_plus12mm.json"
)
PROXY = CONFIG / "so101_collision_proxies.json"
CANDIDATE = CONFIG / "so101_side_grasp_candidate.json"
SCENE = CONFIG / "scene.json"


def _assert_equal(actual: object, expected: object) -> None:
    if isinstance(actual, dict) and isinstance(expected, dict):
        assert actual.keys() == expected.keys()
        for key in actual:
            _assert_equal(actual[key], expected[key])
        return
    if isinstance(actual, list) and isinstance(expected, list):
        assert len(actual) == len(expected)
        for left, right in zip(actual, expected, strict=True):
            _assert_equal(left, right)
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


def test_selected_grasp_profiles_are_generator_reproducible() -> None:
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        value
        for value in (str(ROOT / "src"), environment.get("PYTHONPATH", ""))
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
            "0.4",
            "--name",
            "edgegrasp_so101_target_cube_grasp_geometry_candidate011_q0p40",
        ],
        cwd=ROOT,
        env=environment,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    _assert_equal(
        json.loads(result.stdout),
        json.loads(PROFILE.read_text(encoding="utf-8")),
    )

    extended = subprocess.run(
        [
            sys.executable,
            "scripts/generate_grasp_geometry_candidate.py",
            "--source-profile",
            PROFILE.relative_to(ROOT).as_posix(),
            "--proxy-contract",
            PROXY.relative_to(ROOT).as_posix(),
            "--gripper-position-rad",
            "0.4",
            "--moving-pad-distal-extension-m",
            "0.012",
            "--name",
            "edgegrasp_so101_target_cube_grasp_geometry_candidate014_moving_pad_plus12mm",
        ],
        cwd=ROOT,
        env=environment,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert extended.returncode == 0, extended.stdout + extended.stderr
    _assert_equal(
        json.loads(extended.stdout),
        json.loads(EXTENDED_PROFILE.read_text(encoding="utf-8")),
    )


def test_candidate011_static_geometry_passes_without_arm_pose_change() -> None:
    source = load_grasp_geometry_profile(SOURCE)
    profile = load_grasp_geometry_profile(PROFILE)
    scene = load_scene_contract(SCENE)
    candidate = json.loads(CANDIDATE.read_text(encoding="utf-8"))
    cube = next(item for item in scene.objects if item.object_id == "target_cube")
    grasp_orientation = tuple(candidate["orientations_xyzw"]["grasp"])
    derived = derive_grasp_stage_geometry(
        cube.pose_world.position_m, grasp_orientation, profile
    )
    validate_routed_grasp_stage_geometry(
        cube_center_m=cube.pose_world.position_m,
        approach_position_m=tuple(candidate["stages"]["approach"]["position_m"]),
        descend_position_m=derived.descend_position_m,
        lift_position_m=derived.lift_position_m,
        approach_orientation_xyzw=tuple(candidate["orientations_xyzw"]["approach"]),
        grasp_orientation_xyzw=grasp_orientation,
        gripper_position_rad=0.4,
        profile=profile,
    )
    assert profile.target_center_in_frame_at_descend_m == (
        source.target_center_in_frame_at_descend_m
    )
    assert profile.gripper_contact_position_rad == 0.4
    assert profile.approach_offset_end_effector_m == (
        source.approach_offset_end_effector_m
    )
    assert derived.descend_position_m == pytest.approx(
        candidate["stages"]["descend"]["position_m"], abs=1e-12
    )
    source_payload = json.loads(SOURCE.read_text(encoding="utf-8"))
    profile_payload = json.loads(PROFILE.read_text(encoding="utf-8"))
    assert profile_payload["geometry_source"][
        "predicted_inner_gap_at_contact_position_m"
    ] < source_payload["geometry_source"][
        "predicted_inner_gap_at_contact_position_m"
    ]
    assert any(
        "The 0.40 rad contact position" in line
        for line in profile_payload["claim_boundary"]
    )
