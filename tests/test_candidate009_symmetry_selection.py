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


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "ros_ws" / "src" / "edgegrasp_ros" / "config"
SOURCE_PROFILE = CONFIG / "so101_grasp_geometry_candidate008_q0p50.json"
SELECTED_PROFILE = (
    CONFIG / "so101_grasp_geometry_candidate009_centered_q0p50.json"
)
PROXY_CONTRACT = CONFIG / "so101_collision_proxies.json"
CANDIDATE = CONFIG / "so101_side_grasp_candidate.json"
OBSERVATION = (
    ROOT
    / "docs"
    / "observations"
    / "2026-08-27-candidate009-static-symmetry-selection.json"
)
GRASP_ORIENTATION = (
    0.3450298741758752,
    0.6172118344266647,
    0.6332767577280262,
    0.3145862131297726,
)


def _run_json(*arguments: str) -> dict[str, object]:
    environment = os.environ.copy()
    source_path = str(ROOT / "src")
    environment["PYTHONPATH"] = os.pathsep.join(
        value
        for value in (source_path, environment.get("PYTHONPATH", ""))
        if value
    )
    result = subprocess.run(
        [sys.executable, *arguments],
        cwd=ROOT,
        env=environment,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return json.loads(result.stdout)


def _assert_cross_interpreter_equal(actual: object, expected: object) -> None:
    """Compare generated JSON while tolerating only insignificant libm tails."""

    if isinstance(actual, dict) and isinstance(expected, dict):
        assert actual.keys() == expected.keys()
        for key in actual:
            _assert_cross_interpreter_equal(actual[key], expected[key])
        return
    if isinstance(actual, list) and isinstance(expected, list):
        assert len(actual) == len(expected)
        for actual_item, expected_item in zip(actual, expected, strict=True):
            _assert_cross_interpreter_equal(actual_item, expected_item)
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


def test_candidate009_static_selection_is_reproducible() -> None:
    selected = _run_json(
        "scripts/select_grasp_symmetry_offset.py",
        "--profile",
        SOURCE_PROFILE.relative_to(ROOT).as_posix(),
        "--grasp-orientation-xyzw",
        *(str(value) for value in GRASP_ORIENTATION),
    )
    observed = json.loads(OBSERVATION.read_text(encoding="utf-8"))["selection"]

    assert selected["selection_method"] == observed["selection_method"]
    assert selected["profile_sha256"] == observed["source_profile_sha256"]
    assert selected["scan"] == observed["scan"]
    assert selected["baseline"] == observed["baseline"]
    _assert_cross_interpreter_equal(selected["selected"], observed["selected"])
    assert selected["selected"]["offset_x_m"] == -0.0009
    assert selected["selected"]["absolute_overlap_difference_m"] < (
        selected["baseline"]["absolute_overlap_difference_m"] / 100.0
    )


def test_candidate009_profile_is_reproducible_from_selected_offset() -> None:
    generated = _run_json(
        "scripts/generate_grasp_geometry_candidate.py",
        "--source-profile",
        SOURCE_PROFILE.relative_to(ROOT).as_posix(),
        "--proxy-contract",
        PROXY_CONTRACT.relative_to(ROOT).as_posix(),
        "--gripper-position-rad",
        "0.5",
        "--target-center-x-offset-m",
        "-0.0009",
        "--name",
        "edgegrasp_so101_target_cube_grasp_geometry_candidate009_centered_q0p50",
    )
    checked_in = json.loads(SELECTED_PROFILE.read_text(encoding="utf-8"))
    _assert_cross_interpreter_equal(generated, checked_in)
    assert checked_in["geometry_source"]["target_center_x_offset_m"] == -0.0009


def test_generator_updates_contact_angle_claim_from_any_source_profile() -> None:
    generated = _run_json(
        "scripts/generate_grasp_geometry_candidate.py",
        "--source-profile",
        SOURCE_PROFILE.relative_to(ROOT).as_posix(),
        "--proxy-contract",
        PROXY_CONTRACT.relative_to(ROOT).as_posix(),
        "--gripper-position-rad",
        "0.45",
        "--name",
        "candidate010_test",
    )
    claims = generated["claim_boundary"]
    assert any("The 0.45 rad contact position" in line for line in claims)
    assert not any("The 0.50 rad contact position" in line for line in claims)


def test_candidate009_derived_stages_pass_exact_routed_geometry_gate() -> None:
    profile = load_grasp_geometry_profile(SELECTED_PROFILE)
    candidate = json.loads(CANDIDATE.read_text(encoding="utf-8"))
    cube_center = tuple(candidate["target"]["center_m"])
    derived = derive_grasp_stage_geometry(
        cube_center,
        GRASP_ORIENTATION,
        profile,
    )
    validated = validate_routed_grasp_stage_geometry(
        cube_center_m=cube_center,
        approach_position_m=tuple(candidate["stages"]["approach"]["position_m"]),
        descend_position_m=derived.descend_position_m,
        lift_position_m=derived.lift_position_m,
        approach_orientation_xyzw=tuple(
            candidate["orientations_xyzw"]["approach"]
        ),
        grasp_orientation_xyzw=GRASP_ORIENTATION,
        gripper_position_rad=0.5,
        profile=profile,
    )
    previous_descend = tuple(candidate["stages"]["descend"]["position_m"])
    delta = tuple(
        selected - previous
        for selected, previous in zip(
            derived.descend_position_m,
            previous_descend,
            strict=True,
        )
    )

    assert validated.cube_center_in_frame_at_descend_m == pytest.approx(
        (-0.026099464387, -0.0105, -0.016),
        abs=1e-12,
    )
    assert math.sqrt(sum(value * value for value in delta)) == pytest.approx(
        0.0009,
        abs=1e-12,
    )
    assert candidate["candidate008_tighter_closure_runtime_observation"][
        "physics_grasp_verified"
    ] is False


def test_candidate009_runtime_path_uses_profile_derived_positions() -> None:
    client = (
        ROOT
        / "ros_ws"
        / "src"
        / "edgegrasp_grasp_sequence"
        / "edgegrasp_grasp_sequence"
        / "trial_client.py"
    ).read_text(encoding="utf-8")
    harness = (ROOT / "scripts" / "run_candidate005_contact_quality.sh").read_text(
        encoding="utf-8"
    )

    assert 'declare_parameter("derive_descend_and_lift_from_profile", False)' in client
    assert "derive_grasp_stage_geometry" in client
    assert "self._resolved_stage_positions(target)" in client
    assert "trial_derive_descend_lift=${6:-false}" in harness
    assert '-p derive_descend_and_lift_from_profile:="$trial_derive_descend_lift"' in harness
    assert json.loads(OBSERVATION.read_text(encoding="utf-8"))["candidate009"][
        "execution_attempted"
    ] is False
