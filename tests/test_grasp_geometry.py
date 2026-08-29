from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

import pytest

from edgegrasp.grasp_geometry import (
    GraspGeometryError,
    derive_grasp_stage_geometry,
    load_grasp_geometry_profile,
    transform_point_from_frame,
    validate_grasp_collision_envelope,
    validate_routed_grasp_stage_geometry,
    validate_grasp_stage_geometry,
)
from edgegrasp.scene import load_scene_contract


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONFIG_ROOT = PROJECT_ROOT / "ros_ws" / "src" / "edgegrasp_ros" / "config"
PROFILE_PATH = CONFIG_ROOT / "so101_grasp_geometry.json"
CANDIDATE008_PROFILE_PATH = (
    CONFIG_ROOT / "so101_grasp_geometry_candidate008_q0p50.json"
)
SIDE_GRASP_PATH = CONFIG_ROOT / "so101_side_grasp_candidate.json"
SCENE_PATH = CONFIG_ROOT / "scene.json"
TASK006_ORIENTATION = (
    0.000140847932006,
    -0.212962906352545,
    0.048190781189021,
    0.975871113051375,
)
SIDE_GRASP_ORIENTATION = (
    0.3450298741758752,
    0.6172118344266647,
    0.6332767577280262,
    0.3145862131297726,
)


def _geometry_precision(value):
    """Ignore sub-picometre libm drift while preserving profile structure."""

    if isinstance(value, float):
        return round(value, 12)
    if isinstance(value, list):
        return [_geometry_precision(item) for item in value]
    if isinstance(value, dict):
        return {key: _geometry_precision(item) for key, item in value.items()}
    return value


def test_candidate008_profile_is_reproducible_and_tighter_than_candidate005() -> None:
    generator = PROJECT_ROOT / "scripts" / "generate_grasp_geometry_candidate.py"
    proxy_contract = CONFIG_ROOT / "so101_collision_proxies.json"
    result = subprocess.run(
        [
            sys.executable,
            str(generator),
            "--source-profile",
            str(PROFILE_PATH),
            "--proxy-contract",
            str(proxy_contract),
            "--gripper-position-rad",
            "0.5",
            "--name",
            "edgegrasp_so101_target_cube_grasp_geometry_candidate008_q0p50",
        ],
        cwd=PROJECT_ROOT,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    generated = json.loads(result.stdout)
    checked_in = json.loads(CANDIDATE008_PROFILE_PATH.read_text(encoding="utf-8"))
    assert _geometry_precision(generated) == _geometry_precision(checked_in)

    previous = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
    assert checked_in["gripper_contact_position_rad"] == 0.5
    assert checked_in["geometry_source"]["generated_gripper_position_rad"] == 0.5
    assert checked_in["geometry_source"][
        "predicted_inner_gap_at_contact_position_m"
    ] < previous["geometry_source"]["predicted_inner_gap_at_contact_position_m"]


def test_candidate008_profile_passes_the_exact_routed_obb_gate() -> None:
    candidate = json.loads(SIDE_GRASP_PATH.read_text(encoding="utf-8"))
    profile = load_grasp_geometry_profile(CANDIDATE008_PROFILE_PATH)
    scene = load_scene_contract(SCENE_PATH)
    cube = next(item for item in scene.objects if item.object_id == "target_cube")

    validated = validate_routed_grasp_stage_geometry(
        cube_center_m=cube.pose_world.position_m,
        approach_position_m=tuple(candidate["stages"]["approach"]["position_m"]),
        descend_position_m=tuple(candidate["stages"]["descend"]["position_m"]),
        lift_position_m=tuple(candidate["stages"]["lift"]["position_m"]),
        approach_orientation_xyzw=tuple(candidate["orientations_xyzw"]["approach"]),
        grasp_orientation_xyzw=tuple(candidate["orientations_xyzw"]["grasp"]),
        gripper_position_rad=0.5,
        profile=profile,
    )
    assert validated.cube_center_in_frame_at_descend_m == pytest.approx(
        profile.target_center_in_frame_at_descend_m,
        abs=max(profile.target_center_tolerance_m),
    )


def test_profile_matches_scene_and_derives_candidate_stages() -> None:
    profile = load_grasp_geometry_profile(PROFILE_PATH)
    scene = load_scene_contract(SCENE_PATH)
    cube = next(item for item in scene.objects if item.object_id == "target_cube")
    assert profile.target_id == cube.object_id
    assert profile.cube_size_m == cube.size_m

    stages = derive_grasp_stage_geometry(
        cube.pose_world.position_m, SIDE_GRASP_ORIENTATION, profile
    )
    assert stages.descend_position_m == pytest.approx(
        (0.2462364349184608, 0.15125054509958977, 0.21671404504175756),
        abs=1e-12,
    )
    assert stages.approach_position_m == pytest.approx(
        (0.2709963789421751, 0.16819000116779706, 0.2967142812839486),
        abs=1e-12,
    )
    assert stages.lift_position_m == pytest.approx(
        (0.2462364349184608, 0.15125054509958977, 0.25671404504175754),
        abs=1e-12,
    )
    assert transform_point_from_frame(
        stages.descend_position_m,
        stages.orientation_xyzw,
        profile.target_center_in_frame_at_descend_m,
    ) == pytest.approx(cube.pose_world.position_m, abs=1e-12)
    validated = validate_grasp_stage_geometry(
        cube_center_m=cube.pose_world.position_m,
        approach_position_m=stages.approach_position_m,
        descend_position_m=stages.descend_position_m,
        lift_position_m=stages.lift_position_m,
        orientation_xyzw=SIDE_GRASP_ORIENTATION,
        gripper_position_rad=profile.gripper_contact_position_rad,
        profile=profile,
    )
    assert validated.cube_center_in_frame_at_descend_m == pytest.approx(
        profile.target_center_in_frame_at_descend_m, abs=1e-12
    )


def test_task006_protocol_pose_is_rejected_as_outside_finger_envelope() -> None:
    profile = load_grasp_geometry_profile(PROFILE_PATH)
    scene = load_scene_contract(SCENE_PATH)
    cube = next(item for item in scene.objects if item.object_id == "target_cube")
    with pytest.raises(
        GraspGeometryError, match="cube_center_outside_finger_contact_envelope"
    ):
        validate_grasp_stage_geometry(
            cube_center_m=cube.pose_world.position_m,
            approach_position_m=(
                0.218593782915783,
                0.008984635830313,
                0.403477545364772,
            ),
            descend_position_m=(
                0.230058188982453,
                0.009558463149666,
                0.382321322136899,
            ),
            lift_position_m=(
                0.218593782915783,
                0.008984635830313,
                0.403477545364772,
            ),
            orientation_xyzw=TASK006_ORIENTATION,
            gripper_position_rad=0.2,
            profile=profile,
        )


def test_profile_rejects_wrong_gripper_contact_position_and_stage_drift() -> None:
    profile = load_grasp_geometry_profile(PROFILE_PATH)
    cube = (0.28, 0.0, 0.205)
    stages = derive_grasp_stage_geometry(cube, SIDE_GRASP_ORIENTATION, profile)
    with pytest.raises(GraspGeometryError, match="gripper_contact_position_mismatch"):
        validate_grasp_stage_geometry(
            cube_center_m=cube,
            approach_position_m=stages.approach_position_m,
            descend_position_m=stages.descend_position_m,
            lift_position_m=stages.lift_position_m,
            orientation_xyzw=SIDE_GRASP_ORIENTATION,
            gripper_position_rad=0.2,
            profile=profile,
        )
    drifted = (
        stages.approach_position_m[0] + 0.001,
        *stages.approach_position_m[1:],
    )
    with pytest.raises(GraspGeometryError, match="approach_position_drift"):
        validate_grasp_stage_geometry(
            cube_center_m=cube,
            approach_position_m=drifted,
            descend_position_m=stages.descend_position_m,
            lift_position_m=stages.lift_position_m,
            orientation_xyzw=SIDE_GRASP_ORIENTATION,
            gripper_position_rad=profile.gripper_contact_position_rad,
            profile=profile,
        )


def test_profile_pin_and_quaternion_fail_closed(tmp_path: Path) -> None:
    payload = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
    payload["upstream"]["commit"] = "wrong"
    drifted = tmp_path / "drifted.json"
    drifted.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(GraspGeometryError, match="upstream pin drift"):
        load_grasp_geometry_profile(drifted)

    profile = load_grasp_geometry_profile(PROFILE_PATH)
    with pytest.raises(GraspGeometryError, match="normalized"):
        derive_grasp_stage_geometry((0.28, 0.0, 0.205), (0.0, 0.0, 0.0, 0.0), profile)

    payload = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
    payload["gazebo_proxy_preflight"]["required_contact_pad_obbs"][0][
        "rotation_rows"
    ][0][0] = 0.5
    invalid_rotation = tmp_path / "invalid-rotation.json"
    invalid_rotation.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(GraspGeometryError, match="must be orthonormal"):
        load_grasp_geometry_profile(invalid_rotation)


def test_profile_separates_tool_axis_retreat_from_world_z_clearance_and_lift() -> None:
    profile = load_grasp_geometry_profile(PROFILE_PATH)
    stages = derive_grasp_stage_geometry(
        (0.28, 0.0, 0.205), SIDE_GRASP_ORIENTATION, profile
    )
    approach_delta = tuple(
        stages.approach_position_m[axis] - stages.descend_position_m[axis]
        for axis in range(3)
    )
    lift_delta = tuple(
        stages.lift_position_m[axis] - stages.descend_position_m[axis]
        for axis in range(3)
    )
    assert approach_delta == pytest.approx(
        (0.024759944023714295, 0.016939456068207294, 0.08000023624219106),
        abs=1e-12,
    )
    assert lift_delta == pytest.approx((0.0, 0.0, 0.04), abs=1e-12)


def test_profile_rejects_ambiguous_offset_frames(tmp_path: Path) -> None:
    payload = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
    payload["lift_offset_planning_m"] = [0.01, 0.0, 0.04]
    ambiguous = tmp_path / "ambiguous.json"
    ambiguous.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(GraspGeometryError, match="must use only axis 2"):
        load_grasp_geometry_profile(ambiguous)

    payload = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
    payload["approach_offset_end_effector_m"] = [0.0, 0.0, -0.08]
    wrong_direction = tmp_path / "wrong-direction.json"
    wrong_direction.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(GraspGeometryError, match="must be positive"):
        load_grasp_geometry_profile(wrong_direction)


def test_task039_palm_contact_geometry_is_rejected_by_proxy_envelope() -> None:
    profile = load_grasp_geometry_profile(PROFILE_PATH)
    cube = (0.24695465627174787, 0.1218646928533295, 0.205)
    with pytest.raises(
        GraspGeometryError,
        match="noncontact_body_clearance:gripper_link_collision",
    ):
        validate_grasp_collision_envelope(
            cube_center_m=cube,
            approach_position_m=(
                0.27859814485128837,
                0.1733906932169842,
                0.29693801242225025,
            ),
            descend_position_m=(
                0.2538169071284581,
                0.15643670690409664,
                0.2166595364540212,
            ),
            orientation_xyzw=SIDE_GRASP_ORIENTATION,
            profile=profile,
        )


def test_candidate003_aabb_false_positive_is_rejected_by_exact_obb(
    tmp_path: Path,
) -> None:
    payload = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
    payload["gripper_contact_position_rad"] = 0.77
    preflight = payload["gazebo_proxy_preflight"]
    preflight["required_contact_pad_aabbs"][1]["minimum_m"] = [
        -0.089685117748,
        -0.008488939421,
        -0.0467978175,
    ]
    preflight["required_contact_pad_aabbs"][1]["maximum_m"] = [
        -0.04989892625,
        0.008725534298,
        -0.00658913632,
    ]
    moving = preflight["required_contact_pad_obbs"][1]
    moving["center_m"] = [-0.06979202451715, 0.000118121, -0.026693548518943]
    moving["rotation_rows"] = [
        [-0.717910652987149, 0.696135282576598, 0.000000000000001],
        [-0.000000000000001, 0.000000000000001, -1.0],
        [-0.696135282576598, -0.717910652987149, 0.0],
    ]
    legacy = tmp_path / "candidate003-legacy-aabb.json"
    legacy.write_text(json.dumps(payload), encoding="utf-8")
    profile = load_grasp_geometry_profile(legacy)
    cube = (0.24695465627174787, 0.1218646928533295, 0.205)
    stages = derive_grasp_stage_geometry(cube, SIDE_GRASP_ORIENTATION, profile)
    with pytest.raises(
        GraspGeometryError,
        match=(
            "required_pad_obb_overlap:"
            "edgegrasp_grasp_proxy_moving_finger_pad:-0.00552"
        ),
    ):
        validate_grasp_collision_envelope(
            cube_center_m=cube,
            approach_position_m=stages.approach_position_m,
            descend_position_m=stages.descend_position_m,
            orientation_xyzw=SIDE_GRASP_ORIENTATION,
            profile=profile,
        )


def test_read_only_fk_scanner_supports_bounded_contact_point_refinement() -> None:
    source = (PROJECT_ROOT / "scripts" / "scan_so101_valid_fk.py").read_text(
        encoding="utf-8"
    )
    for flag in (
        "--point-in-frame",
        "--lift-bounds",
        "--elbow-bounds",
        "--wrist-bounds",
        "--min-frame-above-point-m",
        "--min-tool-z-world",
        "--roll-values",
        "--grasp-profile",
        "--approach-position",
        "--approach-orientation",
    ):
        assert flag in source
    assert "transform_point_from_frame" in source
    assert "validate_routed_grasp_stage_geometry" in source
    assert "WRIST_SCAN_LIMITS_RAD = (-1.60806, 1.60806)" in source
    assert "ActionClient" not in source
    assert "create_publisher" not in source


def test_joint_plan_probe_is_plan_only_and_reuses_adapter_validation() -> None:
    source = (PROJECT_ROOT / "scripts" / "probe_so101_joint_plan.py").read_text(
        encoding="utf-8"
    )
    assert "GetMotionPlan" in source
    assert "validate_and_convert_robot_trajectory" in source
    assert '"/plan_kinematic_path"' in source
    assert 'parser.add_argument("--pipeline-id", default="ompl")' in source
    assert "motion.pipeline_id = args.pipeline_id" in source
    assert 'choices=("joint", "pose")' in source
    assert 'choices=("base_link", "world")' in source
    assert "--capture-source-positions" in source
    assert "PositionConstraint" in source
    assert "OrientationConstraint" in source
    assert "SolidPrimitive.SPHERE" in source
    assert 'position.link_name = "gripper_frame_link"' in source
    assert 'orientation.link_name = "gripper_frame_link"' in source
    assert '"execution_attempted": False' in source
    assert "ActionClient" not in source
    assert "create_publisher" not in source


def test_side_grasp_candidate_matches_scene_profile_and_plan_only_boundary() -> None:
    candidate = json.loads(SIDE_GRASP_PATH.read_text(encoding="utf-8"))
    scene = load_scene_contract(SCENE_PATH)
    profile = load_grasp_geometry_profile(PROFILE_PATH)
    cube = next(item for item in scene.objects if item.object_id == "target_cube")
    assert candidate["scene_contract_sha256"] == scene.digest
    assert tuple(candidate["target"]["center_m"]) == cube.pose_world.position_m
    assert tuple(candidate["target"]["size_m"]) == cube.size_m
    approach_orientation = tuple(candidate["orientations_xyzw"]["approach"])
    grasp_orientation = tuple(candidate["orientations_xyzw"]["grasp"])
    validated = validate_routed_grasp_stage_geometry(
        cube_center_m=cube.pose_world.position_m,
        approach_position_m=tuple(candidate["stages"]["approach"]["position_m"]),
        descend_position_m=tuple(candidate["stages"]["descend"]["position_m"]),
        lift_position_m=tuple(candidate["stages"]["lift"]["position_m"]),
        approach_orientation_xyzw=approach_orientation,
        grasp_orientation_xyzw=grasp_orientation,
        gripper_position_rad=candidate["stages"]["lift"]["gripper_rad"],
        profile=profile,
    )
    assert validated.cube_center_in_frame_at_descend_m == pytest.approx(
        (-0.025199464387, -0.0105, -0.016),
        abs=max(profile.target_center_tolerance_m),
    )
    plan = candidate["plan_only_observation"]
    assert plan["execution_attempted"] is False
    assert candidate["schema_version"] == 14
    assert candidate["name"] == "so101_side_grasp_candidate_005"
    assert plan["status"] == (
        "RUNTIME_SELECTIVE_ACM_ROUTED_PTP_ALL_SEGMENTS_ACCEPTED"
    )
    assert plan["scene_policy"] == {
        "service": "/edgegrasp/set_target_pad_contacts",
        "target_cube_retained": True,
        "confirmed": True,
        "allowed_target_contact_links": [
            "edgegrasp_fixed_finger_pad_link",
            "edgegrasp_moving_finger_pad_link",
        ],
        "forbidden_target_contact_links": [
            "gripper_link",
            "moving_jaw_so101_v1_link",
        ],
    }
    direct = plan["direct_path_baselines"]
    assert direct["pilz_lin"]["moveit_error_code"] == 99999
    assert direct["pilz_lin"]["collision_pair"] == [
        "edgegrasp_target_cube",
        "gripper_link",
    ]
    assert direct["ompl_rrt_connect"] == {
        "attempts": 10,
        "moveit_success_responses": 7,
        "edgegrasp_accepted": 6,
        "edgegrasp_rejected": 4,
        "deterministic": False,
    }
    segments = plan["routed_pose_segments"]
    assert [item["name"] for item in segments] == [
        "home_to_approach_via",
        "approach_via_to_descend",
        "descend_to_lift_closed",
    ]
    assert all(item["attempts"] == item["accepted"] == 10 for item in segments)
    assert [item["source_point_count"] for item in segments] == [83, 32, 22]
    assert [item["gate_ready_point_count"] for item in segments] == [82, 31, 21]
    geometry = candidate["geometry_preflight"]
    assert geometry["status"] == "PASS_STATIC_ROUTED_APPROACH_ONLY"
    assert geometry["approach_state_validity_runtime"] is True
    assert geometry["moving_pad_minimum_normalized_overlap_m"] > 0.002
    assert candidate["stages"]["lift"]["gripper_rad"] == 0.6
    assert candidate["execution_authorized"] is False
    assert candidate["execution_blocker"] == (
        "force_closure_required_lift_and_retention_unverified"
    )
    assert candidate["runtime_execution_status"] == (
        "LATEST_CANDIDATE012_PAIRED_SEQUENCE_COMPLETED_NO_PHYSICS_GRASP"
    )
    assert candidate["physics_grasp_status"] == "FAILED_LIFT_AND_RETENTION"
    runtime = candidate["runtime_execution_observation"]
    assert runtime["sequence_completed"] is True
    assert runtime["edgegrasp_proxy_move_group_used"] is True
    assert runtime["gripper_contact_count"] == 1313
    assert runtime["per_pad_contact_histogram_captured"] is True
    assert runtime["gripper_contact_counts_by_token"] == {
        "edgegrasp_grasp_proxy_fixed_finger_pad": 1125,
        "edgegrasp_grasp_proxy_moving_finger_pad": 188,
    }
    assert runtime["all_gripper_tokens_observed"] is True
    assert runtime["simultaneous_gripper_contact_sample_count"] == 35
    assert runtime["simultaneous_contact_samples_observed"] is True
    assert runtime["simultaneous_bilateral_force_closure_verified"] is False
    assert runtime["peak_lift_m"] < runtime["required_lift_m"]
    assert runtime["physics_grasp_verified"] is False
    quality = candidate["contact_quality_runtime_observation"]
    assert quality["sequence_completed"] is True
    assert quality["physics_grasp_verified"] is False
    assert quality["simultaneous_gripper_contact_sample_count"] == 36
    assert quality["same_sample_weak_side_maxima"][
        "max_min_penetration_depth_m"
    ] > 0.0
    assert quality["same_sample_weak_side_maxima"][
        "max_min_abs_normal_force_n"
    ] > 0.0
    assert quality["peak_lift_m"] < quality["required_lift_m"]
    assert quality["retention_observed"] is False
