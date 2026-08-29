from __future__ import annotations

import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from edgegrasp.grasp_geometry import (
    derive_grasp_stage_geometry,
    load_grasp_geometry_profile,
    validate_routed_grasp_stage_geometry,
)
from edgegrasp.scene import (
    SceneContractError,
    load_scene_contract,
    render_gazebo_sdf,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONFIG = PROJECT_ROOT / "ros_ws" / "src" / "edgegrasp_ros" / "config" / "scene.json"
WORLD = PROJECT_ROOT / "ros_ws" / "src" / "edgegrasp_ros" / "worlds" / "table_cube.sdf"


def test_shared_scene_contract_drives_gazebo_and_planning_objects() -> None:
    contract = load_scene_contract(CONFIG)
    assert contract.gazebo_world_frame == "world"
    assert contract.planning_frame == "base_link"
    assert contract.clock_domain == "ros_sim"
    assert [item.planning_scene_id for item in contract.planning_objects(include_optional=False)] == [
        "edgegrasp_table"
    ]
    assert [item.planning_scene_id for item in contract.planning_objects(include_optional=True)] == [
        "edgegrasp_table",
        "edgegrasp_target_cube",
    ]

    root = ET.fromstring(render_gazebo_sdf(contract))
    world = root.find("./world[@name='edgegrasp_table_cube']")
    assert world is not None
    plugins = [plugin.get("name") for plugin in world.findall("plugin")]
    assert plugins.index("gz::sim::systems::UserCommands") < plugins.index(
        "gz::sim::systems::Contact"
    )
    assert world.findtext("plugin[@name='gz::sim::systems::Physics']/engine/filename") == (
        "gz-physics-dartsim-plugin"
    )
    for item in contract.objects:
        model = world.find(f"model[@name='{item.gazebo_name}']")
        assert model is not None
        assert tuple(map(float, model.findtext("pose").split()[:3])) == pytest.approx(
            item.pose_world.position_m
        )
        assert tuple(
            map(float, model.findtext("link/collision/geometry/box/size").split())
        ) == pytest.approx(item.size_m)
    table_contact = world.find("model[@name='table']/link/sensor/contact")
    assert table_contact is not None
    assert table_contact.findtext("topic") == "/edgegrasp/table_contacts"
    cube_contact = world.find(
        "model[@name='target_cube']/link/sensor/contact"
    )
    assert cube_contact is not None
    assert cube_contact.findtext("collision") == "collision"
    assert cube_contact.findtext("topic") == "/edgegrasp/target_cube_contacts"
    cube_pose_publisher = world.find(
        "model[@name='target_cube']/plugin[@name='gz::sim::systems::PosePublisher']"
    )
    assert cube_pose_publisher is not None
    assert cube_pose_publisher.findtext("publish_link_pose") == "false"
    assert cube_pose_publisher.findtext("publish_model_pose") == "true"
    assert cube_pose_publisher.findtext("publish_nested_model_pose") == "false"
    assert cube_pose_publisher.findtext("use_pose_vector_msg") == "false"
    assert cube_pose_publisher.findtext("update_frequency") == "100"
    assert next(
        item for item in contract.objects if item.object_id == "target_cube"
    ).pose_topic == "/model/target_cube/pose"
    # Gazebo Sim 8.11 uses the scoped default topic; custom <topic> support is
    # newer and deliberately not emitted for the pinned runtime.
    assert cube_pose_publisher.find("topic") is None
    assert WORLD.read_text(encoding="utf-8") == render_gazebo_sdf(contract)

    table, cube = contract.objects
    assert cube.pose_world.position_m[2] - cube.size_m[2] / 2.0 == pytest.approx(
        table.pose_world.position_m[2] + table.size_m[2] / 2.0
    )
    for axis in (0, 1):
        assert cube.pose_world.position_m[axis] - cube.size_m[axis] / 2.0 >= (
            table.pose_world.position_m[axis] - table.size_m[axis] / 2.0
        )
        assert cube.pose_world.position_m[axis] + cube.size_m[axis] / 2.0 <= (
            table.pose_world.position_m[axis] + table.size_m[axis] / 2.0
        )

    aligned_config = CONFIG.with_name("scene_candidate017.json")
    aligned_world = WORLD.with_name("table_cube_candidate017.sdf")
    aligned = load_scene_contract(aligned_config)
    assert aligned_world.read_text(encoding="utf-8") == render_gazebo_sdf(aligned)
    aligned_table, aligned_cube = aligned.objects
    assert aligned_cube.pose_world.position_m[2] - aligned_cube.size_m[2] / 2.0 == pytest.approx(
        aligned_table.pose_world.position_m[2] + aligned_table.size_m[2] / 2.0
    )
    for axis in (0, 1):
        assert aligned_cube.pose_world.position_m[axis] - aligned_cube.size_m[axis] / 2.0 >= (
            aligned_table.pose_world.position_m[axis]
            - aligned_table.size_m[axis] / 2.0
        )
        assert aligned_cube.pose_world.position_m[axis] + aligned_cube.size_m[axis] / 2.0 <= (
            aligned_table.pose_world.position_m[axis]
            + aligned_table.size_m[axis] / 2.0
        )

    baseline_profile = load_grasp_geometry_profile(
        CONFIG.with_name("so101_grasp_geometry_candidate011_q0p40.json")
    )
    aligned_profile = load_grasp_geometry_profile(
        CONFIG.with_name("so101_grasp_geometry_candidate017_aligned_q0p40.json")
    )
    grasp_orientation = (
        0.3450298741758752,
        0.6172118344266647,
        0.6332767577280262,
        0.3145862131297726,
    )
    baseline_stages = derive_grasp_stage_geometry(
        cube.pose_world.position_m, grasp_orientation, baseline_profile
    )
    aligned_stages = derive_grasp_stage_geometry(
        aligned_cube.pose_world.position_m, grasp_orientation, aligned_profile
    )
    assert aligned_stages.descend_position_m == pytest.approx(
        baseline_stages.descend_position_m, abs=1e-12
    )
    assert aligned_stages.lift_position_m == pytest.approx(
        baseline_stages.lift_position_m, abs=1e-12
    )

    preclose_profile = load_grasp_geometry_profile(
        CONFIG.with_name(
            "so101_grasp_geometry_candidate019_preclose_clearance_q0p40.json"
        )
    )
    preclose_candidate = json.loads(
        CONFIG.with_name(
            "so101_side_grasp_candidate019_preclose_clearance.json"
        ).read_text(encoding="utf-8")
    )
    preclose_stages = derive_grasp_stage_geometry(
        cube.pose_world.position_m, grasp_orientation, preclose_profile
    )
    assert preclose_profile.preclose_contact_policy is not None
    assert (
        preclose_profile.preclose_contact_policy.enable_target_pad_contacts_after
        == "descend_terminal"
    )
    validated = validate_routed_grasp_stage_geometry(
        cube_center_m=cube.pose_world.position_m,
        approach_position_m=tuple(
            preclose_candidate["stages"]["approach"]["position_m"]
        ),
        descend_position_m=preclose_stages.descend_position_m,
        lift_position_m=preclose_stages.lift_position_m,
        approach_orientation_xyzw=grasp_orientation,
        grasp_orientation_xyzw=grasp_orientation,
        gripper_position_rad=preclose_profile.gripper_contact_position_rad,
        profile=preclose_profile,
    )
    assert validated.descend_position_m == pytest.approx(
        (0.23941358427044526, 0.1612233128413513, 0.2173027926431172),
        abs=1e-12,
    )

    face_aligned_config = CONFIG.with_name(
        "scene_candidate024_face_aligned.json"
    )
    face_aligned_world = WORLD.with_name(
        "table_cube_candidate024_face_aligned.sdf"
    )
    face_aligned = load_scene_contract(face_aligned_config)
    assert face_aligned_world.read_text(encoding="utf-8") == render_gazebo_sdf(
        face_aligned
    )
    face_aligned_root = ET.fromstring(render_gazebo_sdf(face_aligned))
    face_aligned_pose = tuple(
        map(
            float,
            face_aligned_root.findtext(
                "./world/model[@name='target_cube']/pose"
            ).split(),
        )
    )
    assert face_aligned_pose == pytest.approx(
        (
            0.24108428841333183,
            0.12958666029737353,
            0.205,
            0.0,
            0.0,
            0.6500077341171558,
        ),
        abs=1e-12,
    )

    face_aligned_cube = next(
        item for item in face_aligned.objects if item.object_id == "target_cube"
    )
    face_aligned_profile = load_grasp_geometry_profile(
        CONFIG.with_name(
            "so101_grasp_geometry_candidate024_face_aligned_q0p40.json"
        )
    )
    face_aligned_candidate = json.loads(
        CONFIG.with_name(
            "so101_side_grasp_candidate024_face_aligned.json"
        ).read_text(encoding="utf-8")
    )
    face_aligned_stages = derive_grasp_stage_geometry(
        face_aligned_cube.pose_world.position_m,
        tuple(face_aligned_candidate["orientations_xyzw"]["grasp"]),
        face_aligned_profile,
    )
    baseline_candidate = json.loads(
        CONFIG.with_name(
            "so101_side_grasp_candidate020_reachable_preclose.json"
        ).read_text(encoding="utf-8")
    )
    baseline_scene = load_scene_contract(CONFIG)
    baseline_cube = next(
        item for item in baseline_scene.objects if item.object_id == "target_cube"
    )
    baseline_profile = load_grasp_geometry_profile(
        CONFIG.with_name(
            "so101_grasp_geometry_candidate020_reachable_preclose_q0p40.json"
        )
    )
    baseline_stages = derive_grasp_stage_geometry(
        baseline_cube.pose_world.position_m,
        tuple(baseline_candidate["orientations_xyzw"]["grasp"]),
        baseline_profile,
    )
    assert face_aligned_stages.descend_position_m == pytest.approx(
        baseline_stages.descend_position_m,
        abs=1e-12,
    )
    assert face_aligned_stages.lift_position_m == pytest.approx(
        baseline_stages.lift_position_m,
        abs=1e-12,
    )
    validate_routed_grasp_stage_geometry(
        cube_center_m=face_aligned_cube.pose_world.position_m,
        approach_position_m=tuple(
            face_aligned_candidate["stages"]["approach"]["position_m"]
        ),
        descend_position_m=face_aligned_stages.descend_position_m,
        lift_position_m=face_aligned_stages.lift_position_m,
        approach_orientation_xyzw=tuple(
            face_aligned_candidate["orientations_xyzw"]["approach"]
        ),
        grasp_orientation_xyzw=tuple(
            face_aligned_candidate["orientations_xyzw"]["grasp"]
        ),
        gripper_position_rad=face_aligned_profile.gripper_contact_position_rad,
        profile=face_aligned_profile,
        cube_orientation_xyzw=face_aligned_cube.pose_world.quaternion_xyzw,
    )


def test_scene_contract_rejects_nonidentity_phase_one_transform(tmp_path: Path) -> None:
    payload = json.loads(CONFIG.read_text(encoding="utf-8"))
    payload["frames"]["world_to_planning"]["position_m"][0] = 0.1
    path = tmp_path / "bad-scene.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(SceneContractError, match="must be identity"):
        load_scene_contract(path)


def test_scene_contract_rejects_duplicate_planning_ids(tmp_path: Path) -> None:
    payload = json.loads(CONFIG.read_text(encoding="utf-8"))
    payload["objects"][1]["planning_scene_id"] = payload["objects"][0][
        "planning_scene_id"
    ]
    path = tmp_path / "bad-scene.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(SceneContractError, match="duplicate"):
        load_scene_contract(path)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    (
        ("z", 0.5, "bottom must equal table top"),
        ("x", 0.9, "footprint must stay inside table"),
    ),
)
def test_scene_contract_rejects_invalid_cube_geometry(
    tmp_path: Path, field: str, value: float, message: str
) -> None:
    payload = json.loads(CONFIG.read_text(encoding="utf-8"))
    axis = {"x": 0, "z": 2}[field]
    payload["objects"][1]["pose_world"]["position_m"][axis] = value
    path = tmp_path / "bad-scene.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(SceneContractError, match=message):
        load_scene_contract(path)
