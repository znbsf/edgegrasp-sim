from __future__ import annotations

import hashlib
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import pytest

from edgegrasp.collision_proxy import (
    CollisionProxyError,
    apply_collision_proxies,
    load_collision_proxy_contract,
    with_moving_pad_distal_extension,
)
from edgegrasp.contact_material import (
    load_pad_contact_material_contract,
    select_pad_contact_material_profile,
)
from edgegrasp.sim_control import (
    apply_gripper_effort_pid_overlay,
    apply_gripper_position_effort_overlay,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONTRACT_PATH = (
    PROJECT_ROOT
    / "ros_ws"
    / "src"
    / "edgegrasp_ros"
    / "config"
    / "so101_collision_proxies.json"
)
MATERIAL_CONTRACT_PATH = (
    PROJECT_ROOT
    / "ros_ws"
    / "src"
    / "edgegrasp_ros"
    / "config"
    / "so101_pad_contact_materials.json"
)
UPSTREAM_MESH_ROOT = (
    PROJECT_ROOT.parents[1]
    / "workspaces"
    / "forks"
    / "so101_ros2"
    / "so101_description"
    / "meshes"
    / "so_arm101"
)
UPSTREAM_XACRO = (
    PROJECT_ROOT.parents[1]
    / "workspaces"
    / "forks"
    / "so101_ros2"
    / "so101_description"
    / "urdf"
    / "so101_robot.urdf.xacro"
)


def test_generated_gripper_control_overlays_are_bounded_and_explicit(
    tmp_path: Path,
) -> None:
    controller_config = tmp_path / "controllers.yaml"
    controller_config.write_text("controller_manager: {}\n", encoding="utf-8")
    source = """<robot name="so101">
      <ros2_control name="GazeboSimSystem" type="system">
        <joint name="gripper">
          <command_interface name="position"/>
          <state_interface name="position"/>
          <state_interface name="velocity"/>
          <state_interface name="effort"/>
        </joint>
      </ros2_control>
      <gazebo><plugin name="gz_ros2_control::GazeboSimROS2ControlPlugin">
        <parameters>upstream.yaml</parameters>
      </plugin></gazebo>
    </robot>"""
    transformed, report = apply_gripper_position_effort_overlay(
        source, controller_config_path=controller_config
    )
    root = ET.fromstring(transformed)
    joint = root.find("./ros2_control/joint[@name='gripper']")
    assert [item.get("name") for item in joint.findall("command_interface")] == [
        "position",
        "effort",
    ]
    assert report.effort_limit_nm == 0.5
    assert root.findtext("./gazebo/plugin/parameters") == str(controller_config)

    effort_only, effort_report = apply_gripper_effort_pid_overlay(
        source, controller_config_path=controller_config
    )
    effort_root = ET.fromstring(effort_only)
    effort_joint = effort_root.find("./ros2_control/joint[@name='gripper']")
    assert [
        item.get("name") for item in effort_joint.findall("command_interface")
    ] == ["effort"]
    effort_interface = effort_joint.find("command_interface[@name='effort']")
    assert float(effort_interface.findtext("param[@name='min']")) == -0.5
    assert float(effort_interface.findtext("param[@name='max']")) == 0.5
    assert effort_report.profile == "effort_pid_preload"


@pytest.mark.skipif(
    not UPSTREAM_XACRO.is_file(), reason="pinned local SO-101 checkout absent"
)
def test_gripper_contact_kinematics_match_the_pinned_xacro() -> None:
    contract = load_collision_proxy_contract(CONTRACT_PATH)
    kinematics = contract["gripper_contact_kinematics"]
    root = ET.parse(UPSTREAM_XACRO).getroot()

    for contract_name in ("reference_joint", "moving_joint"):
        expected = kinematics[contract_name]
        joint = root.find(f".//joint[@name='{expected['name']}']")
        assert joint is not None
        assert joint.get("type") == expected["type"]
        assert joint.find("parent").get("link") == kinematics["common_parent_link"]
        assert joint.find("child").get("link") == expected["child_link"]
        origin = joint.find("origin")
        assert tuple(float(item) for item in origin.get("xyz").split()) == pytest.approx(
            expected["origin_xyz"], abs=1e-12
        )
        assert tuple(float(item) for item in origin.get("rpy").split()) == pytest.approx(
            expected["origin_rpy"], abs=1e-12
        )

    moving = root.find(".//joint[@name='gripper']")
    assert tuple(float(item) for item in moving.find("axis").get("xyz").split()) == (
        0.0,
        0.0,
        1.0,
    )
    limits = moving.find("limit")
    assert float(limits.get("lower")) == pytest.approx(
        kinematics["moving_joint"]["lower_rad"], abs=1e-12
    )
    assert float(limits.get("upper")) == pytest.approx(
        kinematics["moving_joint"]["upper_rad"], abs=1e-12
    )


def _synthetic_expanded_urdf(contract: dict) -> str:
    root = ET.Element("robot", {"name": "so101"})
    by_link: dict[str, list[dict]] = {}
    for proxy in contract["proxies"]:
        by_link.setdefault(proxy["link"], []).append(proxy)
    for link_name, proxies in by_link.items():
        link = ET.SubElement(root, "link", {"name": link_name})
        for proxy in sorted(proxies, key=lambda item: item["collision_index"]):
            visual = ET.SubElement(link, "visual")
            visual_geometry = ET.SubElement(visual, "geometry")
            ET.SubElement(
                visual_geometry,
                "mesh",
                {"filename": f"file:///pinned/{proxy['source_mesh']}"},
            )
            collision = ET.SubElement(link, "collision")
            ET.SubElement(
                collision,
                "origin",
                {
                    "xyz": " ".join(map(str, proxy["source_pose_xyz"])),
                    "rpy": " ".join(map(str, proxy["source_pose_rpy"])),
                },
            )
            geometry = ET.SubElement(collision, "geometry")
            ET.SubElement(
                geometry,
                "mesh",
                {"filename": f"file:///pinned/{proxy['source_mesh']}"},
            )
    for proxy in contract["gazebo_gripper_contact_extensions"]:
        link_name = proxy["link"]
        link = root.find(f"./link[@name='{link_name}']")
        if link is None:
            link = ET.SubElement(root, "link", {"name": link_name})
        visual = ET.SubElement(link, "visual")
        visual_geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(
            visual_geometry,
            "mesh",
            {"filename": f"file:///pinned/{proxy['source_mesh']}"},
        )
        index = int(proxy["reference_collision_index"])
        while len(link.findall("collision")) <= index:
            slot = len(link.findall("collision"))
            collision = ET.SubElement(link, "collision")
            origin = ET.SubElement(collision, "origin")
            geometry = ET.SubElement(collision, "geometry")
            box = ET.SubElement(geometry, "box")
            if slot == index:
                origin.set("xyz", " ".join(map(str, proxy["source_pose_xyz"])))
                origin.set("rpy", " ".join(map(str, proxy["source_pose_rpy"])))
                box.set(
                    "size",
                    " ".join(map(str, proxy["expected_upstream_box_size_m"])),
                )
            else:
                origin.set("xyz", "0 0 0")
                origin.set("rpy", "0 0 0")
                box.set("size", "0.04 0.04 0.05")
    link_names = [link.get("name", "") for link in root.findall("link")]
    for index, (parent, child) in enumerate(zip(link_names, link_names[1:])):
        joint = ET.SubElement(
            root, "joint", {"name": f"synthetic_joint_{index}", "type": "fixed"}
        )
        ET.SubElement(joint, "parent", {"link": parent})
        ET.SubElement(joint, "child", {"link": child})
    return ET.tostring(root, encoding="unicode")


def _stl_vertices(path: Path) -> list[tuple[float, float, float]]:
    payload = path.read_bytes()
    count = struct.unpack_from("<I", payload, 80)[0]
    if 84 + count * 50 != len(payload):
        raise AssertionError(f"expected binary STL: {path}")
    vertices: list[tuple[float, float, float]] = []
    for index in range(count):
        values = struct.unpack_from("<12fH", payload, 84 + index * 50)
        vertices.extend((values[3:6], values[6:9], values[9:12]))
    return vertices


def _rotate_rpy(
    rpy: list[float], vector: tuple[float, float, float]
) -> tuple[float, float, float]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )
    return tuple(
        sum(rotation[row][column] * vector[column] for column in range(3))
        for row in range(3)
    )


def test_proxy_transform_replaces_exact_contract_and_preserves_visuals() -> None:
    contract = load_collision_proxy_contract(CONTRACT_PATH)
    extended = with_moving_pad_distal_extension(contract, 0.012)
    baseline_moving = contract["gazebo_gripper_contact_extensions"][1]
    extended_moving = extended["gazebo_gripper_contact_extensions"][1]
    baseline_proximal_y = (
        baseline_moving["proxy_pose_xyz"][1]
        + baseline_moving["proxy_size_m"][1] / 2.0
    )
    extended_proximal_y = (
        extended_moving["proxy_pose_xyz"][1]
        + extended_moving["proxy_size_m"][1] / 2.0
    )
    assert extended_moving["proxy_size_m"][1] == pytest.approx(
        baseline_moving["proxy_size_m"][1] + 0.012
    )
    assert extended_proximal_y == pytest.approx(baseline_proximal_y, abs=1e-15)
    assert contract["gazebo_gripper_contact_extensions"][1] == baseline_moving
    transformed, report = apply_collision_proxies(
        _synthetic_expanded_urdf(contract), contract
    )
    root = ET.fromstring(transformed)
    assert len(report.replaced) == 13
    assert len(report.validated_collision_meshes) == 13
    assert len(report.contact_extensions_added) == 2
    assert report.visual_meshes_before == report.visual_meshes_after
    assert report.remaining_collision_meshes == ()
    assert report.world_anchor_added is True
    assert report.root_links_before == ("base_link",)
    assert report.root_links_after == ("world",)
    world_joint = root.find("./joint[@name='world_joint']")
    assert world_joint is not None and world_joint.get("type") == "fixed"
    assert world_joint.find("parent").get("link") == "world"
    assert world_joint.find("child").get("link") == "base_link"
    assert len(root.findall("./link/collision/geometry/box")) == 18
    assert root.findall("./link/collision/geometry/mesh") == []
    assert len(root.findall("./link/visual/geometry/mesh")) == 15
    collision_names = [
        collision.get("name", "") for collision in root.findall("./link/collision")
    ]
    assert sum(name.startswith("edgegrasp_proxy_") for name in collision_names) == 13
    assert sum(
        name.startswith("edgegrasp_grasp_proxy_") for name in collision_names
    ) == 2
    fixed = root.find(
        "./link[@name='edgegrasp_fixed_finger_pad_link']/collision"
    )
    moving = root.find(
        "./link[@name='edgegrasp_moving_finger_pad_link']/collision"
    )
    assert fixed is not None and moving is not None
    assert fixed.get("name") == "edgegrasp_grasp_proxy_fixed_finger_pad"
    assert moving.get("name") == "edgegrasp_grasp_proxy_moving_finger_pad"
    fixed_joint = root.find("./joint[@name='edgegrasp_fixed_finger_pad_joint']")
    moving_joint = root.find("./joint[@name='edgegrasp_moving_finger_pad_joint']")
    assert fixed_joint is not None and fixed_joint.get("type") == "fixed"
    assert fixed_joint.find("parent").get("link") == "gripper_link"
    assert fixed_joint.find("child").get("link") == "edgegrasp_fixed_finger_pad_link"
    assert moving_joint is not None and moving_joint.get("type") == "fixed"
    assert moving_joint.find("parent").get("link") == "moving_jaw_so101_v1_link"
    assert moving_joint.find("child").get("link") == "edgegrasp_moving_finger_pad_link"
    assert tuple(float(value) for value in fixed.find("geometry/box").get("size").split()) == pytest.approx(
        contract["gazebo_gripper_contact_extensions"][0]["proxy_size_m"]
    )
    assert tuple(float(value) for value in moving.find("geometry/box").get("size").split()) == pytest.approx(
        contract["gazebo_gripper_contact_extensions"][1]["proxy_size_m"]
    )


def test_proxy_transform_scopes_explicit_friction_to_both_generated_pads() -> None:
    contract = load_collision_proxy_contract(CONTRACT_PATH)
    materials = load_pad_contact_material_contract(MATERIAL_CONTRACT_PATH)
    profile = select_pad_contact_material_profile(
        materials, "candidate012_treatment_mu1p5"
    )
    transformed, report = apply_collision_proxies(
        _synthetic_expanded_urdf(contract),
        contract,
        pad_contact_material=profile,
    )
    root = ET.fromstring(transformed)
    pad_collisions = [
        root.find(f"./link[@name='{link_name}']/collision")
        for link_name in profile.target_links
    ]
    assert all(item is not None and item.get("name") is None for item in pad_collisions)
    gazebo_extensions = root.findall("./gazebo")
    assert [item.get("reference") for item in gazebo_extensions] == list(
        profile.target_links
    )
    assert [float(item.findtext("mu1")) for item in gazebo_extensions] == [
        1.5,
        1.5,
    ]
    assert [float(item.findtext("mu2")) for item in gazebo_extensions] == [
        1.5,
        1.5,
    ]
    assert report.contact_material_profile == profile.name
    assert len(report.contact_material_references) == 2


def test_implicit_contact_material_preserves_pre_candidate012_urdf() -> None:
    contract = load_collision_proxy_contract(CONTRACT_PATH)
    materials = load_pad_contact_material_contract(MATERIAL_CONTRACT_PATH)
    profile = select_pad_contact_material_profile(materials, "implicit_default")
    transformed, report = apply_collision_proxies(
        _synthetic_expanded_urdf(contract),
        contract,
        pad_contact_material=profile,
    )
    root = ET.fromstring(transformed)
    assert root.findall("./gazebo") == []
    assert report.contact_material_profile == "implicit_default"
    assert report.contact_material_references == ()


def test_moveit_proxy_rejects_gazebo_contact_material_profile() -> None:
    contract = load_collision_proxy_contract(CONTRACT_PATH)
    materials = load_pad_contact_material_contract(MATERIAL_CONTRACT_PATH)
    profile = select_pad_contact_material_profile(
        materials, "candidate012_control_mu1p0"
    )
    with pytest.raises(CollisionProxyError, match="generated Gazebo proxy"):
        apply_collision_proxies(
            _synthetic_expanded_urdf(contract),
            contract,
            require_no_collision_meshes=False,
            anchor_to_world=False,
            replace_collision_meshes=False,
            pad_contact_material=profile,
        )


def test_proxy_transform_fails_closed_on_mesh_drift() -> None:
    contract = load_collision_proxy_contract(CONTRACT_PATH)
    root = ET.fromstring(_synthetic_expanded_urdf(contract))
    root.find("./link/collision/geometry/mesh").set("filename", "wrong.stl")
    with pytest.raises(CollisionProxyError, match="mesh drift"):
        apply_collision_proxies(ET.tostring(root, encoding="unicode"), contract)


def test_moveit_transform_keeps_meshes_unanchored_and_adds_contact_pads() -> None:
    contract = load_collision_proxy_contract(CONTRACT_PATH)
    transformed, report = apply_collision_proxies(
        _synthetic_expanded_urdf(contract),
        contract,
        require_no_collision_meshes=False,
        anchor_to_world=False,
        replace_collision_meshes=False,
    )
    root = ET.fromstring(transformed)
    assert report.world_anchor_added is False
    assert report.root_links_before == ("base_link",)
    assert report.root_links_after == ("base_link",)
    assert root.find("./link[@name='world']") is None
    assert root.find("./joint[@name='world_joint']") is None
    assert len(root.findall("./link/collision/geometry/mesh")) == 13
    assert report.replaced == ()
    assert len(report.validated_collision_meshes) == 13
    assert len(report.contact_extensions_added) == 2
    assert root.find("./link[@name='edgegrasp_fixed_finger_pad_link']") is not None
    assert root.find("./link[@name='edgegrasp_moving_finger_pad_link']") is not None


def test_moveit_proxy_mode_refuses_an_existing_world_anchor() -> None:
    contract = load_collision_proxy_contract(CONTRACT_PATH)
    anchored, _ = apply_collision_proxies(
        _synthetic_expanded_urdf(contract), contract
    )
    with pytest.raises(CollisionProxyError, match="must not contain"):
        apply_collision_proxies(anchored, contract, anchor_to_world=False)


def test_proxy_transform_fails_closed_on_gripper_primitive_drift() -> None:
    contract = load_collision_proxy_contract(CONTRACT_PATH)
    root = ET.fromstring(_synthetic_expanded_urdf(contract))
    box = root.find(
        "./link[@name='gripper_link']/collision[2]/geometry/box"
    )
    assert box is not None
    box.set("size", "0.01 0.01 0.01")
    with pytest.raises(CollisionProxyError, match="gripper primitive size"):
        apply_collision_proxies(ET.tostring(root, encoding="unicode"), contract)


def test_proxy_transform_fails_closed_on_ambiguous_root() -> None:
    contract = load_collision_proxy_contract(CONTRACT_PATH)
    root = ET.fromstring(_synthetic_expanded_urdf(contract))
    ET.SubElement(root, "link", {"name": "unexpected_root"})
    with pytest.raises(CollisionProxyError, match="single unanchored base root"):
        apply_collision_proxies(ET.tostring(root, encoding="unicode"), contract)


def test_dart_proxy_launch_keeps_collision_and_behavior_observability() -> None:
    ros_package = PROJECT_ROOT / "ros_ws" / "src" / "edgegrasp_ros"
    world = ET.parse(ros_package / "worlds" / "table_cube.sdf").getroot()
    engine = world.find(
        "./world/plugin[@name='gz::sim::systems::Physics']/engine/filename"
    )
    assert engine is not None and engine.text == "gz-physics-dartsim-plugin"
    assert world.find("./world/plugin[@name='gz::sim::systems::Contact']") is not None
    plugin_names = [plugin.get("name") for plugin in world.findall("./world/plugin")]
    assert plugin_names.index("gz::sim::systems::UserCommands") < plugin_names.index(
        "gz::sim::systems::Contact"
    )
    contact = world.find(
        "./world/model[@name='table']/link/sensor[@name='table_contact']"
    )
    assert contact is not None
    assert contact.findtext("contact/collision") == "collision"
    assert contact.findtext("contact/topic") == "/edgegrasp/table_contacts"
    assert contact.find("topic") is None

    launch = (
        ros_package / "launch" / "edgegrasp_proxy_gazebo.launch.py"
    ).read_text(encoding="utf-8")
    assert "build_proxy_robot_description" in launch
    assert '" -s -r -v 4 "' in launch
    assert '"edgegrasp_table_cube"' in launch
    assert '"/robot_description"' in launch
    assert "load_ros2_controllers.launch.py" in launch
    assert '"-allow_renaming",\n                    "false"' in launch
    assert "static_transform_publisher" not in launch
    assert '"pad_contact_material_profile"' in launch
    assert 'default_value="implicit_default"' in launch
    assert 'DeclareLaunchArgument("world_filename"' in launch
    assert 'DeclareLaunchArgument("scene_config_filename"' in launch
    assert '"scene_config": scene_config' in launch

    moveit_launch = (
        ros_package / "launch" / "edgegrasp_proxy_move_group.launch.py"
    ).read_text(encoding="utf-8")
    assert "MoveItConfigsBuilder" in moveit_launch
    assert "build_proxy_robot_description" in moveit_launch
    assert "world_anchor=False" in moveit_launch
    assert "replace_collision_meshes=False" in moveit_launch
    assert 'moveit_parameters["robot_description"] = proxy_description' in moveit_launch
    assert "ExecuteTaskSolutionCapability" in moveit_launch
    assert '"capabilities"' not in moveit_launch
    assert '"allow_trajectory_execution": False' in moveit_launch
    assert '"disable_capabilities"' in moveit_launch
    assert 'pilz_parameters["response_adapters"]' in moveit_launch
    assert "default_planning_response_adapters/ValidateSolution" in moveit_launch
    for upstream_config in (
        "moveit_controllers.yaml",
        "joint_limits.yaml",
        "kinematics.yaml",
        "pilz_cartesian_limits.yaml",
    ):
        assert upstream_config in moveit_launch
    assert 'f"{robot_name}.srdf"' in moveit_launch

    package_xml = (ros_package / "package.xml").read_text(encoding="utf-8")
    for dependency in (
        "moveit_configs_utils",
        "moveit_ros_move_group",
        "rviz2",
    ):
        assert f"<exec_depend>{dependency}</exec_depend>" in package_xml

    probe = ET.parse(ros_package / "worlds" / "base_proxy_probe.sdf").getroot()
    probe_model = probe.find("./model[@name='edgegrasp_base_proxy_probe']")
    assert probe_model is not None
    assert probe_model.findtext("pose") == "0 0 0 0 0 0"
    assert probe_model.findtext("link/collision/geometry/box/size") == (
        "0.015 0.015 0.015"
    )
    probe_contact = probe_model.find("link/sensor[@name='probe_contact']")
    assert probe_contact is not None
    assert probe_contact.findtext("contact/collision") == "probe_collision"
    assert probe_contact.findtext("contact/topic") == (
        "/edgegrasp/base_proxy_probe_contacts"
    )
    probe_script = (PROJECT_ROOT / "scripts" / "probe_base_collision.sh").read_text(
        encoding="utf-8"
    )
    assert 'gz service -l | grep -Fxq "/world/${world_name}/create"' in probe_script
    assert "ros2 service type" not in probe_script
    assert "INITIAL_POSE" in probe_script
    assert "SETTLED_POSE_T_PLUS_3S" in probe_script
    assert "RETENTION_POSE_T_PLUS_5S" in probe_script
    assert "one_base_proxy_contact_only" in probe_script
    assert '-x "$probe_x"' in probe_script
    assert '-y "$probe_y"' in probe_script
    assert '-z "$probe_z"' in probe_script
    assert 'probe_x="0.0"' in probe_script
    assert "gz topic" in probe_script
    assert "/edgegrasp/base_proxy_probe_contacts" in probe_script
    assert "SETTLED_Z=" in probe_script
    assert "RETENTION_Z=" in probe_script
    assert "settled_tolerance" in probe_script
    assert "retention_tolerance" in probe_script
    assert "edgegrasp_proxy_base_link_collision_1" in probe_script
    assert "SPECIFIC_BASE_PROXY_CONTACT_PASS" in probe_script


@pytest.mark.skipif(
    not UPSTREAM_MESH_ROOT.is_dir(), reason="pinned local SO-101 checkout absent"
)
def test_proxy_dimensions_and_pose_are_derived_from_pinned_stl() -> None:
    contract = load_collision_proxy_contract(CONTRACT_PATH)
    padding = float(contract["method"]["padding_per_side_m"])
    cache: dict[str, tuple[list[float], list[float]]] = {}
    all_proxies = (
        *contract["proxies"],
        *contract["gazebo_gripper_contact_extensions"],
    )
    for proxy in all_proxies:
        mesh_path = UPSTREAM_MESH_ROOT / proxy["source_mesh"]
        assert hashlib.sha256(mesh_path.read_bytes()).hexdigest() == proxy[
            "source_mesh_sha256"
        ]
        if proxy["source_mesh"] not in cache:
            vertices = _stl_vertices(mesh_path)
            region = proxy.get("mesh_region")
            if region is not None:
                axis = int(region["axis"])
                minimum_bound = region.get("min_m")
                maximum_bound = region.get("max_m")
                vertices = [
                    vertex
                    for vertex in vertices
                    if (minimum_bound is None or vertex[axis] >= minimum_bound)
                    and (maximum_bound is None or vertex[axis] <= maximum_bound)
                ]
                assert vertices
            minimum = [min(vertex[axis] for vertex in vertices) for axis in range(3)]
            maximum = [max(vertex[axis] for vertex in vertices) for axis in range(3)]
            cache[proxy["source_mesh"]] = (minimum, maximum)
        minimum, maximum = cache[proxy["source_mesh"]]
        center = tuple((low + high) / 2.0 for low, high in zip(minimum, maximum))
        size = tuple(high - low + 2.0 * padding for low, high in zip(minimum, maximum))
        assert tuple(proxy["mesh_local_center_m"]) == pytest.approx(center, abs=1e-8)
        assert tuple(proxy["proxy_size_m"]) == pytest.approx(size, abs=1e-8)
        rotated_center = _rotate_rpy(proxy["source_pose_rpy"], center)
        proxy_origin = tuple(
            source + offset
            for source, offset in zip(proxy["source_pose_xyz"], rotated_center)
        )
        assert tuple(proxy["proxy_pose_xyz"]) == pytest.approx(proxy_origin, abs=1e-7)
