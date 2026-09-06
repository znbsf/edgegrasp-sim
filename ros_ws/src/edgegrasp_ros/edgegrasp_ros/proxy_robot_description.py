"""Build an SO-101 collision-proxy robot description for Gazebo or MoveIt."""

from __future__ import annotations

from pathlib import Path
import subprocess
from edgegrasp.camera import configure_sim_camera

from ament_index_python.packages import get_package_share_directory
from edgegrasp.collision_proxy import (
    CollisionProxyReport,
    apply_collision_proxies,
    load_collision_proxy_contract,
    with_moving_pad_distal_extension,
)
from edgegrasp.contact_material import (
    load_pad_contact_material_contract,
    select_pad_contact_material_profile,
)
from edgegrasp.sim_control import (
    EFFORT_PID_PRELOAD_PROFILE,
    POSITION_EFFORT_PRELOAD_PROFILE,
    POSITION_ONLY_PROFILE,
    apply_gripper_effort_pid_overlay,
    apply_gripper_position_effort_overlay,
)


def build_proxy_robot_description(
    *,
    robot_name: str,
    prefix: str,
    use_camera: str,
    world_anchor: bool = True,
    replace_collision_meshes: bool = True,
    pad_contact_material_profile: str | None = None,
    moving_pad_distal_extension_m: float = 0.0,
    gripper_control_profile: str = POSITION_ONLY_PROFILE,
    camera_update_rate_hz: float = 5.0,
    camera_view: str = "upstream",
    camera_resolution: str = "upstream",
) -> tuple[str, CollisionProxyReport]:
    if robot_name != "so101":
        raise RuntimeError("collision proxy supports robot_name=so101 only")
    if prefix:
        raise RuntimeError("collision proxy supports prefix='' only")
    if use_camera not in ("true", "false"):
        raise RuntimeError("use_camera must be true or false")
    if use_camera == "false" and camera_view != "upstream":
        raise ValueError("camera view requires use_camera=true")

    description_share = Path(get_package_share_directory("so101_description"))
    edgegrasp_share = Path(get_package_share_directory("edgegrasp_ros"))
    xacro_path = description_share / "urdf" / "robots" / "so101.urdf.xacro"
    contract_path = edgegrasp_share / "config" / "so101_collision_proxies.json"
    material_contract_path = (
        edgegrasp_share / "config" / "so101_pad_contact_materials.json"
    )
    expanded = subprocess.run(
        [
            "xacro",
            str(xacro_path),
            "robot_name:=so101",
            "prefix:=",
            "use_gazebo:=true",
            f"use_camera:={use_camera}",
        ],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    if use_camera == "true":
        expanded = configure_sim_camera(expanded, rate_hz=camera_update_rate_hz, view=camera_view, resolution=camera_resolution)
    contract = load_collision_proxy_contract(contract_path)
    contract = with_moving_pad_distal_extension(
        contract, moving_pad_distal_extension_m
    )
    material_profile = None
    if pad_contact_material_profile is not None:
        material_contract = load_pad_contact_material_contract(
            material_contract_path
        )
        material_profile = select_pad_contact_material_profile(
            material_contract, pad_contact_material_profile
        )
    robot_description, report = apply_collision_proxies(
        expanded,
        contract,
        require_no_collision_meshes=replace_collision_meshes,
        anchor_to_world=world_anchor,
        replace_collision_meshes=replace_collision_meshes,
        pad_contact_material=material_profile,
    )
    if gripper_control_profile == POSITION_EFFORT_PRELOAD_PROFILE:
        controller_config = (
            edgegrasp_share / "config" / "ros2_controllers_position_effort.yaml"
        )
        robot_description, _ = apply_gripper_position_effort_overlay(
            robot_description,
            controller_config_path=controller_config,
        )
    elif gripper_control_profile == EFFORT_PID_PRELOAD_PROFILE:
        controller_config = (
            edgegrasp_share / "config" / "ros2_controllers_effort_pid.yaml"
        )
        robot_description, _ = apply_gripper_effort_pid_overlay(
            robot_description,
            controller_config_path=controller_config,
        )
    elif gripper_control_profile != POSITION_ONLY_PROFILE:
        raise RuntimeError(
            f"unsupported gripper_control_profile: {gripper_control_profile}"
        )
    expected_root = ("world",) if world_anchor else ("base_link",)
    expected_replaced = 13 if replace_collision_meshes else 0
    expected_remaining = 0 if replace_collision_meshes else 13
    if (
        len(report.replaced) != expected_replaced
        or len(report.validated_collision_meshes) != 13
        or len(report.contact_extensions_added) != 2
        or len(report.remaining_collision_meshes) != expected_remaining
        or report.root_links_after != expected_root
        or report.world_anchor_added is not world_anchor
        or report.contact_material_profile != pad_contact_material_profile
    ):
        raise RuntimeError("collision proxy transformation did not close safely")
    return robot_description, report
