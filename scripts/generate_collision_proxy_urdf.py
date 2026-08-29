#!/usr/bin/env python3
"""Generate an SO-101 Gazebo URDF with EdgeGrasp collision proxies."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess

from edgegrasp.collision_proxy import (
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


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = (
    PROJECT_ROOT
    / "ros_ws"
    / "src"
    / "edgegrasp_ros"
    / "config"
    / "so101_collision_proxies.json"
)
DEFAULT_MATERIAL_CONTRACT = (
    PROJECT_ROOT
    / "ros_ws"
    / "src"
    / "edgegrasp_ros"
    / "config"
    / "so101_pad_contact_materials.json"
)
DEFAULT_POSITION_EFFORT_CONTROLLERS = (
    PROJECT_ROOT
    / "ros_ws"
    / "src"
    / "edgegrasp_ros"
    / "config"
    / "ros2_controllers_position_effort.yaml"
)
DEFAULT_EFFORT_PID_CONTROLLERS = (
    PROJECT_ROOT
    / "ros_ws"
    / "src"
    / "edgegrasp_ros"
    / "config"
    / "ros2_controllers_effort_pid.yaml"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--xacro", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument(
        "--material-contract", type=Path, default=DEFAULT_MATERIAL_CONTRACT
    )
    parser.add_argument(
        "--pad-contact-material-profile", default="implicit_default"
    )
    parser.add_argument("--robot-name", default="so101")
    parser.add_argument("--use-camera", choices=("true", "false"), default="true")
    parser.add_argument("--xacro-command", default="xacro")
    parser.add_argument("--moving-pad-distal-extension-m", type=float, default=0.0)
    parser.add_argument(
        "--gripper-control-profile",
        choices=(
            POSITION_ONLY_PROFILE,
            POSITION_EFFORT_PRELOAD_PROFILE,
            EFFORT_PID_PRELOAD_PROFILE,
        ),
        default=POSITION_ONLY_PROFILE,
    )
    parser.add_argument(
        "--position-effort-controller-config",
        type=Path,
        default=DEFAULT_POSITION_EFFORT_CONTROLLERS,
    )
    parser.add_argument(
        "--effort-pid-controller-config",
        type=Path,
        default=DEFAULT_EFFORT_PID_CONTROLLERS,
    )
    parser.add_argument("--report", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    completed = subprocess.run(
        [
            args.xacro_command,
            str(args.xacro),
            f"robot_name:={args.robot_name}",
            "prefix:=",
            "use_gazebo:=true",
            f"use_camera:={args.use_camera}",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    contract = load_collision_proxy_contract(args.contract)
    contract = with_moving_pad_distal_extension(
        contract, args.moving_pad_distal_extension_m
    )
    material_contract = load_pad_contact_material_contract(args.material_contract)
    material_profile = select_pad_contact_material_profile(
        material_contract, args.pad_contact_material_profile
    )
    transformed, report = apply_collision_proxies(
        completed.stdout, contract, pad_contact_material=material_profile
    )
    control_report = None
    if args.gripper_control_profile == POSITION_EFFORT_PRELOAD_PROFILE:
        transformed, control_report = apply_gripper_position_effort_overlay(
            transformed,
            controller_config_path=args.position_effort_controller_config,
        )
    elif args.gripper_control_profile == EFFORT_PID_PRELOAD_PROFILE:
        transformed, control_report = apply_gripper_effort_pid_overlay(
            transformed,
            controller_config_path=args.effort_pid_controller_config,
        )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(transformed, encoding="utf-8")
    payload = {
        "contract": str(args.contract.resolve()),
        "input_xacro": str(args.xacro.resolve()),
        "output_urdf": str(args.output.resolve()),
        "replaced_count": len(report.replaced),
        "replaced": list(report.replaced),
        "contact_extensions_added_count": len(report.contact_extensions_added),
        "contact_extensions_added": list(report.contact_extensions_added),
        "visual_mesh_count": len(report.visual_meshes_after),
        "remaining_collision_meshes": list(report.remaining_collision_meshes),
        "world_anchor_added": report.world_anchor_added,
        "root_links_before": list(report.root_links_before),
        "root_links_after": list(report.root_links_after),
        "pad_contact_material_contract": str(args.material_contract.resolve()),
        "pad_contact_material_profile": report.contact_material_profile,
        "pad_contact_material_references": list(
            report.contact_material_references
        ),
        "moving_pad_distal_extension_m": args.moving_pad_distal_extension_m,
        "geometry_role": (
            "edgegrasp_simulation_attachment"
            if args.moving_pad_distal_extension_m != 0.0
            else "pinned_mesh_derived_proxy"
        ),
        "gripper_control_profile": args.gripper_control_profile,
        "gripper_control_overlay": (
            None
            if control_report is None
            else {
                "joint_name": control_report.joint_name,
                "command_interfaces_before": list(
                    control_report.command_interfaces_before
                ),
                "command_interfaces_after": list(
                    control_report.command_interfaces_after
                ),
                "state_interfaces": list(control_report.state_interfaces),
                "effort_limit_nm": control_report.effort_limit_nm,
                "controller_config_path": control_report.controller_config_path,
            }
        ),
    }
    if args.report is not None:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    print(json.dumps(payload, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
