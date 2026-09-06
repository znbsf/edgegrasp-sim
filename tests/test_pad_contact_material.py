from __future__ import annotations

import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from edgegrasp.contact_material import (
    ContactMaterialError,
    load_pad_contact_material_contract,
    select_pad_contact_material_profile,
    validate_converted_pad_contact_material,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONTRACT = (
    PROJECT_ROOT
    / "ros_ws"
    / "src"
    / "edgegrasp_ros"
    / "config"
    / "so101_pad_contact_materials.json"
)


def _write_contract(tmp_path: Path, payload: dict) -> Path:
    path = tmp_path / "materials.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def _converted_sdf(
    *,
    target_collision_tokens: tuple[str, ...],
    coefficient: float | None,
    leak: bool = False,
) -> str:
    root = ET.Element("sdf", {"version": "1.11"})
    model = ET.SubElement(root, "model", {"name": "so101"})
    joint = ET.SubElement(model, "joint", {"name": "world_joint", "type": "fixed"})
    ET.SubElement(joint, "parent").text = "world"
    ET.SubElement(joint, "child").text = "base_link"
    link = ET.SubElement(model, "link", {"name": "gripper_link"})
    collision_names = (
        f"gripper_link_fixed_joint_lump__{target_collision_tokens[0]}_2",
        f"moving_jaw_fixed_joint_lump__{target_collision_tokens[1]}_1",
        "edgegrasp_proxy_unrelated_collision",
    )
    for index, name in enumerate(collision_names):
        collision = ET.SubElement(link, "collision", {"name": name})
        geometry = ET.SubElement(collision, "geometry")
        ET.SubElement(geometry, "box")
        should_have_friction = coefficient is not None and (index < 2 or leak)
        if should_have_friction:
            surface = ET.SubElement(collision, "surface")
            friction = ET.SubElement(surface, "friction")
            ode = ET.SubElement(friction, "ode")
            ET.SubElement(ode, "mu").text = str(coefficient)
            ET.SubElement(ode, "mu2").text = str(coefficient)
    return ET.tostring(root, encoding="unicode")


def test_material_contract_has_bounded_single_variable_profiles() -> None:
    contract = load_pad_contact_material_contract(CONTRACT)
    implicit = select_pad_contact_material_profile(contract, "implicit_default")
    control = select_pad_contact_material_profile(
        contract, "candidate012_control_mu1p0"
    )
    treatment = select_pad_contact_material_profile(
        contract, "candidate012_treatment_mu1p5"
    )
    assert implicit.apply_explicit_friction is False
    assert implicit.coefficient is None
    assert control.mu1 == control.mu2 == 1.0
    assert treatment.mu1 == treatment.mu2 == 1.5
    assert control.target_links == treatment.target_links
    assert control.target_collisions == treatment.target_collisions


@pytest.mark.parametrize("coefficient", [float("nan"), float("inf"), 0.49, 1.51])
def test_material_contract_rejects_nonfinite_or_out_of_bound_coefficients(
    tmp_path: Path, coefficient: float
) -> None:
    payload = copy.deepcopy(json.loads(CONTRACT.read_text(encoding="utf-8")))
    payload["profiles"]["candidate012_treatment_mu1p5"][
        "coefficient"
    ] = coefficient
    with pytest.raises(ContactMaterialError):
        load_pad_contact_material_contract(_write_contract(tmp_path, payload))


def test_material_contract_rejects_unknown_profile() -> None:
    contract = load_pad_contact_material_contract(CONTRACT)
    with pytest.raises(ContactMaterialError, match="unknown"):
        select_pad_contact_material_profile(contract, "candidate012_unbounded")


@pytest.mark.parametrize(
    ("profile_name", "coefficient", "expected_explicit_count"),
    [
        ("implicit_default", None, 0),
        ("candidate012_control_mu1p0", 1.0, 2),
        ("candidate012_treatment_mu1p5", 1.5, 2),
    ],
)
def test_converted_sdf_mapping_is_exact_and_fixed_joint_lump_tolerant(
    profile_name: str, coefficient: float | None, expected_explicit_count: int
) -> None:
    contract = load_pad_contact_material_contract(CONTRACT)
    profile = select_pad_contact_material_profile(contract, profile_name)
    report = validate_converted_pad_contact_material(
        _converted_sdf(
            target_collision_tokens=profile.target_collisions,
            coefficient=coefficient,
        ),
        profile,
    )
    assert report["status"] == "PASS"
    assert report["evidence_level"] == "URDF_TO_SDF_STATIC_MAPPING_ONLY"
    assert len(report["explicit_friction_collision_names"]) == expected_explicit_count
    assert report["remaining_mesh_collision_count"] == 0
    assert report["world_anchor_present"] is True


def test_converted_sdf_rejects_friction_scope_leak() -> None:
    contract = load_pad_contact_material_contract(CONTRACT)
    profile = select_pad_contact_material_profile(
        contract, "candidate012_treatment_mu1p5"
    )
    with pytest.raises(ContactMaterialError, match="leaked"):
        validate_converted_pad_contact_material(
            _converted_sdf(
                target_collision_tokens=profile.target_collisions,
                coefficient=1.5,
                leak=True,
            ),
            profile,
        )


def test_candidate_harness_records_and_preflights_material_profile() -> None:
    harness = (PROJECT_ROOT / "scripts" / "run_candidate005_contact_quality.sh").read_text(
        encoding="utf-8"
    )
    assert "PAD_CONTACT_MATERIAL_PROFILE" in harness
    assert "pad_contact_material_profile:=" in harness
    assert "generate_collision_proxy_urdf.py" in harness
    assert "gz sdf -p" in harness
    assert "validate_pad_contact_material_sdf.py" in harness
    assert "pad_contact_material_mapping.json" in harness

@pytest.mark.parametrize("branch", ["<ode />", "<ode><mu>1</mu></ode>", '<ode mu="1" />'])
def test_converter_placeholder_does_not_hide_real_friction_settings(branch):
    profile = select_pad_contact_material_profile(
        load_pad_contact_material_contract(CONTRACT), "candidate012_control_mu1p0")
    root = ET.fromstring(_converted_sdf(
        target_collision_tokens=profile.target_collisions, coefficient=1.0))
    collision = root.findall(".//collision")[-1]
    surface = ET.SubElement(collision, "surface")
    surface.append(ET.fromstring("<friction>" + branch + "</friction>"))
    text = ET.tostring(root, encoding="unicode")
    if branch == "<ode />":
        report = validate_converted_pad_contact_material(text, profile)
        assert len(report["explicit_friction_collision_names"]) == 2
    else:
        with pytest.raises(ContactMaterialError, match="leaked"):
            validate_converted_pad_contact_material(text, profile)
