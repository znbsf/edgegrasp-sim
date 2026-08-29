"""Fail-closed contact-material profiles for generated Gazebo finger pads.

The profiles in this module are simulation sensitivity parameters.  They are
not measurements of the real SO-101 gripper or target material.  A selected
profile is translated into URDF Gazebo extension tags and must still be
confirmed in the converted SDF before a runtime observation is accepted.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Any
import xml.etree.ElementTree as ET


PAD_CONTACT_MATERIAL_SCHEMA_VERSION = 1
IMPLICIT_PAD_CONTACT_MATERIAL_PROFILE = "implicit_default"

_EXPECTED_TARGETS = (
    (
        "edgegrasp_fixed_finger_pad_link",
        "edgegrasp_grasp_proxy_fixed_finger_pad",
        "edgegrasp_fixed_finger_pad_link_collision",
    ),
    (
        "edgegrasp_moving_finger_pad_link",
        "edgegrasp_grasp_proxy_moving_finger_pad",
        "edgegrasp_moving_finger_pad_link_collision",
    ),
)


class ContactMaterialError(ValueError):
    """Raised when a contact-material contract or selection is unsafe."""


@dataclass(frozen=True, slots=True)
class PadContactMaterialProfile:
    """One immutable, isotropic friction profile for the two generated pads."""

    name: str
    apply_explicit_friction: bool
    coefficient: float | None
    target_links: tuple[str, ...]
    target_collisions: tuple[str, ...]
    urdf_collision_names: tuple[str | None, ...]

    @property
    def mu1(self) -> float | None:
        return self.coefficient

    @property
    def mu2(self) -> float | None:
        return self.coefficient


def _finite_number(value: object, *, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ContactMaterialError(f"{label} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ContactMaterialError(f"{label} must be finite")
    return result


def load_pad_contact_material_contract(path) -> dict[str, Any]:
    """Load and validate the machine-readable Candidate012 material contract."""

    contract = json.loads(path.read_text(encoding="utf-8"))
    if contract.get("schema_version") != PAD_CONTACT_MATERIAL_SCHEMA_VERSION:
        raise ContactMaterialError("unsupported pad contact-material schema")
    if contract.get("scope") != "generated_edgegrasp_finger_pad_collisions_only":
        raise ContactMaterialError("invalid pad contact-material scope")
    if contract.get("model") != "isotropic_coulomb_friction":
        raise ContactMaterialError("unsupported pad contact-material model")

    targets = contract.get("targets")
    if not isinstance(targets, list):
        raise ContactMaterialError("pad contact-material targets must be a list")
    observed_targets: list[tuple[str, str, str]] = []
    for target in targets:
        if not isinstance(target, dict):
            raise ContactMaterialError("pad contact-material target must be an object")
        observed_targets.append(
            (
                target.get("extension_link"),
                target.get("legacy_urdf_collision_name"),
                target.get("explicit_converted_collision_token"),
            )
        )
    if tuple(observed_targets) != _EXPECTED_TARGETS:
        raise ContactMaterialError("pad contact-material target drift")

    bounds = contract.get("coefficient_bounds")
    if not isinstance(bounds, dict):
        raise ContactMaterialError("missing contact-material coefficient bounds")
    minimum = _finite_number(bounds.get("minimum"), label="minimum coefficient")
    maximum = _finite_number(bounds.get("maximum"), label="maximum coefficient")
    if minimum <= 0.0 or maximum < minimum:
        raise ContactMaterialError("invalid contact-material coefficient bounds")

    profiles = contract.get("profiles")
    if not isinstance(profiles, dict) or not profiles:
        raise ContactMaterialError("contact-material profiles must be an object")
    for name, profile in profiles.items():
        if not isinstance(name, str) or not name or not isinstance(profile, dict):
            raise ContactMaterialError("invalid contact-material profile entry")
        explicit = profile.get("apply_explicit_friction")
        if not isinstance(explicit, bool):
            raise ContactMaterialError(
                f"profile {name} apply_explicit_friction must be boolean"
            )
        coefficient = profile.get("coefficient")
        if explicit:
            value = _finite_number(
                coefficient, label=f"profile {name} coefficient"
            )
            if value < minimum or value > maximum:
                raise ContactMaterialError(
                    f"profile {name} coefficient outside configured bounds"
                )
        elif coefficient is not None:
            raise ContactMaterialError(
                f"implicit profile {name} must not assign a coefficient"
            )

    implicit = profiles.get(IMPLICIT_PAD_CONTACT_MATERIAL_PROFILE)
    if not isinstance(implicit, dict) or implicit.get(
        "apply_explicit_friction"
    ) is not False:
        raise ContactMaterialError("missing fail-safe implicit default profile")
    return contract


def select_pad_contact_material_profile(
    contract: dict[str, Any], name: str
) -> PadContactMaterialProfile:
    """Resolve one validated profile; unknown names fail closed."""

    profiles = contract.get("profiles", {})
    profile = profiles.get(name)
    if not isinstance(profile, dict):
        raise ContactMaterialError(f"unknown pad contact-material profile: {name}")
    explicit = profile["apply_explicit_friction"]
    coefficient = float(profile["coefficient"]) if explicit else None
    return PadContactMaterialProfile(
        name=name,
        apply_explicit_friction=explicit,
        coefficient=coefficient,
        target_links=tuple(item[0] for item in _EXPECTED_TARGETS),
        target_collisions=tuple(
            item[2] if explicit else item[1] for item in _EXPECTED_TARGETS
        ),
        urdf_collision_names=tuple(
            None if explicit else item[1] for item in _EXPECTED_TARGETS
        ),
    )


def validate_converted_pad_contact_material(
    sdf_text: str, profile: PadContactMaterialProfile
) -> dict[str, Any]:
    """Validate the actual URDF-to-SDF mapping for one generated robot.

    Fixed-joint lumping decorates collision names, so target matching permits
    a converter prefix/suffix while still requiring each contracted collision
    name exactly once.  Any friction surface outside the two targets is a
    fail-closed scope leak for this controlled experiment.
    """

    try:
        root = ET.fromstring(sdf_text)
    except ET.ParseError as exc:
        raise ContactMaterialError(f"invalid converted SDF XML: {exc}") from exc
    if root.tag != "sdf":
        raise ContactMaterialError(f"expected SDF root, got {root.tag!r}")

    collision_links: dict[ET.Element, str] = {}
    for link in root.findall(".//link"):
        for collision in link.findall("collision"):
            collision_links[collision] = link.get("name", "")
    collisions = list(collision_links)
    target_matches: dict[str, ET.Element] = {}
    for target_name in profile.target_collisions:
        matches = [
            collision
            for collision in collisions
            if target_name in collision.get("name", "")
        ]
        if len(matches) != 1:
            raise ContactMaterialError(
                f"converted SDF target {target_name} matched {len(matches)} collisions"
            )
        target_matches[target_name] = matches[0]

    explicit_surfaces: list[str] = []
    target_values: dict[str, dict[str, float | str | None]] = {}
    target_elements = set(target_matches.values())
    for collision in collisions:
        friction = collision.find("surface/friction")
        mu_element = collision.find("surface/friction/ode/mu")
        mu2_element = collision.find("surface/friction/ode/mu2")
        if friction is not None:
            if collision not in target_elements:
                raise ContactMaterialError(
                    "contact-material friction leaked to an unintended collision: "
                    + collision.get("name", "")
                )
            if [child.tag for child in friction] != ["ode"]:
                raise ContactMaterialError(
                    "converted SDF contains an unexpected friction branch"
                )
            ode = friction.find("ode")
            assert ode is not None
            if [child.tag for child in ode] != ["mu", "mu2"]:
                raise ContactMaterialError(
                    "converted SDF contains unexpected ODE friction fields"
                )
        if (mu_element is None) != (mu2_element is None):
            raise ContactMaterialError(
                "converted SDF contains a partial mu/mu2 friction mapping"
            )
        if mu_element is None:
            continue
        collision_name = collision.get("name", "")
        explicit_surfaces.append(collision_name)
        try:
            mu = float(mu_element.text or "")
            mu2 = float(mu2_element.text or "")
        except ValueError as exc:
            raise ContactMaterialError(
                f"{collision_name} friction values must be numeric"
            ) from exc
        if not math.isfinite(mu) or not math.isfinite(mu2):
            raise ContactMaterialError(
                f"{collision_name} friction values must be finite"
            )
        target_values[collision_name] = {"mu": mu, "mu2": mu2}

    if profile.apply_explicit_friction:
        if len(explicit_surfaces) != len(profile.target_collisions):
            raise ContactMaterialError(
                "explicit profile did not map to both pad collision surfaces"
            )
        assert profile.coefficient is not None
        for collision_name, values in target_values.items():
            if not math.isclose(
                float(values["mu"]), profile.coefficient, rel_tol=0.0, abs_tol=1e-12
            ) or not math.isclose(
                float(values["mu2"]),
                profile.coefficient,
                rel_tol=0.0,
                abs_tol=1e-12,
            ):
                raise ContactMaterialError(
                    "converted friction coefficient drift: " + collision_name
                )
    elif explicit_surfaces:
        raise ContactMaterialError(
            "implicit profile unexpectedly produced explicit friction surfaces"
        )

    mesh_collisions = [
        collision.get("name", "")
        for collision in collisions
        if collision.find("geometry/mesh") is not None
    ]
    if mesh_collisions:
        raise ContactMaterialError(
            "converted Gazebo proxy still contains mesh collisions: "
            + ",".join(mesh_collisions)
        )
    world_joint = root.find(".//joint[@name='world_joint']")
    if (
        world_joint is None
        or world_joint.findtext("parent") != "world"
        or world_joint.findtext("child") != "base_link"
    ):
        raise ContactMaterialError("converted Gazebo proxy lost the world anchor")

    return {
        "status": "PASS",
        "evidence_level": "URDF_TO_SDF_STATIC_MAPPING_ONLY",
        "profile": profile.name,
        "apply_explicit_friction": profile.apply_explicit_friction,
        "configured_coefficient": profile.coefficient,
        "collision_count": len(collisions),
        "target_collision_names": {
            target_name: target_matches[target_name].get("name", "")
            for target_name in profile.target_collisions
        },
        "target_collision_parent_links": {
            target_name: collision_links[target_matches[target_name]]
            for target_name in profile.target_collisions
        },
        "explicit_friction_collision_names": explicit_surfaces,
        "target_values": target_values,
        "remaining_mesh_collision_count": 0,
        "world_anchor_present": True,
        "claim_boundary": (
            "Static SDF surface mapping does not prove DART friction behavior, "
            "contact retention, or grasp success."
        ),
    }
