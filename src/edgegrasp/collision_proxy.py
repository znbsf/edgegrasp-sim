"""Strict SO-101 collision-mesh to primitive-proxy transformation.

The transformer operates on an already-expanded URDF.  It deliberately
touches only collision geometries listed in the pinned, machine-readable
contract.  Visual geometry is compared before and after the transformation so
that a malformed contract cannot silently remove the display meshes.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import json
import math
from pathlib import PurePosixPath
from typing import Any
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET

from .contact_material import PadContactMaterialProfile


PINNED_SO101_COMMIT = "0305e03ab54e64aae9263fcbf339622e654012f3"


class CollisionProxyError(ValueError):
    """Raised when the input URDF and the pinned proxy contract disagree."""


@dataclass(frozen=True, slots=True)
class CollisionProxyReport:
    replaced: tuple[str, ...]
    validated_collision_meshes: tuple[str, ...]
    contact_extensions_added: tuple[str, ...]
    visual_meshes_before: tuple[str, ...]
    visual_meshes_after: tuple[str, ...]
    remaining_collision_meshes: tuple[str, ...]
    world_anchor_added: bool
    root_links_before: tuple[str, ...]
    root_links_after: tuple[str, ...]
    contact_material_profile: str | None
    contact_material_references: tuple[str, ...]


def load_collision_proxy_contract(path) -> dict[str, Any]:
    """Load and fail closed on a malformed or drifting proxy contract."""

    contract = json.loads(path.read_text(encoding="utf-8"))
    if contract.get("schema_version") != 1:
        raise CollisionProxyError("unsupported collision proxy schema")
    upstream = contract.get("upstream", {})
    if upstream.get("commit") != PINNED_SO101_COMMIT:
        raise CollisionProxyError("collision proxy upstream pin drift")
    gripper_kinematics = contract.get("gripper_contact_kinematics")
    if not isinstance(gripper_kinematics, dict):
        raise CollisionProxyError("missing gripper contact kinematics")
    if (
        gripper_kinematics.get("reference_frame") != "gripper_frame_link"
        or gripper_kinematics.get("common_parent_link") != "gripper_link"
    ):
        raise CollisionProxyError("invalid gripper contact kinematic frames")
    for label, expected in (
        ("reference_joint", ("gripper_frame_joint", "fixed")),
        ("moving_joint", ("gripper", "revolute")),
    ):
        joint = gripper_kinematics.get(label)
        if not isinstance(joint, dict) or (
            joint.get("name"), joint.get("type")
        ) != expected:
            raise CollisionProxyError(f"invalid {label} identity")
        for field in ("origin_xyz", "origin_rpy"):
            values = joint.get(field)
            if (
                not isinstance(values, list)
                or len(values) != 3
                or any(
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(float(value))
                    for value in values
                )
            ):
                raise CollisionProxyError(f"invalid {label} {field}")
    moving_joint = gripper_kinematics["moving_joint"]
    if moving_joint.get("axis_xyz") != [0.0, 0.0, 1.0]:
        raise CollisionProxyError("unsupported gripper joint axis")
    if not (
        isinstance(moving_joint.get("lower_rad"), (int, float))
        and isinstance(moving_joint.get("upper_rad"), (int, float))
        and float(moving_joint["lower_rad"]) < float(moving_joint["upper_rad"])
    ):
        raise CollisionProxyError("invalid gripper joint limits")
    proxies = contract.get("proxies")
    if not isinstance(proxies, list) or len(proxies) != contract.get(
        "expected_proxy_count"
    ):
        raise CollisionProxyError("collision proxy count mismatch")
    anchor = contract.get("gazebo_world_anchor")
    if not isinstance(anchor, dict):
        raise CollisionProxyError("missing Gazebo world anchor contract")
    expected_anchor = {
        "world_link": "world",
        "joint_name": "world_joint",
        "joint_type": "fixed",
        "parent_link": "world",
        "child_link": "base_link",
    }
    for field, expected in expected_anchor.items():
        if anchor.get(field) != expected:
            raise CollisionProxyError(f"invalid Gazebo world anchor {field}")
    for field in ("origin_xyz", "origin_rpy"):
        values = anchor.get(field)
        if (
            not isinstance(values, list)
            or len(values) != 3
            or not all(isinstance(value, (int, float)) for value in values)
        ):
            raise CollisionProxyError(f"invalid Gazebo world anchor {field}")
    identities: set[tuple[str, int]] = set()
    for proxy in proxies:
        identity = (proxy.get("link"), proxy.get("collision_index"))
        if (
            not isinstance(identity[0], str)
            or not identity[0]
            or isinstance(identity[1], bool)
            or not isinstance(identity[1], int)
            or identity[1] < 0
            or identity in identities
        ):
            raise CollisionProxyError(f"invalid proxy identity: {identity!r}")
        identities.add(identity)
        if proxy.get("shape") != "box" or proxy.get("conservative") is not True:
            raise CollisionProxyError(f"unsupported proxy shape: {identity!r}")
        for field in ("proxy_pose_xyz", "proxy_pose_rpy", "proxy_size_m"):
            values = proxy.get(field)
            if not isinstance(values, list) or len(values) != 3:
                raise CollisionProxyError(f"invalid {field}: {identity!r}")
            if not all(isinstance(value, (int, float)) for value in values):
                raise CollisionProxyError(f"non-numeric {field}: {identity!r}")
        if not all(float(value) > 0.0 for value in proxy["proxy_size_m"]):
            raise CollisionProxyError(f"non-positive proxy size: {identity!r}")

    contact_proxies = contract.get("gazebo_gripper_contact_extensions")
    if (
        not isinstance(contact_proxies, list)
        or len(contact_proxies) != contract.get("expected_contact_extension_count")
    ):
        raise CollisionProxyError("gripper contact extension count mismatch")
    contact_identities: set[tuple[str, int]] = set()
    extension_links: set[str] = set()
    extension_joints: set[str] = set()
    for proxy in contact_proxies:
        identity = (proxy.get("link"), proxy.get("reference_collision_index"))
        if (
            not isinstance(identity[0], str)
            or not identity[0]
            or isinstance(identity[1], bool)
            or not isinstance(identity[1], int)
            or identity[1] < 0
            or identity in contact_identities
        ):
            raise CollisionProxyError(
                f"invalid gripper contact extension identity: {identity!r}"
            )
        contact_identities.add(identity)
        extension_link = proxy.get("extension_link")
        extension_joint = proxy.get("extension_joint")
        if (
            not isinstance(extension_link, str)
            or not extension_link
            or extension_link in extension_links
        ):
            raise CollisionProxyError(
                f"invalid gripper contact extension link: {extension_link!r}"
            )
        if (
            not isinstance(extension_joint, str)
            or not extension_joint
            or extension_joint in extension_joints
        ):
            raise CollisionProxyError(
                f"invalid gripper contact extension joint: {extension_joint!r}"
            )
        extension_links.add(extension_link)
        extension_joints.add(extension_joint)
        if proxy.get("shape") != "box" or proxy.get("conservative") is not True:
            raise CollisionProxyError(
                f"unsupported gripper contact extension shape: {identity!r}"
            )
        if not isinstance(proxy.get("source_mesh"), str) or not proxy["source_mesh"]:
            raise CollisionProxyError(
                f"missing gripper contact source mesh: {identity!r}"
            )
        for field in (
            "source_pose_xyz",
            "source_pose_rpy",
            "expected_upstream_box_size_m",
            "proxy_pose_xyz",
            "proxy_pose_rpy",
            "proxy_size_m",
        ):
            values = proxy.get(field)
            if not isinstance(values, list) or len(values) != 3:
                raise CollisionProxyError(f"invalid {field}: {identity!r}")
            if not all(isinstance(value, (int, float)) for value in values):
                raise CollisionProxyError(f"non-numeric {field}: {identity!r}")
        for field in ("expected_upstream_box_size_m", "proxy_size_m"):
            if not all(float(value) > 0.0 for value in proxy[field]):
                raise CollisionProxyError(f"non-positive {field}: {identity!r}")
        region = proxy.get("mesh_region")
        if not isinstance(region, dict) or region.get("axis") not in (0, 1, 2):
            raise CollisionProxyError(f"invalid mesh_region: {identity!r}")
        bounds = [region.get("min_m"), region.get("max_m")]
        if all(value is None for value in bounds) or any(
            value is not None and not isinstance(value, (int, float))
            for value in bounds
        ):
            raise CollisionProxyError(f"invalid mesh_region bounds: {identity!r}")
        if (
            bounds[0] is not None
            and bounds[1] is not None
            and float(bounds[0]) >= float(bounds[1])
        ):
            raise CollisionProxyError(f"empty mesh_region: {identity!r}")
    return contract


def with_moving_pad_distal_extension(
    contract: dict[str, Any], extension_m: float
) -> dict[str, Any]:
    """Return one bounded moving-pad attachment variant.

    The pinned-mesh-derived proximal edge is held fixed while the box extends
    along negative moving-jaw local Y.  This is an EdgeGrasp simulation
    attachment, not a claim about the upstream mesh or real SO-101 hardware.
    """

    extension = float(extension_m)
    if not math.isfinite(extension) or not 0.0 <= extension <= 0.015:
        raise CollisionProxyError(
            "moving pad distal extension must be finite and in [0, 0.015] m"
        )
    result = deepcopy(contract)
    if extension == 0.0:
        return result

    matches = [
        item
        for item in result["gazebo_gripper_contact_extensions"]
        if item.get("name") == "moving_finger_pad"
    ]
    if len(matches) != 1:
        raise CollisionProxyError("moving finger pad contract is not unique")
    moving = matches[0]
    region = moving.get("mesh_region", {})
    if (
        region.get("axis") != 1
        or region.get("min_m") is not None
        or not isinstance(region.get("max_m"), (int, float))
    ):
        raise CollisionProxyError("moving pad distal direction contract drift")

    baseline_center = float(moving["proxy_pose_xyz"][1])
    baseline_size = float(moving["proxy_size_m"][1])
    proximal_edge = baseline_center + baseline_size / 2.0
    generated_size = baseline_size + extension
    generated_center = proximal_edge - generated_size / 2.0
    moving["proxy_pose_xyz"][1] = round(generated_center, 15)
    moving["proxy_size_m"][1] = round(generated_size, 15)
    moving["experimental_attachment"] = {
        "kind": "moving_pad_distal_extension",
        "extension_m": extension,
        "baseline_local_y_size_m": baseline_size,
        "generated_local_y_size_m": generated_size,
        "preserved_proximal_local_y_m": proximal_edge,
        "claim_boundary": (
            "EdgeGrasp simulation attachment; not pinned-mesh or hardware geometry."
        ),
    }
    moving["source"] = (
        f"{moving['source']} EdgeGrasp experiment extends only the distal local-Y "
        f"contact length by {extension:.6f} m while preserving the proximal edge."
    )
    return result


def _mesh_basename(uri: str) -> str:
    parsed = urlparse(uri)
    path = parsed.path if parsed.scheme else uri
    return PurePosixPath(unquote(path.replace("\\", "/"))).name


def _format_vector(values: list[float]) -> str:
    return " ".join(format(float(value), ".12g") for value in values)


def _parse_vector(value: str | None, *, label: str) -> tuple[float, float, float]:
    try:
        parsed = tuple(float(item) for item in (value or "").split())
    except ValueError as exc:
        raise CollisionProxyError(f"invalid vector for {label}") from exc
    if len(parsed) != 3 or not all(math.isfinite(item) for item in parsed):
        raise CollisionProxyError(f"invalid vector for {label}")
    return parsed


def _require_vector_match(
    actual: tuple[float, float, float],
    expected: list[float],
    *,
    label: str,
) -> None:
    if any(
        not math.isclose(value, float(reference), rel_tol=0.0, abs_tol=1e-9)
        for value, reference in zip(actual, expected, strict=True)
    ):
        raise CollisionProxyError(
            f"{label} drift: expected={tuple(expected)},actual={actual}"
        )


def _visual_meshes(root: ET.Element) -> tuple[str, ...]:
    return tuple(
        mesh.get("filename", "")
        for mesh in root.findall("./link/visual/geometry/mesh")
    )


def _collision_meshes(root: ET.Element) -> tuple[str, ...]:
    return tuple(
        mesh.get("filename", "")
        for mesh in root.findall("./link/collision/geometry/mesh")
    )


def _root_links(root: ET.Element) -> tuple[str, ...]:
    link_names = [link.get("name", "") for link in root.findall("link")]
    child_names = {
        child.get("link", "") for child in root.findall("./joint/child")
    }
    return tuple(name for name in link_names if name not in child_names)


def _ensure_gazebo_world_anchor(
    root: ET.Element, anchor: dict[str, Any]
) -> tuple[bool, tuple[str, ...], tuple[str, ...]]:
    """Inject and validate the fixed world->base_link anchor.

    The pinned upstream Gazebo expansion omits this joint even though the
    model is expected to be fixed.  A free base is a fail-closed condition for
    collision/contact evidence because the complete arm can fall or drift.
    """

    world_name = anchor["world_link"]
    joint_name = anchor["joint_name"]
    child_name = anchor["child_link"]
    roots_before = _root_links(root)
    if root.find(f"./link[@name='{child_name}']") is None:
        raise CollisionProxyError(f"missing anchor child link: {child_name}")

    world_link = root.find(f"./link[@name='{world_name}']")
    world_joint = root.find(f"./joint[@name='{joint_name}']")
    if (world_link is None) != (world_joint is None):
        raise CollisionProxyError("partial Gazebo world anchor present")

    added = world_link is None
    if added:
        if roots_before != (child_name,):
            raise CollisionProxyError(
                "expected a single unanchored base root, got: "
                + ",".join(roots_before)
            )
        root.insert(0, ET.Element("link", {"name": world_name}))
        world_joint = ET.Element(
            "joint", {"name": joint_name, "type": anchor["joint_type"]}
        )
        ET.SubElement(world_joint, "parent", {"link": anchor["parent_link"]})
        ET.SubElement(world_joint, "child", {"link": child_name})
        ET.SubElement(
            world_joint,
            "origin",
            {
                "xyz": _format_vector(anchor["origin_xyz"]),
                "rpy": _format_vector(anchor["origin_rpy"]),
            },
        )
        root.append(world_joint)
    else:
        assert world_joint is not None
        parent = world_joint.find("parent")
        child = world_joint.find("child")
        origin = world_joint.find("origin")
        if (
            world_joint.get("type") != anchor["joint_type"]
            or parent is None
            or parent.get("link") != anchor["parent_link"]
            or child is None
            or child.get("link") != child_name
            or origin is None
            or origin.get("xyz") != _format_vector(anchor["origin_xyz"])
            or origin.get("rpy") != _format_vector(anchor["origin_rpy"])
        ):
            raise CollisionProxyError("Gazebo world anchor drift")

    roots_after = _root_links(root)
    if roots_after != (world_name,):
        raise CollisionProxyError(
            "generated model is not world anchored: " + ",".join(roots_after)
        )
    return added, roots_before, roots_after


def apply_collision_proxies(
    robot_description: str,
    contract: dict[str, Any],
    *,
    require_no_collision_meshes: bool = True,
    anchor_to_world: bool = True,
    replace_collision_meshes: bool = True,
    pad_contact_material: PadContactMaterialProfile | None = None,
) -> tuple[str, CollisionProxyReport]:
    """Apply contracted arm-mesh and gripper-contact primitive replacements.

    Gazebo needs an explicit fixed world anchor because the pinned
    ``use_gazebo:=true`` expansion otherwise leaves ``base_link`` free.  MoveIt
    instead owns the fixed ``world`` to ``base_link`` relationship through its
    SRDF virtual joint, so its robot model must retain ``base_link`` as the URDF
    root.  ``anchor_to_world=False`` is therefore a deliberate, fail-closed
    MoveIt mode rather than a request to remove an existing anchor.

    DART cannot rely on the pinned arm collision meshes, so Gazebo replaces
    them. MoveIt can consume those meshes and should retain their concavities;
    its overlay validates every pinned source and appends the same
    distal-finger contact boxes without replacing the arm meshes. This avoids
    turning a full-mesh AABB into a permanent false self-collision.
    """

    try:
        root = ET.fromstring(robot_description)
    except ET.ParseError as exc:
        raise CollisionProxyError(f"invalid URDF XML: {exc}") from exc
    if root.tag != "robot":
        raise CollisionProxyError(f"expected robot root, got {root.tag!r}")
    if pad_contact_material is not None and (
        not anchor_to_world or not replace_collision_meshes
    ):
        raise CollisionProxyError(
            "pad contact material is allowed only on the generated Gazebo proxy"
        )
    if not anchor_to_world:
        anchor = contract["gazebo_world_anchor"]
        if (
            root.find(f"./link[@name='{anchor['world_link']}']") is not None
            or root.find(f"./joint[@name='{anchor['joint_name']}']") is not None
        ):
            raise CollisionProxyError(
                "MoveIt proxy input must not contain a Gazebo world anchor"
            )

    visuals_before = _visual_meshes(root)
    replaced: list[str] = []
    validated_collision_meshes: list[str] = []
    for proxy in contract["proxies"]:
        link_name = proxy["link"]
        index = int(proxy["collision_index"])
        link = root.find(f"./link[@name='{link_name}']")
        if link is None:
            raise CollisionProxyError(f"missing contracted link: {link_name}")
        collisions = link.findall("collision")
        if index >= len(collisions):
            raise CollisionProxyError(
                f"missing collision {link_name}[{index}], count={len(collisions)}"
            )
        collision = collisions[index]
        geometry = collision.find("geometry")
        mesh = geometry.find("mesh") if geometry is not None else None
        if geometry is None or mesh is None:
            raise CollisionProxyError(
                f"contracted collision is not a mesh: {link_name}[{index}]"
            )
        actual_mesh = _mesh_basename(mesh.get("filename", ""))
        if actual_mesh != proxy["source_mesh"]:
            raise CollisionProxyError(
                f"mesh drift for {link_name}[{index}]: "
                f"expected={proxy['source_mesh']},actual={actual_mesh}"
            )

        validated_collision_meshes.append(
            f"{link_name}[{index}]:{actual_mesh}"
        )
        if not replace_collision_meshes:
            continue

        origin = collision.find("origin")
        if origin is None:
            origin = ET.Element("origin")
            collision.insert(0, origin)
        origin.set("xyz", _format_vector(proxy["proxy_pose_xyz"]))
        origin.set("rpy", _format_vector(proxy["proxy_pose_rpy"]))
        for child in list(geometry):
            geometry.remove(child)
        box = ET.SubElement(geometry, "box")
        box.set("size", _format_vector(proxy["proxy_size_m"]))
        collision.set("name", f"edgegrasp_proxy_{proxy['offending_log_name']}")
        replaced.append(f"{link_name}[{index}]:{actual_mesh}")

    contact_extensions_added: list[str] = []
    contact_material_references: list[str] = []
    for proxy in contract["gazebo_gripper_contact_extensions"]:
        link_name = proxy["link"]
        index = int(proxy["reference_collision_index"])
        link = root.find(f"./link[@name='{link_name}']")
        if link is None:
            raise CollisionProxyError(
                f"missing gripper contact proxy link: {link_name}"
            )
        collisions = link.findall("collision")
        if index >= len(collisions):
            raise CollisionProxyError(
                f"missing gripper collision {link_name}[{index}], "
                f"count={len(collisions)}"
            )
        collision = collisions[index]
        geometry = collision.find("geometry")
        box = geometry.find("box") if geometry is not None else None
        origin = collision.find("origin")
        if geometry is None or box is None or origin is None:
            raise CollisionProxyError(
                f"contracted gripper collision is not an originated box: "
                f"{link_name}[{index}]"
            )
        if len(list(geometry)) != 1:
            raise CollisionProxyError(
                f"ambiguous gripper collision geometry: {link_name}[{index}]"
            )
        _require_vector_match(
            _parse_vector(origin.get("xyz"), label=f"{link_name}[{index}] xyz"),
            proxy["source_pose_xyz"],
            label=f"gripper origin xyz {link_name}[{index}]",
        )
        _require_vector_match(
            _parse_vector(origin.get("rpy"), label=f"{link_name}[{index}] rpy"),
            proxy["source_pose_rpy"],
            label=f"gripper origin rpy {link_name}[{index}]",
        )
        _require_vector_match(
            _parse_vector(box.get("size"), label=f"{link_name}[{index}] size"),
            proxy["expected_upstream_box_size_m"],
            label=f"gripper primitive size {link_name}[{index}]",
        )
        extension_link_name = proxy["extension_link"]
        extension_joint_name = proxy["extension_joint"]
        if (
            root.find(f"./link[@name='{extension_link_name}']") is not None
            or root.find(f"./joint[@name='{extension_joint_name}']") is not None
        ):
            raise CollisionProxyError(
                f"gripper contact extension already exists: {extension_link_name}"
            )
        extension_link = ET.SubElement(root, "link", {"name": extension_link_name})
        legacy_collision_name = f"edgegrasp_grasp_proxy_{proxy['name']}"
        extension_attributes: dict[str, str] = {"name": legacy_collision_name}
        if (
            pad_contact_material is not None
            and pad_contact_material.apply_explicit_friction
        ):
            # libsdformat currently drops link-level mu1/mu2 extensions on a
            # named URDF collision.  Explicit Candidate012 profiles therefore
            # use the converter-generated name, while the implicit profile
            # preserves the exact pre-Candidate012 URDF.
            extension_attributes = {}
        extension = ET.SubElement(
            extension_link,
            "collision",
            extension_attributes,
        )
        generated_collision_name = extension.get("name")
        if pad_contact_material is not None:
            try:
                target_index = pad_contact_material.target_links.index(
                    extension_link_name
                )
            except ValueError as exc:
                raise CollisionProxyError(
                    "contact-material target does not match generated pad link: "
                    + extension_link_name
                ) from exc
            if pad_contact_material.urdf_collision_names[target_index] != (
                generated_collision_name
            ):
                raise CollisionProxyError(
                    "contact-material collision target drift: "
                    + str(generated_collision_name)
                )
        ET.SubElement(
            extension,
            "origin",
            {
                "xyz": _format_vector(proxy["proxy_pose_xyz"]),
                "rpy": _format_vector(proxy["proxy_pose_rpy"]),
            },
        )
        extension_geometry = ET.SubElement(extension, "geometry")
        ET.SubElement(
            extension_geometry,
            "box",
            {"size": _format_vector(proxy["proxy_size_m"])},
        )
        extension_joint = ET.SubElement(
            root,
            "joint",
            {"name": extension_joint_name, "type": "fixed"},
        )
        ET.SubElement(extension_joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        ET.SubElement(extension_joint, "parent", {"link": link_name})
        ET.SubElement(extension_joint, "child", {"link": extension_link_name})
        contact_extensions_added.append(
            f"{extension_link_name}:{link_name}[{index}]:{proxy['source_mesh']}"
        )
        if (
            pad_contact_material is not None
            and pad_contact_material.apply_explicit_friction
        ):
            assert pad_contact_material.mu1 is not None
            assert pad_contact_material.mu2 is not None
            gazebo = ET.SubElement(root, "gazebo", {"reference": extension_link_name})
            ET.SubElement(gazebo, "mu1").text = format(
                pad_contact_material.mu1, ".12g"
            )
            ET.SubElement(gazebo, "mu2").text = format(
                pad_contact_material.mu2, ".12g"
            )
            contact_material_references.append(
                f"{extension_link_name}:"
                f"{pad_contact_material.target_collisions[target_index]}"
            )

    if anchor_to_world:
        world_anchor_added, roots_before, roots_after = _ensure_gazebo_world_anchor(
            root, contract["gazebo_world_anchor"]
        )
    else:
        anchor = contract["gazebo_world_anchor"]
        roots_before = _root_links(root)
        if roots_before != (anchor["child_link"],):
            raise CollisionProxyError(
                "MoveIt proxy expected a single base root, got: "
                + ",".join(roots_before)
            )
        world_anchor_added = False
        roots_after = roots_before
    visuals_after = _visual_meshes(root)
    if visuals_after != visuals_before:
        raise CollisionProxyError("visual mesh set changed during proxy transform")
    remaining = _collision_meshes(root)
    if require_no_collision_meshes and remaining:
        raise CollisionProxyError(
            "uncontracted collision meshes remain: " + ",".join(remaining)
        )
    if len(validated_collision_meshes) != contract["expected_proxy_count"]:
        raise CollisionProxyError("not all contracted collision meshes were validated")
    expected_replaced = (
        contract["expected_proxy_count"] if replace_collision_meshes else 0
    )
    if len(replaced) != expected_replaced:
        raise CollisionProxyError("not all collision proxies were applied")
    if len(contact_extensions_added) != contract["expected_contact_extension_count"]:
        raise CollisionProxyError("not all gripper contact extensions were applied")
    if pad_contact_material is not None:
        generated_links = tuple(
            item.split(":", maxsplit=1)[0] for item in contact_extensions_added
        )
        if generated_links != pad_contact_material.target_links:
            raise CollisionProxyError("not all contact-material pad targets were generated")
        expected_material_references = (
            len(pad_contact_material.target_links)
            if pad_contact_material.apply_explicit_friction
            else 0
        )
        if len(contact_material_references) != expected_material_references:
            raise CollisionProxyError("not all contact-material targets were applied")

    xml = ET.tostring(root, encoding="unicode", xml_declaration=True)
    return xml, CollisionProxyReport(
        replaced=tuple(replaced),
        validated_collision_meshes=tuple(validated_collision_meshes),
        contact_extensions_added=tuple(contact_extensions_added),
        visual_meshes_before=visuals_before,
        visual_meshes_after=visuals_after,
        remaining_collision_meshes=remaining,
        world_anchor_added=world_anchor_added,
        root_links_before=roots_before,
        root_links_after=roots_after,
        contact_material_profile=(
            pad_contact_material.name if pad_contact_material is not None else None
        ),
        contact_material_references=tuple(contact_material_references),
    )
