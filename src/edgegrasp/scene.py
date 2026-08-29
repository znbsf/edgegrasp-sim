"""Validated, deterministic Gazebo / MoveIt shared scene contract."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET


class SceneContractError(ValueError):
    """Raised when the scene source is incomplete, unsafe, or drifting."""


@dataclass(frozen=True, slots=True)
class ScenePose:
    position_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class SceneBox:
    object_id: str
    gazebo_name: str
    planning_scene_id: str
    size_m: tuple[float, float, float]
    pose_world: ScenePose
    static: bool
    required: bool
    planning_scene_default: bool
    mass_kg: float | None
    color_rgba: tuple[float, float, float, float]
    contact_topic: str | None
    pose_topic: str | None


@dataclass(frozen=True, slots=True)
class SceneContract:
    name: str
    gazebo_world_frame: str
    planning_frame: str
    world_to_planning: ScenePose
    clock_domain: str
    physics_engine: str
    max_step_size_s: float
    real_time_factor: float
    ground_size_m: tuple[float, float]
    objects: tuple[SceneBox, ...]
    digest: str

    def planning_objects(self, *, include_optional: bool) -> tuple[SceneBox, ...]:
        return tuple(
            item
            for item in self.objects
            if item.planning_scene_default or (include_optional and not item.required)
        )


def _numbers(
    value: Any, *, size: int, field: str, positive: bool = False
) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != size:
        raise SceneContractError(f"{field} must contain {size} numbers")
    if not all(
        isinstance(item, (int, float))
        and not isinstance(item, bool)
        and math.isfinite(float(item))
        for item in value
    ):
        raise SceneContractError(f"{field} contains a non-finite value")
    result = tuple(float(item) for item in value)
    if positive and not all(item > 0.0 for item in result):
        raise SceneContractError(f"{field} must be positive")
    return result


def _pose(value: Any, *, field: str) -> ScenePose:
    if not isinstance(value, dict):
        raise SceneContractError(f"{field} must be an object")
    position = _numbers(value.get("position_m"), size=3, field=f"{field}.position_m")
    quaternion = _numbers(
        value.get("quaternion_xyzw"), size=4, field=f"{field}.quaternion_xyzw"
    )
    norm = math.sqrt(sum(item * item for item in quaternion))
    if abs(norm - 1.0) > 1e-9:
        raise SceneContractError(f"{field}.quaternion_xyzw is not normalized")
    return ScenePose(position, quaternion)


def _quaternion_rotation_rows(
    quaternion_xyzw: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], ...]:
    x, y, z, w = quaternion_xyzw
    return (
        (
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ),
        (
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ),
        (
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ),
    )


def _quaternion_to_rpy(
    quaternion_xyzw: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    """Convert a validated quaternion to the SDF Euler pose convention."""

    x, y, z, w = quaternion_xyzw
    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return roll, pitch, yaw


def load_scene_contract(path: Path) -> SceneContract:
    """Load the one source used to generate Gazebo and MoveIt objects."""

    raw = json.loads(path.read_text(encoding="utf-8"))
    if raw.get("schema_version") != 1:
        raise SceneContractError("unsupported scene schema")
    frames = raw.get("frames")
    if not isinstance(frames, dict):
        raise SceneContractError("missing scene frames")
    if frames.get("gazebo_world") != "world" or frames.get("planning") != "base_link":
        raise SceneContractError("phase-one scene requires world and base_link frames")
    transform = _pose(frames.get("world_to_planning"), field="world_to_planning")
    if transform != ScenePose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)):
        raise SceneContractError("phase-one world_to_planning must be identity")
    if frames.get("clock_domain") != "ros_sim":
        raise SceneContractError("Gazebo scene clock domain must be ros_sim")

    physics = raw.get("physics")
    if not isinstance(physics, dict):
        raise SceneContractError("missing physics contract")
    if physics.get("engine") != "gz-physics-dartsim-plugin":
        raise SceneContractError("scene must pin the DART engine")
    max_step = float(physics.get("max_step_size_s", 0.0))
    real_time_factor = float(physics.get("real_time_factor", 0.0))
    if not math.isfinite(max_step) or max_step <= 0.0:
        raise SceneContractError("invalid max_step_size_s")
    if not math.isfinite(real_time_factor) or real_time_factor <= 0.0:
        raise SceneContractError("invalid real_time_factor")

    ground = raw.get("ground")
    if not isinstance(ground, dict) or ground.get("shape") != "plane":
        raise SceneContractError("missing ground plane")
    ground_size = _numbers(
        ground.get("size_m"), size=2, field="ground.size_m", positive=True
    )

    object_rows = raw.get("objects")
    if not isinstance(object_rows, list) or not object_rows:
        raise SceneContractError("scene objects are missing")
    objects: list[SceneBox] = []
    ids: set[str] = set()
    gazebo_names: set[str] = set()
    planning_ids: set[str] = set()
    for row in object_rows:
        if not isinstance(row, dict) or row.get("shape") != "box":
            raise SceneContractError("only box scene objects are supported")
        object_id = row.get("id")
        gazebo_name = row.get("gazebo_name")
        planning_id = row.get("planning_scene_id")
        if not all(
            isinstance(value, str) and value
            for value in (object_id, gazebo_name, planning_id)
        ):
            raise SceneContractError("scene object IDs must be nonempty")
        if object_id in ids or gazebo_name in gazebo_names or planning_id in planning_ids:
            raise SceneContractError("duplicate scene object identity")
        ids.add(object_id)
        gazebo_names.add(gazebo_name)
        planning_ids.add(planning_id)
        required = row.get("required")
        planning_default = row.get("planning_scene_default")
        static = row.get("static")
        if not all(
            isinstance(value, bool)
            for value in (required, planning_default, static)
        ):
            raise SceneContractError("scene object flags must be boolean")
        mass_value = row.get("mass_kg")
        mass = None if mass_value is None else float(mass_value)
        if not static and (mass is None or not math.isfinite(mass) or mass <= 0.0):
            raise SceneContractError("dynamic scene objects need positive mass")
        contact_topic = row.get("contact_topic")
        if contact_topic is not None and (
            not isinstance(contact_topic, str) or not contact_topic.startswith("/")
        ):
            raise SceneContractError("contact_topic must be an absolute topic")
        pose_topic = row.get("pose_topic")
        if pose_topic is not None and (
            not isinstance(pose_topic, str) or not pose_topic.startswith("/")
        ):
            raise SceneContractError("pose_topic must be an absolute topic")
        if pose_topic is not None:
            expected_pose_topic = f"/model/{gazebo_name}/pose"
            if static or pose_topic != expected_pose_topic:
                raise SceneContractError(
                    "dynamic pose_topic must use /model/<gazebo_name>/pose"
                )
        objects.append(
            SceneBox(
                object_id=object_id,
                gazebo_name=gazebo_name,
                planning_scene_id=planning_id,
                size_m=_numbers(
                    row.get("size_m"),
                    size=3,
                    field=f"objects.{object_id}.size_m",
                    positive=True,
                ),
                pose_world=_pose(
                    row.get("pose_world"), field=f"objects.{object_id}.pose_world"
                ),
                static=static,
                required=required,
                planning_scene_default=planning_default,
                mass_kg=mass,
                color_rgba=_numbers(
                    row.get("color_rgba"),
                    size=4,
                    field=f"objects.{object_id}.color_rgba",
                ),
                contact_topic=contact_topic,
                pose_topic=pose_topic,
            )
        )
    by_id = {item.object_id: item for item in objects}
    if "table" not in by_id or not by_id["table"].required:
        raise SceneContractError("required table object is missing")
    if "target_cube" not in by_id or by_id["target_cube"].required:
        raise SceneContractError("optional target_cube object is missing")
    table = by_id["table"]
    cube = by_id["target_cube"]
    table_top = table.pose_world.position_m[2] + table.size_m[2] / 2.0
    cube_bottom = cube.pose_world.position_m[2] - cube.size_m[2] / 2.0
    if abs(table_top - cube_bottom) > 1e-9:
        raise SceneContractError("target_cube bottom must equal table top")
    cube_rotation = _quaternion_rotation_rows(cube.pose_world.quaternion_xyzw)
    cube_half_extents = tuple(
        sum(
            abs(cube_rotation[axis][cube_axis]) * cube.size_m[cube_axis] / 2.0
            for cube_axis in range(3)
        )
        for axis in range(3)
    )
    for axis in (0, 1):
        table_low = table.pose_world.position_m[axis] - table.size_m[axis] / 2.0
        table_high = table.pose_world.position_m[axis] + table.size_m[axis] / 2.0
        cube_low = cube.pose_world.position_m[axis] - cube_half_extents[axis]
        cube_high = cube.pose_world.position_m[axis] + cube_half_extents[axis]
        if cube_low < table_low or cube_high > table_high:
            raise SceneContractError("target_cube footprint must stay inside table")

    canonical = json.dumps(raw, sort_keys=True, separators=(",", ":"))
    return SceneContract(
        name=str(raw.get("name")),
        gazebo_world_frame="world",
        planning_frame="base_link",
        world_to_planning=transform,
        clock_domain="ros_sim",
        physics_engine="gz-physics-dartsim-plugin",
        max_step_size_s=max_step,
        real_time_factor=real_time_factor,
        ground_size_m=(ground_size[0], ground_size[1]),
        objects=tuple(objects),
        digest=hashlib.sha256(canonical.encode("utf-8")).hexdigest(),
    )


def _fmt(values: tuple[float, ...]) -> str:
    return " ".join(format(value, ".12g") for value in values)


def render_gazebo_sdf(contract: SceneContract) -> str:
    """Generate the Gazebo world from the validated shared scene source."""

    sdf = ET.Element("sdf", {"version": "1.9"})
    world = ET.SubElement(sdf, "world", {"name": contract.name})
    world.append(ET.Comment(f" scene_contract_sha256={contract.digest} "))
    physics = ET.SubElement(world, "physics", {"name": "1ms", "type": "ignored"})
    ET.SubElement(physics, "max_step_size").text = format(
        contract.max_step_size_s, ".12g"
    )
    ET.SubElement(physics, "real_time_factor").text = format(
        contract.real_time_factor, ".12g"
    )
    physics_plugin = ET.SubElement(
        world,
        "plugin",
        {"filename": "gz-sim-physics-system", "name": "gz::sim::systems::Physics"},
    )
    engine = ET.SubElement(physics_plugin, "engine")
    ET.SubElement(engine, "filename").text = contract.physics_engine
    for filename, name in (
        ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
        ("gz-sim-contact-system", "gz::sim::systems::Contact"),
        ("gz-sim-sensors-system", "gz::sim::systems::Sensors"),
        ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
    ):
        plugin = ET.SubElement(world, "plugin", {"filename": filename, "name": name})
        if name.endswith("Sensors"):
            ET.SubElement(plugin, "render_engine").text = "ogre2"

    light = ET.SubElement(world, "light", {"type": "directional", "name": "sun"})
    ET.SubElement(light, "pose").text = "0 0 10 0 0 0"
    ET.SubElement(light, "diffuse").text = "0.8 0.8 0.8 1"
    ET.SubElement(light, "specular").text = "0.2 0.2 0.2 1"
    ET.SubElement(light, "direction").text = "-0.5 0.1 -0.9"

    ground = ET.SubElement(world, "model", {"name": "ground"})
    ET.SubElement(ground, "static").text = "true"
    ground_link = ET.SubElement(ground, "link", {"name": "ground_link"})
    for tag in ("collision", "visual"):
        element = ET.SubElement(ground_link, tag, {"name": tag})
        geometry = ET.SubElement(element, "geometry")
        plane = ET.SubElement(geometry, "plane")
        ET.SubElement(plane, "normal").text = "0 0 1"
        ET.SubElement(plane, "size").text = _fmt(contract.ground_size_m)

    for item in contract.objects:
        model = ET.SubElement(world, "model", {"name": item.gazebo_name})
        ET.SubElement(model, "static").text = str(item.static).lower()
        x, y, z = item.pose_world.position_m
        roll, pitch, yaw = _quaternion_to_rpy(item.pose_world.quaternion_xyzw)
        ET.SubElement(model, "pose").text = _fmt((x, y, z, roll, pitch, yaw))
        link = ET.SubElement(model, "link", {"name": f"{item.object_id}_link"})
        if not item.static:
            assert item.mass_kg is not None
            sx, sy, sz = item.size_m
            inertial = ET.SubElement(link, "inertial")
            ET.SubElement(inertial, "mass").text = format(item.mass_kg, ".12g")
            inertia = ET.SubElement(inertial, "inertia")
            ET.SubElement(inertia, "ixx").text = format(
                item.mass_kg * (sy * sy + sz * sz) / 12.0, ".12g"
            )
            ET.SubElement(inertia, "iyy").text = format(
                item.mass_kg * (sx * sx + sz * sz) / 12.0, ".12g"
            )
            ET.SubElement(inertia, "izz").text = format(
                item.mass_kg * (sx * sx + sy * sy) / 12.0, ".12g"
            )
        collision = ET.SubElement(link, "collision", {"name": "collision"})
        collision_geometry = ET.SubElement(collision, "geometry")
        collision_box = ET.SubElement(collision_geometry, "box")
        ET.SubElement(collision_box, "size").text = _fmt(item.size_m)
        if item.contact_topic is not None:
            sensor = ET.SubElement(
                link, "sensor", {"name": f"{item.object_id}_contact", "type": "contact"}
            )
            ET.SubElement(sensor, "always_on").text = "true"
            ET.SubElement(sensor, "update_rate").text = "100"
            contact = ET.SubElement(sensor, "contact")
            ET.SubElement(contact, "collision").text = "collision"
            ET.SubElement(contact, "topic").text = item.contact_topic
        if item.pose_topic is not None:
            pose_publisher = ET.SubElement(
                model,
                "plugin",
                {
                    "filename": "gz-sim-pose-publisher-system",
                    "name": "gz::sim::systems::PosePublisher",
                },
            )
            ET.SubElement(pose_publisher, "publish_link_pose").text = "false"
            ET.SubElement(pose_publisher, "publish_model_pose").text = "true"
            ET.SubElement(
                pose_publisher, "publish_nested_model_pose"
            ).text = "false"
            ET.SubElement(pose_publisher, "publish_collision_pose").text = "false"
            ET.SubElement(pose_publisher, "publish_visual_pose").text = "false"
            ET.SubElement(pose_publisher, "publish_sensor_pose").text = "false"
            ET.SubElement(pose_publisher, "use_pose_vector_msg").text = "false"
            ET.SubElement(pose_publisher, "update_frequency").text = "100"
        visual = ET.SubElement(link, "visual", {"name": "visual"})
        visual_geometry = ET.SubElement(visual, "geometry")
        visual_box = ET.SubElement(visual_geometry, "box")
        ET.SubElement(visual_box, "size").text = _fmt(item.size_m)
        material = ET.SubElement(visual, "material")
        ET.SubElement(material, "diffuse").text = _fmt(item.color_rgba)

    ET.indent(sdf, space="  ")
    return '<?xml version="1.0"?>\n' + ET.tostring(sdf, encoding="unicode") + "\n"
