"""Load the shared EdgeGrasp table/cube contract into MoveIt fail closed."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from edgegrasp.config import (
    DEFAULT_TARGET_FRAME,
    ROS_SIM_CLOCK_DOMAIN,
    validate_ros_clock_domain,
)
from edgegrasp.collision_proxy import load_collision_proxy_contract
from edgegrasp.scene import SceneBox, SceneContract, load_scene_contract
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import GetPlanningScene
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger


_GEOMETRY_COMPONENTS = (
    PlanningSceneComponents.WORLD_OBJECT_NAMES
    | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
    | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
)
_ABS_TOL = 1e-7


@dataclass(frozen=True, slots=True)
class ExpectedCollisionObject:
    object_id: str
    frame_id: str
    size_m: tuple[float, float, float]
    position_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]


def expected_collision_objects(
    contract: SceneContract, *, include_optional: bool
) -> tuple[ExpectedCollisionObject, ...]:
    """Project the shared scene into a ROS-independent verification contract."""

    return tuple(
        ExpectedCollisionObject(
            object_id=item.planning_scene_id,
            frame_id=contract.planning_frame,
            size_m=item.size_m,
            position_m=item.pose_world.position_m,
            quaternion_xyzw=item.pose_world.quaternion_xyzw,
        )
        for item in contract.planning_objects(include_optional=include_optional)
    )


def build_collision_object(item: SceneBox, frame_id: str) -> CollisionObject:
    """Build one box object without copying scene dimensions into ROS code."""

    message = CollisionObject()
    message.header.frame_id = frame_id
    message.pose.orientation.w = 1.0
    message.id = item.planning_scene_id
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = list(item.size_m)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = item.pose_world.position_m
    (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ) = item.pose_world.quaternion_xyzw
    message.primitives = [primitive]
    message.primitive_poses = [pose]
    message.operation = CollisionObject.ADD
    return message


def build_remove_collision_object(object_id: str, frame_id: str) -> CollisionObject:
    """Build an explicit removal for one EdgeGrasp-managed scene object."""

    message = CollisionObject()
    message.header.frame_id = frame_id
    message.id = object_id
    message.operation = CollisionObject.REMOVE
    return message


def verify_planning_scene_objects(
    observed: list[CollisionObject] | tuple[CollisionObject, ...],
    expected: tuple[ExpectedCollisionObject, ...],
    forbidden_ids: tuple[str, ...] = (),
) -> str | None:
    """Require expected geometry and absence of disabled managed objects."""

    by_id = {item.id: item for item in observed}
    for object_id in forbidden_ids:
        if object_id in by_id:
            return f"unexpected_object:{object_id}"
    for contract in expected:
        item = by_id.get(contract.object_id)
        if item is None:
            return f"missing_object:{contract.object_id}"
        if item.header.frame_id != contract.frame_id:
            return (
                f"frame_mismatch:{contract.object_id}:"
                f"{item.header.frame_id}!={contract.frame_id}"
            )
        if len(item.primitives) != 1 or len(item.primitive_poses) != 1:
            return f"primitive_count_mismatch:{contract.object_id}"
        primitive = item.primitives[0]
        if primitive.type != SolidPrimitive.BOX:
            return f"primitive_type_mismatch:{contract.object_id}"
        if len(primitive.dimensions) != 3 or not _close_vector(
            primitive.dimensions, contract.size_m
        ):
            return f"dimensions_mismatch:{contract.object_id}"
        object_pose = item.pose
        primitive_pose = item.primitive_poses[0]
        object_quaternion = _quaternion_tuple(object_pose.orientation)
        primitive_quaternion = _quaternion_tuple(primitive_pose.orientation)
        if not _unit_quaternion(object_quaternion):
            return f"invalid_object_orientation:{contract.object_id}"
        if not _unit_quaternion(primitive_quaternion):
            return f"invalid_primitive_orientation:{contract.object_id}"
        primitive_offset = _rotate_vector(
            object_quaternion,
            (
                float(primitive_pose.position.x),
                float(primitive_pose.position.y),
                float(primitive_pose.position.z),
            ),
        )
        effective_position = tuple(
            coordinate + offset
            for coordinate, offset in zip(
                (
                    float(object_pose.position.x),
                    float(object_pose.position.y),
                    float(object_pose.position.z),
                ),
                primitive_offset,
                strict=True,
            )
        )
        if not _close_vector(effective_position, contract.position_m):
            return f"position_mismatch:{contract.object_id}"
        observed_quaternion = _quaternion_multiply(
            object_quaternion,
            primitive_quaternion,
        )
        if not _same_orientation(observed_quaternion, contract.quaternion_xyzw):
            return f"orientation_mismatch:{contract.object_id}"
    return None


def _matrix_rows(
    matrix: AllowedCollisionMatrix,
) -> tuple[tuple[str, ...], tuple[tuple[bool, ...], ...]]:
    names = tuple(str(name) for name in matrix.entry_names)
    if len(set(names)) != len(names) or any(not name for name in names):
        raise ValueError("allowed collision matrix names must be unique")
    if len(matrix.entry_values) != len(names):
        raise ValueError("allowed collision matrix must be square")
    rows = tuple(tuple(bool(value) for value in row.enabled) for row in matrix.entry_values)
    if any(len(row) != len(names) for row in rows):
        raise ValueError("allowed collision matrix must be square")
    for row in range(len(names)):
        for column in range(len(names)):
            if rows[row][column] != rows[column][row]:
                raise ValueError("allowed collision matrix must be symmetric")
    if len(matrix.default_entry_names) != len(matrix.default_entry_values):
        raise ValueError("allowed collision defaults are malformed")
    return names, rows


def build_target_pad_collision_matrix(
    observed: AllowedCollisionMatrix,
    *,
    target_object_id: str,
    allowed_pad_links: tuple[str, ...],
    forbidden_parent_links: tuple[str, ...],
    allow: bool,
) -> AllowedCollisionMatrix:
    """Preserve the complete ACM while changing only target-to-pad pairs."""

    if (
        not target_object_id
        or not allowed_pad_links
        or len(set(allowed_pad_links)) != len(allowed_pad_links)
        or set(allowed_pad_links) & set(forbidden_parent_links)
    ):
        raise ValueError("target contact policy identities are invalid")
    names, rows = _matrix_rows(observed)
    required = (target_object_id, *allowed_pad_links, *forbidden_parent_links)
    expanded_names = list(names)
    for name in required:
        if not name:
            raise ValueError("target contact policy identities are invalid")
        if name not in expanded_names:
            expanded_names.append(name)
    size = len(expanded_names)
    expanded = [[False for _ in range(size)] for _ in range(size)]
    for row in range(len(names)):
        for column in range(len(names)):
            expanded[row][column] = rows[row][column]
    target_index = expanded_names.index(target_object_id)
    for link in allowed_pad_links:
        link_index = expanded_names.index(link)
        expanded[target_index][link_index] = bool(allow)
        expanded[link_index][target_index] = bool(allow)
    for link in forbidden_parent_links:
        link_index = expanded_names.index(link)
        expanded[target_index][link_index] = False
        expanded[link_index][target_index] = False

    result = AllowedCollisionMatrix()
    result.entry_names = expanded_names
    for values in expanded:
        entry = AllowedCollisionEntry()
        entry.enabled = values
        result.entry_values.append(entry)
    result.default_entry_names = list(observed.default_entry_names)
    result.default_entry_values = list(observed.default_entry_values)
    return result


def _effective_collision_permission(
    matrix: AllowedCollisionMatrix, first: str, second: str
) -> bool:
    names, rows = _matrix_rows(matrix)
    by_name = {name: index for index, name in enumerate(names)}
    if first in by_name and second in by_name:
        return rows[by_name[first]][by_name[second]]
    defaults = dict(
        zip(
            matrix.default_entry_names,
            matrix.default_entry_values,
            strict=True,
        )
    )
    values = [bool(defaults[name]) for name in (first, second) if name in defaults]
    if len(values) == 2:
        return all(values)
    return values[0] if values else False


def verify_target_pad_collision_policy(
    observed: AllowedCollisionMatrix,
    *,
    target_object_id: str,
    allowed_pad_links: tuple[str, ...],
    forbidden_parent_links: tuple[str, ...],
    allow: bool,
) -> str | None:
    try:
        for link in allowed_pad_links:
            if _effective_collision_permission(observed, target_object_id, link) != bool(
                allow
            ):
                return f"target_pad_policy_mismatch:{link}"
        for link in forbidden_parent_links:
            if _effective_collision_permission(observed, target_object_id, link):
                return f"forbidden_target_contact_allowed:{link}"
    except ValueError as error:
        return f"allowed_collision_matrix_invalid:{error}"
    return None


def _close_vector(observed, expected) -> bool:
    try:
        return len(observed) == len(expected) and all(
            math.isfinite(float(value))
            and math.isclose(
                float(value), float(reference), rel_tol=0.0, abs_tol=_ABS_TOL
            )
            for value, reference in zip(observed, expected, strict=True)
        )
    except (TypeError, ValueError):
        return False


def _quaternion_tuple(quaternion) -> tuple[float, float, float, float]:
    return (
        float(quaternion.x),
        float(quaternion.y),
        float(quaternion.z),
        float(quaternion.w),
    )


def _unit_quaternion(quaternion: tuple[float, float, float, float]) -> bool:
    return all(math.isfinite(value) for value in quaternion) and math.isclose(
        sum(value * value for value in quaternion),
        1.0,
        rel_tol=0.0,
        abs_tol=_ABS_TOL,
    )


def _quaternion_multiply(
    first: tuple[float, float, float, float],
    second: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = first
    bx, by, bz, bw = second
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate_vector(
    quaternion: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    rotated = _quaternion_multiply(
        _quaternion_multiply(quaternion, (*vector, 0.0)),
        (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3]),
    )
    return rotated[:3]


def _same_orientation(
    observed: tuple[float, float, float, float],
    expected: tuple[float, float, float, float],
) -> bool:
    return _close_vector(observed, expected) or _close_vector(
        observed,
        tuple(-value for value in expected),
    )


class PlanningSceneLoader(Node):
    """Publish scene objects and report ready only after MoveIt service echo."""

    def __init__(self, **node_kwargs) -> None:
        super().__init__("edgegrasp_planning_scene_loader", **node_kwargs)
        self.declare_parameter("scene_config", "")
        self.declare_parameter("target_frame", DEFAULT_TARGET_FRAME)
        self.declare_parameter("include_optional_cube", False)
        self.declare_parameter("collision_object_topic", "/collision_object")
        self.declare_parameter("planning_scene_diff_topic", "/planning_scene")
        self.declare_parameter("get_planning_scene_service", "/get_planning_scene")
        self.declare_parameter(
            "planning_scene_ready_topic", "/edgegrasp/planning_scene_ready"
        )
        self.declare_parameter(
            "planning_scene_status_topic", "/edgegrasp/planning_scene_status"
        )
        self.declare_parameter(
            "optional_cube_collision_service",
            "/edgegrasp/set_optional_cube_collision",
        )
        self.declare_parameter(
            "target_pad_contact_service",
            "/edgegrasp/set_target_pad_contacts",
        )
        self.declare_parameter("clock_domain", ROS_SIM_CLOCK_DOMAIN)
        self.declare_parameter("clock_epoch", 0)
        self.declare_parameter("retry_period_ms", 500.0)
        self.declare_parameter("query_timeout_ms", 750.0)

        self._target_frame = str(self.get_parameter("target_frame").value)
        self._clock_domain = str(self.get_parameter("clock_domain").value)
        self._clock_epoch = int(self.get_parameter("clock_epoch").value)
        validate_ros_clock_domain(
            bool(self.get_parameter("use_sim_time").value), self._clock_domain
        )
        if not self._target_frame or self._clock_epoch < 0:
            raise ValueError("target_frame and clock_epoch must be valid")
        self._retry_s = self._positive_ms("retry_period_ms") / 1000.0
        self._query_timeout_s = self._positive_ms("query_timeout_ms") / 1000.0

        package_share = Path(get_package_share_directory("edgegrasp_ros"))
        configured_path = str(self.get_parameter("scene_config").value).strip()
        config_path = (
            Path(configured_path)
            if configured_path
            else package_share / "config" / "scene.json"
        )
        self._contract = load_scene_contract(config_path)
        if self._contract.planning_frame != self._target_frame:
            raise ValueError(
                "scene planning frame does not match configured target_frame"
            )
        proxy_contract = load_collision_proxy_contract(
            package_share / "config" / "so101_collision_proxies.json"
        )
        self._target_contact_links = tuple(
            item["extension_link"]
            for item in proxy_contract["gazebo_gripper_contact_extensions"]
        )
        self._forbidden_target_contact_links = tuple(
            dict.fromkeys(
                item["link"]
                for item in proxy_contract["gazebo_gripper_contact_extensions"]
            )
        )
        targets = tuple(item for item in self._contract.objects if not item.required)
        if len(targets) != 1:
            raise ValueError("phase-one scene must contain one optional target")
        self._target_object_id = targets[0].planning_scene_id
        self._include_optional_cube = bool(
            self.get_parameter("include_optional_cube").value
        )
        self._messages: tuple[CollisionObject, ...] = ()
        self._expected: tuple[ExpectedCollisionObject, ...] = ()
        self._forbidden_ids: tuple[str, ...] = ()
        self._configure_scene_policy(self._include_optional_cube)

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._object_publisher = self.create_publisher(
            CollisionObject,
            str(self.get_parameter("collision_object_topic").value),
            10,
        )
        self._scene_diff_publisher = self.create_publisher(
            PlanningScene,
            str(self.get_parameter("planning_scene_diff_topic").value),
            10,
        )
        self._ready_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter("planning_scene_ready_topic").value),
            latched_qos,
        )
        self._status_publisher = self.create_publisher(
            String,
            str(self.get_parameter("planning_scene_status_topic").value),
            latched_qos,
        )
        self._client = self.create_client(
            GetPlanningScene,
            str(self.get_parameter("get_planning_scene_service").value),
        )
        self.create_service(
            Trigger,
            "/edgegrasp/reset_planning_scene_epoch",
            self._on_reset,
        )
        self.create_service(
            SetBool,
            str(self.get_parameter("optional_cube_collision_service").value),
            self._on_set_optional_cube,
        )
        self.create_service(
            SetBool,
            str(self.get_parameter("target_pad_contact_service").value),
            self._on_set_target_pad_contacts,
        )
        self._pending = None
        self._pending_started_wall: float | None = None
        self._next_attempt_wall = 0.0
        self._last_clock_ns: int | None = None
        self._fault_latched: str | None = None
        self._allow_target_pad_contacts = False
        self._last_observed_acm: AllowedCollisionMatrix | None = None
        self._desired_acm: AllowedCollisionMatrix | None = None
        self._ready = False
        self._last_reason = "initializing"
        self._publish_ready(False, self._last_reason)
        tick_period = min(0.1, max(0.02, self._retry_s / 4.0))
        self._timer = self.create_timer(tick_period, self._tick)

    def _positive_ms(self, parameter_name: str) -> float:
        value = float(self.get_parameter(parameter_name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{parameter_name} must be finite and positive")
        return value

    def _configure_scene_policy(self, include_optional: bool) -> None:
        selected = self._contract.planning_objects(
            include_optional=include_optional
        )
        all_managed = self._contract.planning_objects(include_optional=True)
        selected_ids = {item.planning_scene_id for item in selected}
        excluded = tuple(
            item for item in all_managed if item.planning_scene_id not in selected_ids
        )
        additions = tuple(
            build_collision_object(item, self._target_frame) for item in selected
        )
        removals = tuple(
            build_remove_collision_object(item.planning_scene_id, self._target_frame)
            for item in excluded
        )
        if not additions:
            raise ValueError("planning scene must contain at least one object")
        self._include_optional_cube = bool(include_optional)
        self._messages = additions + removals
        self._expected = expected_collision_objects(
            self._contract, include_optional=include_optional
        )
        self._forbidden_ids = tuple(item.planning_scene_id for item in excluded)

    @property
    def ready(self) -> bool:
        return self._ready

    @property
    def contract_digest(self) -> str:
        return self._contract.digest

    def _observe_clock(self) -> str | None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns < 0:
            return "invalid_clock"
        if self._last_clock_ns is not None and now_ns < self._last_clock_ns:
            return f"clock_rollback:{now_ns}<{self._last_clock_ns}"
        self._last_clock_ns = now_ns
        return None

    def _tick(self) -> None:
        clock_fault = self._observe_clock()
        if clock_fault is not None:
            self._fault_latched = clock_fault
        if self._fault_latched is not None:
            self._publish_ready(False, f"fault_latched:{self._fault_latched}")
            return

        wall_now = time.monotonic()
        if self._pending is not None:
            if self._pending.done():
                self._finish_query()
            elif (
                self._pending_started_wall is not None
                and wall_now - self._pending_started_wall > self._query_timeout_s
            ):
                self._pending = None
                self._pending_started_wall = None
                self._ready = False
                self._last_reason = "planning_scene_query_timeout"
                self._publish_ready(False, self._last_reason)
                self._next_attempt_wall = wall_now + self._retry_s
            return

        if wall_now < self._next_attempt_wall:
            self._publish_ready(self._ready, self._last_reason)
            return
        # CollisionObject uses volatile topic delivery.  Publish while the
        # scene is unconfirmed so a late MoveGroup subscriber can still see
        # the contract, but do not continuously re-ADD already-confirmed
        # geometry during periodic service revalidation.  Repeated ADDs were
        # observed to contend with MoveIt's kinematics services on Jazzy.
        if not self._ready:
            for message in self._messages:
                message.header.stamp = self.get_clock().now().to_msg()
                self._object_publisher.publish(message)
            if self._desired_acm is not None:
                self._publish_acm_diff(self._desired_acm)
        if not self._client.service_is_ready():
            self._ready = False
            self._last_reason = "get_planning_scene_unavailable"
            self._publish_ready(False, self._last_reason)
            self._next_attempt_wall = wall_now + self._retry_s
            return
        request = GetPlanningScene.Request()
        request.components.components = _GEOMETRY_COMPONENTS
        try:
            self._pending = self._client.call_async(request)
        except Exception as error:
            self._pending = None
            self._ready = False
            self._last_reason = (
                f"get_planning_scene_request_exception:{type(error).__name__}"
            )
            self._publish_ready(False, self._last_reason)
            self._next_attempt_wall = wall_now + self._retry_s
            return
        self._pending_started_wall = wall_now
        self._last_reason = (
            "confirmed_revalidation_pending"
            if self._ready
            else "awaiting_planning_scene_confirmation"
        )
        # A periodic revalidation must not emit a false pulse after the same
        # geometry was already confirmed.  The adapter applies its own
        # receive-time TTL; a timeout, service loss, mismatch, or clock fault
        # below still revokes readiness fail closed.
        self._publish_ready(self._ready, self._last_reason)

    def _finish_query(self) -> None:
        future = self._pending
        self._pending = None
        self._pending_started_wall = None
        try:
            response = future.result()
            reason = verify_planning_scene_objects(
                response.scene.world.collision_objects,
                self._expected,
                self._forbidden_ids,
            )
            self._last_observed_acm = response.scene.allowed_collision_matrix
            if reason is None:
                reason = verify_target_pad_collision_policy(
                    self._last_observed_acm,
                    target_object_id=self._target_object_id,
                    allowed_pad_links=self._target_contact_links,
                    forbidden_parent_links=self._forbidden_target_contact_links,
                    allow=self._allow_target_pad_contacts,
                )
        except Exception as error:
            reason = f"planning_scene_query_exception:{type(error).__name__}"
        self._ready = reason is None
        self._last_reason = "confirmed" if reason is None else str(reason)
        self._publish_ready(self._ready, self._last_reason)
        self._next_attempt_wall = time.monotonic() + self._retry_s

    def _publish_acm_diff(self, matrix: AllowedCollisionMatrix) -> None:
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.allowed_collision_matrix = matrix
        self._scene_diff_publisher.publish(scene)

    def _publish_ready(self, ready: bool, reason: str) -> None:
        self._ready_publisher.publish(Bool(data=bool(ready)))
        status = String()
        status.data = json.dumps(
            {
                "ready": bool(ready),
                "reason": reason,
                "expected_objects": [item.object_id for item in self._expected],
                "forbidden_objects": list(self._forbidden_ids),
                "include_optional_cube": self._include_optional_cube,
                "allow_target_pad_contacts": self._allow_target_pad_contacts,
                "target_contact_links": list(self._target_contact_links),
                "forbidden_target_contact_links": list(
                    self._forbidden_target_contact_links
                ),
                "scene_digest": self._contract.digest,
                "target_frame": self._target_frame,
                "clock_domain": self._clock_domain,
                "clock_epoch": self._clock_epoch,
            },
            sort_keys=True,
        )
        self._status_publisher.publish(status)

    def _on_reset(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        if self._pending is not None and not self._pending.done():
            response.success = False
            response.message = "reset refused while planning-scene query is pending"
            return response
        self._clock_epoch += 1
        self._last_clock_ns = self.get_clock().now().nanoseconds
        self._fault_latched = None
        self._ready = False
        self._last_reason = "epoch_reset_requires_reconfirmation"
        self._next_attempt_wall = 0.0
        self._publish_ready(False, self._last_reason)
        response.success = True
        response.message = f"planning scene reset to epoch {self._clock_epoch}"
        return response

    def _on_set_optional_cube(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        if self._fault_latched is not None:
            response.success = False
            response.message = f"fault_latched:{self._fault_latched}"
            return response
        if self._pending is not None:
            if self._pending.done():
                self._finish_query()
            else:
                # The response belongs to the old policy.  No callback is
                # attached, so dropping our reference is safe; readiness is
                # revoked below before any new geometry is published.
                self._pending = None
                self._pending_started_wall = None
        requested = bool(request.data)
        if requested == self._include_optional_cube:
            response.success = True
            response.message = (
                "optional cube policy unchanged; wait for confirmed readiness"
            )
            return response
        self._allow_target_pad_contacts = False
        if self._last_observed_acm is not None:
            try:
                self._desired_acm = build_target_pad_collision_matrix(
                    self._last_observed_acm,
                    target_object_id=self._target_object_id,
                    allowed_pad_links=self._target_contact_links,
                    forbidden_parent_links=self._forbidden_target_contact_links,
                    allow=False,
                )
            except ValueError as error:
                self._fault_latched = f"allowed_collision_matrix_invalid:{error}"
                response.success = False
                response.message = self._fault_latched
                return response
        self._configure_scene_policy(requested)
        self._ready = False
        self._last_reason = (
            "optional_cube_enabled_requires_confirmation"
            if requested
            else "optional_cube_disabled_requires_confirmation"
        )
        self._next_attempt_wall = 0.0
        self._publish_ready(False, self._last_reason)
        response.success = True
        response.message = (
            "optional cube collision enabled; wait for confirmed readiness"
            if requested
            else "optional cube collision disabled; wait for confirmed readiness"
        )
        return response

    def _on_set_target_pad_contacts(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        if self._fault_latched is not None:
            response.success = False
            response.message = f"fault_latched:{self._fault_latched}"
            return response
        if not self._include_optional_cube:
            response.success = False
            response.message = "target cube must remain collision-active"
            return response
        if self._pending is not None:
            if self._pending.done():
                self._finish_query()
            elif self._ready and self._last_observed_acm is not None:
                # This is a periodic query for the already-confirmed policy.
                # Drop only our reference to that read-only response before
                # revoking readiness and publishing the new ACM diff below.
                # Initial/unconfirmed scene queries remain fail-closed.
                self._pending = None
                self._pending_started_wall = None
            else:
                response.success = False
                response.message = (
                    "target contact policy refused while unconfirmed query is pending"
                )
                return response
        if not self._ready or self._last_observed_acm is None:
            response.success = False
            response.message = "planning scene must be confirmed before policy change"
            return response
        requested = bool(request.data)
        if requested == self._allow_target_pad_contacts:
            response.success = True
            response.message = "target pad contact policy unchanged and confirmed"
            return response
        try:
            matrix = build_target_pad_collision_matrix(
                self._last_observed_acm,
                target_object_id=self._target_object_id,
                allowed_pad_links=self._target_contact_links,
                forbidden_parent_links=self._forbidden_target_contact_links,
                allow=requested,
            )
        except ValueError as error:
            self._fault_latched = f"allowed_collision_matrix_invalid:{error}"
            self._publish_ready(False, f"fault_latched:{self._fault_latched}")
            response.success = False
            response.message = self._fault_latched
            return response
        self._allow_target_pad_contacts = requested
        self._desired_acm = matrix
        self._ready = False
        self._last_reason = (
            "target_pad_contacts_enabled_requires_confirmation"
            if requested
            else "target_pad_contacts_disabled_requires_confirmation"
        )
        self._next_attempt_wall = 0.0
        self._publish_ready(False, self._last_reason)
        self._publish_acm_diff(matrix)
        response.success = True
        response.message = (
            "target cube retained; distal-pad contact allowance awaits confirmation"
            if requested
            else "target-pad allowance revoked; wait for confirmation"
        )
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlanningSceneLoader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # A launch-level SIGINT can already have shut the default context down
        # before spin unwinds. Avoid turning a controlled stop into RCLError.
        if rclpy.ok():
            rclpy.shutdown()
