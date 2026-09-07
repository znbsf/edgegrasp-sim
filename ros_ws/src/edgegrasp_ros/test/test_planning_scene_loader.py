"""Jazzy tests for service-confirmed shared planning-scene loading."""

from __future__ import annotations

from itertools import count
import threading
import time

from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import GetPlanningScene
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

from edgegrasp_ros.planning_scene_loader import (
    ExpectedCollisionObject,
    PlanningSceneLoader,
    build_target_pad_collision_matrix,
    expected_collision_objects,
    verify_planning_scene_objects,
    verify_target_pad_collision_policy,
)


DOMAIN_IDS = count(201)


def wait_until(predicate, timeout_s: float = 5.0, reason: str = "condition") -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError(f"timed out waiting for {reason}")


class FakePlanningScene(Node):
    def __init__(
        self,
        *,
        context: Context,
        topic: str,
        service: str,
        ready_topic: str,
        scene_topic: str,
        response_delay_s: float = 0.0,
    ) -> None:
        super().__init__("fake_edgegrasp_planning_scene", context=context)
        self.objects: dict[str, CollisionObject] = {}
        self.object_messages = 0
        self.requests: list[int] = []
        self.ready_states: list[bool] = []
        self.allowed_collision_matrix = AllowedCollisionMatrix()
        self.response_delay_s = response_delay_s
        self.subscription = self.create_subscription(
            CollisionObject, topic, self._on_object, 10
        )
        self.service = self.create_service(
            GetPlanningScene, service, self._on_get_scene
        )
        self.ready_subscription = self.create_subscription(
            Bool,
            ready_topic,
            lambda message: self.ready_states.append(bool(message.data)),
            10,
        )
        self.scene_subscription = self.create_subscription(
            PlanningScene, scene_topic, self._on_scene, 10
        )

    def _on_object(self, message: CollisionObject) -> None:
        self.object_messages += 1
        if message.operation == CollisionObject.REMOVE:
            self.objects.pop(message.id, None)
        else:
            self.objects[message.id] = message

    def _on_get_scene(self, request, response):
        if self.response_delay_s:
            time.sleep(self.response_delay_s)
        self.requests.append(int(request.components.components))
        response.scene.world.collision_objects = list(self.objects.values())
        response.scene.allowed_collision_matrix = self.allowed_collision_matrix
        return response

    def _on_scene(self, message: PlanningScene) -> None:
        if message.is_diff:
            self.allowed_collision_matrix = message.allowed_collision_matrix


@pytest.mark.parametrize(
    ("include_optional", "expected_ids"),
    [
        (False, {"edgegrasp_table"}),
        (True, {"edgegrasp_table", "edgegrasp_target_cube"}),
    ],
)
def test_loader_reports_ready_only_after_service_echo(
    include_optional: bool, expected_ids: set[str]
) -> None:
    context = Context()
    domain_id = next(DOMAIN_IDS)
    rclpy.init(context=context, domain_id=domain_id)
    # ROS graph-name tokens cannot begin with a digit.
    suffix = f"d{domain_id}"
    topic = f"/test/{suffix}/collision_object"
    service = f"/test/{suffix}/get_planning_scene"
    ready_topic = f"/test/{suffix}/ready"
    scene_topic = f"/test/{suffix}/planning_scene"
    fake = FakePlanningScene(
        context=context,
        topic=topic,
        service=service,
        ready_topic=ready_topic,
        scene_topic=scene_topic,
    )
    loader = PlanningSceneLoader(
        context=context,
        parameter_overrides=[
            Parameter("use_sim_time", value=False),
            Parameter("clock_domain", value="ros_system"),
            Parameter("include_optional_cube", value=include_optional),
            Parameter("collision_object_topic", value=topic),
            Parameter("get_planning_scene_service", value=service),
            Parameter("planning_scene_ready_topic", value=ready_topic),
            Parameter("planning_scene_diff_topic", value=scene_topic),
            Parameter("planning_scene_status_topic", value=f"/test/{suffix}/status"),
            Parameter("retry_period_ms", value=50.0),
            Parameter("query_timeout_ms", value=200.0),
        ],
    )
    executor = MultiThreadedExecutor(num_threads=3, context=context)
    executor.add_node(fake)
    executor.add_node(loader)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert not loader.ready
        wait_until(lambda: loader.ready, reason="confirmed planning scene")
        published_after_confirmation = fake.object_messages
        wait_until(lambda: len(fake.requests) >= 3, reason="periodic revalidation")
        assert set(fake.objects) == expected_ids
        assert fake.object_messages == published_after_confirmation
        assert fake.requests
        expected_mask = (PlanningSceneComponents.WORLD_OBJECT_NAMES
                         | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                         | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
                         | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        assert all(mask == expected_mask for mask in fake.requests)
        assert loader._last_reason in {
            "confirmed",
            "confirmed_revalidation_pending",
        }
        first_ready = fake.ready_states.index(True)
        assert all(fake.ready_states[first_ready:])
    finally:
        executor.shutdown(timeout_sec=3.0)
        thread.join(timeout=3.0)
        fake.destroy_node()
        loader.destroy_node()
        rclpy.shutdown(context=context)


def test_verifier_rejects_missing_and_drifted_geometry() -> None:
    context = Context()
    rclpy.init(context=context, domain_id=next(DOMAIN_IDS))
    loader = PlanningSceneLoader(
        context=context,
        parameter_overrides=[
            Parameter("use_sim_time", value=False),
            Parameter("clock_domain", value="ros_system"),
            Parameter(
                "get_planning_scene_service", value="/test/never_available_scene"
            ),
            Parameter("retry_period_ms", value=50.0),
        ],
    )
    try:
        expected = expected_collision_objects(
            loader._contract, include_optional=False
        )
        assert verify_planning_scene_objects([], expected) == (
            "missing_object:edgegrasp_table"
        )
        message = loader._messages[0]
        message.primitives[0].dimensions[0] += 0.01
        assert verify_planning_scene_objects([message], expected) == (
            "dimensions_mismatch:edgegrasp_table"
        )
        optional = loader._contract.planning_objects(include_optional=True)[1]
        optional_message = loader._messages[0]
        optional_message.id = optional.planning_scene_id
        assert verify_planning_scene_objects(
            [optional_message], (), (optional.planning_scene_id,)
        ) == "unexpected_object:edgegrasp_target_cube"

        # MoveIt may canonicalize a primitive pose into CollisionObject.pose.
        # The verifier must compare the composed transform, not reject this
        # equivalent representation merely because object.pose is rotated.
        yaw_quaternion = (0.0, 0.0, 0.3193124504754839, 0.9476494916219507)
        rotated_expected = (
            ExpectedCollisionObject(
                object_id="rotated_cube",
                frame_id="base_link",
                size_m=(0.05, 0.05, 0.05),
                position_m=(0.24, 0.13, 0.205),
                quaternion_xyzw=yaw_quaternion,
            ),
        )
        rotated = CollisionObject()
        rotated.header.frame_id = "base_link"
        rotated.id = "rotated_cube"
        rotated.pose.position.x = 0.24
        rotated.pose.position.y = 0.13
        rotated.pose.position.z = 0.205
        (
            rotated.pose.orientation.x,
            rotated.pose.orientation.y,
            rotated.pose.orientation.z,
            rotated.pose.orientation.w,
        ) = yaw_quaternion
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.05, 0.05, 0.05]
        primitive_pose = Pose()
        primitive_pose.orientation.w = 1.0
        rotated.primitives = [primitive]
        rotated.primitive_poses = [primitive_pose]
        assert verify_planning_scene_objects(
            [rotated], rotated_expected
        ) is None
        rotated.pose.orientation.z = 0.0
        rotated.pose.orientation.w = 1.0
        assert verify_planning_scene_objects(
            [rotated], rotated_expected
        ) == "orientation_mismatch:rotated_cube"
    finally:
        loader.destroy_node()
        rclpy.shutdown(context=context)


def test_target_pad_matrix_preserves_existing_entries_and_forbids_parents() -> None:
    observed = AllowedCollisionMatrix()
    observed.entry_names = ["existing_a", "existing_b"]
    for values in ([True, False], [False, True]):
        entry = AllowedCollisionEntry()
        entry.enabled = list(values)
        observed.entry_values.append(entry)
    observed.default_entry_names = ["default_safe"]
    observed.default_entry_values = [False]
    pads = (
        "edgegrasp_fixed_finger_pad_link",
        "edgegrasp_moving_finger_pad_link",
    )
    parents = ("gripper_link", "moving_jaw_so101_v1_link")
    updated = build_target_pad_collision_matrix(
        observed,
        target_object_id="edgegrasp_target_cube",
        allowed_pad_links=pads,
        forbidden_parent_links=parents,
        allow=True,
    )
    assert updated.entry_names[:2] == observed.entry_names
    assert list(updated.entry_values[0].enabled[:2]) == [True, False]
    assert list(updated.entry_values[1].enabled[:2]) == [False, True]
    assert verify_target_pad_collision_policy(
        updated,
        target_object_id="edgegrasp_target_cube",
        allowed_pad_links=pads,
        forbidden_parent_links=parents,
        allow=True,
    ) is None
    revoked = build_target_pad_collision_matrix(
        updated,
        target_object_id="edgegrasp_target_cube",
        allowed_pad_links=pads,
        forbidden_parent_links=parents,
        allow=False,
    )
    assert verify_target_pad_collision_policy(
        revoked,
        target_object_id="edgegrasp_target_cube",
        allowed_pad_links=pads,
        forbidden_parent_links=parents,
        allow=False,
    ) is None


def test_runtime_policy_removes_optional_cube_before_ready() -> None:
    context = Context()
    domain_id = next(DOMAIN_IDS)
    rclpy.init(context=context, domain_id=domain_id)
    suffix = f"d{domain_id}"
    topic = f"/test/{suffix}/collision_object"
    service = f"/test/{suffix}/get_planning_scene"
    ready_topic = f"/test/{suffix}/ready"
    scene_topic = f"/test/{suffix}/planning_scene"
    policy_service = f"/test/{suffix}/optional_cube"
    fake = FakePlanningScene(
        context=context,
        topic=topic,
        service=service,
        ready_topic=ready_topic,
        scene_topic=scene_topic,
    )
    loader = PlanningSceneLoader(
        context=context,
        parameter_overrides=[
            Parameter("use_sim_time", value=False),
            Parameter("clock_domain", value="ros_system"),
            Parameter("include_optional_cube", value=True),
            Parameter("collision_object_topic", value=topic),
            Parameter("get_planning_scene_service", value=service),
            Parameter("planning_scene_ready_topic", value=ready_topic),
            Parameter("planning_scene_diff_topic", value=scene_topic),
            Parameter("planning_scene_status_topic", value=f"/test/{suffix}/status"),
            Parameter("optional_cube_collision_service", value=policy_service),
            Parameter("retry_period_ms", value=50.0),
            Parameter("query_timeout_ms", value=200.0),
        ],
    )
    client = fake.create_client(SetBool, policy_service)
    executor = MultiThreadedExecutor(num_threads=3, context=context)
    executor.add_node(fake)
    executor.add_node(loader)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(
            lambda: loader.ready
            and set(fake.objects)
            == {"edgegrasp_table", "edgegrasp_target_cube"},
            reason="optional cube confirmed",
        )
        request = SetBool.Request()
        request.data = False
        response_future = client.call_async(request)
        wait_until(response_future.done, reason="scene-policy response")
        assert response_future.result().success
        wait_until(
            lambda: loader.ready and set(fake.objects) == {"edgegrasp_table"},
            reason="optional cube removal confirmation",
        )
        assert loader._forbidden_ids == ("edgegrasp_target_cube",)
        assert loader._last_reason in {
            "confirmed",
            "confirmed_revalidation_pending",
        }
    finally:
        executor.shutdown(timeout_sec=3.0)
        thread.join(timeout=3.0)
        fake.destroy_node()
        loader.destroy_node()
        rclpy.shutdown(context=context)


def test_target_pad_policy_keeps_cube_and_confirms_only_distal_links() -> None:
    context = Context()
    domain_id = next(DOMAIN_IDS)
    rclpy.init(context=context, domain_id=domain_id)
    suffix = f"d{domain_id}"
    topic = f"/test/{suffix}/collision_object"
    scene_topic = f"/test/{suffix}/planning_scene"
    scene_service = f"/test/{suffix}/get_planning_scene"
    ready_topic = f"/test/{suffix}/ready"
    contact_service = f"/test/{suffix}/target_pad_contacts"
    fake = FakePlanningScene(
        context=context,
        topic=topic,
        service=scene_service,
        ready_topic=ready_topic,
        scene_topic=scene_topic,
        response_delay_s=0.1,
    )
    loader = PlanningSceneLoader(
        context=context,
        parameter_overrides=[
            Parameter("use_sim_time", value=False),
            Parameter("clock_domain", value="ros_system"),
            Parameter("include_optional_cube", value=True),
            Parameter("collision_object_topic", value=topic),
            Parameter("planning_scene_diff_topic", value=scene_topic),
            Parameter("get_planning_scene_service", value=scene_service),
            Parameter("planning_scene_ready_topic", value=ready_topic),
            Parameter("planning_scene_status_topic", value=f"/test/{suffix}/status"),
            Parameter("target_pad_contact_service", value=contact_service),
            Parameter("retry_period_ms", value=50.0),
            Parameter("query_timeout_ms", value=500.0),
        ],
    )
    client = fake.create_client(SetBool, contact_service)
    executor = MultiThreadedExecutor(num_threads=3, context=context)
    executor.add_node(fake)
    executor.add_node(loader)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(
            lambda: loader.ready,
            reason="initial collision policy confirmation",
        )
        # Reproduce the loaded-system runtime case: an already-confirmed
        # scene has a periodic read-only revalidation in flight when the ACM
        # change is requested.  This must not create a timing lottery.
        wait_until(
            lambda: loader._pending is not None and not loader._pending.done(),
            reason="periodic revalidation in flight",
        )
        request = SetBool.Request()
        request.data = True
        response_future = client.call_async(request)
        wait_until(response_future.done, reason="target-pad policy response")
        assert response_future.result().success
        wait_until(
            lambda: loader.ready and loader._allow_target_pad_contacts,
            timeout_s=7.0,
            reason="target-pad policy confirmation",
        )
        assert set(fake.objects) == {"edgegrasp_table", "edgegrasp_target_cube"}
        assert verify_target_pad_collision_policy(
            fake.allowed_collision_matrix,
            target_object_id=loader._target_object_id,
            allowed_pad_links=loader._target_contact_links,
            forbidden_parent_links=loader._forbidden_target_contact_links,
            allow=True,
        ) is None
    finally:
        executor.shutdown(timeout_sec=3.0)
        thread.join(timeout=3.0)
        fake.destroy_node()
        loader.destroy_node()
        rclpy.shutdown(context=context)


def test_unavailable_service_and_clock_rollback_never_report_ready() -> None:
    context = Context()
    rclpy.init(context=context, domain_id=next(DOMAIN_IDS))
    loader = PlanningSceneLoader(
        context=context,
        parameter_overrides=[
            Parameter("use_sim_time", value=False),
            Parameter("clock_domain", value="ros_system"),
            Parameter(
                "get_planning_scene_service", value="/test/missing_planning_scene"
            ),
            Parameter("retry_period_ms", value=50.0),
        ],
    )
    try:
        loader._tick()
        assert not loader.ready
        assert loader._last_reason == "get_planning_scene_unavailable"

        loader._last_clock_ns = loader.get_clock().now().nanoseconds + 1_000_000
        loader._tick()
        assert not loader.ready
        assert loader._fault_latched is not None
        response = loader._on_reset(Trigger.Request(), Trigger.Response())
        assert response.success
        assert not loader.ready
        assert loader._last_reason == "epoch_reset_requires_reconfirmation"
    finally:
        loader.destroy_node()
        rclpy.shutdown(context=context)
