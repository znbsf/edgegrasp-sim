"""ROS-runtime tests for the read-only grasp-physics evidence observer.

These tests require a built ROS 2 Jazzy workspace.  They use only in-process
publishers and an ``ActionClient`` for the observer action; they do not start
Gazebo, MoveIt, a controller, or any motion backend.  All synchronization is
bounded by an event/future or a monotonic deadline.  There are no protocol
``sleep(0.05)`` assumptions in this file.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import threading
import time
from typing import Callable

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration, Time
from edgegrasp.scene import load_scene_contract
from edgegrasp.trajectory_identity import make_trajectory_command_id
from edgegrasp_interfaces.action import GraspPhysicsEvidence
from edgegrasp_interfaces.msg import GraspSequenceTerminal
from edgegrasp_ros.grasp_physics_observer import GraspPhysicsObserver
from geometry_msgs.msg import PoseStamped, Vector3
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from ros_gz_interfaces.msg import Contact, Contacts, JointWrench
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState


BASE_NS = 1_000_000_000
TARGET_SOURCE_NS = BASE_NS
FRESHNESS_NS = 200_000_000
ARM_CUBE = "target_cube::target_cube_link::collision"
TABLE = "table::table_link::collision"
GRIPPER = (
    "so101::gripper_link::"
    "gripper_link_fixed_joint_lump__edgegrasp_fixed_finger_pad_link_collision_2"
)
MOVING_GRIPPER = (
    "so101::moving_jaw_so101_v1_link::"
    "moving_jaw_so101_v1_link_fixed_joint_lump__"
    "edgegrasp_moving_finger_pad_link_collision_1"
)
PALM = "so101::gripper_link::gripper_link_collision"
POSE_TOPIC = "/edgegrasp/test/target_cube_pose"
CONTACT_TOPIC = "/edgegrasp/test/target_cube_contacts"
SEQUENCE_TOPIC = "/edgegrasp/test/grasp_sequence_terminal"
JOINT_STATES_TOPIC = "/edgegrasp/test/joint_states"
ACTION_NAME = "/edgegrasp/test/grasp_physics_evidence"


def wait_until(
    predicate: Callable[[], bool],
    *,
    timeout_s: float = 5.0,
    reason: str,
) -> None:
    """Wait for an observable condition without an unbounded or fixed delay."""

    deadline = time.monotonic() + timeout_s
    last_error: BaseException | None = None
    while time.monotonic() < deadline:
        try:
            if predicate():
                return
        except BaseException as error:  # expose predicate failures in timeout
            last_error = error
        time.sleep(0.005)
    detail = "" if last_error is None else f"; last_error={last_error!r}"
    raise AssertionError(f"timed out waiting for {reason}{detail}")


def wait_future(future, *, timeout_s: float = 10.0, reason: str):
    """Wait for an rclpy future using its completion event."""

    completed = threading.Event()
    future.add_done_callback(lambda _: completed.set())
    if not future.done() and not completed.wait(timeout_s):
        raise AssertionError(f"timed out waiting for {reason}")
    return future.result()


def stamp(value_ns: int) -> Time:
    message = Time()
    message.sec = value_ns // 1_000_000_000
    message.nanosec = value_ns % 1_000_000_000
    return message


def duration(value_ns: int) -> Duration:
    message = Duration()
    message.sec = value_ns // 1_000_000_000
    message.nanosec = value_ns % 1_000_000_000
    return message


def pair_message(
    value_ns: int,
    first: str,
    second: str,
    *additional_pairs: tuple[str, str],
    depth_m: float | None = None,
    force_n: float | None = None,
) -> Contacts:
    message = Contacts()
    message.header.frame_id = "world"
    message.header.stamp = stamp(value_ns)
    for collision1, collision2 in ((first, second), *additional_pairs):
        pair = Contact()
        pair.collision1.name = collision1
        pair.collision2.name = collision2
        if depth_m is not None:
            pair.depths.append(depth_m)
        if force_n is not None:
            pair.normals.append(Vector3(x=1.0, y=0.0, z=0.0))
            wrench = JointWrench()
            wrench.body_1_wrench.force.x = force_n
            wrench.body_2_wrench.force.x = -force_n
            pair.wrenches.append(wrench)
        message.contacts.append(pair)
    return message


def pose_message(value_ns: int, frame_id: str, z_m: float = 0.425) -> PoseStamped:
    message = PoseStamped()
    message.header.frame_id = frame_id
    message.header.stamp = stamp(value_ns)
    message.pose.position.x = 0.2
    message.pose.position.y = 0.0
    message.pose.position.z = z_m
    message.pose.orientation.w = 1.0
    return message


def sequence_message(
    value_ns: int,
    *,
    task_id: str = "task-001",
    target_id: str = "cube-001",
    lift_command_id: str | None = None,
    source_timestamp_ns: int = TARGET_SOURCE_NS,
    completed: bool = True,
) -> GraspSequenceTerminal:
    message = GraspSequenceTerminal()
    message.terminal_stamp = stamp(value_ns)
    message.task_id = task_id
    message.target_id = target_id
    message.lift_command_id = lift_command_id or make_trajectory_command_id(
        task_id, "lift", 3
    )
    message.accepted = completed
    message.sequence_completed = completed
    message.terminal_phase = "COMPLETE" if completed else "FAULT"
    message.reason = "sequence_complete" if completed else "sequence_failed"
    message.trajectory_digest = "a" * 64
    message.action_goal_status = (
        GoalStatus.STATUS_SUCCEEDED if completed else GoalStatus.STATUS_ABORTED
    )
    message.source_timestamp_ns = source_timestamp_ns
    message.clock_domain = "ros_sim"
    message.clock_epoch = 0
    return message


@dataclass
class ObserverResultSnapshot:
    phase: str
    reason: str
    physics_grasp_verified: bool
    pose_sample_count: int
    gripper_contact_count: int
    table_contact_count: int
    sequence_completed: bool
    gripper_effort_sample_count: int


class PhysicsObserverHarness:
    """In-process ROS graph for one isolated observer action invocation."""

    def __init__(self) -> None:
        rclpy.init()
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory("edgegrasp_ros"))
        self.scene = load_scene_contract(share / "config" / "scene.json")
        overrides = [
            Parameter("use_sim_time", value=True),
            Parameter("evidence_action", value=ACTION_NAME),
            Parameter("cube_pose_topic", value=POSE_TOPIC),
            Parameter("cube_contact_topic", value=CONTACT_TOPIC),
            Parameter("sequence_terminal_topic", value=SEQUENCE_TOPIC),
            Parameter("joint_states_topic", value=JOINT_STATES_TOPIC),
            Parameter("scene_config", value=str(share / "config" / "scene.json")),
            Parameter("clock_domain", value="ros_sim"),
            Parameter("clock_epoch", value=0),
            Parameter("minimum_baseline_samples", value=5),
            Parameter("minimum_lift_m", value=0.02),
            Parameter("minimum_clearance_m", value=0.01),
            Parameter("minimum_retention_s", value=0.5),
            Parameter("maximum_xy_drift_m", value=0.01),
            Parameter("maximum_freshness_ms", value=200.0),
            Parameter("maximum_observation_s", value=5.0),
            Parameter("loop_rate_hz", value=100.0),
        ]
        self.observer = GraspPhysicsObserver(parameter_overrides=overrides)
        self.driver = Node("grasp_physics_observer_test_driver")
        self.clock = self.driver.create_publisher(Clock, "/clock", 10)
        self.pose = self.driver.create_publisher(PoseStamped, POSE_TOPIC, 10)
        self.contacts = self.driver.create_publisher(Contacts, CONTACT_TOPIC, 10)
        self.sequence = self.driver.create_publisher(
            GraspSequenceTerminal, SEQUENCE_TOPIC, 10
        )
        self.joint_states = self.driver.create_publisher(
            JointState, JOINT_STATES_TOPIC, 10
        )
        self.action = ActionClient(self.driver, GraspPhysicsEvidence, ACTION_NAME)
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.observer)
        self.executor.add_node(self.driver)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self._wait_for_graph()

    def _wait_for_graph(self) -> None:
        wait_until(
            self.action.server_is_ready,
            reason="grasp physics evidence action server",
        )
        for publisher, name in (
            (self.clock, "/clock subscriber"),
            (self.pose, "pose subscriber"),
            (self.contacts, "contact subscriber"),
            (self.sequence, "sequence subscriber"),
            (self.joint_states, "joint state subscriber"),
        ):
            wait_until(
                lambda publisher=publisher: publisher.get_subscription_count() >= 1,
                reason=name,
            )

    def close(self) -> None:
        self.executor.shutdown(timeout_sec=3.0)
        self.executor._executor.shutdown(wait=True, cancel_futures=False)
        self.thread.join(timeout=3.0)
        with self.executor._tasks_lock:
            completed_tasks = tuple(self.executor._pending_tasks)
        for task in completed_tasks:
            if task.done() and not task.cancelled():
                task.exception()
        self.action.destroy()
        self.driver.destroy_node()
        self.observer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def publish_clock(self, value_ns: int) -> None:
        message = Clock()
        message.clock = stamp(value_ns)

        def publish_until_observed() -> bool:
            if self.observer.get_clock().now().nanoseconds == value_ns:
                return True
            # A freshly-created /clock subscription can legitimately miss a
            # single sample before discovery settles. Republish the identical
            # immutable value within the bounded wait instead of sleeping and
            # assuming delivery.
            self.clock.publish(message)
            return False

        wait_until(
            publish_until_observed,
            reason=f"observer ROS clock {value_ns}",
        )

    def _core_snapshot(self) -> ObserverResultSnapshot | None:
        with self.observer._lock:
            core = self.observer._core
            result = None if core is None else core.result
        if result is None:
            return None
        return ObserverResultSnapshot(
            phase=result.phase.value,
            reason=result.reason,
            physics_grasp_verified=result.physics_grasp_verified,
            pose_sample_count=result.pose_sample_count,
            gripper_contact_count=result.gripper_contact_count,
            table_contact_count=result.table_contact_count,
            sequence_completed=result.sequence_completed,
            gripper_effort_sample_count=result.gripper_effort_sample_count,
        )

    def start_goal(self):
        self.publish_clock(BASE_NS)
        goal = GraspPhysicsEvidence.Goal()
        goal.task_id = "task-001"
        goal.target_id = "cube-001"
        goal.cube_model_name = "target_cube"
        goal.pose_frame_id = self.scene.name
        goal.expected_lift_command_id = make_trajectory_command_id(
            goal.task_id, "lift", 3
        )
        goal.scene_digest = self.scene.digest
        goal.started_at = stamp(BASE_NS)
        goal.target_source_timestamp_ns = TARGET_SOURCE_NS
        goal.clock_domain = "ros_sim"
        goal.clock_epoch = 0
        goal.baseline_sample_count = 5
        goal.min_lift_m = 0.02
        goal.min_clearance_m = 0.01
        goal.retention_duration = duration(500_000_000)
        goal.max_xy_drift_m = 0.01
        goal.freshness_timeout = duration(FRESHNESS_NS)
        goal.observation_timeout = duration(2_000_000_000)
        goal_handle = wait_future(
            self.action.send_goal_async(goal),
            reason="observer goal acceptance",
        )
        assert goal_handle.accepted, "observer rejected a valid evidence goal"
        wait_until(
            lambda: self._core_snapshot() is not None,
            reason="observer core task activation",
        )
        return goal_handle, goal_handle.get_result_async()

    def publish_pose(
        self,
        value_ns: int,
        *,
        z_m: float = 0.425,
        frame_id: str | None = None,
        expected_count: int | None = None,
    ) -> None:
        self.publish_clock(value_ns)
        # Gazebo Sim 8.11 PosePublisher identifies the containing named world
        # in Pose.header.frame_id.  The live bridge was observed to preserve
        # this value as ``edgegrasp_table_cube``; never relabel it base_link.
        message = pose_message(
            value_ns,
            self.scene.name if frame_id is None else frame_id,
            z_m,
        )
        self.pose.publish(message)
        if expected_count is not None:
            wait_until(
                lambda: (
                    self._core_snapshot() is not None
                    and self._core_snapshot().pose_sample_count >= expected_count
                ),
                reason=f"pose sample {expected_count}",
            )

    def publish_contact(
        self,
        value_ns: int,
        first: str,
        second: str,
        *,
        additional_pairs: tuple[tuple[str, str], ...] = (),
        expected_gripper_count: int | None = None,
        expected_table_count: int | None = None,
        depth_m: float | None = None,
        force_n: float | None = None,
    ) -> None:
        self.publish_clock(value_ns)
        self.contacts.publish(
            pair_message(
                value_ns,
                first,
                second,
                *additional_pairs,
                depth_m=depth_m,
                force_n=force_n,
            )
        )
        if expected_gripper_count is not None:
            wait_until(
                lambda: (
                    self._core_snapshot() is not None
                    and self._core_snapshot().gripper_contact_count
                    >= expected_gripper_count
                ),
                reason=f"gripper contact {expected_gripper_count}",
            )
        if expected_table_count is not None:
            wait_until(
                lambda: (
                    self._core_snapshot() is not None
                    and self._core_snapshot().table_contact_count >= expected_table_count
                ),
                reason=f"table contact {expected_table_count}",
            )

    def publish_effort(self, value_ns: int, value: float) -> None:
        self.publish_clock(value_ns)
        message = JointState()
        message.header.stamp = stamp(value_ns)
        message.name = ["gripper"]
        message.position = [0.6]
        message.effort = [value]
        self.joint_states.publish(message)
        wait_until(
            lambda: (
                self._core_snapshot() is not None
                and self._core_snapshot().gripper_effort_sample_count >= 1
            ),
            reason="gripper effort sample",
        )

    def publish_sequence(self, value_ns: int) -> None:
        self.publish_clock(value_ns)
        self.sequence.publish(sequence_message(value_ns))
        wait_until(
            lambda: (
                self._core_snapshot() is not None
                and self._core_snapshot().sequence_completed
            ),
            reason="matching sequence terminal",
        )

    def baseline(self, *, frame_id: str | None = None) -> None:
        for index in range(5):
            self.publish_pose(
                BASE_NS + (index + 1) * 10_000_000,
                frame_id=frame_id,
                expected_count=index + 1,
            )

    def result(self, result_future):
        return wait_future(result_future, reason="observer action result").result

    def assert_no_motion_interfaces(self) -> None:
        publishers = self.observer.get_publisher_names_and_types_by_node(
            self.observer.get_name(), self.observer.get_namespace()
        )
        clients = self.observer.get_client_names_and_types_by_node(
            self.observer.get_name(), self.observer.get_namespace()
        )
        forbidden = (
            "follow_joint_trajectory",
            "joint_trajectory",
            "trajectory_request",
            "execute_trajectory",
        )
        names = [name.lower() for name, _ in (*publishers, *clients)]
        assert not any(
            token in name for name in names for token in forbidden
        ), f"observer graph exposes motion interfaces: {names}"


@pytest.fixture
def harness():
    instance = PhysicsObserverHarness()
    try:
        yield instance
    finally:
        instance.close()


def test_baseline_contact_sequence_lift_retention_succeeds_and_observer_is_read_only(
    harness: PhysicsObserverHarness,
) -> None:
    _, result_future = harness.start_goal()
    harness.baseline()
    harness.publish_effort(BASE_NS + 55_000_000, -0.75)
    harness.publish_contact(
        BASE_NS + 60_000_000,
        ARM_CUBE,
        GRIPPER,
        additional_pairs=((ARM_CUBE, MOVING_GRIPPER),),
        expected_gripper_count=2,
        depth_m=0.0004,
        force_n=1.25,
    )
    harness.publish_sequence(BASE_NS + 70_000_000)
    harness.publish_pose(
        BASE_NS + 80_000_000, z_m=0.46, expected_count=6
    )
    for contact_count, offset_ns in enumerate((
        180_000_000,
        280_000_000,
        380_000_000,
        480_000_000,
    ), start=2):
        harness.publish_contact(
            BASE_NS + offset_ns,
            ARM_CUBE,
            GRIPPER,
            additional_pairs=((ARM_CUBE, MOVING_GRIPPER),),
            expected_gripper_count=contact_count * 2,
        )
        harness.publish_pose(
            BASE_NS + offset_ns,
            z_m=0.46,
            expected_count=5 + contact_count,
        )
    # The retained pose and contact are both only 100 ms old. Advancing the
    # single ROS clock to the exact 500 ms retention boundary must let tick()
    # verify without relying on another input callback.
    harness.publish_clock(BASE_NS + 580_000_000)

    result = harness.result(result_future)
    assert result.physics_grasp_verified, (
        f"phase={result.terminal_phase};reason={result.reason};"
        f"poses={result.pose_sample_count};"
        f"gripper_contacts={result.gripper_contact_count};"
        f"table_contacts={result.table_contact_count}"
    )
    assert result.terminal_phase == "VERIFIED"
    assert result.sequence_completed
    assert result.gripper_contact_observed
    assert result.lift_observed
    assert result.retention_observed
    assert result.gripper_contact_count >= 1
    assert result.gripper_contact_tokens == [
        "fixed_finger_pad",
        "moving_finger_pad",
    ]
    assert result.gripper_contact_counts_by_token[0] >= 1
    assert result.gripper_contact_counts_by_token[1] >= 1
    assert list(result.gripper_first_contact_stamp_available_by_token) == [
        True,
        True,
    ]
    assert list(result.gripper_first_contact_source_timestamp_ns_by_token) == [
        BASE_NS + 60_000_000,
        BASE_NS + 60_000_000,
    ]
    assert list(result.gripper_last_contact_source_timestamp_ns_by_token) == [
        BASE_NS + 480_000_000,
        BASE_NS + 480_000_000,
    ]
    assert result.all_gripper_tokens_observed
    assert result.simultaneous_gripper_contact_sample_count >= 1
    assert result.first_simultaneous_contact_stamp_available
    assert (
        result.first_simultaneous_contact_source_timestamp_ns
        == BASE_NS + 60_000_000
    )
    assert result.last_simultaneous_contact_stamp_available
    assert (
        result.last_simultaneous_contact_source_timestamp_ns
        == BASE_NS + 480_000_000
    )
    assert list(result.gripper_penetration_depth_available_by_token) == [True, True]
    assert list(result.gripper_max_penetration_depth_m_by_token) == pytest.approx(
        [0.0004, 0.0004]
    )
    assert list(result.gripper_force_magnitude_available_by_token) == [True, True]
    assert list(result.gripper_max_force_magnitude_n_by_token) == pytest.approx(
        [1.25, 1.25]
    )
    assert list(result.gripper_normal_force_available_by_token) == [True, True]
    assert list(result.gripper_max_abs_normal_force_n_by_token) == pytest.approx(
        [1.25, 1.25]
    )
    assert result.simultaneous_min_penetration_depth_available
    assert result.simultaneous_max_min_penetration_depth_m == 0.0004
    assert result.simultaneous_min_force_magnitude_available
    assert result.simultaneous_max_min_force_magnitude_n == 1.25
    assert result.simultaneous_min_normal_force_available
    assert result.simultaneous_max_min_abs_normal_force_n == 1.25
    assert result.gripper_joint_name == "gripper"
    assert result.gripper_effort_available
    assert result.gripper_effort_sample_count == 1
    assert result.latest_gripper_effort == -0.75
    assert result.peak_abs_gripper_effort == 0.75
    assert result.sequence_completion_stamp_available
    assert result.sequence_completed_source_timestamp_ns == BASE_NS + 70_000_000
    assert (
        result.simultaneous_gripper_contact_sample_count_at_sequence_completion
        == 1
    )
    assert result.last_simultaneous_contact_at_sequence_completion_available
    assert (
        result.last_simultaneous_contact_source_timestamp_ns_at_sequence_completion
        == BASE_NS + 60_000_000
    )
    assert result.gripper_effort_at_sequence_completion_available
    assert result.gripper_effort_at_sequence_completion == -0.75
    assert result.peak_abs_gripper_effort_at_sequence_completion == 0.75
    assert result.table_contact_count == 0
    assert {result.matched_gripper_collision1, result.matched_gripper_collision2} == {
        ARM_CUBE,
        GRIPPER,
    }
    harness.assert_no_motion_interfaces()


def test_one_pad_only_never_verifies_a_two_pad_observer_task(
    harness: PhysicsObserverHarness,
) -> None:
    _, result_future = harness.start_goal()
    harness.baseline()
    harness.publish_contact(
        BASE_NS + 60_000_000,
        ARM_CUBE,
        GRIPPER,
        expected_gripper_count=1,
    )
    harness.publish_sequence(BASE_NS + 70_000_000)
    harness.publish_pose(BASE_NS + 80_000_000, z_m=0.46, expected_count=6)
    for contact_count, offset_ns in enumerate(
        (180_000_000, 280_000_000, 380_000_000, 480_000_000),
        start=2,
    ):
        harness.publish_contact(
            BASE_NS + offset_ns,
            ARM_CUBE,
            GRIPPER,
            expected_gripper_count=contact_count,
        )
        harness.publish_pose(
            BASE_NS + offset_ns,
            z_m=0.46,
            expected_count=5 + contact_count,
        )
    harness.publish_clock(BASE_NS + 2_100_000_000)

    result = harness.result(result_future)
    assert not result.physics_grasp_verified
    assert result.gripper_contact_observed
    assert result.gripper_contact_counts_by_token[0] >= 1
    assert result.gripper_contact_counts_by_token[1] == 0
    assert not result.all_gripper_tokens_observed
    assert result.simultaneous_gripper_contact_sample_count == 0


def test_table_contact_without_gripper_or_sequence_never_verifies(
    harness: PhysicsObserverHarness,
) -> None:
    _, result_future = harness.start_goal()
    harness.baseline()
    harness.publish_contact(
        BASE_NS + 60_000_000,
        ARM_CUBE,
        TABLE,
        expected_table_count=1,
    )
    harness.publish_clock(BASE_NS + 350_000_000)
    result = harness.result(result_future)
    assert not result.physics_grasp_verified
    assert not result.sequence_completed
    assert result.table_contact_count >= 1


def test_palm_contact_does_not_count_as_distal_pad_contact(
    harness: PhysicsObserverHarness,
) -> None:
    _, result_future = harness.start_goal()
    harness.baseline()
    harness.publish_contact(BASE_NS + 60_000_000, ARM_CUBE, PALM)
    harness.publish_clock(BASE_NS + 2_100_000_000)
    result = harness.result(result_future)
    assert not result.physics_grasp_verified
    assert not result.gripper_contact_observed
    assert result.gripper_contact_count == 0


def test_gripper_contact_without_matching_sequence_never_verifies(
    harness: PhysicsObserverHarness,
) -> None:
    _, result_future = harness.start_goal()
    harness.baseline()
    harness.publish_contact(
        BASE_NS + 60_000_000,
        ARM_CUBE,
        GRIPPER,
        expected_gripper_count=1,
    )
    harness.publish_pose(BASE_NS + 70_000_000, z_m=0.46)
    harness.publish_clock(BASE_NS + 400_000_000)
    result = harness.result(result_future)
    assert not result.physics_grasp_verified
    assert not result.sequence_completed
    assert result.gripper_contact_count >= 1


def test_frame_mismatch_is_fail_closed(
    harness: PhysicsObserverHarness,
) -> None:
    _, result_future = harness.start_goal()
    harness.publish_pose(BASE_NS + 10_000_000, frame_id="base_link")
    result = harness.result(result_future)
    assert not result.physics_grasp_verified
    assert result.terminal_phase == "FAULT"
    assert result.reason == "pose_frame_mismatch"


def test_ros_clock_rollback_is_fail_closed(
    harness: PhysicsObserverHarness,
) -> None:
    _, result_future = harness.start_goal()
    harness.publish_pose(BASE_NS + 10_000_000, expected_count=1)
    harness.publish_clock(BASE_NS + 5_000_000)
    result = harness.result(result_future)
    assert not result.physics_grasp_verified
    assert result.terminal_phase == "FAULT"
    assert "rollback" in result.reason


def test_concurrent_ros_clock_reads_do_not_create_a_false_rollback(
    harness: PhysicsObserverHarness,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """An older callback sample must not commit after a newer sample.

    The production node runs under a ``MultiThreadedExecutor``.  This fake clock
    deliberately lets a second caller overtake the first when clock acquisition
    is outside the observer lock.  A serialized implementation returns the two
    samples in order and never latches a synthetic rollback.
    """

    first_entered = threading.Event()
    second_entered = threading.Event()
    call_lock = threading.Lock()
    call_count = 0

    class _Instant:
        def __init__(self, nanoseconds: int) -> None:
            self.nanoseconds = nanoseconds

    class _OvertakingClock:
        def now(self) -> _Instant:
            nonlocal call_count
            with call_lock:
                call_count += 1
                call = call_count
            if call == 1:
                first_entered.set()
                if second_entered.wait(0.05):
                    # Give an implementation that samples outside its lock a
                    # deterministic opportunity to commit the newer value.
                    time.sleep(0.02)
                return _Instant(BASE_NS + 10_000_000)
            second_entered.set()
            return _Instant(BASE_NS + 20_000_000)

    fake_clock = _OvertakingClock()
    monkeypatch.setattr(harness.observer, "get_clock", lambda: fake_clock)
    with harness.observer._lock:
        harness.observer._last_clock_ns = None
        harness.observer._clock_fault = None

    samples: list[int] = []
    first = threading.Thread(target=lambda: samples.append(harness.observer._now_ns()))
    second = threading.Thread(target=lambda: samples.append(harness.observer._now_ns()))
    first.start()
    assert first_entered.wait(1.0)
    second.start()
    first.join(1.0)
    second.join(1.0)

    assert not first.is_alive()
    assert not second.is_alive()
    assert samples == [BASE_NS + 10_000_000, BASE_NS + 20_000_000]
    assert harness.observer._clock_fault is None


def test_small_same_domain_future_pose_waits_for_ros_clock(
    harness: PhysicsObserverHarness,
) -> None:
    goal_handle, result_future = harness.start_goal()
    source_ns = BASE_NS + 10_000_000
    harness.pose.publish(pose_message(source_ns, harness.scene.name))
    wait_until(
        lambda: len(harness.observer._pending_observations) == 1,
        reason="future pose to enter bounded pending queue",
    )
    snapshot = harness._core_snapshot()
    assert snapshot is not None
    assert snapshot.phase == "BASELINE"
    assert snapshot.pose_sample_count == 0

    harness.publish_clock(source_ns)
    wait_until(
        lambda: (
            harness._core_snapshot() is not None
            and harness._core_snapshot().pose_sample_count == 1
        ),
        reason="pending pose after ROS clock catch-up",
    )
    assert harness._core_snapshot().reason != "source_timestamp_in_future"

    cancel_response = wait_future(
        goal_handle.cancel_goal_async(),
        reason="observer cancellation after pending-clock test",
    )
    assert cancel_response.goals_canceling
    result = harness.result(result_future)
    assert not result.physics_grasp_verified
    assert result.reason == "observer_cancelled"


def test_future_pose_beyond_freshness_bound_is_fail_closed(
    harness: PhysicsObserverHarness,
) -> None:
    _, result_future = harness.start_goal()
    source_ns = BASE_NS + FRESHNESS_NS + 1
    harness.pose.publish(pose_message(source_ns, harness.scene.name))
    result = harness.result(result_future)
    assert not result.physics_grasp_verified
    assert result.terminal_phase == "FAULT"
    assert result.reason == "source_timestamp_in_future"
    assert result.pose_sample_count == 0
