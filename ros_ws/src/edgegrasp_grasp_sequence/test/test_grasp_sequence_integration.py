"""ROS-runtime contract tests for the correlated grasp-sequence wrapper.

The fake PlanTarget and ExecuteTrajectory servers model typed terminal results;
they do not prove MoveIt, Gazebo contact, controller physics, or a real grasp.
"""

from __future__ import annotations

import json
import threading
import time
import traceback
from itertools import count
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from edgegrasp.so101_contract import SO101_ARM_JOINTS
from edgegrasp.trajectory_identity import (
    make_trajectory_command_id,
    trajectory_digest,
)
from edgegrasp_interfaces.action import ExecuteTrajectory, GraspSequence, PlanTarget
from edgegrasp_interfaces.msg import TrackedTarget
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from edgegrasp_grasp_sequence.node import GraspSequenceNode


# Keep the complete parametrized module below Fast DDS' high-domain port
# ceiling while remaining disjoint from the other package-scoped test ranges.
DOMAIN_IDS = count(61)


def wait_until(predicate, timeout_s: float = 5.0, reason: str = "condition") -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError(f"timed out waiting for {reason}")


def wait_future(future, timeout_s: float = 5.0):
    wait_until(future.done, timeout_s, "ROS future")
    return future.result()


@pytest.mark.parametrize(
    "stamp",
    [Time(sec=-1, nanosec=0), Time(sec=0, nanosec=1_000_000_000)],
)
def test_non_normalized_or_negative_ros_timestamp_is_rejected(stamp) -> None:
    with pytest.raises(ValueError, match="normalized and non-negative"):
        GraspSequenceNode._stamp_ns(stamp)


@pytest.mark.parametrize(
    "stamp",
    [
        SimpleNamespace(sec=1.5, nanosec=0),
        SimpleNamespace(sec=1, nanosec=1.25),
        SimpleNamespace(sec=1, nanosec=True),
    ],
)
def test_non_integer_ros_timestamp_fields_are_rejected(stamp) -> None:
    with pytest.raises(ValueError, match="integer sec/nanosec"):
        GraspSequenceNode._stamp_ns(stamp)


def point_payload(trajectory) -> list[dict[str, object]]:
    return [
        {
            "positions": point.positions,
            "velocities": point.velocities,
            "accelerations": point.accelerations,
            "effort": point.effort,
            "time_from_start_ns": (
                point.time_from_start.sec * 1_000_000_000
                + point.time_from_start.nanosec
            ),
        }
        for point in trajectory.points
    ]


class FakeSequenceDependencies(Node):
    def __init__(
        self,
        *,
        arm_modes: dict[str, str] | None = None,
        gripper_mode: str = "success",
        create_plan_server: bool = True,
        context: Context,
    ) -> None:
        super().__init__("fake_grasp_sequence_dependencies", context=context)
        self.arm_modes = arm_modes or {}
        self.gripper_mode = gripper_mode
        self.arm_goals: list[PlanTarget.Goal] = []
        self.gripper_goals: list[ExecuteTrajectory.Goal] = []
        self.arm_cancel_requests = 0
        self.gripper_cancel_requests = 0
        self.contact_policy_requests: list[bool] = []
        self.allow_target_pad_contacts = False
        self.arm_started = threading.Event()
        self.release_arm = threading.Event()
        group = ReentrantCallbackGroup()
        self.plan_server = None
        if create_plan_server:
            self.plan_server = ActionServer(
                self,
                PlanTarget,
                "/edgegrasp/plan_target",
                goal_callback=self._accept_arm,
                cancel_callback=self._cancel_arm,
                execute_callback=self._execute_arm,
                callback_group=group,
            )
        self.gate_server = ActionServer(
            self,
            ExecuteTrajectory,
            "/edgegrasp/execute_trajectory",
            goal_callback=self._accept_gripper,
            cancel_callback=self._cancel_gripper,
            execute_callback=self._execute_gripper,
            callback_group=group,
        )
        scene_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.planning_scene_status = self.create_publisher(
            String, "/edgegrasp/planning_scene_status", scene_qos
        )
        self.contact_policy_service = self.create_service(
            SetBool,
            "/edgegrasp/set_target_pad_contacts",
            self._set_contact_policy,
            callback_group=group,
        )
        self._publish_contact_policy_status()

    def _publish_contact_policy_status(
        self, reason: str = "confirmed"
    ) -> None:
        message = String()
        message.data = json.dumps(
            {
                "ready": True,
                "reason": reason,
                "allow_target_pad_contacts": self.allow_target_pad_contacts,
                "scene_digest": "fake-scene-digest",
                "target_frame": "base_link",
                "clock_domain": "ros_system",
                "clock_epoch": 0,
            },
            sort_keys=True,
        )
        self.planning_scene_status.publish(message)

    def _set_contact_policy(self, request, response):
        self.allow_target_pad_contacts = bool(request.data)
        self.contact_policy_requests.append(self.allow_target_pad_contacts)
        self._publish_contact_policy_status()
        response.success = True
        response.message = "confirmed"
        return response

    def _accept_arm(self, request: PlanTarget.Goal) -> GoalResponse:
        self.arm_goals.append(request)
        if self.arm_modes.get(request.stage) == "reject":
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_arm(self, goal_handle) -> CancelResponse:
        del goal_handle
        self.arm_cancel_requests += 1
        return CancelResponse.ACCEPT

    def _execute_arm(self, goal_handle) -> PlanTarget.Result:
        request = goal_handle.request
        mode = self.arm_modes.get(request.stage, "success")
        self.arm_started.set()
        feedback = PlanTarget.Feedback()
        feedback.stage = "executing_through_gate"
        goal_handle.publish_feedback(feedback)
        if mode in ("hold", "hold_ignore_cancel_success"):
            while (
                not goal_handle.is_cancel_requested
                and not self.release_arm.is_set()
            ):
                time.sleep(0.005)
        result = PlanTarget.Result()
        result.task_id = request.task_id
        result.command_id = make_trajectory_command_id(
            request.task_id, request.stage, int(request.sequence_no)
        )
        if mode == "wrong_command":
            result.command_id = make_trajectory_command_id(
                request.task_id, "descend", 99
            )
        result.target_id = request.target_id
        result.stage = request.stage
        result.sequence_no = request.sequence_no
        result.accepted = True
        result.success = True
        result.reason = "trajectory_executed_through_gate"
        result.moveit_error_code = 1
        result.requested_at = request.target_pose.header.stamp
        result.source_timestamp_ns = request.source_timestamp_ns
        result.clock_domain = request.clock_domain
        result.clock_epoch = request.clock_epoch
        result.trajectory_dispatched = True
        result.trajectory_digest = "a" * 64
        result.gate_accepted = True
        result.gate_terminal = True
        result.downstream_terminal_observed = True
        result.cancel_requested = bool(goal_handle.is_cancel_requested)
        result.action_goal_status = GoalStatus.STATUS_SUCCEEDED
        if mode == "unknown_status":
            result.action_goal_status = GoalStatus.STATUS_UNKNOWN
        elif mode == "dispatch_without_gate_acceptance":
            result.gate_accepted = False
        result.fjt_error_code = 0
        result.fjt_error_string = ""
        if goal_handle.is_cancel_requested and mode != "hold_ignore_cancel_success":
            result.success = False
            result.reason = "canceled"
            result.action_goal_status = GoalStatus.STATUS_CANCELED
            goal_handle.canceled()
        elif mode == "fjt_error":
            result.success = False
            result.reason = "trajectory_gate:fjt_error:-4"
            result.fjt_error_code = -4
            result.fjt_error_string = "synthetic path tolerance violation"
            goal_handle.abort()
        elif mode == "outer_abort_success":
            goal_handle.abort()
        else:
            goal_handle.succeed()
        return result

    def _accept_gripper(self, request: ExecuteTrajectory.Goal) -> GoalResponse:
        self.gripper_goals.append(request)
        if self.gripper_mode == "reject":
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_gripper(self, goal_handle) -> CancelResponse:
        del goal_handle
        self.gripper_cancel_requests += 1
        return CancelResponse.ACCEPT

    def _execute_gripper(self, goal_handle) -> ExecuteTrajectory.Result:
        request = goal_handle.request
        result = ExecuteTrajectory.Result()
        result.task_id = request.task_id
        result.command_id = request.command_id
        result.target_id = request.target_id
        result.stage = request.stage
        result.sequence_no = request.sequence_no
        result.controller = request.controller
        result.trajectory_digest = trajectory_digest(
            request.controller,
            request.trajectory.joint_names,
            point_payload(request.trajectory),
        )
        if self.gripper_mode == "wrong_digest":
            result.trajectory_digest = "b" * 64
        elif self.gripper_mode == "wrong_command":
            result.command_id = make_trajectory_command_id(
                request.task_id, "lift", 99
            )
        result.accepted = True
        result.terminal = True
        result.downstream_terminal_observed = True
        result.cancel_requested = bool(goal_handle.is_cancel_requested)
        result.action_goal_status = GoalStatus.STATUS_SUCCEEDED
        if self.gripper_mode == "unknown_status":
            result.action_goal_status = GoalStatus.STATUS_UNKNOWN
        result.fjt_error_code = 0
        result.fjt_error_string = ""
        result.success = True
        result.reason = "succeeded"
        result.source_timestamp_ns = request.source_timestamp_ns
        result.completed_timestamp_ns = self.get_clock().now().nanoseconds
        result.clock_domain = request.clock_domain
        result.clock_epoch = request.clock_epoch
        if goal_handle.is_cancel_requested:
            result.success = False
            result.reason = "canceled"
            result.action_goal_status = GoalStatus.STATUS_CANCELED
            goal_handle.canceled()
        elif self.gripper_mode == "fjt_error":
            result.success = False
            result.reason = "fjt_error:-4"
            result.fjt_error_code = -4
            result.fjt_error_string = "synthetic path tolerance violation"
            goal_handle.abort()
        elif self.gripper_mode == "outer_abort_success":
            goal_handle.abort()
        else:
            goal_handle.succeed()
        return result

    def close(self) -> None:
        self.release_arm.set()
        if self.plan_server is not None:
            self.plan_server.destroy()
        self.gate_server.destroy()
        self.destroy_service(self.contact_policy_service)


class SequenceHarness:
    def __init__(
        self,
        *,
        arm_modes: dict[str, str] | None = None,
        gripper_mode: str = "success",
        create_plan_server: bool = True,
    ) -> None:
        self.context = Context()
        rclpy.init(context=self.context, domain_id=next(DOMAIN_IDS))
        self.dependencies = FakeSequenceDependencies(
            arm_modes=arm_modes,
            gripper_mode=gripper_mode,
            create_plan_server=create_plan_server,
            context=self.context,
        )
        self.sequence = GraspSequenceNode(
            context=self.context,
            parameter_overrides=[
                Parameter("use_sim_time", value=False),
                Parameter("clock_domain", value="ros_system"),
                # Package-scoped colcon can run this graph concurrently with
                # the ROS and MoveIt fake-action suites.  Keep these bounded
                # test-only liveness windows above scheduler contention so a
                # terminal-contract case cannot be misclassified as a health
                # timeout before its synthetic result callback is inspected.
                Parameter("source_timeout_ms", value=5_000.0),
                Parameter("health_timeout_ms", value=5_000.0),
                Parameter("permission_timeout_ms", value=5_000.0),
                Parameter("interface_timeout_ms", value=5_000.0),
                Parameter("target_receive_timeout_ms", value=5_000.0),
                Parameter("joint_state_timeout_ms", value=5_000.0),
                Parameter("command_timeout_ms", value=10_000.0),
            ],
        )
        self.client = Node("grasp_sequence_test_client", context=self.context)
        self.action = ActionClient(
            self.client, GraspSequence, "/edgegrasp/grasp_sequence"
        )
        self.target_pub = self.client.create_publisher(
            TrackedTarget, "/edgegrasp/tracked_target", 10
        )
        self.permission_pub = self.client.create_publisher(
            Bool, "/edgegrasp/motion_allowed", 10
        )
        self.interface_pub = self.client.create_publisher(
            Bool, "/edgegrasp/interface_ready", 10
        )
        self.joints_pub = self.client.create_publisher(
            JointState, "/joint_states", 10
        )
        self.target_position = [0.2, 0.0, 0.425]
        self.latest_target: TrackedTarget | None = None
        self.feedback: list[GraspSequence.Feedback] = []
        self.timer = self.client.create_timer(0.02, self.publish_inputs)
        self.executor = MultiThreadedExecutor(num_threads=8, context=self.context)
        for node in (self.dependencies, self.sequence, self.client):
            self.executor.add_node(node)
        self._closing = False
        self._spin_error: Exception | None = None
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()
        wait_until(self.action.server_is_ready, reason="GraspSequence server")
        wait_until(
            self.sequence._target_pad_contacts.service_is_ready,
            reason="target-pad contact policy service",
        )
        if create_plan_server:
            wait_until(
                self.sequence._arm_client.server_is_ready,
                reason="PlanTarget client/server discovery",
            )
        wait_until(
            self.sequence._gripper_client.server_is_ready,
            reason="ExecuteTrajectory client/server discovery",
        )
        wait_until(
            lambda: self.target_pub.get_subscription_count() == 1,
            reason="tracked-target subscription",
        )
        wait_until(
            lambda: self.permission_pub.get_subscription_count() == 1,
            reason="permission subscription",
        )
        wait_until(
            lambda: self.interface_pub.get_subscription_count() == 1,
            reason="interface subscription",
        )
        wait_until(
            lambda: self.joints_pub.get_subscription_count() == 1,
            reason="joint-state subscription",
        )
        wait_until(
            lambda: self.sequence._latest_target is not None,
            reason="fresh tracked target",
        )
        wait_until(
            lambda: self.sequence._planning_scene_policy_reason(
                allow=False,
                now_ns=self.sequence.get_clock().now().nanoseconds,
            )
            is None,
            reason="initial target-pad contact policy",
        )

    def _spin(self) -> None:
        try:
            self.executor.spin()
        except Exception as error:
            if not self._closing:
                self._spin_error = error

    def publish_inputs(self) -> None:
        now = self.client.get_clock().now()
        target = TrackedTarget()
        target.target_id = "cube-1"
        target.observation.header.frame_id = "base_link"
        target.observation.header.stamp = now.to_msg()
        target.observation.point.x = self.target_position[0]
        target.observation.point.y = self.target_position[1]
        target.observation.point.z = self.target_position[2]
        target.clock_domain = "ros_system"
        target.clock_epoch = 0
        self.latest_target = target
        self.target_pub.publish(target)
        self.permission_pub.publish(Bool(data=True))
        self.interface_pub.publish(Bool(data=True))
        self.joints_pub.publish(
            JointState(
                name=list(SO101_ARM_JOINTS) + ["gripper"],
                position=[0.0] * 6,
            )
        )

    @staticmethod
    def _copy_target(source: TrackedTarget) -> TrackedTarget:
        target = TrackedTarget()
        target.target_id = source.target_id
        target.observation.header.frame_id = source.observation.header.frame_id
        target.observation.header.stamp = source.observation.header.stamp
        target.observation.point.x = source.observation.point.x
        target.observation.point.y = source.observation.point.y
        target.observation.point.z = source.observation.point.z
        target.clock_domain = source.clock_domain
        target.clock_epoch = source.clock_epoch
        return target

    def goal(self, task_id: str = "pick-cube-1") -> GraspSequence.Goal:
        wait_until(lambda: self.latest_target is not None, reason="goal target")
        assert self.latest_target is not None
        goal = GraspSequence.Goal()
        goal.task_id = task_id
        goal.target = self._copy_target(self.latest_target)
        goal.approach_position.x = 0.2
        goal.approach_position.z = 0.52
        goal.descend_position.x = 0.2
        goal.descend_position.z = 0.46
        goal.lift_position.x = 0.2
        goal.lift_position.z = 0.56
        goal.approach_orientation.w = 1.0
        goal.grasp_orientation.y = 0.7071067811865476
        goal.grasp_orientation.w = 0.7071067811865476
        goal.gripper_closed_position_rad = 0.2
        goal.pipeline_id = "ompl"
        goal.planner_id = ""
        goal.planning_timeout_s = 0.2
        goal.velocity_scaling = 0.1
        goal.acceleration_scaling = 0.1
        return goal

    def send(self, goal: GraspSequence.Goal | None = None):
        return wait_future(
            self.action.send_goal_async(
                goal or self.goal(), feedback_callback=self._on_feedback
            )
        )

    def _on_feedback(self, message) -> None:
        self.feedback.append(message.feedback)

    def close(self) -> None:
        self._closing = True
        self.dependencies.release_arm.set()
        self.timer.cancel()
        with self.sequence._state_lock:
            retained_by_contract = bool(self.sequence._uncertain_commands)
        if not retained_by_contract:
            wait_until(
                lambda: self.sequence._arm_slot is None
                and self.sequence._gripper_slot is None,
                timeout_s=2.0,
                reason="sequence action callbacks to drain",
            )
        else:
            # A deliberately unavailable/broken downstream path may retain a
            # stop-uncertain slot. The test assertion owns that evidence; the
            # teardown still shuts the isolated ROS context down safely.
            with self.sequence._state_lock:
                retained_slots = (
                    self.sequence._arm_slot,
                    self.sequence._gripper_slot,
                )
                self.sequence._arm_slot = None
                self.sequence._gripper_slot = None
                self.sequence._uncertain_commands.clear()
            for slot in retained_slots:
                if slot is None:
                    continue
                for future in (
                    slot.send_future,
                    slot.result_future,
                    slot.cancel_future,
                ):
                    if future is None:
                        continue
                    try:
                        if future.done():
                            future.exception()
                        else:
                            future.cancel()
                    except Exception:
                        pass
            # Let already-queued callbacks observe the cleared test-only slots
            # before the isolated executor and action clients are destroyed.
            time.sleep(0.05)
        def executor_is_quiescent() -> bool:
            with self.executor._tasks_lock:
                active_tasks = any(
                    not task.done() and not task.cancelled()
                    for task in self.executor._pending_tasks
                )
                ready_tasks = bool(self.executor._ready_tasks)
            active_workers = any(
                not future.done() for future in self.executor._futures
            )
            return not active_tasks and not ready_tasks and not active_workers

        # Clearing the last sequence slot happens inside a client result
        # callback, slightly before the corresponding ActionServer coroutine
        # itself returns.  Wait for that remaining server work before stopping
        # the executor; otherwise an already-queued waitable can run against a
        # handle whose destruction has begun.
        wait_until(
            executor_is_quiescent,
            timeout_s=2.0,
            reason="ROS action executor to become quiescent",
        )
        self.executor.shutdown(timeout_sec=5.0)
        self.thread.join(timeout=5.0)
        assert not self.thread.is_alive(), "test executor did not stop"
        # Jazzy's MultiThreadedExecutor.shutdown() stops the ROS spin loop but
        # does not join its internal ThreadPoolExecutor.  A worker that reaches
        # an Action waitable after node destruction otherwise leaves an
        # unfetched rclpy Future exception ("cannot use Destroyable ...").
        # This is test-harness teardown only; keep the nodes alive until every
        # already-submitted callback has returned.
        self.executor._executor.shutdown(wait=True, cancel_futures=False)
        with self.executor._tasks_lock:
            completed_tasks = tuple(self.executor._pending_tasks)
        task_errors = []
        for task in completed_tasks:
            if task.done() and not task.cancelled():
                error = task.exception()
                if error is not None:
                    task_errors.append(error)
        self.dependencies.close()
        for node in (self.sequence, self.dependencies, self.client):
            node.destroy_node()
        rclpy.shutdown(context=self.context)
        if task_errors:
            raise AssertionError(
                "executor callback errors during teardown: "
                + "\n".join(
                    "".join(
                        traceback.format_exception(
                            type(error), error, error.__traceback__
                        )
                    )
                    for error in task_errors
                )
            )
        if self._spin_error is not None:
            raise self._spin_error
        if self.sequence._last_unhandled_exception is not None:
            raise AssertionError(self.sequence._last_unhandled_exception)


@pytest.fixture
def harness():
    instance = SequenceHarness()
    try:
        yield instance
    finally:
        instance.close()


def test_four_correlated_actions_complete_without_physics_claim(harness) -> None:
    goal_handle = harness.send()
    assert goal_handle.accepted
    wrapped = wait_future(goal_handle.get_result_async())

    assert wrapped.status == GoalStatus.STATUS_SUCCEEDED, (
        f"status={wrapped.status};phase={wrapped.result.terminal_phase};"
        f"reason={wrapped.result.reason};"
        f"active={wrapped.result.active_command_id};"
        f"last={wrapped.result.last_terminal_command_id};"
        f"feedback={[(item.phase, item.reason) for item in harness.feedback]};"
        f"uncertain={sorted(harness.sequence._uncertain_commands)}"
    )
    assert wrapped.result.accepted
    assert wrapped.result.sequence_completed
    assert not wrapped.result.physics_grasp_verified
    assert wrapped.result.terminal_phase == "COMPLETE"
    assert wrapped.result.active_command_id == ""
    assert wrapped.result.last_terminal_command_id == "pick-cube-1|lift|3"
    assert wrapped.result.last_trajectory_digest == "a" * 64
    assert [goal.stage for goal in harness.dependencies.arm_goals] == [
        "approach",
        "descend",
        "lift",
    ]
    assert [goal.sequence_no for goal in harness.dependencies.arm_goals] == [0, 1, 3]
    assert harness.dependencies.contact_policy_requests == [True]
    orientations = [
        (
            goal.target_pose.pose.orientation.x,
            goal.target_pose.pose.orientation.y,
            goal.target_pose.pose.orientation.z,
            goal.target_pose.pose.orientation.w,
        )
        for goal in harness.dependencies.arm_goals
    ]
    assert orientations == [
        (0.0, 0.0, 0.0, 1.0),
        (0.0, 0.7071067811865476, 0.0, 0.7071067811865476)
    ] + [(0.0, 0.7071067811865476, 0.0, 0.7071067811865476)]
    assert len(harness.dependencies.gripper_goals) == 1
    gripper = harness.dependencies.gripper_goals[0]
    assert gripper.stage == "close_gripper" and gripper.sequence_no == 2
    assert gripper.controller == "gripper_controller"
    assert list(gripper.trajectory.joint_names) == ["gripper"]
    assert [feedback.phase for feedback in harness.feedback][-1] == "COMPLETE"


def test_fresh_confirmed_revalidation_pending_keeps_scene_admissible(
    harness,
) -> None:
    harness.dependencies._publish_contact_policy_status(
        "confirmed_revalidation_pending"
    )
    wait_until(
        lambda: (
            harness.sequence._planning_scene_status is not None
            and harness.sequence._planning_scene_status.reason
            == "confirmed_revalidation_pending"
        ),
        reason="pending planning-scene revalidation status",
    )
    assert harness.sequence._planning_scene_policy_reason(
        allow=False,
        now_ns=harness.sequence.get_clock().now().nanoseconds,
    ) is None


def test_pinned_pilz_pipeline_and_ptp_planner_are_forwarded(harness) -> None:
    goal = harness.goal(task_id="pick-cube-pilz")
    goal.pipeline_id = "pilz_industrial_motion_planner"
    goal.planner_id = "PTP"

    goal_handle = harness.send(goal)
    assert goal_handle.accepted
    wrapped = wait_future(goal_handle.get_result_async())

    assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
    assert wrapped.result.sequence_completed
    assert [item.pipeline_id for item in harness.dependencies.arm_goals] == [
        "pilz_industrial_motion_planner"
    ] * 3
    assert [item.planner_id for item in harness.dependencies.arm_goals] == [
        "PTP"
    ] * 3


def test_unpinned_planning_pipeline_is_rejected_before_dispatch(harness) -> None:
    goal = harness.goal(task_id="pick-cube-unpinned")
    goal.pipeline_id = "pilz"

    goal_handle = harness.send(goal)

    assert not goal_handle.accepted
    assert harness.dependencies.arm_goals == []
    assert harness.dependencies.gripper_goals == []


@pytest.mark.parametrize(
    "mode",
    [
        "fjt_error",
        "outer_abort_success",
        "wrong_command",
        "unknown_status",
        "dispatch_without_gate_acceptance",
    ],
)
def test_arm_terminal_failure_never_dispatches_descend(mode: str) -> None:
    harness = SequenceHarness(arm_modes={"approach": mode})
    try:
        goal_handle = harness.send()
        wrapped = wait_future(goal_handle.get_result_async())
        assert wrapped.status == GoalStatus.STATUS_ABORTED
        assert not wrapped.result.sequence_completed
        assert not wrapped.result.physics_grasp_verified
        assert wrapped.result.terminal_phase == "SAFE_STOP"
        assert len(harness.dependencies.arm_goals) == 1
        assert harness.dependencies.gripper_goals == []
        if mode in (
            "wrong_command",
            "unknown_status",
            "dispatch_without_gate_acceptance",
        ):
            assert harness.sequence._arm_slot is not None
            command_id = harness.sequence._arm_slot.command.command_id
            assert command_id in harness.sequence._uncertain_commands
            reset = harness.sequence._on_reset(
                Trigger.Request(), Trigger.Response()
            )
            assert not reset.success
    finally:
        harness.close()


@pytest.mark.parametrize(
    "mode", ["fjt_error", "outer_abort_success", "unknown_status"]
)
def test_gripper_terminal_failure_never_dispatches_lift(mode: str) -> None:
    harness = SequenceHarness(gripper_mode=mode)
    try:
        goal_handle = harness.send()
        wrapped = wait_future(goal_handle.get_result_async())
        assert wrapped.status == GoalStatus.STATUS_ABORTED
        assert wrapped.result.terminal_phase == "SAFE_STOP"
        assert [goal.stage for goal in harness.dependencies.arm_goals] == [
            "approach",
            "descend",
        ]
        assert len(harness.dependencies.gripper_goals) == 1
    finally:
        harness.close()


@pytest.mark.parametrize("mode", ["wrong_digest", "wrong_command"])
def test_untrusted_gripper_terminal_retains_slot_and_blocks_reset(mode: str) -> None:
    harness = SequenceHarness(gripper_mode=mode)
    try:
        goal_handle = harness.send()
        wrapped = wait_future(goal_handle.get_result_async())

        assert wrapped.status == GoalStatus.STATUS_ABORTED
        assert wrapped.result.terminal_phase == "SAFE_STOP"
        assert [goal.stage for goal in harness.dependencies.arm_goals] == [
            "approach",
            "descend",
        ]
        assert len(harness.dependencies.gripper_goals) == 1
        assert harness.sequence._gripper_slot is not None
        command_id = harness.sequence._gripper_slot.command.command_id
        assert command_id in harness.sequence._uncertain_commands
        reset = harness.sequence._on_reset(Trigger.Request(), Trigger.Response())
        assert not reset.success
    finally:
        harness.close()


def test_target_drift_during_arm_stage_cancels_and_blocks_followup() -> None:
    harness = SequenceHarness(arm_modes={"approach": "hold"})
    try:
        goal_handle = harness.send()
        assert goal_handle.accepted
        wait_until(harness.dependencies.arm_started.is_set, reason="held arm stage")
        harness.target_position[0] += 0.02
        wrapped = wait_future(goal_handle.get_result_async())
        assert wrapped.status in (
            GoalStatus.STATUS_ABORTED,
            GoalStatus.STATUS_CANCELED,
        )
        assert wrapped.result.terminal_phase == "SAFE_STOP"
        wait_until(
            lambda: harness.dependencies.arm_cancel_requests == 1,
            reason="PlanTarget cancel",
        )
        assert len(harness.dependencies.arm_goals) == 1
        assert harness.dependencies.gripper_goals == []
    finally:
        harness.close()


def test_concurrent_sequence_goal_is_rejected() -> None:
    harness = SequenceHarness(arm_modes={"approach": "hold"})
    try:
        first = harness.send(harness.goal("task-first"))
        assert first.accepted
        wait_until(harness.dependencies.arm_started.is_set, reason="held first stage")
        second = harness.send(harness.goal("task-second"))
        assert not second.accepted
        cancel = wait_future(first.cancel_goal_async())
        assert cancel.goals_canceling
        wait_future(first.get_result_async())
    finally:
        harness.close()


@pytest.mark.parametrize("field", ("approach_orientation", "grasp_orientation"))
@pytest.mark.parametrize(
    "orientation",
    (
        (0.0, 0.0, 0.0, 0.0),
        (0.0, 0.5, 0.0, 0.5),
        (float("nan"), 0.0, 0.0, 1.0),
    ),
)
def test_invalid_task_orientation_is_rejected_before_planning(
    field, orientation
) -> None:
    harness = SequenceHarness()
    try:
        goal = harness.goal("invalid-orientation")
        target = getattr(goal, field)
        target.x, target.y, target.z, target.w = orientation
        goal_handle = harness.send(goal)

        assert not goal_handle.accepted
        assert harness.dependencies.arm_goals == []
        assert harness.dependencies.gripper_goals == []
    finally:
        harness.close()


def test_client_cancel_wins_race_with_late_arm_success() -> None:
    harness = SequenceHarness(
        arm_modes={"approach": "hold_ignore_cancel_success"}
    )
    try:
        goal_handle = harness.send()
        assert goal_handle.accepted
        wait_until(harness.dependencies.arm_started.is_set, reason="held arm stage")

        cancel = wait_future(goal_handle.cancel_goal_async())
        assert cancel.goals_canceling
        wrapped = wait_future(goal_handle.get_result_async())

        assert wrapped.status in (
            GoalStatus.STATUS_ABORTED,
            GoalStatus.STATUS_CANCELED,
        )
        assert wrapped.result.terminal_phase == "SAFE_STOP"
        assert [goal.stage for goal in harness.dependencies.arm_goals] == [
            "approach"
        ]
        assert harness.dependencies.gripper_goals == []
    finally:
        harness.close()


def test_missing_plan_target_server_fails_before_any_gripper_command() -> None:
    harness = SequenceHarness(create_plan_server=False)
    try:
        goal_handle = harness.send()
        assert goal_handle.accepted
        wrapped = wait_future(goal_handle.get_result_async())
        assert wrapped.status == GoalStatus.STATUS_ABORTED
        assert wrapped.result.terminal_phase == "SAFE_STOP"
        assert not wrapped.result.sequence_completed
        assert harness.dependencies.gripper_goals == []
    finally:
        harness.close()
