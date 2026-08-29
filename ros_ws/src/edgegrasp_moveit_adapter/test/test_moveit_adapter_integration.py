"""ROS-runtime tests for the plan-only MoveIt-to-gate boundary.

These tests use fake ``/compute_ik`` and ``/move_action`` servers.  They prove
the adapter's ROS contract and fail-closed publication behavior; they do not
prove Gazebo physics, the upstream kinematics plugin, or physical stopping.
"""

from __future__ import annotations

import json
import threading
import time
from itertools import count

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration, Time
from edgegrasp.so101_contract import SO101_ARM_JOINTS
from edgegrasp.trajectory_identity import trajectory_digest
from edgegrasp_interfaces.action import ExecuteTrajectory, PlanTarget
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from edgegrasp_moveit_adapter.adapter_node import MoveItPlanOnlyAdapter


START = (0.0, 0.0, 0.0, 0.0, 0.0)
IK_GOAL = (0.05, -0.05, 0.05, -0.05, 0.0)
DOMAIN_IDS = count(151)


def wait_until(predicate, timeout_s: float = 3.0, reason: str = "condition") -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError(f"timed out waiting for {reason}")


def wait_future(future, timeout_s: float = 3.0):
    wait_until(future.done, timeout_s, "ROS future")
    return future.result()


class FakeMoveIt(Node):
    def __init__(
        self,
        *,
        ik_error: int = MoveItErrorCodes.SUCCESS,
        ik_delay_s: float = 0.0,
        move_group_mode: str = "success",
        gate_mode: str = "success",
        cancel_response: CancelResponse = CancelResponse.ACCEPT,
        publish_gate_subscriber: bool = True,
        context: Context,
    ) -> None:
        super().__init__(
            "fake_edgegrasp_moveit_dependencies",
            context=context,
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.ik_error = ik_error
        self.ik_delay_s = ik_delay_s
        self.move_group_mode = move_group_mode
        self.gate_mode = gate_mode
        self.cancel_response = cancel_response
        self.move_group_goals: list[MoveGroup.Goal] = []
        self.cancel_requests = 0
        self.gate_cancel_requests = 0
        self.published_trajectories: list[JointTrajectory] = []
        self.gate_goals: list[ExecuteTrajectory.Goal] = []
        self.contact_policy_requests: list[bool] = []
        self.goal_started = threading.Event()
        self.release_goal = threading.Event()
        self.gate_goal_started = threading.Event()
        self.release_gate = threading.Event()
        group = ReentrantCallbackGroup()
        self.ik_service = self.create_service(
            GetPositionIK,
            "/compute_ik",
            self._compute_ik,
            callback_group=group,
        )
        scene_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.planning_scene_ready = self.create_publisher(
            Bool, "/edgegrasp/planning_scene_ready", scene_qos
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
        self.move_group_server = ActionServer(
            self,
            MoveGroup,
            "/move_action",
            execute_callback=self._execute_move_group,
            goal_callback=self._accept_move_group,
            cancel_callback=self._cancel_move_group,
            callback_group=group,
        )
        self.gate_server = None
        if publish_gate_subscriber:
            self.gate_server = ActionServer(
                self,
                ExecuteTrajectory,
                "/edgegrasp/execute_trajectory",
                execute_callback=self._execute_gate,
                goal_callback=self._accept_gate,
                cancel_callback=self._cancel_gate,
                callback_group=group,
            )

    def _set_contact_policy(self, request, response):
        allow = bool(request.data)
        self.contact_policy_requests.append(allow)
        self.planning_scene_ready.publish(Bool(data=True))
        status = String()
        status.data = json.dumps(
            {
                "ready": True,
                "reason": "confirmed",
                "allow_target_pad_contacts": allow,
                "scene_digest": "fake-scene-digest",
                "target_frame": "base_link",
                "clock_domain": "ros_sim",
                "clock_epoch": 0,
            },
            sort_keys=True,
        )
        self.planning_scene_status.publish(status)
        response.success = True
        response.message = "confirmed"
        return response

    def _accept_gate(self, request) -> GoalResponse:
        self.gate_goals.append(request)
        self.published_trajectories.append(request.trajectory)
        return GoalResponse.ACCEPT

    def _execute_gate(self, goal_handle):
        request = goal_handle.request
        self.gate_goal_started.set()
        if self.gate_mode == "hold":
            while (
                not goal_handle.is_cancel_requested
                and not self.release_gate.is_set()
            ):
                time.sleep(0.005)
        payload = [
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
            for point in request.trajectory.points
        ]
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
            payload,
        )
        result.accepted = True
        result.terminal = True
        result.downstream_terminal_observed = True
        result.cancel_requested = bool(goal_handle.is_cancel_requested)
        if goal_handle.is_cancel_requested:
            result.action_goal_status = GoalStatus.STATUS_CANCELED
            result.fjt_error_code = 0
            result.fjt_error_string = ""
            result.success = False
            result.reason = "canceled:adapter_requested"
        elif self.gate_mode == "fjt_error":
            result.action_goal_status = GoalStatus.STATUS_SUCCEEDED
            result.fjt_error_code = -4
            result.fjt_error_string = "synthetic path tolerance violation"
            result.success = False
            result.reason = "fjt_error:-4"
        else:
            result.action_goal_status = GoalStatus.STATUS_SUCCEEDED
            result.fjt_error_code = 0
            result.fjt_error_string = ""
            result.success = True
            result.reason = "succeeded"
        result.source_timestamp_ns = request.source_timestamp_ns
        if self.gate_mode == "source_mismatch":
            result.source_timestamp_ns += 1
        result.completed_timestamp_ns = self.get_clock().now().nanoseconds
        result.clock_domain = request.clock_domain
        result.clock_epoch = request.clock_epoch
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _cancel_gate(self, goal_handle) -> CancelResponse:
        del goal_handle
        self.gate_cancel_requests += 1
        return CancelResponse.ACCEPT

    def _compute_ik(self, request, response):
        assert request.ik_request.group_name == "arm"
        assert request.ik_request.ik_link_name == "gripper_frame_link"
        assert request.ik_request.avoid_collisions
        assert tuple(request.ik_request.robot_state.joint_state.name) == (
            SO101_ARM_JOINTS
        )
        if self.ik_delay_s > 0.0:
            time.sleep(self.ik_delay_s)
        response.error_code.val = self.ik_error
        if self.ik_error == MoveItErrorCodes.SUCCESS:
            response.solution.joint_state.name = list(SO101_ARM_JOINTS) + [
                "gripper"
            ]
            response.solution.joint_state.position = list(IK_GOAL) + [0.0]
        return response

    def _accept_move_group(self, request) -> GoalResponse:
        self.move_group_goals.append(request)
        if self.move_group_mode == "reject":
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_move_group(self, goal_handle) -> CancelResponse:
        del goal_handle
        self.cancel_requests += 1
        return self.cancel_response

    def _execute_move_group(self, goal_handle):
        self.goal_started.set()
        if self.move_group_mode == "hold":
            while (
                not goal_handle.is_cancel_requested
                and not self.release_goal.is_set()
            ):
                time.sleep(0.005)
        result = MoveGroup.Result()
        result.error_code.val = MoveItErrorCodes.SUCCESS
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return result
        if self.move_group_mode == "abort":
            result.error_code.val = MoveItErrorCodes.PLANNING_FAILED
            goal_handle.abort()
            return result
        source = goal_handle.request.request.start_state.joint_state
        trajectory = result.planned_trajectory.joint_trajectory
        trajectory.joint_names = list(SO101_ARM_JOINTS)
        start = JointTrajectoryPoint()
        start.positions = list(source.position)
        start.time_from_start = Duration(sec=0)
        finish = JointTrajectoryPoint()
        finish.positions = list(IK_GOAL)
        finish.time_from_start = Duration(sec=1)
        trajectory.points = [start, finish]
        goal_handle.succeed()
        return result

    def close(self) -> None:
        self.release_gate.set()
        self.move_group_server.destroy()
        if self.gate_server is not None:
            self.gate_server.destroy()
        self.destroy_service(self.contact_policy_service)
        self.destroy_service(self.ik_service)


class AdapterHarness:
    def __init__(
        self,
        *,
        ik_error: int = MoveItErrorCodes.SUCCESS,
        move_group_mode: str = "success",
        gate_mode: str = "success",
        publish_gate_subscriber: bool = True,
        target_timeout_ms: float = 2_000.0,
        ik_delay_s: float = 0.0,
        ik_response_timeout_ms: float = 200.0,
    ) -> None:
        self.context = Context()
        rclpy.init(context=self.context, domain_id=next(DOMAIN_IDS))
        self._clock_ns = 1_000_000_000
        self.dependencies = FakeMoveIt(
            ik_error=ik_error,
            ik_delay_s=ik_delay_s,
            move_group_mode=move_group_mode,
            gate_mode=gate_mode,
            publish_gate_subscriber=publish_gate_subscriber,
            context=self.context,
        )
        self.adapter = MoveItPlanOnlyAdapter(
            context=self.context,
            parameter_overrides=[
                Parameter("use_sim_time", value=True),
                Parameter("clock_domain", value="ros_sim"),
                Parameter("permission_timeout_ms", value=2_000.0),
                Parameter("interface_timeout_ms", value=2_000.0),
                Parameter("planning_scene_timeout_ms", value=2_000.0),
                Parameter("joint_state_timeout_ms", value=2_000.0),
                Parameter("target_freshness_timeout_ms", value=target_timeout_ms),
                Parameter("ik_service_discovery_timeout_ms", value=100.0),
                Parameter(
                    "ik_response_timeout_ms", value=ik_response_timeout_ms
                ),
                Parameter("move_group_discovery_timeout_ms", value=100.0),
                Parameter("goal_response_timeout_ms", value=200.0),
                Parameter("cancel_response_timeout_ms", value=200.0),
                Parameter("result_timeout_margin_ms", value=200.0),
            ]
        )
        self.client = Node(
            "edgegrasp_moveit_adapter_test_client",
            context=self.context,
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.plan_client = ActionClient(
            self.client, PlanTarget, "/edgegrasp/plan_target"
        )
        self.permission = self.client.create_publisher(
            Bool, "/edgegrasp/motion_allowed", 10
        )
        self.interface = self.client.create_publisher(
            Bool, "/edgegrasp/interface_ready", 10
        )
        scene_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.planning_scene = self.client.create_publisher(
            Bool, "/edgegrasp/planning_scene_ready", scene_qos
        )
        self.joints = self.client.create_publisher(
            JointState, "/joint_states", 10
        )
        self.clock = self.client.create_publisher(Clock, "/clock", 10)
        self.executor = MultiThreadedExecutor(
            num_threads=6, context=self.context
        )
        for node in (self.dependencies, self.adapter, self.client):
            self.executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        wait_until(self.plan_client.server_is_ready, reason="PlanTarget server")
        wait_until(
            self.adapter._target_pad_contacts.service_is_ready,
            reason="target-pad contact policy service",
        )
        if publish_gate_subscriber:
            wait_until(
                self.adapter._trajectory_gate.server_is_ready,
                reason="ExecuteTrajectory server",
            )
        wait_until(
            lambda: self.permission.get_subscription_count() == 1,
            reason="permission subscription",
        )
        wait_until(
            lambda: self.interface.get_subscription_count() == 1,
            reason="interface subscription",
        )
        wait_until(
            lambda: self.planning_scene.get_subscription_count() == 1,
            reason="planning-scene subscription",
        )
        wait_until(
            lambda: self.joints.get_subscription_count() == 1,
            reason="joint-state subscription",
        )
        wait_until(
            lambda: self.clock.get_subscription_count() >= 3,
            reason="simulation clock subscriptions",
        )
        self.publish_clock(self._clock_ns)

    def publish_clock(self, value_ns: int) -> None:
        self._clock_ns = value_ns
        message = Clock()
        message.clock = Time(
            sec=value_ns // 1_000_000_000,
            nanosec=value_ns % 1_000_000_000,
        )

        def observed() -> bool:
            self.clock.publish(message)
            return all(
                node.get_clock().now().nanoseconds == value_ns
                for node in (self.dependencies, self.adapter, self.client)
            )

        wait_until(observed, reason=f"simulation clock {value_ns}")

    def advance_clock(self, delta_ns: int) -> None:
        if delta_ns <= 0:
            raise ValueError("simulation clock delta must be positive")
        self.publish_clock(self._clock_ns + delta_ns)

    def publish_inputs(self, *, planning_scene_ready: bool | None = True) -> None:
        self.publish_clock(self._clock_ns)
        self.permission.publish(Bool(data=True))
        self.interface.publish(Bool(data=True))
        if planning_scene_ready is not None:
            self.planning_scene.publish(Bool(data=planning_scene_ready))
        self.joints.publish(
            JointState(
                name=list(SO101_ARM_JOINTS) + ["gripper"],
                position=list(START) + [0.0],
            )
        )
        wait_until(
            lambda: self.adapter._permission.evaluate(
                self.adapter.get_clock().now().nanoseconds
            ).allowed
            and self.adapter._interface_permission.evaluate(
                self.adapter.get_clock().now().nanoseconds
            ).allowed
            and self.adapter._last_joint_receive_ns is not None
            and (
                planning_scene_ready is None
                or self.adapter._planning_scene_permission.evaluate(
                    self.adapter.get_clock().now().nanoseconds
                ).allowed
                is planning_scene_ready
            ),
            reason="fresh adapter inputs",
        )

    def goal(self, target_id: str = "test-target") -> PlanTarget.Goal:
        goal = PlanTarget.Goal()
        now = self.client.get_clock().now()
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.header.stamp = now.to_msg()
        goal.target_pose.pose.position.x = 0.3
        goal.target_pose.pose.position.z = 0.2
        goal.target_pose.pose.orientation.w = 1.0
        goal.task_id = f"task-{target_id}"
        goal.stage = "diagnostic"
        goal.sequence_no = 0
        goal.target_id = target_id
        goal.source_timestamp_ns = now.nanoseconds
        goal.clock_domain = "ros_sim"
        goal.clock_epoch = 0
        goal.planning_group = "arm"
        goal.pipeline_id = "ompl"
        goal.planning_timeout_s = 0.2
        goal.velocity_scaling = 0.1
        goal.acceleration_scaling = 0.1
        goal.plan_only = True
        return goal

    def send(self, goal: PlanTarget.Goal | None = None):
        response = wait_future(self.plan_client.send_goal_async(goal or self.goal()))
        return response

    def close(self) -> None:
        self.dependencies.release_goal.set()
        self.dependencies.close()
        self.executor.shutdown(timeout_sec=3.0)
        self.thread.join(timeout=3.0)
        for node in (self.adapter, self.dependencies, self.client):
            node.destroy_node()
        rclpy.shutdown(context=self.context)


@pytest.fixture
def harness():
    instance = AdapterHarness()
    try:
        yield instance
    finally:
        instance.close()


@pytest.mark.parametrize(
    ("stage", "expected_contact_policy"),
    [("approach", False), ("descend", False), ("lift", True)],
)
def test_plan_only_success_publishes_exactly_once_to_gate(
    harness, stage: str, expected_contact_policy: bool
) -> None:
    harness.publish_inputs()
    goal = harness.goal()
    goal.stage = stage
    goal_handle = harness.send(goal)
    assert goal_handle.accepted
    wrapped = wait_future(goal_handle.get_result_async())
    result = wrapped.result
    assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
    assert result.success and result.trajectory_dispatched
    assert result.gate_accepted and result.gate_terminal
    assert result.downstream_terminal_observed
    assert result.action_goal_status == GoalStatus.STATUS_SUCCEEDED
    assert result.fjt_error_code == 0
    assert len(result.trajectory_digest) == 64
    wait_until(
        lambda: len(harness.dependencies.published_trajectories) == 1,
        reason="one gate publication",
    )
    assert len(harness.dependencies.move_group_goals) == 1
    move_group_goal = harness.dependencies.move_group_goals[0]
    assert move_group_goal.planning_options.plan_only
    assert move_group_goal.request.group_name == "arm"
    constraints = move_group_goal.request.goal_constraints[0].joint_constraints
    assert tuple(item.joint_name for item in constraints) == SO101_ARM_JOINTS
    assert tuple(item.position for item in constraints) == IK_GOAL
    assert tuple(
        harness.dependencies.published_trajectories[0].joint_names
    ) == SO101_ARM_JOINTS
    assert harness.dependencies.contact_policy_requests == [
        expected_contact_policy
    ]


def test_slow_wall_clock_ik_can_finish_while_ros_target_remains_fresh() -> None:
    """The wall guard must tolerate slow simulation without weakening safety.

    The fake ROS clock is deliberately held constant, so the immutable target
    remains fresh while a 250 ms wall-clock IK response exceeds the old 150 ms
    guard.  Other tests advance the ROS clock and prove the independent target
    freshness boundary still aborts.
    """

    harness = AdapterHarness(
        ik_delay_s=0.25,
        ik_response_timeout_ms=1_000.0,
    )
    try:
        harness.publish_inputs()
        goal_handle = harness.send()
        assert goal_handle.accepted
        wrapped = wait_future(goal_handle.get_result_async())
        assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
        assert wrapped.result.success
        assert wrapped.result.gate_accepted
        assert wrapped.result.downstream_terminal_observed
        assert len(harness.dependencies.gate_goals) == 1
    finally:
        harness.close()


@pytest.mark.parametrize(
    ("ik_error", "move_group_mode", "reason_fragment"),
    [
        (MoveItErrorCodes.NO_IK_SOLUTION, "success", "compute_ik_error:-31"),
        (MoveItErrorCodes.SUCCESS, "reject", "move_group_goal_rejected"),
        (MoveItErrorCodes.SUCCESS, "abort", "move_group_action_status"),
    ],
)
def test_ik_and_move_group_failures_never_publish(
    ik_error: int, move_group_mode: str, reason_fragment: str
) -> None:
    harness = AdapterHarness(ik_error=ik_error, move_group_mode=move_group_mode)
    try:
        harness.publish_inputs()
        goal_handle = harness.send()
        assert goal_handle.accepted
        wrapped = wait_future(goal_handle.get_result_async())
        assert wrapped.status == GoalStatus.STATUS_ABORTED
        assert not wrapped.result.trajectory_dispatched
        assert reason_fragment in wrapped.result.reason
        assert harness.dependencies.published_trajectories == []
    finally:
        harness.close()


def test_downstream_gate_unavailable_rejects_before_ik_or_move_group() -> None:
    harness = AdapterHarness(publish_gate_subscriber=False)
    try:
        harness.publish_inputs()
        goal_handle = harness.send()
        wrapped = wait_future(goal_handle.get_result_async())
        assert wrapped.status == GoalStatus.STATUS_ABORTED
        assert wrapped.result.reason == "trajectory_gate_unavailable"
        assert harness.dependencies.move_group_goals == []
        assert harness.dependencies.published_trajectories == []
    finally:
        harness.close()


@pytest.mark.parametrize(
    ("planning_scene_ready", "expected_reason"),
    [
        (None, "planning_scene:safety_unknown"),
        (False, "planning_scene:safety_false"),
    ],
)
def test_unknown_or_false_planning_scene_blocks_before_ik(
    planning_scene_ready: bool | None, expected_reason: str
) -> None:
    harness = AdapterHarness()
    try:
        harness.publish_inputs(planning_scene_ready=planning_scene_ready)
        goal_handle = harness.send()
        wrapped = wait_future(goal_handle.get_result_async())
        assert wrapped.status == GoalStatus.STATUS_ABORTED
        assert wrapped.result.reason == expected_reason
        assert harness.dependencies.move_group_goals == []
        assert harness.dependencies.published_trajectories == []
    finally:
        harness.close()


def test_typed_gate_fjt_failure_is_preserved_in_plan_target_result() -> None:
    harness = AdapterHarness(gate_mode="fjt_error")
    try:
        harness.publish_inputs()
        goal_handle = harness.send()
        wrapped = wait_future(goal_handle.get_result_async())
        result = wrapped.result
        assert wrapped.status == GoalStatus.STATUS_ABORTED
        assert not result.success
        assert result.trajectory_dispatched
        assert result.gate_accepted and result.gate_terminal
        assert result.downstream_terminal_observed
        assert result.action_goal_status == GoalStatus.STATUS_SUCCEEDED
        assert result.fjt_error_code == -4
        assert "path tolerance" in result.fjt_error_string
        assert result.reason == "trajectory_gate:fjt_error:-4"
    finally:
        harness.close()


def test_typed_gate_source_timestamp_mismatch_is_fail_closed() -> None:
    harness = AdapterHarness(gate_mode="source_mismatch")
    try:
        harness.publish_inputs()
        goal_handle = harness.send()
        wrapped = wait_future(goal_handle.get_result_async())
        result = wrapped.result
        assert wrapped.status == GoalStatus.STATUS_ABORTED
        assert not result.success
        assert result.trajectory_dispatched
        assert result.reason == "trajectory_gate_result_correlation_mismatch"
    finally:
        harness.close()


def test_source_age_is_not_reused_as_execution_liveness_after_gate_accepts() -> None:
    harness = AdapterHarness(gate_mode="hold", target_timeout_ms=100.0)
    try:
        harness.publish_inputs()
        goal_handle = harness.send()
        assert goal_handle.accepted
        wait_until(
            harness.dependencies.gate_goal_started.is_set,
            reason="typed gate goal",
        )
        harness.advance_clock(150_000_000)
        harness.dependencies.release_gate.set()
        wrapped = wait_future(goal_handle.get_result_async())
        result = wrapped.result
        assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
        assert result.success
        assert result.trajectory_dispatched and result.gate_terminal
        assert result.downstream_terminal_observed
        assert not result.cancel_requested
        assert result.action_goal_status == GoalStatus.STATUS_SUCCEEDED
        assert result.reason == "trajectory_executed_through_gate"
        assert harness.dependencies.gate_cancel_requests == 0
    finally:
        harness.close()


def test_plan_target_cancel_cancels_typed_gate_and_returns_correlated_terminal() -> None:
    harness = AdapterHarness(gate_mode="hold")
    try:
        harness.publish_inputs()
        goal_handle = harness.send()
        assert goal_handle.accepted
        wait_until(
            harness.dependencies.gate_goal_started.is_set,
            reason="typed gate goal",
        )
        cancel = wait_future(goal_handle.cancel_goal_async())
        assert cancel.goals_canceling
        wrapped = wait_future(goal_handle.get_result_async())
        result = wrapped.result
        assert wrapped.status == GoalStatus.STATUS_CANCELED
        assert not result.success
        assert result.command_id == harness.dependencies.gate_goals[0].command_id
        assert result.trajectory_digest
        assert result.gate_terminal and result.downstream_terminal_observed
        assert result.cancel_requested
        assert result.action_goal_status == GoalStatus.STATUS_CANCELED
        assert result.reason.startswith(
            "plan_target_cancel_requested:trajectory_gate:canceled:"
        )
        assert harness.dependencies.gate_cancel_requests == 1
    finally:
        harness.close()


def test_stale_during_planning_cancels_move_group_and_never_publishes() -> None:
    harness = AdapterHarness(move_group_mode="hold", target_timeout_ms=100.0)
    try:
        harness.publish_inputs()
        goal_handle = harness.send()
        assert goal_handle.accepted
        wait_until(harness.dependencies.goal_started.is_set, reason="MoveGroup goal")
        harness.advance_clock(150_000_000)
        wrapped = wait_future(goal_handle.get_result_async())
        assert wrapped.status == GoalStatus.STATUS_ABORTED
        assert wrapped.result.reason == "stale_target"
        wait_until(
            lambda: harness.dependencies.cancel_requests == 1,
            reason="MoveGroup cancellation",
        )
        assert harness.dependencies.published_trajectories == []
    finally:
        harness.close()


def test_concurrent_plan_request_is_rejected_and_old_result_cannot_publish() -> None:
    harness = AdapterHarness(move_group_mode="hold")
    try:
        harness.publish_inputs()
        first = harness.send(harness.goal("first"))
        assert first.accepted
        wait_until(harness.dependencies.goal_started.is_set, reason="first plan")
        second = harness.send(harness.goal("second"))
        assert not second.accepted
        wait_future(first.cancel_goal_async())
        wrapped = wait_future(first.get_result_async())
        assert wrapped.status == GoalStatus.STATUS_CANCELED
        assert not wrapped.result.trajectory_dispatched
        assert harness.dependencies.published_trajectories == []
    finally:
        harness.close()
