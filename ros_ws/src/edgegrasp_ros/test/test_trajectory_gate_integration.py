"""ROS-runtime integration test for the two pinned FJT command boundaries.

This file is intentionally outside the dependency-free root pytest suite. Run
it with colcon/ament on ROS 2 Jazzy; its presence is not runtime evidence.
"""

from __future__ import annotations

from collections import defaultdict
from functools import partial
import threading
import time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PointStamped
from edgegrasp.trajectory_identity import make_trajectory_command_id
from edgegrasp_interfaces.action import ExecuteTrajectory
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from edgegrasp_ros.safety_monitor import SafetyMonitor
from edgegrasp_ros.trajectory_gate import TrajectoryGate


ARM_ACTION = "/arm_controller/follow_joint_trajectory"
GRIPPER_ACTION = "/gripper_controller/follow_joint_trajectory"
ARM_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]


class FakeControllerServers(Node):
    def __init__(self, *, modes: dict[str, str] | None = None) -> None:
        super().__init__("fake_so101_controller_servers")
        self.modes = modes or {}
        self.goals = defaultdict(list)
        self.requests = defaultdict(list)
        self.cancels = defaultdict(int)
        self._servers = [
            self._server("arm_controller", ARM_ACTION),
            self._server("gripper_controller", GRIPPER_ACTION),
        ]

    def _server(self, controller: str, action: str) -> ActionServer:
        return ActionServer(
            self,
            FollowJointTrajectory,
            action,
            execute_callback=partial(self._execute, controller),
            goal_callback=partial(self._goal, controller),
            cancel_callback=partial(self._cancel, controller),
        )

    def _goal(self, controller: str, request) -> GoalResponse:
        self.goals[controller].append(tuple(request.trajectory.joint_names))
        self.requests[controller].append(request)
        return GoalResponse.ACCEPT

    def _cancel(self, controller: str, handle) -> CancelResponse:
        del handle
        self.cancels[controller] += 1
        return CancelResponse.ACCEPT

    def _execute(self, controller: str, goal_handle):
        result = FollowJointTrajectory.Result()
        mode = self.modes.get(controller, "hold")
        if mode == "success":
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result
        if mode == "fjt_error":
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            result.error_string = "synthetic path tolerance violation"
            return result
        while not goal_handle.is_cancel_requested:
            time.sleep(0.01)
        goal_handle.canceled()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def close(self) -> None:
        for server in self._servers:
            server.destroy()


def wait_until(
    predicate, timeout_s: float = 10.0, reason: str = "ROS integration condition"
) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError(f"timed out waiting for {reason}")


def trajectory(joints: list[str], positions: list[float]) -> JointTrajectory:
    message = JointTrajectory()
    message.header.frame_id = "base_link"
    message.joint_names = joints
    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = Duration(sec=2)
    message.points = [point]
    return message


def wait_future(future, timeout_s: float = 10.0):
    wait_until(future.done, timeout_s, "ROS future")
    return future.result()


def publish_clock(
    publisher, monitor: SafetyMonitor, gate: TrajectoryGate, timestamp_ns: int
) -> None:
    message = Clock()
    message.clock.sec = timestamp_ns // 1_000_000_000
    message.clock.nanosec = timestamp_ns % 1_000_000_000
    publisher.publish(message)
    wait_until(
        lambda: monitor.get_clock().now().nanoseconds == timestamp_ns
        and gate.get_clock().now().nanoseconds == timestamp_ns,
        reason=f"both safety nodes to observe /clock={timestamp_ns}",
    )


class TypedGateHarness:
    def __init__(self, *, arm_mode: str) -> None:
        rclpy.init()
        overrides = [
            Parameter("use_sim_time", value=False),
            Parameter("clock_domain", value="ros_system"),
            Parameter("permission_timeout_ms", value=2_000.0),
            Parameter("interface_timeout_ms", value=2_000.0),
            Parameter("target_watchdog_timeout_ms", value=2_000.0),
            Parameter("joint_state_timeout_ms", value=2_000.0),
        ]
        self.gate = TrajectoryGate(parameter_overrides=overrides)
        self.servers = FakeControllerServers(
            modes={"arm_controller": arm_mode}
        )
        self.client = Node("edgegrasp_typed_gate_test_client")
        self.permission = self.client.create_publisher(
            Bool, "/edgegrasp/motion_allowed", 10
        )
        self.interface_ready = self.client.create_publisher(
            Bool, "/edgegrasp/interface_ready", 10
        )
        self.target = self.client.create_publisher(
            PointStamped, "/edgegrasp/target_3d", 10
        )
        self.joints = self.client.create_publisher(
            JointState, "/joint_states", 10
        )
        self.execute = ActionClient(
            self.client,
            ExecuteTrajectory,
            "/edgegrasp/execute_trajectory",
        )
        self.executor = MultiThreadedExecutor(num_threads=8)
        for node in (self.gate, self.servers, self.client):
            self.executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        wait_until(self.execute.server_is_ready, reason="ExecuteTrajectory server")
        wait_until(
            lambda: all(
                client.server_is_ready()
                for client in self.gate._goal_clients.values()
            ),
            reason="both downstream FJT servers",
        )
        wait_until(lambda: self.permission.get_subscription_count() == 1)
        wait_until(lambda: self.interface_ready.get_subscription_count() == 1)
        wait_until(lambda: self.target.get_subscription_count() == 1)
        wait_until(lambda: self.joints.get_subscription_count() == 1)

    def publish_inputs(self) -> int:
        now = self.client.get_clock().now()
        self.permission.publish(Bool(data=True))
        self.interface_ready.publish(Bool(data=True))
        target = PointStamped()
        target.header.frame_id = "base_link"
        target.header.stamp = now.to_msg()
        target.point.x = 0.2
        target.point.z = 0.1
        self.target.publish(target)
        self.joints.publish(
            JointState(
                name=ARM_JOINTS + ["gripper"],
                position=[0.0] * 6,
            )
        )
        wait_until(
            lambda: self.gate._permission.evaluate(
                self.gate._now_ns()
            ).allowed
            and self.gate._interface_permission.evaluate(
                self.gate._now_ns()
            ).allowed
            and self.gate._last_target_receive_ns is not None
            and self.gate._last_joint_state_ns is not None,
            reason="fresh typed-gate admission inputs",
        )
        return now.nanoseconds

    def goal(
        self,
        *,
        task_id: str,
        stage: str = "approach",
        sequence_no: int = 0,
        controller: str = "arm_controller",
        source_timestamp_ns: int,
    ) -> ExecuteTrajectory.Goal:
        goal = ExecuteTrajectory.Goal()
        goal.task_id = task_id
        goal.command_id = make_trajectory_command_id(
            task_id, stage, sequence_no
        )
        goal.target_id = "cube-1"
        goal.stage = stage
        goal.sequence_no = sequence_no
        goal.controller = controller
        goal.trajectory = trajectory(
            ARM_JOINTS,
            [0.0, -0.1, 0.2, -0.1, 0.0],
        )
        goal.source_timestamp_ns = source_timestamp_ns
        goal.clock_domain = "ros_system"
        goal.clock_epoch = 0
        return goal

    def close(self) -> None:
        self.executor.shutdown(timeout_sec=3.0)
        self.thread.join(timeout=3.0)
        self.servers.close()
        self.execute.destroy()
        for node in (self.gate, self.servers, self.client):
            node.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def ros_graph():
    rclpy.init()
    overrides = [
        # This fixture deliberately holds two fake FJT goals while colcon may
        # run other package action suites in parallel.  Keep the test-only
        # admission samples fresh across scheduler contention; the dedicated
        # 200 ms watchdog test below retains the production boundary check.
        Parameter("permission_timeout_ms", value=5_000.0),
        Parameter("interface_timeout_ms", value=5_000.0),
        Parameter("target_watchdog_timeout_ms", value=5_000.0),
        Parameter("joint_state_timeout_ms", value=5_000.0),
    ]
    gate = TrajectoryGate(parameter_overrides=overrides)
    servers = FakeControllerServers()
    client = Node("edgegrasp_trajectory_gate_test_client")
    permission = client.create_publisher(Bool, "/edgegrasp/motion_allowed", 10)
    interface_ready = client.create_publisher(Bool, "/edgegrasp/interface_ready", 10)
    target = client.create_publisher(PointStamped, "/edgegrasp/target_3d", 10)
    joint_states = client.create_publisher(JointState, "/joint_states", 10)
    arm = client.create_publisher(
        JointTrajectory, "/edgegrasp/arm_joint_trajectory_request", 10
    )
    gripper = client.create_publisher(
        JointTrajectory, "/edgegrasp/gripper_joint_trajectory_request", 10
    )
    statuses: list[str] = []
    client.create_subscription(
        String,
        "/edgegrasp/trajectory_gate_status",
        lambda message: statuses.append(message.data),
        10,
    )

    # Two accepted controller goals intentionally block in their execute
    # callbacks until cancellation.  Leave independent executor capacity for
    # both cancel services, result callbacks, gate subscriptions, and timers,
    # including when colcon runs packages in parallel under load.
    executor = MultiThreadedExecutor(num_threads=8)
    for node in (gate, servers, client):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        yield (
            gate,
            servers,
            client,
            permission,
            interface_ready,
            target,
            joint_states,
            arm,
            gripper,
            statuses,
        )
    finally:
        executor.shutdown(timeout_sec=3.0)
        thread.join(timeout=3.0)
        servers.close()
        for node in (gate, servers, client):
            node.destroy_node()
        rclpy.shutdown()


def test_arm_and_gripper_forward_and_cancel_fail_closed(ros_graph) -> None:
    (
        gate,
        servers,
        client,
        permission,
        interface_ready,
        target,
        joint_states,
        arm,
        gripper,
        statuses,
    ) = ros_graph
    wait_until(lambda: permission.get_subscription_count() == 1)
    wait_until(lambda: interface_ready.get_subscription_count() == 1)
    wait_until(lambda: target.get_subscription_count() == 1)
    wait_until(lambda: joint_states.get_subscription_count() == 1)
    wait_until(lambda: arm.get_subscription_count() == 1)
    wait_until(lambda: gripper.get_subscription_count() == 1)
    wait_until(
        lambda: all(client.server_is_ready() for client in gate._goal_clients.values())
    )

    permission.publish(Bool(data=True))
    interface_ready.publish(Bool(data=True))
    target_message = PointStamped()
    target_message.header.frame_id = "base_link"
    target_message.header.stamp = client.get_clock().now().to_msg()
    target_message.point.x = 0.2
    target_message.point.z = 0.1
    target.publish(target_message)
    joint_states.publish(
        JointState(
            name=ARM_JOINTS + ["gripper"],
            position=[0.0] * 6,
        )
    )
    wait_until(
        lambda: gate._permission.evaluate(gate._now_ns()).allowed
        and gate._interface_permission.evaluate(gate._now_ns()).allowed
        and gate._last_target_receive_ns is not None
        and gate._last_joint_state_ns is not None
    )
    arm.publish(trajectory(ARM_JOINTS, [0.0, -0.1, 0.2, -0.1, 0.0]))
    wait_until(lambda: len(servers.goals["arm_controller"]) == 1)

    gripper.publish(trajectory(["gripper"], [0.1, 0.2]))
    wait_until(
        lambda: any("rejected:position_length_mismatch" in item for item in statuses)
    )
    assert servers.goals["gripper_controller"] == []

    target_message.header.stamp = client.get_clock().now().to_msg()
    target.publish(target_message)
    joint_states.publish(
        JointState(name=ARM_JOINTS + ["gripper"], position=[0.0] * 6)
    )
    interface_ready.publish(Bool(data=True))
    permission.publish(Bool(data=True))
    gripper.publish(trajectory(["gripper"], [0.2]))
    wait_until(lambda: len(servers.goals["gripper_controller"]) == 1)
    permission.publish(Bool(data=False))
    wait_until(lambda: servers.cancels["arm_controller"] >= 1)
    wait_until(lambda: servers.cancels["gripper_controller"] >= 1)
    wait_until(
        lambda: all(
            gate._goal_handles[controller] is None
            for controller in ("arm_controller", "gripper_controller")
        ),
        reason="canceled controller results",
    )


@pytest.mark.parametrize(
    ("arm_mode", "outer_status", "expected_success", "expected_fjt_code"),
    [
        (
            "success",
            GoalStatus.STATUS_SUCCEEDED,
            True,
            FollowJointTrajectory.Result.SUCCESSFUL,
        ),
        (
            "fjt_error",
            GoalStatus.STATUS_ABORTED,
            False,
            FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
        ),
    ],
)
def test_typed_execution_requires_wrapper_and_fjt_success(
    arm_mode: str,
    outer_status: int,
    expected_success: bool,
    expected_fjt_code: int,
) -> None:
    harness = TypedGateHarness(arm_mode=arm_mode)
    try:
        source_ns = harness.publish_inputs()
        goal = harness.goal(task_id=f"typed-{arm_mode}", source_timestamp_ns=source_ns)
        goal_handle = wait_future(harness.execute.send_goal_async(goal))
        assert goal_handle.accepted
        wrapped = wait_future(goal_handle.get_result_async())
        result = wrapped.result
        assert wrapped.status == outer_status
        assert result.task_id == goal.task_id
        assert result.command_id == goal.command_id
        assert result.target_id == goal.target_id
        assert result.stage == "approach"
        assert result.sequence_no == 0
        assert result.controller == "arm_controller"
        assert result.source_timestamp_ns == source_ns
        assert len(result.trajectory_digest) == 64
        assert result.accepted and result.terminal
        assert result.downstream_terminal_observed
        assert not result.cancel_requested
        assert result.action_goal_status == GoalStatus.STATUS_SUCCEEDED
        assert result.fjt_error_code == expected_fjt_code
        assert result.success is expected_success
        assert len(harness.servers.goals["arm_controller"]) == 1
        if expected_success:
            assert result.reason == "succeeded"
            assert result.fjt_error_string == ""
        else:
            assert result.reason == f"fjt_error:{expected_fjt_code}"
            assert "path tolerance" in result.fjt_error_string
    finally:
        harness.close()


def test_typed_gate_uses_immediate_fjt_stamp_and_retains_source_correlation() -> None:
    harness = TypedGateHarness(arm_mode="success")
    try:
        source_timestamp_ns = harness.publish_inputs()
        goal = harness.goal(
            task_id="typed-zero-fjt-stamp",
            source_timestamp_ns=source_timestamp_ns,
        )
        goal.trajectory.header.stamp.sec = 123
        goal.trajectory.header.stamp.nanosec = 456

        handle = wait_future(harness.execute.send_goal_async(goal))
        assert handle.accepted
        wrapped = wait_future(handle.get_result_async())

        assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
        assert wrapped.result.success
        assert wrapped.result.source_timestamp_ns == source_timestamp_ns
        forwarded = harness.servers.requests["arm_controller"][-1].trajectory
        assert forwarded.header.frame_id == "base_link"
        assert forwarded.header.stamp.sec == 0
        assert forwarded.header.stamp.nanosec == 0
        assert goal.trajectory.header.stamp.sec == 123
        assert goal.trajectory.header.stamp.nanosec == 456
    finally:
        harness.close()


def test_typed_execution_rejects_duplicate_wrong_id_and_frame_mismatch() -> None:
    harness = TypedGateHarness(arm_mode="success")
    try:
        source_ns = harness.publish_inputs()
        first_goal = harness.goal(task_id="typed-correlation", source_timestamp_ns=source_ns)
        first_handle = wait_future(harness.execute.send_goal_async(first_goal))
        first_result = wait_future(first_handle.get_result_async())
        assert first_result.result.success

        duplicate = wait_future(harness.execute.send_goal_async(first_goal))
        assert not duplicate.accepted

        wrong_id = harness.goal(task_id="typed-wrong-id", source_timestamp_ns=source_ns)
        wrong_id.command_id = "typed-wrong-id|lift|99"
        rejected = wait_future(harness.execute.send_goal_async(wrong_id))
        assert not rejected.accepted

        frame_goal = harness.goal(
            task_id="typed-frame-mismatch",
            source_timestamp_ns=source_ns,
        )
        frame_goal.trajectory.header.frame_id = "world"
        frame_handle = wait_future(harness.execute.send_goal_async(frame_goal))
        assert frame_handle.accepted
        frame_result = wait_future(frame_handle.get_result_async()).result
        assert not frame_result.success
        assert not frame_result.accepted
        assert not frame_result.downstream_terminal_observed
        assert frame_result.fjt_error_code == -(2**31)
        assert frame_result.reason.startswith("trajectory_frame_mismatch:")
        assert len(harness.servers.goals["arm_controller"]) == 1
    finally:
        harness.close()


def test_typed_concurrent_goal_rejected_and_cancel_waits_for_terminal_result() -> None:
    harness = TypedGateHarness(arm_mode="hold")
    try:
        source_ns = harness.publish_inputs()
        first_goal = harness.goal(task_id="typed-hold", source_timestamp_ns=source_ns)
        first = wait_future(harness.execute.send_goal_async(first_goal))
        assert first.accepted
        wait_until(lambda: len(harness.servers.goals["arm_controller"]) == 1)

        second_goal = harness.goal(
            task_id="typed-concurrent",
            source_timestamp_ns=source_ns,
        )
        second = wait_future(harness.execute.send_goal_async(second_goal))
        assert not second.accepted

        cancel_response = wait_future(first.cancel_goal_async())
        assert cancel_response.goals_canceling
        wrapped = wait_future(first.get_result_async())
        result = wrapped.result
        assert wrapped.status == GoalStatus.STATUS_CANCELED
        assert not result.success
        assert result.accepted and result.terminal
        assert result.cancel_requested
        assert result.downstream_terminal_observed
        assert result.action_goal_status == GoalStatus.STATUS_CANCELED
        assert result.fjt_error_code == FollowJointTrajectory.Result.SUCCESSFUL
        assert result.reason.startswith("canceled:client_cancel_requested")
    finally:
        harness.close()


def test_sim_clock_target_dropout_reaches_gate_and_cancels_active_goal() -> None:
    """Exercise the raw target -> monitor -> gate watchdog path with /clock."""
    rclpy.init()
    common = [
        Parameter("use_sim_time", value=True),
        Parameter("clock_domain", value="ros_sim"),
        Parameter("check_rate_hz", value=50.0),
    ]
    monitor = SafetyMonitor(
        parameter_overrides=common
        + [
            Parameter("stale_after_ms", value=200.0),
            Parameter("watchdog_timeout_ms", value=200.0),
        ]
    )
    gate = TrajectoryGate(
        parameter_overrides=common
        + [
            Parameter("permission_timeout_ms", value=1_000.0),
            Parameter("interface_timeout_ms", value=1_000.0),
            Parameter("target_watchdog_timeout_ms", value=200.0),
            Parameter("joint_state_timeout_ms", value=1_000.0),
        ]
    )
    servers = FakeControllerServers()
    client = Node("edgegrasp_watchdog_integration_client")
    clock = client.create_publisher(Clock, "/clock", 10)
    interface_ready = client.create_publisher(Bool, "/edgegrasp/interface_ready", 10)
    target_publisher = client.create_publisher(
        PointStamped, "/edgegrasp/target_3d", 10
    )
    joint_states = client.create_publisher(JointState, "/joint_states", 10)
    arm = client.create_publisher(
        JointTrajectory, "/edgegrasp/arm_joint_trajectory_request", 10
    )
    safety_statuses: list[str] = []
    client.create_subscription(
        String,
        "/edgegrasp/safety_status",
        lambda message: safety_statuses.append(message.data),
        10,
    )
    executor = MultiThreadedExecutor(num_threads=8)
    for node in (monitor, gate, servers, client):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(lambda: clock.get_subscription_count() >= 2)
        wait_until(lambda: target_publisher.get_subscription_count() == 2)
        wait_until(lambda: interface_ready.get_subscription_count() == 1)
        wait_until(lambda: joint_states.get_subscription_count() == 1)
        wait_until(lambda: arm.get_subscription_count() == 1)
        wait_until(
            lambda: gate._goal_clients["arm_controller"].server_is_ready()
        )

        source_ns = 1_000_000_000
        publish_clock(clock, monitor, gate, source_ns)
        interface_ready.publish(Bool(data=True))
        target_message = PointStamped()
        target_message.header.frame_id = "base_link"
        target_message.header.stamp.sec = 1
        target_message.point.x = 0.2
        target_message.point.z = 0.1
        target_publisher.publish(target_message)
        joint_states.publish(
            JointState(name=ARM_JOINTS + ["gripper"], position=[0.0] * 6)
        )

        publish_clock(clock, monitor, gate, source_ns + 20_000_000)
        wait_until(
            lambda: gate._permission.evaluate(gate._now_ns()).allowed
            and gate._interface_permission.evaluate(gate._now_ns()).allowed
            and gate._last_target_receive_ns is not None
            and gate._last_joint_state_ns is not None,
            reason="fresh monitor permission and trajectory admission inputs",
        )
        arm.publish(trajectory(ARM_JOINTS, [0.0, -0.1, 0.2, -0.1, 0.0]))
        wait_until(lambda: len(servers.goals["arm_controller"]) == 1)

        # The core/source contract permits exactly 200 ms.  The next 50 Hz
        # timer tick (220 ms) must deny and cancel, giving a configured bound
        # of timeout + one timer period before executor scheduling jitter.
        status_count = len(safety_statuses)
        publish_clock(clock, monitor, gate, source_ns + 200_000_000)
        wait_until(lambda: len(safety_statuses) > status_count)
        assert safety_statuses[-1].startswith("allowed;")
        assert servers.cancels["arm_controller"] == 0

        publish_clock(clock, monitor, gate, source_ns + 220_000_000)
        wait_until(
            lambda: any(item.startswith("stale_target;") for item in safety_statuses)
        )
        wait_until(lambda: servers.cancels["arm_controller"] >= 1)
        wait_until(
            lambda: gate._goal_handles["arm_controller"] is None,
            reason="watchdog-canceled arm result",
        )
    finally:
        executor.shutdown(timeout_sec=3.0)
        thread.join(timeout=3.0)
        servers.close()
        for node in (monitor, gate, servers, client):
            node.destroy_node()
        rclpy.shutdown()
