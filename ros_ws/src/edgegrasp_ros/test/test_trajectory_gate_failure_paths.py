"""ROS-node-level failure-path tests for the FJT command boundary."""

from __future__ import annotations

import threading
import time
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from action_msgs.msg import GoalInfo
from action_msgs.srv import CancelGoal
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from edgegrasp.so101_contract import SO101_ARM_JOINTS
from edgegrasp.trajectory_identity import make_trajectory_command_id
from edgegrasp_interfaces.action import ExecuteTrajectory
import pytest
import rclpy
from rclpy.action import GoalResponse
from rclpy.context import Context
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from edgegrasp_ros.trajectory_gate import TrajectoryGate, _TypedExecution


class FakeFuture:
    def __init__(self, *, value=None, error: Exception | None = None, done=True):
        self._value = value
        self._error = error
        self._done = done
        self._callbacks = []

    def done(self):
        return self._done

    def result(self):
        if self._error is not None:
            raise self._error
        return self._value

    def add_done_callback(self, callback):
        self._callbacks.append(callback)
        if self._done:
            callback(self)


class FakeGoalHandle:
    def __init__(self, *, response=None, error: Exception | None = None, done=True):
        self.response = response
        self.error = error
        self.done = done
        self.cancel_calls = 0

    def cancel_goal_async(self):
        self.cancel_calls += 1
        if self.error is not None:
            raise self.error
        return FakeFuture(value=self.response, done=self.done)


class FakeAcceptedGoalHandle(FakeGoalHandle):
    def __init__(
        self,
        *,
        result_future=None,
        result_error: Exception | None = None,
        cancel_response_value=None,
    ):
        super().__init__(response=cancel_response_value)
        self.accepted = True
        self._result_future = result_future
        self._result_error = result_error

    def get_result_async(self):
        if self._result_error is not None:
            raise self._result_error
        if self._result_future is None:
            return FakeFuture(done=False)
        return self._result_future


class FakeActionClient:
    def __init__(self, *, ready: bool, send_error: Exception | None = None):
        self.ready = ready
        self.send_error = send_error
        self.sent = 0

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal):
        del goal
        self.sent += 1
        if self.send_error is not None:
            raise self.send_error
        return FakeFuture(done=False)


def cancel_response(accepted: bool) -> CancelGoal.Response:
    response = CancelGoal.Response()
    if accepted:
        response.goals_canceling = [GoalInfo()]
    return response


def reset_cancel_case(gate: TrajectoryGate, controller: str) -> None:
    """Isolate one cancel outcome while preserving the node under test."""
    gate._fault_latched = None
    gate._cancel_requested[controller] = False
    gate._cancel_futures[controller] = None
    gate._cancel_started_ns[controller] = None
    gate._cancel_attempts[controller] = 0
    gate._cancel_retry_due_ns[controller] = None
    gate._cancel_exhausted[controller] = False
    gate._result_deadline_ns[controller] = None


def valid_trajectory() -> JointTrajectory:
    message = JointTrajectory()
    message.header.frame_id = "base_link"
    message.joint_names = list(SO101_ARM_JOINTS)
    point = JointTrajectoryPoint()
    point.positions = [0.05, -0.05, 0.05, -0.05, 0.0]
    point.time_from_start = Duration(sec=1)
    message.points = [point]
    return message


def typed_goal(task_id: str) -> ExecuteTrajectory.Goal:
    goal = ExecuteTrajectory.Goal()
    goal.task_id = task_id
    goal.stage = "approach"
    goal.sequence_no = 0
    goal.command_id = make_trajectory_command_id(task_id, goal.stage, 0)
    goal.target_id = "cube-1"
    goal.controller = "arm_controller"
    goal.trajectory = valid_trajectory()
    goal.source_timestamp_ns = 1
    goal.clock_domain = "ros_system"
    goal.clock_epoch = 0
    return goal


def typed_execution(task_id: str) -> _TypedExecution:
    request = typed_goal(task_id)
    return _TypedExecution(
        goal_handle=SimpleNamespace(request=request),
        task_id=request.task_id,
        command_id=request.command_id,
        target_id=request.target_id,
        stage=request.stage,
        sequence_no=int(request.sequence_no),
        controller=request.controller,
        source_timestamp_ns=int(request.source_timestamp_ns),
        trajectory_digest="0" * 64,
    )


@pytest.fixture
def gate():
    context = Context()
    rclpy.init(context=context, domain_id=191)
    node = TrajectoryGate(
        context=context,
        parameter_overrides=[
            Parameter("use_sim_time", value=False),
            Parameter("clock_domain", value="ros_system"),
        ],
    )
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)


def test_cancel_accept_reject_exception_and_timeout_are_distinct(gate) -> None:
    accepted = FakeGoalHandle(response=cancel_response(True))
    gate._goal_handles["arm_controller"] = accepted
    gate._cancel_controller("arm_controller", "test")
    assert accepted.cancel_calls == 1
    assert gate._cancel_requested["arm_controller"]
    assert gate._fault_latched is None

    reset_cancel_case(gate, "arm_controller")
    rejected = FakeGoalHandle(response=cancel_response(False))
    gate._goal_handles["arm_controller"] = rejected
    gate._cancel_controller("arm_controller", "test")
    assert gate._fault_latched == "cancel_failed:arm_controller:rejected"
    assert gate._cancel_retry_due_ns["arm_controller"] is not None

    reset_cancel_case(gate, "arm_controller")
    throwing = FakeGoalHandle(error=RuntimeError("cancel transport"))
    gate._goal_handles["arm_controller"] = throwing
    gate._cancel_controller("arm_controller", "test")
    assert gate._fault_latched == "cancel_failed:arm_controller:RuntimeError"

    reset_cancel_case(gate, "arm_controller")
    pending = FakeGoalHandle(done=False)
    gate._goal_handles["arm_controller"] = pending
    now = 1_000_000_000
    gate._now_ns = lambda: now
    gate._cancel_controller("arm_controller", "test")
    now += gate._cancel_response_timeout_ns + 1
    gate._now_ns = lambda: now
    gate._watchdog()
    assert gate._fault_latched == "cancel_failed:arm_controller:timeout"


def test_concurrent_interface_clock_reads_do_not_false_latch_rollback(
    gate: TrajectoryGate,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A newer callback cannot commit before an older sampled ROS time.

    ``TrajectoryGate`` uses a ``MultiThreadedExecutor`` in production.  The
    fake clock makes the second caller overtake the first unless clock sampling
    and ``MotionPermissionGate.update`` share the trajectory-gate state lock.
    """

    base_ns = 8_000_000_000
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
                    time.sleep(0.02)
                return _Instant(base_ns + 10_000_000)
            second_entered.set()
            return _Instant(base_ns + 20_000_000)

    monkeypatch.setattr(gate, "get_clock", lambda: _OvertakingClock())
    first = threading.Thread(target=lambda: gate._on_interface_ready(Bool(data=True)))
    second = threading.Thread(target=lambda: gate._on_interface_ready(Bool(data=True)))
    first.start()
    assert first_entered.wait(1.0)
    second.start()
    first.join(1.0)
    second.join(1.0)

    assert not first.is_alive()
    assert not second.is_alive()
    with gate._state_lock:
        decision = gate._interface_permission.evaluate(base_ns + 20_000_000)
    assert decision.allowed
    assert decision.reason == "allowed"


def test_cancel_accept_without_terminal_result_retries_then_escalates(gate) -> None:
    now = 3_000_000_000
    gate._now_ns = lambda: now
    active = FakeGoalHandle(response=cancel_response(True))
    gate._goal_handles["arm_controller"] = active
    gate._controller_owners["arm_controller"] = "legacy:arm_controller:test"
    gate._result_futures["arm_controller"] = FakeFuture(done=False)

    gate._cancel_controller("arm_controller", "permission_denied")
    assert active.cancel_calls == 1
    assert gate._cancel_requested["arm_controller"]

    for expected_attempts in (2, 3):
        now += gate._cancel_completion_timeout_ns + 1
        gate._watchdog()
        assert gate._cancel_retry_due_ns["arm_controller"] is not None
        now = gate._cancel_retry_due_ns["arm_controller"]
        gate._watchdog()
        assert active.cancel_calls == expected_attempts

    now += gate._cancel_completion_timeout_ns + 1
    gate._watchdog()
    assert gate._cancel_exhausted["arm_controller"]
    assert gate._cancel_retry_due_ns["arm_controller"] is None
    assert gate._fault_latched == (
        "cancel_failed:arm_controller:accepted_but_goal_still_active"
    )
    assert gate._typed_tombstones["arm_controller"].command_id == (
        "legacy:arm_controller:test"
    )

    # Further watchdog ticks remain fail-closed and do not exceed the bound.
    now += gate._cancel_retry_ns + 1
    gate._watchdog()
    assert active.cancel_calls == gate._max_cancel_attempts == 3


def test_goal_and_result_future_exceptions_latch_and_do_not_assume_success(gate) -> None:
    goal_future = FakeFuture(error=RuntimeError("goal response"))
    gate._send_goal_futures["arm_controller"] = goal_future
    gate._controller_owners["arm_controller"] = "legacy:test:goal"
    gate._send_goal_started_ns["arm_controller"] = 1
    gate._on_goal_response(
        "arm_controller",
        goal_future,
        "legacy:test:goal",
        None,
        goal_future,
    )
    assert gate._fault_latched == "goal_response_failed:RuntimeError"
    assert gate._goal_handles["arm_controller"] is None

    gate._fault_latched = None
    gate._goal_handles["arm_controller"] = FakeGoalHandle(
        response=cancel_response(True)
    )
    result_future = FakeFuture(error=TimeoutError("result"))
    gate._result_futures["arm_controller"] = result_future
    gate._controller_owners["arm_controller"] = "legacy:test:result"
    gate._on_result(
        "arm_controller",
        result_future,
        "legacy:test:result",
        None,
        result_future,
    )
    assert gate._fault_latched == "result_failed:TimeoutError"
    assert gate._goal_handles["arm_controller"].cancel_calls == 1


def test_downstream_unready_and_send_exception_never_create_a_pending_goal(gate) -> None:
    now = 2_000_000_000
    gate._now_ns = lambda: now
    gate._permission.update(True, now)
    gate._interface_permission.update(True, now)
    gate._last_target_receive_ns = now
    gate._last_joint_state_ns = now
    gate._joint_positions = {joint: 0.0 for joint in SO101_ARM_JOINTS}
    message = valid_trajectory()

    unavailable = FakeActionClient(ready=False)
    gate._goal_clients["arm_controller"] = unavailable
    gate._on_trajectory("arm_controller", message)
    assert unavailable.sent == 0
    assert gate._send_goal_futures["arm_controller"] is None

    throwing = FakeActionClient(ready=True, send_error=RuntimeError("send"))
    gate._goal_clients["arm_controller"] = throwing
    gate._on_trajectory("arm_controller", message)
    assert throwing.sent == 1
    assert gate._send_goal_futures["arm_controller"] is None
    assert gate._fault_latched == "send_goal_failed:RuntimeError"


def test_typed_and_legacy_share_one_controller_reservation(gate) -> None:
    fake = FakeActionClient(ready=True)
    gate._goal_clients["arm_controller"] = fake

    request = typed_goal("typed-reserves-first")
    assert gate._on_typed_goal(request) == GoalResponse.ACCEPT
    gate._on_trajectory("arm_controller", valid_trajectory())
    assert fake.sent == 0

    gate._typed_reservations.clear()
    gate._controller_owners["arm_controller"] = None
    now = 5_000_000_000
    gate._now_ns = lambda: now
    gate._permission.update(True, now)
    gate._interface_permission.update(True, now)
    gate._last_target_receive_ns = now
    gate._last_joint_state_ns = now
    gate._joint_positions = {joint: 0.0 for joint in SO101_ARM_JOINTS}
    gate._on_trajectory("arm_controller", valid_trajectory())
    assert fake.sent == 1
    assert gate._controller_owners["arm_controller"].startswith("legacy:")

    competing = typed_goal("legacy-reserves-first")
    assert gate._on_typed_goal(competing) == GoalResponse.REJECT


def test_stop_unconfirmed_tombstone_blocks_typed_legacy_and_reset(gate) -> None:
    dispatch_id = "typed-stop-unconfirmed|approach|0"
    gate._controller_owners["arm_controller"] = dispatch_id
    gate._latch_typed_tombstone("arm_controller", dispatch_id)
    fake = FakeActionClient(ready=True)
    gate._goal_clients["arm_controller"] = fake

    assert gate._on_typed_goal(typed_goal("blocked-after-timeout")) == (
        GoalResponse.REJECT
    )
    gate._on_trajectory("arm_controller", valid_trajectory())
    assert fake.sent == 0

    response = gate._on_reset(Trigger.Request(), Trigger.Response())
    assert not response.success
    assert gate._typed_tombstones["arm_controller"].command_id == dispatch_id
    assert gate._controller_owners["arm_controller"] == dispatch_id


@pytest.mark.parametrize(
    "status",
    [
        GoalStatus.STATUS_SUCCEEDED,
        GoalStatus.STATUS_CANCELED,
        GoalStatus.STATUS_ABORTED,
    ],
)
def test_same_dispatch_late_terminal_requires_explicit_reset(gate, status: int) -> None:
    context = typed_execution("late-tombstone")
    command_id = context.command_id
    context.terminal = True
    context.success = False
    context.reason = "typed_terminal_timeout_stop_unconfirmed"
    context.event.set()
    gate._typed_commands["arm_controller"] = context
    gate._controller_owners["arm_controller"] = command_id
    gate._latch_typed_tombstone("arm_controller", command_id)
    gate._fault_latched = "typed_terminal_timeout:arm_controller"

    future = FakeFuture(
        value=SimpleNamespace(
            status=status,
            result=SimpleNamespace(
                error_code=FollowJointTrajectory.Result.SUCCESSFUL,
                error_string="",
            ),
        )
    )
    gate._result_futures["arm_controller"] = future
    gate._goal_handles["arm_controller"] = FakeGoalHandle()

    gate._on_result(
        "arm_controller", future, command_id, command_id, future
    )

    tombstone = gate._typed_tombstones["arm_controller"]
    assert tombstone.command_id == command_id
    assert tombstone.terminal_confirmed
    assert tombstone.action_goal_status == status
    assert gate._controller_owners["arm_controller"] == command_id
    assert gate._goal_handles["arm_controller"] is None
    assert gate._result_futures["arm_controller"] is None
    assert not context.success
    assert not context.downstream_terminal_observed
    assert context.reason == "typed_terminal_timeout_stop_unconfirmed"

    fake = FakeActionClient(ready=True)
    gate._goal_clients["arm_controller"] = fake
    assert gate._on_typed_goal(typed_goal("late-result-cannot-unlock")) == (
        GoalResponse.REJECT
    )
    gate._on_trajectory("arm_controller", valid_trajectory())
    assert fake.sent == 0

    # Simulate the already-returned ExecuteTrajectory callback's finally block.
    gate._typed_commands["arm_controller"] = None
    gate._typed_reservations.pop(command_id, None)
    response = gate._on_reset(Trigger.Request(), Trigger.Response())
    assert response.success
    assert "arm_controller" not in gate._typed_tombstones
    assert gate._controller_owners["arm_controller"] is None
    assert gate._fault_latched is None


def test_result_request_exception_completes_typed_failure_and_latches(gate) -> None:
    context = typed_execution("result-request-exception")
    command_id = context.command_id
    downstream = FakeAcceptedGoalHandle(
        result_error=RuntimeError("get result"),
        cancel_response_value=cancel_response(True),
    )
    send_future = FakeFuture(value=downstream)
    gate._typed_commands["arm_controller"] = context
    gate._controller_owners["arm_controller"] = command_id
    gate._send_goal_futures["arm_controller"] = send_future

    gate._on_goal_response(
        "arm_controller",
        send_future,
        command_id,
        command_id,
        send_future,
    )

    assert context.event.is_set() and context.terminal
    assert not context.success
    assert context.reason == "result_request_failed_stop_unconfirmed:RuntimeError"
    assert context.cancel_reason == "result_request_failed"
    assert downstream.cancel_calls == 1
    tombstone = gate._typed_tombstones["arm_controller"]
    assert tombstone.command_id == command_id
    assert not tombstone.terminal_confirmed


def test_result_future_exception_completes_typed_failure_and_latches(gate) -> None:
    context = typed_execution("result-future-exception")
    command_id = context.command_id
    downstream = FakeGoalHandle(response=cancel_response(True))
    future = FakeFuture(error=TimeoutError("result"))
    gate._typed_commands["arm_controller"] = context
    gate._controller_owners["arm_controller"] = command_id
    gate._goal_handles["arm_controller"] = downstream
    gate._result_futures["arm_controller"] = future

    gate._on_result(
        "arm_controller", future, command_id, command_id, future
    )

    assert context.event.is_set() and context.terminal
    assert not context.success
    assert context.reason == "result_failed_stop_unconfirmed:TimeoutError"
    assert context.cancel_reason == "result_failed"
    assert downstream.cancel_calls == 1
    tombstone = gate._typed_tombstones["arm_controller"]
    assert tombstone.command_id == command_id
    assert not tombstone.terminal_confirmed


@pytest.mark.parametrize(
    ("status", "fjt_error_code", "fault_prefix"),
    [
        (
            GoalStatus.STATUS_SUCCEEDED,
            FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
            "fjt_execution_failed:arm_controller",
        ),
        (
            GoalStatus.STATUS_ABORTED,
            FollowJointTrajectory.Result.SUCCESSFUL,
            "downstream_action_failed:arm_controller",
        ),
    ],
)
def test_wrapper_and_fjt_result_must_both_report_success(
    gate, status: int, fjt_error_code: int, fault_prefix: str
) -> None:
    downstream = SimpleNamespace(
        error_code=fjt_error_code,
        error_string="synthetic failure",
    )
    wrapped = SimpleNamespace(status=status, result=downstream)
    future = FakeFuture(value=wrapped)
    gate._result_futures["arm_controller"] = future
    gate._goal_handles["arm_controller"] = FakeGoalHandle()
    gate._controller_owners["arm_controller"] = "legacy:test:double-success"
    gate._on_result(
        "arm_controller",
        future,
        "legacy:test:double-success",
        None,
        future,
    )
    assert gate._fault_latched is not None
    assert gate._fault_latched.startswith(fault_prefix)


def test_late_goal_and_result_callbacks_are_ignored_by_future_identity(gate) -> None:
    current_goal = FakeFuture(done=False)
    stale_goal = FakeFuture(error=RuntimeError("must not be observed"))
    gate._send_goal_futures["arm_controller"] = current_goal
    gate._controller_owners["arm_controller"] = "current-dispatch"
    gate._on_goal_response(
        "arm_controller",
        stale_goal,
        "old-dispatch",
        "old-command",
        stale_goal,
    )
    assert gate._send_goal_futures["arm_controller"] is current_goal
    assert gate._fault_latched is None

    current_result = FakeFuture(done=False)
    stale_result = FakeFuture(error=RuntimeError("must not be observed"))
    gate._result_futures["arm_controller"] = current_result
    gate._on_result(
        "arm_controller",
        stale_result,
        "old-dispatch",
        "old-command",
        stale_result,
    )
    assert gate._result_futures["arm_controller"] is current_result
    assert gate._fault_latched is None


@pytest.mark.parametrize(
    ("cancel_reason", "status", "expected_reason", "fault"),
    [
        (
            "client_cancel_requested",
            GoalStatus.STATUS_SUCCEEDED,
            "cancel_race_downstream_succeeded:client_cancel_requested",
            "cancel_race_downstream_succeeded:arm_controller",
        ),
        (
            None,
            GoalStatus.STATUS_CANCELED,
            "unexpected_downstream_canceled",
            "unexpected_downstream_canceled:arm_controller",
        ),
    ],
)
def test_cancel_success_race_and_unexpected_cancel_fail_closed(
    gate,
    cancel_reason: str | None,
    status: int,
    expected_reason: str,
    fault: str,
) -> None:
    command_id = "race-task|approach|0"
    context = _TypedExecution(
        goal_handle=SimpleNamespace(request=typed_goal("race-task")),
        task_id="race-task",
        command_id=command_id,
        target_id="cube-1",
        stage="approach",
        sequence_no=0,
        controller="arm_controller",
        source_timestamp_ns=1,
        trajectory_digest="0" * 64,
        accepted=True,
        cancel_reason=cancel_reason,
    )
    future = FakeFuture(
        value=SimpleNamespace(
            status=status,
            result=SimpleNamespace(
                error_code=FollowJointTrajectory.Result.SUCCESSFUL,
                error_string="",
            ),
        )
    )
    gate._typed_commands["arm_controller"] = context
    gate._controller_owners["arm_controller"] = command_id
    gate._result_futures["arm_controller"] = future
    gate._goal_handles["arm_controller"] = FakeGoalHandle()

    gate._on_result(
        "arm_controller", future, command_id, command_id, future
    )
    assert context.terminal and context.downstream_terminal_observed
    assert not context.success
    assert context.reason == expected_reason
    assert gate._fault_latched == fault
