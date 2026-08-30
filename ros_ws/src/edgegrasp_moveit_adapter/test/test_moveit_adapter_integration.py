"""ROS-runtime tests for the plan-only MoveIt-to-gate boundary.

These tests use fake ``/compute_ik`` and ``/move_action`` servers.  They prove
the adapter's ROS contract and fail-closed publication behavior; they do not
prove Gazebo physics, the upstream kinematics plugin, or physical stopping.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import threading
import time
from itertools import count
import uuid

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
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from edgegrasp_moveit_adapter.adapter_node import MoveItPlanOnlyAdapter


START = (0.0, 0.0, 0.0, 0.0, 0.0)
IK_GOAL = (0.05, -0.05, 0.05, -0.05, 0.0)
DOMAIN_IDS = count(151)


class _InjectedGateResultExceptionFuture:
    """Minimal done future whose terminal result cannot be decoded."""

    @staticmethod
    def done() -> bool:
        return True

    @staticmethod
    def result():
        raise RuntimeError("injected gate result future failure")


def goal_uuid_hex(goal_handle: object) -> str | None:
    """Return an action goal UUID as lowercase hex for evidence joins.

    ``rclpy`` exposes UUIDs as ``unique_identifier_msgs.msg.UUID`` on some
    goal-handle implementations and as a bytes-like value on others.  Keep
    the conversion in the fake harness so the test evidence never depends on
    the repr of a generated message object.
    """

    goal_id = getattr(goal_handle, "goal_id", None)
    if goal_id is None:
        return None
    raw_uuid = getattr(goal_id, "uuid", goal_id)
    try:
        if isinstance(raw_uuid, uuid.UUID):
            return raw_uuid.hex
        if isinstance(raw_uuid, str):
            return uuid.UUID(raw_uuid).hex
        if hasattr(raw_uuid, "bytes") and not isinstance(
            raw_uuid, (bytes, bytearray)
        ):
            raw_uuid = raw_uuid.bytes
        value = bytes(raw_uuid)
    except (TypeError, ValueError, AttributeError):
        return None
    if len(value) != 16:
        return None
    return value.hex()


def goal_status_name(status: int) -> str:
    names = {
        GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
        GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
        GoalStatus.STATUS_EXECUTING: "EXECUTING",
        GoalStatus.STATUS_CANCELING: "CANCELING",
        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
        GoalStatus.STATUS_CANCELED: "CANCELED",
        GoalStatus.STATUS_ABORTED: "ABORTED",
    }
    return names.get(int(status), f"STATUS_{int(status)}")


def _append_injected_event(
    scenario: str,
    harness: "AdapterHarness",
    adapter_goal_handle,
    wrapped,
    *,
    request_id: str,
    checks: dict[str, bool] | None = None,
) -> None:
    """Append one JSONL event when the injected runner requests it.

    The log is deliberately opt-in.  It is a test-run join of the PlanTarget
    wrapper, adapter status topic, and fake ActionServer ledger; it is not a
    controller or hardware trace.
    """

    log_path = os.environ.get("EDGEGRASP_INJECTED_EVENT_LOG")
    if not log_path:
        return

    result = wrapped.result
    move_group_records = list(harness.dependencies.move_group_goal_records)
    gate_records = list(harness.dependencies.gate_goal_records)
    move_group_server_goal_ids = [
        record["server_goal_id"]
        for record in move_group_records
        if record.get("server_goal_id")
    ]
    gate_server_goal_ids = [
        record["server_goal_id"]
        for record in gate_records
        if record.get("server_goal_id")
    ]
    constraint_names = [
        record["constraint_name"]
        for record in move_group_records
        if record.get("constraint_name")
    ]
    status_events = [
        event
        for event in harness.adapter_status_events
        if event.get("request_id") in {request_id, "<none>"}
    ]
    correlated_status = next(
        (
            event
            for event in reversed(status_events)
            if event.get("request_id") == request_id
            and event.get("attempt_generation") is not None
            and (
                event.get("move_group_goal_id") is not None
                or event.get("gate_goal_id") is not None
            )
        ),
        {},
    )
    relevant_server_goal_ids = (
        gate_server_goal_ids
        if scenario.startswith("gate_")
        else move_group_server_goal_ids
    )
    relevant_constraint_names = (
        [] if scenario.startswith("gate_") else constraint_names
    )
    server_goal_id = (
        relevant_server_goal_ids[0]
        if len(relevant_server_goal_ids) == 1
        else relevant_server_goal_ids
    )
    move_group_client_goal_ids = [
        record.get("client_goal_id") for record in move_group_records
    ]
    gate_client_goal_ids = [
        record.get("client_goal_id") for record in gate_records
    ]
    plan_target_goal_id = goal_uuid_hex(adapter_goal_handle)
    assert_canonical_goal_uuid(plan_target_goal_id)
    for record in (*move_group_records, *gate_records):
        assert isinstance(record.get("attempt_generation"), int)
        assert record["attempt_generation"] >= 1
        assert_canonical_goal_uuid(record.get("server_goal_id"))
        assert_canonical_goal_uuid(record.get("client_goal_id"))
    constraint_name = (
        relevant_constraint_names[0]
        if len(relevant_constraint_names) == 1
        else relevant_constraint_names
    )
    event = {
        "schema_version": 2,
        "scenario": scenario,
        "request_id": request_id,
        "plan_target_goal_id": plan_target_goal_id,
        "adapter_goal_id": plan_target_goal_id,
        "server_goal_id": server_goal_id,
        "move_group_server_goal_ids": move_group_server_goal_ids,
        "gate_server_goal_ids": gate_server_goal_ids,
        "move_group_client_goal_ids": move_group_client_goal_ids,
        "gate_client_goal_ids": gate_client_goal_ids,
        "move_group_client_goal_id": (
            move_group_client_goal_ids[0]
            if len(move_group_client_goal_ids) == 1
            else move_group_client_goal_ids
        ),
        "move_group_server_goal_id": (
            move_group_server_goal_ids[0]
            if len(move_group_server_goal_ids) == 1
            else move_group_server_goal_ids
        ),
        "gate_client_goal_id": (
            gate_client_goal_ids[0]
            if len(gate_client_goal_ids) == 1
            else gate_client_goal_ids
        ),
        "gate_server_goal_id": (
            gate_server_goal_ids[0]
            if len(gate_server_goal_ids) == 1
            else gate_server_goal_ids
        ),
        "constraint_name": constraint_name,
        "constraint_names": relevant_constraint_names,
        "move_group": {
            "request_id": request_id,
            "client_goal_ids": move_group_client_goal_ids,
            "server_goal_ids": move_group_server_goal_ids,
            "constraint_names": constraint_names,
            "attempt_generations": [
                record.get("attempt_generation")
                for record in move_group_records
            ],
        },
        "gate": {
            "request_id": request_id,
            "client_goal_ids": gate_client_goal_ids,
            "server_goal_ids": gate_server_goal_ids,
            "attempt_generations": [
                record.get("attempt_generation") for record in gate_records
            ],
        },
        "attempt_generation": correlated_status.get("attempt_generation"),
        "attempt_generations": [
            record.get("attempt_generation")
            for record in (*move_group_records, *gate_records)
        ],
        "move_group_goal_id": correlated_status.get("move_group_goal_id"),
        "gate_goal_id": correlated_status.get("gate_goal_id"),
        "goal_response_future_pending_at_timeout": correlated_status.get(
            "goal_response_future_pending_at_timeout"
        ),
        "result_future_pending_at_timeout": correlated_status.get(
            "result_future_pending_at_timeout"
        ),
        "wrapper": {
            "status": int(wrapped.status),
            "status_name": goal_status_name(wrapped.status),
            "reason": result.reason,
            "trajectory_dispatched": bool(result.trajectory_dispatched),
            "gate_accepted": bool(result.gate_accepted),
            "gate_terminal": bool(result.gate_terminal),
            "downstream_terminal_observed": bool(
                result.downstream_terminal_observed
            ),
            "cancel_requested": bool(result.cancel_requested),
            "action_goal_status": int(result.action_goal_status),
        },
        # Keep flat aliases for simple line-oriented consumers.
        "wrapper_status": int(wrapped.status),
        "wrapper_reason": result.reason,
        "trajectory_dispatched": bool(result.trajectory_dispatched),
        "cancel": {
            "requested": bool(result.cancel_requested),
            "move_group_requests": harness.dependencies.cancel_requests,
            "gate_requests": harness.dependencies.gate_cancel_requests,
            "response_accepted": correlated_status.get(
                "cancel_response_accepted"
            ),
        },
        "terminal": {
            "move_group": {
                "goals": move_group_records,
                "server_goal_ids": move_group_server_goal_ids,
            },
            "gate": {
                "goals": gate_records,
                "server_goal_ids": gate_server_goal_ids,
            },
        },
        "counters": {
            "move_group_goals": len(harness.dependencies.move_group_goals),
            "move_group_server_goals": len(move_group_records),
            "gate_goals": len(harness.dependencies.gate_goals),
            "gate_server_goals": len(gate_records),
            "published_trajectories": len(
                harness.dependencies.published_trajectories
            ),
            "cancel_requests": harness.dependencies.cancel_requests,
            "gate_cancel_requests": harness.dependencies.gate_cancel_requests,
        },
        "adapter_status_events": status_events,
        "checks": dict(checks or {}),
        "evidence_class": "INJECTED_IN_PROCESS_FAKE",
        "scope": {
            "injected_runtime_only": True,
            "in_process_fake": True,
            "real_move_group": False,
            "controller": False,
            "simulation_physics": False,
            "hardware": False,
        },
    }
    path = Path(log_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(event, sort_keys=True, default=str) + "\n")


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


def wait_for_status_event(
    harness: "AdapterHarness",
    request_id: str,
    *,
    stage: str | None = None,
    reason: str | None = None,
    move_group_goal_id: str | None = None,
    gate_goal_id: str | None = None,
) -> dict[str, object]:
    def matching_event() -> dict[str, object] | None:
        for event in reversed(harness.adapter_status_events):
            if event.get("request_id") != request_id:
                continue
            if stage is not None and event.get("stage") != stage:
                continue
            if reason is not None and event.get("reason") != reason:
                continue
            if not isinstance(event.get("attempt_generation"), int):
                continue
            if int(event["attempt_generation"]) < 1:
                continue
            if move_group_goal_id is not None and event.get(
                "move_group_goal_id"
            ) != move_group_goal_id:
                continue
            if gate_goal_id is not None and event.get("gate_goal_id") != gate_goal_id:
                continue
            return event
        return None

    result: dict[str, object] | None = None

    def observed() -> bool:
        nonlocal result
        result = matching_event()
        return result is not None

    wait_until(observed, reason=f"adapter status event {stage or reason}")
    assert result is not None
    return result


def assert_canonical_goal_uuid(value: object) -> str:
    assert isinstance(value, str)
    assert len(value) == 32
    assert value == value.lower()
    bytes.fromhex(value)
    return value


def wait_for_move_group_identity(
    harness: "AdapterHarness",
    request_id: str,
    *,
    after_generation: int | None = None,
    **status_filters,
) -> dict[str, object]:
    def matching_record(record: dict[str, object]) -> bool:
        generation = record.get("attempt_generation")
        return (
            record.get("request_id") == request_id
            and record.get("server_goal_id") is not None
            and isinstance(generation, int)
            and (after_generation is None or generation > after_generation)
        )

    def record_ready() -> bool:
        return any(
            matching_record(record)
            for record in harness.dependencies.move_group_goal_records
        )

    wait_until(record_ready, reason="fake MoveGroup server goal identity")
    record = next(
        record
        for record in reversed(harness.dependencies.move_group_goal_records)
        if matching_record(record)
    )
    server_goal_id = assert_canonical_goal_uuid(record.get("server_goal_id"))
    status = wait_for_status_event(
        harness,
        request_id,
        move_group_goal_id=server_goal_id,
        **status_filters,
    )
    record["status_move_group_goal_id"] = status["move_group_goal_id"]

    def client_identity_ready() -> bool:
        return record.get("client_goal_id") is not None

    wait_until(client_identity_ready, reason="MoveGroup client goal identity")
    client_goal_id = assert_canonical_goal_uuid(record.get("client_goal_id"))
    assert client_goal_id == server_goal_id
    assert record.get("constraint_name") == f"edgegrasp:{request_id}"
    assert record.get("attempt_generation") == status["attempt_generation"]
    assert status["move_group_goal_id"] == server_goal_id
    return status


def wait_for_gate_identity(
    harness: "AdapterHarness", request_id: str, **status_filters
) -> dict[str, object]:
    def record_ready() -> bool:
        return any(
            record.get("request_id") == request_id
            and record.get("server_goal_id") is not None
            for record in harness.dependencies.gate_goal_records
        )

    wait_until(record_ready, reason="fake gate server goal identity")
    record = next(
        record
        for record in reversed(harness.dependencies.gate_goal_records)
        if record.get("request_id") == request_id
    )
    server_goal_id = assert_canonical_goal_uuid(record.get("server_goal_id"))
    status = wait_for_status_event(
        harness,
        request_id,
        gate_goal_id=server_goal_id,
        **status_filters,
    )
    record["status_gate_goal_id"] = status["gate_goal_id"]
    wait_until(
        lambda: record.get("client_goal_id") is not None,
        reason="gate client goal identity",
    )
    client_goal_id = assert_canonical_goal_uuid(record.get("client_goal_id"))
    assert client_goal_id == server_goal_id
    assert record.get("attempt_generation") == status["attempt_generation"]
    assert status["gate_goal_id"] == server_goal_id
    return status


class FakeMoveIt(Node):
    def __init__(
        self,
        *,
        ik_error: int = MoveItErrorCodes.SUCCESS,
        ik_delay_s: float = 0.0,
        move_group_mode: str = "success",
        move_group_goal_response_delay_s: float = 0.0,
        gate_mode: str = "success",
        gate_goal_response_delay_s: float = 0.0,
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
        self.clock_epoch = 0
        self.ik_delay_s = ik_delay_s
        self.move_group_mode = move_group_mode
        self.move_group_goal_response_delay_s = move_group_goal_response_delay_s
        self.gate_mode = gate_mode
        self.gate_goal_response_delay_s = gate_goal_response_delay_s
        self.cancel_response = cancel_response
        self.move_group_goals: list[MoveGroup.Goal] = []
        self.move_group_server_goal_ids: list[str | None] = []
        self.move_group_constraint_names: list[str] = []
        self.move_group_request_ids: list[str] = []
        self.move_group_terminal_statuses: dict[str, int] = {}
        self.move_group_goal_records: list[dict[str, object]] = []
        self.move_group_client_goal_ids: list[str | None] = []
        self._pending_move_group_client_goal_ids: dict[
            str, list[tuple[str | None, int]]
        ] = {}
        self._pending_move_group_attempt_generations: dict[
            str, list[int]
        ] = {}
        self.cancel_requests = 0
        self.move_group_cancelled_server_goal_ids: list[str | None] = []
        self.gate_cancel_requests = 0
        self.published_trajectories: list[JointTrajectory] = []
        self.gate_goals: list[ExecuteTrajectory.Goal] = []
        self.gate_server_goal_ids: list[str | None] = []
        self.gate_terminal_statuses: dict[str, int] = {}
        self.gate_goal_records: list[dict[str, object]] = []
        self.gate_client_goal_ids: list[str | None] = []
        self._pending_gate_client_goal_ids: dict[
            str, list[tuple[str | None, int]]
        ] = {}
        self._pending_gate_attempt_generations: dict[str, list[int]] = {}
        self.gate_cancelled_server_goal_ids: list[str | None] = []
        self.contact_policy_requests: list[bool] = []
        self.goal_started = threading.Event()
        self.release_goal = threading.Event()
        self.release_old_generation_goal = threading.Event()
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
                "clock_epoch": self.clock_epoch,
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
        if self.gate_goal_response_delay_s > 0.0:
            time.sleep(self.gate_goal_response_delay_s)
        return GoalResponse.ACCEPT

    def _execute_gate(self, goal_handle):
        request = goal_handle.request
        server_goal_id = goal_uuid_hex(goal_handle)
        self.gate_server_goal_ids.append(server_goal_id)
        record = {
            "server_goal_id": server_goal_id,
            "client_goal_id": None,
            "request_id": request.command_id,
            "constraint_name": None,
            "cancel_requested": False,
            "terminal_status": None,
            "terminal_status_name": None,
            "result_success_payload": False,
            "result_error_code": None,
        }
        pending_generations = self._pending_gate_attempt_generations.get(
            request.command_id, []
        )
        if pending_generations:
            record["attempt_generation"] = pending_generations.pop(0)
        pending_client_ids = self._pending_gate_client_goal_ids.get(
            request.command_id, []
        )
        if pending_client_ids:
            client_goal_id, attempt_generation = pending_client_ids.pop(0)
            if record.get("attempt_generation") != attempt_generation:
                raise AssertionError("gate client/server generation mismatch")
            record["client_goal_id"] = client_goal_id
        self.gate_goal_records.append(record)
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
            terminal_status = GoalStatus.STATUS_CANCELED
        elif result.success:
            goal_handle.succeed()
            terminal_status = GoalStatus.STATUS_SUCCEEDED
        else:
            goal_handle.abort()
            terminal_status = GoalStatus.STATUS_ABORTED
        record["cancel_requested"] = bool(goal_handle.is_cancel_requested)
        record["result_success_payload"] = bool(result.success)
        record["result_error_code"] = int(result.fjt_error_code)
        record["terminal_status"] = int(terminal_status)
        record["terminal_status_name"] = goal_status_name(terminal_status)
        if server_goal_id is not None:
            self.gate_terminal_statuses[server_goal_id] = int(terminal_status)
        return result

    def _cancel_gate(self, goal_handle) -> CancelResponse:
        self.gate_cancel_requests += 1
        self.gate_cancelled_server_goal_ids.append(goal_uuid_hex(goal_handle))
        server_goal_id = goal_uuid_hex(goal_handle)
        for record in self.gate_goal_records:
            if record.get("server_goal_id") == server_goal_id:
                record["cancel_requested"] = True
        return CancelResponse.ACCEPT

    def record_move_group_client_goal_id(
        self,
        request_id: str,
        client_goal_id: str | None,
        attempt_generation: int,
    ) -> None:
        self.move_group_client_goal_ids.append(client_goal_id)
        for record in reversed(self.move_group_goal_records):
            if (
                record.get("request_id") == request_id
                and record.get("attempt_generation") == attempt_generation
                and record.get("client_goal_id") is None
            ):
                record["client_goal_id"] = client_goal_id
                record["attempt_generation"] = attempt_generation
                break
        else:
            self._pending_move_group_client_goal_ids.setdefault(
                request_id, []
            ).append((client_goal_id, attempt_generation))

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
        if self.move_group_goal_response_delay_s > 0.0:
            time.sleep(self.move_group_goal_response_delay_s)
        if self.move_group_mode == "reject":
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_move_group(self, goal_handle) -> CancelResponse:
        self.cancel_requests += 1
        self.move_group_cancelled_server_goal_ids.append(
            goal_uuid_hex(goal_handle)
        )
        server_goal_id = goal_uuid_hex(goal_handle)
        for record in self.move_group_goal_records:
            if record.get("server_goal_id") == server_goal_id:
                record["cancel_api_requested"] = True
                record["cancel_requested"] = True
        return self.cancel_response

    def _execute_move_group(self, goal_handle):
        request = goal_handle.request
        server_goal_id = goal_uuid_hex(goal_handle)
        constraints = request.request.goal_constraints
        constraint_name = constraints[0].name if constraints else ""
        request_id = (
            constraint_name.removeprefix("edgegrasp:")
            if constraint_name.startswith("edgegrasp:")
            else ""
        )
        self.move_group_server_goal_ids.append(server_goal_id)
        self.move_group_constraint_names.append(constraint_name)
        self.move_group_request_ids.append(request_id)
        record = {
            "server_goal_id": server_goal_id,
            "client_goal_id": None,
            "request_id": request_id,
            "constraint_name": constraint_name,
            "cancel_api_requested": False,
            "cancel_requested": False,
            "terminal_status": None,
            "terminal_status_name": None,
            "result_success_payload": False,
            "result_error_code": None,
        }
        pending_generations = self._pending_move_group_attempt_generations.get(
            request_id, []
        )
        if pending_generations:
            record["attempt_generation"] = pending_generations.pop(0)
        pending_client_ids = self._pending_move_group_client_goal_ids.get(
            request_id, []
        )
        if pending_client_ids:
            client_goal_id, attempt_generation = pending_client_ids.pop(0)
            if record.get("attempt_generation") != attempt_generation:
                raise AssertionError(
                    "MoveGroup client/server generation mismatch"
                )
            record["client_goal_id"] = client_goal_id
        self.move_group_goal_records.append(record)
        self.goal_started.set()
        first_generation_hold = (
            self.move_group_mode == "first_cancel_success_then_success"
            and len(self.move_group_goal_records) == 1
        )
        synthetic_success_after_cancel = self.move_group_mode in {
            "success_after_cancel",
            "first_cancel_success_then_success",
        }
        if self.move_group_mode == "hold":
            while (
                not goal_handle.is_cancel_requested
                and not self.release_goal.is_set()
            ):
                time.sleep(0.005)
        elif self.move_group_mode == "hold_after_cancel" or first_generation_hold:
            while not self.release_goal.is_set():
                time.sleep(0.005)
        elif (
            self.move_group_mode == "old_generation_late_success_then_success"
            and len(self.move_group_goal_records) == 1
        ):
            while not self.release_old_generation_goal.is_set():
                time.sleep(0.005)
        elif self.move_group_mode == "old_generation_late_success_then_success":
            while not self.release_goal.is_set():
                time.sleep(0.005)
        elif self.move_group_mode == "first_cancel_success_then_success":
            while not self.release_goal.is_set():
                time.sleep(0.005)
        elif self.move_group_mode == "success_after_cancel":
            while (
                not goal_handle.is_cancel_requested
                and not self.release_goal.is_set()
            ):
                time.sleep(0.005)
        result = MoveGroup.Result()
        result.error_code.val = MoveItErrorCodes.SUCCESS
        if goal_handle.is_cancel_requested:
            if synthetic_success_after_cancel:
                # The payload is intentionally SUCCESS while the action
                # protocol terminal is CANCELED.  This is a cancel/result
                # race fixture; the adapter must never dispatch this result.
                result.error_code.val = MoveItErrorCodes.SUCCESS
                result_success_payload = True
            else:
                result_success_payload = False
            goal_handle.canceled()
            terminal_status = GoalStatus.STATUS_CANCELED
            record["cancel_requested"] = True
            record["result_success_payload"] = result_success_payload
            record["result_error_code"] = int(result.error_code.val)
            record["terminal_status"] = int(terminal_status)
            record["terminal_status_name"] = goal_status_name(terminal_status)
            if server_goal_id is not None:
                self.move_group_terminal_statuses[server_goal_id] = int(
                    terminal_status
                )
            return result
        if self.move_group_mode == "abort":
            result.error_code.val = MoveItErrorCodes.PLANNING_FAILED
            goal_handle.abort()
            terminal_status = GoalStatus.STATUS_ABORTED
            record["result_error_code"] = int(result.error_code.val)
            record["terminal_status"] = int(terminal_status)
            record["terminal_status_name"] = goal_status_name(terminal_status)
            if server_goal_id is not None:
                self.move_group_terminal_statuses[server_goal_id] = int(
                    terminal_status
                )
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
        terminal_status = GoalStatus.STATUS_SUCCEEDED
        record["result_error_code"] = int(result.error_code.val)
        record["result_success_payload"] = True
        record["cancel_requested"] = bool(goal_handle.is_cancel_requested)
        record["terminal_status"] = int(terminal_status)
        record["terminal_status_name"] = goal_status_name(terminal_status)
        if server_goal_id is not None:
            self.move_group_terminal_statuses[server_goal_id] = int(
                terminal_status
            )
        return result

    def close(self) -> None:
        self.release_goal.set()
        self.release_old_generation_goal.set()
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
        move_group_goal_response_delay_s: float = 0.0,
        gate_mode: str = "success",
        gate_goal_response_delay_s: float = 0.0,
        cancel_response: CancelResponse = CancelResponse.ACCEPT,
        publish_gate_subscriber: bool = True,
        target_timeout_ms: float = 2_000.0,
        ik_delay_s: float = 0.0,
        ik_response_timeout_ms: float = 200.0,
        goal_response_timeout_ms: float = 200.0,
        result_timeout_margin_ms: float = 200.0,
        move_group_terminal_timeout_ms: float = 200.0,
        move_group_result_future_none: bool = False,
        move_group_result_request_exception: bool = False,
        gate_result_future_none: bool = False,
        gate_result_exception: bool = False,
    ) -> None:
        self.context = Context()
        rclpy.init(context=self.context, domain_id=next(DOMAIN_IDS))
        self._clock_ns = 1_000_000_000
        self.dependencies = FakeMoveIt(
            ik_error=ik_error,
            ik_delay_s=ik_delay_s,
            move_group_mode=move_group_mode,
            move_group_goal_response_delay_s=move_group_goal_response_delay_s,
            gate_mode=gate_mode,
            gate_goal_response_delay_s=gate_goal_response_delay_s,
            cancel_response=cancel_response,
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
                Parameter(
                    "goal_response_timeout_ms", value=goal_response_timeout_ms
                ),
                Parameter("cancel_response_timeout_ms", value=200.0),
                Parameter(
                    "result_timeout_margin_ms", value=result_timeout_margin_ms
                ),
                Parameter(
                    "move_group_terminal_timeout_ms",
                    value=move_group_terminal_timeout_ms,
                ),
            ]
        )
        self._source_ledger_lock = threading.Lock()
        self._source_attempt_generations: dict[str, int] = {}
        if move_group_result_request_exception:
            self.adapter._get_move_group_result_future = (
                self._raise_move_group_result_request
            )
        if move_group_result_future_none:
            self.adapter._get_move_group_result_future = (
                self._return_no_move_group_result_future
            )
        if gate_result_future_none:
            self.adapter._get_gate_result_future = self._return_no_gate_result_future
        if gate_result_exception:
            self.adapter._get_gate_result_future = (
                self._return_gate_result_exception_future
            )
        self._original_move_group_send_goal_async = (
            self.adapter._move_group.send_goal_async
        )
        self._original_gate_send_goal_async = (
            self.adapter._trajectory_gate.send_goal_async
        )
        self.adapter._move_group.send_goal_async = (
            self._capture_move_group_send_goal_async
        )
        self.adapter._trajectory_gate.send_goal_async = (
            self._capture_gate_send_goal_async
        )
        self.client = Node(
            "edgegrasp_moveit_adapter_test_client",
            context=self.context,
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.plan_client = ActionClient(
            self.client, PlanTarget, "/edgegrasp/plan_target"
        )
        self.reset_client = self.client.create_client(
            Trigger, "/edgegrasp/reset_moveit_adapter_epoch"
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
        self.adapter_status_events: list[dict[str, object]] = []
        self.adapter_status_messages: list[str] = []
        self.adapter_status = self.client.create_subscription(
            String,
            "/edgegrasp/moveit_adapter_status",
            self._on_adapter_status,
            10,
        )
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
        wait_until(
            lambda: self.adapter_status.get_publisher_count() == 1,
            reason="adapter status publisher",
        )
        wait_until(
            self.reset_client.service_is_ready,
            reason="adapter reset service",
        )
        self.publish_clock(self._clock_ns)

    @staticmethod
    def _raise_move_group_result_request(goal_handle):
        del goal_handle
        raise RuntimeError("injected MoveGroup result future request failure")

    @staticmethod
    def _return_no_move_group_result_future(goal_handle):
        del goal_handle
        return None

    @staticmethod
    def _return_no_gate_result_future(goal_handle):
        del goal_handle
        return None

    @staticmethod
    def _return_gate_result_exception_future(goal_handle):
        del goal_handle
        return _InjectedGateResultExceptionFuture()

    def _on_adapter_status(self, message: String) -> None:
        self.adapter_status_messages.append(message.data)
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if isinstance(payload, dict):
            self.adapter_status_events.append(payload)

    def _capture_move_group_send_goal_async(
        self,
        goal,
        *,
        request_id: str | None = None,
        attempt_generation: int | None = None,
    ):
        constraints = goal.request.goal_constraints
        constraint_name = constraints[0].name if constraints else ""
        parsed_request_id = (
            constraint_name.removeprefix("edgegrasp:")
            if constraint_name.startswith("edgegrasp:")
            else ""
        )
        if request_id is not None:
            assert request_id == parsed_request_id
        request_id = parsed_request_id
        with self._source_ledger_lock:
            previous_generation = self._source_attempt_generations.get(
                request_id, 0
            )
            if attempt_generation is None:
                attempt_generation = previous_generation + 1
            else:
                assert attempt_generation >= previous_generation
            self._source_attempt_generations[request_id] = attempt_generation
        assert isinstance(attempt_generation, int)
        assert attempt_generation >= 1
        self.dependencies._pending_move_group_attempt_generations.setdefault(
            request_id, []
        ).append(attempt_generation)
        future = self._original_move_group_send_goal_async(goal)
        future.add_done_callback(
            lambda done_future: self._record_move_group_client_future(
                request_id, attempt_generation, done_future
            )
        )
        return future

    def _record_move_group_client_future(
        self, request_id: str, attempt_generation: int, future
    ) -> None:
        try:
            goal_handle = future.result()
        except Exception:
            return
        if goal_handle is None:
            return
        self.dependencies.record_move_group_client_goal_id(
            request_id, goal_uuid_hex(goal_handle), attempt_generation
        )

    def _capture_gate_send_goal_async(
        self,
        goal,
        *,
        request_id: str | None = None,
        attempt_generation: int | None = None,
    ):
        parsed_request_id = goal.command_id
        if request_id is not None:
            assert request_id == parsed_request_id
        request_id = parsed_request_id
        with self._source_ledger_lock:
            source_generation = self._source_attempt_generations.get(request_id)
            if attempt_generation is None:
                attempt_generation = source_generation
            elif source_generation is None:
                self._source_attempt_generations[request_id] = attempt_generation
            else:
                assert attempt_generation == source_generation
        assert isinstance(attempt_generation, int)
        assert attempt_generation >= 1
        self.dependencies._pending_gate_attempt_generations.setdefault(
            request_id, []
        ).append(attempt_generation)
        future = self._original_gate_send_goal_async(goal)
        future.add_done_callback(
            lambda done_future: self._record_gate_client_future(
                request_id, attempt_generation, done_future
            )
        )
        return future

    def _record_gate_client_future(
        self, request_id: str, attempt_generation: int, future
    ) -> None:
        try:
            goal_handle = future.result()
        except Exception:
            return
        if goal_handle is None:
            return
        client_goal_id = goal_uuid_hex(goal_handle)
        self.dependencies.gate_client_goal_ids.append(client_goal_id)
        for record in reversed(self.dependencies.gate_goal_records):
            if (
                record.get("request_id") == request_id
                and record.get("attempt_generation") == attempt_generation
                and record.get("client_goal_id") is None
            ):
                record["client_goal_id"] = client_goal_id
                break
        else:
            self.dependencies._pending_gate_client_goal_ids.setdefault(
                request_id, []
            ).append((client_goal_id, attempt_generation))

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
        goal.clock_epoch = int(self.adapter._clock_epoch)
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

    def reset_adapter(self) -> None:
        response = wait_future(self.reset_client.call_async(Trigger.Request()))
        assert response.success
        self.dependencies.clock_epoch = int(self.adapter._clock_epoch)
        self.publish_inputs()

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
        assert result.reason == "plan_target_cancel_requested:gate_result_after_cancel"
        assert harness.dependencies.gate_cancel_requests == 1
    finally:
        harness.close()


def test_injected_stale_move_group_request_cancels_and_never_publishes() -> None:
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


def test_injected_move_group_goal_response_timeout_cancels_late_accepted_goal() -> None:
    harness = AdapterHarness(
        move_group_mode="hold",
        move_group_goal_response_delay_s=0.35,
        goal_response_timeout_ms=50.0,
    )
    try:
        harness.publish_inputs()
        goal = harness.goal("delayed-goal-response")
        request_id = "task-delayed-goal-response|diagnostic|0"
        goal_handle = harness.send(goal)
        assert goal_handle.accepted
        plan_target_goal_id = assert_canonical_goal_uuid(
            goal_uuid_hex(goal_handle)
        )
        wrapped = wait_future(goal_handle.get_result_async())
        assert wrapped.status == GoalStatus.STATUS_ABORTED
        assert wrapped.result.reason == "move_group_timeout"
        status = wait_for_move_group_identity(
            harness,
            request_id,
            stage="late_move_group_goal_response",
            reason="accepted_cancel_requested",
        )
        assert status["result_future_pending_at_timeout"] is None
        assert status["goal_response_future_pending_at_timeout"] is True
        assert status["gate_goal_response_future_pending_at_timeout"] is None
        assert status["move_group_cancel_requested"] is True
        wait_until(
            lambda: harness.dependencies.cancel_requests == 1,
            reason="late fake MoveGroup goal cancellation",
        )
        server_goal_id = assert_canonical_goal_uuid(
            harness.dependencies.move_group_server_goal_ids[0]
        )
        wait_until(
            lambda: harness.dependencies.move_group_terminal_statuses.get(
                server_goal_id
            )
            == GoalStatus.STATUS_CANCELED,
            reason="late fake MoveGroup terminal",
        )
        cancel_status = wait_for_status_event(
            harness,
            request_id,
            stage="late_move_group_goal_cancel_response",
            reason="accepted",
            move_group_goal_id=status["move_group_goal_id"],
        )
        assert cancel_status["cancel_response_accepted"] is True
        terminal_status = wait_for_status_event(
            harness,
            request_id,
            stage="late_move_group_goal_terminal",
            reason="observed",
            move_group_goal_id=status["move_group_goal_id"],
        )
        assert terminal_status["move_group_terminal_observed"] is True
        assert (
            terminal_status["move_group_terminal_status"]
            == GoalStatus.STATUS_CANCELED
        )
        assert terminal_status["action_goal_status"] == GoalStatus.STATUS_CANCELED
        assert harness.dependencies.published_trajectories == []
        assert harness.dependencies.gate_goals == []
        assert plan_target_goal_id == goal_uuid_hex(goal_handle)
        _append_injected_event(
            "move_group_goal_response_timeout",
            harness,
            goal_handle,
            wrapped,
            request_id=request_id,
        )
    finally:
        harness.close()


def test_injected_move_group_accepted_result_timeout_is_correlated_and_fail_closed() -> None:
    harness = AdapterHarness(
        move_group_mode="hold",
        target_timeout_ms=2_000.0,
        result_timeout_margin_ms=50.0,
        move_group_terminal_timeout_ms=500.0,
    )
    try:
        harness.publish_inputs()
        goal = harness.goal("accepted-result-timeout")
        goal.planning_timeout_s = 0.05
        request_id = "task-accepted-result-timeout|diagnostic|0"
        goal_handle = harness.send(goal)
        assert goal_handle.accepted
        assert_canonical_goal_uuid(goal_uuid_hex(goal_handle))
        accepted_status = wait_for_move_group_identity(
            harness,
            request_id,
            stage="move_group_goal_accepted",
            reason="result_pending",
        )
        assert accepted_status["attempt_generation"] >= 1
        wrapped = wait_future(goal_handle.get_result_async())
        assert wrapped.status == GoalStatus.STATUS_ABORTED
        assert wrapped.result.reason == "move_group_result_timeout"
        move_group_goal_id = accepted_status["move_group_goal_id"]
        timeout_status = wait_for_status_event(
            harness,
            request_id,
            stage="move_group_result_timeout",
            reason="move_group_result_timeout",
            move_group_goal_id=move_group_goal_id,
        )
        assert timeout_status["result_future_pending_at_timeout"] is True
        assert timeout_status["goal_response_future_pending_at_timeout"] is None
        assert timeout_status["move_group_cancel_requested"] is True
        cancel_status = wait_for_status_event(
            harness,
            request_id,
            stage="cancelled_move_group",
            reason="move_group_result_timeout",
            move_group_goal_id=move_group_goal_id,
        )
        assert cancel_status["cancel_response_accepted"] is True
        assert cancel_status["result_future_pending_at_timeout"] is True
        terminal_status = wait_for_status_event(
            harness,
            request_id,
            stage="move_group_terminal_observed",
            reason="observed_after_cancel",
            move_group_goal_id=move_group_goal_id,
        )
        assert terminal_status["move_group_terminal_observed"] is True
        assert terminal_status["action_goal_status"] == GoalStatus.STATUS_CANCELED
        assert harness.dependencies.cancel_requests == 1
        assert harness.dependencies.gate_goals == []
        assert harness.dependencies.published_trajectories == []
        record = harness.dependencies.move_group_goal_records[0]
        assert record["server_goal_id"] == move_group_goal_id
        assert record["client_goal_id"] == move_group_goal_id
        assert record["constraint_name"] == f"edgegrasp:{request_id}"
        assert record["terminal_status"] == GoalStatus.STATUS_CANCELED
        _append_injected_event(
            "move_group_accepted_result_timeout",
            harness,
            goal_handle,
            wrapped,
            request_id=request_id,
            checks={"move_group_terminal_confirmed": True},
        )
    finally:
        harness.close()


def test_move_group_result_future_none_is_fail_closed() -> None:
    harness = AdapterHarness(
        move_group_mode="hold",
        move_group_result_future_none=True,
        target_timeout_ms=2_000.0,
    )
    try:
        harness.publish_inputs()
        goal = harness.goal("result-future-none")
        request_id = "task-result-future-none|diagnostic|0"
        goal_handle = harness.send(goal)
        assert goal_handle.accepted
        assert_canonical_goal_uuid(goal_uuid_hex(goal_handle))
        accepted_status = wait_for_move_group_identity(
            harness,
            request_id,
            stage="move_group_goal_accepted",
            reason="result_pending",
        )
        move_group_goal_id = assert_canonical_goal_uuid(
            accepted_status["move_group_goal_id"]
        )
        unavailable_status = wait_for_status_event(
            harness,
            request_id,
            stage="move_group_result_future_unavailable",
            reason="accepted_goal_has_no_result_future",
            move_group_goal_id=move_group_goal_id,
        )
        assert unavailable_status["attempt_generation"] == accepted_status[
            "attempt_generation"
        ]
        assert unavailable_status["result_future_pending_at_timeout"] is None
        assert unavailable_status["move_group_cancel_requested"] is True
        assert unavailable_status["move_group_terminal_observed"] is False
        wrapped = wait_future(goal_handle.get_result_async())
        assert wrapped.status == GoalStatus.STATUS_ABORTED
        assert wrapped.result.reason == "move_group_result_future_unavailable"
        assert not wrapped.result.trajectory_dispatched
        assert not wrapped.result.gate_accepted
        assert not wrapped.result.gate_terminal
        assert not wrapped.result.downstream_terminal_observed
        cancel_status = wait_for_status_event(
            harness,
            request_id,
            stage="cancelled_move_group",
            reason="result_future_unavailable",
            move_group_goal_id=move_group_goal_id,
        )
        assert cancel_status["cancel_response_accepted"] is True
        assert cancel_status["result_future_pending_at_timeout"] is None
        terminal_status = wait_for_status_event(
            harness,
            request_id,
            stage="move_group_terminal_unconfirmed",
            reason="result_future_unavailable",
            move_group_goal_id=move_group_goal_id,
        )
        assert terminal_status["move_group_terminal_observed"] is False
        assert terminal_status["result_future_pending_at_timeout"] is None
        assert harness.dependencies.cancel_requests == 1
        assert len(harness.dependencies.move_group_cancelled_server_goal_ids) == 1
        assert (
            harness.dependencies.move_group_cancelled_server_goal_ids[0]
            == move_group_goal_id
        )
        assert harness.dependencies.gate_goals == []
        assert harness.dependencies.published_trajectories == []
        assert len(harness.dependencies.move_group_goal_records) == 1
        record = harness.dependencies.move_group_goal_records[0]
        assert record["server_goal_id"] == move_group_goal_id
        assert record["client_goal_id"] == move_group_goal_id
        assert record["constraint_name"] == f"edgegrasp:{request_id}"
        assert record["attempt_generation"] == accepted_status["attempt_generation"]
        assert record["cancel_api_requested"] is True
        wait_until(
            lambda: harness.dependencies.move_group_terminal_statuses.get(
                move_group_goal_id
            )
            == GoalStatus.STATUS_CANCELED,
            reason="fake MoveGroup terminal after missing result future",
        )
        assert record["terminal_status"] == GoalStatus.STATUS_CANCELED
        _append_injected_event(
            "move_group_result_future_unavailable",
            harness,
            goal_handle,
            wrapped,
            request_id=request_id,
        )
    finally:
        harness.close()


def test_move_group_result_future_request_exception_is_fail_closed() -> None:
    harness = AdapterHarness(
        move_group_mode="hold",
        target_timeout_ms=2_000.0,
        move_group_result_request_exception=True,
    )
    try:
        harness.publish_inputs()
        goal = harness.goal("result-future-request-exception")
        request_id = "task-result-future-request-exception|diagnostic|0"
        goal_handle = harness.send(goal)
        assert goal_handle.accepted
        accepted_status = wait_for_move_group_identity(
            harness,
            request_id,
            stage="move_group_goal_accepted",
            reason="result_pending",
        )
        wrapped = wait_future(goal_handle.get_result_async())
        assert wrapped.status == GoalStatus.STATUS_ABORTED
        assert wrapped.result.reason == "move_group_result_request_exception"
        cancel_status = wait_for_status_event(
            harness,
            request_id,
            stage="cancelled_move_group",
            reason="result_request_exception",
            move_group_goal_id=accepted_status["move_group_goal_id"],
        )
        assert cancel_status["cancel_response_accepted"] is True
        assert cancel_status["result_future_pending_at_timeout"] is None
        terminal_status = wait_for_status_event(
            harness,
            request_id,
            stage="move_group_terminal_unconfirmed",
            reason="result_future_unavailable",
            move_group_goal_id=accepted_status["move_group_goal_id"],
        )
        assert terminal_status["move_group_terminal_observed"] is False
        assert terminal_status["result_future_pending_at_timeout"] is None
        assert harness.dependencies.cancel_requests == 1
        server_goal_id = assert_canonical_goal_uuid(
            harness.dependencies.move_group_server_goal_ids[0]
        )
        wait_until(
            lambda: harness.dependencies.move_group_terminal_statuses.get(
                server_goal_id
            )
            == GoalStatus.STATUS_CANCELED,
            reason="fake MoveGroup terminal after result future exception",
        )
        assert harness.dependencies.gate_goals == []
        assert harness.dependencies.published_trajectories == []
        _append_injected_event(
            "move_group_result_future_exception",
            harness,
            goal_handle,
            wrapped,
            request_id=request_id,
        )
    finally:
        harness.close()


def test_injected_gate_goal_response_timeout_cancels_late_accepted_gate_goal() -> None:
    harness = AdapterHarness(
        move_group_mode="success",
        gate_mode="hold",
        gate_goal_response_delay_s=0.35,
        goal_response_timeout_ms=50.0,
    )
    try:
        harness.publish_inputs()
        goal = harness.goal("delayed-gate-response")
        request_id = "task-delayed-gate-response|diagnostic|0"
        goal_handle = harness.send(goal)
        assert goal_handle.accepted
        assert_canonical_goal_uuid(goal_uuid_hex(goal_handle))
        wait_for_move_group_identity(
            harness,
            request_id,
            stage="move_group_goal_accepted",
            reason="result_pending",
        )
        wrapped = wait_future(goal_handle.get_result_async())
        assert wrapped.status == GoalStatus.STATUS_ABORTED
        assert wrapped.result.reason == "trajectory_gate_goal_response_timeout"
        assert not wrapped.result.trajectory_dispatched
        assert not wrapped.result.gate_accepted
        assert not wrapped.result.gate_terminal
        assert not wrapped.result.downstream_terminal_observed
        assert wrapped.result.action_goal_status == GoalStatus.STATUS_UNKNOWN
        assert wrapped.result.fjt_error_code == -(2**31)
        assert len(harness.dependencies.gate_goals) == 1
        status = wait_for_gate_identity(
            harness,
            request_id,
            stage="late_gate_goal_response",
            reason="accepted_cancel_requested",
        )
        assert status["result_future_pending_at_timeout"] is None
        assert status["goal_response_future_pending_at_timeout"] is None
        assert status["gate_goal_response_future_pending_at_timeout"] is True
        assert status["gate_cancel_requested"] is True
        wait_until(
            lambda: harness.dependencies.gate_cancel_requests == 1,
            reason="late fake gate goal cancellation",
        )
        server_goal_id = assert_canonical_goal_uuid(
            harness.dependencies.gate_server_goal_ids[0]
        )
        wait_until(
            lambda: harness.dependencies.gate_terminal_statuses.get(server_goal_id)
            == GoalStatus.STATUS_CANCELED,
            reason="late fake gate terminal",
        )
        terminal_status = wait_for_status_event(
            harness,
            request_id,
            stage="late_gate_goal_terminal",
            reason="observed",
            gate_goal_id=status["gate_goal_id"],
        )
        assert terminal_status["gate_terminal_observed"] is True
        assert terminal_status["gate_terminal_status"] == GoalStatus.STATUS_CANCELED
        assert terminal_status["action_goal_status"] == GoalStatus.STATUS_CANCELED
        assert len(harness.dependencies.published_trajectories) == 1
        record = harness.dependencies.gate_goal_records[0]
        assert record["server_goal_id"] == status["gate_goal_id"]
        assert record["client_goal_id"] == status["gate_goal_id"]
        assert record["terminal_status"] == GoalStatus.STATUS_CANCELED
        _append_injected_event(
            "gate_goal_response_timeout",
            harness,
            goal_handle,
            wrapped,
            request_id=request_id,
        )
    finally:
        harness.close()


def test_gate_result_future_none_is_fail_closed_and_cancels_exact_goal() -> None:
    harness = AdapterHarness(
        move_group_mode="success",
        gate_mode="hold",
        gate_result_future_none=True,
        move_group_terminal_timeout_ms=100.0,
    )
    try:
        harness.publish_inputs()
        goal = harness.goal("gate-result-future-none")
        request_id = "task-gate-result-future-none|diagnostic|0"
        goal_handle = harness.send(goal)
        assert goal_handle.accepted
        wait_for_move_group_identity(
            harness,
            request_id,
            stage="move_group_goal_accepted",
            reason="result_pending",
        )
        gate_status = wait_for_gate_identity(
            harness,
            request_id,
            stage="gate_goal_accepted",
            reason="result_pending",
        )
        gate_goal_id = gate_status["gate_goal_id"]
        wrapped = wait_future(goal_handle.get_result_async())
        assert wrapped.status == GoalStatus.STATUS_ABORTED
        assert wrapped.result.reason == (
            "trajectory_gate_result_future_unavailable:"
            "gate_terminal_unconfirmed"
        )
        assert wrapped.result.trajectory_dispatched
        assert wrapped.result.gate_accepted
        assert not wrapped.result.gate_terminal
        assert not wrapped.result.downstream_terminal_observed
        assert wrapped.result.cancel_requested
        cancel_status = wait_for_status_event(
            harness,
            request_id,
            stage="cancelled_gate_command",
            reason="accepted",
            gate_goal_id=gate_goal_id,
        )
        assert cancel_status["gate_cancel_requested"] is True
        assert cancel_status["cancel_response_accepted"] is True
        terminal_status = wait_for_status_event(
            harness,
            request_id,
            stage="gate_terminal_unconfirmed",
            reason="result_future_unavailable",
            gate_goal_id=gate_goal_id,
        )
        assert terminal_status["gate_cancel_requested"] is True
        assert terminal_status["gate_terminal_observed"] is False
        assert harness.dependencies.gate_cancel_requests == 1
        server_goal_id = assert_canonical_goal_uuid(
            harness.dependencies.gate_server_goal_ids[0]
        )
        wait_until(
            lambda: harness.dependencies.gate_terminal_statuses.get(
                server_goal_id
            )
            == GoalStatus.STATUS_CANCELED,
            reason="fake gate terminal after unavailable result future",
        )
        record = harness.dependencies.gate_goal_records[0]
        assert record["client_goal_id"] == gate_goal_id
        assert record["server_goal_id"] == gate_goal_id
        assert record["terminal_status"] == GoalStatus.STATUS_CANCELED
        assert len(harness.dependencies.gate_goals) == 1
        assert len(harness.dependencies.published_trajectories) == 1
        _append_injected_event(
            "gate_result_future_unavailable",
            harness,
            goal_handle,
            wrapped,
            request_id=request_id,
        )
    finally:
        harness.close()


def test_gate_result_exception_is_fail_closed_and_cancels_exact_goal() -> None:
    harness = AdapterHarness(
        move_group_mode="success",
        gate_mode="hold",
        gate_result_exception=True,
        move_group_terminal_timeout_ms=100.0,
    )
    try:
        harness.publish_inputs()
        goal = harness.goal("gate-result-exception")
        request_id = "task-gate-result-exception|diagnostic|0"
        goal_handle = harness.send(goal)
        assert goal_handle.accepted
        wait_for_move_group_identity(
            harness,
            request_id,
            stage="move_group_goal_accepted",
            reason="result_pending",
        )
        gate_status = wait_for_gate_identity(
            harness,
            request_id,
            stage="gate_goal_accepted",
            reason="result_pending",
        )
        gate_goal_id = gate_status["gate_goal_id"]
        wrapped = wait_future(goal_handle.get_result_async())
        assert wrapped.status == GoalStatus.STATUS_ABORTED
        assert wrapped.result.reason == (
            "trajectory_gate_result_exception:gate_terminal_unconfirmed"
        )
        assert wrapped.result.trajectory_dispatched
        assert wrapped.result.gate_accepted
        assert not wrapped.result.gate_terminal
        assert not wrapped.result.downstream_terminal_observed
        assert wrapped.result.cancel_requested
        cancel_status = wait_for_status_event(
            harness,
            request_id,
            stage="cancelled_gate_command",
            reason="accepted",
            gate_goal_id=gate_goal_id,
        )
        assert cancel_status["gate_cancel_requested"] is True
        assert cancel_status["cancel_response_accepted"] is True
        terminal_status = wait_for_status_event(
            harness,
            request_id,
            stage="gate_terminal_unconfirmed",
            reason="result_exception:RuntimeError",
            gate_goal_id=gate_goal_id,
        )
        assert terminal_status["gate_cancel_requested"] is True
        assert terminal_status["gate_terminal_observed"] is False
        assert harness.dependencies.gate_cancel_requests == 1
        server_goal_id = assert_canonical_goal_uuid(
            harness.dependencies.gate_server_goal_ids[0]
        )
        wait_until(
            lambda: harness.dependencies.gate_terminal_statuses.get(
                server_goal_id
            )
            == GoalStatus.STATUS_CANCELED,
            reason="fake gate terminal after result exception",
        )
        record = harness.dependencies.gate_goal_records[0]
        assert record["client_goal_id"] == gate_goal_id
        assert record["server_goal_id"] == gate_goal_id
        assert record["terminal_status"] == GoalStatus.STATUS_CANCELED
        assert len(harness.dependencies.gate_goals) == 1
        assert len(harness.dependencies.published_trajectories) == 1
        _append_injected_event(
            "gate_result_exception",
            harness,
            goal_handle,
            wrapped,
            request_id=request_id,
        )
    finally:
        harness.close()


def test_move_group_result_timeout_with_unconfirmed_terminal_stays_fail_closed() -> None:
    harness = AdapterHarness(
        move_group_mode="hold_after_cancel",
        target_timeout_ms=2_000.0,
        result_timeout_margin_ms=25.0,
        move_group_terminal_timeout_ms=40.0,
    )
    try:
        harness.publish_inputs()
        goal = harness.goal("terminal-unconfirmed")
        goal.planning_timeout_s = 0.05
        request_id = "task-terminal-unconfirmed|diagnostic|0"
        goal_handle = harness.send(goal)
        assert goal_handle.accepted
        accepted_status = wait_for_move_group_identity(
            harness,
            request_id,
            stage="move_group_goal_accepted",
            reason="result_pending",
        )
        wrapped = wait_future(goal_handle.get_result_async())
        assert wrapped.status == GoalStatus.STATUS_ABORTED
        assert wrapped.result.reason == (
            "move_group_result_timeout:move_group_terminal_unconfirmed"
        )
        cancel_status = wait_for_status_event(
            harness,
            request_id,
            stage="cancelled_move_group",
            reason="move_group_result_timeout",
            move_group_goal_id=accepted_status["move_group_goal_id"],
        )
        assert cancel_status["cancel_response_accepted"] is True
        terminal_status = wait_for_status_event(
            harness,
            request_id,
            stage="move_group_terminal_unconfirmed",
            reason="result_future_timeout",
            move_group_goal_id=accepted_status["move_group_goal_id"],
        )
        assert terminal_status["move_group_terminal_observed"] is False
        assert harness.dependencies.cancel_requests == 1
        assert harness.dependencies.gate_goals == []
        assert harness.dependencies.published_trajectories == []
        assert harness.dependencies.move_group_terminal_statuses == {}
        harness.dependencies.release_goal.set()
        server_goal_id = assert_canonical_goal_uuid(
            harness.dependencies.move_group_server_goal_ids[0]
        )
        wait_until(
            lambda: harness.dependencies.move_group_terminal_statuses.get(
                server_goal_id
            )
            == GoalStatus.STATUS_CANCELED,
            reason="released fake MoveGroup terminal",
        )
        _append_injected_event(
            "move_group_terminal_unconfirmed",
            harness,
            goal_handle,
            wrapped,
            request_id=request_id,
        )
    finally:
        harness.close()


def test_move_group_cancel_rejected_then_late_success_never_reaches_gate() -> None:
    harness = AdapterHarness(
        move_group_mode="success_after_cancel",
        cancel_response=CancelResponse.REJECT,
        target_timeout_ms=2_000.0,
        result_timeout_margin_ms=25.0,
        move_group_terminal_timeout_ms=500.0,
    )
    try:
        harness.publish_inputs()
        goal = harness.goal("success-after-cancel")
        goal.planning_timeout_s = 0.05
        request_id = "task-success-after-cancel|diagnostic|0"
        goal_handle = harness.send(goal)
        assert goal_handle.accepted
        accepted_status = wait_for_move_group_identity(
            harness,
            request_id,
            stage="move_group_goal_accepted",
            reason="result_pending",
        )
        release_thread = threading.Thread(
            target=lambda: (
                wait_until(
                    lambda: harness.dependencies.cancel_requests == 1,
                    reason="rejected MoveGroup cancel request",
                ),
                harness.dependencies.release_goal.set(),
            ),
            daemon=True,
        )
        release_thread.start()
        wrapped = wait_future(goal_handle.get_result_async())
        assert wrapped.status == GoalStatus.STATUS_ABORTED
        assert "move_group_result_timeout" in wrapped.result.reason
        assert "success_after_cancel" in wrapped.result.reason
        cancel_status = wait_for_status_event(
            harness,
            request_id,
            stage="cancelled_move_group",
            reason="move_group_result_timeout",
            move_group_goal_id=accepted_status["move_group_goal_id"],
        )
        assert cancel_status["cancel_response_accepted"] is False
        success_race_status = wait_for_status_event(
            harness,
            request_id,
            stage="move_group_terminal_observed",
            reason="success_after_cancel",
            move_group_goal_id=accepted_status["move_group_goal_id"],
        )
        assert success_race_status["move_group_terminal_observed"] is True
        assert (
            success_race_status["move_group_terminal_status"]
            == GoalStatus.STATUS_SUCCEEDED
        )
        assert (
            success_race_status["action_goal_status"]
            == GoalStatus.STATUS_SUCCEEDED
        )
        assert harness.dependencies.cancel_requests == 1
        release_thread.join(timeout=1.0)
        assert not release_thread.is_alive()
        server_goal_id = assert_canonical_goal_uuid(
            harness.dependencies.move_group_server_goal_ids[0]
        )
        wait_until(
            lambda: harness.dependencies.move_group_terminal_statuses.get(
                server_goal_id
            )
            == GoalStatus.STATUS_SUCCEEDED,
            reason="late fake MoveGroup success terminal",
        )
        record = harness.dependencies.move_group_goal_records[0]
        assert record["result_success_payload"] is True
        assert record["result_error_code"] == MoveItErrorCodes.SUCCESS
        assert record["terminal_status"] == GoalStatus.STATUS_SUCCEEDED
        assert harness.dependencies.gate_goals == []
        assert harness.dependencies.published_trajectories == []
        _append_injected_event(
            "success_after_cancel_race",
            harness,
            goal_handle,
            wrapped,
            request_id=request_id,
        )
    finally:
        harness.close()


def test_move_group_late_result_from_old_generation_cannot_dispatch_same_id_retry() -> None:
    harness = AdapterHarness(
        move_group_mode="first_cancel_success_then_success",
        target_timeout_ms=2_000.0,
        result_timeout_margin_ms=25.0,
        move_group_terminal_timeout_ms=40.0,
    )
    try:
        harness.publish_inputs()
        first_goal = harness.goal("same-id-retry")
        first_goal.planning_timeout_s = 0.05
        request_id = "task-same-id-retry|diagnostic|0"
        first = harness.send(first_goal)
        assert first.accepted
        first_status = wait_for_move_group_identity(
            harness,
            request_id,
            stage="move_group_goal_accepted",
            reason="result_pending",
        )
        first_wrapped = wait_future(first.get_result_async())
        assert first_wrapped.status == GoalStatus.STATUS_ABORTED
        assert first_wrapped.result.reason.endswith(
            "move_group_terminal_unconfirmed"
        )
        assert harness.dependencies.gate_goals == []
        assert len(harness.dependencies.move_group_server_goal_ids) == 1
        first_server_goal_id = assert_canonical_goal_uuid(
            harness.dependencies.move_group_server_goal_ids[0]
        )
        assert harness.dependencies.move_group_terminal_statuses == {}

        # The unconfirmed terminal latches a safety fault.  Reset the adapter
        # epoch before retrying, while the first fake action remains pending;
        # this models an explicit operator recovery boundary.  The fake uses
        # the new epoch for the retry's planning-scene confirmation.
        harness.reset_adapter()
        second = harness.send(harness.goal("same-id-retry"))
        assert second.accepted
        second_status = wait_for_move_group_identity(
            harness,
            request_id,
            after_generation=int(first_status["attempt_generation"]),
            stage="move_group_goal_accepted",
            reason="result_pending",
        )
        assert second_status["attempt_generation"] > first_status[
            "attempt_generation"
        ]
        assert len(harness.dependencies.move_group_goal_records) == 2
        assert harness.dependencies.move_group_terminal_statuses == {}

        # Both generations are now active at the fake server.  Releasing the
        # shared gate lets the old result arrive while the new attempt is in
        # flight; only the new generation may dispatch through the gate.
        harness.dependencies.release_goal.set()
        wait_until(
            lambda: harness.dependencies.move_group_terminal_statuses.get(
                first_server_goal_id
            )
            == GoalStatus.STATUS_CANCELED,
            reason="old-generation terminal during retry",
        )
        old_terminal_status = wait_for_status_event(
            harness,
            request_id,
            stage="late_move_group_goal_terminal",
            reason="observed",
            move_group_goal_id=first_server_goal_id,
        )
        assert old_terminal_status["attempt_generation"] == first_status[
            "attempt_generation"
        ]
        assert old_terminal_status["move_group_terminal_observed"] is True
        assert (
            old_terminal_status["move_group_terminal_status"]
            == GoalStatus.STATUS_CANCELED
        )
        second_wrapped = wait_future(second.get_result_async())
        assert second_wrapped.status == GoalStatus.STATUS_SUCCEEDED
        assert second_wrapped.result.success
        assert second_wrapped.result.command_id == request_id
        assert len(harness.dependencies.gate_goals) == 1
        assert harness.dependencies.gate_goals[0].command_id == request_id
        assert len(harness.dependencies.move_group_goal_records) == 2
        assert harness.dependencies.move_group_goal_records[0][
            "request_id"
        ] == request_id
        assert harness.dependencies.move_group_goal_records[1][
            "request_id"
        ] == request_id
        with harness.adapter._state_lock:
            assert harness.adapter._fault_latched is None
        assert harness.adapter._fault_latched is None
        _append_injected_event(
            "old_generation_isolation",
            harness,
            second,
            second_wrapped,
            request_id=request_id,
            checks={"old_generation_isolation": True},
        )
    finally:
        harness.close()


def test_move_group_late_success_from_old_generation_cannot_dispatch_same_id_retry() -> None:
    harness = AdapterHarness(
        move_group_mode="old_generation_late_success_then_success",
        cancel_response=CancelResponse.REJECT,
        target_timeout_ms=2_000.0,
        result_timeout_margin_ms=25.0,
        move_group_terminal_timeout_ms=40.0,
    )
    try:
        harness.publish_inputs()
        first_goal = harness.goal("same-id-late-success")
        first_goal.planning_timeout_s = 0.05
        request_id = "task-same-id-late-success|diagnostic|0"
        first = harness.send(first_goal)
        assert first.accepted
        first_status = wait_for_move_group_identity(
            harness,
            request_id,
            stage="move_group_goal_accepted",
            reason="result_pending",
        )
        first_wrapped = wait_future(first.get_result_async())
        assert first_wrapped.status == GoalStatus.STATUS_ABORTED
        assert first_wrapped.result.reason.endswith(
            "move_group_terminal_unconfirmed"
        )
        first_move_group_goal_id = assert_canonical_goal_uuid(
            first_status["move_group_goal_id"]
        )
        first_cancel_status = wait_for_status_event(
            harness,
            request_id,
            stage="cancelled_move_group",
            reason="move_group_result_timeout",
            move_group_goal_id=first_move_group_goal_id,
        )
        assert first_cancel_status["cancel_response_accepted"] is False
        assert len(harness.dependencies.move_group_cancelled_server_goal_ids) == 1
        assert (
            harness.dependencies.move_group_cancelled_server_goal_ids[0]
            == first_move_group_goal_id
        )
        first_terminal_status = wait_for_status_event(
            harness,
            request_id,
            stage="move_group_terminal_unconfirmed",
            reason="result_future_timeout",
            move_group_goal_id=first_move_group_goal_id,
        )
        assert first_terminal_status["move_group_terminal_observed"] is False
        assert harness.dependencies.gate_goals == []
        assert harness.dependencies.published_trajectories == []
        assert harness.dependencies.move_group_terminal_statuses == {}
        assert len(harness.dependencies.move_group_goal_records) == 1
        first_record = harness.dependencies.move_group_goal_records[0]
        assert first_record["server_goal_id"] == first_move_group_goal_id
        assert first_record["client_goal_id"] == first_move_group_goal_id
        assert first_record["constraint_name"] == f"edgegrasp:{request_id}"
        assert first_record["attempt_generation"] == first_status[
            "attempt_generation"
        ]
        assert first_record["cancel_api_requested"] is True
        assert first_record["terminal_status"] is None

        # Reset the adapter safety epoch while the first fake result remains
        # pending, then start a same-ID retry.  The old fake action is released
        # only after the second generation is accepted, so its protocol
        # SUCCEEDED terminal arrives while the new attempt is active.
        harness.reset_adapter()
        second = harness.send(harness.goal("same-id-late-success"))
        assert second.accepted
        second_status = wait_for_move_group_identity(
            harness,
            request_id,
            after_generation=int(first_status["attempt_generation"]),
            stage="move_group_goal_accepted",
            reason="result_pending",
        )
        assert second_status["attempt_generation"] > first_status[
            "attempt_generation"
        ]
        assert len(harness.dependencies.move_group_goal_records) == 2
        assert harness.dependencies.move_group_terminal_statuses == {}

        harness.dependencies.release_old_generation_goal.set()
        old_late_terminal = wait_for_status_event(
            harness,
            request_id,
            stage="late_move_group_goal_terminal",
            reason="observed",
            move_group_goal_id=first_move_group_goal_id,
        )
        assert old_late_terminal["attempt_generation"] == first_status[
            "attempt_generation"
        ]
        assert old_late_terminal["move_group_terminal_observed"] is True
        assert (
            old_late_terminal["move_group_terminal_status"]
            == GoalStatus.STATUS_SUCCEEDED
        )
        assert old_late_terminal["action_goal_status"] == GoalStatus.STATUS_SUCCEEDED
        wait_until(
            lambda: harness.dependencies.move_group_terminal_statuses.get(
                first_move_group_goal_id
            )
            == GoalStatus.STATUS_SUCCEEDED,
            reason="old-generation late fake MoveGroup success terminal",
        )
        assert first_record["result_success_payload"] is True
        assert first_record["terminal_status"] == GoalStatus.STATUS_SUCCEEDED

        # The old callback carried generation 1.  It must not fault generation
        # 2 or reserve/dispatch the same command id on its behalf.
        assert not any(
            event.get("stage") == "ERROR"
            and event.get("reason")
            == "late_move_group_goal_success_after_cancel"
            and event.get("attempt_generation") == first_status["attempt_generation"]
            for event in harness.adapter_status_events
        )
        harness.dependencies.release_goal.set()
        second_wrapped = wait_future(second.get_result_async())
        assert second_wrapped.status == GoalStatus.STATUS_SUCCEEDED
        assert second_wrapped.result.success
        assert second_wrapped.result.command_id == request_id
        assert len(harness.dependencies.gate_goals) == 1
        assert len(harness.dependencies.published_trajectories) == 1
        gate_status = wait_for_gate_identity(
            harness,
            request_id,
            stage="gate_goal_accepted",
            reason="result_pending",
        )
        second_record = harness.dependencies.move_group_goal_records[1]
        second_move_group_goal_id = assert_canonical_goal_uuid(
            second_status["move_group_goal_id"]
        )
        assert second_record["server_goal_id"] == second_move_group_goal_id
        assert second_record["client_goal_id"] == second_move_group_goal_id
        assert second_record["constraint_name"] == f"edgegrasp:{request_id}"
        assert second_record["attempt_generation"] == second_status[
            "attempt_generation"
        ]
        assert second_record["terminal_status"] == GoalStatus.STATUS_SUCCEEDED
        assert gate_status["attempt_generation"] == second_status["attempt_generation"]
        assert harness.dependencies.gate_goal_records[0][
            "attempt_generation"
        ] == second_status["attempt_generation"]
        assert harness.dependencies.gate_goal_records[0][
            "client_goal_id"
        ] == harness.dependencies.gate_goal_records[0]["server_goal_id"]
        _append_injected_event(
            "old_generation_late_success_isolation",
            harness,
            second,
            second_wrapped,
            request_id=request_id,
            checks={"old_generation_late_success_isolation": True},
        )
    finally:
        harness.close()


def test_injected_explicit_cancel_cannot_publish_old_result() -> None:
    harness = AdapterHarness(move_group_mode="hold")
    try:
        harness.publish_inputs()
        goal = harness.goal("first")
        request_id = "task-first|diagnostic|0"
        first = harness.send(goal)
        assert first.accepted
        assert_canonical_goal_uuid(goal_uuid_hex(first))
        wait_until(harness.dependencies.goal_started.is_set, reason="first plan")
        accepted_status = wait_for_move_group_identity(
            harness,
            request_id,
            stage="move_group_goal_accepted",
            reason="result_pending",
        )
        second = harness.send(harness.goal("second"))
        assert not second.accepted
        wait_future(first.cancel_goal_async())
        wrapped = wait_future(first.get_result_async())
        assert wrapped.status == GoalStatus.STATUS_CANCELED
        assert wrapped.result.reason == "plan_target_cancel_requested"
        assert harness.dependencies.cancel_requests == 1
        assert not wrapped.result.trajectory_dispatched
        assert harness.dependencies.published_trajectories == []
        cancel_status = wait_for_status_event(
            harness,
            request_id,
            stage="cancelled_move_group",
            reason="plan_target_cancel_requested",
            move_group_goal_id=accepted_status["move_group_goal_id"],
        )
        assert cancel_status["cancel_response_accepted"] is True
        terminal_status = wait_for_status_event(
            harness,
            request_id,
            stage="move_group_terminal_observed",
            reason="observed_after_cancel",
            move_group_goal_id=accepted_status["move_group_goal_id"],
        )
        assert terminal_status["move_group_terminal_observed"] is True
        assert harness.dependencies.move_group_goal_records[0][
            "terminal_status"
        ] == GoalStatus.STATUS_CANCELED
        _append_injected_event(
            "explicit_cancel",
            harness,
            first,
            wrapped,
            request_id=request_id,
        )
    finally:
        harness.close()
