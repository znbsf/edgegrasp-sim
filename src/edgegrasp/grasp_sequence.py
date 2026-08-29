"""Dependency-free orchestration for a correlated SO-101 grasp sequence.

The controller deliberately knows nothing about ROS futures or MoveIt message
types.  Adapters submit immutable commands through the injected ports and feed
back correlated events.  A planning result is accepted as an executed arm
stage only when it carries the complete PlanTarget/gate/FJT terminal contract.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from functools import wraps
from math import isfinite
import re
from threading import RLock
from typing import Protocol

from .config import DEFAULT_CLOCK_DOMAIN, DEFAULT_TARGET_FRAME
from .models import NANOSECONDS_PER_SECOND, Vector3
from .trajectory_identity import make_trajectory_command_id


ACTION_STATUS_SUCCEEDED = 4
ACTION_STATUS_CANCELED = 5
ACTION_STATUS_ABORTED = 6
FJT_SUCCESSFUL = 0
MOVEIT_SUCCESS = 1
_DIGEST = re.compile(r"^[0-9a-f]{64}$")
_ACTION_TERMINAL_STATUSES = frozenset(
    (ACTION_STATUS_SUCCEEDED, ACTION_STATUS_CANCELED, ACTION_STATUS_ABORTED)
)


def _serialized(method):
    """Serialize every public state-changing event on one re-entrant lock."""

    @wraps(method)
    def guarded(self, *args, **kwargs):
        with self._lock:
            return method(self, *args, **kwargs)

    return guarded


class GraspPhase(str, Enum):
    IDLE = "IDLE"
    APPROACH_PLAN = "APPROACH_PLAN"
    APPROACH_EXEC = "APPROACH_EXEC"
    DESCEND_PLAN = "DESCEND_PLAN"
    DESCEND_EXEC = "DESCEND_EXEC"
    CLOSE_GRIPPER_EXEC = "CLOSE_GRIPPER_EXEC"
    LIFT_PLAN = "LIFT_PLAN"
    LIFT_EXEC = "LIFT_EXEC"
    COMPLETE = "COMPLETE"
    SAFE_STOP = "SAFE_STOP"


class CommandKind(str, Enum):
    ARM_PLAN = "arm_plan"
    GRIPPER_EXECUTION = "gripper_execution"


class CancelDisposition(str, Enum):
    TERMINAL_CONFIRMED = "terminal_confirmed"
    ACCEPTED_PENDING = "accepted_pending"
    REJECTED = "rejected"
    TIMEOUT = "timeout"


@dataclass(frozen=True, slots=True)
class SignalSnapshot:
    ready: bool
    observed_at_ns: int
    reason: str = ""

    def __post_init__(self) -> None:
        if not isinstance(self.ready, bool):
            raise TypeError("ready must be a boolean")
        _require_ns(self.observed_at_ns, "observed_at_ns")


@dataclass(frozen=True, slots=True)
class SequenceHealth:
    """Local receive/liveness evidence supplied by a future ROS wrapper.

    The task's original source timestamp is checked at admission. Every later
    stage binds its command to ``target_source_timestamp_ns`` from the current
    health snapshot, while the four local observations independently prove
    receive liveness. A ROS wrapper must also reject target-position drift;
    this dependency-free core does not own a target tracker.
    """

    target_id: str
    target_source_timestamp_ns: int
    permission: SignalSnapshot
    interface: SignalSnapshot
    target: SignalSnapshot
    joint_state: SignalSnapshot
    clock_domain: str = DEFAULT_CLOCK_DOMAIN
    clock_epoch: int = 0

    def __post_init__(self) -> None:
        if not self.target_id.strip():
            raise ValueError("target_id must not be empty")
        if not self.clock_domain.strip():
            raise ValueError("clock_domain must not be empty")
        _require_ns(self.target_source_timestamp_ns, "target_source_timestamp_ns")
        for name, signal in (
            ("permission", self.permission),
            ("interface", self.interface),
            ("target", self.target),
            ("joint_state", self.joint_state),
        ):
            if not isinstance(signal, SignalSnapshot):
                raise TypeError(f"{name} must be SignalSnapshot")
        _require_epoch(self.clock_epoch)


@dataclass(frozen=True, slots=True)
class GraspTaskSnapshot:
    """Immutable geometry and clock identity for one sequence attempt."""

    task_id: str
    target_id: str
    approach_position_m: Vector3
    descend_position_m: Vector3
    lift_position_m: Vector3
    approach_orientation_xyzw: tuple[float, float, float, float]
    grasp_orientation_xyzw: tuple[float, float, float, float]
    gripper_closed_position_rad: float
    source_timestamp_ns: int
    frame_id: str = DEFAULT_TARGET_FRAME
    clock_domain: str = DEFAULT_CLOCK_DOMAIN
    clock_epoch: int = 0

    def __post_init__(self) -> None:
        # This also validates task_id against the delimiter-free command ID
        # contract used independently by the ROS nodes.
        make_trajectory_command_id(self.task_id, "approach", 0)
        if not self.target_id.strip():
            raise ValueError("target_id must not be empty")
        if not self.frame_id.strip():
            raise ValueError("frame_id must not be empty")
        if not self.clock_domain.strip():
            raise ValueError("clock_domain must not be empty")
        _require_orientation(
            self.approach_orientation_xyzw,
            "approach_orientation_xyzw",
        )
        _require_orientation(
            self.grasp_orientation_xyzw,
            "grasp_orientation_xyzw",
        )
        if not isfinite(self.gripper_closed_position_rad):
            raise ValueError("gripper_closed_position_rad must be finite")
        _require_ns(self.source_timestamp_ns, "source_timestamp_ns")
        _require_epoch(self.clock_epoch)


@dataclass(frozen=True, slots=True)
class ArmPlanCommand:
    task_id: str
    command_id: str
    target_id: str
    stage: str
    sequence_no: int
    target_position_m: Vector3
    target_orientation_xyzw: tuple[float, float, float, float]
    frame_id: str
    source_timestamp_ns: int
    clock_domain: str
    clock_epoch: int


@dataclass(frozen=True, slots=True)
class GripperExecutionCommand:
    task_id: str
    command_id: str
    target_id: str
    stage: str
    sequence_no: int
    position_rad: float
    source_timestamp_ns: int
    clock_domain: str
    clock_epoch: int


@dataclass(frozen=True, slots=True)
class DispatchOutcome:
    accepted: bool
    reason: str

    def __post_init__(self) -> None:
        if not isinstance(self.accepted, bool):
            raise TypeError("accepted must be a boolean")
        if not isinstance(self.reason, str):
            raise TypeError("reason must be a string")


@dataclass(frozen=True, slots=True)
class CancelOutcome:
    disposition: CancelDisposition
    reason: str

    def __post_init__(self) -> None:
        if not isinstance(self.disposition, CancelDisposition):
            raise TypeError("disposition must be CancelDisposition")
        if not isinstance(self.reason, str):
            raise TypeError("reason must be a string")

    @property
    def terminal_confirmed(self) -> bool:
        return self.disposition is CancelDisposition.TERMINAL_CONFIRMED


@dataclass(frozen=True, slots=True)
class StopAllOutcome:
    arm_terminal_confirmed: bool
    gripper_terminal_confirmed: bool
    reason: str

    def __post_init__(self) -> None:
        if not isinstance(self.arm_terminal_confirmed, bool):
            raise TypeError("arm_terminal_confirmed must be a boolean")
        if not isinstance(self.gripper_terminal_confirmed, bool):
            raise TypeError("gripper_terminal_confirmed must be a boolean")
        if not isinstance(self.reason, str):
            raise TypeError("reason must be a string")

    @property
    def terminal_confirmed(self) -> bool:
        return self.arm_terminal_confirmed and self.gripper_terminal_confirmed


@dataclass(frozen=True, slots=True)
class ArmStageTerminal:
    """ROS-independent projection of a terminal ``PlanTarget.Result``.

    Planning creates the trajectory, so the digest cannot be precomputed by
    this core. It is validated as audit evidence; a ROS wrapper must ensure the
    same digest is carried from PlanTarget through ExecuteTrajectory.
    """

    task_id: str
    command_id: str
    target_id: str
    stage: str
    sequence_no: int
    accepted: bool
    success: bool
    reason: str
    moveit_error_code: int
    source_timestamp_ns: int
    clock_domain: str
    clock_epoch: int
    trajectory_dispatched: bool
    trajectory_digest: str
    gate_accepted: bool
    gate_terminal: bool
    downstream_terminal_observed: bool
    cancel_requested: bool
    plan_action_goal_status: int
    action_goal_status: int
    fjt_error_code: int
    fjt_error_string: str


@dataclass(frozen=True, slots=True)
class GripperStageTerminal:
    """ROS-independent projection of a terminal ``ExecuteTrajectory.Result``."""

    task_id: str
    command_id: str
    target_id: str
    stage: str
    sequence_no: int
    controller: str
    trajectory_digest: str
    accepted: bool
    terminal: bool
    downstream_terminal_observed: bool
    cancel_requested: bool
    gate_action_goal_status: int
    action_goal_status: int
    fjt_error_code: int
    fjt_error_string: str
    success: bool
    reason: str
    source_timestamp_ns: int
    completed_timestamp_ns: int
    clock_domain: str
    clock_epoch: int


@dataclass(frozen=True, slots=True)
class SequenceTransition:
    previous: GraspPhase
    current: GraspPhase
    reason: str
    timestamp_ns: int


@dataclass(frozen=True, slots=True)
class SequenceDecision:
    accepted: bool
    ignored: bool
    reason: str
    phase: GraspPhase


@dataclass(frozen=True, slots=True)
class SequenceResetResult:
    success: bool
    reason: str
    phase: GraspPhase
    clock_epoch: int


@dataclass(frozen=True, slots=True)
class GraspSequenceOutcome:
    task_id: str
    sequence_completed: bool
    physics_grasp_verified: bool
    reason: str


class ArmPlanPort(Protocol):
    def submit(self, command: ArmPlanCommand) -> DispatchOutcome: ...

    def cancel(self, command_id: str, reason: str) -> CancelOutcome: ...


class GripperExecutionPort(Protocol):
    def submit(self, command: GripperExecutionCommand) -> DispatchOutcome: ...

    def cancel(self, command_id: str, reason: str) -> CancelOutcome: ...


class StopAllPort(Protocol):
    def stop_all(self, task_id: str, reason: str) -> StopAllOutcome: ...


@dataclass(frozen=True, slots=True)
class _ActiveCommand:
    kind: CommandKind
    command_id: str
    stage: str
    sequence_no: int
    source_timestamp_ns: int
    issued_at_ns: int


_ARM_STAGES: tuple[tuple[str, int, GraspPhase, GraspPhase], ...] = (
    ("approach", 0, GraspPhase.APPROACH_PLAN, GraspPhase.APPROACH_EXEC),
    ("descend", 1, GraspPhase.DESCEND_PLAN, GraspPhase.DESCEND_EXEC),
    ("lift", 3, GraspPhase.LIFT_PLAN, GraspPhase.LIFT_EXEC),
)
_PLAN_TO_EXEC = {plan: execute for _, _, plan, execute in _ARM_STAGES}
_STAGE_TO_PHASES = {
    stage: (sequence_no, plan, execute)
    for stage, sequence_no, plan, execute in _ARM_STAGES
}


class GraspSequenceController:
    """Fail-closed asynchronous grasp phase coordinator.

    ``COMPLETE`` proves only that the correlated action sequence completed.
    Physical grasp evidence is intentionally outside this class, so its
    outcome always reports ``physics_grasp_verified=False``.
    """

    def __init__(
        self,
        arm_port: ArmPlanPort,
        gripper_port: GripperExecutionPort,
        stop_port: StopAllPort,
        *,
        target_frame: str = DEFAULT_TARGET_FRAME,
        clock_domain: str = DEFAULT_CLOCK_DOMAIN,
        clock_epoch: int = 0,
        source_timeout_ms: float = 200.0,
        health_timeout_ms: float = 200.0,
        permission_timeout_ms: float | None = None,
        interface_timeout_ms: float | None = None,
        target_receive_timeout_ms: float | None = None,
        joint_state_timeout_ms: float | None = None,
        command_timeout_ms: float = 10_000.0,
    ) -> None:
        if not target_frame.strip():
            raise ValueError("target_frame must not be empty")
        if not clock_domain.strip():
            raise ValueError("clock_domain must not be empty")
        _require_epoch(clock_epoch)
        for name, value in (
            ("source_timeout_ms", source_timeout_ms),
            ("health_timeout_ms", health_timeout_ms),
            ("command_timeout_ms", command_timeout_ms),
        ):
            if not isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        self.arm_port = arm_port
        self.gripper_port = gripper_port
        self.stop_port = stop_port
        self._lock = RLock()
        self.target_frame = target_frame
        self.clock_domain = clock_domain
        self.source_timeout_ns = int(source_timeout_ms * NANOSECONDS_PER_SECOND / 1000)
        self.health_timeout_ns = int(health_timeout_ms * NANOSECONDS_PER_SECOND / 1000)
        signal_timeout_ms = {
            "permission": permission_timeout_ms,
            "interface": interface_timeout_ms,
            "target": target_receive_timeout_ms,
            "joint_state": joint_state_timeout_ms,
        }
        self.signal_timeout_ns: dict[str, int] = {}
        for name, value in signal_timeout_ms.items():
            resolved = health_timeout_ms if value is None else value
            if not isfinite(resolved) or resolved <= 0:
                raise ValueError(f"{name}_timeout_ms must be finite and positive")
            self.signal_timeout_ns[name] = int(
                resolved * NANOSECONDS_PER_SECOND / 1000
            )
        self.command_timeout_ns = int(command_timeout_ms * NANOSECONDS_PER_SECOND / 1000)
        self._clock_epoch = clock_epoch
        self._phase = GraspPhase.IDLE
        self._task: GraspTaskSnapshot | None = None
        self._active: _ActiveCommand | None = None
        self._completed_commands: set[str] = set()
        self._used_task_ids: set[str] = set()
        self._history: list[SequenceTransition] = []
        self._last_now_ns: int | None = None
        self._last_target_source_timestamp_ns: int | None = None
        self._fault_evidence: list[str] = []
        self._recovery_required = False
        self._cancel_terminal_confirmed = True
        self._stop_all_terminal_confirmed = True
        self._clock_fault = False
        self._outcome: GraspSequenceOutcome | None = None

    @property
    def phase(self) -> GraspPhase:
        return self._phase

    @property
    def clock_epoch(self) -> int:
        return self._clock_epoch

    @property
    def active_command_id(self) -> str | None:
        return None if self._active is None else self._active.command_id

    @property
    def history(self) -> tuple[SequenceTransition, ...]:
        return tuple(self._history)

    @property
    def fault_evidence(self) -> tuple[str, ...]:
        return tuple(self._fault_evidence)

    @property
    def recovery_required(self) -> bool:
        return self._recovery_required

    @property
    def outcome(self) -> GraspSequenceOutcome | None:
        return self._outcome

    @_serialized
    def start(
        self, task: GraspTaskSnapshot, health: SequenceHealth, now_ns: int
    ) -> SequenceDecision:
        if self._phase is not GraspPhase.IDLE:
            return self._decision(False, "sequence_not_idle")
        clock_reason = self._observe_clock(now_ns)
        if clock_reason is not None:
            return self._fail_closed(clock_reason, now_ns, clock_fault=True)
        if not isinstance(task, GraspTaskSnapshot):
            return self._fail_closed("invalid_task_type", now_ns)
        self._task = task
        reason = self._validate_task(task, now_ns)
        if reason is not None:
            return self._fail_closed(
                reason, now_ns, clock_fault=_is_clock_fault_reason(reason)
            )
        if task.task_id in self._used_task_ids:
            return self._fail_closed("task_id_reuse", now_ns)
        reason = self._health_reason(health, now_ns)
        if reason is not None:
            return self._fail_closed(
                reason, now_ns, clock_fault=_is_clock_fault_reason(reason)
            )
        if health.target_source_timestamp_ns != task.source_timestamp_ns:
            return self._fail_closed("task_health_source_timestamp_mismatch", now_ns)
        self._used_task_ids.add(task.task_id)
        return self._issue_arm(
            "approach", now_ns, health.target_source_timestamp_ns
        )

    @_serialized
    def arm_execution_started(
        self,
        *,
        task_id: str,
        command_id: str,
        stage: str,
        sequence_no: int,
        health: SequenceHealth,
        now_ns: int,
    ) -> SequenceDecision:
        reason = self._event_guard(health, now_ns)
        if reason is not None:
            return reason
        correlation = self._correlation_reason(
            task_id, command_id, stage, sequence_no, CommandKind.ARM_PLAN
        )
        if correlation == "completed_duplicate":
            return self._decision(False, correlation, ignored=True)
        if correlation is not None:
            return self._fail_closed(correlation, now_ns)
        if self._phase in _PLAN_TO_EXEC:
            self._transition(_PLAN_TO_EXEC[self._phase], "arm_execution_started", now_ns)
            return self._decision(True, "arm_execution_started")
        expected_exec = _STAGE_TO_PHASES[stage][2]
        if self._phase is expected_exec:
            return self._decision(False, "duplicate_execution_feedback", ignored=True)
        return self._fail_closed("execution_feedback_phase_mismatch", now_ns)

    @_serialized
    def on_arm_terminal(
        self,
        terminal: ArmStageTerminal,
        health: SequenceHealth,
        now_ns: int,
    ) -> SequenceDecision:
        if self._is_known_prior_task_event(terminal):
            return self._decision(False, "late_event_from_prior_task", ignored=True)
        if not isinstance(terminal, ArmStageTerminal):
            return self._fail_closed("invalid_arm_terminal_type", now_ns)
        if self._phase is GraspPhase.SAFE_STOP:
            return self._confirm_late_terminal(
                terminal, CommandKind.ARM_PLAN, now_ns
            )
        event = self._event_guard(health, now_ns)
        if event is not None:
            return event
        correlation = self._correlation_reason(
            terminal.task_id,
            terminal.command_id,
            terminal.stage,
            terminal.sequence_no,
            CommandKind.ARM_PLAN,
        )
        if correlation == "completed_duplicate":
            return self._decision(False, correlation, ignored=True)
        if correlation is not None:
            return self._fail_closed(correlation, now_ns)
        assert self._task is not None
        if terminal.target_id != self._task.target_id:
            return self._fail_closed("terminal_target_mismatch", now_ns)
        if terminal.clock_domain != self.clock_domain:
            return self._fail_closed(
                "terminal_clock_domain_mismatch", now_ns, clock_fault=True
            )
        if terminal.clock_epoch != self._clock_epoch:
            return self._fail_closed(
                "terminal_clock_epoch_mismatch", now_ns, clock_fault=True
            )
        assert self._active is not None
        if terminal.source_timestamp_ns != self._active.source_timestamp_ns:
            return self._fail_closed("terminal_source_timestamp_mismatch", now_ns)
        expected_plan, expected_exec = _STAGE_TO_PHASES[terminal.stage][1:]
        if self._phase is expected_plan and terminal.trajectory_dispatched:
            self._transition(expected_exec, "arm_terminal_reports_dispatch", now_ns)
        elif self._phase not in (expected_plan, expected_exec):
            return self._fail_closed("arm_terminal_phase_mismatch", now_ns)
        contract_reason = _arm_terminal_reason(terminal)
        if contract_reason is not None:
            return self._terminal_failure(contract_reason, terminal, now_ns)
        self._complete_active(terminal.command_id)
        if terminal.stage == "approach":
            return self._issue_arm(
                "descend", now_ns, health.target_source_timestamp_ns
            )
        if terminal.stage == "descend":
            return self._issue_gripper(now_ns, health.target_source_timestamp_ns)
        self._transition(GraspPhase.COMPLETE, "lift_terminal_success", now_ns)
        self._outcome = GraspSequenceOutcome(
            task_id=self._task.task_id,
            sequence_completed=True,
            physics_grasp_verified=False,
            reason="correlated_sequence_completed;physics_unverified",
        )
        return self._decision(True, "sequence_completed;physics_unverified")

    @_serialized
    def on_gripper_terminal(
        self,
        terminal: GripperStageTerminal,
        health: SequenceHealth,
        now_ns: int,
    ) -> SequenceDecision:
        if self._is_known_prior_task_event(terminal):
            return self._decision(False, "late_event_from_prior_task", ignored=True)
        if not isinstance(terminal, GripperStageTerminal):
            return self._fail_closed("invalid_gripper_terminal_type", now_ns)
        if self._phase is GraspPhase.SAFE_STOP:
            return self._confirm_late_terminal(
                terminal, CommandKind.GRIPPER_EXECUTION, now_ns
            )
        event = self._event_guard(health, now_ns)
        if event is not None:
            return event
        correlation = self._correlation_reason(
            terminal.task_id,
            terminal.command_id,
            terminal.stage,
            terminal.sequence_no,
            CommandKind.GRIPPER_EXECUTION,
        )
        if correlation == "completed_duplicate":
            return self._decision(False, correlation, ignored=True)
        if correlation is not None:
            return self._fail_closed(correlation, now_ns)
        assert self._task is not None
        if terminal.target_id != self._task.target_id:
            return self._fail_closed("terminal_target_mismatch", now_ns)
        if terminal.clock_domain != self.clock_domain:
            return self._fail_closed(
                "terminal_clock_domain_mismatch", now_ns, clock_fault=True
            )
        if terminal.clock_epoch != self._clock_epoch:
            return self._fail_closed(
                "terminal_clock_epoch_mismatch", now_ns, clock_fault=True
            )
        if terminal.controller != "gripper_controller":
            return self._fail_closed("gripper_controller_mismatch", now_ns)
        assert self._active is not None
        if terminal.source_timestamp_ns != self._active.source_timestamp_ns:
            return self._fail_closed("terminal_source_timestamp_mismatch", now_ns)
        timestamp_reason = _terminal_timestamp_reason(
            terminal.completed_timestamp_ns,
            self._active.issued_at_ns,
            now_ns,
        )
        if timestamp_reason is not None:
            return self._fail_closed(
                timestamp_reason,
                now_ns,
                clock_fault=timestamp_reason == "terminal_timestamp_future",
            )
        if self._phase is not GraspPhase.CLOSE_GRIPPER_EXEC:
            return self._fail_closed("gripper_terminal_phase_mismatch", now_ns)
        contract_reason = _gripper_terminal_reason(terminal)
        if contract_reason is not None:
            return self._terminal_failure(contract_reason, terminal, now_ns)
        self._complete_active(terminal.command_id)
        return self._issue_arm("lift", now_ns, health.target_source_timestamp_ns)

    @_serialized
    def tick(self, health: SequenceHealth, now_ns: int) -> SequenceDecision:
        if self._phase in (GraspPhase.IDLE, GraspPhase.COMPLETE):
            clock_reason = self._observe_clock(now_ns)
            if clock_reason is not None:
                return self._fail_closed(clock_reason, now_ns, clock_fault=True)
            return self._decision(False, "no_active_sequence", ignored=True)
        if self._phase is GraspPhase.SAFE_STOP:
            return self._decision(False, "sequence_latched", ignored=True)
        clock_reason = self._observe_clock(now_ns)
        if clock_reason is not None:
            return self._fail_closed(clock_reason, now_ns, clock_fault=True)
        reason = self._health_reason(health, now_ns)
        if reason is not None:
            return self._fail_closed(reason, now_ns)
        assert self._active is not None
        if now_ns - self._active.issued_at_ns > self.command_timeout_ns:
            return self._fail_closed("command_terminal_timeout", now_ns)
        return self._decision(True, "sequence_healthy")

    @_serialized
    def request_stop(self, reason: str, now_ns: int) -> SequenceDecision:
        """Fail closed on an explicit wrapper/operator cancellation request."""

        if not isinstance(reason, str) or not reason.strip():
            reason = "unspecified_stop_request"
        if self._phase is GraspPhase.SAFE_STOP:
            return self._decision(False, "sequence_latched", ignored=True)
        if self._phase in (GraspPhase.IDLE, GraspPhase.COMPLETE):
            clock_reason = self._observe_clock(now_ns)
            if clock_reason is not None:
                return self._fail_closed(clock_reason, now_ns, clock_fault=True)
            return self._decision(False, "no_active_sequence", ignored=True)
        return self._fail_closed(f"stop_requested:{reason.strip()}", now_ns)

    @_serialized
    def reset(
        self,
        now_ns: int,
        *,
        new_clock_epoch: int | None = None,
        external_stop_confirmed: bool = False,
    ) -> SequenceResetResult:
        primitive = _clock_value_reason(now_ns)
        if primitive is not None:
            return SequenceResetResult(False, primitive, self._phase, self._clock_epoch)
        if self._phase not in (GraspPhase.SAFE_STOP, GraspPhase.COMPLETE):
            return SequenceResetResult(
                False, "reset_requires_terminal_phase", self._phase, self._clock_epoch
            )
        if self._recovery_required and not external_stop_confirmed:
            return SequenceResetResult(
                False,
                "external_stop_confirmation_required",
                self._phase,
                self._clock_epoch,
            )
        if self._clock_fault:
            if (
                isinstance(new_clock_epoch, bool)
                or not isinstance(new_clock_epoch, int)
                or new_clock_epoch <= self._clock_epoch
            ):
                return SequenceResetResult(
                    False, "new_clock_epoch_required", self._phase, self._clock_epoch
                )
        elif new_clock_epoch is not None:
            if (
                isinstance(new_clock_epoch, bool)
                or not isinstance(new_clock_epoch, int)
                or new_clock_epoch <= self._clock_epoch
            ):
                return SequenceResetResult(
                    False, "clock_epoch_must_advance", self._phase, self._clock_epoch
                )
        elif self._last_now_ns is not None and now_ns < self._last_now_ns:
            return SequenceResetResult(
                False, "reset_clock_rollback", self._phase, self._clock_epoch
            )
        if new_clock_epoch is not None:
            self._clock_epoch = new_clock_epoch
        self._phase = GraspPhase.IDLE
        self._task = None
        self._active = None
        self._completed_commands.clear()
        self._history.clear()
        self._last_now_ns = now_ns
        self._last_target_source_timestamp_ns = None
        self._fault_evidence.clear()
        self._recovery_required = False
        self._cancel_terminal_confirmed = True
        self._stop_all_terminal_confirmed = True
        self._clock_fault = False
        self._outcome = None
        return SequenceResetResult(True, "reset", self._phase, self._clock_epoch)

    def _event_guard(
        self, health: SequenceHealth, now_ns: int
    ) -> SequenceDecision | None:
        if self._phase is GraspPhase.SAFE_STOP:
            return self._decision(False, "late_event_after_safe_stop", ignored=True)
        if self._phase in (GraspPhase.IDLE, GraspPhase.COMPLETE):
            return self._decision(False, "event_without_active_sequence", ignored=True)
        clock_reason = self._observe_clock(now_ns)
        if clock_reason is not None:
            return self._fail_closed(clock_reason, now_ns, clock_fault=True)
        reason = self._health_reason(health, now_ns)
        if reason is not None:
            return self._fail_closed(
                reason, now_ns, clock_fault=_is_clock_fault_reason(reason)
            )
        if (
            self._active is not None
            and now_ns - self._active.issued_at_ns > self.command_timeout_ns
        ):
            return self._fail_closed("command_terminal_timeout", now_ns)
        return None

    def _confirm_late_terminal(
        self,
        terminal: ArmStageTerminal | GripperStageTerminal,
        kind: CommandKind,
        now_ns: int,
    ) -> SequenceDecision:
        """Absorb stop evidence after failure without ever resuming the sequence."""

        clock_reason = self._observe_clock(now_ns)
        if clock_reason is not None:
            self._clock_fault = True
            self._fault_evidence.append(f"late_terminal_ignored:{clock_reason}")
            return self._decision(False, clock_reason, ignored=True)
        if self._task is None or self._active is None:
            return self._decision(False, "late_terminal_without_active", ignored=True)
        active = self._active
        identity_matches = (
            active.kind is kind
            and terminal.task_id == self._task.task_id
            and terminal.command_id == active.command_id
            and terminal.target_id == self._task.target_id
            and terminal.stage == active.stage
            and terminal.sequence_no == active.sequence_no
            and terminal.clock_domain == self.clock_domain
            and terminal.clock_epoch == self._clock_epoch
        )
        if isinstance(terminal, ArmStageTerminal):
            identity_matches = identity_matches and (
                terminal.source_timestamp_ns == active.source_timestamp_ns
            )
        else:
            identity_matches = identity_matches and (
                terminal.controller == "gripper_controller"
                and terminal.source_timestamp_ns == active.source_timestamp_ns
            )
            identity_matches = identity_matches and (
                _terminal_timestamp_reason(
                    terminal.completed_timestamp_ns,
                    active.issued_at_ns,
                    now_ns,
                )
                is None
            )
        if not identity_matches:
            self._fault_evidence.append("late_terminal_identity_mismatch")
            return self._decision(
                False, "late_terminal_identity_mismatch", ignored=True
            )
        downstream_terminal = _has_correlated_downstream_terminal(terminal)
        if not downstream_terminal:
            self._fault_evidence.append("late_terminal_unconfirmed")
            return self._decision(False, "late_terminal_unconfirmed", ignored=True)
        self._complete_active(terminal.command_id)
        self._cancel_terminal_confirmed = True
        self._recovery_required = not (
            self._cancel_terminal_confirmed and self._stop_all_terminal_confirmed
        )
        self._fault_evidence.append(
            f"late_terminal_confirmed:{terminal.command_id}:"
            f"status={terminal.action_goal_status}"
        )
        return self._decision(False, "late_terminal_confirmed", ignored=True)

    def _issue_arm(
        self, stage: str, now_ns: int, source_timestamp_ns: int
    ) -> SequenceDecision:
        assert self._task is not None
        sequence_no, plan_phase, _ = _STAGE_TO_PHASES[stage]
        position = {
            "approach": self._task.approach_position_m,
            "descend": self._task.descend_position_m,
            "lift": self._task.lift_position_m,
        }[stage]
        orientation = (
            self._task.approach_orientation_xyzw
            if stage == "approach"
            else self._task.grasp_orientation_xyzw
        )
        command_id = make_trajectory_command_id(self._task.task_id, stage, sequence_no)
        command = ArmPlanCommand(
            task_id=self._task.task_id,
            command_id=command_id,
            target_id=self._task.target_id,
            stage=stage,
            sequence_no=sequence_no,
            target_position_m=position,
            target_orientation_xyzw=orientation,
            frame_id=self._task.frame_id,
            source_timestamp_ns=source_timestamp_ns,
            clock_domain=self._task.clock_domain,
            clock_epoch=self._task.clock_epoch,
        )
        self._transition(plan_phase, f"dispatch_{stage}_plan", now_ns)
        self._active = _ActiveCommand(
            CommandKind.ARM_PLAN,
            command_id,
            stage,
            sequence_no,
            source_timestamp_ns,
            now_ns,
        )
        try:
            outcome = self.arm_port.submit(command)
        except Exception as error:
            return self._fail_closed(
                f"arm_submit_exception:{type(error).__name__}", now_ns
            )
        if not isinstance(outcome, DispatchOutcome):
            return self._fail_closed("invalid_arm_dispatch_outcome", now_ns)
        if not outcome.accepted:
            self._active = None
            return self._fail_closed(f"arm_dispatch_rejected:{outcome.reason}", now_ns)
        return self._decision(True, f"{stage}_plan_dispatched")

    def _issue_gripper(
        self, now_ns: int, source_timestamp_ns: int
    ) -> SequenceDecision:
        assert self._task is not None
        stage = "close_gripper"
        sequence_no = 2
        command_id = make_trajectory_command_id(self._task.task_id, stage, sequence_no)
        command = GripperExecutionCommand(
            task_id=self._task.task_id,
            command_id=command_id,
            target_id=self._task.target_id,
            stage=stage,
            sequence_no=sequence_no,
            position_rad=self._task.gripper_closed_position_rad,
            source_timestamp_ns=source_timestamp_ns,
            clock_domain=self._task.clock_domain,
            clock_epoch=self._task.clock_epoch,
        )
        self._transition(
            GraspPhase.CLOSE_GRIPPER_EXEC, "dispatch_close_gripper", now_ns
        )
        self._active = _ActiveCommand(
            CommandKind.GRIPPER_EXECUTION,
            command_id,
            stage,
            sequence_no,
            source_timestamp_ns,
            now_ns,
        )
        try:
            outcome = self.gripper_port.submit(command)
        except Exception as error:
            return self._fail_closed(
                f"gripper_submit_exception:{type(error).__name__}", now_ns
            )
        if not isinstance(outcome, DispatchOutcome):
            return self._fail_closed("invalid_gripper_dispatch_outcome", now_ns)
        if not outcome.accepted:
            self._active = None
            return self._fail_closed(
                f"gripper_dispatch_rejected:{outcome.reason}", now_ns
            )
        return self._decision(True, "close_gripper_dispatched")

    def _correlation_reason(
        self,
        task_id: str,
        command_id: str,
        stage: str,
        sequence_no: int,
        kind: CommandKind,
    ) -> str | None:
        if command_id in self._completed_commands:
            return "completed_duplicate"
        if self._task is None or task_id != self._task.task_id:
            return "foreign_task_event"
        if self._active is None:
            return "no_active_command"
        expected = self._active
        if (
            expected.kind is not kind
            or command_id != expected.command_id
            or stage != expected.stage
            or sequence_no != expected.sequence_no
        ):
            return "command_correlation_mismatch"
        return None

    def _terminal_failure(
        self,
        reason: str,
        terminal: ArmStageTerminal | GripperStageTerminal,
        now_ns: int,
    ) -> SequenceDecision:
        # If the correlated result proves a downstream terminal, cancellation
        # is unnecessary; stop_all is still issued before latching SAFE_STOP.
        terminal_observed = _has_correlated_downstream_terminal(terminal)
        if terminal_observed:
            self._complete_active(terminal.command_id)
        return self._fail_closed(f"terminal_failure:{reason}:{terminal.reason}", now_ns)

    def _complete_active(self, command_id: str) -> None:
        self._completed_commands.add(command_id)
        self._active = None

    def _is_known_prior_task_event(self, terminal: object) -> bool:
        """Ignore a correlated callback from a completed/reset task.

        Unknown foreign identities still fail closed.  This narrow exception
        prevents a late ROS action callback from an earlier immutable task
        from faulting a newer task after explicit recovery.
        """

        terminal_task_id = getattr(terminal, "task_id", None)
        return (
            isinstance(terminal_task_id, str)
            and terminal_task_id in self._used_task_ids
            and self._task is not None
            and terminal_task_id != self._task.task_id
        )

    def _validate_task(self, task: GraspTaskSnapshot, now_ns: int) -> str | None:
        if not isinstance(task, GraspTaskSnapshot):
            return "invalid_task_type"
        if task.frame_id != self.target_frame:
            return f"task_frame_mismatch:{task.frame_id}!={self.target_frame}"
        if task.clock_domain != self.clock_domain:
            return "task_clock_domain_mismatch"
        if task.clock_epoch != self._clock_epoch:
            return "task_clock_epoch_mismatch"
        if task.source_timestamp_ns > now_ns:
            return "task_source_timestamp_future"
        if now_ns - task.source_timestamp_ns > self.source_timeout_ns:
            return "task_source_stale"
        return None

    def _health_reason(self, health: SequenceHealth, now_ns: int) -> str | None:
        if not isinstance(health, SequenceHealth):
            return "invalid_health_type"
        if health.clock_domain != self.clock_domain:
            return "health_clock_domain_mismatch"
        if health.clock_epoch != self._clock_epoch:
            return "health_clock_epoch_mismatch"
        if self._task is not None and health.target_id != self._task.target_id:
            return "health_target_mismatch"
        source_ns = health.target_source_timestamp_ns
        if source_ns > now_ns:
            return "target_source_timestamp_future"
        if now_ns - source_ns > self.source_timeout_ns:
            return "target_source_stale"
        if (
            self._last_target_source_timestamp_ns is not None
            and source_ns < self._last_target_source_timestamp_ns
        ):
            return "target_source_timestamp_rollback"
        for name, signal, timeout_ns in (
            (
                "permission",
                health.permission,
                self.signal_timeout_ns["permission"],
            ),
            (
                "interface",
                health.interface,
                self.signal_timeout_ns["interface"],
            ),
            ("target", health.target, self.signal_timeout_ns["target"]),
            (
                "joint_state",
                health.joint_state,
                self.signal_timeout_ns["joint_state"],
            ),
        ):
            if signal.observed_at_ns > now_ns:
                return f"{name}_timestamp_future"
            if now_ns - signal.observed_at_ns > timeout_ns:
                return f"{name}_stale"
            if not signal.ready:
                suffix = f":{signal.reason}" if signal.reason else ""
                return f"{name}_not_ready{suffix}"
        self._last_target_source_timestamp_ns = source_ns
        return None

    def _observe_clock(self, now_ns: int) -> str | None:
        reason = _clock_value_reason(now_ns)
        if reason is not None:
            return reason
        if self._last_now_ns is not None and now_ns < self._last_now_ns:
            return f"clock_rollback:{now_ns}<{self._last_now_ns}"
        self._last_now_ns = now_ns
        return None

    def _fail_closed(
        self, reason: str, now_ns: int, *, clock_fault: bool = False
    ) -> SequenceDecision:
        if self._phase is GraspPhase.SAFE_STOP:
            return self._decision(False, "sequence_latched", ignored=True)
        clock_reason = self._observe_clock(now_ns)
        if clock_reason is not None:
            clock_fault = True
        self._clock_fault = self._clock_fault or clock_fault
        evidence = [reason]
        if clock_reason is not None and clock_reason != reason:
            evidence.append(clock_reason)
        active = self._active
        cancel_terminal_confirmed = active is None
        if active is not None:
            try:
                if active.kind is CommandKind.ARM_PLAN:
                    cancel = self.arm_port.cancel(active.command_id, reason)
                else:
                    cancel = self.gripper_port.cancel(active.command_id, reason)
                if not isinstance(cancel, CancelOutcome):
                    evidence.append("cancel_invalid_outcome")
                elif not isinstance(cancel.disposition, CancelDisposition):
                    evidence.append("cancel_invalid_disposition")
                else:
                    evidence.append(f"cancel:{cancel.disposition.value}:{cancel.reason}")
                    cancel_terminal_confirmed = cancel.terminal_confirmed
            except Exception as error:
                evidence.append(f"cancel_exception:{type(error).__name__}")
        task_id = self._task.task_id if self._task is not None else "unassigned"
        stop_all_terminal_confirmed = False
        try:
            stopped = self.stop_port.stop_all(task_id, reason)
            if not isinstance(stopped, StopAllOutcome):
                evidence.append("stop_all_invalid_outcome")
            else:
                evidence.append(f"stop_all:{stopped.reason}")
                stop_all_terminal_confirmed = stopped.terminal_confirmed
        except Exception as error:
            evidence.append(f"stop_all_exception:{type(error).__name__}")
        self._cancel_terminal_confirmed = cancel_terminal_confirmed
        self._stop_all_terminal_confirmed = stop_all_terminal_confirmed
        self._recovery_required = not (
            cancel_terminal_confirmed and stop_all_terminal_confirmed
        )
        self._fault_evidence.extend(evidence)
        self._transition(GraspPhase.SAFE_STOP, reason, now_ns)
        self._outcome = GraspSequenceOutcome(
            task_id=task_id,
            sequence_completed=False,
            physics_grasp_verified=False,
            reason=reason,
        )
        return self._decision(False, reason)

    def _transition(self, phase: GraspPhase, reason: str, now_ns: int) -> None:
        previous = self._phase
        self._phase = phase
        self._history.append(SequenceTransition(previous, phase, reason, now_ns))

    def _decision(
        self, accepted: bool, reason: str, *, ignored: bool = False
    ) -> SequenceDecision:
        return SequenceDecision(accepted, ignored, reason, self._phase)


def _arm_terminal_reason(terminal: ArmStageTerminal) -> str | None:
    checks = (
        (terminal.accepted, "plan_not_accepted"),
        (terminal.success, "plan_or_execution_failed"),
        (terminal.moveit_error_code == MOVEIT_SUCCESS, "moveit_not_success"),
        (terminal.trajectory_dispatched, "trajectory_not_dispatched"),
        (_valid_digest(terminal.trajectory_digest), "invalid_digest"),
        (terminal.gate_accepted, "gate_not_accepted"),
        (terminal.gate_terminal, "gate_not_terminal"),
        (terminal.downstream_terminal_observed, "downstream_terminal_unobserved"),
        (not terminal.cancel_requested, "cancel_was_requested"),
        (
            terminal.plan_action_goal_status == ACTION_STATUS_SUCCEEDED,
            "plan_action_not_succeeded",
        ),
        (terminal.action_goal_status == ACTION_STATUS_SUCCEEDED, "action_not_succeeded"),
        (terminal.fjt_error_code == FJT_SUCCESSFUL, "fjt_error"),
    )
    return next((reason for valid, reason in checks if not valid), None)


def _gripper_terminal_reason(terminal: GripperStageTerminal) -> str | None:
    checks = (
        (terminal.accepted, "gate_not_accepted"),
        (terminal.terminal, "gate_not_terminal"),
        (terminal.downstream_terminal_observed, "downstream_terminal_unobserved"),
        (not terminal.cancel_requested, "cancel_was_requested"),
        (
            terminal.gate_action_goal_status == ACTION_STATUS_SUCCEEDED,
            "gate_action_not_succeeded",
        ),
        (terminal.action_goal_status == ACTION_STATUS_SUCCEEDED, "action_not_succeeded"),
        (terminal.fjt_error_code == FJT_SUCCESSFUL, "fjt_error"),
        (terminal.success, "gate_execution_failed"),
        (_valid_digest(terminal.trajectory_digest), "invalid_digest"),
    )
    return next((reason for valid, reason in checks if not valid), None)


def _clock_value_reason(now_ns: int) -> str | None:
    if isinstance(now_ns, bool) or not isinstance(now_ns, int):
        return "invalid_clock:integer_required"
    if now_ns < 0:
        return "invalid_clock:non_negative_required"
    return None


def _is_clock_fault_reason(reason: str) -> bool:
    return any(
        marker in reason
        for marker in (
            "clock_domain",
            "clock_epoch",
            "clock_rollback",
            "timestamp_future",
            "timestamp_rollback",
        )
    )


def _valid_digest(value: object) -> bool:
    return isinstance(value, str) and _DIGEST.fullmatch(value) is not None


def _has_correlated_downstream_terminal(
    terminal: ArmStageTerminal | GripperStageTerminal,
) -> bool:
    """Return true only for a legal wrapper and downstream action terminal.

    The caller establishes immutable command correlation before invoking this
    helper.  A boolean ``downstream_terminal_observed`` without legal action
    terminal statuses is not stop evidence and must retain the active command.
    """

    if isinstance(terminal, ArmStageTerminal):
        wrapper_status = terminal.plan_action_goal_status
        gate_terminal = terminal.gate_terminal
    else:
        wrapper_status = terminal.gate_action_goal_status
        gate_terminal = terminal.terminal
    return bool(
        gate_terminal
        and terminal.downstream_terminal_observed
        and wrapper_status in _ACTION_TERMINAL_STATUSES
        and terminal.action_goal_status in _ACTION_TERMINAL_STATUSES
    )


def _terminal_timestamp_reason(
    completed_timestamp_ns: object,
    issued_at_ns: int,
    now_ns: int,
) -> str | None:
    if (
        isinstance(completed_timestamp_ns, bool)
        or not isinstance(completed_timestamp_ns, int)
        or completed_timestamp_ns < 0
    ):
        return "invalid_terminal_timestamp"
    if completed_timestamp_ns < issued_at_ns:
        return "terminal_timestamp_before_issue"
    if completed_timestamp_ns > now_ns:
        return "terminal_timestamp_future"
    return None


def _require_orientation(
    value: tuple[float, float, float, float], field: str
) -> None:
    if not isinstance(value, tuple) or len(value) != 4:
        raise TypeError(f"{field} must be a four-value tuple")
    if any(isinstance(item, bool) or not isinstance(item, (int, float)) for item in value):
        raise TypeError(f"{field} must contain numeric values")
    if any(not isfinite(float(item)) for item in value):
        raise ValueError(f"{field} must contain finite values")
    norm = sum(float(item) * float(item) for item in value) ** 0.5
    if abs(norm - 1.0) > 1e-3:
        raise ValueError(f"{field} must be a normalized quaternion")


def _require_ns(value: int, field: str) -> None:
    reason = _clock_value_reason(value)
    if reason is not None:
        raise TypeError(f"{field} must be a non-negative integer") if isinstance(
            value, (bool, float)
        ) else ValueError(f"{field} must be non-negative")


def _require_epoch(value: int) -> None:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError("clock_epoch must be an integer")
    if value < 0:
        raise ValueError("clock_epoch must be non-negative")
