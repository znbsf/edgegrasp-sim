"""Pipeline orchestration with explicit fail-closed contracts."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .backend import ExecutionResult, MotionBackend, StopResult
from .config import (
    DEFAULT_CLOCK_DOMAIN,
    DEFAULT_MIN_CONFIDENCE,
    DEFAULT_TARGET_FRAME,
    DEFAULT_WATCHDOG_TIMEOUT_MS,
)
from .fsm import PipelineState, StateMachine
from .models import MotionPlan, NANOSECONDS_PER_SECOND, Prediction, Target3D
from .planner import PlanReason, PlanResult, Planner
from .predictor import ConstantVelocityPredictor
from .safety import StaleTargetGate


class ResultCode(str, Enum):
    EXECUTED = "executed"
    LATCHED = "latched"
    BUSY = "busy"
    CLOCK_FAULT = "clock_fault"
    TARGET_REJECTED = "target_rejected"
    FRAME_MISMATCH = "frame_mismatch"
    CLOCK_DOMAIN_MISMATCH = "clock_domain_mismatch"
    CLOCK_EPOCH_MISMATCH = "clock_epoch_mismatch"
    LOW_CONFIDENCE = "low_confidence"
    SOURCE_FRESHNESS = "source_freshness"
    PLAN_REJECTED = "plan_rejected"
    PLAN_PROTOCOL_INVALID = "plan_protocol_invalid"
    PLANNER_EXCEPTION = "planner_exception"
    WATCHDOG_TIMEOUT = "watchdog_timeout"
    INTERRUPTED = "interrupted"
    BACKEND_EXCEPTION = "backend_exception"
    EXECUTION_INVALID = "execution_invalid"
    EXECUTION_FAILED = "execution_failed"
    SAFE_STOP = "safe_stop"
    STOP_FAILED = "stop_failed"


@dataclass(frozen=True, slots=True)
class CycleResult:
    accepted: bool
    reason: str
    state: PipelineState
    prediction: Prediction | None = None
    plan_reason: PlanReason | None = None
    plan: MotionPlan | None = None
    code: ResultCode = ResultCode.SAFE_STOP


@dataclass(frozen=True, slots=True)
class WatchdogResult:
    triggered: bool
    reason: str
    state: PipelineState
    receive_age_ns: int | None


@dataclass(frozen=True, slots=True)
class ResetResult:
    success: bool
    reason: str
    state: PipelineState
    clock_epoch: int


class EdgeGraspController:
    """Connect input, prediction, endpoint gating, safety, and one backend.

    Clock contract:
    - ``now_ns`` and required ``execute_now_ns`` use ``clock_domain`` and the
      current ``clock_epoch``.
    - time is a non-negative, monotonically non-decreasing integer; rollback
      latches a fault until ``reset(..., new_clock_epoch=True)``.
    - source timestamp freshness and local receive-stream liveness are separate
      checks. ``tick`` is the active liveness watchdog.

    Execution contract:
    - ``MotionBackend.execute`` is synchronous. It returns only after completion
      or stop. A re-entrant target while state is EXECUTING is rejected.
    """

    def __init__(
        self,
        predictor: ConstantVelocityPredictor,
        planner: Planner,
        safety_gate: StaleTargetGate,
        backend: MotionBackend,
        prediction_horizon_ms: float = 100.0,
        watchdog_timeout_ms: float = DEFAULT_WATCHDOG_TIMEOUT_MS,
        target_frame: str = DEFAULT_TARGET_FRAME,
        clock_domain: str = DEFAULT_CLOCK_DOMAIN,
        min_confidence: float = DEFAULT_MIN_CONFIDENCE,
    ) -> None:
        if prediction_horizon_ms < 0:
            raise ValueError("prediction_horizon_ms must be non-negative")
        if watchdog_timeout_ms <= 0:
            raise ValueError("watchdog_timeout_ms must be positive")
        if not target_frame.strip():
            raise ValueError("target_frame must not be empty")
        if not clock_domain.strip():
            raise ValueError("clock_domain must not be empty")
        if not 0.0 <= min_confidence <= 1.0:
            raise ValueError("min_confidence must be in [0.0, 1.0]")

        self.predictor = predictor
        self.planner = planner
        self.safety_gate = safety_gate
        self.backend = backend
        self.prediction_horizon_ns = int(
            prediction_horizon_ms * NANOSECONDS_PER_SECOND / 1000.0
        )
        self.watchdog_timeout_ns = int(
            watchdog_timeout_ms * NANOSECONDS_PER_SECOND / 1000.0
        )
        self.target_frame = target_frame
        self.clock_domain = clock_domain
        self.min_confidence = min_confidence
        self.state_machine = StateMachine()
        self._clock_epoch = 0
        self._last_now_ns: int | None = None
        self._last_fresh_receive_ns: int | None = None
        self._clock_fault_latched = False

    @property
    def state(self) -> PipelineState:
        return self.state_machine.state

    @property
    def clock_epoch(self) -> int:
        return self._clock_epoch

    @property
    def last_now_ns(self) -> int | None:
        return self._last_now_ns

    def process_target(
        self,
        target: Target3D,
        now_ns: int,
        execute_now_ns: int,
    ) -> CycleResult:
        if self.state in (PipelineState.SAFE_STOP, PipelineState.ERROR):
            return CycleResult(
                False,
                "controller_latched; reset_required",
                self.state,
                code=ResultCode.LATCHED,
            )
        if self.state in (PipelineState.PLANNING, PipelineState.EXECUTING):
            return CycleResult(
                False,
                f"{self.state.value.lower()}_in_progress",
                self.state,
                code=ResultCode.BUSY,
            )

        clock_error = self._observe_clock(now_ns, "receive")
        if clock_error is not None:
            return self._clock_fault(clock_error)
        assert self._last_now_ns is not None

        if not isinstance(target, Target3D):
            return self._safe_stop(
                "invalid_target_type", self._last_now_ns, code=ResultCode.TARGET_REJECTED
            )

        self.state_machine.transition(PipelineState.TRACKING, "target_received", now_ns)
        if target.frame_id != self.target_frame:
            return self._safe_stop(
                f"frame_mismatch:{target.frame_id}!={self.target_frame}",
                now_ns,
                code=ResultCode.FRAME_MISMATCH,
            )
        if target.clock_domain != self.clock_domain:
            return self._safe_stop(
                f"clock_domain_mismatch:{target.clock_domain}!={self.clock_domain}",
                now_ns,
                code=ResultCode.CLOCK_DOMAIN_MISMATCH,
            )
        if target.clock_epoch != self._clock_epoch:
            return self._safe_stop(
                f"clock_epoch_mismatch:{target.clock_epoch}!={self._clock_epoch}",
                now_ns,
                code=ResultCode.CLOCK_EPOCH_MISMATCH,
            )
        if target.confidence < self.min_confidence:
            return self._safe_stop(
                f"low_confidence:{target.confidence:.6g}<{self.min_confidence:.6g}",
                now_ns,
                code=ResultCode.LOW_CONFIDENCE,
            )

        try:
            source_safety = self.safety_gate.evaluate(target, now_ns)
        except (TypeError, ValueError) as error:
            return self._clock_fault(f"invalid_source_clock:{type(error).__name__}:{error}")
        if not source_safety.allowed:
            return self._safe_stop(
                source_safety.reason.value,
                now_ns,
                code=ResultCode.SOURCE_FRESHNESS,
            )

        try:
            self.predictor.update(target)
        except (TypeError, ValueError) as error:
            return self._safe_stop(
                f"invalid_target_sequence:{error}",
                now_ns,
                code=ResultCode.TARGET_REJECTED,
            )
        self._last_fresh_receive_ns = now_ns

        prediction = self.predictor.predict_at(now_ns + self.prediction_horizon_ns)
        self.state_machine.transition(PipelineState.PLANNING, "prediction_ready", now_ns)
        try:
            plan_result = self.planner.plan(prediction, now_ns)
        except Exception as error:
            return self._enter_error(
                f"planner_exception:{type(error).__name__}",
                now_ns,
                prediction,
                code=ResultCode.PLANNER_EXCEPTION,
            )

        if self.state is not PipelineState.PLANNING:
            return CycleResult(
                False,
                f"planning_interrupted:{self.state.value}",
                self.state,
                prediction,
                code=ResultCode.INTERRUPTED,
            )

        plan_error = self._validate_plan_result(plan_result, prediction, now_ns)
        if plan_error is not None:
            reason, observed_reason = plan_error
            code = (
                ResultCode.PLAN_PROTOCOL_INVALID
                if reason.startswith("invalid_plan_result:")
                else ResultCode.PLAN_REJECTED
            )
            return self._safe_stop(
                reason, now_ns, prediction, observed_reason, code=code
            )
        assert isinstance(plan_result, PlanResult)
        assert isinstance(plan_result.plan, MotionPlan)
        plan = plan_result.plan

        clock_error = self._observe_clock(execute_now_ns, "execute_boundary")
        if clock_error is not None:
            return self._clock_fault(
                clock_error,
                prediction=prediction,
                plan_reason=plan_result.reason,
                plan=plan,
            )
        assert self._last_now_ns is not None

        if self.state is not PipelineState.PLANNING:
            return CycleResult(
                False,
                f"execution_interrupted:{self.state.value}",
                self.state,
                prediction,
                plan_result.reason,
                plan,
                ResultCode.INTERRUPTED,
            )

        source_safety = self.safety_gate.evaluate(target, execute_now_ns)
        if not source_safety.allowed:
            return self._safe_stop(
                source_safety.reason.value,
                execute_now_ns,
                prediction,
                plan_result.reason,
                plan,
                code=ResultCode.SOURCE_FRESHNESS,
            )
        receive_age_ns = execute_now_ns - self._last_fresh_receive_ns
        if receive_age_ns > self.watchdog_timeout_ns:
            return self._safe_stop(
                "target_stream_timeout",
                execute_now_ns,
                prediction,
                plan_result.reason,
                plan,
                code=ResultCode.WATCHDOG_TIMEOUT,
            )

        self.state_machine.transition(
            PipelineState.EXECUTING, "plan_accepted", execute_now_ns
        )
        try:
            execution = self.backend.execute(plan)
        except Exception as error:
            return self._enter_error(
                f"backend_exception:{type(error).__name__}",
                execute_now_ns,
                prediction,
                plan_result.reason,
                plan,
                code=ResultCode.BACKEND_EXCEPTION,
            )

        if self.state is not PipelineState.EXECUTING:
            return CycleResult(
                False,
                f"execution_completed_after:{self.state.value}",
                self.state,
                prediction,
                plan_result.reason,
                plan,
                ResultCode.INTERRUPTED,
            )
        if not isinstance(execution, ExecutionResult) or not isinstance(
            execution.success, bool
        ):
            return self._enter_error(
                "invalid_execution_result",
                execute_now_ns,
                prediction,
                plan_result.reason,
                plan,
                code=ResultCode.EXECUTION_INVALID,
            )
        if not execution.success:
            return self._enter_error(
                execution.reason,
                execute_now_ns,
                prediction,
                plan_result.reason,
                plan,
                code=ResultCode.EXECUTION_FAILED,
            )

        self.state_machine.transition(
            PipelineState.TRACKING, execution.reason, execute_now_ns
        )
        return CycleResult(
            True,
            execution.reason,
            self.state,
            prediction,
            plan_result.reason,
            plan,
            ResultCode.EXECUTED,
        )

    def tick(self, now_ns: int) -> WatchdogResult:
        """Evaluate local receive-stream liveness without a new target."""

        if self.state in (PipelineState.SAFE_STOP, PipelineState.ERROR):
            clock_error = self._observe_clock(now_ns, "watchdog_latched")
            if clock_error is not None:
                self._clock_fault_latched = True
                return WatchdogResult(False, clock_error, self.state, None)
            return WatchdogResult(False, "controller_already_latched", self.state, None)

        clock_error = self._observe_clock(now_ns, "watchdog")
        if clock_error is not None:
            result = self._clock_fault(clock_error)
            return WatchdogResult(True, result.reason, result.state, None)

        if self._last_fresh_receive_ns is None:
            if self.state is PipelineState.IDLE:
                return WatchdogResult(False, "no_target_yet", self.state, None)
            result = self._safe_stop(
                "watchdog_no_fresh_target", now_ns, code=ResultCode.WATCHDOG_TIMEOUT
            )
            return WatchdogResult(True, result.reason, result.state, None)

        receive_age_ns = now_ns - self._last_fresh_receive_ns
        if receive_age_ns <= self.watchdog_timeout_ns:
            return WatchdogResult(False, "within_watchdog_budget", self.state, receive_age_ns)

        result = self._safe_stop(
            "target_stream_timeout", now_ns, code=ResultCode.WATCHDOG_TIMEOUT
        )
        return WatchdogResult(True, result.reason, result.state, receive_age_ns)

    def stop(self, reason: str, now_ns: int) -> CycleResult:
        if self.state in (PipelineState.SAFE_STOP, PipelineState.ERROR):
            return CycleResult(
                False,
                "controller_already_latched",
                self.state,
                code=ResultCode.LATCHED,
            )
        clock_error = self._observe_clock(now_ns, "manual_stop")
        if clock_error is not None:
            return self._clock_fault(clock_error)
        return self._safe_stop(reason, now_ns)

    def reset(self, now_ns: int, new_clock_epoch: bool = False) -> ResetResult:
        """Reset a latch; a clock fault requires an explicit new epoch."""

        primitive_error = self._validate_clock_value(now_ns, "reset")
        if primitive_error is not None:
            return ResetResult(False, primitive_error, self.state, self._clock_epoch)
        if self._clock_fault_latched and not new_clock_epoch:
            return ResetResult(
                False, "new_clock_epoch_required", self.state, self._clock_epoch
            )
        if not new_clock_epoch:
            clock_error = self._observe_clock(now_ns, "reset")
            if clock_error is not None:
                self._clock_fault_latched = True
                return ResetResult(False, clock_error, self.state, self._clock_epoch)

        if self.state in (
            PipelineState.PLANNING,
            PipelineState.EXECUTING,
            PipelineState.ERROR,
        ):
            stop_result = self._attempt_backend_stop("reset_requested")
            if not stop_result.success:
                reason = f"reset_stop_failed:{stop_result.reason}"
                self._transition_to_error(reason, now_ns)
                return ResetResult(False, reason, self.state, self._clock_epoch)
            if self.state in (PipelineState.PLANNING, PipelineState.EXECUTING):
                self.state_machine.transition(
                    PipelineState.SAFE_STOP, "stopped_for_reset", now_ns
                )

        self.predictor.reset()
        self._last_fresh_receive_ns = None
        if new_clock_epoch:
            self._clock_epoch += 1
            self._last_now_ns = now_ns
            self._clock_fault_latched = False
        if self.state is not PipelineState.IDLE:
            self.state_machine.reset(now_ns)
        return ResetResult(True, "reset", self.state, self._clock_epoch)

    def _observe_clock(self, now_ns: int, label: str) -> str | None:
        primitive_error = self._validate_clock_value(now_ns, label)
        if primitive_error is not None:
            return primitive_error
        if self._last_now_ns is not None and now_ns < self._last_now_ns:
            return f"clock_rollback:{label}:{now_ns}<{self._last_now_ns}"
        self._last_now_ns = now_ns
        return None

    @staticmethod
    def _validate_clock_value(now_ns: int, label: str) -> str | None:
        if isinstance(now_ns, bool) or not isinstance(now_ns, int):
            return f"invalid_clock:{label}:integer_required"
        if now_ns < 0:
            return f"invalid_clock:{label}:non_negative_required"
        return None

    def _clock_fault(
        self,
        reason: str,
        prediction: Prediction | None = None,
        plan_reason: PlanReason | None = None,
        plan: MotionPlan | None = None,
        code: ResultCode = ResultCode.CLOCK_FAULT,
    ) -> CycleResult:
        self._clock_fault_latched = True
        transition_ns = 0 if self._last_now_ns is None else self._last_now_ns
        return self._safe_stop(
            reason, transition_ns, prediction, plan_reason, plan, code=code
        )

    def _validate_plan_result(
        self,
        result: object,
        prediction: Prediction,
        now_ns: int,
    ) -> tuple[str, PlanReason | None] | None:
        if not isinstance(result, PlanResult):
            return ("invalid_plan_result:type", None)
        if not isinstance(result.success, bool) or not isinstance(result.reason, PlanReason):
            return ("invalid_plan_result:fields", None)

        if not result.success:
            if result.reason is PlanReason.PLANNED or result.plan is not None:
                return ("invalid_plan_result:inconsistent_rejection", result.reason)
            return (result.reason.value, result.reason)

        if result.reason is not PlanReason.PLANNED or result.plan is None:
            return ("invalid_plan_result:inconsistent_success", result.reason)
        if not isinstance(result.plan, MotionPlan):
            return ("invalid_plan_result:plan_type", result.reason)

        plan = result.plan
        expected = (
            plan.target_id == prediction.target_id
            and plan.source_timestamp_ns == prediction.source_timestamp_ns
            and plan.requested_at_ns == now_ns
            and plan.frame_id == self.target_frame == prediction.frame_id
            and plan.clock_domain == self.clock_domain == prediction.clock_domain
            and plan.clock_epoch == self._clock_epoch == prediction.clock_epoch
            and bool(plan.waypoints_m)
            and plan.waypoints_m[-1] == prediction.position_m
        )
        if not expected:
            return ("invalid_plan_result:plan_contract", result.reason)
        return None

    def _attempt_backend_stop(self, reason: str) -> StopResult:
        try:
            result = self.backend.stop(reason)
        except Exception as error:
            return StopResult(False, f"stop_exception:{type(error).__name__}:{error}")
        if not isinstance(result, StopResult) or not isinstance(result.success, bool):
            return StopResult(False, f"invalid_stop_result:{type(result).__name__}")
        return result

    def _safe_stop(
        self,
        reason: str,
        now_ns: int,
        prediction: Prediction | None = None,
        plan_reason: PlanReason | None = None,
        plan: MotionPlan | None = None,
        code: ResultCode = ResultCode.SAFE_STOP,
    ) -> CycleResult:
        if self.state in (PipelineState.SAFE_STOP, PipelineState.ERROR):
            return CycleResult(
                False, reason, self.state, prediction, plan_reason, plan, code
            )

        stop_result = self._attempt_backend_stop(reason)
        if stop_result.success:
            self.state_machine.transition(PipelineState.SAFE_STOP, reason, now_ns)
            final_reason = reason
        else:
            final_reason = f"{reason};stop_failed:{stop_result.reason}"
            self._transition_to_error(final_reason, now_ns)
            code = ResultCode.STOP_FAILED
        return CycleResult(
            False, final_reason, self.state, prediction, plan_reason, plan, code
        )

    def _enter_error(
        self,
        reason: str,
        now_ns: int,
        prediction: Prediction | None = None,
        plan_reason: PlanReason | None = None,
        plan: MotionPlan | None = None,
        code: ResultCode = ResultCode.EXECUTION_FAILED,
    ) -> CycleResult:
        stop_result = self._attempt_backend_stop(reason)
        final_reason = reason
        if not stop_result.success:
            final_reason = f"{reason};stop_failed:{stop_result.reason}"
            code = ResultCode.STOP_FAILED
        self._transition_to_error(final_reason, now_ns)
        return CycleResult(
            False, final_reason, self.state, prediction, plan_reason, plan, code
        )

    def _transition_to_error(self, reason: str, now_ns: int) -> None:
        if self.state is not PipelineState.ERROR:
            self.state_machine.transition(PipelineState.ERROR, reason, now_ns)
