"""Backend contract shared by mock, Gazebo, and future real adapters."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Protocol

from .models import MotionPlan


class BackendKind(str, Enum):
    MOCK = "mock"
    GAZEBO = "gazebo"
    REAL = "real"


@dataclass(frozen=True, slots=True)
class ExecutionResult:
    success: bool
    reason: str


@dataclass(frozen=True, slots=True)
class StopResult:
    success: bool
    reason: str


class MotionBackend(Protocol):
    """Synchronous backend contract.

    ``execute`` returns only after the plan has completed or stopped. An
    asynchronous ROS action adapter must add an explicit completion state
    machine instead of pretending to implement this interface.
    """

    @property
    def kind(self) -> BackendKind: ...

    def execute(self, plan: MotionPlan) -> ExecutionResult: ...

    def stop(self, reason: str) -> StopResult: ...


class MockBackend:
    """In-memory deterministic backend used by tests and replay."""

    def __init__(
        self,
        fail_execution: bool = False,
        fail_stop: bool = False,
        raise_on_stop: bool = False,
    ) -> None:
        self.fail_execution = fail_execution
        self.fail_stop = fail_stop
        self.raise_on_stop = raise_on_stop
        self.executed_plans: list[MotionPlan] = []
        self.stop_reasons: list[str] = []

    @property
    def kind(self) -> BackendKind:
        return BackendKind.MOCK

    def execute(self, plan: MotionPlan) -> ExecutionResult:
        if self.fail_execution:
            return ExecutionResult(False, "mock_execution_failure")
        self.executed_plans.append(plan)
        return ExecutionResult(True, "executed")

    def stop(self, reason: str) -> StopResult:
        self.stop_reasons.append(reason)
        if self.raise_on_stop:
            raise RuntimeError("mock_stop_exception")
        if self.fail_stop:
            return StopResult(False, "mock_stop_failed")
        return StopResult(True, "stopped")


class DisabledBackend:
    """Fail-closed placeholder until a ROS adapter is explicitly attached."""

    def __init__(self, kind: BackendKind) -> None:
        if kind is BackendKind.MOCK:
            raise ValueError("use MockBackend for the mock backend")
        self._kind = kind
        self.stop_reasons: list[str] = []

    @property
    def kind(self) -> BackendKind:
        return self._kind

    def execute(self, plan: MotionPlan) -> ExecutionResult:
        return ExecutionResult(False, f"{self._kind.value}_adapter_not_configured")

    def stop(self, reason: str) -> StopResult:
        self.stop_reasons.append(reason)
        return StopResult(True, "already_disabled")


def create_backend(kind: str | BackendKind) -> MotionBackend:
    selected = BackendKind(kind)
    if selected is BackendKind.MOCK:
        return MockBackend()
    return DisabledBackend(selected)
