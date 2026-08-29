"""Dependency-free liveness gate used at the ROS motion-command boundary."""

from __future__ import annotations

from dataclasses import dataclass

from .models import NANOSECONDS_PER_SECOND


@dataclass(frozen=True, slots=True)
class PermissionDecision:
    allowed: bool
    reason: str
    signal_age_ns: int | None


class MotionPermissionGate:
    """Fail closed when a permission signal is false, unknown, stale, or rewound."""

    def __init__(self, timeout_ms: float = 100.0) -> None:
        if timeout_ms <= 0:
            raise ValueError("timeout_ms must be positive")
        self.timeout_ns = int(timeout_ms * NANOSECONDS_PER_SECOND / 1000.0)
        self._allowed: bool | None = None
        self._signal_ns: int | None = None
        self._last_now_ns: int | None = None
        self._fault: str | None = None

    def update(self, allowed: bool, now_ns: int) -> PermissionDecision:
        if not isinstance(allowed, bool):
            raise TypeError("allowed must be bool")
        clock_error = self._observe(now_ns)
        if clock_error is not None:
            self._fault = clock_error
            return PermissionDecision(False, clock_error, None)
        if self._fault is not None:
            return PermissionDecision(False, self._fault, None)
        self._allowed = allowed
        self._signal_ns = now_ns
        return PermissionDecision(allowed, "allowed" if allowed else "safety_false", 0)

    def evaluate(self, now_ns: int) -> PermissionDecision:
        clock_error = self._observe(now_ns)
        if clock_error is not None:
            self._fault = clock_error
        if self._fault is not None:
            return PermissionDecision(False, self._fault, None)
        if self._allowed is None or self._signal_ns is None:
            return PermissionDecision(False, "safety_unknown", None)
        age_ns = now_ns - self._signal_ns
        if age_ns > self.timeout_ns:
            return PermissionDecision(False, "safety_signal_stale", age_ns)
        if not self._allowed:
            return PermissionDecision(False, "safety_false", age_ns)
        return PermissionDecision(True, "allowed", age_ns)

    def reset_epoch(self, now_ns: int) -> None:
        error = self._validate_now(now_ns)
        if error is not None:
            raise ValueError(error)
        self._allowed = None
        self._signal_ns = None
        self._last_now_ns = now_ns
        self._fault = None

    def _observe(self, now_ns: int) -> str | None:
        error = self._validate_now(now_ns)
        if error is not None:
            return error
        if self._last_now_ns is not None and now_ns < self._last_now_ns:
            return f"clock_rollback:{now_ns}<{self._last_now_ns}"
        self._last_now_ns = now_ns
        return None

    @staticmethod
    def _validate_now(now_ns: int) -> str | None:
        if isinstance(now_ns, bool) or not isinstance(now_ns, int):
            return "invalid_clock:integer_required"
        if now_ns < 0:
            return "invalid_clock:non_negative_required"
        return None
