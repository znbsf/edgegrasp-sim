"""Safety decisions that are independent of ROS and any robot driver."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .models import NANOSECONDS_PER_SECOND, Target3D


class SafetyReason(str, Enum):
    ALLOWED = "allowed"
    STALE_TARGET = "stale_target"
    FUTURE_TARGET = "future_target"


@dataclass(frozen=True, slots=True)
class SafetyDecision:
    allowed: bool
    reason: SafetyReason
    age_ns: int


class StaleTargetGate:
    """Reject target observations older than a configurable threshold."""

    def __init__(self, max_age_ms: float = 200.0) -> None:
        if max_age_ms <= 0:
            raise ValueError("max_age_ms must be positive")
        self.max_age_ns = int(max_age_ms * NANOSECONDS_PER_SECOND / 1000.0)

    def evaluate(self, target: Target3D, now_ns: int) -> SafetyDecision:
        if isinstance(now_ns, bool) or not isinstance(now_ns, int):
            raise TypeError("now_ns must be an integer")
        if now_ns < 0:
            raise ValueError("now_ns must be non-negative")
        age_ns = now_ns - target.timestamp_ns
        if age_ns < 0:
            return SafetyDecision(False, SafetyReason.FUTURE_TARGET, age_ns)
        if age_ns > self.max_age_ns:
            return SafetyDecision(False, SafetyReason.STALE_TARGET, age_ns)
        return SafetyDecision(True, SafetyReason.ALLOWED, age_ns)
