import pytest

from edgegrasp.models import Target3D, Vector3
from edgegrasp.safety import SafetyReason, StaleTargetGate


def make_target(timestamp_ns: int) -> Target3D:
    return Target3D("cube", Vector3(0.1, 0.0, 0.1), timestamp_ns)


def test_target_at_200_ms_boundary_is_allowed() -> None:
    gate = StaleTargetGate(max_age_ms=200)

    decision = gate.evaluate(make_target(1_000_000_000), 1_200_000_000)

    assert decision.allowed
    assert decision.reason is SafetyReason.ALLOWED


def test_target_older_than_200_ms_is_rejected() -> None:
    gate = StaleTargetGate(max_age_ms=200)

    decision = gate.evaluate(make_target(1_000_000_000), 1_200_000_001)

    assert not decision.allowed
    assert decision.reason is SafetyReason.STALE_TARGET


def test_far_future_target_is_rejected() -> None:
    gate = StaleTargetGate(max_age_ms=200)

    decision = gate.evaluate(make_target(1_010_000_000), 1_000_000_000)

    assert not decision.allowed
    assert decision.reason is SafetyReason.FUTURE_TARGET


@pytest.mark.parametrize("invalid_now", [-1, 1.5, True])
def test_invalid_evaluation_clock_is_rejected(invalid_now: object) -> None:
    gate = StaleTargetGate(max_age_ms=200)

    with pytest.raises((TypeError, ValueError)):
        gate.evaluate(make_target(1_000_000_000), invalid_now)  # type: ignore[arg-type]
