import pytest

from edgegrasp.permission import MotionPermissionGate


def test_unknown_false_and_stale_permission_all_fail_closed() -> None:
    gate = MotionPermissionGate(timeout_ms=100)

    assert gate.evaluate(1_000).reason == "safety_unknown"
    assert not gate.update(False, 2_000).allowed
    assert gate.evaluate(3_000).reason == "safety_false"

    assert gate.update(True, 10_000).allowed
    assert gate.evaluate(100_010_000).allowed
    stale = gate.evaluate(100_010_001)
    assert not stale.allowed
    assert stale.reason == "safety_signal_stale"


def test_permission_clock_rollback_latches_until_epoch_reset() -> None:
    gate = MotionPermissionGate(timeout_ms=100)
    gate.update(True, 1_000)

    rollback = gate.evaluate(999)
    assert not rollback.allowed
    assert rollback.reason.startswith("clock_rollback:")
    assert not gate.update(True, 1_001).allowed

    gate.reset_epoch(10)
    assert gate.evaluate(10).reason == "safety_unknown"
    assert gate.update(True, 11).allowed


@pytest.mark.parametrize("invalid_now", [-1, 1.5, True])
def test_permission_invalid_clock_fails_closed(invalid_now: object) -> None:
    gate = MotionPermissionGate()

    decision = gate.evaluate(invalid_now)  # type: ignore[arg-type]

    assert not decision.allowed
    assert decision.reason.startswith("invalid_clock:")
