"""Clock callback ordering must not require accepting or rewriting future stamps."""

from types import SimpleNamespace

from edgegrasp_ros.grasp_physics_observer import GraspPhysicsObserver
import edgegrasp_ros.grasp_physics_observer as module


def test_admission_waits_for_actual_clock(monkeypatch):
    samples=iter((999_000_000, 999_500_000, 1_000_000_000))
    node=SimpleNamespace(_now_ns=lambda: next(samples), _clock_fault=None)
    monkeypatch.setattr(module.time, 'sleep', lambda _: None)
    result=GraspPhysicsObserver._wait_for_admission_clock(node, 1_000_000_000, 200_000_000)
    assert result == 1_000_000_000


def test_stopped_clock_remains_future_after_bounded_wait(monkeypatch):
    wall=iter((0., .05, .1, .15, .21))
    node=SimpleNamespace(_now_ns=lambda: 999_000_000, _clock_fault=None)
    monkeypatch.setattr(module.time, 'sleep', lambda _: None)
    monkeypatch.setattr(module.time, 'monotonic', lambda: next(wall))
    result=GraspPhysicsObserver._wait_for_admission_clock(node, 1_000_000_000, 200_000_000)
    assert result < 1_000_000_000


def test_clock_fault_aborts_wait_and_large_future_is_not_buffered(monkeypatch):
    node=SimpleNamespace(_now_ns=lambda: 999_000_000, _clock_fault='ros_clock_rollback')
    monkeypatch.setattr(module.time, 'sleep', lambda _: (_ for _ in ()).throw(AssertionError('must not wait')))
    assert GraspPhysicsObserver._wait_for_admission_clock(node, 1_000_000_000, 200_000_000) == 999_000_000
    node._clock_fault=None
    assert GraspPhysicsObserver._wait_for_admission_clock(node, 2_000_000_000, 200_000_000) == 999_000_000
