from __future__ import annotations
from edgegrasp_interfaces.msg import TrackedTarget

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from edgegrasp.grasp_geometry import load_grasp_geometry_profile
from edgegrasp_grasp_sequence import client as client_module
from edgegrasp_grasp_sequence.client import GraspSequenceClient
from edgegrasp_grasp_sequence.trial_client import GraspTrialClient, _config_basename


class _GoalBuilt(RuntimeError):
    pass


@pytest.mark.parametrize(
    "value",
    ("", "../profile.json", "nested/profile.json", "profile.yaml", 7),
)
def test_trial_client_rejects_nonlocal_or_non_json_geometry_config(value) -> None:
    with pytest.raises(ValueError, match="grasp_geometry_filename"):
        _config_basename(value)


def test_trial_client_accepts_an_installed_geometry_config_basename() -> None:
    assert _config_basename("so101_grasp_geometry_candidate008_q0p50.json") == (
        "so101_grasp_geometry_candidate008_q0p50.json"
    )


def test_trial_client_checks_the_bound_target_snapshot_not_a_newer_message() -> None:
    node = object.__new__(GraspTrialClient)
    node.get_parameter = lambda name: SimpleNamespace(
        value={"max_target_age_before_send_ms": 100.0}[name]
    )
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)
    )

    def target_at(source_ns: int):
        return SimpleNamespace(
            observation=SimpleNamespace(
                header=SimpleNamespace(
                    stamp=SimpleNamespace(
                        sec=source_ns // 1_000_000_000,
                        nanosec=source_ns % 1_000_000_000,
                    )
                )
            )
        )

    bound_target = target_at(850_000_000)
    node._latest = target_at(990_000_000)

    assert not node._target_is_recent(bound_target)
    assert node._latest_is_recent()

    node._physics_attempt_generation = 2
    node._physics_phase = "WAIT_BASELINE"
    node._physics_reason = "current"
    feedback = SimpleNamespace(
        feedback=SimpleNamespace(
            task_id="trial",
            phase="WAIT_CONTACT",
            reason="late",
        )
    )
    node._physics_feedback(feedback, generation=1)
    assert (node._physics_phase, node._physics_reason) == (
        "WAIT_BASELINE",
        "current",
    )
    node._physics_feedback(feedback, generation=2)
    assert (node._physics_phase, node._physics_reason) == (
        "WAIT_CONTACT",
        "late",
    )


def test_trial_client_restarts_only_after_correlated_physics_cancel_terminal(
    capsys: pytest.CaptureFixture[str],
) -> None:
    node = object.__new__(GraspTrialClient)
    node._positive = lambda _name: 1.0
    node._wait_future = lambda _future, _timeout: True

    expected = ("task", "target", 900, "ros_sim", 0)

    class _Future:
        def __init__(self, value) -> None:
            self._value = value

        def result(self):
            return self._value

    class _Handle:
        @staticmethod
        def cancel_goal_async():
            return _Future(SimpleNamespace(goals_canceling=[object()]))

    result = SimpleNamespace(
        accepted=True,
        terminal_phase="FAULT",
        reason="observer_cancelled",
        task_id="task",
        target_id="target",
        target_source_timestamp_ns=900,
        clock_domain="ros_sim",
        clock_epoch=0,
    )
    canceled = _Future(
        SimpleNamespace(status=GoalStatus.STATUS_CANCELED, result=result)
    )
    assert node._cancel_and_confirm(
        _Handle(),
        canceled,
        "physics_baseline_target_stale",
        expected_physics_restart=expected,
    )

    aborted = _Future(
        SimpleNamespace(status=GoalStatus.STATUS_ABORTED, result=result)
    )
    assert not node._cancel_and_confirm(
        _Handle(),
        aborted,
        "physics_baseline_target_stale",
        expected_physics_restart=expected,
    )
    records = [json.loads(line) for line in capsys.readouterr().out.splitlines()]
    assert records[0]["physics_restart_terminal_confirmed"] is True
    assert records[1]["physics_restart_terminal_confirmed"] is False


def test_trial_client_derives_candidate009_descend_and_lift_from_profile() -> None:
    node = object.__new__(GraspTrialClient)
    profile = load_grasp_geometry_profile(
        Path(get_package_share_directory("edgegrasp_ros"))
        / "config"
        / "so101_grasp_geometry_candidate009_centered_q0p50.json"
    )
    node._grasp_profile = profile
    node._cube = SimpleNamespace(
        pose_world=SimpleNamespace(
            position_m=(0.24695465627174787, 0.1218646928533295, 0.205)
        )
    )
    parameters = {
        "derive_descend_and_lift_from_profile": True,
        "grasp_orientation_xyzw": [
            0.3450298741758752,
            0.6172118344266647,
            0.6332767577280262,
            0.3145862131297726,
        ],
    }
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    node._position = lambda name: {
        "approach_position_m": (
            0.18606933614192925,
            0.11976423272744036,
            0.38721872836424076,
        )
    }[name]

    target = TrackedTarget()
    target.observation.point.x = 0.24695465627174787
    target.observation.point.y = 0.1218646928533295
    target.observation.point.z = 0.205
    approach, descend, lift = node._resolved_stage_positions(target)

    assert approach == (
        0.18606933614192925,
        0.11976423272744036,
        0.38721872836424076,
    )
    assert descend == pytest.approx(
        (0.24572885309767808, 0.1519924630851577, 0.2167578445611808),
        abs=1e-12,
    )
    assert lift == pytest.approx(
        (0.24572885309767808, 0.1519924630851577, 0.2567578445611808),
        abs=1e-12,
    )
    target.observation.point.x += 0.0002
    # Poison scene truth: stage generation must depend only on the observation.
    node._cube.pose_world.position_m = (9., 9., 9.)
    _, shifted_descend, shifted_lift = node._resolved_stage_positions(target)
    assert shifted_descend[0] == pytest.approx(descend[0] + 0.0002)
    assert shifted_lift[0] == pytest.approx(lift[0] + 0.0002)


def test_client_waits_for_action_server_then_snapshots_fresh_target(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    events: list[str] = []
    node = object.__new__(GraspSequenceClient)
    node._latest = object()  # A pre-discovery sample must be discarded.
    node._seconds = lambda _name: 1.0
    node._latest_is_recent = lambda: node._latest is not None

    class _ReadyClient:
        def wait_for_server(self, *, timeout_sec: float) -> bool:
            assert timeout_sec == 1.0
            events.append("server_ready")
            return True

        def send_goal_async(self, *_args, **_kwargs):
            raise AssertionError("goal construction should stop this test")

    node._client = _ReadyClient()

    def _spin_once(actual_node, *, timeout_sec: float) -> None:
        assert actual_node is node
        assert timeout_sec == 0.05
        assert node._latest is None
        events.append("fresh_target")
        node._latest = SimpleNamespace(target_id="fresh")

    def _goal():
        events.append("build_goal")
        raise _GoalBuilt

    node._goal = _goal
    monkeypatch.setattr(client_module.rclpy, "spin_once", _spin_once)

    with pytest.raises(_GoalBuilt):
        node.run()

    assert events == ["server_ready", "fresh_target", "build_goal"]


def test_client_skips_a_queued_stale_target_before_building_goal(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    events: list[str] = []
    node = object.__new__(GraspSequenceClient)
    node._latest = object()
    node._seconds = lambda _name: 1.0
    node._client = SimpleNamespace(
        wait_for_server=lambda **_kwargs: events.append("server_ready") or True,
        send_goal_async=lambda *_args, **_kwargs: pytest.fail(
            "goal construction should stop this test"
        ),
    )
    samples = iter(("stale", "fresh"))

    def _spin_once(actual_node, *, timeout_sec: float) -> None:
        assert actual_node is node
        assert timeout_sec == 0.05
        node._latest = SimpleNamespace(target_id=next(samples))
        events.append(node._latest.target_id)

    node._latest_is_recent = lambda: (
        node._latest is not None and node._latest.target_id == "fresh"
    )

    def _goal():
        events.append("build_goal")
        raise _GoalBuilt

    node._goal = _goal
    monkeypatch.setattr(client_module.rclpy, "spin_once", _spin_once)

    with pytest.raises(_GoalBuilt):
        node.run()

    assert events == ["server_ready", "stale", "fresh", "build_goal"]


def test_client_does_not_consume_target_while_server_is_unavailable(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    node = object.__new__(GraspSequenceClient)
    node._latest = object()
    node._seconds = lambda _name: 1.0
    node._client = SimpleNamespace(wait_for_server=lambda **_kwargs: False)
    monkeypatch.setattr(
        client_module.rclpy,
        "spin_once",
        lambda *_args, **_kwargs: pytest.fail("target wait ran before discovery"),
    )

    assert node.run() == 3
    assert "grasp_sequence_action_unavailable" in capsys.readouterr().out


@pytest.mark.parametrize(
    "parameter",
    ("approach_orientation_xyzw", "grasp_orientation_xyzw"),
)
def test_client_requires_explicit_normalized_task_orientation(parameter) -> None:
    node = object.__new__(GraspSequenceClient)
    node.get_parameter = lambda _name: SimpleNamespace(
        value=[0.0, 0.0, 0.0, 0.0]
    )

    with pytest.raises(ValueError, match="normalized quaternion"):
        node._orientation(parameter)


@pytest.mark.parametrize(
    "parameter",
    ("approach_orientation_xyzw", "grasp_orientation_xyzw"),
)
def test_client_preserves_normalized_task_orientation(parameter) -> None:
    expected = [0.0, 0.7071067811865476, 0.0, 0.7071067811865476]
    node = object.__new__(GraspSequenceClient)
    node.get_parameter = lambda _name: SimpleNamespace(value=expected)

    assert node._orientation(parameter) == tuple(expected)
