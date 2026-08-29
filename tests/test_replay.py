from edgegrasp.factory import build_controller
from edgegrasp.models import Vector3
from edgegrasp.replay import replay_once, verify_repeated_replay
from edgegrasp.scenarios import LinearScenario, benchmark_scenarios


def test_replay_is_identical_across_100_runs() -> None:
    scenario = LinearScenario(
        name="determinism",
        start_m=Vector3(0.1, 0.0, 0.1),
        velocity_mps=Vector3(0.02, 0.0, 0.0),
    )

    reports = verify_repeated_replay(scenario, build_controller, runs=100)

    assert len(reports) == 100
    assert len({report.digest for report in reports}) == 1
    assert all(report.rejected == 0 for report in reports)


def test_all_acceptance_speeds_have_replay_fixtures() -> None:
    scenarios = benchmark_scenarios()

    assert [scenario.velocity_mps.x for scenario in scenarios] == [0.0, 0.02, 0.04]
    assert all(
        replay_once(scenario, build_controller).accepted == scenario.sample_count
        for scenario in scenarios
    )


def test_replay_digest_covers_receive_and_execute_boundary_times() -> None:
    scenario = LinearScenario(
        name="clock-contract",
        start_m=Vector3(0.1, 0.0, 0.1),
        velocity_mps=Vector3(0.0, 0.0, 0.0),
        sample_count=2,
    )

    fast = replay_once(scenario, build_controller, planning_delay_ms=1)
    slower = replay_once(scenario, build_controller, planning_delay_ms=2)

    assert fast.accepted == slower.accepted == 2
    assert fast.digest != slower.digest


def test_rejected_replay_is_also_deterministic() -> None:
    scenario = LinearScenario(
        name="frame-mismatch",
        start_m=Vector3(0.1, 0.0, 0.1),
        velocity_mps=Vector3(0.0, 0.0, 0.0),
        frame_id="camera_link",
        sample_count=3,
    )

    reports = verify_repeated_replay(scenario, build_controller, runs=20)

    assert len({report.digest for report in reports}) == 1
    assert all(report.accepted == 0 and report.rejected == 3 for report in reports)
