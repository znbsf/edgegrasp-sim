"""Measure deterministic prediction error and local compute latency.

The output describes this invocation only. It is not a Gazebo, ROS, or robot
grasp-success benchmark.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from statistics import median
import sys
from time import perf_counter_ns

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from edgegrasp.factory import build_controller  # noqa: E402
from edgegrasp.predictor import ConstantVelocityPredictor  # noqa: E402
from edgegrasp.replay import verify_repeated_replay  # noqa: E402
from edgegrasp.scenarios import benchmark_scenarios  # noqa: E402


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    index = min(len(ordered) - 1, int((len(ordered) - 1) * fraction + 0.5))
    return ordered[index]


def measure_scenario(scenario, replay_runs: int, horizon_ms: int) -> dict[str, object]:
    predictor = ConstantVelocityPredictor()
    errors_mm: list[float] = []
    compute_us: list[float] = []
    horizon_ns = horizon_ms * 1_000_000

    for index, sample in enumerate(scenario.samples()):
        predictor.update(sample)
        start_ns = perf_counter_ns()
        prediction = predictor.predict_at(sample.timestamp_ns + horizon_ns)
        compute_us.append((perf_counter_ns() - start_ns) / 1000.0)
        if index > 0:  # two observations are required to estimate velocity
            truth = scenario.position_at(sample.timestamp_ns + horizon_ns)
            errors_mm.append(prediction.position_m.distance_to(truth) * 1000.0)

    replay_start_ns = perf_counter_ns()
    reports = verify_repeated_replay(scenario, build_controller, runs=replay_runs)
    replay_elapsed_ms = (perf_counter_ns() - replay_start_ns) / 1_000_000.0
    return {
        "scenario": scenario.name,
        "speed_mm_s": scenario.velocity_mps.x * 1000.0,
        "prediction_horizon_ms": horizon_ms,
        "prediction_error_mm": {
            "max": max(errors_mm),
            "mean": sum(errors_mm) / len(errors_mm),
        },
        "predict_compute_us": {
            "p50": median(compute_us),
            "p95": percentile(compute_us, 0.95),
        },
        "replay": {
            "runs": replay_runs,
            "cycles_per_run": reports[0].cycles,
            "accepted_per_run": reports[0].accepted,
            "digest": reports[0].digest,
            "identical": len({report.digest for report in reports}) == 1,
            "wall_time_ms": replay_elapsed_ms,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs", type=int, default=100)
    parser.add_argument("--horizon-ms", type=int, default=100)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.runs <= 0 or args.horizon_ms < 0:
        parser.error("--runs must be positive and --horizon-ms non-negative")

    payload = {
        "scope": "pure_python_deterministic_core_only",
        "results": [
            measure_scenario(scenario, args.runs, args.horizon_ms)
            for scenario in benchmark_scenarios()
        ],
    }
    rendered = json.dumps(payload, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
