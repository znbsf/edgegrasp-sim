"""Command-line smoke checks that need no ROS installation."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json

from .factory import build_controller
from .replay import verify_repeated_replay
from .scenarios import benchmark_scenarios


def _replay_command(runs: int) -> int:
    reports = []
    for scenario in benchmark_scenarios():
        repeated = verify_repeated_replay(scenario, build_controller, runs=runs)
        reports.append(asdict(repeated[0]) | {"runs": runs, "deterministic": True})
    print(json.dumps({"replays": reports}, indent=2, sort_keys=True))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="edgegrasp")
    subparsers = parser.add_subparsers(dest="command", required=True)
    replay = subparsers.add_parser("replay", help="run deterministic scenario replay")
    replay.add_argument("--runs", type=int, default=100)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "replay":
        return _replay_command(args.runs)
    raise AssertionError(f"unhandled command: {args.command}")


if __name__ == "__main__":
    raise SystemExit(main())
