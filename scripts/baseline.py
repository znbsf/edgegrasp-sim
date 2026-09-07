#!/usr/bin/env python3
"""Unified offline baseline check, command preparation and existing-evidence report."""

import argparse
import csv
import json
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))

from edgegrasp.baseline import (  # noqa: E402
    inspect_baseline, raw_path, read_json, render_command,
)
from edgegrasp.baseline_results import build_report, markdown_report  # noqa: E402


def new_output(value):
    path = Path(value).resolve()
    if path.is_relative_to(REPO) or path.exists():
        raise ValueError("output must be a NEW directory outside the repository")
    path.mkdir(parents=True, exist_ok=False)
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", nargs="?", default="check", choices=("check", "command", "summarize"))
    parser.add_argument("--case", default="control", choices=("control", "x_minus_1mm", "x_plus_1mm"))
    parser.add_argument("--evidence-root", type=Path, help="Existing test_results root, Linux path or Windows UNC")
    parser.add_argument("--runtime-root", type=Path, help="Filesystem root of existing Ubuntu installation; normally / inside WSL")
    parser.add_argument("--require-local-runtime", action="store_true", help="Fail unless pinned local dependency files and AJ plan match")
    parser.add_argument("--output", help="New external result directory")
    parser.add_argument("--domain", type=int)
    parser.add_argument("--artifact-dir", help="New Linux experiment directory; command generation only")
    parser.add_argument("--task-id")
    parser.add_argument("--linux-repo", help="Absolute Linux path of this checkout; command generation only")
    args = parser.parse_args(argv)
    try:
        config = read_json(REPO / "config/release_cycle_baseline.json")
        lock = read_json(REPO / "config/release_cycle_baseline.lock.json")
        if args.action == "summarize":
            if args.evidence_root is None or args.output is None:
                raise ValueError("summarize requires --evidence-root and --output")
            report = build_report(REPO, args.evidence_root)
            output = new_output(args.output)
            (output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
            (output / "report.md").write_text(markdown_report(report), encoding="utf-8")
            fields = ("id", "cohort", "candidate", "cycle", "position", "state", "attempt_kind", "grasp_success",
                      "full_cycle_success", "failure_stage", "failure_reason", "lift_m", "source_directory")
            with (output / "rounds.csv").open("w", encoding="utf-8-sig", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
                writer.writeheader()
                writer.writerows(report["rows"])
            print(json.dumps({"output": str(output), "integrity_ok": report["integrity_ok"], "summary": report["summary"]}, indent=2))
            return 0 if report["integrity_ok"] else 2
        result = inspect_baseline(REPO, config, lock, args.evidence_root, args.runtime_root)
        selected = next(c for c in config["cases"] if c["id"] == args.case)
        result["selected_case"] = selected
        result["selected_case_status"] = "CONFIGURED" if selected["ready"] else "NOT_READY"
        command = None
        if args.action == "command":
            if any(value is None for value in (args.domain, args.artifact_dir, args.task_id, args.linux_repo, args.output)):
                raise ValueError("command requires --domain, --artifact-dir, --task-id, --linux-repo and --output")
            if args.evidence_root is not None and raw_path(args.evidence_root, args.artifact_dir).exists():
                raise ValueError("experiment output already exists; historical evidence is read-only")
            command = render_command(config, result, case_id=args.case, domain=args.domain,
                                     output=args.artifact_dir, task_id=args.task_id, linux_repo=args.linux_repo)
        if args.output:
            output = new_output(args.output)
            (output / "check.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
            if command:
                (output / "reviewed-command.sh").write_text(command, encoding="utf-8", newline="\n")
            print(json.dumps({"output": str(output), "command_generated": command is not None, "motion_started": False}))
        else:
            print(json.dumps(result, indent=2))
        ok = result["repository_ok"] and selected["ready"]
        if args.evidence_root is not None:
            ok = ok and result["raw_plan"]["status"] == "verified"
        if args.require_local_runtime:
            ok = ok and result["command_preparation_ready"]
        return 0 if ok else 2
    except (ValueError, OSError, KeyError, TypeError) as exc:
        print(f"NOT_READY: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
