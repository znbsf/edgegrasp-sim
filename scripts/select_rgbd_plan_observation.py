#!/usr/bin/env python3
"""Select the first qualified LIVE static estimate for spatial plan-only reuse."""

import argparse
import hashlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    args = parser.parse_args()
    p = args.capture
    def read(name):
        return json.loads((p / name).read_text())
    if not read("live_safety_validation.json")["passed"] or not read("permission_timing_validation.json")["passed"]:
        raise ValueError("live qualification failed")
    evaluation = read("spatial_evaluation.json")
    samples = evaluation["samples"]
    if not evaluation["all_samples_retained"] or not all(
            row["spatial_gate_passed"] for row in samples if row["live_accepted"]):
        raise ValueError("accepted static estimate failed spatial qualification")
    selected = next(row for row in samples if row["live_accepted"] and row["spatial_gate_passed"])
    live = [json.loads(line) for line in (p / "camera/estimates.jsonl").read_text().splitlines()]
    estimate = next(row["estimate"] for row in live if row["sample"] == selected["sample"] and row["accepted"])
    if estimate["center_m"] != selected["estimate"]["center_m"]:
        raise ValueError("live and evaluated center differ")
    scope = read("scope.json")
    view = next(argument.split(":=", 1)[1] for argument in scope["command"] if argument.startswith("camera_view:="))
    estimate.update(camera_view=view, capture_scope=scope,
        qualification_sha256={name: hashlib.sha256((p / name).read_bytes()).hexdigest()
            for name in ("live_safety_validation.json", "permission_timing_validation.json", "spatial_evaluation.json")})
    with (p / "first_accepted_estimate.json").open("x") as stream:
        json.dump(estimate, stream, indent=2)
    print(json.dumps({"source_timestamp_ns": estimate["source_timestamp_ns"],
                      "center_m": estimate["center_m"], "claim": "recorded_spatial_plan_only"}))


if __name__ == "__main__":
    main()
