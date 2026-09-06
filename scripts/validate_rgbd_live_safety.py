#!/usr/bin/env python3
"""Validate recorded live perception/permission behavior; no motion interfaces."""

import argparse
import hashlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    args = parser.parse_args()
    folder = args.capture
    path = folder / "camera/pipeline.jsonl"
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    targets = [row for row in rows if row["kind"] == "tracked"]
    permissions = [row for row in rows if row["kind"] == "allowed"]
    if not targets or not any(row["data"] for row in permissions):
        raise ValueError("no observed live target and permission")
    stamps = [row["source_ns"] for row in targets]
    last = stamps[-1]
    loss = json.loads((folder / "input_loss.json").read_text())
    after_loss = [row for row in permissions if row["wall_ns"] > loss["wall_ns"]]
    late = [row for row in after_loss if row["receive_sim_ns"] > last + 200_000_000]
    images = [json.loads(line) for line in (folder / "camera/received.jsonl").read_text().splitlines()]
    input_stamps = {kind: {row["stamp_ns"] for row in images if row["kind"] == kind}
                    for kind in ("rgb", "depth", "info")}
    checks = {
        "strictly_increasing_source": all(a < b for a, b in zip(stamps, stamps[1:])),
        "source_preserved_from_three_inputs": all(all(stamp in s for s in input_stamps.values()) for stamp in stamps),
        "identity_preserved": all(row["target_id"] == "target_cube" and row["frame"] == "base_link"
                                  and row["clock_domain"] == "ros_sim" and row["clock_epoch"] == 0
                                  for row in targets),
        "receiver_observed_fresh_targets": all(0 <= row["receive_sim_ns"] - row["source_ns"] <= 100_000_000 for row in targets),
        "permission_denied_after_source_expiry": bool(late) and all(not row["data"] for row in late),
        "cleaned_scoped_processes": not json.loads((folder / "cleanup.json").read_text())["remaining_pids"],
    }
    report = {"checks": checks, "passed": all(checks.values()),
              "pipeline_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
              "observed_targets": len(targets), "claim": "live_perception_and_permission_only",
              "motion_requested": False, "physics_grasp_verified": False}
    with (folder / "live_safety_validation.json").open("x") as stream:
        stream.write(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
