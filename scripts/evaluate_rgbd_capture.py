#!/usr/bin/env python3
"""Offline spatial evaluation; never admits stale recorded data to execution."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path

import numpy as np

from edgegrasp.rgbd import ObservationRejected, estimate_cube


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--yaw-rad", type=float, required=True)
    args = parser.parse_args()
    source = args.capture
    truth = [json.loads(line) for line in (source / "evaluation_truth.jsonl").read_text().splitlines()]
    rows = [json.loads(line) for line in (source / "estimates.jsonl").read_text().splitlines()]
    results = []
    for row in rows:
        result = {"sample": row["sample"], "live_accepted": row["accepted"],
                  "live_reason": row.get("reason"), "offline_spatial_only": True}
        stamp = row["source_timestamp_ns"]
        try:
            with np.load(source / f"sample_{row['sample']:02d}.npz") as data:
                estimate = estimate_cube(
                    **dict(data), source_timestamp_ns=stamp, depth_timestamp_ns=stamp,
                    info_timestamp_ns=stamp, now_ns=stamp, frame_id="base_link",
                    clock_domain="ros_sim", clock_epoch=0, yaw_rad=args.yaw_rad,
                    pixel_center_offset=0.5)
            result["estimate"] = asdict(estimate)
            nearest = min(truth, key=lambda item: abs(item["stamp_ns"] - stamp))
            if abs(nearest["stamp_ns"] - stamp) > 10_000_000:
                raise ObservationRejected("evaluation_truth_not_correlated")
            result["truth"] = nearest
            error = np.asarray(estimate.center_m) - nearest["center_m"]
            result["error_xyz_m"] = error.tolist()
            result["error_norm_m"] = float(np.linalg.norm(error))
            # Strict 0.25 mm spatial budget reserves half the narrowest 0.5 mm
            # preclose-clearance margin. This is evaluation, not a runtime bound.
            result["spatial_gate_passed"] = result["error_norm_m"] <= 0.00025
        except (ObservationRejected, FileNotFoundError) as exc:
            result.update(spatial_gate_passed=False, reason=str(exc))
        results.append(result)
    payload = {"claim": "offline_spatial_evaluation_only", "samples": results,
               "all_samples_retained": len(results) == len(rows),
               "physics_grasp_verified": False}
    with args.output.open("x") as stream:
        stream.write(json.dumps(payload, indent=2) + "\n")
    print(json.dumps({"samples": len(results), "spatial_passes": sum(
        r["spatial_gate_passed"] for r in results)}))


if __name__ == "__main__":
    main()
