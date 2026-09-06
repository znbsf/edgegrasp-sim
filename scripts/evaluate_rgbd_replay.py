"""Offline truth comparison; interpolation is reported separately from nearest samples."""

import argparse
import bisect
import json
import math
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("run", type=Path)
parser.add_argument("estimates", type=Path)
parser.add_argument("output", type=Path)
args = parser.parse_args()
p = args.run
r = rosbag2_py.SequentialReader()
r.open(
    rosbag2_py.StorageOptions(uri=str(p / "mcap"), storage_id="mcap"),
    rosbag2_py.ConverterOptions("cdr", "cdr"),
)
truth = {}
while r.has_next():
    topic, data, _ = r.read_next()
    if topic != "/edgegrasp/target_cube_pose":
        continue
    m = deserialize_message(data, PoseStamped)
    stamp = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
    v = m.pose.position
    truth[stamp] = (v.x, v.y, v.z)
stamps = sorted(truth)
out = []
estimates = {}
rejections = []
for line in args.estimates.read_text().splitlines():
    row = json.loads(line)
    n = row["source_ns"]
    i = bisect.bisect_left(stamps, n)
    near = min(stamps[max(0, i - 1) : i + 1], key=lambda k: abs(n - k))
    if row["reason"] == "accepted":
        estimates[n] = row["estimate"]["center_m"]
        error = math.dist(row["estimate"]["center_m"], truth[near])
        out.append(
            {
                "source_ns": n,
                "truth_source_ns": near,
                "error_m": error,
                "passed": abs(n - near) <= 10000000 and error <= 0.00025,
            }
        )
    else:
        rejections.append({"source_ns": n, "reason": row["reason"]})
for row in out:
    n = row["source_ns"]
    i = bisect.bisect_left(stamps, n)
    if n in truth:
        row["interpolated_error_m"] = row["error_m"]
        row["bracketing_truth_ns"] = [n, n]
        continue
    if i == 0 or i == len(stamps):
        continue
    lo, hi = stamps[i - 1], stamps[i]
    if hi - lo > 20000000:
        continue
    estimate = estimates[n]
    a = (n - lo) / (hi - lo)
    interpolated = tuple((1 - a) * x + a * y for x, y in zip(truth[lo], truth[hi]))
    row["interpolated_error_m"] = math.dist(estimate, interpolated)
    row["bracketing_truth_ns"] = [lo, hi]
summary = {
    "claim": "offline_independent_spatial_evaluation_only",
    "samples": len(out),
    "rejected_rows": rejections,
    "passed": sum(x["passed"] for x in out),
    "max_error_m": max(x["error_m"] for x in out),
    "rows": out,
}
summary["interpolation_missing_count"] = sum(
    "interpolated_error_m" not in x for x in out
)
summary["max_interpolated_error_m"] = max(
    (x["interpolated_error_m"] for x in out if "interpolated_error_m" in x),
    default=None,
)
with args.output.open("x") as f:
    json.dump(summary, f, indent=2)
print({k: v for k, v in summary.items() if k not in ("rows", "rejected_rows")})
