#!/usr/bin/env python3
"""Check that MCAP receive time and target header time share one clock domain.

Run this only in a sourced ROS 2 Jazzy environment.  It is intentionally a
post-recording evidence check, not part of the dependency-free Python core.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TARGET_TOPIC = "/edgegrasp/target_3d"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument(
        "--max-absolute-delta-ms",
        type=float,
        default=1_000.0,
        help="largest allowed abs(receive_timestamp-header.stamp)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if (
        not args.bag.exists()
        or not math.isfinite(args.max_absolute_delta_ms)
        or args.max_absolute_delta_ms < 0.0
    ):
        raise SystemExit("bag must exist and tolerance must be finite/non-negative")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    if TARGET_TOPIC not in topic_types:
        raise SystemExit(f"required topic missing: {TARGET_TOPIC}")
    target_type = get_message(topic_types[TARGET_TOPIC])
    limit_ns = int(args.max_absolute_delta_ms * 1_000_000)
    count = 0
    minimum_delta_ns: int | None = None
    maximum_delta_ns: int | None = None
    violations = 0
    while reader.has_next():
        topic, serialized, receive_timestamp_ns = reader.read_next()
        if topic != TARGET_TOPIC:
            continue
        message = deserialize_message(serialized, target_type)
        header_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        delta_ns = int(receive_timestamp_ns) - header_ns
        minimum_delta_ns = (
            delta_ns if minimum_delta_ns is None else min(minimum_delta_ns, delta_ns)
        )
        maximum_delta_ns = (
            delta_ns if maximum_delta_ns is None else max(maximum_delta_ns, delta_ns)
        )
        count += 1
        violations += abs(delta_ns) > limit_ns

    summary = {
        "bag": str(args.bag.resolve()),
        "topic": TARGET_TOPIC,
        "messages": count,
        "max_absolute_delta_ms": args.max_absolute_delta_ms,
        "minimum_delta_ns": minimum_delta_ns,
        "maximum_delta_ns": maximum_delta_ns,
        "violations": violations,
        "status": "PASS" if count > 0 and violations == 0 else "FAIL",
    }
    print(json.dumps(summary, sort_keys=True))
    return 0 if summary["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
