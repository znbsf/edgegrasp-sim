#!/usr/bin/env python3
"""Extract recorded atomic observations and audit exact source provenance; no replay publication."""

import argparse
import json
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from edgegrasp.target_motion import MeasuredCubeObservation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("output", type=Path, help="new directory")
    args = parser.parse_args()
    args.output.mkdir(exist_ok=False)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.run / "mcap"), storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    sources = {topic: set() for topic in ("/camera_head/color/image_raw",
        "/camera_head/depth/image_rect_raw", "/camera_head/depth/camera_info")}
    observations, tracked = [], {}
    while reader.has_next():
        topic, data, receive = reader.read_next()
        if topic not in sources and topic not in ("/edgegrasp/measured_cube", "/edgegrasp/tracked_target"):
            continue
        message = deserialize_message(data, types[topic])
        if topic in sources:
            stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            sources[topic].add(stamp)
        elif topic == "/edgegrasp/measured_cube":
            value = json.loads(message.data)
            MeasuredCubeObservation.parse(value)
            observations.append(value | {"bag_receive_ns": receive})
        else:
            stamp = message.observation.header.stamp.sec * 10**9 + message.observation.header.stamp.nanosec
            v = message.observation.point
            tracked[(message.target_id, stamp, message.clock_domain, message.clock_epoch)] = [v.x, v.y, v.z]
    logs = [json.loads(line) for line in (args.run / "trial.log").read_text().splitlines() if line.startswith("{")]
    result = next(row for row in logs if row.get("type") == "grasp_trial_result")
    start = result["target_source_timestamp_ns"]
    end = result["simultaneous_contact_timing"]["last_source_timestamp_ns"]
    rows = []
    for value in observations:
        n = value["source_ns"]
        key = value["target_id"], n, value["clock_domain"], value["clock_epoch"]
        rows.append(dict(source_ns=n, active_interval=start <= n <= end,
            three_recorded_inputs=all(n in stamps for stamps in sources.values()),
            tracked_center_matches=tracked.get(key) == value["center_m"]))
    active = [row for row in rows if row["active_interval"]]
    report = dict(claim="recorded_atomic_source_provenance_only", total=len(rows), active=len(active),
        active_three_inputs=bool(active) and all(row["three_recorded_inputs"] for row in active),
        active_tracked_centers_match=bool(active) and all(row["tracked_center_matches"] for row in active),
        sources_strictly_increasing=all(b["source_ns"] > a["source_ns"] for a, b in zip(rows, rows[1:])),
        active_interval_ns=[start, end], rows=rows)
    with (args.output / "estimates.jsonl").open("x") as stream:
        for value in observations:
            stream.write(json.dumps(dict(source_ns=value["source_ns"], reason="accepted", estimate=value)) + "\n")
    (args.output / "provenance.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k != "rows"}))


if __name__ == "__main__":
    main()
