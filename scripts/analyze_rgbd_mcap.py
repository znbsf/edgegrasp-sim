#!/usr/bin/env python3
"""Offline localization replay; never republishes recorded targets or commands."""

import argparse
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from edgegrasp.rgbd import ObservationRejected, estimate_cube
from edgegrasp_ros.rgbd_images import decode_image, stamp_ns


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("calibration_npz", type=Path,
                        help="same fixed camera's captured extrinsics; contains no object truth")
    parser.add_argument("output", type=Path)
    parser.add_argument("--max-rotation-deg", type=float, choices=(0.0, 5.0, 20.0, 40.0), default=0.0)
    args = parser.parse_args()
    args.output.mkdir(exist_ok=False)
    with np.load(args.calibration_npz) as calibration:
        transform = calibration["optical_to_planning"].copy()
        intrinsic = calibration["intrinsics"].copy()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {item.name: get_message(item.type) for item in reader.get_all_topics_and_types()}
    topics = {"/camera_head/color/image_raw": "rgb",
              "/camera_head/depth/image_rect_raw": "depth",
              "/camera_head/depth/camera_info": "info"}
    pending, counts, published = {}, Counter(), []
    transition = None
    with (args.output / "offline_estimates.jsonl").open("x") as stream:
        while reader.has_next():
            topic, data, receive = reader.read_next()
            if topic not in topics and topic != "/edgegrasp/tracked_target":
                continue
            message = deserialize_message(data, types[topic])
            if topic == "/edgegrasp/tracked_target":
                published.append(stamp_ns(message.observation))
                continue
            stamp = stamp_ns(message)
            sample = pending.setdefault(stamp, {})
            sample[topics[topic]] = message
            while len(pending) > 100:
                del pending[min(pending)]
                counts["incomplete_recorded_triad"] += 1
            if len(sample) != 3:
                continue
            del pending[stamp]
            rgb, depth, info = (sample[key] for key in ("rgb", "depth", "info"))
            row = {"source_ns": stamp, "bag_receive_ns": receive, "offline_spatial_only": True}
            pixels = decode_image(rgb)
            try:
                if not np.allclose(np.asarray(info.k).reshape(3, 3), intrinsic):
                    raise ObservationRejected("calibration_intrinsics_mismatch")
                result = estimate_cube(
                    pixels, decode_image(depth), intrinsic, transform,
                    source_timestamp_ns=stamp, depth_timestamp_ns=stamp_ns(depth),
                    info_timestamp_ns=stamp_ns(info), now_ns=stamp, frame_id="base_link",
                    clock_domain="ros_sim", clock_epoch=0, yaw_rad=0.6500077341171558,
                    pixel_center_offset=0.5, max_rotation_deg=args.max_rotation_deg)
                row.update(reason="accepted", estimate=asdict(result))
            except ObservationRejected as exc:
                row["reason"] = str(exc)
            counts[row["reason"]] += 1
            if row["reason"] != transition:
                # Standard portable raw image, without optional image libraries.
                (args.output / f"transition_{stamp}.ppm").write_bytes(
                    f"P6\n{rgb.width} {rgb.height}\n255\n".encode() + pixels.tobytes())
                transition = row["reason"]
            stream.write(json.dumps(row) + "\n")
    summary = {"claim": "offline_spatial_replay_with_fixed_camera_calibration",
               "counts": dict(counts), "published_target_count": len(published),
               "last_published_source_ns": max(published) if published else None,
               "unmatched_final_triads": len(pending), "motion_requested": False}
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
