#!/usr/bin/env python3
"""Offline measured-cube/fixed-pad geometry; never contact or grasp proof."""

import argparse
import json
from pathlib import Path

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from tf2_ros import Buffer, TransformException

from edgegrasp.grasp_geometry import (
    OrientedBoxEnvelope, load_grasp_geometry_profile,
    oriented_box_overlap_margins, transform_point_from_frame,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("estimates", type=Path)
    parser.add_argument("profile", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--from-ns", type=int, required=True)
    parser.add_argument("--to-ns", type=int, required=True)
    args = parser.parse_args()
    profile = load_grasp_geometry_profile(args.profile)
    if profile.preclose_contact_policy is None:
        raise ValueError("fixed-pad identity missing")
    pad = next(box for box in profile.required_contact_pad_obbs
               if box.name == profile.preclose_contact_policy.fixed_pad_name)
    rclpy.init()
    try:
        buffer = Buffer(cache_time=Duration(seconds=1000))
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="mcap"),
                    rosbag2_py.ConverterOptions("cdr", "cdr"))
        types = {topic.name: get_message(topic.type) for topic in reader.get_all_topics_and_types()}
        while reader.has_next():
            topic, data, _ = reader.read_next()
            if topic not in ("/tf", "/tf_static"):
                continue
            for transform in deserialize_message(data, types[topic]).transforms:
                setter = buffer.set_transform_static if topic == "/tf_static" else buffer.set_transform
                setter(transform, "recorded")
        rows, missing = [], []
        for line in args.estimates.read_text().splitlines():
            item = json.loads(line)
            stamp = item["source_ns"]
            if item["reason"] != "accepted" or not args.from_ns <= stamp <= args.to_ns:
                continue
            estimate = item["estimate"]
            try:
                tf = buffer.lookup_transform(estimate["frame_id"], profile.end_effector_frame,
                                             Time(nanoseconds=stamp))
                actual = tf.header.stamp.sec * 10**9 + tf.header.stamp.nanosec
                if actual != stamp:
                    raise ValueError("TF source mismatch")
            except (TransformException, ValueError) as exc:
                missing.append({"source_ns": stamp, "reason": str(exc)})
                continue
            p, q = tf.transform.translation, tf.transform.rotation
            rotation = np.column_stack([
                transform_point_from_frame((0., 0., 0.), (q.x, q.y, q.z, q.w), axis)
                for axis in np.eye(3)])
            center = rotation.T @ (np.asarray(estimate["center_m"]) - [p.x, p.y, p.z])
            axes = rotation.T @ np.asarray(estimate["orientation_rows"])
            cube = OrientedBoxEnvelope("measured_cube", tuple(center), profile.cube_size_m,
                                       tuple(tuple(row) for row in axes))
            margins = oriented_box_overlap_margins(cube, pad)
            rows.append({"source_ns": stamp, "fixed_pad_min_sat_overlap_m": min(margins),
                         "cube_center_in_gripper_m": list(center),
                         "radial_distance_to_gripper_origin_m": float(np.linalg.norm(center))})
        out = {"claim": "offline_visual_cube_and_recorded_TF_fixed_pad_geometry_only",
               "physics_grasp_verified": False, "rows": rows, "missing_TF": missing,
               "minimum_fixed_pad_sat_overlap_m": min(
                   (row["fixed_pad_min_sat_overlap_m"] for row in rows), default=None)}
        with args.output.open("x") as stream:
            json.dump(out, stream, indent=2)
        print(json.dumps({key: value for key, value in out.items() if key != "rows"}))
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
