#!/usr/bin/env python3
"""Bounded read-only camera capture. Never publishes a target or motion request."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from edgegrasp_interfaces.msg import TrackedTarget
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from edgegrasp.rgbd import ObservationRejected, estimate_cube
from edgegrasp_ros.rgbd_images import decode_image, matrix, stamp_ns


class Capture(Node):
    def __init__(self, output, yaw):
        super().__init__("edgegrasp_rgbd_read_only_capture",
                         parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.output, self.yaw = output, yaw
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pending = {}
        self.count = 0
        self.assets = []
        self.logs = {}
        self.subscriptions_kept = []
        self.truth_subscription = self.create_subscription(
            PoseStamped, "/edgegrasp/target_cube_pose", self.record_truth, 10)
        self.cloud_subscription = self.create_subscription(
            PointCloud2, "/camera_head/depth/color/points", self.record_cloud,
            qos_profile_sensor_data)
        self.cloud_count = 0
        for cls, topic, kind in [
            (TrackedTarget, "/edgegrasp/tracked_target", "tracked"),
            (Bool, "/edgegrasp/motion_allowed", "allowed"),
            (String, "/edgegrasp/safety_status", "safety"),
            (String, "/edgegrasp/perception_status", "perception"),
            (String, "/edgegrasp/permission_evidence", "permission_evidence"),
        ]:
            self.subscriptions_kept.append(self.create_subscription(
                cls, topic, lambda msg, key=kind: self.record_pipeline(key, msg), 10))
        for kind, topic, cls in [
            ("rgb", "/camera_head/color/image_raw", Image),
            ("depth", "/camera_head/depth/image_rect_raw", Image),
            ("info", "/camera_head/depth/camera_info", CameraInfo),
        ]:
            self.subscriptions_kept.append(self.create_subscription(
                cls, topic, lambda msg, key=kind: self.receive(key, msg),
                QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)))

    def record_row(self, name, row):
        self.logs.setdefault(name, []).append(row)

    def record_pipeline(self, kind, message):
        row = {"kind": kind, "wall_ns": time.time_ns(),
               "receive_sim_ns": self.get_clock().now().nanoseconds}
        if kind == "tracked":
            point = message.observation.point
            row.update(source_ns=stamp_ns(message.observation),
                       frame=message.observation.header.frame_id,
                       target_id=message.target_id, clock_domain=message.clock_domain,
                       clock_epoch=message.clock_epoch, center_m=[point.x, point.y, point.z])
        else:
            row["data"] = message.data
        self.record_row("pipeline.jsonl", row)

    def record_cloud(self, message):
        if self.cloud_count >= 10:
            return
        self.cloud_count += 1
        name = f"cloud_{stamp_ns(message)}"
        self.assets.append((self.output / (name + ".bin"), bytes(message.data)))
        (self.output / (name + ".json")).write_text(json.dumps({
            "stamp_ns": stamp_ns(message), "frame": message.header.frame_id,
            "width": message.width, "height": message.height,
            "point_step": message.point_step, "row_step": message.row_step,
            "is_bigendian": message.is_bigendian,
            "fields": [{"name": f.name, "offset": f.offset,
                        "datatype": f.datatype, "count": f.count} for f in message.fields],
        }) + "\n")

    def record_truth(self, message):
        # Evaluation-only sink: never stored in estimator state or input arrays.
        p, q = message.pose.position, message.pose.orientation
        self.record_row("evaluation_truth.jsonl", {"stamp_ns": stamp_ns(message),
                                     "frame": message.header.frame_id,
                                     "center_m": [p.x, p.y, p.z],
                                     "orientation_xyzw": [q.x, q.y, q.z, q.w]})

    def receive(self, kind, msg):
        stamp = stamp_ns(msg)
        self.record_row("received.jsonl", {"kind": kind, "stamp_ns": stamp,
                                           "frame": msg.header.frame_id})
        sample = self.pending.setdefault(stamp, {})
        sample[kind] = msg
        while len(self.pending) > 8:
            del self.pending[min(self.pending)]
        if len(sample) != 3 or self.count >= 10:
            return
        del self.pending[stamp]
        self.count += 1
        record = {"sample": self.count, "source_timestamp_ns": stamp}
        try:
            rgb, depth, info = (sample[key] for key in ("rgb", "depth", "info"))
            if {m.header.frame_id for m in (rgb, depth, info)} != {"camera_head_link"}:
                raise ObservationRejected("unexpected_sensor_frames")
            if any(info.d) or info.width != rgb.width or info.height != rgb.height:
                raise ObservationRejected("unsupported_camera_model")
            # The pinned Gazebo rgbd sensor labels optical data camera_head_link.
            # Use its colocated optical TF explicitly, never the color-frame offset.
            tf = self.buffer.lookup_transform(
                "base_link", "camera_head_depth_optical_frame", Time.from_msg(rgb.header.stamp))
            inputs = dict(rgb=decode_image(rgb), depth_m=decode_image(depth),
                          intrinsics=np.array(info.k).reshape(3, 3),
                          optical_to_planning=matrix(tf.transform))
            now_ns = self.get_clock().now().nanoseconds
            record["now_ns"] = now_ns
            try:
                result = estimate_cube(
                    **inputs, source_timestamp_ns=stamp, depth_timestamp_ns=stamp_ns(depth),
                    info_timestamp_ns=stamp_ns(info), now_ns=now_ns, frame_id="base_link",
                    clock_domain="ros_sim", clock_epoch=0, yaw_rad=self.yaw,
                    pixel_center_offset=0.5)
                record.update(accepted=True, estimate=asdict(result))
            finally:
                self.assets.append((self.output / f"sample_{self.count:02d}.npz", inputs))
        except Exception as exc:
            record.update(accepted=False, reason=f"{type(exc).__name__}: {exc}")
        self.record_row("estimates.jsonl", record)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--yaw-rad", type=float, required=True)
    parser.add_argument("--observe-seconds", type=float, default=0.0)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Capture(args.output, args.yaw_rad)
    if not 0 <= args.observe_seconds <= 40:
        parser.error("observe-seconds must be in [0, 40]")
    deadline = time.monotonic() + (args.observe_seconds or 40)
    try:
        while time.monotonic() < deadline and (args.observe_seconds or node.count < 10):
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        for name, rows in node.logs.items():
            (args.output / name).write_text("".join(json.dumps(row) + "\n" for row in rows))
        for path, data in node.assets:
            if isinstance(data, bytes):
                path.write_bytes(data)
            else:
                np.savez_compressed(path, **data)
        (args.output / "summary.json").write_text(json.dumps({
            "synchronized_samples": node.count, "motion_requests": 0,
            "target_publications": 0,
        }, indent=2) + "\n")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
