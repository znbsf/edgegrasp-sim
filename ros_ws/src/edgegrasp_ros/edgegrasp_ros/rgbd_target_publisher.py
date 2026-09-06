"""Publish source-stamped fixed-yaw cube observations, with no truth subscription."""

import json
import math

import numpy as np
import rclpy
from edgegrasp_interfaces.msg import TrackedTarget
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from edgegrasp.config import validate_ros_clock_domain
from edgegrasp.rgbd import ObservationRejected, estimate_cube
from edgegrasp_ros.rgbd_images import decode_image, matrix, stamp_ns


class RgbdTargetPublisher(Node):
    def __init__(self, **kwargs):
        super().__init__("edgegrasp_rgbd_target_publisher", **kwargs)
        self.declare_parameter("clock_domain", "ros_sim")
        self.declare_parameter("clock_epoch", 0)
        self.declare_parameter("target_id", "target_cube")
        self.declare_parameter("yaw_rad", float("nan"))
        self.declare_parameter("max_rotation_deg", 0.0)
        self.max_rotation_deg = float(self.get_parameter("max_rotation_deg").value)
        if self.max_rotation_deg not in (0.0, 5.0, 20.0, 40.0):
            raise ValueError("rotation bound must be 0, 5, 20 or 40 degrees")
        self.domain = str(self.get_parameter("clock_domain").value)
        self.epoch = int(self.get_parameter("clock_epoch").value)
        self.target_id = str(self.get_parameter("target_id").value)
        self.yaw = float(self.get_parameter("yaw_rad").value)
        validate_ros_clock_domain(bool(self.get_parameter("use_sim_time").value), self.domain)
        if (self.domain != "ros_sim" or self.epoch < 0 or not math.isfinite(self.yaw)
                or not self.target_id or self.target_id.strip() != self.target_id):
            raise ValueError("declare static cube yaw and valid simulation identity")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pending = {}
        self.last_stamp = 0
        self.last_clock = 0
        self.clock_fault = False
        self.point_publisher = self.create_publisher(PointStamped, "/edgegrasp/target_3d", 10)
        self.tracked_publisher = self.create_publisher(TrackedTarget, "/edgegrasp/tracked_target", 10)
        self.cube_publisher = self.create_publisher(String, "/edgegrasp/measured_cube", 10)
        self.status_publisher = self.create_publisher(String, "/edgegrasp/perception_status", 10)
        self.input_subscriptions = []
        for key, topic, message_type in [
            ("rgb", "/camera_head/color/image_raw", Image),
            ("depth", "/camera_head/depth/image_rect_raw", Image),
            ("info", "/camera_head/depth/camera_info", CameraInfo),
        ]:
            self.input_subscriptions.append(self.create_subscription(
                message_type, topic, lambda msg, kind=key: self.receive(kind, msg),
                qos_profile_sensor_data))
        self.clock_guard = self.create_timer(0.02, self.check_clock)

    def status(self, reason):
        msg = String()
        msg.data = reason
        self.status_publisher.publish(msg)

    def check_clock(self):
        now = self.get_clock().now().nanoseconds
        if now < self.last_clock:
            self.clock_fault = True
            self.pending.clear()
            self.status("rejected:clock_rollback_restart_with_new_pipeline_epoch")
        self.last_clock = now
        return now

    def receive(self, kind, message):
        now = self.check_clock()
        if self.clock_fault:
            return
        stamp = stamp_ns(message)
        if stamp <= self.last_stamp or stamp <= 0:
            self.status("rejected:nonincreasing_source")
            return
        sample = self.pending.setdefault(stamp, {})
        sample[kind] = message
        while len(self.pending) > 8:
            del self.pending[min(self.pending)]
        if len(sample) != 3:
            return
        del self.pending[stamp]
        self.last_stamp = stamp
        try:
            rgb, depth, info = (sample[key] for key in ("rgb", "depth", "info"))
            if {msg.header.frame_id for msg in (rgb, depth, info)} != {"camera_head_link"}:
                raise ObservationRejected("unexpected_sensor_frames")
            if (any(info.d) or info.width != rgb.width or info.height != rgb.height
                    or info.binning_x not in (0, 1) or info.binning_y not in (0, 1)
                    or info.roi.x_offset or info.roi.y_offset):
                raise ObservationRejected("unsupported_camera_model")
            transform = self.buffer.lookup_transform(
                "base_link", "camera_head_depth_optical_frame", Time.from_msg(rgb.header.stamp))
            result = estimate_cube(
                decode_image(rgb), decode_image(depth), np.asarray(info.k).reshape(3, 3),
                matrix(transform.transform), source_timestamp_ns=stamp,
                depth_timestamp_ns=stamp_ns(depth), info_timestamp_ns=stamp_ns(info),
                now_ns=now, frame_id="base_link", clock_domain=self.domain,
                clock_epoch=self.epoch, yaw_rad=self.yaw, pixel_center_offset=0.5, max_rotation_deg=self.max_rotation_deg)
            # Processing itself consumes the admission budget. Never restamp.
            if not 0 <= self.check_clock() - stamp <= 100_000_000 or self.clock_fault:
                raise ObservationRejected("expired_during_localization")
            point = PointStamped()
            point.header.stamp = rgb.header.stamp
            point.header.frame_id = result.frame_id
            point.point.x, point.point.y, point.point.z = result.center_m
            tracked = TrackedTarget()
            tracked.target_id, tracked.observation = self.target_id, point
            tracked.clock_domain, tracked.clock_epoch = self.domain, self.epoch
            cube = String()
            cube.data = json.dumps(dict(schema_version=1, target_id=self.target_id,
                frame_id=result.frame_id, source_ns=stamp, clock_domain=self.domain,
                clock_epoch=self.epoch, center_m=result.center_m,
                orientation_rows=result.orientation_rows), allow_nan=False)
            self.cube_publisher.publish(cube)
            self.tracked_publisher.publish(tracked)
            self.point_publisher.publish(point)
            self.status(f"accepted:{stamp}")
        except (ObservationRejected, TransformException, ValueError) as exc:
            self.status(f"rejected:{exc}")


def main(args=None):
    rclpy.init(args=args)
    node = RgbdTargetPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
