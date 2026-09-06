"""Source identity and refusal tests using ROS messages, without motion nodes."""

import json
import math
from pathlib import Path
import runpy

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.context import Context
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from edgegrasp_ros.rgbd_target_publisher import RgbdTargetPublisher


def test_source_identity_duplicate_stale_frame_and_clock_rollback(monkeypatch):
    fixture = runpy.run_path(str(Path(__file__).resolve().parents[4] / "tests/test_rgbd.py"))
    inputs = fixture["rendered_cube"](pixel_offset=0.5)
    context = Context()
    rclpy.init(context=context, domain_id=167)
    node = RgbdTargetPublisher(context=context, parameter_overrides=[
        Parameter("use_sim_time", value=True), Parameter("yaw_rad", value=0.65),
        Parameter("clock_epoch", value=7)])
    tracked, points, statuses, cubes = [], [], [], []
    monkeypatch.setattr(node.cube_publisher, "publish", lambda msg: cubes.append(json.loads(msg.data)))
    monkeypatch.setattr(node.tracked_publisher, "publish", tracked.append)
    monkeypatch.setattr(node.point_publisher, "publish", points.append)
    monkeypatch.setattr(node.status_publisher, "publish", lambda msg: statuses.append(msg.data))
    transform = TransformStamped()
    transform.header.frame_id = "base_link"
    transform.child_frame_id = "camera_head_depth_optical_frame"
    transform.transform.translation.x = 0.22
    transform.transform.translation.y = -0.29
    transform.transform.translation.z = 0.5
    transform.transform.rotation.x = -math.sin(math.radians(57.5))
    transform.transform.rotation.w = math.cos(math.radians(57.5))
    node.buffer.set_transform_static(transform, "test-calibration")

    def send(stamp, age=50_000_000, frame="camera_head_link"):
        node.get_clock().set_ros_time_override(Time(nanoseconds=stamp + age))
        info = CameraInfo()
        info.width, info.height = 424, 240
        info.k = inputs["intrinsics"].ravel().tolist()
        rgb, depth = Image(), Image()
        for msg, arr, encoding in [
            (rgb, inputs["rgb"], "rgb8"),
            (depth, inputs["depth_m"].astype(np.float32), "32FC1"),
        ]:
            msg.width, msg.height = 424, 240
            msg.encoding, msg.step = encoding, arr.strides[0]
            msg.data = arr.tobytes()
        for kind, msg in [("info", info), ("depth", depth), ("rgb", rgb)]:
            msg.header.stamp = Time(nanoseconds=stamp).to_msg()
            msg.header.frame_id = frame
            node.receive(kind, msg)

    try:
        send(1_000_000_000)
        assert len(tracked) == len(points) == len(cubes) == 1
        assert cubes[0]["source_ns"] == 1_000_000_000 and cubes[0]["clock_epoch"] == 7
        assert np.allclose(cubes[0]["center_m"], [points[0].point.x, points[0].point.y, points[0].point.z])
        assert np.linalg.det(cubes[0]["orientation_rows"]) > .999
        assert tracked[0].clock_epoch == 7
        assert tracked[0].clock_domain == "ros_sim"
        assert tracked[0].observation == points[0]
        assert tracked[0].observation.header.stamp.sec == 1
        send(1_000_000_000)
        assert len(tracked) == 1
        send(2_000_000_000, age=100_000_001)
        assert len(tracked) == 1
        send(3_000_000_000, frame="wrong-frame")
        assert len(tracked) == 1
        send(500_000_000)
        send(4_000_000_000)
        assert node.clock_fault and len(tracked) == 1
        assert any("source_not_fresh" in value for value in statuses)
        assert any("unexpected_sensor_frames" in value for value in statuses)
    finally:
        node.destroy_node()
        context.shutdown()
