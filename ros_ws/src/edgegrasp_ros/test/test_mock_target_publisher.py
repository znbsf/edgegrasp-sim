"""ROS-runtime contract for the legacy and correlated target streams."""

from __future__ import annotations

from itertools import count
import time

from edgegrasp_interfaces.msg import TrackedTarget
from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

from edgegrasp_ros.mock_target_publisher import MockTargetPublisher


DOMAINS = count(211)


def test_mock_target_publishes_matching_legacy_and_correlated_observations() -> None:
    context = Context()
    rclpy.init(context=context, domain_id=next(DOMAINS))
    publisher = MockTargetPublisher(
        context=context,
        parameter_overrides=[
            Parameter("use_sim_time", value=False),
            Parameter("clock_domain", value="ros_system"),
            Parameter("clock_epoch", value=7),
            Parameter("target_id", value="cube-fixture"),
            Parameter("publish_rate_hz", value=50.0),
        ],
    )
    observer = Node("tracked_target_test_observer", context=context)
    legacy: list[PointStamped] = []
    tracked: list[TrackedTarget] = []
    legacy_subscription = observer.create_subscription(
        PointStamped,
        "/edgegrasp/target_3d",
        legacy.append,
        10,
    )
    tracked_subscription = observer.create_subscription(
        TrackedTarget,
        "/edgegrasp/tracked_target",
        tracked.append,
        10,
    )
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(publisher)
    executor.add_node(observer)
    try:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and (not legacy or not tracked):
            executor.spin_once(timeout_sec=0.05)
        assert legacy and tracked
        correlated = tracked[-1]
        matching = next(
            (
                item
                for item in reversed(legacy)
                if item.header.stamp == correlated.observation.header.stamp
            ),
            None,
        )
        assert matching is not None
        assert correlated.target_id == "cube-fixture"
        assert correlated.clock_domain == "ros_system"
        assert correlated.clock_epoch == 7
        assert correlated.observation.header.frame_id == "base_link"
        assert correlated.observation.point == matching.point

        first_stamp = correlated.observation.header.stamp
        tracked.clear()
        response = publisher._on_reset(Trigger.Request(), Trigger.Response())
        assert response.success
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not tracked:
            executor.spin_once(timeout_sec=0.05)
        assert tracked
        assert tracked[-1].target_id == "cube-fixture"
        assert tracked[-1].clock_domain == "ros_system"
        assert tracked[-1].clock_epoch == 8
        reset_stamp = tracked[-1].observation.header.stamp
        assert (reset_stamp.sec, reset_stamp.nanosec) >= (
            first_stamp.sec,
            first_stamp.nanosec,
        )
    finally:
        observer.destroy_subscription(legacy_subscription)
        observer.destroy_subscription(tracked_subscription)
        executor.remove_node(observer)
        executor.remove_node(publisher)
        executor.shutdown()
        observer.destroy_node()
        publisher.destroy_node()
        rclpy.shutdown(context=context)
