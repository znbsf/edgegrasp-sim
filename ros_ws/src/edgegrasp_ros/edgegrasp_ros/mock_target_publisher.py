"""Publish a deterministic linear PointStamped target."""

from __future__ import annotations

import rclpy
from edgegrasp.config import (
    DEFAULT_TARGET_FRAME,
    ROS_SYSTEM_CLOCK_DOMAIN,
    validate_ros_clock_domain,
)
from geometry_msgs.msg import PointStamped
from edgegrasp_interfaces.msg import TrackedTarget
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger


class MockTargetPublisher(Node):
    def __init__(self, **node_kwargs) -> None:
        super().__init__("edgegrasp_mock_target_publisher", **node_kwargs)
        self.declare_parameter("target_topic", "/edgegrasp/target_3d")
        self.declare_parameter("tracked_target_topic", "/edgegrasp/tracked_target")
        self.declare_parameter("target_id", "ros-target")
        self.declare_parameter("frame_id", DEFAULT_TARGET_FRAME)
        self.declare_parameter("clock_domain", ROS_SYSTEM_CLOCK_DOMAIN)
        self.declare_parameter("clock_epoch", 0)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("start_x_m", 0.15)
        self.declare_parameter("start_y_m", -0.10)
        self.declare_parameter("start_z_m", 0.08)
        self.declare_parameter("velocity_x_mps", 0.0)

        clock_domain = str(self.get_parameter("clock_domain").value)
        target_id = str(self.get_parameter("target_id").value)
        self._clock_epoch = int(self.get_parameter("clock_epoch").value)
        validate_ros_clock_domain(
            bool(self.get_parameter("use_sim_time").value), clock_domain
        )
        if (
            self._clock_epoch < 0
            or not target_id
            or target_id.strip() != target_id
            or len(target_id) > 128
        ):
            raise ValueError("clock_epoch and target_id must be valid")
        self._clock_domain = clock_domain
        self._target_id = target_id

        topic = self.get_parameter("target_topic").value
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        self._publisher = self.create_publisher(PointStamped, topic, 10)
        self._tracked_publisher = self.create_publisher(
            TrackedTarget,
            str(self.get_parameter("tracked_target_topic").value),
            10,
        )
        self._stream_enabled = True
        # Keep application callbacks mutually exclusive while leaving the
        # node's default callback group free for rclpy's internal /clock
        # subscription.  A single-threaded executor can otherwise let the
        # simulated clock observed by peer nodes drift by queued callbacks.
        self._application_group = MutuallyExclusiveCallbackGroup()
        self._start_ns = self.get_clock().now().nanoseconds
        self._last_clock_ns = self._start_ns
        self._timer = self.create_timer(
            1.0 / rate_hz,
            self._publish,
            callback_group=self._application_group,
        )
        self.create_service(
            Trigger,
            "/edgegrasp/reset_mock_target_epoch",
            self._on_reset,
            callback_group=self._application_group,
        )
        self.create_service(
            SetBool,
            "/edgegrasp/set_mock_target_stream",
            self._on_set_stream,
            callback_group=self._application_group,
        )

    def _publish(self) -> None:
        if not self._stream_enabled:
            return
        now = self.get_clock().now()
        if now.nanoseconds < self._last_clock_ns:
            self.get_logger().error(
                "ROS clock rolled back; publisher epoch must be reset with the pipeline"
            )
            self._timer.cancel()
            return
        self._last_clock_ns = now.nanoseconds
        elapsed_s = (now.nanoseconds - self._start_ns) / 1_000_000_000.0
        message = PointStamped()
        message.header.stamp = now.to_msg()
        message.header.frame_id = str(self.get_parameter("frame_id").value)
        message.point.x = float(self.get_parameter("start_x_m").value) + float(
            self.get_parameter("velocity_x_mps").value
        ) * elapsed_s
        message.point.y = float(self.get_parameter("start_y_m").value)
        message.point.z = float(self.get_parameter("start_z_m").value)
        self._publisher.publish(message)
        tracked = TrackedTarget()
        tracked.target_id = self._target_id
        tracked.observation = message
        tracked.clock_domain = self._clock_domain
        tracked.clock_epoch = self._clock_epoch
        self._tracked_publisher.publish(tracked)

    def _on_reset(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        now_ns = self.get_clock().now().nanoseconds
        self._clock_epoch += 1
        self._start_ns = now_ns
        self._last_clock_ns = now_ns
        self._timer.reset()
        response.success = True
        response.message = f"mock publisher reset to local clock epoch {self._clock_epoch}"
        return response

    def _on_set_stream(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        self._stream_enabled = bool(request.data)
        response.success = True
        response.message = (
            "mock target stream enabled"
            if self._stream_enabled
            else "mock target stream paused"
        )
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MockTargetPublisher()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
