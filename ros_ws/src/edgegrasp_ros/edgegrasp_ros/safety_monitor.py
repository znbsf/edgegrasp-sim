"""ROS-clock source freshness and target-stream watchdog monitor.

This node publishes a permission signal; the separate trajectory_gate node is
the motion-command boundary that consumes it. PointStamped cannot carry the
EdgeGrasp clock domain/epoch, so this wrapper assigns its configured local
domain and epoch and latches on ROS clock rollback.
"""

from __future__ import annotations

import rclpy
from edgegrasp.config import (
    DEFAULT_ROS_FUTURE_SKEW_TOLERANCE_MS,
    DEFAULT_STALE_AFTER_MS,
    DEFAULT_TARGET_FRAME,
    DEFAULT_WATCHDOG_TIMEOUT_MS,
    ROS_SYSTEM_CLOCK_DOMAIN,
    validate_ros_clock_domain,
)
from edgegrasp.models import Target3D, Vector3
from edgegrasp.safety import SafetyReason, StaleTargetGate
from geometry_msgs.msg import PointStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class SafetyMonitor(Node):
    def __init__(self, **node_kwargs) -> None:
        super().__init__("edgegrasp_safety_monitor", **node_kwargs)
        self.declare_parameter("target_topic", "/edgegrasp/target_3d")
        self.declare_parameter("target_frame", DEFAULT_TARGET_FRAME)
        self.declare_parameter("clock_domain", ROS_SYSTEM_CLOCK_DOMAIN)
        self.declare_parameter("clock_epoch", 0)
        self.declare_parameter("stale_after_ms", DEFAULT_STALE_AFTER_MS)
        self.declare_parameter("watchdog_timeout_ms", DEFAULT_WATCHDOG_TIMEOUT_MS)
        self.declare_parameter(
            "future_skew_tolerance_ms", DEFAULT_ROS_FUTURE_SKEW_TOLERANCE_MS
        )
        self.declare_parameter("check_rate_hz", 50.0)

        self._target_frame = str(self.get_parameter("target_frame").value)
        self._clock_domain = str(self.get_parameter("clock_domain").value)
        self._clock_epoch = int(self.get_parameter("clock_epoch").value)
        validate_ros_clock_domain(
            bool(self.get_parameter("use_sim_time").value), self._clock_domain
        )
        if not self._target_frame or not self._clock_domain or self._clock_epoch < 0:
            raise ValueError("target_frame, clock_domain, and clock_epoch must be valid")
        self._gate = StaleTargetGate(float(self.get_parameter("stale_after_ms").value))
        self._watchdog_timeout_ns = int(
            float(self.get_parameter("watchdog_timeout_ms").value) * 1_000_000
        )
        self._future_skew_tolerance_ns = int(
            float(self.get_parameter("future_skew_tolerance_ms").value) * 1_000_000
        )
        if self._future_skew_tolerance_ns <= 0:
            raise ValueError("future_skew_tolerance_ms must be positive")
        self._latest: Target3D | None = None
        self._last_receive_ns: int | None = None
        self._pending: Target3D | None = None
        self._pending_receive_ns: int | None = None
        self._last_source_timestamp_ns: int | None = None
        self._last_clock_ns: int | None = None
        self._latched_reason: str | None = None
        # Target/timer/reset state stays serialized in this group, while the
        # default group remains available to the node's internal /clock
        # subscription on the second executor thread.
        self._application_group = MutuallyExclusiveCallbackGroup()
        self._allowed = self.create_publisher(Bool, "/edgegrasp/motion_allowed", 10)
        self._status = self.create_publisher(String, "/edgegrasp/safety_status", 10)
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("target_topic").value),
            self._on_target,
            10,
            callback_group=self._application_group,
        )
        self.create_service(
            Trigger,
            "/edgegrasp/reset_safety_epoch",
            self._on_reset,
            callback_group=self._application_group,
        )
        rate_hz = float(self.get_parameter("check_rate_hz").value)
        if rate_hz <= 0.0:
            raise ValueError("check_rate_hz must be positive")
        self._timer = self.create_timer(
            1.0 / rate_hz,
            self._evaluate,
            callback_group=self._application_group,
        )

    def _observe_clock(self, now_ns: int) -> bool:
        if self._last_clock_ns is not None and now_ns < self._last_clock_ns:
            self._latched_reason = f"clock_rollback:{now_ns}<{self._last_clock_ns}"
            return False
        self._last_clock_ns = now_ns
        return True

    def _on_target(self, message: PointStamped) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if not self._observe_clock(now_ns):
            self._latch(self._latched_reason or "clock_rollback")
            return
        if self._latched_reason is not None:
            self._publish(False, self._latched_reason)
            return
        frame_id = message.header.frame_id or "<empty>"
        if frame_id != self._target_frame:
            self._latch(f"frame_mismatch:{frame_id}!={self._target_frame}")
            return

        timestamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if (
            self._last_source_timestamp_ns is not None
            and timestamp_ns <= self._last_source_timestamp_ns
        ):
            self._latch(
                "source_timestamp_not_monotonic:"
                f"{timestamp_ns}<={self._last_source_timestamp_ns}"
            )
            return
        try:
            candidate = Target3D(
                target_id="ros-target",
                position_m=Vector3(message.point.x, message.point.y, message.point.z),
                timestamp_ns=timestamp_ns,
                frame_id=frame_id,
                clock_domain=self._clock_domain,
                clock_epoch=self._clock_epoch,
            )
        except Exception as error:
            self._latch(f"invalid_target:{type(error).__name__}")
            return
        decision = self._gate.evaluate(candidate, now_ns)
        if not decision.allowed:
            if decision.reason is SafetyReason.FUTURE_TARGET:
                # Different ROS nodes can observe a new /clock sample in a
                # different callback order.  Keep motion denied until this
                # monitor's clock catches up; never reinterpret the negative
                # age as fresh.  Larger future offsets remain latched faults.
                self._pending = candidate
                self._pending_receive_ns = now_ns
                self._last_source_timestamp_ns = timestamp_ns
                self._publish_pending_decision(now_ns, -decision.age_ns)
                return
            if decision.reason is SafetyReason.STALE_TARGET:
                self._publish(False, decision.reason.value)
                return
            self._latch(decision.reason.value)
            return
        self._latest = candidate
        self._last_receive_ns = now_ns
        self._pending = None
        self._pending_receive_ns = None
        self._last_source_timestamp_ns = timestamp_ns

    def _evaluate(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if not self._observe_clock(now_ns):
            self._publish(False, self._latched_reason or "clock_rollback")
            return
        if self._latched_reason is not None:
            self._publish(False, self._latched_reason)
            return
        if self._pending is not None and self._pending_receive_ns is not None:
            pending_decision = self._gate.evaluate(self._pending, now_ns)
            if pending_decision.allowed:
                self._latest = self._pending
                self._last_receive_ns = self._pending_receive_ns
                self._pending = None
                self._pending_receive_ns = None
            elif pending_decision.reason is SafetyReason.FUTURE_TARGET:
                self._publish_pending_decision(now_ns, -pending_decision.age_ns)
                return
            elif pending_decision.reason is SafetyReason.STALE_TARGET:
                self._pending = None
                self._pending_receive_ns = None
            else:
                self._latched_reason = pending_decision.reason.value
                self._publish(False, self._latched_reason)
                return
        if self._latest is None or self._last_receive_ns is None:
            self._publish(False, "no_fresh_target")
            return

        source_decision = self._gate.evaluate(self._latest, now_ns)
        if not source_decision.allowed:
            if source_decision.reason is SafetyReason.STALE_TARGET:
                self._publish(False, source_decision.reason.value)
                return
            self._latched_reason = source_decision.reason.value
            self._publish(False, self._latched_reason)
            return
        receive_age_ns = now_ns - self._last_receive_ns
        if receive_age_ns > self._watchdog_timeout_ns:
            self._publish(False, "target_stream_timeout")
            return
        self._publish(True, "allowed")

    def _publish_pending_decision(self, now_ns: int, future_ns: int) -> None:
        """Keep only a still-fresh prior target live while ROS clock catches up.

        The pending sample is never accepted early.  Different ROS processes can
        observe a `/clock` update in adjacent callback cycles, so a bounded future
        sample must not create a one-message permission glitch when the previously
        accepted target is still fresh in both the source and receive-time domains.
        """

        if future_ns > self._future_skew_tolerance_ns:
            self._latch(
                "future_target_skew_exceeds_tolerance:"
                f"{future_ns}>{self._future_skew_tolerance_ns}"
            )
            return
        if self._latest is not None and self._last_receive_ns is not None:
            prior_decision = self._gate.evaluate(self._latest, now_ns)
            receive_age_ns = now_ns - self._last_receive_ns
            if (
                prior_decision.allowed
                and 0 <= receive_age_ns <= self._watchdog_timeout_ns
            ):
                self._publish(
                    True, f"allowed_previous_target_pending:{future_ns}ns"
                )
                return
        self._publish(False, f"future_target_pending:{future_ns}ns")

    def _publish(self, allowed: bool, reason: str) -> None:
        allowed_message = Bool()
        allowed_message.data = allowed
        status_message = String()
        status_message.data = (
            f"{reason};clock_domain={self._clock_domain};clock_epoch={self._clock_epoch}"
        )
        self._allowed.publish(allowed_message)
        self._status.publish(status_message)

    def _latch(self, reason: str) -> None:
        self._latched_reason = reason
        self._latest = None
        self._pending = None
        self._pending_receive_ns = None
        self._publish(False, reason)

    def _on_reset(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        now_ns = self.get_clock().now().nanoseconds
        self._clock_epoch += 1
        self._last_clock_ns = now_ns
        self._latest = None
        self._last_receive_ns = None
        self._pending = None
        self._pending_receive_ns = None
        self._last_source_timestamp_ns = None
        self._latched_reason = None
        self._publish(False, "epoch_reset_waiting_for_target")
        response.success = True
        response.message = f"safety reset to local clock epoch {self._clock_epoch}"
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SafetyMonitor()
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
