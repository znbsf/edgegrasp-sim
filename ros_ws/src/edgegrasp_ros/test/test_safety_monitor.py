"""ROS-node-level tests for invalid targets, ordering, and epoch recovery."""

from __future__ import annotations

from itertools import count
import math
import time

from geometry_msgs.msg import PointStamped
import pytest
import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

from edgegrasp_ros.safety_monitor import SafetyMonitor


DOMAINS = count(201)


@pytest.fixture
def monitor():
    context = Context()
    rclpy.init(context=context, domain_id=next(DOMAINS))
    node = SafetyMonitor(
        context=context,
        parameter_overrides=[
            Parameter("use_sim_time", value=False),
            Parameter("clock_domain", value="ros_system"),
        ],
    )
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)


def target(node: SafetyMonitor, *, timestamp_ns: int | None = None) -> PointStamped:
    message = PointStamped()
    message.header.frame_id = "base_link"
    value = node.get_clock().now().nanoseconds if timestamp_ns is None else timestamp_ns
    message.header.stamp.sec = value // 1_000_000_000
    message.header.stamp.nanosec = value % 1_000_000_000
    message.point.x = 0.2
    message.point.z = 0.1
    return message


@pytest.mark.parametrize("invalid", (math.nan, math.inf, -math.inf))
def test_nonfinite_target_latches_and_clears_old_permission(monitor, invalid) -> None:
    fresh = target(monitor)
    monitor._on_target(fresh)
    assert monitor._latest is not None
    malformed = target(monitor, timestamp_ns=fresh.header.stamp.sec * 1_000_000_000 + fresh.header.stamp.nanosec + 1)
    malformed.point.x = invalid
    monitor._on_target(malformed)
    assert monitor._latched_reason == "invalid_target:ValueError"
    assert monitor._latest is None


def test_duplicate_out_of_order_and_frame_mismatch_fail_closed(monitor) -> None:
    first = target(monitor)
    monitor._on_target(first)
    duplicate = target(
        monitor,
        timestamp_ns=first.header.stamp.sec * 1_000_000_000
        + first.header.stamp.nanosec,
    )
    monitor._on_target(duplicate)
    assert monitor._latched_reason is not None
    assert monitor._latched_reason.startswith("source_timestamp_not_monotonic:")

    response = monitor._on_reset(Trigger.Request(), Trigger.Response())
    assert response.success
    mismatch = target(monitor)
    mismatch.header.frame_id = "camera_link"
    monitor._on_target(mismatch)
    assert monitor._latched_reason == "frame_mismatch:camera_link!=base_link"


def test_bounded_callback_order_future_target_recovers_but_rollback_needs_epoch_reset(
    monitor,
) -> None:
    future = target(
        monitor,
        timestamp_ns=monitor.get_clock().now().nanoseconds + 20_000_000,
    )
    monitor._on_target(future)
    assert monitor._pending is not None
    assert monitor._latest is None
    deadline = time.monotonic() + 1.0
    while monitor._pending is not None and time.monotonic() < deadline:
        monitor._evaluate()
        time.sleep(0.005)
    assert monitor._pending is None
    assert monitor._latest is not None

    # The node has already observed the live ROS system clock while accepting
    # and evaluating the target above.  Exercise rollback relative to that
    # observed value instead of assuming a newly constructed node starts at 0.
    baseline_ns = monitor._last_clock_ns
    assert baseline_ns is not None
    assert monitor._observe_clock(baseline_ns + 100)
    assert not monitor._observe_clock(baseline_ns + 99)
    assert monitor._latched_reason == (
        f"clock_rollback:{baseline_ns + 99}<{baseline_ns + 100}"
    )
    old_epoch = monitor._clock_epoch
    response = monitor._on_reset(Trigger.Request(), Trigger.Response())
    assert response.success
    assert monitor._clock_epoch == old_epoch + 1
    assert monitor._latched_reason is None
    assert monitor._latest is None


def test_bounded_future_sample_keeps_a_still_fresh_prior_target_allowed(
    monitor, monkeypatch
) -> None:
    published: list[tuple[bool, str]] = []
    monkeypatch.setattr(
        monitor,
        "_publish",
        lambda allowed, reason, **diagnostic: published.append((allowed, reason)),
    )

    first = target(monitor)
    monitor._on_target(first)
    assert monitor._latest is not None

    future = target(
        monitor,
        timestamp_ns=monitor.get_clock().now().nanoseconds + 20_000_000,
    )
    monitor._on_target(future)

    assert monitor._pending is not None
    assert monitor._latest is not None
    assert published[-1][0] is True
    assert published[-1][1].startswith("allowed_previous_target_pending:")
    observed_future_ns = int(
        published[-1][1]
        .removeprefix("allowed_previous_target_pending:")
        .removesuffix("ns")
    )
    assert 0 < observed_future_ns <= 20_000_000


def test_future_sample_beyond_clock_skew_tolerance_latches_false(
    monitor, monkeypatch
) -> None:
    published: list[tuple[bool, str]] = []
    monkeypatch.setattr(
        monitor,
        "_publish",
        lambda allowed, reason, **diagnostic: published.append((allowed, reason)),
    )

    first = target(monitor)
    monitor._on_target(first)
    future_ns = monitor._future_skew_tolerance_ns + 100_000_000
    future = target(
        monitor,
        timestamp_ns=monitor.get_clock().now().nanoseconds + future_ns,
    )
    monitor._on_target(future)

    assert monitor._latched_reason is not None
    assert monitor._latched_reason.startswith(
        "future_target_skew_exceeds_tolerance:"
    )
    assert monitor._latest is None
    assert published[-1][0] is False


def test_permission_diagnostic_preserves_source_and_issuance_interval(monitor):
    import json
    from types import SimpleNamespace

    records = []
    monitor._permission_evidence = SimpleNamespace(
        publish=lambda message: records.append(json.loads(message.data)))
    monitor._on_target(target(monitor))
    monitor._evaluate()
    row = records[-1]
    assert row["allowed"]
    assert row["target_source_ns"] == monitor._latest.timestamp_ns
    assert row["target_source_ns"] <= row["decision_ns"] <= row["publish_before_ns"] <= row["publish_after_ns"]
    assert row["publish_after_ns"] - row["target_source_ns"] <= 200_000_000
