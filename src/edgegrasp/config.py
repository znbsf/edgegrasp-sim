"""Cross-layer defaults shared by the core and ROS overlay."""

DEFAULT_TARGET_FRAME = "base_link"
DEFAULT_CLOCK_DOMAIN = "sim"
ROS_SYSTEM_CLOCK_DOMAIN = "ros_system"
ROS_SIM_CLOCK_DOMAIN = "ros_sim"
DEFAULT_STALE_AFTER_MS = 200.0
DEFAULT_WATCHDOG_TIMEOUT_MS = 200.0
# A ROS sim-clock sample can reach sibling nodes in different callback cycles.
# This is only a bounded "wait for local clock to catch up" window.  A future
# sample is never accepted early; a ROS wrapper may continue using a separately
# verified prior sample only while that prior sample remains fresh and live.
DEFAULT_ROS_FUTURE_SKEW_TOLERANCE_MS = 500.0
DEFAULT_MIN_CONFIDENCE = 0.5


def ros_clock_domain(use_sim_time: bool) -> str:
    """Return the explicit EdgeGrasp domain for one ROS clock policy."""

    if not isinstance(use_sim_time, bool):
        raise TypeError("use_sim_time must be bool")
    return ROS_SIM_CLOCK_DOMAIN if use_sim_time else ROS_SYSTEM_CLOCK_DOMAIN


def validate_ros_clock_domain(use_sim_time: bool, clock_domain: str) -> None:
    """Reject launch/node clock-domain drift instead of inferring silently."""

    expected = ros_clock_domain(use_sim_time)
    if clock_domain != expected:
        raise ValueError(
            f"clock_domain mismatch: use_sim_time={use_sim_time} "
            f"requires {expected}, got {clock_domain}"
        )
