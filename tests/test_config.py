import pytest

from edgegrasp.config import (
    DEFAULT_ROS_FUTURE_SKEW_TOLERANCE_MS,
    DEFAULT_TARGET_FRAME,
    ROS_SIM_CLOCK_DOMAIN,
    ROS_SYSTEM_CLOCK_DOMAIN,
    ros_clock_domain,
    validate_ros_clock_domain,
)
from edgegrasp.planner import EndpointWorkspaceGate


def test_target_frame_has_one_core_default() -> None:
    assert DEFAULT_TARGET_FRAME == "base_link"
    assert EndpointWorkspaceGate().target_frame == DEFAULT_TARGET_FRAME


def test_ros_clock_policy_is_explicit() -> None:
    assert DEFAULT_ROS_FUTURE_SKEW_TOLERANCE_MS == 500.0
    assert ros_clock_domain(False) == ROS_SYSTEM_CLOCK_DOMAIN == "ros_system"
    assert ros_clock_domain(True) == ROS_SIM_CLOCK_DOMAIN == "ros_sim"
    validate_ros_clock_domain(False, "ros_system")
    validate_ros_clock_domain(True, "ros_sim")


@pytest.mark.parametrize(
    ("use_sim_time", "wrong_domain"),
    [(False, "ros_sim"), (True, "ros_system")],
)
def test_ros_clock_policy_rejects_mismatch(
    use_sim_time: bool, wrong_domain: str
) -> None:
    with pytest.raises(ValueError, match="clock_domain mismatch"):
        validate_ros_clock_domain(use_sim_time, wrong_domain)
