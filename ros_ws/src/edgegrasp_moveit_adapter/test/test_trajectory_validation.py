import math

from edgegrasp.so101_contract import SO101_ARM_JOINTS
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint, MultiDOFJointTrajectoryPoint

from edgegrasp_moveit_adapter.trajectory_validation import (
    RobotTrajectoryReason,
    validate_and_convert_robot_trajectory,
)


START = (0.0, 0.0, 0.0, 0.0, 0.0)


def point(positions, seconds, *, velocities=None, accelerations=None):
    message = JointTrajectoryPoint()
    message.positions = list(positions)
    message.velocities = list(velocities or [])
    message.accelerations = list(accelerations or [])
    message.time_from_start.sec = int(seconds)
    message.time_from_start.nanosec = int((seconds - int(seconds)) * 1e9)
    return message


def trajectory(*points):
    result = RobotTrajectory()
    result.joint_trajectory.joint_names = list(SO101_ARM_JOINTS)
    result.joint_trajectory.points = list(points)
    return result


def validate(message):
    return validate_and_convert_robot_trajectory(
        message,
        fresh_start_positions=START,
        start_tolerance_rad=0.02,
    )


def test_valid_moveit_trajectory_discards_only_validated_zero_time_start() -> None:
    decision = validate(
        trajectory(
            point(START, 0.0),
            point((0.05, -0.05, 0.05, -0.05, 0.0), 1.0),
        )
    )
    assert decision.accepted
    assert decision.reason is RobotTrajectoryReason.VALID
    assert decision.joint_trajectory is not None
    assert len(decision.joint_trajectory.points) == 1
    assert decision.joint_trajectory.points[0].time_from_start.sec == 1


def test_fixed_base_rejects_multi_dof_points() -> None:
    message = trajectory(point(START, 0.0), point(START, 1.0))
    message.multi_dof_joint_trajectory.points = [MultiDOFJointTrajectoryPoint()]
    decision = validate(message)
    assert not decision.accepted
    assert decision.reason is RobotTrajectoryReason.MULTI_DOF_NOT_EMPTY


def test_wrong_joint_order_empty_and_start_mismatch_fail_closed() -> None:
    wrong_order = trajectory(point(START, 0.0), point(START, 1.0))
    wrong_order.joint_trajectory.joint_names.reverse()
    assert validate(wrong_order).reason is RobotTrajectoryReason.JOINT_ORDER_MISMATCH
    assert validate(trajectory()).reason is RobotTrajectoryReason.EMPTY
    mismatched = trajectory(
        point((0.03, 0.0, 0.0, 0.0, 0.0), 0.0), point(START, 1.0)
    )
    assert validate(mismatched).reason is RobotTrajectoryReason.START_STATE_MISMATCH


def test_zero_time_only_and_malformed_start_fail_closed() -> None:
    zero_only = validate(trajectory(point(START, 0.0)))
    assert zero_only.reason is RobotTrajectoryReason.NO_POSITIVE_TIME_POINT
    malformed = validate(trajectory(point((0.0,) * 4, 0.0), point(START, 1.0)))
    assert malformed.reason is RobotTrajectoryReason.INVALID_START_POINT


def test_nonfinite_nonmonotonic_limits_velocity_and_acceleration_fail_closed() -> None:
    cases = [
        trajectory(point(START, 0.0), point((0.0, 0.0, math.nan, 0.0, 0.0), 1.0)),
        trajectory(point(START, 0.0), point(START, 1.0), point(START, 0.5)),
        trajectory(point(START, 0.0), point((1.81, 0.0, 0.0, 0.0, 0.0), 2.0)),
        trajectory(
            point(START, 0.0),
            point(START, 1.0, velocities=(1.01, 0.0, 0.0, 0.0, 0.0)),
        ),
        trajectory(
            point(START, 0.0),
            point(START, 1.0, accelerations=(2.51, 0.0, 0.0, 0.0, 0.0)),
        ),
    ]
    for message in cases:
        decision = validate(message)
        assert not decision.accepted
        assert decision.reason is RobotTrajectoryReason.CONTRACT_REJECTED
