from edgegrasp.config import (
    DEFAULT_ROS_FUTURE_SKEW_TOLERANCE_MS,
    DEFAULT_TARGET_FRAME,
    ROS_SIM_CLOCK_DOMAIN,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    target_frame = LaunchConfiguration("target_frame")
    clock_domain = LaunchConfiguration("clock_domain")
    clock_epoch = ParameterValue(LaunchConfiguration("clock_epoch"), value_type=int)
    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool
    )
    future_skew_tolerance_ms = ParameterValue(
        LaunchConfiguration("future_skew_tolerance_ms"), value_type=float
    )
    gripper_feedforward_effort_nm = ParameterValue(
        LaunchConfiguration("gripper_feedforward_effort_nm"), value_type=float
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("target_frame", default_value=DEFAULT_TARGET_FRAME),
            DeclareLaunchArgument("clock_domain", default_value=ROS_SIM_CLOCK_DOMAIN),
            DeclareLaunchArgument("clock_epoch", default_value="0"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "future_skew_tolerance_ms",
                default_value=str(DEFAULT_ROS_FUTURE_SKEW_TOLERANCE_MS),
            ),
            DeclareLaunchArgument(
                "gripper_feedforward_effort_nm", default_value="0.0"
            ),
            Node(
                package="edgegrasp_grasp_sequence",
                executable="grasp_sequence",
                parameters=[
                    {
                        "target_frame": target_frame,
                        "clock_domain": clock_domain,
                        "clock_epoch": clock_epoch,
                        "use_sim_time": use_sim_time,
                        "future_skew_tolerance_ms": future_skew_tolerance_ms,
                        "gripper_feedforward_effort_nm": (
                            gripper_feedforward_effort_nm
                        ),
                    }
                ],
                output="screen",
            ),
        ]
    )
