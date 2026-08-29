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
    planning_frame = LaunchConfiguration("planning_frame")
    clock_domain = LaunchConfiguration("clock_domain")
    clock_epoch = ParameterValue(LaunchConfiguration("clock_epoch"), value_type=int)
    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool
    )
    planning_scene_ready_topic = LaunchConfiguration(
        "planning_scene_ready_topic"
    )
    ik_response_timeout_ms = ParameterValue(
        LaunchConfiguration("ik_response_timeout_ms"), value_type=float
    )
    gate_terminal_wall_guard_ms = ParameterValue(
        LaunchConfiguration("gate_terminal_wall_guard_ms"), value_type=float
    )
    future_skew_tolerance_ms = ParameterValue(
        LaunchConfiguration("future_skew_tolerance_ms"), value_type=float
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("planning_frame", default_value=DEFAULT_TARGET_FRAME),
            DeclareLaunchArgument("clock_domain", default_value=ROS_SIM_CLOCK_DOMAIN),
            DeclareLaunchArgument("clock_epoch", default_value="0"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "planning_scene_ready_topic",
                default_value="/edgegrasp/planning_scene_ready",
            ),
            DeclareLaunchArgument(
                "ik_response_timeout_ms",
                default_value="5000.0",
                description=(
                    "bounded wall-clock /compute_ik response guard; ROS-time "
                    "target freshness remains independently fail-closed"
                ),
            ),
            DeclareLaunchArgument(
                "gate_terminal_wall_guard_ms",
                default_value="120000.0",
                description=(
                    "bounded outer wall guard for a crashed typed gate; the "
                    "gate retains ROS-time execution watchdog authority"
                ),
            ),
            DeclareLaunchArgument(
                "future_skew_tolerance_ms",
                default_value=str(DEFAULT_ROS_FUTURE_SKEW_TOLERANCE_MS),
            ),
            Node(
                package="edgegrasp_moveit_adapter",
                executable="moveit_adapter",
                parameters=[
                    {
                        "planning_frame": planning_frame,
                        "clock_domain": clock_domain,
                        "clock_epoch": clock_epoch,
                        "use_sim_time": use_sim_time,
                        "planning_scene_ready_topic": planning_scene_ready_topic,
                        "ik_response_timeout_ms": ik_response_timeout_ms,
                        "gate_terminal_wall_guard_ms": (
                            gate_terminal_wall_guard_ms
                        ),
                        "future_skew_tolerance_ms": future_skew_tolerance_ms,
                    }
                ],
                output="screen",
            ),
        ]
    )
