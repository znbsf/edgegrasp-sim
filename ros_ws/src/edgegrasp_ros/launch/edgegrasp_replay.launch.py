from edgegrasp.config import DEFAULT_TARGET_FRAME, ROS_SIM_CLOCK_DOMAIN
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    target_frame = LaunchConfiguration("target_frame")
    clock_domain = LaunchConfiguration("clock_domain")
    clock_epoch = ParameterValue(LaunchConfiguration("clock_epoch"), value_type=int)
    return LaunchDescription(
        [
            DeclareLaunchArgument("target_frame", default_value=DEFAULT_TARGET_FRAME),
            DeclareLaunchArgument("clock_domain", default_value=ROS_SIM_CLOCK_DOMAIN),
            DeclareLaunchArgument("clock_epoch", default_value="0"),
            Node(
                package="edgegrasp_ros",
                executable="safety_monitor",
                parameters=[
                    {
                        "target_frame": target_frame,
                        "clock_domain": clock_domain,
                        "clock_epoch": clock_epoch,
                        "use_sim_time": True,
                    }
                ],
                output="screen",
            ),
            Node(
                package="edgegrasp_ros",
                executable="trajectory_gate",
                parameters=[
                    {
                        "target_frame": target_frame,
                        "clock_domain": clock_domain,
                        "clock_epoch": clock_epoch,
                        "use_sim_time": True,
                    }
                ],
                output="screen",
            ),
        ]
    )
