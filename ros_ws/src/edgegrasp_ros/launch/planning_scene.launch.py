from edgegrasp.config import DEFAULT_TARGET_FRAME, ROS_SIM_CLOCK_DOMAIN
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    scene_config = LaunchConfiguration("scene_config")
    target_frame = LaunchConfiguration("target_frame")
    clock_domain = LaunchConfiguration("clock_domain")
    clock_epoch = ParameterValue(LaunchConfiguration("clock_epoch"), value_type=int)
    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool
    )
    include_optional_cube = ParameterValue(
        LaunchConfiguration("include_optional_cube"), value_type=bool
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("scene_config", default_value=""),
            DeclareLaunchArgument("target_frame", default_value=DEFAULT_TARGET_FRAME),
            DeclareLaunchArgument("clock_domain", default_value=ROS_SIM_CLOCK_DOMAIN),
            DeclareLaunchArgument("clock_epoch", default_value="0"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("include_optional_cube", default_value="false"),
            Node(
                package="edgegrasp_ros",
                executable="planning_scene_loader",
                parameters=[
                    {
                        "scene_config": scene_config,
                        "target_frame": target_frame,
                        "clock_domain": clock_domain,
                        "clock_epoch": clock_epoch,
                        "use_sim_time": use_sim_time,
                        "include_optional_cube": include_optional_cube,
                    }
                ],
                output="screen",
            ),
        ]
    )
