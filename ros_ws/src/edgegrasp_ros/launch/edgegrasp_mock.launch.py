from edgegrasp.config import DEFAULT_TARGET_FRAME, ROS_SYSTEM_CLOCK_DOMAIN
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
    arm_downstream_action = LaunchConfiguration("arm_downstream_action")
    gripper_downstream_action = LaunchConfiguration("gripper_downstream_action")
    typed_terminal_wall_guard_ms = ParameterValue(
        LaunchConfiguration("typed_terminal_wall_guard_ms"), value_type=float
    )
    backend = LaunchConfiguration("backend")
    require_camera = ParameterValue(
        LaunchConfiguration("require_camera"), value_type=bool
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("target_frame", default_value=DEFAULT_TARGET_FRAME),
            DeclareLaunchArgument(
                "clock_domain", default_value=ROS_SYSTEM_CLOCK_DOMAIN
            ),
            DeclareLaunchArgument("clock_epoch", default_value="0"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("backend", default_value="mock"),
            DeclareLaunchArgument("require_camera", default_value="false"),
            DeclareLaunchArgument(
                "arm_downstream_action",
                default_value="/arm_controller/follow_joint_trajectory",
            ),
            DeclareLaunchArgument(
                "gripper_downstream_action",
                default_value="/gripper_controller/follow_joint_trajectory",
            ),
            DeclareLaunchArgument(
                "typed_terminal_wall_guard_ms", default_value="90000.0"
            ),
            Node(
                package="edgegrasp_ros",
                executable="mock_target_publisher",
                name="mock_target",
                parameters=[
                    {
                        "frame_id": target_frame,
                        "clock_domain": clock_domain,
                        "clock_epoch": clock_epoch,
                        "velocity_x_mps": 0.02,
                        "use_sim_time": use_sim_time,
                    }
                ],
                output="screen",
            ),
            Node(
                package="edgegrasp_ros",
                executable="safety_monitor",
                name="safety_monitor",
                parameters=[
                    {
                        "target_frame": target_frame,
                        "clock_domain": clock_domain,
                        "clock_epoch": clock_epoch,
                        "stale_after_ms": 200.0,
                        "watchdog_timeout_ms": 200.0,
                        "use_sim_time": use_sim_time,
                    }
                ],
                output="screen",
            ),
            Node(
                package="edgegrasp_ros",
                executable="trajectory_gate",
                name="trajectory_gate",
                parameters=[
                    {
                        "target_frame": target_frame,
                        "clock_domain": clock_domain,
                        "clock_epoch": clock_epoch,
                        "arm_downstream_action": arm_downstream_action,
                        "gripper_downstream_action": gripper_downstream_action,
                        "typed_terminal_wall_guard_ms": (
                            typed_terminal_wall_guard_ms
                        ),
                        "use_sim_time": use_sim_time,
                    }
                ],
                output="screen",
            ),
            Node(
                package="edgegrasp_ros",
                executable="interface_probe",
                name="interface_probe",
                parameters=[
                    {
                        "backend": backend,
                        "require_camera": require_camera,
                        "use_sim_time": use_sim_time,
                    }
                ],
                output="screen",
            ),
        ]
    )
