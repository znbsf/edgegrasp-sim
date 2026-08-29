from edgegrasp.config import DEFAULT_TARGET_FRAME, ROS_SIM_CLOCK_DOMAIN
from edgegrasp_ros.proxy_robot_description import build_proxy_robot_description
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _proxy_robot_state_publisher(context):
    """Expand the pinned xacro and replace only contracted collision meshes."""

    if LaunchConfiguration("robot_name").perform(context) != "so101":
        raise RuntimeError("collision proxy overlay supports robot_name=so101 only")
    if LaunchConfiguration("prefix").perform(context):
        raise RuntimeError("collision proxy overlay supports prefix='' only")
    use_camera = LaunchConfiguration("use_camera").perform(context)
    robot_description, _ = build_proxy_robot_description(
        robot_name="so101", prefix="", use_camera=use_camera
    )
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": robot_description,
                    "use_sim_time": True,
                }
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="edgegrasp_world_to_base_link",
            arguments=[
                "--x",
                "0",
                "--y",
                "0",
                "--z",
                "0",
                "--roll",
                "0",
                "--pitch",
                "0",
                "--yaw",
                "0",
                "--frame-id",
                "world",
                "--child-frame-id",
                DEFAULT_TARGET_FRAME,
            ],
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    use_rviz = LaunchConfiguration("use_rviz")
    use_camera = LaunchConfiguration("use_camera")
    target_frame = LaunchConfiguration("target_frame")
    clock_domain = LaunchConfiguration("clock_domain")
    clock_epoch = ParameterValue(LaunchConfiguration("clock_epoch"), value_type=int)
    require_camera = ParameterValue(use_camera, value_type=bool)
    arm_downstream_action = LaunchConfiguration("arm_downstream_action")
    gripper_downstream_action = LaunchConfiguration("gripper_downstream_action")
    upstream = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("so101_gazebo"), "launch", "so101.gazebo.launch.py"]
            )
        ),
        launch_arguments={
            "world_file": "empty.world",
            "use_rviz": use_rviz,
            "use_camera": use_camera,
            "use_robot_state_pub": "false",
            "use_sim_time": "true",
            "robot_name": LaunchConfiguration("robot_name"),
            "x": "0.0",
            "y": "0.0",
            "z": "0.0",
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("use_camera", default_value="true"),
            DeclareLaunchArgument("robot_name", default_value="so101"),
            DeclareLaunchArgument("prefix", default_value=""),
            DeclareLaunchArgument("target_frame", default_value=DEFAULT_TARGET_FRAME),
            DeclareLaunchArgument("clock_domain", default_value=ROS_SIM_CLOCK_DOMAIN),
            DeclareLaunchArgument("clock_epoch", default_value="0"),
            DeclareLaunchArgument(
                "arm_downstream_action",
                default_value="/arm_controller/follow_joint_trajectory",
            ),
            DeclareLaunchArgument(
                "gripper_downstream_action",
                default_value="/gripper_controller/follow_joint_trajectory",
            ),
            OpaqueFunction(function=_proxy_robot_state_publisher),
            upstream,
            Node(
                package="edgegrasp_ros",
                executable="mock_target_publisher",
                parameters=[
                    {
                        "frame_id": target_frame,
                        "clock_domain": clock_domain,
                        "clock_epoch": clock_epoch,
                        "velocity_x_mps": 0.0,
                        "use_sim_time": True,
                    }
                ],
                output="screen",
            ),
            Node(
                package="edgegrasp_ros",
                executable="safety_monitor",
                parameters=[
                    {
                        "target_frame": target_frame,
                        "clock_domain": clock_domain,
                        "clock_epoch": clock_epoch,
                        "stale_after_ms": 200.0,
                        "watchdog_timeout_ms": 200.0,
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
                        "arm_downstream_action": arm_downstream_action,
                        "gripper_downstream_action": gripper_downstream_action,
                        "use_sim_time": True,
                    }
                ],
                output="screen",
            ),
            Node(
                package="edgegrasp_ros",
                executable="interface_probe",
                parameters=[
                    {
                        "backend": "gazebo",
                        "arm_trajectory_action": arm_downstream_action,
                        "gripper_trajectory_action": gripper_downstream_action,
                        "require_camera": require_camera,
                        "use_sim_time": True,
                    }
                ],
                output="screen",
            ),
        ]
    )
