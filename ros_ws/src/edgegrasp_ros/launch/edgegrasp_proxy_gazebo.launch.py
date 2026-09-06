from edgegrasp.config import (
    DEFAULT_ROS_FUTURE_SKEW_TOLERANCE_MS,
    DEFAULT_TARGET_FRAME,
    ROS_SIM_CLOCK_DOMAIN,
)
from edgegrasp_ros.proxy_robot_description import build_proxy_robot_description
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _proxy_publishers(context):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    prefix = LaunchConfiguration("prefix").perform(context)
    use_camera = LaunchConfiguration("use_camera").perform(context)
    pad_contact_material_profile = LaunchConfiguration(
        "pad_contact_material_profile"
    ).perform(context)
    moving_pad_distal_extension_m = float(
        LaunchConfiguration("moving_pad_distal_extension_m").perform(context)
    )
    gripper_control_profile = LaunchConfiguration(
        "gripper_control_profile"
    ).perform(context)
    robot_description, _ = build_proxy_robot_description(
        robot_name=robot_name,
        prefix=prefix,
        use_camera=use_camera,
        camera_update_rate_hz=float(LaunchConfiguration("camera_update_rate_hz").perform(context)),
        camera_view=LaunchConfiguration("camera_view").perform(context),
        camera_resolution=LaunchConfiguration("camera_resolution").perform(context),
        pad_contact_material_profile=pad_contact_material_profile,
        moving_pad_distal_extension_m=moving_pad_distal_extension_m,
        gripper_control_profile=gripper_control_profile,
    )
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description, "use_sim_time": True}
            ],
            output="screen",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    use_camera = LaunchConfiguration("use_camera")
    launch_edgegrasp_nodes = LaunchConfiguration("launch_edgegrasp_nodes")
    launch_physics_observer = LaunchConfiguration("launch_physics_observer")
    target_frame = LaunchConfiguration("target_frame")
    clock_domain = LaunchConfiguration("clock_domain")
    clock_epoch = ParameterValue(LaunchConfiguration("clock_epoch"), value_type=int)
    target_id = LaunchConfiguration("target_id")
    target_x_m = ParameterValue(LaunchConfiguration("target_x_m"), value_type=float)
    target_y_m = ParameterValue(LaunchConfiguration("target_y_m"), value_type=float)
    target_z_m = ParameterValue(LaunchConfiguration("target_z_m"), value_type=float)
    arm_downstream_action = LaunchConfiguration("arm_downstream_action")
    gripper_downstream_action = LaunchConfiguration("gripper_downstream_action")
    typed_terminal_wall_guard_ms = ParameterValue(
        LaunchConfiguration("typed_terminal_wall_guard_ms"), value_type=float
    )
    future_skew_tolerance_ms = ParameterValue(
        LaunchConfiguration("future_skew_tolerance_ms"), value_type=float
    )
    target_publisher_start_delay_s = LaunchConfiguration(
        "target_publisher_start_delay_s"
    )
    require_camera = ParameterValue(use_camera, value_type=bool)
    scene_config = PathJoinSubstitution(
        [
            FindPackageShare("edgegrasp_ros"),
            "config",
            LaunchConfiguration("scene_config_filename"),
        ]
    )

    world = PathJoinSubstitution(
        [
            FindPackageShare("edgegrasp_ros"),
            "worlds",
            LaunchConfiguration("world_filename"),
        ]
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": [" -s -r -v 4 ", world]}.items(),
    )
    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("so101_moveit_config"),
                    "launch",
                    "load_ros2_controllers.launch.py",
                ]
            )
        ),
        launch_arguments={"controller_load_delay": "5.0"}.items(),
    )
    spawn_robot = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-world",
                    "edgegrasp_table_cube",
                    "-topic",
                    "/robot_description",
                    "-name",
                    LaunchConfiguration("robot_name"),
                    "-allow_renaming",
                    "false",
                    "-x",
                    "0.0",
                    "-y",
                    "0.0",
                    "-z",
                    "0.0",
                ],
                output="screen",
            )
        ],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        condition=UnlessCondition(use_camera),
        output="screen",
    )
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": PathJoinSubstitution(
                    [
                        FindPackageShare("so101_gazebo"),
                        "config",
                        "ros_gz_bridge.yaml",
                    ]
                )
            }
        ],
        condition=IfCondition(use_camera),
        output="screen",
    )
    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera_head/depth_image", "/camera_head/image"],
        remappings=[
            ("/camera_head/depth_image", "/camera_head/depth/image_rect_raw"),
            ("/camera_head/image", "/camera_head/color/image_raw"),
        ],
        condition=IfCondition(use_camera),
        output="screen",
    )
    scene_evidence_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="edgegrasp_scene_evidence_bridge",
        parameters=[
            {
                "config_file": PathJoinSubstitution(
                    [
                        FindPackageShare("edgegrasp_ros"),
                        "config",
                        "scene_bridge.yaml",
                    ]
                )
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="so101"),
            DeclareLaunchArgument("prefix", default_value=""),
            DeclareLaunchArgument("use_camera", default_value="false"),
            DeclareLaunchArgument("camera_update_rate_hz", default_value="5.0"),
            DeclareLaunchArgument("camera_view", default_value="upstream"),
            DeclareLaunchArgument("camera_resolution", default_value="upstream"),
            DeclareLaunchArgument(
                "pad_contact_material_profile", default_value="implicit_default"
            ),
            DeclareLaunchArgument(
                "moving_pad_distal_extension_m", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "gripper_control_profile", default_value="position_only"
            ),
            DeclareLaunchArgument("launch_edgegrasp_nodes", default_value="true"),
            DeclareLaunchArgument("launch_mock_target", default_value="true"),
            DeclareLaunchArgument("launch_physics_observer", default_value="true"),
            DeclareLaunchArgument("observation_wall_factor", default_value="2.0"),
            DeclareLaunchArgument("world_filename", default_value="table_cube.sdf"),
            DeclareLaunchArgument("scene_config_filename", default_value="scene.json"),
            DeclareLaunchArgument("target_frame", default_value=DEFAULT_TARGET_FRAME),
            DeclareLaunchArgument("clock_domain", default_value=ROS_SIM_CLOCK_DOMAIN),
            DeclareLaunchArgument("clock_epoch", default_value="0"),
            DeclareLaunchArgument("target_id", default_value="ros-target"),
            DeclareLaunchArgument("target_x_m", default_value="0.15"),
            DeclareLaunchArgument("target_y_m", default_value="-0.10"),
            DeclareLaunchArgument("target_z_m", default_value="0.08"),
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
            DeclareLaunchArgument(
                "future_skew_tolerance_ms",
                default_value=str(DEFAULT_ROS_FUTURE_SKEW_TOLERANCE_MS),
                description=(
                    "bounded cross-process ROS clock catch-up window; future "
                    "targets remain denied until local clock catches up"
                ),
            ),
            DeclareLaunchArgument(
                "target_publisher_start_delay_s",
                default_value="3.0",
                description=(
                    "start the synthetic target only after the Gazebo /clock "
                    "bridge and fail-closed safety consumer have initialized"
                ),
            ),
            OpaqueFunction(function=_proxy_publishers),
            gazebo,
            clock_bridge,
            camera_bridge,
            image_bridge,
            scene_evidence_bridge,
            spawn_robot,
            controllers,
            TimerAction(
                period=target_publisher_start_delay_s,
                actions=[
                    Node(
                        package="edgegrasp_ros",
                        executable="mock_target_publisher",
                        parameters=[
                            {
                                "frame_id": target_frame,
                                "target_id": target_id,
                                "clock_domain": clock_domain,
                                "clock_epoch": clock_epoch,
                                "start_x_m": target_x_m,
                                "start_y_m": target_y_m,
                                "start_z_m": target_z_m,
                                "velocity_x_mps": 0.0,
                                "use_sim_time": True,
                            }
                        ],
                        condition=IfCondition(PythonExpression([
                            "'", launch_edgegrasp_nodes, "' == 'true' and '",
                            LaunchConfiguration("launch_mock_target"), "' == 'true'",
                        ])),
                        output="screen",
                    )
                ],
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
                        "future_skew_tolerance_ms": future_skew_tolerance_ms,
                        "use_sim_time": True,
                    }
                ],
                condition=IfCondition(launch_edgegrasp_nodes),
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
                        "typed_terminal_wall_guard_ms": (
                            typed_terminal_wall_guard_ms
                        ),
                        "use_sim_time": True,
                    }
                ],
                condition=IfCondition(launch_edgegrasp_nodes),
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
                condition=IfCondition(launch_edgegrasp_nodes),
                output="screen",
            ),
            Node(
                package="edgegrasp_ros",
                executable="grasp_physics_observer",
                parameters=[
                    {
                        "observation_wall_factor": ParameterValue(LaunchConfiguration("observation_wall_factor"), value_type=float),
                        "clock_domain": clock_domain,
                        "clock_epoch": clock_epoch,
                        "scene_config": scene_config,
                        "use_sim_time": True,
                    }
                ],
                condition=IfCondition(launch_physics_observer),
                output="screen",
            ),
        ]
    )
