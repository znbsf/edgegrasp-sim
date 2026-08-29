"""Launch pinned SO-101 MoveIt config with EdgeGrasp contact geometry.

No upstream MoveIt file is copied. The pinned package remains the source for
SRDF, kinematics, joint limits, controller mapping, and planning pipelines;
``robot_description`` preserves MoveIt's pinned arm collision meshes and adds
only the two EdgeGrasp distal-finger contact boxes. The 13 full-mesh AABB
replacements remain Gazebo/DART-only because they over-approximate concavities.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from edgegrasp_ros.proxy_robot_description import build_proxy_robot_description
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _moveit_nodes(context):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    prefix = LaunchConfiguration("prefix").perform(context)
    use_camera = LaunchConfiguration("use_camera").perform(context).lower()
    use_gazebo = LaunchConfiguration("use_gazebo").perform(context).lower()
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    moving_pad_distal_extension_m = float(
        LaunchConfiguration("moving_pad_distal_extension_m").perform(context)
    )
    if use_gazebo != "true":
        raise RuntimeError("proxy MoveGroup supports use_gazebo:=true only")
    if use_camera not in ("true", "false"):
        raise RuntimeError("use_camera must be true or false")

    proxy_description, report = build_proxy_robot_description(
        robot_name=robot_name,
        prefix=prefix,
        use_camera=use_camera,
        world_anchor=False,
        replace_collision_meshes=False,
        moving_pad_distal_extension_m=moving_pad_distal_extension_m,
    )
    if (
        report.world_anchor_added
        or report.root_links_after != ("base_link",)
        or report.replaced
        or len(report.remaining_collision_meshes) != 13
        or len(report.contact_extensions_added) != 2
    ):
        raise RuntimeError("MoveIt collision-proxy model failed its launch contract")

    moveit_share = Path(get_package_share_directory("so101_moveit_config"))
    description_share = Path(get_package_share_directory("so101_description"))
    config_dir = moveit_share / "config" / robot_name
    upstream_xacro = (
        description_share / "urdf" / "robots" / f"{robot_name}.urdf.xacro"
    )
    moveit_config = (
        MoveItConfigsBuilder(robot_name, package_name="so101_moveit_config")
        .robot_description(
            file_path=str(upstream_xacro),
            mappings={
                "robot_name": robot_name,
                "use_gazebo": use_gazebo,
                "use_camera": use_camera,
            },
        )
        .trajectory_execution(
            file_path=str(config_dir / "moveit_controllers.yaml")
        )
        .robot_description_semantic(
            file_path=str(config_dir / f"{robot_name}.srdf")
        )
        .joint_limits(file_path=str(config_dir / "joint_limits.yaml"))
        .robot_description_kinematics(
            file_path=str(config_dir / "kinematics.yaml")
        )
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"],
            default_planning_pipeline="ompl",
        )
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .pilz_cartesian_limits(
            file_path=str(config_dir / "pilz_cartesian_limits.yaml")
        )
        .to_moveit_configs()
    )
    moveit_parameters = moveit_config.to_dict()
    moveit_parameters["robot_description"] = proxy_description
    pilz_parameters = moveit_parameters.get("pilz_industrial_motion_planner")
    if not isinstance(pilz_parameters, dict):
        raise RuntimeError("pinned Pilz planning parameters are unavailable")
    # The pinned Pilz YAML has request-side start checks but no response
    # adapters.  Without ValidateSolution, a generated PTP trajectory can be
    # returned without a whole-trajectory collision check.  Keep the upstream
    # file untouched and close that gap in this project-owned launch overlay.
    pilz_parameters["response_adapters"] = [
        "default_planning_response_adapters/ValidateSolution"
    ]

    # MoveGroup remains available for plan_only action goals, while direct
    # trajectory execution and the optional ExecuteTaskSolutionCapability are
    # disabled. Execution belongs exclusively to EdgeGrasp's typed gate.
    execution_lockdown = {
        "allow_trajectory_execution": False,
        "disable_capabilities": (
            "move_group/MoveGroupExecuteTrajectoryAction "
            "move_group/ExecuteTaskSolutionCapability"
        ),
    }
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_parameters,
            {"use_sim_time": use_sim_time},
            {
                "start_state": {
                    "content": str(config_dir / "initial_positions.yaml")
                }
            },
            execution_lockdown,
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=[
            "-d",
            str(
                moveit_share
                / "rviz"
                / LaunchConfiguration("rviz_config_file").perform(context)
            ),
        ],
        parameters=[moveit_parameters, {"use_sim_time": use_sim_time}],
    )
    return [move_group, rviz]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="so101"),
            DeclareLaunchArgument("prefix", default_value=""),
            DeclareLaunchArgument("use_camera", default_value="false"),
            DeclareLaunchArgument("use_gazebo", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "moving_pad_distal_extension_m", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "rviz_config_file", default_value="move_group.rviz"
            ),
            OpaqueFunction(function=_moveit_nodes),
        ]
    )
