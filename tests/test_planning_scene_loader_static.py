from pathlib import Path
import ast
import xml.etree.ElementTree as ET


PROJECT_ROOT = Path(__file__).resolve().parents[1]
ROS_PACKAGE = PROJECT_ROOT / "ros_ws" / "src" / "edgegrasp_ros"
ADAPTER_PACKAGE = PROJECT_ROOT / "ros_ws" / "src" / "edgegrasp_moveit_adapter"
SEQUENCE_PACKAGE = PROJECT_ROOT / "ros_ws" / "src" / "edgegrasp_grasp_sequence"


def test_planning_scene_loader_is_service_confirmed_and_packaged() -> None:
    source_path = ROS_PACKAGE / "edgegrasp_ros" / "planning_scene_loader.py"
    source = source_path.read_text(encoding="utf-8")
    ast.parse(source)

    assert "GetPlanningScene" in source
    assert "WORLD_OBJECT_NAMES" in source
    assert "WORLD_OBJECT_GEOMETRY" in source
    assert "verify_planning_scene_objects" in source
    assert "planning_scene_query_timeout" in source
    assert "get_planning_scene_unavailable" in source
    assert "self._publish_ready(self._ready, self._last_reason)" in source
    assert "load_scene_contract" in source
    assert "CollisionObject.ADD" in source
    assert "planning_scene_ready" in source
    assert "except KeyboardInterrupt:" in source
    assert "if rclpy.ok():" in source

    setup = (ROS_PACKAGE / "setup.py").read_text(encoding="utf-8")
    assert (
        '"planning_scene_loader = edgegrasp_ros.planning_scene_loader:main"'
        in setup
    )
    launch = (ROS_PACKAGE / "launch" / "planning_scene.launch.py").read_text(
        encoding="utf-8"
    )
    ast.parse(launch)
    assert 'executable="planning_scene_loader"' in launch
    assert 'default_value="false"' in launch
    assert 'DeclareLaunchArgument("scene_config"' in launch
    assert '"scene_config": scene_config' in launch

    dependencies = {
        node.text for node in ET.parse(ROS_PACKAGE / "package.xml").findall("exec_depend")
    }
    assert {
        "moveit_msgs",
        "shape_msgs",
        "edgegrasp_core",
        "ros_gz_bridge",
        "ros_gz_image",
    } <= dependencies


def test_moveit_adapter_requires_recent_confirmed_planning_scene() -> None:
    source = (
        ADAPTER_PACKAGE / "edgegrasp_moveit_adapter" / "adapter_node.py"
    ).read_text(encoding="utf-8")
    launch = (
        ADAPTER_PACKAGE / "launch" / "moveit_adapter.launch.py"
    ).read_text(encoding="utf-8")
    ast.parse(source)
    ast.parse(launch)

    assert '"planning_scene_ready_topic", "/edgegrasp/planning_scene_ready"' in source
    assert "_planning_scene_permission = MotionPermissionGate" in source
    assert "planning_scene:safety_unknown" not in source
    assert 'return f"planning_scene:{planning_scene.reason}"' in source
    assert "_planning_scene_permission.reset_epoch" in source
    assert 'allow = request.stage in {"lift", "place", "retreat"}' in source
    assert "status_generation > status_generation_before_call" in source
    assert '"configuring_contact_policy"' in source
    assert "planning_scene_ready_topic" in launch

    sequence = (
        SEQUENCE_PACKAGE / "edgegrasp_grasp_sequence" / "node.py"
    ).read_text(encoding="utf-8")
    ast.parse(sequence)
    assert "_enable_target_pad_contacts_after_descend" in sequence
    assert "target.observed_at_ns >= confirmed_at_ns" in sequence
    assert sequence.index("_enable_target_pad_contacts_after_descend()") < (
        sequence.index("self._core.on_arm_terminal(")
    )
