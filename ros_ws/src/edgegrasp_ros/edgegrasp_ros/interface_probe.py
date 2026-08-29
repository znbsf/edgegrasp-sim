"""Read-only ROS graph probe for the pinned SO-101 controller contract."""

from __future__ import annotations

import json
import time

from control_msgs.action import FollowJointTrajectory
from edgegrasp.so101_contract import (
    SO101_ARM_ACTION,
    SO101_CAMERA_COLOR_TOPIC,
    SO101_CAMERA_DEPTH_TOPIC,
    SO101_CAMERA_INFO_TOPIC,
    SO101_GRIPPER_ACTION,
)
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String


class InterfaceProbe(Node):
    def __init__(self) -> None:
        super().__init__("edgegrasp_interface_probe")
        self.declare_parameter("backend", "mock")
        self.declare_parameter("target_topic", "/edgegrasp/target_3d")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("arm_trajectory_action", SO101_ARM_ACTION)
        self.declare_parameter("gripper_trajectory_action", SO101_GRIPPER_ACTION)
        self.declare_parameter("camera_color_topic", SO101_CAMERA_COLOR_TOPIC)
        self.declare_parameter("camera_depth_topic", SO101_CAMERA_DEPTH_TOPIC)
        self.declare_parameter("camera_info_topic", SO101_CAMERA_INFO_TOPIC)
        self.declare_parameter("require_camera", False)
        # This probe measures wall/steady receive liveness.  Keep the threshold
        # above the observed worst-case 335 ms WSL camera-simulation interval;
        # the trajectory gate separately enforces its 250 ms ROS-time state age.
        self.declare_parameter("joint_state_timeout_ms", 750.0)
        self.declare_parameter("publish_rate_hz", 10.0)

        backend = str(self.get_parameter("backend").value)
        if backend not in {"mock", "gazebo", "real"}:
            raise ValueError("backend must be mock, gazebo, or real")
        actions = (
            str(self.get_parameter("arm_trajectory_action").value),
            str(self.get_parameter("gripper_trajectory_action").value),
        )
        if actions != (SO101_ARM_ACTION, SO101_GRIPPER_ACTION):
            raise ValueError("action endpoints do not match the pinned SO-101 contract")

        self._last_joint_state_monotonic_ns: int | None = None
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._on_joint_state,
            10,
        )
        self._arm_client = ActionClient(self, FollowJointTrajectory, actions[0])
        self._gripper_client = ActionClient(self, FollowJointTrajectory, actions[1])
        self._publisher = self.create_publisher(String, "/edgegrasp/interface_status", 10)
        self._ready_publisher = self.create_publisher(
            Bool, "/edgegrasp/interface_ready", 10
        )
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        self._timer = self.create_timer(
            1.0 / publish_rate_hz,
            self._publish_status,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def _on_joint_state(self, message: JointState) -> None:
        del message
        self._last_joint_state_monotonic_ns = time.monotonic_ns()

    def _topic_has_publishers(self, parameter_name: str) -> bool:
        topic = str(self.get_parameter(parameter_name).value)
        return bool(self.get_publishers_info_by_topic(topic))

    def _publish_status(self) -> None:
        backend = str(self.get_parameter("backend").value)
        target_ready = self._topic_has_publishers("target_topic")
        now_ns = time.monotonic_ns()
        joint_state_timeout_ns = int(
            float(self.get_parameter("joint_state_timeout_ms").value) * 1_000_000
        )
        joint_states_ready = (
            self._last_joint_state_monotonic_ns is not None
            and 0
            <= now_ns - self._last_joint_state_monotonic_ns
            <= joint_state_timeout_ns
        )
        arm_ready = self._arm_client.server_is_ready()
        gripper_ready = self._gripper_client.server_is_ready()
        camera_ready = all(
            self._topic_has_publishers(name)
            for name in ("camera_color_topic", "camera_depth_topic", "camera_info_topic")
        )
        require_camera = bool(self.get_parameter("require_camera").value)
        ready = target_ready
        if backend != "mock":
            ready = ready and joint_states_ready and arm_ready and gripper_ready
        if require_camera:
            ready = ready and camera_ready

        message = String()
        message.data = json.dumps(
            {
                "backend": backend,
                "ready": ready,
                "target": target_ready,
                "joint_states": joint_states_ready,
                "arm_follow_joint_trajectory": arm_ready,
                "gripper_follow_joint_trajectory": gripper_ready,
                "camera": camera_ready,
                "probe_only": True,
                "runtime_contract_verified": False,
            },
            sort_keys=True,
        )
        self._publisher.publish(message)
        self._ready_publisher.publish(Bool(data=ready))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = InterfaceProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
