"""One-shot client for the immutable EdgeGrasp PlanTarget action."""

from __future__ import annotations

import json
import math
import sys
import time

from edgegrasp.config import (
    DEFAULT_TARGET_FRAME,
    ROS_SIM_CLOCK_DOMAIN,
    validate_ros_clock_domain,
)
from edgegrasp_interfaces.action import PlanTarget
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class PlanTargetClient(Node):
    def __init__(self) -> None:
        super().__init__("edgegrasp_plan_target_client")
        self.declare_parameter("action_name", "/edgegrasp/plan_target")
        self.declare_parameter("target_frame", DEFAULT_TARGET_FRAME)
        self.declare_parameter("clock_domain", ROS_SIM_CLOCK_DOMAIN)
        self.declare_parameter("clock_epoch", 0)
        self.declare_parameter("task_id", "manual-runtime-task")
        self.declare_parameter("stage", "diagnostic")
        self.declare_parameter("sequence_no", 0)
        self.declare_parameter("target_id", "manual-runtime-probe")
        self.declare_parameter("target_x_m", 0.15)
        self.declare_parameter("target_y_m", -0.10)
        self.declare_parameter("target_z_m", 0.08)
        self.declare_parameter("use_current_end_effector_pose", False)
        self.declare_parameter("end_effector_link", "gripper_frame_link")
        self.declare_parameter("target_offset_x_m", 0.0)
        self.declare_parameter("target_offset_y_m", 0.0)
        self.declare_parameter("target_offset_z_m", 0.0)
        self.declare_parameter("tf_discovery_timeout_s", 5.0)
        self.declare_parameter("orientation_x", 0.0)
        self.declare_parameter("orientation_y", 0.0)
        self.declare_parameter("orientation_z", 0.0)
        self.declare_parameter("orientation_w", 0.0)
        self.declare_parameter("pipeline_id", "ompl")
        self.declare_parameter("planner_id", "")
        self.declare_parameter("planning_timeout_s", 0.15)
        self.declare_parameter("velocity_scaling", 0.1)
        self.declare_parameter("acceleration_scaling", 0.1)
        self.declare_parameter("server_timeout_s", 5.0)
        self.declare_parameter("result_timeout_s", 5.0)
        self.declare_parameter("clock_start_timeout_s", 5.0)
        self._use_sim_time = bool(self.get_parameter("use_sim_time").value)
        validate_ros_clock_domain(
            self._use_sim_time,
            str(self.get_parameter("clock_domain").value),
        )
        self._client = ActionClient(
            self,
            PlanTarget,
            str(self.get_parameter("action_name").value),
        )
        self._feedback: list[str] = []
        self._tf_buffer = Buffer(node=self)
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False
        )

    def _current_end_effector_pose(self):
        target_frame = str(self.get_parameter("target_frame").value)
        end_effector = str(self.get_parameter("end_effector_link").value)
        deadline = time.monotonic() + float(
            self.get_parameter("tf_discovery_timeout_s").value
        )
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._tf_buffer.can_transform(target_frame, end_effector, Time()):
                return self._tf_buffer.lookup_transform(
                    target_frame,
                    end_effector,
                    Time(),
                    timeout=Duration(seconds=0.05),
                )
        raise TimeoutError(f"transform {target_frame}<-{end_effector} unavailable")

    def _goal(self) -> PlanTarget.Goal:
        goal = PlanTarget.Goal()
        goal.target_pose.header.frame_id = str(
            self.get_parameter("target_frame").value
        )
        if bool(self.get_parameter("use_current_end_effector_pose").value):
            transform = self._current_end_effector_pose()
            goal.target_pose.pose.position.x = transform.transform.translation.x + float(
                self.get_parameter("target_offset_x_m").value
            )
            goal.target_pose.pose.position.y = transform.transform.translation.y + float(
                self.get_parameter("target_offset_y_m").value
            )
            goal.target_pose.pose.position.z = transform.transform.translation.z + float(
                self.get_parameter("target_offset_z_m").value
            )
            goal.target_pose.pose.orientation = transform.transform.rotation
        else:
            goal.target_pose.pose.position.x = float(
                self.get_parameter("target_x_m").value
            )
            goal.target_pose.pose.position.y = float(
                self.get_parameter("target_y_m").value
            )
            goal.target_pose.pose.position.z = float(
                self.get_parameter("target_z_m").value
            )
            for field in ("x", "y", "z", "w"):
                value = float(self.get_parameter(f"orientation_{field}").value)
                if not math.isfinite(value):
                    raise ValueError("orientation values must be finite")
                setattr(goal.target_pose.pose.orientation, field, value)
        now = self.get_clock().now()
        goal.target_pose.header.stamp = now.to_msg()
        goal.task_id = str(self.get_parameter("task_id").value)
        goal.stage = str(self.get_parameter("stage").value)
        goal.sequence_no = int(self.get_parameter("sequence_no").value)
        goal.target_id = str(self.get_parameter("target_id").value)
        goal.source_timestamp_ns = now.nanoseconds
        goal.clock_domain = str(self.get_parameter("clock_domain").value)
        goal.clock_epoch = int(self.get_parameter("clock_epoch").value)
        goal.planning_group = "arm"
        goal.pipeline_id = str(self.get_parameter("pipeline_id").value)
        goal.planner_id = str(self.get_parameter("planner_id").value)
        goal.planning_timeout_s = float(
            self.get_parameter("planning_timeout_s").value
        )
        goal.velocity_scaling = float(
            self.get_parameter("velocity_scaling").value
        )
        goal.acceleration_scaling = float(
            self.get_parameter("acceleration_scaling").value
        )
        goal.plan_only = True
        return goal

    def _on_feedback(self, message) -> None:
        stage = message.feedback.stage
        self._feedback.append(stage)
        self.get_logger().info(f"stage={stage}")

    def run_once(self) -> tuple[int, dict[str, object]]:
        server_timeout = float(self.get_parameter("server_timeout_s").value)
        if not self._client.wait_for_server(timeout_sec=server_timeout):
            return 1, {"status": "failed", "reason": "adapter_unavailable"}
        if self._use_sim_time:
            deadline = time.monotonic() + float(
                self.get_parameter("clock_start_timeout_s").value
            )
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.get_clock().now().nanoseconds > 0:
                    break
            else:
                return 1, {"status": "failed", "reason": "sim_clock_unavailable"}
        send_future = self._client.send_goal_async(
            self._goal(), feedback_callback=self._on_feedback
        )
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=server_timeout)
        if not send_future.done():
            return 1, {"status": "failed", "reason": "goal_response_timeout"}
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return 1, {"status": "failed", "reason": "goal_rejected"}
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=float(self.get_parameter("result_timeout_s").value),
        )
        if not result_future.done():
            goal_handle.cancel_goal_async()
            return 1, {"status": "failed", "reason": "result_timeout"}
        wrapped = result_future.result()
        result = wrapped.result
        summary = {
            "action_status": int(wrapped.status),
            "task_id": result.task_id,
            "command_id": result.command_id,
            "target_id": result.target_id,
            "stage": result.stage,
            "sequence_no": int(result.sequence_no),
            "accepted": bool(result.accepted),
            "success": bool(result.success),
            "reason": result.reason,
            "moveit_error_code": int(result.moveit_error_code),
            "source_timestamp_ns": int(result.source_timestamp_ns),
            "clock_domain": result.clock_domain,
            "clock_epoch": int(result.clock_epoch),
            "trajectory_dispatched": bool(result.trajectory_dispatched),
            "trajectory_digest": result.trajectory_digest,
            "gate_accepted": bool(result.gate_accepted),
            "gate_terminal": bool(result.gate_terminal),
            "downstream_terminal_observed": bool(
                result.downstream_terminal_observed
            ),
            "cancel_requested": bool(result.cancel_requested),
            "downstream_action_status": int(result.action_goal_status),
            "fjt_error_code": int(result.fjt_error_code),
            "fjt_error_string": result.fjt_error_string,
            "feedback_stages": self._feedback,
            "moveit_direct_execution_used": False,
        }
        return (
            0
            if result.success
            and result.gate_terminal
            and result.downstream_terminal_observed
            else 1,
            summary,
        )


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = PlanTargetClient()
    try:
        code, summary = node.run_once()
        print(json.dumps(summary, sort_keys=True))
        return code
    except Exception as error:
        print(
            json.dumps(
                {"status": "failed", "error": f"{type(error).__name__}:{error}"},
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
