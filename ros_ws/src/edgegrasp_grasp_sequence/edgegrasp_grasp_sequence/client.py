"""One-shot client for a correlated GraspSequence task.

This client snapshots an actually observed TrackedTarget before sending the
task. It never sets ``physics_grasp_verified``; that field belongs to an
independent contact/pose-retention observer.
"""

from __future__ import annotations

import json
import math
import time

from action_msgs.msg import GoalStatus
from edgegrasp.config import ROS_SIM_CLOCK_DOMAIN, validate_ros_clock_domain
from edgegrasp_interfaces.action import GraspSequence
from edgegrasp_interfaces.msg import TrackedTarget
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class GraspSequenceClient(Node):
    def __init__(self) -> None:
        super().__init__("edgegrasp_grasp_sequence_client")
        self.declare_parameter("sequence_action", "/edgegrasp/grasp_sequence")
        self.declare_parameter("tracked_target_topic", "/edgegrasp/tracked_target")
        self.declare_parameter("task_id", "grasp-runtime-001")
        self.declare_parameter("target_id", "target_cube")
        self.declare_parameter("clock_domain", ROS_SIM_CLOCK_DOMAIN)
        self.declare_parameter("clock_epoch", 0)
        self.declare_parameter("target_wait_timeout_s", 5.0)
        self.declare_parameter("action_wait_timeout_s", 5.0)
        self.declare_parameter("result_timeout_s", 45.0)
        self.declare_parameter("max_target_age_before_send_ms", 50.0)
        self.declare_parameter(
            "approach_position_m",
            [0.18606933614192925, 0.11976423272744036, 0.38721872836424076],
        )
        self.declare_parameter(
            "descend_position_m",
            [0.2462364349184608, 0.15125054509958977, 0.21671404504175756],
        )
        self.declare_parameter(
            "lift_position_m",
            [0.2462364349184608, 0.15125054509958977, 0.25671404504175754],
        )
        self.declare_parameter(
            "approach_orientation_xyzw",
            [
                0.19946281649811778,
                0.38019026690132723,
                0.8150060012713882,
                0.38914671228183106,
            ],
        )
        self.declare_parameter(
            "grasp_orientation_xyzw",
            [
                0.3450298741758752,
                0.6172118344266647,
                0.6332767577280262,
                0.3145862131297726,
            ],
        )
        self.declare_parameter("gripper_closed_position_rad", 0.60)
        self.declare_parameter("pipeline_id", "pilz_industrial_motion_planner")
        self.declare_parameter("planner_id", "PTP")
        self.declare_parameter("planning_timeout_s", 2.0)
        self.declare_parameter("velocity_scaling", 0.1)
        self.declare_parameter("acceleration_scaling", 0.1)

        self._clock_domain = str(self.get_parameter("clock_domain").value)
        self._clock_epoch = int(self.get_parameter("clock_epoch").value)
        validate_ros_clock_domain(
            bool(self.get_parameter("use_sim_time").value), self._clock_domain
        )
        self._target_id = str(self.get_parameter("target_id").value)
        self._latest: TrackedTarget | None = None
        self.create_subscription(
            TrackedTarget,
            str(self.get_parameter("tracked_target_topic").value),
            self._on_target,
            1,
        )
        self._client = ActionClient(
            self,
            GraspSequence,
            str(self.get_parameter("sequence_action").value),
        )

    def _on_target(self, message: TrackedTarget) -> None:
        if self._target_id and message.target_id != self._target_id:
            return
        if (
            message.clock_domain != self._clock_domain
            or int(message.clock_epoch) != self._clock_epoch
        ):
            return
        self._latest = message

    def _seconds(self, parameter: str) -> float:
        value = float(self.get_parameter(parameter).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{parameter} must be finite and positive")
        return value

    def _position(self, parameter: str) -> tuple[float, float, float]:
        values = tuple(float(value) for value in self.get_parameter(parameter).value)
        if len(values) != 3 or any(not math.isfinite(value) for value in values):
            raise ValueError(f"{parameter} must contain three finite values")
        return values

    def _orientation(self, parameter: str) -> tuple[float, float, float, float]:
        values = tuple(float(value) for value in self.get_parameter(parameter).value)
        if len(values) != 4 or any(not math.isfinite(value) for value in values):
            raise ValueError(f"{parameter} must contain four finite values")
        norm = math.sqrt(sum(value * value for value in values))
        if not math.isfinite(norm) or abs(norm - 1.0) > 1e-3:
            raise ValueError(f"{parameter} must be a normalized quaternion")
        return values

    def _latest_is_recent(self) -> bool:
        if self._latest is None:
            return False
        stamp = self._latest.observation.header.stamp
        sec = int(stamp.sec)
        nanosec = int(stamp.nanosec)
        if sec < 0 or not 0 <= nanosec < 1_000_000_000:
            return False
        now_ns = int(self.get_clock().now().nanoseconds)
        source_ns = sec * 1_000_000_000 + nanosec
        age_ns = now_ns - source_ns
        limit_ms = float(self.get_parameter("max_target_age_before_send_ms").value)
        if not math.isfinite(limit_ms) or not 0.0 < limit_ms < 200.0:
            raise ValueError(
                "max_target_age_before_send_ms must be finite, positive, and below 200"
            )
        return 0 <= age_ns <= int(limit_ms * 1_000_000.0)

    def _goal(self) -> GraspSequence.Goal:
        if self._latest is None:
            raise RuntimeError("no matching TrackedTarget was observed")
        goal = GraspSequence.Goal()
        goal.task_id = str(self.get_parameter("task_id").value)
        goal.target = self._latest
        for field, parameter in (
            (goal.approach_position, "approach_position_m"),
            (goal.descend_position, "descend_position_m"),
            (goal.lift_position, "lift_position_m"),
        ):
            field.x, field.y, field.z = self._position(parameter)
        for field, parameter in (
            (goal.approach_orientation, "approach_orientation_xyzw"),
            (goal.grasp_orientation, "grasp_orientation_xyzw"),
        ):
            field.x, field.y, field.z, field.w = self._orientation(parameter)
        goal.gripper_closed_position_rad = float(
            self.get_parameter("gripper_closed_position_rad").value
        )
        goal.pipeline_id = str(self.get_parameter("pipeline_id").value)
        goal.planner_id = str(self.get_parameter("planner_id").value)
        goal.planning_timeout_s = self._seconds("planning_timeout_s")
        goal.velocity_scaling = float(
            self.get_parameter("velocity_scaling").value
        )
        goal.acceleration_scaling = float(
            self.get_parameter("acceleration_scaling").value
        )
        return goal

    def _feedback(self, message) -> None:
        feedback = message.feedback
        print(
            json.dumps(
                {
                    "type": "feedback",
                    "task_id": feedback.task_id,
                    "phase": feedback.phase,
                    "command_id": feedback.command_id,
                    "reason": feedback.reason,
                },
                sort_keys=True,
            ),
            flush=True,
        )

    def run(self) -> int:
        if not self._client.wait_for_server(
            timeout_sec=self._seconds("action_wait_timeout_s")
        ):
            print('{"error":"grasp_sequence_action_unavailable"}', flush=True)
            return 3

        # Action discovery can consume the complete source-freshness budget.
        # Discard any target cached before readiness and snapshot a new sample
        # immediately before constructing the immutable sequence goal.
        self._latest = None
        target_deadline = time.monotonic() + self._seconds("target_wait_timeout_s")
        while time.monotonic() < target_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._latest_is_recent():
                break
        if not self._latest_is_recent():
            print('{"error":"tracked_target_timeout"}', flush=True)
            return 2
        send_future = self._client.send_goal_async(
            self._goal(), feedback_callback=self._feedback
        )
        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=self._seconds("action_wait_timeout_s"),
        )
        if not send_future.done():
            print('{"error":"goal_response_timeout"}', flush=True)
            return 4
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print('{"error":"goal_rejected"}', flush=True)
            return 5
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=self._seconds("result_timeout_s"),
        )
        if not result_future.done():
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
            print('{"error":"sequence_result_timeout_cancel_requested"}', flush=True)
            return 6
        wrapped = result_future.result()
        result = wrapped.result
        print(
            json.dumps(
                {
                    "type": "result",
                    "wrapper_status": int(wrapped.status),
                    "task_id": result.task_id,
                    "accepted": bool(result.accepted),
                    "sequence_completed": bool(result.sequence_completed),
                    "physics_grasp_verified": bool(
                        result.physics_grasp_verified
                    ),
                    "terminal_phase": result.terminal_phase,
                    "reason": result.reason,
                    "active_command_id": result.active_command_id,
                    "last_terminal_command_id": result.last_terminal_command_id,
                    "last_trajectory_digest": result.last_trajectory_digest,
                    "clock_epoch": int(result.clock_epoch),
                },
                sort_keys=True,
            ),
            flush=True,
        )
        return 0 if (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and result.sequence_completed
            and not result.physics_grasp_verified
        ) else 7

    def destroy_node(self) -> bool:
        self._client.destroy()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GraspSequenceClient()
    try:
        exit_code = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
