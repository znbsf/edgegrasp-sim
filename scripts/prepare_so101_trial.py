#!/usr/bin/env python3
"""Prepare a SO-101 trial only through EdgeGrasp's typed trajectory gate.

This setup command intentionally targets ``/edgegrasp/execute_trajectory``.
It never creates an upstream controller-action client or accesses either
controller action directly. A fresh immutable TrackedTarget snapshot is used
for each correlated setup command.
"""

from __future__ import annotations

import argparse
import json
import math
import time

from action_msgs.msg import GoalStatus
from edgegrasp.trajectory_identity import make_trajectory_command_id
from edgegrasp.sim_control import GRIPPER_PRELOAD_EFFORT_LIMIT_NM
from edgegrasp_interfaces.action import ExecuteTrajectory
from edgegrasp_interfaces.msg import TrackedTarget
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


ARM_JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)
TERMINAL_POSITION_TOLERANCE_RAD = {
    "arm_controller": 0.05,
    "gripper_controller": 0.08,
}


def _arguments() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--arm",
        nargs=5,
        type=float,
        default=(-0.05, 0.75, -1.35, -1.5, 0.0),
    )
    parser.add_argument("--gripper", type=float, default=1.5)
    parser.add_argument(
        "--gripper-effort",
        type=float,
        default=None,
        help="optional Gazebo-only gripper effort feed-forward in N m",
    )
    parser.add_argument(
        "--gripper-only",
        action="store_true",
        help=(
            "open the gripper through ExecuteTrajectory but leave the arm to "
            "the collision-planned GraspSequence path"
        ),
    )
    parser.add_argument("--target-id", default="target_cube")
    parser.add_argument("--task-id", default="grasp-trial-preparation")
    parser.add_argument("--timeout-s", type=float, default=15.0)
    return parser.parse_known_args()


class PreparationClient(Node):
    def __init__(self, target_id: str) -> None:
        # This tool is intentionally Gazebo-only: its immutable target contract
        # requires ros_sim/epoch 0, so source freshness must be evaluated on the
        # same ROS simulation clock even when the caller omits ROS arguments.
        super().__init__(
            "edgegrasp_trial_preparation_client",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        if not bool(self.get_parameter("use_sim_time").value):
            raise RuntimeError("trial preparation requires use_sim_time=true")
        self.target_id = target_id
        self.latest: TrackedTarget | None = None
        self.latest_joint_positions: dict[str, float] | None = None
        self.latest_joint_observed_at = 0.0
        self.create_subscription(
            TrackedTarget,
            "/edgegrasp/tracked_target",
            self._on_target,
            1,
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            10,
        )
        self.gate = ActionClient(
            self, ExecuteTrajectory, "/edgegrasp/execute_trajectory"
        )

    def _on_target(self, message: TrackedTarget) -> None:
        if (
            message.target_id == self.target_id
            and message.clock_domain == "ros_sim"
            and int(message.clock_epoch) == 0
        ):
            self.latest = message

    def _on_joint_state(self, message: JointState) -> None:
        names = list(message.name)
        if len(names) != len(set(names)) or len(message.position) != len(names):
            return
        positions = {
            name: float(value) for name, value in zip(names, message.position)
        }
        if not all(math.isfinite(value) for value in positions.values()):
            return
        self.latest_joint_positions = positions
        self.latest_joint_observed_at = time.monotonic()

    def terminal_joint_state(
        self,
        *,
        joints: tuple[str, ...],
        positions: tuple[float, ...],
        observed_after: float,
        timeout_s: float,
    ) -> tuple[dict[str, float], dict[str, float]]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            observed = self.latest_joint_positions
            if (
                observed is None
                or self.latest_joint_observed_at < observed_after
                or any(name not in observed for name in joints)
            ):
                continue
            selected = {name: observed[name] for name in joints}
            errors = {
                name: abs(observed[name] - expected)
                for name, expected in zip(joints, positions)
            }
            return selected, errors
        raise TimeoutError("fresh terminal joint state not observed")

    @staticmethod
    def _source_ns(target: TrackedTarget) -> int:
        stamp = target.observation.header.stamp
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def fresh_target(self, timeout_s: float) -> TrackedTarget:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.latest is None:
                continue
            age_ns = self.get_clock().now().nanoseconds - self._source_ns(self.latest)
            if 0 <= age_ns <= 100_000_000:
                return self.latest
        raise TimeoutError("fresh matching TrackedTarget not observed")

    def wait_future(self, future, timeout_s: float, reason: str):
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done():
            raise TimeoutError(reason)
        return future.result()

    def execute(
        self,
        *,
        task_id: str,
        sequence_no: int,
        controller: str,
        joints: tuple[str, ...],
        positions: tuple[float, ...],
        efforts: tuple[float, ...] | None,
        duration_s: float,
        timeout_s: float,
    ) -> dict[str, object]:
        target = self.fresh_target(timeout_s)
        goal = ExecuteTrajectory.Goal()
        goal.task_id = task_id
        goal.stage = "diagnostic"
        goal.sequence_no = sequence_no
        goal.command_id = make_trajectory_command_id(
            task_id, goal.stage, sequence_no
        )
        goal.target_id = target.target_id
        goal.controller = controller
        goal.source_timestamp_ns = self._source_ns(target)
        goal.clock_domain = "ros_sim"
        goal.clock_epoch = 0
        goal.trajectory.header.frame_id = "base_link"
        goal.trajectory.joint_names = list(joints)
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        if efforts is not None:
            point.effort = list(efforts)
        point.time_from_start.sec = int(duration_s)
        point.time_from_start.nanosec = int(
            round((duration_s - int(duration_s)) * 1_000_000_000.0)
        )
        goal.trajectory.points = [point]
        command_started_at = time.monotonic()
        handle = self.wait_future(
            self.gate.send_goal_async(goal), timeout_s, "gate goal response timeout"
        )
        if not handle.accepted:
            return {
                "command_id": goal.command_id,
                "controller": controller,
                "success": False,
                "reason": "typed_gate_goal_rejected",
            }
        wrapped = self.wait_future(
            handle.get_result_async(), timeout_s, "typed gate result timeout"
        )
        result = wrapped.result
        success = (
            int(wrapped.status) == GoalStatus.STATUS_SUCCEEDED
            and bool(result.success)
            and bool(result.terminal)
            and bool(result.downstream_terminal_observed)
            and int(result.fjt_error_code) == 0
            and result.command_id == goal.command_id
        )
        observed_positions: dict[str, float] | None = None
        terminal_errors: dict[str, float] | None = None
        terminal_tolerance = TERMINAL_POSITION_TOLERANCE_RAD[controller]
        reported_reason = result.reason
        if success:
            observed_positions, terminal_errors = self.terminal_joint_state(
                joints=joints,
                positions=positions,
                observed_after=command_started_at,
                timeout_s=timeout_s,
            )
            if any(error > terminal_tolerance for error in terminal_errors.values()):
                success = False
                reported_reason = "terminal_joint_state_mismatch"
        return {
            "command_id": result.command_id,
            "controller": result.controller,
            "downstream_terminal_observed": result.downstream_terminal_observed,
            "fjt_error_code": result.fjt_error_code,
            "fjt_error_string": result.fjt_error_string,
            "reason": reported_reason,
            "success": success,
            "terminal_joint_positions_rad": observed_positions,
            "terminal_position_errors_rad": terminal_errors,
            "terminal_position_tolerance_rad": terminal_tolerance,
            "wrapper_status": int(wrapped.status),
        }


def main() -> int:
    args, ros_args = _arguments()
    values = (*args.arm, args.gripper, args.timeout_s)
    if any(not math.isfinite(value) for value in values) or args.timeout_s <= 0.0:
        raise ValueError("joint positions and timeout must be finite")
    if args.gripper_effort is not None and (
        not math.isfinite(args.gripper_effort)
        or abs(args.gripper_effort) > GRIPPER_PRELOAD_EFFORT_LIMIT_NM
    ):
        raise ValueError(
            "gripper effort must be finite and within "
            f"+/-{GRIPPER_PRELOAD_EFFORT_LIMIT_NM} N m"
        )
    rclpy.init(args=ros_args)
    node = PreparationClient(args.target_id)
    try:
        if not node.gate.wait_for_server(timeout_sec=min(args.timeout_s, 10.0)):
            raise TimeoutError("EdgeGrasp ExecuteTrajectory action unavailable")
        results = [
            node.execute(
                task_id=args.task_id,
                sequence_no=0,
                controller="gripper_controller",
                joints=("gripper",),
                positions=(args.gripper,),
                efforts=(args.gripper_effort,) if args.gripper_effort is not None else None,
                duration_s=2.0,
                timeout_s=args.timeout_s,
            )
        ]
        if results[-1]["success"] and not args.gripper_only:
            results.append(
                node.execute(
                    task_id=args.task_id,
                    sequence_no=1,
                    controller="arm_controller",
                    joints=ARM_JOINTS,
                    positions=tuple(args.arm),
                    efforts=None,
                    duration_s=4.0,
                    timeout_s=args.timeout_s,
                )
            )
        expected_result_count = 1 if args.gripper_only else 2
        success = len(results) == expected_result_count and all(
            item["success"] for item in results
        )
        print(
            json.dumps(
                {
                    "capability": "typed_edgegrasp_gate_trial_preparation",
                    "direct_fjt_bypass": False,
                    "gripper_only": bool(args.gripper_only),
                    "gripper_effort_nm": args.gripper_effort,
                    "arm_motion_requested": not args.gripper_only,
                    "results": results,
                    "success": success,
                },
                sort_keys=True,
                indent=2,
            )
        )
        return 0 if success else 2
    finally:
        node.gate.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
