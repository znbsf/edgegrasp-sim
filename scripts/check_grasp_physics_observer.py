#!/usr/bin/env python3
"""Run a read-only negative check against GraspPhysicsEvidence.

The observer action cannot command motion.  This client deliberately supplies
no sequence terminal and no synthetic gripper contact; a healthy live Gazebo
graph must therefore terminate with ``physics_grasp_verified == false``.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from edgegrasp.scene import load_scene_contract
from edgegrasp.trajectory_identity import make_trajectory_command_id
from edgegrasp_interfaces.action import GraspPhysicsEvidence
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter


ACTION_NAME = "/edgegrasp/grasp_physics_evidence"


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Verify that the read-only physics observer fails closed when no "
            "matching grasp sequence/contact/lift evidence is supplied."
        )
    )
    parser.add_argument("--action", default=ACTION_NAME)
    parser.add_argument("--task-id", default="physics-negative-check")
    parser.add_argument("--target-id", default="cube-1")
    parser.add_argument("--server-timeout-s", type=float, default=10.0)
    parser.add_argument("--result-timeout-s", type=float, default=10.0)
    return parser.parse_args()


def _wait_for_clock(node: Node, timeout_s: float) -> int:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        now_ns = node.get_clock().now().nanoseconds
        if now_ns > 0:
            return now_ns
    raise TimeoutError("ROS simulation clock did not become non-zero")


def _wait_future(node: Node, future, timeout_s: float):
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise TimeoutError("ROS future did not complete before the wall timeout")
    return future.result()


def _goal(node: Node, task_id: str, target_id: str) -> GraspPhysicsEvidence.Goal:
    scene_path = (
        Path(get_package_share_directory("edgegrasp_ros"))
        / "config"
        / "scene.json"
    )
    scene = load_scene_contract(scene_path)
    goal = GraspPhysicsEvidence.Goal()
    goal.task_id = task_id
    goal.target_id = target_id
    goal.target_source_timestamp_ns = node.get_clock().now().nanoseconds
    goal.cube_model_name = "target_cube"
    goal.pose_frame_id = scene.name
    goal.expected_lift_command_id = make_trajectory_command_id(task_id, "lift", 3)
    goal.scene_digest = scene.digest
    goal.started_at = node.get_clock().now().to_msg()
    goal.clock_domain = "ros_sim"
    goal.clock_epoch = 0
    goal.baseline_sample_count = 5
    goal.min_lift_m = 0.02
    goal.min_clearance_m = 0.01
    goal.retention_duration.sec = 0
    goal.retention_duration.nanosec = 500_000_000
    goal.max_xy_drift_m = 0.01
    goal.freshness_timeout.sec = 0
    goal.freshness_timeout.nanosec = 200_000_000
    goal.observation_timeout.sec = 0
    goal.observation_timeout.nanosec = 750_000_000
    return goal


def main() -> int:
    args = _arguments()
    started_wall = datetime.now(timezone.utc).isoformat()
    rclpy.init()
    node = Node(
        "edgegrasp_physics_negative_check",
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )
    client = ActionClient(node, GraspPhysicsEvidence, args.action)
    try:
        if not client.wait_for_server(timeout_sec=args.server_timeout_s):
            raise TimeoutError(f"action server unavailable: {args.action}")
        _wait_for_clock(node, args.server_timeout_s)
        goal_future = client.send_goal_async(
            _goal(node, args.task_id, args.target_id)
        )
        goal_handle = _wait_future(node, goal_future, args.result_timeout_s)
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("physics observer rejected the negative-check goal")
        wrapped = _wait_future(
            node, goal_handle.get_result_async(), args.result_timeout_s
        )
        result = wrapped.result
        record = {
            "action": args.action,
            "action_goal_status": int(wrapped.status),
            "accepted": bool(result.accepted),
            "clock_domain": result.clock_domain,
            "clock_epoch": int(result.clock_epoch),
            "command_capability": "none_read_only_observer",
            "evidence_digest": result.evidence_digest,
            "finished_at_utc": datetime.now(timezone.utc).isoformat(),
            "physics_grasp_verified": bool(result.physics_grasp_verified),
            "gripper_contact_observed": bool(result.gripper_contact_observed),
            "lift_observed": bool(result.lift_observed),
            "retention_observed": bool(result.retention_observed),
            "pose_sample_count": int(result.pose_sample_count),
            "reason": result.reason,
            "scene_digest": result.scene_digest,
            "sequence_completed": bool(result.sequence_completed),
            "started_at_utc": started_wall,
            "target_id": result.target_id,
            "task_id": result.task_id,
            "terminal_phase": result.terminal_phase,
        }
        print(json.dumps(record, sort_keys=True, indent=2))
        expected_abort = wrapped.status == GoalStatus.STATUS_ABORTED
        if (
            result.physics_grasp_verified
            or result.sequence_completed
            or result.terminal_phase != "FAULT"
            or not expected_abort
        ):
            print("negative physics-evidence contract FAILED", file=sys.stderr)
            return 5
        return 0
    except Exception as error:
        print(
            json.dumps(
                {
                    "action": args.action,
                    "error": f"{type(error).__name__}:{error}",
                    "finished_at_utc": datetime.now(timezone.utc).isoformat(),
                    "started_at_utc": started_wall,
                    "status": "UNVERIFIED",
                },
                sort_keys=True,
                indent=2,
            ),
            file=sys.stderr,
        )
        return 2
    finally:
        client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
