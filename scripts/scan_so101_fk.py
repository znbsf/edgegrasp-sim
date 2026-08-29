#!/usr/bin/env python3
"""Read-only coarse SO-101 FK scan through MoveIt's GetPositionFK service.

This diagnostic never calls an action server or publishes a trajectory. It
selects pose/orientation candidates for later collision-aware IK checks.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import time

from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK
import rclpy
from rclpy.node import Node


ARM_JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)


@dataclass(frozen=True)
class Candidate:
    distance_m: float
    joints: tuple[float, ...]
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]


def _values(lower: float, upper: float, step: float) -> tuple[float, ...]:
    count = int(math.floor((upper - lower) / step))
    return tuple(lower + index * step for index in range(count + 1)) + (upper,)


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", nargs=3, type=float, required=True)
    parser.add_argument("--step", type=float, default=0.45)
    parser.add_argument("--top", type=int, default=12)
    parser.add_argument("--service", default="/compute_fk")
    parser.add_argument("--timeout-s", type=float, default=30.0)
    return parser.parse_args()


class Scanner(Node):
    def __init__(self, service: str) -> None:
        super().__init__("edgegrasp_read_only_fk_scan")
        self.client = self.create_client(GetPositionFK, service)

    def evaluate(
        self, joints: tuple[float, ...], timeout_s: float
    ) -> Candidate | None:
        request = GetPositionFK.Request()
        request.header.frame_id = "base_link"
        request.fk_link_names = ["gripper_frame_link"]
        state = RobotState()
        state.joint_state.header.frame_id = "base_link"
        state.joint_state.name = [*ARM_JOINTS, "gripper"]
        state.joint_state.position = [*joints, 1.5]
        state.is_diff = False
        request.robot_state = state
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done():
            raise TimeoutError("GetPositionFK response timeout")
        response = future.result()
        if response is None or int(response.error_code.val) != 1:
            return None
        if len(response.pose_stamped) != 1:
            raise RuntimeError("GetPositionFK returned an unexpected pose count")
        pose = response.pose_stamped[0].pose
        return Candidate(
            distance_m=0.0,
            joints=joints,
            position=(pose.position.x, pose.position.y, pose.position.z),
            orientation=(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )


def main() -> int:
    args = _arguments()
    if (
        len(args.target) != 3
        or any(not math.isfinite(value) for value in args.target)
        or not math.isfinite(args.step)
        or args.step <= 0.0
        or args.top <= 0
        or not math.isfinite(args.timeout_s)
        or args.timeout_s <= 0.0
    ):
        raise ValueError("scan arguments must be finite and positive")
    rclpy.init()
    scanner = Scanner(args.service)
    started = time.monotonic()
    candidates: list[Candidate] = []
    evaluated = 0
    try:
        if not scanner.client.wait_for_service(timeout_sec=min(args.timeout_s, 10.0)):
            raise TimeoutError(f"GetPositionFK unavailable: {args.service}")
        pan_values = _values(-0.35, 0.35, args.step)
        lift_values = _values(-1.65, 1.65, args.step)
        elbow_values = _values(-1.6, 1.6, args.step)
        wrist_values = _values(-1.55, 1.55, args.step)
        deadline = started + args.timeout_s
        for pan in pan_values:
            for lift in lift_values:
                for elbow in elbow_values:
                    for wrist in wrist_values:
                        remaining = deadline - time.monotonic()
                        if remaining <= 0.0:
                            raise TimeoutError("bounded FK scan deadline expired")
                        candidate = scanner.evaluate(
                            (pan, lift, elbow, wrist, 0.0), min(remaining, 1.0)
                        )
                        evaluated += 1
                        if candidate is None:
                            continue
                        distance = math.dist(candidate.position, args.target)
                        candidates.append(
                            Candidate(
                                distance_m=distance,
                                joints=candidate.joints,
                                position=candidate.position,
                                orientation=candidate.orientation,
                            )
                        )
        candidates.sort(key=lambda item: (item.distance_m, item.joints))
        print(
            json.dumps(
                {
                    "capability": "read_only_fk_service_scan_no_motion",
                    "evaluated": evaluated,
                    "elapsed_s": time.monotonic() - started,
                    "target_m": args.target,
                    "results": [
                        {
                            "distance_m": item.distance_m,
                            "joints": dict(zip(ARM_JOINTS, item.joints, strict=True)),
                            "position_m": item.position,
                            "orientation_xyzw": item.orientation,
                        }
                        for item in candidates[: args.top]
                    ],
                },
                sort_keys=True,
                indent=2,
            )
        )
        return 0
    finally:
        scanner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
