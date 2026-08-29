#!/usr/bin/env python3
"""Read-only collision-aware SO-101 IK probe through MoveIt's service.

The diagnostic calls only ``moveit_msgs/srv/GetPositionIK``. It does not
create an action client, publish a trajectory, or command a controller.
"""

from __future__ import annotations

import argparse
import json
import math
import time

from moveit_msgs.srv import GetPositionIK
import rclpy
from rclpy.node import Node


ARM_JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--position", nargs=3, type=float, required=True)
    parser.add_argument("--orientation", nargs=4, type=float, required=True)
    parser.add_argument("--seed", nargs=5, type=float, default=(0.0,) * 5)
    parser.add_argument("--gripper", type=float, default=1.5)
    parser.add_argument(
        "--allow-collisions",
        action="store_true",
        help="diagnostic comparison only; never use this result for motion",
    )
    parser.add_argument("--service", default="/compute_ik")
    parser.add_argument("--timeout-s", type=float, default=5.0)
    return parser.parse_args()


class Probe(Node):
    def __init__(self, service: str) -> None:
        super().__init__("edgegrasp_read_only_ik_probe")
        self.client = self.create_client(GetPositionIK, service)


def main() -> int:
    args = _arguments()
    values = (*args.position, *args.orientation, *args.seed, args.gripper)
    if any(not math.isfinite(value) for value in values):
        raise ValueError("pose, seed, and gripper values must be finite")
    norm = math.sqrt(sum(value * value for value in args.orientation))
    if abs(norm - 1.0) > 1e-3:
        raise ValueError("orientation quaternion must be normalized")
    if not math.isfinite(args.timeout_s) or args.timeout_s <= 0.0:
        raise ValueError("timeout must be finite and positive")

    rclpy.init()
    probe = Probe(args.service)
    started = time.monotonic()
    try:
        if not probe.client.wait_for_service(timeout_sec=args.timeout_s):
            raise TimeoutError(f"GetPositionIK unavailable: {args.service}")
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = "arm"
        ik.ik_link_name = "gripper_frame_link"
        ik.robot_state.is_diff = True
        ik.robot_state.joint_state.name = [*ARM_JOINTS, "gripper"]
        ik.robot_state.joint_state.position = [*args.seed, args.gripper]
        ik.avoid_collisions = not args.allow_collisions
        ik.pose_stamped.header.frame_id = "base_link"
        ik.pose_stamped.pose.position.x = args.position[0]
        ik.pose_stamped.pose.position.y = args.position[1]
        ik.pose_stamped.pose.position.z = args.position[2]
        ik.pose_stamped.pose.orientation.x = args.orientation[0]
        ik.pose_stamped.pose.orientation.y = args.orientation[1]
        ik.pose_stamped.pose.orientation.z = args.orientation[2]
        ik.pose_stamped.pose.orientation.w = args.orientation[3]
        whole = int(args.timeout_s)
        ik.timeout.sec = whole
        ik.timeout.nanosec = int(
            round((args.timeout_s - whole) * 1_000_000_000.0)
        )
        future = probe.client.call_async(request)
        rclpy.spin_until_future_complete(
            probe, future, timeout_sec=args.timeout_s + 1.0
        )
        if not future.done():
            raise TimeoutError("GetPositionIK response timeout")
        response = future.result()
        if response is None:
            raise RuntimeError("GetPositionIK returned no response")
        names = tuple(response.solution.joint_state.name)
        positions = tuple(response.solution.joint_state.position)
        by_name = dict(zip(names, positions, strict=False))
        code = int(response.error_code.val)
        print(
            json.dumps(
                {
                    "capability": "read_only_collision_aware_ik_service_no_motion",
                    "elapsed_s": time.monotonic() - started,
                    "error_code": code,
                    "avoid_collisions": not args.allow_collisions,
                    "message": str(getattr(response.error_code, "message", "")),
                    "source": str(getattr(response.error_code, "source", "")),
                    "position_m": args.position,
                    "orientation_xyzw": args.orientation,
                    "solution": {joint: by_name.get(joint) for joint in ARM_JOINTS},
                    "success": code == 1,
                },
                sort_keys=True,
                indent=2,
            )
        )
        return 0 if code == 1 else 2
    finally:
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
