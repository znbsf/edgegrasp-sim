#!/usr/bin/env python3
"""Read-only SO-101 MoveIt planning probe.

The probe calls ``moveit_msgs/srv/GetMotionPlan`` and validates the returned
``RobotTrajectory`` with the same converter used by the EdgeGrasp MoveIt
adapter.  It can send either joint constraints or Cartesian pose constraints;
the latter is required for Pilz ``LIN`` probing.  It never publishes a
trajectory and never calls an execute action.
"""

from __future__ import annotations

import argparse
import json
import math
import time

from edgegrasp.so101_contract import SO101_ARM_JOINTS
from edgegrasp_moveit_adapter.trajectory_validation import (
    validate_and_convert_robot_trajectory,
)
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
import rclpy
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--label", required=True)
    parser.add_argument("--start", nargs=5, type=float, required=True)
    parser.add_argument(
        "--ik-seed",
        nargs=5,
        type=float,
        help="optional IK-only seed; the MotionPlan start remains --start",
    )
    goal = parser.add_mutually_exclusive_group(required=True)
    goal.add_argument("--goal", nargs=5, type=float)
    goal.add_argument("--target-position", nargs=3, type=float)
    parser.add_argument("--target-orientation", nargs=4, type=float)
    parser.add_argument("--start-gripper", type=float, default=1.5)
    parser.add_argument("--goal-tolerance-rad", type=float, default=0.005)
    parser.add_argument("--start-tolerance-rad", type=float, default=0.01)
    parser.add_argument("--allowed-planning-time-s", type=float, default=2.0)
    parser.add_argument("--service-timeout-s", type=float, default=10.0)
    parser.add_argument("--velocity-scaling", type=float, default=0.2)
    parser.add_argument("--acceleration-scaling", type=float, default=0.2)
    parser.add_argument("--pipeline-id", default="ompl")
    parser.add_argument("--planner-id", default="")
    parser.add_argument("--ik-timeout-s", type=float, default=0.05)
    parser.add_argument(
        "--goal-constraint-mode",
        choices=("joint", "pose"),
        default="joint",
        help=(
            "joint: resolve --target-position with IK and plan to joint "
            "constraints; pose: send position/orientation constraints directly"
        ),
    )
    parser.add_argument("--position-tolerance-m", type=float, default=0.002)
    parser.add_argument("--orientation-tolerance-rad", type=float, default=0.02)
    parser.add_argument(
        "--constraint-frame",
        choices=("base_link", "world"),
        default="base_link",
        help=(
            "frame for Cartesian constraints; world is valid only for the "
            "current fixed identity world->base_link contract"
        ),
    )
    parser.add_argument(
        "--capture-source-positions",
        action="store_true",
        help=(
            "include returned joint-space samples for read-only route diagnosis; "
            "the trajectory is still never published or executed"
        ),
    )
    return parser.parse_args()


def _duration_ns(point) -> int:
    return point.time_from_start.sec * 1_000_000_000 + point.time_from_start.nanosec


class PlanProbe(Node):
    def __init__(self) -> None:
        super().__init__("edgegrasp_read_only_joint_plan_probe")
        self.client = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

    def solve_ik(self, args: argparse.Namespace) -> tuple[tuple[float, ...], int]:
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = "arm"
        ik.ik_link_name = "gripper_frame_link"
        ik.robot_state.is_diff = True
        ik.robot_state.joint_state.name = list(SO101_ARM_JOINTS)
        ik.robot_state.joint_state.position = list(args.ik_seed or args.start)
        ik.avoid_collisions = True
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        ) = args.target_position
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = args.target_orientation
        ik.pose_stamped = pose
        ik.timeout = Duration(
            sec=int(args.ik_timeout_s),
            nanosec=int((args.ik_timeout_s % 1.0) * 1_000_000_000),
        )
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=args.service_timeout_s
        )
        if not future.done():
            raise TimeoutError("/compute_ik response timeout")
        response = future.result()
        if response is None:
            raise RuntimeError("/compute_ik returned no response")
        error_code = int(response.error_code.val)
        if error_code != MoveItErrorCodes.SUCCESS:
            return (), error_code
        positions = dict(
            zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
                strict=True,
            )
        )
        if any(name not in positions for name in SO101_ARM_JOINTS):
            raise RuntimeError("/compute_ik response omitted an arm joint")
        return tuple(float(positions[name]) for name in SO101_ARM_JOINTS), error_code

    def plan(self, args: argparse.Namespace):
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = "arm"
        motion.pipeline_id = args.pipeline_id
        motion.planner_id = args.planner_id
        motion.num_planning_attempts = 1
        motion.allowed_planning_time = args.allowed_planning_time_s
        motion.max_velocity_scaling_factor = args.velocity_scaling
        motion.max_acceleration_scaling_factor = args.acceleration_scaling

        start = RobotState()
        start.joint_state.header.frame_id = args.constraint_frame
        start.joint_state.name = [*SO101_ARM_JOINTS, "gripper"]
        start.joint_state.position = [*args.start, args.start_gripper]
        start.is_diff = False
        motion.start_state = start

        constraints = Constraints()
        constraints.name = args.label
        if args.goal_constraint_mode == "joint":
            if args.goal is None:
                raise ValueError("joint constraint mode requires a resolved joint goal")
            for name, position in zip(SO101_ARM_JOINTS, args.goal, strict=True):
                item = JointConstraint()
                item.joint_name = name
                item.position = position
                item.tolerance_above = args.goal_tolerance_rad
                item.tolerance_below = args.goal_tolerance_rad
                item.weight = 1.0
                constraints.joint_constraints.append(item)
        else:
            if args.target_position is None or args.target_orientation is None:
                raise ValueError(
                    "pose constraint mode requires --target-position and "
                    "--target-orientation"
                )
            region = BoundingVolume()
            sphere = SolidPrimitive()
            sphere.type = SolidPrimitive.SPHERE
            sphere.dimensions = [args.position_tolerance_m]
            region.primitives = [sphere]
            region_pose = Pose()
            (
                region_pose.position.x,
                region_pose.position.y,
                region_pose.position.z,
            ) = args.target_position
            region_pose.orientation.w = 1.0
            region.primitive_poses = [region_pose]

            position = PositionConstraint()
            position.header.frame_id = args.constraint_frame
            position.link_name = "gripper_frame_link"
            position.constraint_region = region
            position.weight = 1.0
            constraints.position_constraints = [position]

            orientation = OrientationConstraint()
            orientation.header.frame_id = args.constraint_frame
            orientation.link_name = "gripper_frame_link"
            (
                orientation.orientation.x,
                orientation.orientation.y,
                orientation.orientation.z,
                orientation.orientation.w,
            ) = args.target_orientation
            orientation.absolute_x_axis_tolerance = args.orientation_tolerance_rad
            orientation.absolute_y_axis_tolerance = args.orientation_tolerance_rad
            orientation.absolute_z_axis_tolerance = args.orientation_tolerance_rad
            orientation.weight = 1.0
            constraints.orientation_constraints = [orientation]
        motion.goal_constraints = [constraints]

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=args.service_timeout_s
        )
        if not future.done():
            raise TimeoutError("/plan_kinematic_path response timeout")
        response = future.result()
        if response is None:
            raise RuntimeError("/plan_kinematic_path returned no response")
        return response.motion_plan_response


def main() -> int:
    args = _arguments()
    numeric = (
        *args.start,
        *(args.ik_seed or ()),
        *(args.goal or ()),
        *(args.target_position or ()),
        *(args.target_orientation or ()),
        args.start_gripper,
        args.goal_tolerance_rad,
        args.start_tolerance_rad,
        args.allowed_planning_time_s,
        args.service_timeout_s,
        args.velocity_scaling,
        args.acceleration_scaling,
        args.ik_timeout_s,
        args.position_tolerance_m,
        args.orientation_tolerance_rad,
    )
    if any(not math.isfinite(value) for value in numeric):
        raise ValueError("all numeric arguments must be finite")
    if (
        args.goal_tolerance_rad <= 0.0
        or args.start_tolerance_rad <= 0.0
        or args.allowed_planning_time_s <= 0.0
        or args.service_timeout_s <= 0.0
        or args.ik_timeout_s <= 0.0
        or args.position_tolerance_m <= 0.0
        or args.orientation_tolerance_rad <= 0.0
        or not 0.0 < args.velocity_scaling <= 1.0
        or not 0.0 < args.acceleration_scaling <= 1.0
    ):
        raise ValueError("timeouts, tolerances, and scaling factors are invalid")
    if (args.target_position is None) != (args.target_orientation is None):
        raise ValueError(
            "--target-position and --target-orientation must be supplied together"
        )
    if args.goal_constraint_mode == "pose" and args.goal is not None:
        raise ValueError("pose constraint mode does not accept --goal")
    if args.target_orientation is not None:
        norm = math.sqrt(sum(value * value for value in args.target_orientation))
        if abs(norm - 1.0) > 1e-3:
            raise ValueError("target orientation must be normalized")

    rclpy.init()
    node = PlanProbe()
    started = time.monotonic()
    try:
        if not node.client.wait_for_service(timeout_sec=5.0):
            raise TimeoutError("/plan_kinematic_path unavailable")
        ik_error_code = None
        if args.goal is None and args.goal_constraint_mode == "joint":
            if not node.ik_client.wait_for_service(timeout_sec=5.0):
                raise TimeoutError("/compute_ik unavailable")
            goal, ik_error_code = node.solve_ik(args)
            if not goal:
                print(
                    json.dumps(
                        {
                            "capability": "moveit_compute_ik_then_get_motion_plan_no_execute",
                            "label": args.label,
                            "ik_error_code": ik_error_code,
                            "plan_accepted": False,
                            "execution_attempted": False,
                        },
                        indent=2,
                        sort_keys=True,
                    )
                )
                return 2
            args.goal = goal
        response = node.plan(args)
        error = response.error_code
        decision = validate_and_convert_robot_trajectory(
            response.trajectory,
            fresh_start_positions=args.start,
            start_tolerance_rad=args.start_tolerance_rad,
        )
        source_points = response.trajectory.joint_trajectory.points
        accepted_points = (
            decision.joint_trajectory.points
            if decision.joint_trajectory is not None
            else ()
        )
        payload = {
            "capability": "moveit_get_motion_plan_no_execute",
            "label": args.label,
            "elapsed_wall_s": time.monotonic() - started,
            "start": dict(zip(SO101_ARM_JOINTS, args.start, strict=True)),
            "ik_seed": (
                dict(zip(SO101_ARM_JOINTS, args.ik_seed, strict=True))
                if args.ik_seed is not None
                else None
            ),
            "goal": (
                dict(zip(SO101_ARM_JOINTS, args.goal, strict=True))
                if args.goal is not None
                else None
            ),
            "goal_constraint_mode": args.goal_constraint_mode,
            "constraint_frame": args.constraint_frame,
            "position_tolerance_m": args.position_tolerance_m,
            "orientation_tolerance_rad": args.orientation_tolerance_rad,
            "target_position_m": args.target_position,
            "target_orientation_xyzw": args.target_orientation,
            "ik_error_code": ik_error_code,
            "start_gripper": args.start_gripper,
            "moveit_error_code": int(error.val),
            "moveit_error_message": error.message,
            "moveit_error_source": error.source,
            "pipeline_id": args.pipeline_id,
            "planner_id": args.planner_id,
            "planning_time_s": float(response.planning_time),
            "source_joint_names": list(
                response.trajectory.joint_trajectory.joint_names
            ),
            "source_point_count": len(source_points),
            "source_positions_rad": (
                [list(point.positions) for point in source_points]
                if args.capture_source_positions
                else None
            ),
            "source_last_time_ns": (
                _duration_ns(source_points[-1]) if source_points else None
            ),
            "validator_accepted": decision.accepted,
            "validator_reason": decision.reason.value,
            "validator_detail": decision.detail,
            "gate_ready_point_count": len(accepted_points),
            "gate_ready_last_time_ns": (
                _duration_ns(accepted_points[-1]) if accepted_points else None
            ),
            "plan_accepted": int(error.val) == 1 and decision.accepted,
            "execution_attempted": False,
        }
        print(json.dumps(payload, indent=2, sort_keys=True))
        return 0 if payload["plan_accepted"] else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
