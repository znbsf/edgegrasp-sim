#!/usr/bin/env python3
"""Chain a grasp candidate through MoveIt's plan-only service.

This probe intentionally creates no publisher or action client.  It calls only
``moveit_msgs/srv/GetMotionPlan`` and validates every returned joint trajectory
with the same converter used by the EdgeGrasp MoveIt adapter.  A successful
result is planning evidence only; it is never execution or grasp evidence.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import sys
import time
from typing import Any

from ament_index_python.packages import get_package_share_directory
from edgegrasp.grasp_geometry import (
    derive_grasp_stage_geometry,
    load_grasp_geometry_profile,
    validate_routed_grasp_stage_geometry,
)
from edgegrasp.scene import load_scene_contract
from edgegrasp.so101_contract import SO101_ARM_JOINTS
from edgegrasp.trajectory_identity import trajectory_digest
from edgegrasp_moveit_adapter.trajectory_validation import (
    validate_and_convert_robot_trajectory,
)
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetMotionPlan
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from std_srvs.srv import SetBool


PLANNING_FRAME = "base_link"
GRIPPER_JOINT = "gripper"


@dataclass(frozen=True, slots=True)
class Stage:
    name: str
    position_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    start_gripper_rad: float
    allow_target_pad_contacts: bool


def _arguments(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--label", required=True)
    parser.add_argument(
        "--candidate-filename", default="so101_side_grasp_candidate.json"
    )
    parser.add_argument("--scene-config-filename", default="scene.json")
    parser.add_argument("--grasp-geometry-filename", required=True)
    parser.add_argument(
        "--target-center-x-offset-m",
        type=float,
        default=0.0,
        help=(
            "diagnostic offset relative to the loaded profile; the effective "
            "geometry is still fully validated before planning"
        ),
    )
    parser.add_argument("--attempts", type=int, default=1)
    parser.add_argument("--joint-state-timeout-s", type=float, default=15.0)
    parser.add_argument("--max-joint-state-age-ms", type=float, default=200.0)
    parser.add_argument("--service-timeout-s", type=float, default=10.0)
    parser.add_argument("--allowed-planning-time-s", type=float, default=2.0)
    parser.add_argument("--start-tolerance-rad", type=float, default=0.01)
    parser.add_argument("--position-tolerance-m", type=float, default=0.002)
    parser.add_argument("--orientation-tolerance-rad", type=float, default=0.02)
    parser.add_argument("--pipeline-id", default="pilz_industrial_motion_planner")
    parser.add_argument("--planner-id", default="PTP")
    parser.add_argument("--velocity-scaling", type=float, default=0.1)
    parser.add_argument("--acceleration-scaling", type=float, default=0.1)
    args = parser.parse_args(argv)
    if args.attempts < 1 or args.attempts > 100:
        parser.error("--attempts must be in [1, 100]")
    positive = (
        args.joint_state_timeout_s,
        args.max_joint_state_age_ms,
        args.service_timeout_s,
        args.allowed_planning_time_s,
        args.start_tolerance_rad,
        args.position_tolerance_m,
        args.orientation_tolerance_rad,
    )
    if any(not math.isfinite(value) or value <= 0.0 for value in positive):
        parser.error("timeouts, ages, and tolerances must be finite and positive")
    for name in ("velocity_scaling", "acceleration_scaling"):
        value = float(getattr(args, name))
        if not math.isfinite(value) or not 0.0 < value <= 1.0:
            parser.error(f"--{name.replace('_', '-')} must be in (0, 1]")
    if not math.isfinite(args.target_center_x_offset_m):
        parser.error("--target-center-x-offset-m must be finite")
    for name in (
        "candidate_filename",
        "grasp_geometry_filename",
        "scene_config_filename",
    ):
        value = str(getattr(args, name))
        if Path(value).name != value or not value.endswith(".json"):
            parser.error(f"--{name.replace('_', '-')} must be one JSON basename")
    return args


def _vector(
    value: Any, *, length: int, field: str
) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"{field} must contain {length} values")
    result = tuple(float(item) for item in value)
    if any(not math.isfinite(item) for item in result):
        raise ValueError(f"{field} contains a non-finite value")
    return result


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load_stages(
    config_dir: Path, args: argparse.Namespace
) -> tuple[tuple[Stage, ...], dict[str, object]]:
    candidate_path = config_dir / args.candidate_filename
    profile_path = config_dir / args.grasp_geometry_filename
    scene_path = config_dir / args.scene_config_filename
    for path in (candidate_path, profile_path, scene_path):
        if not path.is_file():
            raise FileNotFoundError(f"required installed config is missing: {path}")

    candidate = json.loads(candidate_path.read_text(encoding="utf-8"))
    source_profile = load_grasp_geometry_profile(profile_path)
    offset_x = float(args.target_center_x_offset_m)
    if abs(offset_x) > source_profile.target_center_tolerance_m[0]:
        raise ValueError("target-center X offset exceeds the profile tolerance")
    source_center = source_profile.target_center_in_frame_at_descend_m
    effective_center = (
        round(source_center[0] + offset_x, 15),
        source_center[1],
        source_center[2],
    )
    profile = replace(
        source_profile,
        target_center_in_frame_at_descend_m=effective_center,
    )
    scene = load_scene_contract(scene_path)
    if candidate.get("planning_frame") != PLANNING_FRAME:
        raise ValueError("candidate planning_frame must be base_link")
    if candidate.get("planning_group") != "arm":
        raise ValueError("candidate planning_group must be arm")
    if candidate.get("target", {}).get("id") != profile.target_id:
        raise ValueError("candidate and grasp profile target_id disagree")
    cube = next(
        (item for item in scene.objects if item.object_id == profile.target_id),
        None,
    )
    if cube is None or cube.size_m != profile.cube_size_m:
        raise ValueError("scene target cube and grasp profile disagree")

    orientations = candidate.get("orientations_xyzw")
    stages = candidate.get("stages")
    if not isinstance(orientations, dict) or not isinstance(stages, dict):
        raise ValueError("candidate stages/orientations are missing")
    approach_position = _vector(
        stages.get("approach", {}).get("position_m"),
        length=3,
        field="stages.approach.position_m",
    )
    approach_orientation = _vector(
        orientations.get("approach"),
        length=4,
        field="orientations_xyzw.approach",
    )
    grasp_orientation = _vector(
        orientations.get("grasp"),
        length=4,
        field="orientations_xyzw.grasp",
    )
    derived = derive_grasp_stage_geometry(
        cube.pose_world.position_m,
        grasp_orientation,
        profile,
    )
    routed = validate_routed_grasp_stage_geometry(
        cube_center_m=cube.pose_world.position_m,
        approach_position_m=approach_position,
        descend_position_m=derived.descend_position_m,
        lift_position_m=derived.lift_position_m,
        approach_orientation_xyzw=approach_orientation,
        grasp_orientation_xyzw=grasp_orientation,
        gripper_position_rad=profile.gripper_contact_position_rad,
        profile=profile,
        cube_orientation_xyzw=cube.pose_world.quaternion_xyzw,
    )
    stage_rows = (
        Stage(
            "home_to_approach",
            routed.approach_position_m,
            routed.approach_orientation_xyzw,
            profile.gripper_open_position_rad,
            False,
        ),
        Stage(
            "approach_to_descend",
            routed.descend_position_m,
            routed.grasp_orientation_xyzw,
            profile.gripper_open_position_rad,
            False,
        ),
        Stage(
            "descend_to_lift_closed",
            routed.lift_position_m,
            routed.grasp_orientation_xyzw,
            profile.gripper_contact_position_rad,
            True,
        ),
    )
    config_evidence: dict[str, object] = {
        "candidate_path": str(candidate_path),
        "candidate_sha256": _sha256(candidate_path),
        "grasp_geometry_path": str(profile_path),
        "grasp_geometry_sha256": _sha256(profile_path),
        "scene_config_path": str(scene_path),
        "scene_config_sha256": _sha256(scene_path),
        "scene_contract_sha256": scene.digest,
        "source_target_center_in_frame_m": list(source_center),
        "diagnostic_target_center_x_offset_m": offset_x,
        "effective_target_center_in_frame_m": list(effective_center),
        "scene_path": str(scene_path),
        "scene_digest": scene.digest,
        "target_id": profile.target_id,
        "cube_center_m": list(cube.pose_world.position_m),
        "cube_size_m": list(cube.size_m),
        "cube_orientation_xyzw": list(cube.pose_world.quaternion_xyzw),
        "gripper_open_position_rad": profile.gripper_open_position_rad,
        "gripper_contact_position_rad": profile.gripper_contact_position_rad,
    }
    return stage_rows, config_evidence


def _duration_ns(point: Any) -> int:
    return int(point.time_from_start.sec) * 1_000_000_000 + int(
        point.time_from_start.nanosec
    )


def _point_payload(point: Any) -> dict[str, object]:
    return {
        "positions": list(point.positions),
        "velocities": list(point.velocities),
        "accelerations": list(point.accelerations),
        "effort": list(point.effort),
        "time_from_start_ns": _duration_ns(point),
    }


class CandidatePlanOnlyProbe(Node):
    def __init__(self) -> None:
        super().__init__("edgegrasp_grasp_candidate_plan_only_probe")
        self._joint_positions: tuple[float, ...] | None = None
        self._gripper_position: float | None = None
        self._joint_stamp_ns: int | None = None
        self._last_joint_stamp_ns: int | None = None
        self._clock_rollback: str | None = None
        self._scene_status: dict[str, object] | None = None
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            String,
            "/edgegrasp/planning_scene_status",
            self._on_scene_status,
            latched,
        )
        self._plan = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self._target_contacts = self.create_client(
            SetBool, "/edgegrasp/set_target_pad_contacts"
        )

    def _on_scene_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if isinstance(payload, dict):
            self._scene_status = payload

    @staticmethod
    def _stamp_ns(message: JointState) -> int:
        sec = int(message.header.stamp.sec)
        nanosec = int(message.header.stamp.nanosec)
        if sec < 0 or not 0 <= nanosec < 1_000_000_000:
            raise ValueError("joint_states contains an invalid timestamp")
        return sec * 1_000_000_000 + nanosec

    def _on_joint_state(self, message: JointState) -> None:
        if len(message.name) != len(message.position):
            return
        try:
            positions = {
                str(name): float(value)
                for name, value in zip(
                    message.name, message.position, strict=True
                )
            }
            required = (*SO101_ARM_JOINTS, GRIPPER_JOINT)
            if any(name not in positions for name in required):
                return
            values = tuple(positions[name] for name in SO101_ARM_JOINTS)
            gripper = positions[GRIPPER_JOINT]
            if any(not math.isfinite(value) for value in (*values, gripper)):
                return
            stamp_ns = self._stamp_ns(message)
        except (TypeError, ValueError):
            return
        if self._last_joint_stamp_ns is not None and stamp_ns < self._last_joint_stamp_ns:
            self._clock_rollback = (
                f"joint_state_clock_rollback:{stamp_ns}<"
                f"{self._last_joint_stamp_ns}"
            )
            return
        self._last_joint_stamp_ns = stamp_ns
        self._joint_stamp_ns = stamp_ns
        self._joint_positions = values
        self._gripper_position = gripper

    def fresh_start(
        self, *, timeout_s: float, max_age_ms: float
    ) -> tuple[tuple[float, ...], float, int, int]:
        deadline = time.monotonic() + timeout_s
        max_age_ns = int(max_age_ms * 1_000_000.0)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._clock_rollback is not None:
                raise RuntimeError(self._clock_rollback)
            if (
                self._joint_positions is None
                or self._gripper_position is None
                or self._joint_stamp_ns is None
            ):
                continue
            now_ns = int(self.get_clock().now().nanoseconds)
            age_ns = now_ns - self._joint_stamp_ns
            if 0 <= age_ns <= max_age_ns:
                return (
                    self._joint_positions,
                    self._gripper_position,
                    self._joint_stamp_ns,
                    now_ns,
                )
        raise TimeoutError("fresh /joint_states sample was not observed")

    def wait_for_plan_service(self, timeout_s: float) -> None:
        if not self._plan.wait_for_service(timeout_sec=timeout_s):
            raise TimeoutError("/plan_kinematic_path is unavailable")

    def set_target_pad_contacts(
        self, *, allow: bool, timeout_s: float, scene_digest: str
    ) -> dict[str, object]:
        if not self._target_contacts.wait_for_service(timeout_sec=timeout_s):
            raise TimeoutError("target-pad contact policy service is unavailable")
        request = SetBool.Request()
        request.data = bool(allow)
        future = self._target_contacts.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done():
            future.cancel()
            raise TimeoutError("target-pad contact policy response timeout")
        response = future.result()
        if response is None or not bool(response.success):
            detail = "no_response" if response is None else response.message
            raise RuntimeError(f"target-pad contact policy rejected:{detail}")
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            status = self._scene_status
            if status is None:
                continue
            if (
                status.get("ready") is True
                and status.get("reason") == "confirmed"
                and status.get("allow_target_pad_contacts") is allow
                and status.get("scene_digest") == scene_digest
                and status.get("target_frame") == PLANNING_FRAME
                and status.get("clock_domain") == "ros_sim"
                and status.get("clock_epoch") == 0
            ):
                return dict(status)
        raise TimeoutError("target-pad contact policy confirmation timeout")

    def graph_boundary(self) -> dict[str, object]:
        node_names = sorted(
            f"{namespace.rstrip('/')}/{name}".replace("//", "/")
            for name, namespace in self.get_node_names_and_namespaces()
        )
        forbidden_tokens = (
            "edgegrasp_moveit_plan_only_adapter",
            "edgegrasp_trajectory_gate",
            "edgegrasp_grasp_sequence",
        )
        forbidden = [
            name for name in node_names if any(token in name for token in forbidden_tokens)
        ]
        services = dict(self.get_service_names_and_types())
        execute_services = sorted(
            name for name in services if name.startswith("/edgegrasp/execute_trajectory")
        )
        return {
            "node_names": node_names,
            "forbidden_motion_nodes": forbidden,
            "execute_trajectory_services": execute_services,
            "motion_boundary_absent": not forbidden and not execute_services,
        }

    @staticmethod
    def _request(
        *,
        stage: Stage,
        start_positions: tuple[float, ...],
        args: argparse.Namespace,
    ) -> GetMotionPlan.Request:
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
        start.joint_state.header.frame_id = PLANNING_FRAME
        start.joint_state.name = [*SO101_ARM_JOINTS, GRIPPER_JOINT]
        start.joint_state.position = [*start_positions, stage.start_gripper_rad]
        start.is_diff = False
        motion.start_state = start

        region = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [args.position_tolerance_m]
        pose = Pose()
        (
            pose.position.x,
            pose.position.y,
            pose.position.z,
        ) = stage.position_m
        pose.orientation.w = 1.0
        region.primitives = [sphere]
        region.primitive_poses = [pose]

        position = PositionConstraint()
        position.header.frame_id = PLANNING_FRAME
        position.link_name = "gripper_frame_link"
        position.constraint_region = region
        position.weight = 1.0
        orientation = OrientationConstraint()
        orientation.header.frame_id = PLANNING_FRAME
        orientation.link_name = "gripper_frame_link"
        (
            orientation.orientation.x,
            orientation.orientation.y,
            orientation.orientation.z,
            orientation.orientation.w,
        ) = stage.orientation_xyzw
        orientation.absolute_x_axis_tolerance = args.orientation_tolerance_rad
        orientation.absolute_y_axis_tolerance = args.orientation_tolerance_rad
        orientation.absolute_z_axis_tolerance = args.orientation_tolerance_rad
        orientation.weight = 1.0
        constraints = Constraints()
        constraints.name = stage.name
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        motion.goal_constraints = [constraints]
        return request

    def plan_stage(
        self,
        *,
        attempt: int,
        stage: Stage,
        start_positions: tuple[float, ...],
        args: argparse.Namespace,
    ) -> tuple[dict[str, object], tuple[float, ...] | None]:
        requested_at = time.monotonic()
        future = self._plan.call_async(
            self._request(stage=stage, start_positions=start_positions, args=args)
        )
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=args.service_timeout_s
        )
        if not future.done():
            future.cancel()
            raise TimeoutError(f"{stage.name} planning response timeout")
        response = future.result()
        if response is None:
            raise RuntimeError(f"{stage.name} returned no planning response")
        motion = response.motion_plan_response
        decision = validate_and_convert_robot_trajectory(
            motion.trajectory,
            fresh_start_positions=start_positions,
            start_tolerance_rad=args.start_tolerance_rad,
        )
        accepted = (
            int(motion.error_code.val) == MoveItErrorCodes.SUCCESS
            and decision.accepted
            and decision.joint_trajectory is not None
        )
        converted = decision.joint_trajectory
        digest = None
        terminal = None
        if accepted:
            assert converted is not None
            digest = trajectory_digest(
                "arm_controller",
                converted.joint_names,
                [_point_payload(point) for point in converted.points],
            )
            source_points = motion.trajectory.joint_trajectory.points
            if not source_points or len(source_points[-1].positions) != len(
                SO101_ARM_JOINTS
            ):
                accepted = False
            else:
                terminal = tuple(float(value) for value in source_points[-1].positions)
                if any(not math.isfinite(value) for value in terminal):
                    accepted = False
                    terminal = None
        source_points = motion.trajectory.joint_trajectory.points
        record: dict[str, object] = {
            "attempt": attempt,
            "segment": stage.name,
            "target_position_m": list(stage.position_m),
            "target_orientation_xyzw": list(stage.orientation_xyzw),
            "start_positions_rad": list(start_positions),
            "start_gripper_rad": stage.start_gripper_rad,
            "moveit_error_code": int(motion.error_code.val),
            "moveit_error_message": motion.error_code.message,
            "moveit_error_source": motion.error_code.source,
            "planning_time_s": float(motion.planning_time),
            "elapsed_wall_s": time.monotonic() - requested_at,
            "source_joint_names": list(motion.trajectory.joint_trajectory.joint_names),
            "source_point_count": len(source_points),
            "source_last_time_ns": (
                _duration_ns(source_points[-1]) if source_points else None
            ),
            "validator_accepted": decision.accepted,
            "validator_reason": decision.reason.value,
            "validator_detail": decision.detail,
            "gate_ready_point_count": len(converted.points) if converted else 0,
            "trajectory_digest": digest,
            "terminal_joint_positions_rad": list(terminal) if terminal else None,
            "plan_accepted": accepted,
            "trajectory_published": False,
            "execute_trajectory_goal_sent": False,
            "fjt_goal_sent": False,
            "execution_attempted": False,
        }
        return record, terminal


def _run(args: argparse.Namespace) -> int:
    config_dir = Path(get_package_share_directory("edgegrasp_ros")) / "config"
    stages, config_evidence = _load_stages(config_dir, args)
    node = CandidatePlanOnlyProbe()
    started = time.monotonic()
    report: dict[str, object] = {
        "schema_version": 1,
        "capability": "chained_moveit_get_motion_plan_no_execute",
        "label": args.label,
        "recorded_at": datetime.now().astimezone().isoformat(),
        "planning_frame": PLANNING_FRAME,
        "planning_group": "arm",
        "pipeline_id": args.pipeline_id,
        "planner_id": args.planner_id,
        "attempts_requested": args.attempts,
        "target_center_x_offset_m": args.target_center_x_offset_m,
        "config": config_evidence,
        "segments": [],
        "trajectory_publication_count": 0,
        "execute_trajectory_goal_count": 0,
        "fjt_goal_count": 0,
        "execution_attempted": False,
        "contact_policy": (
            "approach_and_descend_disallowed_lift_allowed_after_close"
        ),
        "contact_policy_events": [],
    }
    try:
        node.wait_for_plan_service(args.service_timeout_s)
        graph_before = node.graph_boundary()
        report["graph_before"] = graph_before
        if not graph_before["motion_boundary_absent"]:
            raise RuntimeError("motion nodes or ExecuteTrajectory services are present")
        arm_start, actual_gripper, joint_stamp_ns, receive_now_ns = node.fresh_start(
            timeout_s=args.joint_state_timeout_s,
            max_age_ms=args.max_joint_state_age_ms,
        )
        report["initial_joint_state"] = {
            "joint_names": [*SO101_ARM_JOINTS, GRIPPER_JOINT],
            "arm_positions_rad": list(arm_start),
            "gripper_position_rad": actual_gripper,
            "source_timestamp_ns": joint_stamp_ns,
            "receive_now_ns": receive_now_ns,
            "age_ns": receive_now_ns - joint_stamp_ns,
        }
        rows: list[dict[str, object]] = []
        all_accepted = True
        for attempt in range(1, args.attempts + 1):
            chained_start = arm_start
            for stage in stages:
                allow_contacts = stage.allow_target_pad_contacts
                policy_status = node.set_target_pad_contacts(
                    allow=allow_contacts,
                    timeout_s=args.service_timeout_s,
                    scene_digest=str(config_evidence["scene_digest"]),
                )
                report["contact_policy_events"].append(
                    {
                        "attempt": attempt,
                        "segment": stage.name,
                        "allow_target_pad_contacts": allow_contacts,
                        "confirmed": True,
                        "reason": policy_status["reason"],
                    }
                )
                row, terminal = node.plan_stage(
                    attempt=attempt,
                    stage=stage,
                    start_positions=chained_start,
                    args=args,
                )
                row["allow_target_pad_contacts"] = allow_contacts
                rows.append(row)
                if not row["plan_accepted"] or terminal is None:
                    all_accepted = False
                    break
                chained_start = terminal
            if not all_accepted:
                break
        graph_after = node.graph_boundary()
        report["segments"] = rows
        report["attempts_completed"] = sum(
            1
            for attempt in range(1, args.attempts + 1)
            if sum(row["attempt"] == attempt for row in rows) == len(stages)
            and all(row["plan_accepted"] for row in rows if row["attempt"] == attempt)
        )
        report["all_segments_accepted"] = (
            all_accepted
            and len(rows) == args.attempts * len(stages)
            and all(row["plan_accepted"] for row in rows)
        )
        report["graph_after"] = graph_after
        report["motion_boundary_absent_throughout"] = (
            bool(graph_before["motion_boundary_absent"])
            and bool(graph_after["motion_boundary_absent"])
        )
        report["elapsed_wall_s"] = time.monotonic() - started
        report["status"] = (
            "PLAN_ONLY_PASS"
            if report["all_segments_accepted"]
            and report["motion_boundary_absent_throughout"]
            else "PLAN_ONLY_REJECTED"
        )
        print(json.dumps(report, indent=2, sort_keys=True), flush=True)
        return 0 if report["status"] == "PLAN_ONLY_PASS" else 2
    except Exception as error:
        report["status"] = "PLAN_ONLY_UNVERIFIED"
        report["error"] = f"{type(error).__name__}:{error}"
        report["elapsed_wall_s"] = time.monotonic() - started
        print(json.dumps(report, indent=2, sort_keys=True), flush=True)
        return 3
    finally:
        node.destroy_node()


def main() -> int:
    non_ros_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    args = _arguments(non_ros_args)
    rclpy.init(args=sys.argv)
    try:
        return _run(args)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
