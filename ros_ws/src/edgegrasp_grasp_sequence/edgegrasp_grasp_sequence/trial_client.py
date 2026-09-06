"""One-shot correlated sequence plus independent physics-evidence client.

The client sends no trajectory itself.  It snapshots one ``TrackedTarget``,
starts the read-only physics observer, waits for a cube-pose baseline, and only
then submits the exact same target snapshot to ``GraspSequence``.  Sequence and
physics results remain separate in the emitted JSON record.
"""

from __future__ import annotations

from functools import partial
import json
import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from edgegrasp.config import ROS_SIM_CLOCK_DOMAIN, validate_ros_clock_domain
from edgegrasp.grasp_geometry import (
    derive_grasp_stage_geometry,
    load_grasp_geometry_profile,
    validate_routed_grasp_stage_geometry,
)
from edgegrasp.scene import load_scene_contract
from edgegrasp.trajectory_identity import make_trajectory_command_id
from edgegrasp_interfaces.action import GraspPhysicsEvidence, GraspSequence
from edgegrasp_interfaces.msg import TrackedTarget
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


def _duration(seconds: float) -> Duration:
    if not math.isfinite(seconds) or seconds <= 0.0:
        raise ValueError("duration must be finite and positive")
    whole = int(seconds)
    message = Duration()
    message.sec = whole
    message.nanosec = int(round((seconds - whole) * 1_000_000_000.0))
    if message.nanosec == 1_000_000_000:
        message.sec += 1
        message.nanosec = 0
    return message


def _config_basename(
    value: object, parameter: str = "grasp_geometry_filename"
) -> str:
    if not isinstance(value, str) or not value or Path(value).name != value:
        raise ValueError(f"{parameter} must be one config basename")
    if Path(value).suffix != ".json":
        raise ValueError(f"{parameter} must name a JSON config")
    return value


class GraspTrialClient(Node):
    """Coordinate two typed actions around one immutable target snapshot."""

    def __init__(self) -> None:
        super().__init__("edgegrasp_grasp_trial_client")
        self.declare_parameter("sequence_action", "/edgegrasp/grasp_sequence")
        self.declare_parameter(
            "physics_action", "/edgegrasp/grasp_physics_evidence"
        )
        self.declare_parameter("tracked_target_topic", "/edgegrasp/tracked_target")
        self.declare_parameter("task_id", "grasp-physics-trial-001")
        self.declare_parameter("target_id", "target_cube")
        self.declare_parameter("clock_domain", ROS_SIM_CLOCK_DOMAIN)
        self.declare_parameter("clock_epoch", 0)
        self.declare_parameter("server_timeout_s", 10.0)
        self.declare_parameter("target_timeout_s", 5.0)
        self.declare_parameter("baseline_timeout_s", 5.0)
        self.declare_parameter("sequence_timeout_s", 60.0)
        self.declare_parameter("physics_timeout_s", 70.0)
        self.declare_parameter("cancel_timeout_s", 3.0)
        # Candidate024 measured 62 ms on a successful run and 181 ms on the
        # stale failure.  A 100 ms pre-send bound rejects the latter while
        # preserving at least 100 ms for planning before the core's fixed
        # 200 ms source-freshness deadline.
        self.declare_parameter("max_target_age_before_send_ms", 100.0)
        self.declare_parameter("max_baseline_restarts", 2)
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
        self.declare_parameter(
            "grasp_geometry_filename", "so101_grasp_geometry.json"
        )
        self.declare_parameter("scene_config_filename", "scene.json")
        self.declare_parameter("derive_descend_and_lift_from_profile", False)
        self.declare_parameter("pipeline_id", "pilz_industrial_motion_planner")
        self.declare_parameter("planner_id", "PTP")
        self.declare_parameter("planning_timeout_s", 2.0)
        self.declare_parameter("velocity_scaling", 0.1)
        self.declare_parameter("acceleration_scaling", 0.1)
        self.declare_parameter("cube_model_name", "target_cube")
        self.declare_parameter("baseline_sample_count", 5)
        self.declare_parameter("minimum_lift_m", 0.02)
        self.declare_parameter("minimum_clearance_m", 0.01)
        self.declare_parameter("retention_duration_s", 0.5)
        self.declare_parameter("maximum_xy_drift_m", 0.01)
        self.declare_parameter("freshness_timeout_s", 0.2)
        self.declare_parameter("observation_timeout_s", 30.0)

        self._clock_domain = str(self.get_parameter("clock_domain").value)
        self._clock_epoch = int(self.get_parameter("clock_epoch").value)
        validate_ros_clock_domain(
            bool(self.get_parameter("use_sim_time").value), self._clock_domain
        )
        if self._clock_epoch < 0:
            raise ValueError("clock_epoch must be non-negative")
        self._target_id = str(self.get_parameter("target_id").value)
        self._latest: TrackedTarget | None = None
        self._physics_phase = ""
        self._physics_reason = ""
        self._physics_attempt_generation = 0
        self.create_subscription(
            TrackedTarget,
            str(self.get_parameter("tracked_target_topic").value),
            self._on_target,
            1,
        )
        self._sequence = ActionClient(
            self,
            GraspSequence,
            str(self.get_parameter("sequence_action").value),
        )
        self._physics = ActionClient(
            self,
            GraspPhysicsEvidence,
            str(self.get_parameter("physics_action").value),
        )
        config_dir = (
            Path(get_package_share_directory("edgegrasp_ros")) / "config"
        )
        self._scene_config_filename = _config_basename(
            self.get_parameter("scene_config_filename").value,
            "scene_config_filename",
        )
        scene_path = config_dir / self._scene_config_filename
        if not scene_path.is_file():
            raise ValueError(
                "scene config is not installed: "
                f"{self._scene_config_filename}"
            )
        self._scene = load_scene_contract(scene_path)
        self._grasp_geometry_filename = _config_basename(
            self.get_parameter("grasp_geometry_filename").value
        )
        grasp_profile_path = config_dir / self._grasp_geometry_filename
        if not grasp_profile_path.is_file():
            raise ValueError(
                "grasp geometry config is not installed: "
                f"{self._grasp_geometry_filename}"
            )
        self._grasp_profile = load_grasp_geometry_profile(grasp_profile_path)
        cube = next(
            (
                item
                for item in self._scene.objects
                if item.object_id == self._grasp_profile.target_id
            ),
            None,
        )
        if cube is None or cube.size_m != self._grasp_profile.cube_size_m:
            raise ValueError("grasp profile and scene target cube disagree")
        if self._target_id != self._grasp_profile.target_id:
            raise ValueError("grasp trial target_id must match the grasp profile")
        self._cube = cube

    def _on_target(self, message: TrackedTarget) -> None:
        if message.target_id != self._target_id:
            return
        if (
            message.clock_domain != self._clock_domain
            or int(message.clock_epoch) != self._clock_epoch
        ):
            return
        self._latest = message

    def _positive(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return value

    def _position(self, name: str) -> tuple[float, float, float]:
        values = tuple(float(value) for value in self.get_parameter(name).value)
        if len(values) != 3 or any(not math.isfinite(value) for value in values):
            raise ValueError(f"{name} must contain three finite values")
        return values

    def _orientation(self, parameter: str) -> tuple[float, float, float, float]:
        values = tuple(
            float(value)
            for value in self.get_parameter(parameter).value
        )
        if len(values) != 4 or any(not math.isfinite(value) for value in values):
            raise ValueError(f"{parameter} must contain four finite values")
        norm = math.sqrt(sum(value * value for value in values))
        if abs(norm - 1.0) > 1e-3:
            raise ValueError(f"{parameter} must be normalized")
        return values

    def _resolved_stage_positions(
        self, target: TrackedTarget,
    ) -> tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ]:
        """Resolve one immutable position triple for validation and dispatch."""

        approach = self._position("approach_position_m")
        derive_from_profile = self.get_parameter(
            "derive_descend_and_lift_from_profile"
        ).value
        if not isinstance(derive_from_profile, bool):
            raise ValueError("derive_descend_and_lift_from_profile must be boolean")
        if not derive_from_profile:
            return (
                approach,
                self._position("descend_position_m"),
                self._position("lift_position_m"),
            )
        derived = derive_grasp_stage_geometry(
            (target.observation.point.x, target.observation.point.y, target.observation.point.z),
            self._orientation("grasp_orientation_xyzw"),
            self._grasp_profile,
        )
        return approach, derived.descend_position_m, derived.lift_position_m

    @staticmethod
    def _stamp_ns(stamp) -> int:
        sec = int(stamp.sec)
        nanosec = int(stamp.nanosec)
        if sec < 0 or not 0 <= nanosec < 1_000_000_000:
            raise ValueError("target contains an invalid ROS timestamp")
        return sec * 1_000_000_000 + nanosec

    def _target_age_ns(self, target: TrackedTarget) -> int:
        source_ns = self._stamp_ns(target.observation.header.stamp)
        now_ns = int(self.get_clock().now().nanoseconds)
        return now_ns - source_ns

    def _target_age_is_recent(self, age_ns: int) -> bool:
        limit_ms = self._positive("max_target_age_before_send_ms")
        if limit_ms >= 200.0:
            raise ValueError("max_target_age_before_send_ms must be below 200")
        return 0 <= age_ns <= int(limit_ms * 1_000_000.0)

    def _target_is_recent(self, target: TrackedTarget) -> bool:
        return self._target_age_is_recent(self._target_age_ns(target))

    def _latest_is_recent(self) -> bool:
        return self._latest is not None and self._target_is_recent(self._latest)

    def _nonnegative_integer(self, name: str) -> int:
        value = self.get_parameter(name).value
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError(f"{name} must be a non-negative integer")
        return value

    def _wait_future(self, future, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.done()

    def _wait_target(self) -> TrackedTarget:
        self._latest = None
        deadline = time.monotonic() + self._positive("target_timeout_s")
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._latest_is_recent():
                assert self._latest is not None
                return self._latest
        raise TimeoutError("fresh matching TrackedTarget was not observed")

    def _sequence_goal(self, target: TrackedTarget) -> GraspSequence.Goal:
        goal = GraspSequence.Goal()
        goal.task_id = str(self.get_parameter("task_id").value)
        goal.target = target
        stage_positions = self._resolved_stage_positions(target)
        for field, position in zip(
            (
                goal.approach_position,
                goal.descend_position,
                goal.lift_position,
            ),
            stage_positions,
            strict=True,
        ):
            field.x, field.y, field.z = position
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
        goal.planning_timeout_s = self._positive("planning_timeout_s")
        goal.velocity_scaling = float(self.get_parameter("velocity_scaling").value)
        goal.acceleration_scaling = float(
            self.get_parameter("acceleration_scaling").value
        )
        return goal

    def _validate_physics_trial_geometry(self, target: TrackedTarget) -> None:
        target_point = target.observation.point
        observed_target = (
            float(target_point.x),
            float(target_point.y),
            float(target_point.z),
        )
        if any(
            abs(observed - expected) > self._grasp_profile.stage_position_tolerance_m
            for observed, expected in zip(
                observed_target, self._cube.pose_world.position_m, strict=True
            )
        ):
            raise ValueError("tracked target and scene cube pose disagree")
        approach_position, descend_position, lift_position = (
            self._resolved_stage_positions(target)
        )
        geometry = validate_routed_grasp_stage_geometry(
            cube_center_m=self._cube.pose_world.position_m,
            approach_position_m=approach_position,
            descend_position_m=descend_position,
            lift_position_m=lift_position,
            approach_orientation_xyzw=self._orientation(
                "approach_orientation_xyzw"
            ),
            grasp_orientation_xyzw=self._orientation(
                "grasp_orientation_xyzw"
            ),
            gripper_position_rad=float(
                self.get_parameter("gripper_closed_position_rad").value
            ),
            profile=self._grasp_profile,
            cube_orientation_xyzw=self._cube.pose_world.quaternion_xyzw,
        )
        print(
            json.dumps(
                {
                    "type": "grasp_geometry_preflight",
                    "target_id": target.target_id,
                    "end_effector_frame": self._grasp_profile.end_effector_frame,
                    "grasp_geometry_filename": self._grasp_geometry_filename,
                    "cube_center_in_frame_at_descend_m": (
                        geometry.cube_center_in_frame_at_descend_m
                    ),
                    "gripper_contact_position_rad": (
                        self._grasp_profile.gripper_contact_position_rad
                    ),
                    "derive_descend_and_lift_from_profile": bool(
                        self.get_parameter(
                            "derive_descend_and_lift_from_profile"
                        ).value
                    ),
                    "approach_position_m": approach_position,
                    "descend_position_m": descend_position,
                    "lift_position_m": lift_position,
                    "status": "PASS",
                },
                sort_keys=True,
            ),
            flush=True,
        )

    def _physics_goal(
        self, target: TrackedTarget, task_id: str
    ) -> GraspPhysicsEvidence.Goal:
        goal = GraspPhysicsEvidence.Goal()
        goal.task_id = task_id
        goal.target_id = target.target_id
        goal.target_source_timestamp_ns = self._stamp_ns(
            target.observation.header.stamp
        )
        goal.cube_model_name = str(self.get_parameter("cube_model_name").value)
        goal.pose_frame_id = self._scene.name
        goal.expected_lift_command_id = make_trajectory_command_id(
            task_id, "lift", 3
        )
        goal.scene_digest = self._scene.digest
        goal.started_at = self.get_clock().now().to_msg()
        goal.clock_domain = self._clock_domain
        goal.clock_epoch = self._clock_epoch
        goal.baseline_sample_count = int(
            self.get_parameter("baseline_sample_count").value
        )
        goal.min_lift_m = self._positive("minimum_lift_m")
        goal.min_clearance_m = float(
            self.get_parameter("minimum_clearance_m").value
        )
        goal.retention_duration = _duration(
            self._positive("retention_duration_s")
        )
        goal.max_xy_drift_m = float(
            self.get_parameter("maximum_xy_drift_m").value
        )
        goal.freshness_timeout = _duration(self._positive("freshness_timeout_s"))
        goal.observation_timeout = _duration(
            self._positive("observation_timeout_s")
        )
        return goal

    def _physics_feedback(self, wrapped, *, generation: int) -> None:
        feedback = wrapped.feedback
        if generation != self._physics_attempt_generation:
            print(
                json.dumps(
                    {
                        "type": "physics_feedback_ignored",
                        "task_id": feedback.task_id,
                        "generation": generation,
                        "active_generation": self._physics_attempt_generation,
                        "reason": "late_observation_attempt",
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
            return
        self._physics_phase = feedback.phase
        self._physics_reason = feedback.reason
        print(
            json.dumps(
                {
                    "type": "physics_feedback",
                    "task_id": feedback.task_id,
                    "phase": feedback.phase,
                    "reason": feedback.reason,
                },
                sort_keys=True,
            ),
            flush=True,
        )

    @staticmethod
    def _sequence_feedback(wrapped) -> None:
        feedback = wrapped.feedback
        print(
            json.dumps(
                {
                    "type": "sequence_feedback",
                    "task_id": feedback.task_id,
                    "phase": feedback.phase,
                    "command_id": feedback.command_id,
                    "reason": feedback.reason,
                },
                sort_keys=True,
            ),
            flush=True,
        )

    def _cancel_and_confirm(
        self,
        goal_handle,
        result_future,
        label: str,
        *,
        expected_physics_restart: tuple[str, str, int, str, int] | None = None,
    ) -> bool:
        """Request cancellation and report whether a terminal result was observed."""

        record = {
            "type": "grasp_trial_cleanup",
            "label": label,
            "cancel_requested": False,
            "cancel_accepted": False,
            "terminal_observed": False,
            "physics_restart_terminal_confirmed": False,
            "wrapper_status": GoalStatus.STATUS_UNKNOWN,
            "reason": "goal_handle_unavailable",
        }
        if goal_handle is None:
            print(json.dumps(record, sort_keys=True), flush=True)
            return False
        timeout_s = self._positive("cancel_timeout_s")
        try:
            cancel_future = goal_handle.cancel_goal_async()
            record["cancel_requested"] = True
        except BaseException as error:
            record["reason"] = f"cancel_request_exception:{type(error).__name__}"
            print(json.dumps(record, sort_keys=True), flush=True)
            return False
        if not self._wait_future(cancel_future, timeout_s):
            record["reason"] = "cancel_response_timeout"
            print(json.dumps(record, sort_keys=True), flush=True)
            return False
        try:
            cancel_response = cancel_future.result()
            record["cancel_accepted"] = bool(cancel_response.goals_canceling)
        except BaseException as error:
            record["reason"] = f"cancel_response_exception:{type(error).__name__}"
            print(json.dumps(record, sort_keys=True), flush=True)
            return False

        if result_future is not None and self._wait_future(result_future, timeout_s):
            try:
                wrapped = result_future.result()
                status = int(wrapped.status)
                record["wrapper_status"] = status
                record["terminal_observed"] = status in {
                    GoalStatus.STATUS_SUCCEEDED,
                    GoalStatus.STATUS_CANCELED,
                    GoalStatus.STATUS_ABORTED,
                }
                record["reason"] = (
                    "terminal_observed"
                    if record["terminal_observed"]
                    else "nonterminal_wrapper_status"
                )
                if expected_physics_restart is not None:
                    (
                        expected_task,
                        expected_target,
                        expected_source,
                        expected_domain,
                        expected_epoch,
                    ) = expected_physics_restart
                    result = wrapped.result
                    restart_confirmed = bool(
                        record["cancel_accepted"]
                        and status == GoalStatus.STATUS_CANCELED
                        and result.accepted
                        and result.terminal_phase == "FAULT"
                        and result.reason == "observer_cancelled"
                        and result.task_id == expected_task
                        and result.target_id == expected_target
                        and int(result.target_source_timestamp_ns)
                        == expected_source
                        and result.clock_domain == expected_domain
                        and int(result.clock_epoch) == expected_epoch
                    )
                    record["physics_restart_terminal_confirmed"] = (
                        restart_confirmed
                    )
                    record["observed_task_id"] = result.task_id
                    record["observed_target_id"] = result.target_id
                    record["observed_target_source_timestamp_ns"] = int(
                        result.target_source_timestamp_ns
                    )
                    record["observed_reason"] = result.reason
                    if not restart_confirmed:
                        record["reason"] = "physics_restart_terminal_mismatch"
            except BaseException as error:
                record["reason"] = (
                    f"result_after_cancel_exception:{type(error).__name__}"
                )
        else:
            record["reason"] = (
                "cancel_accepted_terminal_unconfirmed"
                if record["cancel_accepted"]
                else "cancel_rejected_terminal_unconfirmed"
            )
        print(json.dumps(record, sort_keys=True), flush=True)
        if expected_physics_restart is not None:
            return bool(record["physics_restart_terminal_confirmed"])
        return bool(record["terminal_observed"])

    def run(self) -> int:
        server_timeout = self._positive("server_timeout_s")
        if not self._physics.wait_for_server(timeout_sec=server_timeout):
            print('{"error":"physics_action_unavailable"}', flush=True)
            return 3
        if not self._sequence.wait_for_server(timeout_sec=server_timeout):
            print('{"error":"sequence_action_unavailable"}', flush=True)
            return 3

        task_id = str(self.get_parameter("task_id").value)
        max_restarts = self._nonnegative_integer("max_baseline_restarts")
        target = None
        physics_handle = None
        physics_result_future = None
        ready_phases = {"WAIT_CONTACT", "WAIT_LIFT", "RETENTION", "VERIFIED"}
        for baseline_attempt in range(max_restarts + 1):
            target = self._wait_target()
            self._validate_physics_trial_geometry(target)
            self._physics_attempt_generation += 1
            attempt_generation = self._physics_attempt_generation
            self._physics_phase = ""
            self._physics_reason = ""
            physics_send = self._physics.send_goal_async(
                self._physics_goal(target, task_id),
                feedback_callback=partial(
                    self._physics_feedback,
                    generation=attempt_generation,
                ),
            )
            if not self._wait_future(physics_send, server_timeout):
                print('{"error":"physics_goal_response_timeout"}', flush=True)
                return 4
            try:
                physics_handle = physics_send.result()
            except BaseException as error:
                print(
                    json.dumps(
                        {
                            "error": "physics_goal_response_exception",
                            "exception_type": type(error).__name__,
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )
                return 5
            if physics_handle is None or not physics_handle.accepted:
                print('{"error":"physics_goal_rejected"}', flush=True)
                return 5
            physics_result_future = physics_handle.get_result_async()

            baseline_deadline = time.monotonic() + self._positive(
                "baseline_timeout_s"
            )
            while (
                self._physics_phase not in ready_phases
                and not physics_result_future.done()
                and time.monotonic() < baseline_deadline
            ):
                rclpy.spin_once(self, timeout_sec=0.05)
            if self._physics_phase not in ready_phases:
                self._cancel_and_confirm(
                    physics_handle,
                    physics_result_future,
                    "physics_baseline_failure",
                )
                print(
                    json.dumps(
                        {
                            "error": "physics_baseline_unavailable",
                            "phase": self._physics_phase,
                            "reason": self._physics_reason,
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )
                return 6

            target_age_ns = self._target_age_ns(target)
            if self._target_age_is_recent(target_age_ns):
                print(
                    json.dumps(
                        {
                            "type": "physics_baseline_ready",
                            "baseline_attempt": baseline_attempt + 1,
                            "generation": attempt_generation,
                            "target_age_ns": target_age_ns,
                            "target_source_timestamp_ns": self._stamp_ns(
                                target.observation.header.stamp
                            ),
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )
                break

            terminal_confirmed = self._cancel_and_confirm(
                physics_handle,
                physics_result_future,
                "physics_baseline_target_stale",
                expected_physics_restart=(
                    task_id,
                    target.target_id,
                    self._stamp_ns(target.observation.header.stamp),
                    self._clock_domain,
                    self._clock_epoch,
                ),
            )
            print(
                json.dumps(
                    {
                        "type": "physics_baseline_restart",
                        "baseline_attempt": baseline_attempt + 1,
                        "generation": attempt_generation,
                        "target_age_ns": target_age_ns,
                        "terminal_confirmed": terminal_confirmed,
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
            if not terminal_confirmed:
                print(
                    '{"error":"physics_baseline_restart_terminal_unconfirmed"}',
                    flush=True,
                )
                return 6
            if baseline_attempt == max_restarts:
                print(
                    '{"error":"physics_baseline_target_stale_retries_exhausted"}',
                    flush=True,
                )
                return 6

        assert target is not None
        assert physics_handle is not None
        assert physics_result_future is not None

        sequence_send = self._sequence.send_goal_async(
            self._sequence_goal(target),
            feedback_callback=self._sequence_feedback,
        )
        if not self._wait_future(sequence_send, server_timeout):
            self._cancel_and_confirm(
                physics_handle, physics_result_future, "sequence_goal_response_timeout"
            )
            print('{"error":"sequence_goal_response_timeout"}', flush=True)
            return 7
        try:
            sequence_handle = sequence_send.result()
        except BaseException as error:
            self._cancel_and_confirm(
                physics_handle,
                physics_result_future,
                "sequence_goal_response_exception",
            )
            print(
                json.dumps(
                    {
                        "error": "sequence_goal_response_exception",
                        "exception_type": type(error).__name__,
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
            return 8
        if sequence_handle is None or not sequence_handle.accepted:
            self._cancel_and_confirm(
                physics_handle, physics_result_future, "sequence_goal_rejected"
            )
            print('{"error":"sequence_goal_rejected"}', flush=True)
            return 8
        sequence_result_future = sequence_handle.get_result_async()
        if not self._wait_future(
            sequence_result_future, self._positive("sequence_timeout_s")
        ):
            self._cancel_and_confirm(
                sequence_handle, sequence_result_future, "sequence_result_timeout"
            )
            self._cancel_and_confirm(
                physics_handle, physics_result_future, "sequence_result_timeout"
            )
            print('{"error":"sequence_result_timeout_cancel_requested"}', flush=True)
            return 9
        try:
            sequence_wrapped = sequence_result_future.result()
        except BaseException as error:
            self._cancel_and_confirm(
                physics_handle, physics_result_future, "sequence_result_exception"
            )
            print(
                json.dumps(
                    {
                        "error": "sequence_result_exception",
                        "exception_type": type(error).__name__,
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
            return 9
        if not self._wait_future(
            physics_result_future, self._positive("physics_timeout_s")
        ):
            self._cancel_and_confirm(
                physics_handle, physics_result_future, "physics_result_timeout"
            )
            print('{"error":"physics_result_timeout_cancel_requested"}', flush=True)
            return 10
        try:
            physics_wrapped = physics_result_future.result()
        except BaseException as error:
            print(
                json.dumps(
                    {
                        "error": "physics_result_exception",
                        "exception_type": type(error).__name__,
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
            return 10

        sequence_result = sequence_wrapped.result
        physics_result = physics_wrapped.result
        record = {
            "type": "grasp_trial_result",
            "task_id": task_id,
            "target_id": target.target_id,
            "target_source_timestamp_ns": self._stamp_ns(
                target.observation.header.stamp
            ),
            "sequence_wrapper_status": int(sequence_wrapped.status),
            "sequence_completed": bool(sequence_result.sequence_completed),
            "sequence_phase": sequence_result.terminal_phase,
            "sequence_reason": sequence_result.reason,
            "last_terminal_command_id": sequence_result.last_terminal_command_id,
            "last_trajectory_digest": sequence_result.last_trajectory_digest,
            "physics_wrapper_status": int(physics_wrapped.status),
            "physics_phase": physics_result.terminal_phase,
            "physics_reason": physics_result.reason,
            "gripper_contact_observed": bool(
                physics_result.gripper_contact_observed
            ),
            "lift_observed": bool(physics_result.lift_observed),
            "retention_observed": bool(physics_result.retention_observed),
            "physics_grasp_verified": bool(
                physics_result.physics_grasp_verified
            ),
            "matched_gripper_collision1": (
                physics_result.matched_gripper_collision1
            ),
            "matched_gripper_collision2": (
                physics_result.matched_gripper_collision2
            ),
            "baseline_z_m": (
                physics_result.baseline_z_m
                if physics_result.baseline_available
                else None
            ),
            "peak_z_m": (
                physics_result.peak_z_m if physics_result.peak_available else None
            ),
            "final_z_m": (
                physics_result.final_z_m
                if physics_result.final_pose_available
                else None
            ),
            "pose_sample_count": int(physics_result.pose_sample_count),
            "gripper_contact_count": int(physics_result.gripper_contact_count),
            "gripper_contact_counts_by_token": {
                token: int(count)
                for token, count in zip(
                    physics_result.gripper_contact_tokens,
                    physics_result.gripper_contact_counts_by_token,
                    strict=True,
                )
            },
            "gripper_contact_quality_by_token": {
                token: {
                    "max_penetration_depth_m": (
                        float(depth) if depth_available else None
                    ),
                    "max_force_magnitude_n": (
                        float(force) if force_available else None
                    ),
                    "max_abs_normal_force_n": (
                        float(normal_force) if normal_force_available else None
                    ),
                }
                for (
                    token,
                    depth_available,
                    depth,
                    force_available,
                    force,
                    normal_force_available,
                    normal_force,
                ) in zip(
                    physics_result.gripper_contact_tokens,
                    physics_result.gripper_penetration_depth_available_by_token,
                    physics_result.gripper_max_penetration_depth_m_by_token,
                    physics_result.gripper_force_magnitude_available_by_token,
                    physics_result.gripper_max_force_magnitude_n_by_token,
                    physics_result.gripper_normal_force_available_by_token,
                    physics_result.gripper_max_abs_normal_force_n_by_token,
                    strict=True,
                )
            },
            "gripper_contact_timing_by_token": {
                token: {
                    "first_source_timestamp_ns": (
                        int(first_stamp) if first_available else None
                    ),
                    "last_source_timestamp_ns": (
                        int(last_stamp) if last_available else None
                    ),
                }
                for (
                    token,
                    first_available,
                    first_stamp,
                    last_available,
                    last_stamp,
                ) in zip(
                    physics_result.gripper_contact_tokens,
                    physics_result.gripper_first_contact_stamp_available_by_token,
                    physics_result.gripper_first_contact_source_timestamp_ns_by_token,
                    physics_result.gripper_last_contact_stamp_available_by_token,
                    physics_result.gripper_last_contact_source_timestamp_ns_by_token,
                    strict=True,
                )
            },
            "all_gripper_tokens_observed": bool(
                physics_result.all_gripper_tokens_observed
            ),
            "simultaneous_gripper_contact_sample_count": int(
                physics_result.simultaneous_gripper_contact_sample_count
            ),
            "simultaneous_contact_quality": {
                "max_min_penetration_depth_m": (
                    float(physics_result.simultaneous_max_min_penetration_depth_m)
                    if physics_result.simultaneous_min_penetration_depth_available
                    else None
                ),
                "max_min_force_magnitude_n": (
                    float(physics_result.simultaneous_max_min_force_magnitude_n)
                    if physics_result.simultaneous_min_force_magnitude_available
                    else None
                ),
                "max_min_abs_normal_force_n": (
                    float(physics_result.simultaneous_max_min_abs_normal_force_n)
                    if physics_result.simultaneous_min_normal_force_available
                    else None
                ),
            },
            "simultaneous_contact_timing": {
                "first_source_timestamp_ns": (
                    int(physics_result.first_simultaneous_contact_source_timestamp_ns)
                    if physics_result.first_simultaneous_contact_stamp_available
                    else None
                ),
                "last_source_timestamp_ns": (
                    int(physics_result.last_simultaneous_contact_source_timestamp_ns)
                    if physics_result.last_simultaneous_contact_stamp_available
                    else None
                ),
                "sample_count_at_sequence_completion": int(
                    physics_result.simultaneous_gripper_contact_sample_count_at_sequence_completion
                ),
                "last_source_timestamp_ns_at_sequence_completion": (
                    int(
                        physics_result.last_simultaneous_contact_source_timestamp_ns_at_sequence_completion
                    )
                    if physics_result.last_simultaneous_contact_at_sequence_completion_available
                    else None
                ),
            },
            "gripper_effort": {
                "joint_name": physics_result.gripper_joint_name,
                "available": bool(physics_result.gripper_effort_available),
                "sample_count": int(physics_result.gripper_effort_sample_count),
                "latest": (
                    float(physics_result.latest_gripper_effort)
                    if physics_result.gripper_effort_available
                    else None
                ),
                "peak_abs": (
                    float(physics_result.peak_abs_gripper_effort)
                    if physics_result.gripper_effort_available
                    else None
                ),
                "at_sequence_completion": (
                    float(physics_result.gripper_effort_at_sequence_completion)
                    if physics_result.gripper_effort_at_sequence_completion_available
                    else None
                ),
                "peak_abs_at_sequence_completion": (
                    float(
                        physics_result.peak_abs_gripper_effort_at_sequence_completion
                    )
                    if physics_result.gripper_effort_at_sequence_completion_available
                    else None
                ),
            },
            "sequence_completed_source_timestamp_ns": (
                int(physics_result.sequence_completed_source_timestamp_ns)
                if physics_result.sequence_completion_stamp_available
                else None
            ),
            "table_contact_count": int(physics_result.table_contact_count),
            "scene_digest": physics_result.scene_digest,
            "evidence_digest": physics_result.evidence_digest,
        }
        print(json.dumps(record, sort_keys=True), flush=True)

        sequence_ok = (
            sequence_wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and sequence_result.sequence_completed
        )
        physics_ok = (
            physics_wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and physics_result.physics_grasp_verified
        )
        if sequence_ok and physics_ok:
            return 0
        if sequence_ok:
            return 11
        return 12

    def destroy_node(self) -> bool:
        self._sequence.destroy()
        self._physics.destroy()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GraspTrialClient()
    try:
        exit_code = node.run()
    except Exception as error:
        print(
            json.dumps(
                {"error": f"{type(error).__name__}:{error}", "status": "UNVERIFIED"},
                sort_keys=True,
            ),
            flush=True,
        )
        exit_code = 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
