"""Deterministic, dependency-free evidence gate for a simulated grasp.

This module observes outcomes; it never authorizes or sends motion.  A
successful arm/gripper command sequence is necessary but insufficient.  The
gate reports ``physics_grasp_verified`` only after correlated gripper/cube
contact, measured lift and clearance, and an uninterrupted retention window.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from hashlib import sha256
import json
from math import hypot, isfinite
import re
from statistics import median

from .config import ROS_SIM_CLOCK_DOMAIN


_DIGEST = re.compile(r"^[0-9a-f]{64}$")


class EvidencePhase(str, Enum):
    IDLE = "IDLE"
    BASELINE = "BASELINE"
    WAIT_CONTACT = "WAIT_CONTACT"
    WAIT_LIFT = "WAIT_LIFT"
    RETENTION = "RETENTION"
    VERIFIED = "VERIFIED"
    FAULT = "FAULT"


@dataclass(frozen=True, slots=True)
class PhysicsEvidenceTask:
    """Immutable identity, geometry and timing contract for one observation."""

    task_id: str
    target_id: str
    target_source_timestamp_ns: int
    expected_lift_command_id: str
    scene_digest: str
    cube_model_name: str
    cube_collision_token: str
    gripper_collision_tokens: tuple[str, ...]
    table_collision_tokens: tuple[str, ...]
    started_at_ns: int
    world_frame: str = "world"
    gripper_joint_name: str = "gripper"
    clock_domain: str = ROS_SIM_CLOCK_DOMAIN
    clock_epoch: int = 0
    table_top_z_m: float = 0.4
    cube_height_m: float = 0.05
    baseline_sample_count: int = 5
    min_lift_m: float = 0.02
    min_clearance_m: float = 0.01
    retention_ns: int = 500_000_000
    max_xy_drift_m: float = 0.01
    freshness_timeout_ns: int = 200_000_000

    def __post_init__(self) -> None:
        for name, value in (
            ("task_id", self.task_id),
            ("target_id", self.target_id),
            ("expected_lift_command_id", self.expected_lift_command_id),
            ("cube_model_name", self.cube_model_name),
            ("cube_collision_token", self.cube_collision_token),
            ("world_frame", self.world_frame),
            ("gripper_joint_name", self.gripper_joint_name),
            ("clock_domain", self.clock_domain),
        ):
            _require_text(value, name)
        if not _DIGEST.fullmatch(self.scene_digest):
            raise ValueError("scene_digest must be a lowercase SHA-256 digest")
        _require_tokens(self.gripper_collision_tokens, "gripper_collision_tokens")
        _require_tokens(self.table_collision_tokens, "table_collision_tokens")
        _require_ns(self.target_source_timestamp_ns, "target_source_timestamp_ns")
        _require_ns(self.started_at_ns, "started_at_ns")
        if self.target_source_timestamp_ns > self.started_at_ns:
            raise ValueError("target_source_timestamp_ns must not follow started_at_ns")
        _require_epoch(self.clock_epoch)
        _require_positive_int(self.baseline_sample_count, "baseline_sample_count")
        _require_positive_int(self.retention_ns, "retention_ns")
        _require_positive_int(self.freshness_timeout_ns, "freshness_timeout_ns")
        _require_finite(self.table_top_z_m, "table_top_z_m")
        _require_positive_float(self.cube_height_m, "cube_height_m")
        _require_positive_float(self.min_lift_m, "min_lift_m")
        _require_nonnegative_float(self.min_clearance_m, "min_clearance_m")
        _require_nonnegative_float(self.max_xy_drift_m, "max_xy_drift_m")


@dataclass(frozen=True, slots=True)
class CubePoseSample:
    model_name: str
    frame_id: str
    x_m: float
    y_m: float
    z_m: float
    source_timestamp_ns: int
    received_at_ns: int
    clock_domain: str
    clock_epoch: int

    def __post_init__(self) -> None:
        _require_text(self.model_name, "model_name")
        _require_text(self.frame_id, "frame_id")
        _require_text(self.clock_domain, "clock_domain")
        for name, value in (("x_m", self.x_m), ("y_m", self.y_m), ("z_m", self.z_m)):
            _require_finite(value, name)
        _require_ns(self.source_timestamp_ns, "source_timestamp_ns")
        _require_ns(self.received_at_ns, "received_at_ns")
        _require_epoch(self.clock_epoch)


@dataclass(frozen=True, slots=True)
class CollisionPair:
    collision1: str
    collision2: str
    max_penetration_depth_m: float | None = None
    max_force_magnitude_n: float | None = None
    max_abs_normal_force_n: float | None = None

    def __post_init__(self) -> None:
        _require_text(self.collision1, "collision1")
        _require_text(self.collision2, "collision2")
        for name, value in (
            ("max_penetration_depth_m", self.max_penetration_depth_m),
            ("max_force_magnitude_n", self.max_force_magnitude_n),
            ("max_abs_normal_force_n", self.max_abs_normal_force_n),
        ):
            if value is not None:
                _require_nonnegative_float(value, name)


@dataclass(frozen=True, slots=True)
class ContactSample:
    pairs: tuple[CollisionPair, ...]
    source_timestamp_ns: int
    received_at_ns: int
    clock_domain: str
    clock_epoch: int

    def __post_init__(self) -> None:
        if not isinstance(self.pairs, tuple):
            raise TypeError("pairs must be a tuple")
        if any(not isinstance(pair, CollisionPair) for pair in self.pairs):
            raise TypeError("pairs must contain CollisionPair values")
        _require_ns(self.source_timestamp_ns, "source_timestamp_ns")
        _require_ns(self.received_at_ns, "received_at_ns")
        _require_text(self.clock_domain, "clock_domain")
        _require_epoch(self.clock_epoch)


@dataclass(frozen=True, slots=True)
class GripperEffortSample:
    """One optional, diagnostic-only gripper effort observation."""

    joint_name: str
    effort: float
    source_timestamp_ns: int
    received_at_ns: int
    clock_domain: str
    clock_epoch: int

    def __post_init__(self) -> None:
        _require_text(self.joint_name, "joint_name")
        _require_finite(self.effort, "effort")
        _require_ns(self.source_timestamp_ns, "source_timestamp_ns")
        _require_ns(self.received_at_ns, "received_at_ns")
        _require_text(self.clock_domain, "clock_domain")
        _require_epoch(self.clock_epoch)


@dataclass(frozen=True, slots=True)
class SequenceCompletion:
    task_id: str
    target_id: str
    target_source_timestamp_ns: int
    lift_command_id: str
    sequence_completed: bool
    source_timestamp_ns: int
    received_at_ns: int
    clock_domain: str
    clock_epoch: int

    def __post_init__(self) -> None:
        _require_text(self.task_id, "task_id")
        _require_text(self.target_id, "target_id")
        _require_ns(self.target_source_timestamp_ns, "target_source_timestamp_ns")
        if not isinstance(self.sequence_completed, bool):
            raise TypeError("sequence_completed must be a boolean")
        if not isinstance(self.lift_command_id, str):
            raise TypeError("lift_command_id must be a string")
        if self.sequence_completed:
            _require_text(self.lift_command_id, "lift_command_id")
        _require_ns(self.source_timestamp_ns, "source_timestamp_ns")
        _require_ns(self.received_at_ns, "received_at_ns")
        _require_text(self.clock_domain, "clock_domain")
        _require_epoch(self.clock_epoch)


@dataclass(frozen=True, slots=True)
class PhysicsEvidenceDecision:
    phase: EvidencePhase
    physics_grasp_verified: bool
    reason: str


@dataclass(frozen=True, slots=True)
class PhysicsEvidenceResult:
    task_id: str
    target_id: str
    target_source_timestamp_ns: int
    expected_lift_command_id: str
    scene_digest: str
    cube_model_name: str
    pose_frame_id: str
    phase: EvidencePhase
    sequence_completed: bool
    gripper_contact_observed: bool
    lift_observed: bool
    retention_observed: bool
    physics_grasp_verified: bool
    reason: str
    matched_gripper_collision1: str | None
    matched_gripper_collision2: str | None
    baseline_z_m: float | None
    final_z_m: float | None
    peak_z_m: float | None
    retention_min_clearance_m: float | None
    max_xy_drift_m: float
    pose_sample_count: int
    gripper_contact_count: int
    gripper_contact_tokens: tuple[str, ...]
    gripper_contact_counts_by_token: tuple[int, ...]
    gripper_first_contact_source_timestamp_ns_by_token: tuple[int | None, ...]
    gripper_last_contact_source_timestamp_ns_by_token: tuple[int | None, ...]
    gripper_max_penetration_depth_m_by_token: tuple[float | None, ...]
    gripper_max_force_magnitude_n_by_token: tuple[float | None, ...]
    gripper_max_abs_normal_force_n_by_token: tuple[float | None, ...]
    all_gripper_tokens_observed: bool
    simultaneous_gripper_contact_sample_count: int
    first_simultaneous_gripper_contact_source_timestamp_ns: int | None
    last_simultaneous_gripper_contact_source_timestamp_ns: int | None
    simultaneous_gripper_contact_max_min_penetration_depth_m: float | None
    simultaneous_gripper_contact_max_min_force_magnitude_n: float | None
    simultaneous_gripper_contact_max_min_abs_normal_force_n: float | None
    gripper_joint_name: str
    gripper_effort_sample_count: int
    latest_gripper_effort: float | None
    peak_abs_gripper_effort: float | None
    sequence_completed_source_timestamp_ns: int | None
    simultaneous_gripper_contact_sample_count_at_sequence_completion: int
    last_simultaneous_contact_source_timestamp_ns_at_sequence_completion: int | None
    gripper_effort_at_sequence_completion: float | None
    peak_abs_gripper_effort_at_sequence_completion: float | None
    table_contact_count: int
    first_source_timestamp_ns: int | None
    last_source_timestamp_ns: int | None
    retention_started_at_ns: int | None
    retention_completed_at_ns: int | None
    evidence_started_at_ns: int
    evidence_completed_at_ns: int | None
    clock_domain: str
    clock_epoch: int

    @property
    def digest(self) -> str:
        """Stable audit digest; timings and all measured outcomes are covered."""

        payload = {
            "baseline_z_m": self.baseline_z_m,
            "clock_domain": self.clock_domain,
            "clock_epoch": self.clock_epoch,
            "cube_model_name": self.cube_model_name,
            "evidence_completed_at_ns": self.evidence_completed_at_ns,
            "evidence_started_at_ns": self.evidence_started_at_ns,
            "expected_lift_command_id": self.expected_lift_command_id,
            "final_z_m": self.final_z_m,
            "first_source_timestamp_ns": self.first_source_timestamp_ns,
            "gripper_contact_count": self.gripper_contact_count,
            "gripper_contact_counts_by_token": self.gripper_contact_counts_by_token,
            "gripper_first_contact_source_timestamp_ns_by_token": (
                self.gripper_first_contact_source_timestamp_ns_by_token
            ),
            "gripper_last_contact_source_timestamp_ns_by_token": (
                self.gripper_last_contact_source_timestamp_ns_by_token
            ),
            "gripper_max_abs_normal_force_n_by_token": (
                self.gripper_max_abs_normal_force_n_by_token
            ),
            "gripper_max_force_magnitude_n_by_token": (
                self.gripper_max_force_magnitude_n_by_token
            ),
            "gripper_max_penetration_depth_m_by_token": (
                self.gripper_max_penetration_depth_m_by_token
            ),
            "gripper_contact_tokens": self.gripper_contact_tokens,
            "gripper_contact_observed": self.gripper_contact_observed,
            "gripper_effort_sample_count": self.gripper_effort_sample_count,
            "gripper_joint_name": self.gripper_joint_name,
            "latest_gripper_effort": self.latest_gripper_effort,
            "peak_abs_gripper_effort": self.peak_abs_gripper_effort,
            "all_gripper_tokens_observed": self.all_gripper_tokens_observed,
            "simultaneous_gripper_contact_sample_count": (
                self.simultaneous_gripper_contact_sample_count
            ),
            "first_simultaneous_gripper_contact_source_timestamp_ns": (
                self.first_simultaneous_gripper_contact_source_timestamp_ns
            ),
            "last_simultaneous_gripper_contact_source_timestamp_ns": (
                self.last_simultaneous_gripper_contact_source_timestamp_ns
            ),
            "simultaneous_gripper_contact_max_min_abs_normal_force_n": (
                self.simultaneous_gripper_contact_max_min_abs_normal_force_n
            ),
            "simultaneous_gripper_contact_max_min_force_magnitude_n": (
                self.simultaneous_gripper_contact_max_min_force_magnitude_n
            ),
            "simultaneous_gripper_contact_max_min_penetration_depth_m": (
                self.simultaneous_gripper_contact_max_min_penetration_depth_m
            ),
            "sequence_completed_source_timestamp_ns": (
                self.sequence_completed_source_timestamp_ns
            ),
            "simultaneous_gripper_contact_sample_count_at_sequence_completion": (
                self.simultaneous_gripper_contact_sample_count_at_sequence_completion
            ),
            "last_simultaneous_contact_source_timestamp_ns_at_sequence_completion": (
                self.last_simultaneous_contact_source_timestamp_ns_at_sequence_completion
            ),
            "gripper_effort_at_sequence_completion": (
                self.gripper_effort_at_sequence_completion
            ),
            "peak_abs_gripper_effort_at_sequence_completion": (
                self.peak_abs_gripper_effort_at_sequence_completion
            ),
            "last_source_timestamp_ns": self.last_source_timestamp_ns,
            "max_xy_drift_m": self.max_xy_drift_m,
            "matched_gripper_collision1": self.matched_gripper_collision1,
            "matched_gripper_collision2": self.matched_gripper_collision2,
            "retention_min_clearance_m": self.retention_min_clearance_m,
            "peak_z_m": self.peak_z_m,
            "phase": self.phase.value,
            "physics_grasp_verified": self.physics_grasp_verified,
            "lift_observed": self.lift_observed,
            "pose_sample_count": self.pose_sample_count,
            "pose_frame_id": self.pose_frame_id,
            "reason": self.reason,
            "retention_observed": self.retention_observed,
            "retention_completed_at_ns": self.retention_completed_at_ns,
            "retention_started_at_ns": self.retention_started_at_ns,
            "scene_digest": self.scene_digest,
            "sequence_completed": self.sequence_completed,
            "table_contact_count": self.table_contact_count,
            "target_id": self.target_id,
            "target_source_timestamp_ns": self.target_source_timestamp_ns,
            "task_id": self.task_id,
        }
        encoded = json.dumps(
            payload,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
        return sha256(encoded).hexdigest()


class GraspPhysicsEvidenceController:
    """Fail-closed observer for contact, lift and retention evidence."""

    def __init__(self) -> None:
        self._task: PhysicsEvidenceTask | None = None
        self._phase = EvidencePhase.IDLE
        self._reason = "idle"
        self._last_now_ns: int | None = None
        self._required_epoch: int | None = None
        self._last_pose: CubePoseSample | None = None
        self._last_contact: ContactSample | None = None
        self._baseline_z: list[float] = []
        self._baseline_z_m: float | None = None
        self._peak_z_m: float | None = None
        self._retention_min_clearance_m: float | None = None
        self._retention_xy: tuple[float, float] | None = None
        self._max_xy_drift_m = 0.0
        self._retention_started_at_ns: int | None = None
        self._retention_completed_at_ns: int | None = None
        self._sequence_completed = False
        self._gripper_contact_seen = False
        self._lift_observed = False
        self._matched_gripper_pair: CollisionPair | None = None
        self._pose_sample_count = 0
        self._gripper_contact_count = 0
        self._gripper_contact_counts_by_token: dict[str, int] = {}
        self._gripper_first_contact_ns_by_token: dict[str, int | None] = {}
        self._gripper_last_contact_ns_by_token: dict[str, int | None] = {}
        self._gripper_max_depth_by_token: dict[str, float | None] = {}
        self._gripper_max_force_by_token: dict[str, float | None] = {}
        self._gripper_max_normal_force_by_token: dict[str, float | None] = {}
        self._last_all_gripper_tokens_received_ns: int | None = None
        self._simultaneous_gripper_contact_sample_count = 0
        self._first_simultaneous_contact_source_ns: int | None = None
        self._last_simultaneous_contact_source_ns: int | None = None
        self._simultaneous_max_min_depth_m: float | None = None
        self._simultaneous_max_min_force_n: float | None = None
        self._simultaneous_max_min_normal_force_n: float | None = None
        self._last_effort: GripperEffortSample | None = None
        self._gripper_effort_sample_count = 0
        self._peak_abs_gripper_effort: float | None = None
        self._sequence_completed_source_ns: int | None = None
        self._simultaneous_count_at_sequence_completion = 0
        self._last_simultaneous_ns_at_sequence_completion: int | None = None
        self._gripper_effort_at_sequence_completion: float | None = None
        self._peak_abs_effort_at_sequence_completion: float | None = None
        self._table_contact_count = 0
        self._first_source_timestamp_ns: int | None = None
        self._last_source_timestamp_ns: int | None = None

    @property
    def phase(self) -> EvidencePhase:
        return self._phase

    @property
    def result(self) -> PhysicsEvidenceResult | None:
        task = self._task
        if task is None:
            return None
        return PhysicsEvidenceResult(
            task_id=task.task_id,
            target_id=task.target_id,
            target_source_timestamp_ns=task.target_source_timestamp_ns,
            expected_lift_command_id=task.expected_lift_command_id,
            scene_digest=task.scene_digest,
            cube_model_name=task.cube_model_name,
            pose_frame_id=task.world_frame,
            phase=self._phase,
            sequence_completed=self._sequence_completed,
            gripper_contact_observed=self._gripper_contact_seen,
            lift_observed=self._lift_observed,
            retention_observed=self._phase is EvidencePhase.VERIFIED,
            physics_grasp_verified=self._phase is EvidencePhase.VERIFIED,
            reason=self._reason,
            matched_gripper_collision1=(
                None
                if self._matched_gripper_pair is None
                else self._matched_gripper_pair.collision1
            ),
            matched_gripper_collision2=(
                None
                if self._matched_gripper_pair is None
                else self._matched_gripper_pair.collision2
            ),
            baseline_z_m=self._baseline_z_m,
            final_z_m=None if self._last_pose is None else self._last_pose.z_m,
            peak_z_m=self._peak_z_m,
            retention_min_clearance_m=self._retention_min_clearance_m,
            max_xy_drift_m=self._max_xy_drift_m,
            pose_sample_count=self._pose_sample_count,
            gripper_contact_count=self._gripper_contact_count,
            gripper_contact_tokens=task.gripper_collision_tokens,
            gripper_contact_counts_by_token=tuple(
                self._gripper_contact_counts_by_token.get(token, 0)
                for token in task.gripper_collision_tokens
            ),
            gripper_first_contact_source_timestamp_ns_by_token=tuple(
                self._gripper_first_contact_ns_by_token.get(token)
                for token in task.gripper_collision_tokens
            ),
            gripper_last_contact_source_timestamp_ns_by_token=tuple(
                self._gripper_last_contact_ns_by_token.get(token)
                for token in task.gripper_collision_tokens
            ),
            gripper_max_penetration_depth_m_by_token=tuple(
                self._gripper_max_depth_by_token.get(token)
                for token in task.gripper_collision_tokens
            ),
            gripper_max_force_magnitude_n_by_token=tuple(
                self._gripper_max_force_by_token.get(token)
                for token in task.gripper_collision_tokens
            ),
            gripper_max_abs_normal_force_n_by_token=tuple(
                self._gripper_max_normal_force_by_token.get(token)
                for token in task.gripper_collision_tokens
            ),
            all_gripper_tokens_observed=all(
                self._gripper_contact_counts_by_token.get(token, 0) > 0
                for token in task.gripper_collision_tokens
            ),
            simultaneous_gripper_contact_sample_count=(
                self._simultaneous_gripper_contact_sample_count
            ),
            first_simultaneous_gripper_contact_source_timestamp_ns=(
                self._first_simultaneous_contact_source_ns
            ),
            last_simultaneous_gripper_contact_source_timestamp_ns=(
                self._last_simultaneous_contact_source_ns
            ),
            simultaneous_gripper_contact_max_min_penetration_depth_m=(
                self._simultaneous_max_min_depth_m
            ),
            simultaneous_gripper_contact_max_min_force_magnitude_n=(
                self._simultaneous_max_min_force_n
            ),
            simultaneous_gripper_contact_max_min_abs_normal_force_n=(
                self._simultaneous_max_min_normal_force_n
            ),
            gripper_joint_name=task.gripper_joint_name,
            gripper_effort_sample_count=self._gripper_effort_sample_count,
            latest_gripper_effort=(
                None if self._last_effort is None else self._last_effort.effort
            ),
            peak_abs_gripper_effort=self._peak_abs_gripper_effort,
            sequence_completed_source_timestamp_ns=(
                self._sequence_completed_source_ns
            ),
            simultaneous_gripper_contact_sample_count_at_sequence_completion=(
                self._simultaneous_count_at_sequence_completion
            ),
            last_simultaneous_contact_source_timestamp_ns_at_sequence_completion=(
                self._last_simultaneous_ns_at_sequence_completion
            ),
            gripper_effort_at_sequence_completion=(
                self._gripper_effort_at_sequence_completion
            ),
            peak_abs_gripper_effort_at_sequence_completion=(
                self._peak_abs_effort_at_sequence_completion
            ),
            table_contact_count=self._table_contact_count,
            first_source_timestamp_ns=self._first_source_timestamp_ns,
            last_source_timestamp_ns=self._last_source_timestamp_ns,
            retention_started_at_ns=self._retention_started_at_ns,
            retention_completed_at_ns=self._retention_completed_at_ns,
            evidence_started_at_ns=task.started_at_ns,
            evidence_completed_at_ns=(
                self._last_now_ns
                if self._phase in (EvidencePhase.VERIFIED, EvidencePhase.FAULT)
                else None
            ),
            clock_domain=task.clock_domain,
            clock_epoch=task.clock_epoch,
        )

    def start(
        self, task: PhysicsEvidenceTask, now_ns: int
    ) -> PhysicsEvidenceDecision:
        if self._phase is not EvidencePhase.IDLE:
            return self._decision("observer_busy")
        if not isinstance(task, PhysicsEvidenceTask):
            raise TypeError("task must be PhysicsEvidenceTask")
        _require_ns(now_ns, "now_ns")
        if self._required_epoch is not None and task.clock_epoch != self._required_epoch:
            return self._start_fault(task, now_ns, "reset_epoch_mismatch")
        if now_ns < task.started_at_ns:
            return self._start_fault(task, now_ns, "task_timestamp_in_future")
        if now_ns - task.started_at_ns > task.freshness_timeout_ns:
            return self._start_fault(task, now_ns, "task_timestamp_stale")
        self._task = task
        self._gripper_contact_counts_by_token = {
            token: 0 for token in task.gripper_collision_tokens
        }
        self._gripper_first_contact_ns_by_token = {
            token: None for token in task.gripper_collision_tokens
        }
        self._gripper_last_contact_ns_by_token = {
            token: None for token in task.gripper_collision_tokens
        }
        self._gripper_max_depth_by_token = {
            token: None for token in task.gripper_collision_tokens
        }
        self._gripper_max_force_by_token = {
            token: None for token in task.gripper_collision_tokens
        }
        self._gripper_max_normal_force_by_token = {
            token: None for token in task.gripper_collision_tokens
        }
        self._last_all_gripper_tokens_received_ns = None
        self._simultaneous_gripper_contact_sample_count = 0
        self._phase = EvidencePhase.BASELINE
        self._reason = "collecting_baseline"
        self._last_now_ns = now_ns
        return self._decision(self._reason)

    def observe_pose(self, sample: CubePoseSample) -> PhysicsEvidenceDecision:
        task = self._require_active()
        if self._phase in (EvidencePhase.FAULT, EvidencePhase.VERIFIED):
            return self._decision("terminal_observation_ignored")
        reason = self._sample_reason(
            source_timestamp_ns=sample.source_timestamp_ns,
            received_at_ns=sample.received_at_ns,
            clock_domain=sample.clock_domain,
            clock_epoch=sample.clock_epoch,
        )
        if reason:
            return self._fault(reason)
        if sample.model_name != task.cube_model_name:
            return self._fault("cube_model_mismatch")
        if sample.frame_id != task.world_frame:
            return self._fault("pose_frame_mismatch")
        if self._last_pose is not None:
            if sample.source_timestamp_ns < self._last_pose.source_timestamp_ns:
                return self._fault("pose_timestamp_out_of_order")
            if sample.source_timestamp_ns == self._last_pose.source_timestamp_ns:
                if sample == self._last_pose:
                    return self._decision("duplicate_pose_ignored")
                return self._fault("pose_timestamp_conflict")

        self._last_pose = sample
        self._pose_sample_count += 1
        self._record_source_time(sample.source_timestamp_ns)
        self._peak_z_m = (
            sample.z_m
            if self._peak_z_m is None
            else max(self._peak_z_m, sample.z_m)
        )
        if self._phase is EvidencePhase.BASELINE:
            self._baseline_z.append(sample.z_m)
            if len(self._baseline_z) >= task.baseline_sample_count:
                self._baseline_z_m = float(median(self._baseline_z))
                self._phase = EvidencePhase.WAIT_CONTACT
                self._reason = "baseline_ready"
            return self._decision(self._reason)

        if self._pose_meets_lift(sample):
            self._lift_observed = True

        if self._phase is EvidencePhase.RETENTION:
            clearance = (
                sample.z_m - task.cube_height_m / 2.0 - task.table_top_z_m
            )
            self._retention_min_clearance_m = (
                clearance
                if self._retention_min_clearance_m is None
                else min(self._retention_min_clearance_m, clearance)
            )
            assert self._retention_xy is not None
            drift = hypot(
                sample.x_m - self._retention_xy[0],
                sample.y_m - self._retention_xy[1],
            )
            self._max_xy_drift_m = max(self._max_xy_drift_m, drift)
            if drift > task.max_xy_drift_m:
                return self._fault("retention_xy_drift_exceeded")
            if not self._pose_meets_lift(sample):
                return self._fault("cube_dropped_during_retention")

        self._maybe_enter_retention(sample.received_at_ns)
        return self._maybe_complete_retention(sample.received_at_ns)

    def observe_contact(self, sample: ContactSample) -> PhysicsEvidenceDecision:
        task = self._require_active()
        if self._phase in (EvidencePhase.FAULT, EvidencePhase.VERIFIED):
            return self._decision("terminal_observation_ignored")
        reason = self._sample_reason(
            source_timestamp_ns=sample.source_timestamp_ns,
            received_at_ns=sample.received_at_ns,
            clock_domain=sample.clock_domain,
            clock_epoch=sample.clock_epoch,
        )
        if reason:
            return self._fault(reason)
        if self._last_contact is not None:
            if sample.source_timestamp_ns < self._last_contact.source_timestamp_ns:
                return self._fault("contact_timestamp_out_of_order")
            if sample.source_timestamp_ns == self._last_contact.source_timestamp_ns:
                if sample == self._last_contact:
                    return self._decision("duplicate_contact_ignored")
                return self._fault("contact_timestamp_conflict")

        self._last_contact = sample
        self._record_source_time(sample.source_timestamp_ns)
        gripper_contacts = 0
        table_contacts = 0
        sample_gripper_tokens: set[str] = set()
        sample_depth_by_token: dict[str, float] = {}
        sample_force_by_token: dict[str, float] = {}
        sample_normal_force_by_token: dict[str, float] = {}
        for pair in sample.pairs:
            matching_gripper_tokens = tuple(
                token
                for token in task.gripper_collision_tokens
                if _pair_matches(pair, task.cube_collision_token, (token,))
            )
            if matching_gripper_tokens:
                gripper_contacts += 1
                for token in matching_gripper_tokens:
                    sample_gripper_tokens.add(token)
                    self._gripper_contact_counts_by_token[token] = (
                        self._gripper_contact_counts_by_token.get(token, 0) + 1
                    )
                    if self._gripper_first_contact_ns_by_token[token] is None:
                        self._gripper_first_contact_ns_by_token[token] = (
                            sample.source_timestamp_ns
                        )
                    self._gripper_last_contact_ns_by_token[token] = (
                        sample.source_timestamp_ns
                    )
                    self._gripper_max_depth_by_token[token] = _optional_max(
                        self._gripper_max_depth_by_token.get(token),
                        pair.max_penetration_depth_m,
                    )
                    self._gripper_max_force_by_token[token] = _optional_max(
                        self._gripper_max_force_by_token.get(token),
                        pair.max_force_magnitude_n,
                    )
                    self._gripper_max_normal_force_by_token[token] = _optional_max(
                        self._gripper_max_normal_force_by_token.get(token),
                        pair.max_abs_normal_force_n,
                    )
                    _sample_max(
                        sample_depth_by_token,
                        token,
                        pair.max_penetration_depth_m,
                    )
                    _sample_max(
                        sample_force_by_token,
                        token,
                        pair.max_force_magnitude_n,
                    )
                    _sample_max(
                        sample_normal_force_by_token,
                        token,
                        pair.max_abs_normal_force_n,
                    )
                if self._matched_gripper_pair is None:
                    self._matched_gripper_pair = pair
            if _pair_matches(
                pair,
                task.cube_collision_token,
                task.table_collision_tokens,
            ):
                table_contacts += 1
        self._gripper_contact_count += gripper_contacts
        self._table_contact_count += table_contacts
        if all(
            token in sample_gripper_tokens
            for token in task.gripper_collision_tokens
        ):
            self._last_all_gripper_tokens_received_ns = sample.received_at_ns
            self._simultaneous_gripper_contact_sample_count += 1
            if self._first_simultaneous_contact_source_ns is None:
                self._first_simultaneous_contact_source_ns = (
                    sample.source_timestamp_ns
                )
            self._last_simultaneous_contact_source_ns = sample.source_timestamp_ns
            self._simultaneous_max_min_depth_m = _optional_max(
                self._simultaneous_max_min_depth_m,
                _minimum_for_all_tokens(
                    sample_depth_by_token, task.gripper_collision_tokens
                ),
            )
            self._simultaneous_max_min_force_n = _optional_max(
                self._simultaneous_max_min_force_n,
                _minimum_for_all_tokens(
                    sample_force_by_token, task.gripper_collision_tokens
                ),
            )
            self._simultaneous_max_min_normal_force_n = _optional_max(
                self._simultaneous_max_min_normal_force_n,
                _minimum_for_all_tokens(
                    sample_normal_force_by_token, task.gripper_collision_tokens
                ),
            )

        if self._phase is EvidencePhase.RETENTION and table_contacts:
            return self._fault("table_contact_during_retention")
        if gripper_contacts and self._phase is not EvidencePhase.BASELINE:
            self._gripper_contact_seen = True
            if self._phase is EvidencePhase.WAIT_CONTACT:
                self._phase = EvidencePhase.WAIT_LIFT
                self._reason = "gripper_cube_contact_observed"

        self._maybe_enter_retention(sample.received_at_ns)
        return self._maybe_complete_retention(sample.received_at_ns)

    def observe_gripper_effort(
        self, sample: GripperEffortSample
    ) -> PhysicsEvidenceDecision:
        """Record optional effort telemetry without changing success criteria."""

        task = self._require_active()
        if self._phase in (EvidencePhase.FAULT, EvidencePhase.VERIFIED):
            return self._decision("terminal_observation_ignored")
        reason = self._sample_reason(
            source_timestamp_ns=sample.source_timestamp_ns,
            received_at_ns=sample.received_at_ns,
            clock_domain=sample.clock_domain,
            clock_epoch=sample.clock_epoch,
        )
        if reason:
            return self._fault(reason)
        if sample.joint_name != task.gripper_joint_name:
            return self._fault("gripper_effort_joint_mismatch")
        if self._last_effort is not None:
            if sample.source_timestamp_ns < self._last_effort.source_timestamp_ns:
                return self._fault("gripper_effort_timestamp_out_of_order")
            if sample.source_timestamp_ns == self._last_effort.source_timestamp_ns:
                if sample == self._last_effort:
                    return self._decision("duplicate_gripper_effort_ignored")
                return self._fault("gripper_effort_timestamp_conflict")
        self._last_effort = sample
        self._gripper_effort_sample_count += 1
        self._peak_abs_gripper_effort = _optional_max(
            self._peak_abs_gripper_effort, abs(sample.effort)
        )
        self._record_source_time(sample.source_timestamp_ns)
        return self._decision("gripper_effort_observed")

    def observe_sequence_completion(
        self, completion: SequenceCompletion
    ) -> PhysicsEvidenceDecision:
        task = self._require_active()
        if self._phase in (EvidencePhase.FAULT, EvidencePhase.VERIFIED):
            return self._decision("terminal_observation_ignored")
        reason = self._sample_reason(
            source_timestamp_ns=completion.source_timestamp_ns,
            received_at_ns=completion.received_at_ns,
            clock_domain=completion.clock_domain,
            clock_epoch=completion.clock_epoch,
        )
        if reason:
            return self._fault(reason)
        if completion.task_id != task.task_id:
            return self._decision("unrelated_sequence_completion_ignored")
        if completion.target_id != task.target_id:
            return self._fault("sequence_target_id_mismatch")
        if completion.target_source_timestamp_ns != task.target_source_timestamp_ns:
            return self._fault("sequence_target_source_timestamp_mismatch")
        if not completion.sequence_completed:
            return self._fault("sequence_not_completed")
        if completion.lift_command_id != task.expected_lift_command_id:
            return self._fault("sequence_lift_command_id_mismatch")
        if self._sequence_completed:
            return self._decision("duplicate_sequence_completion_ignored")

        self._sequence_completed = True
        self._sequence_completed_source_ns = completion.source_timestamp_ns
        self._simultaneous_count_at_sequence_completion = (
            self._simultaneous_gripper_contact_sample_count
        )
        self._last_simultaneous_ns_at_sequence_completion = (
            self._last_simultaneous_contact_source_ns
        )
        self._gripper_effort_at_sequence_completion = (
            None if self._last_effort is None else self._last_effort.effort
        )
        self._peak_abs_effort_at_sequence_completion = self._peak_abs_gripper_effort
        self._record_source_time(completion.source_timestamp_ns)
        self._maybe_enter_retention(completion.received_at_ns)
        return self._maybe_complete_retention(completion.received_at_ns)

    def tick(self, now_ns: int) -> PhysicsEvidenceDecision:
        task = self._require_active()
        if self._phase in (EvidencePhase.FAULT, EvidencePhase.VERIFIED):
            return self._decision(self._reason)
        reason = self._clock_reason(now_ns)
        if reason:
            return self._fault(reason)
        if self._last_pose is None:
            if now_ns - task.started_at_ns > task.freshness_timeout_ns:
                return self._fault("pose_liveness_timeout")
        elif now_ns - self._last_pose.received_at_ns > task.freshness_timeout_ns:
            return self._fault("pose_liveness_timeout")
        if self._phase in (EvidencePhase.WAIT_LIFT, EvidencePhase.RETENTION):
            if self._last_contact is None:
                return self._fault("contact_liveness_timeout")
            if now_ns - self._last_contact.received_at_ns > task.freshness_timeout_ns:
                return self._fault("contact_liveness_timeout")

        self._maybe_enter_retention(now_ns)
        return self._maybe_complete_retention(now_ns)

    def fail_closed(self, reason: str, now_ns: int) -> PhysicsEvidenceDecision:
        """Latch an external observer/wrapper fault without issuing motion."""

        self._require_active()
        _require_text(reason, "reason")
        if self._phase in (EvidencePhase.FAULT, EvidencePhase.VERIFIED):
            return self._decision(self._reason)
        clock_reason = self._clock_reason(now_ns)
        return self._fault(clock_reason or reason)

    def reset(self, *, new_epoch: int, now_ns: int) -> PhysicsEvidenceDecision:
        if self._phase not in (
            EvidencePhase.IDLE,
            EvidencePhase.FAULT,
            EvidencePhase.VERIFIED,
        ):
            return self._decision("reset_rejected_observation_active")
        _require_epoch(new_epoch)
        _require_ns(now_ns, "now_ns")
        if self._task is not None and new_epoch <= self._task.clock_epoch:
            return self._decision("reset_requires_new_epoch")
        self.__init__()
        self._required_epoch = new_epoch
        self._last_now_ns = now_ns
        return self._decision("reset")

    def _require_active(self) -> PhysicsEvidenceTask:
        if self._task is None or self._phase is EvidencePhase.IDLE:
            raise RuntimeError("physics evidence task is not active")
        return self._task

    def _start_fault(
        self, task: PhysicsEvidenceTask, now_ns: int, reason: str
    ) -> PhysicsEvidenceDecision:
        self._task = task
        self._last_now_ns = now_ns
        self._phase = EvidencePhase.FAULT
        self._reason = reason
        return self._decision(reason)

    def _sample_reason(
        self,
        *,
        source_timestamp_ns: int,
        received_at_ns: int,
        clock_domain: str,
        clock_epoch: int,
    ) -> str | None:
        task = self._require_active()
        reason = self._clock_reason(received_at_ns)
        if reason:
            return reason
        if clock_domain != task.clock_domain:
            return "clock_domain_mismatch"
        if clock_epoch != task.clock_epoch:
            return "clock_epoch_mismatch"
        if source_timestamp_ns > received_at_ns:
            return "source_timestamp_in_future"
        if received_at_ns - source_timestamp_ns > task.freshness_timeout_ns:
            return "source_timestamp_stale"
        return None

    def _clock_reason(self, now_ns: int) -> str | None:
        _require_ns(now_ns, "now_ns")
        if self._last_now_ns is not None and now_ns < self._last_now_ns:
            return "clock_rollback"
        self._last_now_ns = now_ns
        return None

    def _record_source_time(self, timestamp_ns: int) -> None:
        if self._first_source_timestamp_ns is None:
            self._first_source_timestamp_ns = timestamp_ns
        self._last_source_timestamp_ns = timestamp_ns

    def _pose_meets_lift(self, sample: CubePoseSample) -> bool:
        task = self._require_active()
        if self._baseline_z_m is None:
            return False
        lifted = sample.z_m - self._baseline_z_m >= task.min_lift_m
        cube_bottom = sample.z_m - task.cube_height_m / 2.0
        clear = cube_bottom - task.table_top_z_m >= task.min_clearance_m
        return lifted and clear

    def _contact_is_fresh(self, now_ns: int) -> bool:
        task = self._require_active()
        return (
            self._last_all_gripper_tokens_received_ns is not None
            and now_ns >= self._last_all_gripper_tokens_received_ns
            and now_ns - self._last_all_gripper_tokens_received_ns
            <= task.freshness_timeout_ns
        )

    def _maybe_enter_retention(self, now_ns: int) -> None:
        task = self._require_active()
        if self._phase is not EvidencePhase.WAIT_LIFT:
            return
        if not self._sequence_completed or not self._gripper_contact_seen:
            return
        if self._last_pose is None or not self._pose_meets_lift(self._last_pose):
            return
        if not self._contact_is_fresh(now_ns):
            return
        self._phase = EvidencePhase.RETENTION
        self._reason = "retention_started"
        self._retention_started_at_ns = now_ns
        self._retention_xy = (self._last_pose.x_m, self._last_pose.y_m)
        self._retention_min_clearance_m = (
            self._last_pose.z_m
            - task.cube_height_m / 2.0
            - task.table_top_z_m
        )

    def _maybe_complete_retention(self, now_ns: int) -> PhysicsEvidenceDecision:
        if self._phase is not EvidencePhase.RETENTION:
            return self._decision(self._reason)
        task = self._require_active()
        assert self._retention_started_at_ns is not None
        if not self._contact_is_fresh(now_ns):
            return self._fault("contact_liveness_timeout")
        if self._last_pose is None:
            return self._fault("pose_liveness_timeout")
        if now_ns - self._last_pose.received_at_ns > task.freshness_timeout_ns:
            return self._fault("pose_liveness_timeout")
        if now_ns - self._retention_started_at_ns < task.retention_ns:
            return self._decision("retention_in_progress")
        self._phase = EvidencePhase.VERIFIED
        self._reason = "contact_lift_retention_verified"
        self._retention_completed_at_ns = now_ns
        return self._decision(self._reason)

    def _fault(self, reason: str) -> PhysicsEvidenceDecision:
        self._phase = EvidencePhase.FAULT
        self._reason = reason
        return self._decision(reason)

    def _decision(self, reason: str) -> PhysicsEvidenceDecision:
        return PhysicsEvidenceDecision(
            phase=self._phase,
            physics_grasp_verified=self._phase is EvidencePhase.VERIFIED,
            reason=reason,
        )


def _pair_matches(
    pair: CollisionPair, cube_token: str, other_tokens: tuple[str, ...]
) -> bool:
    first_is_cube = cube_token in pair.collision1
    second_is_cube = cube_token in pair.collision2
    return (
        first_is_cube and any(token in pair.collision2 for token in other_tokens)
    ) or (second_is_cube and any(token in pair.collision1 for token in other_tokens))


def _optional_max(current: float | None, candidate: float | None) -> float | None:
    if candidate is None:
        return current
    return candidate if current is None else max(current, candidate)


def _sample_max(values: dict[str, float], token: str, candidate: float | None) -> None:
    if candidate is None:
        return
    values[token] = max(values.get(token, candidate), candidate)


def _minimum_for_all_tokens(
    values: dict[str, float], tokens: tuple[str, ...]
) -> float | None:
    if any(token not in values for token in tokens):
        return None
    return min(values[token] for token in tokens)


def _require_text(value: object, field: str) -> None:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field} must be a non-empty string")


def _require_tokens(value: object, field: str) -> None:
    if not isinstance(value, tuple) or not value:
        raise ValueError(f"{field} must be a non-empty tuple")
    for token in value:
        _require_text(token, field)


def _require_ns(value: object, field: str) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{field} must be a non-negative integer")


def _require_epoch(value: object) -> None:
    _require_ns(value, "clock_epoch")


def _require_positive_int(value: object, field: str) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{field} must be a positive integer")


def _require_finite(value: object, field: str) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value):
        raise ValueError(f"{field} must be finite")


def _require_positive_float(value: object, field: str) -> None:
    _require_finite(value, field)
    if value <= 0:
        raise ValueError(f"{field} must be positive")


def _require_nonnegative_float(value: object, field: str) -> None:
    _require_finite(value, field)
    if value < 0:
        raise ValueError(f"{field} must be non-negative")
