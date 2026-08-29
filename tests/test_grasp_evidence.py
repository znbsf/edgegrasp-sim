from __future__ import annotations

from dataclasses import replace
from math import nan

import pytest

from edgegrasp.grasp_evidence import (
    CollisionPair,
    ContactSample,
    CubePoseSample,
    EvidencePhase,
    GraspPhysicsEvidenceController,
    GripperEffortSample,
    PhysicsEvidenceTask,
    SequenceCompletion,
)


SCENE_DIGEST = "54fabe93f95b57379c2579775e9c0d45117460c6eed578071f7b68f9177cb55d"
CUBE = "target_cube::cube_link::collision"
GRIPPER = "so101::gripper_link::gripper_collision"
TABLE = "table::table_top::collision"


def task(*, epoch: int = 0, **changes: object) -> PhysicsEvidenceTask:
    values: dict[str, object] = {
        "task_id": "task-001",
        "target_id": "cube-001",
        "target_source_timestamp_ns": 900,
        "expected_lift_command_id": "task-001|lift|3",
        "scene_digest": SCENE_DIGEST,
        "cube_model_name": "target_cube",
        "cube_collision_token": "target_cube::",
        "gripper_collision_tokens": ("so101::gripper",),
        "table_collision_tokens": ("table::",),
        "started_at_ns": 1_000,
        "clock_epoch": epoch,
        "baseline_sample_count": 3,
        "retention_ns": 500,
        "freshness_timeout_ns": 250,
    }
    values.update(changes)
    return PhysicsEvidenceTask(**values)  # type: ignore[arg-type]


def pose(
    timestamp_ns: int,
    *,
    x_m: float = 0.2,
    y_m: float = 0.0,
    z_m: float = 0.425,
    received_at_ns: int | None = None,
    epoch: int = 0,
    frame_id: str = "world",
    model_name: str = "target_cube",
    domain: str = "ros_sim",
) -> CubePoseSample:
    return CubePoseSample(
        model_name=model_name,
        frame_id=frame_id,
        x_m=x_m,
        y_m=y_m,
        z_m=z_m,
        source_timestamp_ns=timestamp_ns,
        received_at_ns=timestamp_ns if received_at_ns is None else received_at_ns,
        clock_domain=domain,
        clock_epoch=epoch,
    )


def contact(
    timestamp_ns: int,
    *pairs: CollisionPair,
    received_at_ns: int | None = None,
    epoch: int = 0,
    domain: str = "ros_sim",
) -> ContactSample:
    return ContactSample(
        pairs=tuple(pairs),
        source_timestamp_ns=timestamp_ns,
        received_at_ns=timestamp_ns if received_at_ns is None else received_at_ns,
        clock_domain=domain,
        clock_epoch=epoch,
    )


def effort(
    timestamp_ns: int,
    value: float,
    *,
    joint_name: str = "gripper",
    received_at_ns: int | None = None,
    epoch: int = 0,
    domain: str = "ros_sim",
) -> GripperEffortSample:
    return GripperEffortSample(
        joint_name=joint_name,
        effort=value,
        source_timestamp_ns=timestamp_ns,
        received_at_ns=timestamp_ns if received_at_ns is None else received_at_ns,
        clock_domain=domain,
        clock_epoch=epoch,
    )


def completion(
    timestamp_ns: int,
    *,
    epoch: int = 0,
    task_id: str = "task-001",
    target_id: str = "cube-001",
    command_id: str = "task-001|lift|3",
    target_source_timestamp_ns: int = 900,
    success: bool = True,
) -> SequenceCompletion:
    return SequenceCompletion(
        task_id=task_id,
        target_id=target_id,
        target_source_timestamp_ns=target_source_timestamp_ns,
        lift_command_id=command_id,
        sequence_completed=success,
        source_timestamp_ns=timestamp_ns,
        received_at_ns=timestamp_ns,
        clock_domain="ros_sim",
        clock_epoch=epoch,
    )


def baseline(controller: GraspPhysicsEvidenceController, *, epoch: int = 0) -> None:
    for timestamp_ns in (1_000, 1_050, 1_100):
        controller.observe_pose(pose(timestamp_ns, epoch=epoch))
    assert controller.phase is EvidencePhase.WAIT_CONTACT


def enter_retention(
    controller: GraspPhysicsEvidenceController, *, epoch: int = 0
) -> None:
    baseline(controller, epoch=epoch)
    controller.observe_contact(contact(1_150, CollisionPair(CUBE, GRIPPER), epoch=epoch))
    controller.observe_sequence_completion(completion(1_200, epoch=epoch))
    decision = controller.observe_pose(pose(1_250, z_m=0.46, epoch=epoch))
    assert decision.phase is EvidencePhase.RETENTION


def run_success() -> str:
    controller = GraspPhysicsEvidenceController()
    controller.start(task(), 1_000)
    enter_retention(controller)
    for timestamp_ns in (1_350, 1_450, 1_550, 1_650, 1_750):
        controller.observe_contact(
            contact(timestamp_ns, CollisionPair(GRIPPER, CUBE))
        )
        decision = controller.observe_pose(pose(timestamp_ns, z_m=0.46))
    assert decision.physics_grasp_verified
    assert controller.result is not None
    return controller.result.digest


def test_success_requires_contact_lift_retention_and_sequence_completion() -> None:
    controller = GraspPhysicsEvidenceController()
    assert controller.start(task(), 1_000).phase is EvidencePhase.BASELINE
    enter_retention(controller)
    for timestamp_ns in (1_350, 1_450, 1_550, 1_650):
        controller.observe_contact(contact(timestamp_ns, CollisionPair(CUBE, GRIPPER)))
        assert not controller.observe_pose(
            pose(timestamp_ns, z_m=0.46)
        ).physics_grasp_verified
    controller.observe_contact(contact(1_750, CollisionPair(CUBE, GRIPPER)))
    decision = controller.observe_pose(pose(1_750, z_m=0.46))

    assert decision.phase is EvidencePhase.VERIFIED
    assert decision.physics_grasp_verified
    result = controller.result
    assert result is not None
    assert result.sequence_completed
    assert result.gripper_contact_observed
    assert result.lift_observed
    assert result.retention_observed
    assert result.physics_grasp_verified
    assert result.target_source_timestamp_ns == 900
    assert result.expected_lift_command_id == "task-001|lift|3"
    assert result.cube_model_name == "target_cube"
    assert result.pose_frame_id == "world"
    assert result.matched_gripper_collision1 == CUBE
    assert result.matched_gripper_collision2 == GRIPPER
    assert result.gripper_contact_count == 6
    assert result.gripper_contact_tokens == ("so101::gripper",)
    assert result.gripper_contact_counts_by_token == (6,)
    assert result.gripper_first_contact_source_timestamp_ns_by_token == (1_150,)
    assert result.gripper_last_contact_source_timestamp_ns_by_token == (1_750,)
    assert result.all_gripper_tokens_observed
    assert result.simultaneous_gripper_contact_sample_count == 6
    assert result.first_simultaneous_gripper_contact_source_timestamp_ns == 1_150
    assert result.last_simultaneous_gripper_contact_source_timestamp_ns == 1_750
    assert result.sequence_completed_source_timestamp_ns == 1_200
    assert (
        result.simultaneous_gripper_contact_sample_count_at_sequence_completion
        == 1
    )
    assert (
        result.last_simultaneous_contact_source_timestamp_ns_at_sequence_completion
        == 1_150
    )
    assert result.evidence_started_at_ns == 1_000
    assert result.evidence_completed_at_ns == 1_750
    assert result.retention_started_at_ns == 1_250
    assert result.retention_completed_at_ns == 1_750
    assert len(result.digest) == 64


def test_sequence_completion_alone_never_verifies_physics() -> None:
    controller = GraspPhysicsEvidenceController()
    controller.start(task(), 1_000)
    baseline(controller)
    decision = controller.observe_sequence_completion(completion(1_150))

    assert decision.phase is EvidencePhase.WAIT_CONTACT
    assert not decision.physics_grasp_verified


def test_wrong_contact_does_not_advance() -> None:
    controller = GraspPhysicsEvidenceController()
    controller.start(task(), 1_000)
    baseline(controller)
    decision = controller.observe_contact(
        contact(1_150, CollisionPair(CUBE, "unrelated::collision"))
    )

    assert decision.phase is EvidencePhase.WAIT_CONTACT
    assert controller.result is not None
    assert controller.result.gripper_contact_count == 0


def test_contact_histogram_keeps_distinct_gripper_tokens() -> None:
    fixed = "edgegrasp_grasp_proxy_fixed_finger_pad"
    moving = "edgegrasp_grasp_proxy_moving_finger_pad"
    controller = GraspPhysicsEvidenceController()
    controller.start(
        task(gripper_collision_tokens=(fixed, moving)),
        1_000,
    )
    baseline(controller)
    controller.observe_contact(
        contact(
            1_150,
            CollisionPair(CUBE, f"so101::{fixed}::collision"),
            CollisionPair(f"so101::{moving}::collision", CUBE),
            CollisionPair(CUBE, f"so101::{fixed}::collision_2"),
        )
    )

    result = controller.result
    assert result is not None
    assert result.gripper_contact_count == 3
    assert result.gripper_contact_tokens == (fixed, moving)
    assert result.gripper_contact_counts_by_token == (2, 1)
    assert result.all_gripper_tokens_observed
    assert result.simultaneous_gripper_contact_sample_count == 1
    assert result.digest == controller.result.digest


def test_contact_quality_is_recorded_per_pad_and_for_same_sample_bottleneck() -> None:
    fixed = "edgegrasp_grasp_proxy_fixed_finger_pad"
    moving = "edgegrasp_grasp_proxy_moving_finger_pad"
    controller = GraspPhysicsEvidenceController()
    controller.start(task(gripper_collision_tokens=(fixed, moving)), 1_000)
    baseline(controller)
    controller.observe_contact(
        contact(
            1_150,
            CollisionPair(
                CUBE,
                f"so101::{fixed}::collision",
                max_penetration_depth_m=0.0005,
                max_force_magnitude_n=2.0,
                max_abs_normal_force_n=1.5,
            ),
            CollisionPair(
                f"so101::{moving}::collision",
                CUBE,
                max_penetration_depth_m=0.0002,
                max_force_magnitude_n=1.2,
                max_abs_normal_force_n=0.8,
            ),
        )
    )

    result = controller.result
    assert result is not None
    assert result.gripper_max_penetration_depth_m_by_token == (0.0005, 0.0002)
    assert result.gripper_max_force_magnitude_n_by_token == (2.0, 1.2)
    assert result.gripper_max_abs_normal_force_n_by_token == (1.5, 0.8)
    assert (
        result.simultaneous_gripper_contact_max_min_penetration_depth_m
        == 0.0002
    )
    assert result.simultaneous_gripper_contact_max_min_force_magnitude_n == 1.2
    assert result.simultaneous_gripper_contact_max_min_abs_normal_force_n == 0.8


def test_gripper_effort_is_diagnostic_and_covered_by_digest() -> None:
    controller = GraspPhysicsEvidenceController()
    controller.start(task(), 1_000)
    baseline(controller)
    before = controller.result
    assert before is not None

    decision = controller.observe_gripper_effort(effort(1_125, -0.75))
    assert decision.phase is EvidencePhase.WAIT_CONTACT
    assert decision.reason == "gripper_effort_observed"
    result = controller.result
    assert result is not None
    assert result.gripper_joint_name == "gripper"
    assert result.gripper_effort_sample_count == 1
    assert result.latest_gripper_effort == -0.75
    assert result.peak_abs_gripper_effort == 0.75
    assert result.digest != before.digest

    controller.observe_gripper_effort(effort(1_130, 0.25))
    result = controller.result
    assert result is not None
    assert result.gripper_effort_sample_count == 2
    assert result.latest_gripper_effort == 0.25
    assert result.peak_abs_gripper_effort == 0.75


def test_sequence_completion_snapshots_contact_timing_and_effort() -> None:
    controller = GraspPhysicsEvidenceController()
    controller.start(task(), 1_000)
    baseline(controller)
    controller.observe_gripper_effort(effort(1_125, -0.7))
    controller.observe_contact(contact(1_150, CollisionPair(CUBE, GRIPPER)))
    controller.observe_sequence_completion(completion(1_200))
    controller.observe_gripper_effort(effort(1_225, -0.1))
    controller.observe_contact(contact(1_225, CollisionPair(CUBE, GRIPPER)))

    result = controller.result
    assert result is not None
    assert result.sequence_completed_source_timestamp_ns == 1_200
    assert (
        result.simultaneous_gripper_contact_sample_count_at_sequence_completion
        == 1
    )
    assert (
        result.last_simultaneous_contact_source_timestamp_ns_at_sequence_completion
        == 1_150
    )
    assert result.gripper_effort_at_sequence_completion == -0.7
    assert result.peak_abs_gripper_effort_at_sequence_completion == 0.7
    assert result.last_simultaneous_gripper_contact_source_timestamp_ns == 1_225
    assert result.latest_gripper_effort == -0.1


def test_invalid_or_conflicting_gripper_effort_fails_closed() -> None:
    with pytest.raises(ValueError, match="effort must be finite"):
        effort(1_000, nan)

    controller = GraspPhysicsEvidenceController()
    controller.start(task(), 1_000)
    baseline(controller)
    controller.observe_gripper_effort(effort(1_125, 0.1))
    decision = controller.observe_gripper_effort(effort(1_125, 0.2))
    assert decision.phase is EvidencePhase.FAULT
    assert decision.reason == "gripper_effort_timestamp_conflict"

    controller = GraspPhysicsEvidenceController()
    controller.start(task(), 1_000)
    baseline(controller)
    decision = controller.observe_gripper_effort(
        effort(1_125, 0.1, joint_name="wrong")
    )
    assert decision.phase is EvidencePhase.FAULT
    assert decision.reason == "gripper_effort_joint_mismatch"


def test_two_pad_task_never_retains_from_one_pad_only() -> None:
    fixed = "edgegrasp_grasp_proxy_fixed_finger_pad"
    moving = "edgegrasp_grasp_proxy_moving_finger_pad"
    controller = GraspPhysicsEvidenceController()
    controller.start(
        task(gripper_collision_tokens=(fixed, moving)),
        1_000,
    )
    baseline(controller)
    controller.observe_contact(
        contact(1_150, CollisionPair(CUBE, f"so101::{fixed}::collision"))
    )
    controller.observe_sequence_completion(completion(1_200))
    decision = controller.observe_pose(pose(1_250, z_m=0.46))

    assert decision.phase is EvidencePhase.WAIT_LIFT
    result = controller.result
    assert result is not None
    assert result.gripper_contact_observed
    assert not result.all_gripper_tokens_observed
    assert result.simultaneous_gripper_contact_sample_count == 0
    assert not result.physics_grasp_verified


def test_insufficient_lift_or_clearance_does_not_start_retention() -> None:
    controller = GraspPhysicsEvidenceController()
    controller.start(task(), 1_000)
    baseline(controller)
    controller.observe_contact(contact(1_150, CollisionPair(CUBE, GRIPPER)))
    controller.observe_sequence_completion(completion(1_200))

    # 15 mm lift is below the configured 20 mm threshold.
    decision = controller.observe_pose(pose(1_250, z_m=0.44))
    assert decision.phase is EvidencePhase.WAIT_LIFT


@pytest.mark.parametrize(
    ("sample", "reason"),
    [
        (pose(1_150, frame_id="base_link"), "pose_frame_mismatch"),
        (pose(1_150, model_name="other_cube"), "cube_model_mismatch"),
        (pose(1_150, domain="ros_system"), "clock_domain_mismatch"),
        (pose(1_150, epoch=1), "clock_epoch_mismatch"),
        (pose(1_300, received_at_ns=1_200), "source_timestamp_in_future"),
        (pose(1_000, received_at_ns=1_300), "source_timestamp_stale"),
    ],
)
def test_invalid_pose_identity_or_time_faults(
    sample: CubePoseSample, reason: str
) -> None:
    controller = GraspPhysicsEvidenceController()
    controller.start(task(), 1_000)
    decision = controller.observe_pose(sample)

    assert decision.phase is EvidencePhase.FAULT
    assert decision.reason == reason


def test_out_of_order_and_conflicting_duplicate_pose_fault() -> None:
    controller = GraspPhysicsEvidenceController()
    controller.start(task(), 1_000)
    first = pose(1_050)
    controller.observe_pose(first)
    assert controller.observe_pose(first).reason == "duplicate_pose_ignored"

    decision = controller.observe_pose(pose(1_050, z_m=0.5))
    assert decision.phase is EvidencePhase.FAULT
    assert decision.reason == "pose_timestamp_conflict"

    controller = GraspPhysicsEvidenceController()
    controller.start(task(), 1_000)
    controller.observe_pose(pose(1_100))
    decision = controller.observe_pose(pose(1_050, received_at_ns=1_150))
    assert decision.reason == "pose_timestamp_out_of_order"


def test_contact_out_of_order_and_duplicate_policy() -> None:
    controller = GraspPhysicsEvidenceController()
    controller.start(task(), 1_000)
    baseline(controller)
    first = contact(1_150, CollisionPair(CUBE, GRIPPER))
    controller.observe_contact(first)
    assert controller.observe_contact(first).reason == "duplicate_contact_ignored"

    decision = controller.observe_contact(
        contact(1_140, CollisionPair(CUBE, GRIPPER), received_at_ns=1_160)
    )
    assert decision.phase is EvidencePhase.FAULT
    assert decision.reason == "contact_timestamp_out_of_order"


def test_table_recontact_during_retention_faults() -> None:
    controller = GraspPhysicsEvidenceController()
    controller.start(task(), 1_000)
    enter_retention(controller)

    decision = controller.observe_contact(contact(1_300, CollisionPair(CUBE, TABLE)))
    assert decision.phase is EvidencePhase.FAULT
    assert decision.reason == "table_contact_during_retention"


def test_drop_and_xy_drift_during_retention_fault() -> None:
    controller = GraspPhysicsEvidenceController()
    controller.start(task(), 1_000)
    enter_retention(controller)
    controller.observe_contact(contact(1_300, CollisionPair(CUBE, GRIPPER)))
    decision = controller.observe_pose(pose(1_300, z_m=0.43))
    assert decision.reason == "cube_dropped_during_retention"

    controller = GraspPhysicsEvidenceController()
    controller.start(task(), 1_000)
    enter_retention(controller)
    controller.observe_contact(contact(1_300, CollisionPair(CUBE, GRIPPER)))
    decision = controller.observe_pose(pose(1_300, x_m=0.211, z_m=0.46))
    assert decision.reason == "retention_xy_drift_exceeded"


def test_liveness_watchdog_faults_pose_and_contact_loss() -> None:
    controller = GraspPhysicsEvidenceController()
    controller.start(task(), 1_000)
    assert controller.tick(1_250).phase is EvidencePhase.BASELINE
    decision = controller.tick(1_251)
    assert decision.reason == "pose_liveness_timeout"

    controller = GraspPhysicsEvidenceController()
    controller.start(task(), 1_000)
    baseline(controller)
    controller.observe_contact(contact(1_150, CollisionPair(CUBE, GRIPPER)))
    decision = controller.tick(1_401)
    assert decision.reason == "pose_liveness_timeout"


def test_clock_rollback_faults_and_latches() -> None:
    controller = GraspPhysicsEvidenceController()
    controller.start(task(), 1_000)
    controller.observe_pose(pose(1_100))

    decision = controller.tick(1_099)
    assert decision.phase is EvidencePhase.FAULT
    assert decision.reason == "clock_rollback"
    assert controller.tick(2_000).phase is EvidencePhase.FAULT


def test_external_wrapper_fault_is_latched_without_motion_side_effects() -> None:
    controller = GraspPhysicsEvidenceController()
    controller.start(task(), 1_000)
    decision = controller.fail_closed("observer_cancelled", 1_010)
    assert decision.phase is EvidencePhase.FAULT
    assert decision.reason == "observer_cancelled"
    assert controller.result is not None
    assert not controller.result.physics_grasp_verified


def test_task_rejects_target_source_after_observer_start() -> None:
    with pytest.raises(
        ValueError, match="target_source_timestamp_ns must not follow started_at_ns"
    ):
        task(target_source_timestamp_ns=1_001)


@pytest.mark.parametrize(
    ("changes", "reason"),
    [
        ({"target_id": "wrong"}, "sequence_target_id_mismatch"),
        (
            {"target_source_timestamp_ns": 899},
            "sequence_target_source_timestamp_mismatch",
        ),
        ({"command_id": "task-001|approach|0"}, "sequence_lift_command_id_mismatch"),
        (
            {"success": False, "command_id": ""},
            "sequence_not_completed",
        ),
    ],
)
def test_correlated_sequence_completion_is_required(
    changes: dict[str, object], reason: str
) -> None:
    controller = GraspPhysicsEvidenceController()
    controller.start(task(), 1_000)
    baseline(controller)
    decision = controller.observe_sequence_completion(completion(1_150, **changes))
    assert decision.phase is EvidencePhase.FAULT
    assert decision.reason == reason


def test_unrelated_late_sequence_result_is_ignored() -> None:
    controller = GraspPhysicsEvidenceController()
    controller.start(task(), 1_000)
    baseline(controller)
    decision = controller.observe_sequence_completion(
        completion(1_150, task_id="old-task")
    )
    assert decision.phase is EvidencePhase.WAIT_CONTACT
    assert decision.reason == "unrelated_sequence_completion_ignored"


def test_reset_requires_terminal_state_and_new_epoch() -> None:
    controller = GraspPhysicsEvidenceController()
    controller.start(task(), 1_000)
    assert controller.reset(new_epoch=1, now_ns=1_010).reason == (
        "reset_rejected_observation_active"
    )
    controller.observe_pose(pose(1_020, frame_id="wrong"))
    assert controller.reset(new_epoch=0, now_ns=1_030).reason == (
        "reset_requires_new_epoch"
    )
    assert controller.reset(new_epoch=1, now_ns=1_030).phase is EvidencePhase.IDLE

    wrong_epoch = replace(task(epoch=1), clock_epoch=2, started_at_ns=1_040)
    decision = controller.start(wrong_epoch, 1_040)
    assert decision.reason == "reset_epoch_mismatch"


def test_exact_retention_boundary() -> None:
    controller = GraspPhysicsEvidenceController()
    controller.start(task(), 1_000)
    enter_retention(controller)
    for timestamp_ns in (1_400, 1_600, 1_749):
        controller.observe_contact(contact(timestamp_ns, CollisionPair(CUBE, GRIPPER)))
        decision = controller.observe_pose(pose(timestamp_ns, z_m=0.46))
    assert decision.phase is EvidencePhase.RETENTION

    controller.observe_contact(contact(1_750, CollisionPair(CUBE, GRIPPER)))
    assert controller.observe_pose(pose(1_750, z_m=0.46)).phase is EvidencePhase.VERIFIED


def test_empty_contact_does_not_refresh_gripper_contact_liveness() -> None:
    controller = GraspPhysicsEvidenceController()
    controller.start(task(), 1_000)
    enter_retention(controller)
    controller.observe_contact(contact(1_400))
    controller.observe_pose(pose(1_400, z_m=0.46))

    decision = controller.tick(1_401)
    assert decision.phase is EvidencePhase.FAULT
    assert decision.reason == "contact_liveness_timeout"


def test_result_digest_covers_identity_and_measurements() -> None:
    digest = run_success()
    controller = GraspPhysicsEvidenceController()
    changed = task(target_id="cube-002")
    controller.start(changed, 1_000)
    assert controller.result is not None
    assert controller.result.digest != digest


def test_one_hundred_replays_are_deterministic() -> None:
    assert len({run_success() for _ in range(100)}) == 1


def test_invalid_numeric_inputs_are_rejected() -> None:
    with pytest.raises(ValueError, match="z_m must be finite"):
        pose(1_000, z_m=nan)
    with pytest.raises(ValueError, match="scene_digest"):
        task(scene_digest="floating-main")
    with pytest.raises(ValueError, match="baseline_sample_count"):
        task(baseline_sample_count=0)
    with pytest.raises(ValueError, match="max_penetration_depth_m"):
        CollisionPair(CUBE, GRIPPER, max_penetration_depth_m=-0.001)
