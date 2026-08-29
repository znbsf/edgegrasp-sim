from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace

import pytest

from edgegrasp.grasp_sequence import (
    ACTION_STATUS_SUCCEEDED,
    FJT_SUCCESSFUL,
    MOVEIT_SUCCESS,
    ArmPlanCommand,
    ArmStageTerminal,
    CancelDisposition,
    CancelOutcome,
    DispatchOutcome,
    GraspPhase,
    GraspSequenceController,
    GraspTaskSnapshot,
    GripperExecutionCommand,
    GripperStageTerminal,
    SequenceHealth,
    SignalSnapshot,
    StopAllOutcome,
)
from edgegrasp.models import Vector3


NOW_NS = 1_000_000_000
DIGEST = "a" * 64
APPROACH_ORIENTATION = (0.0, 0.0, 0.0, 1.0)
GRASP_ORIENTATION = (0.0, 2**-0.5, 0.0, 2**-0.5)


class RecordingArmPort:
    def __init__(self) -> None:
        self.commands: list[ArmPlanCommand] = []
        self.cancels: list[tuple[str, str]] = []
        self.submit_outcome = DispatchOutcome(True, "submitted")
        self.cancel_outcome = CancelOutcome(
            CancelDisposition.TERMINAL_CONFIRMED, "terminal"
        )
        self.submit_error: Exception | None = None
        self.cancel_error: Exception | None = None

    def submit(self, command: ArmPlanCommand) -> DispatchOutcome:
        self.commands.append(command)
        if self.submit_error is not None:
            raise self.submit_error
        return self.submit_outcome

    def cancel(self, command_id: str, reason: str) -> CancelOutcome:
        self.cancels.append((command_id, reason))
        if self.cancel_error is not None:
            raise self.cancel_error
        return self.cancel_outcome


class RecordingGripperPort:
    def __init__(self) -> None:
        self.commands: list[GripperExecutionCommand] = []
        self.cancels: list[tuple[str, str]] = []
        self.submit_outcome = DispatchOutcome(True, "submitted")
        self.cancel_outcome = CancelOutcome(
            CancelDisposition.TERMINAL_CONFIRMED, "terminal"
        )

    def submit(self, command: GripperExecutionCommand) -> DispatchOutcome:
        self.commands.append(command)
        return self.submit_outcome

    def cancel(self, command_id: str, reason: str) -> CancelOutcome:
        self.cancels.append((command_id, reason))
        return self.cancel_outcome


class RecordingStopPort:
    def __init__(self) -> None:
        self.calls: list[tuple[str, str]] = []
        self.outcome = StopAllOutcome(True, True, "stopped")
        self.error: Exception | None = None

    def stop_all(self, task_id: str, reason: str) -> StopAllOutcome:
        self.calls.append((task_id, reason))
        if self.error is not None:
            raise self.error
        return self.outcome


def make_controller():
    arm = RecordingArmPort()
    gripper = RecordingGripperPort()
    stop = RecordingStopPort()
    return (
        GraspSequenceController(
            arm,
            gripper,
            stop,
            clock_domain="ros_system",
        ),
        arm,
        gripper,
        stop,
    )


def task(
    *,
    task_id: str = "pick-cube-1",
    source_timestamp_ns: int = NOW_NS - 10_000_000,
) -> GraspTaskSnapshot:
    return GraspTaskSnapshot(
        task_id=task_id,
        target_id="cube-1",
        approach_position_m=Vector3(0.25, 0.0, 0.18),
        descend_position_m=Vector3(0.25, 0.0, 0.11),
        lift_position_m=Vector3(0.25, 0.0, 0.24),
        approach_orientation_xyzw=APPROACH_ORIENTATION,
        grasp_orientation_xyzw=GRASP_ORIENTATION,
        gripper_closed_position_rad=0.2,
        source_timestamp_ns=source_timestamp_ns,
        frame_id="base_link",
        clock_domain="ros_system",
        clock_epoch=0,
    )


def health(
    now_ns: int,
    *,
    target_id: str = "cube-1",
    target_source_timestamp_ns: int = NOW_NS - 10_000_000,
    permission: bool = True,
    interface: bool = True,
    target_ready: bool = True,
    joint_state: bool = True,
) -> SequenceHealth:
    return SequenceHealth(
        target_id=target_id,
        target_source_timestamp_ns=target_source_timestamp_ns,
        permission=SignalSnapshot(permission, now_ns, "permission"),
        interface=SignalSnapshot(interface, now_ns, "interface"),
        target=SignalSnapshot(target_ready, now_ns, "target"),
        joint_state=SignalSnapshot(joint_state, now_ns, "joint_state"),
        clock_domain="ros_system",
        clock_epoch=0,
    )


def arm_terminal(
    command: ArmPlanCommand,
    *,
    status: int = ACTION_STATUS_SUCCEEDED,
    success: bool = True,
    cancel_requested: bool = False,
) -> ArmStageTerminal:
    return ArmStageTerminal(
        task_id=command.task_id,
        command_id=command.command_id,
        target_id=command.target_id,
        stage=command.stage,
        sequence_no=command.sequence_no,
        accepted=True,
        success=success,
        reason="succeeded" if success else "canceled",
        moveit_error_code=MOVEIT_SUCCESS,
        source_timestamp_ns=command.source_timestamp_ns,
        clock_domain=command.clock_domain,
        clock_epoch=command.clock_epoch,
        trajectory_dispatched=True,
        trajectory_digest=DIGEST,
        gate_accepted=True,
        gate_terminal=True,
        downstream_terminal_observed=True,
        cancel_requested=cancel_requested,
        plan_action_goal_status=ACTION_STATUS_SUCCEEDED,
        action_goal_status=status,
        fjt_error_code=FJT_SUCCESSFUL,
        fjt_error_string="",
    )


def gripper_terminal(
    command: GripperExecutionCommand,
    *,
    status: int = ACTION_STATUS_SUCCEEDED,
    success: bool = True,
    completed_timestamp_ns: int = NOW_NS + 3,
) -> GripperStageTerminal:
    return GripperStageTerminal(
        task_id=command.task_id,
        command_id=command.command_id,
        target_id=command.target_id,
        stage=command.stage,
        sequence_no=command.sequence_no,
        controller="gripper_controller",
        trajectory_digest=DIGEST,
        accepted=True,
        terminal=True,
        downstream_terminal_observed=True,
        cancel_requested=not success,
        gate_action_goal_status=ACTION_STATUS_SUCCEEDED,
        action_goal_status=status,
        fjt_error_code=FJT_SUCCESSFUL,
        fjt_error_string="",
        success=success,
        reason="succeeded" if success else "canceled",
        source_timestamp_ns=command.source_timestamp_ns,
        completed_timestamp_ns=completed_timestamp_ns,
        clock_domain=command.clock_domain,
        clock_epoch=command.clock_epoch,
    )


def test_four_correlated_stages_complete_without_claiming_physical_grasp() -> None:
    controller, arm, gripper, stop = make_controller()

    assert controller.start(task(), health(NOW_NS), NOW_NS).accepted
    assert controller.phase is GraspPhase.APPROACH_PLAN
    assert [(item.stage, item.sequence_no) for item in arm.commands] == [
        ("approach", 0)
    ]
    assert arm.commands[0].target_orientation_xyzw == APPROACH_ORIENTATION

    approach = arm_terminal(arm.commands[0])
    assert controller.on_arm_terminal(
        approach, health(NOW_NS + 1), NOW_NS + 1
    ).accepted
    assert controller.phase is GraspPhase.DESCEND_PLAN

    descend = arm_terminal(arm.commands[1])
    assert controller.on_arm_terminal(
        descend, health(NOW_NS + 2), NOW_NS + 2
    ).accepted
    assert controller.phase is GraspPhase.CLOSE_GRIPPER_EXEC
    assert gripper.commands[0].sequence_no == 2

    closed = gripper_terminal(gripper.commands[0])
    assert controller.on_gripper_terminal(
        closed, health(NOW_NS + 3), NOW_NS + 3
    ).accepted
    assert controller.phase is GraspPhase.LIFT_PLAN

    lift = arm_terminal(arm.commands[2])
    result = controller.on_arm_terminal(lift, health(NOW_NS + 4), NOW_NS + 4)
    assert result.accepted
    assert controller.phase is GraspPhase.COMPLETE
    assert [(item.stage, item.sequence_no) for item in arm.commands] == [
        ("approach", 0),
        ("descend", 1),
        ("lift", 3),
    ]
    assert [item.target_orientation_xyzw for item in arm.commands] == [
        APPROACH_ORIENTATION,
        GRASP_ORIENTATION,
        GRASP_ORIENTATION,
    ]
    assert len(gripper.commands) == 1
    assert stop.calls == []
    assert controller.outcome is not None
    assert controller.outcome.sequence_completed
    assert not controller.outcome.physics_grasp_verified


def test_each_stage_binds_a_fresh_target_source_timestamp() -> None:
    controller, arm, gripper, _ = make_controller()
    initial_source_ns = NOW_NS - 10_000_000
    assert controller.start(
        task(source_timestamp_ns=initial_source_ns),
        health(NOW_NS, target_source_timestamp_ns=initial_source_ns),
        NOW_NS,
    ).accepted
    assert arm.commands[0].source_timestamp_ns == initial_source_ns

    approach_now = NOW_NS + 300_000_000
    approach_source = approach_now - 5_000_000
    assert controller.on_arm_terminal(
        arm_terminal(arm.commands[0]),
        health(approach_now, target_source_timestamp_ns=approach_source),
        approach_now,
    ).accepted
    assert arm.commands[1].source_timestamp_ns == approach_source

    descend_now = NOW_NS + 600_000_000
    descend_source = descend_now - 5_000_000
    assert controller.on_arm_terminal(
        arm_terminal(arm.commands[1]),
        health(descend_now, target_source_timestamp_ns=descend_source),
        descend_now,
    ).accepted
    assert gripper.commands[0].source_timestamp_ns == descend_source

    gripper_now = NOW_NS + 900_000_000
    gripper_source = gripper_now - 5_000_000
    assert controller.on_gripper_terminal(
        gripper_terminal(
            gripper.commands[0], completed_timestamp_ns=gripper_now
        ),
        health(gripper_now, target_source_timestamp_ns=gripper_source),
        gripper_now,
    ).accepted
    assert arm.commands[2].source_timestamp_ns == gripper_source

    lift_now = NOW_NS + 1_200_000_000
    lift_source = lift_now - 5_000_000
    result = controller.on_arm_terminal(
        arm_terminal(arm.commands[2]),
        health(lift_now, target_source_timestamp_ns=lift_source),
        lift_now,
    )
    assert result.accepted
    assert controller.phase is GraspPhase.COMPLETE


def test_task_and_initial_health_source_timestamps_must_match() -> None:
    controller, arm, _, _ = make_controller()
    decision = controller.start(
        task(),
        health(NOW_NS, target_source_timestamp_ns=NOW_NS - 9_000_000),
        NOW_NS,
    )
    assert not decision.accepted
    assert decision.reason == "task_health_source_timestamp_mismatch"
    assert controller.phase is GraspPhase.SAFE_STOP
    assert arm.commands == []


def test_target_source_timestamp_rollback_is_fail_closed() -> None:
    controller, arm, _, _ = make_controller()
    controller.start(task(), health(NOW_NS), NOW_NS)
    decision = controller.tick(
        health(
            NOW_NS + 1,
            target_source_timestamp_ns=NOW_NS - 10_000_001,
        ),
        NOW_NS + 1,
    )
    assert not decision.accepted
    assert decision.reason == "target_source_timestamp_rollback"
    assert controller.phase is GraspPhase.SAFE_STOP
    assert len(arm.cancels) == 1


def test_target_source_staleness_is_fail_closed_during_a_sequence() -> None:
    controller, arm, _, _ = make_controller()
    controller.start(task(), health(NOW_NS), NOW_NS)
    now_ns = NOW_NS + 201_000_000
    decision = controller.tick(health(now_ns), now_ns)
    assert not decision.accepted
    assert decision.reason == "target_source_stale"
    assert controller.phase is GraspPhase.SAFE_STOP
    assert len(arm.cancels) == 1


def test_arm_terminal_must_echo_the_active_stage_source_timestamp() -> None:
    controller, arm, _, _ = make_controller()
    controller.start(task(), health(NOW_NS), NOW_NS)
    terminal = replace(
        arm_terminal(arm.commands[0]),
        source_timestamp_ns=arm.commands[0].source_timestamp_ns + 1,
    )
    decision = controller.on_arm_terminal(
        terminal,
        health(NOW_NS + 1),
        NOW_NS + 1,
    )
    assert not decision.accepted
    assert decision.reason == "terminal_source_timestamp_mismatch"
    assert controller.phase is GraspPhase.SAFE_STOP


def test_completed_terminal_duplicate_is_idempotently_ignored() -> None:
    controller, arm, _, _ = make_controller()
    controller.start(task(), health(NOW_NS), NOW_NS)
    terminal = arm_terminal(arm.commands[0])
    controller.on_arm_terminal(terminal, health(NOW_NS + 1), NOW_NS + 1)

    duplicate = controller.on_arm_terminal(
        terminal, health(NOW_NS + 2), NOW_NS + 2
    )
    assert duplicate.ignored
    assert duplicate.reason == "completed_duplicate"
    assert controller.phase is GraspPhase.DESCEND_PLAN
    assert len(arm.commands) == 2


def test_correlated_terminal_must_match_target_identity() -> None:
    controller, arm, _, stop = make_controller()
    controller.start(task(), health(NOW_NS), NOW_NS)

    decision = controller.on_arm_terminal(
        replace(arm_terminal(arm.commands[0]), target_id="different-cube"),
        health(NOW_NS + 1),
        NOW_NS + 1,
    )
    assert not decision.accepted
    assert controller.phase is GraspPhase.SAFE_STOP
    assert len(arm.commands) == 1
    assert len(stop.calls) == 1


def test_invalid_task_type_fails_closed_instead_of_raising() -> None:
    controller, _, _, stop = make_controller()
    decision = controller.start(object(), health(NOW_NS), NOW_NS)  # type: ignore[arg-type]
    assert not decision.accepted
    assert decision.reason == "invalid_task_type"
    assert controller.phase is GraspPhase.SAFE_STOP
    assert stop.calls == [("unassigned", "invalid_task_type")]


@pytest.mark.parametrize(
    ("field", "kwargs"),
    [
        ("permission", {"permission": False}),
        ("interface", {"interface": False}),
        ("target", {"target_ready": False}),
        ("joint_state", {"joint_state": False}),
    ],
)
def test_health_loss_cancels_and_stops_once(field: str, kwargs: dict) -> None:
    controller, arm, _, stop = make_controller()
    controller.start(task(), health(NOW_NS), NOW_NS)

    decision = controller.tick(health(NOW_NS + 1, **kwargs), NOW_NS + 1)
    assert not decision.accepted
    assert field in decision.reason
    assert controller.phase is GraspPhase.SAFE_STOP
    assert len(arm.cancels) == 1
    assert len(stop.calls) == 1

    late_tick = controller.tick(health(NOW_NS + 2, **kwargs), NOW_NS + 2)
    assert late_tick.ignored
    assert len(arm.cancels) == 1
    assert len(stop.calls) == 1


@pytest.mark.parametrize(
    ("field", "timeout_ms"),
    [
        ("permission", 100.0),
        ("interface", 500.0),
        ("target", 200.0),
        ("joint_state", 250.0),
    ],
)
def test_signal_specific_liveness_timeout_boundary_is_fail_closed(
    field: str, timeout_ms: float
) -> None:
    arm = RecordingArmPort()
    gripper = RecordingGripperPort()
    stop = RecordingStopPort()
    controller = GraspSequenceController(
        arm,
        gripper,
        stop,
        clock_domain="ros_system",
        health_timeout_ms=1_000.0,
        permission_timeout_ms=100.0,
        interface_timeout_ms=500.0,
        target_receive_timeout_ms=200.0,
        joint_state_timeout_ms=250.0,
    )
    assert controller.start(task(), health(NOW_NS), NOW_NS).accepted

    boundary_ns = NOW_NS + int(timeout_ms * 1_000_000)
    boundary_health = health(
        boundary_ns, target_source_timestamp_ns=boundary_ns
    )
    boundary_health = replace(
        boundary_health,
        **{
            field: replace(
                getattr(boundary_health, field), observed_at_ns=NOW_NS
            )
        },
    )
    assert controller.tick(boundary_health, boundary_ns).accepted

    stale_ns = boundary_ns + 1
    stale_health = health(stale_ns, target_source_timestamp_ns=stale_ns)
    stale_health = replace(
        stale_health,
        **{
            field: replace(
                getattr(stale_health, field), observed_at_ns=NOW_NS
            )
        },
    )
    decision = controller.tick(stale_health, stale_ns)

    assert not decision.accepted
    assert decision.reason == f"{field}_stale"
    assert controller.phase is GraspPhase.SAFE_STOP
    assert len(arm.cancels) == 1
    assert len(stop.calls) == 1


def test_gate_failure_never_dispatches_the_next_stage() -> None:
    controller, arm, _, stop = make_controller()
    controller.start(task(), health(NOW_NS), NOW_NS)
    failed = replace(
        arm_terminal(arm.commands[0]),
        success=False,
        gate_terminal=False,
        downstream_terminal_observed=False,
        reason="gate_timeout",
    )

    decision = controller.on_arm_terminal(
        failed, health(NOW_NS + 1), NOW_NS + 1
    )
    assert not decision.accepted
    assert controller.phase is GraspPhase.SAFE_STOP
    assert len(arm.commands) == 1
    assert len(stop.calls) == 1


@pytest.mark.parametrize(
    "changes",
    [
        {"accepted": False},
        {"success": False},
        {"moveit_error_code": -1},
        {"trajectory_dispatched": False},
        {"trajectory_digest": "bad"},
        {"gate_accepted": False},
        {"gate_terminal": False},
        {"downstream_terminal_observed": False},
        {"cancel_requested": True},
        {"plan_action_goal_status": 6},
        {"action_goal_status": 6},
        {"fjt_error_code": -4},
    ],
)
def test_every_arm_terminal_contract_failure_blocks_next_stage(changes: dict) -> None:
    controller, arm, _, stop = make_controller()
    controller.start(task(), health(NOW_NS), NOW_NS)
    terminal = replace(arm_terminal(arm.commands[0]), **changes)

    decision = controller.on_arm_terminal(
        terminal, health(NOW_NS + 1), NOW_NS + 1
    )
    assert not decision.accepted
    assert controller.phase is GraspPhase.SAFE_STOP
    assert len(arm.commands) == 1
    assert len(stop.calls) == 1


def test_gripper_terminal_failure_and_target_mismatch_never_issue_lift() -> None:
    for changes in (
        {"fjt_error_code": -4, "success": False},
        {"gate_action_goal_status": 6, "success": False},
        {"target_id": "different-cube"},
    ):
        controller, arm, gripper, stop = make_controller()
        controller.start(task(), health(NOW_NS), NOW_NS)
        controller.on_arm_terminal(
            arm_terminal(arm.commands[0]), health(NOW_NS + 1), NOW_NS + 1
        )
        controller.on_arm_terminal(
            arm_terminal(arm.commands[1]), health(NOW_NS + 2), NOW_NS + 2
        )

        terminal = replace(gripper_terminal(gripper.commands[0]), **changes)
        decision = controller.on_gripper_terminal(
            terminal, health(NOW_NS + 3), NOW_NS + 3
        )
        assert not decision.accepted
        assert controller.phase is GraspPhase.SAFE_STOP
        assert len(arm.commands) == 2
        assert len(stop.calls) == 1


@pytest.mark.parametrize(
    ("completed_timestamp_ns", "reason"),
    [
        (-1, "invalid_terminal_timestamp"),
        (NOW_NS + 1, "terminal_timestamp_before_issue"),
        (NOW_NS + 4, "terminal_timestamp_future"),
    ],
)
def test_gripper_terminal_timestamp_is_bounded_by_issue_and_receive(
    completed_timestamp_ns: int, reason: str
) -> None:
    controller, arm, gripper, _ = make_controller()
    controller.start(task(), health(NOW_NS), NOW_NS)
    controller.on_arm_terminal(
        arm_terminal(arm.commands[0]), health(NOW_NS + 1), NOW_NS + 1
    )
    controller.on_arm_terminal(
        arm_terminal(arm.commands[1]), health(NOW_NS + 2), NOW_NS + 2
    )
    terminal = replace(
        gripper_terminal(gripper.commands[0]),
        completed_timestamp_ns=completed_timestamp_ns,
    )
    decision = controller.on_gripper_terminal(
        terminal, health(NOW_NS + 3), NOW_NS + 3
    )
    assert not decision.accepted
    assert decision.reason == reason
    assert controller.phase is GraspPhase.SAFE_STOP


def test_command_timeout_is_fail_closed_and_idempotent() -> None:
    controller, arm, _, stop = make_controller()
    controller.command_timeout_ns = 10
    controller.start(task(), health(NOW_NS), NOW_NS)

    decision = controller.tick(health(NOW_NS + 11), NOW_NS + 11)
    assert not decision.accepted
    assert decision.reason == "command_terminal_timeout"
    assert controller.phase is GraspPhase.SAFE_STOP
    assert len(arm.cancels) == 1
    assert len(stop.calls) == 1


def test_explicit_stop_request_cancels_once_and_latches() -> None:
    controller, arm, _, stop = make_controller()
    controller.start(task(), health(NOW_NS), NOW_NS)

    decision = controller.request_stop("client_cancel", NOW_NS + 1)

    assert not decision.accepted
    assert decision.reason == "stop_requested:client_cancel"
    assert controller.phase is GraspPhase.SAFE_STOP
    assert len(arm.cancels) == 1
    assert len(stop.calls) == 1
    assert controller.request_stop("again", NOW_NS + 2).ignored
    assert len(arm.cancels) == 1
    assert len(stop.calls) == 1

    controller.tick(health(NOW_NS + 12), NOW_NS + 12)
    assert len(arm.cancels) == 1
    assert len(stop.calls) == 1


def test_terminal_callback_cannot_bypass_command_timeout_without_tick() -> None:
    controller, arm, _, stop = make_controller()
    controller.command_timeout_ns = 10
    controller.start(task(), health(NOW_NS), NOW_NS)

    decision = controller.on_arm_terminal(
        arm_terminal(arm.commands[0]),
        health(NOW_NS + 11),
        NOW_NS + 11,
    )
    assert not decision.accepted
    assert decision.reason == "command_terminal_timeout"
    assert controller.phase is GraspPhase.SAFE_STOP
    assert len(arm.commands) == 1
    assert len(stop.calls) == 1


def test_clock_rollback_latches_and_requires_a_new_epoch() -> None:
    controller, _, _, _ = make_controller()
    controller.start(task(), health(NOW_NS), NOW_NS)

    decision = controller.tick(health(NOW_NS - 1), NOW_NS - 1)
    assert not decision.accepted
    assert decision.reason.startswith("clock_rollback")
    assert controller.phase is GraspPhase.SAFE_STOP
    assert not controller.reset(
        NOW_NS + 1, external_stop_confirmed=True
    ).success
    reset = controller.reset(
        NOW_NS + 1,
        new_clock_epoch=1,
        external_stop_confirmed=True,
    )
    assert reset.success
    assert controller.clock_epoch == 1


def test_terminal_epoch_mismatch_requires_epoch_advancing_reset() -> None:
    controller, arm, _, _ = make_controller()
    controller.start(task(), health(NOW_NS), NOW_NS)
    terminal = replace(arm_terminal(arm.commands[0]), clock_epoch=1)

    decision = controller.on_arm_terminal(
        terminal, health(NOW_NS + 1), NOW_NS + 1
    )
    assert not decision.accepted
    assert decision.reason == "terminal_clock_epoch_mismatch"
    assert not controller.reset(NOW_NS + 2).success
    assert controller.reset(NOW_NS + 2, new_clock_epoch=1).success


def test_late_downstream_terminal_confirms_stop_but_never_resumes_sequence() -> None:
    controller, arm, _, stop = make_controller()
    arm.cancel_outcome = CancelOutcome(
        CancelDisposition.ACCEPTED_PENDING, "cancel accepted"
    )
    stop.outcome = StopAllOutcome(True, True, "controllers stopped")
    controller.start(task(), health(NOW_NS), NOW_NS)
    active_command = arm.commands[0]

    controller.tick(
        health(NOW_NS + 1, permission=False), NOW_NS + 1
    )
    assert controller.phase is GraspPhase.SAFE_STOP
    assert controller.recovery_required

    late = arm_terminal(
        active_command,
        status=5,
        success=False,
        cancel_requested=True,
    )
    decision = controller.on_arm_terminal(
        late, health(NOW_NS + 2), NOW_NS + 2
    )
    assert decision.ignored
    assert controller.phase is GraspPhase.SAFE_STOP
    assert not controller.recovery_required
    assert len(arm.commands) == 1
    assert len(stop.calls) == 1
    assert controller.reset(NOW_NS + 3).success


def test_task_id_cannot_be_reused_after_reset() -> None:
    controller, arm, gripper, stop = make_controller()
    first = task(task_id="attempt-1")
    controller.start(first, health(NOW_NS), NOW_NS)
    controller.on_arm_terminal(
        arm_terminal(arm.commands[0]), health(NOW_NS + 1), NOW_NS + 1
    )
    controller.on_arm_terminal(
        arm_terminal(arm.commands[1]), health(NOW_NS + 2), NOW_NS + 2
    )
    controller.on_gripper_terminal(
        gripper_terminal(gripper.commands[0]),
        health(NOW_NS + 3),
        NOW_NS + 3,
    )
    controller.on_arm_terminal(
        arm_terminal(arm.commands[2]), health(NOW_NS + 4), NOW_NS + 4
    )
    assert controller.reset(NOW_NS + 5).success

    reused = controller.start(first, health(NOW_NS + 6), NOW_NS + 6)
    assert not reused.accepted
    assert reused.reason == "task_id_reuse"
    assert controller.phase is GraspPhase.SAFE_STOP
    assert len(arm.commands) == 3
    assert len(stop.calls) == 1


def test_unconfirmed_late_result_does_not_unlock_reset() -> None:
    controller, arm, _, stop = make_controller()
    arm.cancel_outcome = CancelOutcome(
        CancelDisposition.ACCEPTED_PENDING, "cancel accepted"
    )
    stop.outcome = StopAllOutcome(True, True, "controllers stopped")
    controller.start(task(), health(NOW_NS), NOW_NS)
    active = arm.commands[0]
    controller.tick(health(NOW_NS + 1, permission=False), NOW_NS + 1)

    late = replace(
        arm_terminal(active),
        downstream_terminal_observed=False,
        gate_terminal=False,
    )
    decision = controller.on_arm_terminal(
        late, health(NOW_NS + 2), NOW_NS + 2
    )
    assert decision.ignored
    assert decision.reason == "late_terminal_unconfirmed"
    assert controller.recovery_required
    assert not controller.reset(NOW_NS + 3).success


@pytest.mark.parametrize(
    "changes",
    [
        {"plan_action_goal_status": 0},
        {"action_goal_status": 0},
    ],
)
def test_unknown_arm_action_status_is_not_downstream_terminal_evidence(
    changes: dict,
) -> None:
    controller, arm, _, _ = make_controller()
    controller.start(task(), health(NOW_NS), NOW_NS)

    decision = controller.on_arm_terminal(
        replace(arm_terminal(arm.commands[0]), success=False, **changes),
        health(NOW_NS + 1),
        NOW_NS + 1,
    )

    assert not decision.accepted
    assert controller.phase is GraspPhase.SAFE_STOP
    assert len(arm.commands) == 1
    assert len(arm.cancels) == 1


@pytest.mark.parametrize(
    "changes",
    [
        {"gate_action_goal_status": 0},
        {"action_goal_status": 0},
    ],
)
def test_unknown_gripper_action_status_is_not_downstream_terminal_evidence(
    changes: dict,
) -> None:
    controller, arm, gripper, _ = make_controller()
    controller.start(task(), health(NOW_NS), NOW_NS)
    controller.on_arm_terminal(
        arm_terminal(arm.commands[0]), health(NOW_NS + 1), NOW_NS + 1
    )
    controller.on_arm_terminal(
        arm_terminal(arm.commands[1]), health(NOW_NS + 2), NOW_NS + 2
    )

    decision = controller.on_gripper_terminal(
        replace(gripper_terminal(gripper.commands[0]), success=False, **changes),
        health(NOW_NS + 3),
        NOW_NS + 3,
    )

    assert not decision.accepted
    assert controller.phase is GraspPhase.SAFE_STOP
    assert len(arm.commands) == 2
    assert len(gripper.cancels) == 1


def test_late_terminal_with_unknown_wrapper_status_keeps_recovery_latched() -> None:
    controller, arm, _, stop = make_controller()
    arm.cancel_outcome = CancelOutcome(
        CancelDisposition.ACCEPTED_PENDING, "cancel accepted"
    )
    stop.outcome = StopAllOutcome(True, True, "controllers stopped")
    controller.start(task(), health(NOW_NS), NOW_NS)
    active = arm.commands[0]
    controller.tick(health(NOW_NS + 1, permission=False), NOW_NS + 1)

    late = replace(arm_terminal(active), plan_action_goal_status=0)
    decision = controller.on_arm_terminal(
        late, health(NOW_NS + 2), NOW_NS + 2
    )

    assert decision.ignored
    assert decision.reason == "late_terminal_unconfirmed"
    assert controller.recovery_required
    assert not controller.reset(NOW_NS + 3).success


def test_malformed_terminal_and_health_fail_closed_without_escaping() -> None:
    controller, arm, _, stop = make_controller()
    controller.start(task(), health(NOW_NS), NOW_NS)
    terminal_decision = controller.on_arm_terminal(  # type: ignore[arg-type]
        object(), health(NOW_NS + 1), NOW_NS + 1
    )
    assert terminal_decision.reason == "invalid_arm_terminal_type"
    assert controller.phase is GraspPhase.SAFE_STOP
    assert len(arm.cancels) == 1
    assert len(stop.calls) == 1

    controller, arm, _, stop = make_controller()
    controller.start(task(task_id="malformed-health"), health(NOW_NS), NOW_NS)
    health_decision = controller.tick(object(), NOW_NS + 1)  # type: ignore[arg-type]
    assert health_decision.reason == "invalid_health_type"
    assert controller.phase is GraspPhase.SAFE_STOP
    assert len(arm.cancels) == 1
    assert len(stop.calls) == 1


def test_terminal_digest_type_failure_is_fail_closed_not_type_error() -> None:
    controller, arm, _, _ = make_controller()
    controller.start(task(), health(NOW_NS), NOW_NS)
    terminal = replace(
        arm_terminal(arm.commands[0]),
        trajectory_digest=object(),  # type: ignore[arg-type]
    )
    decision = controller.on_arm_terminal(
        terminal, health(NOW_NS + 1), NOW_NS + 1
    )
    assert not decision.accepted
    assert controller.phase is GraspPhase.SAFE_STOP


def test_concurrent_duplicate_terminal_advances_exactly_once() -> None:
    controller, arm, _, _ = make_controller()
    controller.start(task(), health(NOW_NS), NOW_NS)
    terminal = arm_terminal(arm.commands[0])

    with ThreadPoolExecutor(max_workers=2) as pool:
        decisions = list(
            pool.map(
                lambda _: controller.on_arm_terminal(
                    terminal, health(NOW_NS + 1), NOW_NS + 1
                ),
                range(2),
            )
        )

    assert sum(decision.accepted for decision in decisions) == 1
    assert sum(decision.ignored for decision in decisions) == 1
    assert len(arm.commands) == 2
    assert controller.phase is GraspPhase.DESCEND_PLAN


def test_stop_exception_requires_external_confirmation_before_reset() -> None:
    controller, arm, _, stop = make_controller()
    stop.error = RuntimeError("stop")
    controller.start(task(), health(NOW_NS), NOW_NS)
    controller.tick(health(NOW_NS + 1, permission=False), NOW_NS + 1)

    assert controller.phase is GraspPhase.SAFE_STOP
    assert controller.recovery_required
    assert len(arm.cancels) == 1
    assert not controller.reset(NOW_NS + 2).success
    assert controller.reset(
        NOW_NS + 2, external_stop_confirmed=True
    ).success


def test_concurrent_start_and_reset_while_active_are_rejected() -> None:
    controller, arm, _, _ = make_controller()
    controller.start(task(), health(NOW_NS), NOW_NS)

    second = controller.start(task(), health(NOW_NS + 1), NOW_NS + 1)
    assert not second.accepted
    assert second.reason == "sequence_not_idle"
    assert len(arm.commands) == 1
    reset = controller.reset(NOW_NS + 1)
    assert not reset.success
    assert reset.reason == "reset_requires_terminal_phase"


def test_submit_exception_is_fail_closed_and_no_followup_is_issued() -> None:
    controller, arm, _, stop = make_controller()
    arm.submit_error = RuntimeError("transport")
    decision = controller.start(task(), health(NOW_NS), NOW_NS)
    assert not decision.accepted
    assert decision.reason == "arm_submit_exception:RuntimeError"
    assert controller.phase is GraspPhase.SAFE_STOP
    assert len(arm.commands) == 1
    assert len(stop.calls) == 1


def test_signal_snapshot_requires_a_real_boolean() -> None:
    with pytest.raises(TypeError, match="ready must be a boolean"):
        SignalSnapshot(1, NOW_NS)  # type: ignore[arg-type]


def test_sequence_health_requires_typed_signal_snapshots() -> None:
    signal = SignalSnapshot(True, NOW_NS)
    with pytest.raises(TypeError, match="permission must be SignalSnapshot"):
        SequenceHealth(
            target_id="cube-1",
            target_source_timestamp_ns=NOW_NS,
            permission=object(),  # type: ignore[arg-type]
            interface=signal,
            target=signal,
            joint_state=signal,
            clock_domain="ros_system",
        )


def test_sequence_effects_are_deterministic_across_100_runs() -> None:
    def run_once() -> tuple:
        controller, arm, gripper, stop = make_controller()
        controller.start(task(), health(NOW_NS), NOW_NS)
        controller.on_arm_terminal(
            arm_terminal(arm.commands[0]), health(NOW_NS + 1), NOW_NS + 1
        )
        controller.on_arm_terminal(
            arm_terminal(arm.commands[1]), health(NOW_NS + 2), NOW_NS + 2
        )
        controller.on_gripper_terminal(
            gripper_terminal(gripper.commands[0]),
            health(NOW_NS + 3),
            NOW_NS + 3,
        )
        controller.on_arm_terminal(
            arm_terminal(arm.commands[2]), health(NOW_NS + 4), NOW_NS + 4
        )
        return (
            tuple(command.command_id for command in arm.commands),
            tuple(command.command_id for command in gripper.commands),
            tuple(
                (item.previous.value, item.current.value, item.reason)
                for item in controller.history
            ),
            tuple(stop.calls),
            controller.outcome,
        )

    baseline = run_once()
    assert all(run_once() == baseline for _ in range(99))


def test_outcome_value_objects_reject_non_boolean_or_untyped_fields() -> None:
    with pytest.raises(TypeError, match="accepted must be a boolean"):
        DispatchOutcome(1, "bad")  # type: ignore[arg-type]
    with pytest.raises(TypeError, match="disposition must be CancelDisposition"):
        CancelOutcome("accepted_pending", "bad")  # type: ignore[arg-type]
    with pytest.raises(TypeError, match="arm_terminal_confirmed must be a boolean"):
        StopAllOutcome(1, True, "bad")  # type: ignore[arg-type]
    with pytest.raises(TypeError, match="gripper_terminal_confirmed must be a boolean"):
        StopAllOutcome(True, 1, "bad")  # type: ignore[arg-type]


def test_malformed_cancel_and_stop_port_outcomes_remain_fail_closed() -> None:
    controller, arm, _, stop = make_controller()
    controller.start(task(), health(NOW_NS), NOW_NS)
    arm.cancel_outcome = object()  # type: ignore[assignment]
    stop.outcome = object()  # type: ignore[assignment]

    decision = controller.tick(
        health(NOW_NS + 1, permission=False), NOW_NS + 1
    )

    assert not decision.accepted
    assert controller.phase is GraspPhase.SAFE_STOP
    assert controller.recovery_required
    assert "cancel_invalid_outcome" in controller.fault_evidence
    assert "stop_all_invalid_outcome" in controller.fault_evidence


def test_malformed_terminal_during_clock_rollback_requires_new_epoch() -> None:
    controller, _, _, _ = make_controller()
    controller.start(task(), health(NOW_NS), NOW_NS)

    decision = controller.on_arm_terminal(  # type: ignore[arg-type]
        object(), health(NOW_NS - 1), NOW_NS - 1
    )

    assert not decision.accepted
    assert controller.phase is GraspPhase.SAFE_STOP
    assert any(item.startswith("clock_rollback") for item in controller.fault_evidence)
    assert not controller.reset(
        NOW_NS + 1, external_stop_confirmed=True
    ).success
    assert controller.reset(
        NOW_NS + 1,
        new_clock_epoch=1,
        external_stop_confirmed=True,
    ).success


def test_late_terminal_from_reset_task_cannot_fault_or_advance_new_task() -> None:
    controller, arm, _, _ = make_controller()
    controller.start(task(task_id="attempt-old"), health(NOW_NS), NOW_NS)
    old_terminal = arm_terminal(arm.commands[0])
    controller.tick(
        health(NOW_NS + 1, permission=False), NOW_NS + 1
    )
    assert controller.reset(NOW_NS + 2).success

    assert controller.start(
        task(task_id="attempt-new"), health(NOW_NS + 3), NOW_NS + 3
    ).accepted
    command_count = len(arm.commands)

    late = controller.on_arm_terminal(
        old_terminal, health(NOW_NS + 4), NOW_NS + 4
    )

    assert late.ignored
    assert late.reason == "late_event_from_prior_task"
    assert controller.phase is GraspPhase.APPROACH_PLAN
    assert len(arm.commands) == command_count
