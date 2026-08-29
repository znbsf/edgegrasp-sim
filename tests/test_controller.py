from __future__ import annotations

from dataclasses import replace
import inspect

import pytest

from edgegrasp.backend import (
    BackendKind,
    DisabledBackend,
    ExecutionResult,
    MockBackend,
    create_backend,
)
from edgegrasp.controller import EdgeGraspController, ResultCode
from edgegrasp.factory import build_controller
from edgegrasp.fsm import PipelineState
from edgegrasp.models import MotionPlan, Prediction, Target3D, Vector3
from edgegrasp.planner import (
    AxisAlignedBox,
    EndpointWorkspaceGate,
    PlanReason,
    PlanResult,
)
from edgegrasp.predictor import ConstantVelocityPredictor
from edgegrasp.safety import StaleTargetGate


RECEIVE_NS = 1_010_000_000
EXECUTE_NS = 1_011_000_000


def make_target(
    *,
    timestamp_ns: int = 1_000_000_000,
    frame_id: str = "base_link",
    confidence: float = 1.0,
    clock_domain: str = "sim",
    clock_epoch: int = 0,
    x_m: float = 0.15,
) -> Target3D:
    return Target3D(
        "cube",
        Vector3(x_m, 0.0, 0.1),
        timestamp_ns,
        frame_id=frame_id,
        confidence=confidence,
        clock_domain=clock_domain,
        clock_epoch=clock_epoch,
    )


def make_controller(
    backend: MockBackend | None = None,
    planner: object | None = None,
) -> tuple[EdgeGraspController, MockBackend]:
    selected_backend = backend or MockBackend()
    return (
        EdgeGraspController(
            ConstantVelocityPredictor(),
            planner or EndpointWorkspaceGate(),  # type: ignore[arg-type]
            StaleTargetGate(max_age_ms=200),
            selected_backend,
            prediction_horizon_ms=0,
        ),
        selected_backend,
    )


def process(
    controller: EdgeGraspController,
    target: Target3D | None = None,
    receive_ns: int = RECEIVE_NS,
    execute_ns: int = EXECUTE_NS,
):
    return controller.process_target(target or make_target(), receive_ns, execute_ns)


def plan_for(prediction: Prediction, now_ns: int) -> MotionPlan:
    return MotionPlan(
        target_id=prediction.target_id,
        waypoints_m=(prediction.position_m,),
        source_timestamp_ns=prediction.source_timestamp_ns,
        requested_at_ns=now_ns,
        frame_id=prediction.frame_id,
        clock_domain=prediction.clock_domain,
        clock_epoch=prediction.clock_epoch,
    )


def test_fresh_reachable_target_executes_on_synchronous_mock_backend() -> None:
    controller, backend = make_controller()

    result = process(controller)

    assert result.accepted
    assert result.code is ResultCode.EXECUTED
    assert result.plan_reason is PlanReason.PLANNED
    assert controller.state is PipelineState.TRACKING
    assert len(backend.executed_plans) == 1
    assert backend.stop_reasons == []


def test_stale_target_latches_safe_stop_and_refuses_motion() -> None:
    controller, backend = make_controller()

    result = process(
        controller,
        receive_ns=1_200_000_001,
        execute_ns=1_200_000_002,
    )

    assert not result.accepted
    assert result.reason == "stale_target"
    assert controller.state is PipelineState.SAFE_STOP
    assert backend.executed_plans == []
    assert backend.stop_reasons == ["stale_target"]

    second = process(
        controller,
        make_target(timestamp_ns=1_300_000_000),
        receive_ns=1_300_000_000,
        execute_ns=1_301_000_000,
    )
    assert not second.accepted
    assert second.code is ResultCode.LATCHED


def test_unreachable_target_is_rejected_before_backend() -> None:
    controller, backend = make_controller()

    result = process(controller, make_target(x_m=0.80))

    assert not result.accepted
    assert result.reason == "unreachable"
    assert result.code is ResultCode.PLAN_REJECTED
    assert result.plan_reason is PlanReason.UNREACHABLE
    assert controller.state is PipelineState.SAFE_STOP
    assert backend.executed_plans == []


def test_blocked_endpoint_is_rejected_without_collision_claim() -> None:
    blocked = AxisAlignedBox(
        Vector3(0.10, -0.05, 0.05),
        Vector3(0.20, 0.05, 0.15),
        "table_fixture",
    )
    controller, backend = make_controller(
        planner=EndpointWorkspaceGate(blocked_regions=(blocked,))
    )

    result = process(controller)

    assert not result.accepted
    assert result.reason == "endpoint_blocked"
    assert result.plan_reason is PlanReason.ENDPOINT_BLOCKED
    assert backend.executed_plans == []


@pytest.mark.parametrize(
    "contradictory_reason",
    [PlanReason.UNREACHABLE, PlanReason.ENDPOINT_BLOCKED],
)
def test_success_with_rejection_reason_and_plan_never_executes(
    contradictory_reason: PlanReason,
) -> None:
    class ContradictoryPlanner:
        def plan(self, prediction: Prediction, now_ns: int) -> PlanResult:
            return PlanResult(True, contradictory_reason, plan_for(prediction, now_ns))

    controller, backend = make_controller(planner=ContradictoryPlanner())

    result = process(controller)

    assert not result.accepted
    assert result.code is ResultCode.PLAN_PROTOCOL_INVALID
    assert result.reason == "invalid_plan_result:inconsistent_success"
    assert backend.executed_plans == []


def test_planned_without_a_plan_never_executes() -> None:
    class EmptyPlanner:
        def plan(self, prediction: Prediction, now_ns: int) -> PlanResult:
            del prediction, now_ns
            return PlanResult(True, PlanReason.PLANNED, None)

    controller, backend = make_controller(planner=EmptyPlanner())

    result = process(controller)

    assert result.code is ResultCode.PLAN_PROTOCOL_INVALID
    assert backend.executed_plans == []


@pytest.mark.parametrize("mode", ["planned_rejection", "rejection_with_plan"])
def test_inconsistent_rejection_never_executes(mode: str) -> None:
    class InconsistentRejectionPlanner:
        def plan(self, prediction: Prediction, now_ns: int) -> PlanResult:
            if mode == "planned_rejection":
                return PlanResult(False, PlanReason.PLANNED, None)
            return PlanResult(
                False,
                PlanReason.UNREACHABLE,
                plan_for(prediction, now_ns),
            )

    controller, backend = make_controller(planner=InconsistentRejectionPlanner())

    result = process(controller)

    assert result.code is ResultCode.PLAN_PROTOCOL_INVALID
    assert result.reason == "invalid_plan_result:inconsistent_rejection"
    assert backend.executed_plans == []


@pytest.mark.parametrize("tamper", ["empty_waypoints", "wrong_endpoint"])
def test_planned_with_illegal_plan_contract_never_executes(tamper: str) -> None:
    class IllegalPlanner:
        def plan(self, prediction: Prediction, now_ns: int) -> PlanResult:
            plan = plan_for(prediction, now_ns)
            if tamper == "empty_waypoints":
                object.__setattr__(plan, "waypoints_m", ())
            else:
                plan = replace(plan, waypoints_m=(Vector3(0.0, 0.0, 0.0),))
            return PlanResult(True, PlanReason.PLANNED, plan)

    controller, backend = make_controller(planner=IllegalPlanner())

    result = process(controller)

    assert result.code is ResultCode.PLAN_PROTOCOL_INVALID
    assert result.reason == "invalid_plan_result:plan_contract"
    assert backend.executed_plans == []


def test_frame_mismatch_fails_closed_without_execute_side_effect() -> None:
    controller, backend = make_controller()

    result = process(controller, make_target(frame_id="camera_link"))

    assert result.code is ResultCode.FRAME_MISMATCH
    assert result.reason == "frame_mismatch:camera_link!=base_link"
    assert controller.state is PipelineState.SAFE_STOP
    assert backend.executed_plans == []


@pytest.mark.parametrize(
    ("target", "code"),
    [
        (make_target(clock_domain="ros_system"), ResultCode.CLOCK_DOMAIN_MISMATCH),
        (make_target(clock_epoch=1), ResultCode.CLOCK_EPOCH_MISMATCH),
    ],
)
def test_clock_domain_or_epoch_mismatch_is_rejected(
    target: Target3D, code: ResultCode
) -> None:
    controller, backend = make_controller()

    result = process(controller, target)

    assert result.code is code
    assert backend.executed_plans == []


def test_low_confidence_is_rejected() -> None:
    controller, backend = make_controller()

    result = process(controller, make_target(confidence=0.49))

    assert result.code is ResultCode.LOW_CONFIDENCE
    assert backend.executed_plans == []


def test_future_target_is_rejected() -> None:
    controller, backend = make_controller()

    result = process(
        controller,
        make_target(timestamp_ns=RECEIVE_NS + 1),
        receive_ns=RECEIVE_NS,
        execute_ns=EXECUTE_NS,
    )

    assert result.reason == "future_target"
    assert result.code is ResultCode.SOURCE_FRESHNESS
    assert backend.executed_plans == []


@pytest.mark.parametrize("invalid_now", [-1, 1.5, True])
def test_invalid_receive_clock_fails_closed(invalid_now: object) -> None:
    controller, backend = make_controller()

    result = controller.process_target(  # type: ignore[arg-type]
        make_target(), invalid_now, EXECUTE_NS
    )

    assert result.code is ResultCode.CLOCK_FAULT
    assert controller.state is PipelineState.SAFE_STOP
    assert backend.executed_plans == []


def test_clock_rollback_latches_until_explicit_new_epoch() -> None:
    controller, backend = make_controller()
    assert process(controller).accepted

    rollback = process(
        controller,
        make_target(timestamp_ns=1_009_000_000),
        receive_ns=1_010_500_000,
        execute_ns=1_012_000_000,
    )
    assert rollback.code is ResultCode.CLOCK_FAULT
    assert rollback.reason.startswith("clock_rollback:receive:")
    assert controller.state is PipelineState.SAFE_STOP
    assert not controller.reset(1_012_000_000).success

    reset = controller.reset(100, new_clock_epoch=True)
    assert reset.success
    assert reset.clock_epoch == 1
    recovered = process(
        controller,
        make_target(timestamp_ns=100, clock_epoch=1),
        receive_ns=110,
        execute_ns=120,
    )
    assert recovered.accepted
    assert len(backend.executed_plans) == 2


def test_execute_boundary_clock_rollback_never_executes() -> None:
    controller, backend = make_controller()

    result = process(controller, execute_ns=RECEIVE_NS - 1)

    assert result.code is ResultCode.CLOCK_FAULT
    assert result.reason.startswith("clock_rollback:execute_boundary:")
    assert backend.executed_plans == []


@pytest.mark.parametrize("invalid_now", [-1, 1.5, True])
def test_invalid_execute_boundary_clock_never_executes(invalid_now: object) -> None:
    controller, backend = make_controller()

    result = controller.process_target(  # type: ignore[arg-type]
        make_target(), RECEIVE_NS, invalid_now
    )

    assert result.code is ResultCode.CLOCK_FAULT
    assert controller.state is PipelineState.SAFE_STOP
    assert backend.executed_plans == []


def test_target_that_expires_during_planning_never_reaches_backend() -> None:
    controller, backend = make_controller()

    result = process(controller, execute_ns=1_200_000_001)

    assert not result.accepted
    assert result.reason == "stale_target"
    assert controller.state is PipelineState.SAFE_STOP
    assert backend.executed_plans == []


def test_out_of_order_target_fails_closed() -> None:
    controller, backend = make_controller()
    assert process(controller).accepted

    result = process(
        controller,
        make_target(timestamp_ns=1_000_000_000, x_m=0.16),
        receive_ns=1_020_000_000,
        execute_ns=1_021_000_000,
    )

    assert not result.accepted
    assert result.reason.startswith("invalid_target_sequence:")
    assert controller.state is PipelineState.SAFE_STOP
    assert len(backend.executed_plans) == 1


def test_watchdog_boundary_and_repeated_ticks_are_fail_closed_and_idempotent() -> None:
    controller, backend = make_controller()
    assert process(controller).accepted

    before = controller.tick(RECEIVE_NS + 199_000_000)
    boundary = controller.tick(RECEIVE_NS + 200_000_000)
    expired = controller.tick(RECEIVE_NS + 200_000_001)
    repeated = controller.tick(RECEIVE_NS + 201_000_000)

    assert not before.triggered
    assert not boundary.triggered
    assert expired.triggered
    assert expired.receive_age_ns == 200_000_001
    assert controller.state is PipelineState.SAFE_STOP
    assert backend.stop_reasons == ["target_stream_timeout"]
    assert not repeated.triggered
    assert backend.stop_reasons == ["target_stream_timeout"]


def test_watchdog_clock_rollback_latches_and_stops() -> None:
    controller, backend = make_controller()
    assert process(controller).accepted

    result = controller.tick(EXECUTE_NS - 1)

    assert result.triggered
    assert result.reason.startswith("clock_rollback:watchdog:")
    assert controller.state is PipelineState.SAFE_STOP
    assert len(backend.stop_reasons) == 1


def test_watchdog_can_interrupt_planning() -> None:
    class WatchdogPlanner:
        controller: EdgeGraspController

        def plan(self, prediction: Prediction, now_ns: int) -> PlanResult:
            self.controller.tick(RECEIVE_NS + 200_000_001)
            return PlanResult(True, PlanReason.PLANNED, plan_for(prediction, now_ns))

    planner = WatchdogPlanner()
    controller, backend = make_controller(planner=planner)
    planner.controller = controller

    result = process(controller)

    assert result.code is ResultCode.INTERRUPTED
    assert result.reason == "planning_interrupted:SAFE_STOP"
    assert backend.executed_plans == []
    assert backend.stop_reasons == ["target_stream_timeout"]


def test_watchdog_can_interrupt_synchronous_execute_boundary() -> None:
    class WatchdogBackend(MockBackend):
        controller: EdgeGraspController

        def execute(self, plan: MotionPlan) -> ExecutionResult:
            del plan
            self.controller.tick(RECEIVE_NS + 200_000_001)
            return ExecutionResult(True, "completed_after_watchdog")

    backend = WatchdogBackend()
    controller, _ = make_controller(backend=backend)
    backend.controller = controller

    result = process(controller)

    assert result.code is ResultCode.INTERRUPTED
    assert result.reason == "execution_completed_after:SAFE_STOP"
    assert backend.executed_plans == []
    assert backend.stop_reasons == ["target_stream_timeout"]


def test_new_target_while_executing_is_rejected_as_busy() -> None:
    class ReentrantBackend(MockBackend):
        controller: EdgeGraspController
        nested_result = None

        def execute(self, plan: MotionPlan) -> ExecutionResult:
            self.nested_result = process(
                self.controller,
                make_target(timestamp_ns=1_020_000_000),
                receive_ns=1_021_000_000,
                execute_ns=1_022_000_000,
            )
            return super().execute(plan)

    backend = ReentrantBackend()
    controller, _ = make_controller(backend=backend)
    backend.controller = controller

    outer = process(controller)

    assert outer.accepted
    assert backend.nested_result is not None
    assert backend.nested_result.code is ResultCode.BUSY
    assert backend.nested_result.state is PipelineState.EXECUTING
    assert len(backend.executed_plans) == 1


def test_reset_while_executing_stops_before_entering_idle() -> None:
    class ResettingBackend(MockBackend):
        controller: EdgeGraspController
        reset_result = None

        def execute(self, plan: MotionPlan) -> ExecutionResult:
            del plan
            self.reset_result = self.controller.reset(EXECUTE_NS + 1)
            return ExecutionResult(True, "completed_after_reset")

    backend = ResettingBackend()
    controller, _ = make_controller(backend=backend)
    backend.controller = controller

    result = process(controller)

    assert backend.reset_result is not None and backend.reset_result.success
    assert backend.stop_reasons == ["reset_requested"]
    assert result.code is ResultCode.INTERRUPTED
    assert controller.state is PipelineState.IDLE


def test_reset_stop_failure_does_not_enter_idle() -> None:
    class ResettingBackend(MockBackend):
        controller: EdgeGraspController
        reset_result = None

        def execute(self, plan: MotionPlan) -> ExecutionResult:
            del plan
            self.reset_result = self.controller.reset(EXECUTE_NS + 1)
            return ExecutionResult(True, "completed_after_failed_reset")

    backend = ResettingBackend(fail_stop=True)
    controller, _ = make_controller(backend=backend)
    backend.controller = controller

    result = process(controller)

    assert backend.reset_result is not None and not backend.reset_result.success
    assert controller.state is PipelineState.ERROR
    assert result.code is ResultCode.INTERRUPTED


def test_planner_exception_and_stop_exception_preserve_both_failures() -> None:
    class ExplodingPlanner:
        def plan(self, prediction: Prediction, now_ns: int) -> PlanResult:
            del prediction, now_ns
            raise RuntimeError("planner offline")

    backend = MockBackend(raise_on_stop=True)
    controller, _ = make_controller(backend=backend, planner=ExplodingPlanner())

    result = process(controller)

    assert result.code is ResultCode.STOP_FAILED
    assert "planner_exception:RuntimeError" in result.reason
    assert "stop_exception:RuntimeError:mock_stop_exception" in result.reason
    assert controller.state is PipelineState.ERROR


def test_watchdog_stop_exception_enters_error() -> None:
    backend = MockBackend(raise_on_stop=True)
    controller, _ = make_controller(backend=backend)
    assert process(controller).accepted

    result = controller.tick(RECEIVE_NS + 200_000_001)

    assert result.triggered
    assert controller.state is PipelineState.ERROR
    assert len(backend.stop_reasons) == 1


def test_backend_execution_failure_enters_error() -> None:
    controller, backend = make_controller(backend=MockBackend(fail_execution=True))

    result = process(controller)

    assert not result.accepted
    assert result.code is ResultCode.EXECUTION_FAILED
    assert controller.state is PipelineState.ERROR
    assert backend.stop_reasons == ["mock_execution_failure"]


def test_backend_exception_enters_error_and_is_latched() -> None:
    class ExplodingBackend(MockBackend):
        def execute(self, plan: MotionPlan) -> ExecutionResult:
            del plan
            raise RuntimeError("transport offline")

    backend = ExplodingBackend()
    controller, _ = make_controller(backend=backend)

    result = process(controller)

    assert result.code is ResultCode.BACKEND_EXCEPTION
    assert controller.state is PipelineState.ERROR
    assert backend.stop_reasons == ["backend_exception:RuntimeError"]
    assert controller.stop("operator_stop", EXECUTE_NS + 1).code is ResultCode.LATCHED
    assert backend.stop_reasons == ["backend_exception:RuntimeError"]


def test_required_execute_boundary_and_public_planner_api_are_explicit() -> None:
    parameter = inspect.signature(EdgeGraspController.process_target).parameters[
        "execute_now_ns"
    ]
    assert parameter.default is inspect.Parameter.empty

    import edgegrasp
    import edgegrasp.planner as planner_module

    assert edgegrasp.EndpointWorkspaceGate is EndpointWorkspaceGate
    assert not hasattr(planner_module, "AxisAlignedWorkspacePlanner")


def test_backend_factory_is_fail_closed_for_gazebo_and_real() -> None:
    assert isinstance(create_backend("gazebo"), DisabledBackend)
    assert isinstance(create_backend(BackendKind.REAL), DisabledBackend)
    assert isinstance(build_controller("mock").backend, MockBackend)
