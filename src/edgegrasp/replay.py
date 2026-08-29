"""Repeatable core replay and stable, behavior-complete output hashing."""

from __future__ import annotations

from dataclasses import dataclass
from hashlib import sha256
import json
from typing import Callable

from .controller import EdgeGraspController
from .models import MotionPlan, Prediction, Target3D, Vector3
from .scenarios import LinearScenario


@dataclass(frozen=True, slots=True)
class ReplayReport:
    scenario: str
    cycles: int
    accepted: int
    rejected: int
    final_state: str
    digest: str


def _number(value: float) -> str:
    return format(value, ".17g")


def _vector(value: Vector3) -> list[str]:
    return [_number(component) for component in value.as_tuple()]


def _target(value: Target3D) -> dict[str, object]:
    return {
        "target_id": value.target_id,
        "position_m": _vector(value.position_m),
        "timestamp_ns": value.timestamp_ns,
        "frame_id": value.frame_id,
        "confidence": _number(value.confidence),
        "clock_domain": value.clock_domain,
        "clock_epoch": value.clock_epoch,
    }


def _prediction(value: Prediction | None) -> dict[str, object] | None:
    if value is None:
        return None
    return {
        "target_id": value.target_id,
        "position_m": _vector(value.position_m),
        "velocity_mps": _vector(value.velocity_mps),
        "source_timestamp_ns": value.source_timestamp_ns,
        "prediction_timestamp_ns": value.prediction_timestamp_ns,
        "frame_id": value.frame_id,
        "clock_domain": value.clock_domain,
        "clock_epoch": value.clock_epoch,
    }


def _plan(value: MotionPlan | None) -> dict[str, object] | None:
    if value is None:
        return None
    return {
        "target_id": value.target_id,
        "waypoints_m": [_vector(waypoint) for waypoint in value.waypoints_m],
        "source_timestamp_ns": value.source_timestamp_ns,
        "requested_at_ns": value.requested_at_ns,
        "frame_id": value.frame_id,
        "clock_domain": value.clock_domain,
        "clock_epoch": value.clock_epoch,
    }


def replay_once(
    scenario: LinearScenario,
    controller_factory: Callable[[], EdgeGraspController],
    input_delay_ms: int = 10,
    planning_delay_ms: int = 1,
) -> ReplayReport:
    controller = controller_factory()
    input_delay_ns = input_delay_ms * 1_000_000
    planning_delay_ns = planning_delay_ms * 1_000_000
    canonical_cycles: list[dict[str, object]] = []
    accepted = 0

    for target in scenario.samples():
        receive_now_ns = target.timestamp_ns + input_delay_ns
        execute_now_ns = receive_now_ns + planning_delay_ns
        transition_start = len(controller.state_machine.history)
        executed_start = len(getattr(controller.backend, "executed_plans", ()))
        stops_start = len(getattr(controller.backend, "stop_reasons", ()))

        result = controller.process_target(target, receive_now_ns, execute_now_ns)
        accepted += int(result.accepted)
        executed_plans = getattr(controller.backend, "executed_plans", ())
        stop_reasons = getattr(controller.backend, "stop_reasons", ())
        canonical_cycles.append(
            {
                "input": _target(target),
                "receive_now_ns": receive_now_ns,
                "execute_now_ns": execute_now_ns,
                "result": {
                    "accepted": result.accepted,
                    "code": result.code.value,
                    "reason": result.reason,
                    "state": result.state.value,
                    "prediction": _prediction(result.prediction),
                    "plan_reason": None
                    if result.plan_reason is None
                    else result.plan_reason.value,
                    "plan": _plan(result.plan),
                },
                "state_transitions": [
                    {
                        "previous": transition.previous.value,
                        "current": transition.current.value,
                        "reason": transition.reason,
                        "timestamp_ns": transition.timestamp_ns,
                    }
                    for transition in controller.state_machine.history[transition_start:]
                ],
                "backend_effects": {
                    "kind": controller.backend.kind.value,
                    "executed_plans": [
                        _plan(plan) for plan in executed_plans[executed_start:]
                    ],
                    "stop_reasons": list(stop_reasons[stops_start:]),
                },
                "controller_clock_epoch": controller.clock_epoch,
            }
        )

    payload = json.dumps(canonical_cycles, sort_keys=True, separators=(",", ":"))
    return ReplayReport(
        scenario=scenario.name,
        cycles=len(canonical_cycles),
        accepted=accepted,
        rejected=len(canonical_cycles) - accepted,
        final_state=controller.state.value,
        digest=sha256(payload.encode("utf-8")).hexdigest(),
    )


def verify_repeated_replay(
    scenario: LinearScenario,
    controller_factory: Callable[[], EdgeGraspController],
    runs: int = 100,
) -> tuple[ReplayReport, ...]:
    if runs <= 0:
        raise ValueError("runs must be positive")
    reports = tuple(replay_once(scenario, controller_factory) for _ in range(runs))
    if len({report.digest for report in reports}) != 1:
        raise AssertionError("core replay output changed between identical runs")
    return reports
