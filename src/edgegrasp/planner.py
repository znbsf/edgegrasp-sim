"""Deterministic reachability and endpoint-only blocking for core replay."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Protocol

from .config import DEFAULT_TARGET_FRAME
from .models import MotionPlan, Prediction, Vector3


@dataclass(frozen=True, slots=True)
class AxisAlignedBox:
    minimum_m: Vector3
    maximum_m: Vector3
    name: str = "box"

    def __post_init__(self) -> None:
        if any(
            lower > upper
            for lower, upper in zip(self.minimum_m.as_tuple(), self.maximum_m.as_tuple())
        ):
            raise ValueError("box minimum must not exceed maximum")

    def contains(self, point: Vector3) -> bool:
        return all(
            lower <= value <= upper
            for lower, value, upper in zip(
                self.minimum_m.as_tuple(), point.as_tuple(), self.maximum_m.as_tuple()
            )
        )


class PlanReason(str, Enum):
    PLANNED = "planned"
    UNREACHABLE = "unreachable"
    ENDPOINT_BLOCKED = "endpoint_blocked"
    FRAME_MISMATCH = "frame_mismatch"


@dataclass(frozen=True, slots=True)
class PlanResult:
    success: bool
    reason: PlanReason
    plan: MotionPlan | None = None


class Planner(Protocol):
    def plan(self, prediction: Prediction, now_ns: int) -> PlanResult: ...


class EndpointWorkspaceGate:
    """Endpoint-only proxy gate used to exercise upper-layer contracts.

    This is not IK, swept-path checking, or collision-safe planning. It checks
    only a target endpoint against a Cartesian box and optional blocked endpoint
    regions, then emits a single waypoint for the mock backend.
    """

    def __init__(
        self,
        workspace: AxisAlignedBox | None = None,
        blocked_regions: tuple[AxisAlignedBox, ...] = (),
        target_frame: str = DEFAULT_TARGET_FRAME,
    ) -> None:
        if not target_frame.strip():
            raise ValueError("target_frame must not be empty")
        self.workspace = workspace or AxisAlignedBox(
            minimum_m=Vector3(-0.35, -0.35, 0.0),
            maximum_m=Vector3(0.35, 0.35, 0.50),
            name="conservative_so101_workspace",
        )
        self.blocked_regions = blocked_regions
        self.target_frame = target_frame

    def plan(self, prediction: Prediction, now_ns: int) -> PlanResult:
        if prediction.frame_id != self.target_frame:
            return PlanResult(False, PlanReason.FRAME_MISMATCH)
        if not self.workspace.contains(prediction.position_m):
            return PlanResult(False, PlanReason.UNREACHABLE)
        if any(region.contains(prediction.position_m) for region in self.blocked_regions):
            return PlanResult(False, PlanReason.ENDPOINT_BLOCKED)
        return PlanResult(
            True,
            PlanReason.PLANNED,
            MotionPlan(
                target_id=prediction.target_id,
                waypoints_m=(prediction.position_m,),
                source_timestamp_ns=prediction.source_timestamp_ns,
                requested_at_ns=now_ns,
                frame_id=prediction.frame_id,
                clock_domain=prediction.clock_domain,
                clock_epoch=prediction.clock_epoch,
            ),
        )
