"""Default wiring for demos, tests, and the ROS wrapper."""

from __future__ import annotations

from .backend import BackendKind, MotionBackend, create_backend
from .config import (
    DEFAULT_CLOCK_DOMAIN,
    DEFAULT_MIN_CONFIDENCE,
    DEFAULT_STALE_AFTER_MS,
    DEFAULT_TARGET_FRAME,
    DEFAULT_WATCHDOG_TIMEOUT_MS,
)
from .controller import EdgeGraspController
from .planner import EndpointWorkspaceGate
from .predictor import ConstantVelocityPredictor
from .safety import StaleTargetGate


def build_controller(
    backend: str | BackendKind | MotionBackend = BackendKind.MOCK,
    stale_after_ms: float = DEFAULT_STALE_AFTER_MS,
    watchdog_timeout_ms: float = DEFAULT_WATCHDOG_TIMEOUT_MS,
    prediction_horizon_ms: float = 100.0,
    target_frame: str = DEFAULT_TARGET_FRAME,
    clock_domain: str = DEFAULT_CLOCK_DOMAIN,
    min_confidence: float = DEFAULT_MIN_CONFIDENCE,
) -> EdgeGraspController:
    selected_backend = create_backend(backend) if isinstance(backend, (str, BackendKind)) else backend
    return EdgeGraspController(
        predictor=ConstantVelocityPredictor(),
        planner=EndpointWorkspaceGate(target_frame=target_frame),
        safety_gate=StaleTargetGate(max_age_ms=stale_after_ms),
        backend=selected_backend,
        prediction_horizon_ms=prediction_horizon_ms,
        watchdog_timeout_ms=watchdog_timeout_ms,
        target_frame=target_frame,
        clock_domain=clock_domain,
        min_confidence=min_confidence,
    )
