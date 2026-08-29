from edgegrasp.models import Prediction, Vector3
from edgegrasp.planner import AxisAlignedBox, EndpointWorkspaceGate, PlanReason


def prediction(position: Vector3, frame_id: str = "base_link") -> Prediction:
    return Prediction(
        target_id="cube",
        position_m=position,
        velocity_mps=Vector3(0.0, 0.0, 0.0),
        source_timestamp_ns=1_000_000_000,
        prediction_timestamp_ns=1_010_000_000,
        frame_id=frame_id,
        clock_domain="sim",
        clock_epoch=0,
    )


def test_endpoint_gate_rejects_frame_mismatch() -> None:
    result = EndpointWorkspaceGate().plan(
        prediction(Vector3(0.15, 0.0, 0.1), frame_id="camera_link"),
        1_010_000_000,
    )

    assert not result.success
    assert result.reason is PlanReason.FRAME_MISMATCH


def test_endpoint_gate_does_not_claim_swept_path_collision_checking() -> None:
    blocked = AxisAlignedBox(
        Vector3(0.10, -0.05, 0.05),
        Vector3(0.20, 0.05, 0.15),
        "implied_path_obstacle",
    )
    gate = EndpointWorkspaceGate(blocked_regions=(blocked,))

    result = gate.plan(prediction(Vector3(0.25, 0.0, 0.1)), 1_010_000_000)

    assert result.success
    assert result.reason is PlanReason.PLANNED
    assert result.plan is not None
    assert result.plan.waypoints_m == (Vector3(0.25, 0.0, 0.1),)
    assert "not IK, swept-path checking, or collision-safe planning" in (
        EndpointWorkspaceGate.__doc__ or ""
    )
