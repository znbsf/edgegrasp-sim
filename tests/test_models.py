import pytest

from edgegrasp.models import Target3D, Vector3


def test_target_requires_timestamp_and_frame_contract() -> None:
    target = Target3D("cube", Vector3(0.1, 0.2, 0.3), 123, "camera_link", 0.9)

    assert target.timestamp_ns == 123
    assert target.frame_id == "camera_link"


@pytest.mark.parametrize(
    ("timestamp_ns", "error_type"),
    [(True, TypeError), (1.5, TypeError), (-1, ValueError)],
)
def test_invalid_target_timestamp_is_rejected(
    timestamp_ns: object, error_type: type[Exception]
) -> None:
    with pytest.raises(error_type, match="timestamp_ns"):
        Target3D(  # type: ignore[arg-type]
            "cube", Vector3(0.1, 0.2, 0.3), timestamp_ns
        )


@pytest.mark.parametrize(
    ("clock_epoch", "error_type"),
    [(True, TypeError), (1.5, TypeError), (-1, ValueError)],
)
def test_invalid_clock_epoch_is_rejected(
    clock_epoch: object, error_type: type[Exception]
) -> None:
    with pytest.raises(error_type, match="clock_epoch"):
        Target3D(  # type: ignore[arg-type]
            "cube",
            Vector3(0.1, 0.2, 0.3),
            123,
            clock_epoch=clock_epoch,
        )


@pytest.mark.parametrize("confidence", [-0.01, 1.01])
def test_invalid_confidence_is_rejected(confidence: float) -> None:
    with pytest.raises(ValueError, match="confidence"):
        Target3D("cube", Vector3(0.1, 0.2, 0.3), 123, confidence=confidence)
