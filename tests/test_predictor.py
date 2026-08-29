import pytest

from edgegrasp.models import Target3D, Vector3
from edgegrasp.predictor import ConstantVelocityPredictor


def target(x_m: float, timestamp_ns: int) -> Target3D:
    return Target3D("cube", Vector3(x_m, 0.0, 0.1), timestamp_ns)


def test_static_target_remains_static() -> None:
    predictor = ConstantVelocityPredictor()
    predictor.update(target(0.2, 1_000_000_000))
    predictor.update(target(0.2, 1_100_000_000))

    prediction = predictor.predict_at(1_300_000_000)

    assert prediction.position_m == Vector3(0.2, 0.0, 0.1)
    assert prediction.velocity_mps == Vector3(0.0, 0.0, 0.0)


def test_constant_velocity_prediction_is_exact_for_linear_motion() -> None:
    predictor = ConstantVelocityPredictor()
    predictor.update(target(0.200, 1_000_000_000))
    predictor.update(target(0.204, 1_100_000_000))  # 40 mm/s

    prediction = predictor.predict_at(1_300_000_000)

    assert prediction.velocity_mps.x == pytest.approx(0.040)
    assert prediction.position_m.x == pytest.approx(0.212)


def test_timestamp_must_increase_for_same_target() -> None:
    predictor = ConstantVelocityPredictor()
    predictor.update(target(0.2, 1_000_000_000))

    with pytest.raises(ValueError, match="strictly increasing"):
        predictor.update(target(0.2, 1_000_000_000))


def test_new_target_resets_velocity_history() -> None:
    predictor = ConstantVelocityPredictor()
    predictor.update(target(0.2, 1_000_000_000))
    predictor.update(target(0.3, 1_100_000_000))
    predictor.update(Target3D("other", Vector3(0.1, 0.0, 0.1), 1_200_000_000))

    assert predictor.velocity_mps() == Vector3(0.0, 0.0, 0.0)
