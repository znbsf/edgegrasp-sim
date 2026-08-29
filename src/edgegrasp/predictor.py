"""Explainable constant-velocity target prediction."""

from __future__ import annotations

from collections import deque

from .models import NANOSECONDS_PER_SECOND, Prediction, Target3D, Vector3


class ConstantVelocityPredictor:
    """Estimate velocity from the latest two observations.

    A single observation intentionally produces a zero-velocity estimate. A
    target-id or frame change clears history so unrelated observations are
    never differenced.
    """

    def __init__(self) -> None:
        self._history: deque[Target3D] = deque(maxlen=2)

    @property
    def sample_count(self) -> int:
        return len(self._history)

    def reset(self) -> None:
        self._history.clear()

    def update(self, target: Target3D) -> None:
        if self._history:
            latest = self._history[-1]
            if (
                target.target_id != latest.target_id
                or target.frame_id != latest.frame_id
                or target.clock_domain != latest.clock_domain
                or target.clock_epoch != latest.clock_epoch
            ):
                self.reset()
            elif target.timestamp_ns <= latest.timestamp_ns:
                raise ValueError("target timestamps must be strictly increasing")
        self._history.append(target)

    def velocity_mps(self) -> Vector3:
        if len(self._history) < 2:
            return Vector3(0.0, 0.0, 0.0)
        previous, latest = self._history
        delta_seconds = (latest.timestamp_ns - previous.timestamp_ns) / NANOSECONDS_PER_SECOND
        return (latest.position_m - previous.position_m).scaled(1.0 / delta_seconds)

    def predict_at(self, timestamp_ns: int) -> Prediction:
        if not self._history:
            raise RuntimeError("cannot predict without an observation")
        if timestamp_ns < 0:
            raise ValueError("prediction timestamp must be non-negative")

        latest = self._history[-1]
        delta_seconds = (timestamp_ns - latest.timestamp_ns) / NANOSECONDS_PER_SECOND
        velocity = self.velocity_mps()
        position = latest.position_m + velocity.scaled(delta_seconds)
        return Prediction(
            target_id=latest.target_id,
            position_m=position,
            velocity_mps=velocity,
            source_timestamp_ns=latest.timestamp_ns,
            prediction_timestamp_ns=timestamp_ns,
            frame_id=latest.frame_id,
            clock_domain=latest.clock_domain,
            clock_epoch=latest.clock_epoch,
        )
