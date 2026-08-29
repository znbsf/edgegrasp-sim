"""Deterministic trajectory fixtures used by tests and future rosbag replay."""

from __future__ import annotations

from dataclasses import dataclass

from .config import DEFAULT_CLOCK_DOMAIN, DEFAULT_TARGET_FRAME
from .models import NANOSECONDS_PER_SECOND, Target3D, Vector3


@dataclass(frozen=True, slots=True)
class LinearScenario:
    name: str
    start_m: Vector3
    velocity_mps: Vector3
    sample_period_ms: int = 50
    sample_count: int = 20
    start_timestamp_ns: int = NANOSECONDS_PER_SECOND
    target_id: str = "cube-1"
    frame_id: str = DEFAULT_TARGET_FRAME
    clock_domain: str = DEFAULT_CLOCK_DOMAIN
    clock_epoch: int = 0

    def __post_init__(self) -> None:
        if self.sample_period_ms <= 0:
            raise ValueError("sample_period_ms must be positive")
        if self.sample_count <= 0:
            raise ValueError("sample_count must be positive")

    @property
    def period_ns(self) -> int:
        return int(self.sample_period_ms * NANOSECONDS_PER_SECOND / 1000)

    def position_at(self, timestamp_ns: int) -> Vector3:
        elapsed_s = (timestamp_ns - self.start_timestamp_ns) / NANOSECONDS_PER_SECOND
        return self.start_m + self.velocity_mps.scaled(elapsed_s)

    def samples(self) -> tuple[Target3D, ...]:
        return tuple(
            Target3D(
                target_id=self.target_id,
                position_m=self.position_at(self.start_timestamp_ns + index * self.period_ns),
                timestamp_ns=self.start_timestamp_ns + index * self.period_ns,
                frame_id=self.frame_id,
                clock_domain=self.clock_domain,
                clock_epoch=self.clock_epoch,
            )
            for index in range(self.sample_count)
        )


def benchmark_scenarios() -> tuple[LinearScenario, ...]:
    return tuple(
        LinearScenario(
            name=f"linear_{speed_mm_s}mm_s",
            start_m=Vector3(0.15, -0.10, 0.08),
            velocity_mps=Vector3(speed_mm_s / 1000.0, 0.0, 0.0),
        )
        for speed_mm_s in (0, 20, 40)
    )
