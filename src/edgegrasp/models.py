"""Small immutable data models shared across the pipeline."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite, sqrt

from .config import DEFAULT_CLOCK_DOMAIN, DEFAULT_TARGET_FRAME

NANOSECONDS_PER_SECOND = 1_000_000_000


@dataclass(frozen=True, slots=True)
class Vector3:
    """A position or velocity vector expressed in SI units."""

    x: float
    y: float
    z: float

    def __post_init__(self) -> None:
        if not all(isfinite(value) for value in (self.x, self.y, self.z)):
            raise ValueError("Vector3 values must be finite")

    def __add__(self, other: "Vector3") -> "Vector3":
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Vector3") -> "Vector3":
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def scaled(self, factor: float) -> "Vector3":
        if not isfinite(factor):
            raise ValueError("scale factor must be finite")
        return Vector3(self.x * factor, self.y * factor, self.z * factor)

    def distance_to(self, other: "Vector3") -> float:
        delta = self - other
        return sqrt(delta.x**2 + delta.y**2 + delta.z**2)

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)


@dataclass(frozen=True, slots=True)
class Target3D:
    """A timestamped 3D target observation.

    ``timestamp_ns`` is an integer timestamp in the clock domain selected by
    the caller. The core never mixes wall time and simulated time implicitly.
    """

    target_id: str
    position_m: Vector3
    timestamp_ns: int
    frame_id: str = DEFAULT_TARGET_FRAME
    confidence: float = 1.0
    clock_domain: str = DEFAULT_CLOCK_DOMAIN
    clock_epoch: int = 0

    def __post_init__(self) -> None:
        if not self.target_id.strip():
            raise ValueError("target_id must not be empty")
        if not self.frame_id.strip():
            raise ValueError("frame_id must not be empty")
        if not self.clock_domain.strip():
            raise ValueError("clock_domain must not be empty")
        if isinstance(self.timestamp_ns, bool) or not isinstance(self.timestamp_ns, int):
            raise TypeError("timestamp_ns must be an integer")
        if self.timestamp_ns < 0:
            raise ValueError("timestamp_ns must be non-negative")
        if isinstance(self.clock_epoch, bool) or not isinstance(self.clock_epoch, int):
            raise TypeError("clock_epoch must be an integer")
        if self.clock_epoch < 0:
            raise ValueError("clock_epoch must be non-negative")
        if not isfinite(self.confidence) or not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be in [0.0, 1.0]")


@dataclass(frozen=True, slots=True)
class Prediction:
    target_id: str
    position_m: Vector3
    velocity_mps: Vector3
    source_timestamp_ns: int
    prediction_timestamp_ns: int
    frame_id: str
    clock_domain: str
    clock_epoch: int


@dataclass(frozen=True, slots=True)
class MotionPlan:
    target_id: str
    waypoints_m: tuple[Vector3, ...]
    source_timestamp_ns: int
    requested_at_ns: int
    frame_id: str
    clock_domain: str
    clock_epoch: int

    def __post_init__(self) -> None:
        if not self.target_id.strip():
            raise ValueError("target_id must not be empty")
        if not self.waypoints_m:
            raise ValueError("a motion plan needs at least one waypoint")
        if not all(isinstance(waypoint, Vector3) for waypoint in self.waypoints_m):
            raise TypeError("all motion-plan waypoints must be Vector3")
        for name, value in (
            ("source_timestamp_ns", self.source_timestamp_ns),
            ("requested_at_ns", self.requested_at_ns),
            ("clock_epoch", self.clock_epoch),
        ):
            if isinstance(value, bool) or not isinstance(value, int):
                raise TypeError(f"{name} must be an integer")
            if value < 0:
                raise ValueError(f"{name} must be non-negative")
        if not self.frame_id.strip():
            raise ValueError("frame_id must not be empty")
        if not self.clock_domain.strip():
            raise ValueError("clock_domain must not be empty")
