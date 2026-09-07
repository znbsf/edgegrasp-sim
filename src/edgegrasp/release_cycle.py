"""Independent placement scoring for the fixed control scene, in metres.

These observations do not supply a planning pose or override motion permission.
"""

from dataclasses import dataclass
from collections import deque
from math import isfinite, sqrt


@dataclass(frozen=True)
class PlacementSample:
    stamp_ns: int
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    table_contact: bool
    pad_contact: bool
    gripper_position: float
    moving_pad_contact: bool = False


class PlacementSynchronizer:
    """Pair a pose with causal contact/joint samples without accepting future data.

    A callback may precede its /clock callback. Keep that sample pending instead
    of consuming it. No clock value or source timestamp is rewritten here.
    """

    def __init__(self):
        self.poses = deque(maxlen=128)
        self.contacts = deque(maxlen=512)
        self.joints = deque(maxlen=128)
        self.last_source = {}
        self.last_now = None
        self.fault = None

    def push(self, kind, stamp_ns, payload):
        if self.fault:
            raise ValueError(self.fault)
        previous = self.last_source.get(kind)
        if stamp_ns < 0 or (previous is not None and stamp_ns < previous):
            self.fault = f'{kind}:source_clock_rollback'
            raise ValueError(self.fault)
        if previous == stamp_ns:
            return
        self.last_source[kind] = stamp_ns
        getattr(self, kind).append((stamp_ns, payload))

    def ready(self, now_ns):
        if self.fault:
            raise ValueError(self.fault)
        if self.last_now is not None and now_ns < self.last_now:
            self.fault = 'observation_clock_rollback'
            raise ValueError(self.fault)
        self.last_now = now_ns
        rows = []
        while self.poses:
            source, pose = self.poses[0]
            if source > now_ns:
                break
            matched = [next((row for row in reversed(queue) if row[0] <= source), None)
                       for queue in (self.contacts, self.joints)]
            reason = None
            if now_ns-source > 200_000_000:
                reason = 'pose_stale'
            elif any(row is None or source-row[0] > 50_000_000 for row in matched):
                if now_ns-source <= 50_000_000:
                    break  # A matching callback can still arrive within the existing bound.
                reason = 'matching_contact_or_joint_missing'
            elif any(now_ns-row[0] > 200_000_000 for row in matched):
                reason = 'matched_input_stale'
            self.poses.popleft()
            rows.append((source, pose, matched, reason))
        return rows


def cube_bottom(position, quaternion):
    x, y, z, w = quaternion
    if (not all(isfinite(v) for v in (*position, *quaternion))
            or abs(sum(v*v for v in quaternion) - 1.0) > 1e-4):
        raise ValueError("invalid cube pose")
    vertical_extent = 0.025 * (
        abs(2*(x*z-w*y)) + abs(2*(y*z+w*x)) + abs(1-2*(x*x+y*y)))
    return position[2] - vertical_extent


class StablePlacement:
    """Require continuous fresh, monotonic evidence; reset on any invalid sample."""

    def __init__(self, *, released=False, opened=False):
        self.released = released
        self.opened = opened or released
        self.first = None
        self.last_ns = None

    def observe(self, sample: PlacementSample, now_ns: int) -> bool:
        previous_ns = self.last_ns
        self.last_ns = sample.stamp_ns
        p = sample.position
        valid = (
            0 <= now_ns - sample.stamp_ns <= 200_000_000
            and (previous_ns is None or 0 < sample.stamp_ns - previous_ns <= 100_000_000)
            and sample.table_contact
            and abs(p[0] - 0.24108428841333183) <= 0.015
            and abs(p[1] - 0.12958666029737353) <= 0.015
            and abs(cube_bottom(p, sample.orientation) - 0.18) <= 0.001
            and isfinite(sample.gripper_position)
            and (not self.opened or (not sample.moving_pad_contact
                                    and abs(sample.gripper_position - 1.5) <= 0.08))
            and (not self.released or not sample.pad_contact)
        )
        if not valid:
            self.first = None
            return False
        if self.first is None:
            self.first = sample
            return False
        distance = sqrt(sum((a-b)**2 for a, b in zip(p, self.first.position)))
        orientation_dot = abs(sum(a*b for a, b in zip(sample.orientation, self.first.orientation)))
        if distance > 0.001 or orientation_dot < 0.99995:
            self.first = sample
            return False
        return sample.stamp_ns - self.first.stamp_ns >= 500_000_000
