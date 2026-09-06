"""Observed-target consistency during a commanded lift; not grasp success."""

from dataclasses import dataclass
import math

from edgegrasp.grasp_geometry import (
    OrientedBoxEnvelope, oriented_box_overlap_margins, transform_point_from_frame,
)


@dataclass(frozen=True)
class LiftObservationBinding:
    task_id: str
    target_id: str
    clock_domain: str
    clock_epoch: int
    bound_source_ns: int
    point_in_gripper_m: tuple[float, float, float]

    @classmethod
    def bind(cls, *, task_id, target_id, clock_domain, clock_epoch, source_ns,
             observed_center_m, gripper_position_m, gripper_orientation_xyzw):
        if (not task_id or not target_id or clock_domain != "ros_sim"
                or clock_epoch < 0 or source_ns <= 0):
            raise ValueError("invalid lift observation identity")
        x, y, z, w = gripper_orientation_xyzw
        delta = tuple(a - b for a, b in zip(observed_center_m, gripper_position_m, strict=True))
        local = transform_point_from_frame((0., 0., 0.), (-x, -y, -z, w), delta)
        return cls(task_id, target_id, clock_domain, clock_epoch, source_ns, local)

    def residual_m(self, *, task_id, target_id, clock_domain, clock_epoch,
                   source_ns, observed_center_m, gripper_position_m,
                   gripper_orientation_xyzw):
        if (task_id != self.task_id or target_id != self.target_id
                or clock_domain != self.clock_domain or clock_epoch != self.clock_epoch
                or source_ns < self.bound_source_ns):
            raise ValueError("lift observation binding mismatch")
        expected = transform_point_from_frame(
            gripper_position_m, gripper_orientation_xyzw, self.point_in_gripper_m)
        if len(observed_center_m) != 3 or not all(math.isfinite(v) for v in observed_center_m):
            raise ValueError("invalid observed center")
        return math.dist(expected, observed_center_m)


@dataclass(frozen=True)
class PlannedLiftRegion:
    """A bounded workspace check; axial lag or no lift is NOT grasp success.

    Only the scoped vertical lift up to 40 mm is supported. The caller keeps
    the existing 5 mm distance limit, freshness and independent physics checks.
    """
    task_id: str
    target_id: str
    clock_domain: str
    clock_epoch: int
    bound_source_ns: int
    start_m: tuple[float, float, float]
    height_m: float

    @classmethod
    def bind(cls, *, task_id, target_id, clock_domain, clock_epoch, source_ns,
             initial_center_m, descend_position_m, lift_position_m):
        if (not task_id or not target_id or clock_domain != "ros_sim"
                or clock_epoch < 0 or source_ns <= 0):
            raise ValueError("invalid lift region identity")
        for point in (initial_center_m, descend_position_m, lift_position_m):
            if len(point) != 3 or not all(math.isfinite(v) for v in point):
                raise ValueError("invalid lift region geometry")
        delta = tuple(a - b for a, b in zip(lift_position_m, descend_position_m, strict=True))
        if abs(delta[0]) > 1e-9 or abs(delta[1]) > 1e-9 or not 0 < delta[2] <= 0.040000001:
            raise ValueError("lift region requires a vertical lift of at most 40 mm")
        return cls(task_id, target_id, clock_domain, clock_epoch, source_ns,
                   tuple(initial_center_m), delta[2])

    def residual_m(self, *, task_id, target_id, clock_domain, clock_epoch,
                   source_ns, observed_center_m):
        if (task_id != self.task_id or target_id != self.target_id
                or clock_domain != self.clock_domain or clock_epoch != self.clock_epoch
                or source_ns < self.bound_source_ns):
            raise ValueError("lift region binding mismatch")
        if len(observed_center_m) != 3 or not all(math.isfinite(v) for v in observed_center_m):
            raise ValueError("invalid observed center")
        nearest_z = min(max(observed_center_m[2], self.start_m[2]),
                        self.start_m[2] + self.height_m)
        return math.dist(observed_center_m, (*self.start_m[:2], nearest_z))


@dataclass(frozen=True)
class MeasuredCubeObservation:
    target_id: str
    frame_id: str
    source_ns: int
    clock_domain: str
    clock_epoch: int
    center_m: tuple[float, float, float]
    orientation_rows: tuple[tuple[float, float, float], ...]

    @classmethod
    def parse(cls, value):
        if not isinstance(value, dict):
            raise ValueError("invalid measured cube payload")
        if (type(value.get("schema_version")) is not int or value.get("schema_version") != 1 or value.get("clock_domain") != "ros_sim"
                or type(value.get("source_ns")) is not int or not 0 < value["source_ns"] < 2**31 * 10**9
                or type(value.get("clock_epoch")) is not int or not 0 <= value["clock_epoch"] < 2**64
                or not isinstance(value.get("target_id"), str) or not value["target_id"]
                or not isinstance(value.get("frame_id"), str) or not value["frame_id"]):
            raise ValueError("invalid measured cube identity")
        center = tuple(float(v) for v in value["center_m"])
        rows = tuple(tuple(float(v) for v in row) for row in value["orientation_rows"])
        if (len(center) != 3 or len(rows) != 3 or any(len(row) != 3 for row in rows)
                or not all(math.isfinite(v) for v in (*center, *(v for row in rows for v in row)))):
            raise ValueError("invalid measured cube geometry")
        for i in range(3):
            for j in range(3):
                if abs(sum(rows[i][k] * rows[j][k] for k in range(3)) - (i == j)) > 1e-6:
                    raise ValueError("invalid measured cube rotation")
        a, b, c = rows
        determinant = (a[0]*(b[1]*c[2]-b[2]*c[1]) - a[1]*(b[0]*c[2]-b[2]*c[0])
                       + a[2]*(b[0]*c[1]-b[1]*c[0]))
        if abs(determinant - 1) > 1e-6:
            raise ValueError("invalid measured cube rotation")
        return cls(value["target_id"], value["frame_id"], value["source_ns"],
                   value["clock_domain"], value["clock_epoch"], center, rows)

    @property
    def key(self):
        return self.target_id, self.source_ns, self.clock_domain, self.clock_epoch


@dataclass(frozen=True)
class MeasuredPadBinding:
    """Measured cube/fixed-pad SAT consistency, not physical contact evidence."""
    task_id: str
    bound_source_ns: int
    identity: tuple[str, str, str, int]
    pad: OrientedBoxEnvelope

    @classmethod
    def bind(cls, *, task_id, observation, profile):
        if (not task_id or observation.target_id != profile.target_id
                or profile.cube_size_m != (.05, .05, .05)
                or profile.end_effector_frame != "gripper_frame_link"
                or profile.preclose_contact_policy is None):
            raise ValueError("invalid measured pad binding")
        pad = next(box for box in profile.required_contact_pad_obbs
                   if box.name == profile.preclose_contact_policy.fixed_pad_name)
        return cls(task_id, observation.source_ns,
                   (observation.target_id, observation.frame_id,
                    observation.clock_domain, observation.clock_epoch), pad)

    def residual_m(self, *, task_id, observation, gripper_position_m, gripper_orientation_xyzw):
        if (task_id != self.task_id or observation.source_ns < self.bound_source_ns
                or (observation.target_id, observation.frame_id, observation.clock_domain,
                    observation.clock_epoch) != self.identity):
            raise ValueError("measured pad binding mismatch")
        x, y, z, w = gripper_orientation_xyzw
        inverse = (-x, -y, -z, w)
        center = transform_point_from_frame((0., 0., 0.), inverse,
                    tuple(a-b for a, b in zip(observation.center_m, gripper_position_m, strict=True)))
        columns = [transform_point_from_frame((0., 0., 0.), inverse,
                    tuple(row[i] for row in observation.orientation_rows)) for i in range(3)]
        cube = OrientedBoxEnvelope("measured_cube", center, (.05, .05, .05),
                                    tuple(tuple(col[i] for col in columns) for i in range(3)))
        return max(0., -min(oriented_box_overlap_margins(cube, self.pad)))
