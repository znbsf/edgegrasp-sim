"""Rigid pose and evidence checks shared by live and recorded-only scene loading."""

from dataclasses import dataclass
from math import isfinite, sqrt


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def rotate(q, p):
    return multiply(multiply(q, (*p, 0.)), (-q[0], -q[1], -q[2], q[3]))[:3]


@dataclass(frozen=True)
class RigidPose:
    position: tuple
    orientation: tuple

    def __post_init__(self):
        if (len(self.position) != 3 or len(self.orientation) != 4
                or not all(isfinite(v) for v in (*self.position, *self.orientation))
                or abs(sum(v*v for v in self.orientation)-1) > 1e-6):
            raise ValueError('invalid_rigid_pose')

    def inverse(self):
        x, y, z, w = self.orientation
        q = (-x, -y, -z, w)
        return RigidPose(rotate(q, tuple(-v for v in self.position)), q)

    def compose(self, other):
        offset = rotate(self.orientation, other.position)
        return RigidPose(tuple(a+b for a, b in zip(self.position, offset)),
                         multiply(self.orientation, other.orientation))


def measured_pose(value):
    rows = value['orientation_rows']
    if (len(rows) != 3 or any(len(r) != 3 for r in rows)
            or not all(isfinite(v) for r in rows for v in r)):
        raise ValueError('invalid_measured_rotation')
    for i in range(3):
        for j in range(3):
            if abs(sum(rows[i][k]*rows[j][k] for k in range(3))-(i == j)) > 1e-6:
                raise ValueError('nonorthogonal_measured_rotation')
    a, b, c = rows
    det = a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0])
    if abs(det-1) > 1e-6:
        raise ValueError('reflected_measured_rotation')
    # Select the largest quaternion component to avoid near-pi cancellation.
    components = [1+a[0]-b[1]-c[2], 1-a[0]+b[1]-c[2],
                  1-a[0]-b[1]+c[2], 1+a[0]+b[1]+c[2]]
    index = max(range(4), key=components.__getitem__)
    v = sqrt(max(0., components[index]))/2
    d = 4*v
    q = ((v, (a[1]+b[0])/d, (a[2]+c[0])/d, (c[1]-b[2])/d),
         ((a[1]+b[0])/d, v, (b[2]+c[1])/d, (a[2]-c[0])/d),
         ((a[2]+c[0])/d, (b[2]+c[1])/d, v, (b[0]-a[1])/d),
         ((c[1]-b[2])/d, (a[2]-c[0])/d, (b[0]-a[1])/d, v))[index]
    return RigidPose(tuple(value['center_m']), q)


def validate_live_measurement(value, *, now_ns, epoch, tf_stamp_ns,
                              last_source_ns=None, receive_age_s=0.):
    if (value['target_id'] != 'target_cube' or value['frame_id'] != 'base_link'
            or value['clock_domain'] != 'ros_sim' or value['clock_epoch'] != epoch):
        raise ValueError('measurement_identity_mismatch')
    source = value['source_ns']
    if type(source) is not int or not 0 <= now_ns-source <= 200_000_000:
        raise ValueError('measurement_not_fresh')
    if not 0 <= receive_age_s <= .2:
        raise ValueError('measurement_receive_stale')
    if last_source_ns is not None and source <= last_source_ns:
        raise ValueError('measurement_not_new')
    if tf_stamp_ns != source:
        raise ValueError('transform_source_mismatch')
    return measured_pose(value)
