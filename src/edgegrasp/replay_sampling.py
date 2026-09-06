"""Read-only display interpolation. Never execution or contact evidence."""

import bisect
import math


def interpolate_pose(a, b, fraction):
    position = [x + fraction * (y - x) for x, y in zip(a[:3], b[:3])]
    qa, qb = a[3:], b[3:]
    for q in (qa, qb):
        if abs(sum(x * x for x in q) - 1) > 1e-4:
            raise ValueError("non-unit quaternion")
    dot = sum(x * y for x, y in zip(qa, qb))
    if dot < 0:
        qb, dot = [-x for x in qb], -dot
    dot = min(1.0, dot)
    if dot > 0.9995:
        q = [x + fraction * (y - x) for x, y in zip(qa, qb)]
    else:
        angle = math.acos(dot)
        q = [(math.sin((1 - fraction) * angle) * x + math.sin(fraction * angle) * y)
             / math.sin(angle) for x, y in zip(qa, qb)]
    norm = math.sqrt(sum(x * x for x in q))
    return position + [x / norm for x in q]


def sample_pose(rows, stamps, stamp, max_gap_ns=200_000_000):
    """Return pose and its true source bracket; no extrapolation or long-gap bridging."""
    i = bisect.bisect_left(stamps, stamp)
    if i < len(stamps) and stamps[i] == stamp:
        return rows[i][1], [stamp, stamp]
    if i == 0 or i == len(stamps):
        return None
    lo, hi = stamps[i - 1], stamps[i]
    if hi - lo > max_gap_ns:
        return None
    return interpolate_pose(rows[i - 1][1], rows[i][1], (stamp - lo) / (hi - lo)), [lo, hi]


def latest_sample(rows, stamps, stamp, max_age_ns):
    i = bisect.bisect_right(stamps, stamp) - 1
    if i < 0 or stamp - stamps[i] > max_age_ns:
        return None
    return rows[i]
