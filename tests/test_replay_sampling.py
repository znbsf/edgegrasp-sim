import math

import pytest

from edgegrasp.replay_sampling import interpolate_pose, latest_sample, sample_pose


def test_quaternion_shortest_arc_and_translation():
    a = [0, 0, 0, 1, 0, 0, 0]
    # Opposite quaternion signs encode the same rotation.
    assert interpolate_pose(a, [2, 0, 0, -1, 0, 0, 0], 0.5) == [1, 0, 0, 1, 0, 0, 0]
    halfway = interpolate_pose(a, [0, 0, 0, 0, 0, 0, 1], 0.5)
    assert halfway[3:] == pytest.approx([math.sqrt(0.5), 0, 0, math.sqrt(0.5)])


def test_gaps_and_out_of_range_do_not_invent_motion():
    rows = [(100, [0, 0, 0, 1, 0, 0, 0]), (400, [3, 0, 0, 1, 0, 0, 0])]
    assert sample_pose(rows, [100, 400], 99) is None
    assert sample_pose(rows, [100, 400], 401) is None
    assert sample_pose(rows, [100, 400], 250, max_gap_ns=200) is None
    value, bracket = sample_pose(rows, [100, 400], 250, max_gap_ns=300)
    assert value[0] == 1.5
    assert bracket == [100, 400]
    assert sample_pose(rows, [100, 400], 100)[1] == [100, 100]


def test_contacts_and_observations_never_use_future_or_stale_samples():
    rows = [(100, {'fixed': True}), (200, {'fixed': False})]
    assert latest_sample(rows, [100, 200], 99, 20) is None
    assert latest_sample(rows, [100, 200], 121, 20) is None
    assert latest_sample(rows, [100, 200], 120, 20) == rows[0]
    assert latest_sample(rows, [100, 200], 200, 20) == rows[1]
