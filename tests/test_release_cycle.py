from dataclasses import replace

import pytest

from edgegrasp.release_cycle import PlacementSample, PlacementSynchronizer, StablePlacement, cube_bottom


def sample(t, **changes):
    return replace(PlacementSample(t, (0.241, 0.13, 0.205), (0., 0., 0., 1.),
                                   True, False, 1.5), **changes)


def test_support_requires_half_second_and_actual_contact():
    scorer = StablePlacement()
    for i in range(10):
        assert not scorer.observe(sample(i*50_000_000), i*50_000_000)
    assert scorer.observe(sample(500_000_000), 500_000_000)
    assert not scorer.observe(sample(550_000_000, table_contact=False), 550_000_000)
    assert not scorer.observe(sample(600_000_000), 600_000_000)


@pytest.mark.parametrize('changes', [dict(pad_contact=True), dict(gripper_position=.81),
                                    dict(position=(.241, .13, .233)),
                                    dict(position=(.3, .13, .205))])
def test_release_rejects_contact_partial_open_hanging_or_outside_region(changes):
    scorer = StablePlacement(released=True)
    for i in range(20):
        assert not scorer.observe(sample(i*50_000_000, **changes), i*50_000_000)


def test_gap_stale_duplicate_and_motion_break_continuity():
    scorer = StablePlacement()
    for i in range(10):
        scorer.observe(sample(i*50_000_000), i*50_000_000)
    assert not scorer.observe(sample(450_000_000), 500_000_000)
    assert not scorer.observe(sample(800_000_000), 800_000_000)
    assert not scorer.observe(sample(850_000_000), 1_100_000_000)
    assert not scorer.observe(sample(1_150_000_000), 1_150_000_000)


def test_tilted_cube_bottom_is_not_center_minus_half_side():
    bottom = cube_bottom((0.2346795035, .1274637891, .2338398720),
                         (-.0878502627, .3001181403, .3137770228, .8965240728))
    assert bottom == pytest.approx(.1981273587)
    with pytest.raises(ValueError):
        cube_bottom((0., 0., .2), (0., 0., 0., 0.))


def test_future_pose_waits_for_clock_and_uses_causal_pairs():
    sync = PlacementSynchronizer()
    sync.push('poses', 10_000_000, 'pose')
    sync.push('contacts', 9_000_000, 'prior_contact')
    sync.push('contacts', 11_000_000, 'future_contact')
    sync.push('joints', 10_000_000, 'joint')
    assert sync.ready(9_000_000) == []
    row, = sync.ready(10_000_000)
    assert row == (10_000_000, 'pose', [(9_000_000, 'prior_contact'), (10_000_000, 'joint')], None)
    assert sync.ready(11_000_000) == []  # Not counted twice.


def test_missing_callback_waits_but_stale_and_unmatched_inputs_reject():
    sync = PlacementSynchronizer()
    sync.push('poses', 100_000_000, 'pose')
    assert sync.ready(110_000_000) == []
    sync.push('contacts', 100_000_000, 'contact')
    sync.push('joints', 100_000_000, 'joint')
    assert sync.ready(120_000_000)[0][-1] is None
    sync.push('poses', 200_000_000, 'missing')
    assert sync.ready(260_000_000)[0][-1] == 'matching_contact_or_joint_missing'
    sync.push('poses', 300_000_000, 'stale')
    assert sync.ready(501_000_000)[0][-1] == 'pose_stale'


@pytest.mark.parametrize('stream', ['poses', 'contacts', 'joints'])
def test_source_rollback_is_latched(stream):
    sync = PlacementSynchronizer()
    sync.push(stream, 10, 'a')
    with pytest.raises(ValueError, match='rollback'):
        sync.push(stream, 9, 'b')
    with pytest.raises(ValueError, match='rollback'):
        sync.ready(20)


def test_clock_rollback_is_latched():
    sync = PlacementSynchronizer()
    sync.ready(100)
    with pytest.raises(ValueError, match='rollback'):
        sync.ready(99)
    with pytest.raises(ValueError, match='rollback'):
        sync.ready(101)


def test_opening_allows_fixed_touch_but_final_withdrawal_requires_full_separation():
    opened = StablePlacement(opened=True)
    separated = StablePlacement(released=True)
    for i in range(11):
        row = sample(i*50_000_000, pad_contact=True, moving_pad_contact=False)
        passed = opened.observe(row, row.stamp_ns)
        assert not separated.observe(row, row.stamp_ns)
    assert passed
    assert not opened.observe(sample(550_000_000, moving_pad_contact=True), 550_000_000)
