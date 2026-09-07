from copy import deepcopy
import math

import pytest

from edgegrasp.carried_scene import RigidPose, measured_pose, validate_live_measurement


def measurement():
    return dict(target_id='target_cube', frame_id='base_link', clock_domain='ros_sim',
                clock_epoch=0, source_ns=1_000_000_000, center_m=[.24, .13, .22],
                orientation_rows=[[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])


def test_relative_pose_roundtrip():
    world = RigidPose((.2, .1, .3), (0., 0., math.sin(.3), math.cos(.3)))
    cube = measured_pose(measurement())
    recovered = world.compose(world.inverse().compose(cube))
    assert recovered.position == pytest.approx(cube.position)
    assert recovered.orientation == pytest.approx(cube.orientation)


@pytest.mark.parametrize('changes,kwargs', [
    ({'clock_epoch': 1}, {}), ({'frame_id': 'world'}, {}),
    ({'source_ns': 1_100_000_001}, {}), ({'source_ns': 899_999_999}, {}),
    ({}, {'tf_stamp_ns': 999_999_999}), ({}, {'last_source_ns': 1_000_000_000}),
    ({}, {'receive_age_s': .201}), ({'target_id': 'other'}, {}),
])
def test_live_identity_time_and_tf_rejected(changes, kwargs):
    args = dict(now_ns=1_100_000_000, epoch=0, tf_stamp_ns=1_000_000_000)
    with pytest.raises(ValueError):
        validate_live_measurement(measurement() | changes, **(args | kwargs))


def test_rotation_rejects_reflection_and_nonorthogonal_input():
    value = measurement()
    reflected = deepcopy(value)
    reflected['orientation_rows'][2][2] = -1.
    with pytest.raises(ValueError, match='reflected'):
        measured_pose(reflected)
    value['orientation_rows'][0][1] = .1
    with pytest.raises(ValueError, match='nonorthogonal'):
        measured_pose(value)


@pytest.mark.parametrize('diagonal', [(1, -1, -1), (-1, 1, -1), (-1, -1, 1)])
def test_pi_rotations_are_stable(diagonal):
    value = measurement()
    value['orientation_rows'] = [[diagonal[i] if i == j else 0. for j in range(3)] for i in range(3)]
    assert sum(q*q for q in measured_pose(value).orientation) == pytest.approx(1.)
