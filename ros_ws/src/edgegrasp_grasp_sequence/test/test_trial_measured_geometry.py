import json
from types import SimpleNamespace as NS

import pytest
from pathlib import Path
from math import sin, cos, radians
from ament_index_python.packages import get_package_share_directory
from edgegrasp.grasp_geometry import load_grasp_geometry_profile, validate_routed_grasp_stage_geometry

from edgegrasp_grasp_sequence.trial_client import GraspTrialClient
from edgegrasp_grasp_sequence import trial_client as trial_module


def client():
    node = object.__new__(GraspTrialClient)
    node._require_measured_geometry = True
    node._target_id = 'target_cube'
    node._clock_domain = 'ros_sim'
    node._clock_epoch = 0
    node._measured_geometry = {}
    node._target_is_recent = lambda target: True
    return node


def measurement():
    return dict(target_id='target_cube', frame_id='base_link', clock_domain='ros_sim',
                clock_epoch=0, source_ns=100, center_m=[.24, .13, .205],
                orientation_rows=[[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])


def target(source=100, x=.24):
    return NS(observation=NS(header=NS(stamp=NS(sec=0, nanosec=source)),
                             point=NS(x=x, y=.13, z=.205)))


def test_measured_geometry_uses_exact_source_and_center_and_freshness():
    node = client()
    node._on_measured_geometry(NS(data=json.dumps(measurement())))
    assert node._geometry_pose(target()) == ((.24, .13, .205), (0., 0., 0., 1.))
    for wrong in (target(101), target(x=.241)):
        with pytest.raises(ValueError, match='source_mismatch'):
            node._geometry_pose(wrong)
    node._target_is_recent = lambda target: False
    with pytest.raises(ValueError, match='source_mismatch'):
        node._geometry_pose(target())


def test_pan_variant_is_bound_once_to_exact_measured_source(monkeypatch):
    node = client()
    node._bounded_control_pan = True
    node._control_variant = None
    node._cube = NS(pose_world=NS(position_m=(.24,.13,.205)))
    node._grasp_profile = object()
    node._position = lambda name: (.1,.2,.3)
    node._orientation = lambda name: (0.,0.,0.,1.)
    node._on_measured_geometry(NS(data=json.dumps(measurement())))
    chosen = NS(approach_position_m=(1,2,3), descend_position_m=(4,5,6),
                lift_position_m=(7,8,9), approach_orientation_xyzw=(0,0,1,0),
                grasp_orientation_xyzw=(1,0,0,0))
    calls=[]
    def select(**kwargs):
        calls.append(kwargs)
        return chosen, {'selection': 'same source'}
    monkeypatch.setattr(trial_module,'select_so101_control_grasp',select)
    assert node._resolved_stage_positions(target()) == ((1,2,3),(4,5,6),(7,8,9))
    assert node._resolved_orientation(target(),'grasp_orientation_xyzw') == (1,0,0,0)
    assert len(calls) == 1
    altered=measurement()
    altered['orientation_rows']=[[-1,0,0],[0,-1,0],[0,0,1]]
    node._on_measured_geometry(NS(data=json.dumps(altered)))
    with pytest.raises(ValueError,match='source_conflict'):
        node._resolved_stage_positions(target())


@pytest.mark.parametrize('invalid', [False, True])
def test_bounded_geometry_overlaps_read_only_baseline_but_precedes_motion(invalid):
    node = object.__new__(GraspTrialClient)
    events=[]
    node._bounded_control_pan = True
    node._positive = lambda name: 100. if name == 'max_target_age_before_send_ms' else 1.
    node._nonnegative_integer = lambda name: 0
    node.get_parameter = lambda name: NS(value='test-task')
    node._wait_target = target
    node._physics_attempt_generation = 0
    node._physics_goal = lambda *args: object()
    node._wait_future = lambda *args: True
    node._target_age_ns = lambda sample: 80_000_000
    result_future=NS(done=lambda: False)
    handle=NS(accepted=True,get_result_async=lambda: result_future)
    def observe(*args,**kwargs):
        events.append('observer')
        return NS(result=lambda: handle)
    def validate(sample):
        events.append('geometry')
        if invalid:
            raise ValueError('invalid geometry')
        node._physics_phase = 'WAIT_CONTACT'
    def cancel(*args):
        assert args == (handle,result_future,'physics_geometry_preflight_failure')
        events.append('cancel')
        return True
    class MotionReached(Exception):
        pass
    def motion(*args,**kwargs):
        events.append('motion')
        raise MotionReached()
    node._physics=NS(wait_for_server=lambda **kwargs: True,send_goal_async=observe)
    node._sequence=NS(wait_for_server=lambda **kwargs: True,send_goal_async=motion)
    node._sequence_goal=lambda sample: object()
    node._validate_physics_trial_geometry=validate
    node._cancel_and_confirm=cancel
    if invalid:
        assert node.run() == 6
        assert events == ['observer','geometry','cancel']
    else:
        with pytest.raises(MotionReached):
            node.run()
        assert events == ['observer','geometry','motion']


@pytest.mark.parametrize('field,value', [('clock_epoch', 1), ('frame_id', 'world'),
    ('target_id', 'other'), ('orientation_rows', [[1, 0, 0]] * 3)])
def test_invalid_measured_geometry_cannot_replace_matching_input(field, value):
    node = client()
    payload = measurement()
    payload[field] = value
    node._on_measured_geometry(NS(data=json.dumps(payload)))
    assert not node._measured_geometry


def test_returned_cube_rotation_rejected_and_control_pose_is_immutable():
    config = Path(get_package_share_directory('edgegrasp_ros')) / 'config'
    candidate = json.loads((config/'so101_side_grasp_candidate024_face_aligned.json').read_text())
    node = client()
    node._fixed_control_stage_geometry = True
    node._cube = NS(pose_world=NS(position_m=tuple(candidate['target']['center_m'])))
    node._grasp_profile = load_grasp_geometry_profile(config/'so101_grasp_geometry_candidate024_face_aligned_q0p40.json')
    node.get_parameter = lambda name: NS(value=True if name == 'derive_descend_and_lift_from_profile' else candidate['orientations_xyzw']['grasp'])
    node._position = lambda name: tuple(candidate['stages']['approach']['position_m'])
    a, b = target(x=.24108428841333183), target(x=.24147515551360582)
    assert node._resolved_stage_positions(a) == node._resolved_stage_positions(b)
    approach, descend, lift = node._resolved_stage_positions(b)
    # M met the position-only ready gate but not the unchanged preclose margin.
    yaw = radians(32.46049237042995)
    with pytest.raises(ValueError, match='preclose_fixed_pad_clearance'):
        validate_routed_grasp_stage_geometry(
            cube_center_m=(.24147515551360582, .12948213525630275, .20500027026946513),
            approach_position_m=approach, descend_position_m=descend, lift_position_m=lift,
            approach_orientation_xyzw=tuple(candidate['orientations_xyzw']['approach']),
            grasp_orientation_xyzw=tuple(candidate['orientations_xyzw']['grasp']),
            gripper_position_rad=.4, profile=node._grasp_profile,
            cube_orientation_xyzw=(0., 0., sin(yaw/2), cos(yaw/2)))


@pytest.mark.parametrize('bounded,limit', [(False,50_000_000),(True,50_000_000)])
def test_baseline_selection_reserves_age_budget_without_relaxing_send_gate(monkeypatch,bounded,limit):
    node = object.__new__(GraspTrialClient)
    node._require_measured_geometry = False
    node.get_parameter = lambda name: NS(value={
        'target_timeout_s': 1., 'max_target_age_before_send_ms': 100.}[name])
    node._bounded_control_pan = bounded
    old, recent = NS(age=limit+1), NS(age=limit)
    queue = iter((old, recent))
    node._target_age_ns = lambda target: target.age
    monkeypatch.setattr(trial_module.rclpy, 'spin_once',
                        lambda node, timeout_sec: setattr(node, '_latest', next(queue)))
    assert node._wait_target() is recent
    assert old.age == limit+1 and recent.age == limit
    assert node._target_age_is_recent(100_000_000)
    assert not node._target_age_is_recent(100_000_001)
