"""No-node checks of the success-only handoff wrapper's admission conditions."""

from threading import RLock
from types import SimpleNamespace as NS

import pytest
from std_srvs.srv import Trigger
from edgegrasp_grasp_sequence.node import GraspSequenceNode


def wrapper():
    calls=[]
    n=NS(_core_event_lock=RLock(), _state_lock=RLock(), _active_goal_handle=None,
         _reserved_task_id=None, _arm_slot=None, _gripper_slot=None,
         _latest_target=NS(position=(.24, .13, .205), source_timestamp_ns=100),
         _joint_state=NS(positions={'gripper': 1.5}),
         _planning_scene_status=NS(carried_state='placed'),
         _active_request=NS(target=NS(observation=NS(point=NS(x=.24, y=.13, z=.205)))),
         _uncertain_commands=set(), _input_fault=None, _now_ns=lambda: 110,
         _planning_scene_policy_reason=lambda **kw: None,
         _observed_gripper_pose=lambda stamp: ((.1, .13, .3), (0., 0., 0., 1.)),
         _health=lambda now: object(), _publish_status=lambda *args: None)
    n._core=NS(finish_completed_cycle=lambda *args:
               (calls.append(args) or NS(accepted=True, reason='completed_cycle_handoff')))
    return n, calls


def test_handoff_calls_core_without_resetting_epoch_or_inputs():
    node, calls=wrapper()
    before=node._latest_target
    result=GraspSequenceNode._on_finish_completed_cycle(node, None, Trigger.Response())
    assert result.success and len(calls) == 1
    assert node._active_request is None and node._latest_target is before


@pytest.mark.parametrize('case', ['busy', 'uncertain', 'fault', 'scene', 'closed', 'displaced', 'near'])
def test_handoff_blocks_unsafe_or_incomplete_conditions(case):
    node, calls=wrapper()
    if case == 'busy': node._arm_slot=object()
    if case == 'uncertain': node._uncertain_commands={'unconfirmed'}
    if case == 'fault': node._input_fault='latched'
    if case == 'scene': node._planning_scene_status.carried_state='attached'
    if case == 'closed': node._joint_state.positions['gripper']=.8
    if case == 'displaced': node._latest_target.position=(.25, .13, .205)
    if case == 'near': node._observed_gripper_pose=lambda stamp: ((.24, .13, .205), (0., 0., 0., 1.))
    result=GraspSequenceNode._on_finish_completed_cycle(node, None, Trigger.Response())
    assert not result.success and not calls
