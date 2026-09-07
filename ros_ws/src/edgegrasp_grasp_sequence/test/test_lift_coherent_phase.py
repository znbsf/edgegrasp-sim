from collections import OrderedDict
from threading import RLock
from types import SimpleNamespace as NS

from tf2_ros import TransformException
from edgegrasp.grasp_sequence import GraspPhase
from edgegrasp_grasp_sequence.node import GraspSequenceNode


def test_planning_and_execution_use_same_fresh_causal_pair():
    node = object.__new__(GraspSequenceNode)
    node._state_lock = RLock()
    node._active_request = NS(target=NS(target_id='cube', observation=NS(header=NS(stamp=0))))
    node._permission = node._interface = node._joint_state = None
    node._input_fault = None
    node._active_anchor = None
    node._target_frame = 'base_link'
    node._clock_domain = 'ros_sim'
    node._clock_epoch = 0
    node._observed_target_motion = 'measured_pad'
    node._source_timeout_ms = 200
    node._stamp_ns = lambda stamp: 100
    def sample(stamp):
        return NS(key=('cube',stamp,'ros_sim',0), target_id='cube',
                  source_timestamp_ns=stamp, observed_at_ns=stamp,
                  frame_id='base_link', clock_domain='ros_sim', clock_epoch=0,
                  position=(.24,.13,.205))
    older, latest = sample(110), sample(120)
    node._latest_target = latest
    node._target_cache = OrderedDict((s.key,s) for s in (older,latest))
    node._lift_observation_binding = NS(bound_source_ns=100)
    available = {110}
    def pose(stamp):
        if stamp not in available:
            raise TransformException('awaiting exact-source TF')
        return ((0,0,0),(0,0,0,1))
    node._observed_gripper_pose = pose
    node._core = NS(phase=GraspPhase.LIFT_PLAN)
    planned = node._health(130)
    node._core.phase = GraspPhase.LIFT_EXEC
    executing = node._health(131)
    assert planned.target_source_timestamp_ns == executing.target_source_timestamp_ns == 110
    available.add(120)
    assert node._health(132).target_source_timestamp_ns == 120
    node._input_fault = 'target_source_timestamp_not_monotonic'
    health = node._health(133)
    assert not health.target.ready
    assert health.target.reason == 'target_source_timestamp_not_monotonic'
