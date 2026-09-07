from threading import Lock
from types import SimpleNamespace as NS

from edgegrasp_moveit_adapter.adapter_node import MoveItPlanOnlyAdapter, SO101_ARM_JOINTS


def make_node(current=120, received=110):
    node = object.__new__(MoveItPlanOnlyAdapter)
    node._state_lock = Lock()
    node._last_clock_ns = received
    node._last_joint_receive_ns = received
    node._joint_positions = dict.fromkeys(SO101_ARM_JOINTS, 0.0)
    node._joint_timeout_ns = 100
    node._fault_latched = None
    def now():
        assert node._state_lock.locked()
        return NS(nanoseconds=current)
    node.get_clock = lambda: NS(now=now)
    return node


def test_callback_advancing_after_caller_clock_is_not_future_data():
    node = make_node()
    assert node._fresh_start_positions(100) == (0.0,) * len(SO101_ARM_JOINTS)


def test_current_clock_still_rejects_expired_sample():
    node = make_node(current=211)
    assert node._fresh_start_positions(100) is None


def test_real_clock_rollback_latches_and_rejects():
    node = make_node(current=109)
    assert node._fresh_start_positions(100) is None
    assert node._fault_latched.startswith('clock_rollback:')


def test_missing_sample_and_existing_fault_remain_rejected():
    node = make_node(received=None)
    assert node._fresh_start_positions(100) is None
    node = make_node()
    node._fault_latched = 'existing_fault'
    assert node._fresh_start_positions(100) is None
