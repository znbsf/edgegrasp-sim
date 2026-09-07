import threading
from types import SimpleNamespace as NS

from edgegrasp_ros.grasp_physics_observer import GraspPhysicsObserver


def test_alternating_input_reasons_do_not_defeat_feedback_buckets():
    node = object.__new__(GraspPhysicsObserver)
    result = NS(phase=NS(value='WAIT_CONTACT'), task_id='trial', pose_sample_count=1,
                gripper_contact_count=0, sequence_completed=False, final_z_m=.205,
                peak_z_m=.205, retention_started_at_ns=None)
    node._core = NS(result=result)
    node._lock = threading.RLock()
    node._last_feedback = {}
    node._event = threading.Event()
    sent = []
    node._active_goal_handle = NS(publish_feedback=sent.append)
    node._publish_status = lambda *args: None
    for count in range(1, 20):
        result.pose_sample_count = count
        node._publish_feedback('baseline_ready')
        node._publish_feedback('gripper_effort_observed')
    assert len(sent) == 2
    assert result.pose_sample_count == 19
    result.pose_sample_count = 20
    node._publish_feedback('baseline_ready')
    assert len(sent) == 3
    result.phase = NS(value='FAULT')
    node._publish_feedback('clock_rollback')
    assert len(sent) == 4 and sent[-1].phase == 'FAULT'
