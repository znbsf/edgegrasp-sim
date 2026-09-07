import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'scripts'))
import carried_scene_client as client


MISSING = 'fresh_measured_cube_and_causal_tf_missing'


def make_harness(monkeypatch, responses, *, fresh=True, pending=False):
    callbacks, records, calls = {}, [], []
    wall = [0.]
    node = NS()
    def subscribe(kind, topic, callback, qos):
        callbacks[topic] = callback
        return topic
    def call(request):
        calls.append(request)
        result = responses[min(len(calls)-1, len(responses)-1)]
        return NS(done=lambda: not pending, result=lambda: result)
    node.create_subscription = subscribe
    node.create_client = lambda *args: NS(wait_for_service=lambda **kw: True, call_async=call)
    node.destroy_client = lambda value: None
    node.destroy_subscription = lambda value: None
    node.record = records.append
    def spin(node, timeout_sec):
        wall[0] += .02
        if fresh:
            callbacks['/edgegrasp/measured_cube'](NS(data=json.dumps(dict(
                target_id='target_cube', frame_id='base_link', clock_domain='ros_sim',
                clock_epoch=0, source_ns=int(wall[0]*1e9)))))
        callbacks['/edgegrasp/planning_scene_status'](NS(data=json.dumps(dict(
            scene_generation=3, carried_state='placed', ready=True, reason='confirmed',
            clock_domain='ros_sim', clock_epoch=0, carried_evidence={'scope': 'live_rgbd_tf'}))))
    monkeypatch.setattr(client.rclpy, 'spin_once', spin)
    monkeypatch.setattr(client.time, 'monotonic', lambda: wall[0])
    return node, calls, records


def test_confirmed_no_mutation_input_rejection_waits_for_new_source(monkeypatch):
    node, calls, records = make_harness(monkeypatch, [NS(success=False, message=MISSING),
        NS(success=True, message=json.dumps(dict(generation=3, state='placed')))])
    result = client.transition(node, 'detach', scope='live_rgbd_tf', timeout_s=.2)
    assert len(calls) == 2
    assert len(result['client_admission_rejections']) == 1
    assert records[0]['stage'] == 'carried_input_wait'


@pytest.mark.parametrize('reason', ['carried_transition_clock_or_source_fault',
    'invalid_carried_transition:world:detach', 'unknown_error'])
def test_other_rejection_never_renews_request(monkeypatch, reason):
    node, calls, _ = make_harness(monkeypatch, [NS(success=False, message=reason)])
    with pytest.raises(RuntimeError, match='carried_transition_rejected'):
        client.transition(node, 'detach', scope='live_rgbd_tf', timeout_s=.2)
    assert len(calls) == 1


@pytest.mark.parametrize('pending,reason', [(True, 'response_timeout'), (False, 'fresh_input_wait_timeout')])
def test_ambiguous_receipt_or_absent_new_input_never_resends(monkeypatch, pending, reason):
    node, calls, _ = make_harness(monkeypatch, [NS(success=False, message=MISSING)], fresh=False, pending=pending)
    with pytest.raises(RuntimeError, match=reason):
        client.transition(node, 'detach', scope='live_rgbd_tf', timeout_s=.2)
    assert len(calls) == 1


def test_input_wait_has_three_request_ceiling(monkeypatch):
    node, calls, records = make_harness(monkeypatch, [NS(success=False, message=MISSING)])
    with pytest.raises(RuntimeError, match='carried_transition_rejected'):
        client.transition(node, 'detach', scope='live_rgbd_tf', timeout_s=.2)
    assert len(calls) == 3
    assert len(records) == 2
