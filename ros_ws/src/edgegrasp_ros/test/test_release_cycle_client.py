"""Zero-node regression: the placement client must use existing bridged evidence."""

import importlib.util
from pathlib import Path
import sys

import yaml


ROOT = Path(__file__).resolve().parents[4]


def test_placement_subscriptions_are_present_in_existing_scene_bridge(monkeypatch, tmp_path):
    monkeypatch.syspath_prepend(str(ROOT / 'scripts'))
    spec = importlib.util.spec_from_file_location('release_cycle_client_test',
                                                 ROOT / 'scripts/complete_release_cycle.py')
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module.PreparationClient, '__init__', lambda *a: None)
    monkeypatch.setattr(module, 'ActionClient', lambda *a: object())
    monkeypatch.setattr(module, 'Buffer', lambda **kw: object())
    monkeypatch.setattr(module, 'TransformListener', lambda *a: object())
    subscribed = []
    monkeypatch.setattr(module.CycleClient, 'create_subscription',
                        lambda self, typ, topic, callback, depth: subscribed.append(topic))
    module.CycleClient(tmp_path / 'events.jsonl')
    bridge = yaml.safe_load((ROOT / 'ros_ws/src/edgegrasp_ros/config/scene_bridge.yaml').read_text())
    available = {row['ros_topic_name'] for row in bridge}
    assert set(subscribed) <= available
    assert '/edgegrasp/target_cube_contacts' in subscribed
