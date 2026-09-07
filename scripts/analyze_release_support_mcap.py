#!/usr/bin/env python3
"""Replay support scoring from recorded evidence only; creates no ROS node."""

import argparse
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path

from edgegrasp.release_cycle import PlacementSample, PlacementSynchronizer, StablePlacement
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def stamp(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--after-ns', type=int, required=True)
    args = parser.parse_args()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[
        '/edgegrasp/target_cube_pose', '/edgegrasp/target_cube_contacts', '/joint_states']))
    sync = PlacementSynchronizer()
    scorer = StablePlacement()
    rows, counts = [], Counter()
    while reader.has_next():
        topic, data, receive_ns = reader.read_next()
        message = deserialize_message(data, types[topic])
        if topic.endswith('contacts'):
            sync.push('contacts', stamp(message), message)
        elif topic == '/joint_states':
            sync.push('joints', stamp(message), message)
        else:
            sync.push('poses', stamp(message), message)
        for source, pose, matched, reason in sync.ready(receive_ns):
            if source < args.after_ns:
                continue
            if reason:
                counts[reason] += 1
                scorer.first = None
                continue
            contacts, joints = matched[0][1], matched[1][1]
            pairs = [f'{c.collision1.name}|{c.collision2.name}' for c in contacts.contacts]
            table = any('target_cube::' in p and 'table::' in p for p in pairs)
            pad = any('target_cube::' in p and 'finger_pad' in p for p in pairs)
            p, q = pose.pose.position, pose.pose.orientation
            sample = PlacementSample(source, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w),
                                     table, pad, joints.position[list(joints.name).index('gripper')])
            passed = scorer.observe(sample, receive_ns)
            counts['scored'] += 1
            counts['supported'] += int(passed)
            rows.append({**asdict(sample), 'supported': passed, 'decision_ns': receive_ns,
                         'contact_source_ns': matched[0][0], 'joint_source_ns': matched[1][0]})
    result = {'scope': 'offline_recorded_support_only', 'execution_attempted': False,
              'release_verified': False, 'counts': dict(counts),
              'first_supported_ns': next((r['stamp_ns'] for r in rows if r['supported']), None),
              'samples': rows}
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps({k: v for k, v in result.items() if k != 'samples'}))


if __name__ == '__main__':
    main()
