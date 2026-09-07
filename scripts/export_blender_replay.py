#!/usr/bin/env python3
"""Export recorded ROS transforms and cube poses to portable Blender input. No ROS node."""

import argparse
import csv
import hashlib
import json
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path
from urllib.parse import unquote, urlparse

from edgegrasp.replay_sampling import latest_sample, sample_pose


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def pose(position, rotation):
    value = [position.x, position.y, position.z, rotation.w, rotation.x, rotation.y, rotation.z]
    if not all(math.isfinite(x) for x in value):
        raise ValueError('non-finite pose')
    if abs(sum(x*x for x in value[3:]) - 1) > 1e-4:
        raise ValueError('non-unit recorded quaternion')
    return value


def main():
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('output', type=Path, help='New portable output directory')
    parser.add_argument('--world', required=True, type=Path, help='Matching experiment SDF world')
    parser.add_argument('--fps', default=30, type=int, choices=(15, 24, 30, 60))
    args = parser.parse_args()
    run = args.run.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    assets = args.output / 'assets'
    assets.mkdir()
    hashes = {}

    def copy_source(source, destination):
        digest = sha(source)
        shutil.copyfile(source, destination)
        hashes[str(source)] = digest
        return digest

    urdf_path = run / 'generated_proxy.urdf'
    robot = ET.parse(urdf_path)
    copy_source(urdf_path, args.output / 'source_robot.urdf')
    for mesh in robot.findall('.//mesh'):
        uri = mesh.attrib['filename']
        if uri.startswith('file://'):
            source = Path(unquote(urlparse(uri).path))
        elif uri.startswith('package://'):
            from ament_index_python.packages import get_package_share_directory
            package, relative = uri[10:].split('/', 1)
            source = Path(get_package_share_directory(package)) / relative
        else:
            source = (urdf_path.parent / uri).resolve()
        digest = sha(source)
        relative = Path('assets') / (digest[:12] + '_' + source.name)
        if not (args.output / relative).exists():
            copy_source(source, args.output / relative)
        mesh.set('filename', relative.as_posix())
    robot.write(args.output / 'robot.urdf', encoding='utf-8', xml_declaration=True)
    copy_source(args.world, args.output / 'world.sdf')
    # Bind the supplied world to the recorded scene digest, not just a similar filename.
    logs = []
    for line in (run / 'trial.log').read_text().splitlines():
        if line.startswith('{'):
            logs.append(json.loads(line))
    result = next((x for x in reversed(logs) if x.get('type') == 'grasp_trial_result'), None)
    if result and result.get('scene_digest'):
        if 'scene_contract_sha256=' + result['scene_digest'] not in args.world.read_text():
            raise ValueError('World does not match recorded scene digest')
    for name in ('trial.log', 'grasp_timeline.csv', 'gripper_release_exit_code.txt',
                 'gripper_release_fallback_status.txt', 'robot_pose_final.log', 'release_cycle.jsonl'):
        if (run / name).exists():
            copy_source(run / name, args.output / name)
    # This exporter supports the fixed-base experiment contract only.
    base_joint = robot.find("joint[@name='world_joint']/origin")
    if base_joint is None or any(float(x) != 0 for key in ('xyz', 'rpy')
                                 for x in base_joint.get(key, '0 0 0').split()):
        raise ValueError('Expected fixed robot world origin')

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(run / 'mcap'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    wanted = {'/tf', '/tf_static', '/edgegrasp/target_cube_pose', '/edgegrasp/measured_cube',
              '/edgegrasp/grasp_physics_status', '/edgegrasp/perception_status',
              '/edgegrasp/safety_status'}
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in wanted}
    tracks, parents, static, cube, observations, statuses = {}, {}, {}, {}, {}, []
    last_clock = None
    while reader.has_next():
        topic, data, receive = reader.read_next()
        if topic not in types:
            continue
        m = deserialize_message(data, types[topic])
        if topic in ('/tf', '/tf_static'):
            for tf in m.transforms:
                child, parent = tf.child_frame_id, tf.header.frame_id
                if child in parents and parents[child] != parent:
                    raise ValueError('TF parent changed')
                parents[child] = parent
                value = pose(tf.transform.translation, tf.transform.rotation)
                stamp = tf.header.stamp.sec * 10**9 + tf.header.stamp.nanosec
                if topic == '/tf_static':
                    if child in static and static[child] != value:
                        raise ValueError('Static TF changed')
                    static[child] = value
                else:
                    prior = tracks.setdefault(child, {})
                    if prior and stamp < max(prior):
                        raise ValueError('TF clock reset: split recording into epochs first')
                    if stamp in prior and prior[stamp] != value:
                        raise ValueError('Conflicting TF sample')
                    prior[stamp] = value
        elif topic == '/edgegrasp/target_cube_pose':
            if m.header.frame_id != 'edgegrasp_table_cube':
                raise ValueError('Unexpected cube truth frame')
            stamp = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
            if last_clock is not None and stamp < last_clock:
                raise ValueError('Cube clock reset')
            last_clock = stamp
            cube[stamp] = pose(m.pose.position, m.pose.orientation)
        elif topic == '/edgegrasp/measured_cube':
            from edgegrasp.target_motion import MeasuredCubeObservation
            value = json.loads(m.data)
            MeasuredCubeObservation.parse(value)
            if value['frame_id'] != 'base_link' or value['clock_epoch'] != 0:
                raise ValueError('Unsupported measured cube frame/epoch')
            observations[value['source_ns']] = value
        else:
            statuses.append({'receive_ns': receive, 'topic': topic, 'text': m.data})
    required = {j.find('child').get('link') for j in robot.findall('joint')}
    if required - parents.keys():
        raise ValueError('Missing recorded TF links: ' + str(required - parents.keys()))
    for link in required:
        seen = set()
        while link in parents:
            if link in seen:
                raise ValueError('TF cycle')
            seen.add(link)
            link = parents[link]
        if link != 'world':
            raise ValueError('Disconnected TF tree')
    if not tracks or not cube:
        raise ValueError('No motion/cube data')
    tracks = {key: sorted(values.items()) for key, values in tracks.items()}
    cube = sorted(cube.items())
    observations = sorted(observations.items())
    stamps = {key: [x[0] for x in rows] for key, rows in tracks.items()}
    cube_stamps, obs_stamps = [x[0] for x in cube], [x[0] for x in observations]
    start = max([cube[0][0]] + [rows[0][0] for rows in tracks.values()])
    end = min([cube[-1][0]] + [rows[-1][0] for rows in tracks.values()])
    if end <= start:
        raise ValueError('No common source time interval')
    timeline = list(csv.DictReader((run / 'grasp_timeline.csv').open()))
    events = [{k: v for k, v in row.items() if v} for row in timeline if row['stream'] == 'sequence']
    cycle_rows = []
    unplaced_cycle_events = []
    if (run / 'release_cycle.jsonl').exists():
        cycle_rows = [json.loads(line) for line in (run / 'release_cycle.jsonl').read_text().splitlines()]
        previous_stage = None
        for row in cycle_rows:
            stage = row.get('status') or row.get('stage')
            if stage != previous_stage and 'observed_now_ns' in row:
                if row['observed_now_ns'] <= 0:
                    # A newly joined client may not have received /clock yet.
                    # Keep its record, but never invent a timeline placement.
                    unplaced_cycle_events.append(row)
                    previous_stage = stage
                    continue
                events.append({'time_ns': row['observed_now_ns'], 'phase': stage,
                               'reason': row.get('reason', 'recorded post-grasp observation'),
                               'time_basis': 'client_observed_ros_clock'})
                previous_stage = stage
        events.sort(key=lambda row: int(row['time_ns']))
    contacts = sorted((int(r['time_ns']), {'fixed': r['fixed_contact'] == 'True',
                        'moving': r['moving_contact'] == 'True'})
                      for r in timeline if r['stream'] == 'contact')
    contact_stamps = [x[0] for x in contacts]
    frames, gaps = [], []
    for index in range(int((end - start) / 1e9 * args.fps) + 1):
        stamp = start + round(index * 1e9 / args.fps)
        tf_samples = {key: sample_pose(rows, stamps[key], stamp) for key, rows in tracks.items()}
        cube_sample = sample_pose(cube, cube_stamps, stamp)
        complete = cube_sample is not None and all(x is not None for x in tf_samples.values())
        if not complete:
            gaps.append(index + 1)
        observation = latest_sample(observations, obs_stamps, stamp, 100_000_000)
        contact = latest_sample(contacts, contact_stamps, stamp, 20_000_000)
        frames.append({'frame': index + 1, 'source_ns': stamp, 'complete': complete,
                       'tf': tf_samples, 'cube': cube_sample,
                       'observation': observation, 'contact': contact})
    for path in (run / 'mcap').glob('*.mcap'):
        hashes[str(path)] = sha(path)
    release = run / 'gripper_release_exit_code.txt'
    spatial_path = run / 'live_rgbd_evaluation' / 'spatial_evaluation.json'
    spatial_failures = []
    if spatial_path.exists():
        copy_source(spatial_path, args.output / 'spatial_evaluation.json')
        spatial_failures = [row for row in json.loads(spatial_path.read_text())['rows'] if not row['passed']]
    report = {'schema_version': 1, 'claim': 'recorded_simulation_visual_replay_only',
              'physics_recomputed': False, 'hardware_verified': False,
              'run': str(run), 'fps': args.fps, 'source_interval_ns': [start, end],
              'display_interpolation': 'linear position + quaternion slerp, max bracket 200 ms',
              'observation_display': 'latest source sample <=100 ms, no interpolation',
              'contact_display': 'latest source sample <=20 ms; sampled display can miss brief contacts',
              'source_hashes': hashes, 'parents': parents, 'static': static,
              'frames': frames, 'gap_frames': gaps, 'events': events, 'statuses': statuses,
              'unplaced_cycle_events': unplaced_cycle_events,
              'spatial_failures': spatial_failures,
              'recorded_result': result,
              'recorded_cycle_status': next((r['status'] for r in reversed(cycle_rows) if 'status' in r), 'NOT_RECORDED'),
              'typed_release_exit_code': int(release.read_text()) if release.exists() else None}
    (args.output / 'replay.json').write_text(json.dumps(report, allow_nan=False), encoding='utf-8')
    print(json.dumps({'output': str(args.output), 'frames': len(frames), 'gaps': len(gaps),
                      'meshes': len(list(assets.iterdir())), 'source_interval_ns': [start, end]}))


if __name__ == '__main__':
    main()
