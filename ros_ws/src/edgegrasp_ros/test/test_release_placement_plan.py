"""Validate candidate input boundaries without creating ROS nodes or motion."""

import hashlib
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'scripts'))
from probe_grasp_candidate_plan_only import _arguments, _load_stages
from complete_release_cycle import validate_regrasp_geometry


def test_placement_candidate_provenance_and_bounds(tmp_path, monkeypatch):
    for key in ('EDGEGRASP_RGBD_OBSERVATION', 'EDGEGRASP_PLACEMENT_COMPENSATION',
                'EDGEGRASP_UPRIGHT_PLACEMENT'):
        monkeypatch.delenv(key, raising=False)
    source = tmp_path / 'measurement.json'
    source.write_text('{}')
    candidate = tmp_path / 'candidate.json'
    value = dict(scope='fixed_place_compensation_from_recorded_rgbd',
                 translation_m=[.001, 0., 0.], yaw_delta_deg=.1,
                 measurement_file=str(source),
                 measurement_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
    candidate.write_text(json.dumps(value))
    args = _arguments(['--label', 'zero-motion-test', '--release-cycle',
                       '--candidate-filename', 'so101_side_grasp_candidate024_face_aligned.json',
                       '--scene-config-filename', 'scene_candidate024_face_aligned.json',
                       '--grasp-geometry-filename', 'so101_grasp_geometry_candidate024_face_aligned_q0p40.json',
                       '--placement-compensation-json', str(candidate)])
    config = ROOT / 'ros_ws/src/edgegrasp_ros/config'
    stages, evidence = _load_stages(config, args)
    assert len(stages) == 5
    plan = {'config': evidence, 'segments': [dict(target_position_m=s.position_m,
            target_orientation_xyzw=s.orientation_xyzw) for s in stages]}
    from edgegrasp.carried_scene import rotate
    q = evidence['cube_orientation_xyzw']
    columns = [rotate(q, axis) for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
    measurement = dict(center_m=evidence['cube_center_m'], source_ns=123,
                       orientation_rows=[list(row) for row in zip(*columns)])
    assert validate_regrasp_geometry(plan, measurement)['preclose_geometry_validated']
    measurement['orientation_rows'] = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    with pytest.raises(ValueError, match='preclose_fixed_pad_clearance'):
        validate_regrasp_geometry(plan, measurement)
    assert evidence['placement_compensation']['measurement_sha256'] == value['measurement_sha256']
    source.write_text('{"changed":true}')
    with pytest.raises(ValueError, match='provenance mismatch'):
        _load_stages(config, args)
    source.write_text('{}')
    for changes in ({'translation_m': [.026, 0, 0]}, {'yaw_delta_deg': 1.01},
                    {'translation_m': [0, 0, .001]}, {'yaw_delta_deg': float('nan')}):
        candidate.write_text(json.dumps(value | changes))
        with pytest.raises(ValueError, match='declared'):
            _load_stages(config, args)
