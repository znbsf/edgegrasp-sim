import json
import math
from pathlib import Path

import pytest

from edgegrasp.grasp_geometry import GraspGeometryError, load_grasp_geometry_profile, select_so101_control_grasp

CONFIG = Path(__file__).resolve().parents[1] / 'ros_ws/src/edgegrasp_ros/config'


def inputs():
    c = json.loads((CONFIG/'so101_side_grasp_candidate024_face_aligned.json').read_text())
    return dict(nominal_center_m=tuple(c['target']['center_m']),
                approach_position_m=tuple(c['stages']['approach']['position_m']),
                approach_orientation_xyzw=tuple(c['orientations_xyzw']['approach']),
                grasp_orientation_xyzw=tuple(c['orientations_xyzw']['grasp']),
                profile=load_grasp_geometry_profile(CONFIG/'so101_grasp_geometry_candidate024_face_aligned_q0p40.json'))


@pytest.mark.parametrize('center,yaw', [
    ((.2417084027,.1291562202,.2049998709),34.826),
    ((.2417283443,.1291064355,.2050000920),34.826),
    ((.2415563839,.1287984614,.2050000721),37.230),
])
def test_measured_regrasp_variants_preserve_original_envelopes(center,yaw):
    theta = math.radians(yaw)/2
    geometry, result = select_so101_control_grasp(
        cube_center_m=center, cube_orientation_xyzw=(0,0,math.sin(theta),math.cos(theta)), **inputs())
    assert abs(result['shoulder_pan_delta_deg']) <= .5
    assert .0005 <= result['fixed_pad_clearance_m'] <= .0015
    assert abs(result['fixed_pad_clearance_m']-.001) < .000025
    assert len(geometry.grasp_orientation_xyzw) == 4


def test_bounded_pan_does_not_admit_outside_control_scene():
    with pytest.raises(GraspGeometryError, match='outside_predeclared_control_scene'):
        select_so101_control_grasp(cube_center_m=(.2417,.128348,.205),
                                  cube_orientation_xyzw=(0,0,0,1), **inputs())


def test_bounded_pan_does_not_make_arbitrary_orientation_valid():
    values = inputs()
    with pytest.raises(GraspGeometryError, match='no_bounded_pan_variant'):
        select_so101_control_grasp(cube_center_m=values['nominal_center_m'],
                                  cube_orientation_xyzw=(0,0,0,1), **values)


def test_gap_prefilter_never_bypasses_full_geometry(monkeypatch):
    from edgegrasp import grasp_geometry as module
    calls=[]
    def reject(**kwargs):
        calls.append(kwargs)
        raise GraspGeometryError('noncontact_body_clearance')
    monkeypatch.setattr(module,'validate_routed_grasp_stage_geometry',reject)
    values=inputs()
    yaw=math.radians(37.2426998157748)/2
    with pytest.raises(GraspGeometryError,match='no_bounded_pan_variant'):
        select_so101_control_grasp(cube_center_m=values['nominal_center_m'],
            cube_orientation_xyzw=(0,0,math.sin(yaw),math.cos(yaw)),**values)
    assert calls
