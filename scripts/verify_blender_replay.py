"""Reopen baked .blend and compare displayed transforms with the exported source samples."""

import json
import sys
from pathlib import Path

import bpy
from mathutils import Matrix, Quaternion, Vector


root = Path(sys.argv[sys.argv.index('--') + 1]).resolve()
data = json.loads((root / 'replay.json').read_text(encoding='utf-8'))
bpy.ops.wm.open_mainfile(filepath=str(root / 'replay.blend'), use_scripts=False)
scene = bpy.context.scene
for child, parent in data['parents'].items():
    assert bpy.data.objects['TF/' + child].parent == bpy.data.objects['TF/' + parent]
assert bpy.data.objects['Cube / recorded Gazebo pose'].parent is None
assert bpy.data.objects['Cube / measured RGB-D (latest <=100ms)'].parent == bpy.data.objects['TF/base_link']
for name, value in data['static'].items():
    obj = bpy.data.objects['TF/' + name]
    assert (obj.location - Vector(value[:3])).length < 1e-6
    assert obj.rotation_quaternion.rotation_difference(Quaternion(value[3:])).angle < 0.001
maximum_position_error = maximum_angle_error = 0.0
checked = 0
for row in data['frames']:
    scene.frame_set(row['frame'])
    expected = {'TF/' + name: value[0] for name, value in row['tf'].items() if value}
    if row['cube']:
        expected['Cube / recorded Gazebo pose'] = row['cube'][0]
    if row['observation']:
        observation = row['observation'][1]
        q = Matrix(observation['orientation_rows']).to_quaternion()
        expected['Cube / measured RGB-D (latest <=100ms)'] = observation['center_m'] + list(q)
    for name, value in expected.items():
        obj = bpy.data.objects[name]
        distance = (obj.location - Vector(value[:3])).length
        angle = obj.rotation_quaternion.rotation_difference(Quaternion(value[3:])).angle
        maximum_position_error = max(maximum_position_error, distance)
        maximum_angle_error = max(maximum_angle_error, min(angle, abs(6.283185307179586 - angle)))
        checked += 1
    assert bpy.data.objects['Cube / recorded Gazebo pose'].hide_render == (row['cube'] is None)
    assert bpy.data.objects['Cube / measured RGB-D (latest <=100ms)'].hide_render == (row['observation'] is None)
assert maximum_position_error < 1e-6, maximum_position_error
assert maximum_angle_error < 0.001, maximum_angle_error
assert not scene.rigidbody_world, 'Replay must not run Blender physics'
report = {'claim': 'baked_display_transform_roundtrip_only', 'checked_transforms': checked,
          'frames': len(data['frames']), 'max_position_error_m': maximum_position_error,
          'max_rotation_error_rad': maximum_angle_error, 'passed': True,
          'physics_recomputed': False}
(root / 'blender-verification.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
print(json.dumps(report))
