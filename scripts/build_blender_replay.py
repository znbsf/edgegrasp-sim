"""Run with Blender --background --factory-startup --python ... -- INPUT_DIR.

Creates a baked, self-contained .blend; opening it needs no Python auto-run.
"""

import argparse
import hashlib
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import bpy
from mathutils import Euler, Matrix, Vector


def material(name, rgba):
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = rgba
    return mat


def visible(obj, show, frame):
    obj.hide_render = obj.hide_viewport = not show
    obj.keyframe_insert('hide_render', frame=frame)
    obj.keyframe_insert('hide_viewport', frame=frame)


def apply_pose(obj, value, frame=None):
    obj.location = value[:3]
    obj.rotation_mode = 'QUATERNION'
    obj.rotation_quaternion = value[3:]
    if frame is not None:
        obj.keyframe_insert('location', frame=frame)
        obj.keyframe_insert('rotation_quaternion', frame=frame)


def origin(obj, element):
    if element is not None:
        obj.location = [float(x) for x in element.get('xyz', '0 0 0').split()]
        obj.rotation_euler = Euler([float(x) for x in element.get('rpy', '0 0 0').split()], 'XYZ')


def cube(name, dimensions, mat):
    bpy.ops.mesh.primitive_cube_add(size=1)
    obj = bpy.context.object
    obj.name = name
    obj.scale = dimensions
    obj.data.materials.append(mat)
    return obj


def wire_box(name, dimensions, mat, thickness=0.0007):
    curve = bpy.data.curves.new(name, 'CURVE')
    curve.dimensions = '3D'
    curve.bevel_depth = thickness
    curve.bevel_resolution = 0
    vertices = [Vector((x*dimensions[0]/2, y*dimensions[1]/2, z*dimensions[2]/2))
                for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
    for i in range(8):
        for bit in (1, 2, 4):
            j = i ^ bit
            if j <= i:
                continue
            line = curve.splines.new('POLY')
            line.points.add(1)
            for point, vertex in zip(line.points, (vertices[i], vertices[j])):
                point.co = (*vertex, 1)
    obj = bpy.data.objects.new(name, curve)
    bpy.context.collection.objects.link(obj)
    curve.materials.append(mat)
    return obj


def text(name, body, camera, location, size, mat):
    font = bpy.data.curves.new(name, 'FONT')
    font.body, font.size = body, size
    font.space_line = 1.25
    obj = bpy.data.objects.new(name, font)
    bpy.context.collection.objects.link(obj)
    obj.parent = camera
    obj.location = location
    font.materials.append(mat)
    return obj


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--render-previews', action='store_true')
    args = parser.parse_args(sys.argv[sys.argv.index('--') + 1:])
    root = args.input.resolve()
    output = root / 'replay.blend'
    if output.exists():
        raise FileExistsError(output)
    data = json.loads((root / 'replay.json').read_text(encoding='utf-8'))
    if data['schema_version'] != 1 or data['claim'] != 'recorded_simulation_visual_replay_only':
        raise ValueError('Unexpected replay contract')
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.unit_settings.system = 'METRIC'
    scene.render.engine = 'BLENDER_WORKBENCH'
    scene.render.resolution_x, scene.render.resolution_y = 1280, 720
    scene.render.resolution_percentage = 100
    scene.render.fps = data['fps']
    scene.frame_start, scene.frame_end = 1, len(data['frames'])
    scene.world = bpy.data.worlds.new('Replay world')
    shade = scene.display.shading
    shade.light, shade.color_type = 'STUDIO', 'MATERIAL'
    shade.show_shadows = False  # Camera-attached labels must not cast shadows onto the table.
    shade.show_cavity = True
    shade.cavity_type = 'BOTH'
    shade.background_type = 'WORLD'
    scene.world.color = (0.025, 0.032, 0.048)
    white = material('Labels', (0.84, 0.91, 1, 1))
    yellow = material('Robot printed parts', (0.91, 0.63, 0.08, 1))
    dark = material('Servos', (0.065, 0.085, 0.12, 1))
    red = material('Recorded cube truth', (0.83, 0.07, 0.045, 1))
    cyan = material('RGB-D estimate', (0.03, 0.88, 1, 1))
    green = material('Contact observed', (0.18, 1, 0.28, 1))
    table_mat = material('Table', (0.25, 0.29, 0.35, 1))
    frames = {}
    for name in sorted(set(data['parents']) | set(data['parents'].values())):
        obj = bpy.data.objects.new('TF/' + name, None)
        bpy.context.collection.objects.link(obj)
        obj.empty_display_size = 0.015
        frames[name] = obj
    for child, parent in data['parents'].items():
        frames[child].parent = frames[parent]
    for name, value in data['static'].items():
        apply_pose(frames[name], value)
    robot_objects = []
    robot = ET.parse(root / 'robot.urdf')
    for link in robot.findall('link'):
        parent = frames[link.get('name')]
        for i, visual in enumerate(link.findall('visual')):
            geo = visual.find('geometry')
            mesh = geo.find('mesh')
            if mesh is not None:
                source = (root / mesh.get('filename')).resolve()
                if not source.is_relative_to(root):
                    raise ValueError('Asset escapes replay directory')
                if source.suffix.lower() != '.stl':
                    raise ValueError('Unsupported mesh format: ' + str(source))
                bpy.ops.wm.stl_import(filepath=str(source))
                obj = bpy.context.object
                obj.scale = [float(x) for x in mesh.get('scale', '1 1 1').split()]
            elif geo.find('box') is not None:
                obj = cube('box', [float(x) for x in geo.find('box').get('size').split()], yellow)
            elif geo.find('cylinder') is not None:
                cylinder = geo.find('cylinder')
                bpy.ops.mesh.primitive_cylinder_add(radius=float(cylinder.get('radius')),
                                                    depth=float(cylinder.get('length')))
                obj = bpy.context.object
            else:
                raise ValueError('Unsupported URDF visual geometry')
            obj.name = link.get('name') + '/visual/' + str(i)
            obj.parent = parent
            origin(obj, visual.find('origin'))
            obj.data.materials.clear()
            mat_name = visual.find('material')
            obj.data.materials.append(dark if mat_name is not None and mat_name.get('name') == 'sts3215' else yellow)
            robot_objects.append(obj)
    pads = {}
    for side in ('fixed', 'moving'):
        name = 'edgegrasp_' + side + '_finger_pad_link'
        element = robot.find("link[@name='" + name + "']/collision")
        box = element.find('geometry/box')
        obj = wire_box(side + ' pad contact', [float(x) for x in box.get('size').split()], green)
        obj.parent = frames[name]
        origin(obj, element.find('origin'))
        pads[side] = obj
    world = ET.parse(root / 'world.sdf')
    table = world.find(".//model[@name='table']")
    table_size = [float(x) for x in table.find('link/collision/geometry/box/size').text.split()]
    table_obj = cube('Table / source SDF', table_size, table_mat)
    table_pose = [float(x) for x in table.find('pose').text.split()]
    table_obj.location, table_obj.rotation_euler = table_pose[:3], table_pose[3:]
    cube_size = [float(x) for x in world.find(".//model[@name='target_cube']/link/collision/geometry/box/size").text.split()]
    truth = cube('Cube / recorded Gazebo pose', cube_size, red)
    observed = wire_box('Cube / measured RGB-D (latest <=100ms)', cube_size, cyan)
    observed.parent = frames['base_link']
    # A genuine recorded path, not a candidate/planned path.
    path = bpy.data.curves.new('Recorded cube centre path', 'CURVE')
    path.dimensions, path.bevel_depth = '3D', 0.00035
    line = path.splines.new('POLY')
    valid = [row for row in data['frames'] if row['cube']]
    line.points.add(len(valid)-1)
    for point, row in zip(line.points, valid):
        point.co = (*row['cube'][0][:3], 1)
    path.materials.append(cyan)
    path_obj = bpy.data.objects.new('Recorded cube centre trail', path)
    bpy.context.collection.objects.link(path_obj)
    path_obj.hide_render = True  # Optional inspection aid, hidden by default.
    path_obj.hide_viewport = True
    for row in data['frames']:
        f = row['frame']
        for name, value in row['tf'].items():
            if value:
                apply_pose(frames[name], value[0], f)
        for obj in robot_objects:
            visible(obj, row['complete'], f)
        visible(truth, row['cube'] is not None, f)
        if row['cube']:
            apply_pose(truth, row['cube'][0], f)
        obs = row['observation']
        visible(observed, obs is not None, f)
        if obs:
            value = obs[1]
            q = Matrix(value['orientation_rows']).to_quaternion()
            apply_pose(observed, value['center_m'] + list(q), f)
        for side, obj in pads.items():
            visible(obj, row['complete'] and row['contact'] is not None and row['contact'][1][side], f)
    camera_data = bpy.data.cameras.new('Overview')
    camera = bpy.data.objects.new('Overview', camera_data)
    bpy.context.collection.objects.link(camera)
    camera.location = (0.94, -1.14, 0.84)
    aim = Vector((0.22, 0.05, 0.20))
    camera.rotation_euler = (aim - camera.location).to_track_quat('-Z', 'Y').to_euler()
    camera.data.type, camera.data.ortho_scale = 'ORTHO', 1.25
    camera.data.lens = 42
    scene.camera = camera
    text('Title', 'EDGEGRASP  /  RECORDED SIMULATION', camera, (-0.59, 0.31, -0.65), 0.021, white)
    text('Legend', 'RED: recorded cube  |  CYAN: RGB-D estimate  |  GREEN: contact proxy sample',
         camera, (-0.59, 0.279, -0.65), 0.012, white)
    text('Boundary', 'Baked replay. No physics rerun. No hardware evidence.', camera,
         (-0.59, -0.318, -0.65), 0.012, white)
    missing = text('RGB-D unavailable', 'RGB-D POSE UNAVAILABLE / STALE', camera,
                   (-0.59, -0.256, -0.65), 0.014, cyan)
    for row in data['frames']:
        visible(missing, row['observation'] is None, row['frame'])
    result = data.get('recorded_result') or {}
    status = 'VERIFIED' if result.get('physics_grasp_verified') else 'NOT VERIFIED'
    text('Recorded outcome', 'RECORDED OUTCOME: ' + status + '\nTyped release exit: ' +
         str(data['typed_release_exit_code']) + ' (see source log)', camera,
         (0.245, -0.245, -0.65), 0.013, white)
    start = data['source_interval_ns'][0]
    phase_by_frame = {}
    reason_by_frame = {}
    for event in data['events']:
        f = max(1, min(scene.frame_end, math.ceil((int(event['time_ns']) - start) / 1e9 * data['fps']) + 1))
        phase_by_frame[f] = event.get('phase', '')
        reason_by_frame[f] = event.get('reason', '')
    phase_keys = sorted(phase_by_frame)
    for error in data.get('spatial_failures', []):
        f = round((error['source_ns'] - start) / 1e9 * data['fps']) + 1
        if 1 <= f <= scene.frame_end:
            scene.timeline_markers.new('NEAREST TRUTH >0.25mm: %.3fmm' % (error['error_m']*1000), frame=f)
    for f in data['gap_frames']:
        scene.timeline_markers.new('MISSING DATA', frame=f)
    for i, f in enumerate(phase_keys):
        phase = phase_by_frame[f]
        scene.timeline_markers.new(phase, frame=f)
        obj = text('Stage/' + phase, phase + '\n' + reason_by_frame[f][:80], camera,
                   (-0.59, 0.248, -0.65), 0.015, cyan)
        visible(obj, False, 1)
        visible(obj, True, f)
        if i + 1 < len(phase_keys):
            visible(obj, False, phase_keys[i+1])
    # Baked per-frame clock labels; no embedded scripts or frame-change handlers.
    for index in range(len(data['frames'])):
        f = index + 1
        obj = text('Clock/' + str(f), 'SOURCE TIME  %.3f s' % (data['frames'][index]['source_ns']/1e9),
                   camera, (-0.59, -0.286, -0.65), 0.014, white)
        visible(obj, False, 1)
        visible(obj, True, f)
        if f < scene.frame_end:
            visible(obj, False, f + 1)
    for obj in bpy.data.objects:
        if obj.animation_data and obj.animation_data.action:
            for curve in obj.animation_data.action.fcurves:
                for key in curve.keyframe_points:
                    key.interpolation = 'CONSTANT' if 'hide_' in curve.data_path else 'LINEAR'
    close_data = bpy.data.cameras.new('Gripper close-up')
    close = bpy.data.objects.new('Gripper close-up', close_data)
    bpy.context.collection.objects.link(close)
    close.location = (0.48, -0.19, 0.40)
    close.rotation_euler = (Vector((0.24, 0.13, 0.23))-close.location).to_track_quat('-Z', 'Y').to_euler()
    close.data.type, close.data.ortho_scale = 'ORTHO', 0.24
    scene['claim'] = data['claim']
    scene['source_run'] = data['run']
    scene['gap_frames'] = json.dumps(data['gap_frames'])
    scene['recorded_physics_result'] = status
    scene['typed_release_exit_code'] = str(data['typed_release_exit_code'])
    readme = bpy.data.texts.new('READ ME - replay evidence')
    readme.write('Space: play/pause. Timeline markers: recorded phases. Numpad 0: camera.\n'
                 'Solid shading is sufficient. The Gripper close-up camera is available in the Outliner.\n'
                 'Animation is baked: do not enable script auto-run.\n'
                 'TF and truth are interpolated only for display; exact source brackets are in replay.json.\n'
                 'Green pad outlines use latest recorded contact <=20ms, not continuous contact proof.\n'
                 'Recorded outcome is a whole-run result, not the verdict at the current frame.\n'
                 'Normal typed release failed in the successful grasp recordings.\n')
    for screen in bpy.data.screens:
        for area in screen.areas:
            if area.type == 'VIEW_3D':
                area.spaces.active.region_3d.view_perspective = 'CAMERA'
                area.spaces.active.shading.color_type = 'MATERIAL'
                area.spaces.active.clip_start = 0.001
    preview_source = result.get('sequence_completed_source_timestamp_ns') or (
        int(data['events'][-1]['time_ns']) if data['events'] else start)
    preview_frame = min(scene.frame_end, max(1, math.ceil((preview_source-start)/1e9*data['fps'])+1))
    scene.frame_set(preview_frame)
    bpy.ops.wm.save_as_mainfile(filepath=str(output))
    audit = {'blend': str(output), 'frames': scene.frame_end, 'objects': len(bpy.data.objects),
             'preview_frame': preview_frame, 'gap_frames': data['gap_frames'], 'render_engine': scene.render.engine,
             'builder_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
             'replay_json_sha256': hashlib.sha256((root / 'replay.json').read_bytes()).hexdigest()}
    (root / 'blender-build.json').write_text(json.dumps(audit, indent=2), encoding='utf-8')
    if args.render_previews:
        scene.render.image_settings.file_format = 'PNG'
        scene.render.filepath = str(root / 'overview.png')
        bpy.ops.render.render(write_still=True)
        scene.camera = close
        scene.render.filepath = str(root / 'gripper.png')
        bpy.ops.render.render(write_still=True)
    print(json.dumps(audit))


if __name__ == '__main__':
    main()
