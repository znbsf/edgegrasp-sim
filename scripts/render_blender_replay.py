"""Optional whole-recording MP4 export; runs in Blender without simulation."""

import sys
from pathlib import Path

import bpy

root = Path(sys.argv[sys.argv.index('--') + 1]).resolve()
output = root / 'replay.mp4'
if output.exists():
    raise FileExistsError(output)
bpy.ops.wm.open_mainfile(filepath=str(root / 'replay.blend'), use_scripts=False)
scene = bpy.context.scene
scene.render.resolution_percentage = 75
scene.render.image_settings.file_format = 'FFMPEG'
scene.render.ffmpeg.format = 'MPEG4'
scene.render.ffmpeg.codec = 'H264'
scene.render.ffmpeg.constant_rate_factor = 'MEDIUM'
scene.render.filepath = str(output)
bpy.ops.render.render(animation=True)
