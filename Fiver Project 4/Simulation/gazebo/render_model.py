"""Blender script: turntable video of a textured OBJ (the WebODM 3D model).

    blender -b --factory-startup -P render_model.py -- model.obj out.mp4 [seconds]
"""
import math
import sys

import bpy
from mathutils import Vector

argv = sys.argv[sys.argv.index("--") + 1:]
obj_path, out_path = argv[0], argv[1]
seconds = float(argv[2]) if len(argv) > 2 else 12.0
fps = 30

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
bpy.ops.wm.obj_import(filepath=obj_path, forward_axis="Y", up_axis="Z")
meshes = [o for o in scene.objects if o.type == "MESH"]

pts = [o.matrix_world @ Vector(c) for o in meshes for c in o.bound_box]
lo = Vector((min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)))
hi = Vector((max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)))
center = (lo + hi) / 2
size = max(hi.x - lo.x, hi.y - lo.y)
for o in meshes:
    o.location -= Vector((center.x, center.y, lo.z))

target = bpy.data.objects.new("target", None)
scene.collection.objects.link(target)
target.location = (0, 0, (hi.z - lo.z) * 0.15)

cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
cam.data.lens = 35
cam.data.clip_end = size * 10
scene.collection.objects.link(cam)
scene.camera = cam
track = cam.constraints.new("TRACK_TO")
track.target = target
track.track_axis = "TRACK_NEGATIVE_Z"
track.up_axis = "UP_Y"

frames = int(seconds * fps)
radius, height = size * 0.75, size * 0.5
for f in range(frames):
    a = 2 * math.pi * f / frames + math.radians(225)
    cam.location = (radius * math.cos(a), radius * math.sin(a), height)
    cam.keyframe_insert("location", frame=f + 1)

scene.frame_start, scene.frame_end = 1, frames
scene.render.fps = fps
scene.render.resolution_x, scene.render.resolution_y = 1920, 1080
scene.render.engine = "BLENDER_WORKBENCH"
scene.display.shading.light = "FLAT"
scene.display.shading.color_type = "TEXTURE"
scene.world = bpy.data.worlds.new("w")
scene.world.color = (0.105, 0.12, 0.14)
scene.render.image_settings.file_format = "FFMPEG"
scene.render.ffmpeg.format = "MPEG4"
scene.render.ffmpeg.codec = "H264"
scene.render.ffmpeg.constant_rate_factor = "HIGH"
scene.render.filepath = out_path
bpy.ops.render.render(animation=True)
