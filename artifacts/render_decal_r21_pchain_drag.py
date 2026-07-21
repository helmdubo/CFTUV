"""R2.1/RF31 read-only field render for the user's pChain drag case.

The same frozen ``sagging_wall`` blend, camera, selected edges and width are
used for the baseline and the candidate.  The source blend is never saved.
"""

from __future__ import annotations

from collections import Counter
import os
from pathlib import Path
import sys

import bmesh
import bpy
from mathutils import Vector


SCRIPT_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(os.environ.get("CFTUV_SOURCE_ROOT", SCRIPT_ROOT)).resolve()
OUTPUT_PNG = Path(
    os.environ.get(
        "CFTUV_R21_RENDER",
        SCRIPT_ROOT / "artifacts" / "decal_r21_pchain_drag.png",
    )
)
WIDTH = float(os.environ.get("CFTUV_R21_WIDTH", "2.4"))
LABEL = os.environ.get("CFTUV_R21_LABEL", "R2.1 candidate")
for path in (REPO_ROOT, SCRIPT_ROOT / "artifacts"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

for module_name in tuple(sys.modules):
    if module_name == "cftuv" or module_name.startswith("cftuv."):
        del sys.modules[module_name]

from cftuv.analysis import build_analysis_bundle  # noqa: E402
from cftuv.decals import (  # noqa: E402
    compile_manual_seam_decal_plan,
    evaluate_manual_seam_faces,
)
from cftuv.model import CornerJoinMode, DecalSettings  # noqa: E402

import render_decal_r14_fix_terminal_wireframe as wire_base  # noqa: E402
import render_decal_r14_terminal_bridge as render_base  # noqa: E402


def _source_bundle(obj):
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    try:
        return build_analysis_bundle(
            bm, [face.index for face in bm.faces], obj
        )
    finally:
        bm.free()


def main():
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    scene = bpy.context.scene
    original_camera = scene.camera
    original_object_names = {item.name for item in bpy.data.objects}
    original_hide_render = {
        item.name: item.hide_render for item in bpy.data.objects
    }
    for item in bpy.data.objects:
        item.hide_render = True

    obj = bpy.data.objects["sagging_wall"]
    selected = (7, 26)
    settings = DecalSettings(
        width_seam=WIDTH,
        offset=0.01,
        corner_join_mode=CornerJoinMode.MITER,
    )
    plan = compile_manual_seam_decal_plan(
        _source_bundle(obj), settings, selected, alpha_budget=36.0
    )
    result = evaluate_manual_seam_faces(
        obj, settings, plan, preview=True
    )
    faces = tuple(result.faces)
    cap_faces = tuple(
        face for face in faces if str(face.component_kind) == "CAP"
    )
    strip_faces = tuple(
        face for face in faces if str(face.component_kind) != "CAP"
    )

    materials = {
        "source": render_base._material(
            "R21_source", (0.075, 0.095, 0.14, 1.0)
        ),
        "strip_fill": render_base._material(
            "R21_strip_fill", (0.06, 0.35, 1.0, 0.42)
        ),
        "strip_wire": render_base._material(
            "R21_strip_wire", (0.08, 0.88, 1.0, 1.0), 1.7
        ),
        "cap_fill": render_base._material(
            "R21_cap_fill", (0.05, 0.95, 0.78, 0.62)
        ),
        "cap_wire": render_base._material(
            "R21_cap_wire", (0.1, 1.0, 0.62, 1.0), 2.1
        ),
        "selected": render_base._material(
            "R21_selected", (1.0, 0.42, 0.01, 1.0), 1.5
        ),
        "label": render_base._material(
            "R21_label", (0.98, 0.99, 1.0, 1.0), 2.0
        ),
    }

    world_points = tuple(
        obj.matrix_world @ vertex.co for vertex in obj.data.vertices
    )
    center = sum(world_points, Vector()) / len(world_points)
    span = max((point - center).length for point in world_points)
    view = Vector((1.0, -0.35, 0.18)).normalized()
    screen_right = Vector((0.0, 1.0, 0.0))
    screen_up = view.cross(screen_right).normalized()
    radius = max(span * 0.0012, 0.002)

    camera_data = bpy.data.cameras.new("R21_CAMERA")
    camera = bpy.data.objects.new("R21_CAMERA", camera_data)
    scene.collection.objects.link(camera)
    camera.location = view * span * 5.5
    render_base._look_at(camera, Vector())
    camera.data.type = "ORTHO"
    camera.data.ortho_scale = span * 1.45
    scene.camera = camera

    wire_base._source_closeup(
        "R21_SOURCE",
        obj,
        set(range(len(obj.data.vertices))),
        center,
        Vector(),
        materials["source"],
    )
    if strip_faces:
        wire_base._wire_overlay(
            "R21_STRIP",
            strip_faces,
            obj,
            center,
            Vector(),
            materials["strip_fill"],
            materials["strip_wire"],
            radius,
        )
    if cap_faces:
        wire_base._wire_overlay(
            "R21_CAP",
            cap_faces,
            obj,
            center,
            Vector(),
            materials["cap_fill"],
            materials["cap_wire"],
            radius * 1.15,
        )
    for edge_id in selected:
        edge = obj.data.edges[edge_id]
        point_a = obj.matrix_world @ obj.data.vertices[edge.vertices[0]].co
        point_b = obj.matrix_world @ obj.data.vertices[edge.vertices[1]].co
        render_base._line(
            f"R21_SELECTED_{edge_id}",
            point_a - center + view * span * 0.012,
            point_b - center + view * span * 0.012,
            materials["selected"],
            radius * 1.15,
        )

    kinds = Counter(str(face.component_kind) for face in faces)
    render_base._label(
        f"{LABEL} | sagging_wall | edges 7 + 26 | MITER | width {WIDTH:.1f}\n"
        f"blue = strip | turquoise = CAP | orange = selected | {dict(kinds)}",
        camera,
        screen_up * span * 0.58 + view * span * 0.035,
        span * 0.20,
        materials["label"],
    )

    light_data = bpy.data.lights.new("R21_KEY", "AREA")
    light_data.energy = 1400.0
    light_data.size = span * 5.0
    light = bpy.data.objects.new("R21_KEY", light_data)
    scene.collection.objects.link(light)
    light.location = view * span * 4.0

    scene.render.engine = "BLENDER_EEVEE_NEXT"
    scene.render.resolution_x = 1800
    scene.render.resolution_y = 1100
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.film_transparent = False
    scene.world.color = (0.006, 0.009, 0.014)
    scene.render.filepath = str(OUTPUT_PNG)
    bpy.ops.render.render(write_still=True)
    print("R21_RENDER", OUTPUT_PNG)
    print("R21_SOURCE_ROOT", REPO_ROOT)
    print("R21_KINDS", dict(kinds))

    scene.camera = original_camera
    for item in tuple(bpy.data.objects):
        if item.name not in original_object_names:
            bpy.data.objects.remove(item, do_unlink=True)
    for name, hidden in original_hide_render.items():
        item = bpy.data.objects.get(name)
        if item is not None:
            item.hide_render = hidden


if __name__ == "__main__":
    main()
