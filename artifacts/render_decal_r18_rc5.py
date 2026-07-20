"""R1.8/RF28 field render for the user's wall-with-openings.

Reads E:\\testscene.blend in background mode and never saves it.  The same
script is run against the pre-R1.8 and current source roots, which guarantees
an identical camera and selection for the visual before/after gate.
"""

from __future__ import annotations

import os
import sys
from dataclasses import replace
from pathlib import Path

import bpy
from mathutils import Vector


SCRIPT_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(os.environ.get("CFTUV_SOURCE_ROOT", SCRIPT_ROOT)).resolve()
ARTIFACT_ROOT = SCRIPT_ROOT / "artifacts"
OBJECT_NAME = os.environ.get("CFTUV_R18_OBJECT", "walls.001")
WIDTH = float(os.environ.get("CFTUV_R18_WIDTH", "3.2"))
JOIN_MODE = os.environ.get("CFTUV_R18_JOIN_MODE", "BEVEL").upper()
OUTPUT_PNG = Path(
    os.environ.get(
        "CFTUV_R18_OUTPUT",
        str(ARTIFACT_ROOT / "decal_r18_rc5.png"),
    )
)
for path in (REPO_ROOT, SCRIPT_ROOT / "artifacts"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

loaded_cftuv = sys.modules.get("cftuv")
if loaded_cftuv is not None:
    loaded_path = Path(getattr(loaded_cftuv, "__file__", "")).resolve()
    if REPO_ROOT not in loaded_path.parents:
        for module_name in tuple(sys.modules):
            if module_name == "cftuv" or module_name.startswith("cftuv."):
                del sys.modules[module_name]

from cftuv.decals import (  # noqa: E402
    compile_manual_seam_decal_plan,
    evaluate_manual_seam_faces,
)
from cftuv.model import DecalSettings  # noqa: E402

import render_decal_r14_fix_terminal_wireframe as wire_base  # noqa: E402
import render_decal_r14_terminal_bridge as render_base  # noqa: E402
import verify_decal_r13_rr9a as field  # noqa: E402


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

    obj = bpy.data.objects[OBJECT_NAME]
    selected = tuple(
        int(edge.index)
        for edge in obj.data.edges
        if edge.select and edge.use_seam
    )
    graph, selected, _all_seams = field._source_graph(obj, selected)
    base = DecalSettings(width_seam=0.8, offset=0.01)
    settings = replace(
        base,
        width_seam=WIDTH,
        corner_join_mode=JOIN_MODE,
    )
    plan = compile_manual_seam_decal_plan(
        graph, base, selected, alpha_budget=max(4.0, WIDTH)
    )
    faces = evaluate_manual_seam_faces(
        obj, settings, plan, preview=True
    ).faces

    materials = {
        "source": render_base._material(
            "R18_source", (0.075, 0.095, 0.14, 1.0)
        ),
        "fill": render_base._material(
            "R18_fill", (0.035, 0.31, 1.0, 0.24)
        ),
        "wire": render_base._material(
            "R18_wire", (0.05, 0.86, 1.0, 1.0), 1.6
        ),
        "bevel_fill": render_base._material(
            "R18_bevel_fill", (1.0, 0.035, 0.24, 0.5)
        ),
        "bevel_wire": render_base._material(
            "R18_bevel_wire", (1.0, 0.12, 0.34, 1.0), 2.1
        ),
        "selected": render_base._material(
            "R18_selected", (1.0, 0.47, 0.015, 1.0), 1.35
        ),
        "label": render_base._material(
            "R18_label", (0.98, 0.99, 1.0, 1.0), 2.0
        ),
    }

    world_points = tuple(obj.matrix_world @ vertex.co for vertex in obj.data.vertices)
    center = sum(world_points, Vector()) / len(world_points)
    span = max((point - center).length for point in world_points)
    view = Vector((1.0, -0.18, 0.08)).normalized()
    screen_right = Vector((0.0, 1.0, 0.0))
    screen_up = view.cross(screen_right).normalized()
    radius = max(span * 0.0012, 0.002)

    camera_data = bpy.data.cameras.new("R18_CAMERA")
    camera = bpy.data.objects.new("R18_CAMERA", camera_data)
    scene.collection.objects.link(camera)
    camera.location = view * span * 5.5
    render_base._look_at(camera, Vector())
    camera.data.type = "ORTHO"
    camera.data.ortho_scale = span * 1.45
    scene.camera = camera

    vertex_ids = set(range(len(obj.data.vertices)))
    wire_base._source_closeup(
        "R18_SOURCE", obj, vertex_ids, center, Vector(), materials["source"]
    )
    regular_faces = tuple(
        face for face in faces if str(face.component_kind) != "BEVEL"
    )
    bevel_faces = tuple(
        face for face in faces if str(face.component_kind) == "BEVEL"
    )
    wire_base._wire_overlay(
        "R18_DECAL",
        regular_faces,
        obj,
        center,
        Vector(),
        materials["fill"],
        materials["wire"],
        radius,
    )
    if bevel_faces:
        wire_base._wire_overlay(
            "R18_BEVEL",
            bevel_faces,
            obj,
            center,
            Vector(),
            materials["bevel_fill"],
            materials["bevel_wire"],
            radius * 1.15,
        )
    for edge_id in selected:
        edge = obj.data.edges[edge_id]
        point_a = obj.matrix_world @ obj.data.vertices[edge.vertices[0]].co
        point_b = obj.matrix_world @ obj.data.vertices[edge.vertices[1]].co
        render_base._line(
            f"R18_SELECTED_{edge_id}",
            point_a - center + view * span * 0.012,
            point_b - center + view * span * 0.012,
            materials["selected"],
            radius * 1.15,
        )

    render_base._label(
        f"R1.8 RC5 | {OBJECT_NAME} | {JOIN_MODE} | width {WIDTH:.1f}\n"
        "orange = selected seams | cyan = strip | MAGENTA = emitted BEVEL",
        camera,
        screen_up * span * 0.58 + view * span * 0.035,
        span * 0.20,
        materials["label"],
    )

    light_data = bpy.data.lights.new("R18_KEY", "AREA")
    light_data.energy = 1400.0
    light_data.size = span * 5.0
    light = bpy.data.objects.new("R18_KEY", light_data)
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
    print("R18_RENDER", OUTPUT_PNG)
    print("R18_SOURCE_ROOT", REPO_ROOT)
    print("R18_SELECTED_COUNT", len(selected))
    print(
        "R18_KINDS",
        {
            kind: sum(
                1 for face in faces if str(face.component_kind) == kind
            )
            for kind in sorted({str(face.component_kind) for face in faces})
        },
    )

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
