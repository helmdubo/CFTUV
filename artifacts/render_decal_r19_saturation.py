"""R1.9 visual gate: same sagging-wall view before/after saturation.

The script is read-only and never saves the source blend.  Run it against the
pre-R1.9 source root to render the named final-evaluation error, and against
the current root to render the saturated terminal geometry.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import bpy
from mathutils import Vector


SCRIPT_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(os.environ.get("CFTUV_SOURCE_ROOT", SCRIPT_ROOT)).resolve()
OUTPUT_PNG = Path(
    os.environ.get(
        "CFTUV_R19_OUTPUT",
        str(SCRIPT_ROOT / "artifacts" / "decal_r19_saturation.png"),
    )
)
WIDTH = float(os.environ.get("CFTUV_R19_WIDTH", "102.75249109705808"))
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

    obj = bpy.data.objects["sagging_wall"]
    selected = tuple(
        int(edge.index)
        for edge in obj.data.edges
        if edge.select and edge.use_seam
    ) or (6, 7, 8)
    graph, selected, _all_seams = field._source_graph(obj, selected)
    settings = DecalSettings(width_seam=WIDTH, offset=0.01)
    status = "UPDATED"
    policies = ()
    faces = ()
    try:
        plan = compile_manual_seam_decal_plan(graph, settings, selected)
        result = evaluate_manual_seam_faces(
            obj, settings, plan, preview=False
        )
        faces = tuple(result.faces)
        policies = tuple(result.policy_counts)
    except Exception as exc:
        status = str(exc)

    materials = {
        "source": render_base._material(
            "R19_source", (0.075, 0.095, 0.14, 1.0)
        ),
        "fill": render_base._material(
            "R19_fill", (0.035, 0.31, 1.0, 0.35)
        ),
        "wire": render_base._material(
            "R19_wire", (0.05, 0.86, 1.0, 1.0), 1.7
        ),
        "selected": render_base._material(
            "R19_selected", (1.0, 0.47, 0.015, 1.0), 1.4
        ),
        "label_ok": render_base._material(
            "R19_label_ok", (0.35, 1.0, 0.42, 1.0), 2.0
        ),
        "label_error": render_base._material(
            "R19_label_error", (1.0, 0.12, 0.08, 1.0), 2.0
        ),
    }
    world_points = tuple(
        obj.matrix_world @ vertex.co for vertex in obj.data.vertices
    )
    center = sum(world_points, Vector()) / len(world_points)
    span = max((point - center).length for point in world_points)
    view = Vector((1.0, -0.20, 0.10)).normalized()
    screen_right = Vector((0.0, 1.0, 0.0))
    screen_up = view.cross(screen_right).normalized()
    radius = max(span * 0.0012, 0.002)

    camera_data = bpy.data.cameras.new("R19_CAMERA")
    camera = bpy.data.objects.new("R19_CAMERA", camera_data)
    scene.collection.objects.link(camera)
    camera.location = view * span * 5.5
    render_base._look_at(camera, Vector())
    camera.data.type = "ORTHO"
    camera.data.ortho_scale = span * 1.48
    scene.camera = camera

    wire_base._source_closeup(
        "R19_SOURCE",
        obj,
        set(range(len(obj.data.vertices))),
        center,
        Vector(),
        materials["source"],
    )
    if faces:
        wire_base._wire_overlay(
            "R19_DECAL",
            faces,
            obj,
            center,
            Vector(),
            materials["fill"],
            materials["wire"],
            radius,
        )
    for edge_id in selected:
        edge = obj.data.edges[edge_id]
        point_a = obj.matrix_world @ obj.data.vertices[edge.vertices[0]].co
        point_b = obj.matrix_world @ obj.data.vertices[edge.vertices[1]].co
        render_base._line(
            f"R19_SELECTED_{edge_id}",
            point_a - center + view * span * 0.012,
            point_b - center + view * span * 0.012,
            materials["selected"],
            radius * 1.15,
        )

    if faces:
        label = (
            f"AFTER R1.9: SATURATED OK | width {WIDTH:.3f}\n"
            "cyan=decal | orange=selected | SAT:3 CLAMP:2"
        )
        label_material = materials["label_ok"]
    else:
        label = (
            f"BEFORE R1.9: FINAL ERROR | width {WIDTH:.3f}\n"
            "TERMINAL_BRIDGE_CUT_INVALID v10/e8 | no decal"
        )
        label_material = materials["label_error"]
    render_base._label(
        label,
        camera,
        screen_up * span * 0.53 + view * span * 0.06,
        span * 0.42,
        label_material,
    )

    light_data = bpy.data.lights.new("R19_KEY", "AREA")
    light_data.energy = 1400.0
    light_data.size = span * 5.0
    light = bpy.data.objects.new("R19_KEY", light_data)
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
    print("R19_RENDER", OUTPUT_PNG)
    print("R19_SOURCE_ROOT", REPO_ROOT)
    print("R19_STATUS", status)
    print("R19_POLICY_COUNTS", policies)

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
