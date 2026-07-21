"""R2.1/RF29b: read-only wireframe at width=7.8 on sagging_wall."""

from __future__ import annotations

from hashlib import sha256
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
        "CFTUV_R21_OUTPUT",
        SCRIPT_ROOT / "artifacts" / "decal_r21_rf29b.png",
    )
)
REPORT_LABEL = os.environ.get("CFTUV_R21_LABEL", "R2.1 RF29b")
for path in (SCRIPT_ROOT, SCRIPT_ROOT / "artifacts", REPO_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

# Эти модули содержат только Blender render helpers. После их загрузки core
# принудительно перечитывается из проверяемого source root.
import render_decal_r14_fix_terminal_wireframe as wire_base  # noqa: E402
import render_decal_r14_terminal_bridge as render_base  # noqa: E402

for module_name in tuple(sys.modules):
    if module_name == "cftuv" or module_name.startswith("cftuv."):
        del sys.modules[module_name]
sys.path.insert(0, str(REPO_ROOT))

from cftuv.analysis import build_analysis_bundle  # noqa: E402
from cftuv.decals import (  # noqa: E402
    compile_manual_seam_decal_plan,
    evaluate_manual_seam_faces,
)
from cftuv.model import DecalSettings  # noqa: E402


def _source_graph(obj, selected):
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
    original_names = {item.name for item in bpy.data.objects}
    original_hidden = {
        item.name: item.hide_render for item in bpy.data.objects
    }
    for item in bpy.data.objects:
        item.hide_render = True

    obj = bpy.data.objects["sagging_wall"]
    selected = (6, 7, 8)
    bundle = _source_graph(obj, selected)
    settings = DecalSettings(width_seam=7.8, offset=0.01)
    faces = ()
    policy_counts = ()
    try:
        plan = compile_manual_seam_decal_plan(
            bundle, settings, selected, alpha_budget=36.0
        )
        result = evaluate_manual_seam_faces(
            obj, settings, plan, preview=False
        )
        faces = tuple(result.faces)
        policy_counts = tuple(result.policy_counts)
        status = "OK"
    except Exception as exc:
        status = repr(exc)

    materials = {
        "source": render_base._material(
            "R21_source", (0.075, 0.095, 0.14, 1.0)
        ),
        "fill": render_base._material(
            "R21_fill", (0.035, 0.31, 1.0, 0.35)
        ),
        "wire": render_base._material(
            "R21_wire", (0.05, 0.86, 1.0, 1.0), 1.7
        ),
        "selected": render_base._material(
            "R21_selected", (1.0, 0.47, 0.015, 1.0), 1.4
        ),
        "ok": render_base._material(
            "R21_ok", (0.35, 1.0, 0.42, 1.0), 2.0
        ),
        "error": render_base._material(
            "R21_error", (1.0, 0.12, 0.08, 1.0), 2.0
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

    camera_data = bpy.data.cameras.new("R21_CAMERA")
    camera = bpy.data.objects.new("R21_CAMERA", camera_data)
    scene.collection.objects.link(camera)
    camera.location = view * span * 5.5
    render_base._look_at(camera, Vector())
    camera.data.type = "ORTHO"
    camera.data.ortho_scale = span * 1.48
    scene.camera = camera

    wire_base._source_closeup(
        "R21_SOURCE",
        obj,
        set(range(len(obj.data.vertices))),
        center,
        Vector(),
        materials["source"],
    )
    if faces:
        wire_base._wire_overlay(
            "R21_DECAL",
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
            f"R21_SELECTED_{edge_id}",
            point_a - center + view * span * 0.012,
            point_b - center + view * span * 0.012,
            materials["selected"],
            radius * 1.15,
        )

    if faces:
        policies = dict(policy_counts)
        label = (
            f"{REPORT_LABEL}: OK | width 7.8 | faces {len(faces)}\n"
            f"STATION_CLAMPED={policies.get('TERMINAL_ROUTE_STATION_CLAMPED', 0)} "
            f"SATURATED={policies.get('TERMINAL_ROUTE_SATURATED', 0)}"
        )
        label_material = materials["ok"]
    else:
        named_error = (
            "TERMINAL_BRIDGE_CUT_INVALID"
            if "TERMINAL_BRIDGE_CUT_INVALID" in status
            else status[:96]
        )
        label = f"{REPORT_LABEL}: ERROR | width 7.8\n{named_error}"
        label_material = materials["error"]
    render_base._label(
        label,
        camera,
        screen_up * span * 0.53 + view * span * 0.06,
        span * 0.42,
        label_material,
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
    print("R21_RF29B_RENDER", OUTPUT_PNG)
    print("R21_RF29B_STATUS", status)
    print("R21_RF29B_POLICIES", policy_counts)
    print("R21_BLEND_SHA256", sha256(Path(bpy.data.filepath).read_bytes()).hexdigest())

    scene.camera = original_camera
    for item in tuple(bpy.data.objects):
        if item.name not in original_names:
            bpy.data.objects.remove(item, do_unlink=True)
    for name, hidden in original_hidden.items():
        item = bpy.data.objects.get(name)
        if item is not None:
            item.hide_render = hidden


if __name__ == "__main__":
    main()
