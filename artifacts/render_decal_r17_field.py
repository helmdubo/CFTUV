"""R1.7: подписанный field-render RM10 и BEVEL на sagging_wall.

Скрипт читает E:\\testscene.blend headless и никогда не сохраняет сцену.
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


OBJECT_NAME = "sagging_wall"
SELECTED = (6, 7, 8)
TARGET_VERTICES = (7, 8, 9, 10)
WIDTH = float(os.environ.get("CFTUV_R17_WIDTH", "3.2"))
JOIN_MODE = os.environ.get("CFTUV_R17_JOIN_MODE", "BEVEL").upper()
OUTPUT_PNG = Path(
    os.environ.get(
        "CFTUV_R17_OUTPUT",
        str(ARTIFACT_ROOT / "decal_r17_field_wireframe.png"),
    )
)


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
    graph, selected, _all_seams = field._source_graph(obj, SELECTED)
    base_settings = DecalSettings(width_seam=0.8, offset=0.01)
    settings = replace(
        base_settings, width_seam=WIDTH, corner_join_mode=JOIN_MODE
    )
    plan = compile_manual_seam_decal_plan(
        graph, base_settings, selected, alpha_budget=4.0
    )
    faces = evaluate_manual_seam_faces(
        obj, settings, plan, preview=True
    ).faces

    materials = {
        "source": render_base._material(
            "R17_source", (0.08, 0.105, 0.15, 1.0)
        ),
        "fill": render_base._material(
            "R17_fill", (0.04, 0.34, 1.0, 0.22)
        ),
        "wire": render_base._material(
            "R17_wire", (0.08, 0.84, 1.0, 1.0), 1.7
        ),
        "bevel_fill": render_base._material(
            "R17_bevel_fill", (1.0, 0.04, 0.28, 0.42)
        ),
        "bevel_wire": render_base._material(
            "R17_bevel_wire", (1.0, 0.12, 0.38, 1.0), 2.0
        ),
        "selected": render_base._material(
            "R17_selected", (1.0, 0.43, 0.02, 1.0), 1.4
        ),
        "vertex": render_base._material(
            "R17_vertex", (1.0, 0.96, 0.02, 1.0), 1.8
        ),
        "route": render_base._material(
            "R17_route", (1.0, 0.92, 0.02, 1.0), 2.2
        ),
        "label": render_base._material(
            "R17_label", (0.97, 0.98, 1.0, 1.0), 2.0
        ),
    }

    target_points = tuple(
        obj.matrix_world @ obj.data.vertices[index].co
        for index in TARGET_VERTICES
    )
    center = sum(target_points, Vector()) / len(target_points)
    neighborhood = set()
    for vertex_id in TARGET_VERTICES:
        neighborhood.update(
            wire_base._vertex_neighborhood(obj, vertex_id, depth=3)
        )
    span = max(
        (
            (obj.matrix_world @ obj.data.vertices[index].co - center).length
            for index in neighborhood
        ),
        default=1.0,
    )
    span = max(span, 1.0)
    view = Vector((1.15, -1.65, 0.95)).normalized()
    screen_right = Vector((-view.y, view.x, 0.0)).normalized()
    screen_up = view.cross(screen_right).normalized()
    radius = span * 0.0018

    camera_data = bpy.data.cameras.new("R17_CAMERA")
    camera = bpy.data.objects.new("R17_CAMERA", camera_data)
    scene.collection.objects.link(camera)
    camera.location = view * span * 6.0
    render_base._look_at(camera, Vector())
    camera.data.type = "ORTHO"
    camera.data.ortho_scale = span * 1.9
    scene.camera = camera

    wire_base._source_closeup(
        "R17_SOURCE",
        obj,
        neighborhood,
        center,
        Vector(),
        materials["source"],
    )
    close_faces = tuple(
        face
        for face in faces
        if wire_base._face_is_close(face, obj, center, span * 1.1)
    )
    regular_faces = tuple(
        face for face in close_faces if str(face.component_kind) != "BEVEL"
    )
    bevel_faces = tuple(
        face for face in close_faces if str(face.component_kind) == "BEVEL"
    )
    wire_base._wire_overlay(
        "R17_DECAL",
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
            "R17_BEVEL",
            bevel_faces,
            obj,
            center,
            Vector(),
            materials["bevel_fill"],
            materials["bevel_wire"],
            radius * 1.2,
        )
    for edge_id in selected:
        edge = obj.data.edges[edge_id]
        point_a = obj.matrix_world @ obj.data.vertices[edge.vertices[0]].co
        point_b = obj.matrix_world @ obj.data.vertices[edge.vertices[1]].co
        render_base._line(
            f"R17_SELECTED_{edge_id}",
            point_a - center,
            point_b - center,
            materials["selected"],
            radius * 1.2,
        )
    route_ids = {
        terminal.route_id
        for terminal in (plan.rail_plan.terminal_uses if plan.rail_plan else ())
        if terminal.spine_vertex_id == 10
        and terminal.spine_edge_id == 8
        and terminal.route_id is not None
    }
    for route in (plan.rail_plan.routes if plan.rail_plan else ()):
        if route.route_id not in route_ids:
            continue
        for segment in route.segments:
            edge = obj.data.edges[segment.edge_id]
            point_a = obj.matrix_world @ obj.data.vertices[edge.vertices[0]].co
            point_b = obj.matrix_world @ obj.data.vertices[edge.vertices[1]].co
            render_base._line(
                f"R17_ROUTE_{route.route_id}_{segment.edge_id}",
                point_a - center + view * span * 0.025,
                point_b - center + view * span * 0.025,
                materials["route"],
                radius * 1.45,
            )
    for label_index, vertex_id in enumerate(TARGET_VERTICES):
        point = obj.matrix_world @ obj.data.vertices[vertex_id].co - center
        render_base._label(
            f"v{vertex_id}",
            camera,
            point
            + screen_up * span * (0.035 + 0.025 * (label_index % 2))
            + view * span * 0.04,
            span * 0.12,
            materials["vertex"],
        )

    render_base._label(
        f"R1.7 FIELD | {JOIN_MODE} | width {WIDTH:.1f}\n"
        "orange = selected | YELLOW = terminal contour routes\n"
        "cyan = decal | MAGENTA = BEVEL face",
        camera,
        screen_up * span * 0.72 + view * span * 0.04,
        span * 0.28,
        materials["label"],
    )

    light_data = bpy.data.lights.new("R17_KEY", "AREA")
    light_data.energy = 1350.0
    light_data.size = span * 5.0
    light = bpy.data.objects.new("R17_KEY", light_data)
    scene.collection.objects.link(light)
    light.location = view * span * 4.0

    scene.render.engine = "BLENDER_EEVEE_NEXT"
    scene.render.resolution_x = 1600
    scene.render.resolution_y = 1100
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.film_transparent = False
    scene.world.color = (0.006, 0.009, 0.014)
    scene.render.filepath = str(OUTPUT_PNG)
    bpy.ops.render.render(write_still=True)
    print("R17_RENDER", OUTPUT_PNG)
    print("R17_SOURCE_ROOT", REPO_ROOT)
    print("R17_SELECTED", selected)
    print("R17_KINDS", {
        kind: sum(1 for face in faces if str(face.component_kind) == kind)
        for kind in sorted({str(face.component_kind) for face in faces})
    })

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
