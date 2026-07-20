"""RF20 field: wireframe close-up after rejecting a pChain continuation."""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

import bpy
from mathutils import Vector


REPO_ROOT = Path(__file__).resolve().parents[1]
ARTIFACT_ROOT = REPO_ROOT / "artifacts"
for path in (REPO_ROOT, ARTIFACT_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import render_decal_r14_fix_terminal_wireframe as wire_base  # noqa: E402
import render_decal_r14_terminal_bridge as render_base  # noqa: E402
import verify_decal_r13_rr9a as field  # noqa: E402

from cftuv.decals import (  # noqa: E402
    compile_manual_seam_decal_plan,
    evaluate_manual_seam_faces,
)
from cftuv.model import DecalSettings  # noqa: E402


OBJECT_NAME = "sagging_wall"
TARGET_VERTEX = 4
OLD_CONTINUATION_EDGE = 3
NEW_FOLD_EDGE = 22
OUTPUT_PNG = ARTIFACT_ROOT / "decal_r14_fix2_beak_after_wireframe.png"


def _edge_arrow(name, obj, edge_id, vertex_id, center, offset, extent, material, radius):
    edge = obj.data.edges[edge_id]
    other_id = next(vertex for vertex in edge.vertices if vertex != vertex_id)
    origin = obj.matrix_world @ obj.data.vertices[vertex_id].co
    target = obj.matrix_world @ obj.data.vertices[other_id].co
    direction = (target - origin).normalized()
    render_base._arrow(
        name,
        origin - center + offset,
        origin - center + offset + direction * extent,
        material,
        radius,
    )


def main():
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    scene = bpy.context.scene
    original_camera = scene.camera
    original_object_names = {scene_object.name for scene_object in bpy.data.objects}
    original_hide_render = {
        scene_object.name: scene_object.hide_render
        for scene_object in bpy.data.objects
    }
    for scene_object in bpy.data.objects:
        scene_object.hide_render = True

    obj = bpy.data.objects[OBJECT_NAME]
    graph, selected, _all_seams = field._source_graph(
        obj,
        tuple(
            sorted(edge.index for edge in obj.data.edges if edge.select and edge.use_seam)
        ),
    )
    base_settings = DecalSettings(width_seam=0.8, offset=0.01)
    plan = compile_manual_seam_decal_plan(graph, base_settings, selected)
    evaluated = tuple(
        (
            width,
            evaluate_manual_seam_faces(
                obj,
                replace(base_settings, width_seam=width),
                plan,
                preview=True,
            ).faces,
        )
        for width in (0.4, 0.8)
    )

    materials = {
        "source": render_base._material(
            "R14_FIX2_source", (0.10, 0.13, 0.18, 1.0)
        ),
        "narrow_fill": render_base._material(
            "R14_FIX2_narrow_fill", (0.0, 0.55, 1.0, 0.18)
        ),
        "narrow_wire": render_base._material(
            "R14_FIX2_narrow_wire", (0.05, 0.88, 1.0, 1.0), 1.5
        ),
        "wide_fill": render_base._material(
            "R14_FIX2_wide_fill", (0.10, 0.24, 1.0, 0.18)
        ),
        "wide_wire": render_base._material(
            "R14_FIX2_wide_wire", (0.30, 0.52, 1.0, 1.0), 1.5
        ),
        "selected": render_base._material(
            "R14_FIX2_selected", (1.0, 0.48, 0.02, 1.0), 1.2
        ),
        "old": render_base._material(
            "R14_FIX2_old", (1.0, 0.04, 0.02, 1.0), 1.8
        ),
        "guide": render_base._material(
            "R14_FIX2_guide", (0.10, 1.0, 0.18, 1.0), 1.8
        ),
        "label": render_base._material(
            "R14_FIX2_label", (0.96, 0.98, 1.0, 1.0), 2.0
        ),
    }

    center = obj.matrix_world @ obj.data.vertices[TARGET_VERTEX].co
    neighborhood = wire_base._vertex_neighborhood(obj, TARGET_VERTEX, depth=3)
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
    offsets = (-0.68, 0.68)
    radius = span * 0.0022

    camera_data = bpy.data.cameras.new("R14_FIX2_CAMERA")
    camera = bpy.data.objects.new("R14_FIX2_CAMERA", camera_data)
    bpy.context.scene.collection.objects.link(camera)
    camera.location = view * span * 5.5
    render_base._look_at(camera, Vector())
    camera.data.type = "ORTHO"
    camera.data.ortho_scale = span * 2.25
    bpy.context.scene.camera = camera

    for panel_index, (width, faces) in enumerate(evaluated):
        offset = screen_right * span * offsets[panel_index]
        wire_base._source_closeup(
            f"R14_FIX2_SOURCE_{panel_index}",
            obj,
            neighborhood,
            center,
            offset,
            materials["source"],
        )
        close_faces = tuple(
            face
            for face in faces
            if wire_base._face_is_close(face, obj, center, span * 0.82)
        )
        wire_base._wire_overlay(
            f"R14_FIX2_DECAL_{panel_index}",
            close_faces,
            obj,
            center,
            offset,
            materials["narrow_fill" if panel_index == 0 else "wide_fill"],
            materials["narrow_wire" if panel_index == 0 else "wide_wire"],
            radius,
        )
        for edge_id in selected:
            edge = obj.data.edges[edge_id]
            if not set(edge.vertices).intersection(neighborhood):
                continue
            point_a = obj.matrix_world @ obj.data.vertices[edge.vertices[0]].co
            point_b = obj.matrix_world @ obj.data.vertices[edge.vertices[1]].co
            render_base._line(
                f"R14_FIX2_SELECTED_{panel_index}_{edge_id}",
                point_a - center + offset,
                point_b - center + offset,
                materials["selected"],
                radius * 1.1,
            )
        _edge_arrow(
            f"R14_FIX2_OLD_{panel_index}",
            obj,
            OLD_CONTINUATION_EDGE,
            TARGET_VERTEX,
            center,
            offset,
            width * 0.5,
            materials["old"],
            radius * 1.3,
        )
        _edge_arrow(
            f"R14_FIX2_NEW_{panel_index}",
            obj,
            NEW_FOLD_EDGE,
            TARGET_VERTEX,
            center,
            offset,
            width * 0.5,
            materials["guide"],
            radius * 1.3,
        )
        render_base._label(
            f"AFTER RR9b | width {width:.1f}\nPERP + FOLD e22",
            camera,
            offset + screen_up * span * 0.34 + view * span * 0.04,
            span * 0.56,
            materials["label"],
        )

    render_base._label(
        "RED e3 = rejected continuation | GREEN e22 = side guide\n"
        "ORANGE = selected spine | BLUE wire = emitted decal faces",
        camera,
        screen_up * span * 0.52 + view * span * 0.04,
        span * 0.34,
        materials["label"],
    )

    light_data = bpy.data.lights.new("R14_FIX2_KEY", "AREA")
    light_data.energy = 1250.0
    light_data.size = span * 5.0
    light = bpy.data.objects.new("R14_FIX2_KEY", light_data)
    bpy.context.scene.collection.objects.link(light)
    light.location = view * span * 4.0

    scene.render.engine = "BLENDER_EEVEE_NEXT"
    scene.render.resolution_x = 1920
    scene.render.resolution_y = 1080
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.film_transparent = False
    scene.world.color = (0.008, 0.011, 0.018)
    scene.render.filepath = str(OUTPUT_PNG)
    bpy.ops.render.render(write_still=True)
    print("R14_FIX2_RENDER", OUTPUT_PNG)
    print("R14_FIX2_SELECTED", selected)
    for line in plan.terminal_routing_report:
        print("R14_FIX2_TERMINAL", line)
    scene.camera = original_camera
    for scene_object in tuple(bpy.data.objects):
        if scene_object.name not in original_object_names:
            bpy.data.objects.remove(scene_object, do_unlink=True)
    for object_name, hidden in original_hide_render.items():
        original = bpy.data.objects.get(object_name)
        if original is not None:
            original.hide_render = hidden


if __name__ == "__main__":
    main()
