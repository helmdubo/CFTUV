"""RD-1/RF12: GPU-Solid style render после поглощения planar CAP."""

from __future__ import annotations

import sys
from pathlib import Path

import bpy
from mathutils import Vector


REPO_ROOT = Path(__file__).resolve().parents[1]
ARTIFACT_ROOT = REPO_ROOT / "artifacts"
for path in (REPO_ROOT, ARTIFACT_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import render_decal_r14_terminal_bridge as render_base  # noqa: E402
import verify_decal_r13_rr9a as field  # noqa: E402

from cftuv.decals import (  # noqa: E402
    compile_manual_seam_decal_plan,
    evaluate_manual_seam_faces,
)
from cftuv.model import DecalSettings  # noqa: E402


OBJECT_NAME = "sagging_wall"
SELECTED = (3, 4, 5, 6, 7, 44, 48, 51, 54, 57, 59)
OUTPUT_PNG = ARTIFACT_ROOT / "decal_rd1_rf12_cap_absorption_after.png"


def main():
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    scene = bpy.context.scene
    original_camera = scene.camera
    original_object_names = {obj.name for obj in bpy.data.objects}
    original_hide_render = {
        obj.name: obj.hide_render for obj in bpy.data.objects
    }
    for scene_object in bpy.data.objects:
        scene_object.hide_render = True

    source = bpy.data.objects[OBJECT_NAME]
    graph, selected, _all_seams = field._source_graph(source, SELECTED)
    settings = DecalSettings(width_seam=0.73, offset=0.01)
    plan = compile_manual_seam_decal_plan(graph, settings, selected)
    faces = evaluate_manual_seam_faces(
        source, settings, plan, preview=True
    ).faces

    selected_points = [
        source.matrix_world @ source.data.vertices[vertex_id].co
        for edge_id in selected
        for vertex_id in source.data.edges[edge_id].vertices
    ]
    center = sum(selected_points, Vector()) / len(selected_points)
    span = max(
        max(point[axis] for point in selected_points)
        - min(point[axis] for point in selected_points)
        for axis in range(3)
    )
    span = max(span, 1.0)

    materials = {
        "source": render_base._material(
            "RD1_SOURCE", (0.12, 0.15, 0.22, 1.0)
        ),
        "SEGMENT": render_base._material(
            "RD1_SEGMENT", (0.24, 0.52, 0.89, 1.0)
        ),
        "KITE": render_base._material(
            "RD1_KITE", (0.93, 0.74, 0.25, 1.0)
        ),
        "CAP": render_base._material(
            "RD1_CAP", (0.35, 0.80, 0.78, 1.0)
        ),
        "selected": render_base._material(
            "RD1_SELECTED", (1.0, 0.46, 0.02, 1.0), 1.4
        ),
        "label": render_base._material(
            "RD1_LABEL", (0.96, 0.98, 1.0, 1.0), 1.8
        ),
    }

    render_base._source_panel(
        "RD1_SOURCE", source, center, Vector(), materials["source"]
    )
    for kind in sorted({face.component_kind for face in faces}):
        render_base._overlay(
            f"RD1_{kind}",
            tuple(face for face in faces if face.component_kind == kind),
            source,
            center,
            Vector(),
            materials.get(kind, materials["SEGMENT"]),
        )
    radius = span * 0.0035
    for edge_id in selected:
        edge = source.data.edges[edge_id]
        point_a = source.matrix_world @ source.data.vertices[
            edge.vertices[0]
        ].co
        point_b = source.matrix_world @ source.data.vertices[
            edge.vertices[1]
        ].co
        render_base._line(
            f"RD1_SELECTED_{edge_id}",
            point_a - center,
            point_b - center,
            materials["selected"],
            radius,
        )

    view = Vector((1.1, -1.7, 0.9)).normalized()
    camera_data = bpy.data.cameras.new("RD1_CAMERA")
    camera = bpy.data.objects.new("RD1_CAMERA", camera_data)
    scene.collection.objects.link(camera)
    camera.location = view * span * 4.8
    render_base._look_at(camera, Vector())
    camera.data.type = "ORTHO"
    camera.data.ortho_scale = span * 1.65
    scene.camera = camera
    screen_right = Vector((-view.y, view.x, 0.0)).normalized()
    screen_up = view.cross(screen_right).normalized()
    render_base._label(
        "AFTER RD-1 / RF12\nCAP 2 -> 0 | one station-UV strip",
        camera,
        screen_up * span * 0.65 + view * span * 0.03,
        span * 0.72,
        materials["label"],
    )

    light_data = bpy.data.lights.new("RD1_KEY", "AREA")
    light_data.energy = 1400.0
    light_data.size = span * 4.0
    light = bpy.data.objects.new("RD1_KEY", light_data)
    scene.collection.objects.link(light)
    light.location = view * span * 4.0

    scene.render.engine = "BLENDER_EEVEE_NEXT"
    scene.render.resolution_x = 1600
    scene.render.resolution_y = 900
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.film_transparent = False
    scene.world.color = (0.008, 0.011, 0.018)
    scene.render.filepath = str(OUTPUT_PNG)
    bpy.ops.render.render(write_still=True)
    print("CFTUV_RD1_RENDER", OUTPUT_PNG)
    print(
        "CFTUV_RD1_COUNTS",
        len(faces),
        sum(face.component_kind == "CAP" for face in faces),
    )

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
