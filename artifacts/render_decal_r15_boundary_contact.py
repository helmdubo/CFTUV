"""R1.5/RF26: wireframe proof of independent boundary contact reach."""

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

loaded_cftuv = sys.modules.get("cftuv")
if loaded_cftuv is not None:
    loaded_path = Path(getattr(loaded_cftuv, "__file__", "")).resolve()
    if REPO_ROOT not in loaded_path.parents:
        for module_name in tuple(sys.modules):
            if module_name == "cftuv" or module_name.startswith("cftuv."):
                del sys.modules[module_name]

import render_decal_r14_fix_terminal_wireframe as wire_base  # noqa: E402
import render_decal_r14_terminal_bridge as render_base  # noqa: E402
import verify_decal_r13_rr9a as field  # noqa: E402

from cftuv.decals import compile_manual_seam_decal_plan  # noqa: E402
from cftuv.decal_rail_geometry import (  # noqa: E402
    evaluate_planar_rail_geometry_plan,
)
from cftuv.model import DecalSettings  # noqa: E402


OBJECT_NAME = "sagging_wall"
SPINE_EDGE_ID = 3
BOUNDARY_EDGE_ID = 1
OUTPUT_PNG = ARTIFACT_ROOT / "decal_r15_boundary_contact_wireframe.png"


def _source_edge_line(name, obj, edge_id, center, offset, material, radius):
    edge = obj.data.edges[edge_id]
    point_a = obj.matrix_world @ obj.data.vertices[edge.vertices[0]].co
    point_b = obj.matrix_world @ obj.data.vertices[edge.vertices[1]].co
    render_base._line(
        name,
        point_a - center + offset,
        point_b - center + offset,
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
    graph, selected, _all_seams = field._source_graph(obj, (SPINE_EDGE_ID,))
    plan = compile_manual_seam_decal_plan(
        graph,
        DecalSettings(width_seam=0.8, offset=0.01),
        selected,
        alpha_budget=4.0,
    )
    geometry_plan = plan.backend_partitions[0].compiled_plan
    channel = next(
        candidate for candidate in geometry_plan.channels if candidate.side_sign == -1
    )
    legacy_width = 2.0 * channel.alpha_limit
    panels = (
        ("BEFORE: request 4.0\nwhole channel frozen at 1.4479", legacy_width),
        ("AFTER: width 3.2\ncontact is source-edge station", 3.2),
        ("AFTER: width 4.0\nlong bank keeps sliding", 4.0),
    )
    evaluated = tuple(
        (label, width, evaluate_planar_rail_geometry_plan(geometry_plan, width))
        for label, width in panels
    )

    materials = {
        "source": render_base._material("R15_source", (0.08, 0.11, 0.16, 1.0)),
        "old_fill": render_base._material("R15_old_fill", (0.90, 0.08, 0.03, 0.18)),
        "old_wire": render_base._material("R15_old_wire", (1.0, 0.18, 0.04, 1.0), 1.5),
        "new_fill": render_base._material("R15_new_fill", (0.02, 0.42, 1.0, 0.18)),
        "new_wire": render_base._material("R15_new_wire", (0.08, 0.82, 1.0, 1.0), 1.5),
        "spine": render_base._material("R15_spine", (1.0, 0.44, 0.02, 1.0), 1.8),
        "boundary": render_base._material("R15_boundary", (1.0, 0.92, 0.02, 1.0), 2.0),
        "label": render_base._material("R15_label", (0.97, 0.98, 1.0, 1.0), 2.0),
    }

    target_vertices = set(obj.data.edges[BOUNDARY_EDGE_ID].vertices)
    target_vertices.update(obj.data.edges[SPINE_EDGE_ID].vertices)
    neighborhood = set()
    for vertex_id in target_vertices:
        neighborhood.update(wire_base._vertex_neighborhood(obj, vertex_id, depth=2))
    center = sum(
        (obj.matrix_world @ obj.data.vertices[index].co for index in target_vertices),
        Vector(),
    ) / len(target_vertices)
    span = max(
        ((obj.matrix_world @ obj.data.vertices[index].co - center).length for index in neighborhood),
        default=1.0,
    )
    span = max(span, 1.0)
    view = Vector((1.1, -1.7, 0.85)).normalized()
    screen_right = Vector((-view.y, view.x, 0.0)).normalized()
    screen_up = view.cross(screen_right).normalized()
    offsets = (-1.18, 0.0, 1.18)
    radius = span * 0.0020

    camera_data = bpy.data.cameras.new("R15_CAMERA")
    camera = bpy.data.objects.new("R15_CAMERA", camera_data)
    scene.collection.objects.link(camera)
    camera.location = view * span * 7.0
    render_base._look_at(camera, Vector())
    camera.data.type = "ORTHO"
    camera.data.ortho_scale = span * 2.15
    scene.camera = camera

    for panel_index, (label, _width, faces) in enumerate(evaluated):
        offset = screen_right * span * offsets[panel_index]
        wire_base._source_closeup(
            f"R15_SOURCE_{panel_index}",
            obj,
            neighborhood,
            center,
            offset,
            materials["source"],
        )
        close_faces = tuple(
            face
            for face in faces
            if wire_base._face_is_close(face, obj, center, span * 0.95)
        )
        wire_base._wire_overlay(
            f"R15_DECAL_{panel_index}",
            close_faces,
            obj,
            center,
            offset,
            materials["old_fill" if panel_index == 0 else "new_fill"],
            materials["old_wire" if panel_index == 0 else "new_wire"],
            radius,
        )
        _source_edge_line(
            f"R15_SPINE_{panel_index}", obj, SPINE_EDGE_ID, center, offset,
            materials["spine"], radius * 1.2,
        )
        _source_edge_line(
            f"R15_BOUNDARY_{panel_index}", obj, BOUNDARY_EDGE_ID, center, offset,
            materials["boundary"], radius * 1.4,
        )
        render_base._label(
            label,
            camera,
            offset + screen_up * span * 0.56 + view * span * 0.04,
            span * 0.42,
            materials["label"],
        )

    render_base._label(
        "YELLOW e1 = mesh boundary | ORANGE e3 = selected spine\n"
        "red = rejected whole-channel clamp | blue = RR8b independent reach",
        camera,
        screen_up * span * 0.78 + view * span * 0.04,
        span * 0.30,
        materials["label"],
    )

    light_data = bpy.data.lights.new("R15_KEY", "AREA")
    light_data.energy = 1250.0
    light_data.size = span * 5.0
    light = bpy.data.objects.new("R15_KEY", light_data)
    scene.collection.objects.link(light)
    light.location = view * span * 4.0

    scene.render.engine = "BLENDER_EEVEE_NEXT"
    scene.render.resolution_x = 2200
    scene.render.resolution_y = 900
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.film_transparent = False
    scene.world.color = (0.006, 0.009, 0.014)
    scene.render.filepath = str(OUTPUT_PNG)
    bpy.ops.render.render(write_still=True)
    print("R15_RENDER", OUTPUT_PNG)
    print("R15_TERMINALS", plan.terminal_routing_report)

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
