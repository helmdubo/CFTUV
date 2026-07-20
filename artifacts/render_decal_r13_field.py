"""Автоскрины шести production-объектов для отчёта R1.3."""

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

import verify_decal_r13_rr9a as field  # noqa: E402

from cftuv.decals import (  # noqa: E402
    compile_manual_seam_decal_plan,
    evaluate_manual_seam_faces,
)
from cftuv.model import DecalSettings  # noqa: E402


def _material(name, color, emission=0.0):
    material = bpy.data.materials.new(name)
    material.diffuse_color = color
    material.use_nodes = True
    principled = material.node_tree.nodes.get("Principled BSDF")
    principled.inputs["Base Color"].default_value = color
    principled.inputs["Roughness"].default_value = 0.48
    if emission:
        principled.inputs["Emission Color"].default_value = color
        principled.inputs["Emission Strength"].default_value = emission
    if color[3] < 1.0:
        principled.inputs["Alpha"].default_value = color[3]
        material.surface_render_method = "DITHERED"
    return material


def _line(name, point_a, point_b, material, radius):
    curve = bpy.data.curves.new(name, "CURVE")
    curve.dimensions = "3D"
    curve.bevel_depth = radius
    curve.bevel_resolution = 2
    spline = curve.splines.new("POLY")
    spline.points.add(1)
    spline.points[0].co = (*point_a, 1.0)
    spline.points[1].co = (*point_b, 1.0)
    obj = bpy.data.objects.new(name, curve)
    bpy.context.scene.collection.objects.link(obj)
    obj.data.materials.append(material)
    return obj


def _arrow(name, point_a, point_b, material, radius):
    created = [_line(name + "_shaft", point_a, point_b, material, radius)]
    direction = (Vector(point_b) - Vector(point_a)).normalized()
    side = direction.cross(Vector((0.0, 0.0, 1.0)))
    if side.length == 0.0:
        side = direction.cross(Vector((0.0, 1.0, 0.0)))
    side.normalize()
    length = max((Vector(point_b) - Vector(point_a)).length * 0.16, radius * 5.0)
    back = Vector(point_b) - direction * length
    created.append(
        _line(name + "_left", point_b, back + side * length * 0.45, material, radius)
    )
    created.append(
        _line(name + "_right", point_b, back - side * length * 0.45, material, radius)
    )
    return created


def _overlay_object(name, faces, matrix_world, material):
    positions = []
    polygons = []
    for face in faces:
        start = len(positions)
        positions.extend(
            tuple(matrix_world @ Vector(position)) for position in face.positions
        )
        polygons.append(tuple(range(start, len(positions))))
    mesh = bpy.data.meshes.new(name + "_mesh")
    mesh.from_pydata(positions, [], polygons)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)
    obj.data.materials.append(material)
    return obj


def _look_at(camera, target):
    direction = Vector(target) - camera.location
    camera.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()


def _label(body, camera, target, scale, material):
    curve = bpy.data.curves.new(body, "FONT")
    curve.body = body
    curve.align_x = "CENTER"
    curve.size = scale * 0.022
    curve.extrude = scale * 0.001
    obj = bpy.data.objects.new(body, curve)
    bpy.context.scene.collection.objects.link(obj)
    obj.data.materials.append(material)
    obj.rotation_euler = camera.rotation_euler
    up = camera.matrix_world.to_quaternion() @ Vector((0.0, 1.0, 0.0))
    toward_camera = (camera.location - Vector(target)).normalized()
    obj.location = Vector(target) + up * scale * 0.40 + toward_camera * scale * 0.03
    return obj


def _render_object(obj, requested_edges, materials):
    graph, selected, _all_seams = field._source_graph(obj, requested_edges)
    settings = DecalSettings(width_seam=0.8, offset=0.01)
    plan = compile_manual_seam_decal_plan(graph, settings, selected)
    primary_backend = plan.backend_summary.split(" | ", 1)[0]
    try:
        evaluated = evaluate_manual_seam_faces(
            obj,
            replace(settings, width_seam=0.8),
            plan,
            preview=True,
        )
        decal_faces = evaluated.faces
        status = f"{primary_backend}\nwidth=0.8 | faces={len(decal_faces)}"
    except Exception as exc:
        decal_faces = ()
        status = f"{primary_backend}\n{type(exc).__name__}"

    mesh = obj.data
    selected_world_edges = []
    focus_points = []
    for edge_id in selected:
        edge = mesh.edges[edge_id]
        point_a = obj.matrix_world @ mesh.vertices[edge.vertices[0]].co
        point_b = obj.matrix_world @ mesh.vertices[edge.vertices[1]].co
        selected_world_edges.append((edge_id, point_a, point_b))
        focus_points.extend((point_a, point_b))
    focus_points.extend(
        obj.matrix_world @ Vector(position)
        for face in decal_faces
        for position in face.positions
    )
    center = sum(focus_points, Vector()) / len(focus_points)
    span = max(
        max(point[axis] for point in focus_points)
        - min(point[axis] for point in focus_points)
        for axis in range(3)
    )
    span = max(span, 1.0)
    radius = span * 0.006

    temp_objects = []
    duplicate = obj.copy()
    duplicate.data = obj.data.copy()
    duplicate.name = "R13_SOURCE_" + obj.name
    bpy.context.scene.collection.objects.link(duplicate)
    duplicate.matrix_world = obj.matrix_world.copy()
    duplicate.data.materials.clear()
    duplicate.data.materials.append(materials["source"])
    temp_objects.append(duplicate)
    if decal_faces:
        temp_objects.append(
            _overlay_object(
                "R13_DECAL_" + obj.name,
                decal_faces,
                obj.matrix_world,
                materials["decal"],
            )
        )

    for edge_id, point_a, point_b in selected_world_edges:
        temp_objects.append(
            _line(
                f"selected_{obj.name}_{edge_id}",
                point_a,
                point_b,
                materials["selected"],
                radius,
            )
        )
    if plan.rail_plan is not None:
        source_vertices = {
            vertex.vertex_id: Vector(vertex.position)
            for vertex in plan.rail_plan.vertices
        }
        source_edges = {edge.edge_id: edge for edge in plan.rail_plan.edges}
        seen_guides = set()
        for use in plan.rail_plan.terminal_uses:
            if use.route_edge_id is None or use.route_edge_id in seen_guides:
                continue
            seen_guides.add(use.route_edge_id)
            edge = source_edges[use.route_edge_id]
            other_vertex_id = next(
                vertex_id
                for vertex_id in edge.vertex_ids
                if vertex_id != use.spine_vertex_id
            )
            point_a = obj.matrix_world @ source_vertices[use.spine_vertex_id]
            point_b = obj.matrix_world @ source_vertices[other_vertex_id]
            temp_objects.extend(
                _arrow(
                    f"guide_{obj.name}_{use.route_edge_id}",
                    point_a,
                    point_b,
                    materials["guide"],
                    radius * 1.2,
                )
            )

    camera_data = bpy.data.cameras.new("R13_CAMERA")
    camera = bpy.data.objects.new("R13_CAMERA", camera_data)
    bpy.context.scene.collection.objects.link(camera)
    view = Vector((1.25, -1.55, 0.85)).normalized()
    camera.location = center + view * span * 3.0
    _look_at(camera, center)
    camera.data.type = "ORTHO"
    camera.data.ortho_scale = span * 1.35
    bpy.context.scene.camera = camera
    temp_objects.append(camera)
    temp_objects.append(
        _label(f"{obj.name}\n{status}", camera, center, span, materials["label"])
    )

    light_data = bpy.data.lights.new("R13_KEY", "AREA")
    light_data.energy = 1300.0
    light_data.size = span * 2.5
    light = bpy.data.objects.new("R13_KEY", light_data)
    bpy.context.scene.collection.objects.link(light)
    light.location = center + Vector((0.0, 0.0, span * 2.5))
    temp_objects.append(light)

    scene = bpy.context.scene
    scene.render.filepath = str(
        ARTIFACT_ROOT
        / f"decal_r13_field_{obj.name.replace('.', '_')}.png"
    )
    bpy.ops.render.render(write_still=True)

    for temp in temp_objects:
        if temp.name in bpy.data.objects:
            bpy.data.objects.remove(temp, do_unlink=True)


def main():
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    for obj in bpy.data.objects:
        obj.hide_render = True
    materials = {
        "source": _material("R13_source", (0.28, 0.31, 0.36, 1.0)),
        "decal": _material("R13_decal", (0.02, 0.50, 1.0, 0.86), 0.3),
        "selected": _material("R13_selected", (1.0, 0.35, 0.02, 1.0), 0.5),
        "guide": _material("R13_guide", (0.10, 1.0, 0.22, 1.0), 0.6),
        "label": _material("R13_label", (0.96, 0.98, 1.0, 1.0), 0.2),
    }
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE_NEXT"
    scene.render.resolution_x = 1280
    scene.render.resolution_y = 800
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.world.color = (0.008, 0.011, 0.018)
    scene.view_settings.look = "AgX - Medium High Contrast"
    for object_name, requested_edges in field.FIELD_OBJECTS.items():
        obj = bpy.data.objects.get(object_name)
        if obj is None:
            raise RuntimeError(f"missing field object: {object_name}")
        _render_object(obj, requested_edges, materials)
    print("CFTUV_R13_FIELD_RENDERS=6")


if __name__ == "__main__":
    main()
