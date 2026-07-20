"""RF19: аннотированные before/after close-up арки и склона."""

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

from cftuv.decal_transform import local_decal_settings_for_source  # noqa: E402
from cftuv.decals import (  # noqa: E402
    _evaluate_manual_backend_partition,
    compile_manual_seam_decal_plan,
    evaluate_manual_seam_faces,
)
from cftuv.model import DecalSettings  # noqa: E402


FIELD_CASES = {
    "rounded_wall.001": (55, 59, 62, 65, 68, 71, 74, 77, 80, 83),
    "sagging_wall": (3, 4, 5, 6, 7, 8, 9, 11),
}


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


def _mesh_object(name, positions, polygons, material):
    mesh = bpy.data.meshes.new(name + "_mesh")
    mesh.from_pydata(positions, [], polygons)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)
    obj.data.materials.append(material)
    return obj


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
    length = max((Vector(point_b) - Vector(point_a)).length * 0.16, radius * 6.0)
    back = Vector(point_b) - direction * length
    created.append(
        _line(name + "_left", point_b, back + side * length * 0.45, material, radius)
    )
    created.append(
        _line(name + "_right", point_b, back - side * length * 0.45, material, radius)
    )
    return created


def _look_at(camera, target):
    direction = Vector(target) - camera.location
    camera.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()


def _label(body, camera, location, scale, material):
    curve = bpy.data.curves.new(body, "FONT")
    curve.body = body
    curve.align_x = "CENTER"
    curve.align_y = "CENTER"
    curve.fill_mode = "BOTH"
    curve.size = scale * 0.13
    curve.extrude = scale * 0.0008
    obj = bpy.data.objects.new(body, curve)
    bpy.context.scene.collection.objects.link(obj)
    obj.data.materials.append(material)
    obj.rotation_euler = camera.rotation_euler
    obj.location = Vector(location)
    return obj


def _overlay(name, faces, source_obj, center, offset, material):
    positions = []
    polygons = []
    for face in faces:
        start = len(positions)
        positions.extend(
            source_obj.matrix_world @ Vector(position) - center + offset
            for position in face.positions
        )
        polygons.append(tuple(range(start, len(positions))))
    return _mesh_object(name, positions, polygons, material)


def _source_panel(name, obj, center, offset, material):
    positions = [
        obj.matrix_world @ vertex.co - center + offset
        for vertex in obj.data.vertices
    ]
    polygons = [tuple(polygon.vertices) for polygon in obj.data.polygons]
    return _mesh_object(name, positions, polygons, material)


def _render_case(obj, selected, materials):
    graph, selected, _all_seams = field._source_graph(obj, selected)
    base_settings = DecalSettings(width_seam=0.8, offset=0.01)
    plan = compile_manual_seam_decal_plan(graph, base_settings, selected)
    partition = next(
        partition
        for partition in plan.backend_partitions
        if partition.backend == "PATCH_VORONOI"
    )
    baseline_settings = local_decal_settings_for_source(base_settings, obj)
    baseline = _evaluate_manual_backend_partition(
        partition,
        baseline_settings,
        width=0.8,
        preview=True,
    ).faces
    narrow = evaluate_manual_seam_faces(
        obj, replace(base_settings, width_seam=0.4), plan, preview=True
    ).faces
    wide = evaluate_manual_seam_faces(
        obj, replace(base_settings, width_seam=0.8), plan, preview=True
    ).faces

    focus_points = []
    selected_edges = []
    for edge_id in selected:
        edge = obj.data.edges[edge_id]
        point_a = obj.matrix_world @ obj.data.vertices[edge.vertices[0]].co
        point_b = obj.matrix_world @ obj.data.vertices[edge.vertices[1]].co
        selected_edges.append((edge_id, point_a, point_b))
        focus_points.extend((point_a, point_b))
    center = sum(focus_points, Vector()) / len(focus_points)
    span = max(
        max(point[axis] for point in focus_points)
        - min(point[axis] for point in focus_points)
        for axis in range(3)
    )
    span = max(span, 1.0)
    view = Vector((1.1, -1.7, 0.9)).normalized()
    screen_right = Vector((-view.y, view.x, 0.0)).normalized()
    screen_up = view.cross(screen_right).normalized()
    panel_offsets = (
        -screen_right * span * 1.45,
        Vector((0.0, 0.0, 0.0)),
        screen_right * span * 1.45,
    )
    overlays = (baseline, narrow, wide)
    overlay_materials = (
        materials["before"], materials["narrow"], materials["wide"]
    )
    labels = (
        "BEFORE\nproduction CAP = PERP",
        "RM9 BRIDGE\nwidth 0.4",
        "RM9 BRIDGE\nwidth 0.8",
    )
    radius = span * 0.0045

    camera_data = bpy.data.cameras.new("R14_CAMERA")
    camera = bpy.data.objects.new("R14_CAMERA", camera_data)
    bpy.context.scene.collection.objects.link(camera)
    camera.location = view * span * 5.5
    _look_at(camera, Vector())
    camera.data.type = "ORTHO"
    camera.data.ortho_scale = span * 3.8
    bpy.context.scene.camera = camera

    for panel_index, (offset, faces, overlay_material, label) in enumerate(
        zip(panel_offsets, overlays, overlay_materials, labels)
    ):
        _source_panel(
            f"R14_SOURCE_{panel_index}", obj, center, offset, materials["source"]
        )
        _overlay(
            f"R14_DECAL_{panel_index}",
            faces,
            obj,
            center,
            offset,
            overlay_material,
        )
        for edge_id, point_a, point_b in selected_edges:
            _line(
                f"R14_SELECTED_{panel_index}_{edge_id}",
                point_a - center + offset,
                point_b - center + offset,
                materials["selected"],
                radius,
            )
        guide_material = (
            materials["ignored"] if panel_index == 0 else materials["guide"]
        )
        vertices = {
            vertex.vertex_id: Vector(vertex.position)
            for vertex in plan.rail_plan.vertices
        }
        edges = {edge.edge_id: edge for edge in plan.rail_plan.edges}
        for terminal in plan.terminal_routing:
            if terminal.backend != "PATCH_VORONOI" or not terminal.edge_ids:
                continue
            guide = edges[terminal.edge_ids[0]]
            other = next(
                vertex_id
                for vertex_id in guide.vertex_ids
                if vertex_id != terminal.spine_vertex_id
            )
            point_a = (
                obj.matrix_world @ vertices[terminal.spine_vertex_id]
                - center
                + offset
            )
            point_b = obj.matrix_world @ vertices[other] - center + offset
            _arrow(
                f"R14_GUIDE_{panel_index}_{terminal.patch_id}_{terminal.route_id}",
                point_a,
                point_b,
                guide_material,
                radius * 1.15,
            )
        _label(
            label,
            camera,
            offset + screen_up * span * 0.92 + view * span * 0.03,
            span,
            materials["label"],
        )

    light_data = bpy.data.lights.new("R14_KEY", "AREA")
    light_data.energy = 1500.0
    light_data.size = span * 4.0
    light = bpy.data.objects.new("R14_KEY", light_data)
    bpy.context.scene.collection.objects.link(light)
    light.location = Vector((0.0, 0.0, span * 4.0))

    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE_NEXT"
    scene.render.resolution_x = 1800
    scene.render.resolution_y = 720
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.film_transparent = False
    scene.world.color = (0.012, 0.016, 0.025)
    scene.render.filepath = str(
        ARTIFACT_ROOT
        / f"decal_r14_terminal_bridge_{obj.name.replace('.', '_')}.png"
    )
    bpy.ops.render.render(write_still=True)


def main():
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    for obj in bpy.data.objects:
        obj.hide_render = True
    materials = {
        "source": _material("R14_source", (0.20, 0.23, 0.29, 1.0)),
        "before": _material("R14_before", (0.95, 0.05, 0.025, 0.82), 0.25),
        "narrow": _material("R14_narrow", (0.0, 0.72, 1.0, 0.84), 0.28),
        "wide": _material("R14_wide", (0.04, 0.30, 1.0, 0.84), 0.28),
        "selected": _material("R14_selected", (1.0, 0.42, 0.02, 1.0), 0.5),
        "guide": _material("R14_guide", (0.10, 1.0, 0.22, 1.0), 0.7),
        "ignored": _material("R14_ignored", (1.0, 0.08, 0.04, 1.0), 0.6),
        "label": _material("R14_label", (0.96, 0.98, 1.0, 1.0), 2.0),
    }
    for object_name, selected in FIELD_CASES.items():
        obj = bpy.data.objects[object_name]
        _render_case(obj, selected, materials)
        for created in tuple(bpy.data.objects):
            if created.hide_render:
                continue
            if created.type in {"CAMERA", "LIGHT"} or created.name.startswith(
                "R14_"
            ) or created.name.startswith(("BEFORE", "RM9")):
                bpy.data.objects.remove(created, do_unlink=True)


if __name__ == "__main__":
    main()
