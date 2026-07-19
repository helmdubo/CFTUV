"""Аннотированный Blender render RF18: snap, drag и losing-chain clip."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import bpy
from mathutils import Vector


REPO_ROOT = Path(__file__).resolve().parents[1]
TEST_ROOT = REPO_ROOT / "tests"
for path in (REPO_ROOT, TEST_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from cftuv import decal_rails  # noqa: E402
from cftuv.decal_rail_geometry import (  # noqa: E402
    compile_planar_rail_geometry_attempt,
    evaluate_planar_rail_geometry_plan,
)
from decal_rail_fixtures import rr9a_rf18_terminal_seam_snap  # noqa: E402


OUTPUT_PNG = REPO_ROOT / "artifacts" / "decal_r13_rr9a_rf18_annotated.png"
OUTPUT_JSON = REPO_ROOT / "artifacts" / "decal_r13_rr9a_rf18_acceptance.json"

SOURCE_VERTICES = {
    0: (0.0, -2.0, 0.0),
    1: (0.0, 0.0, 0.0),
    2: (1.0, 0.1, 0.0),
    3: (1.4, 1.0, 0.0),
    4: (0.45, 1.0, 0.0),
    5: (-1.4, 0.0, 0.0),
}
SOURCE_FACES = (
    (0, 1, 2),
    (2, 1, 3),
    (3, 1, 4),
    (4, 1, 5),
    (5, 1, 0),
)


def _material(name, color, emission=0.0):
    material = bpy.data.materials.new(name)
    material.diffuse_color = color
    material.use_nodes = True
    principled = material.node_tree.nodes.get("Principled BSDF")
    principled.inputs["Base Color"].default_value = color
    principled.inputs["Roughness"].default_value = 0.4
    if emission:
        principled.inputs["Emission Color"].default_value = color
        principled.inputs["Emission Strength"].default_value = emission
    if color[3] < 1.0:
        principled.inputs["Alpha"].default_value = color[3]
        material.surface_render_method = "DITHERED"
    return material


def _mesh_object(name, positions, faces, material, offset_x=0.0):
    mesh = bpy.data.meshes.new(f"{name}_mesh")
    shifted = [
        (float(position[0]) + offset_x, float(position[1]), float(position[2]))
        for position in positions
    ]
    mesh.from_pydata(shifted, [], faces)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)
    obj.data.materials.append(material)
    return obj


def _source_panel(name, offset_x, source_material):
    positions = [SOURCE_VERTICES[index] for index in sorted(SOURCE_VERTICES)]
    _mesh_object(name, positions, SOURCE_FACES, source_material, offset_x)


def _evaluated_overlay(name, faces, offset_x, material):
    positions = []
    polygons = []
    for face in faces:
        if face.surface_id != 1000:
            continue
        start = len(positions)
        positions.extend(
            (float(point[0]), float(point[1]), float(point[2]) + 0.035)
            for point in face.positions
        )
        polygons.append(tuple(range(start, len(positions))))
    return _mesh_object(name, positions, polygons, material, offset_x)


def _line(name, point_a, point_b, material, offset_x=0.0, radius=0.026):
    curve = bpy.data.curves.new(name, "CURVE")
    curve.dimensions = "3D"
    curve.bevel_depth = radius
    curve.bevel_resolution = 3
    spline = curve.splines.new("POLY")
    spline.points.add(1)
    spline.points[0].co = (
        point_a[0] + offset_x,
        point_a[1],
        point_a[2],
        1.0,
    )
    spline.points[1].co = (
        point_b[0] + offset_x,
        point_b[1],
        point_b[2],
        1.0,
    )
    obj = bpy.data.objects.new(name, curve)
    bpy.context.scene.collection.objects.link(obj)
    obj.data.materials.append(material)
    return obj


def _arrow(name, point_a, point_b, material, offset_x=0.0):
    _line(name + "_shaft", point_a, point_b, material, offset_x, 0.032)
    direction = (Vector(point_b) - Vector(point_a)).normalized()
    tip = Vector(point_b)
    perpendicular = Vector((-direction.y, direction.x, 0.0))
    wing = 0.16
    back = tip - direction * 0.32
    _line(name + "_left", tip, back + perpendicular * wing, material, offset_x, 0.032)
    _line(name + "_right", tip, back - perpendicular * wing, material, offset_x, 0.032)


def _text(body, location, material, size=0.34):
    curve = bpy.data.curves.new(body, "FONT")
    curve.body = body
    curve.align_x = "CENTER"
    curve.size = size
    curve.extrude = 0.01
    obj = bpy.data.objects.new(body, curve)
    bpy.context.scene.collection.objects.link(obj)
    obj.location = location
    obj.data.materials.append(material)
    return obj


def _terminal_frontier(faces, selected_edge_id, route_id):
    face = next(face for face in faces if face.surface_id == 1000)
    return next(
        Vector(position)
        for key, position in zip(face.vert_keys, face.positions)
        if key[:3] == (
            "rail-frontier",
            selected_edge_id,
            ("ROUTE", route_id),
        )
    )


def main():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)

    source_mat = _material("source", (0.10, 0.12, 0.16, 1.0))
    red = _material("before", (0.95, 0.04, 0.025, 0.82), 0.25)
    blue = _material("snap_small", (0.02, 0.32, 1.0, 0.84), 0.25)
    cyan = _material("snap_large", (0.0, 0.82, 1.0, 0.84), 0.3)
    magenta = _material("mark", (0.95, 0.06, 0.75, 0.84), 0.3)
    green = _material("chosen", (0.15, 1.0, 0.24, 1.0), 0.4)
    orange = _material("losing_clip", (1.0, 0.42, 0.02, 1.0), 0.4)
    white = _material("labels", (0.95, 0.97, 1.0, 1.0), 0.2)

    graph, edge_ids, selected, _vertex_at = rr9a_rf18_terminal_seam_snap()
    rail_plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=1.0,
    )
    terminal_use = next(
        use
        for use in rail_plan.terminal_uses
        if use.spine_vertex_id == 1 and 1000 in use.source_face_ids
    )
    attempt = compile_planar_rail_geometry_attempt(
        rail_plan,
        edge_indices=selected,
    )
    if attempt.plan is None:
        raise RuntimeError(attempt.failures)
    small_faces = evaluate_planar_rail_geometry_plan(attempt.plan, width=0.4)
    large_faces = evaluate_planar_rail_geometry_plan(attempt.plan, width=1.4)

    oblique_edge = edge_ids[(1, 4)]
    losing_edge = edge_ids[(1, 2)]
    marked_plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=1.0,
        rail_mark_edge_indices=(oblique_edge,),
    )
    marked_use = next(
        use
        for use in marked_plan.terminal_uses
        if use.spine_vertex_id == 1 and 1000 in use.source_face_ids
    )
    marked_attempt = compile_planar_rail_geometry_attempt(
        marked_plan,
        edge_indices=selected,
    )
    if marked_attempt.plan is None:
        raise RuntimeError(marked_attempt.failures)
    marked_faces = evaluate_planar_rail_geometry_plan(
        marked_attempt.plan,
        width=1.4,
    )

    panels = (-6.0, -2.0, 2.0, 6.0)
    for index, offset_x in enumerate(panels):
        _source_panel(f"source_{index}", offset_x, source_mat)
        _line(
            f"spine_{index}",
            SOURCE_VERTICES[0],
            SOURCE_VERTICES[1],
            white,
            offset_x,
            0.035,
        )

    # BEFORE: старый wavefront-перпендикуляр без snap.
    old_overlay = (
        (0.0, -2.0, 0.04),
        (0.0, 0.0, 0.04),
        (0.7, 0.0, 0.04),
        (0.7, -2.0, 0.04),
    )
    _mesh_object("old_perpendicular", old_overlay, ((0, 1, 2, 3),), red, panels[0])
    _arrow("old_direction", (0.0, 0.0, 0.10), (1.0, 0.0, 0.10), red, panels[0])

    _evaluated_overlay("snap_small", small_faces, panels[1], blue)
    _evaluated_overlay("snap_large", large_faces, panels[2], cyan)
    _evaluated_overlay("mark_override", marked_faces, panels[3], magenta)

    for panel_index, offset_x in ((1, panels[1]), (2, panels[2])):
        _arrow(
            f"chosen_{panel_index}",
            (0.0, 0.0, 0.10),
            (1.0, 0.1, 0.10),
            green,
            offset_x,
        )
    _arrow(
        "marked_oblique",
        (0.0, 0.0, 0.10),
        (0.45, 1.0, 0.10),
        magenta,
        panels[3],
    )
    _line(
        "losing_pchain_clip",
        (0.0, 0.0, 0.12),
        (1.0, 0.1, 0.12),
        orange,
        panels[3],
        0.055,
    )

    labels = (
        ("BEFORE: perpendicular", panels[0], red),
        ("AFTER: width 0.4", panels[1], blue),
        ("AFTER: width 1.4", panels[2], cyan),
        ("MARK wins; orange still clips", panels[3], magenta),
    )
    for body, offset_x, material in labels:
        _text(body, (offset_x, 2.05, 0.06), material)
    _text(
        "green arrow = snapped terminal guide; one compiled route, drag changes extent only",
        (0.0, -3.0, 0.06),
        white,
        0.30,
    )

    camera_data = bpy.data.cameras.new("Camera")
    camera = bpy.data.objects.new("Camera", camera_data)
    bpy.context.scene.collection.objects.link(camera)
    camera.location = (0.0, 0.0, 24.0)
    camera.rotation_euler = (0.0, 0.0, 0.0)
    camera.data.type = "ORTHO"
    camera.data.ortho_scale = 17.0
    bpy.context.scene.camera = camera

    light_data = bpy.data.lights.new("Key", "AREA")
    light_data.energy = 1100.0
    light_data.size = 14.0
    light = bpy.data.objects.new("Key", light_data)
    bpy.context.scene.collection.objects.link(light)
    light.location = (0.0, 0.0, 12.0)

    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE_NEXT"
    scene.render.resolution_x = 1600
    scene.render.resolution_y = 760
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = str(OUTPUT_PNG)
    scene.world.color = (0.008, 0.011, 0.019)
    scene.view_settings.look = "AgX - Medium High Contrast"
    bpy.ops.render.render(write_still=True)

    small_frontier = _terminal_frontier(
        small_faces, selected[0], terminal_use.route_id
    )
    large_frontier = _terminal_frontier(
        large_faces, selected[0], terminal_use.route_id
    )
    losing_route = next(
        route
        for route in marked_plan.routes
        if route.key.side.spine_vertex_id == 1
        and route.key.side.start_edge_id == losing_edge
    )
    report = {
        "schema": "cftuv.decal_r13_rr9a.rf18.v1",
        "blender_version": bpy.app.version_string,
        "selected_spine_edge": selected[0],
        "candidate_edges": list(terminal_use.candidate_edge_ids),
        "snap_chosen_edge": terminal_use.route_edge_id,
        "snap_route_id": terminal_use.route_id,
        "small_width_frontier": list(small_frontier),
        "large_width_frontier": list(large_frontier),
        "same_compiled_plan": attempt.plan.rail_plan is rail_plan,
        "mark_chosen_edge": marked_use.route_edge_id,
        "losing_edge": losing_edge,
        "losing_route_id": losing_route.route_id,
        "losing_edge_remains_pchain": next(
            edge.is_pchain for edge in marked_plan.edges if edge.edge_id == losing_edge
        ),
        "blend_saved": False,
        "image": str(OUTPUT_PNG),
    }
    OUTPUT_JSON.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print("CFTUV_R13_RR9A_RF18=" + json.dumps(report, sort_keys=True))


if __name__ == "__main__":
    main()
