"""RF19-fix: edit-mode/wireframe close-ups terminal zones before/after."""

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

import render_decal_r14_terminal_bridge as render_base  # noqa: E402
import verify_decal_r13_rr9a as field  # noqa: E402

from cftuv.decal_transform import local_decal_settings_for_source  # noqa: E402
from cftuv.decal_rail_geometry import _station_position  # noqa: E402
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


def _vertex_neighborhood(obj, root_vertex, depth=2):
    adjacency = {vertex.index: set() for vertex in obj.data.vertices}
    for edge in obj.data.edges:
        first, second = edge.vertices
        adjacency[first].add(second)
        adjacency[second].add(first)
    result = {int(root_vertex)}
    frontier = set(result)
    for _step in range(depth):
        frontier = {
            neighbor
            for vertex in frontier
            for neighbor in adjacency.get(vertex, ())
            if neighbor not in result
        }
        result.update(frontier)
    return result


def _source_closeup(name, obj, vertex_ids, center, offset, material):
    polygons = [
        polygon
        for polygon in obj.data.polygons
        if set(polygon.vertices).intersection(vertex_ids)
    ]
    used_vertices = sorted(
        {vertex for polygon in polygons for vertex in polygon.vertices}
    )
    local_index = {
        vertex_id: index for index, vertex_id in enumerate(used_vertices)
    }
    positions = [
        obj.matrix_world @ obj.data.vertices[vertex_id].co - center + offset
        for vertex_id in used_vertices
    ]
    faces = [
        tuple(local_index[vertex_id] for vertex_id in polygon.vertices)
        for polygon in polygons
    ]
    return render_base._mesh_object(name, positions, faces, material)


def _face_is_close(face, obj, center, radius):
    return any(
        (obj.matrix_world @ Vector(point) - center).length <= radius
        for point in face.positions
    )


def _wire_overlay(
    name,
    faces,
    source_obj,
    center,
    offset,
    fill_material,
    wire_material,
    radius,
):
    close_faces = tuple(faces)
    render_base._overlay(
        name + "_FILL",
        close_faces,
        source_obj,
        center,
        offset,
        fill_material,
    )
    for face_index, face in enumerate(close_faces):
        points = [
            source_obj.matrix_world @ Vector(point) - center + offset
            for point in face.positions
        ]
        for edge_index, point_a in enumerate(points):
            point_b = points[(edge_index + 1) % len(points)]
            render_base._line(
                f"{name}_EDGE_{face_index}_{edge_index}",
                point_a,
                point_b,
                wire_material,
                radius,
            )


def _terminal_rows(plan):
    grouped = {}
    for terminal in plan.terminal_routing:
        if terminal.backend != "PATCH_VORONOI":
            continue
        grouped.setdefault(terminal.spine_vertex_id, []).append(terminal)
    return tuple(
        (vertex_id, tuple(grouped[vertex_id])) for vertex_id in sorted(grouped)
    )


def _route_extent_position(rail_plan, terminal, alpha):
    route = next(
        item for item in rail_plan.routes if item.route_id == terminal.route_id
    )
    stations = {station.station_index: station for station in route.stations}
    vertices = {
        vertex.vertex_id: Vector(vertex.position) for vertex in rail_plan.vertices
    }
    edges = {edge.edge_id: edge for edge in rail_plan.edges}
    extent = min(float(alpha), float(route.stations[-1].distance))
    for segment in route.segments:
        station_a = stations[segment.from_station_index]
        station_b = stations[segment.to_station_index]
        if not station_a.distance <= extent <= station_b.distance:
            continue
        point_a = _station_position(route, station_a, vertices, edges)
        point_b = _station_position(route, station_b, vertices, edges)
        interval = float(station_b.distance - station_a.distance)
        factor = 0.0 if interval <= 0.0 else (
            extent - float(station_a.distance)
        ) / interval
        return point_a.lerp(point_b, factor)
    raise RuntimeError(
        f"RF19_STATION_INTERVAL_MISSING: route={terminal.route_id} alpha={alpha}"
    )


def _render_case(obj, selected, materials):
    graph, selected, _all_seams = field._source_graph(obj, selected)
    base_settings = DecalSettings(width_seam=0.8, offset=0.01)
    local_base_settings = local_decal_settings_for_source(base_settings, obj)
    plan = compile_manual_seam_decal_plan(graph, base_settings, selected)
    partition = next(
        item
        for item in plan.backend_partitions
        if item.backend == "PATCH_VORONOI"
    )
    baseline = _evaluate_manual_backend_partition(
        partition,
        local_base_settings,
        width=0.8,
        preview=True,
    ).faces
    narrow = evaluate_manual_seam_faces(
        obj, replace(base_settings, width_seam=0.4), plan, preview=True
    ).faces
    wide = evaluate_manual_seam_faces(
        obj, replace(base_settings, width_seam=0.8), plan, preview=True
    ).faces
    rows = _terminal_rows(plan)
    if not rows:
        raise RuntimeError(f"RF19_NO_TERMINALS: object={obj.name}")

    rail_vertices = {
        vertex.vertex_id: Vector(vertex.position)
        for vertex in plan.rail_plan.vertices
    }
    rail_edges = {edge.edge_id: edge for edge in plan.rail_plan.edges}
    guide_lengths = []
    for _vertex_id, terminals in rows:
        for terminal in terminals:
            for edge_id in terminal.edge_ids[:1]:
                edge = rail_edges[edge_id]
                first, second = edge.vertex_ids
                guide_lengths.append(
                    (rail_vertices[first] - rail_vertices[second]).length
                )
    span = max(max(guide_lengths, default=1.0) * 3.0, 1.0)
    view = Vector((1.1, -1.7, 0.9)).normalized()
    screen_right = Vector((-view.y, view.x, 0.0)).normalized()
    screen_up = view.cross(screen_right).normalized()
    column_offsets = (-1.25, 0.0, 1.25)
    row_offsets = tuple(
        (len(rows) - 1 - 2 * index) * 0.62 for index in range(len(rows))
    )
    camera_data = bpy.data.cameras.new("R14_FIX_CAMERA")
    camera = bpy.data.objects.new("R14_FIX_CAMERA", camera_data)
    bpy.context.scene.collection.objects.link(camera)
    camera.location = view * span * 7.5
    render_base._look_at(camera, Vector())
    camera.data.type = "ORTHO"
    camera.data.ortho_scale = span * max(3.25, 1.5 * len(rows))
    bpy.context.scene.camera = camera
    radius = span * 0.0022

    panels = (
        ("BEFORE: PERP", baseline, materials["before_fill"], materials["before_wire"], local_base_settings.width_seam * 0.5),
        ("CUT: width 0.4", narrow, materials["narrow_fill"], materials["narrow_wire"], local_decal_settings_for_source(replace(base_settings, width_seam=0.4), obj).width_seam * 0.5),
        ("CUT: width 0.8", wide, materials["wide_fill"], materials["wide_wire"], local_base_settings.width_seam * 0.5),
    )
    for row_index, (vertex_id, terminals) in enumerate(rows):
        center = obj.matrix_world @ obj.data.vertices[vertex_id].co
        neighborhood = _vertex_neighborhood(obj, vertex_id)
        for column_index, (label, faces, fill, wire, alpha) in enumerate(panels):
            offset = (
                screen_right * span * column_offsets[column_index]
                + screen_up * span * row_offsets[row_index]
            )
            _source_closeup(
                f"R14_FIX_SOURCE_{row_index}_{column_index}",
                obj,
                neighborhood,
                center,
                offset,
                materials["source"],
            )
            close_faces = tuple(
                face
                for face in faces
                if _face_is_close(face, obj, center, span * 0.82)
            )
            _wire_overlay(
                f"R14_FIX_DECAL_{row_index}_{column_index}",
                close_faces,
                obj,
                center,
                offset,
                fill,
                wire,
                radius,
            )
            for edge_id in selected:
                edge = obj.data.edges[edge_id]
                if not set(edge.vertices).intersection(neighborhood):
                    continue
                point_a = obj.matrix_world @ obj.data.vertices[edge.vertices[0]].co
                point_b = obj.matrix_world @ obj.data.vertices[edge.vertices[1]].co
                render_base._line(
                    f"R14_FIX_SELECTED_{row_index}_{column_index}_{edge_id}",
                    point_a - center + offset,
                    point_b - center + offset,
                    materials["selected"],
                    radius * 1.15,
                )
            guide_material = (
                materials["old_guide"]
                if column_index == 0
                else materials["guide"]
            )
            for terminal in terminals:
                if terminal.choice == "PERP" or terminal.route_id is None:
                    continue
                extent_position = _route_extent_position(
                    plan.rail_plan, terminal, alpha
                )
                render_base._arrow(
                    f"R14_FIX_GUIDE_{row_index}_{column_index}_{terminal.patch_id}",
                    offset,
                    obj.matrix_world @ extent_position - center + offset,
                    guide_material,
                    radius * 1.3,
                )
            render_base._label(
                f"{label}\nterminal v{vertex_id}",
                camera,
                offset + screen_up * span * 0.47 + view * span * 0.04,
                span * 0.52,
                materials["label"],
            )

    render_base._label(
        "RED = ignored guide before | GREEN = rail-cut extent after\n"
        "ORANGE = spine | colored wire = actual decal face borders",
        camera,
        screen_up * span * 1.44 + view * span * 0.04,
        span * 0.38,
        materials["label"],
    )

    light_data = bpy.data.lights.new("R14_FIX_KEY", "AREA")
    light_data.energy = 1250.0
    light_data.size = span * 5.0
    light = bpy.data.objects.new("R14_FIX_KEY", light_data)
    bpy.context.scene.collection.objects.link(light)
    light.location = view * span * 4.0

    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE_NEXT"
    scene.render.resolution_x = 1920
    scene.render.resolution_y = 1080
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.film_transparent = False
    scene.world.color = (0.008, 0.011, 0.018)
    scene.render.filepath = str(
        ARTIFACT_ROOT
        / f"decal_r14_fix_terminal_wireframe_{obj.name.replace('.', '_')}.png"
    )
    bpy.ops.render.render(write_still=True)


def main():
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    for obj in bpy.data.objects:
        obj.hide_render = True
    materials = {
        "source": render_base._material("R14_FIX_source", (0.12, 0.14, 0.19, 1.0)),
        "before_fill": render_base._material("R14_FIX_before_fill", (0.9, 0.03, 0.02, 0.18)),
        "before_wire": render_base._material("R14_FIX_before_wire", (1.0, 0.12, 0.04, 1.0), 1.5),
        "narrow_fill": render_base._material("R14_FIX_narrow_fill", (0.0, 0.55, 1.0, 0.16)),
        "narrow_wire": render_base._material("R14_FIX_narrow_wire", (0.05, 0.85, 1.0, 1.0), 1.5),
        "wide_fill": render_base._material("R14_FIX_wide_fill", (0.1, 0.22, 1.0, 0.16)),
        "wide_wire": render_base._material("R14_FIX_wide_wire", (0.25, 0.48, 1.0, 1.0), 1.5),
        "selected": render_base._material("R14_FIX_selected", (1.0, 0.48, 0.02, 1.0), 1.2),
        "guide": render_base._material("R14_FIX_guide", (0.1, 1.0, 0.2, 1.0), 1.8),
        "old_guide": render_base._material("R14_FIX_old_guide", (1.0, 0.05, 0.03, 1.0), 1.8),
        "label": render_base._material("R14_FIX_label", (0.96, 0.98, 1.0, 1.0), 2.0),
    }
    for object_name, selected in FIELD_CASES.items():
        _render_case(bpy.data.objects[object_name], selected, materials)
        for created in tuple(bpy.data.objects):
            if created.hide_render:
                continue
            bpy.data.objects.remove(created, do_unlink=True)


if __name__ == "__main__":
    main()
