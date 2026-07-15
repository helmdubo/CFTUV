"""Patch-bounded segment-Voronoi backend для Decal Seams.

Диаграмма строится один раз на owner patch по выбранным boundary segments.
Во время modal drag меняется только фронт ``distance(site) <= width / 2``.
Ячейки заранее пересечены с точной триангуляцией PatchGraph, поэтому крылья
не выходят за mesh boundary, не пересекаются внутри patch и меняют топологию
в тот же момент, когда фронты встречаются.

``pyvoronoi`` изолирован в этом модуле. Если wheel недоступен или patch не
планарен, вызывающий код явно возвращается к legacy seam-network backend.
"""

from dataclasses import dataclass
from math import atan2, cos, pi, sin, sqrt

from mathutils import Vector

from .constants import DECAL_WELD_DISTANCE
from .decal_network import (
    _NetworkFace,
    _lift_position,
    _polygon_area2,
    _segment_point_distance2,
)

try:
    import pyvoronoi
except ImportError:  # Blender может открыть старый файл без установленного wheel.
    pyvoronoi = None


_DIAGRAM_SCALE = 100000
_GEOMETRY_EPS = 1e-9


@dataclass(frozen=True)
class _PatchVoronoiSite:
    patch_id: int
    edge_index: int
    vert_a: int
    vert_b: int
    source_a: Vector
    source_b: Vector
    point_a: tuple[float, float]
    point_b: tuple[float, float]
    arc_start: float
    segment_length: float
    uv_sign: float


@dataclass(frozen=True)
class _PatchVoronoiAtom:
    site_index: int
    polygon: tuple[tuple[float, float], ...]
    max_site_distance: float


@dataclass(frozen=True)
class _PatchVoronoiSurface:
    patch_id: int
    origin: Vector
    normal: Vector
    basis_u: Vector
    basis_v: Vector
    sites: tuple[_PatchVoronoiSite, ...]
    atoms: tuple[_PatchVoronoiAtom, ...]


@dataclass(frozen=True)
class PatchVoronoiPlan:
    """Width-independent segment-Voronoi diagrams одного modal invoke."""

    offset: float
    surfaces: tuple[_PatchVoronoiSurface, ...]
    lifted_vertices: dict[int, Vector]
    max_lateral_lift_ratio: float


def patch_voronoi_available():
    return pyvoronoi is not None


def _dist2(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return sqrt(dx * dx + dy * dy)


def _dedupe_polygon(points, tolerance=1e-8):
    result = []
    for point in points:
        point = (float(point[0]), float(point[1]))
        if result and _dist2(result[-1], point) <= tolerance:
            continue
        result.append(point)
    if len(result) > 1 and _dist2(result[0], result[-1]) <= tolerance:
        result.pop()
    if len(result) < 3:
        return []
    return result


def _line_clip_intersection(point_a, point_b, clip_a, clip_b):
    """Пересечение subject edge с бесконечной прямой clip edge."""

    sx = point_b[0] - point_a[0]
    sy = point_b[1] - point_a[1]
    cx = clip_b[0] - clip_a[0]
    cy = clip_b[1] - clip_a[1]
    denom = sx * cy - sy * cx
    if abs(denom) < 1e-15:
        return point_a
    qx = clip_a[0] - point_a[0]
    qy = clip_a[1] - point_a[1]
    t = (qx * cy - qy * cx) / denom
    return (point_a[0] + sx * t, point_a[1] + sy * t)


def _clip_to_convex(points, clip_polygon):
    """Sutherland-Hodgman: произвольный polygon ∩ convex polygon."""

    clip = list(clip_polygon)
    if _polygon_area2(clip) < 0.0:
        clip.reverse()
    output = list(points)
    for index in range(len(clip)):
        if not output:
            break
        clip_a = clip[index]
        clip_b = clip[(index + 1) % len(clip)]

        def inside(point):
            return (
                (clip_b[0] - clip_a[0]) * (point[1] - clip_a[1])
                - (clip_b[1] - clip_a[1]) * (point[0] - clip_a[0])
            ) >= -1e-10

        source = output
        output = []
        previous = source[-1]
        previous_inside = inside(previous)
        for current in source:
            current_inside = inside(current)
            if current_inside != previous_inside:
                output.append(
                    _line_clip_intersection(
                        previous, current, clip_a, clip_b
                    )
                )
            if current_inside:
                output.append(current)
            previous = current
            previous_inside = current_inside
    return _dedupe_polygon(output)


def _clip_to_triangle(points, triangle):
    return _clip_to_convex(points, triangle)


def _capsule_polygon(point_a, point_b, radius, half_segments):
    """Low-res convex approximation distance(segment) <= radius.

    Диаграмма отвечает за collision topology, capsule — только за текущий
    distance-front. Малое фиксированное число дуговых сегментов не создаёт
    прежние fan-вееры и не зависит от плотности исходного mesh.
    """

    dx = point_b[0] - point_a[0]
    dy = point_b[1] - point_a[1]
    length = sqrt(dx * dx + dy * dy)
    if length <= _GEOMETRY_EPS:
        return []
    angle = atan2(dy, dx)
    points = []
    for index in range(half_segments + 1):
        theta = angle - pi * 0.5 + pi * index / half_segments
        points.append(
            (
                point_b[0] + cos(theta) * radius,
                point_b[1] + sin(theta) * radius,
            )
        )
    for index in range(half_segments + 1):
        theta = angle + pi * 0.5 + pi * index / half_segments
        points.append(
            (
                point_a[0] + cos(theta) * radius,
                point_a[1] + sin(theta) * radius,
            )
        )
    return _dedupe_polygon(points)


def _edge_points(diagram, edges, vertices, edge_index, curve_step):
    edge = edges[edge_index]
    if edge.start < 0 or edge.end < 0:
        return None
    start = (vertices[edge.start].X, vertices[edge.start].Y)
    end = (vertices[edge.end].X, vertices[edge.end].Y)
    if edge.is_linear:
        return [start, end]
    points = [
        (float(point[0]), float(point[1]))
        for point in diagram.DiscretizeCurvedEdge(
            edge_index, curve_step, 0.0001
        )
    ]
    if not points:
        return [start, end]
    if _dist2(points[0], start) > _dist2(points[-1], start):
        points.reverse()
    points[0] = start
    points[-1] = end
    return points


def _cell_polygon(diagram, edges, vertices, cell, curve_step):
    """Восстанавливает ordered boundary конечной pyvoronoi cell."""

    polygon = []
    for edge_index in cell.edges:
        edge_points = _edge_points(
            diagram, edges, vertices, edge_index, curve_step
        )
        if edge_points is None:
            return None
        if not polygon:
            polygon.extend(edge_points)
            continue
        if _dist2(polygon[-1], edge_points[0]) <= 1e-7:
            polygon.extend(edge_points[1:])
        elif _dist2(polygon[-1], edge_points[-1]) <= 1e-7:
            polygon.extend(reversed(edge_points[:-1]))
        else:
            # Cell.edges обычно уже циклически упорядочены. Этот fallback
            # сохраняет контур при небольшой ошибке округления backend.
            polygon.extend(edge_points)
    polygon = _dedupe_polygon(polygon)
    if len(polygon) < 3 or abs(_polygon_area2(polygon)) <= 1e-12:
        return None
    if _polygon_area2(polygon) < 0.0:
        polygon.reverse()
    return polygon


def _project(point, origin, basis_u, basis_v):
    delta = point - origin
    return (delta.dot(basis_u), delta.dot(basis_v))


def _patch_is_planar(node):
    if not node.mesh_verts or not node.mesh_tris:
        return False
    normal = node.normal.normalized()
    origin = node.centroid
    projected = [_project(point, origin, node.basis_u, node.basis_v) for point in node.mesh_verts]
    if not projected:
        return False
    min_x = min(point[0] for point in projected)
    max_x = max(point[0] for point in projected)
    min_y = min(point[1] for point in projected)
    max_y = max(point[1] for point in projected)
    extent = max(max_x - min_x, max_y - min_y, 1.0)
    tolerance = max(DECAL_WELD_DISTANCE * 0.25, extent * 1e-5)
    return all(abs((point - origin).dot(normal)) <= tolerance for point in node.mesh_verts)


def _collect_patch_sites(graph, selected_edges):
    """Сохраняет patch/chain identity вместо сведения сети к голым runs."""

    raw_by_patch = {}
    uses_by_edge = {}
    normals_by_vert = {}
    positions_by_vert = {}
    for patch_id in sorted(graph.nodes):
        node = graph.nodes[patch_id]
        seen_edges = set()
        patch_sites = []
        for boundary_loop in node.boundary_loops:
            for chain in boundary_loop.chains:
                if len(chain.vert_cos) < 2:
                    continue
                arc = 0.0
                segment_count = len(chain.edge_indices)
                for segment_index in range(segment_count):
                    next_index = segment_index + 1
                    if next_index >= len(chain.vert_cos):
                        if not chain.is_closed:
                            continue
                        next_index = 0
                    source_a = chain.vert_cos[segment_index].copy()
                    source_b = chain.vert_cos[next_index].copy()
                    length = (source_b - source_a).length
                    edge_index = int(chain.edge_indices[segment_index])
                    if (
                        edge_index in selected_edges
                        and edge_index not in seen_edges
                        and length > _GEOMETRY_EPS
                    ):
                        vert_a = int(chain.vert_indices[segment_index])
                        vert_b = int(chain.vert_indices[next_index])
                        raw = {
                            "patch_id": patch_id,
                            "edge_index": edge_index,
                            "vert_a": vert_a,
                            "vert_b": vert_b,
                            "source_a": source_a,
                            "source_b": source_b,
                            "arc_start": arc,
                            "segment_length": length,
                        }
                        patch_sites.append(raw)
                        uses_by_edge.setdefault(edge_index, []).append(raw)
                        for vert_index, position in (
                            (vert_a, source_a),
                            (vert_b, source_b),
                        ):
                            positions_by_vert.setdefault(vert_index, position.copy())
                            normals_by_vert.setdefault(vert_index, []).append(
                                node.normal.copy()
                            )
                        seen_edges.add(edge_index)
                    arc += length
        if patch_sites:
            raw_by_patch[patch_id] = patch_sites

    for edge_index, uses in uses_by_edge.items():
        for use_index, raw in enumerate(
            sorted(uses, key=lambda item: (item["patch_id"], item["vert_a"], item["vert_b"]))
        ):
            raw["uv_sign"] = -1.0 if use_index % 2 == 0 else 1.0
    return raw_by_patch, normals_by_vert, positions_by_vert


def _compile_surface(node, raw_sites):
    origin = node.centroid.copy()
    normal = node.normal.normalized()
    basis_u = node.basis_u.normalized()
    basis_v = node.basis_v.normalized()
    sites = []
    for raw in raw_sites:
        sites.append(
            _PatchVoronoiSite(
                patch_id=node.patch_id,
                edge_index=raw["edge_index"],
                vert_a=raw["vert_a"],
                vert_b=raw["vert_b"],
                source_a=raw["source_a"],
                source_b=raw["source_b"],
                point_a=_project(raw["source_a"], origin, basis_u, basis_v),
                point_b=_project(raw["source_b"], origin, basis_u, basis_v),
                arc_start=raw["arc_start"],
                segment_length=raw["segment_length"],
                uv_sign=raw["uv_sign"],
            )
        )

    triangles = []
    for tri in node.mesh_tris:
        points = [
            _project(node.mesh_verts[index], origin, basis_u, basis_v)
            for index in tri
        ]
        if abs(_polygon_area2(points)) > 1e-12:
            triangles.append(points)
    if not triangles:
        return None

    all_points = [point for triangle in triangles for point in triangle]
    min_x = min(point[0] for point in all_points)
    max_x = max(point[0] for point in all_points)
    min_y = min(point[1] for point in all_points)
    max_y = max(point[1] for point in all_points)
    diagonal = sqrt((max_x - min_x) ** 2 + (max_y - min_y) ** 2)
    margin = max(10.0, diagonal * 8.0)
    guard = (
        (min_x - margin, min_y - margin),
        (max_x + margin, min_y - margin),
        (max_x + margin, max_y + margin),
        (min_x - margin, max_y + margin),
    )

    diagram = pyvoronoi.Pyvoronoi(_DIAGRAM_SCALE)
    for site in sites:
        diagram.AddSegment([site.point_a, site.point_b])
    for index in range(4):
        diagram.AddSegment([guard[index], guard[(index + 1) % 4]])
    diagram.Construct()

    curve_step = max(diagonal / 48.0, DECAL_WELD_DISTANCE * 2.0)
    diagram_edges = diagram.GetEdges()
    diagram_vertices = diagram.GetVertices()
    atoms = []
    for cell in diagram.GetCells():
        if (
            cell.site < 0
            or cell.site >= len(sites)
            or cell.is_open
            or cell.is_degenerate
        ):
            continue
        polygon = _cell_polygon(
            diagram, diagram_edges, diagram_vertices, cell, curve_step
        )
        if polygon is None:
            continue
        site = sites[cell.site]
        for triangle in triangles:
            clipped = _clip_to_triangle(polygon, triangle)
            if len(clipped) < 3 or abs(_polygon_area2(clipped)) <= 1e-10:
                continue
            if _polygon_area2(clipped) < 0.0:
                clipped.reverse()
            max_distance = max(
                _segment_point_distance2(site.point_a, site.point_b, point)[0]
                for point in clipped
            )
            atoms.append(
                _PatchVoronoiAtom(
                    site_index=int(cell.site),
                    polygon=tuple(clipped),
                    max_site_distance=max_distance,
                )
            )
    if not atoms:
        return None
    return _PatchVoronoiSurface(
        patch_id=node.patch_id,
        origin=origin,
        normal=normal,
        basis_u=basis_u,
        basis_v=basis_v,
        sites=tuple(sites),
        atoms=tuple(atoms),
    )


def compile_patch_voronoi_plan(graph, selected_edge_indices, offset):
    """Компилирует диаграммы touched planar patches; иначе отдаёт fallback."""

    if pyvoronoi is None:
        return None
    selected_edges = {int(edge_index) for edge_index in selected_edge_indices or ()}
    if not selected_edges:
        return None
    raw_by_patch, normals_by_vert, positions_by_vert = _collect_patch_sites(
        graph, selected_edges
    )
    if not raw_by_patch:
        return None
    # Один backend владеет всей selection. Частичное смешение с legacy
    # вернуло бы двойные крылья на границе planar/non-planar patches.
    if any(not _patch_is_planar(graph.nodes[patch_id]) for patch_id in raw_by_patch):
        return None

    surfaces = []
    for patch_id in sorted(raw_by_patch):
        surface = _compile_surface(graph.nodes[patch_id], raw_by_patch[patch_id])
        if surface is None:
            return None
        surfaces.append(surface)
    lifted_vertices = {
        vert_index: _lift_position(
            positions_by_vert[vert_index], normals, float(offset)
        )
        for vert_index, normals in normals_by_vert.items()
    }
    max_lateral_lift_ratio = 0.0
    if abs(float(offset)) > _GEOMETRY_EPS:
        for surface in surfaces:
            for site in surface.sites:
                for vert_index, source in (
                    (site.vert_a, site.source_a),
                    (site.vert_b, site.source_b),
                ):
                    delta = lifted_vertices[vert_index] - source
                    lateral = delta - surface.normal * float(offset)
                    max_lateral_lift_ratio = max(
                        max_lateral_lift_ratio,
                        lateral.length / abs(float(offset)),
                    )
    return PatchVoronoiPlan(
        offset=float(offset),
        surfaces=tuple(surfaces),
        lifted_vertices=lifted_vertices,
        max_lateral_lift_ratio=max_lateral_lift_ratio,
    )


def _position_and_key(
    plan, surface, site, point, lift_scale, effective_offset
):
    distance, t = _segment_point_distance2(site.point_a, site.point_b, point)
    spine_eps = max(DECAL_WELD_DISTANCE * 0.25, site.segment_length * 1e-7)
    if distance <= spine_eps:
        if t <= 1e-7:
            position = site.source_a.lerp(
                plan.lifted_vertices[site.vert_a], lift_scale
            )
            return position, ("pv-sv", site.vert_a)
        if t >= 1.0 - 1e-7:
            position = site.source_b.lerp(
                plan.lifted_vertices[site.vert_b], lift_scale
            )
            return position, ("pv-sv", site.vert_b)
        start = site.source_a.lerp(
            plan.lifted_vertices[site.vert_a], lift_scale
        )
        end = site.source_b.lerp(
            plan.lifted_vertices[site.vert_b], lift_scale
        )
        return start.lerp(end, t), ("pv-se", site.edge_index, round(t, 7))
    position = (
        surface.origin
        + surface.basis_u * point[0]
        + surface.basis_v * point[1]
        + surface.normal * effective_offset
    )
    quantum = max(DECAL_WELD_DISTANCE * 0.25, 1e-7)
    return position, (
        "pv",
        surface.patch_id,
        round(point[0] / quantum),
        round(point[1] / quantum),
    )


def _component_signed_area(plan, surface, site, component, lift_scale):
    """Signed area после shared-rail lift для заданного offset-scale."""

    positions = []
    used_keys = set()
    effective_offset = plan.offset * lift_scale
    for point in component:
        position, key = _position_and_key(
            plan,
            surface,
            site,
            point,
            lift_scale,
            effective_offset,
        )
        if key in used_keys:
            continue
        used_keys.add(key)
        positions.append(position)
    if len(positions) < 3:
        return None
    area_vector = Vector((0.0, 0.0, 0.0))
    origin = positions[0]
    for index in range(1, len(positions) - 1):
        area_vector += (positions[index] - origin).cross(
            positions[index + 1] - origin
        )
    return area_vector.dot(surface.normal) * 0.5


def _orientation_safe_lift_scale(plan, pending, desired_scale):
    """Аналитический общий offset-scale без перевёрнутых wing faces.

    Каждая 3D-позиция affine по scale, следовательно signed area face —
    квадратный полином. Три измерения дают его точно и заменяют дорогой
    бинарный rebuild всей сети на один линейный проход.
    """

    safe_fraction = 1.0
    for surface, site, component in pending:
        area_0 = _component_signed_area(
            plan, surface, site, component, 0.0
        )
        area_half = _component_signed_area(
            plan, surface, site, component, desired_scale * 0.5
        )
        area_1 = _component_signed_area(
            plan, surface, site, component, desired_scale
        )
        if area_0 is None or area_half is None or area_1 is None:
            continue
        # A(t) = qa*t² + qb*t + qc, где t = scale / desired_scale.
        qc = area_0
        qa = 2.0 * area_1 + 2.0 * area_0 - 4.0 * area_half
        qb = area_1 - area_0 - qa
        roots = []
        if abs(qa) <= 1e-18:
            if abs(qb) > 1e-18:
                roots.append(-qc / qb)
        else:
            discriminant = qb * qb - 4.0 * qa * qc
            if discriminant >= 0.0:
                root_delta = sqrt(discriminant)
                roots.extend(
                    (
                        (-qb - root_delta) / (2.0 * qa),
                        (-qb + root_delta) / (2.0 * qa),
                    )
                )
        for root in sorted(value for value in roots if 0.0 < value <= 1.0):
            probe = min(1.0, root + max(1e-5, (1.0 - root) * 1e-3))
            after_root = qa * probe * probe + qb * probe + qc
            if after_root < 0.0:
                safe_fraction = min(safe_fraction, root)
                break
    if safe_fraction >= 1.0:
        return desired_scale
    # Запас не даёт float32/weld вернуть face точно на нулевой Jacobian.
    return desired_scale * safe_fraction * 0.99


def evaluate_patch_voronoi_plan(plan, width, preview=False):
    """Двигает distance-front внутри статических patch Voronoi cells."""

    alpha = max(1e-6, float(width) * 0.5)
    keep_eps = max(1e-9, alpha * 1e-7)
    lateral_at_full_offset = (
        abs(plan.offset) * plan.max_lateral_lift_ratio
    )
    if lateral_at_full_offset > _GEOMETRY_EPS:
        # Общий offset-rail не должен пересекать внешний фронт узкого крыла.
        # Масштаб един для всей selection: shared vertices остаются общими,
        # а все owner planes сохраняют одинаковый фактический offset.
        lift_scale = min(1.0, alpha * 0.75 / lateral_at_full_offset)
    else:
        lift_scale = 1.0
    pending = []
    # Preview и confirm обязаны иметь один и тот же collision contour.
    # Дополнительная финальная дуговая точка меняла cell intersections после
    # отпускания мыши: митеры визуально раскрывались и расходились с крылом.
    capsule_segments = 2
    for surface in plan.surfaces:
        for atom in surface.atoms:
            site = surface.sites[atom.site_index]
            if atom.max_site_distance <= alpha + keep_eps:
                components = [list(atom.polygon)]
            else:
                capsule = _capsule_polygon(
                    site.point_a,
                    site.point_b,
                    alpha,
                    capsule_segments,
                )
                clipped = _clip_to_convex(atom.polygon, capsule)
                components = [clipped] if clipped else []
            for component in components:
                component = _dedupe_polygon(component, tolerance=1e-7)
                if len(component) < 3 or abs(_polygon_area2(component)) <= 1e-10:
                    continue
                if _polygon_area2(component) < 0.0:
                    component.reverse()
                pending.append((surface, site, component))

    lift_scale = _orientation_safe_lift_scale(plan, pending, lift_scale)
    effective_offset = plan.offset * lift_scale
    faces = []
    emitted_faces = set()
    for surface, site, component in pending:
        vert_keys = []
        positions = []
        u_fracs = []
        v_lengths = []
        used_keys = set()
        for point in component:
            position, key = _position_and_key(
                plan,
                surface,
                site,
                point,
                lift_scale,
                effective_offset,
            )
            # Triangle boundaries и pyvoronoi endpoint-cells могут
            # дать две почти одинаковые 2D точки, которые после
            # conformal lift закономерно становятся одной вершиной.
            if key in used_keys:
                continue
            used_keys.add(key)
            distance, t = _segment_point_distance2(
                site.point_a, site.point_b, point
            )
            vert_keys.append(key)
            positions.append(position)
            u_fracs.append(
                site.uv_sign * max(0.0, min(1.0, distance / alpha))
            )
            v_lengths.append(site.arc_start + t * site.segment_length)
        if len(vert_keys) < 3:
            continue
        face_identity = frozenset(vert_keys)
        if face_identity in emitted_faces:
            continue
        emitted_faces.add(face_identity)
        faces.append(
            _NetworkFace(
                surface_id=surface.patch_id,
                surface_normal=surface.normal.copy(),
                vert_keys=vert_keys,
                positions=positions,
                u_fracs=u_fracs,
                v_lengths=v_lengths,
            )
        )
    return faces
