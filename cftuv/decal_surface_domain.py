"""Patch-scoped intrinsic domain для overlap-gated Decal Seams.

Producer передаёт чистый snapshot owner faces и selected edges. Legacy network
остаётся основным путём; intrinsic rebuild вызывается только когда физический
overlap-oracle нашёл пересечение внутри одного topology-domain.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt

from mathutils import Vector

from .constants import DECAL_WELD_DISTANCE
from .decal_network import _NetworkFace, _lift_position, build_seam_network_faces
from .decal_overlap import detect_network_overlaps, map_faces_to_owner_domains


@dataclass(frozen=True)
class OwnerDomainFace:
    face_index: int
    vert_indices: tuple[int, ...]
    points: tuple[Vector, ...]
    normal: Vector


@dataclass(frozen=True)
class OwnerDomainEdge:
    edge_index: int
    vert_a: int
    vert_b: int
    face_index: int


@dataclass(frozen=True)
class OwnerFaceDomain:
    domain_id: int
    patch_id: int
    faces: tuple[OwnerDomainFace, ...]
    source_edges: tuple[OwnerDomainEdge, ...]


@dataclass(frozen=True)
class ChartCut:
    domain_id: int
    face_a: int
    face_b: int
    edge: tuple[int, int]


@dataclass
class IntrinsicSeamBuild:
    faces: list[_NetworkFace]
    domain_by_face: dict[int, set[int]]
    chart_cuts: list[ChartCut]


@dataclass
class GatedSeamBuild:
    faces: list[_NetworkFace]
    intrinsic_used: bool
    baseline_overlaps: list
    remaining_overlaps: list
    chart_cuts: list[ChartCut]


@dataclass
class _ChartRun:
    vert_indices: list[int]
    points: list[Vector]
    segment_normals_a: list[Vector]
    segment_normals_b: list[Vector]
    segment_convexities: list[float]
    segment_edge_indices: list[int]

    @property
    def is_closed(self):
        return (
            len(self.vert_indices) > 2
            and self.vert_indices[0] == self.vert_indices[-1]
        )


@dataclass
class _ClipVertex:
    point: tuple[float, float]
    u_frac: float
    v_length: float
    source_key: tuple | None = None


def _sub2(a, b):
    return (a[0] - b[0], a[1] - b[1])


def _add2(a, b):
    return (a[0] + b[0], a[1] + b[1])


def _mul2(a, scalar):
    return (a[0] * scalar, a[1] * scalar)


def _cross2(a, b):
    return a[0] * b[1] - a[1] * b[0]


def _dot2(a, b):
    return a[0] * b[0] + a[1] * b[1]


def _length2(a):
    return sqrt(_dot2(a, a))


def _polygon_area2(points):
    return 0.5 * sum(
        _cross2(points[index], points[(index + 1) % len(points)])
        for index in range(len(points))
    )


def _edge_key(a, b):
    return (a, b) if a < b else (b, a)


def _face_edges(face):
    for index in range(len(face.vert_indices)):
        yield (
            face.vert_indices[index],
            face.vert_indices[(index + 1) % len(face.vert_indices)],
        )


def _seed_face_uv(face):
    origin = face.points[0]
    axis_x = None
    for point in face.points[1:]:
        delta = point - origin
        if delta.length_squared > 1e-12:
            axis_x = delta.normalized()
            break
    if axis_x is None:
        return None
    normal = face.normal.normalized()
    axis_y = normal.cross(axis_x)
    if axis_y.length_squared < 1e-12:
        return None
    axis_y.normalize()
    return {
        vert: ((point - origin).dot(axis_x), (point - origin).dot(axis_y))
        for vert, point in zip(face.vert_indices, face.points)
    }


def _centroid2(face_uv):
    count = len(face_uv)
    return (
        sum(point[0] for point in face_uv.values()) / count,
        sum(point[1] for point in face_uv.values()) / count,
    )


def _unfold_neighbor(current, neighbor, current_uv, shared):
    vert_a, vert_b = shared
    points = dict(zip(neighbor.vert_indices, neighbor.points))
    point_a3 = points[vert_a]
    edge3 = points[vert_b] - point_a3
    length3 = edge3.length
    if length3 < 1e-12:
        return None
    edge3 /= length3

    point_a2 = current_uv[vert_a]
    edge2 = _sub2(current_uv[vert_b], point_a2)
    length2 = _length2(edge2)
    if length2 < 1e-12:
        return None
    edge2 = _mul2(edge2, 1.0 / length2)
    perp2 = (-edge2[1], edge2[0])
    current_side = _cross2(edge2, _sub2(_centroid2(current_uv), point_a2))
    target_sign = -1.0 if current_side >= 0.0 else 1.0

    result = {}
    for vert, point3 in zip(neighbor.vert_indices, neighbor.points):
        delta = point3 - point_a3
        along = delta.dot(edge3)
        lateral = (delta - edge3 * along).length
        result[vert] = _add2(
            point_a2,
            _add2(
                _mul2(edge2, along),
                _mul2(perp2, target_sign * lateral),
            ),
        )
    result[vert_a] = current_uv[vert_a]
    result[vert_b] = current_uv[vert_b]
    return result


def _unfold_domain(domain, tolerance):
    """Разворачивает adjacency spanning tree и помечает cycle cuts."""

    faces = {face.face_index: face for face in domain.faces}
    if not faces:
        return None
    faces_by_edge = {}
    for face in domain.faces:
        for edge in _face_edges(face):
            faces_by_edge.setdefault(_edge_key(*edge), []).append(face.face_index)

    seed = min(faces)
    seed_uv = _seed_face_uv(faces[seed])
    if seed_uv is None:
        return None
    face_uv = {seed: seed_uv}
    queue = [seed]
    cuts = set()
    while queue:
        current_index = queue.pop(0)
        current = faces[current_index]
        for edge in _face_edges(current):
            shared = _edge_key(*edge)
            for neighbor_index in faces_by_edge.get(shared, ()):
                if neighbor_index == current_index:
                    continue
                candidate = _unfold_neighbor(
                    current,
                    faces[neighbor_index],
                    face_uv[current_index],
                    shared,
                )
                if candidate is None:
                    return None
                if neighbor_index not in face_uv:
                    face_uv[neighbor_index] = candidate
                    queue.append(neighbor_index)
                    continue
                error = max(
                    _length2(
                        _sub2(face_uv[neighbor_index][vert], candidate[vert])
                    )
                    for vert in shared
                )
                if error > tolerance:
                    cuts.add(
                        (
                            min(current_index, neighbor_index),
                            max(current_index, neighbor_index),
                            shared,
                        )
                    )
    if len(face_uv) != len(faces):
        return None
    chart_cuts = [
        ChartCut(domain.domain_id, face_a, face_b, edge)
        for face_a, face_b, edge in sorted(cuts)
    ]
    return face_uv, faces_by_edge, chart_cuts


def _oriented_source_edges(domain, face_uv):
    faces = {face.face_index: face for face in domain.faces}
    result = []
    for edge in domain.source_edges:
        uv = face_uv.get(edge.face_index)
        face = faces.get(edge.face_index)
        if uv is None or face is None:
            continue
        point_a = uv[edge.vert_a]
        point_b = uv[edge.vert_b]
        center = _centroid2(uv)
        if _cross2(_sub2(point_b, point_a), _sub2(center, point_a)) < 0.0:
            result.append((edge.edge_index, edge.vert_b, edge.vert_a))
        else:
            result.append((edge.edge_index, edge.vert_a, edge.vert_b))
    return result


def _chart_point(domain, face_uv, vert):
    for edge in domain.source_edges:
        if vert not in (edge.vert_a, edge.vert_b):
            continue
        uv = face_uv.get(edge.face_index)
        if uv is not None and vert in uv:
            return uv[vert]
    return None


def _canonicalize_run(verts, edge_indices):
    closed = len(verts) > 2 and verts[0] == verts[-1]
    if not closed:
        if verts[0] <= verts[-1]:
            return verts, edge_indices, False
        return list(reversed(verts)), list(reversed(edge_indices)), True

    core = list(verts[:-1])
    candidates = []
    orientations = (
        (core, list(edge_indices), False),
        ([core[0], *reversed(core[1:])], list(reversed(edge_indices)), True),
    )
    for base_verts, base_edges, reversed_path in orientations:
        for start in range(len(base_verts)):
            rotated_verts = base_verts[start:] + base_verts[:start]
            rotated_edges = base_edges[start:] + base_edges[:start]
            candidates.append(
                (tuple(rotated_verts), rotated_verts, rotated_edges, reversed_path)
            )
    _key, canonical, edges, reversed_path = min(
        candidates,
        key=lambda item: item[0],
    )
    return canonical + [canonical[0]], edges, reversed_path


def _build_chart_runs(domain, face_uv):
    edges = _oriented_source_edges(domain, face_uv)
    if not edges:
        return []
    outgoing = {}
    incident = {}
    for index, (_edge, start, end) in enumerate(edges):
        outgoing.setdefault(start, []).append(index)
        incident.setdefault(start, []).append(index)
        incident.setdefault(end, []).append(index)

    visited = set()
    paths = []

    def walk(first_index):
        path = []
        current = first_index
        while current not in visited:
            visited.add(current)
            path.append(current)
            _edge, _start, end = edges[current]
            if len(incident.get(end, ())) != 2:
                break
            candidates = [
                item for item in outgoing.get(end, ()) if item not in visited
            ]
            if len(candidates) != 1:
                break
            current = candidates[0]
        return path

    for index, (_edge, start, _end) in enumerate(edges):
        if len(incident.get(start, ())) != 2 and index not in visited:
            paths.append(walk(index))
    for index in range(len(edges)):
        if index not in visited:
            paths.append(walk(index))

    normal = Vector((0.0, 0.0, 1.0))
    zero = Vector((0.0, 0.0, 0.0))
    runs = []
    for path in paths:
        source_edge, start, _end = edges[path[0]]
        verts = [start]
        edge_indices = []
        for edge_index in path:
            source_edge, segment_start, segment_end = edges[edge_index]
            if verts[-1] != segment_start:
                break
            verts.append(segment_end)
            edge_indices.append(source_edge)
        if len(verts) < 2:
            continue
        verts, edge_indices, reversed_path = _canonicalize_run(
            verts,
            edge_indices,
        )
        points2 = [_chart_point(domain, face_uv, vert) for vert in verts]
        if any(point is None for point in points2):
            continue
        if reversed_path:
            normals_a = [normal.copy() for _ in edge_indices]
            normals_b = [zero.copy() for _ in edge_indices]
        else:
            normals_a = [zero.copy() for _ in edge_indices]
            normals_b = [normal.copy() for _ in edge_indices]
        runs.append(
            _ChartRun(
                vert_indices=verts,
                points=[Vector((point[0], point[1], 0.0)) for point in points2],
                segment_normals_a=normals_a,
                segment_normals_b=normals_b,
                segment_convexities=[0.0 for _ in edge_indices],
                segment_edge_indices=edge_indices,
            )
        )
    return runs


def _point_in_triangle(point, triangle, tolerance=1e-9):
    signs = [
        _cross2(
            _sub2(triangle[(index + 1) % 3], triangle[index]),
            _sub2(point, triangle[index]),
        )
        for index in range(3)
    ]
    return min(signs) >= -tolerance or max(signs) <= tolerance


def _triangulate_indices(points):
    if len(points) < 3:
        return []
    order = list(range(len(points)))
    if _polygon_area2(points) < 0.0:
        order.reverse()
    triangles = []
    guard = 0
    while len(order) > 3 and guard < len(points) * len(points):
        guard += 1
        clipped = False
        for offset in range(len(order)):
            previous = order[offset - 1]
            current = order[offset]
            following = order[(offset + 1) % len(order)]
            triangle = (points[previous], points[current], points[following])
            if _polygon_area2(triangle) <= 1e-10:
                continue
            if any(
                _point_in_triangle(points[item], triangle)
                for item in order
                if item not in (previous, current, following)
            ):
                continue
            triangles.append((previous, current, following))
            order.pop(offset)
            clipped = True
            break
        if not clipped:
            return []
    if len(order) == 3:
        triangles.append(tuple(order))
    return triangles


def _lerp_clip_vertex(a, b, factor):
    source_key = None
    if factor <= 1e-9:
        source_key = a.source_key
    elif factor >= 1.0 - 1e-9:
        source_key = b.source_key
    return _ClipVertex(
        point=_add2(a.point, _mul2(_sub2(b.point, a.point), factor)),
        u_frac=a.u_frac + (b.u_frac - a.u_frac) * factor,
        v_length=a.v_length + (b.v_length - a.v_length) * factor,
        source_key=source_key,
    )


def _clip_halfplane(vertices, edge_a, edge_b):
    if not vertices:
        return []
    output = []
    for index, current in enumerate(vertices):
        previous = vertices[index - 1]
        value_current = _cross2(
            _sub2(edge_b, edge_a), _sub2(current.point, edge_a)
        )
        value_previous = _cross2(
            _sub2(edge_b, edge_a), _sub2(previous.point, edge_a)
        )
        current_inside = value_current >= -1e-9
        previous_inside = value_previous >= -1e-9
        if current_inside != previous_inside:
            denominator = value_previous - value_current
            if abs(denominator) > 1e-12:
                output.append(
                    _lerp_clip_vertex(
                        previous,
                        current,
                        value_previous / denominator,
                    )
                )
        if current_inside:
            output.append(current)
    return output


def _clip_triangle(vertices, triangle):
    if _polygon_area2(triangle) < 0.0:
        triangle = tuple(reversed(triangle))
    result = vertices
    for index in range(3):
        result = _clip_halfplane(
            result,
            triangle[index],
            triangle[(index + 1) % 3],
        )
        if len(result) < 3:
            return []
    return result


def _barycentric(point, triangle):
    a, b, c = triangle
    denominator = _cross2(_sub2(b, a), _sub2(c, a))
    if abs(denominator) < 1e-12:
        return None
    weight_b = _cross2(_sub2(point, a), _sub2(c, a)) / denominator
    weight_c = _cross2(_sub2(b, a), _sub2(point, a)) / denominator
    return (1.0 - weight_b - weight_c, weight_b, weight_c)


def _edge_lift(point2, domain, face_uv, faces_by_edge, offset, tolerance):
    faces = {face.face_index: face for face in domain.faces}
    for edge, face_indices in faces_by_edge.items():
        face_index = face_indices[0]
        uv = face_uv[face_index]
        point_a2, point_b2 = uv[edge[0]], uv[edge[1]]
        delta = _sub2(point_b2, point_a2)
        length_sq = _dot2(delta, delta)
        if length_sq < 1e-12:
            continue
        factor = max(
            0.0,
            min(1.0, _dot2(_sub2(point2, point_a2), delta) / length_sq),
        )
        nearest = _add2(point_a2, _mul2(delta, factor))
        if _length2(_sub2(point2, nearest)) > tolerance:
            continue
        endpoint = None
        if factor * sqrt(length_sq) <= tolerance:
            endpoint = edge[0]
        elif (1.0 - factor) * sqrt(length_sq) <= tolerance:
            endpoint = edge[1]
        if endpoint is not None:
            incident = [face for face in domain.faces if endpoint in face.vert_indices]
            position = next(
                point
                for face in incident
                for vert, point in zip(face.vert_indices, face.points)
                if vert == endpoint
            )
            return (
                ("domain_vertex", domain.domain_id, endpoint),
                _lift_position(position, [face.normal for face in incident], offset),
            )
        owner = faces[face_index]
        points = dict(zip(owner.vert_indices, owner.points))
        base = points[edge[0]].lerp(points[edge[1]], factor)
        return (
            ("domain_edge", domain.domain_id, edge, round(factor, 6)),
            _lift_position(
                base,
                [faces[index].normal for index in face_indices],
                offset,
            ),
        )
    return None


def _map_fragment(
    vertices,
    domain,
    owner,
    owner_triangle,
    owner_triangle2,
    face_uv,
    faces_by_edge,
    spine_lift,
    offset,
    tolerance,
):
    keys = []
    positions = []
    u_fracs = []
    v_lengths = []
    points3 = tuple(owner.points[index] for index in owner_triangle)
    for vertex in vertices:
        if vertex.source_key and vertex.source_key[0] == "sv":
            source_vert = vertex.source_key[1]
            key = ("sv", source_vert)
            position = spine_lift[source_vert]
        else:
            edge_value = _edge_lift(
                vertex.point,
                domain,
                face_uv,
                faces_by_edge,
                offset,
                tolerance,
            )
            if edge_value is not None:
                key, position = edge_value
            else:
                weights = _barycentric(vertex.point, owner_triangle2)
                if weights is None:
                    return None
                base = sum(
                    (point * weight for point, weight in zip(points3, weights)),
                    Vector((0.0, 0.0, 0.0)),
                )
                position = base + owner.normal.normalized() * offset
                key = (
                    "domain",
                    domain.domain_id,
                    round(vertex.point[0], 6),
                    round(vertex.point[1], 6),
                )
        if keys and keys[-1] == key:
            continue
        keys.append(key)
        positions.append(position.copy())
        u_fracs.append(vertex.u_frac)
        v_lengths.append(vertex.v_length)
    if len(keys) > 2 and keys[0] == keys[-1]:
        keys.pop()
        positions.pop()
        u_fracs.pop()
        v_lengths.pop()
    if len(keys) < 3:
        return None
    return _NetworkFace(
        surface_id=owner.face_index,
        surface_normal=owner.normal.copy(),
        vert_keys=keys,
        positions=positions,
        u_fracs=u_fracs,
        v_lengths=v_lengths,
    )


def _clip_chart_faces(
    chart_faces,
    domain,
    face_uv,
    faces_by_edge,
    spine_lift,
    offset,
    tolerance,
):
    result = []
    for owner in domain.faces:
        owner_points2 = [face_uv[owner.face_index][vert] for vert in owner.vert_indices]
        owner_triangles = _triangulate_indices(owner_points2)
        if not owner_triangles:
            return None
        for chart_face in chart_faces:
            chart_points2 = [(point.x, point.y) for point in chart_face.positions]
            chart_triangles = _triangulate_indices(chart_points2)
            if not chart_triangles:
                continue
            for chart_triangle in chart_triangles:
                subject = [
                    _ClipVertex(
                        point=chart_points2[index],
                        u_frac=chart_face.u_fracs[index],
                        v_length=chart_face.v_lengths[index],
                        source_key=chart_face.vert_keys[index],
                    )
                    for index in chart_triangle
                ]
                for owner_triangle in owner_triangles:
                    owner_triangle2 = tuple(
                        owner_points2[index] for index in owner_triangle
                    )
                    clipped = _clip_triangle(subject, owner_triangle2)
                    if len(clipped) < 3:
                        continue
                    mapped = _map_fragment(
                        clipped,
                        domain,
                        owner,
                        owner_triangle,
                        owner_triangle2,
                        face_uv,
                        faces_by_edge,
                        spine_lift,
                        offset,
                        tolerance,
                    )
                    if mapped is not None:
                        result.append(mapped)
    return result


def build_intrinsic_seam_faces(domains, offset, width, tolerance=None):
    domains = list(domains or ())
    if not domains:
        return None
    tolerance = max(DECAL_WELD_DISTANCE, tolerance or 0.0)

    normals_by_vert = {}
    positions_by_vert = {}
    for domain in domains:
        faces = {face.face_index: face for face in domain.faces}
        for edge in domain.source_edges:
            owner = faces.get(edge.face_index)
            if owner is None:
                continue
            points = dict(zip(owner.vert_indices, owner.points))
            for vert in (edge.vert_a, edge.vert_b):
                normals_by_vert.setdefault(vert, []).append(owner.normal)
                positions_by_vert.setdefault(vert, points[vert])
    spine_lift = {
        vert: _lift_position(positions_by_vert[vert], normals, offset)
        for vert, normals in normals_by_vert.items()
    }

    result = []
    domain_by_face = {}
    chart_cuts = []
    for domain in domains:
        unfolded = _unfold_domain(domain, tolerance * 2.0)
        if unfolded is None:
            return None
        face_uv, faces_by_edge, cuts = unfolded
        chart_cuts.extend(cuts)
        runs = _build_chart_runs(domain, face_uv)
        if not runs:
            continue
        chart_faces = build_seam_network_faces(runs, 0.0, width)
        mapped = _clip_chart_faces(
            chart_faces,
            domain,
            face_uv,
            faces_by_edge,
            spine_lift,
            offset,
            tolerance * 2.0,
        )
        if mapped is None:
            return None
        for face in mapped:
            domain_by_face[len(result)] = {domain.domain_id}
            result.append(face)
    return IntrinsicSeamBuild(result, domain_by_face, chart_cuts)


def build_overlap_gated_seam_faces(
    runs,
    domains,
    offset,
    width,
    arc_tolerance=None,
):
    """Возвращает legacy bit-for-bit, пока oracle не доказал overlap."""

    legacy = build_seam_network_faces(runs, offset, width, arc_tolerance)
    if not legacy or not domains:
        return GatedSeamBuild(legacy, False, [], [], [])
    tolerance = max(DECAL_WELD_DISTANCE, arc_tolerance or 0.0)
    legacy_domains = map_faces_to_owner_domains(
        legacy,
        domains,
        offset=offset,
        tolerance=tolerance * 2.0,
    )
    baseline = detect_network_overlaps(
        legacy,
        legacy_domains,
        tolerance=tolerance * 2.0,
    )
    if not baseline:
        return GatedSeamBuild(legacy, False, [], [], [])

    intrinsic = build_intrinsic_seam_faces(
        domains,
        offset,
        width,
        tolerance=tolerance,
    )
    if intrinsic is None or not intrinsic.faces:
        return GatedSeamBuild(legacy, False, baseline, baseline, [])
    remaining = detect_network_overlaps(
        intrinsic.faces,
        intrinsic.domain_by_face,
        tolerance=tolerance * 2.0,
    )
    if remaining:
        return GatedSeamBuild(
            legacy,
            False,
            baseline,
            remaining,
            intrinsic.chart_cuts,
        )
    return GatedSeamBuild(
        intrinsic.faces,
        True,
        baseline,
        [],
        intrinsic.chart_cuts,
    )
