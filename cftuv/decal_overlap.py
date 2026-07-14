"""Диагностический oracle физических пересечений Decal Seams.

Модуль ничего не исправляет и не использует близость поверхностей. Он сравнивает
уже lifted-полигоны только внутри явно заданного topology-domain. Это позволяет
сначала доказать межчартовый overlap, прежде чем вводить общий intrinsic atlas.
"""

from __future__ import annotations

from dataclasses import dataclass

from mathutils import Vector


@dataclass(frozen=True)
class DecalOverlap:
    face_a: int
    face_b: int
    surface_a: int
    surface_b: int
    domain_id: int
    area: float = 0.0
    crossing_length: float = 0.0


def _sub2(a, b):
    return (a[0] - b[0], a[1] - b[1])


def _add2(a, b):
    return (a[0] + b[0], a[1] + b[1])


def _mul2(a, scalar):
    return (a[0] * scalar, a[1] * scalar)


def _cross2(a, b):
    return a[0] * b[1] - a[1] * b[0]


def _polygon_area2(points):
    return 0.5 * sum(
        _cross2(points[index], points[(index + 1) % len(points)])
        for index in range(len(points))
    )


def _project(point, drop_axis):
    if drop_axis == 0:
        return (point.y, point.z)
    if drop_axis == 1:
        return (point.x, point.z)
    return (point.x, point.y)


def _drop_axis(normal):
    values = (abs(normal.x), abs(normal.y), abs(normal.z))
    return max(range(3), key=values.__getitem__)


def _point_in_triangle2(point, triangle, tolerance):
    signs = [
        _cross2(
            _sub2(triangle[(index + 1) % 3], triangle[index]),
            _sub2(point, triangle[index]),
        )
        for index in range(3)
    ]
    return min(signs) >= -tolerance or max(signs) <= tolerance


def _triangulate_positions(positions, keys, normal, tolerance):
    """Ear clipping в dominant-проекции с сохранением source keys."""

    if len(positions) < 3:
        return []
    if normal.length_squared < tolerance * tolerance:
        normal = Vector((0.0, 0.0, 1.0))
    axis = _drop_axis(normal)
    points2 = [_project(point, axis) for point in positions]
    order = list(range(len(points2)))
    if _polygon_area2(points2) < 0.0:
        order.reverse()
    triangles = []
    guard = 0
    while len(order) > 3 and guard < len(points2) * len(points2):
        guard += 1
        clipped = False
        for offset in range(len(order)):
            previous = order[offset - 1]
            current = order[offset]
            following = order[(offset + 1) % len(order)]
            triangle2 = (points2[previous], points2[current], points2[following])
            if _polygon_area2(triangle2) <= tolerance:
                continue
            if any(
                _point_in_triangle2(points2[item], triangle2, tolerance)
                for item in order
                if item not in (previous, current, following)
            ):
                continue
            indices = (previous, current, following)
            triangles.append(
                (
                    tuple(positions[index] for index in indices),
                    tuple(keys[index] for index in indices),
                )
            )
            order.pop(offset)
            clipped = True
            break
        if not clipped:
            return []
    if len(order) == 3:
        triangles.append(
            (
                tuple(positions[index] for index in order),
                tuple(keys[index] for index in order),
            )
        )
    return triangles


def _triangulate_face(face, tolerance):
    return _triangulate_positions(
        face.positions,
        face.vert_keys,
        face.surface_normal,
        tolerance,
    )


def _clip_convex(subject, clip, tolerance):
    """Sutherland-Hodgman для двух CCW convex-контуров."""

    result = list(subject)
    if _polygon_area2(clip) < 0.0:
        clip = list(reversed(clip))
    for index in range(len(clip)):
        edge_a = clip[index]
        edge_b = clip[(index + 1) % len(clip)]
        output = []
        for item in range(len(result)):
            current = result[item]
            previous = result[item - 1]
            value_current = _cross2(
                _sub2(edge_b, edge_a), _sub2(current, edge_a)
            )
            value_previous = _cross2(
                _sub2(edge_b, edge_a), _sub2(previous, edge_a)
            )
            current_inside = value_current >= -tolerance
            previous_inside = value_previous >= -tolerance
            if current_inside != previous_inside:
                denominator = value_previous - value_current
                if abs(denominator) > tolerance:
                    factor = value_previous / denominator
                    output.append(
                        _add2(previous, _mul2(_sub2(current, previous), factor))
                    )
            if current_inside:
                output.append(current)
        result = output
        if len(result) < 3:
            return []
    return result


def _triangle_normal(triangle):
    return (triangle[1] - triangle[0]).cross(triangle[2] - triangle[0])


def _coplanar_overlap_area(first, second, tolerance):
    normal_a = _triangle_normal(first)
    normal_b = _triangle_normal(second)
    if normal_a.length_squared <= tolerance * tolerance:
        return 0.0
    if normal_b.length_squared <= tolerance * tolerance:
        return 0.0
    normal_a.normalize()
    normal_b.normalize()
    if abs(normal_a.dot(normal_b)) < 1.0 - 1e-6:
        return 0.0
    if max(abs(normal_a.dot(point - first[0])) for point in second) > tolerance:
        return 0.0
    axis = _drop_axis(normal_a)
    first2 = [_project(point, axis) for point in first]
    second2 = [_project(point, axis) for point in second]
    clipped = _clip_convex(first2, second2, tolerance)
    return abs(_polygon_area2(clipped)) if clipped else 0.0


def _point_in_triangle3(point, triangle, normal, tolerance):
    signs = []
    for index in range(3):
        edge = triangle[(index + 1) % 3] - triangle[index]
        delta = point - triangle[index]
        signs.append(normal.dot(edge.cross(delta)))
    return min(signs) >= -tolerance or max(signs) <= tolerance


def _append_unique(points, candidate, tolerance):
    if all((candidate - point).length > tolerance for point in points):
        points.append(candidate)


def _edge_plane_hits(triangle, other, other_normal, tolerance, result):
    plane_d = other_normal.dot(other[0])
    for index in range(3):
        start = triangle[index]
        end = triangle[(index + 1) % 3]
        distance_start = other_normal.dot(start) - plane_d
        distance_end = other_normal.dot(end) - plane_d
        if abs(distance_start) <= tolerance and _point_in_triangle3(
            start, other, other_normal, tolerance
        ):
            _append_unique(result, start, tolerance)
        if distance_start * distance_end > tolerance * tolerance:
            continue
        denominator = distance_start - distance_end
        if abs(denominator) <= tolerance:
            continue
        factor = distance_start / denominator
        if factor < -tolerance or factor > 1.0 + tolerance:
            continue
        point = start.lerp(end, max(0.0, min(1.0, factor)))
        if _point_in_triangle3(point, other, other_normal, tolerance):
            _append_unique(result, point, tolerance)


def _crossing_segment(first, second, tolerance):
    normal_a = _triangle_normal(first)
    normal_b = _triangle_normal(second)
    if normal_a.length_squared <= tolerance * tolerance:
        return None
    if normal_b.length_squared <= tolerance * tolerance:
        return None
    normal_a.normalize()
    normal_b.normalize()
    if abs(normal_a.dot(normal_b)) > 1.0 - 1e-6:
        return None
    points = []
    _edge_plane_hits(first, second, normal_b, tolerance, points)
    _edge_plane_hits(second, first, normal_a, tolerance, points)
    if len(points) < 2:
        return None
    best = max(
        (
            ((points[a] - points[b]).length, points[a], points[b])
            for a in range(len(points))
            for b in range(a + 1, len(points))
        ),
        key=lambda item: item[0],
    )
    return best if best[0] > tolerance else None


def _point_on_segment(point, start, end, tolerance):
    direction = end - start
    length_sq = direction.length_squared
    if length_sq <= tolerance * tolerance:
        return (point - start).length <= tolerance
    factor = max(0.0, min(1.0, (point - start).dot(direction) / length_sq))
    return (point - start.lerp(end, factor)).length <= tolerance


def _is_shared_boundary(segment, face_a, face_b, tolerance):
    shared = set(face_a.vert_keys) & set(face_b.vert_keys)
    if len(shared) < 2:
        return False
    positions = {}
    for face in (face_a, face_b):
        for key, position in zip(face.vert_keys, face.positions):
            if key in shared:
                positions.setdefault(key, position)
    start, end = segment[1], segment[2]
    keys = list(positions)
    return any(
        _point_on_segment(start, positions[keys[a]], positions[keys[b]], tolerance)
        and _point_on_segment(end, positions[keys[a]], positions[keys[b]], tolerance)
        for a in range(len(keys))
        for b in range(a + 1, len(keys))
    )


def detect_network_overlaps(
    faces,
    domain_by_face,
    tolerance=1e-6,
    area_epsilon=1e-10,
):
    """Находит cross-surface overlap только внутри topology-domain.

    `domain_by_face` обязателен и индексирует выходные faces: один surface_id
    может встречаться в разных несвязных patch domains, поэтому использовать
    surface как topology identity нельзя. Отсутствие face в mapping означает,
    что detector не имеет права сравнивать её по одной лишь 3D близости.
    """

    faces = list(faces or ())
    triangles = [
        _triangulate_face(face, tolerance)
        for face in faces
    ]
    overlaps = []
    for index_a, face_a in enumerate(faces):
        domains_a = domain_by_face.get(index_a)
        if domains_a is None:
            continue
        if not isinstance(domains_a, (set, frozenset, tuple, list)):
            domains_a = (domains_a,)
        domains_a = set(domains_a)
        for index_b in range(index_a + 1, len(faces)):
            face_b = faces[index_b]
            if face_a.surface_id == face_b.surface_id:
                continue
            domains_b = domain_by_face.get(index_b)
            if domains_b is None:
                continue
            if not isinstance(domains_b, (set, frozenset, tuple, list)):
                domains_b = (domains_b,)
            common_domains = domains_a & set(domains_b)
            if not common_domains:
                continue
            area = 0.0
            crossing = 0.0
            for (triangle_a, _keys_a) in triangles[index_a]:
                for (triangle_b, _keys_b) in triangles[index_b]:
                    area += _coplanar_overlap_area(
                        triangle_a, triangle_b, tolerance
                    )
                    segment = _crossing_segment(
                        triangle_a, triangle_b, tolerance
                    )
                    if segment is not None and not _is_shared_boundary(
                        segment, face_a, face_b, tolerance * 4.0
                    ):
                        crossing = max(crossing, segment[0])
            if area <= area_epsilon and crossing <= tolerance:
                continue
            overlaps.append(
                DecalOverlap(
                    face_a=index_a,
                    face_b=index_b,
                    surface_a=face_a.surface_id,
                    surface_b=face_b.surface_id,
                    domain_id=min(common_domains),
                    area=area,
                    crossing_length=crossing,
                )
            )
    return overlaps


def map_faces_to_owner_domains(faces, domains, offset=0.0, tolerance=1e-6):
    """Назначает lifted faces всем owner domains, которые они покрывают.

    Сравнение выполняется только в offset-плоскости конкретной source face.
    Один output face может пересекать несколько соседних domains, поэтому
    mapping хранит множество IDs на индекс face.
    """

    faces = list(faces or ())
    network_triangles = [
        _triangulate_face(face, tolerance)
        for face in faces
    ]
    result = {}
    for domain in domains or ():
        owner_triangles = []
        for owner in domain.faces:
            normal = owner.normal.normalized()
            positions = [point + normal * offset for point in owner.points]
            keys = [("owner", owner.face_index, index) for index in range(len(positions))]
            owner_triangles.extend(
                triangle
                for triangle, _keys in _triangulate_positions(
                    positions,
                    keys,
                    normal,
                    tolerance,
                )
            )
        if not owner_triangles:
            continue
        for face_index, face in enumerate(faces):
            matched = False
            for triangle, _keys in network_triangles[face_index]:
                if matched:
                    break
                for owner_triangle in owner_triangles:
                    if _coplanar_overlap_area(
                        triangle,
                        owner_triangle,
                        tolerance,
                    ) > tolerance * tolerance:
                        matched = True
                        break
            if matched:
                result.setdefault(face_index, set()).add(domain.domain_id)
    return result
