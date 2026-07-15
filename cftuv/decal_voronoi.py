"""Patch-bounded segment-Voronoi backend для Decal Seams.

Диаграмма строится один раз на owner patch по выбранным boundary segments.
Во время modal drag segment- и endpoint-cells обрезаются динамическими
extrusion polygons текущей ширины. Ячейки заранее пересечены с boundary-доменом
patch, поэтому крылья не выходят за mesh boundary, не пересекаются внутри patch
и меняют топологию в тот же момент, когда фронты встречаются. Внутренняя
триангуляция домена используется только для clipping и сваривается обратно:
topology исходного mesh не должна отпечатываться на итоговой декали.

``pyvoronoi`` изолирован в этом модуле. Если wheel недоступен или patch не
планарен, вызывающий код явно возвращается к legacy seam-network backend.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import atan2, pi, sqrt, tau

from mathutils import Vector

try:
    from mathutils.geometry import tessellate_polygon as _tessellate_polygon
except ImportError:  # Unit tests используют минимальный mathutils stub.
    _tessellate_polygon = None

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
_ACUTE_SPLIT_ANGLE = pi / 3.0


class _CornerPolicy(str, Enum):
    """Intrinsic corner policy; не зависит от способа lift на owner mesh."""

    CAP = "CAP"
    MITER = "MITER"
    KITE = "KITE"
    ACUTE_SPLIT = "ACUTE_SPLIT"
    BEVEL = "BEVEL"
    JUNCTION = "JUNCTION"


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
    inward_normal: tuple[float, float]


@dataclass(frozen=True)
class CornerSpec:
    """Decal-local endpoint topology одного planar patch."""

    vert_index: int
    point: tuple[float, float]
    incident_sites: tuple[int, ...]
    ordered_sites: tuple[int, ...]
    policy: _CornerPolicy
    turn_sign: float
    interior_angle: float
    extrusion_angle: float
    is_convex: bool
    miter_ratio: float


@dataclass(frozen=True)
class _PatchVoronoiAtom:
    site_index: int
    fragments: tuple[tuple[tuple[float, float], ...], ...]
    cell_kind: str
    corner_index: int = -1
    source_category: int = 0


@dataclass(frozen=True)
class _PatchVoronoiSurface:
    patch_id: int
    origin: Vector
    normal: Vector
    basis_u: Vector
    basis_v: Vector
    sites: tuple[_PatchVoronoiSite, ...]
    corners: tuple[CornerSpec, ...]
    atoms: tuple[_PatchVoronoiAtom, ...]
    site_grid_size: float
    site_grid: dict[tuple[int, int], tuple[int, ...]]


@dataclass(frozen=True)
class PatchVoronoiPlan:
    """Width-independent segment-Voronoi diagrams одного modal invoke."""

    offset: float
    surfaces: tuple[_PatchVoronoiSurface, ...]
    lifted_vertices: dict[int, Vector]
    max_lateral_lift_ratio: float


@dataclass(frozen=True)
class _DecalArrangementFace:
    """Cell face после общей surface-level edge conformity."""

    surface: _PatchVoronoiSurface
    site: _PatchVoronoiSite
    points: tuple[tuple[float, float], ...]
    crop: _CropComponent


@dataclass(frozen=True)
class DecalArrangement:
    """Математическая decal-сеть до preview/final BMesh adapters."""

    faces: tuple[_DecalArrangementFace, ...]
    inserted_stations: int


@dataclass(frozen=True)
class _CropComponent:
    """Один convex runtime crop с семантикой, живущей до NetworkFace."""

    kind: str
    side: str
    points: tuple[tuple[float, float], ...]
    uv_anchors: tuple[tuple[float, float], ...] = ()
    v_origin: float = 0.0


@dataclass(frozen=True)
class _PendingArrangementFace:
    surface: _PatchVoronoiSurface
    site: _PatchVoronoiSite
    points: tuple[tuple[float, float], ...]
    crop: _CropComponent


@dataclass(frozen=True)
class _JunctionPort:
    """Открытый core-to-rail луч математической decal-сети."""

    surface_id: int
    edge_index: int
    surface_normal: Vector
    core_position: Vector
    outer_key: tuple
    outer_position: Vector
    outer_u: float
    outer_v: float
    core_v: float


def patch_voronoi_available():
    return pyvoronoi is not None


def _dist2(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return sqrt(dx * dx + dy * dy)


def _sub2(point_a, point_b):
    return (point_a[0] - point_b[0], point_a[1] - point_b[1])


def _dot2(vector_a, vector_b):
    return vector_a[0] * vector_b[0] + vector_a[1] * vector_b[1]


def _cross2(vector_a, vector_b):
    return vector_a[0] * vector_b[1] - vector_a[1] * vector_b[0]


def _norm2(vector):
    length = sqrt(_dot2(vector, vector))
    if length <= _GEOMETRY_EPS:
        return None
    return (vector[0] / length, vector[1] / length)


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


def _simplify_collinear_polygon(points, tolerance):
    """Убирает вычислительные stations на прямой, не меняя silhouette."""

    result = _dedupe_polygon(points, tolerance=tolerance)
    changed = True
    while changed and len(result) > 3:
        changed = False
        simplified = []
        count = len(result)
        for index, point in enumerate(result):
            previous = result[(index - 1) % count]
            following = result[(index + 1) % count]
            dx = following[0] - previous[0]
            dy = following[1] - previous[1]
            length = sqrt(dx * dx + dy * dy)
            if length <= tolerance:
                changed = True
                continue
            deviation = abs(
                dx * (point[1] - previous[1])
                - dy * (point[0] - previous[0])
            ) / length
            between = (
                (point[0] - previous[0]) * (point[0] - following[0])
                + (point[1] - previous[1]) * (point[1] - following[1])
            ) <= tolerance * tolerance
            if deviation <= tolerance and between:
                changed = True
                continue
            simplified.append(point)
        if len(simplified) < 3:
            break
        result = simplified
    return result


def _polygon_is_simple(points, tolerance=1e-8):
    """Отсекает contours, которые BMesh превратил бы в пересекающиеся tris."""

    count = len(points)
    if count < 3:
        return False
    quantum = max(tolerance, 1e-12)
    keys = [
        (round(point[0] / quantum), round(point[1] / quantum))
        for point in points
    ]
    if len(set(keys)) != count:
        return False

    def orientation(point_a, point_b, point_c):
        return (
            (point_b[0] - point_a[0]) * (point_c[1] - point_a[1])
            - (point_b[1] - point_a[1]) * (point_c[0] - point_a[0])
        )

    def on_segment(point_a, point_b, point):
        return (
            min(point_a[0], point_b[0]) - tolerance
            <= point[0]
            <= max(point_a[0], point_b[0]) + tolerance
            and min(point_a[1], point_b[1]) - tolerance
            <= point[1]
            <= max(point_a[1], point_b[1]) + tolerance
        )

    def intersects(point_a, point_b, point_c, point_d):
        turns = (
            orientation(point_a, point_b, point_c),
            orientation(point_a, point_b, point_d),
            orientation(point_c, point_d, point_a),
            orientation(point_c, point_d, point_b),
        )
        if (
            turns[0] * turns[1] < -tolerance * tolerance
            and turns[2] * turns[3] < -tolerance * tolerance
        ):
            return True
        return (
            (abs(turns[0]) <= tolerance and on_segment(point_a, point_b, point_c))
            or (abs(turns[1]) <= tolerance and on_segment(point_a, point_b, point_d))
            or (abs(turns[2]) <= tolerance and on_segment(point_c, point_d, point_a))
            or (abs(turns[3]) <= tolerance and on_segment(point_c, point_d, point_b))
        )

    for edge_index in range(count):
        point_a = points[edge_index]
        point_b = points[(edge_index + 1) % count]
        for other_index in range(edge_index + 1, count):
            if other_index in (
                edge_index,
                (edge_index + 1) % count,
                (edge_index - 1) % count,
            ):
                continue
            if edge_index == 0 and other_index == count - 1:
                continue
            point_c = points[other_index]
            point_d = points[(other_index + 1) % count]
            if intersects(point_a, point_b, point_c, point_d):
                return False
    return True


def _polygon_is_convex(points, tolerance=1e-9):
    sign = 0
    for index, point in enumerate(points):
        point_a = points[(index - 1) % len(points)]
        point_b = points[(index + 1) % len(points)]
        turn = (
            (point[0] - point_a[0]) * (point_b[1] - point[1])
            - (point[1] - point_a[1]) * (point_b[0] - point[0])
        )
        if abs(turn) <= tolerance:
            continue
        current_sign = 1 if turn > 0.0 else -1
        if sign and current_sign != sign:
            return False
        sign = current_sign
    return sign != 0


def _convex_fragment_decomposition(polygons, tolerance):
    """Укрупняет triangle clips только когда их union остаётся convex."""

    result = [list(polygon) for polygon in polygons]
    quantum = max(tolerance, 1e-10)

    def point_key(point):
        return (round(point[0] / quantum), round(point[1] / quantum))

    while True:
        edge_owner = {}
        merge_pair = None
        for polygon_index, polygon in enumerate(result):
            for index, point in enumerate(polygon):
                other = polygon[(index + 1) % len(polygon)]
                undirected = tuple(sorted((point_key(point), point_key(other))))
                previous = edge_owner.get(undirected)
                if previous is not None and previous != polygon_index:
                    pair = tuple(sorted((previous, polygon_index)))
                    first = result[pair[0]]
                    second = result[pair[1]]
                    hull = _convex_hull(first + second)
                    source_area = abs(_polygon_area2(first)) + abs(
                        _polygon_area2(second)
                    )
                    area_tolerance = max(
                        tolerance * tolerance * 4.0,
                        source_area * 1e-7,
                    )
                    if (
                        len(hull) >= 3
                        and abs(abs(_polygon_area2(hull)) - source_area)
                        <= area_tolerance
                    ):
                        merge_pair = pair, hull
                        break
                else:
                    edge_owner[undirected] = polygon_index
            if merge_pair is not None:
                break
        if merge_pair is None:
            return result
        (first_index, second_index), hull = merge_pair
        result[first_index] = hull
        del result[second_index]


def _merge_polygon_fragments(fragments, tolerance=1e-7):
    """Собирает triangle-clips одной cell обратно в цельные contours.

    Internal edges совпадают попарно в противоположных направлениях и
    удаляются. Компоненты, касающиеся только вершиной, намеренно не слипаются.
    Если union содержит hole, возвращаем исходные fragments этой компоненты:
    Blender face не может хранить внутренний контур без разбиения.
    """

    normalized = []
    for fragment in fragments:
        polygon = _dedupe_polygon(fragment, tolerance=tolerance)
        if (
            len(polygon) < 3
            or abs(_polygon_area2(polygon)) <= tolerance * tolerance
        ):
            continue
        if _polygon_area2(polygon) < 0.0:
            polygon.reverse()
        normalized.append(polygon)
    if len(normalized) <= 1:
        return normalized

    quantum = max(tolerance, 1e-10)

    def point_key(point):
        return (
            round(point[0] / quantum),
            round(point[1] / quantum),
        )

    parents = list(range(len(normalized)))

    def find(index):
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    def union(first, second):
        first_root = find(first)
        second_root = find(second)
        if first_root != second_root:
            parents[second_root] = first_root

    edge_owners = {}
    for fragment_index, polygon in enumerate(normalized):
        for index, point in enumerate(polygon):
            other = polygon[(index + 1) % len(polygon)]
            key_a = point_key(point)
            key_b = point_key(other)
            undirected = tuple(sorted((key_a, key_b)))
            previous_owner = edge_owners.get(undirected)
            if previous_owner is None:
                edge_owners[undirected] = fragment_index
            else:
                union(fragment_index, previous_owner)

    groups = {}
    for fragment_index in range(len(normalized)):
        groups.setdefault(find(fragment_index), []).append(fragment_index)

    merged = []
    for group_indices in groups.values():
        boundary_edges = {}
        representatives = {}
        for fragment_index in group_indices:
            polygon = normalized[fragment_index]
            for index, point in enumerate(polygon):
                other = polygon[(index + 1) % len(polygon)]
                key_a = point_key(point)
                key_b = point_key(other)
                representatives.setdefault(key_a, point)
                representatives.setdefault(key_b, other)
                reverse = (key_b, key_a)
                if reverse in boundary_edges:
                    del boundary_edges[reverse]
                else:
                    boundary_edges[(key_a, key_b)] = True

        outgoing = {}
        for key_a, key_b in boundary_edges:
            outgoing.setdefault(key_a, []).append(key_b)
        unused = set(boundary_edges)
        loops = []
        while unused:
            start_edge = next(iter(unused))
            start_key, current_key = start_edge
            unused.remove(start_edge)
            loop_keys = [start_key]
            previous_key = start_key
            guard = len(boundary_edges) + 1
            while current_key != start_key and guard > 0:
                loop_keys.append(current_key)
                candidates = [
                    next_key
                    for next_key in outgoing.get(current_key, ())
                    if (current_key, next_key) in unused
                ]
                if not candidates:
                    break
                # В vertex-contact могут сходиться несколько contours одной
                # edge-connected группы. Продолжаем по ближайшему clockwise
                # half-edge от обратного входящего направления: interior
                # каждого CCW contour остаётся слева, а касающиеся loops не
                # сшиваются случайной лексикографической хордой.
                current_point = representatives[current_key]
                previous_point = representatives[previous_key]
                reverse = (
                    previous_point[0] - current_point[0],
                    previous_point[1] - current_point[1],
                )

                def clockwise_turn(next_key):
                    next_point = representatives[next_key]
                    outgoing = (
                        next_point[0] - current_point[0],
                        next_point[1] - current_point[1],
                    )
                    cross = reverse[0] * outgoing[1] - reverse[1] * outgoing[0]
                    dot = reverse[0] * outgoing[0] + reverse[1] * outgoing[1]
                    return (-atan2(cross, dot)) % tau, next_key

                next_key = min(candidates, key=clockwise_turn)
                unused.remove((current_key, next_key))
                previous_key = current_key
                current_key = next_key
                guard -= 1
            if current_key != start_key or len(loop_keys) < 3:
                loops = []
                break
            polygon = [representatives[key] for key in loop_keys]
            polygon = _simplify_collinear_polygon(polygon, tolerance)
            if len(polygon) >= 3 and abs(_polygon_area2(polygon)) > tolerance * tolerance:
                loops.append(polygon)

        # Negative loop is a real hole. Preserve the correct triangulated
        # representation instead of illegally filling it with one ngon.
        if (
            not loops
            or any(_polygon_area2(loop) < 0.0 for loop in loops)
            or any(not _polygon_is_simple(loop, tolerance) for loop in loops)
            or any(not _polygon_is_convex(loop, tolerance) for loop in loops)
        ):
            merged.extend(
                _convex_fragment_decomposition(
                    [normalized[index] for index in group_indices],
                    tolerance,
                )
            )
            continue
        merged.extend(loops)
    return merged


def _patch_domain_triangles(node, origin, basis_u, basis_v):
    """Триангулирует только boundary loops, не topology owner faces."""

    domain_loops = []
    if _tessellate_polygon is not None:
        ordered_loops = sorted(
            (loop for loop in node.boundary_loops if len(loop.vert_cos) >= 3),
            key=lambda loop: (
                int(getattr(loop, "depth", 0)),
                str(getattr(loop, "kind", "")),
            ),
        )
        for boundary_loop in ordered_loops:
            points = _dedupe_polygon(
                [
                    _project(point, origin, basis_u, basis_v)
                    for point in boundary_loop.vert_cos
                ]
            )
            if len(points) < 3:
                continue
            is_hole = int(getattr(boundary_loop, "depth", 0)) % 2 == 1
            area = _polygon_area2(points)
            if (not is_hole and area < 0.0) or (is_hole and area > 0.0):
                points.reverse()
            domain_loops.append(points)

    if domain_loops:
        flat_points = [point for loop in domain_loops for point in loop]
        vector_loops = [
            [Vector((point[0], point[1], 0.0)) for point in loop]
            for loop in domain_loops
        ]
        try:
            triangle_indices = _tessellate_polygon(vector_loops)
        except (RuntimeError, ValueError):
            triangle_indices = []
        triangles = []
        for indices in triangle_indices:
            if len(indices) != 3:
                continue
            triangle = [flat_points[int(index)] for index in indices]
            if abs(_polygon_area2(triangle)) <= 1e-12:
                continue
            if _polygon_area2(triangle) < 0.0:
                triangle.reverse()
            triangles.append(triangle)
        if triangles:
            return triangles

    # Compatibility fallback для старых serialized graphs/unit tests.
    triangles = []
    for tri in node.mesh_tris:
        points = [
            _project(node.mesh_verts[index], origin, basis_u, basis_v)
            for index in tri
        ]
        if abs(_polygon_area2(points)) > 1e-12:
            if _polygon_area2(points) < 0.0:
                points.reverse()
            triangles.append(points)
    return triangles


def _point_in_triangle(point, triangle, tolerance=1e-10):
    """Проверяет принадлежность point треугольнику в patch space."""

    signs = []
    for index in range(3):
        point_a = triangle[index]
        point_b = triangle[(index + 1) % 3]
        signs.append(
            (point_b[0] - point_a[0]) * (point[1] - point_a[1])
            - (point_b[1] - point_a[1]) * (point[0] - point_a[0])
        )
    return min(signs) >= -tolerance or max(signs) <= tolerance


def _point_in_domain(point, triangles):
    return any(_point_in_triangle(point, triangle) for triangle in triangles)


def _inward_site_normal(point_a, point_b, triangles, probe_distance):
    """Выбирает нормаль segment в сторону owner patch, а не по winding chain."""

    dx = point_b[0] - point_a[0]
    dy = point_b[1] - point_a[1]
    length = sqrt(dx * dx + dy * dy)
    if length <= _GEOMETRY_EPS:
        return (0.0, 0.0)
    left = (-dy / length, dx / length)
    midpoint = ((point_a[0] + point_b[0]) * 0.5, (point_a[1] + point_b[1]) * 0.5)
    left_probe = (
        midpoint[0] + left[0] * probe_distance,
        midpoint[1] + left[1] * probe_distance,
    )
    right_probe = (
        midpoint[0] - left[0] * probe_distance,
        midpoint[1] - left[1] * probe_distance,
    )
    left_inside = _point_in_domain(left_probe, triangles)
    right_inside = _point_in_domain(right_probe, triangles)
    if left_inside != right_inside:
        return left if left_inside else (-left[0], -left[1])

    # На очень коротком edge probe может пересечь соседний угол. Ближайшая
    # owner triangle даёт устойчивый fallback без зависимости от winding.
    best_centroid = None
    best_distance = float("inf")
    for triangle in triangles:
        centroid = (
            sum(point[0] for point in triangle) / 3.0,
            sum(point[1] for point in triangle) / 3.0,
        )
        distance = _dist2(midpoint, centroid)
        if distance < best_distance:
            best_distance = distance
            best_centroid = centroid
    if best_centroid is not None:
        towards_owner = (
            best_centroid[0] - midpoint[0],
            best_centroid[1] - midpoint[1],
        )
        if towards_owner[0] * left[0] + towards_owner[1] * left[1] < 0.0:
            return (-left[0], -left[1])
    return left


def _line_intersection(point_a, direction_a, point_b, direction_b):
    denominator = (
        direction_a[0] * direction_b[1]
        - direction_a[1] * direction_b[0]
    )
    if abs(denominator) <= 1e-12:
        return None
    delta = (point_b[0] - point_a[0], point_b[1] - point_a[1])
    factor = (
        delta[0] * direction_b[1] - delta[1] * direction_b[0]
    ) / denominator
    return (
        point_a[0] + direction_a[0] * factor,
        point_a[1] + direction_a[1] * factor,
    )


def _convex_hull(points):
    """Минимальный deterministic hull для runtime extrusion crop."""

    unique = sorted({(float(point[0]), float(point[1])) for point in points})
    if len(unique) < 3:
        return []

    def cross(origin, point_a, point_b):
        return (
            (point_a[0] - origin[0]) * (point_b[1] - origin[1])
            - (point_a[1] - origin[1]) * (point_b[0] - origin[0])
        )

    lower = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 1e-12:
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 1e-12:
            upper.pop()
        upper.append(point)
    return _dedupe_polygon(lower[:-1] + upper[:-1])


def _segment_crop_polygon(site, alpha):
    """Extrusion polygon segment-cell: прямой strip без endpoint fan."""

    dx = site.point_b[0] - site.point_a[0]
    dy = site.point_b[1] - site.point_a[1]
    length = max(site.segment_length, _GEOMETRY_EPS)
    normal = (-dy / length * alpha, dx / length * alpha)
    return [
        (site.point_a[0] + normal[0], site.point_a[1] + normal[1]),
        (site.point_b[0] + normal[0], site.point_b[1] + normal[1]),
        (site.point_b[0] - normal[0], site.point_b[1] - normal[1]),
        (site.point_a[0] - normal[0], site.point_a[1] - normal[1]),
    ]


def _corner_offset_lines(surface, corner, alpha):
    """Две ordered offset-линии intrinsic corner."""

    offset_lines = []
    for site_index in corner.ordered_sites:
        site = surface.sites[site_index]
        if site.vert_a == corner.vert_index:
            direction = (
                (site.point_b[0] - corner.point[0]) / site.segment_length,
                (site.point_b[1] - corner.point[1]) / site.segment_length,
            )
        else:
            direction = (
                (site.point_a[0] - corner.point[0]) / site.segment_length,
                (site.point_a[1] - corner.point[1]) / site.segment_length,
            )
        offset_point = (
            corner.point[0] + site.inward_normal[0] * alpha,
            corner.point[1] + site.inward_normal[1] * alpha,
        )
        offset_lines.append((offset_point, direction))
    return offset_lines


def _kite_crop_polygon(surface, corner, alpha):
    """Один realtime kite из двух endpoint triangles convex corner."""

    offset_lines = _corner_offset_lines(surface, corner, alpha)
    if len(offset_lines) != 2:
        return []
    intersection = _line_intersection(
        offset_lines[0][0],
        offset_lines[0][1],
        offset_lines[1][0],
        offset_lines[1][1],
    )
    if (
        intersection is None
        or _dist2(corner.point, intersection) > alpha * 8.0
    ):
        return _convex_hull(
            [corner.point, offset_lines[0][0], offset_lines[1][0]]
        )
    return _convex_hull(
        [
            corner.point,
            offset_lines[0][0],
            intersection,
            offset_lines[1][0],
        ]
    )


def _corner_arc_origin(surface, corner):
    values = []
    for site_index in corner.ordered_sites:
        site = surface.sites[site_index]
        values.append(
            site.arc_start
            if site.vert_a == corner.vert_index
            else site.arc_start + site.segment_length
        )
    return sum(values) / len(values) if values else 0.0


def _crop_component_from_anchors(kind, side, anchors, v_origin=0.0):
    """Сохраняет UV anchors после deterministic hull ordering."""

    polygon = _convex_hull([point for point, _uv in anchors])
    if len(polygon) < 3:
        return None
    quantum = max(DECAL_WELD_DISTANCE * 0.1, 1e-9)

    def key(point):
        return (
            round(point[0] / quantum),
            round(point[1] / quantum),
        )

    uv_by_point = {key(point): uv for point, uv in anchors}
    return _CropComponent(
        kind=kind,
        side=side,
        points=tuple(polygon),
        uv_anchors=tuple(uv_by_point[key(point)] for point in polygon),
        v_origin=v_origin,
    )


def _acute_crop_components(surface, corner, alpha):
    """Разделяет острый kite на inner/outer faces во время drag."""

    offset_lines = _corner_offset_lines(surface, corner, alpha)
    if len(offset_lines) != 2:
        return ()
    intersection = _line_intersection(
        offset_lines[0][0],
        offset_lines[0][1],
        offset_lines[1][0],
        offset_lines[1][1],
    )
    if intersection is None:
        fallback = _crop_component_from_anchors(
            _CornerPolicy.BEVEL.value,
            "",
            (
                (corner.point, (0.0, 0.0)),
                (offset_lines[0][0], (-1.0, alpha)),
                (offset_lines[1][0], (1.0, alpha)),
            ),
            _corner_arc_origin(surface, corner),
        )
        return (fallback,) if fallback is not None else ()

    cap_a = offset_lines[0][0]
    cap_b = offset_lines[1][0]
    chord_midpoint = (
        (cap_a[0] + cap_b[0]) * 0.5,
        (cap_a[1] + cap_b[1]) * 0.5,
    )
    inner_height = _dist2(corner.point, chord_midpoint)
    outer_height = _dist2(intersection, chord_midpoint)
    orientation = 1.0 if corner.turn_sign >= 0.0 else -1.0
    v_origin = _corner_arc_origin(surface, corner)
    inner = _crop_component_from_anchors(
        _CornerPolicy.ACUTE_SPLIT.value,
        "INNER",
        (
            (corner.point, (0.0, 0.0)),
            (cap_a, (-orientation, inner_height)),
            (cap_b, (orientation, inner_height)),
        ),
        v_origin,
    )
    outer = _crop_component_from_anchors(
        _CornerPolicy.ACUTE_SPLIT.value,
        "OUTER",
        (
            (cap_a, (-orientation, 0.0)),
            (intersection, (0.0, outer_height)),
            (cap_b, (orientation, 0.0)),
        ),
        v_origin,
    )
    return tuple(component for component in (inner, outer) if component)


def _corner_crop_components(surface, corner, alpha):
    if corner.policy == _CornerPolicy.ACUTE_SPLIT:
        return _acute_crop_components(surface, corner, alpha)
    polygon = _corner_crop_polygon(surface, corner, alpha)
    if len(polygon) < 3:
        return ()
    return (
        _CropComponent(
            kind=corner.policy.value,
            side="",
            points=tuple(polygon),
        ),
    )


def _crop_component_uv(crop, point):
    """Affine UV внутри triangle component; None оставляет site UV."""

    if len(crop.points) != 3 or len(crop.uv_anchors) != 3:
        return None
    point_a, point_b, point_c = crop.points
    denominator = _cross2(
        _sub2(point_b, point_a), _sub2(point_c, point_a)
    )
    if abs(denominator) <= 1e-12:
        return None
    weight_b = _cross2(
        _sub2(point, point_a), _sub2(point_c, point_a)
    ) / denominator
    weight_c = _cross2(
        _sub2(point_b, point_a), _sub2(point, point_a)
    ) / denominator
    weight_a = 1.0 - weight_b - weight_c
    u_value = sum(
        weight * uv[0]
        for weight, uv in zip(
            (weight_a, weight_b, weight_c), crop.uv_anchors
        )
    )
    v_value = crop.v_origin + sum(
        weight * uv[1]
        for weight, uv in zip(
            (weight_a, weight_b, weight_c), crop.uv_anchors
        )
    )
    return u_value, v_value


def _corner_crop_polygon(surface, corner, alpha):
    """Runtime endpoint extrusion polygon выбранной corner policy."""

    point = corner.point
    if len(corner.incident_sites) != 2:
        return [
            (point[0] - alpha, point[1] - alpha),
            (point[0] + alpha, point[1] - alpha),
            (point[0] + alpha, point[1] + alpha),
            (point[0] - alpha, point[1] + alpha),
        ]
    if corner.policy == _CornerPolicy.KITE:
        return _kite_crop_polygon(surface, corner, alpha)

    offset_lines = _corner_offset_lines(surface, corner, alpha)

    intersection = _line_intersection(
        offset_lines[0][0],
        offset_lines[0][1],
        offset_lines[1][0],
        offset_lines[1][1],
    )
    points = [point, offset_lines[0][0], offset_lines[1][0]]
    if (
        intersection is not None
        and _dist2(point, intersection) <= alpha * 8.0
        and corner.policy != _CornerPolicy.BEVEL
    ):
        points.append(intersection)
    return _convex_hull(points)


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


def _triangulate_cell_polygon(points):
    """Разбивает Voronoi-cell до clipping, сохраняя convex atoms."""

    polygon = _dedupe_polygon(points)
    if len(polygon) < 3 or not _polygon_is_simple(polygon):
        return []
    if _polygon_area2(polygon) < 0.0:
        polygon.reverse()
    if _polygon_is_convex(polygon):
        return [polygon]

    remaining = list(range(len(polygon)))
    triangles = []
    guard = len(polygon) * len(polygon)
    while len(remaining) > 3 and guard > 0:
        ear_found = False
        for position, current_index in enumerate(remaining):
            previous_index = remaining[position - 1]
            next_index = remaining[(position + 1) % len(remaining)]
            triangle = [
                polygon[previous_index],
                polygon[current_index],
                polygon[next_index],
            ]
            if _polygon_area2(triangle) <= 1e-12:
                continue
            has_inner_point = False
            for candidate_index in remaining:
                if candidate_index in (
                    previous_index,
                    current_index,
                    next_index,
                ):
                    continue
                candidate = polygon[candidate_index]
                turns = []
                for edge_index in range(3):
                    point_a = triangle[edge_index]
                    point_b = triangle[(edge_index + 1) % 3]
                    turns.append(
                        (point_b[0] - point_a[0])
                        * (candidate[1] - point_a[1])
                        - (point_b[1] - point_a[1])
                        * (candidate[0] - point_a[0])
                    )
                if min(turns) > 1e-10:
                    has_inner_point = True
                    break
            if has_inner_point:
                continue
            triangles.append(triangle)
            del remaining[position]
            ear_found = True
            break
        if not ear_found:
            return []
        guard -= 1
    if len(remaining) == 3:
        triangle = [polygon[index] for index in remaining]
        if _polygon_area2(triangle) > 1e-12:
            triangles.append(triangle)
    return triangles


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


def _compile_corners(sites):
    incidents = {}
    points = {}
    for site_index, site in enumerate(sites):
        for vert_index, point in (
            (site.vert_a, site.point_a),
            (site.vert_b, site.point_b),
        ):
            incidents.setdefault(vert_index, []).append(site_index)
            points.setdefault(vert_index, point)

    corners = []
    for vert_index in sorted(incidents):
        incident_sites = tuple(sorted(incidents[vert_index]))
        point = points[vert_index]
        ordered_sites = incident_sites
        policy = _CornerPolicy.JUNCTION
        turn_sign = 0.0
        interior_angle = 0.0
        extrusion_angle = 0.0
        is_convex = False
        miter_ratio = float("inf")
        if len(incident_sites) == 1:
            policy = _CornerPolicy.CAP
        elif len(incident_sites) == 2:
            rays = {}
            incoming = []
            outgoing = []
            for site_index in incident_sites:
                site = sites[site_index]
                if site.vert_a == vert_index:
                    other = site.point_b
                    outgoing.append(site_index)
                else:
                    other = site.point_a
                    incoming.append(site_index)
                rays[site_index] = _norm2(_sub2(other, point))
            if (
                len(incoming) == 1
                and len(outgoing) == 1
                and incoming[0] != outgoing[0]
            ):
                ordered_sites = (incoming[0], outgoing[0])
                incoming_ray = rays[incoming[0]]
                outgoing_ray = rays[outgoing[0]]
                if incoming_ray is not None and outgoing_ray is not None:
                    travel_in = (-incoming_ray[0], -incoming_ray[1])
                    signed_turn = _cross2(travel_in, outgoing_ray)
                    if abs(signed_turn) > 1e-9:
                        turn_sign = 1.0 if signed_turn > 0.0 else -1.0
            else:
                ordered_sites = tuple(
                    sorted(
                        incident_sites,
                        key=lambda site_index: atan2(
                            rays[site_index][1], rays[site_index][0]
                        )
                        if rays[site_index] is not None
                        else 0.0,
                    )
                )
                first_ray = rays[ordered_sites[0]]
                second_ray = rays[ordered_sites[1]]
                if first_ray is not None and second_ray is not None:
                    signed_turn = _cross2(first_ray, second_ray)
                    if abs(signed_turn) > 1e-9:
                        turn_sign = 1.0 if signed_turn > 0.0 else -1.0

            first_ray = rays[incident_sites[0]]
            second_ray = rays[incident_sites[1]]
            if first_ray is not None and second_ray is not None:
                minor_angle = atan2(
                    abs(_cross2(first_ray, second_ray)),
                    max(-1.0, min(1.0, _dot2(first_ray, second_ray))),
                )
                bisector = _norm2(
                    (
                        first_ray[0] + second_ray[0],
                        first_ray[1] + second_ray[1],
                    )
                )
                if bisector is None:
                    interior_angle = pi
                    is_convex = False
                else:
                    inside_small_wedge = all(
                        _dot2(bisector, sites[site_index].inward_normal)
                        >= -1e-7
                        for site_index in incident_sites
                    )
                    is_convex = inside_small_wedge and minor_angle < pi - 1e-7
                    interior_angle = (
                        minor_angle if inside_small_wedge else tau - minor_angle
                    )
                    extrusion_angle = (
                        tau - interior_angle
                        if interior_angle > pi
                        else interior_angle
                    )

            offset_lines = []
            for site_index in incident_sites:
                site = sites[site_index]
                other = site.point_b if site.vert_a == vert_index else site.point_a
                direction = (
                    (other[0] - point[0]) / site.segment_length,
                    (other[1] - point[1]) / site.segment_length,
                )
                offset_lines.append(
                    (
                        (
                            point[0] + site.inward_normal[0],
                            point[1] + site.inward_normal[1],
                        ),
                        direction,
                    )
                )
            intersection = _line_intersection(
                offset_lines[0][0],
                offset_lines[0][1],
                offset_lines[1][0],
                offset_lines[1][1],
            )
            if intersection is not None:
                miter_ratio = _dist2(point, intersection)
            if abs(interior_angle - pi) <= 1e-7 or is_convex:
                policy = _CornerPolicy.MITER
            elif extrusion_angle < _ACUTE_SPLIT_ANGLE:
                policy = _CornerPolicy.ACUTE_SPLIT
            elif interior_angle > pi:
                policy = _CornerPolicy.KITE
            elif miter_ratio <= 8.0:
                policy = _CornerPolicy.MITER
            else:
                policy = _CornerPolicy.BEVEL
        corners.append(
            CornerSpec(
                vert_index=vert_index,
                point=point,
                incident_sites=incident_sites,
                ordered_sites=ordered_sites,
                policy=policy,
                turn_sign=turn_sign,
                interior_angle=interior_angle,
                extrusion_angle=extrusion_angle,
                is_convex=is_convex,
                miter_ratio=miter_ratio,
            )
        )
    return tuple(corners)


def _compile_surface(node, raw_sites):
    origin = node.centroid.copy()
    normal = node.normal.normalized()
    basis_u = node.basis_u.normalized()
    basis_v = node.basis_v.normalized()
    triangles = _patch_domain_triangles(
        node, origin, basis_u, basis_v
    )
    if not triangles:
        return None

    all_points = [point for triangle in triangles for point in triangle]
    min_x = min(point[0] for point in all_points)
    max_x = max(point[0] for point in all_points)
    min_y = min(point[1] for point in all_points)
    max_y = max(point[1] for point in all_points)
    diagonal = sqrt((max_x - min_x) ** 2 + (max_y - min_y) ** 2)
    probe_distance = max(
        DECAL_WELD_DISTANCE * 2.0,
        min(diagonal * 1e-4, max(diagonal, 1.0) * 1e-3),
    )
    sites = []
    for raw in raw_sites:
        point_a = _project(raw["source_a"], origin, basis_u, basis_v)
        point_b = _project(raw["source_b"], origin, basis_u, basis_v)
        sites.append(
            _PatchVoronoiSite(
                patch_id=node.patch_id,
                edge_index=raw["edge_index"],
                vert_a=raw["vert_a"],
                vert_b=raw["vert_b"],
                source_a=raw["source_a"],
                source_b=raw["source_b"],
                point_a=point_a,
                point_b=point_b,
                arc_start=raw["arc_start"],
                segment_length=raw["segment_length"],
                uv_sign=raw["uv_sign"],
                inward_normal=_inward_site_normal(
                    point_a, point_b, triangles, probe_distance
                ),
            )
        )
    corners = _compile_corners(sites)
    corner_by_vertex = {
        corner.vert_index: index for index, corner in enumerate(corners)
    }
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

    # Curved segment-Voronoi bisectors are parabolas. Их прежняя частота
    # (diagonal / 48) давала визуальный fan из десятков почти избыточных
    # stations в широких collisions. Ограничиваем шаг и размером patch, и
    # характерной длиной site: контур остаётся стабильным, но low-poly.
    sorted_site_lengths = sorted(site.segment_length for site in sites)
    median_site_length = sorted_site_lengths[len(sorted_site_lengths) // 2]
    curve_step = max(
        min(diagonal / 20.0, median_site_length * 0.5),
        DECAL_WELD_DISTANCE * 2.0,
    )
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
        cell_triangles = _triangulate_cell_polygon(polygon)
        if not cell_triangles:
            continue
        site = sites[cell.site]
        fragments = []
        for cell_triangle in cell_triangles:
            for domain_triangle in triangles:
                clipped = _clip_to_triangle(cell_triangle, domain_triangle)
                if (
                    len(clipped) < 3
                    or abs(_polygon_area2(clipped)) <= 1e-10
                ):
                    continue
                if _polygon_area2(clipped) < 0.0:
                    clipped.reverse()
                fragments.append(tuple(clipped))
        if fragments:
            source_category = int(cell.source_category)
            corner_index = -1
            cell_kind = "SEGMENT"
            if cell.contains_point:
                cell_kind = "POINT"
                if source_category == 1:
                    source_vertex = site.vert_a
                elif source_category == 2:
                    source_vertex = site.vert_b
                else:
                    source_vertex = -1
                corner_index = corner_by_vertex.get(source_vertex, -1)
            atoms.append(
                _PatchVoronoiAtom(
                    site_index=int(cell.site),
                    fragments=tuple(fragments),
                    cell_kind=cell_kind,
                    corner_index=corner_index,
                    source_category=source_category,
                )
            )
    if not atoms:
        return None
    site_grid_size = max(
        median_site_length,
        diagonal / max(8.0, sqrt(len(sites)) * 2.0),
        DECAL_WELD_DISTANCE * 4.0,
    )
    site_grid = {}
    for site_index, site in enumerate(sites):
        margin = max(DECAL_WELD_DISTANCE, site.segment_length * 1e-7)
        min_grid_x = int(
            (min(site.point_a[0], site.point_b[0]) - margin)
            // site_grid_size
        )
        max_grid_x = int(
            (max(site.point_a[0], site.point_b[0]) + margin)
            // site_grid_size
        )
        min_grid_y = int(
            (min(site.point_a[1], site.point_b[1]) - margin)
            // site_grid_size
        )
        max_grid_y = int(
            (max(site.point_a[1], site.point_b[1]) + margin)
            // site_grid_size
        )
        for grid_x in range(min_grid_x, max_grid_x + 1):
            for grid_y in range(min_grid_y, max_grid_y + 1):
                site_grid.setdefault((grid_x, grid_y), []).append(site_index)
    return _PatchVoronoiSurface(
        patch_id=node.patch_id,
        origin=origin,
        normal=normal,
        basis_u=basis_u,
        basis_v=basis_v,
        sites=tuple(sites),
        corners=corners,
        atoms=tuple(atoms),
        site_grid_size=site_grid_size,
        site_grid={
            key: tuple(indices) for key, indices in site_grid.items()
        },
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


def _insert_surface_edge_stations(polygons, tolerance):
    """Делит edge каждой cell всеми point-on-edge вершинами surface.

    До этого соседние cells могли геометрически совпадать, но оставлять
    топологический T-контакт: вершина одной face лежала в середине цельного
    edge другой. BMesh/remove_doubles такой edge не разрезает.
    """

    quantum = max(tolerance * 0.5, 1e-10)

    def point_key(point):
        return (
            round(point[0] / quantum),
            round(point[1] / quantum),
        )

    representatives = {}
    keyed_polygons = []
    for polygon in polygons:
        keys = []
        for point in polygon:
            key = point_key(point)
            representatives.setdefault(key, (float(point[0]), float(point[1])))
            if not keys or keys[-1] != key:
                keys.append(key)
        if len(keys) > 1 and keys[0] == keys[-1]:
            keys.pop()
        keyed_polygons.append(keys)
    if not representatives:
        return [list(polygon) for polygon in polygons], 0

    all_points = list(representatives.values())
    min_x = min(point[0] for point in all_points)
    max_x = max(point[0] for point in all_points)
    min_y = min(point[1] for point in all_points)
    max_y = max(point[1] for point in all_points)
    diagonal = sqrt((max_x - min_x) ** 2 + (max_y - min_y) ** 2)
    grid_size = max(
        tolerance * 4.0,
        diagonal / max(8.0, sqrt(len(representatives)) * 2.0),
    )

    def grid_key(point):
        return (
            int(point[0] // grid_size),
            int(point[1] // grid_size),
        )

    point_grid = {}
    for key, point in representatives.items():
        point_grid.setdefault(grid_key(point), []).append(key)

    rebuilt = []
    inserted_stations = 0
    for polygon_keys in keyed_polygons:
        result_keys = []
        for index, key_a in enumerate(polygon_keys):
            key_b = polygon_keys[(index + 1) % len(polygon_keys)]
            point_a = representatives[key_a]
            point_b = representatives[key_b]
            edge_length = _dist2(point_a, point_b)
            if edge_length <= _GEOMETRY_EPS:
                continue
            result_keys.append(key_a)
            min_grid_x = int((min(point_a[0], point_b[0]) - tolerance) // grid_size)
            max_grid_x = int((max(point_a[0], point_b[0]) + tolerance) // grid_size)
            min_grid_y = int((min(point_a[1], point_b[1]) - tolerance) // grid_size)
            max_grid_y = int((max(point_a[1], point_b[1]) + tolerance) // grid_size)
            candidates = set()
            for grid_x in range(min_grid_x, max_grid_x + 1):
                for grid_y in range(min_grid_y, max_grid_y + 1):
                    candidates.update(point_grid.get((grid_x, grid_y), ()))
            stations = []
            endpoint_fraction = min(0.1, tolerance / edge_length)
            for candidate_key in candidates:
                if candidate_key in (key_a, key_b):
                    continue
                candidate = representatives[candidate_key]
                distance, factor = _segment_point_distance2(
                    point_a, point_b, candidate
                )
                if (
                    distance <= tolerance
                    and endpoint_fraction < factor < 1.0 - endpoint_fraction
                ):
                    stations.append((factor, candidate_key))
            for _factor, candidate_key in sorted(stations):
                if result_keys[-1] == candidate_key:
                    continue
                result_keys.append(candidate_key)
                inserted_stations += 1
        result = [representatives[key] for key in result_keys]
        result = _dedupe_polygon(result, tolerance=quantum)
        rebuilt.append(result)
    return rebuilt, inserted_stations


def _build_decal_arrangement(pending, tolerance):
    """Создаёт conforming subdivision отдельно на каждом owner surface."""

    grouped = {}
    for pending_index, pending_face in enumerate(pending):
        grouped.setdefault(pending_face.surface.patch_id, []).append(
            (pending_index, pending_face)
        )

    arranged_by_index = {}
    inserted_stations = 0
    for entries in grouped.values():
        polygons, inserted = _insert_surface_edge_stations(
            [entry[1].points for entry in entries], tolerance
        )
        inserted_stations += inserted
        for entry, polygon in zip(entries, polygons):
            if len(polygon) < 3 or abs(_polygon_area2(polygon)) <= 1e-10:
                continue
            if _polygon_area2(polygon) < 0.0:
                polygon.reverse()
            pending_face = entry[1]
            arranged_by_index[entry[0]] = _DecalArrangementFace(
                surface=pending_face.surface,
                site=pending_face.site,
                points=tuple(polygon),
                crop=pending_face.crop,
            )
    return DecalArrangement(
        faces=tuple(
            arranged_by_index[index] for index in sorted(arranged_by_index)
        ),
        inserted_stations=inserted_stations,
    )


def _position_and_key(
    plan, surface, site, point, lift_scale, effective_offset
):
    distance, t = _segment_point_distance2(site.point_a, site.point_b, point)
    spine_eps = max(DECAL_WELD_DISTANCE * 0.25, site.segment_length * 1e-7)
    if distance <= spine_eps:
        endpoint_eps = max(
            site.segment_length * 1e-6,
            min(DECAL_WELD_DISTANCE, site.segment_length * 0.01),
        )
        if _dist2(point, site.point_a) <= endpoint_eps:
            position = site.source_a.lerp(
                plan.lifted_vertices[site.vert_a], lift_scale
            )
            return position, ("pv-sv", site.vert_a)
        if _dist2(point, site.point_b) <= endpoint_eps:
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


def _arrangement_position_and_key(
    plan, surface, owning_site, point, lift_scale, effective_offset
):
    """Одна 3D identity для station независимо от owning Voronoi site."""

    grid_size = surface.site_grid_size
    grid_x = int(point[0] // grid_size)
    grid_y = int(point[1] // grid_size)
    candidate_indices = set()
    for offset_x in (-1, 0, 1):
        for offset_y in (-1, 0, 1):
            candidate_indices.update(
                surface.site_grid.get(
                    (grid_x + offset_x, grid_y + offset_y), ()
                )
            )
    best_site = owning_site
    best_distance, _factor = _segment_point_distance2(
        owning_site.point_a, owning_site.point_b, point
    )
    for candidate_index in candidate_indices:
        candidate = surface.sites[candidate_index]
        distance, _factor = _segment_point_distance2(
            candidate.point_a, candidate.point_b, point
        )
        if (
            distance < best_distance - 1e-12
            or (
                abs(distance - best_distance) <= 1e-12
                and candidate.edge_index < best_site.edge_index
            )
        ):
            best_distance = distance
            best_site = candidate
    return _position_and_key(
        plan,
        surface,
        best_site,
        point,
        lift_scale,
        effective_offset,
    )


def _component_signed_area(plan, surface, site, component, lift_scale):
    """Signed area после shared-rail lift для заданного offset-scale."""

    positions = []
    used_keys = set()
    effective_offset = plan.offset * lift_scale
    for point in component:
        position, key = _arrangement_position_and_key(
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
    for pending_face in pending:
        surface = pending_face.surface
        site = pending_face.site
        component = pending_face.points
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


def _junction_connector_faces(plan, faces, alpha):
    """Закрывает парные cross-patch sectors до BMesh materialization.

    Внутри planar surface endpoint Voronoi-cell уже соединяет соседние
    strips. После fold остаются парные открытые core-to-rail rays: это
    математическая граница отсутствующего junction sector, а не mesh-hole
    для последующего fill. Соединяем только такие rays и только локально у
    одной source-вершины.
    """

    surfaces_by_id = {
        surface.patch_id: surface for surface in plan.surfaces
    }
    incident_edges_by_vertex = {}
    for surface in plan.surfaces:
        for site in surface.sites:
            incident_edges_by_vertex.setdefault(site.vert_a, set()).add(
                site.edge_index
            )
            incident_edges_by_vertex.setdefault(site.vert_b, set()).add(
                site.edge_index
            )

    edge_uses = {}
    for face_index, face in enumerate(faces):
        for index, key_a in enumerate(face.vert_keys):
            key_b = face.vert_keys[(index + 1) % len(face.vert_keys)]
            edge_key = tuple(sorted((key_a, key_b), key=repr))
            edge_uses.setdefault(edge_key, []).append((face_index, index))

    ports_by_vertex = {}
    for edge_key, uses in edge_uses.items():
        if len(uses) != 1:
            continue
        core_key = next(
            (key for key in edge_key if key[:1] == ("pv-sv",)), None
        )
        if core_key is None:
            continue
        outer_key = edge_key[0] if edge_key[1] == core_key else edge_key[1]
        if outer_key[:1] == ("pv-sv",):
            continue
        face_index, _edge_index = uses[0]
        face = faces[face_index]
        surface = surfaces_by_id.get(face.surface_id)
        if surface is None:
            continue
        core_index = face.vert_keys.index(core_key)
        outer_index = face.vert_keys.index(outer_key)
        outer_position = face.positions[outer_index]
        outer_point = (
            (outer_position - surface.origin).dot(surface.basis_u),
            (outer_position - surface.origin).dot(surface.basis_v),
        )
        matching_sites = []
        for site in surface.sites:
            if site.vert_a == core_key[1]:
                station = site.point_a
            elif site.vert_b == core_key[1]:
                station = site.point_b
            else:
                continue
            expected_outer = (
                station[0] + site.inward_normal[0] * alpha,
                station[1] + site.inward_normal[1] * alpha,
            )
            matching_sites.append(
                (_dist2(outer_point, expected_outer), site.edge_index)
            )
        if not matching_sites:
            continue
        match_distance, matched_edge_index = min(matching_sites)
        # Boundary edge большой Voronoi-cell тоже может начинаться в core,
        # но junction port обязан лежать у локального alpha-offset cap.
        if match_distance > max(DECAL_WELD_DISTANCE, alpha * 0.05):
            continue
        ports_by_vertex.setdefault(core_key[1], []).append(
            _JunctionPort(
                surface_id=face.surface_id,
                edge_index=matched_edge_index,
                surface_normal=face.surface_normal.copy(),
                core_position=face.positions[core_index].copy(),
                outer_key=outer_key,
                outer_position=face.positions[outer_index].copy(),
                outer_u=face.u_fracs[outer_index],
                outer_v=face.v_lengths[outer_index],
                core_v=face.v_lengths[core_index],
            )
        )

    connectors = []
    max_span = max(alpha * 2.25, DECAL_WELD_DISTANCE * 4.0)
    for vert_index, ports in sorted(ports_by_vertex.items()):
        # Cross-patch connector описывает поворот одной ветви. T/X junction
        # уже принадлежит surface Voronoi arrangement и не должен получать
        # произвольный greedy chord между несколькими ветвями.
        if len(incident_edges_by_vertex.get(vert_index, ())) != 2:
            continue
        # Одна ray — штатный открытый cap. Несколько rays образуют
        # независимые парные sectors; greedy matching детерминирован и не
        # делает fan, где один boundary edge получил бы три owner faces.
        candidates = []
        for index, first in enumerate(ports):
            for other_index in range(index + 1, len(ports)):
                second = ports[other_index]
                if first.outer_key == second.outer_key:
                    continue
                if first.surface_id == second.surface_id:
                    continue
                if first.edge_index == second.edge_index:
                    continue
                distance = (first.outer_position - second.outer_position).length
                candidates.append(
                    (
                        distance,
                        first.surface_id,
                        repr(first.outer_key),
                        second.surface_id,
                        repr(second.outer_key),
                        index,
                        other_index,
                    )
                )
        used_ports = set()
        for distance, _sa, _ka, _sb, _kb, index, other_index in sorted(
            candidates
        ):
            if distance > max_span:
                break
            if index in used_ports or other_index in used_ports:
                continue
            first = ports[index]
            second = ports[other_index]
            core_position = first.core_position.lerp(
                second.core_position, 0.5
            )
            blended = first.surface_normal + second.surface_normal
            if blended.length_squared <= 1e-12:
                continue
            normal = blended.normalized()
            entries = [first, second]
            winding = (first.outer_position - core_position).cross(
                second.outer_position - core_position
            )
            if winding.dot(normal) < 0.0:
                entries.reverse()
            connectors.append(
                _NetworkFace(
                    surface_id=-1,
                    surface_normal=normal,
                    vert_keys=[("pv-sv", vert_index)]
                    + [entry.outer_key for entry in entries],
                    positions=[core_position]
                    + [entry.outer_position for entry in entries],
                    u_fracs=[0.0] + [entry.outer_u for entry in entries],
                    v_lengths=[
                        (first.core_v + second.core_v) * 0.5
                    ]
                    + [entry.outer_v for entry in entries],
                )
            )
            used_ports.update((index, other_index))
    return connectors


def evaluate_patch_voronoi_plan(plan, width, preview=False):
    """Перестраивает extrusion polygons внутри статических Voronoi cells."""

    alpha = max(1e-6, float(width) * 0.5)
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
    for surface in plan.surfaces:
        for atom in surface.atoms:
            site = surface.sites[atom.site_index]
            if atom.cell_kind == "POINT" and atom.corner_index >= 0:
                crop_components = _corner_crop_components(
                    surface, surface.corners[atom.corner_index], alpha
                )
            else:
                crop_components = (
                    _CropComponent(
                        kind="SEGMENT",
                        side="",
                        points=tuple(_segment_crop_polygon(site, alpha)),
                    ),
                )
            for crop in crop_components:
                if len(crop.points) < 3:
                    continue
                fragments = []
                for fragment in atom.fragments:
                    clipped = _clip_to_convex(fragment, crop.points)
                    if clipped:
                        fragments.append(clipped)
                components = _merge_polygon_fragments(
                    fragments,
                    tolerance=max(1e-8, DECAL_WELD_DISTANCE * 0.25),
                )
                for component in components:
                    component = _dedupe_polygon(component, tolerance=1e-7)
                    if (
                        len(component) < 3
                        or abs(_polygon_area2(component)) <= 1e-10
                    ):
                        continue
                    if _polygon_area2(component) < 0.0:
                        component.reverse()
                    pending.append(
                        _PendingArrangementFace(
                            surface=surface,
                            site=site,
                            points=tuple(component),
                            crop=crop,
                        )
                    )

    arrangement = _build_decal_arrangement(
        pending,
        tolerance=max(1e-8, DECAL_WELD_DISTANCE * 0.5),
    )
    pending = arrangement.faces
    lift_scale = _orientation_safe_lift_scale(plan, pending, lift_scale)
    effective_offset = plan.offset * lift_scale
    faces = []
    emitted_faces = set()
    for pending_face in pending:
        surface = pending_face.surface
        site = pending_face.site
        component = pending_face.points
        crop = pending_face.crop
        vert_keys = []
        positions = []
        u_fracs = []
        v_lengths = []
        used_keys = set()
        for point in component:
            position, key = _arrangement_position_and_key(
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
            component_uv = _crop_component_uv(crop, point)
            if component_uv is None:
                u_fracs.append(
                    site.uv_sign * max(0.0, min(1.0, distance / alpha))
                )
                v_lengths.append(site.arc_start + t * site.segment_length)
            else:
                u_fracs.append(component_uv[0])
                v_lengths.append(component_uv[1])
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
                component_kind=crop.kind,
                component_side=crop.side,
            )
        )
    faces.extend(_junction_connector_faces(plan, faces, alpha))
    return faces
