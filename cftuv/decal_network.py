"""CFTUV Decal Network — α-clipped nearest-branch partition для Decal Seams.

Замена последовательности «независимые ribbons → junction cut → радиальные
сектора → dissolve поперечных рёбер» единым глобальным разбиением по образцу
segment-Voronoi (Johannsen, Edge Decal Generator), адаптированным для
многоповерхностных (corner) decal-сетей.

Ключевые идеи:

1. Задача разбиения — внутренне-метрическая (intrinsic). Corner-поверхность
   из планарных owner surfaces изометрична плоскости: двугранный угол не
   входит во внутреннюю геометрию, а постоянная ширина декали — геодезическое
   расстояние. Поэтому partition считается в 2D на каждой owner offset-плоскости,
   а fold-линия метрически исчезает.
2. Полная Voronoi-диаграмма не нужна: нужны только ячейки, обрезанные на
   половине ширины α. Ячейка ветви = α-полоса вокруг её spine ∩ полурегионы
   «ближе ко мне, чем к соседней ветви». Никакого sweepline: прямой клиппинг
   малых faces имплицитной функцией f = d_other − d_own с уточнением хорд.
3. Сайты — ветви (полилинии), а не отдельные сегменты: поперечные рёбра
   внутри ветви не возникают по построению. Коллинеарные продолжения через
   junction сливаются в сквозной сайт (непрерывный UV сквозь узел).
4. Ghost-сайты через fold не нужны: путь с соседней поверхности пересекает
   fold-линию, а если fold выбран как ветвь — его spine всегда ближе
   (d(q, fold) ≤ d(q, x) для любого x за fold). Поэтому per-surface сайтов
   достаточно; конкуренция между поверхностями невозможна.
5. Разрешение ties в общих vertex regions двух ветвей, оканчивающихся в одном
   junction: spine обеих ветвей стягивается (retraction) на δ от узла — tie
   схлопывается в корректную угловую биссектрису («усреднённая divider-линия»
   T039), а имплицитная функция получает чистую смену знака.
6. Лифт в 3D — экстраинсический слой: интерьер станций → offset вдоль
   локальной normal; fold-станции → точка на пересечении offset-плоскостей;
   junction-узлы → least-squares core (как в legacy `_prepare_seam_junctions`).
   Общие вершины идут через единый registry по source vert index, поэтому
   сшивка крыльев вдоль spine и junction водонепроницаема по построению.

Модуль — чистая геометрия: без bpy/bmesh, только mathutils.Vector для 3D.
Все 2D-вычисления на кортежах float. Потребитель — decals.py.
"""

from dataclasses import dataclass, field
from math import acos, atan2, ceil, cos, pi, sin, sqrt

from mathutils import Vector

from .constants import (
    DECAL_COPLANAR_DOT,
    DECAL_CORNER_MITER_LIMIT,
    DECAL_NETWORK_CONTINUATION_DOT,
    DECAL_SPINE_MERGE_DISTANCE,
    DECAL_WELD_DISTANCE,
)


# ============================================================
# 2D примитивы (кортежи, без mathutils)
# ============================================================


def _sub2(a, b):
    return (a[0] - b[0], a[1] - b[1])


def _add2(a, b):
    return (a[0] + b[0], a[1] + b[1])


def _mul2(a, s):
    return (a[0] * s, a[1] * s)


def _dot2(a, b):
    return a[0] * b[0] + a[1] * b[1]


def _cross2(a, b):
    return a[0] * b[1] - a[1] * b[0]


def _len2(a):
    return sqrt(a[0] * a[0] + a[1] * a[1])


def _dist2(a, b):
    return _len2(_sub2(a, b))


def _norm2(a):
    length = _len2(a)
    if length < 1e-15:
        return None
    return (a[0] / length, a[1] / length)


def _rot90(a):
    """Поворот на +90°: left-сторона направления в right-handed 2D базисе."""

    return (-a[1], a[0])


def _segment_point_distance2(seg_a, seg_b, q):
    """(расстояние, clamped t) от точки до 2D сегмента."""

    delta = _sub2(seg_b, seg_a)
    denom = _dot2(delta, delta)
    if denom < 1e-24:
        return _dist2(q, seg_a), 0.0
    t = _dot2(_sub2(q, seg_a), delta) / denom
    t = max(0.0, min(1.0, t))
    closest = _add2(seg_a, _mul2(delta, t))
    return _dist2(q, closest), t


def _line_intersection2(point_a, dir_a, point_b, dir_b):
    """Пересечение двух 2D прямых; None для (почти) параллельных."""

    denom = _cross2(dir_a, dir_b)
    if abs(denom) < 1e-12:
        return None
    t = _cross2(_sub2(point_b, point_a), dir_b) / denom
    return _add2(point_a, _mul2(dir_a, t))


def _polygon_area2(points):
    area = 0.0
    for index in range(len(points)):
        area += _cross2(points[index], points[(index + 1) % len(points)])
    return area * 0.5


# ============================================================
# Лифт станций на offset-поверхности
# ============================================================


def _group_average_normals(normals):
    """Группирует почти-одинаковые oriented normals и усредняет группы."""

    groups = []
    for normal in normals:
        if normal.length_squared < 1e-12:
            continue
        candidate = normal.normalized()
        placed = False
        for group in groups:
            if candidate.dot(group[0]) > DECAL_COPLANAR_DOT:
                group[1].append(candidate)
                placed = True
                break
        if not placed:
            groups.append([candidate, [candidate]])
    averaged = []
    for _representative, members in groups:
        total = Vector((0.0, 0.0, 0.0))
        for member in members:
            total = total + member
        if total.length_squared > 1e-12:
            averaged.append(total.normalized())
    return averaged


def _lift_position(source_pos, normals, offset):
    """Least-squares пересечение offset-плоскостей owner surfaces точки.

    Одна плоскость → source + n*offset; две → точка на линии пересечения
    offset-плоскостей (эквивалент `_corner_spine_position`); три и более →
    общий junction core (эквивалент legacy `_offset_plane_junction_center`).
    """

    unique = _group_average_normals(normals)
    if not unique:
        return source_pos.copy()
    delta = Vector((0.0, 0.0, 0.0))
    for _iteration in range(64):
        correction = Vector((0.0, 0.0, 0.0))
        for normal in unique:
            correction += normal * (offset - normal.dot(delta))
        correction /= len(unique)
        delta += correction
        if correction.length_squared < 1e-18:
            break
    return source_pos + delta


def _corner_wing_directions(direction, normal_a, normal_b, dihedral_convexity=0.0):
    """Направления крыльев вдоль обеих поверхностей с учётом inner/outer."""

    wing_dir_a = direction.cross(normal_a)
    wing_dir_b = direction.cross(normal_b)
    if wing_dir_a.length_squared < 1e-8 or wing_dir_b.length_squared < 1e-8:
        return None

    wing_dir_a = wing_dir_a.normalized()
    wing_dir_b = wing_dir_b.normalized()
    is_concave = dihedral_convexity < -0.01
    if (wing_dir_a.dot(normal_b) < 0.0) == is_concave:
        wing_dir_a = wing_dir_a * -1.0
    if (wing_dir_b.dot(normal_a) < 0.0) == is_concave:
        wing_dir_b = wing_dir_b * -1.0
    return wing_dir_a, wing_dir_b


# ============================================================
# Ветви сети: merge коллинеарных продолжений через junction
# ============================================================


@dataclass
class _NetworkBranch:
    """Сайт-полилиния сети; может проходить сквозь junction узлы."""

    vert_indices: list
    points: list
    normals_a: list
    normals_b: list
    convexities: list
    closed: bool


def _branch_from_run(run):
    return _NetworkBranch(
        vert_indices=list(run.vert_indices),
        points=[point.copy() for point in run.points],
        normals_a=[normal.copy() for normal in run.segment_normals_a],
        normals_b=[normal.copy() for normal in run.segment_normals_b],
        convexities=list(run.segment_convexities),
        closed=run.is_closed,
    )


def _reverse_branch(branch):
    # Convexity — геометрический инвариант фолда (reverse/swap-независимый).
    return _NetworkBranch(
        vert_indices=list(reversed(branch.vert_indices)),
        points=list(reversed(branch.points)),
        normals_a=list(reversed(branch.normals_a)),
        normals_b=list(reversed(branch.normals_b)),
        convexities=list(reversed(branch.convexities)),
        closed=branch.closed,
    )


def _swap_branch_sides(branch):
    return _NetworkBranch(
        vert_indices=list(branch.vert_indices),
        points=list(branch.points),
        normals_a=list(branch.normals_b),
        normals_b=list(branch.normals_a),
        convexities=list(branch.convexities),
        closed=branch.closed,
    )


def _join_branches(first, second):
    """Соединяет две ветви с общей вершиной, сохраняя идентичность сторон."""

    same_score = (
        first.normals_a[-1].dot(second.normals_a[0])
        + first.normals_b[-1].dot(second.normals_b[0])
    )
    swapped_score = (
        first.normals_a[-1].dot(second.normals_b[0])
        + first.normals_b[-1].dot(second.normals_a[0])
    )
    if swapped_score > same_score:
        second = _swap_branch_sides(second)
    return _NetworkBranch(
        vert_indices=first.vert_indices + second.vert_indices[1:],
        points=first.points + second.points[1:],
        normals_a=first.normals_a + second.normals_a,
        normals_b=first.normals_b + second.normals_b,
        convexities=first.convexities + second.convexities,
        closed=False,
    )


def _branch_end_tangent(branch, at_start):
    if at_start:
        delta = branch.points[1] - branch.points[0]
    else:
        delta = branch.points[-1] - branch.points[-2]
        delta = delta * -1.0
    if delta.length_squared < 1e-12:
        return None
    return delta.normalized()


def _endpoint_valence(branches):
    valence = {}
    for branch in branches:
        if branch.closed:
            continue
        for vert in (branch.vert_indices[0], branch.vert_indices[-1]):
            valence[vert] = valence.get(vert, 0) + 1
    return valence


def _merge_junction_continuations(branches):
    """Сливает анти-параллельные пары ветвей в junction в сквозные сайты.

    Сквозной сайт даёт непрерывный UV через узел и убирает tie-бисектрису
    (поперечную линию) между коллинеарными продолжениями. Пары выбираются
    жадно от наиболее анти-параллельной; критерий геометрический
    (DECAL_NETWORK_CONTINUATION_DOT), а не произвольная «главная ветка».
    """

    branches = list(branches)
    while True:
        valence = _endpoint_valence(branches)
        junction_ends = {}
        for branch_index, branch in enumerate(branches):
            if branch.closed:
                continue
            for at_start in (True, False):
                vert = (
                    branch.vert_indices[0]
                    if at_start
                    else branch.vert_indices[-1]
                )
                if valence.get(vert, 0) < 3:
                    continue
                tangent = _branch_end_tangent(branch, at_start)
                if tangent is None:
                    continue
                junction_ends.setdefault(vert, []).append(
                    (branch_index, at_start, tangent)
                )

        best_pair = None
        best_dot = DECAL_NETWORK_CONTINUATION_DOT
        for vert, ends in junction_ends.items():
            for i in range(len(ends)):
                for j in range(i + 1, len(ends)):
                    index_i, _start_i, tangent_i = ends[i]
                    index_j, _start_j, tangent_j = ends[j]
                    if index_i == index_j:
                        continue
                    # Outgoing тангенсы обеих ветвей смотрят ИЗ узла:
                    # продолжение — анти-параллельная пара.
                    pair_dot = tangent_i.dot(tangent_j)
                    if pair_dot <= best_dot:
                        best_dot = pair_dot
                        best_pair = (vert, ends[i], ends[j])
        if best_pair is None:
            return branches

        _vert, end_i, end_j = best_pair
        index_i, start_i, _tangent_i = end_i
        index_j, start_j, _tangent_j = end_j
        first = branches[index_i]
        second = branches[index_j]
        # first должен ЗАКАНЧИВАТЬСЯ в узле, second — НАЧИНАТЬСЯ в нём.
        if start_i:
            first = _reverse_branch(first)
        if not start_j:
            second = _reverse_branch(second)
        merged = _join_branches(first, second)
        branches = [
            branch
            for branch_index, branch in enumerate(branches)
            if branch_index not in (index_i, index_j)
        ]
        branches.append(merged)


# ============================================================
# Поверхности и сайты
# ============================================================


@dataclass
class _Surface:
    surface_id: int
    normal: Vector
    plane_d: float
    origin: Vector = None
    basis_x: Vector = None
    basis_y: Vector = None

    def finalize(self, offset):
        self.origin = self.normal * (self.plane_d + offset)
        axis = Vector((1.0, 0.0, 0.0))
        if abs(self.normal.dot(axis)) > 0.9:
            axis = Vector((0.0, 1.0, 0.0))
        basis_x = axis - self.normal * axis.dot(self.normal)
        self.basis_x = basis_x.normalized()
        self.basis_y = self.normal.cross(self.basis_x)

    def to_2d(self, position):
        delta = position - self.origin
        return (delta.dot(self.basis_x), delta.dot(self.basis_y))

    def to_3d(self, point2):
        return self.origin + self.basis_x * point2[0] + self.basis_y * point2[1]


class _SurfaceRegistry:
    def __init__(self):
        self.surfaces = []

    def surface_for(self, normal, point):
        if normal.length_squared < 1e-12:
            return None
        unit = normal.normalized()
        plane_d = unit.dot(point)
        for surface in self.surfaces:
            if (
                unit.dot(surface.normal) > DECAL_COPLANAR_DOT
                and abs(surface.normal.dot(point) - surface.plane_d)
                < DECAL_SPINE_MERGE_DISTANCE
            ):
                return surface.surface_id
        surface = _Surface(
            surface_id=len(self.surfaces), normal=unit, plane_d=plane_d
        )
        self.surfaces.append(surface)
        return surface.surface_id


@dataclass
class _Site:
    """Односторонняя half-cell полоса вдоль подпути ветви на одной поверхности."""

    site_id: int
    branch_id: int
    side: str  # 'A' | 'B'
    surface_id: int
    start_station: int
    end_station: int  # inclusive; > start_station
    pts2: list = field(default_factory=list)
    arcs: list = field(default_factory=list)  # global branch arc на станциях
    lat_sign: float = 1.0
    start_kind: str = "free"  # 'free' | 'junction' | 'split'
    end_kind: str = "free"
    closed: bool = False
    vert_indices: list = field(default_factory=list)
    retract_start: float = 0.0
    retract_end: float = 0.0
    bbox: tuple = None
    cap_start_sweep: float = 0.0
    cap_end_sweep: float = 0.0

    def finalize(self, delta):
        if not self.closed:
            if self.start_kind == "junction":
                self.retract_start = delta
            if self.end_kind == "junction":
                self.retract_end = delta
        xs = [p[0] for p in self.pts2]
        ys = [p[1] for p in self.pts2]
        self.bbox = (min(xs), min(ys), max(xs), max(ys))

    # -------- расстояния --------

    def competition_distance(self, q):
        """Расстояние до spine со стянутыми junction-концами (tie-break)."""

        best = None
        last = len(self.pts2) - 1
        for index in range(last):
            seg_a = self.pts2[index]
            seg_b = self.pts2[index + 1]
            if index == 0 and self.retract_start > 0.0:
                direction = _norm2(_sub2(seg_b, seg_a))
                if direction is not None:
                    seg_a = _add2(seg_a, _mul2(direction, self.retract_start))
            if index == last - 1 and self.retract_end > 0.0:
                direction = _norm2(_sub2(seg_a, seg_b))
                if direction is not None:
                    seg_b = _add2(seg_b, _mul2(direction, self.retract_end))
            distance, _t = _segment_point_distance2(seg_a, seg_b, q)
            if best is None or distance < best:
                best = distance
        return best if best is not None else float("inf")

    def uv(self, q, alpha):
        """(u_occupied ∈ [0..1], v в мировых единицах) в frame собственной ветви.

        За концами открытого подпути V продолжается по неограниченной проекции
        на крайний сегмент (linear extension, как в legacy junction UV).
        """

        best = None
        last = len(self.pts2) - 1
        for index in range(last):
            distance, t = _segment_point_distance2(
                self.pts2[index], self.pts2[index + 1], q
            )
            if best is None or distance < best[0]:
                best = (distance, index, t)
        _distance, index, t = best
        seg_a = self.pts2[index]
        seg_b = self.pts2[index + 1]
        delta = _sub2(seg_b, seg_a)
        denom = _dot2(delta, delta)
        raw_t = _dot2(_sub2(q, seg_a), delta) / denom if denom > 1e-24 else 0.0
        extend = not self.closed and (
            (index == 0 and raw_t < 0.0) or (index == last - 1 and raw_t > 1.0)
        )
        used_t = raw_t if extend else max(0.0, min(1.0, raw_t))
        arc_v = self.arcs[index] + used_t * (
            self.arcs[index + 1] - self.arcs[index]
        )
        direction = _norm2(delta)
        if direction is None:
            lateral = 0.0
        else:
            lateral = _cross2(direction, _sub2(q, seg_a))
        occupied = lateral * self.lat_sign
        if alpha > 1e-12:
            occupied = max(0.0, min(1.0, occupied / alpha))
        else:
            occupied = 0.0
        return occupied, arc_v


def _node_spine_directions(sites):
    """Углы всех spine-направлений сайтов в каждой (vert, surface) точке."""

    directions = {}
    for site in sites:
        pts = site.pts2
        for index, vert in enumerate(site.vert_indices):
            for neighbor in (index - 1, index + 1):
                if not 0 <= neighbor < len(pts):
                    continue
                direction = _norm2(_sub2(pts[neighbor], pts[index]))
                if direction is None:
                    continue
                directions.setdefault((vert, site.surface_id), []).append(
                    (site.site_id, atan2(direction[1], direction[0]))
                )
    return directions


def _junction_cap_sweep(site, at_start, node_directions):
    """Угловая длина round cap: до биссектрисы с угловым соседом.

    Vertex region узла принадлежит декали только между крыльями соседних
    ветвей (красная схема T039): промежуток до соседа γ > 180° закрывается
    дугой γ/2 − 90°; γ ≤ 180° закрыт самими полосами; поверхность без
    соседей получает плоский конец — дуга висела бы за границей меша.
    """

    pts = site.pts2
    if at_start:
        d_me = _norm2(_sub2(pts[1], pts[0]))
        sense = site.lat_sign
        vert = site.vert_indices[0]
    else:
        d_me = _norm2(_sub2(pts[-2], pts[-1]))
        sense = -site.lat_sign
        vert = site.vert_indices[-1]
    if d_me is None:
        return 0.0
    theta = atan2(d_me[1], d_me[0])
    best = None
    for other_id, angle in node_directions.get(
        (vert, site.surface_id), ()
    ):
        if other_id == site.site_id:
            continue
        delta = ((angle - theta) * sense) % (2.0 * pi)
        # Совпадающее направление — вторая сторона того же spine.
        if delta < 1e-3 or delta > 2.0 * pi - 1e-3:
            continue
        if best is None or delta < best:
            best = delta
    if best is None or best <= pi + 1e-4:
        return 0.0
    return best / 2.0 - pi / 2.0


def _sites_compete(site_a, site_b):
    if site_a.site_id == site_b.site_id:
        return False
    if site_a.surface_id != site_b.surface_id:
        return False
    if site_a.branch_id == site_b.branch_id:
        overlap = min(site_a.end_station, site_b.end_station) - max(
            site_a.start_station, site_b.start_station
        )
        if overlap >= 0:
            return False
    return True


# ============================================================
# Построение face-набора half-cell полосы
# ============================================================


def _arc_points(center, radius, angle_start, sweep, arc_tolerance):
    """Полилинизация дуги с ограничением стрелки прогиба (sagitta)."""

    if radius <= 1e-12 or abs(sweep) < 1e-9:
        return []
    max_step = 2.0 * acos(max(0.0, min(1.0, 1.0 - arc_tolerance / radius)))
    if max_step < 1e-3:
        max_step = 1e-3
    steps = max(1, int(ceil(abs(sweep) / max_step)))
    points = []
    for index in range(steps + 1):
        angle = angle_start + sweep * (index / steps)
        points.append(
            (
                center[0] + radius * cos(angle),
                center[1] + radius * sin(angle),
            )
        )
    return points


def _station_outer_points(
    station,
    tangent_prev,
    tangent_next,
    lat_sign,
    alpha,
    arc_tolerance,
):
    """Outer-контур станции: MITER, обрезка concave или round join."""

    lat_prev = _mul2(_rot90(tangent_prev), lat_sign)
    lat_next = _mul2(_rot90(tangent_next), lat_sign)
    incoming = _add2(station, _mul2(lat_prev, alpha))
    outgoing = _add2(station, _mul2(lat_next, alpha))
    if _dist2(incoming, outgoing) < 1e-12:
        return [incoming]

    turn = _cross2(tangent_prev, tangent_next)
    intersection = _line_intersection2(
        incoming, tangent_prev, outgoing, tangent_next
    )
    miter_limit = alpha * DECAL_CORNER_MITER_LIMIT
    if turn * lat_sign > 0.0:
        # Concave для этой стороны: offset-линии обрезают друг друга.
        if intersection is not None and _dist2(intersection, station) <= miter_limit:
            return [intersection]
        return [_mul2(_add2(incoming, outgoing), 0.5)]
    # Convex: miter в пределах лимита, иначе round join.
    if intersection is not None and _dist2(intersection, station) <= miter_limit:
        return [intersection]
    angle_start = atan2(lat_prev[1], lat_prev[0])
    angle_end = atan2(lat_next[1], lat_next[0])
    sweep = angle_end - angle_start
    while sweep > pi:
        sweep -= 2.0 * pi
    while sweep < -pi:
        sweep += 2.0 * pi
    points = _arc_points(station, alpha, angle_start, sweep, arc_tolerance)
    return points if points else [_mul2(_add2(incoming, outgoing), 0.5)]


def _site_band_faces(site, alpha, arc_tolerance):
    """2D faces half-cell до клиппинга: quads сегментов + fans станций/caps."""

    pts = site.pts2
    count = len(pts)
    if count < 2:
        return []
    tangents = []
    for index in range(count - 1):
        direction = _norm2(_sub2(pts[index + 1], pts[index]))
        tangents.append(direction)
    if any(direction is None for direction in tangents):
        return []

    lat_sign = site.lat_sign
    outer = {}
    segment_count = count - 1

    if site.closed:
        for index in range(count - 1):
            prev_index = index - 1 if index > 0 else segment_count - 1
            outer[index] = _station_outer_points(
                pts[index],
                tangents[prev_index],
                tangents[index],
                lat_sign,
                alpha,
                arc_tolerance,
            )
        outer[count - 1] = outer[0]
    else:
        # Начальная станция.
        lat0 = _mul2(_rot90(tangents[0]), lat_sign)
        if site.start_kind == "junction" and site.cap_start_sweep > 1e-6:
            sweep_len = site.cap_start_sweep
            angle_start = atan2(lat0[1], lat0[0]) + lat_sign * sweep_len
            outer[0] = _arc_points(
                pts[0], alpha, angle_start, -lat_sign * sweep_len, arc_tolerance
            ) or [_add2(pts[0], _mul2(lat0, alpha))]
        else:
            outer[0] = [_add2(pts[0], _mul2(lat0, alpha))]
        # Внутренние станции.
        for index in range(1, count - 1):
            outer[index] = _station_outer_points(
                pts[index],
                tangents[index - 1],
                tangents[index],
                lat_sign,
                alpha,
                arc_tolerance,
            )
        # Конечная станция.
        lat_end = _mul2(_rot90(tangents[-1]), lat_sign)
        if site.end_kind == "junction" and site.cap_end_sweep > 1e-6:
            angle_lat = atan2(lat_end[1], lat_end[0])
            outer[count - 1] = _arc_points(
                pts[-1],
                alpha,
                angle_lat,
                -lat_sign * site.cap_end_sweep,
                arc_tolerance,
            ) or [_add2(pts[-1], _mul2(lat_end, alpha))]
        else:
            outer[count - 1] = [_add2(pts[-1], _mul2(lat_end, alpha))]

    faces = []
    for index in range(segment_count):
        if _dist2(pts[index], pts[index + 1]) < 1e-12:
            continue
        quad = [
            pts[index],
            pts[index + 1],
            outer[index + 1][0],
            outer[index][-1],
        ]
        faces.append(quad)
    station_range = (
        range(count - 1) if site.closed else range(count)
    )
    for index in station_range:
        points = outer[index]
        for arc_index in range(len(points) - 1):
            faces.append(
                [pts[index], points[arc_index], points[arc_index + 1]]
            )
    return faces


# ============================================================
# Клиппинг имплицитной функцией f = d_other − d_own
# ============================================================

_EDGE_SAMPLES = 8
_CHORD_DEPTH = 8


def _edge_zero_crossings(point_a, point_b, value_a, value_b, implicit):
    """Параметры t всех смен знака f вдоль ребра (a, b)."""

    samples = [(0.0, value_a)]
    for index in range(1, _EDGE_SAMPLES):
        t = index / _EDGE_SAMPLES
        q = _add2(point_a, _mul2(_sub2(point_b, point_a), t))
        samples.append((t, implicit(q)))
    samples.append((1.0, value_b))

    crossings = []
    for (t_lo, f_lo), (t_hi, f_hi) in zip(samples, samples[1:]):
        if (f_lo >= 0.0) == (f_hi >= 0.0):
            continue
        lo, hi = t_lo, t_hi
        for _iteration in range(40):
            mid = (lo + hi) * 0.5
            q = _add2(point_a, _mul2(_sub2(point_b, point_a), mid))
            if (implicit(q) >= 0.0) == (f_lo >= 0.0):
                lo = mid
            else:
                hi = mid
            if hi - lo < 1e-12:
                break
        crossings.append((lo + hi) * 0.5)
    return crossings


def _refine_chord(point_a, point_b, implicit, tolerance, depth=_CHORD_DEPTH):
    """Вставляет точки f=0 между двумя crossing, пока хорда не сойдётся."""

    if depth <= 0 or _dist2(point_a, point_b) < 2.0 * tolerance:
        return []
    midpoint = _mul2(_add2(point_a, point_b), 0.5)
    direction = _norm2(_sub2(point_b, point_a))
    if direction is None:
        return []
    normal = _rot90(direction)
    value_mid = implicit(midpoint)
    # Поиск нуля вдоль нормали к хорде: геометрический рост шага.
    limit = max(_dist2(point_a, point_b), 4.0 * tolerance)
    step = tolerance
    zero = None
    sign_mid = value_mid >= 0.0
    while step <= limit:
        for orientation in (1.0, -1.0):
            probe = _add2(midpoint, _mul2(normal, orientation * step))
            if (implicit(probe) >= 0.0) != sign_mid:
                lo, hi = 0.0, orientation * step
                for _iteration in range(40):
                    mid = (lo + hi) * 0.5
                    q = _add2(midpoint, _mul2(normal, mid))
                    if (implicit(q) >= 0.0) == sign_mid:
                        lo = mid
                    else:
                        hi = mid
                    if abs(hi - lo) < 1e-12:
                        break
                zero = _add2(midpoint, _mul2(normal, (lo + hi) * 0.5))
                break
        if zero is not None:
            break
        step *= 2.0
    if zero is None or _dist2(zero, midpoint) <= tolerance:
        return []
    return (
        _refine_chord(point_a, zero, implicit, tolerance, depth - 1)
        + [zero]
        + _refine_chord(zero, point_b, implicit, tolerance, depth - 1)
    )


def _clip_polygon(points, implicit, keep_eps, tolerance):
    """Оставляет часть полигона с f ≥ 0 (граница уточняется по кривой)."""

    values = [implicit(point) for point in points]
    if all(value >= -keep_eps for value in values):
        return [points]
    if all(value < -keep_eps for value in values):
        return []

    output = []  # (point, is_crossing, entering)
    count = len(points)
    for index in range(count):
        point_a = points[index]
        point_b = points[(index + 1) % count]
        value_a = values[index]
        value_b = values[(index + 1) % count]
        if value_a >= -keep_eps:
            output.append((point_a, False, False))
        inside = value_a >= 0.0
        for t in _edge_zero_crossings(
            point_a, point_b, value_a, value_b, implicit
        ):
            crossing = _add2(point_a, _mul2(_sub2(point_b, point_a), t))
            inside = not inside
            output.append((crossing, True, inside))
    if len(output) < 3:
        return []

    # Уточнение хорд между exit- и entry-crossing.
    refined = []
    total = len(output)
    for index in range(total):
        point, is_crossing, entering = output[index]
        refined.append(point)
        if not is_crossing or entering:
            continue
        next_point, next_is_crossing, next_entering = output[
            (index + 1) % total
        ]
        if next_is_crossing and next_entering:
            refined.extend(
                _refine_chord(point, next_point, implicit, tolerance)
            )

    deduped = []
    for point in refined:
        if deduped and _dist2(point, deduped[-1]) < 1e-9:
            continue
        deduped.append(point)
    if len(deduped) > 2 and _dist2(deduped[0], deduped[-1]) < 1e-9:
        deduped.pop()
    if len(deduped) < 3:
        return []
    return [deduped]


def _bbox_distance(bbox_a, bbox_b):
    dx = max(bbox_a[0] - bbox_b[2], bbox_b[0] - bbox_a[2], 0.0)
    dy = max(bbox_a[1] - bbox_b[3], bbox_b[1] - bbox_a[3], 0.0)
    return sqrt(dx * dx + dy * dy)


# ============================================================
# Сборка сети
# ============================================================


@dataclass
class _NetworkFace:
    """Готовая face декали: 3D позиции, shared-ключи вершин и UV-факты."""

    surface_id: int
    surface_normal: Vector
    vert_keys: list
    positions: list
    u_fracs: list  # [-1..1]: −1 = внешний край стороны A, +1 = стороны B
    v_lengths: list  # мировые единицы вдоль ветви


def _branch_station_normals(branch):
    """Уникальные owner-нормали каждой станции (для лифта)."""

    per_station = []
    segment_count = len(branch.points) - 1
    for station in range(len(branch.points)):
        normals = []
        prev_segment = station - 1
        next_segment = station
        if station == 0 and branch.closed:
            prev_segment = segment_count - 1
        if station == len(branch.points) - 1:
            next_segment = 0 if branch.closed else -1
        for segment in (prev_segment, next_segment):
            if 0 <= segment < segment_count:
                normals.append(branch.normals_a[segment])
                normals.append(branch.normals_b[segment])
        per_station.append(normals)
    return per_station


def _split_side_sites(
    branch, branch_id, side, registry, lift_map, arcs, valence, site_counter
):
    """Режет одну сторону ветви на maximal подпути постоянной поверхности."""

    normals = branch.normals_a if side == "A" else branch.normals_b
    other = branch.normals_b if side == "A" else branch.normals_a
    segment_count = len(branch.points) - 1
    segment_surfaces = []
    for segment in range(segment_count):
        midpoint = (branch.points[segment] + branch.points[segment + 1]) * 0.5
        segment_surfaces.append(
            registry.surface_for(normals[segment], midpoint)
        )

    sites = []
    span_start = 0
    for segment in range(segment_count + 1):
        boundary = (
            segment == segment_count
            or segment_surfaces[segment] != segment_surfaces[span_start]
        )
        if not boundary:
            continue
        span_end = segment  # сегменты span_start .. segment-1
        surface_id = segment_surfaces[span_start]
        if surface_id is not None and span_end > span_start:
            site = _Site(
                site_id=site_counter[0],
                branch_id=branch_id,
                side=side,
                surface_id=surface_id,
                start_station=span_start,
                end_station=span_end,
            )
            site_counter[0] += 1
            surface = registry.surfaces[surface_id]
            for station in range(span_start, span_end + 1):
                vert = branch.vert_indices[station]
                site.pts2.append(surface.to_2d(lift_map[vert]))
                site.arcs.append(arcs[station])
                site.vert_indices.append(vert)
            whole_side = span_start == 0 and span_end == segment_count
            site.closed = branch.closed and whole_side
            if not site.closed:
                site.start_kind = _site_end_kind(
                    branch, span_start, span_start == 0, valence
                )
                site.end_kind = _site_end_kind(
                    branch, span_end, span_end == segment_count, valence
                )
            site.lat_sign = _site_lat_sign(
                branch, side, normals, other, span_start, surface, site
            )
            sites.append(site)
        span_start = segment
    return sites


def _site_end_kind(branch, station, at_branch_end, valence):
    vert = branch.vert_indices[station]
    if not at_branch_end:
        # Смена поверхности внутри ветви.
        return "junction" if valence.get(vert, 0) >= 3 else "split"
    if branch.closed:
        # Открытый подпуть замкнутой ветви — смена поверхности на кольце.
        return "split"
    return "junction" if valence.get(vert, 0) >= 3 else "free"


def _site_lat_sign(branch, side, normals, other_normals, span_start, surface, site):
    """Знак занятой боковой стороны сайта в 2D базисе поверхности."""

    for offset in range(len(site.pts2) - 1):
        segment = span_start + offset
        direction3 = branch.points[segment + 1] - branch.points[segment]
        if direction3.length_squared < 1e-12:
            continue
        direction3 = direction3.normalized()
        normal_own = normals[segment]
        normal_other = other_normals[segment]
        if normal_own.dot(normal_other) > DECAL_COPLANAR_DOT:
            # Копланарный шов: стороны разводим детерминированно.
            wing3 = surface.normal.cross(direction3)
            if side == "B":
                wing3 = wing3 * -1.0
        else:
            # `_corner_wing_directions` симметрична к порядку normals при
            # инвариантной convexity: wings[0] — крыло первой normal.
            wings = _corner_wing_directions(
                direction3,
                normal_own,
                normal_other,
                branch.convexities[segment],
            )
            if wings is None:
                continue
            wing3 = wings[0]
        tangent2 = _norm2(
            _sub2(site.pts2[offset + 1], site.pts2[offset])
        )
        if tangent2 is None:
            continue
        wing2 = (wing3.dot(surface.basis_x), wing3.dot(surface.basis_y))
        lateral = _dot2(_rot90(tangent2), wing2)
        if abs(lateral) > 1e-6:
            return 1.0 if lateral > 0.0 else -1.0
    return 1.0 if side == "A" else -1.0


def build_seam_network_faces(runs, offset, width, arc_tolerance=None):
    """Строит faces decal-сети для manual Decal Seams.

    runs — ститченные `_OrientedCornerRun` ветви (duck-typed), offset —
    отступ от поверхности, width — полная ширина шва. Возвращает список
    `_NetworkFace`; пустой список означает «нечего строить».
    """

    runs = [run for run in runs if len(run.points) >= 2]
    if not runs:
        return []
    alpha = max(1e-6, float(width) * 0.5)
    if arc_tolerance is None:
        arc_tolerance = DECAL_WELD_DISTANCE
    delta = max(1e-9, alpha * 1e-4)
    keep_eps = 2.0 * delta

    branches = _merge_junction_continuations(
        [_branch_from_run(run) for run in runs]
    )
    valence = _endpoint_valence(branches)

    # --- лифт станций: единый registry по source vert ---
    normals_by_vert = {}
    position_by_vert = {}
    for branch in branches:
        station_normals = _branch_station_normals(branch)
        for station, vert in enumerate(branch.vert_indices):
            normals_by_vert.setdefault(vert, []).extend(
                station_normals[station]
            )
            position_by_vert.setdefault(vert, branch.points[station])
    lift_map = {
        vert: _lift_position(position_by_vert[vert], normals, offset)
        for vert, normals in normals_by_vert.items()
    }

    # --- поверхности и сайты ---
    registry = _SurfaceRegistry()
    sites = []
    site_counter = [0]
    branch_arcs = []
    for branch_id, branch in enumerate(branches):
        arcs = [0.0]
        for index in range(1, len(branch.points)):
            arcs.append(
                arcs[-1]
                + (branch.points[index] - branch.points[index - 1]).length
            )
        branch_arcs.append(arcs)
    # Все поверхности регистрируются до finalize (single pass достаточно,
    # surface_for детерминирован); базисы нужны до проекции сайтов.
    for branch_id, branch in enumerate(branches):
        for side in ("A", "B"):
            normals = branch.normals_a if side == "A" else branch.normals_b
            for segment in range(len(branch.points) - 1):
                midpoint = (
                    branch.points[segment] + branch.points[segment + 1]
                ) * 0.5
                registry.surface_for(normals[segment], midpoint)
    for surface in registry.surfaces:
        surface.finalize(offset)
    for branch_id, branch in enumerate(branches):
        for side in ("A", "B"):
            sites.extend(
                _split_side_sites(
                    branch,
                    branch_id,
                    side,
                    registry,
                    lift_map,
                    branch_arcs[branch_id],
                    valence,
                    site_counter,
                )
            )
    for site in sites:
        site.finalize(delta)
    node_directions = _node_spine_directions(sites)
    for site in sites:
        if site.closed:
            continue
        if site.start_kind == "junction":
            site.cap_start_sweep = _junction_cap_sweep(
                site, True, node_directions
            )
        if site.end_kind == "junction":
            site.cap_end_sweep = _junction_cap_sweep(
                site, False, node_directions
            )

    # --- faces + клиппинг ---
    # Точка face может отстоять от своего spine на miter-длину (до α·LIMIT);
    # конкурент влияет, если сам ближе этой длины → spine bboxes в 2·α·LIMIT.
    reach = alpha * 2.0 * DECAL_CORNER_MITER_LIMIT
    result = []
    station_keys = {}
    for site in sites:
        for point2, vert in zip(site.pts2, site.vert_indices):
            station_keys[
                (site.surface_id, round(point2[0], 6), round(point2[1], 6))
            ] = vert

    for site in sites:
        competitors = [
            other
            for other in sites
            if _sites_compete(site, other)
            and _bbox_distance(site.bbox, other.bbox) <= reach
        ]
        polygons = _site_band_faces(site, alpha, arc_tolerance)
        for competitor in competitors:
            def implicit(q, _site=site, _competitor=competitor):
                return _competitor.competition_distance(
                    q
                ) - _site.competition_distance(q)

            clipped = []
            for polygon in polygons:
                clipped.extend(
                    _clip_polygon(polygon, implicit, keep_eps, arc_tolerance)
                )
            polygons = clipped
            if not polygons:
                break
        surface = registry.surfaces[site.surface_id]
        side_sign_uv = -1.0 if site.side == "A" else 1.0
        for polygon in polygons:
            if abs(_polygon_area2(polygon)) < 1e-10:
                continue
            if _polygon_area2(polygon) < 0.0:
                polygon = list(reversed(polygon))
            keys = []
            positions = []
            u_fracs = []
            v_lengths = []
            for point2 in polygon:
                lookup = (
                    site.surface_id,
                    round(point2[0], 6),
                    round(point2[1], 6),
                )
                vert = station_keys.get(lookup)
                if vert is not None:
                    keys.append(("sv", vert))
                    positions.append(lift_map[vert].copy())
                else:
                    keys.append(("p",) + lookup)
                    positions.append(surface.to_3d(point2))
                occupied, arc_v = site.uv(point2, alpha)
                u_fracs.append(side_sign_uv * occupied)
                v_lengths.append(arc_v)
            result.append(
                _NetworkFace(
                    surface_id=site.surface_id,
                    surface_normal=surface.normal.copy(),
                    vert_keys=keys,
                    positions=positions,
                    u_fracs=u_fracs,
                    v_lengths=v_lengths,
                )
            )

    result.extend(
        _split_connector_faces(sites, registry, lift_map, alpha)
    )
    return result


def _split_connector_faces(sites, registry, lift_map, alpha):
    """Соединительные faces на станциях смены поверхности внутри ветви.

    Аналог legacy BEVEL: общая spine-вершина уже сшита lift registry, а
    внешние flat-cap точки двух смежных сайтов соединяются треугольником.
    Для почти копланарной смены поверхности точки почти совпадают и
    свариваются финальным weld.
    """

    open_ends = {}
    for site in sites:
        if site.closed:
            continue
        if site.start_kind == "split":
            open_ends.setdefault(
                (site.branch_id, site.side, site.vert_indices[0]), []
            ).append((site, True))
        if site.end_kind == "split":
            open_ends.setdefault(
                (site.branch_id, site.side, site.vert_indices[-1]), []
            ).append((site, False))

    faces = []
    for (branch_id, side, vert), ends in open_ends.items():
        if len(ends) != 2:
            continue
        outer_points = []
        for site, at_start in ends:
            surface = registry.surfaces[site.surface_id]
            pts = site.pts2
            if at_start:
                tangent = _norm2(_sub2(pts[1], pts[0]))
                station2 = pts[0]
                arc_v = site.arcs[0]
            else:
                tangent = _norm2(_sub2(pts[-1], pts[-2]))
                station2 = pts[-1]
                arc_v = site.arcs[-1]
            if tangent is None:
                outer_points = []
                break
            lat = _mul2(_rot90(tangent), site.lat_sign)
            outer2 = _add2(station2, _mul2(lat, alpha))
            outer_points.append(
                (surface.to_3d(outer2), surface.normal, arc_v, site.side)
            )
        if len(outer_points) != 2:
            continue
        first, second = outer_points
        if (first[0] - second[0]).length <= DECAL_WELD_DISTANCE:
            continue
        spine_pos = lift_map[vert]
        blended = first[1] + second[1]
        if blended.length_squared < 1e-12:
            continue
        normal = blended.normalized()
        side_sign_uv = -1.0 if first[3] == "A" else 1.0
        entries = [
            (("c", vert, side, 0), first),
            (("c", vert, side, 1), second),
        ]
        winding = (first[0] - spine_pos).cross(second[0] - spine_pos)
        if winding.dot(normal) < 0.0:
            entries.reverse()
        faces.append(
            _NetworkFace(
                surface_id=-1,
                surface_normal=normal,
                vert_keys=[("sv", vert)] + [entry[0] for entry in entries],
                positions=[spine_pos.copy()]
                + [entry[1][0] for entry in entries],
                u_fracs=[0.0, side_sign_uv, side_sign_uv],
                v_lengths=[(first[2] + second[2]) * 0.5]
                + [entry[1][2] for entry in entries],
            )
        )
    return faces
