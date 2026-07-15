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

from dataclasses import dataclass, field, replace
from math import acos, atan2, ceil, cos, pi, sin, sqrt

from mathutils import Vector

from .constants import (
    DECAL_COPLANAR_DOT,
    DECAL_CORNER_MITER_LIMIT,
    DECAL_NETWORK_CLIP_CURVE_BUDGET,
    DECAL_NETWORK_CLIP_CURVE_DEPTH,
    DECAL_NETWORK_CLIP_CURVE_RELATIVE,
    DECAL_NETWORK_CLIP_CURVE_STYLE,
    DECAL_NETWORK_CONTINUATION_DOT,
    DECAL_NETWORK_JUNCTION_CAP_STYLE,
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
    if len(unique) == 1:
        return source_pos + unique[0] * offset
    if len(unique) == 2:
        dot = max(-1.0, min(1.0, unique[0].dot(unique[1])))
        denominator = 1.0 + dot
        if denominator > 1e-8:
            # Minimum-norm точка линии пересечения двух offset-плоскостей.
            delta = (unique[0] + unique[1]) * (offset / denominator)
            return source_pos + delta

    # Решаем normal equations напрямую. Итерационный projection сходился
    # слишком медленно для почти встречных торцевых normals и оставлял core
    # внутри одной из offset-плоскостей, из-за чего wing визуально загибался.
    xx = sum(normal.x * normal.x for normal in unique)
    xy = sum(normal.x * normal.y for normal in unique)
    xz = sum(normal.x * normal.z for normal in unique)
    yy = sum(normal.y * normal.y for normal in unique)
    yz = sum(normal.y * normal.z for normal in unique)
    zz = sum(normal.z * normal.z for normal in unique)
    rhs = Vector(
        tuple(
            offset * sum(normal[axis] for normal in unique)
            for axis in range(3)
        )
    )
    trace = xx + yy + zz
    determinant_limit = max(1e-14, trace * trace * trace * 1e-12)
    determinant = (
        xx * (yy * zz - yz * yz)
        - xy * (xy * zz - yz * xz)
        + xz * (xy * yz - yy * xz)
    )
    if abs(determinant) <= determinant_limit:
        # Rank-deficient junction: alternating projections устойчивее
        # инверсии почти сингулярной normal matrix.
        delta = Vector((0.0, 0.0, 0.0))
        for _iteration in range(128):
            correction = Vector((0.0, 0.0, 0.0))
            for normal in unique:
                correction += normal * (offset - normal.dot(delta))
            correction /= len(unique)
            delta += correction
            if correction.length_squared < 1e-20:
                break
        return source_pos + delta
    cofactor_xx = yy * zz - yz * yz
    cofactor_xy = xz * yz - xy * zz
    cofactor_xz = xy * yz - xz * yy
    cofactor_yy = xx * zz - xz * xz
    cofactor_yz = xy * xz - xx * yz
    cofactor_zz = xx * yy - xy * xy
    delta = Vector(
        (
            (
                cofactor_xx * rhs.x
                + cofactor_xy * rhs.y
                + cofactor_xz * rhs.z
            )
            / determinant,
            (
                cofactor_xy * rhs.x
                + cofactor_yy * rhs.y
                + cofactor_yz * rhs.z
            )
            / determinant,
            (
                cofactor_xz * rhs.x
                + cofactor_yz * rhs.y
                + cofactor_zz * rhs.z
            )
            / determinant,
        )
    )
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


def _is_boundary_branch(branch):
    """Односторонняя boundary-ветвь: сторона A пустая (нулевые нормали).

    Wing такой ветви задаётся chain boundary-ориентацией (n × t), поэтому
    её нельзя разворачивать/swap'ить в continuation merge.
    """

    return bool(branch.normals_a) and branch.normals_a[0].length_squared < 1e-12


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
            if branch.closed or _is_boundary_branch(branch):
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


_DISTANCE_INDEX_MIN_SEGMENTS = 40
_DISTANCE_INDEX_LEAF_SIZE = 6

# Arrangement хранит shared 2D keys с округлением до 6 знаков. Допуск split
# обязан покрывать полный шаг этой сетки: иначе две численно эквивалентные
# boundary-точки могут попасть в соседние quantization cells, вершина одной
# face не вставится в ребро другой и materialize оставит точный T-junction.
_ARRANGEMENT_POINT_TOLERANCE = 1e-6


def _build_segment_distance_index(segments):
    """Строит компактный exact BVH над 2D segment records сайта.

    Индекс нужен только длинным chains. Для коротких sites линейный цикл
    дешевле обхода дерева. Leaf хранит сами immutable segment records,
    поэтому query не создаёт временных векторов и не прыгает по индексам.
    """

    if len(segments) < _DISTANCE_INDEX_MIN_SEGMENTS:
        return (), -1

    records = []
    for segment in segments:
        ax, ay, dx, dy, _inv = segment
        bx = ax + dx
        by = ay + dy
        records.append(
            (
                min(ax, bx),
                min(ay, by),
                max(ax, bx),
                max(ay, by),
                segment,
            )
        )

    nodes = []

    def build(items):
        min_x = min(item[0] for item in items)
        min_y = min(item[1] for item in items)
        max_x = max(item[2] for item in items)
        max_y = max(item[3] for item in items)
        if len(items) <= _DISTANCE_INDEX_LEAF_SIZE:
            node_index = len(nodes)
            nodes.append(
                (min_x, min_y, max_x, max_y, -1, -1,
                 tuple(item[4] for item in items))
            )
            return node_index

        use_x = max_x - min_x >= max_y - min_y
        if use_x:
            items.sort(key=lambda item: item[0] + item[2])
        else:
            items.sort(key=lambda item: item[1] + item[3])
        middle = len(items) // 2
        left = build(items[:middle])
        right = build(items[middle:])
        node_index = len(nodes)
        nodes.append((min_x, min_y, max_x, max_y, left, right, None))
        return node_index

    root = build(records)
    return tuple(nodes), root


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
    _segments: list = field(default_factory=list, repr=False)
    _distance_nodes: tuple = field(default_factory=tuple, repr=False)
    _distance_root: int = field(default=-1, repr=False)
    cap_start_sweep: float = 0.0
    cap_end_sweep: float = 0.0
    # Конформный лифт: настоящие per-segment нормали и lifted станции.
    segment_normals3: list = field(default_factory=list)
    lifts3: list = field(default_factory=list)
    surface: object = None
    # Shared rail на split-станциях: (point2, key, position3) вместо
    # flat cap; обе стороны разреза ссылаются на один vertex key.
    cap_start_override: tuple = None
    cap_end_override: tuple = None

    def to_3d(self, q2):
        """Конформный лифт: точка чарта поднимается на плоскость своей грани.

        2D-чарт span'а — усреднённая плоскость (метрика для клиппинга), но
        реальная стена под лентой гранёная: лифт через chart.to_3d клал
        ленту на плоскость-хорду, которая при offset ≪ стрелки прогиба
        протыкает стену, а углы flat caps выпирали наружу («зубцы»).
        Вершина мапится аффинно в frame ближайшего сегмента: станции
        сохраняются точно, ширина вдоль реальной грани — точно α.
        """

        best = None
        for index in range(len(self.pts2) - 1):
            distance, _t = _segment_point_distance2(
                self.pts2[index], self.pts2[index + 1], q2
            )
            if best is None or distance < best[0]:
                best = (distance, index)
        if best is None:
            return self.surface.to_3d(q2)
        index = best[1]
        seg_a2 = self.pts2[index]
        seg_b2 = self.pts2[index + 1]
        tangent2 = _norm2(_sub2(seg_b2, seg_a2))
        length2 = _dist2(seg_a2, seg_b2)
        lift_a = self.lifts3[index]
        lift_b = self.lifts3[index + 1]
        tangent3 = lift_b - lift_a
        length3 = tangent3.length
        if tangent2 is None or length2 < 1e-12 or length3 < 1e-12:
            return self.surface.to_3d(q2)
        tangent3 = tangent3 / length3
        normal3 = self.segment_normals3[index]
        if normal3.length_squared < 1e-12:
            return self.surface.to_3d(q2)
        lateral3 = normal3.cross(tangent3)
        if lateral3.length_squared < 1e-12:
            return self.surface.to_3d(q2)
        lateral3 = lateral3.normalized()
        delta2 = _sub2(q2, seg_a2)
        along = _dot2(delta2, tangent2)
        across = _cross2(tangent2, delta2)
        return lift_a + tangent3 * (along * length3 / length2) + lateral3 * across

    def finalize(self, delta):
        if not self.closed:
            if self.start_kind == "junction":
                self.retract_start = delta
            if self.end_kind == "junction":
                self.retract_end = delta
        xs = [p[0] for p in self.pts2]
        ys = [p[1] for p in self.pts2]
        self.bbox = (min(xs), min(ys), max(xs), max(ys))
        self._build_segment_cache()

    def _build_segment_cache(self):
        """Плоские записи (ax, ay, dx, dy, inv_len_sq) со стянутыми концами.

        Считаются один раз на генерацию и переиспользуются миллионами
        squared-distance запросов клиппинга — без создания tuple и без sqrt
        в горячем цикле. Retraction junction-концов встроена в записи,
        точно как в прежнем inline-варианте.
        """

        segments = []
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
            ax, ay = seg_a
            dx = seg_b[0] - ax
            dy = seg_b[1] - ay
            length_sq = dx * dx + dy * dy
            inv = 1.0 / length_sq if length_sq > 1e-24 else 0.0
            segments.append((ax, ay, dx, dy, inv))
        self._segments = segments
        self._distance_nodes, self._distance_root = (
            _build_segment_distance_index(segments)
        )

    # -------- расстояния --------

    def competition_distance_sq(self, q):
        """Квадрат расстояния до spine со стянутыми junction-концами.

        Быстрое ядро: скалярная арифметика по предвычисленным записям
        сегментов, без tuple и sqrt. `competition_distance` берёт из него
        корень. Клиппер использует именно метрическую разность
        (`competition_distance`), а не квадратичную: `keep_eps` вокруг
        нулевой линии имеет размерность длины, и сравнение с `d²`-разностью
        сделало бы epsilon-зону масштабозависимой (сдвиг слияний вершин).
        """

        qx, qy = q
        if self._distance_root >= 0:
            return self._indexed_competition_distance_sq(qx, qy)

        best = float("inf")
        for ax, ay, dx, dy, inv in self._segments:
            rx = qx - ax
            ry = qy - ay
            if inv == 0.0:
                dist_sq = rx * rx + ry * ry
            else:
                t = (rx * dx + ry * dy) * inv
                if t < 0.0:
                    t = 0.0
                elif t > 1.0:
                    t = 1.0
                ex = rx - dx * t
                ey = ry - dy * t
                dist_sq = ex * ex + ey * ey
            if dist_sq < best:
                best = dist_sq
        return best

    def _indexed_competition_distance_sq(self, qx, qy):
        """Exact nearest segment через branch-and-bound по BVH."""

        best = float("inf")
        nodes = self._distance_nodes
        stack = [(0.0, self._distance_root)]
        while stack:
            lower_bound, node_index = stack.pop()
            # Небольшой запас исключает numerical over-pruning около tie.
            if lower_bound > best + max(1e-15, abs(best) * 1e-12):
                continue
            node = nodes[node_index]
            leaf = node[6]
            if leaf is not None:
                for ax, ay, dx, dy, inv in leaf:
                    rx = qx - ax
                    ry = qy - ay
                    if inv == 0.0:
                        dist_sq = rx * rx + ry * ry
                    else:
                        t = (rx * dx + ry * dy) * inv
                        if t < 0.0:
                            t = 0.0
                        elif t > 1.0:
                            t = 1.0
                        ex = rx - dx * t
                        ey = ry - dy * t
                        dist_sq = ex * ex + ey * ey
                    if dist_sq < best:
                        best = dist_sq
                continue

            left_index = node[4]
            left = nodes[left_index]
            if qx < left[0]:
                left_dx = left[0] - qx
            elif qx > left[2]:
                left_dx = qx - left[2]
            else:
                left_dx = 0.0
            if qy < left[1]:
                left_dy = left[1] - qy
            elif qy > left[3]:
                left_dy = qy - left[3]
            else:
                left_dy = 0.0
            left_bound = left_dx * left_dx + left_dy * left_dy
            right_index = node[5]
            right = nodes[right_index]
            if qx < right[0]:
                right_dx = right[0] - qx
            elif qx > right[2]:
                right_dx = qx - right[2]
            else:
                right_dx = 0.0
            if qy < right[1]:
                right_dy = right[1] - qy
            elif qy > right[3]:
                right_dy = qy - right[3]
            else:
                right_dy = 0.0
            right_bound = right_dx * right_dx + right_dy * right_dy
            # Stack LIFO: дальний первым, ближний будет обработан сразу и
            # уменьшит best до проверки второй ветви.
            if left_bound <= right_bound:
                stack.append((right_bound, right_index))
                stack.append((left_bound, left_index))
            else:
                stack.append((left_bound, left_index))
                stack.append((right_bound, right_index))
        return best

    def competition_distance(self, q):
        """Метрическое расстояние до spine (sqrt квадратичного варианта)."""

        return sqrt(self.competition_distance_sq(q))

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


def _profile_arc(center, radius, angle_start, sweep, curve):
    """Дуга round-join по профилю кривой (HARD/LOW/SMOOTH).

    HARD — прямой bevel (только концы дуги, без внутренних вершин);
    LOW — грубая дуга с бюджетом внутренних сегментов; SMOOTH — точная.
    Так острый convex-угол не разрастается в веер из десятков вершин.
    """

    style, tolerance, budget = curve
    if radius <= 1e-12 or abs(sweep) < 1e-9:
        return _arc_points(center, radius, angle_start, sweep, tolerance)
    end = (
        center[0] + radius * cos(angle_start + sweep),
        center[1] + radius * sin(angle_start + sweep),
    )
    start = (
        center[0] + radius * cos(angle_start),
        center[1] + radius * sin(angle_start),
    )
    if style == "HARD":
        return [start, end]
    points = _arc_points(center, radius, angle_start, sweep, tolerance)
    if not points:
        return [start, end]
    if budget is not None and len(points) - 1 > budget:
        # Пересэмплируем дугу ровно в budget сегментов (жёсткий лимит).
        points = [
            (
                center[0] + radius * cos(angle_start + sweep * i / budget),
                center[1] + radius * sin(angle_start + sweep * i / budget),
            )
            for i in range(budget + 1)
        ]
    return points


def _station_outer_points(
    station,
    tangent_prev,
    tangent_next,
    lat_sign,
    alpha,
    curve,
):
    """Outer-контур станции: MITER, обрезка concave или round join.

    curve = (style, tolerance, budget) управляет детализацией convex
    round-join за miter-лимитом (острый угол): HARD — прямой bevel, LOW —
    грубая дуга с бюджетом, SMOOTH — точная. MITER/concave не меняются.
    """

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
    # Convex: miter в пределах лимита, иначе round join по профилю.
    if intersection is not None and _dist2(intersection, station) <= miter_limit:
        return [intersection]
    angle_start = atan2(lat_prev[1], lat_prev[0])
    angle_end = atan2(lat_next[1], lat_next[0])
    sweep = angle_end - angle_start
    while sweep > pi:
        sweep -= 2.0 * pi
    while sweep < -pi:
        sweep += 2.0 * pi
    points = _profile_arc(station, alpha, angle_start, sweep, curve)
    return points if points else [_mul2(_add2(incoming, outgoing), 0.5)]


def _junction_cap_outer(
    station, lat_unit, bisector_angle, sweep_len, alpha, curve,
    perpendicular_first,
):
    """Outer-контур junction cap для reflex-промежутка.

    MITER (по умолчанию) — одна общая вершина на биссектрисе с угловым
    соседом на радиусе α/cos(sweep): жёсткий стык вместо округлого веера.
    За miter-лимитом — прямой bevel (хорда до биссекторной точки на α), а
    не длинный шип. ROUND — округлая дуга по профилю curve (без веера).

    perpendicular_first управляет порядком под конвенцию сборки quad:
    начальная станция ждёт перпендикулярную точку последней, конечная —
    первой.
    """

    perp_point = _add2(station, _mul2(lat_unit, alpha))
    if DECAL_NETWORK_JUNCTION_CAP_STYLE == "ROUND":
        perp_angle = atan2(lat_unit[1], lat_unit[0])
        arc = _profile_arc(
            station, alpha, bisector_angle, perp_angle - bisector_angle, curve
        )
        cap = arc if arc else [perp_point]
        # Дуга идёт от биссектрисы к перпендикуляру; при perpendicular_first
        # разворачиваем под конвенцию конечной станции.
        return list(reversed(cap)) if perpendicular_first else cap

    cos_sweep = cos(sweep_len)
    if cos_sweep > 1.0 / DECAL_CORNER_MITER_LIMIT:
        radius = alpha / cos_sweep
        far = (
            station[0] + radius * cos(bisector_angle),
            station[1] + radius * sin(bisector_angle),
        )
    else:
        # Слишком острый reflex — прямой bevel до биссекторной точки на α.
        far = (
            station[0] + alpha * cos(bisector_angle),
            station[1] + alpha * sin(bisector_angle),
        )
    return [perp_point, far] if perpendicular_first else [far, perp_point]


def _site_band_faces(site, alpha, curve):
    """2D faces + строгий owner-distance bound до клиппинга.

    curve = (style, tolerance, budget) — профиль детализации дуг round-join
    и junction cap (см. `_profile_arc`). Каждый элемент —
    ``(polygon, owner_upper)``. Quad ограничен своим spine-сегментом,
    station fan — центральной spine-вершиной; выпуклость distance-функции
    гарантирует максимум на вершинах исходного face. Любой последующий
    clipped component является подмножеством и наследует тот же bound.
    """

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
                curve,
            )
        outer[count - 1] = outer[0]
    else:
        # Начальная станция.
        lat0 = _mul2(_rot90(tangents[0]), lat_sign)
        if site.cap_start_override is not None:
            outer[0] = [site.cap_start_override[0]]
        elif site.start_kind == "junction" and site.cap_start_sweep > 1e-6:
            sweep_len = site.cap_start_sweep
            bisector_angle = atan2(lat0[1], lat0[0]) + lat_sign * sweep_len
            outer[0] = _junction_cap_outer(
                pts[0], lat0, bisector_angle, sweep_len, alpha,
                curve, perpendicular_first=False,
            )
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
                curve,
            )
        # Конечная станция.
        lat_end = _mul2(_rot90(tangents[-1]), lat_sign)
        if site.cap_end_override is not None:
            outer[count - 1] = [site.cap_end_override[0]]
        elif site.end_kind == "junction" and site.cap_end_sweep > 1e-6:
            sweep_len = site.cap_end_sweep
            bisector_angle = atan2(lat_end[1], lat_end[0]) - lat_sign * sweep_len
            outer[count - 1] = _junction_cap_outer(
                pts[-1], lat_end, bisector_angle, sweep_len, alpha,
                curve, perpendicular_first=True,
            )
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
        owner_upper = max(
            _segment_point_distance2(
                pts[index], pts[index + 1], point
            )[0]
            for point in quad
        )
        faces.append((quad, owner_upper))
    station_range = (
        range(count - 1) if site.closed else range(count)
    )
    for index in station_range:
        points = outer[index]
        for arc_index in range(len(points) - 1):
            polygon = [
                pts[index], points[arc_index], points[arc_index + 1]
            ]
            owner_upper = max(
                _dist2(point, pts[index]) for point in polygon
            )
            faces.append((polygon, owner_upper))
    return faces


# ============================================================
# Клиппинг имплицитной функцией f = d_other − d_own
# ============================================================

_EDGE_SAMPLES = 8
_CHORD_DEPTH = 8
_PREVIEW_ROOT_T_TOLERANCE = 1e-6


def _edge_zero_crossings(
    point_a,
    point_b,
    value_a,
    value_b,
    implicit,
    t_tolerance=1e-12,
):
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
            if hi - lo < t_tolerance:
                break
        crossings.append((lo + hi) * 0.5)
    return crossings


def _edge_can_cross(point_a, point_b, value_a, value_b):
    """Консервативный gate для нулей distance-difference на ребре.

    Расстояние до множества 1-Lipschitz, поэтому разность двух расстояний
    2-Lipschitz. Если ноль существует на ребре длины L, обязательно
    ``|f(a)| + |f(b)| <= 2L``. Обратное не гарантирует crossing, но позволяет
    безопасно не запускать восьмиточечный поиск на заведомо далёких рёбрах.
    """

    if (value_a >= 0.0) != (value_b >= 0.0):
        return True
    return abs(value_a) + abs(value_b) <= 2.0 * _dist2(point_a, point_b)


def _refine_chord(
    point_a, point_b, implicit, tolerance, depth=_CHORD_DEPTH, budget=None
):
    """Вставляет точки f=0 между двумя crossing, пока хорда не сойдётся.

    tolerance — ВИЗУАЛЬНЫЙ допуск кривой (не weld): при большем значении
    вставляется меньше вершин. budget — общий лимит внутренних вершин на
    одну кривую (список-счётчик), чтобы геометрия не разрасталась вне
    зависимости от кривизны. Позиция каждой вставленной вершины при этом
    находится точно (внутренняя bisection остаётся плотной).
    """

    if (
        depth <= 0
        or (budget is not None and budget[0] <= 0)
        or _dist2(point_a, point_b) < 2.0 * tolerance
    ):
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
    if budget is not None:
        budget[0] -= 1
    return (
        _refine_chord(point_a, zero, implicit, tolerance, depth - 1, budget)
        + [zero]
        + _refine_chord(zero, point_b, implicit, tolerance, depth - 1, budget)
    )


def _point_cache_key(point, digits=10):
    return (round(point[0], digits), round(point[1], digits))


def _dedupe_polygon_loop(points, digits=6):
    """Убирает совпадающие соседние точки в точности materializer key.

    Root search и canonical chord могут вернуть две точки с разницей порядка
    1e-8. В 2D это разные float, но после quantization в ``('p', surface, x,
    y)`` они становятся одной BMesh-вершиной и делают face невалидным.
    Нормализуем loop до той же точности до построения topology.
    """

    deduped = []
    keys = []
    for point in points:
        key = _point_cache_key(point, digits)
        if keys and key == keys[-1]:
            continue
        deduped.append(point)
        keys.append(key)
    if len(keys) > 1 and keys[0] == keys[-1]:
        deduped.pop()
    return deduped


def _split_repeated_polygon_loop(points):
    """Разлагает touching loop по повторной вершине на простые контуры.

    Повтор не удаляется вслепую: он означает, что граница ячейки пришла в
    уже посещённый multi-site узел. Две дуги между одинаковыми ключами — это
    две независимые boundary-компоненты. Такое разложение сохраняет обе, а
    BMesh получает только loops без повторных вершин.
    """

    pending = [_dedupe_polygon_loop(points)]
    simple = []
    while pending:
        polygon = pending.pop()
        first_by_key = {}
        split = None
        for index, point in enumerate(polygon):
            key = _point_cache_key(point, 6)
            previous = first_by_key.get(key)
            if previous is not None:
                split = (previous, index)
                break
            first_by_key[key] = index
        if split is None:
            if len(polygon) >= 3 and abs(_polygon_area2(polygon)) >= 1e-12:
                simple.append(polygon)
            continue

        first, second = split
        candidates = (
            polygon[first:second],
            polygon[second:] + polygon[:first],
        )
        added = False
        for candidate in candidates:
            candidate = _dedupe_polygon_loop(candidate)
            if len(candidate) < 3 or abs(_polygon_area2(candidate)) < 1e-12:
                continue
            pending.append(candidate)
            added = True
        if not added:
            # Защита от численно вырожденного spur: сохраняем исходную
            # область без повторной точки, вместо исчезновения всей face.
            unique = []
            seen = set()
            for point in polygon:
                key = _point_cache_key(point, 6)
                if key in seen:
                    continue
                seen.add(key)
                unique.append(point)
            if len(unique) >= 3 and abs(_polygon_area2(unique)) >= 1e-12:
                simple.append(unique)
    return simple


def _shared_refined_chord(
    point_a,
    point_b,
    implicit,
    tolerance,
    depth,
    curve_budget,
    boundary_cache=None,
    boundary_key=None,
):
    """Каноническая выборка одной zero-дуги для обеих сторон site-пары.

    Направление polygon loop и знак owner-implicit не должны менять точки
    общей границы. Концы сортируются в chart-space, refinement считается
    canonical implicit-функцией пары и переиспользуется обратной стороной.
    """

    key_a = _point_cache_key(point_a)
    key_b = _point_cache_key(point_b)
    reverse = key_b < key_a
    start, end = (point_b, point_a) if reverse else (point_a, point_b)
    start_key, end_key = (key_b, key_a) if reverse else (key_a, key_b)
    cache_key = None
    if boundary_cache is not None and boundary_key is not None:
        cache_key = (
            boundary_key,
            start_key,
            end_key,
            round(tolerance, 12),
            depth,
            curve_budget,
        )
        cached = boundary_cache.get(cache_key)
        if cached is not None:
            points = list(cached)
            return list(reversed(points)) if reverse else points

    budget = None if curve_budget is None else [curve_budget]
    points = _refine_chord(
        start,
        end,
        implicit,
        tolerance,
        depth,
        budget,
    )
    if cache_key is not None:
        boundary_cache[cache_key] = tuple(points)
    return list(reversed(points)) if reverse else points


def _clip_polygon(
    points,
    implicit,
    keep_eps,
    curve_tolerance,
    refine=True,
    curve_depth=_CHORD_DEPTH,
    curve_budget=None,
    boundary_cache=None,
    boundary_key=None,
    boundary_implicit=None,
    detect_hidden_crossings=True,
    root_tolerance=1e-12,
):
    """Оставляет компоненты полигона с f ≥ 0.

    Один convex band-face может быть пересечён кривой несколько раз и дать
    несколько независимых компонент. Поэтому boundary сначала раскладывается
    на inside-runs, и каждый run замыкается своей zero-дугой. Прежний единый
    ``output`` склеивал такие компоненты в самопересекающийся ngon.
    """

    values = [implicit(point) for point in points]
    if not detect_hidden_crossings:
        # Modal preview намеренно грубый: скрытая double-crossing линза будет
        # восстановлена финальным rebuild, зато drag не платит за scan всех
        # same-sign рёбер.
        if all(value >= -keep_eps for value in values):
            return [points]
        if all(value < -keep_eps for value in values):
            return []
    nodes = []  # точки boundary после вставки всех crossings
    crossing_count = 0
    count = len(points)
    for index in range(count):
        point_a = points[index]
        point_b = points[(index + 1) % count]
        value_a = values[index]
        value_b = values[(index + 1) % count]
        nodes.append((point_a, value_a, False))
        if not _edge_can_cross(point_a, point_b, value_a, value_b):
            continue
        crossings = _edge_zero_crossings(
            point_a,
            point_b,
            value_a,
            value_b,
            implicit,
            t_tolerance=root_tolerance,
        )
        for t in crossings:
            crossing = _add2(point_a, _mul2(_sub2(point_b, point_a), t))
            nodes.append((crossing, 0.0, True))
            crossing_count += 1

    if crossing_count == 0:
        # Консервативный edge-gate доказал отсутствие пересечений boundary.
        # Для обычной Voronoi half-cell знак тогда однороден; keep_eps
        # сохраняет прежнее поведение на почти tie-полигонах.
        return [points] if all(value >= -keep_eps for value in values) else []

    # Классифицируем каждый атомарный boundary-сегмент по midpoint. Это
    # корректно и для двух crossings на одном исходном ребре.
    total = len(nodes)
    inside_edges = []
    for index in range(total):
        point_a = nodes[index][0]
        point_b = nodes[(index + 1) % total][0]
        midpoint = _mul2(_add2(point_a, point_b), 0.5)
        inside_edges.append(implicit(midpoint) >= 0.0)

    if all(inside_edges):
        return [points]
    if not any(inside_edges):
        return []

    # Начинаем после гарантированно outside-сегмента, чтобы cyclic inside-run
    # не разрезался на начало/конец списка.
    start = next(index for index, inside in enumerate(inside_edges) if not inside)
    components = []
    index = (start + 1) % total
    visited = 0
    curve_implicit = boundary_implicit or implicit
    while visited < total:
        if not inside_edges[index]:
            index = (index + 1) % total
            visited += 1
            continue

        run = [nodes[index][0]]
        while visited < total and inside_edges[index]:
            run.append(nodes[(index + 1) % total][0])
            index = (index + 1) % total
            visited += 1

        # Boundary run идёт entry -> exit. Замыкаем его общей zero-дугой
        # exit -> entry, канонической для обеих сторон site-пары.
        if refine:
            run.extend(
                _shared_refined_chord(
                    run[-1],
                    run[0],
                    curve_implicit,
                    curve_tolerance,
                    curve_depth,
                    curve_budget,
                    boundary_cache,
                    boundary_key,
                )
            )

        deduped = _dedupe_polygon_loop(run)
        if len(deduped) >= 3 and abs(_polygon_area2(deduped)) >= 1e-12:
            components.append(deduped)

    return components


def _bbox_distance(bbox_a, bbox_b):
    dx = max(bbox_a[0] - bbox_b[2], bbox_b[0] - bbox_a[2], 0.0)
    dy = max(bbox_a[1] - bbox_b[3], bbox_b[1] - bbox_a[3], 0.0)
    return sqrt(dx * dx + dy * dy)


def _candidate_competitors(sites, reach):
    """Broadphase пар сайтов через плоский sweep внутри surface.

    Прежний comprehension для каждого site просматривал все остальные
    сайты, даже если их bbox находились на другом конце большой стены:
    O(S²) до первого настоящего геометрического теста. Здесь каждая
    неупорядоченная пара рассматривается один раз. Sweep axis выбирается по
    большему разбросу bbox-центров, чтобы длинные вертикальные и
    горизонтальные сети одинаково хорошо отсекались.

    Возвращаем competitors в порядке ``site_id`` — это тот же порядок, что
    у исходного списка ``sites``. Последовательность half-plane clips не
    меняется, поэтому broadphase не имеет права менять geometry output.
    """

    competitors = {site.site_id: [] for site in sites}
    sites_by_surface = {}
    for site in sites:
        sites_by_surface.setdefault(site.surface_id, []).append(site)

    for surface_sites in sites_by_surface.values():
        if len(surface_sites) < 2:
            continue
        center_x = [
            (site.bbox[0] + site.bbox[2]) * 0.5 for site in surface_sites
        ]
        center_y = [
            (site.bbox[1] + site.bbox[3]) * 0.5 for site in surface_sites
        ]
        use_x = max(center_x) - min(center_x) >= max(center_y) - min(center_y)
        min_axis = 0 if use_x else 1
        max_axis = 2 if use_x else 3
        ordered = sorted(
            surface_sites,
            key=lambda site: (
                site.bbox[min_axis],
                site.bbox[max_axis],
                site.site_id,
            ),
        )
        active = []
        for site in ordered:
            sweep_min = site.bbox[min_axis]
            active = [
                other
                for other in active
                if other.bbox[max_axis] + reach >= sweep_min
            ]
            for other in active:
                if (
                    _bbox_distance(site.bbox, other.bbox) <= reach
                    and _sites_compete(site, other)
                ):
                    competitors[site.site_id].append(other)
                    competitors[other.site_id].append(site)
            active.append(site)

    for site_competitors in competitors.values():
        site_competitors.sort(key=lambda site: site.site_id)
    return competitors


def _polygon_bbox2(points):
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return (min(xs), min(ys), max(xs), max(ys))


def _point_segment_projection2(point, point_a, point_b):
    delta = _sub2(point_b, point_a)
    length_sq = _dot2(delta, delta)
    if length_sq < 1e-24:
        return 0.0, _dist2(point, point_a)
    param = _dot2(_sub2(point, point_a), delta) / length_sq
    closest = _add2(point_a, _mul2(delta, param))
    return param, _dist2(point, closest)


def _synchronize_pair_boundaries(polygons_by_site, pairs, curve_tolerance):
    """Вставляет общий набор zero-samples в обе стороны каждой site-пары.

    Последовательный clip режет разные band-faces разными интервалами одной
    биссектрисы. Даже при каноническом refinement один face может получить
    дополнительную точку, а соседний — длинную хорду через тот же участок.
    Здесь рассматриваются только рёбра, оба конца которых лежат на конкретной
    pair-bisector; samples других пар/силуэтов принципиально не участвуют.
    """

    zero_eps = max(1e-9, curve_tolerance * 1e-7)
    chord_eps = max(1e-9, curve_tolerance * 1.25)

    for site_a, site_b in pairs:
        polygons_a = polygons_by_site.get(site_a.site_id, ())
        polygons_b = polygons_by_site.get(site_b.site_id, ())
        if not polygons_a or not polygons_b:
            continue

        def pair_implicit(point):
            return (
                site_b.competition_distance(point)
                - site_a.competition_distance(point)
            )

        samples = {}
        for polygon in list(polygons_a) + list(polygons_b):
            for point in polygon:
                if abs(pair_implicit(point)) <= zero_eps:
                    samples[_point_cache_key(point)] = point
        if len(samples) < 3:
            continue
        sample_points = list(samples.values())

        for site_id in (site_a.site_id, site_b.site_id):
            polygons = polygons_by_site.get(site_id, ())
            for polygon_index, polygon in enumerate(polygons):
                polygon_keys = {_point_cache_key(point) for point in polygon}
                rebuilt = []
                count = len(polygon)
                for edge_index, point_a in enumerate(polygon):
                    point_b = polygon[(edge_index + 1) % count]
                    rebuilt.append(point_a)
                    if (
                        abs(pair_implicit(point_a)) > zero_eps
                        or abs(pair_implicit(point_b)) > zero_eps
                    ):
                        continue
                    inserts = []
                    for point in sample_points:
                        point_key = _point_cache_key(point)
                        if point_key in polygon_keys:
                            continue
                        param, distance = _point_segment_projection2(
                            point, point_a, point_b
                        )
                        if 1e-7 < param < 1.0 - 1e-7 and distance <= chord_eps:
                            inserts.append((param, point_key, point))
                    inserts.sort(key=lambda item: (item[0], item[1]))
                    for _param, point_key, point in inserts:
                        if point_key in polygon_keys:
                            continue
                        rebuilt.append(point)
                        polygon_keys.add(point_key)
                polygons[polygon_index] = rebuilt


def _normalize_surface_arrangement(polygons_by_site, sites, tolerance):
    """Разбивает рёбра общей 2D-сети во всех уже существующих вершинах.

    Это не post-healing готового BMesh: координаты и faces не удаляются и не
    сдвигаются. Проход завершает planar arrangement до lift/UV/materialize.
    Он нужен для вырожденных multi-site случаев, где вершина третьего site
    точно попадает внутрь уже построенного pair-divider. Без split две
    корректные pair-границы всё равно образуют T-junction на общем owner
    surface.

    ``polygon_keys`` запрещает вставлять вершину, которая уже встречается в
    этом loop. Именно отсутствие этой защиты в старом final-only healer
    создавало повтор BMesh-вершины и исчезающие faces.
    """

    sites_by_surface = {}
    for site in sites:
        polygons = polygons_by_site.get(site.site_id, ())
        if polygons:
            sites_by_surface.setdefault(site.surface_id, []).append(site)

    point_eps = max(1e-9, tolerance)
    for surface_sites in sites_by_surface.values():
        samples = {}
        for site in surface_sites:
            for polygon in polygons_by_site.get(site.site_id, ()):
                for point in polygon:
                    samples.setdefault(_point_cache_key(point, 6), point)
        if len(samples) < 3:
            continue
        sample_points = list(samples.items())

        for site in surface_sites:
            normalized = []
            for polygon in polygons_by_site.get(site.site_id, ()):
                rebuilt = []
                count = len(polygon)
                for edge_index, point_a in enumerate(polygon):
                    point_b = polygon[(edge_index + 1) % count]
                    key_a = _point_cache_key(point_a, 6)
                    key_b = _point_cache_key(point_b, 6)
                    rebuilt.append(point_a)
                    min_x = min(point_a[0], point_b[0]) - point_eps
                    max_x = max(point_a[0], point_b[0]) + point_eps
                    min_y = min(point_a[1], point_b[1]) - point_eps
                    max_y = max(point_a[1], point_b[1]) + point_eps
                    inserts = []
                    edge_keys = set()
                    for point_key, point in sample_points:
                        if point_key in (key_a, key_b) or point_key in edge_keys:
                            continue
                        if not (
                            min_x <= point[0] <= max_x
                            and min_y <= point[1] <= max_y
                        ):
                            continue
                        param, distance = _point_segment_projection2(
                            point, point_a, point_b
                        )
                        if (
                            1e-7 < param < 1.0 - 1e-7
                            and distance <= point_eps
                        ):
                            inserts.append((param, point_key, point))
                            edge_keys.add(point_key)
                    inserts.sort(key=lambda item: (item[0], item[1]))
                    rebuilt.extend(point for _param, _key, point in inserts)
                normalized.extend(_split_repeated_polygon_loop(rebuilt))
            polygons_by_site[site.site_id] = normalized


def _simplify_collinear_surface_arrangement(
    polygons_by_site,
    sites,
    station_keys,
    tolerance,
):
    """Удаляет общий degree-2 sample с прямой сразу из всех faces.

    Wide overlap может оставить у одного pair-divider короткий sliver:
    source station → лишний zero-sample → следующий sample, хотя все три
    точки лежат на одной прямой. Локальный dissolve одной face создал бы
    T-junction, поэтому решение принимается по полной planar arrangement.

    Вершина удалима, только если у неё ровно два уникальных глобальных
    соседа, она лежит между ними и не является source station. Изгибы
    биссектрис, junctions и structural stations принципиально сохраняются.
    """

    sites_by_surface = {}
    for site in sites:
        if polygons_by_site.get(site.site_id):
            sites_by_surface.setdefault(site.surface_id, []).append(site)

    for surface_id, surface_sites in sites_by_surface.items():
        points_by_key = {}
        neighbors_by_key = {}
        for site in surface_sites:
            for polygon in polygons_by_site.get(site.site_id, ()):
                count = len(polygon)
                for index, point in enumerate(polygon):
                    key = _point_cache_key(point, 6)
                    prev_key = _point_cache_key(
                        polygon[index - 1], 6
                    )
                    next_key = _point_cache_key(
                        polygon[(index + 1) % count], 6
                    )
                    points_by_key.setdefault(key, point)
                    neighbors = neighbors_by_key.setdefault(key, set())
                    if prev_key != key:
                        neighbors.add(prev_key)
                    if next_key != key:
                        neighbors.add(next_key)

        removable = set()
        for key, neighbors in neighbors_by_key.items():
            if len(neighbors) != 2:
                continue
            if (surface_id, key[0], key[1]) in station_keys:
                continue
            neighbor_keys = tuple(neighbors)
            point_a = points_by_key.get(neighbor_keys[0])
            point_b = points_by_key.get(neighbor_keys[1])
            point = points_by_key.get(key)
            if point_a is None or point_b is None or point is None:
                continue
            param, distance = _point_segment_projection2(
                point, point_a, point_b
            )
            if 1e-7 < param < 1.0 - 1e-7 and distance <= tolerance:
                removable.add(key)

        if not removable:
            continue
        for site in surface_sites:
            simplified = []
            for polygon in polygons_by_site.get(site.site_id, ()):
                rebuilt = [
                    point
                    for point in polygon
                    if _point_cache_key(point, 6) not in removable
                ]
                simplified.extend(_split_repeated_polygon_loop(rebuilt))
            polygons_by_site[site.site_id] = simplified


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
    component_kind: str = "SURFACE"
    component_side: str = ""


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


def _side_surface_spans(branch, normals):
    """Maximal подпути стороны с почти постоянной плоскостью.

    Решение о разрезе локальное: сегмент продолжает span, пока его normal
    близка к бегущему среднему span и midpoint лежит на его плоскости.
    Прежний global greedy-match по registry мог флип-флопить surface id
    соседних сегментов на пороговых тесселяциях (нормали около
    DECAL_COPLANAR_DOT) и рубил сторону на per-station подпути — визуальные
    «зубцы» из flat caps и connectors на каждой станции. Бегущее среднее
    также не даёт транзитивно склеить длинную пологую дугу в одну хорду.
    """

    segment_count = len(branch.points) - 1
    spans = []  # (span_start, span_end_exclusive, avg_normal, ref_midpoint)
    span_start = None
    normal_sum = None
    ref_midpoint = None
    span_first_normal = None
    for segment in range(segment_count):
        normal = normals[segment]
        midpoint = (branch.points[segment] + branch.points[segment + 1]) * 0.5
        if normal.length_squared < 1e-12:
            if span_start is not None:
                spans.append(
                    (span_start, segment, normal_sum.normalized(), ref_midpoint)
                )
                span_start = None
            continue
        if span_start is not None:
            average = normal_sum.normalized()
            on_plane = (
                abs(average.dot(midpoint - ref_midpoint))
                < DECAL_SPINE_MERGE_DISTANCE
            )
            unit = normal.normalized()
            # Дрейф бегущего среднего не должен накапливать кривизну span
            # сверх порога: проверяем и против среднего, и против первой
            # нормали span (иначе пологая дуга склеивается чартом на 20°+).
            if (
                unit.dot(average) > DECAL_COPLANAR_DOT
                and unit.dot(span_first_normal) > DECAL_COPLANAR_DOT
                and on_plane
            ):
                normal_sum = normal_sum + unit
                continue
            spans.append(
                (span_start, segment, average, ref_midpoint)
            )
        span_start = segment
        normal_sum = normal.normalized()
        span_first_normal = normal.normalized()
        ref_midpoint = midpoint
    if span_start is not None:
        spans.append(
            (span_start, segment_count, normal_sum.normalized(), ref_midpoint)
        )
    return spans


def _split_side_sites(
    branch, branch_id, side, registry, lift_map, arcs, valence, site_counter
):
    """Режет одну сторону ветви на maximal подпути постоянной поверхности."""

    normals = branch.normals_a if side == "A" else branch.normals_b
    other = branch.normals_b if side == "A" else branch.normals_a
    segment_count = len(branch.points) - 1

    sites = []
    for span_start, span_end, span_normal, span_ref in _side_surface_spans(
        branch, normals
    ):
        surface_id = registry.surface_for(span_normal, span_ref)
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
            site.surface = surface
            for station in range(span_start, span_end + 1):
                vert = branch.vert_indices[station]
                site.pts2.append(surface.to_2d(lift_map[vert]))
                site.arcs.append(arcs[station])
                site.vert_indices.append(vert)
                site.lifts3.append(lift_map[vert])
            for segment in range(span_start, span_end):
                normal = normals[segment]
                site.segment_normals3.append(
                    normal.normalized()
                    if normal.length_squared > 1e-12
                    else span_normal.copy()
                )
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
        if normal_other.length_squared < 1e-12:
            # Boundary wing: у ребра одна owner surface, единственное крыло
            # направлено внутрь patch (n × t, chain boundary-oriented).
            wing3 = surface.normal.cross(direction3)
        elif normal_own.dot(normal_other) > DECAL_COPLANAR_DOT:
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


@dataclass(frozen=True)
class DecalNetworkPlan:
    """Статическая компиляция selected seam network для modal drag.

    Branch topology, owner surfaces, lift frames и продольные UV-факты не
    зависят от ширины. План хранится в экземпляре modal operator и явно
    передаётся evaluator'у — без global mutable cache.
    """

    offset: float
    registry: object
    lift_map: dict
    site_templates: tuple
    station_keys: dict


def compile_seam_network_plan(runs, offset):
    """Компилирует width-independent часть seam network один раз."""

    runs = [run for run in runs if len(run.points) >= 2]
    registry = _SurfaceRegistry()
    if not runs:
        return DecalNetworkPlan(
            offset=float(offset),
            registry=registry,
            lift_map={},
            site_templates=(),
            station_keys={},
        )

    branches = _merge_junction_continuations(
        [_branch_from_run(run) for run in runs]
    )
    valence = _endpoint_valence(branches)

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

    sites = []
    site_counter = [0]
    branch_arcs = []
    for branch in branches:
        arcs = [0.0]
        for index in range(1, len(branch.points)):
            arcs.append(
                arcs[-1]
                + (branch.points[index] - branch.points[index - 1]).length
            )
        branch_arcs.append(arcs)

    # Сначала регистрируем все owner planes: basis каждой surface должен
    # быть стабилен независимо от порядка последующего site split.
    for branch in branches:
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

    station_keys = {}
    for site in sites:
        # Bbox и segment cache статичны для обычных sites. Junction-sites
        # перестроят только крайний стянутый сегмент под текущий delta.
        site.finalize(0.0)
        for point2, vert in zip(site.pts2, site.vert_indices):
            station_keys[
                (site.surface_id, round(point2[0], 6), round(point2[1], 6))
            ] = vert

    return DecalNetworkPlan(
        offset=float(offset),
        registry=registry,
        lift_map=lift_map,
        site_templates=tuple(sites),
        station_keys=station_keys,
    )


def evaluate_seam_network_plan(
    plan,
    width,
    arc_tolerance=None,
    preview=False,
    _gate=True,
    _broadphase=True,
    _distance_index=True,
    _pair_cache=False,
    _site_distance_cache=True,
    _adaptive_gate=True,
    _fast_preview_roots=True,
):
    """Вычисляет width-dependent faces ранее скомпилированной сети.

    Site templates копируются неглубоко: геометрические списки и обычные
    segment caches immutable в рамках плана, а retraction/junction cache и
    shared rail принадлежат одному evaluation. Это не даёт кадрам modal
    drag загрязнять друг друга.

    preview=True — режим быстрого превью для интерактивного modal drag:
    криволинейные границы (point-vs-segment параболы) не уточняются хордами.
    Preview может быть грубее и содержать МЕНЬШЕ вершин, чем финал; это
    ожидаемо — финальная генерация (LMB/Enter, preview=False) полностью
    заменяет геометрию точной, допустим небольшой визуальный скачок.

    _gate=False отключает polygon-bbox-гейт клиппинга (только для
    differential-тестов: гейт обязан давать тот же результат, что и без него).
    _broadphase=False возвращает прежний полный scan candidate pairs для
    differential-тестов; production всегда использует sweep.
    _distance_index/_pair_cache/_site_distance_cache=False оставляют
    линейное reference-ядро для exact differential-тестов distance path.
    _adaptive_gate=False оставляет прежний constant miter gate для
    differential/performance сравнения polygon-local upper bound.
    _fast_preview_roots=False возвращает финальную t-точность crossings в
    preview; production preview использует допуск ниже weld threshold.
    """

    if not plan.site_templates:
        return []
    alpha = max(1e-6, float(width) * 0.5)
    if arc_tolerance is None:
        arc_tolerance = DECAL_WELD_DISTANCE
    delta = max(1e-9, alpha * 1e-4)
    keep_eps = 2.0 * delta

    # Профиль детализации кривой биссектрисы (число вершин на стыке лент).
    # Пересечения всегда точны (arc_tolerance / _EDGE_SAMPLES не трогаем).
    # preview форсирует HARD.
    curve_style = "HARD" if preview else DECAL_NETWORK_CLIP_CURVE_STYLE
    if curve_style == "HARD":
        clip_refine = False
        curve_tolerance = arc_tolerance
        curve_depth = 0
        curve_budget = None
    elif curve_style == "SMOOTH":
        clip_refine = True
        curve_tolerance = arc_tolerance
        curve_depth = _CHORD_DEPTH
        curve_budget = None
    else:  # LOW (default)
        clip_refine = True
        curve_tolerance = max(
            arc_tolerance, alpha * DECAL_NETWORK_CLIP_CURVE_RELATIVE
        )
        curve_depth = DECAL_NETWORK_CLIP_CURVE_DEPTH
        curve_budget = DECAL_NETWORK_CLIP_CURVE_BUDGET
    # Один профиль на round-join станций/caps и на клип-кривые.
    curve = (curve_style, curve_tolerance, curve_budget)
    root_tolerance = (
        _PREVIEW_ROOT_T_TOLERANCE
        if preview and _fast_preview_roots
        else 1e-12
    )

    # Только mutable width-state: templates и их geometry принадлежат plan.
    sites = [
        replace(
            site,
            retract_start=0.0,
            retract_end=0.0,
            cap_start_override=None,
            cap_end_override=None,
        )
        for site in plan.site_templates
    ]
    if not _distance_index:
        for site in sites:
            site._distance_nodes = ()
            site._distance_root = -1
    for site in sites:
        if (
            not site.closed
            and (site.start_kind == "junction" or site.end_kind == "junction")
        ):
            site.finalize(delta)
    registry = plan.registry
    lift_map = plan.lift_map
    station_keys = plan.station_keys
    _bridge_split_stations(sites, lift_map, alpha)

    # --- faces + клиппинг ---
    # Точка face может отстоять от своего spine на miter-длину (до α·LIMIT);
    # конкурент влияет, если сам ближе этой длины → spine bboxes в 2·α·LIMIT.
    reach = alpha * 2.0 * DECAL_CORNER_MITER_LIMIT
    result = []

    # Полигон P может быть срезан конкурентом C, только если некоторая его
    # точка ближе к C, чем к собственному спайну. Максимальное расстояние
    # точки band до спайна — α·MITER_LIMIT (острый convex-miter), поэтому
    # если bbox P дальше этого от bbox C, C гарантированно не режет P и
    # клиппинг (со всеми вызовами implicit) пропускается.
    gate_margin = alpha * DECAL_CORNER_MITER_LIMIT
    if _broadphase:
        competitors_by_site = _candidate_competitors(sites, reach)
    else:
        # Differential reference: точный прежний O(S²) full scan.
        competitors_by_site = {
            site.site_id: [
                other
                for other in sites
                if _sites_compete(site, other)
                and _bbox_distance(site.bbox, other.bbox) <= reach
            ]
            for site in sites
        }
    boundary_cache = {}
    pair_value_caches = {} if _pair_cache else None
    site_distance_caches = (
        [dict() for _site in sites] if _site_distance_cache else None
    )
    missing_value = object()
    boundary_pairs = {}
    polygons_by_site = {}
    for site in sites:
        competitors = competitors_by_site[site.site_id]
        polygons = _site_band_faces(site, alpha, curve)
        for competitor in competitors:
            pair_a, pair_b = sorted(
                (site, competitor), key=lambda candidate: candidate.site_id
            )
            pair_key = (
                site.surface_id,
                pair_a.site_id,
                pair_b.site_id,
            )
            boundary_pairs[pair_key] = (pair_a, pair_b)
            if _site_distance_cache:
                distance_values_a = site_distance_caches[pair_a.site_id]
                distance_values_b = site_distance_caches[pair_b.site_id]
                if _pair_cache:
                    pair_values = pair_value_caches.setdefault(pair_key, {})

                    def pair_implicit(
                        q,
                        _pair_a=pair_a,
                        _pair_b=pair_b,
                        _pair_values=pair_values,
                        _values_a=distance_values_a,
                        _values_b=distance_values_b,
                        _missing=missing_value,
                    ):
                        cached = _pair_values.get(q, _missing)
                        if cached is not _missing:
                            return cached
                        distance_a = _values_a.get(q, _missing)
                        if distance_a is _missing:
                            distance_a = _pair_a.competition_distance(q)
                            _values_a[q] = distance_a
                        distance_b = _values_b.get(q, _missing)
                        if distance_b is _missing:
                            distance_b = _pair_b.competition_distance(q)
                            _values_b[q] = distance_b
                        value = distance_b - distance_a
                        _pair_values[q] = value
                        return value
                else:

                    def pair_implicit(
                        q,
                        _pair_a=pair_a,
                        _pair_b=pair_b,
                        _values_a=distance_values_a,
                        _values_b=distance_values_b,
                        _missing=missing_value,
                    ):
                        distance_a = _values_a.get(q, _missing)
                        if distance_a is _missing:
                            distance_a = _pair_a.competition_distance(q)
                            _values_a[q] = distance_a
                        distance_b = _values_b.get(q, _missing)
                        if distance_b is _missing:
                            distance_b = _pair_b.competition_distance(q)
                            _values_b[q] = distance_b
                        return distance_b - distance_a
            elif _pair_cache:
                pair_values = pair_value_caches.setdefault(pair_key, {})

                def pair_implicit(
                    q,
                    _pair_a=pair_a,
                    _pair_b=pair_b,
                    _values=pair_values,
                    _missing=missing_value,
                ):
                    cached = _values.get(q, _missing)
                    if cached is not _missing:
                        return cached
                    value = _pair_b.competition_distance(
                        q
                    ) - _pair_a.competition_distance(q)
                    _values[q] = value
                    return value
            else:

                def pair_implicit(q, _pair_a=pair_a, _pair_b=pair_b):
                    return _pair_b.competition_distance(
                        q
                    ) - _pair_a.competition_distance(q)

            owner_sign = 1.0 if site.site_id == pair_a.site_id else -1.0

            def implicit(q, _owner_sign=owner_sign):
                return pair_implicit(q) * _owner_sign

            competitor_bbox = competitor.bbox
            clipped = []
            for polygon, owner_upper in polygons:
                if _gate:
                    polygon_bbox = _polygon_bbox2(polygon)
                    polygon_gate = gate_margin
                    if _adaptive_gate:
                        safe_upper = owner_upper + max(
                            1e-12, owner_upper * 1e-12
                        )
                        polygon_gate = min(polygon_gate, safe_upper)
                    if (
                        _bbox_distance(polygon_bbox, competitor_bbox)
                        > polygon_gate
                    ):
                        clipped.append(
                            (polygon, owner_upper)
                        )  # конкурент не достаёт до P
                        continue
                clipped.extend(
                    (component, owner_upper)
                    for component in _clip_polygon(
                        polygon,
                        implicit,
                        keep_eps,
                        curve_tolerance,
                        refine=clip_refine,
                        curve_depth=curve_depth,
                        curve_budget=curve_budget,
                        boundary_cache=boundary_cache,
                        boundary_key=pair_key,
                        boundary_implicit=pair_implicit,
                        detect_hidden_crossings=not preview,
                        root_tolerance=root_tolerance,
                    )
                )
            polygons = clipped
            if not polygons:
                break
        polygons_by_site[site.site_id] = [
            polygon for polygon, _owner_upper in polygons
        ]

    if not preview:
        _synchronize_pair_boundaries(
            polygons_by_site,
            boundary_pairs.values(),
            curve_tolerance,
        )
        _normalize_surface_arrangement(
            polygons_by_site,
            sites,
            _ARRANGEMENT_POINT_TOLERANCE,
        )
        _simplify_collinear_surface_arrangement(
            polygons_by_site,
            sites,
            station_keys,
            _ARRANGEMENT_POINT_TOLERANCE,
        )

    # --- lift + UV после общей 2D arrangement ---
    for site in sites:
        polygons = polygons_by_site.get(site.site_id, ())
        surface = registry.surfaces[site.surface_id]
        side_sign_uv = -1.0 if site.side == "A" else 1.0
        rail_overrides = {}
        for override in (site.cap_start_override, site.cap_end_override):
            if override is not None:
                point2, rail_key, position3 = override
                rail_overrides[
                    (round(point2[0], 6), round(point2[1], 6))
                ] = (rail_key, position3)
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
                rail = rail_overrides.get(lookup[1:])
                if vert is not None:
                    keys.append(("sv", vert))
                    positions.append(lift_map[vert].copy())
                elif rail is not None:
                    keys.append(rail[0])
                    positions.append(rail[1].copy())
                else:
                    keys.append(("p",) + lookup)
                    positions.append(site.to_3d(point2))
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


def build_seam_network_faces(
    runs,
    offset,
    width,
    arc_tolerance=None,
    preview=False,
    _gate=True,
    _broadphase=True,
    _distance_index=True,
    _pair_cache=False,
    _site_distance_cache=True,
    _adaptive_gate=True,
    _fast_preview_roots=True,
):
    """Совместимый one-shot API: compile + evaluate.

    Modal-инструмент использует обе фазы раздельно, остальные callers и
    differential-тесты сохраняют прежний контракт функции.
    """

    plan = compile_seam_network_plan(runs, offset)
    return evaluate_seam_network_plan(
        plan,
        width,
        arc_tolerance=arc_tolerance,
        preview=preview,
        _gate=_gate,
        _broadphase=_broadphase,
        _distance_index=_distance_index,
        _pair_cache=_pair_cache,
        _site_distance_cache=_site_distance_cache,
        _adaptive_gate=_adaptive_gate,
        _fast_preview_roots=_fast_preview_roots,
    )


def _rotate_rodrigues(vector, axis, cos_angle, sin_angle):
    return (
        vector * cos_angle
        + axis.cross(vector) * sin_angle
        + axis * (axis.dot(vector) * (1.0 - cos_angle))
    )


def _split_station_ends(sites):
    """Пары split-концов одной branch-стороны в общей станции."""

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
    return open_ends


def _site_end_frame(site, at_start):
    """(t3, w3, len2, len3) крайнего сегмента: t наружу вдоль движения."""

    if at_start:
        lift_a, lift_b = site.lifts3[0], site.lifts3[1]
        seg_a2, seg_b2 = site.pts2[0], site.pts2[1]
        normal = site.segment_normals3[0]
    else:
        lift_a, lift_b = site.lifts3[-2], site.lifts3[-1]
        seg_a2, seg_b2 = site.pts2[-2], site.pts2[-1]
        normal = site.segment_normals3[-1]
    tangent3 = lift_b - lift_a
    length3 = tangent3.length
    length2 = _dist2(seg_a2, seg_b2)
    if length3 < 1e-12 or length2 < 1e-12 or normal.length_squared < 1e-12:
        return None
    return tangent3 / length3, normal.normalized(), length2, length3


def _rail_chart_point(site, at_start, position3, station3):
    """2D-координата rail-вершины в чарте сайта через inverse conformal."""

    frame = _site_end_frame(site, at_start)
    if frame is None:
        return None
    tangent3, normal3, length2, length3 = frame
    lateral3 = normal3.cross(tangent3)
    if lateral3.length_squared < 1e-12:
        return None
    lateral3 = lateral3.normalized()
    delta = position3 - station3
    along = delta.dot(tangent3) * (length2 / length3)
    across = delta.dot(lateral3)
    station2 = site.pts2[0] if at_start else site.pts2[-1]
    tangent2 = (
        _norm2(_sub2(site.pts2[1], site.pts2[0]))
        if at_start
        else _norm2(_sub2(site.pts2[-1], site.pts2[-2]))
    )
    if tangent2 is None:
        return None
    return _add2(
        _add2(station2, _mul2(tangent2, along)),
        _mul2(_rot90(tangent2), across),
    )


def _bridge_split_stations(sites, lift_map, alpha):
    """Общая rail-вершина на смене поверхности внутри ветви.

    Смена owner surface — общая fold-станция внешнего rail, а не два
    flat-cap конца: соседняя грань разворачивается вокруг общей fold-оси
    (axis ∥ n_B × n_A), offset-контур строится на развёрнутой паре, единая
    outer-вершина складывается обратно в свою грань, и оба сайта ссылаются
    на один vertex key. Connector-треугольник тогда не нужен; при
    вырожденной развёртке или miter за лимитом остаётся старый fallback.
    """

    for (branch_id, side, vert), ends in _split_station_ends(sites).items():
        if len(ends) != 2:
            continue
        ending = next((s for s, at_start in ends if not at_start), None)
        starting = next((s for s, at_start in ends if at_start), None)
        if ending is None or starting is None:
            continue
        frame_a = _site_end_frame(ending, False)
        frame_b = _site_end_frame(starting, True)
        if frame_a is None or frame_b is None:
            continue
        tangent_a, normal_a, _la2, _la3 = frame_a
        tangent_b, normal_b, _lb2, _lb3 = frame_b
        station3 = lift_map[vert]
        wing_a = normal_a.cross(tangent_a) * ending.lat_sign
        wing_b3 = normal_b.cross(tangent_b) * starting.lat_sign

        # Развернуть сторону B вокруг fold-оси (нормаль B -> нормаль A).
        axis = normal_b.cross(normal_a)
        sin_angle = axis.length
        cos_angle = max(-1.0, min(1.0, normal_b.dot(normal_a)))
        if sin_angle > 1e-9:
            axis = axis / sin_angle
            tangent_b_flat = _rotate_rodrigues(
                tangent_b, axis, cos_angle, sin_angle
            )
            wing_b_flat = _rotate_rodrigues(
                wing_b3, axis, cos_angle, sin_angle
            )
        else:
            tangent_b_flat = tangent_b
            wing_b_flat = wing_b3

        # Miter offset-контуров в развёрнутой плоскости A.
        basis_x = tangent_a
        basis_y = normal_a.cross(tangent_a)

        def to2(vector3):
            return (vector3.dot(basis_x), vector3.dot(basis_y))

        rail_a_point = _mul2(to2(wing_a), alpha)
        rail_a_dir = (1.0, 0.0)
        rail_b_point = _mul2(to2(wing_b_flat), alpha)
        rail_b_dir = to2(tangent_b_flat)
        miter2 = _line_intersection2(
            rail_a_point, rail_a_dir, rail_b_point, rail_b_dir
        )
        if miter2 is None:
            if _dist2(rail_a_point, rail_b_point) > 1e-9:
                continue  # параллельные rails с разным offset — fallback
            miter2 = rail_a_point
        if _len2(miter2) > alpha * DECAL_CORNER_MITER_LIMIT:
            continue  # слишком острый разворот — connector fallback

        miter3 = station3 + basis_x * miter2[0] + basis_y * miter2[1]
        # Столбец грани: сторона fold-линии, куда смотрит развёрнутый B.
        # Ориентир — точка строго внутри развёрнутой полосы B (тангенс +
        # крыло): rail-направление вырождается, когда fold идёт вдоль
        # спайна (продольный corner), крыло — когда fold поперёк спайна.
        fold2 = to2(axis) if sin_angle > 1e-9 else None
        if fold2 is not None and _len2(fold2) > 1e-9:
            probe_b = _add2(to2(tangent_b_flat), to2(wing_b_flat))
            side_b = _cross2(fold2, probe_b)
            side_m = _cross2(fold2, miter2)
            if side_m * side_b > 0.0:
                # Точка в столбце B — сложить обратно в плоскость B.
                miter3 = station3 + _rotate_rodrigues(
                    miter3 - station3, axis, cos_angle, -sin_angle
                )

        key = ("rail", branch_id, side, vert)
        point2_a = _rail_chart_point(ending, False, miter3, station3)
        point2_b = _rail_chart_point(starting, True, miter3, station3)
        if point2_a is None or point2_b is None:
            continue
        ending.cap_end_override = (point2_a, key, miter3)
        starting.cap_start_override = (point2_b, key, miter3)


def _split_connector_faces(sites, registry, lift_map, alpha):
    """Соединительные faces на станциях смены поверхности внутри ветви.

    Fallback для случаев, где shared rail (`_bridge_split_stations`) не
    построился: общая spine-вершина уже сшита lift registry, а внешние
    flat-cap точки двух смежных сайтов соединяются треугольником.
    """

    faces = []
    for (branch_id, side, vert), ends in _split_station_ends(sites).items():
        if len(ends) != 2:
            continue
        if any(
            (site.cap_start_override if at_start else site.cap_end_override)
            is not None
            for site, at_start in ends
        ):
            continue  # смена поверхности закрыта shared rail вершиной
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
                (site.to_3d(outer2), surface.normal, arc_v, site.side)
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
