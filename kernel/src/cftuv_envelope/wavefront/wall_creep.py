"""WALL_CREEP_V1: частичное выделение швов — торец декали заподлицо с невыделенным соседом.

ЗАДАЧА. Выделена часть цепочек патча (капли только на вертикальных рёбрах). Торец декали
обязан лечь ЗАПОДЛИЦО вдоль невыделенного соседнего ребра (на вогнутом конце — вдоль его
продолжения), а на T-стыке, где выделенное ребро коллинеарно невыделенному, — поперечным резом;
митр в 45 градусов, который даёт выделение всех швов, здесь неверен.

ПОЧЕМУ НЕ НЕПОДВИЖНАЯ СТЕНА. Ребро без закона прихода — стена `q = 0`, и у неё торец как раз
заподлицо: вершина между источником и стеной скользит по прямой стены. Но фронт, который не
обязан заметать всю область, оставляет живые вершины (неподвижный стык двух стен, вершина,
уехавшая по продолжению стены внутрь), и скелет честно отвечает `WAVEFRONT_LEFT_UNRESOLVED`:
на `building` так отказывали 175 из 295 одиночных цепочек.

ЗАКОН. Стена становится ПОЛЗУЩИМ источником: `q = base * eps^2 * |d|^2`, где `base` — наименьшее
отношение `q/|d|^2` среди настоящих источников домена, а `eps = WALL_CREEP_EPSILON`. Взвешенный
скелет с положительными скоростями закрывается всегда, а бисектриса между ребром скорости 1 и
ребром скорости `eps` отстоит от прямой стены на угол порядка `eps`: торец заподлицо с ошибкой
`~eps * alpha` (при `2^-20` — около `1e-6 alpha`, ниже шага решётки). Грани ползущих стен —
полоски ширины `eps * alpha` — разбиению не принадлежат: `drop_wall_faces` убирает их из
разбиения ДО всякого потребителя (покрытие, шаблоны ширины, материализатор, нативные стадии).
Это эвристика, меняющая ответ на величину ниже решётки, и её исход записан именем: счётчики
`CONVEYOR_WALL_CREEP_V1_*` в профиле подготовки (`wall_creep_counters`). Она — ВТОРОЙ проход: домен
сперва считается с неподвижными стенами (прежний ответ побайтно, побитовая сварка торцов соседей), и
ползут только стены домена, чей скелет или грани тогда не закрылись (`needs_creeping_walls`).

T-НАДРЕЗ. Коллинеарные источник и стена с разными скоростями дают параллельные прямые без точки
пересечения (`SUPERLEVEL_COMPONENT_UNRESOLVABLE`). В вершине стыка вставляется ОДИН узел решётки
СНАРУЖИ области, на шаг вдоль `-m`, где `m` — направление движения фронта источника в координатах
карты (`G^-1 n`, единичная нормаль опоры; на скошенной карте это не евклидов перпендикуляр
решётки). Конец источника становится вогнутым против короткого ребра-ступеньки и уходит по его
прямой, то есть поперёк ребра в метрике патча. Шаг — целый вектор длины не больше
`T_NOTCH_MAX_STEP` узлов, ближайший по углу к `-m`; ошибка угла записана
(`CONVEYOR_WALL_CREEP_V1_T_NOTCH_MAX_ERROR_MICRODEG`). Надрез, который пересёк бы контур, не
ставится и считается под именем (`..._T_NOTCHES_REFUSED`).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from fractions import Fraction

from ..robust.predicates import orient2d
from .event_time import normalized_speed
from .faces import FaceOutcome
from .polygon import LoopV1, PolygonV1
from .sqrt_sum import SqrtSumV1

#: Скорость ползущей стены относительно самого медленного источника домена.
WALL_CREEP_EPSILON = Fraction(1, 1 << 20)
#: Наибольшая координата целого шага T-надреза, в узлах решётки.
T_NOTCH_MAX_STEP = 256
#: Наибольший тангенс излома между выделенным ребром и невыделенным продолжением, при котором стык — T (поперечный
#: рез), а не угол (торец заподлицо с соседом). 1/2 — около 26.6 градуса: круче торец заподлицо уходит не дальше 2 alpha.
T_NOTCH_BEND_TANGENT = Fraction(1, 2)
#: Двоичная точность корня скорости ползущей стены, когда `base * |d|^2` не квадрат.
CREEP_ROOT_BITS = 24

Point = tuple[int, int]
EdgeKey4 = tuple[int, int, int, int]


@dataclass(frozen=True, slots=True)
class WallCreepV1:
    """Полигон с ползущими стенами и всё, что о них известно."""

    polygon: PolygonV1
    #: Рёбра ползущих стен (с T-надрезами), ключами граней разбиения `(x0, y0, x1, y1)`.
    wall_keys: tuple[EdgeKey4, ...]
    creeping_wall_count: int
    t_notch_count: int
    t_notch_refused_count: int
    t_notch_max_angle_error_microdeg: int


def with_support_extent(law, source, density_metric=None):
    """Закон с концами своего опорного сегмента и направлением движения фронта (`bridge.PlainArrivalLawV1`)."""

    start = _rational_point(source.start, density_metric)
    end = _rational_point(source.end, density_metric)
    normal = source.owner_normal
    nx, ny = normal.expressions() if density_metric is None else density_metric.density_expressions(normal)
    try:
        motion = (float(nx), float(ny))
    except (TypeError, ValueError):
        motion = None
    span = None if start is None or end is None else (start, end)
    return replace(law, span=span, motion=motion)


def _rational_point(point, density_metric) -> tuple[Fraction, Fraction] | None:
    values = point.expressions() if density_metric is None else density_metric.density_expressions(point)
    out = []
    for value in values:
        if isinstance(value, (int, Fraction)) and not isinstance(value, bool):
            out.append(Fraction(value))
        elif getattr(value, "is_Rational", False):  # рациональное выражение: числитель и знаменатель как есть
            out.append(Fraction(int(value.p), int(value.q)))
        else:
            return None
    return (out[0], out[1])


def _line_key(start, end) -> tuple[Fraction, Fraction, Fraction]:
    """Прямая `a*x + b*y = c` с нормалью ВЛЕВО от хода, нормированная на положительный множитель."""

    a, b = -(end[1] - start[1]), end[0] - start[0]
    c = a * start[0] + b * start[1]
    scale = abs(a) if a != 0 else abs(b)
    return (Fraction(a) / scale, Fraction(b) / scale, Fraction(c) / scale)


def _motions_by_line(laws) -> dict:
    motions = {}
    for law in laws:
        if law.motion is None:
            continue
        a, b, c = Fraction(law.normal_x), Fraction(law.normal_y), Fraction(law.constant)
        scale = abs(a) if a != 0 else abs(b)
        if scale == 0:
            continue
        motions[(a / scale, b / scale, c / scale)] = law.motion
    return motions


def _integer_step(direction: tuple[float, float]) -> tuple[Point, float]:
    """Целый вектор не длиннее `T_NOTCH_MAX_STEP` по координате, ближайший по углу к `direction`."""

    angle = math.atan2(direction[1], direction[0])
    cos, sin = math.cos(angle), math.sin(angle)
    best: tuple[Point, float] | None = None
    for k in range(1, T_NOTCH_MAX_STEP + 1):
        if abs(cos) >= abs(sin):
            step = (k if cos > 0 else -k, round(k * sin / abs(cos)))
        else:
            step = (round(k * cos / abs(sin)), k if sin > 0 else -k)
        error = abs(math.remainder(math.atan2(step[1], step[0]) - angle, math.tau))
        if best is None or error < best[1]:
            best = (step, error)
        if error == 0.0:
            break
    return best


def _crosses(p: Point, q: Point, a: Point, b: Point) -> bool:
    """Отрезки `pq` и `ab` имеют общую точку (касание считается), кроме общих концов."""

    shared = {p, q} & {a, b}
    o1, o2 = orient2d(p, q, a), orient2d(p, q, b)
    o3, o4 = orient2d(a, b, p), orient2d(a, b, q)
    if o1 * o2 < 0 and o3 * o4 < 0:
        return True
    for point, (s, e), sign in ((a, (p, q), o1), (b, (p, q), o2), (p, (a, b), o3), (q, (a, b), o4)):
        if sign == 0 and point not in shared and min(s[0], e[0]) <= point[0] <= max(s[0], e[0]) and min(s[1], e[1]) <= point[1] <= max(s[1], e[1]):
            return True
    return False


def _notch_is_clear(polygon: PolygonV1, new_edges, replaced: tuple[Point, Point]) -> bool:
    for loop in polygon.loops:
        points = loop.points
        size = len(points)
        for index in range(size):
            a, b = points[index], points[(index + 1) % size]
            if {a, b} == set(replaced):
                continue
            if any(_crosses(p, q, a, b) for p, q in new_edges):
                return False
    return True


def _t_junction(pre, prev: Point, vertex: Point, following: Point) -> bool:
    """Излом в узле не круче `T_NOTCH_BEND_TANGENT` (тангенс поворота) В ДОРЕШЁТОЧНЫХ дробях — точно.

    Коллинеарный стык — частный случай (тангенс 0). Почти коллинеарный (выделена часть цепочки с лёгким изломом):
    заподлицо с соседом торец ушёл бы вдоль него на `alpha / tan` — при изломе 6 градусов на десять alpha.
    """

    try:
        p0, p1, p2 = pre[prev], pre[vertex], pre[following]
    except KeyError:
        return False
    d_in = (p1[0] - p0[0], p1[1] - p0[1])
    d_out = (p2[0] - p1[0], p2[1] - p1[1])
    cross = d_in[0] * d_out[1] - d_in[1] * d_out[0]
    dot = d_in[0] * d_out[0] + d_in[1] * d_out[1]
    return dot > 0 and cross * cross <= T_NOTCH_BEND_TANGENT * T_NOTCH_BEND_TANGENT * dot * dot


def _notch_loop(polygon: PolygonV1, loop: LoopV1, pre, motions, fan_points, stats):
    points, speeds = loop.points, loop.edge_speeds_squared
    size = len(points)
    out_points: list[Point] = []
    out_speeds: list = []
    for index in range(size):
        prev, vertex, following = points[index - 1], points[index], points[(index + 1) % size]
        q_in, q_out = speeds[index - 1], speeds[index]
        if bool(q_in) == bool(q_out) or vertex in fan_points or not _t_junction(pre, prev, vertex, following):
            out_points.append(vertex)
            out_speeds.append(q_out)
            continue
        start, end = (prev, vertex) if q_in else (vertex, following)
        key = _line_key(pre[start], pre[end])
        motion = motions.get(key) or motions.get(tuple(-value for value in key))
        step = None
        if motion is not None:
            left = (-(end[1] - start[1]), end[0] - start[0])
            sign = 1.0 if motion[0] * left[0] + motion[1] * left[1] > 0 else -1.0
            step, error = _integer_step((-sign * motion[0], -sign * motion[1]))
        notch = None if step is None else (vertex[0] + step[0], vertex[1] + step[1])
        if notch is not None and notch not in set(points):
            new_edges = ((vertex, notch), (notch, following)) if q_in else ((prev, notch), (notch, vertex))
            replaced = (vertex, following) if q_in else (prev, vertex)
            if not _notch_is_clear(polygon, new_edges, replaced):
                notch = None
        else:
            notch = None
        if notch is None:
            stats["refused"] += 1
            out_points.append(vertex)
            out_speeds.append(q_out)
            continue
        stats["notches"] += 1
        stats["error"] = max(stats["error"], math.degrees(error))
        if q_in:  # источник кончается в `vertex`: ступенька `vertex -> notch`, стена `notch -> following`
            out_points += [vertex, notch]
            out_speeds += [0, 0]
        else:  # источник начинается в `vertex`: стена `prev -> notch`, ступенька `notch -> vertex`
            out_points += [notch, vertex]
            out_speeds += [0, q_out]
    return tuple(out_points), tuple(out_speeds)


def _square_root_below(value: Fraction) -> Fraction:
    """Точный корень, если `value` — квадрат рационального; иначе двоичная дробь чуть меньше корня
    (относительная точность `2^-CREEP_ROOT_BITS`)."""

    numerator, denominator = value.numerator, value.denominator
    root_n, root_d = math.isqrt(numerator), math.isqrt(denominator)
    if root_n * root_n == numerator and root_d * root_d == denominator:
        return Fraction(root_n, root_d)
    shift = max(0, CREEP_ROOT_BITS - (numerator.bit_length() - denominator.bit_length()) // 2)
    return Fraction(math.isqrt((numerator << (2 * shift)) // denominator), 1 << shift)


def creeping_speed(base: Fraction, length_squared: int) -> int | Fraction:
    """`q` ползущей стены: `(eps * r)^2 * |d|^2`, где `r` — корень `base` (снизу, если он не рационален).

    Нормальная скорость стены `eps * r` — РАЦИОНАЛЬНА и ОДНА на весь домен: корень `sqrt(q)` несёт
    только радиканд `|d|^2`, как у любого ребра решётки. Иначе на карте с метрикой (почти плоский
    патч, `base` — длинная дробь) радиканд `base * |d|^2` уходил в разложение Полларда и съедал бюджет
    канонизации (`EXACT_CANONICALIZATION_WORK_BUDGET_EXHAUSTED`); а округлять корень ПО РЕБРУ нельзя —
    коллинеарные стены разной длины получили бы разные скорости, параллельные прямые без пересечения
    (`SUPERLEVEL_COMPONENT_UNRESOLVABLE`). Корень снизу — стена не быстрее закона `eps * sqrt(base)`;
    на плоской карте (`base = 1`) это ровно `base * eps^2 * |d|^2`."""

    root = _square_root_below(Fraction(base)) * WALL_CREEP_EPSILON
    return normalized_speed(root * root * length_squared)


def creep_walls(polygon: PolygonV1, loops, lattice_loops, laws) -> WallCreepV1 | None:
    """Стены полигона — ползущими источниками, с T-надрезами; `None`, если стен нет (все швы выделены)."""

    edges = polygon.edges()
    if not any(speed == 0 for _, _, speed in edges):
        return None
    sources = [
        Fraction(speed) / ((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)
        for start, end, speed in edges
        if speed
    ]
    if not sources:
        return None
    base = min(sources)
    pre = {}
    for loop, snapped in zip(loops, lattice_loops):
        for point, node in zip(loop, snapped):
            pre[node] = point
    stats = {"notches": 0, "refused": 0, "error": 0.0}
    motions = _motions_by_line(laws)
    fan_points = {fan.point for fan in polygon.vertex_fans}
    built: list[LoopV1] = []
    walls: list[EdgeKey4] = []
    for loop in polygon.loops:
        points, speeds = _notch_loop(polygon, loop, pre, motions, fan_points, stats)
        size = len(points)
        row = []
        for index, speed in enumerate(speeds):
            start, end = points[index], points[(index + 1) % size]
            if speed:
                row.append(speed)
                continue
            row.append(creeping_speed(base, (end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2))
            walls.append((start[0], start[1], end[0], end[1]))
        built.append(LoopV1(points, tuple(row)))
    return WallCreepV1(
        polygon=PolygonV1(built[0], tuple(built[1:]), polygon.vertex_fans),
        wall_keys=tuple(sorted(walls)),
        creeping_wall_count=len(walls),
        t_notch_count=stats["notches"],
        t_notch_refused_count=stats["refused"],
        t_notch_max_angle_error_microdeg=int(round(stats["error"] * 1e6)),
    )


#: Исходы скелета, которые ползущие стены не лечат: бюджет — цена, а не форма фронта.
_CREEP_DOES_NOT_CURE = frozenset({"LEVEL_BUDGET_EXHAUSTED"})


def needs_creeping_walls(prepared, creeping_done: bool = True) -> bool:
    """Домен с неподвижными стенами, чей скелет или грани не закрылись, — ещё раз с ползущими стенами.

    Неподвижные стены идут ПЕРВЫМИ: где они закрываются, торец заподлицо побайтно как прежде, и вершины на стене у
    соседних доменов совпадают побитово (сварка `WALL_MITER_OFFSET_V1`); ползущая стена отступает на `eps * alpha`
    внутрь, и пара соседних торцов разошлась бы на `2 eps alpha` — без сварки. Поэтому ползут только те домены, которые
    без этого не строятся (`WAVEFRONT_LEFT_UNRESOLVED`, коллинеарный T-стык `SUPERLEVEL_COMPONENT_UNRESOLVABLE`, ...).
    `creeping_done=False` — тот же разбор исхода для прохода, где стены уже ползут (`needs_hole_channels`).
    """

    if prepared is None or not prepared.wall_spans or prepared.bridge is None or prepared.bridge.polygon is None:
        return False
    if creeping_done and prepared.bridge.creeping_wall_count:
        return False
    skeleton = prepared.skeleton_outcome
    if skeleton is not None and skeleton.value != "EXACT":
        return skeleton.value not in _CREEP_DOES_NOT_CURE
    return prepared.face_outcome is not None and prepared.face_outcome is not FaceOutcome.EXACT


def drop_wall_faces(partition, report):
    """Разбиение без граней ползущих стен: их доля не принадлежит ни одному запросу."""

    if partition is None or not report.creeping_wall_count or partition.outcome is not FaceOutcome.EXACT:
        return partition
    walls = frozenset(report.wall_spans)
    kept = tuple(face for face in partition.faces if tuple(face.owner) not in walls)
    area = SqrtSumV1.zero()
    for face in kept:
        area = area + face.doubled_area
    return replace(partition, faces=kept, doubled_area=area)


def wall_creep_counters(regions) -> tuple[tuple[str, int], ...]:
    """Счётчики WALL_CREEP_V1 по регионам; пусты, когда ветки не было (все швы выделены — счётчики прежние)."""

    reports = [item.bridge for item in regions if item.bridge is not None]
    rows = (
        ("CONVEYOR_WALL_CREEP_V1_WALLS", sum(item.creeping_wall_count for item in reports)),
        ("CONVEYOR_WALL_CREEP_V1_T_NOTCHES", sum(item.t_notch_count for item in reports)),
        ("CONVEYOR_WALL_CREEP_V1_T_NOTCHES_REFUSED", sum(item.t_notch_refused_count for item in reports)),
        ("CONVEYOR_WALL_CREEP_V1_T_NOTCH_MAX_ERROR_MICRODEG", max((item.t_notch_max_angle_error_microdeg for item in reports), default=0)),
        ("CONVEYOR_SPAN_RESOLVED_SOURCE_EDGES", sum(item.span_resolved_edge_count for item in reports)),
        ("CONVEYOR_HOLE_CHANNEL_V1_CHANNELS", sum(item.hole_channel_count for item in reports)),
        ("CONVEYOR_HOLE_CHANNEL_V1_REFUSED", sum(item.hole_channel_refused_count for item in reports)),
        (
            "CONVEYOR_HOLE_CHANNEL_V1_MIN_DISTANCE_MICRO",
            min((item.hole_channel_min_distance_micro for item in reports if item.hole_channel_count), default=0),
        ),
    )
    return tuple(row for row in rows if row[1])
