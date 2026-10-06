"""Заверенный интервал ширины: при каких alpha структура покрытия и резки домена та же, что при alpha прогона.

ЗАЧЕМ. Живая ширина считает домен заново на каждом шаге ползунка, хотя большая часть шагов не меняет его СТРУКТУРУ:
те же вершины, те же грани, те же цепи; меняются числа, аффинные по ширине. Чтобы когда-нибудь пересчитывать только числа,
нужен ЗАВЕРЕННЫЙ ответ на вопрос «до какой ширины структура та же». Этот модуль только ЗАПИСЫВАЕТ такой ответ (поведения не
меняет): для домена, посчитанного при alpha0, это интервал `(low, high)`, внутри которого событий нет.

ЧТО ЗАВЕРЕНО (`SCOPE`). Два рода событий, и у обоих корень решается в замкнутой форме.

1. ПОКРЫТИЕ. Покрытие грани скелета - грань, обрезанная полуплоскостью `a*x + b*y - c <= alpha*sqrt(q)` (`wavefront.coverage`):
   комбинаторика обрезка меняется, лишь когда уровень проходит вершину грани. Время прихода вершины - `(a*x + b*y - c) / sqrt(q)`
   в единицах решётки; деление на масштаб решётки даёт метры.
2. РЕЗКА. Резка режет многоугольники покрытия по рёбрам и вершинам источника (`clip`); её знаки - ориентации узла у прямой ребра
   источника (с допуском `NODE_EDGE_SNAP_CELLS` ячеек у внутреннего ребра, `clip_snap`), а движется только фронт. Знак меняется,
   когда (а) узел фронта, скользящий по ребру грани `PQ`, проходит прямую `l` (ориентация аффинна по параметру узла: корень в
   замкнутой форме; события три: нуль и обе границы допуска), либо (б) уровень фронта проходит вершину источника `Z`, лежащую
   в грани (порядок пересечений фронта с рёбрами источника у вершины меняется). Ребро скользит по грани, пока уровень идёт
   между временами его концов, а время события - значение закона грани в точке пересечения.

ЧТО НЕ ЗАВЕРЕНО (`UNCERTIFIED`, названо в записи, а не молчаливо): решения тесселяции и разбиения (отсечение ушей, выпуклость),
растворение рёбер и вершин закона силуэта (допуск от alpha), закон положения вершин `src:` у хоста, план станций и UV,
решение о диагоналях ячейки (глубина хорды куска против допуска). У них нет корня в замкнутой форме; структуру они
проверяются СРАВНЕНИЕМ (`structure.batch_structure`), а не интервалом.

ЭТО ПРЕДЕЛ СВЕРХУ, А НЕ ТОЧНЫЙ ПОРОГ. Событие считается и у грани, до которой фронт ещё не дошёл, и у ребра, по которому узел
в этот момент не скользит; такие события лежат за ближайшим событием обоих краёв, поэтому ответ не меняют, а лишь могут
сузить интервал (но не расширить его). Перебор событий идёт по всем рёбрам границы грани и по ВСЕМ прямым источника, чей
габарит касается ребра с запасом допуска.

ВСЁ ЧИСЛЕННОЕ - С ГРАНИЦЕЙ ОШИБКИ. Координаты вершин скелета - суммы корней; центр и граница их `binary64` берутся у
`float_filter.centre_and_bound` (заведомо верхняя граница ошибки), а арифметика - интервальная с округлением наружу
(`math.nextafter`). Событие, чьи границы накрывают alpha0, либо не считаемое (координата не берёт `binary64`, прямая не
целочисленна, перебор не уложился в `PAIR_LIMIT`), не молчит: интервал тогда СХЛОПНУТ в точку с названным статусом.
Границы интервала округлены ВНУТРЬ: заявленное `(low, high)` лежит внутри настоящего.

ТАБЛИЦА СОБЫТИЙ - функция подготовки и подъёма домена (alpha в неё не входит), поэтому она живёт в памяти подготовки
(`memo`) и считается один раз; ответ на alpha0 - два поиска в отсортированных массивах.
"""

from __future__ import annotations

import math
from bisect import bisect_right
from dataclasses import dataclass
from fractions import Fraction

from .. import float_filter
from ..exact_sqrt_sum import SqrtSumV1
from .clip_snap import NODE_EDGE_SNAP_CELLS
from .memo import memo_of

INTERVAL_LAW = "COVERAGE_AND_CLIP_CROSSINGS_V1"
SCOPE = ("COVERAGE_NODE_ARRIVALS", "CLIP_NODE_SOURCE_LINE_CROSSINGS", "CLIP_FRONT_SOURCE_VERTEX_CROSSINGS")
UNCERTIFIED = (
    "TESSELLATION_AND_PARTITION_DECISIONS",
    "SILHOUETTE_DISSOLVE_TOLERANCE",
    "HOST_POSITION_LAW",
    "STATION_PLAN_AND_UV",
    "CELL_DIAGONAL_CHORD_VERDICT",
)

#: Исходы: интервал есть (структура покрытия и резки та же внутри него); alpha0 - сама точка события; события посчитать нельзя.
CERTIFIED = "CERTIFIED"
AT_EVENT = "AT_EVENT"
NOT_CERTIFIED = "NOT_CERTIFIED"
#: Причины `NOT_CERTIFIED`.
REASON_PARTITION = "PARTITION_NOT_EXACT"
REASON_FACE = "FACE_NOT_REPRESENTABLE_IN_BINARY64"
REASON_LINES = "SOURCE_LINES_NOT_INTEGER_LATTICE"
REASON_PAIRS = "CLIP_EVENT_PAIR_LIMIT"

#: Предел пар «ребро грани, прямая источника» на домен: тяжелее - домен не получает интервала, а причина названа.
PAIR_LIMIT = 600_000
#: Целые координаты источника не крупнее: `binary64` представляет их ТОЧНО (то же ограничение, что у фильтра знака резки).
_COORDINATE_LIMIT = 1 << 40
_INF = math.inf


# --------------------------------------------------------------------------
# Интервалы binary64 с округлением наружу
# --------------------------------------------------------------------------


def _down(value: float) -> float:
    return math.nextafter(value, -_INF)


def _up(value: float) -> float:
    return math.nextafter(value, _INF)


def _add(first, second):
    return (_down(first[0] + second[0]), _up(first[1] + second[1]))


def _sub(first, second):
    return (_down(first[0] - second[1]), _up(first[1] - second[0]))


def _mul(first, second):
    products = (first[0] * second[0], first[0] * second[1], first[1] * second[0], first[1] * second[1])
    return (_down(min(products)), _up(max(products)))


def _div(first, second):
    """Частное; делитель обязан исключать нуль (вызывающий проверил)."""

    quotients = (first[0] / second[0], first[0] / second[1], first[1] / second[0], first[1] / second[1])
    return (_down(min(quotients)), _up(max(quotients)))


def _const(value) -> tuple[float, float]:
    """Интервал вокруг целого/дроби: точный, если `binary64` его представляет."""

    number = float(value)
    return (number, number) if number == value else (_down(number), _up(number))


def _excludes_zero(interval) -> bool:
    return interval[0] > 0.0 or interval[1] < 0.0


def _sqrt_interval(value) -> tuple[float, float]:
    """`sqrt(value)` у рационального `value > 0`: перевод в `binary64` и корень дают по `2^-53` относительной ошибки."""

    root = math.sqrt(float(value))
    return (_down(root * (1.0 - 2.3e-16)), _up(root * (1.0 + 2.3e-16)))


def _coordinate(value):
    """Интервал координаты (сумма корней) либо `None`: `binary64` её не берёт."""

    entry = float_filter.centre_and_bound(value)
    if entry is None:
        return None
    return (_down(entry[0] - entry[1]), _up(entry[0] + entry[1]))


# --------------------------------------------------------------------------
# Источник: прямые и углы привязанной триангуляции
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class _Line:
    """Прямая ребра источника: целые концы, шаг, длина и допуск закона 2 (`NODE_EDGE_SNAP_CELLS` ячеек по нормали)."""

    x0: float
    y0: float
    dx: float
    dy: float
    tolerance: float
    box: tuple


def _source_geometry(triangles):
    """`(прямые, углы)` источника либо `None`, если координаты не целочисленная решётка в пределах `binary64`."""

    lines: dict = {}
    corners: set = set()
    limit = _COORDINATE_LIMIT
    for triangle in triangles:
        chart = triangle.chart
        for index, (x0, y0) in enumerate(chart):
            x1, y1 = chart[(index + 1) % len(chart)]
            if not (x0.denominator == y0.denominator == x1.denominator == y1.denominator == 1):
                return None
            ax, ay, bx, by = x0.numerator, y0.numerator, x1.numerator, y1.numerator
            if max(abs(ax), abs(ay), abs(bx), abs(by)) >= limit:
                return None
            corners.add((ax, ay))
            key = (ax, ay, bx, by) if (ax, ay) <= (bx, by) else (bx, by, ax, ay)
            if key not in lines:
                bx_, by_ = key[2] - key[0], key[3] - key[1]
                tolerance = _up(_up(math.sqrt(float(bx_ * bx_ + by_ * by_))) * float(NODE_EDGE_SNAP_CELLS))
                lines[key] = _Line(
                    float(key[0]),
                    float(key[1]),
                    float(bx_),
                    float(by_),
                    tolerance,
                    (min(key[0], key[2]), max(key[0], key[2]), min(key[1], key[3]), max(key[1], key[3])),
                )
    return tuple(lines.values()), tuple(sorted(corners))


class _Grid:
    """Равномерная сетка: `ячейка -> [номера]` по габаритам; запрос отдаёт номера, чьи габариты касаются прямоугольника."""

    def __init__(self, boxes, step: float) -> None:
        self.step = step
        self.cells: dict = {}
        for number, (x_low, x_high, y_low, y_high) in enumerate(boxes):
            for i in range(math.floor(x_low / step), math.floor(x_high / step) + 1):
                for j in range(math.floor(y_low / step), math.floor(y_high / step) + 1):
                    self.cells.setdefault((i, j), []).append(number)

    def query(self, x_low, x_high, y_low, y_high) -> set:
        step = self.step
        found: set = set()
        for i in range(math.floor(x_low / step), math.floor(x_high / step) + 1):
            for j in range(math.floor(y_low / step), math.floor(y_high / step) + 1):
                found.update(self.cells.get((i, j), ()))
        return found


# --------------------------------------------------------------------------
# Таблица событий
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class EventTableV1:
    """События alpha одной подготовки под одним законом укладки: интервалы `[lo, hi]` в метрах, по `lo` по возрастанию.

    `running_hi[i]` - наибольшее `hi` среди первых `i + 1` событий: по нему и по `lo` находятся ближайшие события и
    событие, накрывающее alpha0. `refusal` непустой - таблица не посчитана (причина названа), интервала нет.
    """

    lo: tuple
    hi: tuple
    running_hi: tuple
    vertex_events: int
    clip_events: int
    refusal: str = ""
    #: Те же три массива ТОЛЬКО по событиям прихода фронта в вершины граней скелета (без событий резки): интервал покрытия
    #: (`coverage_interval`) шире общего, и именно он решает, можно ли воспроизвести покрытие из шаблона (`wavefront.coverage_template`).
    arrival_lo: tuple = ()
    arrival_hi: tuple = ()
    arrival_running_hi: tuple = ()


def _arrival(point, a, b, c, root, scale):
    """Время прихода вершины в метрах (интервал) либо `None`: `(a*x + b*y - c) / (sqrt(q) * scale)`."""

    x, y = _coordinate(point[0]), _coordinate(point[1])
    if x is None or y is None:
        return None
    value = _sub(_add(_mul(_const(a), x), _mul(_const(b), y)), _const(c))
    return _div(_div(value, root), _const(scale))


def _band_events(first, last, line, lines_u, lines_w, span, events) -> None:
    """События узла, скользящего по ребру `PQ` от времени `first` до `last`, у прямой `line`: нуль и обе границы допуска.

    `lines_u` и `lines_w` - ориентации концов ребра у прямой (интервалы). Узел на параметре `s` стоит на ориентации
    `u + s * (w - u)` и меняет знак (либо обнуляется допуском), когда она проходит `0` либо `+-tolerance`.
    """

    tolerance = line.tolerance
    if (lines_u[0] > tolerance and lines_w[0] > tolerance) or (lines_u[1] < -tolerance and lines_w[1] < -tolerance):
        return  # ребро целиком по одну сторону полосы допуска
    difference = _sub(lines_w, lines_u)
    if not _excludes_zero(difference):
        # Ребро параллельно прямой (разность ориентаций не отделена от нуля). Целиком внутри полосы - знак нулевой на всём
        # ребре, событий нет; иначе событие неразличимо по параметру, и берётся вся длина ребра.
        if abs(lines_u[0]) < tolerance and abs(lines_u[1]) < tolerance and abs(lines_w[0]) < tolerance and abs(lines_w[1]) < tolerance:
            return
        events.append(span)
        return
    delta_time = _sub(last, first)
    for level in (0.0, tolerance, -tolerance):
        parameter = _div(_sub((level, level), lines_u), difference)
        if parameter[1] < 0.0 or parameter[0] > 1.0:
            continue  # интервал параметра (границы округлены наружу) целиком вне ребра: узла на нём при этом уровне нет
        events.append(_add(first, _mul(parameter, delta_time)))


def _build_table(prepared, triangles, scale: int) -> EventTableV1:
    faces = []
    for region in prepared.regions:
        partition = region.partition
        if partition is None or partition.outcome.value != "EXACT":
            return EventTableV1((), (), (), 0, 0, REASON_PARTITION)
        faces.extend(partition.faces)
    geometry = _source_geometry(triangles) if triangles else ((), ())
    if geometry is None:
        return EventTableV1((), (), (), 0, 0, REASON_LINES)
    lines, corners = geometry
    grid_lines = grid_corners = None
    if lines:
        step = _grid_step(faces)
        grid_lines = _Grid((line.box for line in lines), step)
        grid_corners = _Grid(((x, x, y, y) for x, y in corners), step)
        corner_points = [(SqrtSumV1.rational(x), SqrtSumV1.rational(y)) for x, y in corners]
    events: list = []
    arrivals: list = []
    vertex_events = 0
    pairs = 0
    for face in faces:
        law = face.line
        if law is None or law.q == 0 or len(face.points) < 3:
            continue  # стена: фронта нет, и событий у неё нет
        a, b, c = int(law.a), int(law.b), int(law.c)
        root = _sqrt_interval(law.q)
        times = []
        for point in face.points:
            arrival = _arrival(point, a, b, c, root, scale)
            if arrival is None:
                return EventTableV1((), (), (), 0, 0, REASON_FACE)
            times.append(arrival)
        events.extend(times)
        arrivals.extend(times)
        vertex_events += len(times)
        if not lines:
            continue
        pairs = _clip_events(
            face, times, (a, b, c, root), scale, lines, corners, corner_points, grid_lines, grid_corners, events, pairs
        )
        if pairs < 0 or pairs > PAIR_LIMIT:
            return EventTableV1((), (), (), 0, 0, REASON_PAIRS)
    lo, hi, running = _sorted_arrays(events)
    arrival_lo, arrival_hi, arrival_running = _sorted_arrays(arrivals)
    return EventTableV1(
        lo, hi, running, vertex_events, len(events) - vertex_events, "", arrival_lo, arrival_hi, arrival_running
    )


def _sorted_arrays(events: list) -> tuple:
    """`(lo, hi, running_hi)` событий по `lo` по возрастанию."""

    events.sort(key=lambda item: item[0])
    lo = tuple(item[0] for item in events)
    hi = tuple(item[1] for item in events)
    running, best = [], -_INF
    for value in hi:
        best = max(best, value)
        running.append(best)
    return lo, hi, tuple(running)


def _grid_step(faces) -> float:
    """Шаг сетки: порядка средней стороны грани, не меньше одной ячейки решётки."""

    sizes = []
    for face in faces:
        entries = [float_filter.centre_and_bound(coordinate) for point in face.points for coordinate in point]
        if any(entry is None for entry in entries):
            continue
        xs, ys = [entries[i][0] for i in range(0, len(entries), 2)], [entries[i][0] for i in range(1, len(entries), 2)]
        sizes.append(max(max(xs) - min(xs), max(ys) - min(ys)))
    return max(1.0, (sum(sizes) / len(sizes)) if sizes else 1.0)


def _clip_events(face, times, law, scale, lines, corners, corner_points, grid_lines, grid_corners, events, pairs) -> int:
    """События резки грани: рёбра грани против прямых источника и вершины источника внутри грани. Возвращает число пар (или -1)."""

    a, b, c, root = law
    points = face.points
    count = len(points)
    coordinates = []
    for point in points:
        x, y = _coordinate(point[0]), _coordinate(point[1])
        if x is None or y is None:
            return -1
        coordinates.append((x, y))
    # Ориентация грани: скелет обходит её против часовой, но знак площади читается, а не предполагается.
    sign = face.doubled_area.certified_sign(64)
    orientation = -1 if sign is not None and sign < 0 else 1
    estimates: dict = {}

    def estimate(vertex: int, number: int):
        key = (vertex, number)
        found = estimates.get(key)
        if found is None:
            line = lines[number]
            entry = float_filter.line_estimate(points[vertex], line.x0, line.y0, line.dx, line.dy)
            found = estimates[key] = None if entry is None else (_down(entry[0] - entry[1]), _up(entry[0] + entry[1]))
        return found

    for index in range(count):
        following = (index + 1) % count
        (px, py), (qx, qy) = coordinates[index], coordinates[following]
        x_low, x_high = min(px[0], qx[0]), max(px[1], qx[1])
        y_low, y_high = min(py[0], qy[0]), max(py[1], qy[1])
        span = (min(times[index][0], times[following][0]), max(times[index][1], times[following][1]))
        # Прямая влияет на ребро, если её габарит, расширенный на допуск, касается габарита ребра.
        for number in grid_lines.query(x_low, x_high, y_low, y_high):
            line = lines[number]
            box = line.box
            margin = line.tolerance
            if box[1] + margin < x_low or box[0] - margin > x_high or box[3] + margin < y_low or box[2] - margin > y_high:
                continue
            pairs += 1
            first, second = estimate(index, number), estimate(following, number)
            if first is None or second is None:
                events.append(span)
                continue
            _band_events(times[index], times[following], line, first, second, span, events)
    # Вершины источника внутри грани: уровень фронта проходит вершину - порядок пересечений у неё меняется.
    box_x = (min(item[0][0] for item in coordinates), max(item[0][1] for item in coordinates))
    box_y = (min(item[1][0] for item in coordinates), max(item[1][1] for item in coordinates))
    inside = grid_corners.query(box_x[0], box_x[1], box_y[0], box_y[1])
    edges = [(points[index], points[(index + 1) % count]) for index in range(count)]
    for number in inside:
        zx, zy = corners[number]
        if zx < box_x[0] or zx > box_x[1] or zy < box_y[0] or zy > box_y[1]:
            continue
        pairs += 1
        z = corner_points[number]
        outside = False
        for start, end in edges:
            side = float_filter.orientation_sign(start, end, z)
            if side is not None and side == -orientation:
                outside = True
                break
        if outside:
            continue
        value = a * zx + b * zy - c
        events.append(_div(_div(_const(value), root), _const(scale)))
    return pairs


@dataclass(frozen=True, slots=True)
class AlphaIntervalV1:
    """Интервал ширины домена: `(low, high)` метров, внутри которого событий покрытия и резки нет.

    `status`: `CERTIFIED` - интервал есть; `AT_EVENT` - alpha0 накрыта границами события, окрестности нет (`low == high == alpha`);
    `NOT_CERTIFIED` - события не посчитаны (`reason` называет почему), интервал схлопнут в точку. `high is None` - выше
    alpha0 событий нет. Границы округлены внутрь. Заверено то, что названо в `SCOPE`; чего нет, названо в `UNCERTIFIED`.
    """

    status: str
    reason: str
    alpha: float
    low: float
    high: float | None
    #: Событий в таблице подготовки (вершины скелета и резка) и из них резки.
    events: int
    clip_events: int

    def contains(self, alpha: float) -> bool:
        """`alpha` лежит ВНУТРИ заверенного интервала (событий между ним и alpha0 нет); концы - сами события."""

        if self.status != CERTIFIED:
            return False
        return self.low < alpha and (self.high is None or alpha < self.high)

    def contains_exact(self, alpha: Fraction) -> bool:
        """Тот же вопрос, что `contains`, но ТОЧНЫЙ: `alpha` - дробь (десятичная ширина запроса), границы - их `binary64` как дроби."""

        if self.status != CERTIFIED:
            return False
        return Fraction(self.low) < alpha and (self.high is None or alpha < Fraction(self.high))

    def as_record(self) -> dict:
        """Запись для квитанции: числа и названия, без секунд."""

        return {
            "law": INTERVAL_LAW,
            "status": self.status,
            "reason": self.reason,
            "alpha": self.alpha,
            "low": self.low,
            "high": self.high,
            "events": self.events,
            "clip_events": self.clip_events,
            "scope": list(SCOPE),
            "uncertified": list(UNCERTIFIED),
        }


def event_table(prepared, triangles, lift_key: str) -> EventTableV1:
    """Таблица событий подготовки под законом укладки `lift_key`: считается один раз и лежит в памяти подготовки."""

    scale = 1 if prepared.lattice is None else int(prepared.lattice.scale)
    memo = memo_of(prepared)
    key = ("alpha_events", lift_key, scale, len(triangles))

    def compute():
        return _build_table(prepared, triangles, scale)

    return compute() if memo is None else memo.remembered(key, compute)


def alpha_interval(prepared, alpha: Fraction, triangles=(), lift_key: str = "") -> AlphaIntervalV1:
    """Заверенный интервал ширины домена, посчитанного при `alpha` (метры, точная дробь).

    `triangles` - треугольники подъёма под резкой (`plane.triangles`); пусто - домен без резки, события только покрытия.
    """

    alpha = Fraction(alpha)
    exact = float(alpha)
    table = event_table(prepared, tuple(triangles), lift_key)
    if table.refusal:
        return AlphaIntervalV1(NOT_CERTIFIED, table.refusal, exact, exact, exact, 0, 0)
    return _interval_between(table.lo, table.running_hi, exact, table.clip_events)


def coverage_interval(prepared, alpha: Fraction, triangles=(), lift_key: str = "") -> AlphaIntervalV1:
    """Интервал ширины, внутри которого комбинаторика ПОКРЫТИЯ та же (приходы фронта в вершины граней скелета), без событий резки.

    Шире `alpha_interval`: покрытие не знает об источнике. По нему решается, можно ли воспроизвести покрытие из шаблона
    (`wavefront.coverage_template`); остальные стадии шага ширины считаются своим кодом на тех же точках.
    """

    alpha = Fraction(alpha)
    exact = float(alpha)
    table = event_table(prepared, tuple(triangles), lift_key)
    if table.refusal:
        return AlphaIntervalV1(NOT_CERTIFIED, table.refusal, exact, exact, exact, 0, 0)
    return _interval_between(table.arrival_lo, table.arrival_running_hi, exact, 0)


def _interval_between(lo: tuple, running_hi: tuple, exact: float, clip_events: int) -> AlphaIntervalV1:
    """Ближайшие события по обе стороны `exact` (или сама точка события): два поиска в отсортированных массивах."""

    near = (_down(exact), _up(exact))
    covered = bisect_right(lo, near[1])
    if covered and running_hi[covered - 1] >= near[0]:
        return AlphaIntervalV1(AT_EVENT, "", exact, exact, exact, len(lo), clip_events)
    low = running_hi[covered - 1] if covered else 0.0
    high = lo[covered] if covered < len(lo) else None
    return AlphaIntervalV1(CERTIFIED, "", exact, max(low, 0.0), high, len(lo), clip_events)
