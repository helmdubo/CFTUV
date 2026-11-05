"""Закон `STRAIGHTEN_RUNGS_V1` (режим Straight strips, политика `UV_DIRECT_STRIP_V1`): одна pChain - прямой прямоугольный UV-остров.

Физические факты `(s, r)` (`mesh.station_values`) не трогаются: регионы, цепи, станции батча и решения топологии читают их, как раньше. Здесь считаются ТОЛЬКО числители UV `(U, r)`:
`u = U / alpha`, `v = r / alpha` (`uv_law.uv_direct_strip_v1`), тем же путём `uv_numerators`, что у карт Uniform.

Правила, по грани `F` пробега `P`, в её собственных координатах `(s, r)`:

* ОСНОВАНИЕ. Вершина на источнике (`r = 0`) - `U = s`, накопленная длина цепи.
* БОКА. Контур грани от концов основания вверх - два бока. Сторона бока идёт, пока `r` строго растёт и подъём не положе 45 градусов (`|ds| <= |dr|`; первое ребро от вершины цепи - сторона бока при любом наклоне).
  Положе - это вершина фронта либо гребень против другой цепи, там бок кончается. Порог меняет ответ, поэтому граница не молчит: всё, что выше боков, идёт законом уровня ниже. На боку `U` постоянна, то есть перекладина от вершины цепи и торец острова вертикальны.
* КЛИН. Вершина, которой отвечают несколько пробегов региона, общая для граней: соседним пробегам - станция вершины цепи между ними (`ChainStationTableV1.join_station`), пробегам со съеденным куском цепи между ними -
  середина съеденного куска `(s_конец(a) + s_начало(b)) / 2`. Вертикально всё ребро скелета выше события; перекладины съеденного куска сходятся в одну точку и не могут быть вертикальны, их число названо (`SLANTED_CONSUMED`).
  Если съедена КРАЙНЯЯ грань потока, узел события получает `U` конца острова, а не станцию стыка: контур острова важнее внутренней перекладины, торец идёт вертикально сквозь событие, и следующая грань наследует этот торец (`END_CONSUMED`). Съедены обе крайние - узел это вершина острова, середина (`APEX_MEAN`).
* УРОВЕНЬ. Любая другая вершина (фронт, гребень, вершина резки) берёт `U` по своей доле поперёк грани на СВОЁМ `r`: интервал `[a(r), b(r)]` между боками линейно ложится на `[U(a), U(b)]`. Бок, кончившийся ниже `r`, продолжается вертикально от своей вершины (гребень в UV идёт от бока к боку, верх острова неровный при `v < 1`).
  Внутренние вершины цепи в слитом пробеге - узлы этого отображения: между ними `U = s`, крайние столбцы ложатся на свои отрезки.
* ВЕРШИНА ГРАНИ. Оба бока сошлись в одной вершине, которой не отвечает никто другой (одна грань-треугольник на остров): у неё нет вертикального ответа, берётся середина и число названо (`APEX_MEAN`).
* ВЕЕР. Постоянная станция дала бы нулевую площадь в UV, поэтому вершина веера получает `U = s_v + t`, где `t` - проекция на ось веера: единичную касательную, сумму касательных входящего и исходящего рёбер угла (для всех углов веера меньше развёрнутого проекция строго монотонна по углу, то есть нулевой площади и выворота нет). Остров веера отделён швом от полос, это его прежний статус.

* ВЫВОРОТ. Съеденный кусок, клин и зажатая доля могут вывернуть маленький треугольник итоговой грани (триангуляция - ушами в карте домена, как у хоста). Вершина выше основания такого треугольника сдвигается по `u` (`v` закон не отдаёт) до ближайшего значения, при котором ни один её треугольник не вывернут; это отступление от вертикали, и оно названо (`FLIP_MOVED`). Нет такого значения - выворот остаётся и назван (`FLIP_LEFT`). Треугольник нулевой площади на прямой перекладине (три вершины одного столбца, изгиб 3D на вертикали UV) не трогается: вертикаль - требование закона, он назван (`UV_ZERO_AREA`).

ТОЧНОСТЬ. Основание и бока - точные `SqrtSumV1`, поэтому равенство UV двух граней на общем ребре побитовое. Доля уровня считается в binary64 от середин строгих оболочек (ни `libm`, ни порядок не влияют на результат: только `+ - * /`), затем становится точным рациональным множителем; радикалы не делятся и бюджет канонизации не тратится.

ЖИВАЯ ШИРИНА. Числа `(k, tx, ty, b)` (`uv_forms`) - те же, что у Uniform: у основания и боков наклон `U` нулевой (точно), у вершин веера - касательная (точно аффинно), у долей уровня это ЛИНЕАРИЗАЦИЯ в текущей ширине: дробно-линейную долю точный кадр после паузы считает заново, а доменов-долей `LEVEL_VERTICES` назван.

ЧИСЛА (`counters`, только ненулевые): то, что закон не смог сделать буквально, и выводы из него; ни один исход не молчит.
"""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction

from ..exact_sqrt_sum import SqrtSumV1
from .lift import sqrt_sum_binary64
from .mesh import _flow_positions

ZERO = SqrtSumV1.zero()
_ANCHORED = (ZERO, ZERO, ZERO)

#: Перекладины, которым закон не может дать вертикаль, потому что съеденный кусок цепи сходится в узел события: узлов события.
SLANTED_CONSUMED = "MATERIALIZE_STRAIGHTEN_SLANTED_CONSUMED_EVENTS"
#: Узлов события, где съедена крайняя грань потока: узел получил `U` торца острова, перекладина съеденной грани наклонена.
END_CONSUMED = "MATERIALIZE_STRAIGHTEN_END_CONSUMED_EVENTS"
#: Вершина, где сошлись оба бока единственной грани: взята середина.
APEX_MEAN = "MATERIALIZE_STRAIGHTEN_APEX_MEAN"
#: Вершины, получившие `U` по доле уровня.
LEVEL_VERTICES = "MATERIALIZE_STRAIGHTEN_LEVEL_VERTICES"
#: Доля уровня вне `[0, 1]` (вершина за боком, у съеденного бока либо у гребня): взята граница.
FRACTION_CLAMPED = "MATERIALIZE_STRAIGHTEN_FRACTION_CLAMPED"
#: Интервал уровня нулевой ширины: взята середина.
LEVEL_WIDTH_ZERO = "MATERIALIZE_STRAIGHTEN_LEVEL_WIDTH_ZERO"
#: Грань без основания в контуре: у неё нет боков, `U` остаётся физической станцией.
FACES_WITHOUT_SOURCE = "MATERIALIZE_STRAIGHTEN_FACES_WITHOUT_SOURCE"
#: Вершина клина, пробеги которой не стоят в одном потоке (резка назвала её по фактам): `U` - физическая станция её факта.
WEDGE_FROM_FACT = "MATERIALIZE_STRAIGHTEN_WEDGE_FROM_FACT"
#: Две грани одного пробега предложили одной вершине разные `U`: взято первое.
PROPOSALS_DIFFER = "MATERIALIZE_STRAIGHTEN_PROPOSALS_DIFFER"
#: Вершины веера, получившие проекцию на ось.
FAN_VERTICES = "MATERIALIZE_STRAIGHTEN_FAN_VERTICES"

#: Вершины, сдвинутые по `u`, чтобы треугольник итоговой грани не был вывернут.
FLIP_MOVED = "MATERIALIZE_STRAIGHTEN_FLIP_MOVED"
#: Вывернутые треугольники, которые сдвиг не исправил.
FLIP_LEFT = "MATERIALIZE_STRAIGHTEN_FLIP_LEFT"
#: Треугольники итоговых граней с нулевой площадью в UV (три вершины одной вертикали).
UV_ZERO_AREA = "MATERIALIZE_STRAIGHTEN_UV_ZERO_AREA"

COUNTER_NAMES = (SLANTED_CONSUMED, END_CONSUMED, APEX_MEAN, LEVEL_VERTICES, FRACTION_CLAMPED, LEVEL_WIDTH_ZERO, FACES_WITHOUT_SOURCE, WEDGE_FROM_FACT, PROPOSALS_DIFFER, FAN_VERTICES, FLIP_MOVED, FLIP_LEFT, UV_ZERO_AREA)

#: Проходов исправления: у вершины выворота мало соседей, больше двух проходов на полях не понадобилось; остаток назван.
_FLIP_SWEEPS = 4


def _f(value: SqrtSumV1) -> float:
    return sqrt_sum_binary64(value)


def _exact(number: float) -> SqrtSumV1:
    return SqrtSumV1.rational(Fraction(number))


@dataclass(slots=True)
class _Side:
    """Бок грани: точки `(r, s, U число, U точно)` снизу вверх."""

    points: list

    def at(self, level: float) -> tuple:
        """`(s, U число, ds/dr, dU/dr)` на уровне: линейно между точками бока, выше конца бока - вертикальное продолжение (наклоны нули)."""

        points = self.points
        if level >= points[-1][0]:
            return points[-1][1], points[-1][2], 0.0, 0.0
        for low, high in zip(points, points[1:]):
            if level <= high[0]:
                span = high[0] - low[0]
                slope_s, slope_u = (high[1] - low[1]) / span, (high[2] - low[2]) / span
                return low[1] + slope_s * (level - low[0]), low[2] + slope_u * (level - low[0]), slope_s, slope_u
        return points[-1][1], points[-1][2], 0.0, 0.0


@dataclass(slots=True)
class _Column:
    """Что грань знает о себе: бока, узлы основания (`s` число по возрастанию) и формула своей станции."""

    low: _Side
    high: _Side
    knots: tuple
    run: object
    tangent: tuple


class _Tally:
    def __init__(self) -> None:
        self.numbers = dict.fromkeys(COUNTER_NAMES, 0)

    def add(self, name: str, count: int = 1) -> None:
        self.numbers[name] += count

    def counters(self) -> tuple:
        return tuple((name, value) for name, value in self.numbers.items() if value)


def _run_origin(run) -> tuple:
    inverse = _f(run.inverse_length)
    return (
        _f(run.s_origin),
        float(run.origin[0]),
        float(run.origin[1]),
        float(run.covector[0]) * inverse,
        float(run.covector[1]) * inverse,
    )


def _wedge_values(table, positions, following, runs_at, facts, faces_of_run, r_of, tally):
    """`{слот: U}` вершин, которым отвечают несколько пробегов региона (`КЛИН`), снизу вверх: состояние торцов идёт от события к событию."""

    wedge: dict = {}
    entered = set(following.values())
    low_end: dict = {}
    high_end: dict = {}
    for run, (_head, _number) in positions.items():
        faces = faces_of_run.get(run, ())
        if len(faces) != 1:
            continue
        if run not in entered:
            low_end[run] = faces[0].low_value
        if run not in following:
            high_end[run] = faces[0].high_value
    pending = [(slot, runs) for slot, runs in runs_at.items() if len(runs) >= 2]
    pending.sort(key=lambda item: (r_of(item[0]), item[0]))
    for slot, runs in pending:
        if not all(run in positions for run in runs) or len({positions[run][0] for run in runs}) != 1:
            tally.add(WEDGE_FROM_FACT)
            wedge[slot] = facts[slot][0]
            continue
        ordered = sorted(runs, key=lambda run: positions[run][1])
        first, last = ordered[0], ordered[-1]

        def collapses(run) -> bool:
            faces = faces_of_run.get(run, ())
            return len(faces) == 1 and faces[0].apex == slot[1]

        low_gone = first in low_end and collapses(first)
        high_gone = last in high_end and collapses(last)
        if low_gone and high_gone:
            wedge[slot] = (low_end[first] + high_end[last]).scaled(Fraction(1, 2))
            tally.add(APEX_MEAN)
        elif low_gone:
            wedge[slot] = low_end[last] = low_end[first]
            tally.add(END_CONSUMED)
        elif high_gone:
            wedge[slot] = high_end[first] = high_end[last]
            tally.add(END_CONSUMED)
        elif positions[last][1] - positions[first][1] == 1:
            wedge[slot] = table.join_station(first, last)
        else:
            wedge[slot] = (table.runs[following[first]].s_origin + table.runs[last].s_origin).scaled(Fraction(1, 2))
            tally.add(SLANTED_CONSUMED)
    return wedge


def _source_span(keys, slots, facts):
    """Индексы подряд идущих вершин контура с `r = 0` (циклически, основание грани) либо `None`, если такой дуги ровно одной нет."""

    size = len(keys)
    flags = [facts[slot][1].is_zero for slot in slots]
    if sum(flags) < 2:
        return None
    starts = [index for index in range(size) if flags[index] and not flags[index - 1]]
    if len(starts) != 1:
        return None
    span = [starts[0]]
    while flags[(span[-1] + 1) % size] and len(span) < size:
        span.append((span[-1] + 1) % size)
    return span


def _walk(start, step, slots, floats, facts):
    """Индексы контура бока от конца основания снизу вверх (структура; значения - отдельно, `_side_values`)."""

    size = len(slots)
    found = [start]
    at = start
    for number in range(size - 1):
        following = (at + step) % size
        rise = floats[following][1] - floats[at][1]
        if rise <= 0.0 or facts[slots[following]][1].is_zero:
            break
        if number and abs(floats[following][0] - floats[at][0]) > rise:
            break
        found.append(following)
        at = following
    return found


def _side_values(walk, slots, facts, wedge) -> list:
    """`[(индекс контура, U точно), ...]`: `U` бока постоянна, клин её задаёт заново."""

    value = facts[slots[walk[0]]][0]
    found = []
    for at in walk:
        known = wedge.get(slots[at])
        value = value if known is None else known
        found.append((at, value))
    return found


@dataclass(slots=True)
class _Face:
    """Разобранная полоса: контур, числа, основание, индексы боков."""

    frame_face: object
    region: int
    slots: list
    floats: list
    span: list | None
    low_walk: list
    high_walk: list
    knots: tuple
    low_value: object
    high_value: object
    #: Ключ вершины, в которой сошлись оба бока, либо `None`.
    apex: str | None


def _face_of(frame_face, cycle, region, facts, floats_of, tally) -> _Face:
    keys = [key for key, _point in cycle]
    slots = [(region, key) for key in keys]
    origin_s, origin_x, origin_y, cov_x, cov_y = _run_origin(frame_face.run)
    floats = []
    for slot, key in zip(slots, keys):
        x, y = floats_of(key)
        floats.append((origin_s + (x - origin_x) * cov_x + (y - origin_y) * cov_y, _f(facts[slot][1])))
    span = _source_span(keys, slots, facts)
    if span is None:
        tally.add(FACES_WITHOUT_SOURCE)
        return _Face(frame_face, region, slots, floats, None, [], [], (), None, None, None)
    head, tail = span[0], span[-1]
    forward, backward = _walk(tail, 1, slots, floats, facts), _walk(head, -1, slots, floats, facts)
    low_walk, high_walk = (backward, forward) if floats[head][0] <= floats[tail][0] else (forward, backward)
    knots = tuple(sorted(floats[at][0] for at in span[1:-1])) if len(span) > 2 else ()
    apex = keys[low_walk[-1]] if low_walk[-1] == high_walk[-1] and len(low_walk) > 1 else None
    return _Face(frame_face, region, slots, floats, span, low_walk, high_walk, knots, facts[slots[low_walk[0]]][0], facts[slots[high_walk[0]]][0], apex)


def _column_of(face: _Face, facts, wedge, proposals, tally) -> _Column | None:
    """Бока грани и предложения её вершин, либо `None`: у грани нет основания (названо)."""

    slots, floats = face.slots, face.floats
    if face.span is None:
        for slot in slots:
            proposals.setdefault(slot, facts[slot][0])
        return None
    low, high = _side_values(face.low_walk, slots, facts, wedge), _side_values(face.high_walk, slots, facts, wedge)
    if face.apex is not None and slots[low[-1][0]] not in wedge:
        # Оба бока сошлись в одной вершине, которой не отвечает никто другой: вертикального ответа нет, середина.
        middle = (low[0][1] + high[0][1]).scaled(Fraction(1, 2))
        low[-1] = (low[-1][0], middle)
        high[-1] = (high[-1][0], middle)
        tally.add(APEX_MEAN)
    for at, value in (*low, *high):
        slot = slots[at]
        held = proposals.setdefault(slot, value)
        if held is not value and held != value:
            tally.add(PROPOSALS_DIFFER)
    for at in face.span:
        proposals.setdefault(slots[at], facts[slots[at]][0])

    def side(walk) -> _Side:
        return _Side([(floats[at][1] if number else 0.0, floats[at][0], _f(value), value) for number, (at, value) in enumerate(walk)])

    run = face.frame_face.run
    tangent = (run.inverse_length.scaled(run.covector[0]), run.inverse_length.scaled(run.covector[1]))
    return _Column(side(low), side(high), face.knots, run, tangent)


def _level_value(column: _Column, s: float, r: float, own, tally):
    """`(U точно, форма (k, tx, ty, b))` вершины по доле на её уровне (`УРОВЕНЬ`)."""

    a, value_a, slope_a, slope_ua = column.low.at(r)
    b, value_b, slope_b, slope_ub = column.high.at(r)
    knots = column.knots
    if knots:
        if knots[0] <= s <= knots[-1]:
            # Между внутренними вершинами цепи столбцы прямые по построению: станция грани и есть ответ.
            return own, (own, *column.tangent, ZERO)
        if s < knots[0]:
            b, value_b, slope_b, slope_ub = knots[0], knots[0], 0.0, 0.0
        else:
            a, value_a, slope_a, slope_ua = knots[-1], knots[-1], 0.0, 0.0
    width = b - a
    if width <= 0.0:
        tally.add(LEVEL_WIDTH_ZERO)
        share, ds, dr = 0.5, 0.0, 0.0
    else:
        share = (s - a) / width
        if share < 0.0 or share > 1.0:
            tally.add(FRACTION_CLAMPED)
            share = min(1.0, max(0.0, share))
        spread = value_b - value_a
        ds = spread / width
        slope_share = (-slope_a - share * (slope_b - slope_a)) / width
        dr = slope_ua + share * (slope_ub - slope_ua) + spread * slope_share
    tally.add(LEVEL_VERTICES)
    tangent_x = column.tangent[0].scaled(Fraction(ds))
    tangent_y = column.tangent[1].scaled(Fraction(ds))
    return _exact(value_a + share * (value_b - value_a)), (ZERO, tangent_x, tangent_y, _exact(dr))


def _unit(gram, vector) -> tuple | None:
    g00, g01, g11 = gram
    x, y = float(vector[0]), float(vector[1])
    squared = float(g00) * x * x + 2.0 * float(g01) * x * y + float(g11) * y * y
    if squared <= 0.0:
        return None
    length = squared**0.5
    return x / length, y / length


def _fan_axis(frame_face, table):
    """`(tx, ty)` проекции веера: метрическая единичная ось вдоль суммы касательных угла, по направлению пробега. `None`: угла нет."""

    node = (int(frame_face.face.owner[0]), int(frame_face.face.owner[1]))
    gram = table.gram
    run = frame_face.run
    for corner in table.corners.get((frame_face.face.region_id, node), ()):
        incoming = _unit(gram, (corner.incoming[2] - corner.incoming[0], corner.incoming[3] - corner.incoming[1]))
        outgoing = _unit(gram, (corner.outgoing[2] - corner.outgoing[0], corner.outgoing[3] - corner.outgoing[1]))
        if incoming is None or outgoing is None:
            continue
        sum_x, sum_y = incoming[0] + outgoing[0], incoming[1] + outgoing[1]
        axis = _unit(gram, (sum_x, sum_y))
        if axis is None:
            continue
        # В ковектор: `<p, axis>_G`, знак - по направлению пробега, чтобы `u` росла как у полос.
        g00, g01, g11 = (float(item) for item in gram)
        cov = (g00 * axis[0] + g01 * axis[1], g01 * axis[0] + g11 * axis[1])
        direction = run.direction
        if cov[0] * float(direction[0]) + cov[1] * float(direction[1]) < 0.0:
            cov = (-cov[0], -cov[1])
        return node, cov
    return None


def _ears(ring: list, floats_of) -> tuple:
    """`(треугольники по номерам кольца, знак площади кольца в карте)`: уши в карте домена, как у хоста; кольцо, у которого уха нет, - веером."""

    points = [floats_of(key) for key in ring]
    size = len(points)
    doubled = sum(points[at][0] * points[(at + 1) % size][1] - points[(at + 1) % size][0] * points[at][1] for at in range(size))
    sign = 1.0 if doubled >= 0.0 else -1.0
    if size == 3:
        return ((0, 1, 2),), sign

    def turn(a, b, c) -> float:
        return sign * ((points[b][0] - points[a][0]) * (points[c][1] - points[a][1]) - (points[b][1] - points[a][1]) * (points[c][0] - points[a][0]))

    left = list(range(size))
    found = []
    while len(left) > 3:
        for number, b in enumerate(left):
            a, c = left[number - 1], left[(number + 1) % len(left)]
            if turn(a, b, c) > 0.0 and not any(j not in (a, b, c) and turn(a, b, j) >= 0.0 and turn(b, c, j) >= 0.0 and turn(c, a, j) >= 0.0 for j in left):
                found.append((a, b, c))
                left.pop(number)
                break
        else:
            found.append(tuple(left[:3]))
            left.pop(1)
    found.append(tuple(left))
    return tuple(found), sign


def _repair_flips(frame_faces, layout, final_polygons, floats_of, values, forms, tally) -> None:
    """Закон `ВЫВОРОТ`: на месте меняет `values` и `forms` вершин, которые двигает; считает `FLIP_MOVED`, `FLIP_LEFT`, `UV_ZERO_AREA`."""

    triangles: list = []
    balance: dict = {}
    uv: dict = {}

    def uv_of(slot) -> tuple:
        found = uv.get(slot)
        if found is None:
            pair = values[slot]
            found = uv[slot] = (_f(pair[0]), _f(pair[1]))
        return found

    def area(a, b, c) -> float:
        pa, pb, pc = uv_of(a), uv_of(b), uv_of(c)
        return 0.5 * ((pb[0] - pa[0]) * (pc[1] - pa[1]) - (pc[0] - pa[0]) * (pb[1] - pa[1]))

    for frame_face, rings in zip(frame_faces, final_polygons):
        region = layout.region_of(frame_face)
        for ring in rings:
            ears, sign = _ears(list(ring), floats_of)
            slots = [(region, key) for key in ring]
            for a, b, c in ears:
                triangle = (slots[a], slots[b], slots[c], sign, region)
                triangles.append(triangle)
                balance[region] = balance.get(region, 0.0) + sign * area(*triangle[:3])
    if not triangles:
        return

    def signed(triangle) -> float:
        return triangle[3] * (1.0 if balance[triangle[4]] >= 0.0 else -1.0) * area(*triangle[:3])

    around: dict = {}
    for triangle in triangles:
        for slot in triangle[:3]:
            around.setdefault(slot, []).append(triangle)

    def feasible(slot) -> tuple | None:
        """`[lo, hi]` значений `U` вершины, при которых её треугольники не вывернуты, либо `None`: треугольник с другой причиной выворота или пустое пересечение."""

        low, high = float("-inf"), float("inf")
        held = uv_of(slot)
        for triangle in around[slot]:
            before = signed(triangle)
            uv[slot] = (held[0] + 1.0, held[1])
            rate = signed(triangle) - before
            uv[slot] = held
            if rate == 0.0:
                if before < 0.0:
                    return None
                continue
            bound = held[0] - before / rate
            if rate > 0.0:
                low = max(low, bound)
            else:
                high = min(high, bound)
        return None if low > high else (low, high)

    moved: set = set()
    for _sweep in range(_FLIP_SWEEPS):
        flipped = [triangle for triangle in triangles if signed(triangle) < 0.0]
        if not flipped:
            break
        for triangle in flipped:
            if signed(triangle) >= 0.0:
                continue
            # Сперва вершина выше всех: основание (`r = 0`) не двигается никогда.
            for slot in sorted(triangle[:3], key=lambda item: (-uv_of(item)[1], item)):
                if uv_of(slot)[1] <= 0.0:
                    continue
                span = feasible(slot)
                if span is None:
                    continue
                low, high = span
                # Ближайшая точка отрезка с отступом: граница дала бы нулевую площадь.
                reach = uv_of(slot)[1] * 1e-3 if low == float("-inf") or high == float("inf") else (high - low) * 0.1
                held = uv_of(slot)
                target = min(max(held[0], low + reach), high - reach) if low != float("-inf") and high != float("inf") else (max(held[0], low + reach) if low != float("-inf") else min(held[0], high - reach))
                uv[slot] = (target, held[1])
                moved.add(slot)
                break
    tally.add(FLIP_LEFT, sum(1 for triangle in triangles if signed(triangle) < 0.0))
    tally.add(UV_ZERO_AREA, sum(1 for triangle in triangles if signed(triangle) == 0.0))
    tally.add(FLIP_MOVED, len(moved))
    for slot in sorted(moved):
        exact = _exact(uv[slot][0])
        values[slot] = (exact, values[slot][1])
        forms[slot] = ((exact, *_ANCHORED),)


def map_straight_uv(frame_faces, layout, table, cycles, polygons, points, facts, final_polygons=None):
    """`(числители UV по (регион, ключ), формы тех же слотов, числа)` под законом `STRAIGHTEN_RUNGS_V1`.

    `cycles` - контуры слитых граней ДО резки, `polygons` - итоговые грани по ключам (с вершинами резки), `points` - все точки, `facts` - все факты `(s, r)`, `final_polygons` - кольца граней после прохода силуэта (их треугольники проверяет `ВЫВОРОТ`; `None` - те же `polygons`).
    """

    tally = _Tally()
    positions = _flow_positions(table)
    following = {before: after for before, after in table.joins}
    float_points: dict = {}

    def floats_of(key):
        found = float_points.get(key)
        if found is None:
            point = points[key]
            found = float_points[key] = (_f(point[0]), _f(point[1]))
        return found

    runs_at: dict = {}
    for frame_face, cycle in zip(frame_faces, cycles):
        if frame_face.is_fan:
            continue
        region = layout.region_of(frame_face)
        for key, _point in cycle:
            runs_at.setdefault((region, key), {})[frame_face.run.run_id] = True
    faces: list = []
    faces_of_run: dict = {}
    for frame_face, cycle in zip(frame_faces, cycles):
        if frame_face.is_fan:
            faces.append(None)
            continue
        face = _face_of(frame_face, cycle, layout.region_of(frame_face), facts, floats_of, tally)
        faces.append(face)
        faces_of_run.setdefault(frame_face.run.run_id, []).append(face)
    wedge = _wedge_values(table, positions, following, runs_at, facts, faces_of_run, lambda slot: _f(facts[slot][1]), tally)
    # Стык двух кадров разомкнутого кольца (`mesh._unify_across_frames`): факт на обеих сторонах уже станция вершины цепи.
    if table.frame_of_run:
        by_key: dict = {}
        for (region, key), runs in runs_at.items():
            by_key.setdefault(key, []).append((region, tuple(runs)))
        for key, entries in by_key.items():
            for (region_a, runs_a), (region_b, runs_b) in zip(entries, entries[1:]):
                if region_a != region_b and any(table.cut_join_station(a, b) is not None for a in runs_a for b in runs_b):
                    wedge.setdefault((region_a, key), facts[(region_a, key)][0])
                    wedge.setdefault((region_b, key), facts[(region_b, key)][0])
    proposals: dict = dict(wedge)
    columns = [None if face is None else _column_of(face, facts, wedge, proposals, tally) for face in faces]
    values: dict = {}
    forms: dict = {}
    for frame_face, column, rings in zip(frame_faces, columns, polygons):
        region = layout.region_of(frame_face)
        fan = _fan_axis(frame_face, table) if frame_face.is_fan else None
        for key in dict.fromkeys(key for ring in rings for key in ring):
            slot = region, key
            if slot in values:
                continue
            s, r = facts[slot]
            if frame_face.is_fan:
                if r.is_zero or fan is None:
                    values[slot], forms[slot] = (s, r), ((s, *_ANCHORED),)
                    continue
                node, cov = fan
                x, y = floats_of(key)
                numbers = cov[0] * (x - node[0]) + cov[1] * (y - node[1])
                values[slot] = (s + _exact(numbers), r)
                forms[slot] = ((ZERO, _exact(cov[0]), _exact(cov[1]), ZERO),)
                tally.add(FAN_VERTICES)
                continue
            known = proposals.get(slot)
            if known is not None or column is None:
                value = s if known is None else known
                values[slot], forms[slot] = (value, r), ((value, *_ANCHORED),)
                continue
            x, y = floats_of(key)
            origin_s, origin_x, origin_y, cov_x, cov_y = _run_origin(frame_face.run)
            value, form = _level_value(column, origin_s + (x - origin_x) * cov_x + (y - origin_y) * cov_y, _f(r), s, tally)
            values[slot], forms[slot] = (value, r), (form,)
    _repair_flips(frame_faces, layout, polygons if final_polygons is None else final_polygons, floats_of, values, forms, tally)
    return values, forms, tally.counters()


__all__ = ("map_straight_uv",)
