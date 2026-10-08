"""Отбор прямых источника для событий резки (`materialize.interval._clip_events`): сетка не теряет кандидатов против полного перебора.

Аудит `ad6074f`, находка F1: сетка строилась по габаритам прямых БЕЗ запаса и спрашивалась габаритом ребра БЕЗ запаса, а проверка
габарита с допуском шла уже над полученными кандидатами. Прямая в соседней ячейке, чья полоса допуска (`NODE_EDGE_SNAP_CELLS`
ячеек, расстояние в координатах) меняет знак движущегося узла, не попадала в кандидаты, а ложный минус отбора точная стадия не
исправляет: записанный интервал ширины (`alpha_interval`) оказывался шире настоящего.

Что утверждается, и чем оно проверено:

1. ГРАНИЦА ЯЧЕЙКИ. Пример аудитора: прямая `(0,0)-(0,10)`, узел идёт от `(-1.5,5)` к `(-0.5,5)`, они в соседних ячейках. Событие у полосы
   (около alpha 1) есть у полного перебора и у отбора по сетке.
2. СЕТКА НЕ ТЕРЯЕТ. Запрос габаритом ребра с запасом на радиус полосы (`_search_box`) возвращает КАЖДУЮ прямую, чья полоса касается
   габарита ребра (точная проверка в дробях), на случайных габаритах, шагах (целых и дробных), отрицательных и больших координатах и на
   границах ячеек; прежний запрос без запаса возвращается им же и теряет (отрицательный контроль: без него тест был бы пуст).
3. ОТБОР ПРОТИВ ПЕРЕБОРА. `_clip_events` на случайных гранях и источниках (малые целые координаты: параллельные, почти параллельные и
   лежащие на границах ячеек прямые встречаются часто) и на метаморфных копиях (сдвиг на целые числа любого знака, в том числе кратные
   шагу и отрицательные; масштаб решётки в целое число раз - допуск ориентации растёт вместе с длиной ребра): каждая пара «ребро, прямая»
   с расстоянием между габаритами не больше радиуса доходит до `_band_events` и даёт те же события, что при переборе ВСЕХ прямых; лишние
   пары (отбор грубее радиуса) допустимы, потерянных нет; отбор ничего не придумывает сверх перебора.
4. ОТРИЦАТЕЛЬНЫЙ КОНТРОЛЬ ОТБОРА. С прежним запросом (без запаса) тот же набор сцен ПРОВАЛИВАЕТСЯ: тест умеет краснеть.
5. ТАБЛИЦЫ ПОЛЕВЫХ ДОМЕНОВ. События таблицы настоящей сетки - между событиями «точного касания габаритов без ячеек» (снизу) и полного
   перебора всех прямых (сверху): сетка ячеек ничего не теряет против точного индекса.
"""

from __future__ import annotations

import math
import random
from collections import Counter
from fractions import Fraction
from types import SimpleNamespace

import pytest

import developable_factories as df
import materialize_factories as factories
from developable_route import developable_domain
from materialize_factories import prepare_and_cover

from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
from cftuv_envelope.materialize import interval as m
from cftuv_envelope.materialize.admit import materialization_request
from cftuv_envelope.materialize.clip_snap import NODE_EDGE_SNAP_CELLS
from cftuv_envelope.materialize.domain import materialize_domain
from cftuv_envelope.materialize.memo import memo_disabled
from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor

REACH = Fraction(NODE_EDGE_SNAP_CELLS)
#: Сдвиги целыми числами (оба знака, кратные и некратные шагу, большие) и масштабы решётки метаморфных копий.
SHIFTS = ((0, 0), (1, 0), (0, -1), (-5, 3), (-13, -13), (7, -37), (64, 64), (1_000_003, -999_983))
SCALES = (1, 3, 7)


def point(x, y):
    return SqrtSumV1.rational(Fraction(x)), SqrtSumV1.rational(Fraction(y))


# --------------------------------------------------------------------------
# 1. Пример аудитора
# --------------------------------------------------------------------------


class FullScan:
    """Индекс «все прямые»: полный перебор, на который проверка габарита в `_clip_events` накладывается так же."""

    def __init__(self, count: int) -> None:
        self.count = count

    def query(self, *_box):
        return set(range(self.count))


def test_a_snap_band_event_is_not_lost_across_a_grid_cell_boundary():
    face = SimpleNamespace(
        points=(point("-1.5", 4), point("-0.5", 4), point("-0.5", 6), point("-1.5", 6)),
        doubled_area=SqrtSumV1.rational(4),
    )
    times = [(0.5, 0.5), (1.5, 1.5), (1.5, 1.5), (0.5, 0.5)]
    tolerance = m._up(m._up(math.sqrt(100.0)) * float(NODE_EDGE_SNAP_CELLS))
    line = m._Line(0.0, 0.0, 0.0, 10.0, tolerance, (0.0, 0.0, 0.0, 10.0))
    step = m._grid_step([face])
    grid = m._Grid([line.box], step)
    empty_corners = m._Grid([], step)

    def events_for(index):
        events = []
        m._clip_events(face, times, (1, 0, -2, (1.0, 1.0)), 1, (line,), (), (), index, empty_corners, events, 0)
        return events

    reference = events_for(FullScan(1))
    optimized = events_for(grid)
    assert any(low <= 1.0 <= high for low, high in reference), "the reference must observe the snap event"
    assert any(low <= 1.0 <= high for low, high in optimized), (
        "the grid broad phase omitted an internal source line of an adjacent cell although its snap band changes the clipping sign: "
        f"reference={reference}, optimized={optimized}, step={step}"
    )
    # Узел и прямая действительно в разных ячейках (иначе пример ничего не доказывает).
    assert math.floor(-0.5 / step) != math.floor(0.0 / step)


# --------------------------------------------------------------------------
# 2. Сетка с запасом на радиус не теряет
# --------------------------------------------------------------------------


def touches(box, window, reach) -> bool:
    """Точно, в дробях: габарит `box` с запасом `reach` касается (закрыто) окна `window`; оба вида `(x_low, x_high, y_low, y_high)`."""

    bx0, bx1, by0, by1 = (Fraction(value) for value in box)
    x0, x1, y0, y1 = (Fraction(value) for value in window)
    return bx1 + reach >= x0 and bx0 - reach <= x1 and by1 + reach >= y0 and by0 - reach <= y1


def random_boxes(rng, span, count):
    boxes = []
    for _ in range(count):
        x0, y0 = rng.randint(-span, span), rng.randint(-span, span)
        x1, y1 = x0 + rng.randint(0, 6), y0 + rng.randint(0, 6)
        boxes.append((float(x0), float(x1), float(y0), float(y1)))
    return boxes


def random_window(rng, span):
    x0, y0 = rng.randint(-4 * span, 4 * span) / 4, rng.randint(-4 * span, 4 * span) / 4
    return (x0, x0 + rng.randint(0, 24) / 4, y0, y0 + rng.randint(0, 24) / 4)


def test_the_grid_query_with_the_search_box_returns_every_line_whose_band_touches_the_edge():
    rng = random.Random(20261007)
    lost_without_reach = 0
    for _ in range(600):
        step = rng.choice((1.0, 2.0, 2.5, 3.0, 4.0, 7.0, 10.0, 16.5))
        span = rng.choice((8, 40, 1_000_000))
        boxes = random_boxes(rng, span, rng.randint(1, 12))
        grid = m._Grid(boxes, step)
        window = random_window(rng, span)
        expected = {number for number, box in enumerate(boxes) if touches(box, window, REACH)}
        found = grid.query(*m._search_box(*window))
        assert expected <= found, (step, window, sorted(expected - found))
        assert grid.query(*window) <= found, "the reach may only add candidates"
        lost_without_reach += bool(expected - grid.query(*window))
    assert lost_without_reach > 0, "the unexpanded query must lose lines on this set, or the test is vacuous"


def test_the_search_box_rounds_outward_and_keeps_the_exact_reach():
    box = m._search_box(-0.1, 0.1, 3.0, 5.0)
    assert Fraction(box[0]) <= Fraction(-0.1) - REACH and Fraction(box[1]) >= Fraction(0.1) + REACH
    assert Fraction(box[2]) <= Fraction(3) - REACH and Fraction(box[3]) >= Fraction(5) + REACH
    # Большие координаты: шаг `float` крупнее радиуса (`big - 1 == big` в binary64), запас обязан быть не меньше радиуса всё равно:
    # округление наружу, а не к ближайшему.
    big = float(1 << 60)
    wide = m._search_box(big, big, -big, -big)
    assert Fraction(wide[0]) <= Fraction(big) - REACH and Fraction(wide[1]) >= Fraction(big) + REACH
    assert Fraction(wide[2]) <= Fraction(-big) - REACH and Fraction(wide[3]) >= Fraction(-big) + REACH


# --------------------------------------------------------------------------
# 3. Отбор `_clip_events` против полного перебора
# --------------------------------------------------------------------------


def convex_hull(points):
    """Выпуклая оболочка (против часовой) точек-дробей; `None`, если площадь нулевая."""

    unique = sorted(set(points))
    if len(unique) < 3:
        return None

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for item in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], item) <= 0:
            lower.pop()
        lower.append(item)
    for item in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], item) <= 0:
            upper.pop()
        upper.append(item)
    hull = lower[:-1] + upper[:-1]
    return hull if len(hull) >= 3 else None


def random_scene(rng):
    """`(вершины грани (дроби), треугольники источника (целые углы))` в малом окне: совпадения и параллели частые."""

    while True:
        corners = [(Fraction(rng.randint(-12, 12), 2), Fraction(rng.randint(-12, 12), 2)) for _ in range(rng.randint(3, 5))]
        hull = convex_hull(corners)
        if hull is not None:
            break
    triangles = [
        [(rng.randint(-8, 8), rng.randint(-8, 8)) for _ in range(3)] for _ in range(rng.randint(1, 5))
    ]
    triangles = [
        item
        for item in triangles
        if len(set(item)) == 3 and (item[1][0] - item[0][0]) * (item[2][1] - item[0][1]) - (item[1][1] - item[0][1]) * (item[2][0] - item[0][0]) != 0
    ]
    return hull, triangles


def transformed(hull, triangles, shift, scale):
    """Метаморфная копия: масштаб решётки целым числом, затем сдвиг целыми числами."""

    moved = [(x * scale + shift[0], y * scale + shift[1]) for x, y in hull]
    chart = [[(x * scale + shift[0], y * scale + shift[1]) for x, y in item] for item in triangles]
    return moved, chart


def build_case(hull, charts):
    """`(грань, времена, прямые, углы, точки углов)` из дробей: прямые и углы - настоящим `_source_geometry`."""

    area = sum(hull[i][0] * hull[(i + 1) % len(hull)][1] - hull[(i + 1) % len(hull)][0] * hull[i][1] for i in range(len(hull)))
    face = SimpleNamespace(points=tuple(point(x, y) for x, y in hull), doubled_area=SqrtSumV1.rational(area))
    triangles = [SimpleNamespace(chart=tuple((Fraction(x), Fraction(y)) for x, y in chart)) for chart in charts]
    geometry = m._source_geometry(triangles) if triangles else ((), ())
    lines, corners = geometry
    # Времена различны по вершинам: по ним шпион узнаёт номер ребра (`first[0] == index + 0.5`).
    times = [(index + 0.5, index + 0.5) for index in range(len(hull))]
    corner_points = [(SqrtSumV1.rational(x), SqrtSumV1.rational(y)) for x, y in corners]
    return face, times, lines, corners, corner_points


def expected_pairs(hull, lines):
    """Пары `(ребро грани, прямая)`, чьё расстояние между габаритами не больше радиуса полосы: ТОЧНО, из дробей вершин, без кода ядра."""

    pairs = set()
    count = len(hull)
    for index in range(count):
        (px, py), (qx, qy) = hull[index], hull[(index + 1) % count]
        window = (min(px, qx), max(px, qx), min(py, qy), max(py, qy))
        for number, line in enumerate(lines):
            if touches(line.box, window, REACH):
                pairs.add((index, number))
    return pairs


def recorded_pairs(monkeypatch, face, times, lines, corners, corner_points, index_of):
    """Какие пары дошли до `_band_events` и что события каждой: `{(ребро, прямая): события}` (шпион на `_band_events`, ответ его же)."""

    numbers = {id(line): number for number, line in enumerate(lines)}
    seen: dict = {}
    original = m._band_events

    def spy(first, last, line, lines_u, lines_w, span, events):
        before = len(events)
        original(first, last, line, lines_u, lines_w, span, events)
        key = (int(first[0]), numbers[id(line)])
        assert key not in seen
        seen[key] = tuple(events[before:])

    monkeypatch.setattr(m, "_band_events", spy)
    step = m._grid_step([face])
    grid_corners = m._Grid(((x, x, y, y) for x, y in corners), step)
    events: list = []
    m._clip_events(face, times, (1, 0, -2, (1.0, 1.0)), 1, lines, corners, corner_points, index_of(lines, step), grid_corners, events, 0)
    monkeypatch.setattr(m, "_band_events", original)
    return seen


def real_grid(lines, step):
    return m._Grid((line.box for line in lines), step)


def full_scan(lines, _step):
    return FullScan(len(lines))


def scenes(count=70, seed=7):
    rng = random.Random(seed)
    for _ in range(count):
        hull, triangles = random_scene(rng)
        if triangles:
            for scale in SCALES:
                for shift in SHIFTS:
                    yield transformed(hull, triangles, shift, scale)


def problems_of(monkeypatch, count=70):
    """Все нарушения отбора на наборе сцен: потерянные пары, пары вне перебора, расхождение событий общей пары."""

    found = []
    checked = 0
    for hull, charts in scenes(count):
        face, times, lines, corners, corner_points = build_case(hull, charts)
        expected = expected_pairs(hull, lines)
        everything = recorded_pairs(monkeypatch, face, times, lines, corners, corner_points, full_scan)
        selected = recorded_pairs(monkeypatch, face, times, lines, corners, corner_points, real_grid)
        checked += len(expected)
        if not expected <= set(everything):
            found.append(("the full scan lacks an expected pair", sorted(expected - set(everything))))
        if expected - set(selected):
            found.append(("the grid lost pairs within the band radius", sorted(expected - set(selected)), hull))
        if set(selected) - set(everything):
            found.append(("the grid invented pairs", sorted(set(selected) - set(everything))))
        for key in set(selected) & set(everything):
            if selected[key] != everything[key]:
                found.append(("events of a shared pair differ", key))
    assert checked > 200, "the set of scenes must contain pairs within the radius, or the test is vacuous"
    return found


def test_the_clip_event_candidates_lose_nothing_against_the_full_scan_on_random_and_metamorphic_scenes(monkeypatch):
    assert problems_of(monkeypatch) == []


def test_the_differential_turns_red_with_the_query_without_reach(monkeypatch):
    """Прежний запрос (габарит ребра как есть) на тех же сценах теряет пары: тест умеет краснеть."""

    monkeypatch.setattr(m, "_search_box", lambda *box: box)
    found = problems_of(monkeypatch, count=25)
    assert any(item[0] == "the grid lost pairs within the band radius" for item in found), found[:3]


def test_a_near_parallel_edge_inside_and_beside_the_band_keeps_its_candidates(monkeypatch):
    """Ребро грани почти параллельно прямой на расстоянии 0.5, 1 (ровно радиус) и 1.5 (вне полосы), со сдвигами через границы ячеек: кандидаты те же."""

    for offset in (Fraction(1, 2), Fraction(1), Fraction(3, 2)):
        for slope in (Fraction(0), Fraction(1, 64), Fraction(-1, 7)):
            for shift in ((0, 0), (-8, -8), (13, -21)):
                hull = convex_hull(
                    [
                        (shift[0] - offset, Fraction(2) + shift[1]),
                        (shift[0] - offset + slope * 6, Fraction(8) + shift[1]),
                        (Fraction(-6) + shift[0], Fraction(8) + shift[1]),
                        (Fraction(-6) + shift[0], Fraction(2) + shift[1]),
                    ]
                )
                # Правая сторона грани почти параллельна прямой `x = shift` (ребро источника длины 10), на расстоянии `offset` от неё.
                charts = [[(shift[0], shift[1]), (shift[0], 10 + shift[1]), (shift[0] + 5, shift[1] + 5)]]
                face, times, lines, corners, corner_points = build_case(hull, charts)
                expected = expected_pairs(hull, lines)
                assert expected or offset > 1, (offset, slope)
                everything = recorded_pairs(monkeypatch, face, times, lines, corners, corner_points, full_scan)
                selected = recorded_pairs(monkeypatch, face, times, lines, corners, corner_points, real_grid)
                assert expected <= set(selected), (offset, slope, shift, sorted(expected - set(selected)))
                assert all(selected[key] == everything[key] for key in selected)


# --------------------------------------------------------------------------
# 5. Таблицы событий полевых доменов
# --------------------------------------------------------------------------


class ExactTouchIndex:
    """Индекс без ячеек: те же номера, что у сетки в пределе: габарит прямой касается запроса (точно, в дробях)."""

    def __init__(self, boxes, _step) -> None:
        self.boxes = list(boxes)

    def query(self, x_low, x_high, y_low, y_high):
        # Сравнения `float` точны: ячеек и округлений здесь нет.
        return {
            number
            for number, (bx0, bx1, by0, by1) in enumerate(self.boxes)
            if bx1 >= x_low and bx0 <= x_high and by1 >= y_low and by0 <= y_high
        }


class FullIndex:
    def __init__(self, boxes, _step) -> None:
        self.count = len(list(boxes))

    def query(self, *_box):
        return set(range(self.count))


ROUTE = ("r0a", "r0b")
BY_TRIANGLES = NearPlanarLiftLawV1.SOURCE_TRIANGLES_CLIPPED_V1
POLYGONS = DecalTopologyLawV1.PLANAR_POLYGONS_V1


def developable(make, alpha="1"):
    snapshot, request = developable_domain(make(), ROUTE, alpha=alpha)
    prepared, _coverage = prepare_and_cover(snapshot, request)
    return prepared


def field(name):
    snapshot, request = factories.load_fixture(name)
    prepared = prepare_conveyor(snapshot, request)
    assert prepared.outcome.value == "EXACT", prepared.detail
    return prepared


CLIPPED = (
    ("fold", lambda: developable(df.fold_strip), "0.3"),
    ("slant", lambda: developable(df.slant_fold), "0.7"),
    ("quarter", lambda: developable(df.quarter_cylinder), "0.7"),
    ("noise_top", lambda: field("wall_noise_top_rung_clip_v1"), "0.25"),
)


@pytest.mark.parametrize("name,build,alpha", CLIPPED, ids=[item[0] for item in CLIPPED])
def test_the_event_table_of_the_grid_lies_between_the_exact_index_and_the_full_scan(monkeypatch, name, build, alpha):
    prepared = build()
    captured = []
    original = m._build_table

    def spy(*arguments):
        captured.append(arguments)
        return original(*arguments)

    monkeypatch.setattr(m, "_build_table", spy)
    coverage = conveyor_coverage(prepared, alpha)
    assert coverage.outcome.value == "EXACT", coverage.detail
    with memo_disabled():
        result = materialize_domain(
            prepared,
            coverage,
            request=materialization_request(prepared, uv_policy_id="UV_DIRECT_STRIP_V1"),
            near_planar_lift_law=BY_TRIANGLES,
            decal_topology_law=POLYGONS,
            certify=True,
        )
    assert result.is_materialized, result.detail
    monkeypatch.setattr(m, "_build_table", original)
    assert captured and captured[0][1], "the fixture must clip (source triangles under the lift)"

    def events_with(grid_type):
        monkeypatch.setattr(m, "_Grid", grid_type)
        table = original(*captured[0])
        assert not table.refusal, table.refusal
        return Counter(zip(table.lo, table.hi))

    real_type = m._Grid
    selected = events_with(real_type)
    exact = events_with(ExactTouchIndex)
    everything = events_with(FullIndex)
    monkeypatch.setattr(m, "_Grid", real_type)
    assert not (exact - selected), "the grid lost events against the exact touch index"
    assert not (selected - everything), "the grid invented events beyond the full scan"
