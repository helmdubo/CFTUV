"""Закон `SILHOUETTE_TOPOLOGY_V1` (срез S1): рёбра и вершины, не влияющие на силуэт, растворяются.

Что здесь доказано и чем.

* СИНТЕТИКА (сетка прямоугольника с точными рациональными станциями): ребро между гранями одного региона в одной плоскости с
  точно аффинной UV растворяется; вершина на прямой между двумя рёбрами растворяется в пределах глубины хорды и сдвига UV
  запроса; каждый отказ назван счётчиком (`KEPT_CHORD`, `KEPT_UV`, `KEPT_NOT_AFFINE`, `KEPT_NOT_SIMPLE`, `KEPT_JUNCTION`);
  ребро между регионами, контур, цепи источника и стены не растворяются никогда; порядок слияния — самое плоское первым.
* ПРОВЕРКА ЗАКОНА (`verify_silhouette`) ловит испорченный итог: красные контроли — растворённая вершина стены, чужие факты,
  сдвиг и хорда сверх записанного, порванная граница, грань с растворённой вершиной.
* ПОЛЕ (`sagging_wall`, alpha 0.987, Max stretch 42 %): доказательство слитых граней НЕЗАВИСИМЫМ точным предикатом (простота и
  аффинность UV итоговой грани), вершины цепей источника и стены те же, что у `PLANAR_POLYGONS_V1`, остальные вершины и их UV
  не изменились, прежние законы побитово прежние, сдвиг UV читается из запроса.
* ДОПУСК UV РЁБЕР: слияние в пределах `silhouette_uv_slide` = ε (одна аффинная карта, остаток записан и пересчитан независимой
  точной арифметикой Fraction), ε = 0 — прежнее точное правило (подбор карты не вызывается, ответ тот же).
* ПОЛИТИКА ЗАПРОСА: `silhouette_uv_slide` на проводе опущено по умолчанию, законность проверяется (нуль законен).
"""

from __future__ import annotations

import dataclasses
import math
from fractions import Fraction
from functools import lru_cache
from pathlib import Path
from types import SimpleNamespace

import pytest

import cftuv_envelope as kernel
from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.contracts.metric import (
    DEFAULT_SILHOUETTE_UV_SLIDE,
    MAX_SILHOUETTE_UV_SLIDE,
    NearPlanarLiftLawV1,
    silhouette_uv_slide_is_lawful,
)
from cftuv_envelope.exact_sqrt_sum import ExactCanonicalizationWorkBudgetExhausted, SqrtSumV1, exact_work_budget
from cftuv_envelope.ids import PolicyId
from cftuv_envelope.materialize import domain as domain_module
from cftuv_envelope.materialize import silhouette
from cftuv_envelope.materialize.admit import MaterializationOutcome, materialization_request
from cftuv_envelope.materialize.assemble import Layout, chains_of
from cftuv_envelope.materialize.clip_cells import CLIP_DIAGONAL_CHORD_BUDGET
from cftuv_envelope.materialize.domain import materialize_domain
from cftuv_envelope.materialize.silhouette import SilhouetteInputV1, apply_silhouette, dissolve_silhouette, verify_silhouette
from cftuv_envelope.materialize.tessellate import contour_is_simple, uv_is_affine_in_chart
from cftuv_envelope.numeric import LocalPoint3V1
from cftuv_envelope.validation import validate_decal_request, validate_geometry_batch
from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor

R = SqrtSumV1.rational
FIXTURES = Path(__file__).resolve().parents[1] / "fixtures"
UV = PolicyId("UV_DIRECT_STRIP_V1")
CELL = Fraction(1, 100)  # метры на ячейку сетки: 1 см
CHORD = float(CLIP_DIAGONAL_CHORD_BUDGET)


# ---------------------------------------------------------------------------
# 1. Синтетика: сетка прямоугольника `columns x rows` с точными станциями `(s, r) = (i, j)`
# ---------------------------------------------------------------------------


def _key(i, j):
    return f"node:{i}:{j}"


def _frame(name="f0"):
    return SimpleNamespace(claim_key="claim", frame_key=name, flow_key=None, is_fan=False)


def _grid(columns=3, rows=1, *, merged=False, lift=None, station=None, slide=Fraction(1, 256), split=None, same_region=False):
    """Вход закона: `columns x rows` единичных квадратов одного региона (либо одна грань целиком при `merged`).

    `alpha` решётки равна `rows`: `r = 0` — источник, `r = rows` — фронт, боковые стороны — стена. `lift(i, j)` — высота точки
    в метрах (по умолчанию плоско), `station(i, j)` — подмена `(s, r)` вершины (рациональные либо `None`), `split` — номер
    столбца, правее которого грани уходят во второй кадр (слитую грань со своим контуром); `same_region` оставляет второй кадр
    в том же регионе (кадры называются одинаково), иначе это второй регион.
    """

    frames = [_frame("f0")] + ([_frame("f0" if same_region else "f1")] if split is not None else [])
    layout = Layout(frames)
    regions = [layout.region_of(item) for item in frames]
    positions, points, facts = {}, {}, {}

    def vertex(i, j, frame_index):
        key = _key(i, j)
        positions[key] = LocalPoint3V1(float(CELL * i), float(CELL * j), 0.0 if lift is None else lift(i, j))
        points[key] = (R(Fraction(i)), R(Fraction(j)))
        s, r = (i, j) if station is None or station(i, j) is None else station(i, j)
        facts[(regions[frame_index], key)] = (R(Fraction(s)), R(Fraction(r)))
        return key

    def outline(low, high):
        ring = [(i, 0) for i in range(low, high + 1)] + [(high, j) for j in range(1, rows + 1)]
        return ring + [(i, rows) for i in range(high - 1, low - 1, -1)] + [(low, j) for j in range(rows - 1, 0, -1)]

    polygons = [[] for _ in frames]
    if merged:
        polygons[0].append(tuple(vertex(i, j, 0) for i, j in outline(0, columns)))
        contours = [outline(0, columns)]
    else:
        for column in range(columns):
            frame_index = 0 if split is None or column < split else 1
            for row in range(rows):
                corners = [(column, row), (column + 1, row), (column + 1, row + 1), (column, row + 1)]
                polygons[frame_index].append(tuple(vertex(i, j, frame_index) for i, j in corners))
        contours = [outline(0, columns)] if split is None else [outline(0, split), outline(split, columns)]
    cycles = [[(vertex(i, j, index), (R(Fraction(i)), R(Fraction(j)))) for i, j in ring] for index, ring in enumerate(contours)]
    return SilhouetteInputV1(
        polygons=polygons,
        cycles=cycles,
        vertex_cycles=None,
        positions=positions,
        points=points,
        facts=facts,
        frame_faces=frames,
        layout=layout,
        lattice_alpha=Fraction(rows),
        uv_slide=slide,
    )


def _run(source):
    result = dissolve_silhouette(source, exact_work_budget(stage="MATERIALIZE"))
    return result, dict(result.counters)


def _faces(result):
    return [ring for face_polygons in result.polygons for ring in face_polygons]


def _shifted(i, j, by=Fraction(1, 500), at=(2, 1)):
    return (i + by, j) if (i, j) == at else None


def test_a_vertex_on_a_straight_line_within_the_slide_is_dissolved_and_the_slide_is_recorded():
    """Реестр `SILHOUETTE_UV_SLIDE_V1`, положительная фикстура: три вершины верхнего ребра прямоугольника растворены, сдвиг записан."""

    source = _grid(4, 1, merged=True, station=_shifted)

    result, counters = _run(source)

    top = {_key(i, 1) for i in (1, 2, 3)}
    assert result.dissolved == top
    assert counters[silhouette.VERTICES_DISSOLVED] == 3
    assert counters[silhouette.MAX_UV_SLIDE_MILLI_ALPHA] == 2  # 1/500 alpha = 2 тысячных alpha
    assert counters[silhouette.MAX_CHORD_NM] <= 5_000_000
    assert len(_faces(result)[0]) == 4 + 3  # четыре угла и три вершины источника
    assert not verify_silhouette(source, result)
    assert set(result.positions) == set(source.positions) - top
    assert set(result.facts) == {slot for slot in source.facts if slot[1] not in top}


def test_a_vertex_whose_uv_would_slide_beyond_the_request_is_kept_by_name():
    """Реестр `SILHOUETTE_UV_SLIDE_V1`, негативная фикстура: сдвиг UV больше допуска запроса оставляет вершину и называется."""

    source = _grid(4, 1, merged=True, station=lambda i, j: _shifted(i, j, Fraction(1, 100)))

    result, counters = _run(source)

    assert not result.dissolved and not result.changed
    assert counters[silhouette.KEPT_UV] == 3  # сдвинутая вершина и оба соседа: лерп по сдвинутому соседу сдвигает и их
    assert not verify_silhouette(source, result)


def test_an_edge_whose_union_uv_is_affine_within_the_tolerance_is_dissolved_and_the_residual_is_recorded():
    """Излом UV одной вершины на 1/500 alpha: точной аффинности нет, ОДНА карта приближает UV всех вершин в допуске 1/256 alpha."""

    source = _grid(3, 1, station=lambda i, j: _shifted(i, j, Fraction(1, 500), (1, 1)))

    result, counters = _run(source)

    assert counters[silhouette.EDGES_WITHIN_UV_TOLERANCE] == 2 and counters[silhouette.EDGES_DISSOLVED] == 2
    assert 1 <= counters[silhouette.MAX_UV_RESIDUAL_MILLI_ALPHA] <= 4
    assert len(_faces(result)) == 1 and len(result.fits) == 2
    assert not verify_silhouette(source, result)


def test_an_edge_beyond_the_uv_tolerance_is_kept_by_name_and_a_looser_request_dissolves_it():
    station = lambda i, j: _shifted(i, j, Fraction(1, 500), (1, 1))
    strict = _run(_grid(3, 1, station=station, slide=Fraction(1, 10_000)))
    loose = _run(_grid(3, 1, station=station, slide=MAX_SILHOUETTE_UV_SLIDE))

    assert strict[1][silhouette.KEPT_NOT_AFFINE] == 2 and not strict[0].changed
    assert loose[1][silhouette.EDGES_WITHIN_UV_TOLERANCE] == 2


def test_a_zero_tolerance_is_the_exact_rule_and_never_fits_a_map(monkeypatch):
    """ε = 0: подбор карты не вызывается вовсе; слияния рёбер — ровно те, что даёт точная аффинность."""

    def refuse(*_args):
        raise AssertionError("the exact rule must not fit a map")

    monkeypatch.setattr(silhouette, "uv_fit_residual", refuse)
    exact = _run(_grid(3, 1, slide=Fraction(0)))
    kinked = _run(_grid(3, 1, station=lambda i, j: _shifted(i, j, Fraction(1, 500), (1, 1)), slide=Fraction(0)))
    regions = _run(_grid(4, 2, split=2, same_region=True, slide=Fraction(0)))

    assert exact[1][silhouette.EDGES_DISSOLVED] == 2 and len(_faces(exact[0])) == 1
    assert kinked[1][silhouette.KEPT_NOT_AFFINE] == 2 and not kinked[0].changed
    for found in (exact, kinked, regions):
        assert not found[0].fits and not found[1].get(silhouette.EDGES_WITHIN_UV_TOLERANCE)
        assert not found[1].get(silhouette.MAX_UV_RESIDUAL_MILLI_ALPHA)


def _exact_fit_residual(chart, uvs):
    """Независимая точная арифметика: наименьшие квадраты `UV ~ a + b x + c y` в Fraction по тем же числам, остаток — наибольшее расстояние."""

    count = len(chart)
    xs = [Fraction(point[0]) for point in chart]
    ys = [Fraction(point[1]) for point in chart]
    mean_x, mean_y = sum(xs) / count, sum(ys) / count
    xs, ys = [x - mean_x for x in xs], [y - mean_y for y in ys]
    sxx, sxy, syy = sum(x * x for x in xs), sum(x * y for x, y in zip(xs, ys)), sum(y * y for y in ys)
    determinant = sxx * syy - sxy * sxy
    if determinant <= 0:
        return None
    residuals = []
    for component in (0, 1):
        values = [Fraction(uv[component]) for uv in uvs]
        mean = sum(values) / count
        shifted = [value - mean for value in values]
        right_x, right_y = sum(x * v for x, v in zip(xs, shifted)), sum(y * v for y, v in zip(ys, shifted))
        slope_x, slope_y = (right_x * syy - right_y * sxy) / determinant, (sxx * right_y - sxy * right_x) / determinant
        residuals.append([v - (slope_x * x + slope_y * y) for v, x, y in zip(shifted, xs, ys)])
    return max(math.hypot(float(a), float(b)) for a, b in zip(*residuals))


def test_the_fitted_residual_is_the_exact_rational_least_squares_residual():
    import random

    rng = random.Random(20261006)
    for _ in range(200):
        count = rng.randint(3, 9)
        chart = [(rng.uniform(-5000, 5000), rng.uniform(-5000, 5000)) for _ in range(count)]
        slope = [rng.uniform(-2, 2) for _ in range(4)]
        uvs = [
            (slope[0] * x / 1000 + slope[1] * y / 1000 + rng.uniform(-0.01, 0.01), slope[2] * x / 1000 + slope[3] * y / 1000 + rng.uniform(-0.01, 0.01))
            for x, y in chart
        ]
        found, independent = silhouette.uv_fit_residual(chart, uvs), _exact_fit_residual(chart, uvs)
        assert (found is None) == (independent is None)
        if found is not None:
            assert abs(found - independent) <= 1e-9 * max(1.0, independent)
    assert silhouette.uv_fit_residual([(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)], [(0.0, 0.0)] * 3) is None  # одна прямая: карты нет


def test_the_residual_of_an_exactly_affine_union_is_zero_and_a_kink_has_the_expected_size():
    chart = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    affine = [(x * 0.3, y * 0.5) for x, y in chart]
    kinked = [(x * 0.3, y * 0.5) for x, y in chart[:3]] + [(0.0, 0.5 + 0.04)]

    assert silhouette.uv_fit_residual(chart, affine) < 1e-12
    assert 0.009 < silhouette.uv_fit_residual(chart, kinked) < 0.04


def test_the_slide_is_the_request_policy_and_a_smaller_one_dissolves_fewer_vertices():
    default = _run(_grid(4, 1, merged=True, station=_shifted))[1]
    strict = _run(_grid(4, 1, merged=True, station=_shifted, slide=Fraction(1, 10_000)))[1]
    loose = _run(_grid(4, 1, merged=True, station=_shifted, slide=MAX_SILHOUETTE_UV_SLIDE))[1]

    assert strict.get(silhouette.VERTICES_DISSOLVED, 0) < default[silhouette.VERTICES_DISSOLVED] <= loose[silhouette.VERTICES_DISSOLVED]
    assert strict[silhouette.KEPT_UV] >= 1


def test_a_vertex_beyond_the_chord_depth_is_kept_by_name():
    source = _grid(4, 1, merged=True, lift=lambda i, j: 4 * CHORD if (i, j) == (2, 1) else 0.0)

    result, counters = _run(source)

    assert not result.dissolved and not result.changed
    assert counters[silhouette.KEPT_CHORD] == 3  # (2, 1) на 20 мм, его соседи (1, 1) и (3, 1) на 10 мм от своих хорд
    assert not verify_silhouette(source, result)


def test_vertices_on_the_source_chain_and_the_wall_are_never_dissolved():
    """Нижняя сторона (`r = 0`, источник) и боковые (стена) лежат на прямых линиях, и ни одна вершина на них не растворена."""

    source = _grid(4, 2, merged=True)

    result, counters = _run(source)

    fixed = {_key(i, 0) for i in range(5)} | {_key(0, j) for j in range(3)} | {_key(4, j) for j in range(3)}
    assert not result.dissolved & fixed
    assert result.dissolved == {_key(i, 2) for i in (1, 2, 3)}  # верхняя сторона (фронт) растворена
    assert not verify_silhouette(source, result)


def test_an_edge_between_coplanar_faces_with_one_affine_uv_is_dissolved_and_the_face_is_one_polygon():
    source = _grid(3, 1)

    result, counters = _run(source)

    assert counters[silhouette.EDGES_DISSOLVED] == 2
    assert len(_faces(result)) == 1 and len(set(_faces(result)[0])) == len(_faces(result)[0])
    assert result.dissolved == {_key(1, 1), _key(2, 1)}  # после слияния вершины верха стали вершинами двух рёбер
    assert counters[silhouette.VERTICES_DISSOLVED] == 2
    assert not verify_silhouette(source, result)


def test_an_edge_whose_union_has_a_non_affine_uv_is_kept_by_name():
    """UV непрерывна на ребре (один регион), но в объединении изломана ВЫШЕ допуска: слияние отдало бы триангуляции хоста выбор UV."""

    source = _grid(3, 1, station=lambda i, j: _shifted(i, j, Fraction(1, 50), (1, 1)))

    result, counters = _run(source)

    assert counters[silhouette.KEPT_NOT_AFFINE] == 2
    assert not counters.get(silhouette.EDGES_DISSOLVED)
    assert len(_faces(result)) == 3 and not result.changed


def test_an_edge_beyond_the_chord_depth_is_kept_by_name():
    source = _grid(3, 1, lift=lambda i, j: 3 * CHORD * (i - 1) if i >= 2 else 0.0)

    result, counters = _run(source)

    assert counters[silhouette.KEPT_CHORD] >= 1
    assert counters.get(silhouette.EDGES_DISSOLVED, 0) < 2
    assert not verify_silhouette(source, result)


def test_an_edge_between_two_regions_is_never_dissolved():
    source = _grid(4, 1, split=2)

    result, counters = _run(source)

    assert len(result.polygons) == 2 and len(_faces(result)) == 2  # по одной грани на регион: внутри слито, граница нет
    assert counters[silhouette.EDGES_DISSOLVED] == 2
    assert not verify_silhouette(source, result)
    _boundary, interface = chains_of(source.frame_faces, result.cycles, source.layout, result.facts, source.lattice_alpha)
    assert interface, "шов между регионами остался цепью"


def test_merging_runs_flattest_first_and_the_answer_is_deterministic(monkeypatch):
    """Порядок слияния — по двугранному углу по возрастанию (затем по ключам): ступенчатая сетка с разными наклонами."""

    heights = {0: 0.0, 1: 0.0, 2: 0.4 * CHORD, 3: 1.0 * CHORD, 4: 1.2 * CHORD}
    source = _grid(4, 1, lift=lambda i, j: heights[i])
    order = []
    original = silhouette._Mesh._try_merge

    def recorded(self, first, second):
        order.append((first, second))
        return original(self, first, second)

    monkeypatch.setattr(silhouette._Mesh, "_try_merge", recorded)
    first = _run(source)
    monkeypatch.undo()
    second = _run(source)

    def angle(edge):
        i = int(edge[0].split(":")[1])  # ребро между столбцами i - 1 и i
        slope = lambda column: heights[column + 1] - heights[column]
        return abs(math.atan2(slope(i), float(CELL)) - math.atan2(slope(i - 1), float(CELL)))

    angles = [angle(edge) for edge in order]
    assert len(order) == 3 and angles == sorted(angles)
    assert first[1] == second[1] and _faces(first[0]) == _faces(second[0])


def test_a_planar_non_convex_polygon_with_one_affine_uv_is_allowed():
    """Г-образная область из трёх квадратов: невыпуклый плоский многоугольник с аффинной UV законен (`PLANAR_AFFINE_UV_POLYGON_V1`)."""

    frames = [_frame()]
    layout = Layout(frames)
    region = layout.region_of(frames[0])
    positions, points, facts, polygons = {}, {}, {}, [[]]

    def vertex(i, j):
        key = _key(i, j)
        positions[key] = LocalPoint3V1(float(CELL * i), float(CELL * j), 0.0)
        points[key] = (R(Fraction(i)), R(Fraction(j)))
        facts[(region, key)] = (R(Fraction(i)), R(Fraction(j)))
        return key

    for column, row in [(0, 0), (1, 0), (0, 1)]:
        polygons[0].append(tuple(vertex(i, j) for i, j in [(column, row), (column + 1, row), (column + 1, row + 1), (column, row + 1)]))
    outline = [(0, 0), (1, 0), (2, 0), (2, 1), (1, 1), (1, 2), (0, 2), (0, 1)]
    source = SilhouetteInputV1(
        polygons,
        [[(_key(i, j), points[_key(i, j)]) for i, j in outline]],
        None,
        positions,
        points,
        facts,
        frames,
        layout,
        Fraction(2),
        Fraction(1, 256),
    )

    result, counters = _run(source)

    assert counters[silhouette.EDGES_DISSOLVED] == 2 and len(_faces(result)) == 1
    ring = _faces(result)[0]
    assert contour_is_simple(tuple(points[key] for key in ring), None)
    assert uv_is_affine_in_chart([points[key] for key in ring], [facts[(region, key)] for key in ring], None)
    assert not verify_silhouette(source, result)


def test_a_face_that_would_collapse_keeps_its_vertex_by_name():
    """Треугольник с тремя вершинами фронта: растворение вершины оставило бы двуугольник."""

    frames = [_frame()]
    layout = Layout(frames)
    region = layout.region_of(frames[0])
    xy = {_key(0, 0): (0, 0), _key(2, 0): (2, 0), _key(1, 1): (1, 1)}
    keys = list(xy)
    source = SilhouetteInputV1(
        [[tuple(keys)]],
        [[(key, (R(Fraction(i)), R(Fraction(j)))) for key, (i, j) in xy.items()]],
        None,
        {key: LocalPoint3V1(float(CELL * i / 100), float(CELL * j / 100), 0.0) for key, (i, j) in xy.items()},  # доли миллиметра
        {key: (R(Fraction(i)), R(Fraction(j))) for key, (i, j) in xy.items()},
        {(region, key): (R(Fraction(0)), R(Fraction(1))) for key in xy},  # `r = alpha` везде (фронт), UV одна: ни хорды, ни сдвига
        frames,
        layout,
        Fraction(1),
        Fraction(1, 256),
    )

    result, counters = _run(source)

    assert not result.changed and _faces(result) == [tuple(keys)]
    assert counters[silhouette.KEPT_NOT_SIMPLE] == 3


def test_a_mesh_with_a_half_edge_in_two_faces_is_left_as_it_is_by_name():
    source = _grid(2, 1)
    broken = dataclasses.replace(source, polygons=[source.polygons[0] + [source.polygons[0][0]]])

    result, counters = _run(broken)

    assert counters == {silhouette.SKIPPED_NOT_MANIFOLD: 1}
    assert result.polygons is broken.polygons and not result.changed


def test_exhausting_the_pass_budget_leaves_the_mesh_as_it_was_and_names_it(monkeypatch):
    source = _grid(3, 1)

    def exhausted(self):
        raise ExactCanonicalizationWorkBudgetExhausted("synthetic")

    monkeypatch.setattr(silhouette._Mesh, "dissolve_edges", exhausted)
    budget = exact_work_budget(stage="MATERIALIZE")
    before = budget.spent_by_article()

    result = dissolve_silhouette(source, budget)

    assert dict(result.counters) == {silhouette.SKIPPED_WORK_BUDGET: 1}
    assert not result.changed and result.polygons is source.polygons
    assert budget.spent_by_article() == before  # бюджет домена не тронут проходом, который не состоялся


def test_a_vertex_where_the_contours_of_two_merged_faces_meet_is_kept_by_name():
    """Вершина степени два в сетке граней, но узел контуров двух слитых граней одного региона: растворение развело бы цепи."""

    source = _grid(2, 1, split=1, same_region=True)

    result, counters = _run(source)

    assert counters[silhouette.EDGES_DISSOLVED] == 1 and len(_faces(result)) == 1
    assert counters[silhouette.KEPT_JUNCTION] == 1
    assert _key(1, 1) not in result.dissolved
    assert not verify_silhouette(source, result)


#: Высоты (мм) сетки 3 x 3 со случайным шумом (`random.Random(3)`, ±6 мм): грань, слитая по плоскости БОЛЬШЕЙ грани, ушла бы от своей
#: собственной плоскости дальше допуска.
NOISY_HEIGHTS_MM = {
    (0, 0): 0.53, (0, 1): -1.56, (0, 2): 1.25, (0, 3): 1.51,
    (1, 0): -5.21, (1, 1): -5.84, (1, 2): 4.05, (1, 3): -2.89,
    (2, 0): -3.19, (2, 1): 5.95, (2, 2): -0.36, (2, 3): 4.04,
    (3, 0): -0.28, (3, 1): 1.67, (3, 2): -4.19, (3, 3): 1.62,
}


def _own_plane_depth(source, ring):
    """Наибольшее отклонение вершин грани от ЕЁ плоскости (Ньюэлл от центра), метры: независимая от закона формула."""

    points = [(source.positions[key].x, source.positions[key].y, source.positions[key].z) for key in ring]
    centre = [sum(p[axis] for p in points) / len(points) for axis in range(3)]
    normal = [0.0, 0.0, 0.0]
    for first, second in zip(points, points[1:] + points[:1]):
        a = [first[axis] - centre[axis] for axis in range(3)]
        b = [second[axis] - centre[axis] for axis in range(3)]
        normal[0] += a[1] * b[2] - a[2] * b[1]
        normal[1] += a[2] * b[0] - a[0] * b[2]
        normal[2] += a[0] * b[1] - a[1] * b[0]
    length = math.sqrt(sum(value * value for value in normal))
    return max(abs(sum((point[axis] - centre[axis]) * normal[axis] for axis in range(3))) / length for point in points)


def test_a_merged_face_stays_within_the_chord_depth_of_its_own_plane_not_only_of_the_larger_one():
    source = _grid(3, 3, lift=lambda i, j: NOISY_HEIGHTS_MM[(i, j)] / 1000.0)

    result, counters = _run(source)

    originals = {tuple(ring) for face_polygons in source.polygons for ring in face_polygons}
    for ring in _faces(result):
        if tuple(ring) not in originals and len(ring) > 4:
            assert _own_plane_depth(source, ring) <= CHORD, ring
    assert not verify_silhouette(source, result)


@pytest.mark.parametrize("exact", (True, False))
@pytest.mark.parametrize("seed", range(40))
def test_random_noisy_grids_keep_every_invariant_of_the_law(seed, exact):
    """Случайная сетка (шум высот, сдвиги UV, два региона): итог проходит проверку; при ε = 0 слитые грани точно аффинны, иначе остаток карты каждого слияния пересчитан точной арифметикой; все — в допуске от своих плоскостей."""

    import random

    rng = random.Random(seed)
    columns, rows = rng.randint(2, 5), rng.randint(1, 3)
    heights = {(i, j): rng.uniform(-7, 7) * 0.001 * rng.choice((0.2, 1.0)) for i in range(columns + 1) for j in range(rows + 1)}
    shifted = {(rng.randint(1, columns), rng.randint(1, rows)) for _ in range(rng.randint(0, 2))}
    source = _grid(
        columns,
        rows,
        lift=lambda i, j: heights[(i, j)],
        station=lambda i, j: (i + Fraction(rng.choice((1, 3, 40)), 1000), j) if (i, j) in shifted else None,
        split=rng.choice((None, rng.randint(1, columns - 1))),
        same_region=rng.random() < 0.5,
        slide=Fraction(0) if exact else Fraction(1, 256),
    )

    result, counters = _run(source)

    assert not verify_silhouette(source, result)
    if exact:
        assert not result.fits and not counters.get(silhouette.EDGES_WITHIN_UV_TOLERANCE)
    fitted = {key for _region, ring in result.fits for key in ring}
    for region, ring in result.fits:
        chart = [(float(source.points[key][0].as_rational()), float(source.points[key][1].as_rational())) for key in ring]
        uvs = [(float(source.facts[(region, key)][0].as_rational()) / source.lattice_alpha, float(source.facts[(region, key)][1].as_rational()) / source.lattice_alpha) for key in ring]
        assert _exact_fit_residual(chart, uvs) <= float(source.uv_slide) * (1 + 1e-9)
    originals = {tuple(ring) for face_polygons in source.polygons for ring in face_polygons}
    for index, face_polygons in enumerate(result.polygons):
        region = source.layout.region_of(source.frame_faces[index])
        for ring in face_polygons:
            points = [source.points[key] for key in ring]
            assert len(set(ring)) == len(ring) >= 3
            assert contour_is_simple(tuple(points), None), ring
            if tuple(ring) not in originals:
                assert set(ring) <= fitted or uv_is_affine_in_chart(points, [source.facts[(region, key)] for key in ring], None), ring
                assert _own_plane_depth(source, ring) <= CHORD, ring
    assert len(_faces(result)) == sum(len(item) for item in source.polygons) - counters.get(silhouette.EDGES_DISSOLVED, 0)


def test_a_vertex_with_a_twin_copy_of_the_ring_cut_is_never_dissolved():
    """Две копии вершины разреза кольца — одно место: растворение одной открыло бы T-стык на шве с другой."""

    from cftuv_envelope.contracts.metric import CUT_RIGHT_COPY_MARK

    source = _grid(4, 1, merged=True)
    mesh = silhouette._Mesh(source, exact_work_budget(stage="MATERIALIZE"))
    incident = {key: {0} for key in source.positions}
    incident[_key(2, 1) + CUT_RIGHT_COPY_MARK] = {0}

    fixed = mesh._fixed_vertices(incident)

    assert {_key(2, 1), _key(2, 1) + CUT_RIGHT_COPY_MARK} <= fixed
    assert _key(1, 1) not in fixed


# ---------------------------------------------------------------------------
# 2. Проверка закона: красные контроли
# ---------------------------------------------------------------------------


def test_verification_catches_a_dissolved_vertex_of_the_source_chain():
    source = _grid(3, 1)
    result, _counters = _run(source)

    assert not verify_silhouette(source, result)
    ruined = dataclasses.replace(result, dissolved=result.dissolved | {_key(1, 0)})

    assert "SOURCE_OR_WALL_VERTEX_DISSOLVED" in verify_silhouette(source, ruined)


def test_verification_catches_a_changed_station_of_a_kept_vertex():
    source = _grid(3, 1)
    result, _counters = _run(source)
    slot = next(iter(result.facts))
    wrong = dict(result.facts)
    wrong[slot] = (wrong[slot][0] + R(1), wrong[slot][1])

    assert "KEPT_VERTEX_CHANGED_ITS_STATION_OR_UV" in verify_silhouette(source, dataclasses.replace(result, facts=wrong))


def test_verification_catches_a_slide_beyond_the_recorded_maximum():
    source = _grid(4, 1, merged=True, station=_shifted)
    result, _counters = _run(source)
    lowered = tuple((name, 0 if name == silhouette.MAX_UV_SLIDE_MILLI_ALPHA else value) for name, value in result.counters)

    assert "UV_SLIDE_BEYOND_RECORDED_MAXIMUM" in verify_silhouette(source, dataclasses.replace(result, counters=lowered))


def test_verification_catches_a_chord_beyond_the_recorded_maximum():
    source = _grid(4, 1, merged=True, lift=lambda i, j: CHORD * 0.5 if (i, j) == (2, 1) else 0.0)
    result, _counters = _run(source)
    assert _key(2, 1) in result.dissolved
    lowered = tuple((name, 1 if name == silhouette.MAX_CHORD_NM else value) for name, value in result.counters)

    assert "CHORD_DEPTH_BEYOND_RECORDED_MAXIMUM" in verify_silhouette(source, dataclasses.replace(result, counters=lowered))


def test_verification_catches_a_residual_beyond_the_recorded_maximum_and_a_fit_with_no_map():
    source = _grid(3, 1, station=lambda i, j: _shifted(i, j, Fraction(1, 500), (1, 1)))
    result, _counters = _run(source)
    assert result.fits and not verify_silhouette(source, result)
    lowered = tuple((name, 0 if name == silhouette.MAX_UV_RESIDUAL_MILLI_ALPHA else value) for name, value in result.counters)
    collinear = dataclasses.replace(result, fits=((result.fits[0][0], (_key(0, 0), _key(1, 0), _key(2, 0))),))

    assert "UV_RESIDUAL_BEYOND_RECORDED_MAXIMUM" in verify_silhouette(source, dataclasses.replace(result, counters=lowered))
    assert "UV_FIT_HAS_NO_AFFINE_MAP" in verify_silhouette(source, collinear)
    assert "UV_RESIDUAL_BEYOND_RECORDED_MAXIMUM" in verify_silhouette(dataclasses.replace(source, uv_slide=Fraction(1, 100_000)), result)


def test_verification_catches_runs_that_do_not_cover_the_dissolved_vertices():
    source = _grid(4, 1, merged=True)
    result, _counters = _run(source)

    assert "RUNS_DO_NOT_COVER_THE_DISSOLVED_VERTICES" in verify_silhouette(source, dataclasses.replace(result, runs=result.runs[:-1]))


def test_verification_catches_a_torn_outline_and_a_face_that_uses_a_dissolved_vertex():
    source = _grid(4, 1, merged=True)
    result, _counters = _run(source)

    torn = [[ring[1:] for ring in face_polygons] for face_polygons in result.polygons]
    assert "OUTLINE_DOES_NOT_MATCH_THE_BOUNDARY_CHAINS" in verify_silhouette(source, dataclasses.replace(result, polygons=torn))
    revived = [[(*ring, _key(2, 1)) for ring in face_polygons] for face_polygons in result.polygons]
    assert "FACE_IS_DEGENERATE_OR_USES_A_DISSOLVED_VERTEX" in verify_silhouette(source, dataclasses.replace(result, polygons=revived))


def test_apply_refuses_by_name_when_the_verification_fails(monkeypatch):
    source = _grid(3, 1)
    monkeypatch.setattr(silhouette, "verify_silhouette", lambda *_args: ("SOURCE_OR_WALL_VERTEX_DISSOLVED",))

    with pytest.raises(silhouette.MaterializationRefusal) as raised:
        apply_silhouette(source, exact_work_budget(stage="MATERIALIZE"))

    assert raised.value.outcome is MaterializationOutcome.BATCH_DID_NOT_VALIDATE
    assert raised.value.detail.startswith("SILHOUETTE:SOURCE_OR_WALL_VERTEX_DISSOLVED")


# ---------------------------------------------------------------------------
# 3. Поле
# ---------------------------------------------------------------------------


def _field(folder, request_file="decal_request.json", replace_alpha=None, **policy):
    root = FIXTURES / folder
    snapshot = kernel.AnalysisSnapshotCodecV1.loads((root / "analysis_snapshot.json").read_bytes())
    text = (root / request_file).read_text(encoding="utf-8")
    if replace_alpha:
        text = text.replace('"value":"0.6"', f'"value":"{replace_alpha}"')
    request = dataclasses.replace(kernel.DecalRequestCodecV1.loads(text.encode("utf-8")), **policy)
    prepared = prepare_conveyor(snapshot, request)
    return prepared, conveyor_coverage(prepared, request.requested_alpha.value)


@lru_cache(maxsize=None)
def _patch_one():
    return _field("sagging_wall_convex_partition_v1")


@lru_cache(maxsize=None)
def _patch_zero():
    return _field("sagging_wall_rung_chord_v1", "decal_request_alpha_0.6.json", "0.987")


def _materialize(pair, law):
    prepared, coverage = pair
    return materialize_domain(
        prepared,
        coverage,
        request=materialization_request(prepared, uv_policy_id=UV),
        near_planar_lift_law=NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1,
        decal_topology_law=law,
    )


def _chain_vertices(batch, kinds=("SOURCE", "WALL")):
    return {
        key.value
        for chain in batch.boundary_chains
        if chain.semantic_boundary_id.value.split(":")[1] in kinds
        for key in chain.ordered_vert_keys
    }


@pytest.mark.parametrize("pair", ("one", "zero"))
def test_the_field_domain_is_dissolved_and_validates(pair):
    domain_pair = _patch_one() if pair == "one" else _patch_zero()
    planar = _materialize(domain_pair, DecalTopologyLawV1.PLANAR_POLYGONS_V1)
    law = _materialize(domain_pair, DecalTopologyLawV1.SILHOUETTE_TOPOLOGY_V1)

    assert law.outcome is MaterializationOutcome.MATERIALIZED, law.detail
    assert law.decal_topology_law is DecalTopologyLawV1.SILHOUETTE_TOPOLOGY_V1
    counters = dict(law.counters)
    assert counters[silhouette.EDGES_DISSOLVED] > 0 and counters[silhouette.VERTICES_DISSOLVED] > 0
    assert len(law.batch.faces) == len(planar.batch.faces) - counters[silhouette.EDGES_DISSOLVED]
    assert len(law.batch.vertices) == len(planar.batch.vertices) - counters[silhouette.VERTICES_DISSOLVED]
    assert not validate_geometry_batch(law.batch)
    assert counters[silhouette.MAX_CHORD_NM] <= 5_000_000 and counters[silhouette.MAX_UV_SLIDE_MILLI_ALPHA] <= 4
    assert counters.get(silhouette.MAX_UV_RESIDUAL_MILLI_ALPHA, 0) <= 4
    assert any(line.startswith("SILHOUETTE_TOPOLOGY_V1:") for line in law.diagnostics)


@pytest.mark.parametrize("pair", ("one", "zero"))
def test_the_field_chains_of_the_source_and_the_wall_and_the_kept_vertices_are_those_of_the_planar_law(pair):
    """Шов с соседними доменами не тронут: вершины цепей источника и стены те же, T-стыков шва закон не рождает."""

    domain_pair = _patch_one() if pair == "one" else _patch_zero()
    planar = _materialize(domain_pair, DecalTopologyLawV1.PLANAR_POLYGONS_V1).batch
    law = _materialize(domain_pair, DecalTopologyLawV1.SILHOUETTE_TOPOLOGY_V1).batch

    assert _chain_vertices(law) == _chain_vertices(planar)
    planar_vertices = {item.vert_key.value: item for item in planar.vertices}
    assert {item.vert_key.value for item in law.vertices} <= set(planar_vertices)
    for vertex in law.vertices:
        before = planar_vertices[vertex.vert_key.value]
        assert vertex.position == before.position and vertex.semantic_location_ref == before.semantic_location_ref
    planar_station = {(f.vert_key.value, f.semantic_region_id.value): (f.source_s, f.source_r) for f in planar.station_facts}
    for fact in law.station_facts:
        assert planar_station[(fact.vert_key.value, fact.semantic_region_id.value)] == (fact.source_s, fact.source_r)
    planar_uv = {(f.semantic_region_id.value, k.value): u.uv for f in planar.faces for k, u in zip(f.ordered_vert_keys, f.uv_facts)}
    for face in law.faces:
        for key, fact in zip(face.ordered_vert_keys, face.uv_facts):
            assert planar_uv[(face.semantic_region_id.value, key.value)] == fact.uv


def test_every_merged_field_face_is_proved_by_an_independent_exact_predicate_under_a_zero_tolerance(monkeypatch):
    captured = {}
    original = domain_module.apply_silhouette

    def capture(source, budget):
        result = original(source, budget)
        captured["pair"] = (source, result)
        return result

    monkeypatch.setattr(domain_module, "apply_silhouette", capture)
    _materialize(_field("sagging_wall_convex_partition_v1", silhouette_uv_slide=kernel.ExactRationalV1(0, 1)), DecalTopologyLawV1.SILHOUETTE_TOPOLOGY_V1)
    source, result = captured["pair"]
    assert not result.fits and source.uv_slide == 0

    before = {tuple(ring) for face_polygons in source.polygons for ring in face_polygons}
    merged = [(index, ring) for index, face_polygons in enumerate(result.polygons) for ring in face_polygons if tuple(ring) not in before]
    assert merged, "поле обязано слить хоть одну грань"
    for index, ring in merged:
        region = source.layout.region_of(source.frame_faces[index])
        points = [source.points[key] for key in ring]
        assert contour_is_simple(tuple(points), None), ring
        assert uv_is_affine_in_chart(points, [source.facts[(region, key)] for key in ring], None), ring
    assert not verify_silhouette(source, result)


def test_the_other_topology_laws_name_no_silhouette_numbers():
    for law in (DecalTopologyLawV1.TRIANGLES_V1, DecalTopologyLawV1.QUAD_STRIPS_V1, DecalTopologyLawV1.PLANAR_POLYGONS_V1):
        found = _materialize(_patch_one(), law)
        assert found.decal_topology_law is law
        assert not [name for name, _value in found.counters if "SILHOUETTE" in name]
        assert not any(line.startswith("SILHOUETTE") for line in found.diagnostics)


def test_a_domain_where_the_law_changes_nothing_is_the_planar_answer_bitwise():
    from materialize_factories import FIELD_FIXTURES, field_domain

    unchanged = 0
    for name in FIELD_FIXTURES:
        prepared, coverage, _request = field_domain(name)
        planar = _materialize((prepared, coverage), DecalTopologyLawV1.PLANAR_POLYGONS_V1)
        law = _materialize((prepared, coverage), DecalTopologyLawV1.SILHOUETTE_TOPOLOGY_V1)
        counters = dict(law.counters)
        if counters.get(silhouette.EDGES_DISSOLVED) or counters.get(silhouette.VERTICES_DISSOLVED):
            assert len(law.batch.faces) < len(planar.batch.faces) or len(law.batch.vertices) < len(planar.batch.vertices)
            continue
        unchanged += 1
        assert law.content_digest == planar.content_digest and law.batch.semantic_digest == planar.batch.semantic_digest
    assert unchanged, "хоть один полевой домен без растворения обязан доказать прежний ответ побитово"


def test_the_pass_is_deterministic_across_runs():
    first = _materialize(_patch_one(), DecalTopologyLawV1.SILHOUETTE_TOPOLOGY_V1)
    second = _materialize(_patch_one(), DecalTopologyLawV1.SILHOUETTE_TOPOLOGY_V1)

    assert first.content_digest == second.content_digest
    assert [item for item in first.counters if "SILHOUETTE" in item[0]] == [item for item in second.counters if "SILHOUETTE" in item[0]]


def test_the_request_slide_reaches_the_law():
    """Запрос, скомпилированный с другим сдвигом UV, даёт закону другой допуск: у строгого вершин растворяется меньше."""

    strict = _field("sagging_wall_convex_partition_v1", silhouette_uv_slide=kernel.ExactRationalV1(1, 1_000_000))
    default = dict(_materialize(_patch_one(), DecalTopologyLawV1.SILHOUETTE_TOPOLOGY_V1).counters)
    found = dict(_materialize(strict, DecalTopologyLawV1.SILHOUETTE_TOPOLOGY_V1).counters)

    assert found.get(silhouette.VERTICES_DISSOLVED, 0) < default[silhouette.VERTICES_DISSOLVED]
    assert found[silhouette.KEPT_UV] > default.get(silhouette.KEPT_UV, 0)


def test_the_outline_pairs_the_twin_copies_of_a_ring_cut_by_place():
    from cftuv_envelope.contracts.metric import CUT_RIGHT_COPY_MARK as mark

    left = (_key(0, 0), _key(1, 0), _key(1, 1), _key(0, 1))
    right = (_key(1, 0) + mark, _key(2, 0), _key(2, 1), _key(1, 1) + mark)

    outline = silhouette._outline(silhouette._directed([[left, right]]))

    assert frozenset((_key(1, 0), _key(1, 1))) not in outline and len(outline) == 6


@pytest.mark.parametrize("lift", (NearPlanarLiftLawV1.SOURCE_TRIANGLES_V1, NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1))
@pytest.mark.parametrize("which", ("column", "dome"))
def test_a_ring_domain_with_a_periodic_cut_keeps_both_copies_of_every_cut_vertex_and_validates(which, lift):
    """Кольцо носителя разрезано по образующей: две копии вершины разреза — одно место, и закон не трогает ни одну из них."""

    import band_factories as factories
    from cftuv_envelope.contracts.metric import CUT_RIGHT_COPY_MARK

    snapshot, request, _band = (
        factories.band_domain(factories.column_top(), reach_cap="1/2")
        if which == "column"
        else factories.band_domain(factories.dome_ring(sides=16, rings=8), reach_cap="1/4")
    )
    prepared = prepare_conveyor(snapshot, request)
    coverage = conveyor_coverage(prepared)

    def run(law):
        return materialize_domain(
            prepared,
            coverage,
            request=materialization_request(prepared, uv_policy_id=UV),
            near_planar_lift_law=lift,
            decal_topology_law=law,
        )

    planar = run(DecalTopologyLawV1.PLANAR_POLYGONS_V1)
    law = run(DecalTopologyLawV1.SILHOUETTE_TOPOLOGY_V1)

    assert planar.outcome is MaterializationOutcome.MATERIALIZED and law.outcome is MaterializationOutcome.MATERIALIZED, law.detail
    copies = {item.vert_key.value for item in planar.batch.vertices if item.vert_key.value.endswith(CUT_RIGHT_COPY_MARK)}
    assert copies and copies <= {item.vert_key.value for item in law.batch.vertices}
    assert not validate_geometry_batch(law.batch)


# ---------------------------------------------------------------------------
# 4. Политика запроса
# ---------------------------------------------------------------------------


def test_the_slide_policy_is_omitted_on_the_wire_by_default_and_round_trips_when_named():
    request = _patch_one()[0].compilation.decal_request

    assert request.silhouette_uv_slide == kernel.ExactRationalV1(1, 256) and DEFAULT_SILHOUETTE_UV_SLIDE == Fraction(1, 256)
    assert b"silhouette_uv_slide" not in kernel.DecalRequestCodecV1.dumps(request)
    named = dataclasses.replace(request, silhouette_uv_slide=kernel.ExactRationalV1(1, 128))
    wire = kernel.DecalRequestCodecV1.dumps(named)
    assert b"silhouette_uv_slide" in wire
    assert kernel.DecalRequestCodecV1.loads(wire).silhouette_uv_slide == kernel.ExactRationalV1(1, 128)


def test_the_slide_policy_has_a_lawful_range():
    request = _patch_one()[0].compilation.decal_request

    assert silhouette_uv_slide_is_lawful(Fraction(1, 256)) and silhouette_uv_slide_is_lawful(MAX_SILHOUETTE_UV_SLIDE)
    assert silhouette_uv_slide_is_lawful(Fraction(0))  # нуль — точное правило
    assert not silhouette_uv_slide_is_lawful(Fraction(-1, 256)) and not silhouette_uv_slide_is_lawful(MAX_SILHOUETTE_UV_SLIDE * 2)
    assert not validate_decal_request(request)
    assert not validate_decal_request(dataclasses.replace(request, silhouette_uv_slide=kernel.ExactRationalV1(0, 1)))
    for bad in (kernel.ExactRationalV1(1, 2), kernel.ExactRationalV1(-1, 256)):
        issues = validate_decal_request(dataclasses.replace(request, silhouette_uv_slide=bad))
        assert [item.path for item in issues] == [("silhouette_uv_slide",)], bad
