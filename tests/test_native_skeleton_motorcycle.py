"""Нативные сетка ячеек, motorcycle graph, закон edge-кандидата и остаток точного вида (`native/cftuv-skeleton`, WP-S1 и WP-S2b) равны эталону на Python.

Эталон — `kernel/src/cftuv_envelope/wavefront`: `cell_grid.py` (`CellGridV1`, `CellIndexV1`: `floor`/`ceil` дробей к минус бесконечности, замкнутые ячейки), `motorcycle.py`
(`build_motorcycle_graph`: трассы, марш по ячейкам, куча крушений на `heapq`, `trace_for`, `TraceCandidateIndexV1`), `candidate_law.evaluate_edge_candidate`,
`exact_candidate_view` (`edge_event_time`, `is_future`, `collapsing_span`, `span_end`, `sliding_projection`), `poststate_span.classify_poststate_span`, `proof.ProofLedger`.

Метод тот же, что у `test_native_skeleton_parts.py`: пошаговая сверка на ОДНОМ состоянии (`tools/native_leaf_gate.py`, `tools/native_motorcycle_gate.py`): состояние ДО вызова копируется,
эталон исполняется на живом, нативный шов — на копии, сравнение ТОЧНОЕ: результат канонически (`int` и `Fraction` различны; у графа — стены, сетка, корзины по порядку, каждое поле трассы,
восемь счётчиков, следующая идентичность), исключение `(класс, текст)`, сдвиг `SIGN_COUNTS`, шесть статей бюджета либо неоплаченное, журнал памяти канонизации против живых таблиц
ПОСЛЕ, рост памяти superlevel'а. Объекты со своим состоянием (индекс трасс, реестр proof) записываются СЦЕНАРИЕМ вызовов и проигрываются нативно после прогона.
Источники вызовов: настоящий `build_skeleton` на корпусах ядра (все бюджеты), полевые полигоны, тесты ядра (плагин `native_motorcycle_gate`, подпроцессом), случайные виды закона и прямые.

Модуль пропускается с названной причиной, пока расширение не собрано (`python tools/native_build.py`) либо дерево ядра ушло от закреплённых файлов листа.
"""

from __future__ import annotations

import json
import math
import os
import random
import subprocess
import sys
from collections import Counter
from dataclasses import dataclass
from fractions import Fraction
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "kernel" / "src", ROOT / "kernel" / "tests", ROOT / "tools", ROOT / "tests"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

try:
    import cftuv_native
except ModuleNotFoundError as error:
    if error.name != "cftuv_native":
        raise
    pytest.skip(
        "расширение cftuv_native не собрано: `python tools/native_build.py` ставит его в dev-venv (сверка сетки, motorcycle graph и закона edge-кандидата с эталоном пропущена)",
        allow_module_level=True,
    )

from cftuv_native import skeleton_seams as wire  # noqa: E402

_STALE = wire.stale_leaf_files()
if _STALE:
    pytest.skip(
        f"файлы эталона, которые зеркалит нативный лист скелета, ушли от закрепления: {', '.join(_STALE)} (перенос дельты и новое закрепление — отдельный шаг)",
        allow_module_level=True,
    )

from native_gate import field_tier  # noqa: E402

import native_corpus as nc  # noqa: E402
import native_leaf_gate as leaf  # noqa: E402
import native_motorcycle_gate as gate  # noqa: E402
import test_native_skeleton_parts as parts  # noqa: E402
import wavefront_cases as cases  # noqa: E402

import cftuv_envelope.exact_sqrt_sum as exact  # noqa: E402
import cftuv_envelope.wavefront.candidate_law as candidate_law  # noqa: E402
import cftuv_envelope.wavefront.cell_grid as cell_grid  # noqa: E402
import cftuv_envelope.wavefront.event_time as et  # noqa: E402
import cftuv_envelope.wavefront.events as events  # noqa: E402
import cftuv_envelope.wavefront.exact_candidate_view as view_module  # noqa: E402
import cftuv_envelope.wavefront.motorcycle as motorcycle  # noqa: E402
import cftuv_envelope.wavefront.poststate_span as poststate_span  # noqa: E402
import cftuv_envelope.wavefront.proof as proof  # noqa: E402
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1  # noqa: E402
from cftuv_envelope.wavefront.candidate_refusal import CandidateRefusal  # noqa: E402
from cftuv_envelope.wavefront.polygon import FanSupportV1, LoopV1, PolygonV1, VertexFanV1, with_vertex_fans  # noqa: E402

CHECKED: Counter = Counter()
FIELD_POLYGONS = gate.FIELD_POLYGONS

#: The polygons of the owner's FIELD scene (`tools/native_leaf_gate.py fetch`): they exist only on the owner's drive, so the tests that need them are the field tier (`native_gate.field_tier`: CI
#: deselects the tier by name and reports it; a skip would be a failure in strict mode).
needs_field_polygons = field_tier(FIELD_POLYGONS.exists(), f"нет записанных полевых полигонов ({FIELD_POLYGONS}): их кладёт `python tools/native_leaf_gate.py fetch`")
#: полевые полигоны, которые тест прогоняет через настоящий `build_skeleton` (остальные — через один `build_motorcycle_graph`: он дёшев на каждом)
FIELD_FAST = parts.FIELD_FAST


@pytest.fixture(autouse=True)
def _kernel_process_state_is_given_back():
    """Эталон пишет в процессные счётчики и память ядра; тест их не оставляет."""

    counts = dict(exact.SIGN_COUNTS)
    unbudgeted = exact.UNBUDGETED_WORK.spent_by_article()
    audit = exact.set_canonical_audit(False)
    with exact.isolated_factorization_memory():
        yield
    exact.set_canonical_audit(audit)
    exact.SIGN_COUNTS.update(counts)
    for name, value in zip(nc._ARTICLES, unbudgeted):
        setattr(exact.UNBUDGETED_WORK, name, value)


@pytest.fixture(scope="module", autouse=True)
def _report_the_comparison_count(request):
    yield
    reporter = request.config.pluginmanager.get_plugin("terminalreporter")
    if reporter is not None:
        reporter.write_line("native motorcycle graph and edge law compared with the Python oracle: " + ", ".join(f"{key} {value}" for key, value in sorted(CHECKED.items())))


def settle(verifier) -> None:
    """Ничего не разошлось, ничего не отказано портом; учёт сверенных вызовов в итог модуля."""

    verifier.finish()
    assert not verifier.mismatches, parts.explain(verifier)
    assert not verifier.unsupported, f"the port refused calls it should carry: {dict(verifier.unsupported)}"
    for seam, count in verifier.checked.items():
        CHECKED[seam] += count


def everything() -> leaf.Sampling:
    return leaf.Sampling(head=10**9, stride=1, cap=10**9)


def verifier_of(**options) -> gate.MotorcycleVerifier:
    return gate.MotorcycleVerifier(everything(), **options)


# --------------------------------------------------------------------------
# the table, the encoders
# --------------------------------------------------------------------------


def test_the_seam_table_of_the_shim_is_the_extensions():
    assert wire.SEAMS == cftuv_native.skeleton_seam_table()


def test_the_new_files_of_the_leaf_are_pinned_and_the_pins_are_the_trees():
    for name in ("wavefront/cell_grid.py", "wavefront/motorcycle.py", "wavefront/poststate_span.py", "wavefront/proof.py", "wavefront/polygon.py", "robust/predicates.py"):
        assert name in wire.LEAF_FILES and name in wire.LEAF_PINS
    assert wire.stale_leaf_files() == ()


def test_a_polygon_beyond_the_machine_range_is_unsupported_not_wrong():
    polygon = PolygonV1.build(((0, 0), (1 << 60, 0), (1 << 60, 8), (0, 8)))
    with pytest.raises(wire.SeamUnsupported):
        wire.enc_polygon(polygon)
    assert wire.enc_polygon(PolygonV1.build(((0, 0), (12, 0), (12, 8), (0, 8)))) is not None


# --------------------------------------------------------------------------
# cell_grid.py: floor and ceiling, the grid, the index
# --------------------------------------------------------------------------


def random_number(rng):
    kind = rng.randrange(5)
    if kind == 0:
        return rng.randrange(-60, 60)
    if kind == 1:
        return Fraction(rng.randrange(-400, 400), rng.choice((2, 3, 4, 5, 7, 8, 12)))
    if kind == 2:
        return Fraction(rng.randrange(-(10**20), 10**20), rng.randrange(1, 10**9))
    return rng.choice((0, 1, -1, Fraction(1, 2), Fraction(-1, 2), Fraction(-7, 3), Fraction(7, 3), Fraction(-6, 3)))


def random_grid(rng):
    x_min, y_min = rng.randrange(-70, 70), rng.randrange(-70, 70)
    return cell_grid.CellGridV1(x_min, y_min, x_min + rng.randrange(0, 90), y_min + rng.randrange(0, 90), rng.choice((1, 2, 3, 4, 5, 8, 16, 64)))


def grid_call(runner, op, *arguments):
    answer = runner.call("CELL_GRID", [op, *arguments])
    assert answer.ok, (op, answer.status, answer.detail)
    return answer.value


def test_floor_and_ceil_of_fractions_go_toward_minus_infinity():
    rng = random.Random(1)
    runner = wire.SeamRunner()
    for _ in range(800):
        value = random_number(rng)
        assert grid_call(runner, 8, value) == [math.floor(value), math.ceil(value)], value
    for value in (Fraction(-1, 2), Fraction(-7, 3), Fraction(7, 3), Fraction(-6, 3), -Fraction(10**40 + 1, 10**20), Fraction(10**40 + 1, 10**20)):
        assert grid_call(runner, 8, value) == [math.floor(value), math.ceil(value)], value
    CHECKED["floor/ceil"] += 806


def test_the_grid_equals_the_oracle_on_random_grids_with_negative_origins_boxes_and_lines():
    rng = random.Random(7)
    runner = wire.SeamRunner()
    counts = Counter()
    for _ in range(500):
        grid = random_grid(rng)
        encoded = wire.enc_grid(grid)
        assert grid_call(runner, 1, encoded) == [grid.columns, grid.rows, grid.x_limit, grid.y_limit]
        x, y = random_number(rng), random_number(rng)
        assert grid_call(runner, 2, encoded, x, y) == [grid.column(x), grid.row(y)]
        edges = sorted((random_number(rng), random_number(rng)))
        other = sorted((random_number(rng), random_number(rng)))
        x_low, x_high, y_low, y_high = edges[0], edges[1], other[0], other[1]
        assert grid_call(runner, 4, encoded, x_low, x_high, y_low, y_high) == grid.contains_box(x_low, x_high, y_low, y_high)
        cells = grid.box_cells(x_low, x_high, y_low, y_high)
        assert wire.dec_cells(grid_call(runner, 3, encoded, x_low, x_high, y_low, y_high, False)) == cells
        counts["box cells"] += len(cells)
        whole = [math.floor(x_low), math.ceil(x_high), math.floor(y_low), math.ceil(y_high)]
        assert wire.dec_cells(grid_call(runner, 3, encoded, *whole, True)) == grid.box_cells(*whole) == wire.dec_cells(grid_call(runner, 3, encoded, *whole, False))
        a, b = rng.randrange(-9, 10), rng.randrange(-9, 10)
        c = rng.randrange(-300, 300)
        window = None if rng.random() < 0.4 else [x_low, x_high, y_low, y_high]
        found = wire.dec_cells(grid_call(runner, 5, encoded, a, b, c, window))
        assert found == grid.line_cells(a, b, c, window=None if window is None else tuple(window))
        counts["line cells"] += len(found)
        start, end = (rng.randrange(-80, 80), rng.randrange(-80, 80)), (rng.randrange(-80, 80), rng.randrange(-80, 80))
        found = wire.dec_cells(grid_call(runner, 6, encoded, *start, *end))
        assert found == grid.segment_cells(start, end)
        counts["segment cells"] += len(found)
    assert counts["box cells"] > 500 and counts["line cells"] > 500 and counts["segment cells"] > 500, counts
    CHECKED["cell grid"] += 500


def test_the_cell_index_equals_the_oracle_in_order_and_in_lookup():
    rng = random.Random(11)
    runner = wire.SeamRunner()
    for _ in range(120):
        oracle = cell_grid.CellIndexV1()
        script, expected = [], []
        for _step in range(rng.randrange(1, 30)):
            cells = tuple((rng.randrange(-3, 6), rng.randrange(-3, 6)) for _ in range(rng.randrange(0, 5)))
            if rng.random() < 0.6:
                ident = rng.randrange(0, 12)
                oracle.add(ident, cells)
                script.append([0, ident, [list(cell) for cell in cells]])
                expected.append(None)
            else:
                script.append([1, [list(cell) for cell in cells]])
                expected.append(oracle.lookup(cells))
        results, buckets = wire.dec_cell_grid(7, grid_call(runner, 7, script))
        assert [None if item is None else tuple(item) for item in results] == expected
        assert buckets == tuple((cell, tuple(idents)) for cell, idents in oracle.buckets.items())
    CHECKED["cell index"] += 120


def test_an_empty_grid_area_is_the_named_refusal_with_the_oracles_text():
    runner = wire.SeamRunner()
    answer = runner.call("CELL_GRID", [0, 5, 0, 4, 9, 4])
    with pytest.raises(cell_grid.CellGridRejected) as caught:
        cell_grid.CellGridV1.covering((5, 0, 4, 9), targets=4)
    assert wire.exception_of(answer, None) == ("CellGridRejected", str(caught.value))
    rng = random.Random(3)
    for _ in range(300):
        box = (rng.randrange(-50, 50), rng.randrange(-50, 50), rng.randrange(-50, 120), rng.randrange(-50, 120))
        targets = rng.randrange(-3, 400)
        answer = runner.call("CELL_GRID", [0, *box, targets])
        try:
            oracle = cell_grid.CellGridV1.covering(box, targets=targets)
        except cell_grid.CellGridRejected as error:
            assert not answer.ok and wire.exception_of(answer, None) == ("CellGridRejected", str(error))
            continue
        assert answer.ok and nc.canonical(wire.dec_cell_grid(0, answer.value)) == nc.canonical(oracle)
    CHECKED["grid covering"] += 300


# --------------------------------------------------------------------------
# the graph: named, weighted, generated and fan polygons, every budget
# --------------------------------------------------------------------------

ORACLE_OUTCOMES = (
    exact.ExactCanonicalizationWorkBudgetExhausted,
    et.ZeroDivisorTimeError,
    et.ParallelSupportLinesError,
    ZeroDivisionError,
    exact.ZeroSqrtSumDivisorError,
    ValueError,
)


def random_weighted(figure, rng):
    """The same figure with a random speed on every edge (a wall, a unit source, a heavier or a fractional one); at least one edge is a source."""

    speeds = []
    for start, end, _ in figure.edges():
        unit = et.SupportLineV1.through(start, end).q
        speeds.append((start, end, rng.choice((0, unit, unit, 4 * unit, unit * Fraction(9, 4), unit * Fraction(1, 4), unit * Fraction(25, 9)))))
    if not any(speed for _, _, speed in speeds):
        speeds[0] = (*speeds[0][:2], et.SupportLineV1.through(*speeds[0][:2]).q)
    from cftuv_envelope.wavefront.polygon import with_edge_speeds

    return with_edge_speeds(figure, tuple(speeds))


def fan_polygons() -> list:
    ell12 = cases.ell(12)
    return [
        ("ell_fan_one", with_vertex_fans(ell12, (VertexFanV1((6, 6), (FanSupportV1(-1, -2, 5),)),))),
        ("ell_fan_two", with_vertex_fans(ell12, (VertexFanV1((6, 6), (FanSupportV1(-1, -2, 5), FanSupportV1(-2, -1, 5))),))),
        ("ell_fan_empty", with_vertex_fans(ell12, (VertexFanV1((6, 6), ()),))),
    ]


def generated_polygons() -> list:
    found = []
    for vertices in (7, 9, 13, 17, 23, 31):
        for seed in range(5):
            polygon = cases.star(vertices, seed)
            if polygon is not None:
                found.append((f"star_{vertices}_{seed}", polygon))
    for teeth in (2, 3, 5, 8, 11):
        found.append((f"comb_{teeth}", cases.comb(teeth)))
    found += [("holes_grid_2x3", cases.holes_grid(2, 3)), ("holes_grid_3x3", cases.holes_grid(3, 3)), ("cross_wide", cases.cross(wide=6, tall=5, right=26, top=24, arm=6))]
    rng = random.Random(2026)
    for name, figure in (("ell", cases.ell(12)), ("staircase", cases.staircase()), ("comb_4", cases.comb(4)), ("u_shape", cases.u_shape()), ("double_notch", cases.double_notch()), ("holes_2", cases.holes_grid(1, 2))):
        for variant in range(4):
            found.append((f"weighted_{name}_{variant}", random_weighted(figure, rng)))
    return found + fan_polygons()


def graph_pass(verifier, polygon, budget):
    """`build_motorcycle_graph` on the live state with the verifier's wrappers; the oracle's own refusals are outcomes."""

    with verifier.installed():
        try:
            return motorcycle.build_motorcycle_graph(polygon, budget)
        except ORACLE_OUTCOMES as error:
            return error


def spent_of(polygon) -> int:
    probe = leaf.fresh_process_state()
    motorcycle.build_motorcycle_graph(polygon, probe)
    return probe.spent


def test_the_graph_equals_the_oracle_on_every_named_weighted_generated_and_fan_polygon():
    verifier = verifier_of()
    polygons = parts.named_polygons() + generated_polygons()
    traces = 0
    for _name, polygon in polygons:
        graph = graph_pass(verifier, polygon, leaf.fresh_process_state())
        traces += 0 if isinstance(graph, Exception) else len(graph.traces)
    settle(verifier)
    assert verifier.checked["BUILD_MOTORCYCLE_GRAPH"] == len(polygons) and traces > 150, traces
    assert verifier.checked["TRACE_INDEX_SCRIPT"] == 0 and verifier.checked["PROOF_SCRIPT"] == 0
    CHECKED["graph polygons"] += len(polygons)


def moved(polygon, dx: int, dy: int, factor: int = 1):
    """The same polygon translated (negative origins, far from zero) and scaled by a whole factor: the lattice moves, the shape and the speeds stay."""

    def loop_of(loop):
        speeds = None if loop.speeds_squared is None else tuple(speed * factor * factor for speed in loop.speeds_squared)
        return LoopV1(tuple((x * factor + dx, y * factor + dy) for x, y in loop.points), speeds)

    fans = tuple(VertexFanV1((fan.point[0] * factor + dx, fan.point[1] * factor + dy), fan.supports) for fan in polygon.vertex_fans)
    return PolygonV1(loop_of(polygon.outer), tuple(loop_of(hole) for hole in polygon.holes), fans)


def test_the_graph_equals_the_oracle_on_translated_and_scaled_polygons():
    """Negative and far origins, big line offsets (`c` of 90 bits), cells of other sizes: the grid is a filter whose floors must be the oracle's on every one of them."""

    verifier = verifier_of()
    chosen = [(name, polygon) for name, polygon in parts.named_polygons() if name in ("ell", "comb_4", "hole_1", "holes_2", "cross", "double_notch", "staircase", "star_9_seed_4")]
    offsets = ((-17, -5, 1), (-10**6, 3 * 10**9, 1), (2**40, -(2**41), 2), (-(2**44), 2**44 + 7, 3), (0, -1, 8))
    for _name, polygon in chosen:
        for dx, dy, factor in offsets:
            graph = graph_pass(verifier, moved(polygon, dx, dy, factor), leaf.fresh_process_state())
            assert not isinstance(graph, Exception)
    settle(verifier)
    assert verifier.checked["BUILD_MOTORCYCLE_GRAPH"] == len(chosen) * len(offsets)


def test_the_graph_equals_the_oracle_unbudgeted():
    """`None` is the unbudgeted telemetry (`UNBUDGETED_WORK`)."""

    verifier = verifier_of()
    polygons = parts.named_polygons()[::2] + generated_polygons()[::3]
    for _name, polygon in polygons:
        leaf.fresh_process_state()
        graph_pass(verifier, polygon, None)
    settle(verifier)
    assert verifier.checked["BUILD_MOTORCYCLE_GRAPH"] == len(polygons)


def test_the_graph_runs_out_of_budget_at_the_operation_the_oracle_does():
    """A cap under what the build spends, swept: the exhaustion falls in the factorization of the speeds, in a radical of a velocity, a hydration of a wall hit, a sign of the
    crash queue, a division of a crash point; the articles and the memory it leaves are the oracle's."""

    verifier = verifier_of()
    swept = 0
    for _name, polygon in parts.named_polygons()[8:] + generated_polygons()[:14]:
        spent = spent_of(polygon)
        if spent == 0:
            continue
        for numerator in (1, 3, 5, 7, 9):
            leaf.fresh_process_state()
            budget = exact.exact_work_budget(stage="PREPARE", domain_id="starved", cap=max(1, spent * numerator // 10))
            graph_pass(verifier, polygon, budget)
            swept += 1
    settle(verifier)
    assert verifier.checked["BUILD_MOTORCYCLE_GRAPH"] == swept
    assert verifier.raised["BUILD_MOTORCYCLE_GRAPH"] >= 40, f"the starved builds rarely ran out inside the graph: {dict(verifier.raised)}"


@needs_field_polygons
def test_the_graph_equals_the_oracle_on_the_field_polygons_when_they_are_in_the_corpus():
    verifier = verifier_of()
    polygons = leaf.load_polygons(FIELD_POLYGONS)
    assert polygons
    for _name, polygon in polygons:
        leaf.fresh_process_state()
        graph_pass(verifier, polygon, leaf.fresh_process_state())
    settle(verifier)
    assert verifier.checked["BUILD_MOTORCYCLE_GRAPH"] == len(polygons)
    CHECKED["field graphs"] += len(polygons)


def test_the_graph_crashes_traces_into_traces_in_the_corpus():
    """The branches the equality means little without: crashes into other traces (TRACE and SIMULTANEOUS) and refusals of a march that is not bounded."""

    kinds, outcomes = Counter(), Counter()
    for _name, polygon in parts.named_polygons() + generated_polygons():
        leaf.fresh_process_state()
        graph = motorcycle.build_motorcycle_graph(polygon, exact.exact_work_budget(stage="PREPARE", domain_id="k"))
        for trace in graph.traces.values():
            kinds[trace.crash_kind.value] += 1
            outcomes[trace.outcome.value] += 1
    assert kinds["WALL"] > 100 and kinds["TRACE"] > 0 and kinds["SIMULTANEOUS"] > 0, dict(kinds)
    assert outcomes["EXACT"] > 100, dict(outcomes)


def test_the_march_that_runs_out_of_steps_is_the_oracles_named_outcome_when_the_budget_is_replaced():
    """The kernel test replaces `march_budget` to force `MOTORCYCLE_MARCH_BUDGET_EXHAUSTED`: the host reads the live function and hands its answer to the native side."""

    verifier = verifier_of()
    original = motorcycle.march_budget
    named = dict(parts.named_polygons())
    try:
        for steps in (0, 1):
            motorcycle.march_budget = lambda grid, steps=steps: steps
            for name in ("comb_4", "ell"):
                graph = graph_pass(verifier, named[name], leaf.fresh_process_state())
                exhausted = any(trace.outcome is motorcycle.TraceOutcome.MOTORCYCLE_MARCH_BUDGET_EXHAUSTED for trace in graph.traces.values())
                assert exhausted or steps > 0, "a march of no steps found a wall"
    finally:
        motorcycle.march_budget = original
    settle(verifier)
    assert verifier.checked["BUILD_MOTORCYCLE_GRAPH"] == 4


# --------------------------------------------------------------------------
# trace_for: the trace of a vertex born during the count
# --------------------------------------------------------------------------


def random_line_in(rng, x_min, y_min, x_max, y_max):
    while True:
        start = (rng.randrange(x_min, x_max + 1), rng.randrange(y_min, y_max + 1))
        end = (rng.randrange(x_min - 3, x_max + 4), rng.randrange(y_min - 3, y_max + 4))
        if start != end:
            return et.SupportLineV1.with_speed(start, end, parts.random_speed(rng))


def born_lines(rng, graph, polygon):
    """Two lines, a start and an origin for a born vertex: the lines of a trace of the graph, or random lines of the polygon's scale, from a place in or beside the polygon."""

    x_min, y_min, x_max, y_max = motorcycle.polygon_box(polygon)
    if graph.traces and rng.random() < 0.5:
        trace = rng.choice(list(graph.traces.values()))
        left, right = trace.left_line, trace.right_line
    else:
        left, right = (random_line_in(rng, x_min, y_min, x_max, y_max) for _ in range(2))
    origin = et.EventPointV1(SqrtSumV1.rational(Fraction(rng.randrange(x_min * 4 - 8, x_max * 4 + 9), 4)), SqrtSumV1.rational(Fraction(rng.randrange(y_min * 4 - 8, y_max * 4 + 9), 4)))
    start = rng.choice((et.ZERO_TIME, parts.small_time(rng), et.EventTimeV1.normalized(Fraction(rng.randrange(1, 4000), 3), SqrtSumV1.rational(1))))
    return left, right, start, origin


def test_trace_for_equals_the_oracle_on_random_born_vertices_in_every_outcome():
    rng = random.Random(41)
    verifier = verifier_of()
    outcomes = Counter()
    polygons = parts.named_polygons()[4:16] + generated_polygons()[::4]
    with verifier.installed():
        for _name, polygon in polygons:
            graph = motorcycle.build_motorcycle_graph(polygon, leaf.fresh_process_state())
            for _ in range(14):
                left, right, start, origin = born_lines(rng, graph, polygon)
                graph.work_budget = parts.random_budget(rng)
                try:
                    trace = graph.trace_for(left, right, start, origin)
                except ORACLE_OUTCOMES:
                    outcomes["refused"] += 1
                    continue
                outcomes[trace.outcome.value + "/" + trace.crash_kind.value] += 1
    settle(verifier)
    assert verifier.checked["TRACE_FOR"] == len(polygons) * 14
    assert outcomes["EXACT/WALL"] > 40 and outcomes["MOTORCYCLE_HAS_NO_BISECTOR/NONE"] > 0 and outcomes["MOTORCYCLE_NEVER_MEETS_A_WALL/NONE"] > 0 and outcomes["refused"] > 0, dict(outcomes)


def test_a_born_vertex_in_the_corpus_gets_the_trace_the_oracle_gives():
    """`trace_for` as the builder calls it: vertices born during the count (and the fan vertices of the input), inside a real `build_skeleton`."""

    verifier = verifier_of()
    for _name, polygon in parts.named_polygons() + fan_polygons():
        parts.run_with_leaf(verifier, polygon)
    settle(verifier)
    assert verifier.checked["TRACE_FOR"] > 5


# --------------------------------------------------------------------------
# the trace index: scripts of the real runs and of random calls
# --------------------------------------------------------------------------


def random_trace_for_index(rng, polygon):
    """A synthetic trace: a box of the polygon's scale with a reach that fits the area, exceeds it, or is unknown."""

    x_min, y_min, x_max, y_max = motorcycle.polygon_box(polygon)
    unit = et.SupportLineV1(1, 0, 0, 1)
    origin = et.EventPointV1(SqrtSumV1.rational(rng.randrange(x_min, x_max + 1)), SqrtSumV1.rational(rng.randrange(y_min, y_max + 1)))
    point = et.EventPointV1(SqrtSumV1.rational(Fraction(rng.randrange(x_min * 3 - 6, x_max * 3 + 7), 3)), SqrtSumV1.rational(Fraction(rng.randrange(y_min * 3 - 6, y_max * 3 + 7), 3)))
    reach = rng.choice((None, Fraction(0), Fraction(rng.randrange(1, 40), 7), Fraction(rng.randrange(1, 4000), 3)))
    crashed = rng.random() < 0.85
    return motorcycle.TraceV1(
        rng.randrange(0, 400),
        motorcycle.TraceOutcome.EXACT if crashed else motorcycle.TraceOutcome.MOTORCYCLE_NEVER_MEETS_A_WALL,
        unit,
        unit,
        et.ZERO_TIME,
        origin,
        (SqrtSumV1.zero(), SqrtSumV1.zero()),
        parts.small_time(rng) if crashed else None,
        point if crashed else None,
        motorcycle.CrashKind.WALL if crashed else motorcycle.CrashKind.NONE,
        0 if crashed else -1,
        reach,
    )


def test_the_trace_index_equals_the_oracle_on_random_scripts():
    rng = random.Random(53)
    verifier = verifier_of()
    registered = Counter()
    polygons = parts.named_polygons()[4:] + generated_polygons()[::3]
    with verifier.installed():
        for _name, polygon in polygons:
            graph = motorcycle.build_motorcycle_graph(polygon, leaf.fresh_process_state())
            index = motorcycle.TraceCandidateIndexV1.covering(polygon, graph)
            lines = [et.SupportLineV1.with_speed(start, end, speed) for start, end, speed in polygon.edges()]
            for _ in range(rng.randrange(4, 30)):
                roll = rng.random()
                if roll < 0.3:
                    index.register_line(rng.randrange(0, 12), rng.choice(lines))
                elif roll < 0.55:
                    trace = rng.choice(list(graph.traces.values())) if graph.traces and rng.random() < 0.6 else random_trace_for_index(rng, polygon)
                    registered[bool(index.register_trace(rng.randrange(0, 20), trace))] += 1
                elif roll < 0.7:
                    try:
                        index.lines_near(rng.randrange(0, 22))
                    except KeyError:
                        registered["unknown vertex"] += 1
                elif roll < 0.85:
                    index.vertices_near(rng.randrange(0, 14))
                else:
                    (index.knows_vertex if rng.random() < 0.5 else index.knows_line)(rng.randrange(0, 22))
    settle(verifier)
    assert verifier.checked["TRACE_INDEX_SCRIPT"] == len(polygons)
    assert registered[True] > 30 and registered[False] > 30 and registered["unknown vertex"] > 5, dict(registered)


def test_the_trace_index_of_a_real_run_equals_the_oracle_including_the_exhaustive_fallback():
    verifier = verifier_of()
    polygons = parts.named_polygons() + fan_polygons() + generated_polygons()[::5]
    for _name, polygon in polygons:
        parts.run_with_leaf(verifier, polygon)
    settle(verifier)
    assert verifier.checked["TRACE_INDEX_SCRIPT"] == len(polygons)
    assert verifier.replayed["index operations"] > 1000


# --------------------------------------------------------------------------
# the edge law and the rest of the exact view, on real skeletons and on random views
# --------------------------------------------------------------------------

EDGE_REASONS = (
    "CANDIDATE",
    "FILTER_SOLO_VERTEX",
    "FILTER_TRIPLE_NEVER_CONCURRENT",
    "NO_RULE_TRIPLE_ALWAYS_CONCURRENT",
    "FILTER_EVENT_IN_THE_PAST",
    "FILTER_SPAN_DOES_NOT_COLLAPSE",
    "FILTER_SPAN_IS_BORN_ZERO",
)


def edge_outcomes(verifier) -> Counter:
    return Counter({key.split(":", 1)[1]: value for key, value in verifier.outcomes.items() if key.startswith("EVALUATE_EDGE_CANDIDATE:")})


def test_evaluate_edge_candidate_equals_the_oracle_on_every_call_of_the_named_polygons():
    verifier = verifier_of()
    polygons = parts.named_polygons()
    for _name, polygon in polygons:
        parts.run_with_leaf(verifier, polygon)
    settle(verifier)
    assert verifier.checked["EVALUATE_EDGE_CANDIDATE"] > 1000
    for reason in ("CANDIDATE", "FILTER_TRIPLE_NEVER_CONCURRENT", "NO_RULE_TRIPLE_ALWAYS_CONCURRENT", "FILTER_EVENT_IN_THE_PAST", "FILTER_SPAN_DOES_NOT_COLLAPSE"):
        assert edge_outcomes(verifier)[reason] > 0, f"no call of the corpus ended in {reason}: {dict(edge_outcomes(verifier))}"
    for seam in ("EDGE_EVENT_TIME", "SPAN_END", "CLASSIFY_POSTSTATE_SPAN", "SLIDING_PROJECTION"):
        assert verifier.checked[seam] > 0, f"the corpus never made a direct call of {seam}"


@pytest.mark.parametrize("kind", ("none", "dense", "weighted_and_fans"))
def test_the_edge_law_equals_the_oracle_unbudgeted_without_the_memory_of_places_and_on_weights_and_fans(kind):
    verifier = verifier_of()
    if kind == "weighted_and_fans":
        chosen = [(name, polygon) for name, polygon in generated_polygons() if name.startswith(("weighted", "ell_fan"))]
    else:
        chosen = parts.chosen_polygons()
    for _name, polygon in chosen:
        parts.run_with_leaf(verifier, polygon, budget_kind="none" if kind == "none" else "full", dense=kind == "dense")
    settle(verifier)
    assert verifier.checked["EVALUATE_EDGE_CANDIDATE"] > 150


def test_the_edge_law_equals_the_oracle_when_the_budget_runs_out_inside_it():
    verifier = verifier_of()
    for fraction in ("0.7", "0.9", "0.98", "0.995"):
        for _name, polygon in parts.chosen_polygons():
            parts.run_with_leaf(verifier, polygon, budget_kind=fraction)
    settle(verifier)
    assert verifier.checked["EVALUATE_EDGE_CANDIDATE"] > 150
    assert sum(verifier.raised[name] for name in verifier.raised) >= 3, f"the starved runs never ran out inside a checked call: {dict(verifier.raised)}"


@needs_field_polygons
def test_the_polygons_of_the_field_equal_the_oracle_on_the_edge_law_when_they_are_in_the_corpus():
    verifier = verifier_of()
    polygons = leaf.load_polygons(FIELD_POLYGONS, only=FIELD_FAST)
    assert polygons
    for _name, polygon in polygons:
        parts.run_with_leaf(verifier, polygon)
    settle(verifier)
    CHECKED["field polygons (edge law)"] += len(polygons)


class MeetingView(parts.RandomView):
    """A random view whose first three spans carry lines through ONE point at one integer time, vertices that share spans like a front (`vertex.next == peer.prev`), births at or
    before the meeting, and (by the mode) sliding vertices on one line: the meetings the edge law decides about, in every branch."""

    def __init__(self, rng, *, budget, memo: bool) -> None:
        super().__init__(rng, budget=budget, memo=memo)
        lines = parts.meeting_lines(rng, 4)
        mode = rng.choice(("plain", "plain", "vertex", "peer", "both"))
        shift = rng.choice((0, 0, rng.randrange(-3, 4)))
        a, b, c = lines[0], lines[1], lines[2]
        if mode in ("vertex", "both"):
            b = et.SupportLineV1(a.a, a.b, a.c + shift, a.q)
        if mode in ("peer", "both"):
            c = et.SupportLineV1(b.a, b.b, b.c + shift, b.q)
        birth = rng.choice((et.ZERO_TIME, et.ZERO_TIME, parts.small_time(rng)))
        point = et.EventPointV1(SqrtSumV1.rational(rng.randrange(-5, 6)), SqrtSumV1.rational(rng.randrange(-5, 6)))
        slide_vertex = view_module.sliding_projection(a, b, point) if mode in ("vertex", "both") else None
        slide_peer = view_module.sliding_projection(b, c, point) if mode in ("peer", "both") else None
        self.vertices = {
            0: view_module.CandidateVertexStateV1(0, 1, birth, slide_vertex),
            1: view_module.CandidateVertexStateV1(1, 2, birth if rng.random() < 0.6 else parts.small_time(rng), slide_peer),
            2: view_module.CandidateVertexStateV1(2, 3, birth, None),
        }
        self.spans = {
            number: view_module.CandidateSpanStateV1(line, tuple(rng.randrange(-9, 10) for _ in range(4)), number % 3, (number + 1) % 3)
            for number, line in enumerate((a, b, c, lines[3]))
        }
        self.vertex_count, self.span_count = 3, 4
        self.traces = {}


def test_the_edge_law_equals_the_oracle_on_meeting_views_in_every_branch():
    rng = random.Random(67)
    verifier = verifier_of()
    with verifier.installed():
        for _round in range(1500):
            budget = parts.random_budget(rng)
            built = MeetingView(rng, budget=budget, memo=rng.random() < 0.8) if rng.random() < 0.75 else parts.RandomView(rng, budget=budget, memo=rng.random() < 0.8)
            view = built.view()
            now = parts.small_time(rng) if rng.random() < 0.5 else et.ZERO_TIME
            vertex, peer = rng.randrange(built.vertex_count), rng.randrange(built.vertex_count)
            factory = (lambda: ("proof", vertex, peer)) if rng.random() < 0.5 else None
            same_vertex = rng.random() < 0.06
            try:
                for _repeat in range(2):
                    candidate_law.evaluate_edge_candidate(view, vertex, peer, now=now, same_vertex=same_vertex, proof_identity_factory=factory)
            except ORACLE_OUTCOMES:
                continue
    settle(verifier)
    assert verifier.checked["EVALUATE_EDGE_CANDIDATE"] > 2000
    reached = edge_outcomes(verifier)
    for reason in EDGE_REASONS:
        assert reached[reason] > 0, f"no random view ended in {reason}: {dict(reached)}"
    assert verifier.raised["EVALUATE_EDGE_CANDIDATE"] > 0, "no random view exhausted a budget or raised inside the edge law"


def test_the_pieces_of_the_exact_view_equal_the_oracle_on_random_views():
    rng = random.Random(71)
    verifier = verifier_of()
    seen = Counter()
    with verifier.installed():
        for _round in range(1200):
            budget = parts.random_budget(rng)
            built = MeetingView(rng, budget=budget, memo=rng.random() < 0.8) if rng.random() < 0.7 else parts.RandomView(rng, budget=budget, memo=rng.random() < 0.8)
            view = built.view()
            time_value = rng.choice((et.ZERO_TIME, parts.small_time(rng), et.EventTimeV1(Fraction(rng.randrange(-3, 6), rng.randrange(1, 4)), SqrtSumV1.rational(1))))
            now = rng.choice((et.ZERO_TIME, parts.small_time(rng)))
            vertex, peer = rng.randrange(built.vertex_count), rng.randrange(built.vertex_count)
            span = rng.randrange(built.span_count)
            calls = (
                lambda: view_module.edge_event_time(view, vertex, peer, now),
                lambda: view_module.is_future(view, time_value, vertex, peer, now=now),
                lambda: view_module.is_future(view, time_value, now=now),
                lambda: view_module.collapsing_span(view, vertex, peer, time_value),
                lambda: view_module.span_end(view, vertex, span, time_value, at_start=True),
                lambda: view_module.span_end(view, vertex, span, time_value, at_start=False),
            )
            for number, call in enumerate(calls):
                try:
                    call()
                    call()
                    seen[number] += 1
                except ORACLE_OUTCOMES:
                    seen["raised"] += 1
    settle(verifier)
    for seam in ("EDGE_EVENT_TIME", "IS_FUTURE", "COLLAPSING_SPAN", "SPAN_END"):
        assert verifier.checked[seam] > 1000, (seam, dict(verifier.checked))


def test_the_sliding_projection_equals_the_oracle():
    rng = random.Random(73)
    verifier = verifier_of()
    found = Counter()
    with verifier.installed():
        for _ in range(900):
            first = parts.random_line(rng, speed=rng.choice((1, 2, 4, Fraction(9, 4), 0, 5)))
            kind = rng.randrange(4)
            if kind == 0:
                second = et.SupportLineV1(first.a, first.b, first.c + rng.randrange(-4, 5), first.q)
            elif kind == 1:
                second = et.SupportLineV1(-first.a, -first.b, first.c, first.q)
            elif kind == 2:
                second = et.SupportLineV1(2 * first.a, 2 * first.b, first.c, first.q * 4)
            else:
                second = parts.random_line(rng)
            point = et.EventPointV1(parts.random_sum(rng), parts.random_sum(rng))
            found["projection" if view_module.sliding_projection(first, second, point) is not None else "none"] += 1
    settle(verifier)
    assert found["projection"] > 100 and found["none"] > 100, dict(found)


# --------------------------------------------------------------------------
# the poststate classification
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class SegmentRef:
    """A span reference of the symbolic layer: it carries the exact end points of its segment as `occurrence = (owner, start terms, end terms)`."""

    name: str
    occurrence: tuple | None


def terms_of(rng):
    return tuple(parts.random_sum(rng).terms)


class PoststateView:
    """Three spans (`low`, `shared`, `high`) and two vertices that share the middle one, with refs that may carry an occurrence."""

    def __init__(self, rng, *, budget, occurrences: bool = True, shared_source: tuple | None = None) -> None:
        occurrence = lambda: ("owner", (terms_of(rng), terms_of(rng)), (terms_of(rng), terms_of(rng))) if occurrences and rng.random() < 0.55 else None  # noqa: E731
        self.refs = {name: SegmentRef(name, occurrence()) for name in ("low", "shared", "high", "other")}
        sides = [parts.random_line(rng, span=12, speed=rng.choice((1, 2, 4, 0, Fraction(9, 4)))) for _ in range(3)]
        if rng.random() < 0.3:
            sides[2] = et.SupportLineV1(sides[0].a, sides[0].b, sides[0].c + rng.randrange(-2, 3), sides[0].q)
        shared = parts.random_line(rng, span=12, speed=rng.choice((0, 1, 2)))
        scale = lambda: tuple(rng.randrange(-6, 7) for _ in range(4))  # noqa: E731
        self.spans = {
            self.refs["low"]: view_module.CandidateSpanStateV1(sides[0], scale(), None, None),
            self.refs["shared"]: view_module.CandidateSpanStateV1(shared, shared_source or (scale() if rng.random() < 0.9 else (0, 0, 0, 0)), "low", "high"),
            self.refs["high"]: view_module.CandidateSpanStateV1(sides[1], scale(), None, None),
            self.refs["other"]: view_module.CandidateSpanStateV1(sides[2], scale(), None, None),
        }
        slide = lambda: SqrtSumV1.rational(rng.randrange(-5, 6)) if rng.random() < 0.25 else None  # noqa: E731
        birth = rng.choice((et.ZERO_TIME, parts.small_time(rng)))
        self.vertices = {
            "low": view_module.CandidateVertexStateV1(self.refs["low"], self.refs["shared"], birth, slide()),
            "high": view_module.CandidateVertexStateV1(self.refs["shared"] if rng.random() < 0.9 else self.refs["other"], self.refs["high"], birth, slide()),
        }
        self.budget = budget
        universe = rng.choice(((), (2, 3), (2, 3, 5, 7, 11, 13)))
        self.view = view_module.ExactCandidateViewV1(universe, self.vertices.__getitem__, self.spans.__getitem__, lambda ref, time: None, budget, view_module.PositionMemoV1(universe))


def test_the_poststate_classification_equals_the_oracle_in_every_disposition():
    rng = random.Random(79)
    verifier = verifier_of()
    found = Counter()
    with verifier.installed():
        for _round in range(1800):
            built = PoststateView(rng, budget=parts.random_budget(rng))
            birth_time = rng.choice((et.ZERO_TIME, parts.small_time(rng), et.EventTimeV1(Fraction(rng.randrange(1, 9), 2), SqrtSumV1.rational(1))))
            try:
                found[poststate_span.classify_poststate_span(built.view, "low", "high", birth_time).disposition.value] += 1
            except ORACLE_OUTCOMES:
                found["raised"] += 1
    settle(verifier)
    assert verifier.checked["CLASSIFY_POSTSTATE_SPAN"] == 1800
    for name in poststate_span.PoststateSpanDisposition:
        assert found[name.value] > 0, f"no random view was classified {name.value}: {dict(found)}"


@pytest.mark.parametrize("source", ((1, 2, 3, 4, 5), (1, 2, 3)))
def test_a_source_span_of_the_wrong_length_raises_the_interpreters_text_in_the_classification(source):
    rng = random.Random(83)
    verifier = verifier_of()
    raised = 0
    with verifier.installed():
        for _ in range(500):
            built = PoststateView(rng, budget=None, occurrences=False, shared_source=source)
            try:
                poststate_span.classify_poststate_span(built.view, "low", "high", rng.choice((et.ZERO_TIME, parts.small_time(rng))))
            except ValueError as error:
                assert "values to unpack" in str(error)
                raised += 1
            except ORACLE_OUTCOMES:
                pass
    settle(verifier)
    assert raised > 5 and verifier.raised["CLASSIFY_POSTSTATE_SPAN"] == raised


# --------------------------------------------------------------------------
# the proof ledger
# --------------------------------------------------------------------------


def random_keys(rng, count: int) -> tuple:
    return tuple(tuple(rng.randrange(-4, 9) for _ in range(rng.choice((4, 5)))) for _ in range(count))


def test_the_proof_ledger_equals_the_oracle_on_random_scripts():
    rng = random.Random(89)
    verifier = verifier_of()
    reasons = list(CandidateRefusal)
    statuses = Counter()
    with verifier.installed():
        for _round in range(260):
            ledger = proof.ProofLedger()
            for _step in range(rng.randrange(1, 25)):
                roll = rng.random()
                level = rng.choice((et.ZERO_TIME, parts.small_time(rng), parts.random_time(rng)))
                vertices = tuple(rng.randrange(0, 12) for _ in range(rng.randrange(0, 4)))
                if roll < 0.5:
                    ledger.record_refusal(rng.choice(reasons), vertex_ids=vertices, participant_edge_keys=random_keys(rng, rng.randrange(0, 3)), target_edge_keys=random_keys(rng, rng.randrange(0, 3)), level=level)
                elif roll < 0.75:
                    cause = rng.choice((*reasons, *proof.ProofObligationBranch))
                    ledger.record(
                        cause=cause,
                        disposition=rng.choice(list(proof.ProofObligationDisposition)),
                        vertex_ids=vertices,
                        participant_edge_keys=random_keys(rng, rng.randrange(0, 3)),
                        target_edge_keys=random_keys(rng, rng.randrange(0, 3)),
                        level=level,
                        event_kind=rng.choice((None, *events.EventKind)),
                    )
                elif roll < 0.9:
                    ledger.discharge(rng.sample(range(12), rng.randrange(0, 6)))
                else:
                    statuses[ledger.finalize(rng.sample(range(12), rng.randrange(0, 8)))[0].value] += 1
            statuses[ledger.finalize(rng.sample(range(12), rng.randrange(0, 12)))[0].value] += 1
    settle(verifier)
    assert verifier.checked["PROOF_SCRIPT"] == 260 and verifier.replayed["ledger operations"] > 2000
    assert statuses["COMPLETE"] > 5 and statuses["INCOMPLETE"] > 50, dict(statuses)


def test_a_ledger_of_a_real_run_is_the_oracles_in_status_and_obligations():
    verifier = verifier_of()
    for _name, polygon in parts.named_polygons():
        parts.run_with_leaf(verifier, polygon)
    settle(verifier)
    assert verifier.checked["PROOF_SCRIPT"] == len(parts.named_polygons()) and verifier.replayed["ledger operations"] > 500


# --------------------------------------------------------------------------
# the parts of the motorcycle module on their own
# --------------------------------------------------------------------------


def part_call(verifier, selector, oracle, arguments, decode, budget=None):
    return parts.attempt(verifier, "MOTORCYCLE_PART", oracle, lambda: [selector, *arguments()], decode, budget)


def test_the_bisector_velocity_and_the_bounds_equal_the_oracle():
    rng = random.Random(97)
    verifier = verifier_of()
    shown = Counter()
    for _ in range(500):
        budget = parts.random_budget(rng)
        left, right = parts.random_line(rng), parts.random_line(rng)
        if rng.random() < 0.15:
            right = et.SupportLineV1(left.a, left.b, left.c + 1, left.q)
        got = part_call(verifier, 0, lambda: motorcycle.bisector_velocity(left, right, budget), lambda: [wire.enc_line(left), wire.enc_line(right)], lambda value: None if value is None else tuple(value), budget)
        shown["none" if got is None else "velocity"] += 1
        value = parts.random_sum(rng)
        part_call(verifier, 1, lambda: motorcycle._upper_bound_of_root(value), lambda: [value], lambda found: found)
        time_value = rng.choice((parts.random_time(rng), parts.small_time(rng), et.EventTimeV1(Fraction(-rng.randrange(1, 5)), SqrtSumV1.rational(1))))
        part_call(verifier, 2, lambda: motorcycle._upper_bound_of_time(time_value), lambda: [wire.enc_time(time_value)], lambda found: found)
        points = tuple(et.EventPointV1(parts.random_sum(rng), parts.random_sum(rng)) for _ in range(rng.randrange(1, 4)))
        part_call(verifier, 3, lambda: motorcycle._point_box(points), lambda: [[wire.enc_point(point) for point in points]], lambda found: tuple(found))
    settle(verifier)
    assert shown["none"] > 20 and shown["velocity"] > 200, dict(shown)


def near_zero_time(digits: int):
    """`1 + (a*sqrt(2) - b*sqrt(3))` for the convergent `a/b` of `sqrt(3/2)` with `b` of about `digits` digits: a divisor within about 10^-digits of one cancelling, so the enclosure
    needs about 3.3 * digits bits before it separates it from zero (the doublings of the upper bound of a time)."""

    ratio = Fraction(math.isqrt(3 * 10 ** (2 * digits) // 2), 10**digits).limit_denominator(10**digits)
    difference = SqrtSumV1.radical(ratio.numerator, 2) - SqrtSumV1.radical(ratio.denominator, 3)
    return et.EventTimeV1.normalized(Fraction(1), difference if difference.sign() > 0 else -difference)


def test_the_upper_bound_of_a_time_doubles_the_enclosure_until_the_divisor_is_apart_from_zero():
    verifier = verifier_of()
    found = Counter()
    for digits in (3, 12, 30, 70, 160, 400):
        time_value = near_zero_time(digits)
        got = part_call(verifier, 2, lambda: motorcycle._upper_bound_of_time(time_value), lambda: [wire.enc_time(time_value)], lambda value: value)
        found["bound" if got is not None else "gave up"] += 1
    settle(verifier)
    assert found["bound"] >= 5, dict(found)


def test_the_speed_bound_the_walls_and_the_march_budget_equal_the_oracle():
    rng = random.Random(101)
    verifier = verifier_of()
    for _name, polygon in parts.named_polygons() + generated_polygons():
        part_call(verifier, 4, lambda: motorcycle.speed_bound_of(polygon), lambda: [wire.enc_polygon(polygon)], lambda found: found)
        part_call(verifier, 5, lambda: motorcycle.walls_of(polygon), lambda: [wire.enc_polygon(polygon)], lambda found: tuple(wire.dec_wall(wall) for wall in found))
    for _ in range(100):
        grid = random_grid(rng)
        part_call(verifier, 6, lambda: motorcycle.march_budget(grid), lambda: [wire.enc_grid(grid)], lambda found: found)
    settle(verifier)


def test_a_wall_hit_a_projection_a_reach_an_arrival_and_the_meeting_times_equal_the_oracle():
    rng = random.Random(103)
    verifier = verifier_of()
    shown = Counter()
    polygons = parts.named_polygons()[4:14] + generated_polygons()[::5]
    for _name, polygon in polygons:
        graph = motorcycle.build_motorcycle_graph(polygon, leaf.fresh_process_state())
        traces = list(graph.traces.values())
        for _ in range(25):
            budget = parts.random_budget(rng)
            graph.work_budget = budget
            if traces and rng.random() < 0.6:
                chosen = rng.choice(traces)
                left, right, start, origin = chosen.left_line, chosen.right_line, rng.choice((et.ZERO_TIME, parts.small_time(rng))), chosen.origin
            else:
                left, right, start, origin = born_lines(rng, graph, polygon)
            wall = rng.choice(graph.walls)
            got = part_call(
                verifier,
                9,
                lambda: graph._wall_hit(left, right, start, wall),
                lambda: [wire.enc_line(left), wire.enc_line(right), wire.enc_time(start), wire.enc_wall(wall)],
                lambda found: None if found is None else (wire.dec_time(found[0]), wire.dec_point(found[1]), found[2]),
                budget,
            )
            shown["wall hit" if got is not None else "no wall hit"] += 1
            point = et.EventPointV1(parts.random_sum(rng), parts.random_sum(rng))
            part_call(verifier, 7, lambda: motorcycle._projection_is_inside(point, wall, budget), lambda: [wire.enc_point(point), wire.enc_wall(wall)], lambda found: found, budget)
            if traces:
                first, second = rng.choice(traces), rng.choice(traces)
                offset = Fraction(rng.randrange(0, 40), rng.randrange(1, 5))
                crash = second.crash_point if second.crash_point is not None else point
                part_call(
                    verifier,
                    8,
                    lambda: motorcycle._reaches(first.origin, first.velocity, crash, offset, budget),
                    lambda: [wire.enc_point(first.origin), [first.velocity[0], first.velocity[1]], wire.enc_point(crash), offset],
                    lambda found: found,
                    budget,
                )
                part_call(verifier, 10, lambda: motorcycle._arrival(first, second, budget), lambda: [wire.enc_trace(first), wire.enc_trace(second)], lambda found: None if found is None else wire.dec_time(found), budget)
                meeting = part_call(
                    verifier,
                    11,
                    lambda: motorcycle._meeting_times(first, second, budget),
                    lambda: [wire.enc_trace(first), wire.enc_trace(second)],
                    lambda found: None if found is None else (wire.dec_time(found[0]), wire.dec_time(found[1])),
                    budget,
                )
                shown["meeting" if meeting is not None else "no meeting"] += 1
    settle(verifier)
    assert shown["wall hit"] > 20 and shown["no wall hit"] > 20 and shown["meeting"] > 5 and shown["no meeting"] > 5, dict(shown)


# --------------------------------------------------------------------------
# the kernel tests of the layer, with the checks inside them
# --------------------------------------------------------------------------

#: Kernel tests of this layer, run in a subprocess with `-p native_motorcycle_gate`: the whole kernel suite makes the calls (graphs of every polygon the tests build, the born
#: vertices, the poststate classes of the symbolic commit, the ledgers, the index, the cells), and the plugin checks them on the same state.
KERNEL_TESTS = (
    "test_wavefront_cell_grid.py",
    "test_wavefront_motorcycle_graph.py",
    "test_wavefront_poststate_span.py",
    "test_wavefront_proof_obligations.py",
)
#: A kernel test that replaces something the native side computes itself (the oracle's `march_budget`) is the one place a comparison cannot be exact; the host hands the live value of
#: that function over (`native_motorcycle_gate.march_steps_for`), so such a test is NOT excused: this list stays empty and a test enters it only with a named reason.
PATCHES_THE_ORACLE: dict = {}


def test_the_kernel_tests_of_the_layer_pass_with_every_call_checked_against_the_native_side(tmp_path):
    out = tmp_path / "gate.json"
    environment = dict(os.environ, **{gate.OUT_ENVIRONMENT: str(out), "PYTHONSAFEPATH": "1", "PYTHONPATH": os.pathsep.join(sys.path)})
    command = [sys.executable, "-m", "pytest", *(str(ROOT / "kernel" / "tests" / name) for name in KERNEL_TESTS), "-p", "native_motorcycle_gate", "-q", "-p", "no:cacheprovider", "-x"]
    finished = subprocess.run(command, cwd=ROOT, env=environment, capture_output=True, text=True, timeout=1500)
    assert finished.returncode == 0, finished.stdout[-3000:] + finished.stderr[-2000:]
    summary = json.loads(out.read_text(encoding="utf8"))
    unexcused = [text for text in summary["mismatches"] if not any(name in text for name in PATCHES_THE_ORACLE)]
    assert not unexcused, "\n".join(unexcused[:8])
    assert not summary["unsupported"], summary["unsupported"]
    for seam, at_least in (("BUILD_MOTORCYCLE_GRAPH", 600), ("TRACE_INDEX_SCRIPT", 600), ("PROOF_SCRIPT", 300), ("CLASSIFY_POSTSTATE_SPAN", 20), ("EVALUATE_EDGE_CANDIDATE", 500), ("TRACE_FOR", 20)):
        assert summary["checked"].get(seam, 0) >= at_least, (seam, summary["checked"])
    for seam, count in summary["checked"].items():
        CHECKED[f"kernel tests: {seam}"] += count


# --------------------------------------------------------------------------
# the harness is not vacuous
# --------------------------------------------------------------------------


def test_the_graph_comparison_reports_a_wrong_trace_a_wrong_counter_and_a_cost_the_native_side_does_not_pay():
    verifier = verifier_of()
    polygon = dict(parts.named_polygons())["comb_4"]

    def wrong_crash_point():
        graph = motorcycle.build_motorcycle_graph(polygon, None)
        first = next(iter(graph.traces))
        trace = graph.traces[first]
        moved = et.EventPointV1(trace.crash_point.x + SqrtSumV1.rational(1), trace.crash_point.y)
        graph.traces[first] = motorcycle.TraceV1(**{**{name: getattr(trace, name) for name in trace.__slots__}, "crash_point": moved})
        return graph

    verifier.lockstep("BUILD_MOTORCYCLE_GRAPH", wrong_crash_point, lambda: [wire.enc_polygon(polygon), None], wire.dec_graph, budget=None, normalize=wire.graph_view)
    assert [item.field for item in verifier.mismatches] == ["result"]
    verifier.mismatches.clear()

    def wrong_counter():
        graph = motorcycle.build_motorcycle_graph(polygon, None)
        graph.counters["motorcycle_wall_tests"] += 1
        return graph

    verifier.lockstep("BUILD_MOTORCYCLE_GRAPH", wrong_counter, lambda: [wire.enc_polygon(polygon), None], wire.dec_graph, budget=None, normalize=wire.graph_view)
    assert [item.field for item in verifier.mismatches] == ["result"]
    verifier.mismatches.clear()

    def one_sign_too_many():
        graph = motorcycle.build_motorcycle_graph(polygon, None)
        exact.SIGN_COUNTS["total"] += 1
        return graph

    verifier.lockstep("BUILD_MOTORCYCLE_GRAPH", one_sign_too_many, lambda: [wire.enc_polygon(polygon), None], wire.dec_graph, budget=None, normalize=wire.graph_view)
    assert [item.field for item in verifier.mismatches] == ["sign_counts"]


def test_the_script_comparison_reports_an_answer_the_native_side_does_not_give():
    verifier = verifier_of()
    polygon = dict(parts.named_polygons())["ell"]
    with verifier.installed():
        graph = motorcycle.build_motorcycle_graph(polygon, leaf.fresh_process_state())
        index = motorcycle.TraceCandidateIndexV1.covering(polygon, graph)
        index.register_line(0, et.SupportLineV1.through((0, 0), (12, 0)))
        index.vertices_near(0)
    index_script = next(iter(verifier.indexes.values()))
    index_script.results[-1] = [99]
    verifier.finish()
    assert [item.seam for item in verifier.mismatches] == ["TRACE_INDEX_SCRIPT"]
