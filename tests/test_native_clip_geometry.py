"""Нативная `clip_geometry` ЦЕЛИКОМ (`native/cftuv-clip`: стадия, резка, выпуск) равна эталону на Python побитово.

Эталон — ядро `kernel/src/cftuv_envelope/materialize/clip.py` этого же интерпретатора (живой вызов, а не запись): оба пути стартуют
с одного восстановленного состояния ДО (`native_corpus.restore_state`), нативный получает те же входы через шов `CLIP_GEOMETRY` (это тестовый
вход: быстрая граница с готовыми объектами — следующий шаг). Сравнивается `nc.compare_outcomes`: результат каноническим кодом (`int` и `Fraction`
различны, `float` по `hex`), исключение `(класс, текст)`, шесть статей бюджета, `SIGN_COUNTS`, четыре таблицы памяти канонизации с порядком,
запись нормалей смещения в `plane._normal_by_position` (в том числе после отказа). `tools/native_clip_geometry.py` — сама сверка.

Источники: 117 полевых вызовов (Blender), 22 производных с урезанным бюджетом, 413 синтетических (тесты ядра и генератор:
`by_faces=False`, все законы, отказы, исчерпание, переполнение, коэффициенты `int`), свип потолка бюджета на тяжёлых записях и свежий
сгенерированный набор (другое зерно, чем у корпуса). Тест запускается под ЛЮБЫМ интерпретатором, на котором собрано расширение: сверка идёт с
им же (3.13 в dev-venv; Blender 4.5 = 3.11: `PYTHONPATH=~/.cftuv-native/py311-site <blender python> -m pytest ...`).

Отрицательный контроль: испорченный нативный исход (порядок граней, счётчик, запись памяти, нормаль) сравнение ловит названным полем. Версия
интерпретатора на ответ не влияет: сортировка `_ordered` и свёртка нормали — явная семантика CPython 3.11 ядра (`_cpython311.py`), в том числе для 64 узлов и больше.

Модуль пропускается с названной причиной, пока расширение не собрано (`python tools/native_build.py`) либо нет корпуса.
"""

from __future__ import annotations

import dataclasses
import math
import os
import sys
from collections import Counter
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "kernel" / "src", ROOT / "kernel" / "tests", ROOT / "tools"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

try:
    import cftuv_native  # noqa: F401
except ModuleNotFoundError as error:
    if error.name != "cftuv_native":
        raise
    pytest.skip(
        "расширение cftuv_native не собрано: `python tools/native_build.py` ставит его в dev-venv (сверка целой clip_geometry с эталоном пропущена)",
        allow_module_level=True,
    )

from native_gate import skip_unless_available  # noqa: E402

skip_unless_available(cftuv_native, "clip")

import native_clip_fuzz as fuzz  # noqa: E402
import native_clip_geometry as geometry  # noqa: E402
import native_corpus as nc  # noqa: E402

import cftuv_envelope.exact_sqrt_sum as exact  # noqa: E402

PYTHON_VERSION = (sys.version_info.major, sys.version_info.minor)
CORPUS = geometry.corpus_base()
HAS_CORPUS = CORPUS.exists()
#: Каждый какой по счёту полевой вызов сверяется (1 — все).
RECORD_STRIDE = int(os.environ.get("CFTUV_CLIP_SEAM_STRIDE", "1"))

CHECKED: Counter = Counter()
TIMINGS: dict = {}


@pytest.fixture(autouse=True)
def _kernel_process_state_is_given_back():
    """Эталон пишет в процессные счётчики и память ядра; тест их не оставляет."""

    counts = dict(exact.SIGN_COUNTS)
    unbudgeted = exact.UNBUDGETED_WORK.spent_by_article()
    with exact.isolated_factorization_memory():
        yield
    exact.SIGN_COUNTS.update(counts)
    for name, value in zip(nc._ARTICLES, unbudgeted):
        setattr(exact.UNBUDGETED_WORK, name, value)


@pytest.fixture(scope="module")
def runner():
    return geometry.WholeRunner()


@pytest.fixture(scope="module", autouse=True)
def _report_the_comparison_count(request):
    yield
    reporter = request.config.pluginmanager.get_plugin("terminalreporter")
    if reporter is None:
        return
    reporter.write_line(f"native clip_geometry compared with the Python oracle (python {sys.version.split()[0]}): " + ", ".join(f"{key} {value}" for key, value in sorted(CHECKED.items())))
    for source, rows in TIMINGS.items():
        reporter.write_line(f"-- {source}")
        reporter.write_line(geometry.timing_table(rows))


def compare_paths(runner, paths, source: str) -> list:
    """`[(имя, Run)]` по записям; в `CHECKED` — сколько сверено, в `TIMINGS` — секунды эталона и нативного compute."""

    runs = []
    rows = TIMINGS.setdefault(source, [])
    for path in paths:
        record = nc.read_record(path)
        run = runner.compare(record)
        runs.append((f"{path.parent.name}/{path.name}", run))
        CHECKED[source] += 1
        if run.native_compute_seconds:
            rows.append((geometry.group_of(record) if source == "field" else source, run.oracle_seconds, run.native_compute_seconds))
            rows.append(("all", run.oracle_seconds, run.native_compute_seconds))
    return runs


def assert_all_equal(runs: list) -> None:
    failed = [(name, run) for name, run in runs if not run.equal]
    assert not failed, geometry.explain(failed)


# --------------------------------------------------------------------------
# Корпуса
# --------------------------------------------------------------------------


@pytest.mark.skipif(not HAS_CORPUS, reason=f"нет полевого корпуса {CORPUS}: `tools/native_corpus_export.py`")
def test_every_field_record_equals_the_oracle(runner):
    paths = geometry.field_paths(RECORD_STRIDE)
    assert len(paths) >= 100 // RECORD_STRIDE
    runs = compare_paths(runner, paths, "field")
    assert_all_equal(runs)
    assert Counter(run.outcome_label for _name, run in runs) == {"CLIPPED": len(runs)}, "the field corpus holds successful cuts only"


@pytest.mark.skipif(not HAS_CORPUS, reason=f"нет полевого корпуса {CORPUS}: `tools/native_corpus_export.py`")
def test_every_derived_starved_budget_record_equals_the_oracle(runner):
    paths = geometry.derived_paths()
    assert len(paths) >= 20
    runs = compare_paths(runner, paths, "derived")
    assert_all_equal(runs)


@pytest.mark.skipif(not HAS_CORPUS or not geometry.synthetic_paths(), reason=f"нет синтетического корпуса {CORPUS / 'synthetic_clip'}: `python tools/native_clip_synthetic.py build`")
def test_every_synthetic_record_equals_the_oracle(runner):
    paths = geometry.synthetic_paths()
    assert len(paths) >= 400
    runs = compare_paths(runner, paths, "synthetic")
    assert_all_equal(runs)
    labels = Counter(run.outcome_label for _name, run in runs)
    # иначе «зелёный» пуст: корпус обязан нести отказы, исчерпание, переполнение
    assert labels["CLIPPED"] > 300 and labels["raised:MaterializationRefusal"] >= 50, labels
    assert labels["raised:ExactCanonicalizationWorkBudgetExhausted"] >= 5 and labels["raised:OverflowError"] >= 1, labels


# --------------------------------------------------------------------------
# Потолок бюджета: исчерпание на каждой границе тяжёлых записей
# --------------------------------------------------------------------------


@pytest.mark.skipif(not HAS_CORPUS, reason=f"нет полевого корпуса {CORPUS}")
def test_a_cap_sweep_on_heavy_records_exhausts_at_the_same_place_with_the_same_partial_state(runner):
    refused = Counter()
    problems = []
    for path in geometry.heavy_paths(4):
        record = nc.read_record(path)
        before = record.before()
        spent = sum(before.budget["articles"])
        delta = sum(record.expected().after.budget["articles"]) - spent
        for level in geometry.cap_levels(delta):
            state = dataclasses.replace(before, budget={**before.budget, "cap": spent + level})
            run = runner.compare(record, before=state)
            CHECKED["cap sweep"] += 1
            refused[run.outcome_label] += 1
            if not run.equal:
                problems.append((f"{path.name} cap +{level}", run))
    assert not problems, geometry.explain(problems)
    assert refused["raised:ExactCanonicalizationWorkBudgetExhausted"] >= 20, refused
    assert refused["CLIPPED"] >= 4, refused


# --------------------------------------------------------------------------
# Свежий сгенерированный набор (зерно не корпуса)
# --------------------------------------------------------------------------


def test_fresh_generated_calls_equal_the_oracle(runner):
    runs = []
    for label, lift, kwargs, cap in geometry.fresh_calls(20261007, 360):
        runs.append((label, geometry.compare_generated(runner, label, lift, kwargs, cap)))
        CHECKED["fresh generated"] += 1
    assert_all_equal(runs)
    labels = Counter(run.outcome_label for _name, run in runs)
    assert labels["CLIPPED"] > 150, labels
    assert len(labels) >= 3, f"the fresh set must reach refusals too: {labels}"


def test_fuzzed_strips_of_faces_with_shared_vertices_equal_the_oracle(runner):
    """Ленты граней с общими вершинами, швами, веерами и всеми законами по решётчатым планам (`tools/native_clip_fuzz.py`)."""

    runs = []
    for seed in (101, 102, 103):
        for label, lift, kwargs, cap in fuzz.fuzz_cases(seed, 300):
            runs.append((label, geometry.compare_generated(runner, label, lift, kwargs, cap)))
            CHECKED["fuzzed strips"] += 1
    assert_all_equal(runs)
    labels = Counter(run.outcome_label for _name, run in runs)
    assert labels["CLIPPED"] > 400 and labels["raised:MaterializationRefusal"] > 30, labels
    assert labels["raised:ExactCanonicalizationWorkBudgetExhausted"] >= 10, labels
    assert sum(1 for _name, run in runs if run.expected.result is not None and run.expected.result.points) > 100, "the strips must cut and create vertices"


def test_the_special_calls_of_the_branches_no_random_input_reaches_equal_the_oracle(runner):
    """Переполнение рационального знака, слишком малая иррациональная координата, нуль нормали, лишние ключи, обрезка вееров и потоков."""

    runs = [(label, geometry.compare_generated(runner, label, lift, kwargs, cap)) for label, lift, kwargs, cap in fuzz.special_cases()]
    CHECKED["special calls"] += len(runs)
    assert_all_equal(runs)
    exceptions = {name: run.expected.exception for name, run in runs}
    assert exceptions["opposed-normals-at-the-midpoint"][1].startswith("SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE: "), exceptions
    assert exceptions["a-polygon-key-no-vertex-has"] == ("KeyError", "'node:9'")
    assert exceptions["a-cycle-key-no-vertex-has"] == ("KeyError", "'node:8'")
    assert exceptions["a-seam-key-no-vertex-has"] == ("KeyError", "'node:7'")
    assert exceptions["opposed-normals-at-the-midpoint"] and runs[4][1].expected.exception, "the oracle fails at the zero normal"


# --------------------------------------------------------------------------
# Названные границы порта
# --------------------------------------------------------------------------


def long_sort_case():
    """`(lift, kwargs)`: тонкая полоса через 70 колонок треугольников; её длинные рёбра пересекают 69 вертикальных и 70 диагональных рёбер."""

    from fractions import Fraction

    from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
    from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
    from cftuv_envelope.materialize.lift_surface import SurfaceLiftV1

    columns, width, height = 70, 10, 40
    items = []
    for column in range(columns):
        x0, x1 = column * width, (column + 1) * width
        for name, chart in ((f"a{column}", ((x0, 0), (x1, 0), (x1, height))), (f"b{column}", ((x0, 0), (x1, height), (x0, height)))):
            corners = tuple((Fraction(x, 4), Fraction(y, 4), Fraction(0)) for x, y in chart)
            items.append((name, chart, corners, (), ""))
    lift = SurfaceLiftV1.from_triangles(items, scale=4)

    def point(x, y):
        return SqrtSumV1.rational(Fraction(x)), SqrtSumV1.rational(Fraction(y))

    def skew(x, y, root):
        """A corner whose ordinates carry a square root: every crossing of the long edges is irrational, so each comparison of the sort pays exact work."""

        return SqrtSumV1.rational(Fraction(x)) + SqrtSumV1.radical(Fraction(1), root), SqrtSumV1.rational(Fraction(y)) + SqrtSumV1.radical(Fraction(1, 2), root + 1)

    edge = columns * width - 5
    cases = {
        "rational": [point(5, 20), point(edge, 20), point(edge, 26), point(5, 26)],
        "irrational": [skew(5, 20, 2), skew(edge, 21, 3), skew(edge, 27, 5), skew(5, 26, 7)],
    }
    result = {}
    for label, vertices in cases.items():
        keys = [f"node:{index}" for index in range(4)]
        points = dict(zip(keys, vertices))
        result[label] = {
            "points": points,
            "cycles": [[(key, points[key]) for key in keys]],
            "polygons": [(tuple(keys),)],
            "law": DecalTopologyLawV1.PLANAR_POLYGONS_V1,
            "seam": frozenset(),
            "fans": None,
            "flows": None,
            "by_faces": False,
        }
    return lift, result


def recorded_sorts(monkeypatch):
    """Sizes of every `_ordered` sort the oracle runs from now on (the explicit CPython 3.11 sort of the kernel)."""

    import cftuv_envelope.materialize.clip as clip_module

    sizes: list = []
    original = clip_module.sorted_as_cpython311

    def recording(items, compare):
        items = list(items)
        sizes.append(len(items))
        return original(items, compare)

    monkeypatch.setattr(clip_module, "sorted_as_cpython311", recording)
    return sizes


def test_a_sort_of_sixty_four_nodes_or_more_equals_the_oracle_answer_and_cost(runner, monkeypatch):
    """`_ordered` с 64 узлами и больше идёт слияниями и галопом `list.sort`: порт теперь повторяет их, ответ и цена равны эталону."""

    lift, cases = long_sort_case()
    sizes = recorded_sorts(monkeypatch)
    for label, kwargs in cases.items():
        run = geometry.compare_generated(runner, f"long-sort-{label}", lift, kwargs, None)
        assert not run.unsupported, run.unsupported
        assert run.expected.exception is None, "the oracle itself cuts the polygon"
        assert run.equal, geometry.explain([(label, run)])
        CHECKED["long sort"] += 1
    assert max(sizes) >= 64, f"no sort of 64 nodes or more was asked: {sizes}"


def test_the_exhaustion_inside_a_long_sort_is_the_same_point_of_the_same_comparison(runner, monkeypatch):
    """Свип потолка бюджета на целой `clip_geometry` с сортировкой 64+ узлов: исчерпание в любой точке — то же, с теми же частичными статьями и памятью
    (исчерпание ВНУТРИ слияний и галопа сортировки — `tests/test_native_clip_parts.py`, шов `ORDERED` на узлах, чьи сравнения платят бюджет)."""

    lift, cases = long_sort_case()
    kwargs = cases["irrational"]
    sizes = recorded_sorts(monkeypatch)
    full = geometry.compare_generated(runner, "long-sort-total", lift, kwargs, None)
    assert full.equal and full.expected.exception is None
    spent = sum(full.expected.after.budget["articles"])
    caps = sorted({0, 1, 2, 3, 5, 8, 13, 21, 34, 55, 89, 144, 233, 377, 610, 987, 1597, 2584, 4181, 6765, 10946})
    exhausted = 0
    for cap in caps:
        run = geometry.compare_generated(runner, f"long-sort-cap{cap}", lift, kwargs, cap)
        assert not run.unsupported, run.unsupported
        assert run.equal, geometry.explain([(f"cap {cap}", run)])
        exhausted += run.expected.exception is not None
        CHECKED["long sort caps"] += 1
    assert max(sizes) >= 64
    assert exhausted >= 2, f"the sweep never exhausted the budget (spent {spent}): the caps do not reach the sort"


# --------------------------------------------------------------------------
# Отрицательные контроли: сравнение видит каждую часть исхода
# --------------------------------------------------------------------------


def heavy_outcomes(runner):
    """`(состояние до, исход эталона, нативный исход)` самой тяжёлой полевой записи: в ней есть и новые вершины, и подъём, и запись памяти."""

    record = nc.read_record(geometry.heavy_paths(1)[0])
    before = record.before()
    call = nc.prepare_call(nc.OP_CLIP, record.call_blob, before)
    expected = nc.execute(call)
    actual, _answer, _total = runner.native(record.call_blob, before)
    return before, expected, actual


def spoilers(actual: "nc.Outcome") -> dict:
    """`{название: (испорченный нативный исход, поле, которое сравнение обязано назвать)}`: каждая часть исхода портится по-своему."""

    result, after = actual.result, actual.after

    def with_result(**changes):
        return dataclasses.replace(actual, result=dataclasses.replace(result, **changes))

    def with_after(**changes):
        return dataclasses.replace(actual, after=dataclasses.replace(after, **changes))

    rotated = [tuple(tuple(keys[1:] + keys[:1]) if index == 0 else keys for index, keys in enumerate(face)) for face in result.polygons[:1]] + result.polygons[1:]
    counters = tuple((name, value + (1 if index == 14 else 0)) for index, (name, value) in enumerate(result.counters))
    articles = list(after.budget["articles"])
    articles[4] += 1
    counts = dict(after.sign_counts)
    counts["total"] += 1
    return {
        "face keys": (with_result(polygons=rotated), "result.polygons"),
        "a counter": (with_result(counters=counters), "result.counters"),
        "the note": (with_result(note=result.note + " "), "result.note"),
        "the order of the new points": (with_result(points=dict(reversed(list(result.points.items())))), "result.points"),
        "a lifted position": (with_result(lifted=dict(list(result.lifted.items())[1:])), "result.lifted"),
        "a contour": (with_result(cycles=[list(reversed(cycle)) for cycle in result.cycles]), "result.cycles"),
        "the snapped points": (with_result(snapped={"src:0": result.points[next(iter(result.points))]}), "result.snapped"),
        "the memory": (with_after(factorization=list(after.factorization[:-1])), "memory.factorization"),
        "the order of the memory": (with_after(squarefree=list(reversed(after.squarefree))), "memory.squarefree"),
        "the budget": (with_after(budget={**after.budget, "articles": tuple(articles)}), "budget.delta"),
        "the sign counters": (with_after(sign_counts=counts), "sign_counts.total"),
        "the offset normals": (dataclasses.replace(actual, observed={"plane_normals": "x"}), "observed.plane_normals"),
        "the exception": (dataclasses.replace(actual, exception=("KeyError", "'x'")), "exception"),
    }


@pytest.mark.skipif(not HAS_CORPUS, reason=f"нет полевого корпуса {CORPUS}")
def test_the_comparison_names_every_part_of_a_spoiled_native_outcome(runner):
    """Отрицательный контроль самого сравнения: исход без порчи равен, исход с порчей любой части назван её полем."""

    before, expected, actual = heavy_outcomes(runner)
    assert nc.compare_outcomes(nc.OP_CLIP, before, expected, actual) == []
    assert len(actual.result.points) >= 2 and len(actual.result.lifted) >= 2 and len(actual.after.factorization) >= 1 and len(actual.after.squarefree) >= 2
    for name, (spoiled, field) in spoilers(actual).items():
        named = {item.field for item in nc.compare_outcomes(nc.OP_CLIP, before, expected, spoiled)}
        assert field in named, f"spoiling {name} must be named {field}, got {sorted(named)}"


def test_the_seam_table_of_the_extension_is_the_table_of_the_harness():
    from cftuv_native import clip_seams as wire

    assert tuple(cftuv_native.clip_seam_table()) == wire.SEAMS
    assert math.isfinite(1.0)
