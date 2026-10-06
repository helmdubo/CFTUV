"""Нативная `clip_geometry` как ВСТАВКА (`cftuv_native.clip_geometry`) равна эталону на Python: тот же вызов, те же побочные эффекты.

Вставка — граница, какой её увидит сеанс Python за переключателем бэкенда и внутри `clip_memo.run_clip` вместо `compute`: сигнатура
`clip.clip_geometry(plane, budget, *, points, cycles, polygons, law, seam, fans, flows, by_faces)`, результат — настоящий `ClippedV1`, а
всё остальное — настоящие объекты процесса: статьи `ExactWorkBudgetV1`, `SIGN_COUNTS`, `UNBUDGETED_WORK`, четыре таблицы памяти канонизации
(порядок вставки входит), `plane._normal_by_position` (в том числе записи до отказа), настоящие исключения. Эталон — ядро этого же
интерпретатора, живой вызов; оба пути стартуют с одного восстановленного состояния ДО (`native_corpus.restore_state`), сравнение — `nc.compare_outcomes`
(канонический код результата: `int` и `Fraction` различны, `float` по `hex`; исключение `(класс, текст)`; цена; память с порядком; нормали плоскости).

Источники: 117 полевых вызовов, 22 производных с урезанным потолком, 413 синтетических, свип потолка бюджета, свежие сгенерированные наборы и полосы
граней (другое зерно, чем у корпуса). Один `mirror` живёт между вызовами (кэш плоскостей, снимки таблиц памяти), как в сеансе; отдельные тесты — повтор на
ОДНОЙ плоскости (тёплый кэш), интеграция с `clip_memo.run_clip` (промах и попадание), тождество переиспользованных объектов, настоящие классы исключений,
вызов без бюджета, названные отказы входа и отрицательные контроли (испорченная вставка названа полем).

Модуль пропускается с названной причиной, пока расширение не собрано либо нативная `clip` не сверена с этим деревом ядра (`native_gate`).
"""

from __future__ import annotations

import dataclasses
import os
import pickle
import sys
from collections import Counter
from fractions import Fraction
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
        "расширение cftuv_native не собрано: `python tools/native_build.py` ставит его в dev-venv (сверка вставки clip_geometry с эталоном пропущена)",
        allow_module_level=True,
    )

from native_gate import skip_unless_available  # noqa: E402

skip_unless_available(cftuv_native, "clip")

import native_clip_fuzz as fuzz  # noqa: E402
import native_clip_geometry as geometry  # noqa: E402
import native_corpus as nc  # noqa: E402

import cftuv_envelope.exact_sqrt_sum as exact  # noqa: E402
import cftuv_envelope.materialize.clip as clip  # noqa: E402
import cftuv_envelope.materialize.clip_memo as clip_memo  # noqa: E402

CORPUS = geometry.corpus_base()
HAS_CORPUS = CORPUS.exists()
needs_corpus = pytest.mark.skipif(not HAS_CORPUS, reason=f"нет полевого корпуса {CORPUS}: `tools/native_corpus_export.py`")
RECORD_STRIDE = int(os.environ.get("CFTUV_CLIP_SEAM_STRIDE", "1"))

CHECKED: Counter = Counter()


@pytest.fixture(autouse=True)
def _kernel_process_state_is_given_back():
    """Эталон и вставка пишут в процессные счётчики и память ядра; тест их не оставляет."""

    counts = dict(exact.SIGN_COUNTS)
    unbudgeted = exact.UNBUDGETED_WORK.spent_by_article()
    with exact.isolated_factorization_memory():
        yield
    exact.SIGN_COUNTS.update(counts)
    for name, value in zip(nc._ARTICLES, unbudgeted):
        setattr(exact.UNBUDGETED_WORK, name, value)


@pytest.fixture(scope="module")
def runner():
    """Один сеанс на весь модуль: кэш плоскостей и снимки таблиц переживают вызовы."""

    return geometry.DropinRunner()


@pytest.fixture(scope="module", autouse=True)
def _report_the_comparison_count(request):
    yield
    reporter = request.config.pluginmanager.get_plugin("terminalreporter")
    if reporter is not None:
        reporter.write_line(
            f"native clip_geometry DROP-IN compared with the Python oracle (python {sys.version.split()[0]}): " + ", ".join(f"{key} {value}" for key, value in sorted(CHECKED.items()))
        )


def compare_paths(runner, paths, source: str) -> list:
    runs = []
    for path in paths:
        run = runner.compare(nc.read_record(path))
        runs.append((f"{path.parent.name}/{path.name}", run))
        CHECKED[source] += 1
    return runs


def assert_all_equal(runs: list) -> None:
    failed = [(name, run) for name, run in runs if not run.equal]
    assert not failed, geometry.explain(failed)


# --------------------------------------------------------------------------
# Корпуса через вставку
# --------------------------------------------------------------------------


@needs_corpus
def test_every_field_record_through_the_dropin_equals_the_oracle(runner):
    runs = compare_paths(runner, geometry.field_paths(RECORD_STRIDE), "field")
    assert len(runs) >= 100 // RECORD_STRIDE
    assert_all_equal(runs)
    assert Counter(run.outcome_label for _name, run in runs) == {"CLIPPED": len(runs)}


@needs_corpus
def test_every_derived_starved_budget_record_through_the_dropin_equals_the_oracle(runner):
    paths = geometry.derived_paths()
    assert len(paths) >= 20
    assert_all_equal(compare_paths(runner, paths, "derived"))


@pytest.mark.skipif(not HAS_CORPUS or not geometry.synthetic_paths(), reason=f"нет синтетического корпуса {CORPUS / 'synthetic_clip'}: `python tools/native_clip_synthetic.py build`")
def test_every_synthetic_record_through_the_dropin_equals_the_oracle(runner):
    runs = compare_paths(runner, geometry.synthetic_paths(), "synthetic")
    assert len(runs) >= 400
    assert_all_equal(runs)
    labels = Counter(run.outcome_label for _name, run in runs)
    assert labels["CLIPPED"] > 300 and labels["raised:MaterializationRefusal"] >= 50, labels
    assert labels["raised:ExactCanonicalizationWorkBudgetExhausted"] >= 5 and labels["raised:OverflowError"] >= 1, labels


@needs_corpus
def test_a_cap_sweep_through_the_dropin_exhausts_at_the_same_place_with_the_same_partial_state(runner):
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
    assert refused["raised:ExactCanonicalizationWorkBudgetExhausted"] >= 20 and refused["CLIPPED"] >= 4, refused


def test_fresh_generated_calls_through_the_dropin_equal_the_oracle(runner):
    runs = []
    for label, lift, kwargs, cap in geometry.fresh_calls(20261007, 360):
        runs.append((label, geometry.compare_generated(runner, label, lift, kwargs, cap)))
        CHECKED["fresh generated"] += 1
    assert_all_equal(runs)
    labels = Counter(run.outcome_label for _name, run in runs)
    assert labels["CLIPPED"] > 150 and len(labels) >= 3, labels


def test_fuzzed_strips_through_the_dropin_equal_the_oracle(runner):
    runs = []
    for seed in (101, 102, 103):
        for label, lift, kwargs, cap in fuzz.fuzz_cases(seed, 300):
            runs.append((label, geometry.compare_generated(runner, label, lift, kwargs, cap)))
            CHECKED["fuzzed strips"] += 1
    assert_all_equal(runs)
    labels = Counter(run.outcome_label for _name, run in runs)
    assert labels["CLIPPED"] > 400 and labels["raised:MaterializationRefusal"] > 30 and labels["raised:ExactCanonicalizationWorkBudgetExhausted"] >= 10, labels


def test_the_special_calls_through_the_dropin_equal_the_oracle(runner):
    runs = [(label, geometry.compare_generated(runner, label, lift, kwargs, cap)) for label, lift, kwargs, cap in fuzz.special_cases()]
    CHECKED["special calls"] += len(runs)
    assert_all_equal(runs)
    exceptions = {name: run.expected.exception for name, run in runs}
    assert exceptions["a-polygon-key-no-vertex-has"] == ("KeyError", "'node:9'")
    assert exceptions["opposed-normals-at-the-midpoint"][1].startswith("SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE: ")


# --------------------------------------------------------------------------
# План станций цепей (`inert`): рёбра между гранями, которые резка не режет
# --------------------------------------------------------------------------


def _lift_of_faces(faces):
    """Подъём по граням `[(имя, карта, 3D углов, ((i, j, k), ...)), ...]`: треугольники грани — по индексам углов."""

    from cftuv_envelope.materialize.lift_surface import SurfaceLiftV1

    items = []
    for name, chart, corners, triangles in faces:
        for number, (i, j, k) in enumerate(triangles):
            items.append((f"{name}.t{number}", (chart[i], chart[j], chart[k]), tuple(tuple(Fraction(axis) for axis in corners[index]) for index in (i, j, k)), (), name))
    return SurfaceLiftV1.from_triangles(items, scale=4)


def _square_face(number: int, heights):
    """Квадрат `f<number>` со стороной 4 в полосе по оси x, высоты углов по обходу, диагональ 0-2."""

    chart = [(4 * number, 0), (4 * number + 4, 0), (4 * number + 4, 4), (4 * number, 4)]
    return (f"f{number}", chart, [(x, y, Fraction(h)) for (x, y), h in zip(chart, heights)], ((0, 1, 2), (0, 2, 3)))


def _strip_of_squares(*heights_by_face, extra_faces=()):
    return _lift_of_faces([*(_square_face(number, heights) for number, heights in enumerate(heights_by_face)), *extra_faces])


def _plan_call(polygons_xy, inert, **extra) -> dict:
    from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
    from cftuv_envelope.exact_sqrt_sum import SqrtSumV1

    keys, points, cycles, polygons = {}, {}, [], []
    for polygon in polygons_xy:
        cycle = []
        for xy in polygon:
            if xy not in keys:
                keys[xy] = f"p{len(keys)}"
                points[keys[xy]] = (SqrtSumV1.rational(Fraction(xy[0])), SqrtSumV1.rational(Fraction(xy[1])))
            cycle.append((keys[xy], points[keys[xy]]))
        cycles.append(cycle)
        polygons.append((tuple(key for key, _point in cycle),))
    call = {
        "points": points, "cycles": cycles, "polygons": polygons, "law": DecalTopologyLawV1.PLANAR_POLYGONS_V1, "seam": frozenset(),
        "fans": [False] * len(polygons), "flows": None, "by_faces": True, "inert": inert,
    }
    call.update(extra)
    return call


def _pairs(*pairs) -> frozenset:
    return frozenset(frozenset(pair) for pair in pairs)


def test_the_chain_station_plan_through_the_dropin_equals_the_oracle(runner):
    """Сценарии `kernel/tests/test_clip_plan_inert.py` и их соседи: склейка двух и трёх граней, невыпуклая грань в группе (оценка группы, расщепление), чужая грань."""

    flat, bent = (0, 0, 0, 0), (0, 0, 0, Fraction(1, 20))
    across = [[(3, 1), (5, 1), (5, 2), (3, 2)]]
    wide = [[(3, 1), (9, 1), (9, 2), (3, 2)]]
    inside_one = [[(1, 1), (3, 1), (3, 2)]]
    on_the_diagonal = [[(3, 1), (9, 3), (6, 3)], [(1, 1), (7, 2), (9, 1), (5, 3)]]
    ell_chart = [(0, 0), (8, 0), (8, 4), (4, 4), (4, 8), (0, 8)]
    ell = ("L", ell_chart, [(x, y, Fraction(h)) for (x, y), h in zip(ell_chart, (0, 0, Fraction(1, 5), 0, 0, 0))], ((0, 1, 2), (0, 2, 3), (0, 3, 4), (0, 4, 5)))
    bow_chart = [(4, 0), (6, 0), (8, 2), (6, 4)]
    bow = ("bow", bow_chart, [(x, y, Fraction(0)) for x, y in bow_chart], ((0, 1, 2), (3, 2, 1)))
    two = _strip_of_squares(flat, flat)
    three = _strip_of_squares(flat, flat, flat)
    folded = _strip_of_squares(flat, bent, flat)
    cases = [
        ("two squares glued", two, across, _pairs(("f0", "f1"))),
        ("two squares, no plan", two, across, frozenset()),
        ("foreign pair", two, across, _pairs(("f1", "elsewhere"))),
        ("chain of three", three, wide, _pairs(("f0", "f1"), ("f1", "f2"))),
        ("the other pair only", three, across, _pairs(("f1", "f2"))),
        ("pair apart", three, wide, _pairs(("f0", "f2"))),
        ("inside one face", two, inside_one, _pairs(("f0", "f1"))),
        ("two polygons", three, on_the_diagonal, _pairs(("f0", "f1"), ("f1", "f2"))),
        ("folded middle", folded, wide, _pairs(("f0", "f1"), ("f1", "f2"))),
        ("a pair that is no pair", two, across, frozenset({frozenset({"f0"}), frozenset({"f0", "f1", "elsewhere"})})),
    ]
    runs = []
    for label, lift, polygons, inert in cases:
        for cap in (None, 0, 3, 40):
            runs.append((f"{label} cap={cap}", geometry.compare_generated(runner, f"plan-{label}", lift, _plan_call(polygons, inert), cap)))
            CHECKED["plan scenarios"] += 1
    # a non-convex face in a group of the plan keeps its own estimate and is split when the chord is deeper than the budget; a bow-tie face stays out of the group
    mixed = _strip_of_squares(flat, extra_faces=[ell])
    crossing = _strip_of_squares(flat, extra_faces=[bow])
    for label, lift, polygons, inert in (
        ("ell and a square", mixed, [[(1, 1), (6, 1), (6, 3), (1, 3)], [(2, 5), (3, 6), (2, 7)]], _pairs(("L", "f0"))),
        ("bow-tie and a square", crossing, [[(3, 1), (7, 1), (7, 3), (3, 3)]], _pairs(("f0", "bow"))),
    ):
        for cap in (None, 0, 5, 60):
            runs.append((f"{label} cap={cap}", geometry.compare_generated(runner, f"plan-{label}", lift, _plan_call(polygons, inert), cap)))
            CHECKED["plan scenarios"] += 1
    assert_all_equal(runs)
    expected = {name: run.expected for name, run in runs}
    glued, plain = expected["two squares glued cap=None"].result, expected["two squares, no plan cap=None"].result
    assert dict(glued.counters)[clip.PLAN_INERT_FACE_PAIRS] == 1 and not glued.points
    assert len(plain.points) == 2 and clip.PLAN_INERT_FACE_PAIRS not in dict(plain.counters), "the real edge cuts without the plan (keys without the `node:` tolerance)"
    chained = expected["chain of three cap=None"].result
    assert dict(chained.counters)[clip.PLAN_INERT_FACE_PAIRS] == 2 and not chained.points
    found = {name: dict(run.result.counters) for name, run in expected.items() if run.exception is None and name.endswith("cap=None")}
    assert found["ell and a square cap=None"][clip.DIAGONAL_KEPT_NOT_PLANAR] == 1, "the non-convex face keeps its own estimate inside the group: the group is split"
    assert found["ell and a square cap=None"][clip.PLAN_INERT_FACE_PAIRS] == 1
    assert clip.PLAN_INERT_FACE_PAIRS not in found["bow-tie and a square cap=None"], "a face with a folded winding stays out of the group"
    assert found["pair apart cap=None"][clip.PLAN_INERT_CUTS_AVOIDED] == 0, "a pair of faces without a common edge glues nothing across it"


# --------------------------------------------------------------------------
# Одна плоскость, много вызовов
# --------------------------------------------------------------------------


@needs_corpus
def test_repeated_calls_on_one_plane_and_one_session_equal_the_oracle_each_time():
    """Плоскость переводится один раз и живёт в сеансе: повторные вызовы на ТОЙ ЖЕ плоскости (состояние и нормали восстановлены) равны эталону."""

    mirror = cftuv_native.new_mirror()
    paths = geometry.heavy_paths(3) + geometry.field_paths()[:2]
    problems = []
    for path in paths:
        record = nc.read_record(path)
        before = record.before()
        expected = nc.execute(nc.prepare_call(nc.OP_CLIP, record.call_blob, before))
        budget, _store = nc.restore_state(before)
        persistent = nc.decode_call(nc.OP_CLIP, record.call_blob, budget, None)
        plane = persistent.args[0]
        normals = dict(plane._normal_by_position)
        for round_number in range(3):
            budget, _store = nc.restore_state(before)
            plane._normal_by_position.clear()
            plane._normal_by_position.update(normals)
            call = nc.Call(nc.OP_CLIP, persistent.args, persistent.kwargs, budget, None)
            actual = nc.execute(call, function=mirror.clip_geometry)
            CHECKED["repeated on one plane"] += 1
            for difference in nc.compare_outcomes(nc.OP_CLIP, before, expected, actual):
                problems.append(f"{path.name} round {round_number}: {difference}")
        assert mirror.clip_cache_size() >= 1
    assert not problems, "\n".join(problems[:12])
    assert mirror.clip_cache_size() == len(paths), "one converted plane per distinct plane object, not per call"


@needs_corpus
def test_the_plane_cache_is_bounded_and_forgets_the_least_recently_used_plane():
    mirror = cftuv_native.new_mirror()
    record = nc.read_record(geometry.field_paths()[-1])
    before = record.before()
    keep = nc.prepare_call(nc.OP_CLIP, record.call_blob, before)
    for _ in range(40):
        call = nc.prepare_call(nc.OP_CLIP, record.call_blob, before)
        nc.execute(call, function=mirror.clip_geometry)
        # the first plane is touched again every round: it must never be the one that goes out
        nc.execute(nc.Call(nc.OP_CLIP, keep.args, keep.kwargs, nc.restore_state(before)[0], None), function=mirror.clip_geometry)
    assert mirror.clip_cache_size() <= 16
    mirror.forget_clip()
    assert mirror.clip_cache_size() == 0


# --------------------------------------------------------------------------
# Кэш результатов между вызовами: ничего не меняет в ответе и в цене
# --------------------------------------------------------------------------
#
# Пересечение отрезка с прямой ребра — функция двух концов и прямой; сеанс держит результат и УПОРЯДОЧЕННЫЕ вопросы вычисления к памяти канонизации
# (`prime_support`, `squarefree_split`), а попадание задаёт их снова на ТЕКУЩЕЙ памяти и ТЕКУЩЕМ бюджете: статьи, таблицы, порядок LRU и точка исчерпания те же.


def _chain_records(minimum: int = 8) -> list:
    chains = geometry.field_chains(minimum)
    if not chains:
        pytest.skip("в полевом корпусе нет цепочек соседних alpha")
    return sorted(chains.items(), key=lambda item: str(item[0]))


@needs_corpus
def test_neighbouring_alphas_through_the_warm_cache_equal_the_oracle_at_every_step_and_the_cache_is_used():
    total_hits = value_hits = lift_hits = 0
    for (mesh, patch), paths in _chain_records():
        mirror = cftuv_native.new_mirror()
        walker = geometry.DropinRunner(mirror)
        order = paths + list(reversed(paths)) + paths  # up, down and up again: the second and third passes hit what the first stored
        runs = []
        for path in order:
            runs.append((f"{mesh} patch {patch} {path.name}", walker.compare(nc.read_record(path))))
            CHECKED["warm chain steps"] += 1
        assert_all_equal(runs)
        stats = mirror.clip_warm_stats()
        total_hits += stats[0]
        value_hits += stats[3]
        lift_hits += stats[4]
    assert total_hits > 1000 and value_hits > 1000 and lift_hits > 100, f"the caches must be exercised, not skipped: {total_hits} crossings, {value_hits} values, {lift_hits} lifts"


@needs_corpus
def test_a_cap_sweep_that_exhausts_inside_a_replayed_computation_leaves_the_oracles_partial_state(runner):
    """Всё в кэше уже лежит (полный бюджет), потолок режет повтор: исчерпание в нужном вопросе, те же частичные статьи, память и нормали."""

    refused = Counter()
    problems = []
    for path in geometry.heavy_paths(3):
        record = nc.read_record(path)
        before = record.before()
        assert runner.compare(record).equal  # fills the cache
        hits_before = runner.mirror.clip_warm_stats()[0]
        spent = sum(before.budget["articles"])
        delta = sum(record.expected().after.budget["articles"]) - spent
        for level in geometry.cap_levels(delta) + [delta // 2, delta // 3, delta * 2 // 3]:
            state = dataclasses.replace(before, budget={**before.budget, "cap": spent + level})
            run = runner.compare(record, before=state)
            CHECKED["warm cap sweep"] += 1
            refused[run.outcome_label] += 1
            if not run.equal:
                problems.append((f"{path.name} cap +{level}", run))
        assert runner.mirror.clip_warm_stats()[0] > hits_before, "the sweep must run through replayed answers"
    assert not problems, geometry.explain(problems)
    assert refused["raised:ExactCanonicalizationWorkBudgetExhausted"] >= 20 and refused["CLIPPED"] >= 3, refused


@needs_corpus
def test_a_cache_that_is_dropped_when_full_changes_nothing():
    mirror = cftuv_native.new_mirror()
    mirror.set_clip_warm_limit(40)
    walker = geometry.DropinRunner(mirror)
    (mesh, patch), paths = _chain_records()[0]
    runs = [(f"{mesh} patch {patch} {path.name}", walker.compare(nc.read_record(path))) for path in paths + paths]
    assert_all_equal(runs)
    assert mirror.clip_warm_stats()[2] <= 2 * 40, "two generations of at most `limit` crossings"
    mirror.forget_clip()
    assert mirror.clip_warm_stats()[2] == 0 and mirror.clip_cache_size() == 0
    assert_all_equal([(f"{mesh} patch {patch} after forget", walker.compare(nc.read_record(paths[-1])))])


@needs_corpus
def test_the_cache_switch_changes_neither_the_answer_nor_the_cost():
    """Кэш выключен: ни одного попадания, а ответ и цена те же, что у эталона (и, значит, у включённого кэша)."""

    mirror = cftuv_native.new_mirror()
    mirror.set_clip_warm_enabled(False)
    walker = geometry.DropinRunner(mirror)
    (mesh, patch), paths = _chain_records()[0]
    assert_all_equal([(f"{mesh} patch {patch} {path.name}", walker.compare(nc.read_record(path))) for path in paths[:6] + paths[:3]])
    assert mirror.clip_warm_stats()[:2] == (0, 0) and mirror.clip_warm_stats()[3:] == (0, 0)
    mirror.set_clip_warm_enabled(True)
    assert_all_equal([(f"{mesh} patch {patch} {path.name}", walker.compare(nc.read_record(path))) for path in paths[:6] + paths[:3]])
    assert mirror.clip_warm_stats()[0] > 0


def _typed_as_ints(point):
    """Тот же точный числовой смысл, другие типы коэффициентов: целые `int` там, где `Fraction` с единичным знаменателем."""

    from fractions import Fraction

    from cftuv_envelope.exact_sqrt_sum import SqrtSumV1

    def coordinate(value):
        return SqrtSumV1(tuple((radicand, int(coef) if isinstance(coef, Fraction) and coef.denominator == 1 else coef) for radicand, coef in value.terms))

    return coordinate(point[0]), coordinate(point[1])


@needs_corpus
def test_the_cache_does_not_hand_a_result_across_different_coefficient_types(runner):
    """Тип коэффициента берётся из входных точек (`product_added` оставляет члены основания как есть): ключ кэша сравнивает концы СТРОГО, с типами."""

    mirror = cftuv_native.new_mirror()
    walker = geometry.DropinRunner(mirror)
    typed_differently = 0
    for path in geometry.field_paths()[:40:4]:
        record = nc.read_record(path)
        before = record.before()
        call = nc.decode_call(nc.OP_CLIP, record.call_blob, nc.build_budget(before.budget), None)
        retyped = {key: _typed_as_ints(point) for key, point in call.kwargs["points"].items()}
        typed_differently += sum(1 for key, point in retyped.items() if nc.canonical(point) != nc.canonical(call.kwargs["points"][key]))
        assert walker.compare(record).equal
        # the same points, int-typed wherever the value allows: the oracle's answer differs in types, and so must the cache's
        blob = nc.encode_call(nc.Call(nc.OP_CLIP, call.args, {**call.kwargs, "points": retyped}, call.budget, None))
        run = walker.compare(nc.Record(dict(record.meta), {"call": blob}), before=before)
        assert run.equal, geometry.explain([(path.name, run)])
        CHECKED["retyped warm calls"] += 1
    assert typed_differently > 0, "the retyped calls must really differ in coefficient types"


# --------------------------------------------------------------------------
# Вызов как у сеанса: через память `clip_memo`
# --------------------------------------------------------------------------


@needs_corpus
@pytest.mark.parametrize("which", ["small", "heavy"])
def test_the_dropin_is_a_valid_compute_for_the_clip_memo_miss_and_hit(which):
    """`run_clip(compute=...)`: промах кладёт в память пикл нативного результата и запись памяти, попадание их проигрывает: состояние то же, что у эталона."""

    path = geometry.field_paths()[0] if which == "small" else geometry.heavy_paths(1)[0]
    record = nc.read_record(path)
    before = record.before()
    mirror = cftuv_native.new_mirror()

    def sequence(compute):
        memo = clip_memo.MEMO
        was = memo.enabled
        memo.enabled = True
        memo.clear()
        try:
            call = nc.prepare_call(nc.OP_CLIP, record.call_blob, before)
            steps = []
            for _ in range(2):
                clipped, status = clip_memo.run_clip(compute, call.args[0], call.budget, clip.clip_policy(), **call.kwargs)
                state = nc.capture_state(call.budget, None)
                steps.append((status, nc.Outcome(clipped, None, state, nc.observe(call), 0.0)))
            return steps
        finally:
            memo.clear()
            memo.enabled = was

    expected = sequence(clip.clip_geometry)
    actual = sequence(mirror.clip_geometry)
    assert [status for status, _ in expected] == [status for status, _ in actual] == [clip_memo.MISS, clip_memo.HIT]
    for (_, want), (_, got) in zip(expected, actual):
        assert nc.compare_outcomes(nc.OP_CLIP, before, want, got) == []
    # the result survives the pickle of the memo entry and equals the oracle's
    native_result = actual[0][1].result
    assert nc.canonical(pickle.loads(pickle.dumps(native_result, protocol=5))) == nc.canonical(native_result)


# --------------------------------------------------------------------------
# Что вставка возвращает: настоящие классы, те же объекты
# --------------------------------------------------------------------------


def _outcome_exceptions(record, before, mirror):
    """`(исключение эталона, исключение вставки)` на одной записи (None, если вызов не бросил)."""

    caught = []
    for function in (None, mirror.clip_geometry):
        call = nc.prepare_call(nc.OP_CLIP, record.call_blob, before)
        try:
            nc.invoke(call, function)
            caught.append(None)
        except Exception as exc:  # noqa: BLE001 - исключение операции — часть её исхода
            caught.append(exc)
    return caught


@pytest.mark.skipif(not HAS_CORPUS or not geometry.synthetic_paths(), reason="нет синтетического корпуса")
def test_the_exceptions_are_the_real_classes_with_the_oracles_fields():
    mirror = cftuv_native.new_mirror()
    seen = Counter()
    refusals = []
    for path in geometry.synthetic_paths():
        record = nc.read_record(path)
        if not record.expected().exception:
            continue
        want, got = _outcome_exceptions(record, record.before(), mirror)
        assert want is not None and got is not None, path.name
        assert type(got) is type(want), (path.name, type(want), type(got))
        assert got.args == want.args, path.name
        assert vars(got) == vars(want), (path.name, vars(want), vars(got))
        seen[type(want).__name__] += 1
        if type(got).__name__ == "MaterializationRefusal":
            refusals.append(got)
    for name in ("MaterializationRefusal", "ExactCanonicalizationWorkBudgetExhausted", "OverflowError"):
        assert seen[name] >= 1, seen
    refusal = refusals[0]
    assert refusal.outcome.value in refusal.args[0] and refusal.counters == () and refusal.station_conflict == ()


@needs_corpus
def test_the_result_reuses_the_objects_the_oracle_would_return():
    """Точка узла, пришедшая из `points`, ключи-строки входа и имена треугольников возвращаются теми же объектами; одна точка — один объект на все списки."""

    record = nc.read_record(geometry.heavy_paths(1)[0])
    mirror = cftuv_native.new_mirror()
    call = nc.prepare_call(nc.OP_CLIP, record.call_blob, record.before())
    result = mirror.clip_geometry(call.args[0], call.budget, **call.kwargs)
    points, plane = call.kwargs["points"], call.args[0]
    originals = {id(point) for point in points.values()}
    shared_point_ids = {id(point) for entries in result.cycles for _key, point in entries} | {id(point) for entries in result.vertex_lists for _key, point in entries}
    assert shared_point_ids & originals, "a node that stands for an input point returns the input's own tuple"
    key_ids = {id(key) for key in points}
    assert any(id(key) in key_ids for entries in result.cycles for key, _point in entries), "input keys come back as the input's own strings"
    new = result.points
    assert new, "the heaviest record creates vertices"
    carried = 0
    for name, point in new.items():
        for entries in result.vertex_lists:
            for key, found in entries:
                if key == name:
                    assert found is point, "one clip vertex is one object in every list"
                    carried += 1
    assert carried >= len(new)
    names = {id(triangle.name) for triangle in plane.triangles}
    assert result.lifted and all(id(name) in names for _position, (name, _normal) in result.lifted.values()), "triangle names are the plane's own strings"
    for key, (_position, (_name, normal)) in result.lifted.items():
        if normal is not None:
            assert any(found is normal for found in plane._normal_by_position.values()), "the lifted normal is the very tuple written into the plane"
            break


@needs_corpus
def test_a_call_without_a_budget_spends_the_unbudgeted_telemetry_like_the_oracle(runner):
    spent = 0
    for path in geometry.heavy_paths(3) + geometry.field_paths()[:3]:
        record = nc.read_record(path)
        before = dataclasses.replace(record.before(), budget=None)
        run = runner.compare(record, before=before)
        CHECKED["without a budget"] += 1
        assert run.equal, geometry.explain([(path.name, run)])
        spent += sum(run.actual.after.unbudgeted) - sum(before.unbudgeted)
    assert spent > 0, "the work of a call without a budget lands in UNBUDGETED_WORK"


# --------------------------------------------------------------------------
# Названные отказы входа: сеанс переживает отказ
# --------------------------------------------------------------------------


@needs_corpus
def test_an_input_the_extension_cannot_carry_is_refused_by_name_and_the_session_survives(runner):
    record = nc.read_record(geometry.field_paths()[0])
    before = record.before()
    call = nc.prepare_call(nc.OP_CLIP, record.call_blob, before)
    mirror = runner.mirror
    broken = dict(call.kwargs)
    broken["points"] = {**call.kwargs["points"], "node:broken": (1, 2)}
    with pytest.raises(TypeError, match="cftuv_native"):
        mirror.clip_geometry(call.args[0], call.budget, **broken)
    other = dict(call.kwargs)
    other["by_faces"] = True
    other["cycles"] = [[(5, None)]]
    with pytest.raises(TypeError, match="cftuv_native"):
        mirror.clip_geometry(call.args[0], call.budget, **other)
    assert runner.compare(record).equal, "the session answers correctly after a refused input"


def test_a_plane_without_a_normal_table_is_a_named_refusal_not_a_guess():
    record = nc.read_record(geometry.field_paths()[0]) if HAS_CORPUS else pytest.skip("нет полевого корпуса")
    call = nc.prepare_call(nc.OP_CLIP, record.call_blob, record.before())

    class Bare:
        triangles = call.args[0].triangles

    with pytest.raises(cftuv_native.NativePortUnsupported, match="_normal_by_position"):
        cftuv_native.new_mirror().clip_geometry(Bare(), call.budget, **call.kwargs)


# --------------------------------------------------------------------------
# Отрицательные контроли: сравнение вставки видит каждый побочный эффект
# --------------------------------------------------------------------------


def _spoilers():
    """`{название: (порча после вызова вставки, поле, которое сравнение обязано назвать)}`."""

    def normals(_plane, _budget, _result):
        _plane._normal_by_position.popitem()

    def counts(_plane, _budget, _result):
        exact.SIGN_COUNTS["total"] += 1

    def article(_plane, budget, _result):
        budget.radical_materializations += 1

    def squarefree(_plane, _budget, _result):
        exact._SQUAREFREE_MEMO.popitem()

    def order(_plane, _budget, _result):
        first = next(iter(exact._PRIME_SUPPORT_MEMO))
        exact._PRIME_SUPPORT_MEMO[first] = exact._PRIME_SUPPORT_MEMO.pop(first)

    return {
        "a normal write is lost": (normals, "observed.plane_normals"),
        "a sign counter moves": (counts, "sign_counts.total"),
        "an article moves": (article, "budget.delta"),
        "a squarefree entry is lost": (squarefree, "memory.squarefree"),
        "a support entry is reordered": (order, "memory.prime_support"),
    }


@needs_corpus
def test_the_comparison_names_every_effect_a_spoiled_dropin_gets_wrong(runner):
    record = nc.read_record(geometry.heavy_paths(1)[0])
    before = record.before()
    expected = nc.execute(nc.prepare_call(nc.OP_CLIP, record.call_blob, before))
    assert nc.compare_outcomes(nc.OP_CLIP, before, expected, runner.outcome(record, before)) == []
    for name, (spoil, field) in _spoilers().items():
        def spoiled(plane, budget, spoil=spoil, **kwargs):
            result = runner.mirror.clip_geometry(plane, budget, **kwargs)
            spoil(plane, budget, result)
            return result

        actual = nc.execute(nc.prepare_call(nc.OP_CLIP, record.call_blob, before), function=spoiled)
        named = {item.field for item in nc.compare_outcomes(nc.OP_CLIP, before, expected, actual)}
        assert field in named, f"spoiling `{name}` must be named {field}, got {sorted(named)}"
    # a result field and an exception are named too
    for how, field in (("note", "result.note"), ("points", "result.points")):
        def spoiled_result(plane, budget, how=how, **kwargs):
            result = runner.mirror.clip_geometry(plane, budget, **kwargs)
            if how == "note":
                return dataclasses.replace(result, note=result.note + " ")
            return dataclasses.replace(result, points=dict(reversed(list(result.points.items()))))

        actual = nc.execute(nc.prepare_call(nc.OP_CLIP, record.call_blob, before), function=spoiled_result)
        assert field in {item.field for item in nc.compare_outcomes(nc.OP_CLIP, before, expected, actual)}


def test_the_comparison_tells_an_int_coefficient_from_a_fraction_and_one_float_bit_from_another():
    """Сравнение вставки с эталоном различает `int` и `Fraction` и `0.0` с `-0.0`: вставка, подменившая тип коэффициента, ловится."""

    from fractions import Fraction

    from cftuv_envelope.exact_sqrt_sum import SqrtSumV1

    assert nc.canonical(SqrtSumV1(((1, 3),))) != nc.canonical(SqrtSumV1(((1, Fraction(3)),)))
    assert nc.canonical(0.0) != nc.canonical(-0.0)
