"""Нативное `coverage._coverage_at` ЦЕЛИКОМ равно эталону на Python (`native/cftuv-core/src/coverage.rs`, `native/cftuv-python/src/coverage.rs`, шим `cftuv_native.coverage_at`).

Эталон — `kernel/src/cftuv_envelope/wavefront/coverage.py`. Ответ операции — не только `CoverageV1`: цена (шесть статей `ExactWorkBudgetV1`, на месте), память
канонизации (четыре таблицы С ПОРЯДКОМ, на месте), `SIGN_COUNTS`, `UNBUDGETED_WORK` (вызов без бюджета), запись `prime-universe` в `store` и исключение (тип
и текст). Всё это сверяется с ЖИВЫМ эталоном в том же интерпретаторе от ОДНОГО и того же состояния «до» (`native_corpus.compare_outcomes`: результат каноническим
кодом, где `int` и `Fraction` различны). Сверх канонической записи проверяется то, чего она не видит: ТОЖДЕСТВО объектов (грань без усечения возвращает
тот же кортеж точек, усечённая — те же объекты точек и новые на разрезах; `owner`, `alpha`, `polygon_doubled_area`, `work_budget` — те же объекты, что на входе),
точные типы (`CoverageV1`, `FaceCoverageV1`, `SqrtSumV1`, `tuple`, `int`/`Fraction` в слотах) и равенство `==` самого эталона.

Операнды: 1. ПОЛЕВОЙ корпус (`E:\\cftuv_native_corpus\\`: 344 записи покрытия с настоящих мешей и производные записи с урезанным потолком), каждая запись — на свежей
нативной сессии И на общей сессии всего модуля (синхронизация памяти между чужими состояниями); 2. СИНТЕТИЧЕСКИЙ корпус, записанный `native_corpus.Recorder`
с вызовов `coverage._coverage_at` на фигурах `kernel/tests/wavefront_cases.py` и на настоящем малом домене: отказ `PARTITION_IS_NOT_EXACT`, отрицательная
alpha, `work_budget=None` (как зовёт `region_contours`), `store=None`, `int`-alpha, `int`-коэффициенты в точках, знаки, которые оболочка 64 бит не решает (alpha в
2^-110 от настоящего корня: сопряжение), веса рёбер; 3. ПОВТОР alpha на одной сессии (кэш разбиения, попадание в `store`) и несколько разбиений в одной сессии
(предел кэша); 4. СВИП ПОТОЛКА: деталь исчерпания и частичное состояние на каждой границе; 5. отказы шима: именованные, без тихого отката на Python.

Модуль пропускается с названной причиной, пока расширение не собрано (`python tools/native_build.py`).
"""

from __future__ import annotations

import collections
import dataclasses
import importlib.util
import math
import os
import pickle
import sys
import time
from fractions import Fraction
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "kernel" / "src", ROOT / "kernel" / "tests"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

try:
    import mpmath  # noqa: F401
except ModuleNotFoundError:  # питон Blender: сторонние пакеты ядра лежат среди пользовательских модулей Blender
    _modules = Path(os.environ.get("APPDATA", "")) / "Blender Foundation" / "Blender" / "4.5" / "scripts" / "modules"
    if _modules.is_dir():
        sys.path.append(str(_modules))

try:
    import cftuv_native
except ModuleNotFoundError as error:
    if error.name != "cftuv_native":
        raise
    pytest.skip(
        "расширение cftuv_native не собрано: `python tools/native_build.py` ставит его в dev-venv (сверка нативного покрытия с эталоном пропущена)",
        allow_module_level=True,
    )

from native_gate import skip_unless_available  # noqa: E402

skip_unless_available(cftuv_native, "coverage")

import cftuv_envelope.exact_sqrt_sum as exact  # noqa: E402
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1  # noqa: E402
from cftuv_envelope.wavefront import build_skeleton  # noqa: E402
from cftuv_envelope.wavefront import coverage as coverage_module  # noqa: E402
from cftuv_envelope.wavefront.coverage import CoverageOutcome, CoverageV1, FaceCoverageV1  # noqa: E402
from cftuv_envelope.wavefront.faces import FaceOutcome, FaceV1, build_faces  # noqa: E402


def _load_corpus_tool():
    cached = sys.modules.get("native_corpus")
    if cached is not None:
        return cached
    spec = importlib.util.spec_from_file_location("native_corpus", ROOT / "tools" / "native_corpus.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules["native_corpus"] = module
    spec.loader.exec_module(module)
    return module


nc = _load_corpus_tool()

COMPARED = collections.Counter()
ARTICLES = nc._ARTICLES


@pytest.fixture(autouse=True)
def _kernel_process_state_is_given_back():
    """Эталон и нативная сторона пишут в процессные счётчики и память ядра; тест их не оставляет."""

    counts = dict(exact.SIGN_COUNTS)
    unbudgeted = exact.UNBUDGETED_WORK.spent_by_article()
    with exact.isolated_factorization_memory():
        yield
    exact.SIGN_COUNTS.update(counts)
    for name, value in zip(ARTICLES, unbudgeted):
        setattr(exact.UNBUDGETED_WORK, name, value)


@pytest.fixture(scope="module", autouse=True)
def _report_the_comparison_counts(request):
    yield
    reporter = request.config.pluginmanager.get_plugin("terminalreporter")
    if reporter is not None:
        reporter.write_line("native coverage compared with the Python oracle: " + ", ".join(f"{name} {count}" for name, count in sorted(COMPARED.items())))


@pytest.fixture(scope="module")
def shared_mirror():
    """Одна сессия на все записи модуля: между записями процесс меняет состояние целиком, зеркало памяти обязано это пережить."""

    return cftuv_native.new_mirror()


# --------------------------------------------------------------------------
# Исполнение и сравнение
# --------------------------------------------------------------------------


def run_native(mirror, call) -> nc.Outcome:
    """Нативный вызов с теми же аргументами, что у эталона; исход — тем же `Outcome` (исключение: тип и текст)."""

    started = time.perf_counter()
    try:
        result, error = mirror.coverage_at(call.args[0], call.args[1], call.budget, call.store), None
    except Exception as exc:  # noqa: BLE001 - исключение операции — часть её исхода
        result, error = None, (type(exc).__qualname__, str(exc))
    seconds = time.perf_counter() - started
    return nc.Outcome(result, error, nc.capture_state(call.budget, call.store), nc.observe(call), seconds)


def identity_view(partition, result):
    """Что видно только по тождеству объектов: у каждой грани — `owner` тот же, `points` тот же кортеж / `()` / индекс оригинальной точки либо `None` (новая)."""

    view = []
    for face, covered in zip(partition.faces, result.faces):
        if covered.points is face.points:
            points = "unchanged"
        else:
            originals = {id(point): index for index, point in enumerate(face.points)}
            points = tuple(originals.get(id(point)) for point in covered.points)
        view.append((covered.owner is face.owner, points))
    return view


def _is_canonical_fraction(value) -> bool:
    return type(value) is Fraction and value.denominator > 0 and math.gcd(value.numerator, value.denominator) == 1


def assert_wellformed(result, partition, alpha, budget) -> None:
    """Точные типы и тождества результата, которых `canonical` не различает."""

    assert type(result) is CoverageV1 and type(result.faces) is tuple
    assert result.alpha is alpha
    # the oracle's refusals are built without the budget (the field keeps its default)
    assert result.work_budget is (budget if result.outcome is CoverageOutcome.EXACT else None)
    assert result.polygon_doubled_area is partition.polygon_doubled_area
    sums = [result.doubled_area]
    if result.outcome is CoverageOutcome.EXACT:
        assert result.detail == "" and len(result.faces) == len(partition.faces)
        for covered in result.faces:
            assert type(covered) is FaceCoverageV1 and type(covered.points) is tuple
            sums.append(covered.doubled_area)
            for point in covered.points:
                assert type(point) is tuple and len(point) == 2
                sums.extend(point)
    for value in sums:
        assert type(value) is SqrtSumV1 and type(value.terms) is tuple
        for term in value.terms:
            assert type(term) is tuple and len(term) == 2 and type(term[0]) is int and term[0] >= 1
            assert type(term[1]) is int or _is_canonical_fraction(term[1])
            assert term[1] != 0
        assert [term[0] for term in value.terms] == sorted({term[0] for term in value.terms})
        hash(value)


def check_call(mirror, op, blob, before, label: str, *, expected=None):
    """Один вызов от состояния `before`: эталон, потом нативная сторона от ТОГО ЖЕ состояния; исходы равны точно."""

    oracle_call = nc.prepare_call(op, blob, before)
    oracle = nc.execute(oracle_call)
    native_call = nc.prepare_call(op, blob, before)
    native = run_native(mirror, native_call)
    found = nc.compare_outcomes(op, before, oracle, native)
    assert not found, f"{label}: " + "; ".join(str(item) for item in found[:4])
    if expected is not None:
        recorded = nc.compare_outcomes(op, before, expected, native)
        assert not recorded, f"{label} (against the recording): " + "; ".join(str(item) for item in recorded[:4])
    if oracle.result is not None:
        partition, alpha = native_call.args
        assert_wellformed(native.result, partition, alpha, native_call.budget)
        assert_wellformed(oracle.result, oracle_call.args[0], oracle_call.args[1], oracle_call.budget)
        assert native.result == oracle.result
        if native.result.outcome is CoverageOutcome.EXACT:
            assert identity_view(partition, native.result) == identity_view(oracle_call.args[0], oracle.result), label
    COMPARED["calls"] += 1
    COMPARED["exceptions"] += oracle.exception is not None
    COMPARED["conjugated signs"] += oracle.after.sign_counts["closed_by_conjugation"] - before.sign_counts["closed_by_conjugation"]
    return oracle


# --------------------------------------------------------------------------
# 1. Полевой корпус
# --------------------------------------------------------------------------


def _corpus_directory() -> Path | None:
    return nc.matching_corpus()


def _field_rows():
    directory = _corpus_directory()
    if directory is None:
        return directory, []
    rows = [row for row in nc.load_index(directory)["records"] if row["op"] == nc.OP_COVERAGE]
    return directory, rows


CORPUS_DIRECTORY, FIELD_ROWS = _field_rows()
FIELD_PARAMS = (
    [pytest.param(row, id=row["id"] + ("-derived" if row.get("derived") else "")) for row in FIELD_ROWS]
    if FIELD_ROWS
    else [pytest.param(None, marks=pytest.mark.skip(reason=nc.describe_missing_corpus() + ": настоящие вызовы не сверены"))]
)


@pytest.mark.parametrize("row", FIELD_PARAMS)
def test_every_field_record_equals_the_oracle_on_a_fresh_and_on_the_shared_session(row, shared_mirror):
    record = nc.read_record(CORPUS_DIRECTORY / row["path"])
    before = record.before()
    expected = record.expected()
    check_call(cftuv_native.new_mirror(), record.op, record.call_blob, before, row["id"] + " [fresh session]", expected=expected)
    check_call(shared_mirror, record.op, record.call_blob, before, row["id"] + " [shared session]", expected=expected)
    COMPARED["field records"] += 1


def test_the_field_corpus_reaches_the_branches_it_is_meant_to_pin():
    if not FIELD_ROWS:
        pytest.skip("корпус не собран: настоящие вызовы не сверены")
    kinds = collections.Counter()
    for row in FIELD_ROWS:
        kinds["derived" if row.get("derived") else "field"] += 1
        kinds["raised" if row["exception"] else "answered"] += 1
        kinds["with store" if nc.read_record(CORPUS_DIRECTORY / row["path"]).before().store else "empty store"] += 1
    assert kinds["field"] > 300 and kinds["derived"] >= 25 and kinds["raised"] >= 20 and kinds["empty store"] > 0, kinds


# --------------------------------------------------------------------------
# 2. Синтетический корпус: Recorder на вызовах `_coverage_at`
# --------------------------------------------------------------------------

#: Фигуры, дающие грани с несколькими радикалами: полевой контур (злые радикалы), звёзды, ромб, 45-градусные гребни; плюс осевые и весовые.
SYNTHETIC_FIGURES = ("axis_square", "right_triangle", "diamond", "ell", "comb_2", "cross", "staircase", "u_shape", "field_building_002_scale_64", "star_9_seed_0", "star_9_seed_3")


def _figures():
    import wavefront_cases

    named = dict(wavefront_cases.named_corpus())
    figures = [(name, named[name]) for name in SYNTHETIC_FIGURES if name in named]
    weighted = {name: polygon for name, polygon in wavefront_cases.partial_source_corpus() if name.startswith(("rect_12x8_source_bottom", "ell_12_all_sources_bottom", "ell_12_source_edge_0"))}
    return figures + sorted(weighted.items())


def _int_coefficients(partition):
    """Тот же разбиение, но целые коэффициенты у тех, что целые: значения равны, типы нет (`int` проходит в ответ сквозь `+` и `-`)."""

    def narrow(value: SqrtSumV1) -> SqrtSumV1:
        return SqrtSumV1(tuple((m, c.numerator if type(c) is Fraction and c.denominator == 1 else c) for m, c in value.terms))

    faces = tuple(dataclasses.replace(face, points=tuple((narrow(x), narrow(y)) for x, y in face.points)) for face in partition.faces)
    return dataclasses.replace(partition, faces=faces)


def near_root_alpha(face: FaceV1, index: int, bits: int = 110) -> Fraction | None:
    """alpha, при которой фронт грани проходит в 2^-bits от вершины `index`: знак там оболочка 64 бит не решает (сопряжение)."""

    base = coverage_module._value(face.line, face.points[index], SqrtSumV1.zero())
    if base.is_rational():
        return None
    low, high = base.enclosure(bits + 40)
    speed_low, speed_high = SqrtSumV1.radical(1, face.line.q).enclosure(bits + 40)
    ratio = ((low + high) / 2) / ((speed_low + speed_high) / 2)
    if ratio <= 0:
        return None
    return Fraction(round(ratio * 2**bits), 2**bits)


def _record_synthetic(recorder) -> dict:
    """Пишет в корпус вызовы `_coverage_at` на фигурах и их вариациях; возвращает счёт видов вызовов."""

    kinds: collections.Counter = collections.Counter()
    call = coverage_module._coverage_at

    def make_budget(cap=None):
        return exact.exact_work_budget(stage="COVERAGE", domain_id="synthetic", superlevel="L1", cap=cap)

    for name, polygon in _figures():
        recorder.context.update(mesh=name, mesh_digest="synthetic")
        partition = build_faces(polygon, build_skeleton(polygon))
        recorder.begin_domain(None, name, None)
        if partition.outcome is not FaceOutcome.EXACT:
            call(partition, Fraction(1), make_budget(), {})
            kinds["natural refusal"] += 1
            continue
        with exact.isolated_factorization_memory():
            _record_figure(recorder, call, make_budget, kinds, polygon, partition)
    return kinds


def _record_figure(recorder, call, make_budget, kinds, polygon, partition) -> None:
    """Вызовы на одной фигуре: память канонизации холодная в начале (первый вызов с бюджетом платит за разложения)."""

    xs = [x for loop in polygon.loops for x, _ in loop.points]
    ys = [y for loop in polygon.loops for _, y in loop.points]
    span = max(max(xs) - min(xs), max(ys) - min(ys))
    ladder = [Fraction(span * step, 8) for step in (0, 1, 2, 3, 5, 8)]
    store: dict = {}
    budget = make_budget()
    for alpha in ladder:
        call(partition, alpha, budget, store)
        call(partition, alpha, None, None)
        kinds["alpha ladder"] += 2
    call(partition, Fraction(-1, 3), make_budget(), {})
    call(partition, Fraction(-2), None, None)
    call(partition, 3, make_budget(), {})
    call(partition, 0, make_budget(), {})
    call(partition, Fraction(7, 3), None, {})
    call(partition, Fraction(7, 3), make_budget(), None)
    call(dataclasses.replace(partition, outcome=FaceOutcome.FACE_CHAIN_DOES_NOT_CLOSE), Fraction(1), make_budget(), {})
    kinds["refusals, int alpha, no budget, no store"] += 8
    narrowed = _int_coefficients(partition)
    narrow_store: dict = {}
    for alpha in ladder[1:5]:
        call(narrowed, alpha, make_budget(), narrow_store)
        kinds["int coefficients"] += 1
    # знаки, которых оболочка 64 бит не решает: alpha в 2^-110 от настоящего корня одной вершины
    near = 0
    for face in partition.faces:
        for index in range(len(face.points)):
            alpha = near_root_alpha(face, index)
            if alpha is not None and near < 3:
                call(partition, alpha, make_budget(), {})
                call(partition, alpha, None, None)
                near += 1
                kinds["near a root"] += 2


@pytest.fixture(scope="module")
def synthetic(tmp_path_factory):
    """Корпус синтетических вызовов `_coverage_at`: `(рекордер, счёт видов)`. Запись ставится вместо операции на время сборки корпуса."""

    root = tmp_path_factory.mktemp("native_coverage_corpus")
    recorder = nc.Recorder(root, {"python": sys.version.split()[0], "kernel_identity": "synthetic", "git_head": "synthetic"})
    saved = coverage_module._coverage_at
    coverage_module._coverage_at = recorder.wrap(nc.OP_COVERAGE, saved)
    try:
        with exact.isolated_factorization_memory():
            kinds = _record_synthetic(recorder)
            kinds.update(_record_real_domain(recorder))
    finally:
        coverage_module._coverage_at = saved
    recorder.write_index()
    return recorder, kinds


def _record_real_domain(recorder) -> collections.Counter:
    """Настоящий малый домен ядра (складка): вызовы `coverage_at` из `region_contours` и `conveyor_coverage` с бюджетом и `store` подготовки."""

    import developable_factories as df
    from developable_route import materialize_developable

    from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
    from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1

    recorder.context.update(mesh="fold_strip", mesh_digest="synthetic", alpha="3.5")
    recorder.begin_domain(None, "fold_strip", "3.5")
    before = len(recorder.rows)
    result, _prepared = materialize_developable(
        df.fold_strip(),
        ("r0a", "r0b"),
        alpha="3.5",
        decal_topology_law=DecalTopologyLawV1.PLANAR_POLYGONS_V1,
        near_planar_lift_law=NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1,
    )
    assert result.is_materialized, result.detail
    return collections.Counter({"real domain": len(recorder.rows) - before})


def test_the_synthetic_corpus_reaches_every_branch_the_field_corpus_lacks(synthetic):
    recorder, kinds = synthetic
    outcomes = collections.Counter(row["outcome"] for row in recorder.rows)
    assert len(recorder.rows) > 150, len(recorder.rows)
    assert {"EXACT", "PARTITION_IS_NOT_EXACT", "ALPHA_IS_NEGATIVE"} <= set(outcomes), outcomes
    assert kinds["near a root"] >= 6 and kinds["int coefficients"] >= 4 and kinds["real domain"] >= 1, kinds
    budgeted = collections.Counter(row["budget"] for row in recorder.rows)
    assert budgeted[True] and budgeted[False], budgeted


def test_every_synthetic_call_equals_the_oracle_on_a_fresh_and_on_the_shared_session(synthetic, shared_mirror):
    recorder, _kinds = synthetic
    conjugated = ints = refusals = no_store = 0
    for row in recorder.rows:
        record = nc.read_record(recorder.root / row["path"])
        before = record.before()
        expected = record.expected()
        label = f"{row['id']} {row['mesh']} alpha={row['lattice_alpha']}"
        check_call(cftuv_native.new_mirror(), record.op, record.call_blob, before, label + " [fresh]", expected=expected)
        oracle = check_call(shared_mirror, record.op, record.call_blob, before, label + " [shared]", expected=expected)
        conjugated += oracle.after.sign_counts["closed_by_conjugation"] - before.sign_counts["closed_by_conjugation"]
        refusals += oracle.result is not None and oracle.result.outcome is not CoverageOutcome.EXACT
        ints += oracle.result is not None and any(type(c) is int for covered in oracle.result.faces for p in covered.points for s in p for _, c in s.terms)
        no_store += record.before().store is None
    assert conjugated > 0, "no sign went through the conjugation: the near-root alphas missed their vertices"
    assert refusals >= 20 and ints >= 1 and no_store >= 10, (refusals, ints, no_store)
    COMPARED["synthetic records"] += len(recorder.rows)


# --------------------------------------------------------------------------
# 3. Повтор alpha на одной сессии, предел кэша, ответ кэша
# --------------------------------------------------------------------------


def _chain_records(count: int = 6):
    """Записи поля для цепочек: самые тяжёлые по каждому мешу и пара малых."""

    if not FIELD_ROWS:
        pytest.skip("корпус не собран: настоящие вызовы не сверены")
    fresh = [row for row in FIELD_ROWS if not row.get("derived") and not row["exception"]]
    by_mesh: dict = {}
    for row in sorted(fresh, key=lambda item: -item["bytes"]):
        by_mesh.setdefault(row["mesh"], []).append(row)
    picked = [rows[0] for rows in by_mesh.values()] + [rows[-1] for rows in by_mesh.values()]
    return picked[:count] + picked[-2:]


def _steps(alpha: Fraction):
    return [alpha, alpha * Fraction(3, 4), alpha / 2, alpha * Fraction(5, 4), alpha, Fraction(0), alpha + Fraction(1, 7), alpha * Fraction(3, 4), alpha * 3]


def _run_chain(runner, partition, alphas, budget, store):
    """Исполняет alpha подряд на ОДНОМ бюджете и ОДНОМ `store`; после каждого шага — исход и снимок процесса."""

    trace = []
    for alpha in alphas:
        try:
            result, error = runner(partition, alpha, budget, store), None
        except Exception as exc:  # noqa: BLE001
            result, error = None, (type(exc).__qualname__, str(exc))
        trace.append(nc.Outcome(result, error, nc.capture_state(budget, store), {}, 0.0))
    return trace


def test_repeated_alphas_on_one_session_reuse_the_partition_and_hit_the_store(shared_mirror):
    records = _chain_records()
    mirror = cftuv_native.new_mirror()
    for row in records:
        record = nc.read_record(CORPUS_DIRECTORY / row["path"])
        before = record.before()
        oracle_call = nc.prepare_call(record.op, record.call_blob, before)
        alphas = _steps(oracle_call.args[1])
        oracle = _run_chain(coverage_module._coverage_at, oracle_call.args[0], alphas, oracle_call.budget, oracle_call.store)
        native_call = nc.prepare_call(record.op, record.call_blob, before)
        partition = native_call.args[0]
        before_cache = mirror._session.coverage_cache()[0]
        native = _run_chain(mirror.coverage_at, partition, alphas, native_call.budget, native_call.store)
        for step, (want, got) in enumerate(zip(oracle, native)):
            found = nc.compare_outcomes(nc.OP_COVERAGE, before, want, got)
            assert not found, f"{row['id']} step {step} alpha={alphas[step]}: " + "; ".join(str(item) for item in found[:3])
        assert mirror._session.coverage_cache()[0] == before_cache + 1, "the partition is converted once and kept"
        COMPARED["chain steps"] += len(alphas)


def test_the_second_call_on_a_partition_is_a_store_hit_that_spends_nothing(shared_mirror):
    if not FIELD_ROWS:
        pytest.skip("корпус не собран: настоящие вызовы не сверены")
    row = next(item for item in FIELD_ROWS if item["mesh"] == "building" and not item.get("derived") and not item["exception"])
    record = nc.read_record(CORPUS_DIRECTORY / row["path"])
    call = nc.prepare_call(record.op, record.call_blob, record.before())
    partition, alpha = call.args
    mirror = cftuv_native.new_mirror()
    store: dict = {}
    budget = exact.unlimited_reference_budget(stage="COVERAGE")
    mirror.coverage_at(partition, alpha, budget, store)
    assert len(store) == 1 and mirror.last_timings[4] > 0, "the first call converted the partition"
    first_cost = budget.spent
    assert first_cost > 0
    mirror.coverage_at(partition, alpha, budget, store)
    assert len(store) == 1 and mirror.last_timings[4] == 0, "the second call found the converted partition"
    assert budget.spent == first_cost, "the same alpha on a warm memory and a store hit costs nothing"
    mirror.coverage_at(partition, alpha * Fraction(3, 4), budget, store)
    assert len(store) == 1, "another alpha does not remember another universe"

def test_the_partition_cache_is_bounded_and_an_evicted_partition_still_answers(shared_mirror):
    import wavefront_cases

    mirror = cftuv_native.new_mirror()
    polygon = wavefront_cases.axis_square(8)
    base = build_faces(polygon, build_skeleton(polygon))
    copies = [pickle.loads(pickle.dumps(base)) for _ in range(70)]
    want = coverage_module._coverage_at(base, Fraction(3), None, None)
    for partition in copies:
        got = mirror.coverage_at(partition, Fraction(3))
        assert got == want
    assert mirror._session.coverage_cache()[0] <= 64
    again = mirror.coverage_at(copies[0], Fraction(3))
    assert again == want and again.faces[0].owner is copies[0].faces[0].owner
    mirror._session.forget_coverage()
    assert mirror._session.coverage_cache() == (0, 0)
    assert mirror.coverage_at(copies[1], Fraction(3)) == want


def test_a_store_that_is_not_a_plain_dict_is_read_and_written_through_its_methods():
    import wavefront_cases

    class Store(dict):
        pass

    polygon = wavefront_cases.right_triangle(12)
    partition = build_faces(polygon, build_skeleton(polygon))
    mirror = cftuv_native.new_mirror()
    oracle_store, native_store = Store(), Store()
    budget_a, budget_b = exact.exact_work_budget(stage="COVERAGE"), exact.exact_work_budget(stage="COVERAGE")
    for alpha in (Fraction(1), Fraction(2)):
        want = coverage_module._coverage_at(partition, alpha, budget_a, oracle_store)
        got = mirror.coverage_at(partition, alpha, budget_b, native_store)
        assert got == want
    assert list(native_store) == list(oracle_store) and budget_a.spent_by_article() == budget_b.spent_by_article()


# --------------------------------------------------------------------------
# 4. Свип потолка
# --------------------------------------------------------------------------


def _cap_values(start: int, cost: int) -> list:
    """Потолки `start .. start+cost+1`: все у начала, геометрическая сетка, все у конца (там и лежит граница отказа)."""

    values = set(range(start, start + min(cost, 40) + 1))
    step = 1
    while step < cost:
        values.add(start + step)
        values.add(start + (step * 3) // 2)
        step *= 2
    values.update(range(max(start, start + cost - 10), start + cost + 2))
    return sorted(values)


def _sweep(row, mirror) -> int:
    record = nc.read_record(CORPUS_DIRECTORY / row["path"])
    before = record.before()
    unlimited = dataclasses.replace(before, budget={**before.budget, "cap": None, "mode": "UNLIMITED_REFERENCE"})
    cost = nc.execute(nc.prepare_call(record.op, record.call_blob, unlimited))
    spent = sum(after - was for after, was in zip(cost.after.budget["articles"], before.budget["articles"]))
    start = sum(before.budget["articles"])
    refused = 0
    for cap in _cap_values(start, spent):
        starved = dataclasses.replace(before, budget={**before.budget, "cap": cap, "mode": "BOUNDED"})
        oracle = check_call(mirror, record.op, record.call_blob, starved, f"{row['id']} cap={cap}")
        refused += oracle.exception is not None
    return refused


def test_a_cap_sweep_over_real_records_gives_the_same_exhaustion_and_the_same_partial_state(shared_mirror):
    if not FIELD_ROWS:
        pytest.skip("корпус не собран: настоящие вызовы не сверены")
    candidates = [row for row in FIELD_ROWS if not row.get("derived") and not row["exception"] and row["budget"]]
    small = sorted(candidates, key=lambda row: row["bytes"])
    picked = [small[0], small[len(small) // 4], small[len(small) // 2], small[-1]]
    refused = sum(_sweep(row, shared_mirror) for row in picked)
    assert refused > 10, "the sweep must meet the exhaustion"
    COMPARED["swept records"] += len(picked)


def test_a_cap_sweep_over_a_synthetic_call_that_factors_and_divides(synthetic, shared_mirror):
    recorder, _kinds = synthetic
    rows = [row for row in recorder.rows if row["mesh"].startswith("field_building") and row["outcome"] == "EXACT" and row["budget"]]
    assert rows
    row = rows[0]
    record = nc.read_record(recorder.root / row["path"])
    before = record.before()
    unlimited = dataclasses.replace(before, budget={**before.budget, "cap": None, "mode": "UNLIMITED_REFERENCE"})
    cost = nc.execute(nc.prepare_call(record.op, record.call_blob, unlimited))
    spent = sum(after - was for after, was in zip(cost.after.budget["articles"], before.budget["articles"]))
    assert spent > 0
    start = sum(before.budget["articles"])
    exhausted = 0
    for cap in _cap_values(start, spent):
        starved = dataclasses.replace(before, budget={**before.budget, "cap": cap, "mode": "BOUNDED"})
        exhausted += check_call(shared_mirror, record.op, record.call_blob, starved, f"{row['id']} cap={cap}").exception is not None
    assert exhausted > 0


# --------------------------------------------------------------------------
# 5. Отказы шима и крайние входы
# --------------------------------------------------------------------------


def _small_partition():
    import wavefront_cases

    polygon = wavefront_cases.axis_square(8)
    return build_faces(polygon, build_skeleton(polygon))


def test_a_face_without_a_supporting_line_raises_the_oracles_value_error_and_spends_nothing():
    partition = _small_partition()
    faces = (partition.faces[0], dataclasses.replace(partition.faces[1], line=None), *partition.faces[2:])
    broken = dataclasses.replace(partition, faces=faces)
    mirror = cftuv_native.new_mirror()
    budget_a, budget_b = exact.exact_work_budget(stage="COVERAGE"), exact.exact_work_budget(stage="COVERAGE")
    with pytest.raises(ValueError) as oracle:
        coverage_module._coverage_at(broken, Fraction(1), budget_a, {})
    with pytest.raises(ValueError) as native:
        mirror.coverage_at(broken, Fraction(1), budget_b, {})
    assert str(oracle.value) == str(native.value)
    assert budget_a.spent_by_article() == budget_b.spent_by_article() == (0, 0, 0, 0, 0, 0)
    assert mirror.coverage_at(partition, Fraction(1)) == coverage_module._coverage_at(partition, Fraction(1), None, None)


def test_the_refusals_are_the_oracles_and_cost_nothing():
    partition = _small_partition()
    mirror = cftuv_native.new_mirror()
    budget = exact.exact_work_budget(stage="COVERAGE")
    for alpha in (Fraction(-1, 3), Fraction(-5), -1):
        got = mirror.coverage_at(partition, alpha, budget, {})
        want = coverage_module._coverage_at(partition, alpha, budget, {})
        assert got == want and got.outcome is CoverageOutcome.ALPHA_IS_NEGATIVE and got.detail == str(alpha) and got.alpha is alpha
    refused = dataclasses.replace(partition, outcome=FaceOutcome.FACE_CHAIN_AMBIGUOUS, faces=())
    got = mirror.coverage_at(refused, Fraction(1), budget, {})
    assert got == coverage_module._coverage_at(refused, Fraction(1), budget, {})
    assert got.outcome is CoverageOutcome.PARTITION_IS_NOT_EXACT and got.detail == "FACE_CHAIN_AMBIGUOUS" and got.faces == ()
    assert budget.spent == 0


def test_an_alpha_the_extension_cannot_carry_is_refused_by_name_and_the_session_survives():
    partition = _small_partition()
    mirror = cftuv_native.new_mirror()
    for alpha in (0.5, 1.0):
        with pytest.raises(TypeError, match="cftuv_native"):
            mirror.coverage_at(partition, alpha)
    with pytest.raises(TypeError):  # the oracle's own first comparison refuses it too
        mirror.coverage_at(partition, "1/2")
    assert mirror.coverage_at(partition, Fraction(1)) == coverage_module._coverage_at(partition, Fraction(1), None, None)


def test_a_partition_the_extension_cannot_carry_is_refused_by_name_and_the_session_survives():
    partition = _small_partition()
    mirror = cftuv_native.new_mirror()
    face = partition.faces[0]
    wrong = (
        ("a coordinate that is not a SqrtSumV1", (((1, 2), face.points[0][1]), *face.points[1:])),
        ("a non-canonical sum", ((SqrtSumV1(((2, Fraction(1)), (1, Fraction(1)))), face.points[0][1]), *face.points[1:])),
        ("a float coefficient", ((SqrtSumV1(((1, 0.5),)), face.points[0][1]), *face.points[1:])),
        ("a point of three", ((*face.points[0], face.points[0][0]), *face.points[1:])),
    )
    for what, points in wrong:
        broken = dataclasses.replace(partition, faces=(dataclasses.replace(face, points=points), *partition.faces[1:]))
        with pytest.raises(TypeError, match="cftuv_native"):
            mirror.coverage_at(broken, Fraction(1))
        assert mirror.coverage_at(partition, Fraction(1)) == coverage_module._coverage_at(partition, Fraction(1), None, None), what


def test_a_session_without_the_bound_classes_says_so_instead_of_guessing():
    session = type(cftuv_native.new_mirror()._session)()
    with pytest.raises(RuntimeError, match="not bound"):
        session.coverage_at(object(), Fraction(1), None, None, None, None, ([], set(), {}, {}, {}))


def _cut_chain(runner, partition, steps):
    """Alpha по очереди на ОДНОМ бюджете и ОДНОМ `store`; между вызовами `steps` портит память канонизации. Снимок процесса после каждого вызова."""

    budget = exact.exact_work_budget(stage="COVERAGE", domain_id="poison", superlevel="L1")
    store: dict = {}
    trace = []
    for alpha, damage in steps:
        if damage is not None:
            damage()
        try:
            result, error = runner(partition, alpha, budget, store), None
        except Exception as exc:  # noqa: BLE001
            result, error = None, (type(exc).__qualname__, str(exc))
        trace.append(nc.Outcome(result, error, nc.capture_state(budget, store), {}, 0.0))
    return trace


def test_a_memory_changed_between_calls_costs_the_planned_cut_what_it_costs_the_oracle():
    """План ребра держит простые, о которых спрашивает цикл сопряжений; память, изменённая между вызовами, обязана дать ту же цену, что у эталона.

    Забытая запись `squarefree_split` простого и забытый носитель платят бюджет заново, полный сброс платит всё заново, а переставленная запись
    факторизации меняет порядок вытеснения. План не вправе помнить ничего, что эта память ему не подтвердила. (Подменять запись памяти ЧУЖОЙ нельзя: цикл
    сопряжений эталона на неканоническом знаменателе не кончается.)
    """

    import wavefront_cases

    figure = wavefront_cases.right_triangle(12)
    partition = build_faces(figure, build_skeleton(figure))
    ladder = [Fraction(2), Fraction(5, 2), Fraction(3), Fraction(7, 2)]

    def forget_a_split():
        exact._SQUAREFREE_MEMO.pop(2, None)
        exact._PRIME_SUPPORT_MEMO.clear()

    def reorder():
        for key in list(exact._FACTORIZATION_MEMO)[:3]:
            exact._FACTORIZATION_MEMO[key] = exact._FACTORIZATION_MEMO.pop(key)

    def forget_everything():
        exact.reset_factorization_memory()

    steps = [(ladder[0], None), (ladder[1], None), (ladder[2], forget_a_split), (ladder[3], reorder), (ladder[0], forget_everything), (ladder[1], None), (ladder[2], forget_a_split)]
    mirror = cftuv_native.new_mirror()
    counts = dict(exact.SIGN_COUNTS)
    with exact.isolated_factorization_memory():
        before = nc.capture_state(exact.exact_work_budget(stage="COVERAGE", domain_id="poison", superlevel="L1"), {})
        wanted = _cut_chain(coverage_module._coverage_at, partition, steps)
    exact.SIGN_COUNTS.update(counts)
    with exact.isolated_factorization_memory():
        got = _cut_chain(mirror.coverage_at, pickle.loads(pickle.dumps(partition)), steps)
    for index, (want, have) in enumerate(zip(wanted, got)):
        found = nc.compare_outcomes(nc.OP_COVERAGE, before, want, have)
        assert not found, f"step {index}: " + "; ".join(str(item) for item in found[:3])
    assert any(step.result is not None and any(len(face.points) >= 3 for face in step.result.faces) for step in got)


def test_budget_and_store_effects_land_before_the_exhaustion_is_raised():
    """Исчерпание посреди ячейки: статьи, счётчики, память и запись `store` — как их оставляет исключение эталона."""

    partition = _small_partition()
    mirror = cftuv_native.new_mirror()
    for cap in range(0, 12):
        budget_a, budget_b = exact.exact_work_budget(stage="COVERAGE", cap=cap), exact.exact_work_budget(stage="COVERAGE", cap=cap)
        store_a, store_b = {}, {}
        with exact.isolated_factorization_memory():
            counts = dict(exact.SIGN_COUNTS)
            try:
                coverage_module._coverage_at(partition, Fraction(3), budget_a, store_a)
                raised_a = None
            except exact.ExactCanonicalizationWorkBudgetExhausted as exc:
                raised_a = str(exc)
            state_a = nc.capture_state(budget_a, store_a)
            exact.SIGN_COUNTS.update(counts)
        with exact.isolated_factorization_memory():
            try:
                mirror.coverage_at(partition, Fraction(3), budget_b, store_b)
                raised_b = None
            except exact.ExactCanonicalizationWorkBudgetExhausted as exc:
                raised_b = str(exc)
            state_b = nc.capture_state(budget_b, store_b)
            exact.SIGN_COUNTS.update(counts)
        assert raised_a == raised_b and state_a.budget == state_b.budget and state_a.store == state_b.store, cap

