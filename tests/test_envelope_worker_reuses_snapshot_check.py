"""Воркер не проверяет снапшот второй раз, когда это ТА ЖЕ проверка (WORKER_REUSES_ITS_SNAPSHOT_CHECK_V1).

Холодная задача воркера гнала `validate_analysis_snapshot` над одним снапшотом четыре раза:

1. `validate_exported_snapshot`, объект, БЕЗ допуска (`None` - допуск самого снапшота);
2. `validate_exported_snapshot`, ДЕКОДИРОВАННЫЙ объект (канонические байты), без допуска;
3. `build_envelope_decal_request` (`WorkerSnapshotCheck`), объект, под допуском ЗАПРОСА;
4. `compile_reference_envelopes` подготовки, тот же объект, под допуском запроса.

Первые три - три РАЗНЫЕ проверки (другой объект либо другой допуск), и четвёртая - повтор третьей. Остаётся ровно она. Утверждения:

1. Воркер делает три проверки на домен, а не четыре, и ни одна пара `(объект, допуск)` не повторяется; без повтора ответ побитово тот же.
2. Повтор пропускается, только когда он ДОКАЗАН: тот же объект (`is`), тот же допуск, замечаний не было, то же исполнение. Другой допуск, равный, но другой
   объект, другое исполнение, снапшот с замечаниями, проверки не было - подготовка проверяет сама, как раньше.
3. Отдаваемое равно свежему пересчёту на настоящих снапшотах (плоских и развёрнутых).
4. Готовые замечания в подготовку отдаёт ровно одно место хоста и только из `clean_issues_for`.
"""

from __future__ import annotations

import ast
import dataclasses
import pickle
import sys
from collections import Counter
from fractions import Fraction
from pathlib import Path

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

import cftuv_envelope as kernel  # noqa: E402
import cftuv_envelope.validation as kernel_validation  # noqa: E402
from cftuv import envelope_domain_pool as pool_module  # noqa: E402
from cftuv import envelope_production_export as production  # noqa: E402
from cftuv import envelope_production_task as task_module  # noqa: E402
from cftuv import envelope_snapshot_check as check_module  # noqa: E402
from cftuv.envelope_debug_session import EnvelopeDebugSessionController  # noqa: E402
from cftuv.envelope_domain_pool import DomainPoolRunV1, order_by_cost, shutdown_domain_pool, solve_task  # noqa: E402
from cftuv.envelope_kernel_backend import DEFAULT_KERNEL_BACKEND, entered_backend  # noqa: E402
from cftuv.envelope_production_export import run_production  # noqa: E402
from cftuv.envelope_snapshot_check import WorkerSnapshotCheck, budget_key  # noqa: E402
from envelope_fixture_bundles import quad_row_bundle  # noqa: E402

ROW = 5
ALPHA = 0.25
HOST = Path(__file__).resolve().parents[1] / "cftuv"


@pytest.fixture(scope="module", autouse=True)
def _no_pool_outlives_the_module():
    yield
    shutdown_domain_pool()


@pytest.fixture
def _pool_always(monkeypatch):
    from cftuv import envelope_queue_pool

    monkeypatch.setattr(envelope_queue_pool, "COVERAGE_POOL_MIN_BYTES", 0)


class _Room:
    """Кто сейчас считает: воркер (внутри `pool.run`) либо родитель."""

    where = "parent"


class _InProcessPool:
    """Пул «в процессе»: задача проходит пиклом туда и обратно, как труба, и считается как в воркере."""

    requested = 2

    def run(self, tasks):
        results = {}
        _Room.where = "worker"
        try:
            for task, _frame in order_by_cost(tasks):
                results[task.task_id] = pickle.loads(pickle.dumps(solve_task(task)))
        finally:
            _Room.where = "parent"
        return DomainPoolRunV1(results, self.requested)


@pytest.fixture(autouse=True)
def _parent_between_pool_runs():
    _Room.where = "parent"
    yield
    _Room.where = "parent"


def _pool_session(monkeypatch):
    pool = _InProcessPool()
    monkeypatch.setattr(pool_module, "get_domain_pool", lambda workers, external_python="": pool)
    return pool


def _press(bundle, *, workers=2, alpha=ALPHA):
    controller = EnvelopeDebugSessionController()
    run = run_production(
        controller,
        bundle,
        frozenset(range(ROW)),
        alpha,
        source_object_key="object",
        source_data_key="mesh",
        density=None,
        workers=workers,
    )
    return run, controller


def _spy_on_validation(monkeypatch):
    """`[(кто, объект снапшота, допуск)]` каждого вызова `validate_analysis_snapshot`, прямого и изнутри компиляции.

    Список держит объекты живыми: `id` декодированного объекта, освобождённого после выгрузки, достался бы следующему, и повтор был бы мнимым.
    """

    calls: list = []
    real = kernel_validation.validate_analysis_snapshot

    def spy(snapshot, **kwargs):
        calls.append((_Room.where, snapshot, kwargs.get("developable_stretch_budget")))
        return real(snapshot, **kwargs)

    monkeypatch.setattr(kernel, "validate_analysis_snapshot", spy)
    monkeypatch.setattr(kernel_validation, "validate_analysis_snapshot", spy)
    return calls


# --------------------------------------------------------------------------
# 1. Три проверки вместо четырёх, ответ тот же
# --------------------------------------------------------------------------


@pytest.mark.parametrize("lifted_corner", [0.0, 1.0], ids=["planar", "with an unfolded domain"])
def test_a_worker_runs_each_distinct_snapshot_check_once_and_the_answer_is_the_same(monkeypatch, _pool_always, lifted_corner):
    bundle = quad_row_bundle(ROW, lifted_corner=lifted_corner)
    expected, _ = _press(bundle, workers=0)
    calls = _spy_on_validation(monkeypatch)
    _pool_session(monkeypatch)

    run, _controller = _press(bundle)

    assert tuple(run.results) == tuple(expected.results)
    worker = [item for item in calls if item[0] == "worker"]
    # Три РАЗНЫЕ проверки на домен: объект без допуска, декодированный объект без допуска, объект под допуском запроса.
    assert len(worker) == 3 * ROW
    assert len({(id(snapshot), budget) for _, snapshot, budget in worker}) == len(worker), "no (object, budget) pair is checked twice"
    assert sum(budget is None for _, _, budget in worker) == 2 * ROW
    assert sum(budget is not None for _, _, budget in worker) == ROW
    # Объект под допуском запроса проверен ровно раз, и это тот же объект, что проверен без допуска (не декодированный).
    budgeted = [snapshot for _, snapshot, budget in worker if budget is not None]
    assert all(sum(snapshot is other for _, other, budget in worker if budget is None) == 1 for snapshot in budgeted)
    assert [item for item in calls if item[0] == "parent" and item[2] is not None] == []


@pytest.mark.parametrize("lifted_corner", [0.0, 1.0], ids=["planar", "with an unfolded domain"])
def test_without_the_reuse_the_worker_validates_four_times_and_the_answer_is_the_same(monkeypatch, _pool_always, lifted_corner):
    """Контроль: шпион видит прежний повтор (четвёртый вызов - тот же объект под тем же допуском), и ответ при выключенном повторе тот же."""

    bundle = quad_row_bundle(ROW, lifted_corner=lifted_corner)
    _pool_session(monkeypatch)
    reused, _ = _press(bundle)
    calls = _spy_on_validation(monkeypatch)
    monkeypatch.setattr(WorkerSnapshotCheck, "clean_issues_for", lambda self, snapshot, stretch_budget: None)

    run, _controller = _press(bundle)

    assert tuple(run.results) == tuple(reused.results)
    worker = [item for item in calls if item[0] == "worker"]
    assert len(worker) == 4 * ROW
    repeats = Counter((id(snapshot), budget) for _, snapshot, budget in worker if budget is not None)
    assert set(repeats.values()) == {2}


# --------------------------------------------------------------------------
# 2. Повтор пропускается, только когда он доказан
# --------------------------------------------------------------------------


def _worker_inputs(monkeypatch, lifted_corner=1.0):
    """`[(снапшот, запрос, проверка воркера)]` настоящих задач воркера (подготовке отдают ровно это)."""

    captured: list = []
    real = production.prepare_for_production_recorded

    def spy(snapshot, request, **kwargs):
        captured.append((snapshot, request, kwargs.get("checker")))
        return real(snapshot, request, **kwargs)

    monkeypatch.setattr(task_module, "prepare_for_production_recorded", spy)  # воркерная сторона: PRODUCTION_EXPORT_SPLIT_V1
    _pool_session(monkeypatch)
    _press(quad_row_bundle(ROW, lifted_corner=lifted_corner))
    assert len(captured) == ROW
    return captured


def _ask(checker, snapshot, budget, backend=DEFAULT_KERNEL_BACKEND):
    """`clean_issues_for` в блоке бэкенда, как зовёт его подготовка (воркер проверял под блоком прогона; вне блока исполнение другое)."""

    with entered_backend(backend):
        return checker.clean_issues_for(snapshot, budget)


def _unfolded(inputs):
    return next(
        item
        for item in inputs
        if type(next(iter(item[0].surface_metric_descriptors)).planarity_certificate).__name__ == "DevelopableUnfoldCertificateV1"
    )


def test_the_worker_hands_the_preparation_the_check_of_the_same_object_and_budget(monkeypatch, _pool_always):
    for snapshot, request, checker in _worker_inputs(monkeypatch):
        assert isinstance(checker, WorkerSnapshotCheck)
        assert _ask(checker, snapshot, request.developable_stretch_budget) == ()


def test_a_check_that_is_not_provably_the_same_is_not_reused(monkeypatch, _pool_always):
    snapshot, request, checker = _unfolded(_worker_inputs(monkeypatch))
    budget = request.developable_stretch_budget
    assert _ask(checker, snapshot, budget) == ()

    # Другой допуск: замечания зависят от него (`None` - допуск самого снапшота - тоже другая проверка).
    assert _ask(checker, snapshot, Fraction(1, 10)) is None
    assert _ask(checker, snapshot, None) is None
    # Равный, но другой объект (пикл, декодированный из байтов): `==` слеп к `1` и `1.0`, `-0.0`; доказывает только тождество.
    twin = pickle.loads(pickle.dumps(snapshot))
    assert twin == snapshot and twin is not snapshot
    assert _ask(checker, twin, budget) is None
    decoded = kernel.AnalysisSnapshotCodecV1.loads(kernel.AnalysisSnapshotCodecV1.dumps(snapshot))
    assert _ask(checker, decoded, budget) is None
    # Другое исполнение: настоящий другой бэкенд (воркер проверял под блоком прогона), другая сборка нативного ядра, другой код, исполнение не определить.
    assert _ask(checker, snapshot, budget, "PYTHON") is None
    real = check_module.active_execution
    monkeypatch.setattr(check_module, "active_execution", lambda: (*real()[:2], "NATIVE:another-build"))
    assert _ask(checker, snapshot, budget) is None
    monkeypatch.setattr(check_module, "active_execution", lambda: (real()[0][::-1], *real()[1:]))
    assert _ask(checker, snapshot, budget) is None
    monkeypatch.setattr(check_module, "active_execution", lambda: None)
    assert _ask(checker, snapshot, budget) is None
    monkeypatch.setattr(check_module, "active_execution", real)
    # Отказ ни во что не превращает запись: тот же вопрос после него снова получает ответ.
    assert _ask(checker, snapshot, budget) == ()


def test_a_check_that_never_ran_or_found_issues_is_never_reused(monkeypatch, _pool_always):
    snapshot, request, checker = _unfolded(_worker_inputs(monkeypatch))
    budget = request.developable_stretch_budget
    assert _ask(WorkerSnapshotCheck(), snapshot, budget) is None
    # Снапшот с замечаниями отдаётся запросу как есть (ради отказа), но подготовке не отдаётся никогда.
    capabilities = sorted(snapshot.analysis_capabilities, key=lambda item: item.value)
    spoiled = dataclasses.replace(snapshot, analysis_capabilities=frozenset(capabilities[1:]))
    refuser = WorkerSnapshotCheck()
    assert refuser(spoiled, budget)
    assert _ask(refuser, spoiled, budget) is None
    # Последняя проверка описывает последний вызов: тот же объект, чьё место заняла другая проверка, не принимается.
    mixed = WorkerSnapshotCheck()
    mixed(snapshot, budget)
    mixed(spoiled, budget)
    assert _ask(mixed, snapshot, budget) is None and _ask(mixed, spoiled, budget) is None


def test_the_preparation_validates_by_itself_unless_it_is_handed_the_same_check(monkeypatch, _pool_always):
    snapshot, request, checker = _unfolded(_worker_inputs(monkeypatch))
    budget = request.developable_stretch_budget
    other = WorkerSnapshotCheck()
    other(snapshot, Fraction(1, 10))

    def prepared(**kwargs):
        with monkeypatch.context() as scope:
            calls = _spy_on_validation(scope)
            result = production.prepare_for_production_recorded(snapshot, request, **kwargs)[0]
        return result, [item for item in calls if item[2] is not None]

    plain, plain_calls = prepared()
    reused, reused_calls = prepared(checker=checker)
    foreign, foreign_calls = prepared(checker=other)
    assert len(plain_calls) == 1 and plain_calls[0][1] is snapshot and plain_calls[0][2] == budget_key(budget)
    assert reused_calls == []
    assert len(foreign_calls) == 1, "a check under another budget is not this check"
    for answer in (reused, foreign):
        assert (answer.outcome, answer.detail, answer.law_names, answer.counters, answer.lattice, answer.regions) == (
            plain.outcome,
            plain.detail,
            plain.law_names,
            plain.counters,
            plain.lattice,
            plain.regions,
        )


# --------------------------------------------------------------------------
# 3. Отдаваемое равно свежему пересчёту на настоящих снапшотах
# --------------------------------------------------------------------------


@pytest.mark.parametrize("lifted_corner", [0.0, 1.0], ids=["planar", "with an unfolded domain"])
def test_the_reused_issues_equal_a_fresh_recompute_on_real_snapshots(monkeypatch, _pool_always, lifted_corner):
    inputs = _worker_inputs(monkeypatch, lifted_corner)
    kinds = set()
    for snapshot, request, checker in inputs:
        budget = request.developable_stretch_budget
        fresh = tuple(kernel.validate_analysis_snapshot(snapshot, developable_stretch_budget=budget_key(budget)))
        assert _ask(checker, snapshot, budget) == fresh == ()
        kinds.add(type(next(iter(snapshot.surface_metric_descriptors)).planarity_certificate).__name__)
    assert ("DevelopableUnfoldCertificateV1" in kinds) == (lifted_corner > 0)


# --------------------------------------------------------------------------
# 4. Готовые замечания в подготовку отдаёт одно место
# --------------------------------------------------------------------------


def _calls_with_keyword(tree, callee, keyword):
    """`[(объемлющая функция, значение аргумента)]` вызовов `callee(...)` с именованным `keyword`."""

    found: list = []

    def visit(node, stack):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            stack = (*stack, node.name)
        if isinstance(node, ast.Call):
            name = getattr(node.func, "id", getattr(node.func, "attr", None))
            for item in node.keywords:
                if name == callee and item.arg == keyword:
                    found.append((stack[0] if stack else "<module>", ast.unparse(item.value)))
        for child in ast.iter_child_nodes(node):
            visit(child, stack)

    visit(tree, ())
    return found


def test_precomputed_snapshot_issues_enter_the_preparation_only_from_the_provable_check():
    """Хост отдаёт подготовке готовые замечания в ОДНОМ месте и только как ответ `clean_issues_for`; у остальных вызовов замечаний нет."""

    sites: list = []
    for path in sorted(HOST.glob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for callee in ("prepare_conveyor", "compile_reference_envelopes"):
            sites += [(path.name, callee, *item) for item in _calls_with_keyword(tree, callee, "snapshot_issues")]
    asked = "checker and checker.clean_issues_for(snapshot, request.developable_stretch_budget)"
    assert sites == [("envelope_production_task.py", "prepare_conveyor", "prepare_for_production_recorded", asked)], sites
    # Проверку воркера пишет один класс, и запись описывает ПОСЛЕДНИЙ его вызов (объект и допуск записываются вместе с ней).
    check_source = (HOST / "envelope_snapshot_check.py").read_text(encoding="utf-8")
    call = check_source.split("    def __call__", 1)[1].split("    def clean_issues_for", 1)[0]
    assert "self.clean = " in call and "self._snapshot = snapshot" in call
