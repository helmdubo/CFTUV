"""Родитель не повторяет проверку снапшота, которую воркер уже сделал, - и только когда это та же проверка того же (ADOPT_REUSES_WORKER_SNAPSHOT_ISSUES_V1).

Холодный домен пула выгружает снапшот воркер и проверяет его (`validate_analysis_snapshot`). Родитель принимает снапшот как метрику
и собирает запрос заново, а память замечаний сессии ключуется объектом снапшота: пришедший по трубе объект новый, и проверка шла
второй раз (на кривых патчах - пересборка карты развёртки). Утверждения:

1. Ответ воркера несёт чистую проверку; родитель кладёт её в память замечаний и не зовёт `validate_analysis_snapshot` на этих снапшотах.
2. Другой допуск, другой объект снапшота, другое исполнение, другой слепок, другие ревизия или домен - проверка как раньше, исход назван.
3. Принятые замечания равны свежему пересчёту на настоящих снапшотах (плоских и развёрнутых), а пикл восстанавливает снапшот равным.
"""

from __future__ import annotations

import dataclasses
import pickle
import sys
from fractions import Fraction
from pathlib import Path

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

import cftuv_envelope as kernel  # noqa: E402
import cftuv_envelope.validation as kernel_validation  # noqa: E402
from cftuv import envelope_domain_pool as pool_module  # noqa: E402
from cftuv import envelope_snapshot_check as check_module  # noqa: E402
from cftuv.envelope_debug_session import EnvelopeDebugSessionController  # noqa: E402
from cftuv.envelope_domain_pool import DomainPoolRunV1, order_by_cost, shutdown_domain_pool, solve_task  # noqa: E402
from cftuv.envelope_kernel_backend import DEFAULT_KERNEL_BACKEND, entered_backend  # noqa: E402
from cftuv.envelope_production_export import run_production  # noqa: E402
from cftuv.envelope_snapshot_check import (  # noqa: E402
    REASON_DOMAIN_DIFFERS,
    REASON_EXECUTION_DIFFERS,
    REASON_NOT_ATTESTED,
    REASON_SNAPSHOT_DIFFERS,
    SNAPSHOT_CHECK_ADOPTED,
    SNAPSHOT_CHECK_COUNTERS,
    SnapshotCleanV1,
    WorkerSnapshotCheck,
    budget_key,
    snapshot_witness,
)
from envelope_fixture_bundles import host_exported_snapshot_paths, quad_row_bundle  # noqa: E402
from surface_adjacency_field_corpus import load_snapshot  # noqa: E402

ROW = 5
ALPHA = 0.25


@pytest.fixture(scope="module", autouse=True)
def _no_pool_outlives_the_module():
    yield
    shutdown_domain_pool()


@pytest.fixture
def _pool_always(monkeypatch):
    from cftuv import envelope_queue_pool

    monkeypatch.setattr(envelope_queue_pool, "COVERAGE_POOL_MIN_BYTES", 0)


class _Room:
    """Кто сейчас считает: воркер (внутри `pool.run`) либо родитель; пул «в процессе» проходит пиклом, как труба."""

    where = "parent"


class _RecordingPool:
    requested = 2

    def __init__(self):
        self.replies: dict = {}

    def run(self, tasks):
        results = {}
        _Room.where = "worker"
        try:
            for task, _frame in order_by_cost(tasks):
                results[task.task_id] = pickle.loads(pickle.dumps(solve_task(task)))
        finally:
            _Room.where = "parent"
        self.replies.update(results)
        return DomainPoolRunV1(results, self.requested)


@pytest.fixture(autouse=True)
def _parent_between_pool_runs():
    _Room.where = "parent"
    yield
    _Room.where = "parent"


def _pool_session(monkeypatch):
    pool = _RecordingPool()
    monkeypatch.setattr(pool_module, "get_domain_pool", lambda workers, external_python="": pool)
    return pool


def _press(bundle, controller=None, *, workers=2, alpha=ALPHA):
    controller = controller or EnvelopeDebugSessionController()
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


def _spy_on_snapshot_validation(monkeypatch):
    """`[(кто, id снапшота, допуск)]` вызовов `validate_analysis_snapshot` и внутренних вызовов `validate_snapshot_request_references`."""

    calls: list = []
    internal: list = []
    real = kernel.validate_analysis_snapshot
    real_internal = kernel_validation.validate_analysis_snapshot

    def spy(snapshot, **kwargs):
        calls.append((_Room.where, id(snapshot), kwargs.get("developable_stretch_budget")))
        return real(snapshot, **kwargs)

    def spy_internal(snapshot, **kwargs):
        internal.append((_Room.where, id(snapshot), kwargs.get("developable_stretch_budget")))
        return real_internal(snapshot, **kwargs)

    monkeypatch.setattr(kernel, "validate_analysis_snapshot", spy)
    monkeypatch.setattr(kernel_validation, "validate_analysis_snapshot", spy_internal)
    return calls, internal


def _budgeted(calls, where):
    return [item for item in calls if item[0] == where and item[2] is not None]


def _parent_snapshots(controller):
    return [entry[0] for entry in controller._snapshot_issues.values()]


# --------------------------------------------------------------------------
# 1. Совпало - родитель не проверяет
# --------------------------------------------------------------------------


@pytest.mark.parametrize("lifted_corner", [0.0, 1.0], ids=["planar", "with an unfolded domain"])
def test_the_parent_does_not_validate_the_snapshots_a_worker_validated_clean(monkeypatch, _pool_always, lifted_corner):
    bundle = quad_row_bundle(ROW, lifted_corner=lifted_corner)
    expected, _ = _press(bundle, workers=0)
    calls, internal = _spy_on_snapshot_validation(monkeypatch)
    pool = _pool_session(monkeypatch)
    run, controller = _press(bundle)

    assert tuple(run.results) == tuple(expected.results)
    # Воркер проверил каждый снапшот ровно один раз под допуском запроса, родитель - ни одного.
    assert len(_budgeted(calls, "worker")) == ROW
    assert _budgeted(calls, "parent") == []
    # Ни прямо, ни изнутри `validate_snapshot_request_references`. (Воркер проверяет ещё раз в `compile_reference_envelopes` - это его время, не родителя.)
    assert [item for item in internal if item[0] == "parent"] == []
    assert run.counter(SNAPSHOT_CHECK_ADOPTED) == ROW
    assert all((run.counter(name) or 0) == 0 for reason, name in SNAPSHOT_CHECK_COUNTERS.items() if reason)
    assert {entry[0].source_revision.value for entry in controller._snapshot_issues.values()} == {
        reply.snapshot.source_revision.value for reply in pool.replies.values()
    }
    # Тёплый шаг ширины: ни одной проверки ни у кого.
    before = len(calls)
    warm, _ = _press(bundle, controller, alpha=0.5)
    assert len(calls) == before and warm.counter(SNAPSHOT_CHECK_ADOPTED) is None


def test_the_adopted_issues_equal_a_fresh_recompute_on_real_snapshots(monkeypatch, _pool_always):
    bundle = quad_row_bundle(ROW, lifted_corner=1.0)
    _pool_session(monkeypatch)
    run, controller = _press(bundle)
    assert run.counter(SNAPSHOT_CHECK_ADOPTED) == ROW

    entries = list(controller._snapshot_issues.items())
    assert len(entries) == ROW
    kinds = set()
    for (identity, budget), (snapshot, issues) in entries:
        assert identity == id(snapshot)
        fresh = tuple(kernel.validate_analysis_snapshot(snapshot, developable_stretch_budget=budget))
        assert controller.snapshot_issues(snapshot, budget) == fresh == issues == ()
        kinds.add(type(next(iter(snapshot.surface_metric_descriptors)).planarity_certificate).__name__)
    assert kinds == {"ExactSourcePlaneCertificateV1", "DevelopableUnfoldCertificateV1"}


def test_the_worker_check_is_the_call_the_request_would_make_itself(monkeypatch, _pool_always):
    """Один вызов вместо внутреннего: тот же аргумент, те же замечания, что у `validate_snapshot_request_references`."""

    bundle = quad_row_bundle(ROW)
    pool = _pool_session(monkeypatch)
    run, controller = _press(bundle)
    for reply in pool.replies.values():
        snapshot, clean = reply.snapshot, reply.snapshot_check
        assert isinstance(clean, SnapshotCleanV1) and clean.witness == snapshot_witness(snapshot)
        direct = kernel_validation.validate_analysis_snapshot(snapshot, developable_stretch_budget=clean.budget)
        assert tuple(direct) == ()
    recorder = WorkerSnapshotCheck()
    snapshot = next(iter(pool.replies.values())).snapshot
    budget = Fraction(7, 20)
    assert recorder(snapshot, budget) == tuple(kernel.validate_analysis_snapshot(snapshot, developable_stretch_budget=budget))
    assert recorder.clean is not None and recorder.clean.budget == budget_key(budget) == budget
    # Снапшот с замечаниями отдаётся запросу как есть (ради отказа), но записью чистоты не становится.
    capabilities = sorted(snapshot.analysis_capabilities, key=lambda item: item.value)
    spoiled = dataclasses.replace(snapshot, analysis_capabilities=frozenset(capabilities[1:]))
    refuser = WorkerSnapshotCheck()
    issues = refuser(spoiled, budget)
    assert issues and issues == tuple(kernel.validate_analysis_snapshot(spoiled, developable_stretch_budget=budget))
    assert refuser.clean is None


# --------------------------------------------------------------------------
# 2. Не совпало - проверка как раньше, исход назван
# --------------------------------------------------------------------------


def _unfolded_reply(monkeypatch):
    """Ответ воркера для развёрнутого (кривого) домена: тот, ради которого проверка и дорога."""

    pool = _pool_session(monkeypatch)
    _press(quad_row_bundle(ROW, lifted_corner=1.0))
    return next(
        reply
        for reply in pool.replies.values()
        if type(next(iter(reply.snapshot.surface_metric_descriptors)).planarity_certificate).__name__ == "DevelopableUnfoldCertificateV1"
    )


def _adopt(controller, reply, *, clean=..., snapshot=None, revision=None, domain_id=None):
    """`adopt_snapshot_check` с тем, что заказал бы родитель (ревизия и домен снапшота), кроме названного."""

    witness = reply.snapshot_check.witness
    with entered_backend(DEFAULT_KERNEL_BACKEND):  # родитель принимает ответ внутри блока бэкенда прогона, как `_adopt_cold`
        return controller.adopt_snapshot_check(
            reply.snapshot if snapshot is None else snapshot,
            reply.snapshot_check if clean is ... else clean,
            revision=witness[0] if revision is None else revision,
            domain_id=witness[1][0] if domain_id is None else domain_id,
        )


def test_a_matching_check_is_taken_and_a_different_budget_or_object_is_validated_again(monkeypatch, _pool_always):
    reply = _unfolded_reply(monkeypatch)
    controller = EnvelopeDebugSessionController()
    calls, _ = _spy_on_snapshot_validation(monkeypatch)
    snapshot, budget = reply.snapshot, reply.snapshot_check.budget

    assert _adopt(controller, reply) == ""
    assert controller.snapshot_issues(snapshot, budget) == () and calls == []
    # Другой допуск: замечания зависят от него, памяти под таким ключом нет.
    controller.snapshot_issues(snapshot, Fraction(1, 10))
    assert [item[2] for item in calls] == [Fraction(1, 10)]
    # Равный, но другой объект: тождество не совпало, проверка идёт.
    twin = pickle.loads(pickle.dumps(snapshot))
    assert twin == snapshot and twin is not snapshot
    controller.snapshot_issues(twin, budget)
    assert [item[1] for item in calls][-1] == id(twin)
    # Допуск самого снапшота (`None`) - тоже другой ключ.
    controller.snapshot_issues(snapshot, None)
    assert calls[-1][2] is None


@pytest.mark.parametrize(
    ("change", "reason"),
    [
        ("no record", REASON_NOT_ATTESTED),
        ("another execution", REASON_EXECUTION_DIFFERS),
        ("another kernel fingerprint", REASON_EXECUTION_DIFFERS),
        ("another witness", REASON_SNAPSHOT_DIFFERS),
        ("a snapshot with a capability less", REASON_SNAPSHOT_DIFFERS),
        ("another revision asked", REASON_DOMAIN_DIFFERS),
        ("another domain asked", REASON_DOMAIN_DIFFERS),
    ],
)
def test_a_check_that_is_not_provably_the_same_is_refused_by_name_and_nothing_is_remembered(monkeypatch, _pool_always, change, reason):
    reply = _unfolded_reply(monkeypatch)
    controller = EnvelopeDebugSessionController()
    clean = reply.snapshot_check
    arguments: dict = {}
    if change == "no record":
        arguments["clean"] = None
    elif change == "another execution":
        arguments["clean"] = dataclasses.replace(clean, execution=(*clean.execution[:2], "NATIVE:another-build"))
    elif change == "another kernel fingerprint":
        arguments["clean"] = dataclasses.replace(clean, execution=("0" * 16, *clean.execution[1:]))
    elif change == "another witness":
        arguments["clean"] = dataclasses.replace(clean, witness=(*clean.witness[:2], (0,) * len(clean.witness[2]), clean.witness[3]))
    elif change == "a snapshot with a capability less":
        capabilities = sorted(reply.snapshot.analysis_capabilities, key=lambda item: item.value)
        arguments["snapshot"] = dataclasses.replace(reply.snapshot, analysis_capabilities=frozenset(capabilities[1:]))
    elif change == "another revision asked":
        arguments["revision"] = "another-revision"
    else:
        arguments["domain_id"] = "another-domain"

    assert _adopt(controller, reply, **arguments) == reason
    assert not controller._snapshot_issues
    calls, _ = _spy_on_snapshot_validation(monkeypatch)
    snapshot = arguments.get("snapshot", reply.snapshot)
    issues = controller.snapshot_issues(snapshot, clean.budget)
    assert len(calls) == 1, "the parent validates the snapshot itself"
    assert issues == tuple(kernel_validation.validate_analysis_snapshot(snapshot, developable_stretch_budget=clean.budget))
    assert bool(issues) == (change == "a snapshot with a capability less")


def test_a_parent_with_another_execution_validates_as_before_and_names_it(monkeypatch, _pool_always):
    """Исполнение воркера и родителя разошлись (другая сборка нативного ядра, другой код): родитель проверяет сам, а исход назван."""

    bundle = quad_row_bundle(ROW, lifted_corner=1.0)
    expected, _ = _press(bundle, workers=0)
    real = check_module.active_execution
    monkeypatch.setattr(
        check_module,
        "active_execution",
        lambda: real() if _Room.where == "worker" else (*real()[:2], "NATIVE:the parent builds elsewhere"),
    )
    calls, _ = _spy_on_snapshot_validation(monkeypatch)
    _pool_session(monkeypatch)

    run, controller = _press(bundle)

    assert tuple(run.results) == tuple(expected.results)
    assert run.counter(SNAPSHOT_CHECK_COUNTERS[REASON_EXECUTION_DIFFERS]) == ROW
    assert run.counter(SNAPSHOT_CHECK_ADOPTED) is None
    assert len(_budgeted(calls, "parent")) == ROW
    assert len(_budgeted(calls, "worker")) == ROW


def test_a_domain_the_worker_did_not_export_is_validated_by_the_parent(monkeypatch, _pool_always):
    """Метрика домена уже в кэше (воркеру его не отдают) либо воркер не вернул ответ: родитель выгружает и проверяет сам."""

    bundle = quad_row_bundle(ROW)
    calls, _ = _spy_on_snapshot_validation(monkeypatch)
    run, controller = _press(bundle, workers=0)
    assert run.counter(SNAPSHOT_CHECK_ADOPTED) is None
    assert len(_budgeted(calls, "parent")) == ROW and _budgeted(calls, "worker") == []


def test_the_debug_build_on_a_pool_does_not_adopt_checks_and_its_profile_does_not_know_the_placement(monkeypatch, _pool_always):
    """Отладка не читает память замечаний, и её профиль (отпечаток) от размещения не зависит: ни записи, ни счётчика."""

    from cftuv.envelope_debug_profile import EnvelopeDebugProfileBuilderV1
    from cftuv.envelope_debug_session import evaluate_envelope_debug_staged

    _pool_session(monkeypatch)
    controller = EnvelopeDebugSessionController()
    profile = EnvelopeDebugProfileBuilderV1("row", "QUEUE")
    evaluate_envelope_debug_staged(
        quad_row_bundle(ROW),
        frozenset(range(ROW)),
        ALPHA,
        profile=profile,
        controller=controller,
        source_object_key="object",
        source_data_key="mesh",
        engine="QUEUE",
        density=None,
        workers=2,
    )
    assert not controller._snapshot_issues
    assert not [item for item in profile.snapshot().counters if item.name.startswith("PRODUCTION_SNAPSHOT_CHECK")]


def test_a_verdict_enters_the_snapshot_issues_memory_only_by_validation_or_by_an_adopted_worker_check():
    """Два писателя и никого больше: `snapshot_issues` (родитель проверил сам) и `adopt_snapshot_check` (после `refusal_reason`)."""

    import ast

    allowed = {("envelope_debug_session.py", "snapshot_issues"), ("envelope_debug_session.py", "adopt_snapshot_check")}
    found: set = set()
    for path in sorted((Path(__file__).resolve().parents[1] / "cftuv").glob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for function in ast.walk(tree):
            if not isinstance(function, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            for node in ast.walk(function):
                writes_memory = (
                    isinstance(node, ast.Call) and getattr(node.func, "attr", None) == "_remember_snapshot_issues"
                ) or (
                    isinstance(node, ast.Subscript)
                    and isinstance(node.ctx, ast.Store)
                    and getattr(node.value, "attr", None) == "_snapshot_issues"
                )
                if writes_memory:
                    found.add((path.name, function.name))
    assert found - {("envelope_debug_session.py", "_remember_snapshot_issues")} == allowed, found
    controller_source = (Path(__file__).resolve().parents[1] / "cftuv" / "envelope_debug_session.py").read_text(encoding="utf-8")
    adopt = controller_source.split("def adopt_snapshot_check", 1)[1].split("\n    def ", 1)[0]
    assert adopt.index("refusal_reason(") < adopt.index("_remember_snapshot_issues("), "the verdict is remembered only after the identity check"


# --------------------------------------------------------------------------
# 3. Пикл возвращает снапшот равным
# --------------------------------------------------------------------------


#: Снапшоты с настоящими картами развёртки (кривые патчи поля): проверка на них и есть та, ради которой родитель её повторял.
UNFOLDED_FIELD_SNAPSHOTS = ("building_patch89_fold_miter_v1", "sagging_wall_convex_partition_v1", "wall_noise_top_rung_clip_v1")
FIELD_SNAPSHOT_PATHS = (
    *host_exported_snapshot_paths(),
    *(Path(__file__).resolve().parents[1] / "kernel" / "fixtures" / name / "analysis_snapshot.json" for name in UNFOLDED_FIELD_SNAPSHOTS),
)


@pytest.mark.parametrize("path", FIELD_SNAPSHOT_PATHS, ids=lambda path: path.parent.name)
def test_a_pickled_field_snapshot_is_equal_and_validates_alike(path):
    """Настоящие снапшоты корпуса, и кривые тоже: после пикла снапшот равен, слепок и канонический дайджест те же, замечания те же."""

    snapshot = load_snapshot(path.parent)
    pickled = pickle.loads(pickle.dumps(snapshot, protocol=pool_module.PICKLE_PROTOCOL))

    assert pickled == snapshot and pickled is not snapshot
    assert snapshot_witness(pickled) == snapshot_witness(snapshot)
    assert kernel.snapshot_digest(pickled) == kernel.snapshot_digest(snapshot)
    assert kernel.validate_analysis_snapshot(pickled) == ()
    for budget in (None, Fraction(1, 5), Fraction(7, 20)):
        assert kernel.validate_analysis_snapshot(pickled, developable_stretch_budget=budget) == kernel.validate_analysis_snapshot(
            snapshot, developable_stretch_budget=budget
        )
