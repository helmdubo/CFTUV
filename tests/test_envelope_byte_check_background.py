"""Первое нажатие проверяет круг кодека ФОНОМ после кнопки (SNAPSHOT_BYTE_CHECK_FIRST_PRESS_IN_BACKGROUND_V1).

Утверждения:

1. Политика: домен, выгруженный воркером на первом нажатии, откладывает круг кодека (`DEFERRED`), а не гонит его; родитель, `all`, `off` и выборка не меняются.
2. Вердикт фона РАВЕН вердикту проверки на нажатии: та же функция (`check_snapshot_bytes`), тот же `SnapshotExportRejected` (имя исхода, текст, домен), на рабочем снапшоте и на снапшоте, чьи байты не неподвижная точка кодека.
3. Нажатие не ждёт фона; найденное помечает домен, и СЛЕДУЮЩЕЕ действие отказывает его тем же исходом и текстом, что дал бы `all` на первом нажатии; чистая проверка помечает идентичность кода проверенной, оборванная - нет.
4. Пул: фоновые шаги уступают прогону (следующее действие не ждёт дольше шага), умерший воркер называется, настоящие воркеры доводят задачу до `CLEAN`.
"""

from __future__ import annotations

import pickle
import sys
import threading
import time
from pathlib import Path

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

import cftuv_envelope as kernel  # noqa: E402
from cftuv import envelope_byte_check_background as background  # noqa: E402
from cftuv import envelope_domain_pool as pool_module  # noqa: E402
from cftuv import envelope_snapshot_byte_check as byte_check  # noqa: E402
from cftuv.envelope_angle_certificate import SnapshotExportRejected, check_snapshot_bytes, validate_exported_snapshot  # noqa: E402
from cftuv.envelope_debug_profile import EnvelopeDebugProfileBuilderV1  # noqa: E402
from cftuv.envelope_debug_session import EnvelopeDebugSessionController  # noqa: E402
from cftuv.envelope_domain_pool import DomainPool, DomainPoolRunV1, order_by_cost, shutdown_domain_pool, solve_task  # noqa: E402
from cftuv.envelope_metric_export import build_envelope_patch_metric_export  # noqa: E402
from cftuv.envelope_production_export import run_production  # noqa: E402
from cftuv.envelope_snapshot_byte_check import (  # noqa: E402
    BYTE_CHECK_ALL,
    BYTE_CHECK_DEFERRED,
    BYTE_CHECK_FIRST_PRESS,
    BYTE_CHECK_SAMPLED,
    BYTE_CHECK_SKIPPED,
    MODE_ALL,
    MODE_SAMPLE,
    ByteCheckReplyV1,
    ByteCheckStepV1,
    SnapshotByteCheckPlanV1,
    finish_press,
    serve_byte_check,
    validate_under_policy,
)
from cftuv.envelope_topology_export import build_envelope_topology_export  # noqa: E402
from envelope_fixture_bundles import quad_row_bundle  # noqa: E402

ROW = 5
ALPHA = 0.25
ENVIRONMENT = byte_check.BYTE_CHECK_ENVIRONMENT
IDENTITY = ("kernel", "host", "NATIVE:build-bg")


@pytest.fixture(autouse=True)
def _a_session_that_has_checked_nothing():
    byte_check.forget_checked_identities()
    with byte_check._DEFERRED_LOCK:
        byte_check._DEFERRED.clear()
    yield
    byte_check.forget_checked_identities()
    with byte_check._DEFERRED_LOCK:
        byte_check._DEFERRED.clear()


@pytest.fixture(scope="module", autouse=True)
def _no_pool_outlives_the_module():
    yield
    shutdown_domain_pool()


def _snapshots(count=ROW):
    """Рабочие снапшоты доменов фикстуры: `{домен: снапшот}` (проверка байтов уже прошла на выгрузке)."""

    bundle = quad_row_bundle(count)
    topology = build_envelope_topology_export(bundle)
    found = {}
    for patch_id in sorted(bundle.patch_graph.nodes):
        metric = build_envelope_patch_metric_export(topology, patch_id)
        found[topology.patch_domain_id_by_patch[patch_id]] = metric.snapshot
    return found


def _not_a_fixed_point(monkeypatch):
    """Второй `dumps` каждой проверки (над декодированным объектом) даёт другие байты: «канонические байты не неподвижная точка кодека»."""

    real = kernel.AnalysisSnapshotCodecV1.dumps
    calls = {"dumps": 0}

    def broken(snapshot):
        calls["dumps"] += 1
        payload = real(snapshot)
        return payload + b" " if calls["dumps"] % 2 == 0 else payload

    monkeypatch.setattr(kernel.AnalysisSnapshotCodecV1, "dumps", staticmethod(broken))


# --------------------------------------------------------------------------
# 1. Политика
# --------------------------------------------------------------------------


def test_only_the_worker_copy_of_a_deferring_plan_defers_and_only_for_a_single_domain():
    plan = SnapshotByteCheckPlanV1(MODE_SAMPLE, True, IDENTITY, "", True, "tok")
    assert plan.decision("patch-domain:t:1") == BYTE_CHECK_FIRST_PRESS  # родитель проверяет на месте
    worker = plan.for_worker()
    assert worker.worker and worker.token == "tok" and worker.for_worker() is worker
    assert worker.decision("patch-domain:t:1") == BYTE_CHECK_DEFERRED
    assert worker.decision(None) == BYTE_CHECK_FIRST_PRESS  # снапшот не одного домена в фон не откладывается
    later = SnapshotByteCheckPlanV1(MODE_SAMPLE, False, IDENTITY, "", False, "")
    assert later.for_worker() is later
    assert byte_check.DECISIONS[BYTE_CHECK_DEFERRED] is False
    assert BYTE_CHECK_DEFERRED in byte_check.BYTE_CHECK_COUNTERS


def test_all_never_defers_and_the_first_press_defers_only_when_the_pool_can_run_steps(monkeypatch):
    monkeypatch.setattr(byte_check, "execution_identity_of", lambda *args: IDENTITY)
    every = byte_check.resolve_plan("NATIVE", environ={ENVIRONMENT: "all"}, defer=True)
    assert (every.mode, every.defer, every.token) == (MODE_ALL, False, "")
    inline = byte_check.resolve_plan("NATIVE", environ={}, defer=False)
    assert inline.first_press and not inline.defer and inline.token == ""
    deferring = byte_check.resolve_plan("NATIVE", environ={}, defer=True)
    assert deferring.first_press and deferring.defer and deferring.token
    assert deferring.token != byte_check.resolve_plan("NATIVE", environ={}, defer=True).token
    monkeypatch.setattr(byte_check, "execution_identity_of", lambda *args: None)
    unknown = byte_check.resolve_plan("NATIVE", environ={}, defer=True)
    assert unknown.first_press and not unknown.defer  # без идентичности отложенное некому засчитать


def test_a_deferred_identity_is_pending_then_checked_only_when_every_snapshot_was_verified(monkeypatch):
    monkeypatch.setattr(byte_check, "execution_identity_of", lambda *args: IDENTITY)
    plan = byte_check.resolve_plan("NATIVE", environ={}, defer=True)
    profile = EnvelopeDebugProfileBuilderV1("t", "t")
    profile.add_counter(BYTE_CHECK_DEFERRED, 1, "patch-domain:t:1")
    finish_press(profile, plan)
    assert {item.name: item.value for item in profile.snapshot().counters if item.patch_domain_id is None} == {BYTE_CHECK_DEFERRED: 1}

    between = byte_check.resolve_plan("NATIVE", environ={}, defer=True)
    assert not between.first_press  # пока фон идёт, нажатия между ними - выборка, а не второе первое

    byte_check.settle_identity(IDENTITY, plan.token, verified=False)  # фон не довёл
    assert byte_check.resolve_plan("NATIVE", environ={}, defer=True).first_press

    again = byte_check.resolve_plan("NATIVE", environ={}, defer=True)
    profile2 = EnvelopeDebugProfileBuilderV1("t", "t")
    profile2.add_counter(BYTE_CHECK_DEFERRED, 1, "patch-domain:t:1")
    finish_press(profile2, again)
    byte_check.settle_identity(IDENTITY, again.token, verified=True)
    assert not byte_check.resolve_plan("NATIVE", environ={}, defer=True).first_press


def test_a_press_that_checked_inline_marks_the_identity_as_before(monkeypatch):
    monkeypatch.setattr(byte_check, "execution_identity_of", lambda *args: IDENTITY)
    plan = byte_check.resolve_plan("NATIVE", environ={}, defer=False)
    profile = EnvelopeDebugProfileBuilderV1("t", "t")
    profile.add_counter(BYTE_CHECK_FIRST_PRESS, 1, "patch-domain:t:1")
    finish_press(profile, plan)
    assert not byte_check.resolve_plan("NATIVE", environ={}, defer=False).first_press


# --------------------------------------------------------------------------
# 2. Воркер: отложенный снапшот и вердикт, равный вердикту нажатия
# --------------------------------------------------------------------------


def _worker_plan(token="tok"):
    return SnapshotByteCheckPlanV1(MODE_SAMPLE, True, IDENTITY, "", True, token).for_worker()


def test_a_deferred_domain_runs_the_object_validators_now_and_the_codec_later(monkeypatch):
    snapshots = _snapshots(2)
    domain, snapshot = next(iter(snapshots.items()))
    calls = {"dumps": 0, "loads": 0}
    codec = kernel.AnalysisSnapshotCodecV1
    for name in calls:
        real = getattr(codec, name)

        def counted(*args, _real=real, _name=name):
            calls[_name] += 1
            return _real(*args)

        monkeypatch.setattr(codec, name, staticmethod(counted))
    profile = EnvelopeDebugProfileBuilderV1("t", "t")

    validate_under_policy(kernel, snapshot, _worker_plan(), domain, profile, None)

    assert calls == {"dumps": 0, "loads": 0}
    assert {(item.name, item.patch_domain_id): item.value for item in profile.snapshot().counters} == {(BYTE_CHECK_DEFERRED, domain): 1}
    assert byte_check.deferred_count("tok") == 1

    reply = serve_byte_check(ByteCheckStepV1("tok"))
    assert (reply.checked, reply.remaining, reply.violations) == (1, 0, ())
    assert calls["dumps"] == 2 and calls["loads"] == 1  # ровно круг кодека, который пропустило нажатие
    assert byte_check.deferred_count() == 0


def _spoil(snapshot):
    """Снапшот, который не проходит проверку ОБЪЕКТА: у физической цепочки отнят список рёбер."""

    import dataclasses

    chains = sorted(snapshot.physical_chains, key=lambda item: item.physical_chain_id.value)
    broken = dataclasses.replace(chains[0], ordered_physical_edge_ids=())
    return dataclasses.replace(snapshot, physical_chains=frozenset({broken, *chains[1:]}))


def test_a_snapshot_that_fails_the_object_check_is_refused_at_once_and_never_deferred():
    domain, snapshot = next(iter(_snapshots(2).items()))
    with pytest.raises(SnapshotExportRejected):
        validate_under_policy(kernel, _spoil(snapshot), _worker_plan(), domain, None, None)
    assert byte_check.deferred_count() == 0


def test_the_background_verdict_equals_the_verdict_of_the_press_on_a_good_and_on_a_broken_snapshot(monkeypatch):
    snapshots = _snapshots(2)
    domain, good = next(iter(snapshots.items()))

    assert validate_exported_snapshot(kernel, good) == check_snapshot_bytes(kernel, good)
    validate_under_policy(kernel, good, _worker_plan("a"), domain, None, None)
    assert serve_byte_check(ByteCheckStepV1("a")).violations == ()

    _not_a_fixed_point(monkeypatch)
    with pytest.raises(SnapshotExportRejected) as press:
        validate_exported_snapshot(kernel, good)
    validate_under_policy(kernel, good, _worker_plan("b"), domain, None, None)  # круг кодека отложен: нажатие не отказало
    (found,) = serve_byte_check(ByteCheckStepV1("b")).violations
    assert (found.outcome_name, found.message, found.patch_domain_id, found.domain_id, found.law) == (
        press.value.outcome_name,
        str(press.value),
        press.value.patch_domain_id,
        domain,
        None,
    )
    assert "fixed point of the codec" in found.message


def test_an_exception_of_the_check_is_a_named_violation_not_a_silent_disappearance(monkeypatch):
    domain, snapshot = next(iter(_snapshots(2).items()))
    validate_under_policy(kernel, snapshot, _worker_plan("c"), domain, None, "PLANE_PRESERVING_V1")

    def broken(_payload):
        raise ValueError("the codec cannot read it")

    monkeypatch.setattr(kernel.AnalysisSnapshotCodecV1, "loads", staticmethod(broken))
    (found,) = serve_byte_check(ByteCheckStepV1("c")).violations
    assert found.outcome_name == byte_check.BYTE_CHECK_RAISED_OUTCOME
    assert "ValueError" in found.message and found.law == "PLANE_PRESERVING_V1"


def test_a_step_stops_at_its_budget_with_one_check_at_least_and_forgets_a_stale_press():
    snapshots = _snapshots(4)
    old = _worker_plan("old")
    new = _worker_plan("new")
    domains = list(snapshots)
    validate_under_policy(kernel, snapshots[domains[0]], old, domains[0], None, None)
    for domain in domains[1:]:
        validate_under_policy(kernel, snapshots[domain], new, domain, None, None)
    assert byte_check.deferred_count() == 3  # регистрация нового нажатия забыла отложенное прежнего

    first = serve_byte_check(ByteCheckStepV1("new", 0.0))
    assert (first.checked, first.remaining) == (1, 2)
    rest = serve_byte_check(ByteCheckStepV1("new", 60.0))
    assert (rest.checked, rest.remaining) == (2, 0)
    assert serve_byte_check(ByteCheckStepV1("new")).checked == 0


def test_the_same_domain_exported_twice_under_two_laws_is_two_checks():
    domain, snapshot = next(iter(_snapshots(2).items()))
    plan = _worker_plan("law")
    validate_under_policy(kernel, snapshot, plan, domain, None, None)
    validate_under_policy(kernel, snapshot, plan, domain, None, "PLANE_PRESERVING_V1")
    assert byte_check.deferred_count("law") == 2


# --------------------------------------------------------------------------
# 3. Нажатие: фон после кнопки, пометка и отказ при следующем действии
# --------------------------------------------------------------------------


class _Room:
    """Пул «в процессе» с фоновыми шагами: задача идёт `solve_task` (воркерная сторона плана работает), шаги - `serve_byte_check` в том же процессе."""

    requested = 2

    def __init__(self, *, die_after=None):
        self.steps = 0
        self.die_after = die_after
        self.held_steps = threading.Event()
        self.held_steps.set()

    def run(self, tasks, cancel=None):
        results = {}
        for task, _frame in order_by_cost(tasks):
            results[task.task_id] = pickle.loads(pickle.dumps(solve_task(task)))
        return DomainPoolRunV1(results, self.requested)

    def run_idle(self, message, on_reply, *, stop, timeout=120.0):
        while not stop.is_set():
            self.held_steps.wait(10.0)
            if self.die_after is not None and self.steps >= self.die_after:
                return False, {0: "worker died"}
            self.steps += 1
            reply = serve_byte_check(ByteCheckStepV1(message.token, 0.0))
            if not on_reply(0, reply):
                return True, {}
        return False, {}


@pytest.fixture
def _room(monkeypatch):
    from cftuv import envelope_queue_pool

    monkeypatch.setattr(envelope_queue_pool, "COVERAGE_POOL_MIN_BYTES", 0)
    room = _Room()
    monkeypatch.setattr(pool_module, "get_domain_pool", lambda workers, external_python="": room if int(workers) >= pool_module.MIN_POOL_WORKERS else None)
    return room


def _press(monkeypatch, mode, *, controller=None, bundle=None, workers=2):
    monkeypatch.setenv(ENVIRONMENT, mode)
    controller = EnvelopeDebugSessionController() if controller is None else controller
    run = run_production(
        controller,
        quad_row_bundle(ROW) if bundle is None else bundle,
        frozenset(range(ROW)),
        ALPHA,
        source_object_key="object",
        source_data_key="mesh",
        density=None,
        workers=workers,
    )
    return run, controller


def _counts(run):
    return {item.name: item.value for item in run.profile.counters if item.patch_domain_id is None and item.name in byte_check.BYTE_CHECK_COUNTERS}


def _answer(run):
    return [(item.patch_id, item.outcome, item.detail, item.content_digest if item.is_materialized else "") for item in run.results]


def test_the_first_press_defers_the_codec_and_the_background_verifies_it_after_the_button(monkeypatch, _room):
    run, controller = _press(monkeypatch, MODE_SAMPLE)

    assert _counts(run) == {BYTE_CHECK_DEFERRED: ROW}  # нажатие не гнало круг кодека ни на одном домене
    assert all(item.is_materialized for item in run.results)
    job = controller.byte_check_job
    assert job is not None and job.expected == ROW and job.wait(30.0)
    assert (job.state, job.checked, job.violations) == (background.STATE_CLEAN, ROW, [])
    assert background.BYTE_CHECK_BACKGROUND_OUTCOME in job.text() and "CLEAN" in job.text()
    # Идентичность проверена целиком: следующее холодное нажатие - выборка.
    assert not byte_check.resolve_plan(run.kernel_backend, environ={}, defer=True).first_press

    # Ответ тот же, что у полной проверки на нажатии.
    full, _ = _press(monkeypatch, MODE_ALL)
    assert _answer(run) == _answer(full)
    assert _counts(full) == {BYTE_CHECK_ALL: ROW}


def test_the_press_does_not_wait_for_the_background(monkeypatch, _room):
    _room.held_steps.clear()  # шаги не идут, пока тест не разрешит
    started = time.perf_counter()
    run, controller = _press(monkeypatch, MODE_SAMPLE)
    job = controller.byte_check_job
    assert time.perf_counter() - started < 60.0 and all(item.is_materialized for item in run.results)
    assert job.state == background.STATE_RUNNING and "verifying" in job.text()
    # Пока задача идёт, идентичность «в работе»: второе нажатие не считается первым.
    assert not byte_check.resolve_plan(run.kernel_backend, environ={}, defer=True).first_press
    _room.held_steps.set()
    assert job.wait(30.0) and job.state == background.STATE_CLEAN


def test_a_domain_the_background_rejects_is_refused_at_the_next_action_exactly_as_the_press_would_have(monkeypatch, _room):
    # Эталон: полная проверка на нажатии.
    _not_a_fixed_point(monkeypatch)
    expected, _ = _press(monkeypatch, MODE_ALL, workers=0)
    assert not any(item.is_materialized for item in expected.results)
    byte_check.forget_checked_identities()

    first, controller = _press(monkeypatch, MODE_SAMPLE)
    assert all(item.is_materialized for item in first.results)  # нажатие не видело байтов: объявленное изменение поведения
    job = controller.byte_check_job
    assert job.wait(30.0) and job.state == background.STATE_VIOLATION and len(job.violations) == ROW
    assert "refused at the next action" in job.text() and "ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID" in job.text()

    second, _ = _press(monkeypatch, MODE_SAMPLE, controller=controller)  # следующее действие на той же сессии: домены в кэше, и всё равно отказаны
    assert not any(item.is_materialized for item in second.results)
    assert [(item.patch_id, item.outcome, item.detail) for item in second.results] == [(item.patch_id, item.outcome, item.detail) for item in expected.results]
    assert controller.byte_check_refused_patches(second.revision) == frozenset(range(ROW))


def test_one_broken_domain_does_not_refuse_the_others(monkeypatch, _room):
    snapshots = _snapshots()
    target = sorted(snapshots)[2]
    real = kernel.AnalysisSnapshotCodecV1.dumps
    seen = {"count": 0}

    def broken(snapshot):
        payload = real(snapshot)
        domains = {item.patch_domain_id.value for item in snapshot.patch_domains}
        if target not in domains:
            return payload
        seen["count"] += 1
        return payload + b" " if seen["count"] % 2 == 0 else payload

    monkeypatch.setattr(kernel.AnalysisSnapshotCodecV1, "dumps", staticmethod(broken))
    first, controller = _press(monkeypatch, MODE_SAMPLE)
    job = controller.byte_check_job
    assert job.wait(30.0) and job.state == background.STATE_VIOLATION
    assert [item.domain_id for item in job.violations] == [target]
    second, _ = _press(monkeypatch, MODE_SAMPLE, controller=controller)
    assert [item.is_materialized for item in second.results].count(False) == 1
    refused = next(item for item in second.results if not item.is_materialized)
    assert refused.domain_id == target and refused.outcome == "ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID"


def test_an_interrupted_background_is_incomplete_and_leaves_the_identity_unchecked(monkeypatch, _room):
    _room.die_after = 0
    run, controller = _press(monkeypatch, MODE_SAMPLE)
    job = controller.byte_check_job
    assert job.wait(30.0) and job.state == background.STATE_INCOMPLETE
    assert "worker died" in job.reason and f"0 of {ROW} domains verified" in job.reason
    assert byte_check.resolve_plan(run.kernel_backend, environ={}, defer=True).first_press  # следующее холодное нажатие проверит снова


def test_a_pool_that_cannot_run_steps_checks_the_first_press_inline_as_before(monkeypatch):
    from cftuv import envelope_queue_pool

    class PlainPool:
        """Пул без фоновых шагов (подставной, как у прежних тестов): только `run`."""

        requested = 2
        run = _Room.run

    assert not hasattr(PlainPool, "run_idle")
    monkeypatch.setattr(envelope_queue_pool, "COVERAGE_POOL_MIN_BYTES", 0)
    plain = PlainPool()
    monkeypatch.setattr(pool_module, "get_domain_pool", lambda workers, external_python="": plain)
    run, controller = _press(monkeypatch, MODE_SAMPLE)
    assert _counts(run) == {BYTE_CHECK_FIRST_PRESS: ROW} and controller.byte_check_job is None


def test_the_parent_exports_in_the_process_and_checks_inline(monkeypatch, _room):
    run, controller = _press(monkeypatch, MODE_SAMPLE, workers=0)
    assert _counts(run) == {BYTE_CHECK_FIRST_PRESS: ROW} and controller.byte_check_job is None


def test_clearing_the_session_stops_the_job_and_forgets_its_marks(monkeypatch, _room):
    _room.held_steps.clear()
    run, controller = _press(monkeypatch, MODE_SAMPLE)
    job = controller.byte_check_job
    controller.clear()
    _room.held_steps.set()
    assert job.wait(30.0) and job.state == background.STATE_INCOMPLETE and controller.byte_check_job is None
    assert controller.byte_check_refused_patches(run.revision) == frozenset()


def test_a_mark_for_a_replaced_job_is_dropped(monkeypatch, _room):
    run, controller = _press(monkeypatch, MODE_SAMPLE)
    job = controller.byte_check_job
    assert job.wait(30.0)
    other = background.ByteCheckJobV1("other", IDENTITY, 1, {}, run.revision)
    controller.start_byte_check_job(other)
    controller.refuse_byte_checked_domain(job, byte_check.ByteCheckViolationV1("patch-domain:x:1", None, "ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID", "late", None))
    assert controller.byte_check_refusal("patch-domain:x:1") is None


def test_the_incremental_subset_neither_serves_nor_skips_a_marked_patch(monkeypatch, _room):
    first, controller = _press(monkeypatch, MODE_SAMPLE)
    assert controller.byte_check_job.wait(30.0)
    domain = first.results[1].domain_id
    job = controller.byte_check_job
    job.domains[domain] = (1, None)
    controller.refuse_byte_checked_domain(job, byte_check.ByteCheckViolationV1(domain, None, "ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID", "marked", domain))
    assert controller.byte_check_refused_patches(first.revision) == frozenset({1})
    monkeypatch.setenv(ENVIRONMENT, MODE_SAMPLE)
    run = run_production(
        controller, quad_row_bundle(ROW), frozenset(range(ROW)), ALPHA, source_object_key="object", source_data_key="mesh", density=None, workers=2, lean=True,
        served={1: first.results[1]}, skipped={2},
    )
    by_patch = {item.patch_id: item for item in run.results}
    assert 2 not in by_patch  # пропущенный патч не назван вовсе, как и прежде
    assert not by_patch[1].is_materialized and by_patch[1].detail == "marked"  # помеченный не обслужен сертификатом


# --------------------------------------------------------------------------
# 4. Пул: шаги уступают прогону; настоящие воркеры
# --------------------------------------------------------------------------


class _StubWorker:
    def __init__(self, index, *, steps=3, die_on_step=None, delay=0.0):
        self.index = index
        self.dead = False
        self.steps = steps
        self.die_on_step = die_on_step
        self.delay = delay
        self.sent = 0
        self.replies = []
        self.closed = False

    def send(self, frame):
        if self.die_on_step is not None and self.sent >= self.die_on_step:
            raise pool_module._WorkerGone("stub died on send")
        self.sent += 1

    def receive(self, timeout=None):
        time.sleep(self.delay)
        return ByteCheckReplyV1("t", 1, max(0, self.steps - self.sent))

    def close(self, grace=2.0):
        self.closed = True
        self.dead = True


def _stub_pool(workers):
    pool = DomainPool(len(workers))
    pool._workers = list(workers)
    return pool


def test_idle_steps_run_until_every_worker_reports_nothing_left():
    workers = [_StubWorker(0, steps=3), _StubWorker(1, steps=1)]
    pool = _stub_pool(workers)
    seen = []
    completed, failures = pool.run_idle(ByteCheckStepV1("t"), lambda index, reply: (seen.append(index), reply.remaining > 0)[1], stop=threading.Event())
    assert completed and failures == {}
    assert (workers[0].sent, workers[1].sent) == (3, 1) and sorted(seen) == [0, 0, 0, 1]


def test_a_worker_that_dies_during_the_idle_work_is_named_and_closed():
    workers = [_StubWorker(0, steps=5), _StubWorker(1, steps=5, die_on_step=1)]
    pool = _stub_pool(workers)
    completed, failures = pool.run_idle(ByteCheckStepV1("t"), lambda index, reply: reply.remaining > 0, stop=threading.Event())
    assert not completed and list(failures) == [1] and "stub died" in failures[1]
    assert workers[1].closed and pool.worker_count == 1


def test_the_stop_event_ends_the_idle_work_between_steps():
    stop = threading.Event()
    workers = [_StubWorker(0, steps=1000)]
    pool = _stub_pool(workers)

    def on_reply(_index, reply):
        stop.set()
        return True

    completed, failures = pool.run_idle(ByteCheckStepV1("t"), on_reply, stop=stop)
    assert not completed and failures == {} and workers[0].sent == 1


def test_a_run_wanted_during_the_idle_work_goes_first_and_waits_no_longer_than_a_step(monkeypatch):
    workers = [_StubWorker(0, steps=400, delay=0.01)]
    pool = _stub_pool(workers)
    ran = {}
    monkeypatch.setattr(pool, "_run_locked", lambda tasks, cancel: (ran.setdefault("at", workers[0].sent), DomainPoolRunV1({}, 0))[1])
    outcome = {}

    def idle():
        outcome["result"] = pool.run_idle(ByteCheckStepV1("t"), lambda index, reply: reply.remaining > 0, stop=threading.Event())

    thread = threading.Thread(target=idle)
    thread.start()
    while workers[0].sent < 5:
        time.sleep(0.005)
    started = time.perf_counter()
    pool.run([])
    waited = time.perf_counter() - started
    thread.join(60.0)

    assert waited < 0.5, waited  # дождался конца шага, а не конца работы
    assert ran["at"] < 100  # прогон встал в середину работы, а не после неё
    assert outcome["result"] == (True, {}) and workers[0].sent == 400  # работа дошла до конца после прогона


def test_real_workers_verify_a_first_press_in_the_background_and_a_second_press_does_not_wait(monkeypatch):
    """Настоящие подпроцессы: отложенные снапшоты лежат в воркерах, шаги идут по трубам, задача кончается `CLEAN`, а прогон между шагами не ждёт дольше шага."""

    from cftuv import envelope_queue_pool

    monkeypatch.setattr(envelope_queue_pool, "COVERAGE_POOL_MIN_BYTES", 0)
    pool = DomainPool(2)
    try:
        monkeypatch.setattr(pool_module, "get_domain_pool", lambda workers, external_python="": pool)
        run, controller = _press(monkeypatch, MODE_SAMPLE)
        assert _counts(run) == {BYTE_CHECK_DEFERRED: ROW}
        assert all(item.is_materialized for item in run.results)
        job = controller.byte_check_job
        assert job.expected == ROW and job.wait(120.0)
        assert (job.state, job.checked) == (background.STATE_CLEAN, ROW), job.text()

        # Второе действие на живом пуле (все домены в кэше): пул свободен, прогон идёт.
        second, _ = _press(monkeypatch, MODE_SAMPLE, controller=controller)
        assert _answer(second) == _answer(run)
        assert BYTE_CHECK_DEFERRED not in _counts(second) and BYTE_CHECK_FIRST_PRESS not in _counts(second)
    finally:
        pool.close()
        shutdown_domain_pool()


# --------------------------------------------------------------------------
# 5. Строка панели: итог фона пишет таймер главного потока
# --------------------------------------------------------------------------


def _operator_watch(monkeypatch):
    """`_watch_byte_check` оператора, вынутая из дерева разбора (оператор не импортируется без Blender), с подставными `bpy` и панелью."""

    import ast
    import types

    source = (Path(__file__).resolve().parents[1] / "cftuv" / "envelope_production_operator.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    wanted = [node for node in tree.body if (isinstance(node, ast.FunctionDef) and node.name == "_watch_byte_check") or (isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") == "BYTE_CHECK_POLL_SECONDS")]
    assert len(wanted) == 2
    settings = types.SimpleNamespace(status="Decal mesh: 5 domains", timing="Decal cold 1.00 s")
    scene = types.SimpleNamespace(hotspotuv_decal_mesh=settings)
    timers = []
    redraws = []
    bpy = types.SimpleNamespace(data=types.SimpleNamespace(scenes={"Scene": scene}), app=types.SimpleNamespace(timers=types.SimpleNamespace(register=lambda function, first_interval: timers.append(function))))
    namespace = {"bpy": bpy, "SETTINGS_ATTRIBUTE": "hotspotuv_decal_mesh", "tag_view3d_redraw": lambda: redraws.append(1)}
    exec(compile(ast.Module(body=wanted, type_ignores=[]), "operator_watch", "exec"), namespace)
    return namespace["_watch_byte_check"], settings, timers, redraws


def _job(state, **fields):
    job = background.ByteCheckJobV1("tok", IDENTITY, 5, {}, "rev")
    job.state, job.checked = state, fields.get("checked", 5)
    job.violations = list(fields.get("violations", ()))
    job.reason = fields.get("reason", "")
    return job


def test_the_panel_line_follows_the_background_on_the_main_thread_timer(monkeypatch):
    watch, settings, timers, redraws = _operator_watch(monkeypatch)
    controller = type("Controller", (), {})()
    controller.byte_check_job = running = _job(background.STATE_RUNNING, checked=0)

    watch(controller, "Scene")
    (poll,) = timers
    assert poll() == 0.5 and settings.timing == "Decal cold 1.00 s"  # задача идёт: ждём
    running.state, running.checked = background.STATE_CLEAN, 5
    assert poll() is None and settings.timing.endswith("| snapshot byte check: 5 verified") and settings.status == "Decal mesh: 5 domains" and redraws

    controller.byte_check_job = None
    watch(controller, "Scene")
    assert len(timers) == 1  # задачи нет - таймер не заводится


def test_a_found_violation_or_an_incomplete_job_replaces_the_status_and_names_the_outcome(monkeypatch):
    for state, expected in ((background.STATE_VIOLATION, "VIOLATION"), (background.STATE_INCOMPLETE, "INCOMPLETE")):
        watch, settings, timers, _redraws = _operator_watch(monkeypatch)
        controller = type("Controller", (), {})()
        violation = byte_check.ByteCheckViolationV1("d", None, "ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID", "m", None)
        controller.byte_check_job = job = _job(state, violations=[violation] if state == background.STATE_VIOLATION else [], reason="worker died")
        watch(controller, "Scene")
        assert timers[0]() is None
        assert background.BYTE_CHECK_BACKGROUND_OUTCOME in settings.status and expected in settings.status, settings.status
        assert job.text() == settings.status


def test_a_line_another_action_rewrote_or_a_replaced_job_is_left_alone(monkeypatch):
    watch, settings, timers, _redraws = _operator_watch(monkeypatch)
    controller = type("Controller", (), {})()
    controller.byte_check_job = job = _job(background.STATE_CLEAN)
    watch(controller, "Scene")
    settings.status = "Decal mesh: a newer press"
    assert timers[0]() is None and "snapshot byte check" not in settings.timing and settings.status == "Decal mesh: a newer press"

    watch2, settings2, timers2, _ = _operator_watch(monkeypatch)
    controller.byte_check_job = job
    watch2(controller, "Scene")
    controller.byte_check_job = _job(background.STATE_RUNNING)  # другая задача заняла место
    assert timers2[0]() is None and settings2.timing == "Decal cold 1.00 s"
