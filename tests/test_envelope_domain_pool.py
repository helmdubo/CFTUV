"""Пул процессов по доменам очереди: кадры, порядок, отказы и РЕАЛЬНЫЙ прогон.

Утверждение среза одно: пул меняет ГДЕ считается домен, а не ЧТО он отвечает.
Поэтому главное здесь — не ускорение (оно измерено на поле, см. DECISIONS.md),
а побитовое равенство последовательному пути и названный исход на каждом
отказе: пул, который не стартовал, и задача, которая упала, не исчезают
молча — они счётчик, строка консоли и диагностика, а домен досчитывается тем
же кодом в родителе.

Тесты без подпроцессов проверяют склейку (`envelope_queue_pool`) пулом «в
процессе», а сам подпроцесс — несколько настоящих запусков с `sys.executable`.
Blender не нужен: воркер поднимает пакет хоста сам, по переданным путям.
"""

from __future__ import annotations

import io
import pickle
import re
import sys
import time
from pathlib import Path

import pytest


KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_domain_pool as pool_module  # noqa: E402
from cftuv.envelope_debug_profile import (  # noqa: E402
    EnvelopeDebugProfileBuilderV1,
)
from cftuv.envelope_debug_session import (  # noqa: E402
    EnvelopeDebugSessionController,
    evaluate_envelope_debug_staged,
)
from cftuv.envelope_domain_pool import (  # noqa: E402
    DomainPool,
    DomainPoolRunV1,
    DomainPoolUnavailable,
    DomainTaskResultV1,
    DomainTaskV1,
    encode_frame,
    get_domain_pool,
    order_by_cost,
    read_frame,
    resolve_python_executable,
    shutdown_domain_pool,
    solve_task,
    write_frame,
)
from cftuv.envelope_queue_export import (  # noqa: E402
    POOL_DISPATCHED,
    POOL_TASK_FALLBACK,
    POOL_UNAVAILABLE,
    POOL_WALL_STAGE,
    POOL_WORKERS,
    build_queue_scene,
    evaluate_envelope_queue_staged,
    queue_scene_payload,
    queue_timing_text,
    run_queue_domain,
)
from cftuv.envelope_request_export import (  # noqa: E402
    build_envelope_analysis_snapshot,
    build_envelope_decal_request,
)
from cftuv.envelope_topology_export import (  # noqa: E402
    build_envelope_topology_export,
)
from envelope_fixture_bundles import quad_row_bundle  # noqa: E402


ROW = 5
POOL_NAMES = (
    POOL_UNAVAILABLE,
    POOL_TASK_FALLBACK,
    POOL_WORKERS,
    POOL_DISPATCHED,
)


@pytest.fixture(scope="module", autouse=True)
def _no_pool_outlives_the_module():
    yield
    shutdown_domain_pool()


# --------------------------------------------------------------------------
# Сравнение прогонов: всё, кроме секунд
# --------------------------------------------------------------------------

_MS = re.compile(r"\d+(?:\.\d+)? ms")


def _timing_free_scene_payload(evaluation):
    payload = queue_scene_payload(build_queue_scene(evaluation.queue_domains))
    for domain in payload["domains"]:
        for key in (
            "prepare_seconds",
            "coverage_seconds",
            "contour_seconds",
            "timings",
        ):
            domain.pop(key)
    return payload


def _fingerprint(evaluation, profile, *, skip_scene_of=()):
    """Ответ прогона без времени.

    Секунды стоят в трёх местах: в записях домена, в таймингах ядра и ТЕКСТОМ
    в сообщении квитанции (`prepare 7.5 ms`), поэтому текст чистится так же,
    как числа. Счётчики пула сравнению не подлежат: у последовательного
    прогона их нет вовсе, и это объявленное различие, а не случайное.
    """

    domains = evaluation.domains
    return {
        "scene": _timing_free_scene_payload(evaluation),
        "geometry": [
            (item.queue.regions, item.queue.faces, item.queue.segments)
            for item in domains
            if item.queue is not None
        ],
        "receipts": [
            (
                item.patch_id,
                item.patch_domain_id,
                item.stage,
                item.outcome,
                _MS.sub("<ms>", item.message),
            )
            for item in evaluation.receipts
        ],
        "diagnostics": [
            (
                str(getattr(item.outcome, "value", item.outcome)),
                item.severity,
                item.message,
                item.patch_domain_id,
            )
            for item in evaluation.diagnostics
            if str(getattr(item.outcome, "value", item.outcome))
            not in POOL_NAMES
        ],
        "counters": [
            (item.name, item.value, item.patch_domain_id)
            for item in profile.snapshot().counters
            if item.name not in POOL_NAMES
        ],
        "debug_scenes": [
            item.debug_scene
            for item in domains
            if item.patch_domain_id not in skip_scene_of
        ],
    }


def _counter(profile, name):
    values = [
        item.value
        for item in profile.snapshot().counters
        if item.name == name and item.patch_domain_id is None
    ]
    return values[0] if values else None


def _session_run(bundle, alpha=0.25, *, workers, controller=None):
    controller = controller or EnvelopeDebugSessionController()
    profile = EnvelopeDebugProfileBuilderV1("row", "QUEUE")
    evaluation = evaluate_envelope_debug_staged(
        bundle,
        frozenset(range(ROW)),
        alpha,
        profile=profile,
        controller=controller,
        source_object_key="object",
        source_data_key="mesh",
        engine="QUEUE",
        density=None,
        workers=workers,
    )
    return evaluation, profile, controller


def _direct_run(bundle, *, pool, alpha=0.25):
    profile = EnvelopeDebugProfileBuilderV1("row", "QUEUE")
    evaluation = evaluate_envelope_queue_staged(
        bundle,
        frozenset(range(ROW)),
        alpha,
        profile=profile,
        topology_export=build_envelope_topology_export(bundle),
        domain_pool=pool,
        density=None,
    )
    return evaluation, profile


class _InProcessPool:
    """Пул без подпроцессов: та же `solve_task`, ответ через pickle, как по трубе.

    Нужен, чтобы ИНЖЕКТИРОВАТЬ отказ задачи и гибель воркера там, где склейка
    обязана их назвать, не ломая настоящие процессы.
    """

    requested = 2

    def __init__(self, *, fail=(), drop=()):
        self.fail = frozenset(fail)
        self.drop = frozenset(drop)
        self.dispatched: list[int] = []

    def run(self, tasks):
        results = {}
        for task, _frame in order_by_cost(tasks):
            self.dispatched.append(task.task_id)
            if task.task_id in self.drop:
                continue
            if task.task_id in self.fail:
                results[task.task_id] = DomainTaskResultV1(
                    task.task_id,
                    error="Traceback (most recent call last):\n"
                    "InjectedTaskFailure: boom",
                )
                continue
            results[task.task_id] = pickle.loads(pickle.dumps(solve_task(task)))
        return DomainPoolRunV1(results, self.requested)


# --------------------------------------------------------------------------
# 1. Кадры, порядок, имя интерпретатора
# --------------------------------------------------------------------------


def test_frames_round_trip_and_clean_end_differs_from_truncation():
    values = (("ready", 7), {"edges": frozenset({1, 2, 3})}, b"x" * 200_000)
    stream = io.BytesIO()
    for value in values:
        write_frame(stream, value)
    stream.seek(0)

    assert [read_frame(stream) for _ in values] == list(values)
    assert read_frame(stream) is None

    frame = encode_frame(("ready", 7))
    for cut in (3, len(frame) - 2):
        with pytest.raises(EOFError):
            read_frame(io.BytesIO(frame[:cut]))


def _task(task_id, weight):
    return DomainTaskV1(
        task_id, task_id, f"d{task_id}", "x" * weight, None, "0.25", frozenset()
    )


def test_tasks_are_ordered_heaviest_first_and_ties_by_task_id():
    tasks = [_task(0, 10), _task(1, 5000), _task(2, 10), _task(3, 900)]

    ordered = order_by_cost(tasks)

    assert [task.task_id for task, _ in ordered] == [1, 3, 0, 2]
    assert all(frame == encode_frame(task) for task, frame in ordered)


def test_a_python_executable_is_refused_by_name_not_guessed():
    for accepted in ("python", "python.exe", "C:/x/python3.11", "python3"):
        assert resolve_python_executable(accepted) == accepted
    assert resolve_python_executable()  # pytest: sys.executable is Python
    for refused in (
        "",
        "C:/Program Files/Blender Foundation/Blender 4.5/blender.exe",
        "pythonw.exe",
        "python-wrapper",
    ):
        with pytest.raises(DomainPoolUnavailable) as caught:
            resolve_python_executable(refused)
        assert "not a Python interpreter" in str(caught.value)


def test_the_shared_pool_is_lazy_resized_and_off_below_two_workers():
    assert get_domain_pool(0) is None
    assert get_domain_pool(1) is None
    three = get_domain_pool(3)
    assert three.requested == 3 and three.worker_count == 0
    assert get_domain_pool(3) is three
    resized = get_domain_pool(4)
    assert resized is not three and resized.requested == 4
    assert get_domain_pool(0) is None
    shutdown_domain_pool()


# --------------------------------------------------------------------------
# 2. Воркер: ошибка задачи — запись, а не пропажа
# --------------------------------------------------------------------------


def test_a_task_that_cannot_run_returns_its_trace_instead_of_raising():
    result = solve_task(_task(9, 10))

    assert not result.ok
    assert result.task_id == 9
    assert result.prepared is None and result.queue_domain is None
    assert "Traceback" in result.error


# --------------------------------------------------------------------------
# 3. Склейка фаз без подпроцессов: равенство, отказ задачи, гибель воркера
# --------------------------------------------------------------------------


def test_the_in_process_pool_reproduces_the_sequential_run_exactly():
    bundle = quad_row_bundle(ROW, lifted_corner=0.05)
    expected, expected_profile = _direct_run(bundle, pool=None)
    pool = _InProcessPool()

    evaluation, profile = _direct_run(bundle, pool=pool)

    assert _fingerprint(evaluation, profile) == _fingerprint(
        expected, expected_profile
    )
    # Четыре точных домена и один отказ выгрузки (метрика): отказавший домен
    # воркеру не уходит, он разбирается тем же обработчиком, что и раньше.
    assert len(pool.dispatched) == ROW - 1
    assert _counter(profile, POOL_DISPATCHED) == ROW - 1
    assert _counter(profile, POOL_WORKERS) == 2
    assert _counter(profile, POOL_TASK_FALLBACK) == 0
    assert _counter(profile, POOL_UNAVAILABLE) == 0
    assert _counter(expected_profile, POOL_WORKERS) is None
    assert POOL_WALL_STAGE in profile.snapshot().stage_totals
    assert POOL_WALL_STAGE not in expected_profile.snapshot().stage_totals
    assert "NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED" in {
        item.outcome for item in evaluation.receipts
    }


def test_the_wall_clock_of_the_pool_phase_reaches_the_owner_text():
    bundle = quad_row_bundle(ROW)
    evaluation, profile = _direct_run(bundle, pool=_InProcessPool())
    scene = build_queue_scene(evaluation.queue_domains)

    text = queue_timing_text(scene, profile.snapshot())

    assert "pool wall" in text and "on 2 workers" in text
    assert "pool wall" not in queue_timing_text(scene)


@pytest.mark.parametrize("injection", ("fail", "drop"))
def test_a_failed_task_is_named_counted_and_recomputed_sequentially(
    injection, capsys
):
    bundle = quad_row_bundle(ROW)
    expected, expected_profile = _direct_run(bundle, pool=None)
    victim_task = 2
    pool = _InProcessPool(**{injection: (victim_task,)})

    evaluation, profile = _direct_run(bundle, pool=pool)

    victim = evaluation.domains[victim_task]
    assert _counter(profile, POOL_TASK_FALLBACK) == 1
    assert _counter(profile, POOL_DISPATCHED) == ROW
    assert _counter(profile, POOL_UNAVAILABLE) == 0
    notices = [
        item
        for item in victim.diagnostics
        if item.outcome == POOL_TASK_FALLBACK
    ]
    assert len(notices) == 1
    assert notices[0].patch_domain_id == victim.patch_domain_id
    assert "recomputed sequentially" in notices[0].message
    assert (
        "InjectedTaskFailure: boom" in notices[0].message
        if injection == "fail"
        else "every worker died" in notices[0].message
    )
    console = capsys.readouterr().out
    assert POOL_TASK_FALLBACK in console
    assert victim.patch_domain_id[-3:] in console
    # Ответ — тот же: домен досчитан тем же кодом в родителе. Сцена самого
    # отказавшего домена несёт ещё и строку о пуле, поэтому её сверять нельзя.
    assert _fingerprint(
        evaluation, profile, skip_scene_of=(victim.patch_domain_id,)
    ) == _fingerprint(
        expected, expected_profile, skip_scene_of=(victim.patch_domain_id,)
    )
    assert victim.queue is not None and victim.queue.is_exact


def test_an_unavailable_pool_is_named_and_the_run_stays_sequential(capsys):
    bundle = quad_row_bundle(ROW)
    expected, expected_profile = _direct_run(bundle, pool=None)
    broken = DomainPool(2, python_executable="C:/nowhere/python.exe")

    evaluation, profile = _direct_run(bundle, pool=broken)

    assert _counter(profile, POOL_UNAVAILABLE) == 1
    assert _counter(profile, POOL_WORKERS) == 0
    assert _counter(profile, POOL_DISPATCHED) == 0
    assert _counter(profile, POOL_TASK_FALLBACK) == 0
    first = evaluation.domains[0]
    notices = [
        item for item in evaluation.diagnostics if item.outcome == POOL_UNAVAILABLE
    ]
    assert [item.patch_domain_id for item in notices] == [first.patch_domain_id]
    assert "cannot start" in notices[0].message
    console = capsys.readouterr().out
    assert POOL_UNAVAILABLE in console and "sequentially" in console
    assert _fingerprint(
        evaluation, profile, skip_scene_of=(first.patch_domain_id,)
    ) == _fingerprint(
        expected, expected_profile, skip_scene_of=(first.patch_domain_id,)
    )
    text = queue_timing_text(
        build_queue_scene(evaluation.queue_domains), profile.snapshot()
    )
    assert "pool unavailable" in text


def test_an_interpreter_refused_by_name_is_unavailable_with_its_reason():
    evaluation, profile = _direct_run(
        quad_row_bundle(ROW),
        pool=DomainPool(2, python_executable="C:/x/blender.exe"),
    )

    assert _counter(profile, POOL_UNAVAILABLE) == 1
    assert any(
        item.outcome == POOL_UNAVAILABLE
        and "not a Python interpreter" in item.message
        for item in evaluation.diagnostics
    )
    assert all(item.queue is not None for item in evaluation.domains)


# --------------------------------------------------------------------------
# 4. Настоящие подпроцессы
# --------------------------------------------------------------------------


def test_a_worker_that_dies_during_start_names_its_stderr(monkeypatch):
    monkeypatch.setattr(
        pool_module, "_BOOTSTRAP", "import sys\nsys.exit('boom from bootstrap')"
    )
    pool = DomainPool(2)

    with pytest.raises(DomainPoolUnavailable) as caught:
        pool.ensure_started()

    assert "boom from bootstrap" in str(caught.value)
    assert pool.worker_count == 0
    pool.close()


def test_the_worker_starts_under_a_foreign_package_name(monkeypatch):
    """Blender 4.2+ грузит аддон как `bl_ext.<репозиторий>.<id>`.

    Такого имени нет ни в каком `sys.path`, а родителей `bl_ext` и
    `bl_ext.user_default` в `sys.modules` воркера нет вовсе, поэтому он
    поднимает пакет по ФАЙЛУ и под тем именем, которое ему передали;
    относительные импорты внутри него обязаны работать при любом имени.
    """

    real = pool_module._worker_specification()
    foreign = "bl_ext.user_default.cftuv"
    assert "." in foreign and foreign not in sys.modules
    assert "bl_ext" not in sys.modules
    monkeypatch.setattr(
        pool_module,
        "_worker_specification",
        lambda: {**real, "package": foreign},
    )
    pool = DomainPool(1)
    try:
        pool.ensure_started()
        assert pool.worker_count == 1
    finally:
        pool.close()


def test_noise_on_the_worker_stdout_cannot_corrupt_the_frames(monkeypatch):
    """Печать и запись в fd 1 после старта воркера уходят в stderr, не в кадры."""

    last = (
        'importlib.import_module(spec["package"] + ".envelope_domain_pool")'
        ".worker_main()"
    )
    noisy = "\n".join(
        (
            'pool = importlib.import_module(spec["package"] + '
            '".envelope_domain_pool")',
            'export = importlib.import_module(spec["package"] + '
            '".envelope_queue_export")',
            "export.load_queue_kernel = lambda: (",
            '    print("noise from print"),',
            '    os.write(1, b"noise from fd 1" + bytes([10])),',
            ")",
            "pool.worker_main()",
        )
    )
    assert last in pool_module._BOOTSTRAP
    monkeypatch.setattr(
        pool_module, "_BOOTSTRAP", pool_module._BOOTSTRAP.replace(last, noisy)
    )
    pool = DomainPool(1)
    try:
        pool.ensure_started()
        assert pool.worker_count == 1
        # Трубу stderr читает отдельный поток: дать ему дочитать.
        deadline = time.monotonic() + 5.0
        described = pool._workers[0].describe()
        while "noise from fd 1" not in described and time.monotonic() < deadline:
            time.sleep(0.05)
            described = pool._workers[0].describe()
        assert "noise from print" in described
        assert "noise from fd 1" in described
    finally:
        pool.close()


def _field_task(task_id=0):
    """Полевой домен с ИРРАЦИОНАЛЬНЫМИ длинами: факторизация здесь настоящая.

    `quad_row_bundle` даёт рациональные длины, и память разложений на нём
    не видна вовсе; этот домен стоит 4408 единиц холодным и 287 тёплым.
    """

    import cftuv_envelope as kernel

    root = KERNEL_SRC.parent / "fixtures" / "building_002_point_contact_v1"
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (root / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (root / "decal_request.json").read_bytes()
    )
    domain_id = next(iter(snapshot.patch_domains)).patch_domain_id.value
    return DomainTaskV1(
        task_id, 7, domain_id, snapshot, request, "0.25", frozenset()
    )


def _articles(prepared):
    return prepared.work_budget.counters()


def test_a_domain_is_priced_from_a_cold_memo_whatever_the_process_has_seen(
    monkeypatch,
):
    from cftuv_envelope import exact_sqrt_sum

    task = _field_task()

    def run():
        return run_queue_domain(
            task.patch_id,
            task.domain_id,
            task.snapshot,
            task.request,
            task.alpha_text,
        )

    cold_prepared, cold_domain = run()
    cold = dict(_articles(cold_prepared))
    assert cold["EXACT_WORK_MODULAR_SQUARINGS"] > 0, cold

    # Контроль: без сброса тот же домен в том же процессе стоит ДЕШЕВЛЕ. Если
    # фикстура перестанет факторизовать, тест упадёт здесь, а не станет слепым.
    with monkeypatch.context() as unreset:
        unreset.setattr(exact_sqrt_sum, "reset_factorization_memory", lambda: None)
        unreset.setattr(exact_sqrt_sum, "reset_unbudgeted_work", lambda: None)
        warm_prepared, _ = run()
    warm = dict(_articles(warm_prepared))
    assert warm["EXACT_WORK_SPENT"] < cold["EXACT_WORK_SPENT"], (cold, warm)

    # Сам продукт: сброс внутри `run_queue_domain`, статьи равны холодным.
    again_prepared, again_domain = run()
    assert dict(_articles(again_prepared)) == cold
    assert again_domain.counters == cold_domain.counters
    assert again_domain.host_counters == cold_domain.host_counters


def test_a_pooled_domain_is_priced_like_a_sequential_one_on_a_warm_worker():
    local_prepared, local_domain = run_queue_domain(
        0,
        *(lambda t: (t.domain_id, t.snapshot, t.request, t.alpha_text))(
            _field_task()
        ),
    )
    cold = dict(_articles(local_prepared))
    # Один воркер и две одинаковые задачи: вторая всегда ложится на ТЁПЛЫЙ
    # воркер, и без сброса её статьи были бы другими.
    pool = DomainPool(1)
    try:
        run = pool.run([_field_task(0), _field_task(1)])
    finally:
        pool.close()

    assert run.workers == 1 and len(run.results) == 2
    for result in run.results.values():
        assert result.ok, result.error
        assert dict(_articles(result.prepared)) == cold
        assert result.queue_domain.counters == local_domain.counters
        assert result.queue_domain.host_counters == local_domain.host_counters


def test_real_workers_reproduce_the_sequential_run_and_warm_the_session_cache():
    bundle = quad_row_bundle(ROW, lifted_corner=0.05)
    expected, expected_profile, sequential = _session_run(bundle, workers=0)

    evaluation, profile, controller = _session_run(bundle, workers=2)

    assert _fingerprint(evaluation, profile) == _fingerprint(
        expected, expected_profile
    )
    assert _counter(profile, POOL_WORKERS) == 2
    assert _counter(profile, POOL_DISPATCHED) == ROW - 1
    assert _counter(profile, POOL_TASK_FALLBACK) == 0
    assert _counter(profile, POOL_UNAVAILABLE) == 0
    assert controller.build_counts == sequential.build_counts
    assert controller.build_counts["CONVEYOR_PREPARATION"] == ROW - 1
    # Подготовки воркеров лежат в кэше сессии: ползунок alpha найдёт их тёплыми.
    assert all(
        item.preparation is not None for item in evaluation.queue_domains
    )

    # Повторная кнопка на другой alpha: всё в кэше, воркерам нечего делать, а
    # ответ тот же, что у последовательного прогона на этой alpha.
    again, again_profile, _ = _session_run(
        bundle, 0.4, workers=2, controller=controller
    )
    reference, reference_profile, _ = _session_run(
        bundle, 0.4, workers=0, controller=sequential
    )
    assert _counter(again_profile, POOL_DISPATCHED) == 0
    assert POOL_WALL_STAGE not in again_profile.snapshot().stage_totals
    assert controller.build_counts["CONVEYOR_PREPARATION"] == ROW - 1
    assert _fingerprint(again, again_profile) == _fingerprint(
        reference, reference_profile
    )


def test_a_killed_worker_costs_one_task_and_the_pool_respawns_it():
    snapshots = []
    bundle = quad_row_bundle(ROW)
    for patch_id in range(ROW):
        snapshot = build_envelope_analysis_snapshot(
            bundle, included_patch_ids=frozenset({patch_id})
        )
        request = build_envelope_decal_request(
            snapshot, frozenset({patch_id}), 0.25
        )
        domain_id = next(iter(snapshot.patch_domains)).patch_domain_id.value
        snapshots.append(
            DomainTaskV1(
                patch_id,
                patch_id,
                domain_id,
                snapshot,
                request,
                "0.25",
                frozenset({patch_id}),
            )
        )
    pool = DomainPool(2)
    try:
        pool.ensure_started()
        assert pool.worker_count == 2
        pool._workers[0].process.kill()
        pool._workers[0].process.wait()

        run = pool.run(snapshots)

        assert run.workers == 2
        failed = [item for item in run.results.values() if not item.ok]
        assert len(failed) == 1 and "worker died" in failed[0].error
        assert len(run.results) == ROW
        assert pool.worker_count == 1

        healed = pool.run(snapshots)
        assert pool.worker_count == 2
        assert all(item.ok for item in healed.results.values())
        assert healed.workers == 2
    finally:
        pool.close()


def test_shutdown_stops_every_worker_process():
    pool = get_domain_pool(2)
    pool.ensure_started()
    processes = [item.process for item in pool._workers]
    assert len(processes) == 2 and all(p.poll() is None for p in processes)

    shutdown_domain_pool()

    assert all(p.poll() is not None for p in processes)
