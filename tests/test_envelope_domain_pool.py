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
import os
import pickle
import re
import subprocess
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
    POOL_COVERAGE_DISPATCHED,
    POOL_DISPATCHED,
    POOL_EXTERNAL_PYTHON,
    POOL_INTERPRETER_FALLBACK,
    POOL_INTERPRETER_REASON,
    POOL_PYTHON_VERSION,
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
from envelope_fixture_bundles import pin_near_planar_only, quad_row_bundle  # noqa: E402


ROW = 5
POOL_NAMES = (
    POOL_UNAVAILABLE,
    POOL_TASK_FALLBACK,
    POOL_WORKERS,
    POOL_DISPATCHED,
    POOL_COVERAGE_DISPATCHED,
    POOL_PYTHON_VERSION,
    POOL_EXTERNAL_PYTHON,
    POOL_INTERPRETER_FALLBACK,
    POOL_INTERPRETER_REASON,
    pool_module.INTERPRETER_MISMATCH,
    pool_module.INTERPRETER_UNUSABLE,
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


def _session_run(
    bundle, alpha=0.25, *, workers, controller=None, density=None
):
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
        density=density,
        workers=workers,
    )
    return evaluation, profile, controller


def _session_state(controller):
    """Кэши сессии без секунд: канонические байты снапшотов и счёт сборок.

    Снапшот сравнивается КАНОНИЧЕСКИМИ БАЙТАМИ кодека ядра, а не `==`: байты —
    то, что видит ядро, и то, что родитель и воркер обязаны выпустить одинаково.
    Отказ метрики лежит в кэше значением (`_CachedMetricFailure`).
    """

    from cftuv_envelope import codec

    metrics = {}
    for key, value in sorted(controller._patch_metric_cache.items()):
        snapshot = getattr(value, "snapshot", None)
        metrics[key] = (
            codec.canonical_json_bytes(snapshot)
            if snapshot is not None
            else (value.outcome, value.message, value.patch_domain_id)
        )
    geometry = {
        key: codec.canonical_json_bytes(value.snapshot)
        for key, value in sorted(controller._domain_geometry_cache.items())
    }
    return (
        metrics,
        geometry,
        controller.build_counts,
        dict(controller._cache_build_counts),
        sorted(map(repr, controller._conveyor_preparation_cache)),
    )


class _ParentExportCalls:
    """Сколько раз РОДИТЕЛЬ выгрузил снапшот патча (а не воркер)."""

    def __init__(self, monkeypatch):
        from cftuv import envelope_request_export

        self.count = 0
        original = envelope_request_export.build_envelope_analysis_snapshot

        def counted(*args, **kwargs):
            self.count += 1
            return original(*args, **kwargs)

        monkeypatch.setattr(
            envelope_request_export, "build_envelope_analysis_snapshot", counted
        )


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


@pytest.mark.parametrize("ladder_off", (False, True), ids=("ladder", "near-planar-only"))
def test_the_in_process_pool_reproduces_the_sequential_run_exactly(monkeypatch, ladder_off):
    if ladder_off:
        pin_near_planar_only(monkeypatch)
    bundle = quad_row_bundle(ROW, lifted_corner=1.0)
    expected, expected_profile = _direct_run(bundle, pool=None)
    pool = _InProcessPool()

    evaluation, profile = _direct_run(bundle, pool=pool)

    assert _fingerprint(evaluation, profile) == _fingerprint(
        expected, expected_profile
    )
    # Без лестницы: четыре точных домена и один отказ выгрузки (метрика), отказавший домен
    # воркеру не уходит, он разбирается тем же обработчиком, что и раньше. С лестницей изогнутый
    # квад разворачивается и уходит воркеру наравне с остальными: отказа выгрузки нет.
    dispatched = ROW - 1 if ladder_off else ROW
    assert len(pool.dispatched) == dispatched
    assert _counter(profile, POOL_DISPATCHED) == dispatched
    assert _counter(profile, POOL_WORKERS) == 2
    assert _counter(profile, POOL_TASK_FALLBACK) == 0
    assert _counter(profile, POOL_UNAVAILABLE) == 0
    assert _counter(expected_profile, POOL_WORKERS) is None
    assert POOL_WALL_STAGE in profile.snapshot().stage_totals
    assert POOL_WALL_STAGE not in expected_profile.snapshot().stage_totals
    assert ("NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED" in {
        item.outcome for item in evaluation.receipts
    }) is ladder_off


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


def test_real_workers_reproduce_the_sequential_run_and_warm_the_session_cache(
    _pool_never,
):
    bundle = quad_row_bundle(ROW, lifted_corner=1.0)
    expected, expected_profile, sequential = _session_run(bundle, workers=0)

    evaluation, profile, controller = _session_run(bundle, workers=2)

    assert _fingerprint(evaluation, profile) == _fingerprint(
        expected, expected_profile
    )
    assert _counter(profile, POOL_WORKERS) == 2
    assert _counter(profile, POOL_DISPATCHED) == ROW
    assert _counter(profile, POOL_TASK_FALLBACK) == 0
    assert _counter(profile, POOL_UNAVAILABLE) == 0
    assert controller.build_counts == sequential.build_counts
    assert controller.build_counts["CONVEYOR_PREPARATION"] == ROW
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
    assert controller.build_counts["CONVEYOR_PREPARATION"] == ROW
    assert _fingerprint(again, again_profile) == _fingerprint(
        reference, reference_profile
    )


# --------------------------------------------------------------------------
# 5. Выгрузка снапшота в воркере (HOST-EXPORT-PARALLEL)
# --------------------------------------------------------------------------


def test_worker_export_modules_import_without_blender():
    """Воркер — обычный интерпретатор: ни `bpy`, ни `bmesh`, ни `mathutils`.

    Выгрузка снапшота в воркере тянет `envelope_request_export`, а тот прежде
    импортировал `model.py`, то есть `mathutils`. Три перечисления, которые ему
    нужны, вынесены в `model_enums`; эта проверка держит стену: любой новый
    импорт Blender в цепочке воркера ломает воркер на старте, и узнать об этом
    иначе можно только в Blender.
    """

    script = "\n".join(
        (
            "import sys",
            "class Block:",
            "    def find_spec(self, name, path=None, target=None):",
            "        if name.split('.')[0] in {'bpy', 'bmesh', 'mathutils'}:",
            "            raise ImportError('blocked: ' + name)",
            "sys.meta_path.insert(0, Block())",
            "import cftuv.envelope_export_input",
            "import cftuv.envelope_domain_pool",
            "import cftuv.envelope_queue_export",
            "import cftuv.envelope_request_export",
            "import cftuv.envelope_metric_export",
            "assert not {'bpy', 'bmesh', 'mathutils'} & set(sys.modules)",
            "print('NO_BLENDER_IN_WORKER_CHAIN')",
        )
    )
    environment = dict(os.environ)
    environment["PYTHONPATH"] = os.pathsep.join(
        (str(KERNEL_SRC.parents[1]), str(KERNEL_SRC))
    )
    completed = subprocess.run(
        [sys.executable, "-c", script],
        capture_output=True,
        env=environment,
        timeout=120,
    )
    assert completed.returncode == 0, completed.stderr.decode(errors="replace")
    assert b"NO_BLENDER_IN_WORKER_CHAIN" in completed.stdout


def test_the_enums_moved_out_of_the_model_are_the_same_objects():
    from cftuv import model, model_enums

    for name in ("ChainNeighborKind", "LoopKind", "PatchType"):
        assert getattr(model, name) is getattr(model_enums, name)


def test_export_tasks_are_ranked_in_snapshot_units():
    light_but_heavy = DomainTaskV1(
        0, 0, "d0", None, None, "0.25", frozenset(), export="x" * 1000
    )
    heavy_snapshot = DomainTaskV1(
        1, 1, "d1", "x" * 3000, None, "0.25", frozenset()
    )
    light = DomainTaskV1(
        2, 2, "d2", None, None, "0.25", frozenset(), export="x" * 100
    )

    ordered = order_by_cost([light, heavy_snapshot, light_but_heavy])

    # 1000 байт входа — это ~5000 байт снапшота: выше 3000 и стократ выше 100.
    assert [task.task_id for task, _ in ordered] == [0, 1, 2]


def _export_inputs(bundle):
    from cftuv.envelope_request_export import _typed_value
    from cftuv.envelope_topology_export import stage_domain_inputs

    topology = build_envelope_topology_export(bundle)
    _, revision, patch_ids, request_id, by_domain = stage_domain_inputs(
        bundle, frozenset(range(ROW)), topology_export=topology
    )
    return topology, revision, patch_ids, request_id, by_domain, _typed_value


@pytest.mark.parametrize("ladder_off", (False, True), ids=("ladder", "near-planar-only"))
def test_the_worker_export_reproduces_the_parent_export_bit_for_bit(monkeypatch, ladder_off):
    if ladder_off:
        pin_near_planar_only(monkeypatch)
    """Снапшот воркера == снапшот родителя: канонические байты, отказы, стадии.

    Вход идёт через pickle, как по трубе: это и проверка того, что лёгкий вход
    не несёт ничего, что не доезжает. Один домен ряда отказывает на метрике, и
    его отказ обязан приехать тем же исходом и текстом.
    """

    from cftuv.envelope_export_input import (
        ExportRefusalV1,
        build_host_export_input,
    )
    from cftuv.envelope_metric_export import build_envelope_patch_metric_export
    from cftuv.envelope_request_export import EnvelopeHostAdapterError
    from cftuv_envelope import codec

    bundle = quad_row_bundle(ROW, lifted_corner=1.0)
    topology, _, patch_ids, request_id, by_domain, _ = _export_inputs(bundle)
    refused = 0
    for patch_id in patch_ids:
        domain_id = topology.patch_domain_id_by_patch[patch_id]
        selected = frozenset(by_domain[domain_id])
        export = pickle.loads(
            pickle.dumps(
                build_host_export_input(
                    topology,
                    patch_id,
                    alpha=0.25,
                    request_id=request_id,
                    density="2",
                ),
                5,
            )
        )
        result = pickle.loads(
            pickle.dumps(
                solve_task(
                    DomainTaskV1(
                        patch_id,
                        patch_id,
                        domain_id,
                        None,
                        None,
                        "0.25",
                        selected,
                        export,
                    )
                ),
                5,
            )
        )
        profile = EnvelopeDebugProfileBuilderV1("row", "QUEUE")
        try:
            parent = build_envelope_patch_metric_export(
                topology, patch_id, profile=profile
            ).snapshot
        except EnvelopeHostAdapterError as exc:
            refused += 1
            assert result.refused and not result.ok
            assert result.snapshot is None
            assert result.refusal == ExportRefusalV1(
                exc.outcome, str(exc), exc.patch_domain_id
            )
        else:
            assert result.ok, result.error
            assert result.snapshot == parent
            assert codec.canonical_json_bytes(
                result.snapshot
            ) == codec.canonical_json_bytes(parent)
            request = build_envelope_decal_request(
                parent,
                selected,
                0.25,
                decal_request_id_value=request_id,
                density="2",
            )
            _, expected = run_queue_domain(
                patch_id, domain_id, parent, request, "0.25",
                selected_edges=selected,
            )
            assert _timing_free_domain(result.queue_domain) == (
                _timing_free_domain(expected)
            )
        recorded = profile.snapshot()
        assert [
            (item.stage, item.patch_domain_id)
            for item in result.export_timings
        ] == [
            (item.stage, item.patch_domain_id) for item in recorded.timings
        ]
        assert result.export_counters == recorded.counters
    # С лестницей (S1) изогнутый квад развёртывается; без неё отказывает на метрике.
    assert refused == (1 if ladder_off else 0)


def _timing_free_domain(domain):
    from dataclasses import replace

    from cftuv.envelope_queue_export import queue_domain_payload

    payload = queue_domain_payload(replace(domain, preparation=None))
    for key in (
        "prepare_seconds",
        "coverage_seconds",
        "contour_seconds",
        "timings",
    ):
        payload.pop(key)
    return payload


def test_real_workers_export_the_snapshots_and_the_session_stays_identical(
    monkeypatch, _pool_never
):
    """Родитель не выгружает ничего, а всё, что он отдаёт дальше, прежнее.

    Кэш метрики и геометрии (канонические байты), счёт сборок, счётчики
    `*_CACHE_HIT/MISS` и `*_BUILD_COUNT`, квитанции (METRIC_REJECTED на
    отказавшей метрике), диагностики и отпечаток ответа — как у
    последовательного прогона. Вторая кнопка находит всё в кэше; смена
    плотности находит метрики тёплыми, а подготовки холодными — тот домен идёт
    воркеру уже готовым снапшотом.
    """

    bundle = quad_row_bundle(ROW, lifted_corner=1.0)
    calls = _ParentExportCalls(monkeypatch)
    expected, expected_profile, sequential = _session_run(bundle, workers=0)
    assert calls.count == ROW
    calls.count = 0

    evaluation, profile, controller = _session_run(bundle, workers=2)

    assert calls.count == 0
    assert _counter(profile, POOL_WORKERS) == 2
    assert _counter(profile, POOL_DISPATCHED) == ROW
    assert _counter(profile, POOL_TASK_FALLBACK) == 0
    assert _counter(profile, POOL_UNAVAILABLE) == 0
    assert _fingerprint(evaluation, profile) == _fingerprint(
        expected, expected_profile
    )
    assert _session_state(controller) == _session_state(sequential)
    stages = {item.stage.value for item in evaluation.receipts}
    # Развёртка (S1) принимает и изогнутый квад: отказавших на метрике нет.
    assert stages == {"QUEUE_RESOLVED"}, stages
    # Стадии выгрузки воркера проиграны в профиль кнопки под теми же именами.
    worker_stages = {"PATCH_METRIC_EXPORT", "FRAME_ADMISSION", "SNAPSHOT_VALIDATION"}
    assert worker_stages <= set(profile.snapshot().stage_totals)

    again, again_profile, _ = _session_run(
        bundle, 0.4, workers=2, controller=controller
    )
    reference, reference_profile, _ = _session_run(
        bundle, 0.4, workers=0, controller=sequential
    )
    assert calls.count == 0  # всё в кэше метрики, у обоих прогонов
    assert _counter(again_profile, POOL_DISPATCHED) == 0
    assert _fingerprint(again, again_profile) == _fingerprint(
        reference, reference_profile
    )
    assert _session_state(controller) == _session_state(sequential)

    calls.count = 0
    density, density_profile, _ = _session_run(
        bundle, workers=2, controller=controller, density="2"
    )
    density_reference, density_reference_profile, _ = _session_run(
        bundle, workers=0, controller=sequential, density="2"
    )
    assert calls.count == 0  # метрики тёплые: родитель берёт их из кэша
    assert _counter(density_profile, POOL_DISPATCHED) == ROW
    assert _fingerprint(density, density_profile) == _fingerprint(
        density_reference, density_reference_profile
    )
    assert _session_state(controller) == _session_state(sequential)


def _in_process_session(monkeypatch, pool):
    monkeypatch.setattr(
        pool_module, "get_domain_pool", lambda workers, external_python="": pool
    )


@pytest.mark.parametrize("injection", ("fail", "drop"))
def test_a_failed_export_task_is_named_and_exported_in_the_parent(
    injection, monkeypatch, capsys
):
    bundle = quad_row_bundle(ROW)
    expected, expected_profile, sequential = _session_run(bundle, workers=0)
    victim_task = 2
    _in_process_session(
        monkeypatch, _InProcessPool(**{injection: (victim_task,)})
    )

    evaluation, profile, controller = _session_run(bundle, workers=2)

    victim = evaluation.domains[victim_task]
    assert _counter(profile, POOL_TASK_FALLBACK) == 1
    assert _counter(profile, POOL_DISPATCHED) == ROW
    notices = [
        item
        for item in victim.diagnostics
        if item.outcome == POOL_TASK_FALLBACK
    ]
    assert len(notices) == 1
    assert notices[0].patch_domain_id == victim.patch_domain_id
    assert victim.queue is not None and victim.queue.is_exact
    assert POOL_TASK_FALLBACK in capsys.readouterr().out
    assert _fingerprint(
        evaluation, profile, skip_scene_of=(victim.patch_domain_id,)
    ) == _fingerprint(
        expected, expected_profile, skip_scene_of=(victim.patch_domain_id,)
    )
    assert _session_state(controller) == _session_state(sequential)


def test_a_refused_domain_whose_worker_died_is_neither_dispatched_nor_fallback(
    monkeypatch,
):
    pin_near_planar_only(monkeypatch)
    bundle = quad_row_bundle(ROW, lifted_corner=1.0)
    expected, expected_profile, sequential = _session_run(bundle, workers=0)
    # Последний патч ряда — тот, что отказывает на метрике.
    _in_process_session(monkeypatch, _InProcessPool(drop=(ROW - 1,)))

    evaluation, profile, controller = _session_run(bundle, workers=2)

    assert _counter(profile, POOL_DISPATCHED) == ROW - 1
    assert _counter(profile, POOL_TASK_FALLBACK) == 0
    assert evaluation.receipts[ROW - 1].stage.value == "METRIC_REJECTED"
    assert _fingerprint(evaluation, profile) == _fingerprint(
        expected, expected_profile
    )
    assert _session_state(controller) == _session_state(sequential)


def test_a_metric_stage_refusal_keeps_its_receipt_stage_through_the_worker(
    monkeypatch,
):
    """METRIC_REJECTED и QUEUE_PREPARE_REJECTED различаются исходом отказа.

    `quad_row_bundle(lifted_corner=...)` даёт `NEAR_PLANAR_RESIDUAL_BUDGET_
    EXCEEDED` (ступень метрики по `METRIC_STAGE_OUTCOMES`), как и «кадр недоступен»;
    его нечем вызвать данными ряда, поэтому кадр патча 1 отказывает подменой
    (пул «в процессе» исполняет ту же выгрузку в этом же процессе).
    """

    from cftuv import envelope_request_export as export_module

    original = export_module._rational_affine_metric

    def refusing(kernel, **kwargs):
        if kwargs["owner_patch_id"].value.endswith(":1"):
            raise export_module.EnvelopeHostAdapterError(
                export_module.EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE,
                "injected frame refusal",
                patch_domain_id=kwargs["patch_domain_id"].value,
            )
        return original(kernel, **kwargs)

    monkeypatch.setattr(export_module, "_rational_affine_metric", refusing)
    bundle = quad_row_bundle(ROW)
    expected, expected_profile, sequential = _session_run(bundle, workers=0)
    _in_process_session(monkeypatch, _InProcessPool())

    evaluation, profile, controller = _session_run(bundle, workers=2)

    assert evaluation.receipts[1].stage.value == "METRIC_REJECTED"
    assert [item.stage.value for item in evaluation.receipts].count(
        "QUEUE_RESOLVED"
    ) == ROW - 1
    assert _counter(profile, POOL_DISPATCHED) == ROW - 1
    assert _fingerprint(evaluation, profile) == _fingerprint(
        expected, expected_profile
    )
    assert _session_state(controller) == _session_state(sequential)
    # Отказ лежит в кэше метрики: повторная кнопка берёт его оттуда.
    again, again_profile, _ = _session_run(
        bundle, 0.4, workers=2, controller=controller
    )
    cache_hits = [
        item
        for item in again_profile.snapshot().counters
        if item.name == "PATCH_METRIC_CACHE_HIT" and item.value == 1
    ]
    assert len(cache_hits) == ROW
    assert again.receipts[1].stage.value == "METRIC_REJECTED"


def test_an_unavailable_pool_exports_in_the_parent_and_names_the_first_domain(
    monkeypatch,
):
    bundle = quad_row_bundle(ROW, lifted_corner=1.0)
    expected, expected_profile, sequential = _session_run(bundle, workers=0)
    _in_process_session(
        monkeypatch, DomainPool(2, python_executable="C:/nowhere/python.exe")
    )

    evaluation, profile, controller = _session_run(bundle, workers=2)

    first = evaluation.domains[0]
    assert _counter(profile, POOL_UNAVAILABLE) == 1
    assert _counter(profile, POOL_DISPATCHED) == 0
    notices = [
        item for item in evaluation.diagnostics if item.outcome == POOL_UNAVAILABLE
    ]
    assert [item.patch_domain_id for item in notices] == [first.patch_domain_id]
    assert _fingerprint(
        evaluation, profile, skip_scene_of=(first.patch_domain_id,)
    ) == _fingerprint(
        expected, expected_profile, skip_scene_of=(first.patch_domain_id,)
    )
    assert _session_state(controller) == _session_state(sequential)


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


def test_a_ready_worker_imports_no_kernel_module_for_its_first_domain():
    """До «готов» воркер поднимает всё ядро очереди; первый домен ничего не ждёт.

    Пакет `wavefront` ленив, а домен берёт `symbolic_*`, `superlevel_*` и
    `source_grid` изнутри функций: с двумя именами в `load_queue_kernel` первый
    домен каждого воркера платил за них импортом в секундах домена, то есть на
    критическом пути стены. Проверка — в чистом интерпретаторе, по образцу
    воркера: `load_queue_kernel`, затем кадр и `solve_task`.
    """

    script = "\n".join(
        (
            "import sys",
            "from cftuv.envelope_domain_pool import read_frame, solve_task",
            "from cftuv.envelope_queue_export import load_queue_kernel",
            "load_queue_kernel()",
            "before = set(sys.modules)",
            "result = solve_task(read_frame(sys.stdin.buffer))",
            "assert result.ok, result.error",
            "late = sorted(",
            "    name for name in set(sys.modules) - before",
            "    if name.startswith('cftuv_envelope')",
            ")",
            "print('LATE', late)",
        )
    )
    environment = dict(os.environ)
    environment["PYTHONPATH"] = os.pathsep.join(
        (str(KERNEL_SRC.parents[1]), str(KERNEL_SRC))
    )
    completed = subprocess.run(
        [sys.executable, "-c", script],
        input=encode_frame(_field_task()),
        capture_output=True,
        env=environment,
        timeout=300,
    )
    assert completed.returncode == 0, completed.stderr.decode(errors="replace")
    assert completed.stdout.decode().strip().endswith("LATE []")


def test_shutdown_stops_every_worker_process():
    pool = get_domain_pool(2)
    pool.ensure_started()
    processes = [item.process for item in pool._workers]
    assert len(processes) == 2 and all(p.poll() is None for p in processes)

    shutdown_domain_pool()

    assert all(p.poll() is not None for p in processes)


# --------------------------------------------------------------------------
# 6. Покрытие кэшированных подготовок в воркерах (WARM-COVERAGE-PARALLEL)
# --------------------------------------------------------------------------


@pytest.fixture
def _pool_always(monkeypatch):
    """Малая партия остаётся в родителе (порог); тесту нужен именно пул."""

    from cftuv import envelope_queue_pool

    monkeypatch.setattr(envelope_queue_pool, "COVERAGE_POOL_MIN_BYTES", 0)


@pytest.fixture
def _pool_never(monkeypatch):
    """Порог выше любой партии теста: покрытие остаётся в родителе."""

    from cftuv import envelope_queue_pool

    monkeypatch.setattr(envelope_queue_pool, "COVERAGE_POOL_MIN_BYTES", 10**9)


def _coverage_task(task_id, weight):
    from cftuv.envelope_queue_pool import CoverageInputV1

    return DomainTaskV1(
        task_id,
        task_id,
        f"d{task_id}",
        None,
        None,
        "0.25",
        frozenset(),
        coverage=CoverageInputV1(b"x" * weight, False),
    )


def test_coverage_tasks_are_ranked_by_what_their_blob_costs_to_cover():
    from cftuv.envelope_domain_pool import COVERAGE_FRAME_COST_DIVISOR

    snapshot = DomainTaskV1(0, 0, "d0", "x" * 3000, None, "0.25", frozenset())
    big_blob = _coverage_task(1, 3000 * COVERAGE_FRAME_COST_DIVISOR * 2)
    small_blob = _coverage_task(2, 3000)

    ordered = order_by_cost([small_blob, snapshot, big_blob])

    # Пикл вдвое «дороже» снапшота после приведения идёт первым, пикл тех же
    # 3 КБ — после снапшота: покрытие за байт стоит меньше решения.
    assert [task.task_id for task, _ in ordered] == [1, 0, 2]


def test_a_preparation_is_pickled_once_and_the_blob_cache_dies_with_the_session():
    bundle = quad_row_bundle(ROW)
    _, _, controller = _session_run(bundle, workers=0)
    prepared = next(iter(controller._conveyor_preparation_cache.values()))
    blobs = controller.preparation_blobs

    first = blobs.blob_of(prepared)

    assert blobs.blob_of(prepared) is first and len(blobs) == 1
    assert pickle.loads(first).outcome == prepared.outcome
    controller.clear()
    assert len(blobs) == 0


def test_a_shipped_coverage_equals_the_parents_for_both_memory_modes():
    """Покрытие воркера == покрытие родителя: запись домена, счётчики, без секунд.

    Домен полевой, с иррациональными длинами: факторизация настоящая, и режим
    памяти (кнопка сбрасывает её, ползунок нет) не должен менять ответ.
    """

    from cftuv.envelope_queue_export import cover_prepared
    from cftuv.envelope_queue_pool import CoverageInputV1

    task = _field_task()
    prepared, _ = run_queue_domain(
        task.patch_id, task.domain_id, task.snapshot, task.request, "0.25"
    )
    blob = pickle.dumps(prepared, protocol=5)
    for alpha in ("0.25", "0.4"):
        expected = cover_prepared(task.patch_id, task.domain_id, prepared, alpha)
        for reset in (True, False):
            shipped = pickle.loads(
                pickle.dumps(
                    solve_task(
                        DomainTaskV1(
                            1,
                            task.patch_id,
                            task.domain_id,
                            None,
                            None,
                            alpha,
                            frozenset(),
                            coverage=CoverageInputV1(blob, reset),
                        )
                    ),
                    5,
                )
            )
            assert shipped.ok, shipped.error
            assert shipped.prepared is None
            assert shipped.queue_domain.preparation is None
            assert _timing_free_domain(shipped.queue_domain) == (
                _timing_free_domain(expected)
            )
            assert shipped.queue_domain.counters == expected.counters
            assert shipped.queue_domain.host_counters == expected.host_counters


def _warm_pair(bundle, *, workers=2, alpha=0.4):
    """Две сессии: пуловая и последовательная, обе холодные, затем тёплая кнопка."""

    cold, cold_profile, pooled = _session_run(bundle, workers=workers)
    cold_ref, cold_ref_profile, sequential = _session_run(bundle, workers=0)
    assert _fingerprint(cold, cold_profile) == _fingerprint(
        cold_ref, cold_ref_profile
    )
    return pooled, sequential


def test_real_workers_cover_cached_preparations_and_the_warm_press_is_identical(
    _pool_always,
):
    bundle = quad_row_bundle(ROW, lifted_corner=1.0)
    pooled, sequential = _warm_pair(bundle)

    warm, warm_profile, _ = _session_run(
        bundle, 0.4, workers=2, controller=pooled
    )
    reference, reference_profile, _ = _session_run(
        bundle, 0.4, workers=0, controller=sequential
    )

    # Воркеры считали покрытие всех пяти доменов (изогнутый квад принят
    # развёрткой), подготовок не строили и пул не заводил заново.
    assert _counter(warm_profile, POOL_WORKERS) == 2
    assert _counter(warm_profile, POOL_DISPATCHED) == ROW
    assert _counter(warm_profile, POOL_COVERAGE_DISPATCHED) == ROW
    assert _counter(warm_profile, POOL_TASK_FALLBACK) == 0
    assert _counter(warm_profile, POOL_UNAVAILABLE) == 0
    assert POOL_WALL_STAGE in warm_profile.snapshot().stage_totals
    assert _fingerprint(warm, warm_profile) == _fingerprint(
        reference, reference_profile
    )
    # Кэши, их счётчики и счёт сборок — как у последовательного прогона: ровно
    # одно попадание подготовки на домен, ни одной новой сборки.
    assert _session_state(pooled) == _session_state(sequential)
    hits = [
        item
        for item in warm_profile.snapshot().counters
        if item.name == "CONVEYOR_PREPARATION_CACHE_HIT" and item.value == 1
    ]
    assert len(hits) == ROW
    # Подготовка в записи — тот самый объект кэша, а не копия воркера.
    cache = list(pooled._conveyor_preparation_cache.values())
    assert all(
        any(item.preparation is cached for cached in cache)
        for item in warm.queue_domains
    )


def test_a_mixed_batch_sends_whole_domains_and_coverages_in_one_run(_pool_always):
    """Домен без подготовки в кэше идёт целиком, остальные — покрытием, разом."""

    bundle = quad_row_bundle(ROW)
    pooled, sequential = _warm_pair(bundle)
    victim = sorted(pooled._conveyor_preparation_cache, key=repr)[2]
    del pooled._conveyor_preparation_cache[victim]
    del sequential._conveyor_preparation_cache[victim]

    warm, warm_profile, _ = _session_run(
        bundle, 0.4, workers=2, controller=pooled
    )
    reference, reference_profile, _ = _session_run(
        bundle, 0.4, workers=0, controller=sequential
    )

    assert _counter(warm_profile, POOL_DISPATCHED) == ROW
    assert _counter(warm_profile, POOL_COVERAGE_DISPATCHED) == ROW - 1
    assert _counter(warm_profile, POOL_TASK_FALLBACK) == 0
    assert _fingerprint(warm, warm_profile) == _fingerprint(
        reference, reference_profile
    )
    assert pooled.build_counts["CONVEYOR_PREPARATION"] == ROW + 1
    assert _session_state(pooled) == _session_state(sequential)


def test_a_small_batch_stays_in_the_parent_and_the_pool_is_not_asked(
    _pool_never,
):
    bundle = quad_row_bundle(ROW)
    pooled, sequential = _warm_pair(bundle)

    warm, warm_profile, _ = _session_run(
        bundle, 0.4, workers=2, controller=pooled
    )
    reference, reference_profile, _ = _session_run(
        bundle, 0.4, workers=0, controller=sequential
    )

    assert _counter(warm_profile, POOL_DISPATCHED) == 0
    assert _counter(warm_profile, POOL_COVERAGE_DISPATCHED) == 0
    assert POOL_WALL_STAGE not in warm_profile.snapshot().stage_totals
    assert _fingerprint(warm, warm_profile) == _fingerprint(
        reference, reference_profile
    )


@pytest.mark.parametrize("injection", ("fail", "drop"))
def test_a_failed_coverage_task_is_named_and_covered_in_the_parent(
    injection, monkeypatch, capsys, _pool_always
):
    bundle = quad_row_bundle(ROW)
    pooled, sequential = _warm_pair(bundle)
    victim_task = 1
    _in_process_session(
        monkeypatch, _InProcessPool(**{injection: (victim_task,)})
    )

    warm, warm_profile, _ = _session_run(
        bundle, 0.4, workers=2, controller=pooled
    )
    reference, reference_profile, _ = _session_run(
        bundle, 0.4, workers=0, controller=sequential
    )

    victim = warm.domains[victim_task]
    assert _counter(warm_profile, POOL_TASK_FALLBACK) == 1
    assert _counter(warm_profile, POOL_DISPATCHED) == ROW
    assert _counter(warm_profile, POOL_COVERAGE_DISPATCHED) == ROW
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
    assert _fingerprint(
        warm, warm_profile, skip_scene_of=(victim.patch_domain_id,)
    ) == _fingerprint(
        reference, reference_profile, skip_scene_of=(victim.patch_domain_id,)
    )
    assert victim.queue is not None and victim.queue.is_exact
    assert _session_state(pooled) == _session_state(sequential)


def test_a_worker_killed_during_a_warm_press_costs_one_domain_and_heals(
    _pool_always,
):
    """Настоящий труп: воркер умер до задачи, домен досчитан в родителе.

    Последовательные прогоны-эталоны закрывают общий пул (0 воркеров — это его
    выключатель), поэтому сначала идут оба пуловых нажатия, потом эталоны.
    """

    bundle = quad_row_bundle(ROW)
    _, _, sequential = _session_run(bundle, workers=0)
    shutdown_domain_pool()
    _, _, pooled = _session_run(bundle, workers=2)
    pool = pool_module._POOL
    assert pool is not None and pool.worker_count == 2
    pool._workers[0].process.kill()
    pool._workers[0].process.wait()

    warm, warm_profile, _ = _session_run(
        bundle, 0.4, workers=2, controller=pooled
    )
    survivors = pool.worker_count
    healed, healed_profile, _ = _session_run(
        bundle, 0.3, workers=2, controller=pooled
    )
    reference, reference_profile, _ = _session_run(
        bundle, 0.4, workers=0, controller=sequential
    )
    healed_reference, healed_reference_profile, _ = _session_run(
        bundle, 0.3, workers=0, controller=sequential
    )

    assert survivors == 1
    assert _counter(warm_profile, POOL_TASK_FALLBACK) == 1
    assert _counter(warm_profile, POOL_COVERAGE_DISPATCHED) == ROW
    fallen = [
        item
        for item in warm.domains
        if any(note.outcome == POOL_TASK_FALLBACK for note in item.diagnostics)
    ]
    assert len(fallen) == 1
    assert "worker died" in " ".join(
        note.message for note in fallen[0].diagnostics
    )
    skip = (fallen[0].patch_domain_id,)
    assert _fingerprint(
        warm, warm_profile, skip_scene_of=skip
    ) == _fingerprint(reference, reference_profile, skip_scene_of=skip)
    # Следующее нажатие воскрешает воркера, и все задачи доходят до воркеров.
    assert _counter(healed_profile, POOL_TASK_FALLBACK) == 0
    assert _counter(healed_profile, POOL_WORKERS) == 2
    assert _fingerprint(healed, healed_profile) == _fingerprint(
        healed_reference, healed_reference_profile
    )
    assert _session_state(pooled) == _session_state(sequential)


def test_an_unavailable_pool_covers_in_the_parent_and_names_the_first_domain(
    monkeypatch, capsys, _pool_always
):
    bundle = quad_row_bundle(ROW)
    pooled, sequential = _warm_pair(bundle)
    _in_process_session(
        monkeypatch, DomainPool(2, python_executable="C:/nowhere/python.exe")
    )

    warm, warm_profile, _ = _session_run(
        bundle, 0.4, workers=2, controller=pooled
    )
    reference, reference_profile, _ = _session_run(
        bundle, 0.4, workers=0, controller=sequential
    )

    first = warm.domains[0]
    assert _counter(warm_profile, POOL_UNAVAILABLE) == 1
    assert _counter(warm_profile, POOL_DISPATCHED) == 0
    assert _counter(warm_profile, POOL_COVERAGE_DISPATCHED) == 0
    notices = [
        item for item in warm.diagnostics if item.outcome == POOL_UNAVAILABLE
    ]
    assert [item.patch_domain_id for item in notices] == [first.patch_domain_id]
    assert POOL_UNAVAILABLE in capsys.readouterr().out
    assert _fingerprint(
        warm, warm_profile, skip_scene_of=(first.patch_domain_id,)
    ) == _fingerprint(
        reference, reference_profile, skip_scene_of=(first.patch_domain_id,)
    )
    assert _session_state(pooled) == _session_state(sequential)


def test_a_preparation_that_cannot_be_shipped_is_a_named_fallback(
    monkeypatch, capsys, _pool_always
):
    bundle = quad_row_bundle(ROW)
    pooled, sequential = _warm_pair(bundle)
    _in_process_session(monkeypatch, _InProcessPool())
    victim_prepared = list(pooled._conveyor_preparation_cache.values())[2]
    blobs = pooled.preparation_blobs
    original = blobs.blob_of

    def refusing(prepared):
        if prepared is victim_prepared:
            raise TypeError("cannot pickle 'mpf' object")
        return original(prepared)

    monkeypatch.setattr(blobs, "blob_of", refusing)

    warm, warm_profile, _ = _session_run(
        bundle, 0.4, workers=2, controller=pooled
    )
    reference, reference_profile, _ = _session_run(
        bundle, 0.4, workers=0, controller=sequential
    )

    fallen = [
        item
        for item in warm.domains
        if any(note.outcome == POOL_TASK_FALLBACK for note in item.diagnostics)
    ]
    assert len(fallen) == 1
    message = " ".join(note.message for note in fallen[0].diagnostics)
    assert "cannot be shipped" in message and "cannot pickle" in message
    assert _counter(warm_profile, POOL_TASK_FALLBACK) == 1
    # Не дошла до воркера: ни отправленной, ни покрытой им.
    assert _counter(warm_profile, POOL_DISPATCHED) == ROW - 1
    assert POOL_TASK_FALLBACK in capsys.readouterr().out
    skip = (fallen[0].patch_domain_id,)
    assert _fingerprint(
        warm, warm_profile, skip_scene_of=skip
    ) == _fingerprint(reference, reference_profile, skip_scene_of=skip)


def _slider_entries(bundle, controller_workers=0):
    from cftuv.envelope_debug_session import remember_queue_session

    evaluation, _, controller = _session_run(bundle, workers=controller_workers)
    remember_queue_session(
        controller,
        "row",
        evaluation.topology_scene,
        evaluation.exact_debug_scenes,
        evaluation,
        density=None,
    )
    return controller, controller.queue_session.entries


def _timing_free_scene(scene):
    payload = queue_scene_payload(scene)
    for domain in payload["domains"]:
        for key in (
            "prepare_seconds",
            "coverage_seconds",
            "contour_seconds",
            "timings",
        ):
            domain.pop(key)
    return payload


def test_the_slider_covers_in_real_workers_and_the_scene_is_identical(
    _pool_always,
):
    from cftuv.envelope_queue_export import recompute_queue_coverage

    controller, entries = _slider_entries(quad_row_bundle(ROW))
    pool = get_domain_pool(2)
    profile = EnvelopeDebugProfileBuilderV1("row", "QUEUE")
    slider = pool_module.peek_domain_pool(2)
    assert slider is None  # пула ещё нет: ползунок его не заводит
    assert controller.slider_coverage_pool(2, profile) is None
    pool.ensure_started()
    coverage_pool = controller.slider_coverage_pool(2, profile)
    assert coverage_pool is not None
    for alpha in ("0.4", "0.3", "0.4"):
        pooled = recompute_queue_coverage(
            entries, alpha, coverage_pool=coverage_pool
        )
        reference = recompute_queue_coverage(entries, alpha)
        assert _timing_free_scene(pooled) == _timing_free_scene(reference)
        assert all(
            item.preparation is prepared
            for item, (_, _, prepared) in zip(pooled.domains, entries)
        )
    snapshot = profile.snapshot()
    assert _counter(profile, POOL_COVERAGE_DISPATCHED) == ROW
    assert _counter(profile, POOL_TASK_FALLBACK) == 0
    text = queue_timing_text(pooled, snapshot)
    assert "pool wall" in text and "on 2 workers" in text
    # Пиклы сняты один раз на подготовку, а не на каждый шаг ползунка.
    assert len(controller.preparation_blobs) == ROW


def test_a_small_slider_batch_never_leaves_the_parent(_pool_never):
    from cftuv.envelope_queue_export import recompute_queue_coverage

    controller, entries = _slider_entries(quad_row_bundle(ROW))
    pool = get_domain_pool(2)
    pool.ensure_started()
    profile = EnvelopeDebugProfileBuilderV1("row", "QUEUE")
    coverage_pool = controller.slider_coverage_pool(2, profile)

    scene = recompute_queue_coverage(entries, "0.4", coverage_pool=coverage_pool)

    assert _timing_free_scene(scene) == _timing_free_scene(
        recompute_queue_coverage(entries, "0.4")
    )
    assert _counter(profile, POOL_WORKERS) is None
    assert POOL_WALL_STAGE not in profile.snapshot().stage_totals
    assert queue_timing_text(scene, profile.snapshot()) == queue_timing_text(scene)


@pytest.mark.parametrize("injection", ("fail", "drop"))
def test_a_failed_slider_task_is_named_counted_and_covered_in_the_parent(
    injection, monkeypatch, capsys, _pool_always
):
    from cftuv.envelope_queue_export import recompute_queue_coverage
    from cftuv.envelope_queue_pool import SliderCoveragePool

    controller, entries = _slider_entries(quad_row_bundle(ROW))
    profile = EnvelopeDebugProfileBuilderV1("row", "QUEUE")
    coverage_pool = SliderCoveragePool(
        _InProcessPool(**{injection: (2,)}), controller.preparation_blobs, profile
    )

    scene = recompute_queue_coverage(entries, "0.4", coverage_pool=coverage_pool)

    assert _timing_free_scene(scene) == _timing_free_scene(
        recompute_queue_coverage(entries, "0.4")
    )
    assert _counter(profile, POOL_TASK_FALLBACK) == 1
    assert _counter(profile, POOL_DISPATCHED) == ROW
    assert _counter(profile, POOL_COVERAGE_DISPATCHED) == ROW
    assert POOL_TASK_FALLBACK in capsys.readouterr().out
    text = queue_timing_text(scene, profile.snapshot())
    assert "pool fallback on 1 domains" in text


def test_the_slider_names_an_unavailable_pool_and_stays_sequential(capsys):
    from cftuv.envelope_queue_export import recompute_queue_coverage
    from cftuv.envelope_queue_pool import SliderCoveragePool
    from cftuv import envelope_queue_pool

    controller, entries = _slider_entries(quad_row_bundle(ROW))
    profile = EnvelopeDebugProfileBuilderV1("row", "QUEUE")
    coverage_pool = SliderCoveragePool(
        DomainPool(2, python_executable="C:/nowhere/python.exe"),
        controller.preparation_blobs,
        profile,
    )
    original = envelope_queue_pool.COVERAGE_POOL_MIN_BYTES
    envelope_queue_pool.COVERAGE_POOL_MIN_BYTES = 0
    try:
        scene = recompute_queue_coverage(
            entries, "0.4", coverage_pool=coverage_pool
        )
    finally:
        envelope_queue_pool.COVERAGE_POOL_MIN_BYTES = original

    assert _timing_free_scene(scene) == _timing_free_scene(
        recompute_queue_coverage(entries, "0.4")
    )
    assert _counter(profile, POOL_UNAVAILABLE) == 1
    assert _counter(profile, POOL_DISPATCHED) == 0
    assert POOL_UNAVAILABLE in capsys.readouterr().out
    assert "pool unavailable" in queue_timing_text(scene, profile.snapshot())


def test_the_slider_never_starts_a_pool_and_ignores_a_dead_or_resized_one():
    shutdown_domain_pool()
    assert pool_module.peek_domain_pool(2) is None
    pool = get_domain_pool(2)
    assert pool_module.peek_domain_pool(2) is None  # заведён, но не запущен
    pool.ensure_started()
    assert pool_module.peek_domain_pool(2) is pool
    assert pool_module.peek_domain_pool(3) is None
    assert pool_module.peek_domain_pool(0) is None
    for worker in pool._workers:
        worker.process.kill()
        worker.process.wait()
        worker.dead = True
    assert pool_module.peek_domain_pool(2) is None
    shutdown_domain_pool()


def test_a_pooled_coverage_does_not_charge_the_parents_budget():
    """Названное расхождение: бюджет подготовки в родителе от покрытия воркера не растёт.

    В последовательном пути `conveyor_coverage` тратит из того же `work_budget`,
    что и подготовка, и подготовка из кэша копит расход от шага к шагу. Воркер
    считает на копии: ответ тот же, а счёт родителя остаётся на подготовке.
    Исчерпать потолок (2^23 единиц) так можно только тысячами холодных покрытий
    одного домена, поэтому на ответ это не влияет; тест держит само различие
    видимым, чтобы его не «починили» молча в одну из сторон.
    """

    from cftuv_envelope import exact_sqrt_sum
    from cftuv.envelope_queue_export import recompute_queue_coverage
    from cftuv.envelope_queue_pool import PreparationBlobsV1, SliderCoveragePool

    task = _field_task()
    prepared, _ = run_queue_domain(
        task.patch_id, task.domain_id, task.snapshot, task.request, "0.25"
    )
    entries = [(task.patch_id, task.domain_id, prepared)]
    start = prepared.work_budget.spent

    exact_sqrt_sum.reset_factorization_memory()
    recompute_queue_coverage(entries, "0.4")
    charged = prepared.work_budget.spent
    assert charged > start, (start, charged)  # контроль: покрытие тратит

    import cftuv.envelope_queue_pool as queue_pool

    original = queue_pool.COVERAGE_POOL_MIN_BYTES
    queue_pool.COVERAGE_POOL_MIN_BYTES = 0
    try:
        profile = EnvelopeDebugProfileBuilderV1("field", "QUEUE")
        pooled = SliderCoveragePool(_InProcessPool(), PreparationBlobsV1(), profile)
        scene = recompute_queue_coverage(entries, "0.3", coverage_pool=pooled)
    finally:
        queue_pool.COVERAGE_POOL_MIN_BYTES = original

    assert _counter(profile, POOL_COVERAGE_DISPATCHED) == 1
    assert prepared.work_budget.spent == charged
    assert scene.domains[0].coverage_outcome == "EXACT"


def test_a_slider_preparation_that_cannot_be_shipped_is_named_in_a_small_batch(
    monkeypatch, capsys, _pool_never
):
    from cftuv.envelope_queue_export import recompute_queue_coverage
    from cftuv.envelope_queue_pool import SliderCoveragePool

    controller, entries = _slider_entries(quad_row_bundle(ROW))
    blobs = controller.preparation_blobs
    victim = entries[1][2]
    original = blobs.blob_of

    def refusing(prepared):
        if prepared is victim:
            raise TypeError("cannot pickle 'mpf' object")
        return original(prepared)

    monkeypatch.setattr(blobs, "blob_of", refusing)
    profile = EnvelopeDebugProfileBuilderV1("row", "QUEUE")

    scene = recompute_queue_coverage(
        entries,
        "0.4",
        coverage_pool=SliderCoveragePool(_InProcessPool(), blobs, profile),
    )

    assert _timing_free_scene(scene) == _timing_free_scene(
        recompute_queue_coverage(entries, "0.4")
    )
    assert _counter(profile, POOL_TASK_FALLBACK) == 1
    assert _counter(profile, POOL_DISPATCHED) == 0
    out = capsys.readouterr().out
    assert POOL_TASK_FALLBACK in out and "cannot be shipped" in out
    assert "pool fallback on 1 domains" in queue_timing_text(
        scene, profile.snapshot()
    )


# --------------------------------------------------------------------------
# 8. Интерпретатор воркеров («Worker Python», срез WORKER-PYTHON)
# --------------------------------------------------------------------------


def _host_entries():
    """Каталоги пакетов хоста: единственное, что внешний воркер берёт у родителя."""

    entries = []
    for name in pool_module.HOST_PACKAGES:
        entry = os.path.dirname(pool_module.package_directory(name))
        if entry not in entries:
            entries.append(entry)
    return entries


def _normal(path):
    return os.path.normcase(os.path.abspath(path))


def test_an_external_worker_inherits_no_path_but_the_host_packages(
    monkeypatch, tmp_path
):
    """Ни stdlib, ни site-packages родителя внешнему воркеру не достаются.

    В `sys.path` родителя подложен «чужой stdlib» с модулем-маркёром: у
    встроенного воркера он был бы виден, у внешнего — нет. Пакеты хоста (ядро,
    `sympy`, `mpmath`) стоят ПОСЛЕ stdlib самого воркера, чтобы не затенить её;
    каталог, из которого загружен сам аддон, не нужен вовсе (пакет поднимается по
    файлу) и не передаётся.
    """

    fake = tmp_path / "fake_python311_stdlib"
    fake.mkdir()
    (fake / "cftuv_fake_stdlib_marker.py").write_text("VALUE = 1\n")
    monkeypatch.syspath_prepend(str(fake))
    assert any(_normal(item) == _normal(fake) for item in sys.path if item)
    pool = DomainPool(1, external_python=sys.executable)
    try:
        pool.ensure_started()
        identity = pool._workers[0].identity
        worker_path = [_normal(item) for item in identity["sys_path"]]
        assert _normal(fake) not in worker_path
        assert _normal(KERNEL_SRC.parent.parent) not in worker_path
        assert len(worker_path) == len(set(worker_path))
        entries = [_normal(item) for item in _host_entries()]
        assert entries
        stdlib = worker_path.index(_normal(os.path.dirname(os.__file__)))
        assert all(worker_path.index(entry) > stdlib for entry in entries)
        # Позади stdlib стоят ровно пакеты хоста и ничего больше.
        assert worker_path[worker_path.index(entries[0]) :] == entries
        assert identity["python"] == tuple(sys.version_info[:3])
        assert pool.interpreter == pool_module.PoolInterpreterV1(
            tuple(sys.version_info[:3]), True
        )
        assert pool.interpreter.version_code == (
            sys.version_info[0] * 10000
            + sys.version_info[1] * 100
            + sys.version_info[2]
        )
        assert not pool.rejected
    finally:
        pool.close()


def test_the_bundled_worker_keeps_the_whole_parent_path_and_no_handshake():
    """Встроенный путь не изменился: тот же `sys.path` родителя, ни одного `hello`."""

    specification = pool_module._worker_specification()
    assert set(specification) == {"sys_path", "package", "package_dir"}
    assert specification["sys_path"] == [
        os.path.abspath(item)
        for item in sys.path
        if isinstance(item, str) and item
    ]
    pool = DomainPool(1)
    try:
        pool.ensure_started()
        assert pool._workers[0].identity == {}
        assert pool.interpreter == pool_module.PoolInterpreterV1(
            tuple(sys.version_info[:3]), False
        )
        assert not pool.rejected
    finally:
        pool.close()


@pytest.mark.parametrize(
    ("key", "code"),
    (
        ("sympy", 5),
        ("mpmath", 6),
        ("numeric_backend", 7),
        ("kernel_fingerprint", 8),
    ),
)
def test_a_mismatching_worker_environment_is_named_and_the_pool_falls_back(
    monkeypatch, key, code
):
    """Расхождение окружений — `INTERPRETER_MISMATCH`, а воркеры идут на встроенном."""

    real = pool_module.describe_environment()
    monkeypatch.setattr(
        pool_module,
        "describe_environment",
        lambda: {**real, key: "0.0.0-not-what-the-worker-runs"},
    )
    pool = DomainPool(2, external_python=sys.executable)
    try:
        pool.ensure_started()
        interpreter = pool.interpreter
        assert pool.worker_count == 2 and pool.rejected
        assert not interpreter.external
        assert interpreter.outcome == pool_module.INTERPRETER_MISMATCH
        assert interpreter.reason_code == code
        assert pool_module.INTERPRETER_REASONS[code] in interpreter.reason
        assert "0.0.0-not-what-the-worker-runs" in interpreter.reason
        assert all(worker.identity == {} for worker in pool._workers)
    finally:
        pool.close()


def test_an_old_python_is_a_named_mismatch(monkeypatch):
    host = {
        "sympy": "1",
        "mpmath": "1",
        "numeric_backend": "b",
        "kernel_fingerprint": "k",
    }
    difference = pool_module._identity_difference
    assert difference(host, {**host, "python": (3, 9, 7)})[0] == 4
    assert difference(host, {**host, "python": (3, 10, 0)}) is None
    assert difference(host, {**host, "python": None})[0] == 4

    monkeypatch.setattr(pool_module, "MIN_EXTERNAL_PYTHON", (99, 0))
    pool = DomainPool(1, external_python=sys.executable)
    try:
        pool.ensure_started()
        assert pool.interpreter.outcome == pool_module.INTERPRETER_MISMATCH
        assert pool.interpreter.reason_code == 4
        assert "older than 3.10" in pool.interpreter.reason
    finally:
        pool.close()


def test_a_mismatch_runs_on_the_bundled_python_and_is_named_everywhere(
    monkeypatch, capsys
):
    """Откат на встроенный: ответ тот же, исход назван диагностикой, консолью и панелью."""

    bundle = quad_row_bundle(ROW)
    expected, expected_profile = _direct_run(bundle, pool=None)
    real = pool_module.describe_environment()
    monkeypatch.setattr(
        pool_module,
        "describe_environment",
        lambda: {**real, "sympy": "0.0.0-not-what-the-worker-runs"},
    )
    pool = DomainPool(2, external_python=sys.executable)
    try:
        evaluation, profile = _direct_run(bundle, pool=pool)
    finally:
        pool.close()

    assert _counter(profile, POOL_INTERPRETER_FALLBACK) == 1
    assert _counter(profile, POOL_INTERPRETER_REASON) == 5
    assert _counter(profile, POOL_EXTERNAL_PYTHON) == 0
    # Откат на встроенный интерпретатор, а не на последовательный путь.
    assert _counter(profile, POOL_UNAVAILABLE) == 0
    assert _counter(profile, POOL_WORKERS) == 2
    assert _counter(profile, POOL_DISPATCHED) == ROW
    assert _counter(profile, POOL_TASK_FALLBACK) == 0
    first = evaluation.domains[0]
    notices = [
        item
        for item in evaluation.diagnostics
        if item.outcome == pool_module.INTERPRETER_MISMATCH
    ]
    assert [item.patch_domain_id for item in notices] == [first.patch_domain_id]
    assert "sympy version differs" in notices[0].message
    assert "bundled Python" in notices[0].message
    console = capsys.readouterr().out
    assert pool_module.INTERPRETER_MISMATCH in console
    assert _fingerprint(
        evaluation, profile, skip_scene_of=(first.patch_domain_id,)
    ) == _fingerprint(
        expected, expected_profile, skip_scene_of=(first.patch_domain_id,)
    )
    text = queue_timing_text(
        build_queue_scene(evaluation.queue_domains), profile.snapshot()
    )
    assert "external Python rejected: sympy version differs" in text
    assert "(bundled)" in text and "pool wall" in text


def test_an_unusable_path_is_named_and_the_pool_falls_back(tmp_path):
    not_python = tmp_path / "tool.exe"
    not_python.write_text("x")
    garbage = tmp_path / "python.exe"
    garbage.write_text("this is not an executable")
    for path, fragment in (
        (str(tmp_path / "nowhere" / "python.exe"), "not a file"),
        (str(tmp_path), "not a file"),
        (str(not_python), "not a Python interpreter"),
        (str(garbage), "cannot start"),
    ):
        pool = DomainPool(1, external_python=path)
        try:
            pool.ensure_started()
            interpreter = pool.interpreter
            assert pool.worker_count == 1 and pool.rejected, path
            assert not interpreter.external
            assert interpreter.outcome == pool_module.INTERPRETER_UNUSABLE
            assert interpreter.reason_code in (1, 2)
            assert fragment in interpreter.reason, (path, interpreter.reason)
        finally:
            pool.close()


def test_an_external_worker_that_dies_is_unusable_and_both_failures_are_named(
    monkeypatch,
):
    monkeypatch.setattr(
        pool_module, "_BOOTSTRAP", "import sys\nsys.exit('boom from bootstrap')"
    )
    pool = DomainPool(1, external_python=sys.executable)

    with pytest.raises(DomainPoolUnavailable) as caught:
        pool.ensure_started()

    assert "external Python rejected" in str(caught.value)
    assert "boom from bootstrap" in str(caught.value)
    assert "bundled Python failed too" in str(caught.value)
    assert pool.rejected and pool.worker_count == 0
    pool.close()


def test_a_rejected_or_changed_external_python_gets_a_fresh_pool():
    shutdown_domain_pool()
    try:
        first = get_domain_pool(2, "")
        assert get_domain_pool(2, "") is first
        assert get_domain_pool(2) is first
        other = get_domain_pool(2, "C:/x/python.exe")
        assert other is not first and other.external_python == "C:/x/python.exe"
        assert get_domain_pool(2, "C:/x/python.exe") is other
        other._rejection = pool_module._InterpreterRejected(5, "sympy differs")
        assert other.rejected
        assert get_domain_pool(2, "C:/x/python.exe") is not other
    finally:
        shutdown_domain_pool()


def test_the_interpreter_reaches_the_owner_text_through_numeric_counters():
    profile = EnvelopeDebugProfileBuilderV1("row", "QUEUE")
    scene = build_queue_scene(())
    assert "worker Python" not in queue_timing_text(scene, profile.snapshot())
    profile.add_timing(POOL_WALL_STAGE, 0.5)
    profile.set_counter(POOL_WORKERS, 8)
    profile.set_counter(POOL_PYTHON_VERSION, 31301)
    profile.set_counter(POOL_EXTERNAL_PYTHON, 1)
    profile.set_counter(POOL_INTERPRETER_FALLBACK, 0)
    text = queue_timing_text(scene, profile.snapshot())
    assert "pool wall 500 ms on 8 workers" in text
    assert "worker Python 3.13.1 (external)" in text
    assert "rejected" not in text
    profile.set_counter(POOL_PYTHON_VERSION, 31111)
    profile.set_counter(POOL_EXTERNAL_PYTHON, 0)
    profile.set_counter(POOL_INTERPRETER_FALLBACK, 1)
    profile.set_counter(POOL_INTERPRETER_REASON, 8)
    text = queue_timing_text(scene, profile.snapshot())
    assert "worker Python 3.11.11 (bundled)" in text
    assert "external Python rejected: kernel source differs" in text


def test_real_external_workers_reproduce_the_sequential_run(
    monkeypatch, _pool_never
):
    """Ответ на внешнем интерпретаторе побитово тот же; кнопка берёт путь из настройки."""

    from cftuv import envelope_worker_python

    monkeypatch.setattr(
        envelope_worker_python, "read_worker_python", lambda: sys.executable
    )
    bundle = quad_row_bundle(ROW, lifted_corner=1.0)
    expected, expected_profile, sequential = _session_run(bundle, workers=0)

    evaluation, profile, controller = _session_run(bundle, workers=2)

    assert _fingerprint(evaluation, profile) == _fingerprint(
        expected, expected_profile
    )
    assert _counter(profile, POOL_EXTERNAL_PYTHON) == 1
    assert _counter(profile, POOL_INTERPRETER_FALLBACK) == 0
    assert _counter(
        profile, POOL_PYTHON_VERSION
    ) == pool_module.PoolInterpreterV1(
        tuple(sys.version_info[:3]), True
    ).version_code
    assert _counter(profile, POOL_WORKERS) == 2
    assert _counter(profile, POOL_DISPATCHED) == ROW
    assert _counter(profile, POOL_TASK_FALLBACK) == 0
    assert _counter(profile, POOL_UNAVAILABLE) == 0
    assert controller.build_counts == sequential.build_counts
    assert "(external)" in queue_timing_text(
        build_queue_scene(evaluation.queue_domains), profile.snapshot()
    )


def test_the_preference_is_read_cleaned_and_empty_by_default(monkeypatch):
    from types import SimpleNamespace

    from cftuv import envelope_worker_python as preference

    stub = sys.modules["bpy"]
    assert preference.read_worker_python() == ""
    monkeypatch.setattr(stub, "context", SimpleNamespace(), raising=False)
    assert preference.read_worker_python() == ""
    addons = {}
    monkeypatch.setattr(
        stub,
        "context",
        SimpleNamespace(preferences=SimpleNamespace(addons=addons)),
        raising=False,
    )
    assert preference.read_worker_python() == ""
    addons[preference.__package__] = SimpleNamespace(
        preferences=SimpleNamespace(worker_python='  "C:/Python313/python.exe" ')
    )
    assert preference.read_worker_python() == "C:/Python313/python.exe"
    addons[preference.__package__].preferences.worker_python = "   "
    assert preference.read_worker_python() == ""


def test_the_preference_is_attached_to_the_registered_class_once(monkeypatch):
    """Свойство вносится в аннотации класса между его снятием и новой регистрацией."""

    from types import SimpleNamespace

    from cftuv import envelope_worker_python as preference

    class Base:
        pass

    class Stale(Base):
        # Сброшенный класс прежней загрузки пакета: тот же `bl_idname`, но не
        # зарегистрирован; пристёгивать к нему нечего и снимать его нельзя.
        bl_idname = preference.__package__
        is_registered = False
        __annotations__ = {}

    class Preferences(Base):
        bl_idname = preference.__package__
        is_registered = True
        __annotations__ = {"clear_pins_after_phase1": "kept"}

    calls = []
    stub = sys.modules["bpy"]
    monkeypatch.setattr(
        stub, "types", SimpleNamespace(AddonPreferences=Base), raising=False
    )
    monkeypatch.setattr(
        stub,
        "utils",
        SimpleNamespace(
            unregister_class=lambda cls: calls.append("unregister"),
            register_class=lambda cls: calls.append(
                ("register", preference.PREFERENCE_NAME in cls.__annotations__)
            ),
        ),
        raising=False,
    )
    monkeypatch.setattr(
        stub, "props", SimpleNamespace(StringProperty=lambda **kw: kw), raising=False
    )

    assert preference.install_worker_python_preference() is True
    assert calls == ["unregister", ("register", True)]
    assert preference.PREFERENCE_NAME not in Stale.__annotations__
    declared = Preferences.__annotations__[preference.PREFERENCE_NAME]
    assert declared["subtype"] == "FILE_PATH" and declared["default"] == ""
    assert Preferences.__annotations__["clear_pins_after_phase1"] == "kept"
    assert preference.install_worker_python_preference() is True
    assert calls == ["unregister", ("register", True)]  # повтор ничего не делает

    # Класс, который не принял свойство, возвращается к прежнему виду.
    del Preferences.__annotations__[preference.PREFERENCE_NAME]
    calls.clear()
    refusals = iter((RuntimeError("refused"), None))

    def register(cls):
        outcome = next(refusals)
        calls.append(
            ("register", preference.PREFERENCE_NAME in cls.__annotations__)
        )
        if outcome is not None:
            raise outcome

    monkeypatch.setattr(stub.utils, "register_class", register)
    assert preference.install_worker_python_preference() is False
    assert calls == ["unregister", ("register", True), ("register", False)]
    assert preference.PREFERENCE_NAME not in Preferences.__annotations__
