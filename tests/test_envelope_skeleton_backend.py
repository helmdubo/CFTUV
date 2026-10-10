"""Стадия SKELETON под блоком бэкенда в хосте: заказ, подготовка под блоком, запись скелета, ключи кэшей подготовки (SKELETON-NATIVE-PREP).

Скелет считается в ПОДГОТОВКЕ (`prepare_conveyor` -> `_prepare_region`), а подготовка идёт до `produce_domain` — в родителе и в воркерах пула. Нативное ядро здесь подставное
(`_fake_native`: отвечает эталоном и пишет, в каком блоке его позвали). Что держат тесты:

1. ПОРЯДОК СТАДИИ — как главный переключатель (`kernel_backend`): у скелета собственного умолчания и собственной настройки сцены нет; постадийный порядок задаёт только API
   (`run_production(skeleton_backend=...)`), и эти тесты изолируют скелет именно им.
2. БЛОК БЭКЕНДА СТОИТ ВОКРУГ ПОДГОТОВКИ на каждом пути, где её строят: родитель (`prepare_for_production`, кэш сессии), холодная задача воркера (`solve_cold_production_task`),
   очередь (`run_queue_domain`, в том числе задача `solve_task` и задача с выгрузкой `solve_exported_task`), провайдер подготовки отладочной сессии.
3. ЗАПИСЬ ДОМЕНА несёт скелет ОТДЕЛЬНО (`skeleton_*`), доезжает из воркера и сливается с записью материализации; строка журнала называет стадии порознь.
4. КЛЮЧИ НЕ СМЕШИВАЮТСЯ: подготовка Python не читается как подготовка Rust ни в кэше сессии, ни в хранилище по содержимому, ни в записях сборки.
5. ОТКАТЫ ИМЕНОВАНЫ: отказ порта — эталон и имя в строке; `NativeDivisionDiverged` — домен отказан по имени и без подготовки, в родителе и в воркере.
"""

from __future__ import annotations

import pickle

import pytest

from cftuv import envelope_domain_pool as pool_module
from cftuv import envelope_kernel_backend as host_backend
from cftuv import envelope_production_export as production
from cftuv import envelope_queue_export as queue_export
from cftuv.envelope_debug_session import EnvelopeDebugSessionController
from cftuv.envelope_production_export import PLACEMENT_CACHED, PLACEMENT_WORKER, run_production
from test_envelope_kernel_backend import BUILD_ID, _backend_state, _fake_native, _projection, _python  # noqa: F401 - фикстура и подставное ядро
from test_envelope_production_content import ROW, _press, pool, row  # noqa: F401 - фикстуры и помощник прогона
from test_envelope_production_export import _debug_build

from cftuv_envelope import backend as kernel_backend
from cftuv_envelope.wavefront.skeleton import build_skeleton as oracle_skeleton


def native_skeleton(module, *, calls=None, raising=None):
    """Подставляет `build_skeleton`: отвечает эталоном и пишет `(блок бэкенда скелета, поток)`; `raising` — исключение вместо ответа."""

    log = [] if calls is None else calls

    def build(polygon, *, split_search=None, work_budget=None, dense_hydration=False):
        log.append((kernel_backend.active_skeleton_backend().value, kernel_backend.active_backend().value))
        if raising is not None:
            raise raising()
        keywords = {"work_budget": work_budget, "dense_hydration": dense_hydration}
        if split_search is not None:
            keywords["split_search"] = split_search
        return oracle_skeleton(polygon, **keywords)

    module.build_skeleton = build
    kernel_backend.refresh_native()
    return log


def press(bundle, *, skeleton, backend="NATIVE", controller=None, workers=0, alpha=0.25):
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
        kernel_backend=backend,
        skeleton_backend=skeleton,
        embedding_backend="PYTHON",  # здесь изолируется стадия скелета; прогон без аргументов проверяет умолчания (в том числе сертификата вложения)
    )
    return run, controller


def keys_of(controller):
    return sorted(key[4] for key in controller._conveyor_preparation_cache)


# --------------------------------------------------------------------------
# 1. Умолчание стадии
# --------------------------------------------------------------------------


def test_the_skeleton_stage_follows_the_master_switch_and_has_no_setting_or_default_of_its_own():
    assert host_backend.DEFAULT_KERNEL_BACKEND == "NATIVE"
    for gone in ("DEFAULT_SKELETON_BACKEND", "skeleton_backend_of", "SKELETON_BACKEND_ITEMS", "SKELETON_SETTING_NAME"):
        assert not hasattr(host_backend, gone), gone
    # задача пула несёт порядок стадии: без слова - как главный переключатель, явное имя - порядок API
    task = pool_module.DomainTaskV1(1, 0, "d", None, None, "0.25", frozenset())
    assert (task.backend, task.skeleton_backend) == ("NATIVE", "NATIVE")
    assert pool_module.DomainTaskV1(1, 0, "d", None, None, "0.25", frozenset(), backend="PYTHON").skeleton_backend == "PYTHON"
    assert pickle.loads(pickle.dumps(pool_module.DomainTaskV1(1, 0, "d", None, None, "0.25", frozenset(), backend="PYTHON", skeleton_backend="NATIVE"))).skeleton_backend == "NATIVE"


def test_the_identity_of_the_stage_and_of_the_execution(monkeypatch):
    assert host_backend.skeleton_identity_of("PYTHON") == "PYTHON" and host_backend.skeleton_identity_of() == "NATIVE:unavailable"
    # идентичность двух стадий (сертификат вложения - третья, заказана явно на PYTHON: она не меняет строку)
    assert host_backend.backend_identity_of("NATIVE", "PYTHON", "PYTHON") == "NATIVE:unavailable"
    assert host_backend.backend_identity_of("NATIVE", embedding_backend="PYTHON") == "NATIVE:unavailable|skeleton=NATIVE:unavailable"
    assert host_backend.backend_identity_of("NATIVE", "NATIVE", "PYTHON") == "NATIVE:unavailable|skeleton=NATIVE:unavailable"
    # умолчание третьей стадии - Native (EMBEDDING_NATIVE_DEFAULT_V1): идентичность умолчания продукта несёт все три
    assert host_backend.backend_identity_of("NATIVE") == "NATIVE:unavailable|skeleton=NATIVE:unavailable|snap_embedding=NATIVE:unavailable"
    _fake_native()
    assert host_backend.skeleton_identity_of("NATIVE") == f"NATIVE:{BUILD_ID}"
    assert host_backend.backend_identity_of("PYTHON", "NATIVE", "PYTHON") == f"PYTHON|skeleton=NATIVE:{BUILD_ID}"
    assert host_backend.backend_identity_of("NATIVE", "NATIVE", "PYTHON") == f"NATIVE:{BUILD_ID}|skeleton=NATIVE:{BUILD_ID}"
    assert host_backend.backend_identity_of("NATIVE", "NATIVE") == f"NATIVE:{BUILD_ID}|skeleton=NATIVE:{BUILD_ID}|snap_embedding=NATIVE:{BUILD_ID}"


# --------------------------------------------------------------------------
# 2-3. Подготовка под блоком, запись скелета, строка журнала
# --------------------------------------------------------------------------


def test_a_native_skeleton_press_gives_the_python_answer_computes_the_skeleton_in_its_own_block_and_names_it_apart(row):
    module = _fake_native()
    calls = native_skeleton(module)
    reference, _ = press(row, skeleton="PYTHON", backend="PYTHON")
    assert calls == []  # стадия на PYTHON нативного скелета не зовёт

    native, controller = press(row, skeleton="NATIVE", backend="PYTHON")

    assert _projection(native) == _projection(reference)
    assert [item.content_digest for item in native.results] == [item.content_digest for item in reference.results]
    # подготовка шла под блоком: в момент нативного вызова скелет заказан NATIVE, а покрытие и резка — PYTHON
    assert calls == [("NATIVE", "PYTHON")] * ROW
    records = [item.backend_record for item in native.results]
    assert all(record.skeleton_requested == "NATIVE" and record.skeleton_ran == "native" for record in records)
    assert all((record.skeleton_native_calls, record.skeleton_python_calls, record.skeleton_outcomes) == (1, 0, ()) for record in records)
    assert all(record.requested == "PYTHON" and (record.native_calls, record.python_calls) == (0, 0) for record in records)
    line = host_backend.backend_console_lines(native.results, "PYTHON", "NATIVE")
    assert line == [f"[CFTUV][Production] BACKEND native: skeleton {ROW} | ordered python: coverage/clip, embedding"]
    assert production.production_timing_text(native).endswith(f" | native: skeleton {ROW}")
    assert native.skeleton_backend == "NATIVE" and native.kernel_backend == "PYTHON"
    assert keys_of(controller) == [f"NATIVE:{BUILD_ID}"] * ROW
    # явный PYTHON на обеих стадиях: ни записи, ни строки
    assert all(item.backend_record is None for item in reference.results)
    assert host_backend.backend_console_lines(reference.results, "PYTHON", "PYTHON") == []
    assert host_backend.backend_timing_suffix(reference.results, "PYTHON", "PYTHON") == ""


def test_the_default_press_computes_the_native_skeleton_and_names_its_actual_calls(row):
    module = _fake_native()
    calls = native_skeleton(module)

    run = _press(row, EnvelopeDebugSessionController(), workers=0)

    assert calls == [("NATIVE", "NATIVE")] * ROW
    assert run.skeleton_backend == "NATIVE" and run.kernel_backend == "NATIVE"
    assert all(item.backend_record.skeleton_requested == "NATIVE" and item.backend_record.skeleton_native_calls == 1 and not item.backend_record.skeleton_outcomes for item in run.results)
    line = host_backend.backend_console_lines(run.results, run.kernel_backend, run.skeleton_backend, run.embedding_backend)[0]
    # умолчание называет все три стадии списком: `native: coverage/clip N, skeleton N, embedding N`
    assert line.startswith("[CFTUV][Production] BACKEND native: coverage/clip ") and f", skeleton {ROW}, embedding " in line and f"skeleton {ROW} (" not in line  # скелет не откатывался
    assert all(item.backend_record.embedding_requested == "NATIVE" for item in run.results) and run.embedding_backend == "NATIVE"
    assert f", skeleton {ROW}, embedding " in production.production_timing_text(run)


def test_both_stages_native_are_named_apart_in_one_line(row):
    module = _fake_native()
    native_skeleton(module)
    run, _ = press(row, skeleton="NATIVE", backend="NATIVE")
    line = host_backend.backend_console_lines(run.results, "NATIVE", "NATIVE")[0]
    assert "BACKEND native: coverage/clip " in line and f", skeleton {ROW}" in line and line.endswith("ordered python: embedding")  # `press` изолирует скелет: вложение заказано на PYTHON


def test_a_cold_press_in_the_worker_pool_ships_the_stage_computes_the_skeleton_in_the_worker_block_and_brings_the_record_back(row, pool):
    module = _fake_native()
    calls = native_skeleton(module)
    tasks: list = []
    original = pool.run

    def spy(batch):
        tasks.extend(batch)
        return original(batch)

    pool.run = spy
    run, controller = press(row, skeleton="NATIVE", workers=2)

    assert tasks and {task.skeleton_backend for task in tasks} == {"NATIVE"} and {task.backend for task in tasks} == {"NATIVE"}
    assert all(item.placement == PLACEMENT_WORKER for item in run.results)
    assert calls == [("NATIVE", "NATIVE")] * ROW  # блок воркера (`solve_cold_production_task`), а не блок родителя
    assert all(item.backend_record.skeleton_ran == "native" and item.backend_record.skeleton_native_calls == 1 for item in run.results)
    assert _projection(run) == _projection(_python(row))
    assert keys_of(controller) == [f"NATIVE:{BUILD_ID}"] * ROW  # подготовка воркера вошла в кэш сессии под ключом своей стадии
    # заказ без слова про скелет — умолчание стадии, оно едет в задачу
    tasks.clear()
    _press(row, EnvelopeDebugSessionController(), workers=2)
    assert tasks and {task.skeleton_backend for task in tasks} == {"NATIVE"}


def test_a_warm_press_on_the_cached_preparations_does_not_compute_a_skeleton_and_the_line_says_so(row, pool):
    module = _fake_native()
    calls = native_skeleton(module)
    controller = EnvelopeDebugSessionController()
    press(row, skeleton="NATIVE", controller=controller, workers=2)
    assert len(calls) == ROW

    warm, _ = press(row, skeleton="NATIVE", controller=controller, workers=2, alpha=0.3)

    assert len(calls) == ROW  # ширина другая, подготовки из кэша: скелет не считался
    assert warm.counter(production.PRODUCTION_PREPARATION_BUILDS) == 0 and warm.counter(production.PRODUCTION_PREPARATION_REUSED) == ROW
    assert all(item.backend_record.skeleton_ran == "" for item in warm.results if item.placement != PLACEMENT_CACHED)
    assert ", skeleton 0" in host_backend.backend_console_lines(warm.results, "NATIVE", "NATIVE", "PYTHON")[0]


def test_a_press_without_the_wheel_gives_the_python_answer_and_names_the_unavailable_skeleton(row):
    reference, _ = press(row, skeleton="PYTHON", backend="PYTHON")
    native, _ = press(row, skeleton="NATIVE", backend="PYTHON")  # колеса в процессе нет (фикстура)

    assert _projection(native) == _projection(reference)
    assert all(item.backend_record.skeleton_outcomes == ("NATIVE_UNAVAILABLE",) and item.backend_record.skeleton_ran == "python" for item in native.results)
    assert f"python: skeleton {ROW} (NATIVE_UNAVAILABLE: patch 0, 1, 2, 3, 4)" in host_backend.backend_console_lines(native.results, "PYTHON", "NATIVE")[0]


def test_a_wheel_without_the_skeleton_operation_falls_back_by_name_and_keeps_coverage_and_clip_available(row):
    module = _fake_native()  # без `build_skeleton`: колесо до скелета
    assert not hasattr(module, "build_skeleton")
    reference, _ = press(row, skeleton="PYTHON", backend="PYTHON")
    native, _ = press(row, skeleton="NATIVE", backend="PYTHON")
    assert _projection(native) == _projection(reference)
    assert {item.backend_record.skeleton_outcomes for item in native.results} == {("NATIVE_UNAVAILABLE",)}
    assert "build_skeleton" in native.results[0].backend_record.skeleton_fallbacks[0][3]


# --------------------------------------------------------------------------
# 4. Ключи не смешиваются
# --------------------------------------------------------------------------


def test_a_native_skeleton_press_after_a_python_skeleton_press_rebuilds_every_preparation_and_the_next_one_is_warm(row):
    module = _fake_native()
    calls = native_skeleton(module)
    controller = EnvelopeDebugSessionController()

    python, _ = press(row, skeleton="PYTHON", controller=controller)
    assert python.counter(production.PRODUCTION_PREPARATION_BUILDS) == ROW and keys_of(controller) == ["PYTHON"] * ROW

    native, _ = press(row, skeleton="NATIVE", controller=controller)
    assert (native.counter(production.PRODUCTION_PREPARATION_BUILDS), native.counter(production.PRODUCTION_PREPARATION_REUSED)) == (ROW, 0)
    assert (native.counter(production.PRODUCTION_RESULT_CACHE_HIT), native.counter(production.PRODUCTION_RESULT_CACHE_MISS)) == (0, ROW)
    assert len(calls) == ROW and keys_of(controller) == sorted(["PYTHON"] * ROW + [f"NATIVE:{BUILD_ID}"] * ROW)
    assert _projection(native) == _projection(python)

    again, _ = press(row, skeleton="NATIVE", controller=controller)
    assert (again.counter(production.PRODUCTION_RESULT_CACHE_HIT), again.counter(production.PRODUCTION_RESULT_CACHE_MISS)) == (ROW, 0)
    assert all(item.placement == PLACEMENT_CACHED for item in again.results) and len(calls) == ROW

    back, _ = press(row, skeleton="PYTHON", controller=controller)  # подготовки Python на месте: ни пересборки, ни нативного вызова
    assert back.counter(production.PRODUCTION_RESULT_CACHE_HIT) == ROW and len(calls) == ROW


def test_a_rebuilt_wheel_is_another_preparation_key(row):
    module = _fake_native()
    native_skeleton(module)
    controller = EnvelopeDebugSessionController()
    press(row, skeleton="NATIVE", controller=controller)
    assert keys_of(controller) == [f"NATIVE:{BUILD_ID}"] * ROW

    module.native_build_id = lambda: "cd" * 32  # то же колесо, другая сборка
    kernel_backend.refresh_native()
    rebuilt, _ = press(row, skeleton="NATIVE", controller=controller)
    assert rebuilt.counter(production.PRODUCTION_PREPARATION_BUILDS) == ROW and rebuilt.counter(production.PRODUCTION_PREPARATION_REUSED) == 0
    assert sorted(set(keys_of(controller))) == sorted({f"NATIVE:{BUILD_ID}", "NATIVE:" + "cd" * 32})


def test_the_debug_session_preparation_is_keyed_by_the_default_stage_and_a_production_press_of_the_other_stage_does_not_read_it(row):
    module = _fake_native()
    calls = native_skeleton(module)
    controller = EnvelopeDebugSessionController()

    _debug_build(row, controller)  # провайдер подготовки отладочной сессии: умолчание стадии (`NATIVE`)
    assert len(calls) == ROW and keys_of(controller) == [f"NATIVE:{BUILD_ID}"] * ROW

    same, _ = press(row, skeleton="NATIVE", backend="NATIVE", controller=controller)  # умолчание: подготовки отладки берутся
    assert same.counter(production.PRODUCTION_PREPARATION_BUILDS) == 0 and same.counter(production.PRODUCTION_PREPARATION_REUSED) == ROW

    python, _ = press(row, skeleton="PYTHON", controller=controller)  # другая стадия: подготовки отладки не читаются
    assert python.counter(production.PRODUCTION_PREPARATION_BUILDS) == ROW and python.counter(production.PRODUCTION_PREPARATION_REUSED) == 0
    assert len(calls) == ROW


def test_the_content_key_and_the_scan_memo_key_carry_the_stage(row, monkeypatch):
    from test_envelope_content_key import _domains
    from cftuv import envelope_scan_memo
    from cftuv.envelope_content_key import domain_content_key, execution_identity

    export, selected = next(iter(_domains(row).values()))
    assert domain_content_key(export, selected) == domain_content_key(export, selected, None, "NATIVE", "NATIVE")
    assert domain_content_key(export, selected, None, "NATIVE", "NATIVE") != domain_content_key(export, selected, None, "NATIVE", "PYTHON")
    assert domain_content_key(export, selected, None, "PYTHON", "NATIVE") != domain_content_key(export, selected, None, "PYTHON", "PYTHON")
    assert execution_identity("NATIVE", "NATIVE") != execution_identity("NATIVE", "PYTHON")
    assert execution_identity() == execution_identity("NATIVE", "NATIVE")

    # запись сборки несёт ключ подготовки, поэтому память записей ключится стадией: запись под другим скелетом не принимается
    seen: list = []
    real = envelope_scan_memo.scan_key
    monkeypatch.setattr(envelope_scan_memo, "scan_key", lambda run: seen.append(real(run)) or seen[-1])
    controller = EnvelopeDebugSessionController()
    press(row, skeleton="PYTHON", controller=controller)
    press(row, skeleton="NATIVE", controller=controller)
    press(row, skeleton="NATIVE", controller=controller, alpha=0.3)
    python_key, native_key, native_again = seen
    assert python_key != native_key and native_key == native_again
    assert python_key[-1] == "PYTHON" and native_key[-1] == "NATIVE:unavailable"


# --------------------------------------------------------------------------
# 5. Откаты и отказы названы
# --------------------------------------------------------------------------


def test_a_named_refusal_of_the_native_skeleton_gives_the_python_answer_and_names_every_domain(row):
    module = _fake_native()
    native_skeleton(module, raising=lambda: module.NativePortStale("the skeleton files moved"))
    reference, _ = press(row, skeleton="PYTHON", backend="PYTHON")

    native, controller = press(row, skeleton="NATIVE", backend="PYTHON")

    assert _projection(native) == _projection(reference)
    assert all(item.backend_record.skeleton_outcomes == ("NATIVE_PORT_STALE",) and item.backend_record.skeleton_ran == "python" for item in native.results)
    assert f"python: skeleton {ROW} (NATIVE_PORT_STALE: patch 0, 1, 2, 3, 4)" in host_backend.backend_console_lines(native.results, "PYTHON", "NATIVE")[0]
    assert keys_of(controller) == [f"NATIVE:{BUILD_ID}"] * ROW  # ключ называет заказ; откат назван в записи, а не в ключе


def _diverged_press(row, workers):
    module = _fake_native()
    calls = native_skeleton(module, raising=lambda: module.NativeDivisionDiverged("the generic division did not finish"))
    controller = EnvelopeDebugSessionController()

    native, _ = press(row, skeleton="NATIVE", backend="PYTHON", controller=controller, workers=workers)

    assert len(calls) == ROW  # эталону домен не отдан: ни одного повторного счёта
    assert {item.outcome for item in native.results} == {"NATIVE_DIVISION_DIVERGED"}
    assert all(item.batch is None and "skeleton: " in item.detail for item in native.results)
    assert all(item.backend_record.skeleton_outcomes == ("NATIVE_DIVISION_DIVERGED",) for item in native.results)
    assert not controller._conveyor_preparation_cache and controller.build_counts["CONVEYOR_PREPARATION"] == 0
    assert "NATIVE_DIVISION_DIVERGED: patch" in host_backend.backend_console_lines(native.results, "PYTHON", "NATIVE")[0]
    return native


def test_a_diverged_native_skeleton_refuses_the_domain_by_name_in_the_parent_and_leaves_no_preparation(row):
    native = _diverged_press(row, 0)
    assert {item.placement for item in native.results} == {production.PLACEMENT_PARENT}


def test_a_diverged_native_skeleton_refuses_the_domain_by_name_in_the_worker_and_the_parent_caches_no_preparation(row, pool):
    native = _diverged_press(row, 2)
    assert {item.placement for item in native.results} == {PLACEMENT_WORKER}


def test_a_native_skeleton_refusal_after_visible_effects_refuses_the_domain_instead_of_computing_on_dirty_state(row):
    module = _fake_native()
    asked: list = []

    def late(polygon, *, split_search=None, work_budget=None, dense_hydration=False):
        asked.append(1)
        work_budget.superlevel = "moved-by-the-port"  # видимое состояние сдвинуто, а порт отказал
        raise module.NativePortUnsupported("late")

    module.build_skeleton = late
    kernel_backend.refresh_native()

    native, _ = press(row, skeleton="NATIVE", backend="PYTHON")

    assert asked
    assert {item.outcome for item in native.results} == {"NATIVE_PARTIAL_EFFECTS_REFUSED"}
    assert all(item.backend_record.skeleton_outcomes == ("NATIVE_PARTIAL_EFFECTS_REFUSED",) for item in native.results)


# --------------------------------------------------------------------------
# 2. Подготовка под блоком на остальных путях
# --------------------------------------------------------------------------


def _first_domain_inputs(bundle, monkeypatch):
    """`(snapshot, request)` первого домена ряда: ровно то, что родитель отдаёт подготовке (берётся шпионом из настоящего прогона)."""

    captured: list = []
    real = production.prepare_for_production_recorded

    def spy(snapshot, request, **kwargs):
        captured.append((snapshot, request))
        return real(snapshot, request, **kwargs)

    with monkeypatch.context() as scope:
        scope.setattr(production, "prepare_for_production_recorded", spy)
        press(bundle, skeleton="PYTHON", backend="PYTHON")
    assert len(captured) == ROW
    return captured[0]


def test_prepare_for_production_runs_the_preparation_in_the_block_and_returns_the_skeleton_record(row, monkeypatch):
    snapshot, request = _first_domain_inputs(row, monkeypatch)
    module = _fake_native()
    calls = native_skeleton(module)

    prepared, record = production.prepare_for_production_recorded(snapshot, request, backend="PYTHON", skeleton_backend="NATIVE")
    assert calls == [("NATIVE", "PYTHON")] and (record.skeleton_ran, record.skeleton_native_calls) == ("native", 1)
    plain = production.prepare_for_production(snapshot, request, backend="PYTHON", skeleton_backend="PYTHON")
    assert plain.outcome == prepared.outcome and len(calls) == 1
    assert production.prepare_for_production_recorded(snapshot, request, backend="PYTHON", skeleton_backend="PYTHON", embedding_backend="PYTHON")[1] is None
    # умолчание продукта: все стадии заказаны NATIVE; запись подготовки называет настоящий вызов скелета
    default_record = production.prepare_for_production_recorded(snapshot, request)[1]
    assert default_record.requested == "NATIVE" and default_record.skeleton_requested == "NATIVE" and default_record.skeleton_ran == "native" and default_record.skeleton_native_calls == 1

    native_skeleton(module, raising=lambda: module.NativeDivisionDiverged("did not finish"))
    with pytest.raises(host_backend.PreparationRefused, match="NATIVE_DIVISION_DIVERGED") as refused:
        production.prepare_for_production(snapshot, request, backend="PYTHON", skeleton_backend="NATIVE")
    assert refused.value.outcome == "NATIVE_DIVISION_DIVERGED" and refused.value.record.skeleton_outcomes == ("NATIVE_DIVISION_DIVERGED",)


def test_the_queue_preparation_runs_in_the_block_through_the_session_provider_and_the_pool_tasks(row, monkeypatch):
    from cftuv.envelope_domain_pool import DomainTaskV1, solve_task
    from cftuv.envelope_export_input import solve_exported_task

    snapshot, request = _first_domain_inputs(row, monkeypatch)
    module = _fake_native()
    calls = native_skeleton(module)
    controller = EnvelopeDebugSessionController()

    # `run_queue_domain` ставит блок вокруг всего шага подготовки, и провайдер кэша сессии (как у отладочной кнопки) строит подготовку изнутри блока
    def provider(_patch, domain_id, selected, snapshot_, request_):
        from cftuv_envelope.wavefront import prepare_conveyor

        return controller.get_conveyor_preparation(
            "rev", domain_id, selected, request_, lambda: prepare_conveyor(snapshot_, request_), skeleton_id=host_backend.skeleton_identity_of("NATIVE")
        )

    prepared, domain = queue_export.run_queue_domain(
        0, "domain", snapshot, request, "0.25", preparation_provider=provider, backend="PYTHON", skeleton_backend="NATIVE"
    )
    assert calls == [("NATIVE", "PYTHON")] and domain.backend_record.skeleton_ran == "native"
    assert domain.backend_record.skeleton_requested == "NATIVE" and keys_of(controller) == [f"NATIVE:{BUILD_ID}"]
    # провайдер, у которого подготовка уже в кэше, скелета не считает
    queue_export.run_queue_domain(0, "domain", snapshot, request, "0.25", preparation_provider=provider, backend="PYTHON", skeleton_backend="NATIVE")
    assert len(calls) == 1
    # без заказа стадии — умолчание NATIVE: настоящий второй вызов и запись подготовки
    _prepared, plain = queue_export.run_queue_domain(0, "domain", snapshot, request, "0.25")
    assert len(calls) == 2 and plain.backend_record.skeleton_requested == "NATIVE" and plain.backend_record.skeleton_native_calls == 1

    # задача очереди воркера и задача с выгрузкой берут заказ у задачи, а не из умолчания
    for solve in (solve_task, solve_exported_task):
        before = len(calls)
        task = DomainTaskV1(1, 0, "domain", snapshot, request, "0.25", frozenset(), backend="PYTHON", skeleton_backend="NATIVE")
        reply = solve(task)
        assert reply.ok and len(calls) == before + 1, solve.__name__
        assert reply.queue_domain.backend_record.skeleton_ran == "native", solve.__name__
        assert reply.prepared is not None and prepared.outcome == reply.prepared.outcome
