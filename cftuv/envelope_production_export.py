"""Продуктовый путь Envelope: вход домена -> подготовка -> покрытие и GeometryBatchV1.

Хост отображает контракты, геометрию считает ядро. Единица работы — весь
(DecalRequestId, PatchDomainId), со всеми выбранными источниками вместе.
Холодный воркер возвращает подготовку и результат; родитель принимает их в
те же кэши сессии. После отказа метрики целого патча полосу строит родитель.

Снапшот и подготовка не зависят от исполнителя B1. Семантическая память B1
и ключ подготовки не расширяются его выбором; идентичность исполнения
результата и хранилища по содержимому несёт все заказанные стадии.
Журнал B1 начинается ДО экспорта и сохраняется при раннем отказе. Вложенная
подготовка заимствует журнал, материализация пишет отдельный. Записи родителя
и воркера складываются один раз при сборке результатов текущего прогона.
Кэшированный результат не приписывает текущему прогону прежние вызовы.

Законы UV/топологии передаются материализатору явно. Результат и подготовка
кэшируются раздельно; перенос между ревизиями проверяет identity/labeling.
Отказ входа, отказ ядра, исключение и пропажа ответа всегда именованы.

ПОДМНОЖЕСТВО ИНКРЕМЕНТАЛЬНОГО ТАКТА (WIDTH_REFRESH_INCREMENTAL_V1, только `lean=True`). `served` - `{номер патча: результат}` доменов, которых прогон НЕ считает: результаты прежнего прогона, сдвинутые аффинным сертификатом на ширину
прогона (`envelope_width_rebase`); `skipped` - номера патчей, которых прогон не считает и в итоге не называет (вызывающий складывает их сам: составной образец, `envelope_width_composite`). Такие домены не сканируются, не уходят воркерам и
не ложатся в кэши; остальные считаются как у lean-прогона, а подмножество всегда идёт в пул, когда он есть. Домен, чья идентичность в этой ревизии не найдена, считается как обычно (подмена молча не делается). Итог -
`refresh=REFRESH_INCREMENTAL`, число обслуженных и посчитанных - в профиле (`PRODUCTION_REFRESH_INCREMENTAL_*`).

Модуль держит сторону РОДИТЕЛЯ после сканирования: досчёт холодных доменов, пересылку воркерам, доведение результата до прогона и сам
`run_production`. Остальное вынесено ПЕРЕНОСОМ без изменения поведения (PRODUCTION_EXPORT_SPLIT_V1) и переэкспортируется отсюда: тип
результата и имена (`envelope_production_result`), воркер (`envelope_production_task`), сканирование (`envelope_production_scan`).
"""

from __future__ import annotations

import time
from dataclasses import dataclass, replace

from .envelope_chart_band import tightened_cap_of_facts
from .envelope_content_key import result_slot  # noqa: F401 - переэкспорт: имя прежнее
from .envelope_content_store import ContentRelabelFailed, RelabelV1, carried_to_run
from .envelope_debug_profile import EnvelopeDebugProfileBuilderV1
from .envelope_host_labels import record_host_tokens
from .envelope_kernel_backend import (
    DEFAULT_KERNEL_BACKEND,
    PreparationRefused,
    backend_timing_suffix,
    entered_backend,
    ledger_record,
    settle_stage_orders,
    with_preparation_record,
)
from .envelope_lazy_preparation import PRODUCTION_PREPARATION_UNPICKLE_CPU_US, PRODUCTION_PREPARATIONS_UNPICKLED, LazyPreparationV1, record_unpickled  # noqa: F401
from .envelope_production_report import (  # noqa: F401 - переэкспорт: имена прежние
    diagnostic_summary_lines,
    export_production_json,
    production_console_lines,
    production_status_text,
    receipt_console_lines,
    receipt_report_level,
    receipt_status_text,
    refused_outcome_counts,
)
from .envelope_production_result import (  # noqa: F401 - переэкспорт: имена прежние
    ColdProductionInputV1,
    HEAVY_COMPRESSION_LEVEL,
    MATERIALIZED,
    OUTCOME_CANON_UNSUPPORTED,
    OUTCOME_DOMAIN_RAISED,
    OUTCOME_PREPARATION_UNAVAILABLE,
    OUTCOME_PRODUCTION_CANCELLED,
    PICKLE_PROTOCOL,
    PLACEMENT_CACHED,
    PLACEMENT_FALLBACK,
    PLACEMENT_PARENT,
    PLACEMENT_UNAVAILABLE,
    PLACEMENT_WORKER,
    PRODUCTION_ALPHA_INTERVALS_AT_EVENT,
    PRODUCTION_ALPHA_INTERVALS_CERTIFIED,
    PRODUCTION_ALPHA_INTERVALS_NOT_CERTIFIED,
    PRODUCTION_BUILD_KIND,
    PRODUCTION_CLIP_MEMO_HITS,
    PRODUCTION_CLIP_MEMO_MISSES,
    PRODUCTION_COLD_FILL,
    PRODUCTION_CONTENT_KEYED,
    PRODUCTION_CONTENT_PREPARATION_REUSED,
    PRODUCTION_CONTENT_REGISTERED,
    PRODUCTION_CONTENT_RELABELED,
    PRODUCTION_CONTENT_RELABEL_FAILED,
    PRODUCTION_CONTENT_RESULT_REUSED,
    PRODUCTION_CONTENT_UNKEYED,
    PRODUCTION_DOMAINS,
    PRODUCTION_DOMAIN_GEOMETRY_BUILDS,
    PRODUCTION_INTERVAL_FALLBACK_PREFIX,
    PRODUCTION_INTERVAL_FAST_HITS,
    PRODUCTION_INTERVAL_VERIFY_MISMATCH,
    PRODUCTION_MATERIALIZED,
    PRODUCTION_PATCH_METRIC_BUILDS,
    PRODUCTION_POOL_BLOBS_SHIPPED,
    PRODUCTION_POOL_BLOB_BYTES_SHIPPED,
    PRODUCTION_POOL_BLOB_HITS,
    PRODUCTION_POOL_BLOB_MISSES,
    PRODUCTION_POOL_BYTES_RECEIVED,
    PRODUCTION_POOL_BYTES_SENT,
    PRODUCTION_POOL_UNPICKLE_CPU_US,
    PRODUCTION_POOL_UNPICKLE_WALL_US,
    PRODUCTION_PREPARATION_BUILDS,
    PRODUCTION_PREPARATION_REUSED,
    PRODUCTION_REFUSED,
    PRODUCTION_RESULT_CACHE_HIT,
    PRODUCTION_RESULT_CACHE_MISS,
    PRODUCTION_SCAN_RECORDS_REUSED,
    PRODUCTION_STAGE_INPUTS_MEMO_HIT,
    PRODUCTION_TOPOLOGY_LAW,
    PRODUCTION_TOPOLOGY_LAWS,
    PRODUCTION_UV_POLICY,
    ProductionCancelled,
    ProductionDomainResultV1,
    ProductionInputV1,
    deferred_result,
    _LAZY_FIELDS,
    _VIEW_INPUT_FIELDS,
    _placed,
    _raised,
    _refusal,
    LeanResultError,
    REFRESH_COMMIT,
    REFRESH_INCREMENTAL,
    REFRESH_LEAN,
    PLACEMENT_REBASED,
    PRODUCTION_REFRESH_INCREMENTAL_RECOMPUTED,
    PRODUCTION_REFRESH_INCREMENTAL_SERVED,
    PRODUCTION_REFRESH_LEAN_DOMAINS,
    PRODUCTION_REFRESH_LEAN_FULL_DOMAINS,
    lean_result,
    rebased_result,
)
from .envelope_production_scan import (  # noqa: F401 - переэкспорт: имена прежние
    _NO_BINDING,
    _DomainEntryV1,
    _RunInputsV1,
    _alpha_refusal,
    _band_key,
    _binding,
    _bound_entry,
    _check_cancel,
    _content_entry,
    _entry_from_record,
    _entry_with_inputs,
    _inputs_of,
    _note_relabel_failure,
    _result_key,
    _scan,
    _slot,
    lean_ready,
    released,
)
from .envelope_production_task import (  # noqa: F401 - переэкспорт: имена прежние
    _domain_of_preparation,
    _solve_cold_production_task,
    prepare_for_production,
    prepare_for_production_recorded,
    produce_domain,
    solve_cold_production_task,
    solve_production_task,
)
from .envelope_snap_retry import retry_snap_lottery
from .envelope_byte_check_background import deferred_domains, launch as launch_byte_check
from .envelope_snapshot_byte_check import finish_press, resolve_plan
from .envelope_scan_memo import remember_inputs
from .envelope_stretch_lines import developable_stretch_lines  # noqa: F401 - переэкспорт: имя прежнее
from .envelope_request_policy import ENVELOPE_UV_POLICIES
from .surface_ir import HOST_NEAR_PLANAR_LIFT_POLICY  # noqa: F401 - переэкспорт: тесты подменяют закон подъёма на этом модуле


def _host_refusal(entry: _DomainEntryV1, failure=None) -> ProductionDomainResultV1:
    failure = entry.failure if failure is None else failure
    outcome = getattr(failure.outcome, "value", failure.outcome)
    return with_preparation_record(_refusal(entry.patch_id, entry.domain_id, outcome, str(failure)), entry.export_record)


def _remember(run: _RunInputsV1, key, result) -> None:
    """Результат — в кэш сессии; исключение внутри домена не кэшируется (оно не ответ)."""

    if key is not None and result.outcome != OUTCOME_DOMAIN_RAISED:
        run.controller.remember_production_result(key, result)


def _produce_cold_in_parent(run: _RunInputsV1, entry, inputs, placement):
    """Холодный домен считает родитель: подготовка через кэш сессии, затем `produce_domain`.

    Сюда идёт домен, которого не взял воркер (нет пула, малая партия, упавшая задача), и
    ответ тот же: код тот же. Исключение подготовки называется, а не уходит в кнопку.
    """

    _check_cancel(run)
    snapshot, request = inputs
    started = time.perf_counter()
    with entered_backend(run.backend, run.skeleton_backend, run.embedding_backend) as ledger:
        def build():
            return prepare_for_production_recorded(
                snapshot, request, backend=run.backend, skeleton_backend=run.skeleton_backend,
                embedding_backend=run.embedding_backend, borrowed_ledger=ledger,
            )[0]

        try:
            prepared = run.controller.get_conveyor_preparation(
                run.revision, entry.domain_id, entry.selected, request, build,
                profile=run.profile, skeleton_id=run.skeleton_id, grid_scale_law=run.grid_scale_law,
            )
        except PreparationRefused as refused:
            return _placed(_refusal(
                entry.patch_id, entry.domain_id, refused.outcome, refused.detail,
                time.perf_counter() - started,
            ).with_changes(backend_record=ledger_record(ledger)), placement)
        except Exception as exc:  # noqa: BLE001 - исход (в том числе названный отказ канона) и запись вычисления сохраняются
            return _raised(exc, entry.patch_id, entry.domain_id, time.perf_counter() - started, placement).with_changes(backend_record=ledger_record(ledger))
    produced = produce_domain(
        entry.patch_id,
        entry.domain_id,
        prepared,
        run.alpha_text,
        uv_policy_id=run.uv_policy_id,
        topology_law=run.topology_law,
        backend=run.backend,
        skeleton_backend=run.skeleton_backend,
        embedding_backend=run.embedding_backend,
    )
    return _placed(with_preparation_record(produced, ledger_record(ledger)), placement)


def _adopt_cold(run: _RunInputsV1, entry: _DomainEntryV1, reply, placement):
    """Родительская сторона холодного домена: выгрузка, подготовка и результат — в кэши сессии.

    `(отказ входа | None, результат | None)`. `reply` — ответ воркера (`None`: задачи не было
    либо воркер её не вернул). Ответ воркера с выгрузкой проходит кэш метрики тем же путём,
    что и у отладочного вычислителя (`export_adopter` -> `snapshot_provider`): счётчики,
    счёт сборок и запомненный отказ те же. Домен, которого воркер не посчитал, считает
    родитель с названным `placement`.
    """

    from .envelope_export_input import replay_export_records
    from .envelope_request_export import EnvelopeHostAdapterError

    answered = reply is not None and (reply.ok or reply.refused)
    run.byte_check_ledger.extend(deferred_domains(reply))  # SNAPSHOT_BYTE_CHECK_FIRST_PRESS_IN_BACKGROUND_V1: сколько снапшотов воркер оставил на фоновый круг кодека
    if answered and entry.export is not None:
        run.hooks.export_adopter(entry.patch_id, reply)
    elif answered:
        replay_export_records(run.profile, reply)
    if reply is not None:
        entry.export_records.append(reply.backend_record)
    inputs = entry.inputs
    parent_log = None
    if inputs is None:
        with entered_backend(run.backend, run.skeleton_backend, run.embedding_backend) as ledger, record_host_tokens() as parent_log:
            try:
                inputs = _inputs_of(
                    run,
                    (entry.patch_id, entry.domain_id, entry.selected),
                    run.hooks.snapshot_provider,
                )
            except EnvelopeHostAdapterError as exc:
                entry.export_records.append(ledger_record(ledger))
                if reply is not None and reply.production is not None:
                    entry.export_records.append(reply.production.backend_record)
                return released(exc), None
        entry.export_records.append(ledger_record(ledger))
        entry.built_inputs.append(inputs)
    request = inputs[1]
    if reply is not None and reply.ok and reply.production is not None:
        prepared = reply.prepared
        if prepared is None and reply.prepared_blob is not None:
            # Подготовка пришла пиклом воркера: родитель держит пикл и факты, а живой объект разворачивает там, где он нужен (`live_preparation`).
            prepared = LazyPreparationV1(reply.prepared_blob, reply.prepared_key, reply.prepared_facts)
        # Домен, чей скелет нативное ядро отказало по имени, приходит отказом без подготовки: в кэш сессии кладут то, что есть.
        if prepared is not None:
            cached = run.controller.get_conveyor_item(
                run.revision,
                entry.domain_id,
                entry.selected,
                request,
                lambda: prepared,
                profile=run.profile,
                skeleton_id=run.skeleton_id,
                grid_scale_law=run.grid_scale_law,
            )
            if reply.prepared_blob is not None and cached is prepared:
                run.controller.preparation_blobs.adopt(prepared, reply.prepared_blob, reply.prepared_key)
        result = reply.production
    else:
        result = _produce_cold_in_parent(run, entry, inputs, placement)
        result = _labeled_by_parent(run, entry.patch_id, result, parent_log)
    _remember(
        run, _result_key(run, entry.domain_id, entry.selected, request), result
    )
    _register_content(run, entry, request, result)
    return None, result


def _labeled_by_parent(run: _RunInputsV1, patch_id, result, log):
    """Результат домена, снапшот которого строил родитель: запись его токенов (если запись полна)."""

    if log is None:
        return result
    labeling = log.labeling(run.revision, run.request_id, patch_id)
    return result.with_changes(labels=labeling) if labeling.has_domain_token() else result


def _register_content(run: _RunInputsV1, entry: _DomainEntryV1, request, result) -> None:
    """Подготовка и результат холодного домена — в хранилище по содержимому, если у них есть запись токенов.

    Домен, чья запись неполна (снапшот строили не под записью), и исключение внутри домена (оно не ответ)
    в хранилище не попадают: перенести такой результат на другую ревизию точно было бы нечем.
    """

    controller = run.controller
    if entry.content_key is None or result.labels is None or result.outcome == OUTCOME_DOMAIN_RAISED:
        return
    prepared = controller.peek_conveyor_item(
        run.revision, entry.domain_id, entry.selected, request, skeleton_id=run.skeleton_id, grid_scale_law=run.grid_scale_law
    )
    if prepared is None:
        return
    store = controller.content_store
    known = store.find(entry.content_key)
    cap = lambda item: tightened_cap_of_facts(controller.preparation_facts(item))  # noqa: E731 - факты, а не разворот подготовки
    if known is not None and cap(known.prepared) != cap(prepared):
        store.forget(entry.content_key)  # запись лежит на карте другой суженной досягаемости: новая подготовка её заменяет
    store.register_preparation(entry.content_key, prepared, result.labels)
    store.register_result(entry.content_key, _slot(run), result)
    run.registered.append(entry.patch_id)


def _finish_ready(run: _RunInputsV1, item: _DomainEntryV1, result):
    """`(отказ входа | None, результат)`: домен на готовой подготовке либо готовом результате доведён до прогона.

    Результат считался на подготовке (или лежал в хранилище), то есть в идентичностях своей записи: он ложится
    в хранилище как есть (слот занимает первый) и переносится на ревизию и запрос прогона (воркер уже мог
    это сделать; при тех же идентичностях перенос — тот же объект). Перенос, которого не вышло, — названная
    причина и домен, посчитанный заново с нуля: устаревший ответ не отдаётся.
    """

    controller = run.controller
    if result.lean:
        # WIDTH_REFRESH_LEAN_V1: lean-результат НЕ ложится ни в кэш результатов сессии, ни в хранилище по содержимому (оба отказывают ему `LeanResultError`):
        # у него нет батча, и ответом, который отдают другому нажатию, он быть не вправе. Зафиксированный ответ даёт прогон полным путём (COMMIT).
        return None, result
    if item.carried is not None:
        result = result.with_changes(backend_record=None)
    if result.outcome == OUTCOME_DOMAIN_RAISED:
        return None, result
    key, labeling = item.content_key, item.labeling
    if key is None and item.prepared is not None:
        held = controller.content_store.key_of(item.prepared)
        if held is not None:
            key, labeling = held[0], held[1].labeling
    if key is None:
        _remember(run, item.result_key, result)
        return None, result
    based = result if result.labels is not None else result.with_changes(labels=labeling)
    controller.content_store.register_result(key, _slot(run), based)
    if item.carried is None and not item.reuse:
        # Подготовка этой ревизии (кэш ревизии): идентичности те же, что у прогона, переносить нечего.
        _remember(run, item.result_key, based)
        return None, based
    try:
        moved = carried_to_run(based, RelabelV1(run.revision, run.request_id, item.patch_id))
        if moved.domain_id != item.domain_id:
            raise ContentRelabelFailed("the domain id moved to another value")
    except ContentRelabelFailed as exc:
        _note_relabel_failure(run, item.patch_id, exc)
        controller.content_store.forget(key)
        anew = replace(item, prepared=None, carried=None, labeling=None, reuse="", result_key=None)
        return _adopt_cold(run, anew, None, PLACEMENT_PARENT)
    foreign = item.carried.labels if item.carried is not None else item.labeling
    if foreign is not None and (
        foreign.revision != run.revision or foreign.patch_id != item.patch_id
    ):
        run.relabeled.append(item.patch_id)
    if item.carried is not None:
        moved = moved.with_changes(seconds=0.0, placement=PLACEMENT_CACHED)
    _remember(run, item.result_key, moved)
    return None, moved


def _produced_by_parent(run: _RunInputsV1, item: _DomainEntryV1) -> ProductionDomainResultV1:
    """Домен на готовой подготовке, который считает родитель: ручку подготовки он разворачивает здесь, и пикл, который не читается, - названный отказ домена."""

    started = time.perf_counter()
    try:
        prepared = run.controller.live_preparation(item.prepared)
    except Exception as exc:  # noqa: BLE001 - исход и причина сохраняются, как у остальных исключений домена
        return _raised(exc, item.patch_id, item.domain_id, time.perf_counter() - started)
    return produce_domain(
        item.patch_id,
        item.labeling.domain_id() if item.labeling is not None else item.domain_id,
        prepared,
        run.alpha_text,
        uv_policy_id=run.uv_policy_id,
        topology_law=run.topology_law,
        lean=lean_ready(run, item),
        backend=run.backend,
        skeleton_backend=run.skeleton_backend,
        embedding_backend=run.embedding_backend,
    )


def _complete_ready(run: _RunInputsV1, ready, done, refused, placement) -> None:
    """Домены на готовой подготовке либо результате: посчитанное воркером берётся, остальное делает родитель."""

    for item in ready:
        if item.domain_id not in done:
            _check_cancel(run)
            done[item.domain_id] = _placed(
                item.carried if item.carried is not None else _produced_by_parent(run, item),
                placement.get(item.domain_id, PLACEMENT_PARENT),
            )
        input_refusal, finished = _finish_ready(run, item, done[item.domain_id])
        if input_refusal is not None:
            refused[item.domain_id] = input_refusal
            del done[item.domain_id]
        else:
            done[item.domain_id] = finished


def _production_input(run: _RunInputsV1, entry: _DomainEntryV1, blob: bytes) -> ProductionInputV1:
    """Вход задачи на готовой подготовке либо готовом результате: перенос на прогон делает воркер."""

    relabel = None
    if entry.carried is not None:
        relabel = RelabelV1(run.revision, run.request_id, entry.patch_id)
    elif entry.labeling is not None:
        relabel = RelabelV1(run.revision, run.request_id, entry.patch_id, entry.labeling)
    # Память подготовок воркера держит подготовки, а не результаты на перенос: у `carried` пикл едет всегда.
    key = "" if entry.carried is not None else run.controller.preparation_blobs.key_of(entry.prepared)
    return ProductionInputV1(
        blob, run.uv_policy_id, run.topology_law, relabel, entry.carried is not None, key, lean_ready(run, entry)
    )


def _worker_tasks(run: _RunInputsV1, ready, shipped, cold):
    """Задачи пула: домены на готовой подготовке (пикл) и холодные домены (вход задачи)."""

    from .envelope_domain_pool import DomainTaskV1

    entry_of = {item.domain_id: item for item in ready}
    tasks = [
        DomainTaskV1(
            index,
            patch_id,
            domain_id,
            None,
            None,
            run.alpha_text,
            entry_of[domain_id].selected,
            production=_production_input(run, entry_of[domain_id], blob),
            affinity=domain_id,
            backend=run.backend,
            skeleton_backend=run.skeleton_backend,
            embedding_backend=run.embedding_backend,
        )
        for index, ((patch_id, domain_id, _payload), blob) in enumerate(shipped)
    ]
    laws = ColdProductionInputV1(run.uv_policy_id, run.topology_law, run.topology_export.snapshot_byte_check)
    for entry in cold:
        snapshot, request = entry.inputs or (None, None)
        tasks.append(
            DomainTaskV1(
                len(tasks),
                entry.patch_id,
                entry.domain_id,
                snapshot,
                request,
                run.alpha_text,
                entry.selected,
                export=entry.export,
                cold=laws,
                affinity=entry.domain_id,
                backend=run.backend,
                skeleton_backend=run.skeleton_backend,
                embedding_backend=run.embedding_backend,
            )
        )
    return tasks


def _dispatch(run: _RunInputsV1, ready, cold, domain_pool):
    """`({domain_id: результат}, {domain_id: отказ входа})`: воркеры на то, что окупает пересылку.

    `ready` — домены на подготовке из кэша (воркеру уходит её пикл), `cold` — домены без
    подготовки (воркеру уходит вход, он готовит и материализует сам). Названные исходы те
    же, что у покрытия отладки: пул, который не стартовал (`ENVELOPE_DOMAIN_POOL_UNAVAILABLE`),
    и задача, которая упала или чей воркер умер (`ENVELOPE_DOMAIN_POOL_TASK_FALLBACK`), —
    счётчик профиля, строка консоли и пометка `placement`; домен при этом считает родитель
    тем же кодом. Партия готовых подготовок, которая стоит меньше пересылки, остаётся в
    родителе без исхода (это размещение, а не отказ) — если только холодные задачи не
    поднимают пул и так: тогда готовые идут с ними в одной партии.
    """

    from .envelope_queue_pool import (
        _note_task_fallback,
        _record_pool_counters,
        _run_tasks,
        _ship_preparations,
        _task_error,
        _worth_the_pool,
    )

    controller = run.controller
    triples = [
        (item.patch_id, item.domain_id, item.prepared if item.carried is None else item.carried)
        for item in ready
    ]
    shipped, shipping_failures = (
        _ship_preparations(triples, controller.preparation_blobs, lambda item: item[2])
        if domain_pool is not None and triples
        else ([], {})
    )
    sent_cold = list(cold) if domain_pool is not None else []
    # WIDTH_REFRESH_INCREMENTAL_V1: подмножество (домены вне сертификата) всегда идёт в пул, когда он есть: мало байт подготовок значит мало доменов, а не то, что пересылка не окупится (замер: пул быстрее и на
    # четырёх доменах), и родитель считал бы их последовательно в потоке рядом с главным.
    if not (sent_cold or run.served or run.skipped or _worth_the_pool(shipped)):
        shipped = []
    tasks = _worker_tasks(run, ready, shipped, sent_cold) if (shipped or sent_cold) else []
    pooled, failure = (
        _run_tasks(domain_pool, tasks, run.profile, cancel=run.cancel) if tasks else (None, "")
    )
    _check_cancel(run)  # остановленный пул отдал только сделанное: докончить за него нельзя
    entry_of = {item.domain_id: item for item in (*ready, *cold)}
    done: dict[str, ProductionDomainResultV1] = {}
    refused: dict[str, object] = {}
    placement: dict[str, str] = {}
    handled: set[str] = set()
    dispatched = fallbacks = 0
    for task in tasks:
        domain_id = task.domain_id
        reply = None if pooled is None or failure else pooled.results.get(task.task_id)
        landed = reply is not None and reply.ok and reply.production is not None
        sent = True
        if failure:
            placement[domain_id] = PLACEMENT_UNAVAILABLE
        elif landed:
            placement[domain_id] = PLACEMENT_WORKER
        elif task.cold is not None and reply is not None and reply.refused:
            # Выгрузка отказала в воркере: домен воркеру «не уходил» (как у отладки).
            sent = False
        else:
            fallbacks += 1
            _note_task_fallback(domain_id, _task_error(pooled, task))
            placement[domain_id] = PLACEMENT_FALLBACK
        dispatched += int(sent and not failure)
        if task.cold is None:
            if landed:
                done[domain_id] = reply.production
            continue
        handled.add(domain_id)
        exc, result = _adopt_cold(
            run, entry_of[domain_id], reply, placement.get(domain_id, PLACEMENT_PARENT)
        )
        if exc is not None:
            refused[domain_id] = exc
        else:
            done[domain_id] = result
    for domain_id, reason in shipping_failures.items():
        fallbacks += 1
        _note_task_fallback(domain_id, reason)
        placement[domain_id] = PLACEMENT_FALLBACK
    _complete_ready(run, ready, done, refused, placement)
    for item in cold:
        if item.domain_id in handled:
            continue
        exc, result = _adopt_cold(run, item, None, PLACEMENT_PARENT)
        if exc is not None:
            refused[item.domain_id] = exc
        else:
            done[item.domain_id] = result
    if domain_pool is not None:
        _record_pool_counters(run.profile, pooled, failure, dispatched, fallbacks, dispatched)
        _record_transfer_counters(run.profile, getattr(pooled, "stats", None))
    _record_cold_inputs(run, cold)
    return done, refused


def _record_cold_inputs(run: _RunInputsV1, cold) -> None:
    """Входы холодных доменов, построенные родителем после ответов воркеров, - в записи сборки прогона: первый шаг ширины после кнопки берёт их, а не собирает снова."""

    records = run.scan_records.get("records")
    for entry in cold if records is not None else ():
        for inputs in entry.built_inputs:
            prep_key = _result_key(run, entry.domain_id, entry.selected, inputs[1])[0]
            remember_inputs(records, entry.patch_id, entry.selected, inputs, prep_key, _binding(run, entry.patch_id, entry.domain_id, entry.selected))


def _record_transfer_counters(profile, stats) -> None:
    """Цена пересылок прогона (`PoolStatsV1`) в профиль; подставной пул без статистики ничего не пишет."""

    if stats is None:
        return
    for name, value in (
        (PRODUCTION_POOL_BYTES_SENT, stats.bytes_sent),
        (PRODUCTION_POOL_BYTES_RECEIVED, stats.bytes_received),
        (PRODUCTION_POOL_BLOB_HITS, stats.blob_hits),
        (PRODUCTION_POOL_BLOBS_SHIPPED, stats.blobs_shipped),
        (PRODUCTION_POOL_BLOB_BYTES_SHIPPED, stats.blob_bytes_shipped),
        (PRODUCTION_POOL_BLOB_MISSES, stats.blob_misses),
        (PRODUCTION_POOL_UNPICKLE_CPU_US, round(stats.unpickle_cpu_seconds * 1e6)),
        (PRODUCTION_POOL_UNPICKLE_WALL_US, round(stats.unpickle_wall_seconds * 1e6)),
    ):
        profile.add_counter(name, int(value))


@dataclass(frozen=True, slots=True)
class ProductionRunV1:
    """Итог продуктового прогона: результаты по доменам и числа прогона."""

    results: tuple[ProductionDomainResultV1, ...]
    revision: str
    cold: bool
    profile: object
    wall_seconds: float
    #: `((патч, (рёбра хоста, чью полосу строит домен патча), ...), ...)`: ровно то, что превью ширины рисует.
    selected_by_patch: tuple = ()
    kernel_backend: str = DEFAULT_KERNEL_BACKEND  # главный переключатель ядра, заказанный прогоном
    skeleton_backend: str | None = None  # порядки стадий прогона: `None` — как `kernel_backend`, `__post_init__` приводит их к именам
    embedding_backend: str | None = None
    #: Класс обновления (`REFRESH_COMMIT` | `REFRESH_LEAN` = WIDTH_REFRESH_LEAN_V1): lean-прогон заказывает путь без работы, нужной только фиксации.
    refresh: str = REFRESH_COMMIT

    def __post_init__(self) -> None:
        settle_stage_orders(self, "kernel_backend")

    @property
    def materialized(self) -> tuple[ProductionDomainResultV1, ...]:
        return tuple(item for item in self.results if item.is_materialized)

    @property
    def refused(self) -> tuple[ProductionDomainResultV1, ...]:
        return tuple(item for item in self.results if not item.is_materialized)

    def counter(self, name: str):
        for item in self.profile.counters:
            if item.name == name and item.patch_domain_id is None:
                return item.value
        return None


_FROM_SETTINGS = object()


def _domain_results(entries, done, refused):
    """Результаты в порядке доменов: отказ входа, результат из кэша, посчитанное либо названная пропажа."""

    results = []
    for entry in entries:
        if entry.failure is not None:
            results.append(_host_refusal(entry))
        elif entry.domain_id in refused:
            results.append(_host_refusal(entry, refused[entry.domain_id]))
        elif entry.rebased:
            results.append(entry.cached)
        elif entry.cached is not None:
            results.append(entry.cached.with_changes(seconds=0.0, placement=PLACEMENT_CACHED, clip_memo="", backend_record=entry.export_record))
        elif entry.domain_id in done:
            results.append(with_preparation_record(done[entry.domain_id], entry.export_record))
        else:
            results.append(
                _refusal(
                    entry.patch_id,
                    entry.domain_id,
                    OUTCOME_PREPARATION_UNAVAILABLE,
                    "the domain came back from the workers and the parent without a result",
                )
            )
    return results


def _record_content_counters(profile, run: _RunInputsV1, entries) -> None:
    profile.set_counter(
        PRODUCTION_CONTENT_KEYED,
        sum(1 for item in entries if item.content_key is not None),
    )
    profile.set_counter(PRODUCTION_CONTENT_REGISTERED, len(run.registered))
    profile.set_counter(
        PRODUCTION_CONTENT_RESULT_REUSED,
        sum(1 for item in entries if item.reuse == "result"),
    )
    profile.set_counter(
        PRODUCTION_CONTENT_PREPARATION_REUSED,
        sum(1 for item in entries if item.reuse == "preparation"),
    )
    profile.set_counter(PRODUCTION_CONTENT_RELABELED, len(run.relabeled))
    profile.set_counter(PRODUCTION_CONTENT_RELABEL_FAILED, len(run.relabel_failures))
    profile.set_counter(
        PRODUCTION_CONTENT_UNKEYED,
        sum(1 for item in entries if item.export is not None and item.content_key is None),
    )
    finish_press(profile, run.topology_export.snapshot_byte_check)  # SNAPSHOT_BYTE_CHECK_SAMPLED_V1: решения по доменам - в итоги нажатия


def _record_run_counters(profile, controller, builds_before, entries, results, cold):
    builds_after = controller.build_counts
    for name, key in (
        (PRODUCTION_PREPARATION_BUILDS, "CONVEYOR_PREPARATION"),
        (PRODUCTION_PATCH_METRIC_BUILDS, "PATCH_METRIC"),
        (PRODUCTION_DOMAIN_GEOMETRY_BUILDS, "DOMAIN_GEOMETRY"),
    ):
        profile.set_counter(name, builds_after[key] - builds_before[key])
    profile.set_counter(
        PRODUCTION_PREPARATION_REUSED,
        sum(1 for item in entries if item.prepared is not None),
    )
    profile.set_counter(PRODUCTION_COLD_FILL, int(cold))
    profile.set_counter(
        PRODUCTION_RESULT_CACHE_HIT,
        sum(
            1
            for item in entries
            if item.failure is None and (item.cached is not None or item.carried is not None) and not item.rebased
        ),
    )
    profile.set_counter(
        PRODUCTION_RESULT_CACHE_MISS, sum(1 for item in entries if item.computes)
    )
    computed = [result for entry, result in zip(entries, results) if entry.computes]
    for name, label in ((PRODUCTION_CLIP_MEMO_HITS, "HIT"), (PRODUCTION_CLIP_MEMO_MISSES, "MISS")):
        profile.set_counter(name, sum(1 for item in computed if item.clip_memo == label))
    _interval_step_counters(profile, computed)
    for name, status in (
        (PRODUCTION_ALPHA_INTERVALS_CERTIFIED, "CERTIFIED"),
        (PRODUCTION_ALPHA_INTERVALS_AT_EVENT, "AT_EVENT"),
        (PRODUCTION_ALPHA_INTERVALS_NOT_CERTIFIED, "NOT_CERTIFIED"),
    ):
        profile.set_counter(
            name, sum(1 for item in results if item.alpha_interval is not None and item.alpha_interval.status == status)
        )
    profile.set_counter(PRODUCTION_DOMAINS, len(results))
    profile.set_counter(
        PRODUCTION_MATERIALIZED, sum(1 for item in results if item.is_materialized)
    )
    profile.set_counter(
        PRODUCTION_REFUSED, sum(1 for item in results if not item.is_materialized)
    )


def _interval_step_counters(profile, computed) -> None:
    """Счётчики шага ширины по посчитанным доменам: попадания, полные счёты по причинам, расхождения сверки (нулевые причины не пишутся)."""

    paths = [item.step_path for item in computed if item.step_path]
    profile.set_counter(PRODUCTION_INTERVAL_FAST_HITS, sum(1 for path in paths if path == "FAST_HIT"))
    reasons: dict = {}
    for path in paths:
        if path.startswith("FALLBACK:"):
            reasons[path[len("FALLBACK:") :]] = reasons.get(path[len("FALLBACK:") :], 0) + 1
    for reason in sorted(reasons):
        profile.set_counter(PRODUCTION_INTERVAL_FALLBACK_PREFIX + reason, reasons[reason])
    profile.set_counter(PRODUCTION_INTERVAL_VERIFY_MISMATCH, sum(1 for path in paths if path.startswith("VERIFY_MISMATCH")))


def _dispatch_work(run: _RunInputsV1, work, pool):
    """`_dispatch` по доменам, которым нужна работа: готовые подготовки (пикл воркеру) и холодные домены (вход задачи)."""

    ready = [item for item in work if item.prepared is not None or item.carried is not None]
    return _dispatch(run, ready, [item for item in work if item.prepared is None and item.carried is None], pool)


def run_production(
    controller,
    analysis_bundle,
    selected_physical_edge_ids,
    alpha: float,
    *,
    source_object_key,
    source_data_key,
    density,
    workers: int = 0,
    uv_policy_id: str = PRODUCTION_UV_POLICY,
    topology_law: str = PRODUCTION_TOPOLOGY_LAW,
    domain_pool=_FROM_SETTINGS,
    developable_stretch_budget=None,
    chart_reach_cap=None,
    silhouette_uv_slide=None,
    cancel=None,
    quiesce: bool = True,
    kernel_backend: str = DEFAULT_KERNEL_BACKEND,
    skeleton_backend: str | None = None,
    embedding_backend: str | None = None,
    lean: bool = False,
    served: dict | None = None,
    skipped=None,
) -> ProductionRunV1:
    """Один продуктовый прогон по доменам выделения: сессия, пул, названные исходы.

    Домен с результатом в кэше сессии не считается вовсе; домен с подготовкой в кэше
    считает только покрытие и материализацию; холодный домен — подготовку и
    материализацию одной задачей. Счётчики сборок контроллера в профиле
    (`PRODUCTION_*_BUILDS`) и счётчики кэша результатов — доказательство повторного
    использования, а не секунды.

    `cancel` (`threading.Event`) — заказ остановки для прогона в фоновом потоке (живая ширина):
    после его установки пул не берёт новые задачи, родитель не начинает новый домен, а прогон бросает
    `ProductionCancelled`. `quiesce=False` у такого прогона обязателен: он сам идёт в потоке планировщика
    превью и не вправе останавливать тот же планировщик из потока счёта; кнопка оставляет `True`.

    `kernel_backend` — ГЛАВНЫЙ переключатель всех стадий ядра; `skeleton_backend`/`embedding_backend` (только API) — порядок одной стадии, `None` — как главный (`stage_orders`).

    `lean=True` — WIDTH_REFRESH_LEAN_V1: обновление БЕЗ работы, нужной только фиксации (воркер не пакует батч, результат не ложится ни в какой кэш). Геометрия, исходы доменов
    и счётчики материализатора те же побитово; зафиксированный ответ даёт только прогон с `lean=False`. Домены без готовой подготовки этой ревизии (холодные, с переносом
    из хранилища) идут полным путём в том же прогоне, и профиль называет их число (`PRODUCTION_REFRESH_LEAN_FULL_DOMAINS`).

    `served` / `skipped` (WIDTH_REFRESH_INCREMENTAL_V1, только с `lean=True`): домены, которых прогон не считает; описание - в шапке модуля `envelope_width_incremental`.
    """

    from .envelope_chart_band import policy_alpha
    from .envelope_domain_pool import get_domain_pool
    from .envelope_topology_export import STAGE_INPUTS_HIT, stage_production_inputs
    from .envelope_worker_python import read_worker_python

    if uv_policy_id not in ENVELOPE_UV_POLICIES:
        raise ValueError(f"unknown UV policy {uv_policy_id!r}")
    if topology_law not in PRODUCTION_TOPOLOGY_LAWS:
        raise ValueError(f"unknown decal topology law {topology_law!r}")
    if (served or skipped) and not lean:
        raise ValueError("served domains (WIDTH_REFRESH_INCREMENTAL_V1) belong to a lean run: a committed answer is computed whole")
    # Поток превью alpha считает на тех же подготовках, что и эта кнопка: сперва стоп и ожидание.
    if quiesce:
        controller.quiesce_preview("Build Decal Mesh")
    started = time.perf_counter()
    profile = EnvelopeDebugProfileBuilderV1(
        getattr(analysis_bundle.source_revision, "source_name", "source"),
        PRODUCTION_BUILD_KIND,
    )
    builds_before, lazy_before = controller.build_counts, controller.lazy_unpickled
    selected = frozenset(int(item) for item in selected_physical_edge_ids)
    pool = (
        get_domain_pool(workers, read_worker_python())
        if domain_pool is _FROM_SETTINGS
        else domain_pool
    )
    topology_export = controller.get_topology_export(
        analysis_bundle, source_object_key, source_data_key, profile=profile
    ).with_developable_stretch_budget(developable_stretch_budget).with_silhouette_uv_slide(
        silhouette_uv_slide
    ).with_chart_band(chart_reach_cap, selected, policy_alpha(alpha)).with_snapshot_byte_check(
        resolve_plan(kernel_backend, skeleton_backend, embedding_backend, defer=hasattr(pool, "run_idle"))
    )
    stage_memo = controller.stage_inputs_memo
    revision, patch_ids, request_id, selected_by_domain = stage_production_inputs(
        analysis_bundle, selected, profile=profile, topology_export=topology_export, memo=stage_memo
    )
    profile.set_counter(PRODUCTION_STAGE_INPUTS_MEMO_HIT, int(stage_memo.last == STAGE_INPUTS_HIT))
    refused_by_byte_check = controller.byte_check_refused_patches(revision)  # домены, помеченные фоном: подмножество инкрементального такта их не обслуживает и не пропускает, `_scan` отказывает их
    run = _RunInputsV1(
        controller,
        analysis_bundle,
        topology_export,
        revision,
        tuple(patch_ids),
        selected_by_domain,
        alpha,
        str(float(alpha)),
        request_id,
        density,
        uv_policy_id,
        topology_law,
        controller.worker_export_hooks(topology_export, profile),
        profile,
        cancel=cancel,
        backend=kernel_backend,
        skeleton_backend=skeleton_backend,
        embedding_backend=embedding_backend,
        lean=lean,
        served={patch: result for patch, result in dict(served or {}).items() if patch not in refused_by_byte_check},
        skipped=frozenset(int(item) for item in (skipped or ())) - refused_by_byte_check,
    )
    entries = _scan(run)
    _check_cancel(run)
    cold = any(item.is_cold for item in entries)
    work = [item for item in entries if item.needs_work]
    with profile.measure("PRODUCTION_DOMAINS_WALL"):
        done, refused = _dispatch_work(run, work, pool)
    results = retry_snap_lottery(run, entries, _domain_results(entries, done, refused), pool, scan=_scan, dispatch=_dispatch, collect=_domain_results)
    produced = _production_run(run, entries, results, cold, started, builds_before, lazy_before)
    launch_byte_check(run, pool, entries)  # SNAPSHOT_BYTE_CHECK_FIRST_PRESS_IN_BACKGROUND_V1: отложенный круг кодека идёт фоном ПОСЛЕ нажатия
    return produced


def _production_run(run: _RunInputsV1, entries, results, cold, started, builds_before, lazy_before) -> ProductionRunV1:
    """Счётчики прогона в профиль и итог `ProductionRunV1`."""

    profile = run.profile
    domains = {entry.patch_id: entry.domain_id for entry in entries}
    _record_run_counters(profile, run.controller, builds_before, entries, results, cold)
    record_unpickled(profile, lazy_before, run.controller.lazy_unpickled)
    _record_content_counters(profile, run, entries)
    rebased = sum(1 for entry in entries if entry.rebased)
    omitted = sum(1 for patch_id in run.patch_ids if patch_id in run.skipped)
    if run.lean:
        _record_lean_counters(profile, results)
    if rebased or omitted:
        profile.set_counter(PRODUCTION_REFRESH_INCREMENTAL_SERVED, rebased + omitted)
        profile.set_counter(PRODUCTION_REFRESH_INCREMENTAL_RECOMPUTED, len(entries) - rebased)
    return ProductionRunV1(
        results=tuple(results),
        revision=run.revision,
        cold=cold,
        profile=profile.snapshot(),
        wall_seconds=time.perf_counter() - started,
        # Идентичность домена патча `_scan` уже вывел (`_typed_value("patch-domain", ревизия, патч)`) и держит в записи каждого домена: второй раз её не выводят.
        selected_by_patch=tuple(
            (int(patch_id), tuple(sorted(run.selected_by_domain[domains[patch_id]])))
            for patch_id in run.patch_ids
            if patch_id in domains
        ),
        kernel_backend=run.backend,
        skeleton_backend=run.skeleton_backend,
        embedding_backend=run.embedding_backend,
        refresh=REFRESH_INCREMENTAL if rebased or omitted else (REFRESH_LEAN if run.lean else REFRESH_COMMIT),
    )


def _record_lean_counters(profile, results) -> None:
    """WIDTH_REFRESH_LEAN_V1: сколько доменов посчитано без фиксирующей работы и сколько пошло полным путём (кэш, холодный домен, перенос); пишется только у lean-прогона."""

    lean = sum(1 for item in results if item.lean and item.placement != PLACEMENT_REBASED)
    profile.set_counter(PRODUCTION_REFRESH_LEAN_DOMAINS, lean)
    profile.set_counter(PRODUCTION_REFRESH_LEAN_FULL_DOMAINS, sum(1 for item in results if item.is_materialized and item.placement != PLACEMENT_REBASED) - lean)


# --------------------------------------------------------------------------
# Строки владельцу и свидетельства
# --------------------------------------------------------------------------


def production_timing_text(run: ProductionRunV1) -> str:
    """Секунды прогона и работа пула в одну строку панели."""

    from .envelope_queue_export import _pool_timing_suffix

    kind = "cold" if run.cold else "warm"
    reused = run.counter(PRODUCTION_PREPARATION_REUSED)
    builds = run.counter(PRODUCTION_PREPARATION_BUILDS)
    cached = run.counter(PRODUCTION_RESULT_CACHE_HIT)
    computed = run.counter(PRODUCTION_RESULT_CACHE_MISS)
    content = (run.counter(PRODUCTION_CONTENT_RESULT_REUSED) or 0) + (
        run.counter(PRODUCTION_CONTENT_PREPARATION_REUSED) or 0
    )
    return (
        f"Decal {kind} {run.wall_seconds:.2f} s | preparations reused {reused}, "
        f"built {builds} | results cached {cached}, computed {computed}"
        f"{_content_timing_suffix(run, content)}"
        f"{_clip_memo_timing_suffix(run)}"
        f"{_lean_timing_suffix(run)}"
        f"{backend_timing_suffix(run.results, run.kernel_backend, run.skeleton_backend, run.embedding_backend)}"
        f"{_pool_timing_suffix(run.profile)}"
    )


def _lean_timing_suffix(run: ProductionRunV1) -> str:
    """` | WIDTH_REFRESH_LEAN_V1 L lean, F full`: класс обновления назван в строке панели (у COMMIT строки нет, она прежняя)."""

    if run.refresh == REFRESH_INCREMENTAL:
        return f" | {REFRESH_INCREMENTAL} {run.counter(PRODUCTION_REFRESH_INCREMENTAL_SERVED)} served by the certificate, {run.counter(PRODUCTION_REFRESH_INCREMENTAL_RECOMPUTED)} recomputed"
    if run.refresh != REFRESH_LEAN:
        return ""
    return f" | {REFRESH_LEAN} {run.counter(PRODUCTION_REFRESH_LEAN_DOMAINS)} lean, {run.counter(PRODUCTION_REFRESH_LEAN_FULL_DOMAINS)} full"


def _clip_memo_timing_suffix(run: ProductionRunV1) -> str:
    """` | clip memo H hits, M misses`, когда у посчитанных доменов была резка с памятью стадии."""

    hits, misses = run.counter(PRODUCTION_CLIP_MEMO_HITS), run.counter(PRODUCTION_CLIP_MEMO_MISSES)
    return f" | clip memo {hits} hits, {misses} misses" if hits or misses else ""


def _content_timing_suffix(run: ProductionRunV1, content: int) -> str:
    """` | content reused R results, P preparations`, когда хранилище по содержимому что-то отдало."""

    if not content:
        return ""
    return (
        f" | content reused {run.counter(PRODUCTION_CONTENT_RESULT_REUSED)} results, "
        f"{run.counter(PRODUCTION_CONTENT_PREPARATION_REUSED)} preparations"
    )


__all__ = (
    "ColdProductionInputV1",
    "MATERIALIZED",
    "OUTCOME_CANON_UNSUPPORTED",
    "OUTCOME_DOMAIN_RAISED",
    "OUTCOME_PREPARATION_UNAVAILABLE",
    "PLACEMENT_CACHED",
    "PRODUCTION_ALPHA_INTERVALS_AT_EVENT",
    "PRODUCTION_ALPHA_INTERVALS_CERTIFIED",
    "PRODUCTION_ALPHA_INTERVALS_NOT_CERTIFIED",
    "PRODUCTION_CLIP_MEMO_HITS",
    "PRODUCTION_CLIP_MEMO_MISSES",
    "PRODUCTION_COLD_FILL",
    "PRODUCTION_DOMAINS",
    "PRODUCTION_DOMAIN_GEOMETRY_BUILDS",
    "PRODUCTION_INTERVAL_FALLBACK_PREFIX",
    "PRODUCTION_INTERVAL_FAST_HITS",
    "PRODUCTION_INTERVAL_VERIFY_MISMATCH",
    "PRODUCTION_MATERIALIZED",
    "PRODUCTION_PATCH_METRIC_BUILDS",
    "PRODUCTION_PREPARATION_BUILDS",
    "PRODUCTION_POOL_BLOBS_SHIPPED",
    "PRODUCTION_POOL_BLOB_BYTES_SHIPPED",
    "PRODUCTION_POOL_BLOB_HITS",
    "PRODUCTION_POOL_BLOB_MISSES",
    "PRODUCTION_POOL_BYTES_RECEIVED",
    "PRODUCTION_POOL_BYTES_SENT",
    "PRODUCTION_POOL_UNPICKLE_CPU_US",
    "PRODUCTION_POOL_UNPICKLE_WALL_US",
    "PRODUCTION_PREPARATIONS_UNPICKLED",
    "PRODUCTION_REFRESH_INCREMENTAL_RECOMPUTED",
    "PRODUCTION_REFRESH_INCREMENTAL_SERVED",
    "PRODUCTION_PREPARATION_REUSED",
    "PRODUCTION_PREPARATION_UNPICKLE_CPU_US",
    "PRODUCTION_REFUSED",
    "PRODUCTION_RESULT_CACHE_HIT",
    "PRODUCTION_RESULT_CACHE_MISS",
    "PRODUCTION_SCAN_RECORDS_REUSED",
    "PRODUCTION_STAGE_INPUTS_MEMO_HIT",
    "PRODUCTION_TOPOLOGY_LAW",
    "PRODUCTION_TOPOLOGY_LAWS",
    "PRODUCTION_UV_POLICY",
    "ProductionDomainResultV1",
    "ProductionInputV1",
    "ProductionRunV1",
    "deferred_result",
    "developable_stretch_lines",
    "diagnostic_summary_lines",
    "export_production_json",
    "prepare_for_production",
    "prepare_for_production_recorded",
    "produce_domain",
    "production_console_lines",
    "receipt_console_lines",
    "receipt_report_level",
    "receipt_status_text",
    "production_status_text",
    "production_timing_text",
    "refused_outcome_counts",
    "run_production",
    "solve_cold_production_task",
    "solve_production_task",
)
