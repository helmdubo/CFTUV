"""Склейка пула доменов с движком QUEUE: входы, диспетчеризация, приём ответов.

Сам пул (`envelope_domain_pool`) ничего не знает об очереди: он пересылает
`DomainTaskV1` и возвращает ответ. Здесь — то, что знает очередь:

- фаза A: входы `(snapshot, request)` доменов, чья метрика уже в кэше сессии,
  берутся в родителе (кэш-попадание стоит доли миллисекунды); у остальных
  выгрузку считает ВОРКЕР (`envelope_export_input`: Blender-объекты воркеру не
  пересылаются, уходит лёгкий вход), и родитель принимает её ответом;
- фаза B: домены, подготовки которых нет в кэше сессии, уходят воркерам разом,
  а счётчики пула и стадия `QUEUE_POOL_WALL` пишутся в профиль;
- фаза C (цикл `evaluate_envelope_queue_staged`): ответ воркера принимается как
  результат домена, а домен, которого воркер не дал, досчитывается прежним
  последовательным путём.

Отказ именован на обоих уровнях: пул, который не стартовал, — счётчик
`ENVELOPE_DOMAIN_POOL_UNAVAILABLE`, строка консоли и диагностика на первом
домене; задача, которая упала либо чей воркер умер, — счётчик
`ENVELOPE_DOMAIN_POOL_TASK_FALLBACK`, строка консоли и диагностика на ЭТОМ
домене. Ни то ни другое не меняет ответ: домен считается тем же кодом.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

from .envelope_queue_export import (
    POOL_DISPATCHED,
    POOL_TASK_FALLBACK,
    POOL_UNAVAILABLE,
    POOL_WALL_STAGE,
    POOL_WORKERS,
    _measure,
    _queue_snapshot_and_request,
)


@dataclass(frozen=True, slots=True)
class StagedQueueDomainV1:
    """Домен, вход которого посчитан ЗАРАНЕЕ, и то, что вернул воркер пула.

    `inputs` — `(snapshot, request)` либо `EnvelopeHostAdapterError`: отказ
    выгрузки хранится значением и разбирается тем же обработчиком, что и в
    последовательном пути, а не пересчитывается. Повторный вызов кэша сессии
    дал бы `_CACHE_HIT` вместо `_CACHE_MISS`, и счётчики разошлись бы с
    последовательным прогоном.

    `pooled` — ответ воркера (`DomainTaskResultV1`) либо `None`: домен в кэше,
    пул недоступен или задача упала, и домен считается прежним путём.
    `pool_outcome` и `pool_message` — именованная причина такого «либо»; пусто,
    когда всё шло как задумано.
    """

    inputs: object
    pooled: object | None = None
    pool_outcome: str = ""
    pool_message: str = ""


def _pool_reason(error: str) -> str:
    lines = [item for item in str(error).strip().splitlines() if item.strip()]
    return lines[-1].strip() if lines else "no reason recorded"


def stage_pool_domains(
    domain_pool,
    analysis_bundle,
    patch_ids,
    revision,
    selected_edges_by_domain,
    alpha,
    alpha_text: str,
    request_id: str,
    *,
    density,
    profile,
    topology_export=None,
    domain_snapshot_provider=None,
    preparation_cached=None,
    export_provider=None,
    export_adopter=None,
):
    """Фазы A и B пула: входы доменов, затем воркеры на те, что не в кэше.

    Фаза C — прежний последовательный цикл, который берёт отсюда готовые входы
    и ответы.

    `export_provider(patch_id, alpha, request_id, density)` отвечает лёгким
    входом выгрузки (`HostExportInputV1`) для домена, метрики которого нет в
    кэше сессии, и `None` для остальных: у тех выгрузка — попадание в кэш, и
    идёт здесь, в родителе (`_queue_snapshot_and_request`). Без провайдера
    (прогон без сессии) выгружается всё в родителе, как и прежде.
    `export_adopter(patch_id, result)` принимает ответ воркера в кэш сессии.
    """

    from .envelope_domain_pool import DomainTaskV1
    from .envelope_request_export import EnvelopeHostAdapterError, _typed_value

    def inputs_of(patch_id, domain_id, selected):
        try:
            return _queue_snapshot_and_request(
                analysis_bundle,
                patch_id,
                domain_id,
                selected,
                alpha,
                request_id,
                density=density,
                profile=profile,
                topology_export=topology_export,
                domain_snapshot_provider=domain_snapshot_provider,
            )
        except EnvelopeHostAdapterError as exc:
            return exc

    staged: dict[str, StagedQueueDomainV1] = {}
    tasks = []
    for patch_id in patch_ids:
        domain_id = _typed_value("patch-domain", revision, patch_id)
        selected = frozenset(selected_edges_by_domain[domain_id])
        export = (
            None
            if export_provider is None
            else export_provider(patch_id, alpha, request_id, density)
        )
        if export is not None:
            tasks.append(
                DomainTaskV1(
                    len(tasks),
                    patch_id,
                    domain_id,
                    None,
                    None,
                    alpha_text,
                    selected,
                    export,
                )
            )
            continue
        inputs = inputs_of(patch_id, domain_id, selected)
        staged[domain_id] = StagedQueueDomainV1(inputs)
        if isinstance(inputs, EnvelopeHostAdapterError):
            continue
        snapshot, request = inputs
        if preparation_cached is not None and preparation_cached(
            domain_id, selected, request
        ):
            continue
        tasks.append(
            DomainTaskV1(
                len(tasks),
                patch_id,
                domain_id,
                snapshot,
                request,
                alpha_text,
                selected,
            )
        )
    return _dispatch_to_pool(
        domain_pool, tasks, staged, profile, inputs_of, export_adopter
    )


def _dispatch_to_pool(
    domain_pool, tasks, staged, profile, inputs_of, export_adopter
):
    """Фаза B: задачи в пул и названный исход по каждой, счётчики пула.

    Задача с выгрузкой в воркере доводится до входов ЗДЕСЬ, после пула и в
    порядке доменов: принятый ответ кладётся в кэш сессии (`export_adopter`) и
    разбирается тем же `inputs_of`, что и родительская выгрузка, — поэтому
    кэш-счётчики, отказы метрики и отказы запроса те же. Домен, чья выгрузка
    отказала, воркеру «не уходил»: он не считается ни отправленным, ни упавшим.
    """

    from .envelope_domain_pool import DomainPoolUnavailable
    from .envelope_request_export import EnvelopeHostAdapterError

    run = None
    failure = ""
    if tasks:
        try:
            with _measure(profile, POOL_WALL_STAGE):
                run = domain_pool.run(tasks)
        except DomainPoolUnavailable as exc:
            failure = str(exc)
        except Exception as exc:  # noqa: BLE001 - пул не должен ронять Build
            failure = f"pool failed: {type(exc).__name__}: {exc}"
    if failure:
        print(
            f"[CFTUV][EnvelopeDomainPool] {POOL_UNAVAILABLE}: {failure}; "
            "QUEUE runs sequentially",
            flush=True,
        )
    dispatched = 0
    fallbacks = 0
    unavailable_named = False
    for task in tasks:
        result = None if run is None else run.results.get(task.task_id)
        if task.export is not None:
            if result is not None and (result.ok or result.refused):
                export_adopter(task.patch_id, result)
            staged[task.domain_id] = StagedQueueDomainV1(
                inputs_of(task.patch_id, task.domain_id, task.selected_edges)
            )
        entry = staged[task.domain_id]
        if isinstance(entry.inputs, EnvelopeHostAdapterError):
            continue
        if failure:
            if not unavailable_named:
                unavailable_named = True
                staged[task.domain_id] = replace(
                    entry,
                    pool_outcome=POOL_UNAVAILABLE,
                    pool_message=(
                        f"Domain pool unavailable: {_pool_reason(failure)}; "
                        "QUEUE domains computed sequentially"
                    ),
                )
            continue
        dispatched += 1
        if result is not None and result.ok:
            staged[task.domain_id] = replace(entry, pooled=result)
            continue
        error = (
            "task was not executed (every worker died)"
            if result is None
            else result.error or "worker refused a domain the host accepted"
        )
        fallbacks += 1
        print(
            f"[CFTUV][EnvelopeDomainPool] {POOL_TASK_FALLBACK} "
            f"{task.domain_id[-3:]}: {error}",
            flush=True,
        )
        staged[task.domain_id] = replace(
            entry,
            pool_outcome=POOL_TASK_FALLBACK,
            pool_message=(
                f"Domain pool task failed: {_pool_reason(error)}; "
                "domain recomputed sequentially"
            ),
        )
    profile.set_counter(POOL_WORKERS, 0 if run is None else run.workers)
    profile.set_counter(POOL_DISPATCHED, dispatched)
    profile.set_counter(POOL_TASK_FALLBACK, fallbacks)
    profile.set_counter(POOL_UNAVAILABLE, int(bool(failure)))
    return staged


def adopt_pooled_domain(
    pooled,
    patch_id: int,
    domain_id: str,
    selected_edges: frozenset[int],
    request,
    profile,
    preparation_adopter,
):
    """Ответ воркера как результат домена: подготовка уходит в кэш сессии.

    Тайминги `QUEUE_PREPARE` и `QUEUE_COVERAGE` — секунды, которые измерил сам
    воркер тем же `run_queue_domain`, что и последовательный путь. Подготовка
    проходит через кэш сессии (`preparation_adopter`) как промах со сборкой, и
    ползунок alpha находит её тёплой, как после последовательной кнопки.
    """

    prepared = pooled.prepared
    queue_domain = pooled.queue_domain
    if profile is not None:
        profile.add_timing(
            "QUEUE_PREPARE", queue_domain.prepare_seconds, domain_id
        )
        profile.add_timing(
            "QUEUE_COVERAGE", queue_domain.coverage_seconds, domain_id
        )
    if preparation_adopter is not None:
        prepared = preparation_adopter(
            patch_id, domain_id, selected_edges, request, prepared
        )
    return prepared, replace(queue_domain, preparation=prepared)


def pool_notice(staged, domain_id: str):
    """Именованная причина, по которой домен считался не воркером, — строкой."""

    if staged is None or not staged.pool_outcome:
        return ()
    from .envelope_request_export import (
        EnvelopeDebugHostDiagnosticV1,
        EnvelopeDebugHostSeverity,
    )

    return (
        EnvelopeDebugHostDiagnosticV1(
            staged.pool_outcome,
            EnvelopeDebugHostSeverity.UNSUPPORTED,
            staged.pool_message,
            domain_id,
        ),
    )


__all__ = (
    "StagedQueueDomainV1",
    "adopt_pooled_domain",
    "pool_notice",
    "stage_pool_domains",
)
