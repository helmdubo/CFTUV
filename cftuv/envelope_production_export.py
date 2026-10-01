"""Продуктовый путь Envelope: подготовка из сессии -> покрытие -> `GeometryBatchV1`.

Модуль ничего не знает о Blender (`bpy` в него не попадает ни прямо, ни через
импорты) и ничего не решает о геометрии: хост ОТОБРАЖАЕТ контракты. Покрытие и
батч считает ядро (`conveyor_coverage` и `materialize.domain.materialize_domain`),
а здесь — только склейка: откуда взять готовую подготовку, где посчитать (воркер
пула либо родитель) и как назвать то, что не вышло.

КЛЮЧ ИСПОЛНЕНИЯ — домен `(DecalRequestId, PatchDomainId)` целиком: покрытие и
материализация берут ВСЕ источники домена вместе (AGENTS.md, п.2). Задача
пула — одна на домен; цепь за цепью с последующей сшивкой тут не бывает.

ЗАКОН UV — явный параметр. Запрос подготовки несёт отладочный
`ENVELOPE_DEBUG_NO_UV_V1`, а продукту нужен `UV_DIRECT_STRIP_V1`. Подготовка от
закона UV не зависит (ключ её кэша — ревизия, домен, рёбра и подпись угловой
политики), поэтому нажатие продукта сразу после отладочной кнопки берёт ТЕ ЖЕ
подготовки из кэша сессии; закон приходит в материализацию через
`materialization_request(prepared, uv_policy_id=...)` — это скомпилированный
запрос подготовки с одной заменой, и ключ исполнения батча остаётся ровно тем,
с которым подготовка скомпилирована (аудит материализатора 2026-10-02).

ОТКУДА ПОДГОТОВКА. Тёплая сессия (кнопка отладки уже нажата на том же выделении,
плотности и ревизии) отдаёт подготовки без единой сборки: это доказывается
счётчиками сборок контроллера (`PRODUCTION_PREPARATION_BUILDS` = 0). Холодная —
запускает ТОТ ЖЕ отладочный вычислитель очереди (`controller.evaluate_staged`)
ради кэшей, без отрисовки, и дальше идёт как тёплая: второго способа подготовить
домен здесь нет, поэтому подготовки продукта и отладки одни и те же объекты.

ОТКАЗ НАЗВАН НА КАЖДОМ УРОВНЕ. Домен, чей вход не выгрузился, называется
исходом хоста (`EnvelopeDebugHostOutcome`); домен, который материализатор
отклонил, — исходом ядра (`MaterializationOutcome`); исключение внутри домена —
`PRODUCTION_DOMAIN_RAISED` с хвостом трассы; домен без подготовки после
наполнения кэша — `PREPARATION_UNAVAILABLE`. Пул, который не стартовал
(`ENVELOPE_DOMAIN_POOL_UNAVAILABLE`), и задача, которая упала
(`ENVELOPE_DOMAIN_POOL_TASK_FALLBACK`), — счётчик профиля, строка консоли и
пометка `placement` домена; считает при этом родитель, ТЕМ ЖЕ `produce_domain`.
"""

from __future__ import annotations

import json
import pickle
import time
import traceback
from dataclasses import dataclass, field, replace
from pathlib import Path

from .envelope_debug_profile import EnvelopeDebugProfileBuilderV1
from .envelope_request_policy import (
    ENVELOPE_UV_POLICIES,
    ENVELOPE_UV_POLICY_DIRECT_STRIP,
)
from .surface_ir import HOST_NEAR_PLANAR_LIFT_POLICY

#: Закон UV продукта. Реестр законов — `envelope_request_policy`.
PRODUCTION_UV_POLICY = ENVELOPE_UV_POLICY_DIRECT_STRIP

MATERIALIZED = "MATERIALIZED"
OUTCOME_PREPARATION_UNAVAILABLE = "PREPARATION_UNAVAILABLE"
OUTCOME_DOMAIN_RAISED = "PRODUCTION_DOMAIN_RAISED"

#: Стадия профиля продукта и числа, которые он называет.
PRODUCTION_BUILD_KIND = "PRODUCTION"
PRODUCTION_COLD_FILL = "PRODUCTION_COLD_FILL"
PRODUCTION_PREPARATION_REUSED = "PRODUCTION_PREPARATION_REUSED"
PRODUCTION_PREPARATION_BUILDS = "PRODUCTION_PREPARATION_BUILDS"
PRODUCTION_PATCH_METRIC_BUILDS = "PRODUCTION_PATCH_METRIC_BUILDS"
PRODUCTION_DOMAIN_GEOMETRY_BUILDS = "PRODUCTION_DOMAIN_GEOMETRY_BUILDS"
PRODUCTION_DOMAINS = "PRODUCTION_DOMAINS"
PRODUCTION_MATERIALIZED = "PRODUCTION_MATERIALIZED"
PRODUCTION_REFUSED = "PRODUCTION_REFUSED"

PLACEMENT_WORKER = "worker"
PLACEMENT_PARENT = "parent"
#: Домен считал родитель, и причина названа: те же имена, что у отладки.
PLACEMENT_UNAVAILABLE = "parent:ENVELOPE_DOMAIN_POOL_UNAVAILABLE"
PLACEMENT_FALLBACK = "parent:ENVELOPE_DOMAIN_POOL_TASK_FALLBACK"


@dataclass(frozen=True, slots=True)
class ProductionInputV1:
    """Вход задачи пула: пикл готовой подготовки и закон UV.

    Остальное воркер берёт из самой задачи (`DomainTaskV1.alpha_text`,
    `patch_id`, `domain_id`). Снапшот и запрос не пересылаются: запрос подготовки
    лежит в ней самой, а закон UV — один параметр.
    """

    blob: bytes
    uv_policy_id: str = PRODUCTION_UV_POLICY


@dataclass(frozen=True, slots=True)
class ProductionDomainResultV1:
    """Исход продуктового пути по ОДНОМУ домену: батч либо названный отказ.

    `outcome` — `MATERIALIZED` либо имя отказа (исход ядра, исход хоста либо
    одно из двух имён этого модуля). `normal` — единичная нормаль плоскости
    домена, лицевая сторона сетки (направление смещения над поверхностью —
    политика писателя меша). `counters` — числа материализатора, всё это ответ.

    Не входят в сравнение: секунды и `placement` — где посчитан домен. Размещение
    — свойство запуска, а не ответа (так же, как счётчики пула отладки).
    """

    patch_id: int
    domain_id: str
    outcome: str
    batch: object | None
    detail: str = ""
    counters: tuple[tuple[str, int], ...] = ()
    normal: tuple[float, float, float] | None = None
    content_digest: str = ""
    diagnostics: tuple[str, ...] = ()
    #: Нормаль первой исходной грани владельца (то, что видит источник) —
    #: свидетельство для именованной проверки хоста `normal . source_normal > 0`.
    source_normal: tuple[float, float, float] | None = None
    #: Ориентация карты домена (`AffineChartOrientationV1.value`): от неё зависят
    #: обход треугольников и знак нормали, поэтому она идёт в сводку зонда.
    chart_orientation: str = ""
    #: Нормаль смещения КАЖДОЙ вершины батча (`vert_key`, единичная) у домена-развёртки
    #: и закон, который её дал (`SOURCE_VERTEX_ANGLE_WEIGHTED_NORMAL_V1`): у развёртки нет
    #: плоскости источника, смещение декали идёт по нормали вершины, а `normal` выше —
    #: лишь сводка (нормированное среднее). У плоского и near-planar домена пусто.
    vertex_normals: tuple = ()
    offset_normal_law: str = ""
    #: Побитовый sha256 этих нормалей (`offset_normals_digest` ядра): они сдвигают вершины
    #: меша, но в дайджест батча не входят, и только этот дайджест виден воротам и свипу.
    offset_normals_digest: str = ""
    seconds: float = field(default=0.0, compare=False)
    placement: str = field(default=PLACEMENT_PARENT, compare=False)

    @property
    def is_materialized(self) -> bool:
        return self.outcome == MATERIALIZED and self.batch is not None


def _refusal(patch_id, domain_id, outcome, detail, seconds=0.0, placement=PLACEMENT_PARENT):
    return ProductionDomainResultV1(
        patch_id=int(patch_id),
        domain_id=str(domain_id),
        outcome=str(outcome),
        batch=None,
        detail=str(detail),
        seconds=seconds,
        placement=placement,
    )


def _source_normal(prepared):
    """Нормаль первой (по имени) исходной грани патча-владельца либо `None`."""

    owner = prepared.compilation.owner_patch_id
    faces = sorted(
        (
            item
            for item in prepared.context.snapshot.surface_ir.source_faces
            if item.patch_id == owner
        ),
        key=lambda item: item.face_id.value,
    )
    normal = faces[0].polygon_normal if faces else None
    return None if normal is None else (normal.x, normal.y, normal.z)


def _summary_normal(vertex_normals):
    """Нормированное среднее нормалей вершин развёртки либо `None` (у плоского домена их нет)."""

    if not vertex_normals:
        return None
    total = [sum(item[1][axis] for item in vertex_normals) for axis in range(3)]
    length = sum(axis * axis for axis in total) ** 0.5
    if not length:
        return None
    from cftuv_envelope.numeric import LocalVector3V1

    return LocalVector3V1(*(axis / length for axis in total))


def produce_domain(
    patch_id: int,
    domain_id: str,
    prepared,
    alpha_text: str,
    *,
    uv_policy_id: str = PRODUCTION_UV_POLICY,
) -> ProductionDomainResultV1:
    """Покрытие на готовой подготовке и материализация: общий код всех путей.

    Его зовут воркер пула (`solve_production_task`) и родитель (малая партия,
    пул недоступен, задача упала, ноль воркеров) — размещение не меняет ответ,
    потому что код один. Память разложений сбрасывается перед доменом: статьи
    бюджета материализации не зависят от истории процесса (как у кнопки отладки).
    Исключение внутри домена не роняет пул и не теряется: оно называется.
    """

    if uv_policy_id not in ENVELOPE_UV_POLICIES:
        raise ValueError(f"unknown UV policy {uv_policy_id!r}")
    started = time.perf_counter()
    try:
        from cftuv_envelope.exact_sqrt_sum import (
            reset_factorization_memory,
            reset_unbudgeted_work,
        )
        from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
        from cftuv_envelope.materialize.admit import (
            admit_domain,
            materialization_request,
        )
        from cftuv_envelope.materialize.domain import materialize_domain
        from cftuv_envelope.materialize.lift import plane_normal_binary64
        from cftuv_envelope.wavefront import conveyor_coverage

        reset_factorization_memory()
        reset_unbudgeted_work()
        if prepared.outcome.value != "EXACT":
            # Ядро называет это само: подготовка без компиляции запроса не
            # имеет, и отказ приходит из `admit` до любой работы.
            refused = admit_domain(prepared, None, None)
            return _refusal(
                patch_id,
                domain_id,
                refused.outcome.value,
                refused.detail,
                time.perf_counter() - started,
            )
        coverage = conveyor_coverage(prepared, alpha_text)
        result = materialize_domain(
            prepared,
            coverage,
            request=materialization_request(prepared, uv_policy_id=uv_policy_id),
            near_planar_lift_law=NearPlanarLiftLawV1(HOST_NEAR_PLANAR_LIFT_POLICY.value),
        )
        if not result.is_materialized:
            return _refusal(
                patch_id,
                domain_id,
                result.outcome.value,
                result.detail,
                time.perf_counter() - started,
            )
        normal = _summary_normal(result.vertex_normals) or plane_normal_binary64(prepared.context.frame)
        return ProductionDomainResultV1(
            patch_id=int(patch_id),
            domain_id=str(domain_id),
            outcome=MATERIALIZED,
            batch=result.batch,
            detail="",
            counters=tuple((str(name), value) for name, value in result.counters),
            normal=(normal.x, normal.y, normal.z),
            content_digest=result.content_digest,
            diagnostics=tuple(result.diagnostics),
            source_normal=_source_normal(prepared),
            chart_orientation=str(prepared.context.frame.chart_orientation.value),
            vertex_normals=tuple(result.vertex_normals),
            offset_normal_law=result.offset_normal_law,
            offset_normals_digest=result.offset_normals_digest,
            seconds=time.perf_counter() - started,
        )
    except Exception:  # noqa: BLE001 - исход называется, а не теряется
        tail = traceback.format_exc().strip().splitlines()[-3:]
        return _refusal(
            patch_id,
            domain_id,
            OUTCOME_DOMAIN_RAISED,
            " | ".join(item.strip() for item in tail),
            time.perf_counter() - started,
        )


def solve_production_task(task):
    """Воркер пула: домен продуктового пути на присланной подготовке.

    Ответ — `DomainTaskResultV1` с полем `production`; подготовку обратно не
    шлют, она у родителя уже есть.
    """

    from .envelope_domain_pool import DomainTaskResultV1

    prepared = pickle.loads(task.production.blob)
    result = produce_domain(
        task.patch_id,
        task.domain_id,
        prepared,
        task.alpha_text,
        uv_policy_id=task.production.uv_policy_id,
    )
    return DomainTaskResultV1(
        task.task_id,
        production=_placed(result, PLACEMENT_WORKER),
    )


def _placed(result: ProductionDomainResultV1, placement: str):
    return replace(result, placement=placement)


# --------------------------------------------------------------------------
# Родитель: что есть в сессии, что досчитать, куда отдать
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class _DomainEntryV1:
    """Что кэши сессии знают о домене: отказ входа, подготовка либо ничего."""

    patch_id: int
    domain_id: str
    selected: frozenset
    failure: object | None = None
    prepared: object | None = None

    @property
    def is_cold(self) -> bool:
        return self.failure is None and self.prepared is None


def _scan(controller, analysis_bundle, topology_export, revision, patch_ids, selected_by_domain, alpha, request_id, density):
    """Домены по кэшам сессии, БЕЗ сборки: чего нет в кэше, то холодно.

    Метрика, снапшот и подготовка читаются из кэшей контроллера; промах метрики
    называется холодным доменом, а не собирается здесь в родителе (сборка
    метрик — работа воркеров пула в холодном наполнении).
    """

    from .envelope_queue_export import _queue_snapshot_and_request
    from .envelope_request_export import EnvelopeHostAdapterError, _typed_value

    def snapshots(patch_id, _domain_id):
        metric = controller.get_patch_metric(topology_export, patch_id)
        return controller.get_domain_geometry(metric).snapshot

    entries = []
    for patch_id in patch_ids:
        domain_id = _typed_value("patch-domain", revision, patch_id)
        selected = frozenset(selected_by_domain[domain_id])
        if not controller.has_patch_metric(topology_export, patch_id):
            entries.append(_DomainEntryV1(patch_id, domain_id, selected))
            continue
        try:
            _snapshot, request = _queue_snapshot_and_request(
                analysis_bundle,
                patch_id,
                domain_id,
                selected,
                alpha,
                request_id,
                density=density,
                topology_export=topology_export,
                domain_snapshot_provider=snapshots,
            )
        except EnvelopeHostAdapterError as exc:
            entries.append(_DomainEntryV1(patch_id, domain_id, selected, failure=exc))
            continue
        entries.append(
            _DomainEntryV1(
                patch_id,
                domain_id,
                selected,
                prepared=controller.peek_conveyor_preparation(
                    revision, domain_id, selected, request
                ),
            )
        )
    return entries


def _host_refusal(entry: _DomainEntryV1) -> ProductionDomainResultV1:
    outcome = getattr(entry.failure.outcome, "value", entry.failure.outcome)
    return _refusal(entry.patch_id, entry.domain_id, outcome, str(entry.failure))


def _dispatch(items, domain_pool, controller, alpha_text, uv_policy_id, profile):
    """`{domain_id: результат}`: воркеры на то, что окупает пересылку, родитель на остальное.

    Названные исходы те же, что у покрытия отладки: пул, который не стартовал
    (`ENVELOPE_DOMAIN_POOL_UNAVAILABLE`), и задача, которая упала или чей воркер
    умер (`ENVELOPE_DOMAIN_POOL_TASK_FALLBACK`), — счётчик профиля, строка
    консоли и пометка `placement`; домен при этом считает родитель тем же
    `produce_domain`. Партия, которая стоит меньше пересылки, остаётся в родителе
    без исхода: это не отказ, а размещение.
    """

    from .envelope_domain_pool import DomainTaskV1
    from .envelope_queue_pool import (
        _note_task_fallback,
        _record_pool_counters,
        _run_tasks,
        _ship_preparations,
        _task_error,
        _worth_the_pool,
    )

    triples = [(item.patch_id, item.domain_id, item.prepared) for item in items]
    shipped, shipping_failures = (
        _ship_preparations(triples, controller.preparation_blobs, lambda item: item[2])
        if domain_pool is not None and triples
        else ([], {})
    )
    tasks = []
    if _worth_the_pool(shipped) and shipped:
        selected_of = {item.domain_id: item.selected for item in items}
        tasks = [
            DomainTaskV1(
                index,
                patch_id,
                domain_id,
                None,
                None,
                alpha_text,
                selected_of[domain_id],
                production=ProductionInputV1(blob, uv_policy_id),
            )
            for index, ((patch_id, domain_id, _prepared), blob) in enumerate(shipped)
        ]
    run, failure = _run_tasks(domain_pool, tasks, profile) if tasks else (None, "")
    done: dict[str, ProductionDomainResultV1] = {}
    placement: dict[str, str] = {}
    dispatched = fallbacks = 0
    for task in tasks:
        if failure:
            placement[task.domain_id] = PLACEMENT_UNAVAILABLE
            continue
        dispatched += 1
        result = None if run is None else run.results.get(task.task_id)
        if result is not None and result.ok and result.production is not None:
            done[task.domain_id] = result.production
            continue
        fallbacks += 1
        _note_task_fallback(task.domain_id, _task_error(run, task))
        placement[task.domain_id] = PLACEMENT_FALLBACK
    for domain_id, reason in shipping_failures.items():
        fallbacks += 1
        _note_task_fallback(domain_id, reason)
        placement[domain_id] = PLACEMENT_FALLBACK
    for item in items:
        if item.domain_id in done:
            continue
        done[item.domain_id] = _placed(
            produce_domain(
                item.patch_id,
                item.domain_id,
                item.prepared,
                alpha_text,
                uv_policy_id=uv_policy_id,
            ),
            placement.get(item.domain_id, PLACEMENT_PARENT),
        )
    if domain_pool is not None:
        _record_pool_counters(profile, run, failure, dispatched, fallbacks, dispatched)
    return done


@dataclass(frozen=True, slots=True)
class ProductionRunV1:
    """Итог продуктового прогона: результаты по доменам и числа прогона."""

    results: tuple[ProductionDomainResultV1, ...]
    revision: str
    cold: bool
    profile: object
    wall_seconds: float

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
    domain_pool=_FROM_SETTINGS,
) -> ProductionRunV1:
    """Один продуктовый прогон по доменам выделения: сессия, пул, названные исходы.

    Тёплый прогон не собирает ничего: счётчики сборок контроллера в профиле
    (`PRODUCTION_*_BUILDS`) — доказательство повторного использования. Холодный
    сперва наполняет кэши отладочным вычислителем очереди (то же, что делает
    кнопка отладки, без отрисовки), затем идёт как тёплый.
    """

    from .envelope_domain_pool import get_domain_pool
    from .envelope_topology_export import stage_domain_inputs
    from .envelope_worker_python import read_worker_python

    if uv_policy_id not in ENVELOPE_UV_POLICIES:
        raise ValueError(f"unknown UV policy {uv_policy_id!r}")
    started = time.perf_counter()
    profile = EnvelopeDebugProfileBuilderV1(
        getattr(analysis_bundle.source_revision, "source_name", "source"),
        PRODUCTION_BUILD_KIND,
    )
    builds_before = controller.build_counts
    selected = frozenset(int(item) for item in selected_physical_edge_ids)
    topology_export = controller.get_topology_export(
        analysis_bundle, source_object_key, source_data_key, profile=profile
    )
    _scene, revision, patch_ids, request_id, selected_by_domain = stage_domain_inputs(
        analysis_bundle, selected, profile=profile, topology_export=topology_export
    )
    scan_args = (
        controller,
        analysis_bundle,
        topology_export,
        revision,
        patch_ids,
        selected_by_domain,
        alpha,
        request_id,
        density,
    )
    entries = _scan(*scan_args)
    reused = sum(1 for item in entries if item.prepared is not None)
    cold = any(item.is_cold for item in entries)
    if cold:
        with profile.measure(PRODUCTION_COLD_FILL):
            controller.evaluate_staged(
                analysis_bundle,
                selected,
                alpha,
                source_object_key=source_object_key,
                source_data_key=source_data_key,
                profile=profile,
                engine="QUEUE",
                density=density,
                workers=workers,
            )
        entries = _scan(*scan_args)
    pool = (
        get_domain_pool(workers, read_worker_python())
        if domain_pool is _FROM_SETTINGS
        else domain_pool
    )
    alpha_text = str(float(alpha))
    ready = [item for item in entries if item.prepared is not None]
    with profile.measure("PRODUCTION_DOMAINS_WALL"):
        produced = _dispatch(ready, pool, controller, alpha_text, uv_policy_id, profile)
    results = []
    for entry in entries:
        if entry.failure is not None:
            results.append(_host_refusal(entry))
        elif entry.prepared is None:
            results.append(
                _refusal(
                    entry.patch_id,
                    entry.domain_id,
                    OUTCOME_PREPARATION_UNAVAILABLE,
                    "no preparation in the session after the debug-equivalent fill",
                )
            )
        else:
            results.append(produced[entry.domain_id])
    builds_after = controller.build_counts
    for name, key in (
        (PRODUCTION_PREPARATION_BUILDS, "CONVEYOR_PREPARATION"),
        (PRODUCTION_PATCH_METRIC_BUILDS, "PATCH_METRIC"),
        (PRODUCTION_DOMAIN_GEOMETRY_BUILDS, "DOMAIN_GEOMETRY"),
    ):
        profile.set_counter(name, builds_after[key] - builds_before[key])
    profile.set_counter(PRODUCTION_PREPARATION_REUSED, reused)
    profile.set_counter(PRODUCTION_COLD_FILL, int(cold))
    profile.set_counter(PRODUCTION_DOMAINS, len(results))
    profile.set_counter(
        PRODUCTION_MATERIALIZED, sum(1 for item in results if item.is_materialized)
    )
    profile.set_counter(
        PRODUCTION_REFUSED, sum(1 for item in results if not item.is_materialized)
    )
    return ProductionRunV1(
        results=tuple(results),
        revision=revision,
        cold=cold,
        profile=profile.snapshot(),
        wall_seconds=time.perf_counter() - started,
    )


# --------------------------------------------------------------------------
# Строки владельцу и свидетельства
# --------------------------------------------------------------------------


def refused_outcome_counts(results) -> dict[str, int]:
    """`{исход: сколько доменов}` по отказам, порядок — по имени исхода."""

    counts: dict[str, int] = {}
    for item in results:
        if not item.is_materialized:
            counts[item.outcome] = counts.get(item.outcome, 0) + 1
    return dict(sorted(counts.items()))


def _outcome_counts(rows) -> dict[str, int]:
    counts: dict[str, int] = {}
    for _patch, _domain, outcome, _detail in rows:
        counts[outcome] = counts.get(outcome, 0) + 1
    return dict(sorted(counts.items()))


def _status_text(written: int, rows, warnings=()) -> str:
    """`MATERIALIZED n / refused m (OUTCOME x2, OTHER)` и, если есть, `| warnings`."""

    rows = tuple(rows)
    text = f"MATERIALIZED {written} / refused {len(rows)}"
    counts = _outcome_counts(rows)
    if counts:
        text += " (" + ", ".join(
            name if count == 1 else f"{name} x{count}"
            for name, count in counts.items()
        ) + ")"
    names: dict[str, int] = {}
    for _patch, outcome, _detail in warnings:
        names[outcome] = names.get(outcome, 0) + 1
    if names:
        text += " | warnings: " + ", ".join(
            name if count == 1 else f"{name} x{count}"
            for name, count in sorted(names.items())
        )
    return text


def _refused_rows(results):
    return [
        (item.patch_id, item.domain_id, item.outcome, item.detail)
        for item in results
        if not item.is_materialized
    ]


def production_status_text(results) -> str:
    """Строка по результатам ПРОДУКТОВОГО пути (до записи меша)."""

    results = tuple(results)
    return _status_text(
        sum(1 for item in results if item.is_materialized), _refused_rows(results)
    )


def receipt_status_text(receipt) -> str:
    """Строка панели по КВИТАНЦИИ записи: сколько домен лежит в меше, а остальное названо.

    Пропуск писателя (`ADAPTER_*`) стоит в ней наравне с отказом продуктового
    пути: домен, которого нет в меше, не может молчать по любой из причин.
    """

    return _status_text(len(receipt.domains), receipt.skipped, receipt.warnings)


def receipt_report_level(receipt) -> str:
    """`WARNING`, если хоть один домен не в меше (по любой причине) либо есть находка; иначе `INFO`."""

    return "WARNING" if receipt.skipped or receipt.warnings else "INFO"


def _row_line(kind, patch_id, domain_id, outcome, detail) -> str:
    return (
        f"[CFTUV][Production] {kind} patch {patch_id} "
        f"(domain ...{str(domain_id)[-6:]}): {outcome}"
        + (f": {detail}" if detail else "")
    )


def production_console_lines(results) -> list[str]:
    """Каждый отказанный домен — строкой с исходом и деталью; затем итог."""

    results = tuple(results)
    lines = [
        _row_line("REFUSED", *row) for row in _refused_rows(results)
    ]
    lines.append(f"[CFTUV][Production] {production_status_text(results)}")
    return lines


def diagnostic_summary_lines(results) -> list[str]:
    """Диагностики батчей (`NEAR_PLANAR_...`, `U_RESTARTS_...`) по именам: сколько доменов и каких."""

    found: dict[str, list[int]] = {}
    for item in results:
        for line in item.diagnostics:
            found.setdefault(line.split(":", 1)[0], []).append(item.patch_id)
    lines = []
    for name, patches in sorted(found.items()):
        shown = sorted(set(patches))
        tail = ", ".join(str(item) for item in shown[:12])
        more = f", ... (+{len(shown) - 12})" if len(shown) > 12 else ""
        lines.append(
            f"[CFTUV][Production] DIAGNOSTIC {name}: {len(patches)} in "
            f"{len(shown)} domains (patch {tail}{more})"
        )
    return lines


def receipt_console_lines(receipt, results) -> list[str]:
    """Консольная сводка квитанции: каждый пропущенный домен, предупреждения, диагностики, итог."""

    lines = [_row_line("REFUSED", *row) for row in receipt.skipped]
    for patch_id, outcome, detail in receipt.warnings:
        where = "mesh" if patch_id is None else f"patch {patch_id}"
        lines.append(
            f"[CFTUV][Production] WARNING {where}: {outcome}"
            + (f": {detail}" if detail else "")
        )
    lines.extend(diagnostic_summary_lines(results))
    lines.append(f"[CFTUV][Production] {receipt_status_text(receipt)}")
    return lines


def production_timing_text(run: ProductionRunV1) -> str:
    """Секунды прогона и работа пула в одну строку панели."""

    from .envelope_queue_export import _pool_timing_suffix

    kind = "cold" if run.cold else "warm"
    reused = run.counter(PRODUCTION_PREPARATION_REUSED)
    builds = run.counter(PRODUCTION_PREPARATION_BUILDS)
    return (
        f"Decal {kind} {run.wall_seconds:.2f} s | preparations reused {reused}, "
        f"built {builds}{_pool_timing_suffix(run.profile)}"
    )


def export_production_json(results, directory, *, label: str = "production") -> Path:
    """Батчи MATERIALIZED-доменов и сводка в `directory`: свидетельство для зонда.

    Батч идёт кодеком ядра (`GeometryBatchCodecV1`): канонические байты, их же
    читает `GeometryBatchCodecV1.loads`. Сводка — исход, детали и дайджесты
    каждого домена, всё без секунд (сравнимо между прогонами и воркерами).
    """

    from cftuv_envelope import GeometryBatchCodecV1

    folder = Path(directory)
    folder.mkdir(parents=True, exist_ok=True)
    rows = []
    for item in results:
        row = {
            "patch_id": item.patch_id,
            "domain_id": item.domain_id,
            "outcome": item.outcome,
            "detail": item.detail,
            "content_digest": item.content_digest,
            "counters": dict(item.counters),
            "diagnostics": list(item.diagnostics),
            "normal": None if item.normal is None else list(item.normal),
            "offset_normal_law": item.offset_normal_law,
            "offset_normals_digest": item.offset_normals_digest,
        }
        if item.is_materialized:
            name = f"{label}_patch{item.patch_id:04d}.geometry_batch.json"
            (folder / name).write_bytes(GeometryBatchCodecV1.dumps(item.batch))
            row["batch_file"] = name
            row["semantic_digest"] = item.batch.semantic_digest.value
        rows.append(row)
    summary = folder / f"{label}_summary.json"
    summary.write_text(
        json.dumps(
            {"label": label, "domains": rows, "status": production_status_text(results)},
            ensure_ascii=False,
            indent=1,
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    return summary


__all__ = (
    "MATERIALIZED",
    "OUTCOME_DOMAIN_RAISED",
    "OUTCOME_PREPARATION_UNAVAILABLE",
    "PRODUCTION_COLD_FILL",
    "PRODUCTION_DOMAINS",
    "PRODUCTION_DOMAIN_GEOMETRY_BUILDS",
    "PRODUCTION_MATERIALIZED",
    "PRODUCTION_PATCH_METRIC_BUILDS",
    "PRODUCTION_PREPARATION_BUILDS",
    "PRODUCTION_PREPARATION_REUSED",
    "PRODUCTION_REFUSED",
    "PRODUCTION_UV_POLICY",
    "ProductionDomainResultV1",
    "ProductionInputV1",
    "ProductionRunV1",
    "diagnostic_summary_lines",
    "export_production_json",
    "produce_domain",
    "production_console_lines",
    "receipt_console_lines",
    "receipt_report_level",
    "receipt_status_text",
    "production_status_text",
    "production_timing_text",
    "refused_outcome_counts",
    "run_production",
    "solve_production_task",
)
