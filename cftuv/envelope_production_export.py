"""Продуктовый путь Envelope: подготовка из сессии -> покрытие -> `GeometryBatchV1`.

Модуль ничего не знает о Blender (`bpy` в него не попадает ни прямо, ни через
импорты) и ничего не решает о геометрии: хост ОТОБРАЖАЕТ контракты. Покрытие и
батч считает ядро (`conveyor_coverage` и `materialize.domain.materialize_domain`),
а здесь — только склейка: откуда взять готовую подготовку, где посчитать (воркер
пула либо родитель) и как назвать то, что не вышло.

КЛЮЧ ИСПОЛНЕНИЯ — домен `(DecalRequestId, PatchDomainId)` целиком: покрытие и
материализация берут ВСЕ источники домена вместе (AGENTS.md, п.2). Задача
пула — одна на домен; цепь за цепью с последующей сшивкой тут не бывает.

ЗАКОН ТОПОЛОГИИ — явный параметр, как закон UV: `HOST_DECAL_TOPOLOGY_POLICY` (кнопка
просит плоские многоугольники лент). Он идёт в задачу пула (`ProductionInputV1`), в
`produce_domain`, в результат домена (`decal_topology_law`) и в строку JSON; сетка
вершин и семантика от него не зависят, меняется только сборка граней.

ЗАКОН UV — явный параметр. Запрос подготовки несёт отладочный
`ENVELOPE_DEBUG_NO_UV_V1`, а продукту нужен `UV_DIRECT_STRIP_V1`. Подготовка от
закона UV не зависит (ключ её кэша — ревизия, домен, рёбра и подпись угловой
политики), поэтому нажатие продукта сразу после отладочной кнопки берёт ТЕ ЖЕ
подготовки из кэша сессии; закон приходит в материализацию через
`materialization_request(prepared, uv_policy_id=...)` — это скомпилированный
запрос подготовки с одной заменой, и ключ исполнения батча остаётся ровно тем,
с которым подготовка скомпилирована (аудит материализатора 2026-10-02).

ОТКУДА ПОДГОТОВКА. Тёплая сессия (на том же выделении, плотности и ревизии уже
считали) отдаёт подготовки без единой сборки: это доказывается счётчиками сборок
контроллера (`PRODUCTION_PREPARATION_BUILDS` = 0). Холодный домен — ОДНА задача пула:
подготовка и материализация подряд (`ColdProductionInputV1`, `prepare_for_production` ->
`produce_domain`), а подготовка возвращается ответом и входит в кэш сессии тем же
путём (`get_conveyor_preparation`), что и подготовка кнопки отладки: подготовки
продукта и отладки одни и те же объекты. Прежний холодный путь сперва гнал отладочный
вычислитель по ВСЕМ доменам (подготовка, покрытие, контур), а потом считал покрытие
второй раз: на `building` ~19 с работы воркеров впустую.

РЕЗУЛЬТАТЫ ДОМЕНОВ КЭШИРУЮТСЯ в сессии (`production_result_key`): ключ — ключ
подготовки, alpha текстом, закон UV, закон топологии и закон подъёма. Домен с тем же
ключом не считается вовсе (`PRODUCTION_RESULT_CACHE_HIT`, размещение `cache`), а
считается только то, чего под ключом нет (`..._MISS`). Правка выделения меняет ключ
подготовки лишь у доменов, которых она касается, поэтому снятая цепочка пересчитывает
ровно их. Вытеснение — по давности (`PRODUCTION_RESULT_CACHE_LIMIT`), а исключение
внутри домена (`PRODUCTION_DOMAIN_RAISED`) в кэш не кладётся.

ХРАНИЛИЩЕ ПО СОДЕРЖИМОМУ (`envelope_content_store`) переживает смену ревизии источника. Кэши выше
ключатся ревизией, а она — хэш всего меша: правка вершины или шва делала холодными ВСЕ домены. Домен
без метрики в кэше ревизии получает ключ содержимого (`envelope_content_key`: вход воркера без ревизии,
`alpha` и id запроса, номера патчей — рангами); с тем же ключом его подготовка и результат берутся из
хранилища, а результат переносится на ревизию, запрос и номер патча прогона в воркере пула
(`ProductionInputV1.relabel`), параллельно. Холодным остаётся домен, чьё содержимое изменилось.
Что именно перенос обещает и чего нет — в `envelope_content_store`; перенос, которого не вышло, назван
(`PRODUCTION_CONTENT_RELABEL_FAILED`), а домен посчитан заново.

ОТКАЗ НАЗВАН НА КАЖДОМ УРОВНЕ. Домен, чей вход не выгрузился, называется
исходом хоста (`EnvelopeDebugHostOutcome`); домен, который материализатор
отклонил, — исходом ядра (`MaterializationOutcome`); исключение внутри домена —
`PRODUCTION_DOMAIN_RAISED` с хвостом трассы; домен, не вернувшийся ни от воркера,
ни от родителя, — `PREPARATION_UNAVAILABLE`. Пул, который не стартовал
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

from .envelope_content_key import result_slot
from .envelope_content_store import ContentRelabelFailed, RelabelV1, carried_to_run
from .envelope_debug_profile import EnvelopeDebugProfileBuilderV1
from .envelope_host_labels import record_host_tokens
from .envelope_production_weld import (
    COUNTER_FACES_OFF_PLANE_AFTER_OFFSET,
    COUNTER_MAX_OFF_PLANE_AFTER_OFFSET,
    COUNTER_WELD_GROUPS,
    COUNTER_WELD_SEAMS_MARKED,
    COUNTER_WELD_VERTICES_MERGED,
    OUTCOME_WELD_MITER_FALLBACK,
    OUTCOME_WELD_POSITION_MISMATCH,
)
from .envelope_request_policy import (
    ENVELOPE_UV_POLICIES,
    ENVELOPE_UV_POLICY_DIRECT_STRIP,
)
from .envelope_stretch_lines import developable_stretch_lines
from .surface_ir import (
    HOST_DECAL_TOPOLOGY_POLICY,
    HOST_NEAR_PLANAR_LIFT_POLICY,
    HostDecalTopologyPolicy,
)

#: Закон UV продукта. Реестр законов — `envelope_request_policy`.
PRODUCTION_UV_POLICY = ENVELOPE_UV_POLICY_DIRECT_STRIP
#: Закон топологии продукта и допустимые имена (политика хоста, не выбор ядра).
PRODUCTION_TOPOLOGY_LAW = HOST_DECAL_TOPOLOGY_POLICY.value
PRODUCTION_TOPOLOGY_LAWS = frozenset(item.value for item in HostDecalTopologyPolicy)

MATERIALIZED = "MATERIALIZED"
OUTCOME_PREPARATION_UNAVAILABLE = "PREPARATION_UNAVAILABLE"
OUTCOME_DOMAIN_RAISED = "PRODUCTION_DOMAIN_RAISED"

#: Стадия профиля продукта и числа, которые он называет.
PRODUCTION_BUILD_KIND = "PRODUCTION"
#: 1, если хоть один домен был холодным (без подготовки в кэше сессии): считались подготовки.
PRODUCTION_COLD_FILL = "PRODUCTION_COLD_FILL"
PRODUCTION_PREPARATION_REUSED = "PRODUCTION_PREPARATION_REUSED"
PRODUCTION_PREPARATION_BUILDS = "PRODUCTION_PREPARATION_BUILDS"
PRODUCTION_PATCH_METRIC_BUILDS = "PRODUCTION_PATCH_METRIC_BUILDS"
PRODUCTION_DOMAIN_GEOMETRY_BUILDS = "PRODUCTION_DOMAIN_GEOMETRY_BUILDS"
PRODUCTION_DOMAINS = "PRODUCTION_DOMAINS"
PRODUCTION_MATERIALIZED = "PRODUCTION_MATERIALIZED"
PRODUCTION_REFUSED = "PRODUCTION_REFUSED"
#: Домены, чей результат лежал в кэше сессии, и домены, которые пришлось считать.
PRODUCTION_RESULT_CACHE_HIT = "PRODUCTION_RESULT_CACHE_HIT"
PRODUCTION_RESULT_CACHE_MISS = "PRODUCTION_RESULT_CACHE_MISS"
#: Хранилище по содержимому домена (`envelope_content_store`): домены с ключом содержимого, домены, чей
#: результат либо подготовка взяты оттуда при другой ревизии, результаты, перенесённые на ревизию прогона,
#: и переносы, которых не вышло (домен тогда считается заново, причина названа строкой консоли).
PRODUCTION_CONTENT_KEYED = "PRODUCTION_CONTENT_KEYED"
#: Из ключённых: сколько доменов этого прогона легло в хранилище. Меньше `KEYED` — домен, чей снапшот строили не
#: под записью токенов (он взят из кэша ревизии), и результат с исключением: переносить их потом нечем.
PRODUCTION_CONTENT_REGISTERED = "PRODUCTION_CONTENT_REGISTERED"
PRODUCTION_CONTENT_RESULT_REUSED = "PRODUCTION_CONTENT_RESULT_REUSED"
PRODUCTION_CONTENT_PREPARATION_REUSED = "PRODUCTION_CONTENT_PREPARATION_REUSED"
PRODUCTION_CONTENT_RELABELED = "PRODUCTION_CONTENT_RELABELED"
PRODUCTION_CONTENT_RELABEL_FAILED = "PRODUCTION_CONTENT_RELABEL_FAILED"
PRODUCTION_CONTENT_UNKEYED = "PRODUCTION_CONTENT_UNKEYED"

PLACEMENT_WORKER = "worker"
PLACEMENT_PARENT = "parent"
#: Домен не считался: результат взят из кэша сессии.
PLACEMENT_CACHED = "cache"
#: Домен считал родитель, и причина названа: те же имена, что у отладки.
PLACEMENT_UNAVAILABLE = "parent:ENVELOPE_DOMAIN_POOL_UNAVAILABLE"
PLACEMENT_FALLBACK = "parent:ENVELOPE_DOMAIN_POOL_TASK_FALLBACK"


@dataclass(frozen=True, slots=True)
class ProductionInputV1:
    """Вход задачи пула: пикл готовой подготовки, закон UV и закон топологии.

    Остальное воркер берёт из самой задачи (`DomainTaskV1.alpha_text`,
    `patch_id`, `domain_id`). Снапшот и запрос не пересылаются: запрос подготовки
    лежит в ней самой, а законы — по одному параметру. Закон топологии — поле со
    значением по умолчанию, поэтому задача без него (прежняя форма) читается.
    """

    blob: bytes
    uv_policy_id: str = PRODUCTION_UV_POLICY
    topology_law: str = PRODUCTION_TOPOLOGY_LAW
    #: Перенос результата на ревизию и запрос прогона (`RelabelV1`) делает воркер, параллельно; `None` — без
    #: переноса (подготовка этой ревизии). `carried` — `blob` это не подготовка, а готовый результат из
    #: хранилища по содержимому: воркер только переносит его.
    relabel: object | None = None
    carried: bool = False


@dataclass(frozen=True, slots=True)
class ColdProductionInputV1:
    """Вход задачи «холодный домен»: два закона; сами входы идут в самой задаче.

    Снапшот и запрос домена воркер берёт у задачи (`DomainTaskV1.snapshot`, `request`) либо
    выгружает сам (`DomainTaskV1.export`) — как у задачи отладочного вычислителя.
    """

    uv_policy_id: str = PRODUCTION_UV_POLICY
    topology_law: str = PRODUCTION_TOPOLOGY_LAW


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
    #: Закон топологии, по которому собрана сетка (`DecalTopologyLawV1.value`): пусто у отказа.
    decal_topology_law: str = ""
    seconds: float = field(default=0.0, compare=False)
    placement: str = field(default=PLACEMENT_PARENT, compare=False)
    #: Запись идентичностей хоста, при которых посчитан результат (`DomainLabelingV1`), либо `None`: по ней
    #: результат переносится на другую ревизию источника (`envelope_content_store.relabel_result`). Это
    #: происхождение идентичностей, а не ответ, поэтому в сравнение не входит.
    labels: object | None = field(default=None, compare=False, repr=False)

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
    topology_law: str = PRODUCTION_TOPOLOGY_LAW,
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
    if topology_law not in PRODUCTION_TOPOLOGY_LAWS:
        raise ValueError(f"unknown decal topology law {topology_law!r}")
    started = time.perf_counter()
    try:
        from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
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
            decal_topology_law=DecalTopologyLawV1(topology_law),
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
            decal_topology_law=result.decal_topology_law.value,
            seconds=time.perf_counter() - started,
        )
    except Exception:  # noqa: BLE001 - исход называется, а не теряется
        return _refusal(
            patch_id,
            domain_id,
            OUTCOME_DOMAIN_RAISED,
            _trace_tail(),
            time.perf_counter() - started,
        )


def prepare_for_production(snapshot, request):
    """Подготовка очереди с холодной памятью разложений: то, что делала кнопка отладки.

    Память разложений и счётчик внебюджетной работы обнуляются ПЕРЕД подготовкой
    (`run_queue_domain` делает ровно это): статьи бюджета не зависят от того, какие
    домены процесс уже видел, а в пуле — от того, как задачи легли на воркеры.
    """

    from cftuv_envelope.exact_sqrt_sum import (
        reset_factorization_memory,
        reset_unbudgeted_work,
    )
    from cftuv_envelope.wavefront import prepare_conveyor

    reset_factorization_memory()
    reset_unbudgeted_work()
    return prepare_conveyor(snapshot, request)


def _trace_tail() -> str:
    tail = traceback.format_exc().strip().splitlines()[-3:]
    return " | ".join(item.strip() for item in tail)


def solve_cold_production_task(task):
    """Воркер пула: холодный домен целиком — выгрузка (если нужна), подготовка, материализация.

    Подготовка и материализация идут подряд тем же кодом, что и порознь (`prepare_for_production`
    и `produce_domain`, который сам обнуляет память перед покрытием), поэтому ответ тот же, что
    у домена на подготовке из кэша. Ответ несёт и подготовку: у родителя её ещё нет, и в кэш
    сессии она входит так же, как подготовка отладочного вычислителя. Отказ выгрузки приходит
    ответом-отказом (`refusal`) и разбирается родителем, как разобрал бы он сам.
    """

    from .envelope_export_input import TaskInputsV1, task_inputs

    inputs = task_inputs(task)
    if not isinstance(inputs, TaskInputsV1):
        return inputs
    started = time.perf_counter()
    prepared = prepare_for_production(inputs.snapshot, inputs.request)
    prepare_seconds = time.perf_counter() - started
    inputs.profile.add_timing("QUEUE_PREPARE", prepare_seconds, task.domain_id)
    produced = produce_domain(
        task.patch_id,
        task.domain_id,
        prepared,
        task.alpha_text,
        uv_policy_id=task.cold.uv_policy_id,
        topology_law=task.cold.topology_law,
    )
    return inputs.result(
        prepared=prepared,
        production=_placed(
            replace(
                produced,
                seconds=produced.seconds + prepare_seconds,
                labels=inputs.labeling,
            ),
            PLACEMENT_WORKER,
        ),
        snapshot=inputs.snapshot if inputs.exported else None,
    )


def solve_production_task(task):
    """Воркер пула: домен продуктового пути на присланной подготовке.

    Ответ — `DomainTaskResultV1` с полем `production`; подготовку обратно не
    шлют, она у родителя уже есть.
    """

    from .envelope_domain_pool import DomainTaskResultV1

    production = task.production
    payload = pickle.loads(production.blob)
    result = (
        payload
        if production.carried
        else produce_domain(
            task.patch_id,
            _domain_of_preparation(task.domain_id, production.relabel),
            payload,
            task.alpha_text,
            uv_policy_id=production.uv_policy_id,
            topology_law=production.topology_law,
        )
    )
    if production.relabel is not None:
        try:
            result = carried_to_run(result, production.relabel)
        except ContentRelabelFailed:
            # Родитель повторит перенос, получит тот же отказ, назовёт его и посчитает домен заново.
            pass
    return DomainTaskResultV1(
        task.task_id,
        production=_placed(result, PLACEMENT_WORKER),
    )


def _domain_of_preparation(domain_id: str, relabel) -> str:
    """Домен, на подготовке которого считают: подготовка из хранилища несёт идентичности СВОЕЙ ревизии.

    Результат получает id домена той ревизии, при которой подготовка посчитана, и переносится целиком
    (`relabel_result`); id домена текущей ревизии, подставленный в него до переноса, был бы единственной
    строкой результата, которой нет в записи подготовки.
    """

    if relabel is None or relabel.base is None:
        return domain_id
    return relabel.base.domain_id()


def _placed(result: ProductionDomainResultV1, placement: str):
    return replace(result, placement=placement)


# --------------------------------------------------------------------------
# Родитель: что есть в сессии, что досчитать, куда отдать
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class _DomainEntryV1:
    """Что сессия знает о домене: отказ входа, готовый результат, подготовка либо ничего.

    `inputs` — `(snapshot, request)`, когда метрика домена лежит в кэше сессии; `export` —
    лёгкий вход выгрузки, когда её там нет (снапшот тогда выгрузит воркер, вместе с
    подготовкой). `result_key` и `cached` — ключ кэша результатов и результат под ним.
    """

    patch_id: int
    domain_id: str
    selected: frozenset
    failure: object | None = None
    prepared: object | None = None
    inputs: tuple | None = None
    export: object | None = None
    result_key: tuple | None = None
    cached: object | None = None
    #: Ключ содержимого домена (`None`: не адресуется по содержимому), запись идентичностей подготовки из
    #: хранилища и что оттуда взято: `result` либо `preparation` (пусто: ничего).
    content_key: str | None = None
    labeling: object | None = None
    reuse: str = ""
    #: Готовый результат из хранилища по содержимому, ещё НЕ перенесённый на ревизию прогона: перенос — работа
    #: (воркера либо родителя), поэтому домен остаётся в `needs_work`, но не считается и не холоден.
    carried: object | None = None

    @property
    def is_cold(self) -> bool:
        return (
            self.failure is None
            and self.prepared is None
            and self.cached is None
            and self.carried is None
        )

    @property
    def needs_work(self) -> bool:
        return self.failure is None and self.cached is None

    @property
    def computes(self) -> bool:
        """Домену нужно ПОСЧИТАТЬ ответ (а не взять готовый из кэша ревизии или из хранилища)."""

        return self.needs_work and self.carried is None


@dataclass(frozen=True, slots=True)
class _RunInputsV1:
    """Всё, что одному прогону известно о выделении, сессии и законах."""

    controller: object
    analysis_bundle: object
    topology_export: object
    revision: str
    patch_ids: tuple
    selected_by_domain: dict
    alpha: float
    alpha_text: str
    request_id: str
    density: object
    uv_policy_id: str
    topology_law: str
    hooks: object
    profile: object
    #: Что хранилище по содержимому сделало в ЭТОМ прогоне: патчи, чей результат перенесён на ревизию
    #: прогона, и переносы, которых не вышло (`(патч, причина)`).
    relabeled: list = field(default_factory=list)
    relabel_failures: list = field(default_factory=list)
    registered: list = field(default_factory=list)


def _inputs_of(run: _RunInputsV1, entry_key, provider):
    from .envelope_queue_export import _queue_snapshot_and_request

    patch_id, domain_id, selected = entry_key
    return _queue_snapshot_and_request(
        run.analysis_bundle,
        patch_id,
        domain_id,
        selected,
        run.alpha,
        run.request_id,
        density=run.density,
        topology_export=run.topology_export,
        domain_snapshot_provider=provider,
        snapshot_issues_of=run.controller.snapshot_issues,
    )


def _result_key(run: _RunInputsV1, domain_id, selected, request) -> tuple:
    return run.controller.production_result_key(
        run.revision,
        domain_id,
        selected,
        request,
        run.alpha_text,
        run.uv_policy_id,
        run.topology_law,
        HOST_NEAR_PLANAR_LIFT_POLICY.value,
    )


def _entry_with_inputs(run: _RunInputsV1, patch_id, domain_id, selected, inputs):
    """Домен с выгруженным входом: подготовка и результат из кэшей сессии, если они там есть."""

    controller = run.controller
    request = inputs[1]
    key = _result_key(run, domain_id, selected, request)
    return _DomainEntryV1(
        patch_id,
        domain_id,
        selected,
        prepared=controller.peek_conveyor_preparation(
            run.revision, domain_id, selected, request
        ),
        inputs=inputs,
        result_key=key,
        cached=controller.peek_production_result(key),
    )


def _slot(run: _RunInputsV1) -> tuple:
    """Слот результата в хранилище содержимого: alpha и законы материализации прогона."""

    return result_slot(
        run.alpha_text,
        run.uv_policy_id,
        run.topology_law,
        HOST_NEAR_PLANAR_LIFT_POLICY.value,
    )


def _bound_entry(run: _RunInputsV1, patch_id, domain_id, selected):
    """Домен, чей ключ содержимого уже известен при этой ревизии и чей результат лежит в кэше ревизии.

    Ключ — функция входа домена, а вход при ревизии, выделении, плотности и допуске один: по привязке
    `(ревизия, домен, выделение, плотность, допуск)` повторное нажатие не строит ни вход воркера, ни ключ.
    """

    binding = _binding(run, domain_id, selected)
    key = None if binding is None else run.controller.content_binding(binding)
    if key is None:
        return None
    cached = run.controller.peek_production_result(("content", key, run.revision, _slot(run)))
    if cached is None:
        return None
    return _DomainEntryV1(
        patch_id, domain_id, selected, cached=cached, content_key=key, reuse="result"
    )


def _binding(run: _RunInputsV1, domain_id, selected):
    from .envelope_request_policy import normalize_envelope_fan_density

    try:
        density = normalize_envelope_fan_density(run.density)
    except (TypeError, ValueError):
        return None
    return (
        run.revision,
        domain_id,
        selected,
        density,
        run.topology_export.developable_stretch_budget,
    )


def _content_entry(run: _RunInputsV1, patch_id, domain_id, selected, export) -> _DomainEntryV1:
    """Домен без метрики в кэше ревизии: из хранилища по содержимому, либо холодный с ключом.

    Правка меша меняет ревизию и сбрасывает кэши ревизии, но домен, чьё содержимое то же, имеет тот же
    ключ: готовый результат из хранилища идёт на перенос к ревизии прогона (`carried`), а готовая
    подготовка идёт воркеру как подготовка из кэша. Домен, которого в хранилище нет, считается как
    раньше — с ключом, чтобы по окончании лечь в хранилище. Домен, чей вход ключ не умеет кодировать,
    считается как раньше без ключа (счётчик `PRODUCTION_CONTENT_UNKEYED`).
    """

    from .envelope_content_key import ContentKeyUnsupported, domain_content_key

    cold = _DomainEntryV1(patch_id, domain_id, selected, export=export)
    try:
        key = domain_content_key(export, selected)
    except ContentKeyUnsupported:
        return cold
    controller = run.controller
    binding = _binding(run, domain_id, selected)
    if binding is not None:
        controller.bind_content_key(binding, key)
    slot = _slot(run)
    cold = replace(cold, content_key=key)
    result_key = ("content", key, run.revision, slot)
    cached = controller.peek_production_result(result_key)
    if cached is not None:
        return replace(cold, export=None, cached=cached, reuse="result")
    found = controller.content_store.find(key)
    if found is None:
        return cold
    stored = controller.content_store.result(key, slot)
    if stored is not None:
        labels = stored.labels
        if labels.revision == run.revision and labels.patch_id == patch_id:
            controller.remember_production_result(result_key, stored)
            return replace(cold, export=None, cached=stored, reuse="result")
        return replace(
            cold, carried=stored, result_key=result_key, labeling=found.labeling, reuse="result"
        )
    return replace(
        cold,
        prepared=found.prepared,
        labeling=found.labeling,
        result_key=result_key,
        reuse="preparation",
    )


def _note_relabel_failure(run: _RunInputsV1, patch_id, exc) -> None:
    """Перенос не вышел: счётчик и строка консоли; домен при этом считается заново, а не пропадает."""

    run.relabel_failures.append((patch_id, str(exc)))
    print(
        f"[CFTUV][Production] {PRODUCTION_CONTENT_RELABEL_FAILED}: patch {patch_id}: {exc}; "
        "the domain is computed anew",
        flush=True,
    )


def _scan(run: _RunInputsV1):
    """Домены по кэшам сессии, БЕЗ сборки: чего нет в кэше, то холодно.

    Метрика, снапшот, подготовка и результат читаются из кэшей контроллера; промах метрики
    называется холодным доменом с лёгким входом выгрузки, а не собирается здесь в родителе
    (сборка метрик — работа воркеров пула в холодной задаче).
    """

    from .envelope_request_export import EnvelopeHostAdapterError, _typed_value

    controller = run.controller

    def snapshots(patch_id, _domain_id):
        metric = controller.get_patch_metric(run.topology_export, patch_id)
        return controller.get_domain_geometry(metric).snapshot

    entries = []
    for patch_id in run.patch_ids:
        domain_id = _typed_value("patch-domain", run.revision, patch_id)
        selected = frozenset(run.selected_by_domain[domain_id])
        bound = _bound_entry(run, patch_id, domain_id, selected)
        if bound is not None:
            entries.append(bound)
            continue
        export = run.hooks.export_provider(
            patch_id, run.alpha, run.request_id, run.density
        )
        if export is not None:
            entries.append(_content_entry(run, patch_id, domain_id, selected, export))
            continue
        try:
            inputs = _inputs_of(run, (patch_id, domain_id, selected), snapshots)
        except EnvelopeHostAdapterError as exc:
            entries.append(_DomainEntryV1(patch_id, domain_id, selected, failure=exc))
            continue
        entries.append(_entry_with_inputs(run, patch_id, domain_id, selected, inputs))
    return entries


def _host_refusal(entry: _DomainEntryV1, failure=None) -> ProductionDomainResultV1:
    failure = entry.failure if failure is None else failure
    outcome = getattr(failure.outcome, "value", failure.outcome)
    return _refusal(entry.patch_id, entry.domain_id, outcome, str(failure))


def _remember(run: _RunInputsV1, key, result) -> None:
    """Результат — в кэш сессии; исключение внутри домена не кэшируется (оно не ответ)."""

    if key is not None and result.outcome != OUTCOME_DOMAIN_RAISED:
        run.controller.remember_production_result(key, result)


def _produce_cold_in_parent(run: _RunInputsV1, entry, inputs, placement):
    """Холодный домен считает родитель: подготовка через кэш сессии, затем `produce_domain`.

    Сюда идёт домен, которого не взял воркер (нет пула, малая партия, упавшая задача), и
    ответ тот же: код тот же. Исключение подготовки называется, а не уходит в кнопку.
    """

    snapshot, request = inputs
    started = time.perf_counter()
    try:
        prepared = run.controller.get_conveyor_preparation(
            run.revision,
            entry.domain_id,
            entry.selected,
            request,
            lambda: prepare_for_production(snapshot, request),
            profile=run.profile,
        )
    except Exception:  # noqa: BLE001 - исход называется, а не теряется
        return _refusal(
            entry.patch_id,
            entry.domain_id,
            OUTCOME_DOMAIN_RAISED,
            _trace_tail(),
            time.perf_counter() - started,
            placement,
        )
    return _placed(
        produce_domain(
            entry.patch_id,
            entry.domain_id,
            prepared,
            run.alpha_text,
            uv_policy_id=run.uv_policy_id,
            topology_law=run.topology_law,
        ),
        placement,
    )


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
    if answered and entry.export is not None:
        run.hooks.export_adopter(entry.patch_id, reply)
    elif answered:
        replay_export_records(run.profile, reply)
    inputs = entry.inputs
    parent_log = None
    if inputs is None:
        with record_host_tokens() as parent_log:
            try:
                inputs = _inputs_of(
                    run,
                    (entry.patch_id, entry.domain_id, entry.selected),
                    run.hooks.snapshot_provider,
                )
            except EnvelopeHostAdapterError as exc:
                return exc, None
    request = inputs[1]
    if reply is not None and reply.ok and reply.production is not None:
        prepared = reply.prepared
        run.controller.get_conveyor_preparation(
            run.revision,
            entry.domain_id,
            entry.selected,
            request,
            lambda: prepared,
            profile=run.profile,
        )
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
    return replace(result, labels=labeling) if labeling.has_domain_token() else result


def _register_content(run: _RunInputsV1, entry: _DomainEntryV1, request, result) -> None:
    """Подготовка и результат холодного домена — в хранилище по содержимому, если у них есть запись токенов.

    Домен, чья запись неполна (снапшот строили не под записью), и исключение внутри домена (оно не ответ)
    в хранилище не попадают: перенести такой результат на другую ревизию точно было бы нечем.
    """

    controller = run.controller
    if entry.content_key is None or result.labels is None or result.outcome == OUTCOME_DOMAIN_RAISED:
        return
    prepared = controller.peek_conveyor_preparation(
        run.revision, entry.domain_id, entry.selected, request
    )
    if prepared is None:
        return
    store = controller.content_store
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
    based = result if result.labels is not None else replace(result, labels=labeling)
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
        moved = replace(moved, seconds=0.0, placement=PLACEMENT_CACHED)
    _remember(run, item.result_key, moved)
    return None, moved


def _complete_ready(run: _RunInputsV1, ready, done, refused, placement) -> None:
    """Домены на готовой подготовке либо результате: посчитанное воркером берётся, остальное делает родитель."""

    for item in ready:
        if item.domain_id not in done:
            done[item.domain_id] = _placed(
                item.carried
                if item.carried is not None
                else produce_domain(
                    item.patch_id,
                    item.labeling.domain_id() if item.labeling is not None else item.domain_id,
                    item.prepared,
                    run.alpha_text,
                    uv_policy_id=run.uv_policy_id,
                    topology_law=run.topology_law,
                ),
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
    return ProductionInputV1(
        blob, run.uv_policy_id, run.topology_law, relabel, entry.carried is not None
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
        )
        for index, ((patch_id, domain_id, _payload), blob) in enumerate(shipped)
    ]
    laws = ColdProductionInputV1(run.uv_policy_id, run.topology_law)
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
    if not (sent_cold or _worth_the_pool(shipped)):
        shipped = []
    tasks = _worker_tasks(run, ready, shipped, sent_cold) if (shipped or sent_cold) else []
    pooled, failure = _run_tasks(domain_pool, tasks, run.profile) if tasks else (None, "")
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
    return done, refused


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


def _domain_results(entries, done, refused):
    """Результаты в порядке доменов: отказ входа, результат из кэша, посчитанное либо названная пропажа."""

    results = []
    for entry in entries:
        if entry.failure is not None:
            results.append(_host_refusal(entry))
        elif entry.domain_id in refused:
            results.append(_host_refusal(entry, refused[entry.domain_id]))
        elif entry.cached is not None:
            results.append(replace(entry.cached, seconds=0.0, placement=PLACEMENT_CACHED))
        elif entry.domain_id in done:
            results.append(done[entry.domain_id])
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
            if item.failure is None and (item.cached is not None or item.carried is not None)
        ),
    )
    profile.set_counter(
        PRODUCTION_RESULT_CACHE_MISS, sum(1 for item in entries if item.computes)
    )
    profile.set_counter(PRODUCTION_DOMAINS, len(results))
    profile.set_counter(
        PRODUCTION_MATERIALIZED, sum(1 for item in results if item.is_materialized)
    )
    profile.set_counter(
        PRODUCTION_REFUSED, sum(1 for item in results if not item.is_materialized)
    )


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
) -> ProductionRunV1:
    """Один продуктовый прогон по доменам выделения: сессия, пул, названные исходы.

    Домен с результатом в кэше сессии не считается вовсе; домен с подготовкой в кэше
    считает только покрытие и материализацию; холодный домен — подготовку и
    материализацию одной задачей. Счётчики сборок контроллера в профиле
    (`PRODUCTION_*_BUILDS`) и счётчики кэша результатов — доказательство повторного
    использования, а не секунды.
    """

    from .envelope_domain_pool import get_domain_pool
    from .envelope_topology_export import stage_domain_inputs
    from .envelope_worker_python import read_worker_python

    if uv_policy_id not in ENVELOPE_UV_POLICIES:
        raise ValueError(f"unknown UV policy {uv_policy_id!r}")
    if topology_law not in PRODUCTION_TOPOLOGY_LAWS:
        raise ValueError(f"unknown decal topology law {topology_law!r}")
    started = time.perf_counter()
    profile = EnvelopeDebugProfileBuilderV1(
        getattr(analysis_bundle.source_revision, "source_name", "source"),
        PRODUCTION_BUILD_KIND,
    )
    builds_before = controller.build_counts
    selected = frozenset(int(item) for item in selected_physical_edge_ids)
    topology_export = controller.get_topology_export(
        analysis_bundle, source_object_key, source_data_key, profile=profile
    ).with_developable_stretch_budget(developable_stretch_budget)
    _scene, revision, patch_ids, request_id, selected_by_domain = stage_domain_inputs(
        analysis_bundle, selected, profile=profile, topology_export=topology_export
    )
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
    )
    entries = _scan(run)
    cold = any(item.is_cold for item in entries)
    work = [item for item in entries if item.needs_work]
    pool = (
        get_domain_pool(workers, read_worker_python())
        if domain_pool is _FROM_SETTINGS
        else domain_pool
    )
    with profile.measure("PRODUCTION_DOMAINS_WALL"):
        done, refused = _dispatch(
            run,
            [item for item in work if item.prepared is not None or item.carried is not None],
            [item for item in work if item.prepared is None and item.carried is None],
            pool,
        )
    results = _domain_results(entries, done, refused)
    _record_run_counters(profile, controller, builds_before, entries, results, cold)
    _record_content_counters(profile, run, entries)
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
    lines.extend(developable_stretch_lines(results))
    weld = dict(getattr(receipt, "weld_counters", ()) or ())
    if weld.get(COUNTER_WELD_GROUPS):
        lines.append(
            f"[CFTUV][Production] WELD: {weld[COUNTER_WELD_GROUPS]} shared vertices "
            f"({weld[COUNTER_WELD_VERTICES_MERGED]} domain vertices merged), "
            f"{weld[COUNTER_WELD_SEAMS_MARKED]} fold seams, "
            f"position mismatches {weld[OUTCOME_WELD_POSITION_MISMATCH]}, "
            f"miter fallbacks {weld[OUTCOME_WELD_MITER_FALLBACK]}"
        )
    offset = dict(getattr(receipt, "offset_counters", ()) or ())
    if offset.get(COUNTER_FACES_OFF_PLANE_AFTER_OFFSET):
        lines.append(
            f"[CFTUV][Production] OFFSET: {offset[COUNTER_FACES_OFF_PLANE_AFTER_OFFSET]} faces of 4+ vertices "
            f"leave their plane after the offset, at most {offset[COUNTER_MAX_OFF_PLANE_AFTER_OFFSET] / 1e6:.3f} mm "
            "(recorded, not judged)"
        )
    lines.append(f"[CFTUV][Production] {receipt_status_text(receipt)}")
    return lines


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
        f"{_pool_timing_suffix(run.profile)}"
    )


def _content_timing_suffix(run: ProductionRunV1, content: int) -> str:
    """` | content reused R results, P preparations`, когда хранилище по содержимому что-то отдало."""

    if not content:
        return ""
    return (
        f" | content reused {run.counter(PRODUCTION_CONTENT_RESULT_REUSED)} results, "
        f"{run.counter(PRODUCTION_CONTENT_PREPARATION_REUSED)} preparations"
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
            "decal_topology_law": item.decal_topology_law,
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
    "ColdProductionInputV1",
    "MATERIALIZED",
    "OUTCOME_DOMAIN_RAISED",
    "OUTCOME_PREPARATION_UNAVAILABLE",
    "PLACEMENT_CACHED",
    "PRODUCTION_COLD_FILL",
    "PRODUCTION_DOMAINS",
    "PRODUCTION_DOMAIN_GEOMETRY_BUILDS",
    "PRODUCTION_MATERIALIZED",
    "PRODUCTION_PATCH_METRIC_BUILDS",
    "PRODUCTION_PREPARATION_BUILDS",
    "PRODUCTION_PREPARATION_REUSED",
    "PRODUCTION_REFUSED",
    "PRODUCTION_RESULT_CACHE_HIT",
    "PRODUCTION_RESULT_CACHE_MISS",
    "PRODUCTION_TOPOLOGY_LAW",
    "PRODUCTION_TOPOLOGY_LAWS",
    "PRODUCTION_UV_POLICY",
    "ProductionDomainResultV1",
    "ProductionInputV1",
    "ProductionRunV1",
    "developable_stretch_lines",
    "diagnostic_summary_lines",
    "export_production_json",
    "prepare_for_production",
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
