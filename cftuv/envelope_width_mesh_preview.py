"""Живое превью меша ширины: склейка модели из аффинного сертификата (`envelope_width_preview_model`) с Blender, сессией и планировщиками.

ЧТО ВИДИТ ВЛАДЕЛЕЦ. Пока он тянет ширину (модальный «Adjust Decal Width» либо ползунок), НАСТОЯЩИЙ меш декали двигается каждый кадр: позиции
и UV пишутся в существующий меш из модели, а точный пересчёт идёт в фоне через тот же планировщик, что и прежде, и его результат
заменяет превью. Строка статуса называет превью (`PREVIEW_MESH_FROM_AFFINE_CERTIFICATE_V1`), придержанные и карантинные домены с их причинами (`PARAM_*`).
Ядро заверяет интервал событий и аффинность внутри него (точное равенство, `kernel/tests/parametric_theorem.py`); решения тесселяции, силуэта и митры сварки модель проверяет апостериори.
Домены-развёртки двигаются приближённо и названы отдельно (`PREVIEW_DEVELOPABLE_FROZEN_NORMAL_V1`: точные наклоны, смещение по нормалям, замороженным на ширине образца; предел ошибки
`DISPLAY_APPROXIMATE_ERROR_LIMIT`, досягаемость `DISPLAY_APPROXIMATE_REACH_RATIO`): такой кадр только на экране, точный результат (lean-такт, подтверждение) заменяет его и записывается как прежде.

ОТКУДА МОДЕЛЬ. Из ОДНОГО точного прогона: на экране лежит ОБРАЗЕЦ (`PreviewSampleV1`: массивы меша и домены прогона, чья геометрия сейчас в меше, вместе с наклонами сертификата ядра), модель
из него складывает базу и наклон. Никаких затравочных прогонов: вход в инструмент не ждёт ничего, модель строится из образца, который кнопка или точное обновление уже положили на
экран, при первом кадре (`ensure_model`, миллисекунды numpy), а каждый следующий точный результат заводит модель сам в потоке счёта (`finish_live_run`). Тот же поток судит прежнюю модель новым
точным прогоном (`deviation`) и ведёт журнал доверия (`advance_ledger`): опровергнутый домен в карантине, возврат — по правилу (шапка модуля модели).

ЧТО ПИШЕТСЯ И ГДЕ. Только позиции и UV, только в СУЩЕСТВУЮЩИЙ меш (`write_preview_geometry`: `foreach_set`, без пересоздания датаблоков, поэтому
таймеры и модальный оператор не ломают шаг отмены).

КОМУ ПРИНАДЛЕЖИТ МЕШ (аудит ad6074f, F2; `MeshOwnershipV1`). Размеры, ширина и ревизия в свойствах меша — не доказательство: меш с теми же числами
и свойствами мог быть подменён (копия датаблока, другая диагональ треугольников, ручная правка). Поэтому КАЖДАЯ точная запись (кнопка,
живая ширина: `capture_ownership`) фиксирует в памяти сессии, а не в свойствах меша: указатель и `session_uid` объекта и меша, число вершин, петель,
граней и рёбер, ПОКОЛЕНИЕ раскладки (счётчик записей точного пути) и отпечаток раскладки (индексы вершин петель и начала граней, `blake2b`),
снятый тут же после записи; образец, чью раскладку записал точный путь, связан с этим владением тождеством объекта. Кадр проверяет за O(1): образец модели
равен образцу владения (иначе `PREVIEW_MESH_LAYOUT_GENERATION_STALE`), указатели и `session_uid` (`PREVIEW_DECAL_OBJECT_REPLACED`,
`PREVIEW_MESH_DATABLOCK_REPLACED`), размеры, ширину и ревизию (`PREVIEW_MESH_NOT_THE_BASE`), режим Edit (`PREVIEW_DECAL_IN_EDIT_MODE`); отпечаток
раскладки перечитывается не чаще раза в `LAYOUT_RECHECK_SECONDS` (и не чаще, чем стоит `LAYOUT_RECHECK_COST_RATIO` его прочтений) — сеть на
случай пропущенного сигнала (`PREVIEW_MESH_LAYOUT_CHANGED`). СТРОГАЯ ИНВАЛИДАЦИЯ: любое внешнее изменение снимает модель сразу, до следующего кадра, —
Undo/Redo и загрузка файла (`PREVIEW_HISTORY_STEP`), обновление геометрии декали в depsgraph, которое сделали не мы
(`note_decal_updates`, `PREVIEW_DECAL_CHANGED_EXTERNALLY`; свою запись обработчик узнаёт по поколению записи на каждый depsgraph, OWN_WRITE_GENERATION_V1), вход в Edit, подмена
объекта или датаблока. Модель снята с названной причиной (`PREVIEW_MODEL_DROPPED:<причина>`), записи в меш нет, превью молчит, линии остаются, а точный
путь работает как прежде: следующий точный результат заводит образец и владение заново.

ТОЧНЫЙ ПУТЬ НЕ ТРОНУТ. Меш после точного результата равен кнопке побитово (общий код записи; массивы строятся тем же `build_mesh_arrays`),
а отмена модального инструмента возвращает базу ПОБИТОВО (`restore_base_mesh`: кадр на `dt = 0` — сама база). Этот модуль `bpy` лениво
импортирует только в кадре и в проверках цели (стена `tests/test_architecture.py`); построение образцов и моделей от Blender не зависит.
"""

from __future__ import annotations

import hashlib
import time
from dataclasses import dataclass, field, replace

import numpy as np

from .envelope_gc_pause import gc_paused_function
from .envelope_production_result import PLACEMENT_REBASED, REFRESH_INCREMENTAL, REFRESH_LEAN
from .envelope_width_incremental import (
    DEFERRABLE_REASONS,
    HELD_NOT_IN_BASE,
    INCREMENTAL_MIN_SERVED_SHARE,
    REASON_FEW_SERVED,
    REASON_MODEL_FOREIGN,
    REASON_NO_MODEL,
    REASON_NO_ORIGINS,
    VERIFY_BACKGROUND,
    IncrementalPlanV1,
    IncrementalStateV1,
    full_plan,
    next_state,
    selection_reason,
    verification_reason,
    verify_batch,
)
from .envelope_width_composite import compose_sample
from .envelope_width_rebase import rebase_result
from .envelope_width_preview_model import (
    DISPLAY_APPROXIMATE_ERROR_LIMIT,
    DOMAIN_DISPLAY_APPROXIMATE,
    PREVIEW_MESH_FROM_AFFINE_CERTIFICATE_V1,
    DeviationV1,
    PreviewModelV1,
    PreviewRefusalV1,
    PreviewSampleV1,
    advance_ledger,
    base_frame,
    build_model,
    deviation,
    evaluate,
    held_name,
    moving_patches,
    moving_patches_at,
    sample_of,
    served_mask,
)

MODEL_DROPPED = "PREVIEW_MODEL_DROPPED"
MODEL_BUILD_FAILED = "PREVIEW_MODEL_BUILD_FAILED"
DECAL_GONE = "PREVIEW_DECAL_GONE"
DECAL_IN_EDIT_MODE = "PREVIEW_DECAL_IN_EDIT_MODE"
MESH_NOT_THE_BASE = "PREVIEW_MESH_NOT_THE_BASE"
#: Принадлежность меша (F2): владения нет, образец не из этого поколения, объект и датаблок подменены, раскладка изменилась,
#: геометрию декали изменил не точный путь, шаг истории.
OWNERSHIP_UNKNOWN = "PREVIEW_MESH_OWNERSHIP_UNKNOWN"
OWNERSHIP_CAPTURE_FAILED = "PREVIEW_MESH_OWNERSHIP_CAPTURE_FAILED"
LAYOUT_GENERATION_STALE = "PREVIEW_MESH_LAYOUT_GENERATION_STALE"
DECAL_OBJECT_REPLACED = "PREVIEW_DECAL_OBJECT_REPLACED"
MESH_DATABLOCK_REPLACED = "PREVIEW_MESH_DATABLOCK_REPLACED"
MESH_LAYOUT_CHANGED = "PREVIEW_MESH_LAYOUT_CHANGED"
DECAL_CHANGED_EXTERNALLY = "PREVIEW_DECAL_CHANGED_EXTERNALLY"
HISTORY_STEP = "PREVIEW_HISTORY_STEP"
#: Точный кадр во время перетаскивания (WIDTH_REFRESH_LEAN_V1, `lean_exact_now`) не лёг в меш: названа причина, меш не тронут, а модель (если меш не наш) снята.
LEAN_NO_BASE = "WIDTH_REFRESH_LEAN_NO_DISPLAYED_SAMPLE"
#: Потоковый такт перетаскивания снят концом перетаскивания (отмена, подтверждение): его результат не ляжет ни в это, ни в следующее перетаскивание.
LEAN_DRAG_ENDED_BY_FINISH = "LIVE_DRAG_TICK_RETIRED_AT_DRAG_END"
LEAN_KEY_MISMATCH = "WIDTH_REFRESH_LEAN_KEY_MISMATCH"
#: Как часто кадр перечитывает отпечаток раскладки меша (сеть на случай пропущенного сигнала) и сколько кадров он вправе стоить в среднем:
#: интервал не меньше `LAYOUT_RECHECK_COST_RATIO` длительностей самого чтения.
LAYOUT_RECHECK_SECONDS = 0.25
LAYOUT_RECHECK_COST_RATIO = 20.0
#: Сколько номеров придержанных патчей печатает строка статуса.
HELD_NAMES_SHOWN = 6


@dataclass(slots=True)
class PreviewLogV1:
    """Счётчики превью меша за жизнь сессии: что построено, что отказано, сколько кадров и чего они стоили, как сверка и владение."""

    models: int = 0
    refusals: dict = field(default_factory=dict)
    dropped: dict = field(default_factory=dict)
    frames: int = 0
    frame_seconds_max: float = 0.0
    frame_seconds_total: float = 0.0
    checks: int = 0
    domains_checked: int = 0
    domains_refuted: int = 0
    max_position_error: float = 0.0
    max_uv_error: float = 0.0
    #: Наибольшее отношение отклонения к пределу среди судимых доменов за все сверки (1.0 — предел).
    max_deviation_ratio: float = 0.0
    #: Исходы журнала доверия: имя -> сколько раз (карантин, возврат со сжатой областью, снятие модели домена).
    trust_events: dict = field(default_factory=dict)
    #: Последняя сверка текстом строки статуса.
    last_check: str = ""
    last_model_bytes: int = 0
    #: Владение мешем: номер поколения раскладки, сколько раз перечитан отпечаток и самое долгое чтение.
    layout_generation: int = 0
    layout_checks: int = 0
    layout_check_seconds_max: float = 0.0
    #: Точные кадры во время перетаскивания (WIDTH_REFRESH_LEAN_V1): сколько легло в меш, из них сколько пересобрало меш на месте (раскладка сменилась), сколько секунд главный
    #: поток платил за запись и сколько раз отмена возвращала образец на экране (кадры успели его заменить).
    lean_frames: int = 0
    lean_rebuilds: int = 0
    lean_frame_seconds_total: float = 0.0
    lean_origin_restores: int = 0
    #: Инкрементальные такты (WIDTH_REFRESH_INCREMENTAL_V1), легшие в меш: сколько доменов обслужил сертификат и сколько посчитано заново (суммы по тактам), секунды сдвига; и такты, заказанные
    #: инкрементальными, но ставшие полными lean-тактами (сверка), по названной причине.
    incremental_ticks: int = 0
    incremental_served: int = 0
    incremental_recomputed: int = 0
    incremental_plan_seconds_total: float = 0.0
    incremental_full: dict = field(default_factory=dict)
    #: Из инкрементальных тактов: сколько сложены в numpy (`envelope_width_composite`) и сколько собраны из сдвинутых результатов всех доменов под названной причиной.
    incremental_composite: int = 0
    incremental_fallback: dict = field(default_factory=dict)
    #: WIDTH_PREVIEW_INPUTS_DEFERRED_V1: кто построил входы линий превью записи кнопки (`idle` - простой после нажатия, `event` - первое событие либо действие раньше него): имя -> сколько раз.
    inputs_built: dict = field(default_factory=dict)

    def refuse(self, name: str) -> None:
        self.refusals[name] = self.refusals.get(name, 0) + 1


@dataclass(frozen=True, slots=True)
class PreviewMeshStateV1:
    """Что показывает меш сейчас: ширина кадра, живые и придержанные домены (номера патчей), исход и цена кадра.

    `moving` — номера патчей, которые кадр двигает целиком (пусто у отказанного кадра): линии превью считаются только для остальных
    (`PREVIEW_CONTOUR_ONLY_WHERE_MODEL_HELD_V1`, `envelope_width_live.width_event_preview`).
    """

    alpha: float
    live: int
    held: int
    held_patches: tuple
    outcome: str
    seconds: float
    serial: int
    moving: frozenset = frozenset()
    #: Сколько из `live` доменов приближённые (`PREVIEW_DEVELOPABLE_FROZEN_NORMAL_V1`: точная аффинная часть плюс замороженное смещение); остальные живые точны.
    approximate: int = 0


@dataclass(frozen=True, slots=True)
class DragOriginV1:
    """Что показывали сессия и меш, когда первый точный lean-кадр перетаскивания собрался заменить образец на экране: к этому возвращает отмена.

    Не начало перетаскивания, а момент перед первой заменой: модель, что легла за время перетаскивания до неё, принадлежит этому состоянию.
    """

    displayed: PreviewSampleV1 | None
    model: object | None
    model_refusal: object | None
    trust: object | None
    #: Запись сверки инкрементальных тактов (`IncrementalStateV1`) на тот же момент.
    incremental: object | None = None


@dataclass(frozen=True, slots=True)
class LiveRunV1:
    """Результат потока точного счёта: прогон, массивы меша (их же пишет главный поток), образец, модель, сверка и журнал доверия."""

    run: object
    arrays: object
    sample: PreviewSampleV1
    #: `PreviewModelV1`, `PreviewRefusalV1` либо `None` (модели не на чем строиться).
    model: object | None
    check: DeviationV1 | None
    #: Журнал доверия после сверки этого прогона (`TrustLedgerV1`) и его события `((патч, домен, имя), ...)`.
    trust: object | None = None
    trust_events: tuple = ()
    #: Решение тика (`IncrementalPlanV1`): инкрементальный такт (часть доменов обслужил сертификат) либо полный lean-такт под названной причиной; `None` - тик не заказывал инкрементальности.
    plan: IncrementalPlanV1 | None = None


@dataclass(slots=True)
class MeshOwnershipV1:
    """Кому принадлежит меш декали: что точный путь записал и что кадр сверяет. Память сессии, не свойства меша (их копия сохраняет)."""

    #: Образец, чью раскладку записал точный путь (тождество): модель чужого образца этому владению не принадлежит.
    sample: object
    #: Поколение раскладки: номер записи точного пути в сессии.
    generation: int
    object_pointer: int
    object_uid: int
    mesh_pointer: int
    mesh_uid: int
    vertices: int
    loops: int
    polygons: int
    edges: int
    #: Отпечаток раскладки: индексы вершин петель и начала граней (`blake2b`), снятый сразу после записи.
    layout: bytes
    verified_at: float
    check_seconds: float
    #: OWN_WRITE_GENERATION_V1: номер нашей последней записи (поколение). Запись при захвате владения — первая.
    own_writes: int = 1
    #: Какое поколение нашей записи уже доложил каждый depsgraph (ключ — depsgraph: окно, слой вида). Один depsgraph докладывает о нашей
    #: записи один раз; второй слой вида или второе окно — свой ключ, и его доклад тоже наш, а не внешний.
    seen_writes: dict = field(default_factory=dict)
    corner_scratch: object = None
    start_scratch: object = None

    def recheck_after(self) -> float:
        return max(LAYOUT_RECHECK_SECONDS, LAYOUT_RECHECK_COST_RATIO * self.check_seconds)

    def note_own_write(self) -> None:
        self.own_writes += 1

    def claim_update(self, source) -> bool:
        """`True` — обновление геометрии от `source` докладывает о нашей записи (поколение ещё не доложено этим depsgraph): оно наше."""

        if self.seen_writes.get(source, 0) >= self.own_writes:
            return False
        self.seen_writes[source] = self.own_writes
        return True

    @property
    def pending_own_update(self) -> bool:
        """Есть ли наша запись, о которой ещё не доложил ни один depsgraph (для отчётов и тестов)."""

        return not self.seen_writes or max(self.seen_writes.values()) < self.own_writes


# --------------------------------------------------------------------------
# Ключ и образец (поток точного счёта и кнопка)
# --------------------------------------------------------------------------


def sample_key(record, offset: float) -> tuple:
    """Чья это геометрия: источник, ревизия, бэкенд ядра, плотность, допуски, смещение и номер сброса сессии.

    Образец и модель другого ключа не смешиваются и не применяются: другой объект, другая ревизия, другой бэкенд либо
    другая политика дают другой меш (`build_model` отказывает именем `PREVIEW_KEY_MISMATCH`).
    """

    from .envelope_request_policy import normalize_envelope_fan_density

    return (
        str(record.source_name),
        str(record.source_digest),
        str(record.kernel_backend),
        str(record.skeleton_backend),
        str(record.embedding_backend),
        str(normalize_envelope_fan_density(record.density)),
        int(record.stretch_percent),
        float(record.dissolve_percent),
        float(offset),
        int(record.invalidation_count),
    )


def build_sample(results, arrays, key: tuple, alpha: float, origins: dict | None = None) -> PreviewSampleV1:
    """Образец точного прогона на `alpha` (текст ширины прогона — `str(float(alpha))`, как у `run_production`)."""

    return sample_of(results, arrays, key=key, alpha_text=str(float(alpha)), origins=origins)


def _model_or_refusal(base, ledger):
    try:
        return build_model(base, ledger)
    except Exception as exc:  # noqa: BLE001 - модель не вправе уронить точный результат: исход назван
        return PreviewRefusalV1(MODEL_BUILD_FAILED, f"{type(exc).__name__}: {exc}")


def plan_tick(grade, displayed, model, state, alpha: float, stream=None) -> IncrementalPlanV1 | None:
    """Решение тика (WIDTH_REFRESH_INCREMENTAL_V1): `None` - тик не заказывал инкрементальности (`grade` не `REFRESH_INCREMENTAL`), иначе `IncrementalPlanV1`.

    Тик инкрементальный, когда политика сверки (`verification_reason`) не требует полного такта, у образца на экране есть своя модель и результаты, которые можно сдвигать, и сертификат обслуживает не меньше
    `INCREMENTAL_MIN_SERVED_SHARE` его доменов. Обслуживает ровно те домены, что двигает кадр модели на этой ширине (`served_mask`: интервал ядра, досягаемость, не в карантине), поэтому обслуженное
    равно кадру модели; остальные домены назывной причиной попадают в `held` и считаются заново. Чистая функция значений (её зовёт поток счёта): ни `bpy`, ни записи в сессию.

    `stream` (`StreamPolicyV1`, LIVE_DRAG_STREAM_V1): `defer` - срок сверки (`DEFERRABLE_REASONS`) не делает такт полным (назван в `IncrementalPlanV1.deferred`), а `batch` обслуживаемых доменов в порядке риска
    такт считает заново ради сверки (DRAG_VERIFY_BACKGROUND_V1: в `held` под именем `VERIFY_BACKGROUND`). `None` - прежнее решение побитово.
    """

    if grade != REFRESH_INCREMENTAL:
        return None
    reason = verification_reason(state, float(alpha))
    deferred = ""
    if reason in DEFERRABLE_REASONS and stream is not None and stream.defer:
        deferred, reason = reason, ""
    if reason:
        return full_plan(reason)
    if model is None:
        return full_plan(REASON_NO_MODEL)
    if displayed is None or model.base is not displayed:
        return full_plan(REASON_MODEL_FOREIGN)
    if not displayed.origins:
        return full_plan(REASON_NO_ORIGINS)
    reason = selection_reason(len(displayed.domains))
    if reason:
        return full_plan(reason, len(displayed.domains))
    started = time.perf_counter()
    live = served_mask(model, alpha)
    origins = displayed.origins
    served: list = []
    held: dict = {}
    for number, domain in enumerate(displayed.domains):
        patch_id = domain.patch_id
        if not live[number]:
            held[patch_id] = held_name(model, number, alpha)
        elif patch_id not in origins:
            held[patch_id] = HELD_NOT_IN_BASE
        else:
            served.append(patch_id)
    domains = len(displayed.domains)
    if len(served) < INCREMENTAL_MIN_SERVED_SHARE * domains:
        return full_plan(REASON_FEW_SERVED, domains)
    if stream is not None and stream.batch > 0:
        rows = {domain.patch_id: domain for domain in displayed.domains}
        candidates = ((patch, bool(rows[patch].approximate), int(rows[patch].vertices.shape[0])) for patch in served)
        for patch_id in verify_batch(candidates, stream.verified, alpha, stream.batch):
            served.remove(patch_id)
            held[patch_id] = VERIFY_BACKGROUND
    splice = stream is not None and stream.splice
    return IncrementalPlanV1(REFRESH_INCREMENTAL, "", frozenset(served), held, domains, time.perf_counter() - started, deferred=deferred, splice=splice)


def rebase_served(plan: IncrementalPlanV1, displayed: PreviewSampleV1, alpha: float) -> dict:
    """`{патч: результат, сдвинутый сертификатом на ширину alpha}` обслуженных доменов плана (путь без составного образца: все результаты нужны `build_mesh_arrays`). Домен, который сдвинуть нечем, - названная ошибка: он уже не считался."""

    alpha_text = str(float(alpha))
    moved = {}
    for patch_id in plan.served:
        shifted, why = rebase_result(displayed.origins[patch_id], alpha_text)
        if shifted is None:
            raise RuntimeError(f"{why}: the served domain of patch {patch_id} cannot be moved by its slopes")
        moved[patch_id] = shifted
    return moved


def _composite_origins(displayed: PreviewSampleV1, results, plan: IncrementalPlanV1) -> dict:
    """Результаты, которые хранит образец инкрементального такта: у обслуженных домена - его ПРЕЖНИЙ результат (следующий такт сдвигает его, а не сдвинутую копию: ошибка округления не копится), у посчитанных - новый."""

    origins = {patch_id: displayed.origins[patch_id] for patch_id in plan.served}
    for item in results:
        if item.placement != PLACEMENT_REBASED and item.is_materialized:
            certificate = getattr(item, "parametric", None)
            if certificate is not None and not certificate.markers and certificate.vertex_count > 0:
                origins[int(item.patch_id)] = item
    return origins


def finish_live_run(run, *, alpha, offset, key, displayed, previous, trust=None, lean=False, plan=None) -> LiveRunV1:
    """Хвост потока точного счёта: массивы меша, образец и модель (всё в потоке, главный поток только применяет).

    `lean=True` (WIDTH_REFRESH_LEAN_V1; так считает точное lean-обновление): массивы без отчётов (`build_mesh_arrays(reports=False)`: ни дайджеста, ни отчётов о плоскости,
    шве и обходе рёбер, геометрия та же побитово). Модель и сверка строятся как у зафиксированного ответа: lean-обновление ставит образец на экране и кормит модель.

    Сверка предсказания прежней модели с этим прогоном (`deviation`), журнал доверия по её итогу (`advance_ledger`: опровергнутый домен — в карантин) и модель НОВОГО образца из его
    собственных аффинных сертификатов, с журналом. Других прогонов модели не нужно.

    `plan` (`plan_tick`) - решение тика. Инкрементальный такт (`plan.incremental`): часть результатов - домены, сдвинутые сертификатом, и они равны предсказанию модели по построению, так что сверка
    (`deviation`) ничего не судила бы; она не идёт, журнал доверия остаётся прежним, а образец хранит прежние результаты обслуженных доменов. Сверку делает полный lean-такт.
    """

    from .envelope_production_mesh import build_mesh_arrays

    incremental = plan is not None and plan.incremental
    if incremental:
        composite = compose_sample(displayed, previous, run.results, alpha=alpha, offset=offset, splice=plan.splice)
        if not isinstance(composite, str):
            # Образец сложен из образца на экране (обслуженные двигает модель, посчитанные ложатся поверх; COMPOSITE_SPLICE_V1 - раскладка группы переложена): массивов меша у такта нет, их держит образец.
            done = replace(plan, composite=True, spliced=composite.spliced, seconds=plan.seconds + composite.seconds)
            return LiveRunV1(run, None, composite.sample, _model_or_refusal(composite.sample, trust), None, trust, (), done)
        plan = replace(plan, fallback=composite)  # составить не вышло: путь этапа A (результаты ВСЕХ доменов: сдвинутые обслуженные и посчитанные), причина названа
        run = replace(run, results=(*run.results, *rebase_served(plan, displayed, alpha).values()))
    arrays = build_mesh_arrays(run.results, offset, reports=not lean)
    sample = build_sample(run.results, arrays, key, alpha, _composite_origins(displayed, run.results, plan) if incremental else None)
    check = None
    events: tuple = ()
    if previous is not None and not incremental:
        try:
            check = deviation(previous, sample)
            trust, events = advance_ledger(trust, previous, check)
        except Exception as exc:  # noqa: BLE001 - сверка не роняет результат
            check = DeviationV1(sample.alpha, 0.0, 0.0, 0, 0, -1, (), f"PREVIEW_CHECK_FAILED:{type(exc).__name__}")
            events = ()
    return LiveRunV1(run, arrays, sample, _model_or_refusal(sample, trust), check, trust, events, plan)


# --------------------------------------------------------------------------
# Состояние сессии (главный поток)
# --------------------------------------------------------------------------


def log_of(controller) -> PreviewLogV1:
    log = controller.width_preview_log
    if log is None:
        log = controller.width_preview_log = PreviewLogV1()
    return log


def _count_model(controller, model) -> None:
    log = log_of(controller)
    if isinstance(model, PreviewModelV1):
        log.models += 1
        log.last_model_bytes = model.own_bytes
    elif isinstance(model, PreviewRefusalV1):
        log.refuse(model.outcome)


def _check_text(check: DeviationV1) -> str:
    return (
        f"{check.domains_checked} domains at {check.relative_distance * 100:.1f}% from the base, max {check.max_position:.2e} m "
        f"({check.worst_ratio:.2f} of the limit), {len(check.refuted)} refuted, {len(check.shadow_clean)} quarantined clean"
    )


def _record_check(controller, live: LiveRunV1) -> None:
    """Результат апостериорной сверки прежней модели с этим точным прогоном и события журнала доверия — в журнал сессии.

    Отказ сверки назван, а не молчит. Опровергнутые домены прежней модели уже в карантине журнала (`live.trust`), и модель нового образца
    построена с ним: прежняя модель после этого результата не живёт.
    """

    check = live.check
    log = log_of(controller)
    for _patch, _domain, name in live.trust_events:
        log.trust_events[name] = log.trust_events.get(name, 0) + 1
    if check is None:
        return
    if check.refusal:
        log.refuse(check.refusal)
        return
    log.checks += 1
    log.domains_checked += check.domains_checked
    log.domains_refuted += len(check.refuted)
    log.max_position_error = max(log.max_position_error, check.max_position)
    log.max_uv_error = max(log.max_uv_error, check.max_uv)
    log.max_deviation_ratio = max(log.max_deviation_ratio, check.worst_ratio)
    log.last_check = _check_text(check)


def note_exact_display(controller, live: LiveRunV1) -> None:
    """Точный результат лёг в меш: образец стал экранным, модель новая, превью снято, журнал доверия сдвинут.

    Владение мешем (`capture_ownership`) записывает тот, кто писал меш, СРАЗУ после записи: здесь меша нет.
    """

    _record_check(controller, live)
    controller.width_displayed = live.sample
    controller.width_trust = live.trust
    controller.width_mesh_preview = None
    controller.width_incremental = next_state(controller.width_incremental, live.plan, live.sample.alpha)
    model = live.model
    _count_model(controller, model)
    controller.width_model = model if isinstance(model, PreviewModelV1) else None
    controller.width_model_refusal = model if isinstance(model, PreviewRefusalV1) else None


def note_button_display(controller, sample: PreviewSampleV1) -> None:
    """Кнопка записала меш: образец на экране, модели ещё нет (её строит первый кадр: `ensure_model`), журнал доверия забыт."""

    controller.width_displayed = sample
    controller.width_model = None
    controller.width_model_refusal = None
    controller.width_mesh_preview = None
    controller.width_trust = None
    controller.width_incremental = IncrementalStateV1(0, sample.alpha)  # кнопка - полный зафиксированный ответ: от него и считается сверка инкрементальных тактов


def ensure_model(controller) -> bool:
    """Модель образца на экране есть (`True`): построена тут же из его сертификатов, если её не было и отказа не называли. Главный поток, миллисекунды numpy.

    Вход инструмента после кнопки не ждёт ни одного прогона: образец с наклонами уже лежит на экране. Отказ построения назван и остаётся (`PreviewRefusalV1`), второй раз не пробуется.
    """

    if controller.width_model is not None:
        return True
    displayed = controller.width_displayed
    if displayed is None or controller.width_model_refusal is not None:
        return False
    _build_displayed_model(controller, displayed)
    return controller.width_model is not None


@gc_paused_function  # GC_PAUSED_DURING_WIDTH_TICK_V1: первая постройка модели на `cover.008` стоила 260-380 мс, потому что в неё попадал проход сборщика (установившаяся - 22 мс): пауза стоит на самой работе
def _build_displayed_model(controller, displayed: PreviewSampleV1) -> None:
    model = _model_or_refusal(displayed, controller.width_trust)
    _count_model(controller, model)
    controller.width_model = model if isinstance(model, PreviewModelV1) else None
    controller.width_model_refusal = model if isinstance(model, PreviewRefusalV1) else None


def drop_model(controller, reason: str) -> None:
    """Модель и образцы сняты с названной причиной: меш ей больше не принадлежит (другой состав, шаг истории, внешняя правка).

    Образец на экране описывает геометрию, которой в меше нет либо за которой больше нет присмотра, поэтому он снят вместе с моделью и владением:
    следующий точный результат (кнопка либо заказ ширины) заводит образец и владение заново, и затравка снова разрешена. Журнал доверия
    остаётся (он про ключ образца, а не про меш), и точный путь не затронут.
    """

    had = controller.width_model is not None or controller.width_displayed is not None
    if had:
        name = f"{MODEL_DROPPED}:{reason}"
        log = log_of(controller)
        log.dropped[name] = log.dropped.get(name, 0) + 1
        controller.width_model_refusal = PreviewRefusalV1(name)
    controller.width_model = None
    controller.width_displayed = None
    controller.width_mesh_preview = None
    controller.width_mesh_owner = None
    controller.width_incremental = None


def note_history(controller) -> None:
    """Undo/Redo/загрузка: что показывает меш, неизвестно (шаг мог лечь на геометрию превью, датаблок мог быть возвращён из истории), поэтому модель СНЯТА.

    Строго, а не «сверим на первом кадре»: после шага истории указатели, `session_uid` и раскладка не доказывают ничего. Расхождение ползунка и
    ширины меша заказывает точный пересчёт (`reconcile_after_history`), и его результат заводит образец и владение заново.
    """

    drop_model(controller, HISTORY_STEP)


# --------------------------------------------------------------------------
# Владение мешем (F2)
# --------------------------------------------------------------------------


def _read_layout(mesh, corner, start) -> bytes:
    """Отпечаток раскладки меша: индексы вершин петель и начала граней в заранее выделенные массивы, `blake2b` (16 байт)."""

    mesh.loops.foreach_get("vertex_index", corner)
    mesh.polygons.foreach_get("loop_start", start)
    digest = hashlib.blake2b(digest_size=16)
    digest.update(memoryview(corner))
    digest.update(memoryview(start))
    return digest.digest()


def capture_ownership(controller, decal, sample) -> MeshOwnershipV1 | None:
    """Точный путь записал меш декали `decal` под образец `sample`: владение (тождество, размеры, поколение, отпечаток раскладки) в сессию.

    Зовёт тот, кто писал меш, СРАЗУ после записи (кнопка и применение живой ширины: тест архитектуры). Сбой чтения называется
    (`PREVIEW_MESH_OWNERSHIP_CAPTURE_FAILED:<тип>`) и владения не оставляет: кадр тогда молчит, названный `PREVIEW_MESH_OWNERSHIP_UNKNOWN`.
    """

    controller.width_mesh_owner = None
    if decal is None or sample is None or getattr(decal, "data", None) is None:
        return None
    mesh = decal.data
    try:
        corner = np.empty(len(mesh.loops), dtype=np.int32)
        start = np.empty(len(mesh.polygons), dtype=np.int32)
        started = time.perf_counter()
        layout = _read_layout(mesh, corner, start)
        finished = time.perf_counter()
        generation = int(controller.width_layout_generation) + 1
        owner = MeshOwnershipV1(
            sample=sample,
            generation=generation,
            object_pointer=int(decal.as_pointer()),
            object_uid=int(decal.session_uid),
            mesh_pointer=int(mesh.as_pointer()),
            mesh_uid=int(mesh.session_uid),
            vertices=len(mesh.vertices),
            loops=len(mesh.loops),
            polygons=len(mesh.polygons),
            edges=len(mesh.edges),
            layout=layout,
            verified_at=finished,
            check_seconds=finished - started,
            corner_scratch=corner,
            start_scratch=start,
        )
    except Exception as exc:  # noqa: BLE001 - владение не роняет точный результат: исход назван
        log_of(controller).refuse(f"{OWNERSHIP_CAPTURE_FAILED}:{type(exc).__name__}")
        return None
    controller.width_layout_generation = generation
    controller.width_mesh_owner = owner
    log_of(controller).layout_generation = generation
    return owner


def _original_uid(ident):
    """`session_uid` ОРИГИНАЛА датаблока: `DepsgraphUpdate.id` — вычисленная копия, и у неё `session_uid` нулевой, а у оригинала — настоящий."""

    original = getattr(ident, "original", None)
    return getattr(ident if original is None else original, "session_uid", None)


def note_decal_updates(controller, updates, source=None) -> str:
    """Обработчик depsgraph: обновление ГЕОМЕТРИИ нашего объекта или меша, которое сделали не мы, снимает модель. Пусто либо причина.

    `updates` — `depsgraph.updates` (их `id` — вычисленные копии: тождество берётся у `id.original`); `source` — ключ depsgraph, который докладывает.
    Свою запись (кадр, возврат базы, точный путь) отмечает поколение (OWN_WRITE_GENERATION_V1, `note_own_write`): первый доклад КАЖДОГО depsgraph
    после неё — наш; следующий доклад того же depsgraph — внешний. Два окна или два слоя вида докладывают о нашей записи дважды, и оба доклада
    наши (булева метка второй из них читала как внешний). Если внешнее слилось с нашим в одном обновлении, сигнал его не увидит, но отпечаток
    раскладки в кадре увидит изменение раскладки (`PREVIEW_MESH_LAYOUT_CHANGED`).
    """

    owner = controller.width_mesh_owner
    if owner is None:
        return ""
    ours = (owner.object_uid, owner.mesh_uid)
    for update in updates:
        if getattr(update, "is_updated_geometry", False) and _original_uid(update.id) in ours:
            break
    else:
        return ""
    if owner.claim_update(source):
        return ""
    drop_model(controller, DECAL_CHANGED_EXTERNALLY)
    return DECAL_CHANGED_EXTERNALLY


# --------------------------------------------------------------------------
# Кадр (главный поток)
# --------------------------------------------------------------------------


def _mesh_problem(controller, base, decal) -> str:
    """Почему меш не принимает кадр образца `base` (пусто — принимает): владение, тождество, размеры, свойства точного пути и раскладка.

    Порядок — от дешёвого к дорогому: объект, режим, владение (поколение образца), тождество (указатели и `session_uid`), размеры, записанные точным
    путём ширина и ревизия, и лишь раз в `recheck_after` — отпечаток раскладки. Каждый ответ — имя исхода.
    """

    from .envelope_production_mesh import DECAL_REVISION_PROPERTY, DECAL_UV_LAYER, DECAL_WIDTH_PROPERTY

    if decal is None:
        return DECAL_GONE
    if getattr(decal, "mode", "OBJECT") == "EDIT":
        return DECAL_IN_EDIT_MODE
    mesh = decal.data
    if mesh is None:
        return MESH_NOT_THE_BASE
    owner = controller.width_mesh_owner
    if owner is None:
        return OWNERSHIP_UNKNOWN
    if owner.sample is not base:
        return LAYOUT_GENERATION_STALE
    if decal.as_pointer() != owner.object_pointer or decal.session_uid != owner.object_uid:
        return DECAL_OBJECT_REPLACED
    if mesh.as_pointer() != owner.mesh_pointer or mesh.session_uid != owner.mesh_uid:
        return MESH_DATABLOCK_REPLACED
    layer = mesh.uv_layers.get(DECAL_UV_LAYER)
    if (
        layer is None
        or len(mesh.vertices) != base.vertex_count
        or len(layer.data) != base.loop_count
        or len(mesh.loops) != owner.loops
        or len(mesh.polygons) != owner.polygons
        or len(mesh.edges) != owner.edges
    ):
        return MESH_NOT_THE_BASE
    if DECAL_WIDTH_PROPERTY not in mesh.keys() or float(mesh[DECAL_WIDTH_PROPERTY]) != base.alpha:
        return MESH_NOT_THE_BASE
    if DECAL_REVISION_PROPERTY not in decal.keys() or str(decal[DECAL_REVISION_PROPERTY]) != base.source_revision:
        return MESH_NOT_THE_BASE
    started = time.perf_counter()
    if started - owner.verified_at >= owner.recheck_after():
        layout = _read_layout(mesh, owner.corner_scratch, owner.start_scratch)
        finished = time.perf_counter()
        owner.check_seconds, owner.verified_at = finished - started, finished
        log = log_of(controller)
        log.layout_checks += 1
        log.layout_check_seconds_max = max(log.layout_check_seconds_max, owner.check_seconds)
        if layout != owner.layout:
            return MESH_LAYOUT_CHANGED
    return ""


def _decal_of(controller):
    import bpy

    from .envelope_production_mesh import find_decal_object

    record = controller.width_build
    source = None if record is None else bpy.data.objects.get(record.source_name)
    return None if source is None else find_decal_object(source)


def preview_mesh_now(controller, width: float, owners=None) -> PreviewMeshStateV1 | None:
    """Меш декали на ширине `width` из модели: кадр пишется сразу, на главном потоке. `None` — кадра нет (причина в статусе).

    Меш, который модель не вправе писать (подмена, правка, Edit, чужое поколение), снимает модель с именем причины и не получает записи.
    `owners` - номера владельцев граней точного образца, чей кадр к курсору заменяет его собственную запись (LIVE_DRAG_ONE_WRITE_V1: такт ближайшей ширины пишет меш один раз).
    """

    if not ensure_model(controller):
        return None
    model = controller.width_model
    from .envelope_production_mesh import ProductionWriteError, write_preview_geometry

    started = time.perf_counter()
    decal = _decal_of(controller)
    problem = _mesh_problem(controller, model.base, decal)
    if problem:
        drop_model(controller, problem)
        return None
    frame = evaluate(model, width)
    try:
        write_preview_geometry(decal.data, frame.positions, frame.uvs, owners)
    except ProductionWriteError as exc:
        drop_model(controller, exc.outcome)
        return None
    controller.width_mesh_owner.note_own_write()
    previous = controller.width_mesh_preview
    moving = frozenset() if frame.refusal else moving_patches(model, frame.held)
    seconds = time.perf_counter() - started
    state = PreviewMeshStateV1(
        frame.alpha,
        frame.live_domains,
        frame.held_domains,
        tuple(patch for patch, _reason in frame.held[:HELD_NAMES_SHOWN]),
        frame.outcome,
        seconds,
        1 if previous is None else previous.serial + 1,
        moving,
        frame.approximate_domains,
    )
    controller.width_mesh_preview = state
    log = log_of(controller)
    log.frames += 1
    log.frame_seconds_total += seconds
    log.frame_seconds_max = max(log.frame_seconds_max, seconds)
    return state


def moving_now(controller, width: float) -> frozenset:
    """Патчи, которые двигала бы модель на ширине `width`, БЕЗ записи в меш (вход инструмента: ещё ни одного кадра, а линии нужны только там, где меш не движется)."""

    ensure_model(controller)
    return moving_patches_at(controller.width_model, width)


def begin_drag(controller) -> None:
    """Перетаскивание началось: действуют точные lean-кадры (`width_drag_active`). К чему вернётся отмена, запоминает первый же кадр, который заменит образец на экране."""

    controller.width_drag_active = True
    controller.width_drag_origin = None


def finish_drag(controller, *, restore: bool) -> bool:
    """Перетаскивание кончилось: lean-кадры, что ещё в пути, больше не действуют. `restore` (отмена) возвращает меш на базу; `True` - было что возвращать.

    Подтверждение ничего не возвращает: меш остаётся таким, каким его оставили кадры, пока зафиксированный ответ не перепишет его целиком.
    """

    controller.width_drag_active = False
    scheduler = getattr(controller, "width_live", None)
    if scheduler is not None and hasattr(scheduler, "retire_stream"):
        # LIVE_DRAG_STREAM_V1: такт перетаскивания, что ещё в пути, больше не нужен ни отмене, ни подтверждению - снят с именем, иначе он лёг бы в СЛЕДУЮЩЕЕ перетаскивание (оно снова
        # включает `width_drag_active`) на ширине прежнего.
        scheduler.retire_stream(LEAN_DRAG_ENDED_BY_FINISH)
    try:
        return restore_base_mesh(controller) if restore else False
    finally:
        controller.width_drag_origin = None


def same_layout_of(base, sample) -> bool:
    """Раскладка меша образца `sample` та же, что у образца на экране `base` (точный кадр ляжет одной записью чисел)."""

    return base is not None and bool(base.layout_digest) and sample.layout_digest == base.layout_digest


def _lay_down(decal, sample, *, same_layout: bool, write: bool = True) -> int:
    """Точный образец `sample` - в меш декали: позиции, UV и номера владельцев одной записью, если раскладка меша та же, иначе меш пересобран на месте. Число помеченных швов (0 - не пересобирали).

    `write=False` при той же раскладке - числа не пишутся: их тут же пишет кадр модели этого образца на ширине курсора вместе с номерами владельцев (LIVE_DRAG_ONE_WRITE_V1).
    """

    from .envelope_production_mesh import record_decal_width, rebuild_mesh_from_image, write_preview_geometry

    if same_layout and not write:
        marked = 0
    elif same_layout:
        frame = base_frame(sample)
        write_preview_geometry(decal.data, frame.positions, frame.uvs, sample.image.face_owner)
        marked = 0
    else:
        marked = rebuild_mesh_from_image(decal.data, sample.positions, sample.uvs, sample.image)
    record_decal_width(decal.data, sample.alpha)
    return marked


def lay_exact_now(controller, sample) -> str:
    """Точный образец на экране - в меш его собственными числами (LIVE_DRAG_ONE_WRITE_V1: кадр к курсору не лёг, модели нет). Пусто - записано, иначе имя причины.

    Та же проверка владения, что у кадра и lean-кадра (`_mesh_problem`): чужой меш не получает записи, модель снимается с названной причиной.
    """

    from .envelope_production_mesh import ProductionWriteError

    decal = _decal_of(controller)
    problem = _mesh_problem(controller, sample, decal)
    if problem:
        drop_model(controller, problem)
        return problem
    try:
        _lay_down(decal, sample, same_layout=True)
    except ProductionWriteError as exc:
        drop_model(controller, exc.outcome)
        return exc.outcome
    capture_ownership(controller, decal, sample)
    return ""


def lean_exact_now(controller, live, *, reframe_to: float | None = None) -> str:
    """Точный lean-кадр (WIDTH_REFRESH_LEAN_V1) во время перетаскивания: точный прогон `live` ложится в меш и становится образцом на экране. Пусто - сделано, иначе - имя причины.

    Это то же, что делает применение зафиксированного ответа (`note_exact_display`: образец на экране, модель нового образца, сверка и журнал доверия,
    владение мешем снято заново), БЕЗ работы, нужной только фиксации: ни квитанции, ни дайджеста меша, ни консоли, ни кэшей. Меш получает ровно то, что записал бы полный путь:
    раскладка та же (равный `layout_digest`) - позиции, UV и номера владельцев граней одной записью, ширина записи - ширина кадра; сменилась (домен сменил структуру) - меш пересобран
    на месте по образу (`rebuild_mesh_from_image`). Проверка владения (`_mesh_problem`) та же, что у кадра модели: чужой меш не получает записи, а модель снимается с названной
    причиной. Отмена инструмента возвращает то, что показывали перед первым кадром (`DragOriginV1`, `restore_base_mesh`), побитово.

    `reframe_to` (LIVE_DRAG_ONE_WRITE_V1): курсор ушёл дальше ширины такта - при той же раскладке числа образца в меш не пишутся, их сразу пишет кадр модели этого образца на ширине
    курсора (`envelope_width_live._apply_lean`): такт ближайшей ширины пишет меш один раз.
    """

    base, sample = controller.width_displayed, live.sample
    log = log_of(controller)
    if base is None:
        log.refuse(LEAN_NO_BASE)
        return LEAN_NO_BASE
    if sample.key != base.key:
        log.refuse(LEAN_KEY_MISMATCH)
        return LEAN_KEY_MISMATCH
    from .envelope_production_mesh import ProductionWriteError

    started = time.perf_counter()
    decal = _decal_of(controller)
    problem = _mesh_problem(controller, base, decal)
    if problem:
        drop_model(controller, problem)
        return problem
    same_layout = same_layout_of(base, sample)
    if controller.width_drag_origin is None and controller.width_drag_active:
        controller.width_drag_origin = DragOriginV1(
            base, controller.width_model, controller.width_model_refusal, controller.width_trust, controller.width_incremental
        )
    try:
        _lay_down(decal, sample, same_layout=same_layout, write=reframe_to is None)
    except ProductionWriteError as exc:
        drop_model(controller, exc.outcome)
        return exc.outcome
    note_exact_display(controller, live)
    capture_ownership(controller, decal, sample)
    previous = controller.width_mesh_preview
    seconds = time.perf_counter() - started
    plan = live.plan
    if plan is not None and plan.incremental:
        recomputed = tuple(sorted(item.patch_id for item in live.run.results if item.placement != PLACEMENT_REBASED))
        controller.width_mesh_preview = PreviewMeshStateV1(
            sample.alpha, len(plan.served), len(recomputed), recomputed[:HELD_NAMES_SHOWN], REFRESH_INCREMENTAL, seconds, 1 if previous is None else previous.serial + 1
        )
        log.incremental_ticks += 1
        log.incremental_served += len(plan.served)
        log.incremental_recomputed += len(recomputed)
        log.incremental_plan_seconds_total += plan.seconds
        if plan.composite:
            log.incremental_composite += 1
        else:
            log.incremental_fallback[plan.fallback] = log.incremental_fallback.get(plan.fallback, 0) + 1
    else:
        controller.width_mesh_preview = PreviewMeshStateV1(
            sample.alpha, len(sample.domains), 0, (), REFRESH_LEAN, seconds, 1 if previous is None else previous.serial + 1
        )
        if plan is not None:
            log.incremental_full[plan.reason] = log.incremental_full.get(plan.reason, 0) + 1
    log.lean_frames += 1
    log.lean_rebuilds += 0 if same_layout else 1
    log.lean_frame_seconds_total += seconds
    return ""


def _restore_origin(controller, origin: DragOriginV1) -> bool:
    """Отмена после lean-кадров: меш и состояние сессии - такие, какими были перед первым из них (кадры заменили образец на экране)."""

    from .envelope_production_mesh import ProductionWriteError

    current, base = controller.width_displayed, origin.displayed
    if current is None or base is None:
        return False
    decal = _decal_of(controller)
    problem = _mesh_problem(controller, current, decal)
    if problem:
        drop_model(controller, problem)
        return False
    try:
        _lay_down(decal, base, same_layout=bool(current.layout_digest) and current.layout_digest == base.layout_digest)
    except ProductionWriteError as exc:
        drop_model(controller, exc.outcome)
        return False
    controller.width_displayed = base
    controller.width_model, controller.width_model_refusal, controller.width_trust = origin.model, origin.model_refusal, origin.trust
    controller.width_incremental = origin.incremental
    controller.width_mesh_preview = None
    capture_ownership(controller, decal, base)
    log_of(controller).lean_origin_restores += 1
    return True


def restore_base_mesh(controller) -> bool:
    """Меш возвращён на геометрию базы (отмена инструмента): побитово то, что записал точный путь. `True` — было что возвращать.

    Меш, который модель не вправе писать, не получает записи, а модель снимается с именем причины (как в кадре). Если точные lean-кадры успели заменить образец на экране,
    возвращается то, что показывали перед первым из них (`_restore_origin`).
    """

    origin = controller.width_drag_origin
    if origin is not None:
        return _restore_origin(controller, origin)
    if controller.width_mesh_preview is None:
        return False
    model = controller.width_model
    controller.width_mesh_preview = None
    if model is None:
        return False
    from .envelope_production_mesh import ProductionWriteError, write_preview_geometry

    decal = _decal_of(controller)
    problem = _mesh_problem(controller, model.base, decal)
    if problem:
        drop_model(controller, problem)
        return False
    frame = base_frame(model.base)
    try:
        write_preview_geometry(decal.data, frame.positions, frame.uvs)
    except ProductionWriteError:
        return False
    controller.width_mesh_owner.note_own_write()
    return True


# --------------------------------------------------------------------------
# Строки статуса
# --------------------------------------------------------------------------


def status_lines(controller) -> tuple[str, ...]:
    lines = []
    state = controller.width_mesh_preview
    if state is not None and state.outcome == REFRESH_INCREMENTAL:
        shown = ", ".join(str(item) for item in state.held_patches)
        more = "" if state.held <= len(state.held_patches) else ", ..."
        lines.append(
            f"Mesh {REFRESH_INCREMENTAL}: width {state.alpha:.4g}, {state.live} domains moved by the affine certificate (structure verified afterwards), {state.held} recomputed exactly (patch {shown}{more}); "
            "the structure of the moved ones is checked by the next full tick (confirming recomputes and writes the committed result)"
        )
    elif state is not None and state.outcome == REFRESH_LEAN:
        lines.append(
            f"Mesh {REFRESH_LEAN}: exact geometry at width {state.alpha:.4g}, {state.live} domains (without the checks and reports of a commit; "
            "confirming recomputes and writes the committed result)"
        )
    elif state is not None:
        held = ""
        if state.held:
            shown = ", ".join(str(item) for item in state.held_patches)
            more = "" if state.held <= len(state.held_patches) else ", ..."
            held = f", {state.held} held at the last exact geometry (patch {shown}{more})"
        refusal = "" if state.outcome == PREVIEW_MESH_FROM_AFFINE_CERTIFICATE_V1 else f" REFUSED {state.outcome}:"
        near = ""
        if state.approximate:
            near = (
                f" ({state.live - state.approximate} exact affine, {state.approximate} display-approximate {DOMAIN_DISPLAY_APPROXIMATE}: "
                f"exact slopes, offset normals frozen at the last exact width, error limit {DISPLAY_APPROXIMATE_ERROR_LIMIT * 1000:g} mm)"
            )
        lines.append(
            f"Mesh {PREVIEW_MESH_FROM_AFFINE_CERTIFICATE_V1} preview (exact affine certificate; structure verified afterwards):{refusal} width {state.alpha:.4g}, "
            f"{state.live} domains moving{near}{held}; the exact result follows"
        )
    model = controller.width_model
    if model is not None:
        reasons = model.reason_counts()
        extra = "" if not reasons else " (" + ", ".join(f"{name} {count}" for name, count in sorted(reasons.items())) + ")"
        approximate = model.approximate_domains
        split = "" if not approximate else f" ({model.modelled_domains - approximate} exact, {approximate} display-approximate {DOMAIN_DISPLAY_APPROXIMATE})"
        lines.append(
            f"Preview model (affine certificate of one exact run, valid inside the certified event interval): {model.modelled_domains}/{model.domain_count} domains "
            f"modelled{split}, {model.own_bytes / 1024:.0f} KB{extra}"
        )
    elif controller.width_model_refusal is not None:
        refusal = controller.width_model_refusal
        lines.append(f"Preview model: none ({refusal.outcome}) - the line overlay only")
    ledger = controller.width_trust
    if ledger is not None and ledger.entries:
        counts = ledger.counts()
        lines.append("Preview trust: " + ", ".join(f"{count} {name.lower()}" for name, count in sorted(counts.items())))
    log = controller.width_preview_log
    if log is not None and log.last_check:
        lines.append(f"Last exact check: {log.last_check}")
    if log is not None and (log.incremental_ticks or log.incremental_full):
        full = ", ".join(f"{name} {count}" for name, count in sorted(log.incremental_full.items()))
        lines.append(
            f"Incremental ticks {REFRESH_INCREMENTAL}: {log.incremental_ticks} incremental ({log.incremental_served} domain-ticks moved by the affine certificate (structure verified afterwards), {log.incremental_recomputed} recomputed)"
            + (f"; full lean ticks: {full}" if full else "")
        )
    return tuple(lines)


__all__ = (
    "DECAL_CHANGED_EXTERNALLY",
    "DECAL_GONE",
    "DECAL_IN_EDIT_MODE",
    "DECAL_OBJECT_REPLACED",
    "HISTORY_STEP",
    "LAYOUT_GENERATION_STALE",
    "LAYOUT_RECHECK_COST_RATIO",
    "LAYOUT_RECHECK_SECONDS",
    "LiveRunV1",
    "MESH_DATABLOCK_REPLACED",
    "MESH_LAYOUT_CHANGED",
    "MESH_NOT_THE_BASE",
    "MODEL_BUILD_FAILED",
    "MODEL_DROPPED",
    "DragOriginV1",
    "MeshOwnershipV1",
    "OWNERSHIP_CAPTURE_FAILED",
    "OWNERSHIP_UNKNOWN",
    "PreviewLogV1",
    "PreviewMeshStateV1",
    "build_sample",
    "capture_ownership",
    "LEAN_KEY_MISMATCH",
    "LEAN_DRAG_ENDED_BY_FINISH",
    "LEAN_NO_BASE",
    "begin_drag",
    "drop_model",
    "ensure_model",
    "finish_drag",
    "finish_live_run",
    "lay_exact_now",
    "lean_exact_now",
    "same_layout_of",
    "log_of",
    "moving_now",
    "note_button_display",
    "note_decal_updates",
    "note_exact_display",
    "note_history",
    "plan_tick",
    "preview_mesh_now",
    "restore_base_mesh",
    "sample_key",
    "status_lines",
)
