"""Живая ширина декали: точный пересчёт продуктового меша в фоне и мгновенное превью до него.

ЧТО ДЕЛАЕТ ШИРИНА. Ширина — это `envelope_debug_alpha` (тот же ползунок, что в отладочной панели; в
продуктовой секции он показан как «Decal width»). После «Build Decal Mesh» её изменение заказывает ДВЕ
вещи, обе на главном потоке дёшево:

1. ПРЕВЬЮ (`envelope_width_preview`, `PREVIEW_BINARY64_V1`): линии отступа на новой ширине считаются тут же,
   в калбэке (чистая функция, единицы миллисекунд), и рисуются поверх вьюпорта (`envelope_width_overlay`).
   Оно названо превью и не пишется в меш.
2. ТОЧНЫЙ РЕЗУЛЬТАТ: тот же планировщик, что у превью alpha (`envelope_alpha_preview.AlphaPreviewScheduler`:
   пауза, слияние, один полёт, устаревшее не применяется), считает продуктовый прогон в потоке и пишет меш.

ЧТО ГДЕ ИСПОЛНЯЕТСЯ (как у `envelope_alpha_preview_gp`). Калбэк (`schedule_width_live`) читает настройки,
считает превью и записывает заказ. `_begin` (главный поток) проверяет цель, берёт живой пул
(`peek_domain_pool`: воркеров посреди перетаскивания не стартуем) и отдаёт потоку замкнутую функцию
`compute`. `compute` (ПОТОК) — `run_production` на готовых кэшах сессии с заказом остановки `cancel`: ни `bpy`,
ни настроек, ни объектов. `_validity` и `_apply` (главный поток) проверяют цель и пишут меш.

ТОТ ЖЕ ОТВЕТ, ЧТО У КНОПКИ. Счёт — тот же `run_production` с теми же выделением, плотностью и допуском, что
сохранила кнопка (`LastProductionBuildV1`); запись — `rewrite_decal_mesh`, общий с `write_decal_object` код
(`_fill_mesh`, `build_mesh_arrays`). Равенство побитово держит смок (`mesh_content_digest` живого результата и
прямого нажатия на последней ширине, в тёплой и холодной сессии), а не слово.

ОТМЕНА (UNDO) — ВЫБРАНО И ЗАПИСАНО. Таймер шага отмены НЕ кладёт: применение переписывает геометрию СУЩЕСТВУЮЩЕГО
меша на месте (`rewrite_decal_mesh`: ни `meshes.new`/`remove`, ни `materials.new`), то есть не повторяет причину
падения из `envelope_production_operator.UNDO_REQUIRED_REASON` (создание и освобождение датаблоков без шага).
Шаг у изменения ширины есть: его кладёт модальный инструмент при подтверждении (один шаг на всё
перетаскивание) либо интерфейс при отпускании ползунка. Цена, названная прямо: этот шаг записан ДО точного
результата, поэтому Redo возвращает значение ширины при меше прежней ширины. Расхождение ловит
`reconcile_after_history` (после Undo/Redo сравнивает `cftuv_decal_width` меша с ползунком) и заказывает
точный пересчёт заново; превью при Undo/Redo снимается.

ЦЕЛЬ — СОБСТВЕННАЯ ДЕКАЛЬ АКТИВНОГО ОБЪЕКТА (`availability_problem`). Запись кнопки одна на окно (последний «Build
Decal Mesh»), а активный объект меняется. Инструмент, кнопка панели, поле «Decal width» и путь калбэка ползунка
доступны только тогда, когда у АКТИВНОГО объекта есть своя построенная декаль (`<источник>.CFTUV_Decal`, найденная
по метке и записанному источнику, как при пересборке) и запись кнопки этого окна — про него и свежая (сессия не
сброшена). Иначе — причина строкой «Build Decal Mesh first for <объект>» (подсказка отключённой кнопки через
`poll_message_set` и строка панели), ни заказа, ни линий превью. Смена активного объекта (обработчик depsgraph,
`follow_active_object`) снимает превью и подтягивает поле к ширине меша нового объекта (`sync_width_field`); заказы,
уже принятые планировщиком для прежнего объекта, не отбрасываются молча: они про ЕГО декаль и завершаются либо
получают названный исход (`BUILD_REPLACED`, `DECAL_GONE`). Удалённая декаль снова делает инструмент недоступным.

НИЧЕГО НЕ ПРОПАДАЕТ МОЛЧА: нет сборки, смена плотности/допуска, правка меша, исчезнувший объект — строки статуса
и счётчики планировщика (`status_lines`); ошибка потока — консоль и статус.

ПРЕВЬЮ МЕША (`envelope_width_mesh_preview`, `PREVIEW_MESH_FROM_AFFINE_CERTIFICATE_V1`). Между точными
пересчётами калбэк ползунка и модальный инструмент двигают НАСТОЯЩИЙ меш декали из модели (`preview_mesh_now`), а линии превью остаются
только там, где меш не движется (`width_event_preview`, `PREVIEW_CONTOUR_ONLY_WHERE_MODEL_HELD_V1`: придержанные, карантинные и не вошедшие
в меш патчи; без модели — линии всех путей, как были). Модель строится из АФФИННОГО СЕРТИФИКАТА одного точного прогона (ядро выпускает его вместе с результатом: наклон по ширине каждой вершины и петли UV):
точный результат (хвост `compute`: массивы меша, образец, модель, сверка с прежней моделью и журнал доверия) заводит её сам, а образец кнопки получает модель при первом кадре (`ensure_model`).
Затравочных прогонов нет: вход в инструмент ничего не ждёт. Меш принадлежит модели, только пока его владение (`capture_ownership`) цело: точная запись фиксирует
его тут же после себя (`_apply`), а любое внешнее изменение (история, правка геометрии декали мимо нас — `note_depsgraph`) снимает модель
с названной причиной, и точный путь продолжает работать без превью меша.

КЛАССЫ ТОЧНОГО ОБНОВЛЕНИЯ (WIDTH_REFRESH_LEAN_V1, `WidthLiveTargetV1.grade`). `COMMIT` - полный путь (проверки, дайджесты, отчёты, кэши, квитанция, консоль): так считают подтверждение
инструмента, ползунок и кнопка, и это зафиксированный ответ. `WIDTH_REFRESH_LEAN_V1` - то же обновление БЕЗ работы, нужной только фиксации (воркер не пакует батч, массивы меша без
дайджеста и отчётов, результат не ложится ни в какой кэш): геометрия, исходы доменов и счётчики те же побитово. Lean — у точных кадров во
время перетаскивания (`request_drag_tick`, настройка «Exact ticks while dragging», по умолчанию выключена): такой кадр заменяет меш и образец на экране (`lean_exact_now`), кормит модель,
а подтверждение пересчитывает и записывает зафиксированный ответ, отмена возвращает меш и образцы, какими они были перед первым кадром.

ИНКРЕМЕНТАЛЬНЫЙ ТАКТ (WIDTH_REFRESH_INCREMENTAL_V1, `envelope_width_incremental`). Такты перетаскивания (`request_drag_tick`) заказываются как инкрементальные: lean-такт, который считает заново только домены, чей аффинный сертификат ширину не
обслуживает (интервал ядра кончился, карантин, нет сертификата), а остальные не считает вовсе: образец такта складывается из образца на экране (`envelope_width_composite`: обслуженные двигает кадр модели, посчитанные ложатся поверх точными;
не сложился - название причины и путь из сдвинутых результатов всех доменов, `envelope_width_rebase`). Структура обслуженных доменов не заверена, поэтому политика сверки (путь ширины от сверки, число тактов, мало обслуженных) делает такт ПОЛНЫМ
lean-тактом, а подтверждение всегда COMMIT; сверку, которую рука прервала, `_begin` отмечает, и тики не запираются. `CFTUV_INCREMENTAL=0` возвращает прежние lean-такты.

ЖИВОЙ ТОЧНЫЙ МЕШ (LIVE_DRAG_STREAM_V1, `envelope_width_stream`, настройка «Live exact mesh while dragging», по умолчанию выключена). Такты того же класса заказываются ПОТОКОМ: без паузы, полёт не
отменяется, последнее значение побеждает, такт ближайшей ширины ложится и меш сдвигается к курсору кадром новой модели. Срок сверки не делает такт полным: сверка идёт фоном партиями доменов внутри
тактов (DRAG_VERIFY_BACKGROUND_V1), а когда рука стоит, простой планировщика заказывает такты сверки на ширине курсора; рука их не отменяет. Подтверждение - тот же COMMIT, отмена - тот же возврат.
Выключенная настройка не меняет ничего.

ПАУЗА СБОРЩИКА (GC_PAUSED_DURING_WIDTH_TICK_V1, `envelope_gc_pause`). Циклический сборщик родителя стоит на паузе ровно на работе тика, а не на простое между ними: счёт в потоке (`compute`:
тот же для точного заказа и подтверждения), применение на главном потоке (`_apply`) и подтверждение инструмента (`envelope_width_session.finish_adjust`). Сборщик общий
на процесс, поэтому пока поток внутри `compute`, он выключен и для главного потока (тот в это время опрашивает таймер; тик длится секунды), а когда `compute` вернулся, включён: и пауза
планировщика, и опрос, и рука на месте идут при включённом. Блоки потоков СЧИТАЮТСЯ (см. `envelope_gc_pause`), поэтому кнопка, вошедшая, пока счёт ещё летит, не получает сборщик обратно от счёта.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace

from .envelope_alpha_preview import (
    AlphaPreviewScheduler,
    PreviewCancelled,
    PreviewUnavailable,
    ThreadedPreviewJob,
)
from .envelope_gc_pause import gc_paused, gc_paused_function
from .envelope_width_incremental import REASON_PERIOD_DISTANCE, REASON_PERIOD_TICKS, StreamPolicyV1, mark_attempt, verification_reason
from .envelope_width_stream import STEP_NO_SAMPLE, LiveDragLogV1, idle_answer, note_applied, note_step, seed_verified
from .envelope_width_stream import status_line as live_drag_status_line
from .envelope_kernel_backend import DEFAULT_KERNEL_BACKEND, resolve_density_backend, settle_stage_orders
from .envelope_production_result import REFRESH_COMMIT, REFRESH_INCREMENTAL, REFRESH_LEAN
from .envelope_width_mesh_preview import (
    capture_ownership,
    ensure_model,
    finish_live_run,
    lean_exact_now,
    log_of,
    note_decal_updates,
    note_exact_display,
    note_history,
    plan_tick,
    preview_mesh_now,
    sample_key,
)
from .envelope_width_mesh_preview import status_lines as mesh_preview_status_lines
from .envelope_width_preview import (
    PREVIEW_BINARY64_V1,
    PendingPreviewInputsV1,
    WidthPreviewV1,
    compute_width_preview,
    prepare_preview_inputs,
)

NO_BUILD = "no decal built yet: press Build Decal Mesh"
SOURCE_CHANGED = "source mesh changed since Build Decal Mesh: press Build Decal Mesh"
SOURCE_GONE = "source object is gone"
DECAL_GONE = "decal object is gone: press Build Decal Mesh"
BUILD_REPLACED = "Build Decal Mesh ran while computing"
POLICY_CHANGED = "{name} changed since Build Decal Mesh: press Build Decal Mesh"
NO_ACTIVE = "select a mesh object: the width tool adjusts the decal of the active object"
NEED_BUILD = "Build Decal Mesh first for {name}"
NO_PREVIEW = "the last Build Decal Mesh has no chain to draw"
#: Точное lean-обновление (WIDTH_REFRESH_LEAN_V1) досчиталось, когда перетаскивание уже кончилось: подтверждение заказало зафиксированный ответ, и lean-кадр ему не нужен.
LEAN_DRAG_ENDED = "WIDTH_REFRESH_LEAN_DRAG_ENDED"
#: Классы обновления, которые заказывает перетаскивание (не зафиксированный ответ): их применяет `_apply_lean`. Инкрементальный (WIDTH_REFRESH_INCREMENTAL_V1) - заказ lean-такта, который политика сверки
#: (`envelope_width_incremental.verification_reason`) сама делает полным, когда сверка пора; в обоих случаях результат - lean-такт, а полный ответ даёт подтверждение.
DRAG_GRADES = (REFRESH_LEAN, REFRESH_INCREMENTAL)


@dataclass(frozen=True, slots=True)
class LastProductionBuildV1:
    """Что последняя кнопка «Build Decal Mesh» знала и построила: основа точного пересчёта и превью."""

    source_name: str
    source_object_key: object
    source_data_key: object
    source_digest: str
    analysis_bundle: object
    selected: frozenset
    density: object
    stretch_percent: int
    invalidation_count: int
    #: `PreviewInputsV1` либо `PendingPreviewInputsV1` (WIDTH_PREVIEW_INPUTS_DEFERRED_V1: кнопка готовит грани и стороны, пути строит простой после неё либо первое чтение): читать через `resolved_preview_inputs`.
    preview_inputs: object
    #: Ширина (alpha), с которой кнопка записала меш.
    width: float
    #: «Dissolve UV tolerance» (проценты ширины), с которой кнопка записала меш.
    dissolve_percent: float = 0.390625
    #: Главный переключатель ядра кнопки: живая ширина считает тем же (смена настройки действует с ближайшей кнопки).
    kernel_backend: str = DEFAULT_KERNEL_BACKEND
    #: Порядки стадий кнопки (`None` — как `kernel_backend`, `__post_init__` приводит их к именам): живая ширина считает ТЕМИ ЖЕ порядками и берёт подготовки из кэша сессии
    #: под ключом стадии скелета (подготовки Python и Rust не смешиваются).
    skeleton_backend: str | None = None
    embedding_backend: str | None = None
    density_backend: str | None = field(default=None, kw_only=True)

    def __post_init__(self) -> None:
        settle_stage_orders(self, "kernel_backend")
        object.__setattr__(self, "density_backend", resolve_density_backend(self.kernel_backend, self.density_backend))


@dataclass(frozen=True, slots=True)
class WidthLiveTargetV1:
    """Что нужно цели из настроек на момент заказа (значения, не ссылки на RNA)."""

    source_name: str
    workers: int
    density: object
    stretch_percent: int
    offset: float
    material_name: str
    dissolve_percent: float = 0.390625
    #: Класс обновления (WIDTH_REFRESH_LEAN_V1): `COMMIT` - полный путь, зафиксированный ответ (умолчание: подтверждение инструмента, ползунок, кнопка);
    #: `WIDTH_REFRESH_LEAN_V1` - без работы, нужной только фиксации.
    grade: str = REFRESH_COMMIT
    #: LIVE_DRAG_STREAM_V1 (`envelope_width_stream`): потоковый такт живого точного меша (срок сверки не делает его полным, сверка идёт фоном: DRAG_VERIFY_BACKGROUND_V1).
    stream: bool = False


@dataclass(frozen=True, slots=True)
class DecalProbeV1:
    """Что записала сборка на объекте декали: имя, источник (`cftuv_source_object`) и ревизия (`cftuv_source_revision`)."""

    object_name: str
    source_name: str
    revision: str


@dataclass(frozen=True, slots=True)
class WidthPreviewStateV1:
    """Текущее мгновенное превью: чей это источник, линии и порядковый номер (кэш батча отрисовки)."""

    source_name: str
    preview: WidthPreviewV1
    serial: int


@dataclass(frozen=True, slots=True)
class _JobContextV1:
    record: LastProductionBuildV1
    pooled: bool


# --------------------------------------------------------------------------
# Запись кнопки и превью
# --------------------------------------------------------------------------


def remember_build(
    controller,
    source_name: str,
    bundle,
    run,
    *,
    source_object_key,
    source_data_key,
    selected,
    density,
    stretch_percent: int,
    width: float,
    dissolve_percent: float = 0.390625,
    kernel_backend: str = DEFAULT_KERNEL_BACKEND,
    skeleton_backend: str | None = None,
    embedding_backend: str | None = None,
    density_backend: str | None = None,
) -> LastProductionBuildV1:
    """Кнопка отработала: запись для живой ширины. Старое превью снимается (оно про прежний прогон)."""

    record = LastProductionBuildV1(
        source_name=str(source_name),
        source_object_key=source_object_key,
        source_data_key=source_data_key,
        source_digest=bundle.source_revision.digest,
        analysis_bundle=bundle,
        selected=frozenset(int(item) for item in selected),
        density=density,
        stretch_percent=int(stretch_percent),
        invalidation_count=controller.invalidation_count,
        preview_inputs=prepare_preview_inputs(bundle.patch_surface, run.selected_by_patch),
        width=float(width),
        dissolve_percent=float(dissolve_percent),
        kernel_backend=str(kernel_backend),
        skeleton_backend=None if skeleton_backend is None else str(skeleton_backend),
        embedding_backend=None if embedding_backend is None else str(embedding_backend),
        density_backend=run.density_backend if density_backend is None else density_backend,
    )
    controller.width_build = record
    controller.width_target = record.source_name
    set_preview(controller, None)
    return record


# WIDTH_PREVIEW_INPUTS_DEFERRED_V1. Кнопка готовит грани и стороны линий превью (`prepare_preview_inputs`), а пути и углы (`PendingPreviewInputsV1.complete`, 73 % цены) строит простой главного потока
# сразу после нажатия (`schedule_idle_preparation`) либо то, что прочтёт их раньше (событие ползунка, ось инструмента): тот же вызов, те же значения побитово. Кто построил, назван в журнале превью (`inputs_built`).
INPUTS_BUILT_IDLE = "idle"
INPUTS_BUILT_EVENT = "event"
INPUTS_BUILD_FAILED = "WIDTH_PREVIEW_INPUTS_BUILD_FAILED"
#: Исходы `run_idle_preparation`.
IDLE_BUILT = "WIDTH_PREVIEW_INPUTS_IDLE_BUILT"
IDLE_ALREADY_BUILT = "WIDTH_PREVIEW_INPUTS_IDLE_ALREADY_BUILT"
IDLE_SUPERSEDED = "WIDTH_PREVIEW_INPUTS_IDLE_SUPERSEDED"
IDLE_NOT_DEFERRED = "WIDTH_PREVIEW_INPUTS_IDLE_NOT_DEFERRED"
#: Через сколько секунд после нажатия простой считает пути: перерисовка окон после оператора успевает раньше, а рука, которая начала тянуть раньше, платит за сборку сама (событие строит синхронно).
IDLE_PREPARE_SECONDS = 0.1


def resolved_preview_inputs(controller, record, built_by: str = INPUTS_BUILT_EVENT):
    """`PreviewInputsV1` записи кнопки: готовые - как есть, отложенные - построены здесь (один раз), и первое построение записано в журнал на `built_by`.

    Построение, которое не вышло, называется (`INPUTS_BUILD_FAILED`) и значит то же, что прежде значил сбой записи на кнопке: запись снята («не запомнено»), ширина не живая до следующего нажатия; результат `None`.
    """

    inputs = record.preview_inputs
    if type(inputs) is not PendingPreviewInputsV1:
        return inputs
    try:
        value = inputs.complete(built_by)
    except Exception as exc:  # noqa: BLE001 - сбой построения называется и снимает запись, как прежде снимал сбой `remember_build`
        print(f"[CFTUV][WidthLive] {INPUTS_BUILD_FAILED}: the build was not remembered: {type(exc).__name__}: {exc}", flush=True)
        if controller is not None and controller.width_build is record:
            controller.width_build = None
        return None
    builder = inputs.take_builder()
    if builder and controller is not None:
        counts = log_of(controller).inputs_built
        counts[builder] = counts.get(builder, 0) + 1
    return value


def has_preview_runs(inputs) -> bool:
    """Есть ли что рисовать: путь есть ровно тогда, когда есть хоть одна сторона, и отложенные входы отвечают без построения путей."""

    return bool(inputs.has_runs) if type(inputs) is PendingPreviewInputsV1 else bool(inputs.runs)


def run_idle_preparation(controller, record=None) -> str:
    """Простой после нажатия: пути линий превью записи кнопки построены (главный поток). Названный исход; запись, которую успели заменить или снять, не трогается."""

    record = controller.width_build if record is None else record
    if record is None or controller.width_build is not record:
        return IDLE_SUPERSEDED
    inputs = record.preview_inputs
    if type(inputs) is not PendingPreviewInputsV1:
        return IDLE_NOT_DEFERRED
    if inputs.built:
        return IDLE_ALREADY_BUILT
    with gc_paused(freeze=True):  # как кнопка (GC_FROZEN_AFTER_BUTTON_V1): тысячи долгоживущих объектов путей не обходят сборки поколений, а первый кадр после простоя не платит за их обход
        resolved_preview_inputs(controller, record, INPUTS_BUILT_IDLE)
    return IDLE_BUILT


def schedule_idle_preparation(controller, record) -> None:
    """Таймер главного потока (`bpy.app.timers`): простой сборки путей после кнопки. Сбой простого называется и снимает запись так же, как сбой события."""

    import bpy

    def prepare():
        try:
            run_idle_preparation(controller, record)
        except Exception as exc:  # noqa: BLE001 - таймер не роняет цикл Blender: исход назван
            print(f"[CFTUV][WidthLive] {INPUTS_BUILD_FAILED}: the idle preparation failed: {type(exc).__name__}: {exc}", flush=True)
        return None

    bpy.app.timers.register(prepare, first_interval=IDLE_PREPARE_SECONDS)


def tag_view3d_redraw() -> None:
    """Строки статуса и оверлей — не свойства RNA: перерисовка вьюпортов по явному заказу."""

    try:
        import bpy

        manager = bpy.context.window_manager
        for window in manager.windows:
            for area in window.screen.areas:
                if area.type == "VIEW_3D":
                    area.tag_redraw()
    except (AttributeError, ImportError, RuntimeError):
        return  # без главного цикла (фоновый Blender, тесты) перерисовывать нечего


def set_preview(controller, preview: WidthPreviewV1 | None, source_name: str = "") -> None:
    """Превью в состояние контроллера и заказ перерисовки; `None` снимает оверлей."""

    if preview is None:
        controller.width_preview = None
    else:
        previous = controller.width_preview
        serial = 1 if previous is None else previous.serial + 1
        controller.width_preview = WidthPreviewStateV1(source_name, preview, serial)
    tag_view3d_redraw()


def preview_now(controller, width: float, offset: float, skip_patches: frozenset = frozenset()) -> WidthPreviewV1 | None:
    """Мгновенное превью на `width` по записи кнопки; кладёт его в состояние. `None` — превью нет.

    `skip_patches` — патчи, которые уже двигает настоящий меш: их линии не считаются (`PREVIEW_CONTOUR_ONLY_WHERE_MODEL_HELD_V1`).
    """

    record = controller.width_build
    if record is None or record.preview_inputs is None:
        return None
    inputs = resolved_preview_inputs(controller, record)
    if inputs is None:
        return None
    preview = compute_width_preview(inputs, width, lift=offset, skip_patches=skip_patches)
    set_preview(controller, preview, record.source_name)
    return preview


def width_event_preview(controller, width: float, offset: float):
    """Превью одного события (калбэк ползунка и перетаскивание): кадр настоящего меша, а линии — только там, где меш не движется.

    PREVIEW_CONTOUR_ONLY_WHERE_MODEL_HELD_V1. Пока модель двигает меш (`preview_mesh_now`), линии каждого события считались зря: на `building`
    4.9 мс, на `cover.008` 34 мс (до 480) против 0.6 и 3.1 мс кадра меша, а видны они поверх самого меша. Теперь линии считаются только для путей патчей,
    которых кадр НЕ двигает целиком: придержанных (интервал ядра кончился либо за досягаемостью), в карантине журнала доверия и тех, что не вошли
    в меш. Нет модели либо кадр отказан (имя в статусе) — линии всех путей, как прежде. Число пропущенных путей названо в статусе линий
    (`WidthPreviewV1.outcomes`). Возвращает состояние кадра меша либо `None` (кадра нет).
    """

    state = preview_mesh_now(controller, width)
    preview_now(controller, width, offset, frozenset() if state is None else state.moving)
    return state


# --------------------------------------------------------------------------
# Проверки цели
# --------------------------------------------------------------------------


def _normalized_density(value):
    from .envelope_request_policy import normalize_envelope_fan_density

    return normalize_envelope_fan_density(value)


def target_problem(controller, target: WidthLiveTargetV1) -> str:
    """Почему точного пересчёта не будет (пусто — будет): запись кнопки и настройки, что с ней расходятся."""

    record = controller.width_build
    if record is None:
        return NO_BUILD
    if controller.invalidation_count != record.invalidation_count:
        return SOURCE_CHANGED
    if _normalized_density(target.density) != _normalized_density(record.density):
        return POLICY_CHANGED.format(name="Fan Density")
    if int(target.stretch_percent) != record.stretch_percent:
        return POLICY_CHANGED.format(name="Max stretch")
    if float(target.dissolve_percent) != record.dissolve_percent:
        return POLICY_CHANGED.format(name="Dissolve UV tolerance")
    return ""


# --------------------------------------------------------------------------
# Цель: собственная декаль активного объекта
# --------------------------------------------------------------------------

#: Поле «Decal width» подтягивается к ширине меша (`sync_width_field`): калбэк ползунка в этот момент не заказывает
#: пересчёт ширины, которая в меше уже есть.
_field_sync = False


def availability_problem(controller, active_name, decal) -> str:
    """Почему ширина недоступна АКТИВНОМУ объекту (пусто — доступна). Чистая: всё приходит параметрами.

    `active_name` — имя исходного меша (`None`: активного меша нет), `decal` — `DecalProbeV1` его собственной декали
    либо `None`. Достаточно, чтобы пересчёт был возможен и про этот объект: у объекта есть декаль, записанная для
    него, а запись кнопки этого окна — про него, не сброшена и той же ревизии.
    """

    if not active_name:
        return NO_ACTIVE
    need = NEED_BUILD.format(name=active_name)
    if decal is None:
        return need
    if decal.source_name != active_name:
        return f"{need}: its decal was built for {decal.source_name or 'another object'}"
    record = None if controller is None else controller.width_build
    if record is None:
        return f"{need}: this window holds no build session"
    if record.source_name != active_name:
        return f"{need}: the build session of this window belongs to {record.source_name}"
    if controller.invalidation_count != record.invalidation_count:
        return f"{need}: the source changed since the last build"
    if decal.revision and record.source_digest not in decal.revision:
        return f"{need}: its decal is from another revision of the source"
    if record.preview_inputs is None or not has_preview_runs(record.preview_inputs):
        return NO_PREVIEW
    return ""


def _active_source(context):
    """Исходный меш активного объекта: сам меш либо, если активна декаль CFTUV, её источник; иначе `None`."""

    import bpy

    from .envelope_production_mesh import DECAL_REVISION_PROPERTY, DECAL_SOURCE_PROPERTY

    active = getattr(context, "active_object", None)
    if active is None or active.type != "MESH":
        return None
    if DECAL_REVISION_PROPERTY not in active.keys():
        return active
    source = active.parent
    if source is None:
        source = bpy.data.objects.get(str(active.get(DECAL_SOURCE_PROPERTY, "")))
    return source if source is not None and source.type == "MESH" else None


def active_target(context):
    """`(имя источника, DecalProbeV1 | None)` активного объекта; `(None, None)` — активного меша нет."""

    from .envelope_production_mesh import (
        DECAL_REVISION_PROPERTY,
        DECAL_SOURCE_PROPERTY,
        ProductionWriteError,
        find_decal_object,
    )

    source = _active_source(context)
    if source is None:
        return None, None
    try:
        decal = find_decal_object(source)
    except ProductionWriteError:
        return source.name, None  # чужой объект занял имя декали: отказ назовёт сама кнопка сборки
    if decal is None:
        return source.name, None
    keys = decal.keys()
    return source.name, DecalProbeV1(
        decal.name,
        str(decal[DECAL_SOURCE_PROPERTY]) if DECAL_SOURCE_PROPERTY in keys else "",
        str(decal[DECAL_REVISION_PROPERTY]) if DECAL_REVISION_PROPERTY in keys else "",
    )


def width_problem(context) -> str:
    """Почему ширина недоступна активному объекту контекста (пусто — доступна): единый вопрос кнопки, поля, клавиши."""

    name, decal = active_target(context)
    return availability_problem(_controller_of(context), name, decal)


def retarget(controller, target_name) -> bool:
    """Активной стала другая цель: превью снято, цель запомнена. `True` — цель сменилась.

    Заказы планировщика не трогаются: принятый заказ — про декаль прежнего объекта и завершится ею либо
    получит названный исход. Чужим становится только мгновенное состояние (линии превью).
    """

    if controller.width_target == target_name:
        return False
    controller.width_target = target_name
    set_preview(controller, None)
    return True


def follow_active_object(context) -> bool:
    """Обработчик depsgraph: сменился активный объект (`True`) либо исчезла цель под линиями превью (линии сняты)."""

    controller = _controller_of(context)
    if controller is None:
        return False
    source = _active_source(context)
    changed = retarget(controller, None if source is None else source.name)
    if not changed and controller.width_preview is not None and width_problem(context):
        set_preview(controller, None)  # декаль удалена (или запись сброшена) под линиями: цели нет, линий не остаётся
    return changed


def note_depsgraph(context, depsgraph) -> str:
    """Обработчик depsgraph: геометрию декали изменили не мы — модель снята. Пусто либо причина (`note_decal_updates`).

    Дёшево: пока у сессии нет владения мешем, обновления не просматриваются вовсе. Запись в Blender обработчик не делает.
    """

    controller = _controller_of(context)
    if controller is None or controller.width_mesh_owner is None or depsgraph is None:
        return ""
    return note_decal_updates(controller, depsgraph.updates)


def sync_width_field(context=None) -> bool:
    """Поле «Decal width» показывает ширину меша активного объекта. `True` — поле подтянуто.

    Только когда ширина доступна, нет заказа в пути (иначе меш вот-вот станет шириной ползунка, а не наоборот) и у
    меша записана ширина. Запись идёт под `_field_sync`: калбэк ползунка не заказывает пересчёт ширины, которая в меше
    уже есть.
    """

    import bpy

    from .envelope_production_mesh import DECAL_WIDTH_PROPERTY, find_decal_object

    global _field_sync
    context = bpy.context if context is None else context
    controller = _controller_of(context)
    settings = getattr(getattr(context, "scene", None), "hotspotuv_settings", None)
    if controller is None or settings is None or width_problem(context):
        return False
    scheduler = controller.width_live
    if scheduler is not None and scheduler.busy:
        return False
    decal = find_decal_object(_active_source(context))
    mesh = None if decal is None else decal.data
    if mesh is None or DECAL_WIDTH_PROPERTY not in mesh.keys():
        return False
    width = float(mesh[DECAL_WIDTH_PROPERTY])
    if float(settings.envelope_debug_alpha) == width:
        return False
    _field_sync = True
    try:
        settings.envelope_debug_alpha = width
    finally:
        _field_sync = False
    tag_view3d_redraw()
    return True


def _current_digest(source_obj) -> str:
    """Отпечаток меша источника СЕЙЧАС (тот же, что считает кнопка): правка после кнопки меняет его."""

    import bmesh

    from .analysis_surface import source_revision_from_bmesh

    own = source_obj.mode != "EDIT"
    bm = bmesh.new() if own else bmesh.from_edit_mesh(source_obj.data)
    try:
        if own:
            bm.from_mesh(source_obj.data)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()
        indices = tuple(face.index for face in bm.faces)
        return source_revision_from_bmesh(bm, source_obj, indices).digest
    finally:
        if own:
            bm.free()


def _source_digest(controller, source_obj) -> str:
    """Отпечаток меша источника СЕЙЧАС для проверки шага: из записи сессии, пока байты массивов меша те же (`envelope_source_check`), иначе полным путём `_current_digest`."""

    from .envelope_source_check import source_digest_now

    return source_digest_now(controller, source_obj, _current_digest)


def _begin(controller, request):
    import bpy

    from .envelope_production_export import ProductionCancelled, run_production
    from .envelope_production_mesh import find_decal_object
    from .envelope_request_policy import envelope_dissolve_uv_slide, envelope_stretch_budget

    target = request.payload
    problem = target_problem(controller, target)
    if problem:
        raise PreviewUnavailable(problem)
    record = controller.width_build
    source = bpy.data.objects.get(record.source_name)
    if source is None:
        raise PreviewUnavailable(SOURCE_GONE)
    decal = find_decal_object(source)
    if decal is None:
        raise PreviewUnavailable(DECAL_GONE)
    if _source_digest(controller, source) != record.source_digest:
        raise PreviewUnavailable(SOURCE_CHANGED)
    from .envelope_domain_pool import peek_domain_pool

    pool = peek_domain_pool(int(target.workers))
    bundle, selected, alpha = record.analysis_bundle, record.selected, float(request.alpha)
    object_key, data_key = record.source_object_key, record.source_data_key
    density = record.density
    budget = envelope_stretch_budget(record.stretch_percent)
    slide = envelope_dissolve_uv_slide(record.dissolve_percent)
    kernel_backend = record.kernel_backend
    skeleton_backend = record.skeleton_backend
    embedding_backend = record.embedding_backend
    density_backend = record.density_backend
    # Образец на экране, прежняя модель, журнал доверия и ключ читает главный поток; поток счёта получает их значениями (образцы, модели и журналы неизменяемы).
    grade = target.grade
    lean = grade in DRAG_GRADES  # WIDTH_REFRESH_LEAN_V1; инкрементальный такт (WIDTH_REFRESH_INCREMENTAL_V1) - тоже lean-прогон
    ensure_model(controller)  # модель образца на экране есть (миллисекунды numpy): первое обновление после кнопки тоже судит предсказание точным прогоном
    displayed, previous = controller.width_displayed, controller.width_model
    trust = controller.width_trust
    state = controller.width_incremental  # запись сверки читает главный поток: поток счёта получает её значением
    # LIVE_DRAG_STREAM_V1: срок сверки не делает потоковый такт полным, а партия доменов (половина воркеров) сверяется внутри такта (DRAG_VERIFY_BACKGROUND_V1); `None` - прежнее решение.
    stream = None
    if target.stream:
        log = controller.width_live_drag
        stream = StreamPolicyV1(verified=None if log is None else dict(log.verified), batch=max(1, int(target.workers) // 2))
    if stream is None and grade == REFRESH_INCREMENTAL and state is not None and verification_reason(state, alpha) in (REASON_PERIOD_TICKS, REASON_PERIOD_DISTANCE):
        controller.width_incremental = mark_attempt(state)  # сверку начали: прервёт её рука - повтор не раньше чем через `INCREMENTAL_RETRY_AFTER_TICKS` тактов (поток считает по записи ДО отметки)
    offset = float(target.offset)
    key = sample_key(record, offset)

    @gc_paused_function  # GC_PAUSED_DURING_WIDTH_TICK_V1: пауза сборщика стоит ровно на счёте тика (поток живёт секунды), а не на простое между тиками; сборщик общий на процесс
    def compute(cancel):
        plan = plan_tick(grade, displayed, previous, state, alpha, stream)  # WIDTH_REFRESH_INCREMENTAL_V1: какие домены обслужит сертификат (`None` - тик не инкрементальный)
        try:
            run = run_production(
                controller,
                bundle,
                selected,
                alpha,
                source_object_key=object_key,
                source_data_key=data_key,
                density=density,
                developable_stretch_budget=budget,
                silhouette_uv_slide=slide,
                domain_pool=pool,
                cancel=cancel,
                quiesce=False,
                kernel_backend=kernel_backend,
                skeleton_backend=skeleton_backend,
                embedding_backend=embedding_backend,
                density_backend=density_backend,
                lean=lean,
                skipped=None if plan is None else plan.served,
            )
        except ProductionCancelled as exc:
            raise PreviewCancelled(str(exc)) from exc
        return finish_live_run(
            run,
            alpha=alpha,
            offset=offset,
            key=key,
            displayed=displayed,
            previous=previous,
            trust=trust,
            lean=lean,
            plan=plan,
        )

    return ThreadedPreviewJob(compute, context=_JobContextV1(record, pool is not None))


def _validity(controller, request, job) -> str | None:
    import bpy

    from .envelope_production_mesh import find_decal_object

    if controller.width_build is not job.context.record:
        return BUILD_REPLACED
    source = bpy.data.objects.get(request.payload.source_name)
    if source is None:
        return SOURCE_GONE
    if find_decal_object(source) is None:
        return DECAL_GONE
    if request.payload.grade in DRAG_GRADES and not controller.width_drag_active:
        return LEAN_DRAG_ENDED
    return None


def _apply_lean(controller, request, job, live) -> None:
    """Точное lean-обновление (WIDTH_REFRESH_LEAN_V1) посчитано во время перетаскивания: меш и образец на экране заменены им (`lean_exact_now`), класс назван в статусе.

    Ни квитанции, ни дайджеста меша, ни консоли: это не зафиксированный ответ, его даёт подтверждение (COMMIT). Отмена инструмента возвращает то, что показывали перед
    первым кадром. Меш, который кадр не вправе писать (чужой, в Edit), - названный отказ (`lean_exact_now`), меш не тронут.
    """

    import bpy

    from .envelope_production_export import production_timing_text

    import time

    applying = time.perf_counter()
    before = controller.width_displayed
    reason = lean_exact_now(controller, live)
    if reason:
        log = controller.width_live_drag
        if request.payload.stream and log is not None:
            log.count(log.refused, reason)
        raise PreviewUnavailable(reason)
    mesh_settings = getattr(bpy.context.scene, "hotspotuv_decal_mesh", None)
    if mesh_settings is not None:
        incremental = live.plan is not None and live.plan.incremental
        mesh_settings.status = (
            f"{REFRESH_INCREMENTAL}: {len(live.plan.served)} domains moved by the affine certificate (structure verified afterwards), the rest exact at width {float(request.alpha):.4g} (the commit follows on confirm)"
            if incremental
            else f"{REFRESH_LEAN}: exact geometry at width {float(request.alpha):.4g} (the commit follows on confirm)"
        )
        mesh_settings.timing = f"{production_timing_text(live.run)} | live width {float(request.alpha):.4g}"
    set_preview(controller, None)  # точный кадр на ширине курсора: линии превью этой ширины не нужны
    log = controller.width_live_drag
    if request.payload.stream and log is not None:
        # LIVE_DRAG_STREAM_V1: точный такт ближайшей ширины лёг, а курсор ушёл дальше - меш сдвинут к курсору кадром модели НОВОГО образца (линии - только где модель не двигает).
        moments = (float(getattr(job, "started_at", 0.0)), float(getattr(job, "finished_at", 0.0)), applying, time.perf_counter())
        pointer = note_applied(log, request, live, before, moments)
        if pointer is not None:
            width_event_preview(controller, pointer, float(request.payload.offset))
    tag_view3d_redraw()


@gc_paused_function  # точное применение на главном потоке (COMMIT и lean-кадр): GC_PAUSED_DURING_WIDTH_TICK_V1
def _apply(controller, request, job, live) -> None:
    if request.payload.grade in DRAG_GRADES:
        return _apply_lean(controller, request, job, live)
    import bpy

    from .envelope_production_export import (
        production_timing_text,
        receipt_console_lines,
        receipt_status_text,
    )
    from .envelope_production_mesh import ProductionWriteError, rewrite_decal_mesh

    run = live.run
    target = request.payload
    source = bpy.data.objects.get(target.source_name)
    if source is None:
        raise PreviewUnavailable(SOURCE_GONE)
    try:
        receipt = rewrite_decal_mesh(
            source,
            run.results,
            offset=float(target.offset),
            material_name=target.material_name,
            width=float(request.alpha),
            arrays=live.arrays,
        )
    except ProductionWriteError as exc:
        raise PreviewUnavailable(str(exc)) from exc
    note_exact_display(controller, live)
    # Точная запись только что переписала раскладку: владение (тождество, размеры, поколение, отпечаток раскладки) — тут же, пока меш не тронул никто.
    capture_ownership(controller, bpy.data.objects.get(receipt.object_name or ""), live.sample)
    mesh_settings = getattr(bpy.context.scene, "hotspotuv_decal_mesh", None)
    if mesh_settings is not None:
        mesh_settings.status = receipt_status_text(receipt)
        mesh_settings.timing = (
            f"{production_timing_text(run)} | {receipt.faces} faces, {receipt.vertices} vertices"
            f" | live width {float(request.alpha):.4g}"
        )
    for line in receipt_console_lines(receipt, run.results):
        print(line, flush=True)
    set_preview(controller, None)  # точный результат на последней ширине применён: превью не нужно


# --------------------------------------------------------------------------
# Планировщик и калбэк ползунка
# --------------------------------------------------------------------------


class _BpyTimers:
    """`bpy.app.timers`, взятый лениво: сам планировщик `bpy` не импортирует."""

    def register(self, function, first_interval):
        import bpy

        bpy.app.timers.register(function, first_interval=first_interval)

    def is_registered(self, function):
        import bpy

        return bpy.app.timers.is_registered(function)


def _in_flight(scheduler) -> bool:
    return scheduler is not None and scheduler.in_flight


def scheduler_of(controller) -> AlphaPreviewScheduler:
    """Планировщик живой ширины контроллера; заводится при первом заказе и живёт с контроллером."""

    scheduler = controller.width_live
    if scheduler is None:
        scheduler = AlphaPreviewScheduler(
            begin=lambda request: _begin(controller, request),
            apply=lambda request, job, live: _apply(controller, request, job, live),
            valid=lambda request, job: _validity(controller, request, job),
            timers=_BpyTimers(),
            on_change=lambda _status: tag_view3d_redraw(),
            label="width",
            hold=lambda: _in_flight(controller.alpha_preview),
            idle=lambda: _live_drag_idle(controller),
        )
        controller.width_live = scheduler
    return scheduler


def _live_drag_idle(controller):
    """DRAG_VERIFY_BACKGROUND_V1: простой после потокового такта - `(ширина курсора, заказ такта сверки)` либо `None` (`envelope_width_stream.idle_answer`)."""

    log = controller.width_live_drag
    width = idle_answer(log, controller.width_displayed, drag_active=controller.width_drag_active)
    return None if width is None else (width, log.target)


def begin_live_drag(controller, mesh_settings) -> LiveDragLogV1 | None:
    """Перетаскивание началось: журнал живого точного меша заведён заново, если настройка «Live exact mesh while dragging» включена (иначе `None`)."""

    log = LiveDragLogV1() if getattr(mesh_settings, "drag_live_mesh", False) else None
    if log is not None and controller.width_live is not None:
        log.baseline = controller.width_live.counters
    if log is not None:
        seed_verified(log, controller.width_displayed, controller.width_incremental)
    controller.width_live_drag = log
    return log


def end_live_drag(controller) -> str:
    """Перетаскивание кончилось: строка журнала в консоль (журнал остаётся до следующего перетаскивания: его показывает панель). Пусто - живого точного меша не было."""

    log = controller.width_live_drag
    if log is None:
        return ""
    scheduler = controller.width_live
    line = live_drag_status_line(log, None if scheduler is None else scheduler.counters)
    print(f"[CFTUV][WidthLive] {line}", flush=True)
    return line


def request_drag_tick(context, controller, width: float) -> bool:
    """Точное lean-обновление (WIDTH_REFRESH_LEAN_V1) на ширине курсора, пока идёт перетаскивание: заказ тем же планировщиком, что и точный пересчёт. `True` - заказано.

    Включается настройкой «Exact ticks while dragging» (`drag_exact_ticks`, по умолчанию выключена: пул занят и пока рука стоит). Заказ живёт, пока идёт перетаскивание
    (`width_drag_active`), и тот же планировщик отменяет его подтверждением: подтверждение заказывает зафиксированный ответ (COMMIT, полный путь), а меш lean-кадра
    он переписывает целиком. Пока на экране нет образца точной записи, обновлять нечего.
    """

    scene = getattr(context, "scene", None)
    settings = getattr(scene, "hotspotuv_settings", None)
    mesh_settings = getattr(scene, "hotspotuv_decal_mesh", None)
    if controller is None or settings is None or mesh_settings is None or controller.width_build is None:
        return False
    # LIVE_DRAG_STREAM_V1: «Live exact mesh while dragging» - потоковый заказ (без паузы, полёт не отменяется, последнее побеждает); иначе прежний заказ «Exact ticks while dragging».
    live_mesh = bool(getattr(mesh_settings, "drag_live_mesh", False))
    # Дешёвое - первым: событие мыши приходит десятки раз в секунду, а вопрос «есть ли у активного объекта своя декаль» обходит объекты сцены.
    if not (live_mesh or getattr(mesh_settings, "drag_exact_ticks", False)) or not controller.width_drag_active:
        return False
    if controller.width_displayed is None:
        if live_mesh and controller.width_live_drag is not None:
            controller.width_live_drag.count(controller.width_live_drag.refused, STEP_NO_SAMPLE)
        return False
    if width_problem(context):
        return False
    target = replace(target_of(settings, mesh_settings, controller.width_build), grade=REFRESH_INCREMENTAL, stream=live_mesh)
    if target_problem(controller, target):
        return False
    request = scheduler_of(controller).request(float(width), target, stream=live_mesh)
    if live_mesh:
        log = controller.width_live_drag
        if log is None:  # настройку включили посреди перетаскивания: журнал с этого шага
            log = controller.width_live_drag = LiveDragLogV1()
            seed_verified(log, controller.width_displayed, controller.width_incremental)
        note_step(log, request, target)
    return True


def target_of(settings, mesh_settings, record: LastProductionBuildV1) -> WidthLiveTargetV1:
    return WidthLiveTargetV1(
        record.source_name,
        int(getattr(settings, "envelope_debug_workers", 0) or 0),
        settings.envelope_debug_fan_density,
        int(settings.envelope_debug_max_stretch),
        float(mesh_settings.offset),
        str(mesh_settings.material_name).strip() or "CFTUV_Decal",
        float(settings.envelope_debug_dissolve_uv_tolerance),
    )


def _controller_of(context):
    manager = getattr(context, "window_manager", None)
    return None if manager is None else getattr(manager, "_cftuv_envelope_debug_session", None)


def schedule_width_live(settings, context) -> None:
    """Калбэк ширины: превью сразу, точный пересчёт — заказом. Без записи кнопки либо когда активный объект не
    цель записи — ничего (нечего менять, причина в `width_problem`).

    Расхождение плотности или допуска с записью кнопки не считает ничего и называет причину строкой:
    тот же закон, что у `_request_policy_update` («press Build»).
    """

    controller = _controller_of(context)
    scene = getattr(context, "scene", None)
    mesh_settings = getattr(scene, "hotspotuv_decal_mesh", None)
    if controller is None or mesh_settings is None or controller.width_build is None:
        return
    if _field_sync:
        return  # поле подтянуто к ширине меша: пересчитывать то, что в меше уже есть, незачем
    if width_problem(context):
        set_preview(controller, None)  # активный объект — не цель записи кнопки: ни заказа, ни чужих линий
        return
    record = controller.width_build
    width = float(settings.envelope_debug_alpha)
    target = target_of(settings, mesh_settings, record)
    problem = target_problem(controller, target)
    scheduler = scheduler_of(controller)
    if problem:
        set_preview(controller, None)
        controller.supersede_preview(problem)
        scheduler.note_unavailable(problem)
        return
    width_event_preview(controller, width, float(mesh_settings.offset))
    scheduler.request(width, target)


def settle_width_live(controller) -> None:
    """Слив заказа без паузы (смоки, пакетные инструменты): после возврата точный результат применён."""

    if controller is not None and controller.width_live is not None:
        controller.width_live.settle()


def status_lines(controller) -> tuple[str, ...]:
    """Строки панели: статус точного пересчёта и, пока оно есть, имя и итог превью."""

    lines = []
    if controller is None:
        return ()
    scheduler = controller.width_live
    if scheduler is not None and scheduler.status_text:
        lines.append(f"Width: {scheduler.status_text}")
    state = controller.width_preview
    if state is not None:
        lines.append(state.preview.status_text())
    lines.extend(mesh_preview_status_lines(controller))
    log = getattr(controller, "width_live_drag", None)
    if log is not None:
        lines.append(live_drag_status_line(log, None if scheduler is None else scheduler.counters))
    return tuple(lines)


def draw_decal_width_rows(layout) -> None:
    """Строки ширины в продуктовой секции: поле «Decal width», инструмент и статус.

    Без собственной декали у активного объекта поле и кнопка отключены, а вместо статуса — причина.
    """

    import bpy

    context = bpy.context
    problem = width_problem(context)
    settings = getattr(getattr(context, "scene", None), "hotspotuv_settings", None)
    if settings is not None:
        row = layout.row()
        row.enabled = not problem
        row.prop(settings, "envelope_debug_alpha", text="Decal width")
    layout.operator(
        "hotspotuv.adjust_decal_width", text="Adjust Decal Width", icon="ARROW_LEFTRIGHT"
    )
    mesh_settings = getattr(getattr(context, "scene", None), "hotspotuv_decal_mesh", None)
    if mesh_settings is not None:
        layout.prop(mesh_settings, "drag_exact_ticks")
        layout.prop(mesh_settings, "drag_live_mesh")
    if problem:
        layout.label(text=problem, icon="INFO")
        return
    manager = getattr(context, "window_manager", None)
    controller = None if manager is None else getattr(manager, "_cftuv_envelope_debug_session", None)
    for line in status_lines(controller):
        layout.label(text=line)


def reconcile_after_history(context=None) -> None:
    """После Undo/Redo/Reload: превью снято, а меш и ползунок сверены (см. «ОТМЕНА» в шапке).

    Шаг истории записан ДО точного результата, поэтому после Redo ползунок может стоять на ширине, которой в
    меше нет. Сверка — по свойству МЕША `cftuv_decal_width` (оно восстанавливается вместе с геометрией);
    расхождение заказывает точный пересчёт.
    """

    import bpy

    from .envelope_production_mesh import DECAL_WIDTH_PROPERTY, find_decal_object

    context = bpy.context if context is None else context
    controller = _controller_of(context)
    if controller is None:
        return
    set_preview(controller, None)
    note_history(controller)  # что показывает меш, неизвестно: модель снята (`PREVIEW_HISTORY_STEP`), точный путь заведёт её заново
    record = controller.width_build
    scene = getattr(context, "scene", None)
    settings = None if scene is None else getattr(scene, "hotspotuv_settings", None)
    mesh_settings = None if scene is None else getattr(scene, "hotspotuv_decal_mesh", None)
    if record is None or settings is None or mesh_settings is None or width_problem(context):
        return
    source = bpy.data.objects.get(record.source_name)
    decal = None if source is None else find_decal_object(source)
    mesh = None if decal is None else decal.data
    if mesh is None or DECAL_WIDTH_PROPERTY not in mesh.keys():
        return
    width = float(settings.envelope_debug_alpha)
    if float(mesh[DECAL_WIDTH_PROPERTY]) == width:
        return
    target = target_of(settings, mesh_settings, record)
    scheduler = scheduler_of(controller)
    problem = target_problem(controller, target)
    if problem:
        scheduler.note_unavailable(problem)
        return
    scheduler.request(width, target)


__all__ = (
    "BUILD_REPLACED",
    "DECAL_GONE",
    "DecalProbeV1",
    "IDLE_ALREADY_BUILT",
    "IDLE_BUILT",
    "IDLE_NOT_DEFERRED",
    "IDLE_SUPERSEDED",
    "INPUTS_BUILD_FAILED",
    "INPUTS_BUILT_EVENT",
    "INPUTS_BUILT_IDLE",
    "LastProductionBuildV1",
    "NEED_BUILD",
    "NO_ACTIVE",
    "NO_BUILD",
    "NO_PREVIEW",
    "PREVIEW_BINARY64_V1",
    "POLICY_CHANGED",
    "SOURCE_CHANGED",
    "SOURCE_GONE",
    "WidthLiveTargetV1",
    "WidthPreviewStateV1",
    "active_target",
    "availability_problem",
    "begin_live_drag",
    "draw_decal_width_rows",
    "end_live_drag",
    "follow_active_object",
    "has_preview_runs",
    "note_depsgraph",
    "preview_now",
    "reconcile_after_history",
    "request_drag_tick",
    "remember_build",
    "resolved_preview_inputs",
    "retarget",
    "run_idle_preparation",
    "schedule_idle_preparation",
    "scheduler_of",
    "schedule_width_live",
    "set_preview",
    "settle_width_live",
    "status_lines",
    "sync_width_field",
    "tag_view3d_redraw",
    "target_of",
    "target_problem",
    "width_event_preview",
    "width_problem",
)
