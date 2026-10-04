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
"""

from __future__ import annotations

from dataclasses import dataclass

from .envelope_alpha_preview import (
    AlphaPreviewScheduler,
    PreviewCancelled,
    PreviewUnavailable,
    ThreadedPreviewJob,
)
from .envelope_width_preview import (
    PREVIEW_BINARY64_V1,
    WidthPreviewV1,
    build_preview_inputs,
    compute_width_preview,
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
    preview_inputs: object
    #: Ширина (alpha), с которой кнопка записала меш.
    width: float


@dataclass(frozen=True, slots=True)
class WidthLiveTargetV1:
    """Что нужно цели из настроек на момент заказа (значения, не ссылки на RNA)."""

    source_name: str
    workers: int
    density: object
    stretch_percent: int
    offset: float
    material_name: str


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
        preview_inputs=build_preview_inputs(bundle.patch_surface, run.selected_by_patch),
        width=float(width),
    )
    controller.width_build = record
    controller.width_target = record.source_name
    set_preview(controller, None)
    return record


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


def preview_now(controller, width: float, offset: float) -> WidthPreviewV1 | None:
    """Мгновенное превью на `width` по записи кнопки; кладёт его в состояние. `None` — превью нет."""

    record = controller.width_build
    if record is None or record.preview_inputs is None:
        return None
    preview = compute_width_preview(record.preview_inputs, width, lift=offset)
    set_preview(controller, preview, record.source_name)
    return preview


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
    if record.preview_inputs is None or not record.preview_inputs.runs:
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


def _begin(controller, request):
    import bpy

    from .envelope_production_export import ProductionCancelled, run_production
    from .envelope_production_mesh import find_decal_object
    from .envelope_request_policy import envelope_stretch_budget

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
    if _current_digest(source) != record.source_digest:
        raise PreviewUnavailable(SOURCE_CHANGED)
    from .envelope_domain_pool import peek_domain_pool

    pool = peek_domain_pool(int(target.workers))
    bundle, selected, alpha = record.analysis_bundle, record.selected, float(request.alpha)
    object_key, data_key = record.source_object_key, record.source_data_key
    density = record.density
    budget = envelope_stretch_budget(record.stretch_percent)

    def compute(cancel):
        try:
            return run_production(
                controller,
                bundle,
                selected,
                alpha,
                source_object_key=object_key,
                source_data_key=data_key,
                density=density,
                developable_stretch_budget=budget,
                domain_pool=pool,
                cancel=cancel,
                quiesce=False,
            )
        except ProductionCancelled as exc:
            raise PreviewCancelled(str(exc)) from exc

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
    return None


def _apply(controller, request, job, run) -> None:
    import bpy

    from .envelope_production_export import (
        production_timing_text,
        receipt_console_lines,
        receipt_status_text,
    )
    from .envelope_production_mesh import ProductionWriteError, rewrite_decal_mesh

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
        )
    except ProductionWriteError as exc:
        raise PreviewUnavailable(str(exc)) from exc
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


def scheduler_of(controller) -> AlphaPreviewScheduler:
    """Планировщик живой ширины контроллера; заводится при первом заказе и живёт с контроллером."""

    scheduler = controller.width_live
    if scheduler is None:
        scheduler = AlphaPreviewScheduler(
            begin=lambda request: _begin(controller, request),
            apply=lambda request, job, run: _apply(controller, request, job, run),
            valid=lambda request, job: _validity(controller, request, job),
            timers=_BpyTimers(),
            on_change=lambda _status: tag_view3d_redraw(),
            label="width",
            hold=lambda: bool(
                controller.alpha_preview is not None and controller.alpha_preview.in_flight
            ),
        )
        controller.width_live = scheduler
    return scheduler


def target_of(settings, mesh_settings, record: LastProductionBuildV1) -> WidthLiveTargetV1:
    return WidthLiveTargetV1(
        record.source_name,
        int(getattr(settings, "envelope_debug_workers", 0) or 0),
        settings.envelope_debug_fan_density,
        int(settings.envelope_debug_max_stretch),
        float(mesh_settings.offset),
        str(mesh_settings.material_name).strip() or "CFTUV_Decal",
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
    preview_now(controller, width, float(mesh_settings.offset))
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
    "draw_decal_width_rows",
    "follow_active_object",
    "preview_now",
    "reconcile_after_history",
    "remember_build",
    "retarget",
    "scheduler_of",
    "schedule_width_live",
    "set_preview",
    "settle_width_live",
    "status_lines",
    "sync_width_field",
    "tag_view3d_redraw",
    "target_of",
    "target_problem",
    "width_problem",
)
