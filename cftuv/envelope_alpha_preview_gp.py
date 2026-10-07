"""Превью alpha для отладочного объекта Envelope (GP): склейка планировщика с Blender.

ЧТО ОБНОВЛЯЕТ ПОЛЗУНОК. Ползунок `envelope_debug_alpha` перерисовывает ТОЛЬКО слои очереди
отладочного GP-объекта `CFTUV_DEBUG_Envelope_<источник>` (движок QUEUE, после кнопки
отладки: тёплая сессия очереди, `QueueSessionStateV1`). Продуктовый меш «Build Decal Mesh»
ползунок не трогал и не трогает: продуктовая кнопка тёплой сессии очереди не заводит.
Эта граница сохранена как была; менялось только КОГДА и ГДЕ считается.

ЧТО ГДЕ ИСПОЛНЯЕТСЯ. Калбэк свойства (`schedule_alpha_preview`) читает настройки и
записывает заказ. Дальше работает `envelope_alpha_preview.AlphaPreviewScheduler`:

- `_begin` (главный поток) читает сессию, берёт живой пул (`slider_coverage_pool`:
  ползунок пул не стартует) и отдаёт потоку замкнутую функцию `compute`;
- `compute` (ПОТОК) — `recompute_queue_coverage` на готовых подготовках и ничего,
  что касается `bpy`: ни объектов, ни настроек, ни текстов;
- `_validity` и `_apply` (главный поток) проверяют, что сессия и GP-объект те же, и
  пишут слои тем же `redraw_envelope_queue_layers`, что и синхронный ползунок.

ОТМЕНА (UNDO). Таймер шага отмены НЕ кладёт: шаг на каждый применённый результат
засорил бы стек прямо во время перетаскивания. Выбрано и здесь записано: применение ничего не создаёт и не
освобождает в `bpy.data` (объект, слои и текст-приложение переиспользуются
`redraw_envelope_queue_layers`), то есть не повторяет причину падения, описанную в
`envelope_production_operator.UNDO_REQUIRED_REASON` (память memfile и висячие
указатели создаваемых датаблоков); собственный шаг отмены у изменения значения есть —
его кладёт сам интерфейс при отпускании ползунка. Цена, названная прямо: Ctrl+Z
возвращает ЗНАЧЕНИЕ alpha, а превью (производное, перерисовывается следующим движением
ползунка либо кнопкой) может остаться на последнем применённом значении.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from .envelope_alpha_preview import (
    AlphaPreviewScheduler,
    PreviewCancelled,
    PreviewUnavailable,
    ThreadedPreviewJob,
)

NO_WARM_SESSION = "no warm queue session: press Build Exact Reference Envelope Debug"
DENSITY_CHANGED = "Fan Density changed; press Build"
OBJECT_GONE = "Envelope debug object is gone"


@dataclass(frozen=True, slots=True)
class GpPreviewTargetV1:
    """Что нужно цели из настроек на момент заказа (значения, не ссылки на RNA)."""

    source_name: str
    density: object
    workers: int


@dataclass(frozen=True, slots=True)
class _JobContextV1:
    """Что поток НЕ трогает, а главный поток читает при применении."""

    session: object
    pool_profile: object
    pooled: bool


class _BpyTimers:
    """`bpy.app.timers`, взятый лениво: сам планировщик `bpy` не импортирует."""

    def register(self, function, first_interval):
        import bpy

        bpy.app.timers.register(function, first_interval=first_interval)

    def is_registered(self, function):
        import bpy

        return bpy.app.timers.is_registered(function)


def _tag_redraw(_status: str) -> None:
    """Строка статуса не свойство RNA: панель перерисовывается по явному заказу."""

    import bpy

    manager = bpy.context.window_manager
    for window in manager.windows:
        for area in window.screen.areas:
            if area.type == "VIEW_3D":
                area.tag_redraw()


def _settings_of(bpy_module):
    scene = getattr(bpy_module.context, "scene", None)
    return None if scene is None else getattr(scene, "hotspotuv_settings", None)


def session_problem(controller, target: GpPreviewTargetV1) -> str:
    """Почему счёта не будет (пусто — будет): те же условия, что у синхронного ползунка."""

    from .envelope_request_policy import normalize_envelope_fan_density

    session = controller.queue_session
    if (
        session is None
        or str(session.source_object_name) != str(target.source_name)
        or not session.entries
    ):
        return NO_WARM_SESSION
    if session.density != normalize_envelope_fan_density(target.density):
        return DENSITY_CHANGED
    return ""


def _object_exists(source_name) -> bool:
    import bpy

    from .envelope_debug_renderer import envelope_debug_object_name

    return bpy.data.objects.get(envelope_debug_object_name(source_name)) is not None


def _begin(controller, request):
    from .envelope_debug_profile import EnvelopeDebugProfileBuilderV1
    from .envelope_queue_export import (
        ENVELOPE_DEBUG_ENGINE_QUEUE,
        CoverageCancelled,
        recompute_queue_coverage,
    )

    target = request.payload
    problem = session_problem(controller, target)
    if problem:
        raise PreviewUnavailable(problem)
    session = controller.queue_session
    pool_profile = EnvelopeDebugProfileBuilderV1(
        str(target.source_name), ENVELOPE_DEBUG_ENGINE_QUEUE
    )
    open_pool = getattr(controller, "slider_coverage_pool", None)
    coverage_pool = (
        None if open_pool is None else open_pool(int(target.workers), pool_profile)
    )
    entries = session.entries
    alpha_text = str(float(request.alpha))

    def compute(cancel):
        try:
            return recompute_queue_coverage(
                entries, alpha_text, coverage_pool=coverage_pool, cancel=cancel
            )
        except CoverageCancelled as exc:
            raise PreviewCancelled(str(exc)) from exc

    return ThreadedPreviewJob(
        compute,
        context=_JobContextV1(session, pool_profile, coverage_pool is not None),
    )


def _validity(controller, request, job) -> str | None:
    import bpy

    from .envelope_debug_renderer import envelope_debug_object_name

    if controller.queue_session is not job.context.session:
        return "warm session was replaced while computing"
    name = envelope_debug_object_name(request.payload.source_name)
    if bpy.data.objects.get(name) is None:
        return OBJECT_GONE
    return None


def _apply(controller, request, job, scene) -> None:
    import bpy

    from .envelope_debug_renderer import (
        redraw_envelope_queue_layers,
        visibility_from_settings,
    )
    from .envelope_queue_export import queue_timing_text

    started = time.perf_counter()
    settings = _settings_of(bpy)
    context = job.context
    summary = redraw_envelope_queue_layers(
        request.payload.source_name,
        context.session.exact_scenes,
        scene,
        visibility_by_layer=(
            None if settings is None else visibility_from_settings(settings)
        ),
    )
    if summary is None:
        raise PreviewUnavailable(OBJECT_GONE)
    if settings is None:
        return
    apply_ms = (time.perf_counter() - started) * 1000.0
    timing = queue_timing_text(
        scene, context.pool_profile.snapshot() if context.pooled else None
    )
    settings.envelope_debug_queue_timing = (
        f"{timing} | alpha redraw {job.seconds * 1000.0 + apply_ms:.0f} ms "
        f"(compute {job.seconds * 1000.0:.0f}, apply {apply_ms:.0f})"
    )


def scheduler_of(controller) -> AlphaPreviewScheduler:
    """Планировщик контроллера окна; заводится при первом заказе и живёт с контроллером."""

    scheduler = controller.alpha_preview
    if scheduler is None:
        scheduler = AlphaPreviewScheduler(
            begin=lambda request: _begin(controller, request),
            apply=lambda request, job, value: _apply(controller, request, job, value),
            valid=lambda request, job: _validity(controller, request, job),
            timers=_BpyTimers(),
            on_change=_tag_redraw,
            # Живая ширина (и затравка её сертификата) делит с отладкой подготовки сессии и пул: два потока счёта разом не летят.
            hold=lambda: bool(
                (controller.width_live is not None and controller.width_live.in_flight)
                or (controller.width_prime is not None and controller.width_prime.in_flight)
            ),
        )
        controller.alpha_preview = scheduler
    return scheduler


def schedule_alpha_preview(settings, context) -> None:
    """Калбэк `update` ползунка alpha: записывает заказ и сразу возвращается.

    Поведение вокруг заказа прежнее: не движок QUEUE — ничего; сессии окна ещё нет
    (кнопку не нажимали) — ничего; смена плотности — строка «press Build», счёта нет.
    Новое — только строка статуса превью там, где раньше молчали (нет тёплой сессии).
    """

    from .envelope_queue_export import ENVELOPE_DEBUG_ENGINE_QUEUE
    from .envelope_width_live import schedule_width_live

    # Продуктовый меш: ширина декали живая независимо от движка отладки. Сбой её заказа называется строкой
    # консоли и не гасит превью отладки ниже.
    try:
        schedule_width_live(settings, context)
    except Exception as exc:  # noqa: BLE001 - сбой заказа называется строкой консоли, а не гасит отладку
        print(f"[CFTUV][WidthLive] order failed: {type(exc).__name__}: {exc}", flush=True)
    if str(settings.envelope_debug_engine) != ENVELOPE_DEBUG_ENGINE_QUEUE:
        return
    source_name = str(settings.envelope_debug_source_object).strip()
    manager = None if context is None else context.window_manager
    controller = (
        None
        if manager is None
        else getattr(manager, "_cftuv_envelope_debug_session", None)
    )
    if not source_name or controller is None:
        return
    target = GpPreviewTargetV1(
        source_name,
        settings.envelope_debug_fan_density,
        int(getattr(settings, "envelope_debug_workers", 0) or 0),
    )
    problem = session_problem(controller, target)
    if problem == DENSITY_CHANGED:
        controller.supersede_preview(problem)
        settings.envelope_debug_queue_timing = problem
        return
    scheduler = scheduler_of(controller)
    if not problem and not _object_exists(source_name):
        problem = OBJECT_GONE  # после Clear считать некуда: пул не гоняем ради выброшенного результата
    if problem:
        controller.supersede_preview(problem)
        scheduler.note_unavailable(problem)
        return
    scheduler.request(float(settings.envelope_debug_alpha), target)


def settle_alpha_preview(controller) -> None:
    """Слив заказа без паузы (смоки, пакетные инструменты): после возврата результат применён."""

    if controller is not None and controller.alpha_preview is not None:
        controller.alpha_preview.settle()


def draw_alpha_preview_row(layout) -> None:
    """Строка статуса превью под ползунком: пусто, пока ползунок ничего не заказывал."""

    import bpy

    controller = getattr(bpy.context.window_manager, "_cftuv_envelope_debug_session", None)
    scheduler = None if controller is None else controller.alpha_preview
    if scheduler is None or not scheduler.status_text:
        return
    layout.label(
        text=f"Preview: {scheduler.status_text}",
        icon="TIME" if scheduler.busy else "CHECKMARK",
    )


__all__ = (
    "DENSITY_CHANGED",
    "GpPreviewTargetV1",
    "NO_WARM_SESSION",
    "OBJECT_GONE",
    "draw_alpha_preview_row",
    "schedule_alpha_preview",
    "scheduler_of",
    "session_problem",
    "settle_alpha_preview",
)
