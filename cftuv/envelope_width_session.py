"""Исполнение модального инструмента ширины: всё, что оператор делает с Blender, кроме самого класса.

`envelope_width_modal` — тонкая оболочка (класс оператора, перевод событий Blender, регистрация). Всё,
что имеет смысл проверить без интерфейса, лежит здесь функциями, и смок вызывает ТЕ ЖЕ функции, что оператор:

- `view_scale_of` — ось (точка выбранных цепей на экране) и метры на пиксель в глубине цепей;
- `begin_adjust` — запись, с которой начинается перетаскивание (стартовая ширина, автомат, смещение декали);
- `apply_step` — итог события: мгновенное превью на новой ширине и заголовок области;
- `finish_adjust` — подтверждение (ширина уходит в ползунок один раз, дальше работает путь ползунка:
  заказ точного пересчёта планировщиком, `envelope_width_live`) либо отмена (превью снято, ничего не менялось).

В течение перетаскивания в свойство ширины НИЧЕГО не пишется: меняются линии превью, заголовок и — когда у сессии есть сертификат —
позиции и UV настоящего меша декали (`envelope_width_mesh_preview`, `PREVIEW_MESH_FROM_INTERVAL_V1`: превью, не сертифицировано).
Меш при этом тот же объект и тот же датаблок (`foreach_set`, ни одного нового датаблока), поэтому отмена возвращает прежний меш ПОБИТОВО
(`restore_base_mesh`: кадр на ширине базы — сама база), а подтверждение — единственная запись в свойство, один заказ точного счёта и
один шаг отмены на всё перетаскивание (его кладёт Blender при завершении оператора с флагом UNDO). Вход в инструмент заказывает затравку
сертификата (`ensure_prime`), если его ещё нет: точный прогон на соседней ширине в фоне, без записи в меш.
"""

from __future__ import annotations

from dataclasses import dataclass

from .envelope_width_adjust import PHASE_CONFIRMED, WidthAdjustSessionV1, WidthStepV1
from .envelope_width_live import ensure_prime, preview_now, set_preview, width_problem
from .envelope_width_mesh_preview import preview_mesh_now, restore_base_mesh

NO_VIEW = "Adjust Decal Width needs a 3D View"


@dataclass(frozen=True, slots=True)
class ViewScaleV1:
    """Ось и масштаб экрана: точка в пикселях региона и единицы длины сцены на пиксель."""

    pivot: tuple[float, float]
    metres_per_pixel: float


@dataclass(slots=True)
class AdjustRuntimeV1:
    """Состояние одного перетаскивания: автомат и то, что нужно его исполнению."""

    session: WidthAdjustSessionV1
    controller: object
    offset: float
    start_width: float
    last_step: WidthStepV1 | None = None


def _controller_of(context):
    manager = getattr(context, "window_manager", None)
    return None if manager is None else getattr(manager, "_cftuv_envelope_debug_session", None)


def poll_problem(context) -> str:
    """Почему инструмент недоступен активному объекту (пусто — доступен): у него нет своей свежей декали.

    Ответ один на кнопку, поле, клавишу и калбэк ползунка: `envelope_width_live.width_problem`.
    """

    return width_problem(context)


def window_region(area):
    """Регион WINDOW области 3D View и его `region_3d`, либо `(None, None)`."""

    if area is None or area.type != "VIEW_3D":
        return None, None
    region = next((item for item in area.regions if item.type == "WINDOW"), None)
    space = area.spaces.active
    return region, getattr(space, "region_3d", None)


def view_scale_of(context) -> ViewScaleV1 | None:
    """Ось и метры на пиксель по текущему виду; `None` — вида нет (фоновый Blender, нет 3D View)."""

    import bpy
    from bpy_extras import view3d_utils
    from mathutils import Vector

    from .envelope_width_preview import chain_centroid

    region, rv3d = window_region(getattr(context, "area", None))
    controller = _controller_of(context)
    record = None if controller is None else controller.width_build
    if region is None or rv3d is None or record is None:
        return None
    source = bpy.data.objects.get(record.source_name)
    centre = chain_centroid(record.preview_inputs)
    if source is None or centre is None:
        return None
    world = source.matrix_world @ Vector(centre)
    pivot = view3d_utils.location_3d_to_region_2d(region, rv3d, world)
    if pivot is None:
        return None
    here = view3d_utils.region_2d_to_location_3d(region, rv3d, pivot, world)
    there = view3d_utils.region_2d_to_location_3d(region, rv3d, (pivot[0] + 1.0, pivot[1]), world)
    scale = max(float(source.matrix_world.median_scale), 1e-12)
    per_pixel = (there - here).length / scale
    if not per_pixel > 0.0:
        return None
    return ViewScaleV1((float(pivot[0]), float(pivot[1])), per_pixel)


def begin_adjust(context, mouse, view: ViewScaleV1) -> AdjustRuntimeV1 | str:
    """Запись перетаскивания либо строка причины отказа (инструмент недоступен)."""

    problem = poll_problem(context)
    if problem:
        return problem
    controller = _controller_of(context)
    settings = context.scene.hotspotuv_settings
    mesh_settings = context.scene.hotspotuv_decal_mesh
    start = float(settings.envelope_debug_alpha)
    session = WidthAdjustSessionV1(
        start,
        pivot=view.pivot,
        start_mouse=mouse,
        metres_per_pixel=view.metres_per_pixel,
    )
    runtime = AdjustRuntimeV1(session, controller, float(mesh_settings.offset), start)
    # Начальное превью на стартовой ширине: линии видны сразу, ещё до первого движения.
    preview_now(controller, start, runtime.offset)
    ensure_prime(context)  # у сессии нет сертификата меша — затравка считается, пока рука доходит до перетаскивания
    return runtime


def set_header(context, text: str | None) -> None:
    """Заголовок области 3D View (`None` возвращает прежний); без области — ничего."""

    area = getattr(context, "area", None)
    if area is not None:
        area.header_text_set(text)


def apply_step(context, runtime: AdjustRuntimeV1, step: WidthStepV1) -> None:
    """Итог события: превью на новой ширине (только если она изменилась) и заголовок."""

    runtime.last_step = step
    if step.changed:
        preview_now(runtime.controller, step.width, runtime.offset)
        preview_mesh_now(runtime.controller, step.width)
    set_header(context, step.header)


def finish_adjust(context, runtime: AdjustRuntimeV1, *, confirmed: bool) -> str:
    """Подтверждение либо отмена: `FINISHED` (ширина записана, заказан точный счёт) или `CANCELLED`.

    Подтверждение без перемены ширины — `CANCELLED` (шага отмены нет, нечего откатывать). Единственная запись
    в свойство ширины стоит здесь: её калбэк (`schedule_width_live`) пересчитает превью и закажет точный
    результат планировщиком.
    """

    set_header(context, None)
    width = runtime.session.width
    if confirmed and runtime.session.phase == PHASE_CONFIRMED and width != runtime.start_width:
        context.scene.hotspotuv_settings.envelope_debug_alpha = width
        return "FINISHED"
    restore_base_mesh(runtime.controller)  # меш назад на геометрию базы, побитово: ничего не менялось
    set_preview(runtime.controller, None)
    return "CANCELLED"


__all__ = (
    "AdjustRuntimeV1",
    "NO_VIEW",
    "ViewScaleV1",
    "apply_step",
    "begin_adjust",
    "finish_adjust",
    "poll_problem",
    "set_header",
    "view_scale_of",
    "window_region",
)
