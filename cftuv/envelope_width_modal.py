"""Оператор «Adjust Decal Width»: ширина декали тянется мышью, как у нативных Inset и Extrude.

Тонкая оболочка над чистым автоматом (`envelope_width_adjust`) и его исполнением
(`envelope_width_session`): здесь только Blender — класс оператора, перевод событий в `WidthEventV1`,
клавиша и обработчики истории. Закон инструмента и что он рисует — в тех модулях; кратко:

- `invoke` запоминает ширину и положение мыши; кнопка панели «Adjust Decal Width» и клавиша
  `Ctrl+Shift+Alt+W` в 3D View (аккорд из трёх модификаторов выбран, чтобы не пересекаться со штатной раскладкой;
  при регистрации пользовательская раскладка сверяется с ним, пересечение называется строкой консоли);
- движение мыши меняет ширину пропорционально пути курсора от оси: пока идёт перетаскивание, в заголовке
  `Decal width: 0.250 m (preview)`, а на экране — ТОЛЬКО превью `PREVIEW_BINARY64_V1`; Ctrl — привязка к шагу,
  Shift — точно (x0.1), цифры/точка/Backspace — число с клавиатуры, колесо и средняя кнопка — навигация вида;
- ЛКМ либо Enter подтверждают: ширина уходит в ползунок один раз, точный пересчёт заказывает планировщик
  (фон), шаг отмены один на всё перетаскивание; ПКМ либо Esc отменяют, ничего не менялось.

Доступность: инструмент (`poll`, подсказка отключённой кнопки через `poll_message_set`), поле «Decal width» панели
и калбэк ползунка требуют у АКТИВНОГО объекта собственную построенную декаль и запись кнопки этого окна про него
(`envelope_width_live.width_problem`); `invoke` называет ту же причину строкой. Смена активного объекта снимает превью
и подтягивает поле к ширине его меша (обработчик depsgraph).

Регистрация живёт здесь и зовётся из `register_production_operator` (рядом с «Build Decal Mesh»): класс,
клавиша, оверлей и обработчики Undo/Redo/загрузки файла.
"""

from __future__ import annotations

import bpy
from bpy.app.handlers import persistent

from .envelope_width_adjust import (
    KIND_BACKSPACE,
    KIND_CANCEL,
    KIND_CONFIRM,
    KIND_DIGIT,
    KIND_MOVE,
    KIND_POINT,
    PHASE_ACTIVE,
    PHASE_CONFIRMED,
    WidthEventV1,
)
from .envelope_width_live import follow_active_object, reconcile_after_history, sync_width_field
from .envelope_width_overlay import register_overlay, unregister_overlay
from .envelope_width_session import (
    NO_VIEW,
    apply_step,
    begin_adjust,
    finish_adjust,
    poll_problem,
    set_header,
    view_scale_of,
    window_region,
)

KEY_TYPE = "W"
PASS_THROUGH_EVENTS = frozenset(
    {
        "MIDDLEMOUSE",
        "WHEELUPMOUSE",
        "WHEELDOWNMOUSE",
        "TRACKPADPAN",
        "TRACKPADZOOM",
        "NDOF_MOTION",
    }
)
MODIFIER_EVENTS = frozenset({"LEFT_CTRL", "RIGHT_CTRL", "LEFT_SHIFT", "RIGHT_SHIFT"})
DIGIT_EVENTS = {
    **{name: str(index) for index, name in enumerate(
        ("ZERO", "ONE", "TWO", "THREE", "FOUR", "FIVE", "SIX", "SEVEN", "EIGHT", "NINE")
    )},
    **{f"NUMPAD_{index}": str(index) for index in range(10)},
}
POINT_EVENTS = frozenset({"PERIOD", "NUMPAD_PERIOD"})
CONFIRM_EVENTS = frozenset({"RET", "NUMPAD_ENTER"})
CANCEL_EVENTS = frozenset({"ESC", "RIGHTMOUSE"})


def translate_event(event, origin, last_mouse):
    """Событие Blender -> `WidthEventV1`, `"PASS"` (навигация вида) либо `None` (проглотить).

    `origin` — `(x, y)` региона WINDOW в координатах окна (из него вычитается позиция мыши: модальный
    обработчик, запущенный кнопкой боковой панели, получает события региона панели, а мышь нужна в регионе вида).
    """

    kind, value = event.type, event.value
    x, y = float(event.mouse_x) - origin[0], float(event.mouse_y) - origin[1]
    ctrl, shift = bool(event.ctrl), bool(event.shift)
    if kind == "MOUSEMOVE":
        return WidthEventV1(KIND_MOVE, x, y, ctrl, shift)
    if kind in MODIFIER_EVENTS:
        # Ctrl и Shift меняют ширину сразу (привязка, точность): перерисовка в той же точке.
        return WidthEventV1(KIND_MOVE, last_mouse[0], last_mouse[1], ctrl, shift)
    if kind in PASS_THROUGH_EVENTS:
        return "PASS"
    if value != "PRESS":
        return None
    if kind == "LEFTMOUSE" or kind in CONFIRM_EVENTS:
        return WidthEventV1(KIND_CONFIRM, x, y, ctrl, shift)
    if kind in CANCEL_EVENTS:
        return WidthEventV1(KIND_CANCEL, x, y, ctrl, shift)
    if kind in DIGIT_EVENTS:
        return WidthEventV1(KIND_DIGIT, x, y, ctrl, shift, DIGIT_EVENTS[kind])
    if kind in POINT_EVENTS:
        return WidthEventV1(KIND_POINT, x, y, ctrl, shift)
    if kind == "BACK_SPACE":
        return WidthEventV1(KIND_BACKSPACE, x, y, ctrl, shift)
    return None


class HOTSPOTUV_OT_AdjustDecalWidth(bpy.types.Operator):
    bl_idname = "hotspotuv.adjust_decal_width"
    bl_label = "Adjust Decal Width"
    bl_description = (
        "Drag in the viewport to change the width of the decal like Inset or Extrude: the new strip "
        "boundary is shown as a preview (PREVIEW_BINARY64_V1) while dragging; LMB or Enter confirms "
        "and recomputes the decal exactly in the background, RMB or Esc cancels"
    )
    # Один шаг отмены на всё перетаскивание: его кладёт Blender при завершении оператора с UNDO.
    bl_options = {"UNDO", "BLOCKING"}

    @classmethod
    def poll(cls, context):
        area = context.area
        if area is None or area.type != "VIEW_3D":
            return False
        problem = poll_problem(context)
        if problem:
            cls.poll_message_set(problem)  # подсказка отключённой кнопки: у активного объекта нет своей декали
            return False
        return True

    def invoke(self, context, event):
        problem = poll_problem(context)
        if problem:  # клавиша и вызов из Python: та же причина строкой, а не молчаливый отказ
            self.report({"WARNING"}, problem)
            return {"CANCELLED"}
        view = view_scale_of(context)
        if view is None:
            self.report({"WARNING"}, NO_VIEW)
            return {"CANCELLED"}
        region, _rv3d = window_region(context.area)
        self._origin = (float(region.x), float(region.y))
        mouse = (float(event.mouse_x) - self._origin[0], float(event.mouse_y) - self._origin[1])
        runtime = begin_adjust(context, mouse, view)
        if isinstance(runtime, str):
            self.report({"WARNING"}, runtime)
            return {"CANCELLED"}
        self._runtime = runtime
        self._mouse = mouse
        context.window_manager.modal_handler_add(self)
        context.window.cursor_modal_set("SCROLL_XY")
        set_header(context, runtime.session.header())
        context.area.tag_redraw()
        return {"RUNNING_MODAL"}

    def modal(self, context, event):
        translated = translate_event(event, self._origin, self._mouse)
        if translated == "PASS":
            return {"PASS_THROUGH"}
        if translated is None:
            return {"RUNNING_MODAL"}
        self._mouse = (translated.x, translated.y) if translated.kind == KIND_MOVE else self._mouse
        step = self._runtime.session.handle(translated)
        apply_step(context, self._runtime, step)
        if context.area is not None:
            context.area.tag_redraw()
        if step.phase == PHASE_ACTIVE:
            return {"RUNNING_MODAL"}
        return self._finish(context, confirmed=step.phase == PHASE_CONFIRMED)

    def cancel(self, context):
        """Blender снимает модальный оператор извне (загрузка файла, выход): превью не остаётся."""

        self._finish(context, confirmed=False)

    def _finish(self, context, *, confirmed):
        context.window.cursor_modal_restore()
        return {finish_adjust(context, self._runtime, confirmed=confirmed)}


_CLASSES = (HOTSPOTUV_OT_AdjustDecalWidth,)
_KEYMAPS: list = []
_HISTORY_DELAY = 0.05


def _sync_once():
    try:
        sync_width_field()
    except Exception as exc:  # noqa: BLE001 - подтягивание поля называется строкой, а не молчит
        print(f"[CFTUV][WidthLive] field sync failed: {type(exc).__name__}: {exc}", flush=True)
    return None


@persistent
def _after_depsgraph(*_args) -> None:
    """Смена активного объекта: превью снято, поле «Decal width» подтягивается к ширине меша нового объекта.

    Обработчик depsgraph вызывается часто, поэтому здесь только дешёвая сверка имени (`follow_active_object`);
    запись в свойство — отложенным таймером, не из обработчика.
    """

    try:
        changed = follow_active_object(bpy.context)
    except Exception as exc:  # noqa: BLE001 - отказ слежения называется строкой консоли, а не молчит
        print(f"[CFTUV][WidthLive] follow active object failed: {type(exc).__name__}: {exc}", flush=True)
        return
    if changed and not bpy.app.timers.is_registered(_sync_once):
        bpy.app.timers.register(_sync_once, first_interval=_HISTORY_DELAY)


def _reconcile_once():
    try:
        reconcile_after_history()
    except Exception as exc:  # noqa: BLE001 - сверка после истории называется строкой, а не молчит
        print(f"[CFTUV][WidthLive] reconcile after history failed: {type(exc).__name__}: {exc}", flush=True)
    return None


@persistent
def _after_history(*_args) -> None:
    """Undo/Redo/загрузка файла: превью снять и меш сверить с ползунком — после того как история отработала."""

    if not bpy.app.timers.is_registered(_reconcile_once):
        bpy.app.timers.register(_reconcile_once, first_interval=_HISTORY_DELAY)


@persistent
def _after_load(*_args) -> None:
    """Загрузка файла: запись кнопки и превью старой сцены забыты у всех контроллеров (см. `forget_width_state`)."""

    from .envelope_debug_session import WINDOW_MANAGER_SESSION_ATTRIBUTE

    descriptor = getattr(bpy.types.WindowManager, WINDOW_MANAGER_SESSION_ATTRIBUTE, None)
    forget = getattr(descriptor, "forget_width_state", None)
    if forget is not None:
        forget()


#: Обработчики истории: список `bpy.app.handlers` -> наш обработчик.
_HANDLERS = (
    ("undo_post", _after_history),
    ("redo_post", _after_history),
    ("load_post", _after_load),
    ("depsgraph_update_post", _after_depsgraph),
)


def _chord_conflicts(wm) -> list:
    """Пользовательские привязки с тем же аккордом, что у инструмента (пусто — свободно)."""

    config = wm.keyconfigs.user
    if config is None:
        return []
    return [
        (keymap.name, item.idname)
        for keymap in config.keymaps
        for item in keymap.keymap_items
        if item.type == KEY_TYPE
        and item.ctrl
        and item.shift
        and item.alt
        and item.active
        and item.idname != HOTSPOTUV_OT_AdjustDecalWidth.bl_idname
    ]


def _register_key() -> None:
    """Клавиша инструмента. Регистрация аддона идёт в ОГРАНИЧЕННОМ контексте (`addon_utils.enable`: у `bpy.data`
    нет коллекций, `bpy.context` урезан), поэтому любой сбой здесь называется строкой консоли и НЕ ломает
    регистрацию аддона: инструмент остаётся доступен кнопкой панели."""

    try:
        wm = bpy.context.window_manager
        config = None if wm is None else wm.keyconfigs.addon
        if config is None:
            return  # фоновый Blender: раскладки аддонов нет, клавиша не нужна
        keymap = config.keymaps.new(name="3D View", space_type="VIEW_3D")
        item = keymap.keymap_items.new(
            HOTSPOTUV_OT_AdjustDecalWidth.bl_idname, KEY_TYPE, "PRESS", ctrl=True, shift=True, alt=True
        )
        _KEYMAPS.append((keymap, item))
        conflicts = _chord_conflicts(wm)
    except (AttributeError, KeyError, RuntimeError, TypeError) as exc:
        print(f"[CFTUV][WidthLive] the key was not registered: {type(exc).__name__}: {exc}", flush=True)
        return
    for name, idname in conflicts:
        print(
            f"[CFTUV][WidthLive] Ctrl+Shift+Alt+{KEY_TYPE} is also bound to {idname} in {name!r}",
            flush=True,
        )


def register_width_tools() -> None:
    """Класс оператора, клавиша, оверлей и обработчики истории. Повтор безопасен."""

    unregister_width_tools()
    for cls in _CLASSES:
        bpy.utils.register_class(cls)
    _register_key()
    register_overlay()
    for name, handler in _HANDLERS:
        handlers = getattr(bpy.app.handlers, name)
        if handler not in handlers:
            handlers.append(handler)


def unregister_width_tools() -> None:
    for name, handler in _HANDLERS:
        handlers = getattr(bpy.app.handlers, name)
        while handler in handlers:
            handlers.remove(handler)
    for timer in (_reconcile_once, _sync_once):
        if bpy.app.timers.is_registered(timer):
            bpy.app.timers.unregister(timer)
    unregister_overlay()
    while _KEYMAPS:
        keymap, item = _KEYMAPS.pop()
        try:
            keymap.keymap_items.remove(item)
        except (ReferenceError, RuntimeError):
            pass  # раскладку уже снял Blender вместе с аддоном: снимать нечего
    for cls in reversed(_CLASSES):
        if getattr(cls, "is_registered", False):
            bpy.utils.unregister_class(cls)


__all__ = (
    "HOTSPOTUV_OT_AdjustDecalWidth",
    "KEY_TYPE",
    "register_width_tools",
    "translate_event",
    "unregister_width_tools",
)
