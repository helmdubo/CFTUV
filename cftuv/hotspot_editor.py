"""Редактор разметки Hotspot в Image Editor (HOTSPOT_MARKUP_EDITOR_V1, HOTSPOT_PANEL_EDITOR_V1): мини-окно, показ меток, инструмент и список.

МИНИ-ОКНО. «Mark Up» панели декали (`hotspotuv.hotspot_mark_up`) берёт материал декали и картинку из его узлов, сам создаёт и связывает
датаблок разметки, загружает черновик и открывает отдельное маленькое окно: временное окно `screen.info_log_show` (одна область, закрытие
убирает его экран), область переключается в Image Editor с этой картинкой, вид вписан в видимую часть, боковая панель открыта на вкладке
Hotspot, инструмент запущен. Повторное нажатие переиспользует окно.

ЧЕРНОВИК И ТЕКСТ. Правится черновик - коллекция меток сцены (`hotspotuv_hotspot_editor`, она сохраняется вместе с .blend). После каждой правки
черновик проверяется (`hotspot_markup.save_markup`): принятый пишется в текстовый датаблок, непринятый НЕ пишется - текст остаётся прежним
принятым, черновик остаётся в сцене, панель и заголовок называют находки по именам. Ничего не теряется молча: открыть разметку другого
материала поверх непринятого черновика - отказ `MARKUP_DRAFT_UNSAVED`, пока черновик не исправлен или явно не отброшен.

ИНСТРУМЕНТ (`hotspotuv.hotspot_markup_edit`, модальный, в области Image Editor): перетаскивание по пустому месту - новый прямоугольник по сетке
пикселей (шаг `[` и `]`), щелчок по метке - выбор, перетаскивание внутри - перенос, за ручку (угол, середина стороны) - размер, `D` - копия,
`X`/`Delete` - удаление, `U` - следующее направление «вверх», `Esc`/правая кнопка - выход. Колесо и средняя кнопка проходят в Image Editor,
как и всё над боковой панелью и заголовком (там правятся поля меток, пока инструмент работает). Закрытие окна завершает инструмент.

ОТДЕЛЬНОЕ ОКНО (HOTSPOT_EDITOR_WINDOW_V1): метки, вкладка Hotspot боковой панели и инструмент есть ТОЛЬКО в области мини-окна - ключ
`(окно, экран, область)` из указателей, окно проверяется среди открытых, загрузка файла ключ очищает. Второй Image Editor с той же картинкой и
UV Editor декали в Edit Mode меток не показывают, пока пользователь сам не включит «Show marks in UV Editor» (по умолчанию выключено). Заголовок
области - «CFTUV Hotspot Markup — <материал>» вместо меню Image Editor (выбор картинки, режим UV, тип области), картинка закреплена.

ПОКАЗ (обработчик `POST_PIXEL` Image Editor): полупрозрачная заливка своего оттенка у каждой метки (`mark_colours`: оттенок от id), контур,
номер и имя (с тенью), стрелка «вверх», нижний край (шов `trim`) или средняя линия (шов `corner`) выделены, у `corner` - центральная полоса
`band` контрастным тоном того же оттенка.
"""

from __future__ import annotations

import bpy
from bpy.props import BoolProperty, CollectionProperty, EnumProperty, IntProperty, PointerProperty, StringProperty

from .hotspot_blender import OVERLAY_EDITOR, ensure_material_markup, linked_markup_name, overlay_where, read_markup_text, scene_material_image, write_markup_text
from .hotspot_markup import KIND_CORNER, KIND_TRIM, KINDS, UP_VECTORS, HotspotMarkV1, HotspotMarkupV1, mark_colours, next_mark_id

EDITOR_ATTRIBUTE = "hotspotuv_hotspot_editor"
MARKUP_DRAFT_UNSAVED = "MARKUP_DRAFT_UNSAVED"
#: Окно мини-редактора не открылось (ни временное окно, ни новое): нажатие отказывает по имени.
HOTSPOT_EDITOR_WINDOW_NOT_OPENED = "HOTSPOT_EDITOR_WINDOW_NOT_OPENED"
#: Область окна не получила размеров за отведённое время: инструмент не запущен, причина - в консоли и в боковой панели (кнопка Edit Marks).
HOTSPOT_EDITOR_TOOL_NOT_STARTED = "HOTSPOT_EDITOR_TOOL_NOT_STARTED"
UP_ORDER = ("+y", "-y", "+x", "-x")
#: Подсказки клавиш инструмента: одна строка на действие; их показывают и боковая панель, и строка в окне.
KEY_HINTS = (
    "Drag on empty: new mark",
    "Drag a mark: move it",
    "Drag its edge or corner: resize",
    "[ / ]: halve / double the grid",
    "D duplicate, X delete, U turn up",
    "Esc or right click: stop the tool",
)
#: Окно, его экран и область мини-редактора (указатели; HOTSPOT_EDITOR_WINDOW_V1: метки, боковая панель и инструмент - только там) и области с
#: работающим инструментом. Закрытое окно не находится среди окон, загрузка файла очищает запись.
_MINI: dict = {}
#: Заголовок области мини-окна вместо меню Image Editor (выбор картинки, режим UV, тип области): окно разметки выглядит отдельным окном.
WINDOW_TITLE = "CFTUV Hotspot Markup"
_RUNNING: set = set()
#: Перетаскивание инструмента пишет черновик много раз в секунду: проверка и запись текста - на отпускании (флаг ставит инструмент).
_SUSPEND = {"commit": False}


def _changed(self, context) -> None:
    if not _SUSPEND["commit"]:
        commit_draft(context.scene)


class HOTSPOTUV_HotspotMark(bpy.types.PropertyGroup):
    mark_id: IntProperty(name="ID", min=0, update=_changed)
    label: StringProperty(name="Name", update=_changed)
    kind: EnumProperty(name="Kind", items=((KIND_TRIM, "Trim", "Bottom edge on the seam, top edge at the outer skirt edge"), (KIND_CORNER, "Corner", "Seam on the centre line, the two skirts as mirrored wings")), update=_changed)
    up: EnumProperty(name="Up", items=tuple((item, item, f"the mark's up direction is {item}") for item in UP_ORDER), update=_changed)
    x: IntProperty(name="X", update=_changed)
    y: IntProperty(name="Y", update=_changed)
    w: IntProperty(name="Width", update=_changed)
    h: IntProperty(name="Height", update=_changed)
    band: IntProperty(name="Band", min=0, update=_changed, description="Corner: width of the centre band of the detail in pixels (display only)")


def _redraw_image_editors(_self=None, _context=None) -> None:
    for window in bpy.context.window_manager.windows:
        for area in window.screen.areas:
            if area.type == "IMAGE_EDITOR":
                area.tag_redraw()


class HOTSPOTUV_HotspotEditor(bpy.types.PropertyGroup):
    draft_of: StringProperty(name="Draft of", description="Text datablock the draft was loaded from and is saved to")
    material: StringProperty(name="Material", description="Material whose markup the draft is (shown in the mini editor)")
    width: IntProperty(name="Texture width", min=0)
    height: IntProperty(name="Texture height", min=0)
    image: StringProperty(name="Image")
    marks: CollectionProperty(type=HOTSPOTUV_HotspotMark)
    active: IntProperty(name="Active mark", default=-1)
    grid: IntProperty(name="Grid", default=8, min=1, max=1024, description="Pixel grid of the edit tool ([ and ] halve and double it)")
    dirty: BoolProperty(name="Unsaved draft", default=False)
    problems: StringProperty(name="Problems")
    show_in_uv_editor: BoolProperty(
        name="Show marks in UV Editor",
        default=False,
        update=_redraw_image_editors,
        description="Also draw the marks (without the tool) in every other Image or UV Editor that shows the markup's texture, for laying UVs",
    )


def editor_of(scene):
    return getattr(scene, EDITOR_ATTRIBUTE, None)


def draft_markup(editor) -> HotspotMarkupV1:
    marks = [HotspotMarkV1(item.mark_id, item.label, item.kind, item.x, item.y, item.w, item.h, item.up, item.band if item.kind == KIND_CORNER else 0) for item in editor.marks]
    return HotspotMarkupV1(int(editor.width), int(editor.height), str(editor.image), tuple(marks))


def commit_draft(scene) -> tuple:
    """Черновик -> текст, если принят: находки (пусто - записан). Непринятый черновик остаётся в сцене, текст - прежним."""

    editor = editor_of(scene)
    if editor is None or not editor.draft_of:
        return ()
    problems = write_markup_text(editor.draft_of, draft_markup(editor))
    editor.dirty = bool(problems)
    editor.problems = "; ".join(problem.text() for problem in problems)
    return problems


def load_draft(scene, text_name: str, *, discard: bool = False) -> str:
    """Текст -> черновик. Отказ по имени (пусто - загружено): непринятый черновик не затирается без `discard`."""

    editor = editor_of(scene)
    if editor.dirty and not discard:
        return MARKUP_DRAFT_UNSAVED
    markup, problems = read_markup_text(text_name)
    if markup is None:
        return problems[0].name
    _SUSPEND["commit"] = True
    try:
        editor.marks.clear()
        editor.draft_of, editor.width, editor.height, editor.image = text_name, int(markup.width), int(markup.height), str(markup.image)
        for mark in markup.marks:
            _append(editor, mark)
        editor.active = 0 if len(editor.marks) else -1
    finally:
        _SUSPEND["commit"] = False
    editor.dirty = bool(problems)
    editor.problems = "; ".join(problem.text() for problem in problems)
    return ""


def _append(editor, mark: HotspotMarkV1):
    item = editor.marks.add()
    item.mark_id, item.label, item.kind, item.up = int(mark.id), str(mark.name), mark.kind if mark.kind in KINDS else KIND_TRIM, mark.up if mark.up in UP_VECTORS else "+y"
    item.x, item.y, item.w, item.h, item.band = int(mark.x), int(mark.y), int(mark.w), int(mark.h), int(mark.band)
    return item


# --------------------------------------------------------------------------
# Операторы
# --------------------------------------------------------------------------


def prepare_markup(scene) -> tuple:
    """Материал декали и его картинка -> датаблок разметки (создан и связан, если не было) -> черновик. `(картинка | None, отказ, подробность)`.

    Непринятый черновик ДРУГОЙ разметки не затирается (`MARKUP_DRAFT_UNSAVED`); непринятый черновик ЭТОЙ же разметки остаётся как есть.
    """

    material, image, refusal, detail, _candidates = scene_material_image(scene)
    if image is None:
        return None, refusal, detail
    editor = editor_of(scene)
    target = linked_markup_name(material) or f"{image.name}.hotspot"
    if editor.dirty and editor.draft_of != target:
        return None, MARKUP_DRAFT_UNSAVED, f"the draft of {editor.draft_of!r} is not saved: fix it or Discard Draft and Reload in the mini editor"
    name, _created, problems = ensure_material_markup(material, image)
    if problems:
        return None, problems[0].name, "; ".join(problem.text() for problem in problems)
    if not (editor.dirty and editor.draft_of == name):
        refusal = load_draft(scene, name)
        if refusal:
            return None, refusal, f"markup {name!r} does not load"
    editor.material = material.name
    return image, "", ""


def _find_window(pointer):
    wm = bpy.context.window_manager
    return next((window for window in wm.windows if window.as_pointer() == pointer), None)


def _mini_area():
    """`(окно, область)` мини-редактора, если оно ещё открыто и его область - Image Editor; иначе `(None, None)`."""

    window = _find_window(_MINI.get("window"))
    area = None if window is None or window.screen.as_pointer() != _MINI.get("screen") else next((item for item in window.screen.areas if item.as_pointer() == _MINI.get("area")), None)
    if area is None or area.type != "IMAGE_EDITOR":
        return None, None
    return window, area


def area_key(window, screen, area) -> tuple:
    """Ключ области для показа меток: указатели окна, его экрана и области (`None` - нет)."""

    return tuple(None if item is None else item.as_pointer() for item in (window, screen, area))


def mini_key():
    """Ключ области мини-окна, пока оно открыто (окно среди окон, экран окна - тот же); иначе `None`."""

    if not _MINI.get("area"):
        return None
    window = _find_window(_MINI.get("window"))
    if window is None or window.screen.as_pointer() != _MINI.get("screen"):
        return None
    return _MINI.get("window"), _MINI.get("screen"), _MINI.get("area")


def overlay_mode(window, screen, area, editor) -> str:
    """Где эта область рисует метки (`hotspot_blender.overlay_where`): мини-окно, другой Image Editor по выбору пользователя, или нигде."""

    if editor is None or not editor.draft_of or area is None or area.type != "IMAGE_EDITOR":
        return ""
    key = area_key(window, screen, area)
    registered = (_MINI.get("window"), _MINI.get("screen"), _MINI.get("area"))
    live = mini_key() if key == registered else None  # окна перебираются только для самой области мини-окна
    shown = getattr(getattr(area.spaces.active, "image", None), "name", "")
    return overlay_where(key, live, shown, (editor.image,), bool(editor.show_in_uv_editor))


def is_mini_context(context) -> bool:
    """Контекст - область мини-окна разметки: только там боковая панель Hotspot и инструмент меток."""

    area = getattr(context, "area", None)
    return area is not None and overlay_mode(getattr(context, "window", None), getattr(context, "screen", None), area, editor_of(context.scene)) == OVERLAY_EDITOR


def window_title(editor) -> str:
    return f"{WINDOW_TITLE} — {editor.material or '?'} | {editor.draft_of}"


def _title_stopped(area, editor) -> None:
    """Заголовок мини-окна, пока инструмент не работает: заголовок остаётся (меню Image Editor не возвращаются), как запустить - словами."""

    if editor is not None and editor.draft_of:
        area.header_text_set(f"{window_title(editor)} | mark tool stopped: Edit Marks in the sidebar (N) starts it")
    area.tag_redraw()


def _open_window(context):
    """Новое маленькое окно с одной областью: временное окно Info (закрытие убирает его экран), иначе новое окно; `(окно, область)` или `(None, None)`."""

    wm = context.window_manager
    for module, name in (("screen", "info_log_show"), ("wm", "window_new")):
        before = {window.as_pointer() for window in wm.windows}
        try:
            getattr(getattr(bpy.ops, module), name)()
        except (AttributeError, RuntimeError) as exc:
            print(f"[CFTUV][Hotspot] {module}.{name} opened no window: {exc}", flush=True)
            continue
        fresh = [window for window in wm.windows if window.as_pointer() not in before]
        if fresh:
            window = fresh[0]
            return window, max(window.screen.areas, key=lambda item: item.width * item.height)
    return None, None


def open_mini_editor(context, image) -> str:
    """Окно мини-редактора с картинкой `image` (переиспользуется, если открыто) и запуск инструмента, когда область получит размеры; отказ по имени."""

    window, area = _mini_area()
    if area is None:
        window, area = _open_window(context)
        if area is None:
            return HOTSPOT_EDITOR_WINDOW_NOT_OPENED
        area.type = "IMAGE_EDITOR"
    _MINI.update(window=window.as_pointer(), screen=window.screen.as_pointer(), area=area.as_pointer())
    space = area.spaces.active
    space.image = image
    space.mode = "VIEW"
    space.show_region_ui = True
    if hasattr(space, "use_image_pin"):
        space.use_image_pin = True  # картинка не следует за активной гранью меша в Edit Mode
    for flag in ("show_region_toolbar", "show_region_tool_header"):
        if hasattr(space, flag):
            setattr(space, flag, False)
    editor = editor_of(context.scene)
    if editor is not None and area.as_pointer() not in _RUNNING:
        area.header_text_set(f"{window_title(editor)} | starting the mark tool")
    attempts = {"left": 40}
    window_pointer, area_pointer = window.as_pointer(), area.as_pointer()

    def start():
        return _start_when_ready(window_pointer, area_pointer, attempts)

    bpy.app.timers.register(start, first_interval=0.05)
    return ""


def _start_when_ready(window_pointer, area_pointer, attempts):
    """Таймер: область окна получила размеры -> вид вписан, вкладка Hotspot, инструмент запущен. Окно закрыто - тихо нечего делать (ничего не начато)."""

    window = _find_window(window_pointer)
    area = None if window is None else next((item for item in window.screen.areas if item.as_pointer() == area_pointer), None)
    if area is None or area.type != "IMAGE_EDITOR":
        return None
    region = _window_region(area)
    if region is None or region.width < 64 or region.height < 64:
        attempts["left"] -= 1
        if attempts["left"] > 0:
            return 0.05
        print(f"[CFTUV][Hotspot] {HOTSPOT_EDITOR_TOOL_NOT_STARTED}: the mini editor area has no size; click Edit Marks in its sidebar", flush=True)
        return None
    with bpy.context.temp_override(window=window, area=area, region=region):  # экран временного окна переопределять нельзя: он берётся из окна
        _fit_view(area, region)
        if area.as_pointer() not in _RUNNING:
            if bpy.ops.hotspotuv.hotspot_markup_edit.poll():
                bpy.ops.hotspotuv.hotspot_markup_edit("INVOKE_DEFAULT")
            else:
                print(f"[CFTUV][Hotspot] {HOTSPOT_EDITOR_TOOL_NOT_STARTED}: the mark tool does not poll in the mini editor area; click Edit Marks", flush=True)
                _title_stopped(area, editor_of(bpy.context.scene))
    for item in area.regions:
        if item.type == "UI":
            try:
                item.active_panel_category = HOTSPOTUV_PT_HotspotMarkup.bl_category
            except (AttributeError, TypeError) as exc:  # Blender без записи вкладки: панель на вкладке Hotspot, вкладка - щелчком
                print(f"[CFTUV][Hotspot] sidebar tab not switched: {exc}", flush=True)
    area.tag_redraw()
    return None


def _fit_view(area, region) -> None:
    """Картинка целиком в видимой части области: слева от перекрывающей боковой панели, с полями."""

    space = area.spaces.active
    image = space.image
    if image is None or not image.size[0] or not image.size[1]:
        return
    sidebar = next((item for item in area.regions if item.type == "UI" and item.width > 1 and space.show_region_ui), None)
    covered = 0 if sidebar is None else max(0, region.x + region.width - sidebar.x)  # перекрытие регионов выключено - 0
    covered = covered if region.width - covered > 128 else 0
    margin = 24
    zoom = min((region.width - covered - 2 * margin) / image.size[0], (region.height - 2 * margin) / image.size[1])
    bpy.ops.image.view_all(fit_view=True)
    bpy.ops.image.view_zoom_ratio(ratio=max(zoom, 1e-3))
    bpy.ops.image.view_pan(offset=(covered / 2.0 / max(zoom, 1e-3), 0.0))


class HOTSPOTUV_OT_HotspotMarkUp(bpy.types.Operator):
    bl_idname = "hotspotuv.hotspot_mark_up"
    bl_label = "Mark Up"
    bl_description = (
        "Open a mini Image Editor window with the texture of the decal material and the mark tool running. "
        "The markup text is created and linked to the material automatically"
    )
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        image, refusal, detail = prepare_markup(context.scene)
        if image is not None:
            refusal, detail = open_mini_editor(context, image), "no new window could be opened"
        if refusal:
            self.report({"ERROR"}, f"{refusal}: {detail}")
            return {"CANCELLED"}
        return {"FINISHED"}


class HOTSPOTUV_OT_HotspotMarkupDiscard(bpy.types.Operator):
    bl_idname = "hotspotuv.hotspot_markup_discard"
    bl_label = "Discard Draft and Reload"
    bl_description = "Throw the unsaved draft away and reload the last accepted markup from its text"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        editor = editor_of(context.scene)
        refusal = load_draft(context.scene, editor.draft_of, discard=True)
        if refusal:
            self.report({"ERROR"}, f"{refusal}: {editor.draft_of}")
            return {"CANCELLED"}
        return {"FINISHED"}


class HOTSPOTUV_OT_HotspotEditorClose(bpy.types.Operator):
    bl_idname = "hotspotuv.hotspot_editor_close"
    bl_label = "Close Mini Editor"
    bl_description = "Close this window; the accepted markup is already in its text, an unsaved draft stays in the scene"

    @classmethod
    def poll(cls, context):
        window = getattr(context, "window", None)
        return window is not None and window.as_pointer() == _MINI.get("window")

    def execute(self, context):
        bpy.ops.wm.window_close()
        return {"FINISHED"}


#: Регионы области, которые перекрывают изображение (боковая панель при перекрытии регионов) или лежат рядом: события над ними - не инструменту.
_PANEL_REGIONS = frozenset(("UI", "HEADER", "TOOL_HEADER", "TOOLS", "ASSET_SHELF", "ASSET_SHELF_HEADER", "FOOTER"))


def _window_region(area):
    return next((region for region in area.regions if region.type == "WINDOW"), None)


def _inside(region, x, y) -> bool:
    return region.width > 1 and region.height > 1 and region.x <= x < region.x + region.width and region.y <= y < region.y + region.height


def _over_image(area, region, event) -> bool:
    """Мышь над регионом изображения области и не над перекрывающей его боковой панелью или заголовком."""

    x, y = event.mouse_x, event.mouse_y
    if not _inside(region, x, y):
        return False
    return not any(item.type in _PANEL_REGIONS and _inside(item, x, y) for item in area.regions)


def _to_pixels(region, editor, mx, my):
    x, y = region.view2d.region_to_view(mx, my)
    return x * editor.width, y * editor.height


def _to_region(region, editor, px, py):
    return region.view2d.view_to_region(px / max(editor.width, 1), py / max(editor.height, 1), clip=False)


def _snap(value, grid, limit):
    return int(max(0, min(limit, round(value / grid) * grid)))


HANDLE_PIXELS = 7


def _hit(region, editor, mx, my):
    """`(индекс метки, ручка)`: ручка - пара сторон `(dx, dy)` из {-1, 0, 1} (у угла обе), `(0, 0)` - внутри; `(-1, None)` - мимо."""

    order = list(range(len(editor.marks)))
    if 0 <= editor.active < len(editor.marks):
        order.remove(editor.active)
        order.insert(0, editor.active)
    for index in order:
        mark = editor.marks[index]
        x0, y0 = _to_region(region, editor, mark.x, mark.y)
        x1, y1 = _to_region(region, editor, mark.x + mark.w, mark.y + mark.h)
        if not (x0 - HANDLE_PIXELS <= mx <= x1 + HANDLE_PIXELS and y0 - HANDLE_PIXELS <= my <= y1 + HANDLE_PIXELS):
            continue
        dx = -1 if abs(mx - x0) <= HANDLE_PIXELS else (1 if abs(mx - x1) <= HANDLE_PIXELS else 0)
        dy = -1 if abs(my - y0) <= HANDLE_PIXELS else (1 if abs(my - y1) <= HANDLE_PIXELS else 0)
        if index == editor.active or (dx, dy) == (0, 0):
            return index, (dx, dy)
    return -1, None


class HOTSPOTUV_OT_HotspotMarkupEdit(bpy.types.Operator):
    bl_idname = "hotspotuv.hotspot_markup_edit"
    bl_label = "Edit Hotspot Marks"
    bl_description = "Draw, select, move, resize, duplicate and delete marks on a pixel grid; every finished edit is validated and saved only when accepted"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        editor = editor_of(context.scene)
        area = getattr(context, "area", None)
        return area is not None and area.as_pointer() not in _RUNNING and editor is not None and editor.width > 0 and is_mini_context(context)

    def invoke(self, context, event):
        # Область, а не регион вызова: кнопка боковой панели вызывает из региона UI, координаты мыши считаются в регионе изображения.
        self._drag = None
        self._area = context.area
        self._area_pointer = context.area.as_pointer()  # после закрытия окна область освобождена: `cancel` держит только число
        _RUNNING.add(self._area_pointer)
        context.window_manager.modal_handler_add(self)
        self._header(context)
        return {"RUNNING_MODAL"}

    def _header(self, context):
        editor = editor_of(context.scene)
        state = f"NOT SAVED: {editor.problems}" if editor.dirty else "saved"
        self._area.header_text_set(f"{window_title(editor)} | grid {editor.grid} px ([ ]) | drag: new, move, resize | D duplicate | X delete | U up | Esc stop | {state}")
        self._area.tag_redraw()

    def _finish(self, context):
        _SUSPEND["commit"] = False
        _RUNNING.discard(self._area_pointer)
        _title_stopped(self._area, editor_of(context.scene))
        return {"FINISHED"}

    def cancel(self, context):
        """Окно закрыто (или файл загружен) во время работы инструмента: незаконченное перетаскивание откатывается, черновик проверяется и пишется."""

        editor = editor_of(context.scene)
        if self._drag is not None and editor is not None:
            self._restore(editor)
        commit_draft(context.scene)
        _SUSPEND["commit"] = False
        _RUNNING.discard(self._area_pointer)

    def modal(self, context, event):
        try:
            return self._modal(context, event)
        except Exception as exc:  # noqa: BLE001 - инструмент не оставляет флаг подвешенным; причина - владельцу
            self._drag = None
            commit_draft(context.scene)
            self.report({"ERROR"}, f"hotspot edit stopped: {type(exc).__name__}: {exc}")
            return self._finish(context)

    def _modal(self, context, event):
        editor = editor_of(context.scene)
        region = _window_region(self._area)
        if region is None:
            return self._finish(context)
        mx, my = event.mouse_x - region.x, event.mouse_y - region.y
        stop = event.value == "PRESS" and (event.type == "ESC" or (event.type == "RIGHTMOUSE" and _over_image(self._area, region, event)))
        if not stop and self._drag is None and not _over_image(self._area, region, event):
            return {"PASS_THROUGH"}  # боковая панель, заголовок, другая область: там правят поля, инструмент не мешает
        if stop:
            if self._drag is not None:
                self._restore(editor)
            commit_draft(context.scene)
            return self._finish(context)
        if event.type == "MOUSEMOVE" and self._drag is not None:
            self._move(editor, region, mx, my)
            self._area.tag_redraw()
            return {"RUNNING_MODAL"}
        if event.type == "LEFTMOUSE" and event.value == "PRESS":
            self._press(editor, region, mx, my)
            self._area.tag_redraw()
            return {"RUNNING_MODAL"}
        if event.type == "LEFTMOUSE" and event.value == "RELEASE" and self._drag is not None:
            self._release(editor)
            commit_draft(context.scene)
            self._header(context)
            return {"RUNNING_MODAL"}
        if event.value == "PRESS" and event.type in {"LEFT_BRACKET", "RIGHT_BRACKET"}:
            editor.grid = max(1, min(1024, editor.grid // 2 if event.type == "LEFT_BRACKET" else editor.grid * 2))
            self._header(context)
            return {"RUNNING_MODAL"}
        if event.value == "PRESS" and event.type in {"D", "X", "DEL", "U"} and 0 <= editor.active < len(editor.marks):
            {"D": duplicate_active, "X": delete_active, "DEL": delete_active, "U": cycle_up}[event.type](editor)
            commit_draft(context.scene)
            self._header(context)
            return {"RUNNING_MODAL"}
        return {"PASS_THROUGH"}

    def _press(self, editor, region, mx, my):
        index, handle = _hit(region, editor, mx, my)
        px, py = _to_pixels(region, editor, mx, my)
        _SUSPEND["commit"] = True
        if index >= 0:
            editor.active = index
            mark = editor.marks[index]
            self._drag = {"mode": "move" if handle == (0, 0) else "resize", "handle": handle, "start": (px, py), "rect": (mark.x, mark.y, mark.w, mark.h)}
            return
        grid = editor.grid
        sx, sy = _snap(px, grid, editor.width), _snap(py, grid, editor.height)
        item = _append(editor, HotspotMarkV1(next_mark_id(draft_markup(editor)), "", KIND_TRIM, sx, sy, 0, 0, "+y", 0))
        item.label = f"mark {item.mark_id}"
        editor.active = len(editor.marks) - 1
        self._drag = {"mode": "new", "anchor": (sx, sy), "rect": None}

    def _move(self, editor, region, mx, my):
        mark = editor.marks[editor.active]
        grid, width, height = editor.grid, editor.width, editor.height
        px, py = _to_pixels(region, editor, mx, my)
        drag = self._drag
        if drag["mode"] == "new":
            ax, ay = drag["anchor"]
            bx, by = _snap(px, grid, width), _snap(py, grid, height)
            mark.x, mark.y, mark.w, mark.h = min(ax, bx), min(ay, by), abs(bx - ax), abs(by - ay)
            return
        x, y, w, h = drag["rect"]
        dx, dy = px - drag["start"][0], py - drag["start"][1]
        if drag["mode"] == "move":
            mark.x = max(0, min(width - w, x + round(dx / grid) * grid))
            mark.y = max(0, min(height - h, y + round(dy / grid) * grid))
            return
        hx, hy = drag["handle"]
        left, bottom, right, top = x, y, x + w, y + h
        if hx < 0:
            left = min(_snap(x + dx, grid, width), right - 1)
        elif hx > 0:
            right = max(_snap(x + w + dx, grid, width), left + 1)
        if hy < 0:
            bottom = min(_snap(y + dy, grid, height), top - 1)
        elif hy > 0:
            top = max(_snap(y + h + dy, grid, height), bottom + 1)
        mark.x, mark.y, mark.w, mark.h = left, bottom, right - left, top - bottom

    def _release(self, editor):
        drag, self._drag = self._drag, None
        _SUSPEND["commit"] = False
        if drag["mode"] == "new" and 0 <= editor.active < len(editor.marks):
            mark = editor.marks[editor.active]
            if mark.w <= 0 or mark.h <= 0:
                delete_active(editor)  # щелчок без перетаскивания - не метка

    def _restore(self, editor):
        drag, self._drag = self._drag, None
        _SUSPEND["commit"] = False
        if drag["mode"] == "new":
            delete_active(editor)
        elif 0 <= editor.active < len(editor.marks):
            mark = editor.marks[editor.active]
            _SUSPEND["commit"] = True
            mark.x, mark.y, mark.w, mark.h = drag["rect"]
            _SUSPEND["commit"] = False


def duplicate_active(editor) -> None:
    """Копия активной метки со следующим номером, сдвинутая на свою толщину вдоль «вверх» (длина остаётся во всю текстуру)."""

    source = editor.marks[editor.active]
    ux, uy = UP_VECTORS[source.up]
    _SUSPEND["commit"] = True
    try:
        item = _append(editor, HotspotMarkV1(next_mark_id(draft_markup(editor)), f"{source.label} copy", source.kind, source.x + ux * source.w, source.y + uy * source.h, source.w, source.h, source.up, source.band))
    finally:
        _SUSPEND["commit"] = False
    editor.active = len(editor.marks) - 1
    del item


def delete_active(editor) -> None:
    _SUSPEND["commit"] = True
    try:
        editor.marks.remove(editor.active)
    finally:
        _SUSPEND["commit"] = False
    editor.active = min(editor.active, len(editor.marks) - 1)


def cycle_up(editor) -> None:
    mark = editor.marks[editor.active]
    _SUSPEND["commit"] = True
    try:
        mark.up = UP_ORDER[(UP_ORDER.index(mark.up) + 1) % len(UP_ORDER)]
    finally:
        _SUSPEND["commit"] = False


# --------------------------------------------------------------------------
# Показ
# --------------------------------------------------------------------------


def _segments(mark):
    """Отрезки меток в пикселях: `(контур, шов, стрелка, полоса)`."""

    x0, y0, x1, y1 = mark.x, mark.y, mark.x + mark.w, mark.y + mark.h
    outline = [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]
    ux, uy = UP_VECTORS.get(mark.up, (0, 1))
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    thickness = mark.h if uy else mark.w
    if mark.kind == KIND_CORNER:
        level = 0.0
    else:
        level = -thickness / 2.0
    if uy:
        seam = [((x0, cy + uy * level), (x1, cy + uy * level))]
    else:
        seam = [((cx + ux * level, y0), (cx + ux * level, y1))]
    tip = (cx + ux * thickness * 0.4, cy + uy * thickness * 0.4)
    tail = (cx - ux * thickness * 0.2, cy - uy * thickness * 0.2)
    side = (-uy * thickness * 0.12, ux * thickness * 0.12)
    back = (tip[0] - ux * thickness * 0.15, tip[1] - uy * thickness * 0.15)
    arrow = [(tail, tip), (tip, (back[0] + side[0], back[1] + side[1])), (tip, (back[0] - side[0], back[1] - side[1]))]
    band = []
    if mark.kind == KIND_CORNER and mark.band:
        half = mark.band / 2.0
        if uy:
            band = [((x0, cy - half), (x1, cy - half)), ((x0, cy + half), (x1, cy + half))]
        else:
            band = [((cx - half, y0), (cx - half, y1)), ((cx + half, y0), (cx + half, y1))]
    return outline, seam, arrow, band


def _draw_marks() -> None:
    import blf
    import gpu
    from gpu_extras.batch import batch_for_shader

    context = bpy.context
    editor = editor_of(context.scene)
    region = context.region
    if editor is None or editor.width <= 0 or region is None:
        return
    mode = overlay_mode(context.window, context.screen, context.area, editor)  # HOTSPOT_EDITOR_WINDOW_V1: только мини-окно (или выбор пользователя)
    if not mode:
        return
    shader = gpu.shader.from_builtin("UNIFORM_COLOR")
    gpu.state.blend_set("ALPHA")

    def fill(pairs, colour):
        if len(pairs) != 2:
            return
        (a, b), (c, d) = pairs
        corners = [_to_region(region, editor, *point) for point in (a, b, d, c)]
        batch = batch_for_shader(shader, "TRIS", {"pos": corners}, indices=((0, 1, 2), (0, 2, 3)))
        shader.uniform_float("color", colour)
        batch.draw(shader)

    def lines(pairs, colour, width):
        if not pairs:
            return
        points = [_to_region(region, editor, *point) for pair in pairs for point in pair]
        gpu.state.line_width_set(width)
        batch = batch_for_shader(shader, "LINES", {"pos": points})
        shader.uniform_float("color", colour)
        batch.draw(shader)

    for index, mark in enumerate(editor.marks):
        active = index == editor.active
        outline, seam, arrow, band = _segments(mark)
        body, tint, edge = mark_colours(mark.mark_id, active)
        fill([outline[0], (outline[2][1], outline[2][0])], body)
        fill(band, tint)
        lines(band, (*tint[:3], 0.9), 1.0)
        lines(outline, edge, 2.0 if active else 1.0)
        lines(seam, (1.0, 0.3, 0.2, 1.0), 3.0)
        lines(arrow, (0.4, 1.0, 0.4, 1.0), 2.0)
        x, y = _to_region(region, editor, mark.x, mark.y + mark.h)
        blf.position(0, x + 4, y - 16, 0)
        blf.size(0, 13)
        blf.enable(0, blf.SHADOW)  # подпись читается и на заливке
        blf.shadow(0, 3, 0.0, 0.0, 0.0, 0.9)
        blf.color(0, 1.0, 1.0, 1.0, 1.0)
        blf.draw(0, f"{mark.mark_id}: {mark.label} ({mark.kind}, up {mark.up})")
        blf.disable(0, blf.SHADOW)
    if mode == OVERLAY_EDITOR:
        _draw_state(blf, context.area, editor)
    gpu.state.line_width_set(1.0)
    gpu.state.blend_set("NONE")


def _draw_state(blf, area, editor) -> None:
    """Строки внизу слева: подсказки клавиш, пока инструмент работает (иначе - как его запустить), и состояние черновика по имени."""

    running = area is not None and area.as_pointer() in _RUNNING
    hint = (0.85, 0.85, 0.85, 1.0)
    if running:
        lines = [(f"Mark tool, grid {editor.grid} px", hint)] + [("  |  ".join(KEY_HINTS[at:at + 2]), hint) for at in range(0, len(KEY_HINTS), 2)]
    else:
        lines = [("Mark tool stopped: Edit Marks in the sidebar (N) starts it", (1.0, 0.8, 0.3, 1.0))]
    if editor.dirty:
        lines.append((f"NOT SAVED (the text keeps the last valid markup): {editor.problems[:160]}", (1.0, 0.4, 0.3, 1.0)))
    else:
        lines.append((f"Saved to {editor.draft_of}", (0.5, 1.0, 0.5, 1.0)))
    blf.size(0, 12)
    for row, (text, colour) in enumerate(reversed(lines)):
        blf.position(0, 12, 12 + 18 * row, 0)
        blf.color(0, *colour)
        blf.draw(0, text)


# --------------------------------------------------------------------------
# Боковая панель Image Editor
# --------------------------------------------------------------------------


class HOTSPOTUV_UL_HotspotMarks(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        row = layout.row(align=True)
        row.label(text=f"{item.mark_id}")
        row.prop(item, "label", text="", emboss=False)
        row.label(text=f"{item.kind} {item.up}")


class HOTSPOTUV_PT_HotspotMarkup(bpy.types.Panel):
    bl_space_type = "IMAGE_EDITOR"
    bl_region_type = "UI"
    bl_category = "Hotspot"
    bl_label = "Hotspot Markup"

    @classmethod
    def poll(cls, context):
        return is_mini_context(context)  # HOTSPOT_EDITOR_WINDOW_V1: вкладка Hotspot есть только в мини-окне разметки

    def draw(self, context):
        layout = self.layout
        editor = editor_of(context.scene)
        column = layout.column(align=True)
        column.label(text=f"Material {editor.material or '?'}", icon="MATERIAL")
        column.label(text=f"Markup {editor.draft_of}", icon="TEXT")
        column.label(text=f"Texture {editor.image}: {editor.width} x {editor.height} px", icon="IMAGE_DATA")
        shown = getattr(context.space_data, "image", None)
        if shown is not None and tuple(shown.size) != (editor.width, editor.height) and shown.size[0]:
            column.label(text=f"Shown image {shown.name} is {shown.size[0]} x {shown.size[1]} px", icon="ERROR")
        self._draw_state(layout, context, editor)
        layout.prop(editor, "show_in_uv_editor")
        layout.template_list("HOTSPOTUV_UL_HotspotMarks", "", editor, "marks", editor, "active", rows=3)
        if 0 <= editor.active < len(editor.marks):
            mark = editor.marks[editor.active]
            column = layout.column(align=True)
            for name in ("mark_id", "label", "kind", "up"):
                column.prop(mark, name)
            if mark.kind == KIND_CORNER:
                column.prop(mark, "band")
            for pair in (("x", "y"), ("w", "h")):
                row = layout.row(align=True)
                for name in pair:
                    row.prop(mark, name)
        if HOTSPOTUV_OT_HotspotEditorClose.poll(context):
            layout.operator(HOTSPOTUV_OT_HotspotEditorClose.bl_idname, icon="CHECKMARK")

    @staticmethod
    def _draw_state(layout, context, editor) -> None:
        """Состояние черновика по имени и запуск инструмента - видны без поиска (подсказки клавиш - строки внизу окна)."""

        if editor.dirty:
            box = layout.box().column(align=True)
            box.label(text="NOT SAVED (text keeps last valid)", icon="ERROR")
            for line in editor.problems.split("; ")[:6]:
                name, _colon, detail = line.partition(": ")
                box.label(text=name)
                if detail:
                    box.label(text=f"   {detail[:120]}")
            box.operator(HOTSPOTUV_OT_HotspotMarkupDiscard.bl_idname, icon="TRASH")
        else:
            layout.label(text=f"Valid, saved to {editor.draft_of}", icon="CHECKMARK")
        area = getattr(context, "area", None)
        row = layout.row()
        if area is not None and area.as_pointer() in _RUNNING:
            row.label(text="Tool running", icon="RADIOBUT_ON")  # клавиши - строки внизу окна и заголовок
        else:
            row.operator(HOTSPOTUV_OT_HotspotMarkupEdit.bl_idname, text="Edit Marks", icon="SELECT_SET")
        row.prop(editor, "grid")


_CLASSES = (
    HOTSPOTUV_HotspotMark,
    HOTSPOTUV_HotspotEditor,
    HOTSPOTUV_OT_HotspotMarkUp,
    HOTSPOTUV_OT_HotspotMarkupDiscard,
    HOTSPOTUV_OT_HotspotEditorClose,
    HOTSPOTUV_OT_HotspotMarkupEdit,
    HOTSPOTUV_UL_HotspotMarks,
    HOTSPOTUV_PT_HotspotMarkup,
)
_HANDLE: dict = {}


@bpy.app.handlers.persistent
def _forget_windows(*_args) -> None:
    """После загрузки файла указатели окна и областей прежнего сеанса ничего не значат (адрес может достаться новому окну)."""

    _MINI.clear()
    _RUNNING.clear()


def register_editor() -> None:
    for cls in _CLASSES:
        if getattr(cls, "is_registered", False):
            bpy.utils.unregister_class(cls)
        bpy.utils.register_class(cls)
    setattr(bpy.types.Scene, EDITOR_ATTRIBUTE, PointerProperty(type=HOTSPOTUV_HotspotEditor))
    if "draw" not in _HANDLE:
        _HANDLE["draw"] = bpy.types.SpaceImageEditor.draw_handler_add(_draw_marks, (), "WINDOW", "POST_PIXEL")
    if _forget_windows not in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.append(_forget_windows)


def unregister_editor() -> None:
    if _forget_windows in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.remove(_forget_windows)
    handle = _HANDLE.pop("draw", None)
    if handle is not None:
        bpy.types.SpaceImageEditor.draw_handler_remove(handle, "WINDOW")
    if hasattr(bpy.types.Scene, EDITOR_ATTRIBUTE):
        delattr(bpy.types.Scene, EDITOR_ATTRIBUTE)
    for cls in reversed(_CLASSES):
        if getattr(cls, "is_registered", False):
            bpy.utils.unregister_class(cls)


__all__ = (
    "EDITOR_ATTRIBUTE",
    "HOTSPOT_EDITOR_TOOL_NOT_STARTED",
    "HOTSPOT_EDITOR_WINDOW_NOT_OPENED",
    "KEY_HINTS",
    "MARKUP_DRAFT_UNSAVED",
    "WINDOW_TITLE",
    "area_key",
    "commit_draft",
    "draft_markup",
    "is_mini_context",
    "load_draft",
    "mini_key",
    "open_mini_editor",
    "overlay_mode",
    "window_title",
    "prepare_markup",
    "register_editor",
    "unregister_editor",
)
