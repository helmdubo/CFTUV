"""Редактор разметки Hotspot в Image Editor (HOTSPOT_MARKUP_EDITOR_V1, HOTSPOT_PANEL_EDITOR_V1, HOTSPOT_EDITOR_UX_V1): мини-окно, показ меток, инструмент.

МИНИ-ОКНО. «Mark Up» панели декали (`hotspotuv.hotspot_mark_up`) берёт материал декали и картинку из его узлов, сам создаёт и связывает
датаблок разметки, загружает черновик и открывает отдельное маленькое окно: временное окно `screen.info_log_show` (одна область, закрытие
убирает его экран), область переключается в Image Editor с этой картинкой, вид вписан, инструмент запущен и работает, пока окно открыто
(закрытие окна - конец, другого «стопа» нет). Повторное нажатие переиспользует окно. Боковой панели нет (HOTSPOT_EDITOR_UX_V1): имя, размер,
вид и «вверх» каждой метки нарисованы на ней самой. Заголовок области - свой (`draw_mini_header` вместо меню Image Editor): имя окна,
выпадающий список подложки (HOTSPOT_BACKDROP_V1: картинки материала; разметка в пикселях своей текстуры, поэтому картинка другого размера при
метках - отказ `HOTSPOT_BACKDROP_SIZE_MISMATCH`, без меток разметка переходит в её размер), меню Markup («Show marks in UV Editor», сброс
черновика), сетка и состояние: saved / NOT SAVED с именами находок, отказы по именам.

ЧЕРНОВИК И ТЕКСТ. Правится черновик - коллекция меток сцены (`hotspotuv_hotspot_editor`, она сохраняется вместе с .blend). После каждой правки
черновик проверяется (`hotspot_markup.save_markup`): принятый пишется в текстовый датаблок, непринятый НЕ пишется - текст остаётся прежним
принятым, черновик остаётся в сцене, панель и заголовок называют находки по именам. Ничего не теряется молча: открыть разметку другого
материала поверх непринятого черновика - отказ `MARKUP_DRAFT_UNSAVED`, пока черновик не исправлен или явно не отброшен.

ИНСТРУМЕНТ (`hotspotuv.hotspot_markup_edit`, модальный, в области Image Editor): перетаскивание по пустому месту - новый прямоугольник по сетке
пикселей, щелчок по метке - выбор, перетаскивание внутри - перенос, за ручку (угол, середина стороны) - размер, у выбранной corner-метки за
край полосы - ширина полосы (симметрично средней линии, по сетке, `band_from_edge`). Ctrl + щелчок по метке - поворот на 90° по часовой
(`rotated_mark`, отказ MARK_ROTATE_BLOCKED по имени), Ctrl + правая кнопка - разворот стрелки «вверх» (`flipped_mark`), правая кнопка по метке -
меню метки (Rotate, Flip Up, Rename, Kind, Delete). Сетка: Ctrl + `+` / Ctrl + `-` (основная и цифровая клавиатура) и `[` / `]` - мельче /
крупнее. `D` - копия, `X`/`Delete` - удаление, `U` - разворот стрелки, `Esc` - отмена начатого перетаскивания. Колесо и средняя кнопка
проходят в Image Editor, как и всё над заголовком.

ОТДЕЛЬНОЕ ОКНО (HOTSPOT_EDITOR_WINDOW_V1): метки, свой заголовок и инструмент есть ТОЛЬКО в области мини-окна - ключ `(окно, экран, область)`
из указателей, окно проверяется среди открытых, загрузка файла ключ очищает. Второй Image Editor с той же картинкой и UV Editor декали в Edit
Mode меток не показывают, пока пользователь сам не включит «Show marks in UV Editor» (меню Markup заголовка, по умолчанию выключено); их
заголовок - обычный заголовок Image Editor. Картинка мини-окна закреплена.

ПОКАЗ (обработчик `POST_PIXEL` Image Editor): полупрозрачная заливка своего оттенка у каждой метки (`mark_colours`: оттенок от id), контур,
номер, имя, вид, «вверх» и размер (с тенью), стрелка «вверх», нижний край (шов `trim`) или средняя линия (шов `corner`) выделены, у `corner` -
центральная полоса `band` контрастным тоном того же оттенка, у выбранной corner-метки - ручки краёв полосы. Внизу слева - клавиши и подробности
находок.
"""

from __future__ import annotations

import bpy
from bpy.props import BoolProperty, CollectionProperty, EnumProperty, IntProperty, PointerProperty, StringProperty

from .hotspot_blender import (
    BACKDROP_REBASE,
    HOTSPOT_MATERIAL_HAS_NO_IMAGE,
    OVERLAY_EDITOR,
    SETTINGS_ATTRIBUTE,
    backdrop_label,
    backdrop_switch,
    ensure_material_markup,
    linked_markup_name,
    material_images,
    overlay_where,
    read_markup_text,
    scene_material_image,
    write_markup_text,
)
from .hotspot_markup import (
    KIND_CORNER,
    KIND_TRIM,
    KINDS,
    UP_OPPOSITE,
    UP_VECTORS,
    HotspotMarkV1,
    HotspotMarkupV1,
    band_from_edge,
    grid_key_step,
    grid_step,
    mark_colours,
    next_mark_id,
    problem_names,
    rotated_mark,
)
from .hotspot_refresh import record_mark_floor, schedule_refresh

EDITOR_ATTRIBUTE = "hotspotuv_hotspot_editor"
MARKUP_DRAFT_UNSAVED = "MARKUP_DRAFT_UNSAVED"
#: Окно мини-редактора не открылось (ни временное окно, ни новое): нажатие отказывает по имени.
HOTSPOT_EDITOR_WINDOW_NOT_OPENED = "HOTSPOT_EDITOR_WINDOW_NOT_OPENED"
#: Область окна не получила размеров за отведённое время: инструмент не запущен, причина - в консоли и в заголовке (кнопка Start Mark Tool).
HOTSPOT_EDITOR_TOOL_NOT_STARTED = "HOTSPOT_EDITOR_TOOL_NOT_STARTED"
#: Инструмент остановлен исключением: причина - в заголовке и внизу окна, кнопка Start Mark Tool запускает его снова.
HOTSPOT_EDITOR_TOOL_STOPPED = "HOTSPOT_EDITOR_TOOL_STOPPED"
UP_ORDER = ("+y", "-y", "+x", "-x")
#: Подсказки клавиш инструмента (строки внизу слева мини-окна, по две в строке).
KEY_HINTS = (
    "Drag on empty: new mark",
    "Drag a mark: move it",
    "Drag its edge or corner: resize",
    "Corner: drag a band edge: band width",
    "Ctrl+click a mark: rotate 90° clockwise",
    "Ctrl+right click: flip up (U)",
    "Right click a mark: Rotate, Rename, Kind, Delete",
    "Ctrl + / Ctrl - or [ / ]: finer / coarser grid",
    "D duplicate, X delete",
    "Esc: cancel a drag; close the window to finish",
)
#: Окно, его экран и область мини-редактора (указатели; HOTSPOT_EDITOR_WINDOW_V1: метки, заголовок и инструмент - только там) и области с
#: работающим инструментом. Закрытое окно не находится среди окон, загрузка файла очищает запись.
_MINI: dict = {}
#: Имя окна разметки - первая строка заголовка мини-окна (вместо меню Image Editor: выбор картинки, режим UV, тип области).
WINDOW_TITLE = "CFTUV Hotspot Markup"
#: HOTSPOT_BACKDROP_V1: мини-окно показывает не выбранную подложку (картинку сменили в обход списка): меток нет, строка и кнопка возврата.
HOTSPOT_BACKDROP_CHANGED = "HOTSPOT_BACKDROP_CHANGED"
#: Ручка края полосы corner (HOTSPOT_EDITOR_UX_V1) - отдельный вид захвата, не сторона прямоугольника.
BAND_HANDLE = "band"
#: Меню правой кнопки по метке.
MARK_MENU = "HOTSPOTUV_MT_hotspot_mark_context"
#: Пункты списка подложек (Blender не копирует строки динамического перечня).
_BACKDROP_ITEMS: list = []
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


def backdrop_images(editor) -> tuple:
    """Картинки материала черновика, годные в подложку (правило `hotspot_blender.material_images`: Base Color узла Principled первой)."""

    return material_images(bpy.data.materials.get(editor.material) if editor.material else None)


def _backdrop_items(self, _context):
    size = (int(self.width), int(self.height))
    _BACKDROP_ITEMS[:] = [(image.name, backdrop_label(image, size), "Image Texture of the material shown under the marks", index) for index, image in enumerate(backdrop_images(self))]
    _BACKDROP_ITEMS[:] = _BACKDROP_ITEMS or [("-", "no image", "the material has no Image Texture node with an image", 0)]
    return _BACKDROP_ITEMS


def _backdrop_get(self) -> int:
    names = [image.name for image in backdrop_images(self)]
    return names.index(self.backdrop) if self.backdrop in names else 0


def _backdrop_set(self, value) -> None:
    images = backdrop_images(self)
    if 0 <= int(value) < len(images):
        switch_backdrop(self.id_data, images[int(value)].name)


class HOTSPOTUV_HotspotEditor(bpy.types.PropertyGroup):
    draft_of: StringProperty(name="Draft of", description="Text datablock the draft was loaded from and is saved to")
    material: StringProperty(name="Material", description="Material whose markup the draft is (shown in the mini editor)")
    width: IntProperty(name="Texture width", min=0)
    height: IntProperty(name="Texture height", min=0)
    image: StringProperty(name="Image")
    next_id: IntProperty(name="Next mark id", min=0, description="HOTSPOT_MARK_ID_MONOTONIC_V1: id of the next new mark; ids of deleted marks are never given again")
    marks: CollectionProperty(type=HOTSPOTUV_HotspotMark)
    active: IntProperty(name="Active mark", default=-1)
    grid: IntProperty(name="Grid", default=8, min=1, max=1024, description="Pixel grid of the edit tool (Ctrl + and Ctrl -, or [ and ], make it finer and coarser)")
    dirty: BoolProperty(name="Unsaved draft", default=False)
    problems: StringProperty(name="Problems")
    notice: StringProperty(name="Notice", description="Named refusal of the last edit of the tool (MARK_ROTATE_BLOCKED, a stopped tool); cleared by the next edit")
    backdrop: StringProperty(name="Backdrop", description="Image shown under the marks in the mini editor (same size as the markup's texture)")
    backdrop_problem: StringProperty(name="Backdrop problem", description="Named refusal of the last backdrop switch")
    backdrop_choice: EnumProperty(
        name="Backdrop",
        items=_backdrop_items,
        get=_backdrop_get,
        set=_backdrop_set,
        description="Image Texture of the material shown under the marks: Base Color of the Principled node first. Another size than the markup's is refused while it has marks",
    )
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
    return HotspotMarkupV1(int(editor.width), int(editor.height), str(editor.image), tuple(marks), int(editor.next_id))


def new_mark_id(editor) -> int:
    """Номер новой метки черновика (HOTSPOT_MARK_ID_MONOTONIC_V1): счётчик идёт только вперёд, номер удалённой метки не достаётся новой."""

    mark_id = next_mark_id(draft_markup(editor))
    editor.next_id = mark_id + 1
    return mark_id


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
        # Разметка без счётчика (прежняя): пол - и номера, на которые ссылаются декали файла (удалённая метка помнится только там).
        editor.next_id = next_mark_id(markup, record_mark_floor(text_name))
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
    action, refusal, detail = backdrop_switch(tuple(image.size[:2]), (editor.width, editor.height), len(editor.marks))
    if refusal:  # черновик (а не текст) решает: у непринятого черновика могут быть метки, которых в тексте ещё нет
        return None, refusal, detail
    editor.material = material.name
    if action == BACKDROP_REBASE:
        _rebase(scene, editor, image)
    editor.backdrop, editor.backdrop_problem = image.name, ""
    return image, "", ""


def _rebase(scene, editor, image) -> None:
    """Разметка без меток переходит в размер картинки (HOTSPOT_BACKDROP_V1): сдвигаться нечему; текст пишется проверенным."""

    print(f"[CFTUV][Hotspot] markup {editor.draft_of!r} has no marks: its texture becomes {image.name!r} ({image.size[0]} x {image.size[1]} px), was {editor.width} x {editor.height}", flush=True)
    editor.width, editor.height, editor.image = int(image.size[0]), int(image.size[1]), image.name
    commit_draft(scene)


def switch_backdrop(scene, image_name: str) -> str:
    """Подложка мини-окна - картинка материала `image_name` (список в боковой панели): отказ по имени (пусто - показана), он же в `backdrop_problem`.

    Тот же размер, что у разметки, - показывается, метки и текст не меняются (имя картинки разметки тоже: подложка - только показ). Разметка без
    меток переходит в размер картинки. С метками другой размер - `HOTSPOT_BACKDROP_SIZE_MISMATCH`, подложка остаётся прежней."""

    editor = editor_of(scene)
    image = next((item for item in backdrop_images(editor) if item.name == image_name), None)
    if image is None:
        refusal, detail = HOTSPOT_MATERIAL_HAS_NO_IMAGE, f"material {editor.material!r} has no Image Texture with image {image_name!r}"
    else:
        action, refusal, detail = backdrop_switch(tuple(image.size[:2]), (editor.width, editor.height), len(editor.marks))
    if refusal:
        editor.backdrop_problem = f"{refusal}: {image_name}: {detail}"
        print(f"[CFTUV][Hotspot] {editor.backdrop_problem}", flush=True)
        return refusal
    if action == BACKDROP_REBASE:
        _rebase(scene, editor, image)
    editor.backdrop_problem = ""
    show_backdrop(scene, image)
    return ""


def show_backdrop(scene, image) -> None:
    """Подложка `image` в области мини-окна; выбор помнится настройками (следующий «Mark Up» откроет её)."""

    editor = editor_of(scene)
    editor.backdrop = image.name
    settings = getattr(scene, SETTINGS_ATTRIBUTE, None)
    if settings is not None and settings.image_name != image.name:
        settings.image_name = image.name
    window, area = _mini_area()
    if area is None:
        return
    space = area.spaces.active
    resized = space.image is None or tuple(space.image.size[:2]) != tuple(image.size[:2])
    space.image = image
    if resized:  # вид вписывается заново таймером: из обработчика свойства операторы вида не зовутся
        window_pointer, area_pointer = window.as_pointer(), area.as_pointer()
        bpy.app.timers.register(lambda: _refit(window_pointer, area_pointer), first_interval=0.05)
    area.tag_redraw()


def _refit(window_pointer, area_pointer):
    window = _find_window(window_pointer)
    area = None if window is None else next((item for item in window.screen.areas if item.as_pointer() == area_pointer), None)
    region = None if area is None else _window_region(area)
    if region is not None and region.width > 64:
        with bpy.context.temp_override(window=window, area=area, region=region):
            _fit_view(area, region)
    return None


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
    return overlay_where(key, live, shown, (editor.image, editor.backdrop), bool(editor.show_in_uv_editor))


def is_mini_context(context) -> bool:
    """Контекст - область мини-окна разметки: только там боковая панель Hotspot и инструмент меток."""

    area = getattr(context, "area", None)
    return area is not None and overlay_mode(getattr(context, "window", None), getattr(context, "screen", None), area, editor_of(context.scene)) == OVERLAY_EDITOR


def window_title(editor) -> str:
    return f"{WINDOW_TITLE} — {editor.material or '?'}"


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
    if hasattr(space, "use_image_pin"):
        space.use_image_pin = True  # картинка не следует за активной гранью меша в Edit Mode
    for flag in ("show_region_ui", "show_region_toolbar", "show_region_tool_header"):  # HOTSPOT_EDITOR_UX_V1: боковой панели нет
        if hasattr(space, flag):
            setattr(space, flag, False)
    if hasattr(space, "show_region_header"):
        space.show_region_header = True
    area.header_text_set(None)  # заголовок рисует `draw_mini_header`, текст поверх него не нужен
    attempts = {"left": 40}
    window_pointer, area_pointer = window.as_pointer(), area.as_pointer()

    def start():
        return _start_when_ready(window_pointer, area_pointer, attempts)

    bpy.app.timers.register(start, first_interval=0.05)
    return ""


def _start_when_ready(window_pointer, area_pointer, attempts):
    """Таймер: область окна получила размеры -> вид вписан, инструмент запущен. Окно закрыто - тихо нечего делать (ничего не начато).
    Не запустился - имя в консоли и в заголовке окна (кнопка Start Mark Tool)."""

    window = _find_window(window_pointer)
    area = None if window is None else next((item for item in window.screen.areas if item.as_pointer() == area_pointer), None)
    if area is None or area.type != "IMAGE_EDITOR":
        return None
    region = _window_region(area)
    if region is None or region.width < 64 or region.height < 64:
        attempts["left"] -= 1
        if attempts["left"] > 0:
            return 0.05
        print(f"[CFTUV][Hotspot] {HOTSPOT_EDITOR_TOOL_NOT_STARTED}: the mini editor area has no size; Start Mark Tool in its header", flush=True)
        area.tag_redraw()
        return None
    with bpy.context.temp_override(window=window, area=area, region=region):  # экран временного окна переопределять нельзя: он берётся из окна
        _fit_view(area, region)
        if area.as_pointer() not in _RUNNING:
            if bpy.ops.hotspotuv.hotspot_markup_edit.poll():
                bpy.ops.hotspotuv.hotspot_markup_edit("INVOKE_DEFAULT")
            else:
                print(f"[CFTUV][Hotspot] {HOTSPOT_EDITOR_TOOL_NOT_STARTED}: the mark tool does not poll in the mini editor area; Start Mark Tool in its header", flush=True)
    area.tag_redraw()
    return None


def _fit_view(area, region) -> None:
    """Картинка целиком в видимой части области (перекрывающая боковая панель, если пользователь её открыл, не закрывает картинку), с полями."""

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


class HOTSPOTUV_OT_HotspotBackdropShow(bpy.types.Operator):
    bl_idname = "hotspotuv.hotspot_backdrop_show"
    bl_label = "Show Backdrop"
    bl_description = "Show the picked backdrop image in this window again (another image was opened over it)"

    @classmethod
    def poll(cls, context):
        return is_mini_context(context)

    def execute(self, context):
        editor = editor_of(context.scene)
        refusal = switch_backdrop(context.scene, editor.backdrop)
        if refusal:
            self.report({"ERROR"}, editor.backdrop_problem)
            return {"CANCELLED"}
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


def mark_of(item) -> HotspotMarkV1:
    """Метка черновика (элемент коллекции) как `HotspotMarkV1` - для чистых правок `hotspot_markup`."""

    return HotspotMarkV1(item.mark_id, item.label, item.kind, item.x, item.y, item.w, item.h, item.up, item.band)


def _band_edges(mark) -> tuple:
    """Края полосы corner в пикселях: `(ось, (координата, координата))`; ось `"y"` - края - горизонтальные линии (up ±y)."""

    if mark.up in ("+y", "-y"):
        centre = mark.y + mark.h / 2.0
        return "y", (centre - mark.band / 2.0, centre + mark.band / 2.0)
    centre = mark.x + mark.w / 2.0
    return "x", (centre - mark.band / 2.0, centre + mark.band / 2.0)


def _near_band(region, editor, mark, mx, my) -> bool:
    """Мышь у края полосы выбранной corner-метки (в пределах длины метки): ручка ширины полосы, а не перенос."""

    if mark.kind != KIND_CORNER:
        return False
    axis, edges = _band_edges(mark)
    for edge in edges:
        if axis == "y":
            x0, ey = _to_region(region, editor, mark.x, edge)
            x1, _ey = _to_region(region, editor, mark.x + mark.w, edge)
            if x0 <= mx <= x1 and abs(my - ey) <= HANDLE_PIXELS:
                return True
        else:
            ex, y0 = _to_region(region, editor, edge, mark.y)
            _ex, y1 = _to_region(region, editor, edge, mark.y + mark.h)
            if y0 <= my <= y1 and abs(mx - ex) <= HANDLE_PIXELS:
                return True
    return False


def _hit(region, editor, mx, my):
    """`(индекс метки, ручка)`: ручка - пара сторон `(dx, dy)` из {-1, 0, 1} (у угла обе), `(0, 0)` - внутри, `BAND_HANDLE` - край полосы
    выбранной corner-метки (стороны прямоугольника важнее); `(-1, None)` - мимо."""

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
        if index == editor.active and (dx, dy) == (0, 0) and _near_band(region, editor, mark, mx, my):
            return index, BAND_HANDLE
        if index == editor.active or (dx, dy) == (0, 0):
            return index, (dx, dy)
    return -1, None


def _mark_at(region, editor, mx, my) -> int:
    """Индекс метки под мышью (выбранная первой, внутри прямоугольника) для правой кнопки и Ctrl + щелчка; `-1` - пусто."""

    order = list(range(len(editor.marks)))
    if 0 <= editor.active < len(editor.marks):
        order.remove(editor.active)
        order.insert(0, editor.active)
    for index in order:
        mark = editor.marks[index]
        x0, y0 = _to_region(region, editor, mark.x, mark.y)
        x1, y1 = _to_region(region, editor, mark.x + mark.w, mark.y + mark.h)
        if x0 <= mx <= x1 and y0 <= my <= y1:
            return index
    return -1


class HOTSPOTUV_OT_HotspotMarkupEdit(bpy.types.Operator):
    bl_idname = "hotspotuv.hotspot_markup_edit"
    bl_label = "Start Mark Tool"
    bl_description = "Draw, select, move, resize, rotate, flip, duplicate and delete marks on a pixel grid; every finished edit is validated and saved only when accepted. Runs while the window is open"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        editor = editor_of(context.scene)
        area = getattr(context, "area", None)
        return area is not None and area.as_pointer() not in _RUNNING and editor is not None and editor.width > 0 and is_mini_context(context)

    def invoke(self, context, event):
        # Область, а не регион вызова: кнопка заголовка вызывает из региона HEADER, координаты мыши считаются в регионе изображения.
        self._drag = None
        self._area = context.area
        self._area_pointer = context.area.as_pointer()  # после закрытия окна область освобождена: `cancel` держит только число
        _RUNNING.add(self._area_pointer)
        editor = editor_of(context.scene)
        if editor.notice.startswith((HOTSPOT_EDITOR_TOOL_STOPPED, HOTSPOT_EDITOR_TOOL_NOT_STARTED)):
            editor.notice = ""
        context.window_manager.modal_handler_add(self)
        self._area.tag_redraw()
        return {"RUNNING_MODAL"}

    def _finish(self, context):
        _SUSPEND["commit"] = False
        _RUNNING.discard(self._area_pointer)
        self._area.tag_redraw()
        return {"FINISHED"}

    def cancel(self, context):
        """Окно закрыто (или файл загружен) во время работы инструмента: незаконченное перетаскивание откатывается, черновик проверяется и пишется."""

        editor = editor_of(context.scene)
        if self._drag is not None and editor is not None:
            self._restore(editor)
        problems = commit_draft(context.scene)
        _SUSPEND["commit"] = False
        _RUNNING.discard(self._area_pointer)
        if editor is not None and editor.draft_of:
            schedule_refresh(editor.draft_of, problems)  # HOTSPOT_REFRESH_ON_CLOSE_V1: декали разметки - таймером, одним шагом отмены

    def modal(self, context, event):
        try:
            return self._modal(context, event)
        except Exception as exc:  # noqa: BLE001 - инструмент не оставляет флаг подвешенным; причина - владельцу, по имени
            self._drag = None
            commit_draft(context.scene)
            editor = editor_of(context.scene)
            editor.notice = f"{HOTSPOT_EDITOR_TOOL_STOPPED}: {type(exc).__name__}: {exc}"
            print(f"[CFTUV][Hotspot] {editor.notice}", flush=True)
            self.report({"ERROR"}, editor.notice)
            return self._finish(context)

    def _modal(self, context, event):
        editor = editor_of(context.scene)
        region = _window_region(self._area)
        if region is None:
            return self._finish(context)
        if event.value == "PRESS":
            step = grid_key_step(event.type, event.ctrl)
            if step:  # сетка - где бы ни была мышь в окне
                editor.grid = grid_step(editor.grid, step)
                self._area.tag_redraw()
                return {"RUNNING_MODAL"}
            if event.type == "ESC":  # Esc отменяет начатое перетаскивание; инструмент живёт, пока открыто окно
                if self._drag is not None:
                    self._restore(editor)
                    commit_draft(context.scene)
                self._area.tag_redraw()
                return {"RUNNING_MODAL"}
        mx, my = event.mouse_x - region.x, event.mouse_y - region.y
        if self._drag is None and not _over_image(self._area, region, event):
            return {"PASS_THROUGH"}  # заголовок, другая область: там меню и список подложки, инструмент не мешает
        if event.type == "MOUSEMOVE" and self._drag is not None:
            self._move(editor, region, mx, my)
            self._area.tag_redraw()
            return {"RUNNING_MODAL"}
        if event.type in {"LEFTMOUSE", "RIGHTMOUSE"} and event.value == "PRESS" and (event.ctrl or event.type == "RIGHTMOUSE"):
            return self._on_mark(context, editor, region, mx, my, event)
        if event.type == "LEFTMOUSE" and event.value == "PRESS":
            self._press(editor, region, mx, my)
            self._area.tag_redraw()
            return {"RUNNING_MODAL"}
        if event.type == "LEFTMOUSE" and event.value == "RELEASE" and self._drag is not None:
            self._release(editor)
            editor.notice = ""
            commit_draft(context.scene)
            self._area.tag_redraw()
            return {"RUNNING_MODAL"}
        if event.type == "RIGHTMOUSE":
            return {"RUNNING_MODAL"}  # отпускание правой кнопки - тоже инструменту: правая кнопка над картинкой - только меню метки
        if event.value == "PRESS" and event.type in {"D", "X", "DEL", "U"} and 0 <= editor.active < len(editor.marks):
            {"D": duplicate_active, "X": delete_active, "DEL": delete_active, "U": flip_active}[event.type](editor)
            editor.notice = ""
            commit_draft(context.scene)
            self._area.tag_redraw()
            return {"RUNNING_MODAL"}
        return {"PASS_THROUGH"}

    def _on_mark(self, context, editor, region, mx, my, event):
        """Ctrl + щелчок - поворот, Ctrl + правая кнопка - разворот стрелки, правая кнопка - меню метки; по пустому месту - ничего."""

        if self._drag is not None:
            return {"RUNNING_MODAL"}
        index = _mark_at(region, editor, mx, my)
        if index < 0:
            return {"RUNNING_MODAL"}
        editor.active = index
        if event.type == "RIGHTMOUSE" and not event.ctrl:
            self._area.tag_redraw()
            bpy.ops.wm.call_menu(name=MARK_MENU)
            return {"RUNNING_MODAL"}
        refusal = rotate_active(editor) if event.type == "LEFTMOUSE" else flip_active(editor)
        if refusal:
            self.report({"WARNING"}, editor.notice)
        commit_draft(context.scene)
        self._area.tag_redraw()
        return {"RUNNING_MODAL"}

    def _press(self, editor, region, mx, my):
        index, handle = _hit(region, editor, mx, my)
        px, py = _to_pixels(region, editor, mx, my)
        _SUSPEND["commit"] = True
        if index >= 0:
            editor.active = index
            mark = editor.marks[index]
            mode = "band" if handle == BAND_HANDLE else "move" if handle == (0, 0) else "resize"
            self._drag = {"mode": mode, "handle": handle, "start": (px, py), "rect": (mark.x, mark.y, mark.w, mark.h), "band": mark.band}
            return
        grid = editor.grid
        sx, sy = _snap(px, grid, editor.width), _snap(py, grid, editor.height)
        item = _append(editor, HotspotMarkV1(new_mark_id(editor), "", KIND_TRIM, sx, sy, 0, 0, "+y", 0))
        item.label = f"mark {item.mark_id}"
        editor.active = len(editor.marks) - 1
        self._drag = {"mode": "new", "anchor": (sx, sy), "rect": None, "id": int(item.mark_id)}

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
        if drag["mode"] == "band":
            mark.band = band_from_edge(mark_of(mark), py if mark.up in ("+y", "-y") else px, grid)
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
                _give_back(editor, drag)

    def _restore(self, editor):
        drag, self._drag = self._drag, None
        _SUSPEND["commit"] = False
        if drag["mode"] == "new":
            delete_active(editor)
            _give_back(editor, drag)
        elif 0 <= editor.active < len(editor.marks):
            mark = editor.marks[editor.active]
            _SUSPEND["commit"] = True
            mark.x, mark.y, mark.w, mark.h = drag["rect"]
            mark.band = drag["band"]
            _SUSPEND["commit"] = False


def _give_back(editor, drag) -> None:
    """Новая метка не родилась (щелчок без перетаскивания, Esc): её номер ни разу не был записан в текст - счётчик возвращается."""

    if int(editor.next_id) == drag.get("id", -2) + 1:
        editor.next_id = drag["id"]


def duplicate_active(editor) -> None:
    """Копия активной метки со следующим номером, сдвинутая на свою толщину вдоль «вверх» (длина остаётся во всю текстуру)."""

    source = editor.marks[editor.active]
    ux, uy = UP_VECTORS[source.up]
    _SUSPEND["commit"] = True
    try:
        item = _append(editor, HotspotMarkV1(new_mark_id(editor), f"{source.label} copy", source.kind, source.x + ux * source.w, source.y + uy * source.h, source.w, source.h, source.up, source.band))
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


def flip_active(editor) -> str:
    """Стрелка «вверх» активной метки в обратную сторону (`flipped_mark`), прямоугольник на месте. Отказов нет: пусто."""

    mark = editor.marks[editor.active]
    _SUSPEND["commit"] = True
    try:
        mark.up = UP_OPPOSITE.get(mark.up, "-y")
    finally:
        _SUSPEND["commit"] = False
    editor.notice = ""
    return ""


def rotate_active(editor) -> str:
    """Активная метка на 90° по часовой (`rotated_mark`): толщина та же, длина во всю текстуру по новой оси, середина по сетке. Отказ
    MARK_ROTATE_BLOCKED (имя возвращается, полный текст - в `notice`): метка не меняется."""

    rotated, problem = rotated_mark(draft_markup(editor), editor.active, editor.grid)
    if problem is not None:
        editor.notice = problem.text()
        print(f"[CFTUV][Hotspot] {editor.notice}", flush=True)
        return problem.name
    mark = editor.marks[editor.active]
    _SUSPEND["commit"] = True
    try:
        mark.up, mark.x, mark.y, mark.w, mark.h = rotated.up, rotated.x, rotated.y, rotated.w, rotated.h
    finally:
        _SUSPEND["commit"] = False
    editor.notice = ""
    return ""


# --------------------------------------------------------------------------
# Меню метки (правая кнопка) и его операторы
# --------------------------------------------------------------------------


def _active_mark(context):
    editor = editor_of(context.scene)
    if editor is None or not editor.draft_of or not 0 <= editor.active < len(editor.marks):
        return editor, None
    return editor, editor.marks[editor.active]


class _ActiveMarkOperator:
    """Правка выбранной метки из меню правой кнопки: как правки инструмента - без своего шага Undo, черновик проверяется и пишется сразу."""

    bl_options = {"INTERNAL"}

    @classmethod
    def poll(cls, context):
        return _active_mark(context)[1] is not None

    def _done(self, context, refusal: str = ""):
        editor = editor_of(context.scene)
        if refusal:
            self.report({"WARNING"}, editor.notice)
        else:
            editor.notice = ""  # отказ прошлой правки снимает следующая удачная
        commit_draft(context.scene)
        _redraw_image_editors()
        return {"FINISHED"}


class HOTSPOTUV_OT_HotspotMarkRotate(_ActiveMarkOperator, bpy.types.Operator):
    bl_idname = "hotspotuv.hotspot_mark_rotate"
    bl_label = "Rotate 90° Clockwise"
    bl_description = "Turn the mark a quarter clockwise: the same thickness, the full texture along the new axis, up turns with it (Ctrl + click)"

    def execute(self, context):
        return self._done(context, rotate_active(editor_of(context.scene)))


class HOTSPOTUV_OT_HotspotMarkFlip(_ActiveMarkOperator, bpy.types.Operator):
    bl_idname = "hotspotuv.hotspot_mark_flip"
    bl_label = "Flip Up"
    bl_description = "Point the up arrow the other way; the rectangle stays (Ctrl + right click, U)"

    def execute(self, context):
        return self._done(context, flip_active(editor_of(context.scene)))


class HOTSPOTUV_OT_HotspotMarkDelete(_ActiveMarkOperator, bpy.types.Operator):
    bl_idname = "hotspotuv.hotspot_mark_delete"
    bl_label = "Delete"
    bl_description = "Delete the mark (X)"

    def execute(self, context):
        editor = editor_of(context.scene)
        delete_active(editor)
        editor.notice = ""
        return self._done(context)


class HOTSPOTUV_OT_HotspotMarkRename(_ActiveMarkOperator, bpy.types.Operator):
    bl_idname = "hotspotuv.hotspot_mark_rename"
    bl_label = "Rename Mark"
    bl_description = "Rename the mark"

    name: StringProperty(name="Name")

    def invoke(self, context, _event):
        self.name = _active_mark(context)[1].label
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        _editor, mark = _active_mark(context)
        _SUSPEND["commit"] = True
        try:
            mark.label = self.name
        finally:
            _SUSPEND["commit"] = False
        return self._done(context)


class HOTSPOTUV_OT_HotspotMarkKind(_ActiveMarkOperator, bpy.types.Operator):
    bl_idname = "hotspotuv.hotspot_mark_kind"
    bl_label = "Kind"
    bl_description = "Trim: bottom edge on the seam, top edge at the outer skirt edge. Corner: seam on the centre line, the two skirts as mirrored wings"

    kind: EnumProperty(name="Kind", items=((KIND_TRIM, "Trim", ""), (KIND_CORNER, "Corner", "")))

    def execute(self, context):
        _editor, mark = _active_mark(context)
        _SUSPEND["commit"] = True
        try:
            mark.kind = self.kind
        finally:
            _SUSPEND["commit"] = False
        return self._done(context)


class HOTSPOTUV_MT_HotspotMarkContext(bpy.types.Menu):
    bl_idname = MARK_MENU
    bl_label = "Mark"

    def draw(self, context):
        layout = self.layout
        _editor, mark = _active_mark(context)
        if mark is None:
            layout.label(text="no mark")
            return
        layout.label(text=f"{mark.mark_id}: {mark.label}", icon="UV")
        layout.separator()
        layout.operator(HOTSPOTUV_OT_HotspotMarkRotate.bl_idname, icon="LOOP_FORWARDS")
        layout.operator(HOTSPOTUV_OT_HotspotMarkFlip.bl_idname, icon="ARROW_LEFTRIGHT")
        layout.operator_context = "INVOKE_DEFAULT"
        layout.operator(HOTSPOTUV_OT_HotspotMarkRename.bl_idname, text="Rename...", icon="SORTALPHA")
        layout.separator()
        for kind, text in ((KIND_TRIM, "Trim"), (KIND_CORNER, "Corner")):
            layout.operator(HOTSPOTUV_OT_HotspotMarkKind.bl_idname, text=f"Kind: {text}", icon="RADIOBUT_ON" if mark.kind == kind else "RADIOBUT_OFF").kind = kind
        layout.separator()
        layout.operator(HOTSPOTUV_OT_HotspotMarkDelete.bl_idname, icon="TRASH")


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
    changed = backdrop_changed(context.area, editor) if mode == OVERLAY_EDITOR else ""
    if changed:  # метки в пикселях своей текстуры поверх чужой картинки не рисуются; имя и что сделать - строкой внизу
        import blf

        _draw_state(blf, context.area, editor, changed)
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
        if active and mode == OVERLAY_EDITOR and mark.kind == KIND_CORNER:  # ручки краёв полосы: тянуть - ширина полосы
            handles = _band_handles(mark)
            lines(handles[0], (1.0, 0.85, 0.2, 1.0), 2.0)
            for point in handles[1]:
                hx, hy = _to_region(region, editor, *point)
                box = [(hx - 4, hy - 4), (hx + 4, hy - 4), (hx + 4, hy + 4), (hx - 4, hy + 4)]
                batch = batch_for_shader(shader, "TRIS", {"pos": box}, indices=((0, 1, 2), (0, 2, 3)))
                shader.uniform_float("color", (1.0, 0.85, 0.2, 1.0))
                batch.draw(shader)
        x, y = _to_region(region, editor, mark.x, mark.y + mark.h)
        blf.position(0, x + 4, y - 16, 0)
        blf.size(0, 13)
        blf.enable(0, blf.SHADOW)  # подпись читается и на заливке
        blf.shadow(0, 3, 0.0, 0.0, 0.0, 0.9)
        blf.color(0, 1.0, 1.0, 1.0, 1.0)
        blf.draw(0, mark_caption(mark))
        blf.disable(0, blf.SHADOW)
    if mode == OVERLAY_EDITOR:
        _draw_state(blf, context.area, editor)
    gpu.state.line_width_set(1.0)
    gpu.state.blend_set("NONE")


def mark_caption(mark) -> str:
    """Подпись метки на картинке: всё, что раньше показывала боковая панель (номер, имя, вид, «вверх», размер, полоса corner)."""

    band = f", band {mark.band}" if mark.kind == KIND_CORNER else ""
    return f"{mark.mark_id}: {mark.label} ({mark.kind}, up {mark.up}, {mark.w} x {mark.h} px{band})"


def _band_handles(mark) -> tuple:
    """`(края полосы отрезками, точки ручек)` в пикселях: края полосы corner по всей длине метки и ручки на их серединах (и на её концах)."""

    axis, edges = _band_edges(mark)
    if axis == "y":
        segments = [((mark.x, edge), (mark.x + mark.w, edge)) for edge in edges]
        points = [(x, edge) for edge in edges for x in (mark.x + 0.5 * mark.w, mark.x + 0.1 * mark.w, mark.x + 0.9 * mark.w)]
    else:
        segments = [((edge, mark.y), (edge, mark.y + mark.h)) for edge in edges]
        points = [(edge, y) for edge in edges for y in (mark.y + 0.5 * mark.h, mark.y + 0.1 * mark.h, mark.y + 0.9 * mark.h)]
    return segments, points


def backdrop_changed(area, editor) -> str:
    """Имя картинки, которую область мини-окна показывает вместо выбранной подложки (сменили в обход списка подложки); пусто - та же."""

    shown = getattr(area.spaces.active, "image", None)
    name = "" if shown is None else shown.name
    return "" if not editor.backdrop or name == editor.backdrop else (name or "no image")


def _draw_state(blf, area, editor, changed: str = "") -> None:
    """Строки внизу слева: подсказки клавиш, пока инструмент работает (иначе - как его запустить), подробности отказов и находок по именам."""

    running = area is not None and area.as_pointer() in _RUNNING
    hint, alarm = (0.85, 0.85, 0.85, 1.0), (1.0, 0.4, 0.3, 1.0)
    if changed:
        lines = [(f"{HOTSPOT_BACKDROP_CHANGED}: the window shows {changed}, the backdrop is {editor.backdrop}: marks hidden; Show Backdrop in the header", alarm)]
    elif running:
        lines = [(f"Mark tool, grid {editor.grid} px", hint)] + [("  |  ".join(KEY_HINTS[at:at + 2]), hint) for at in range(0, len(KEY_HINTS), 2)]
    else:
        lines = [("Mark tool not running: Start Mark Tool in the header", (1.0, 0.8, 0.3, 1.0))]
    for text in (editor.notice, editor.backdrop_problem):
        if text:
            lines.append((text[:220], alarm))
    if editor.dirty:
        lines.append(("NOT SAVED (the text keeps the last valid markup):", alarm))
        lines += [(f"  {item[:200]}", alarm) for item in editor.problems.split("; ")[:6]]
    else:
        lines.append((f"Saved to {editor.draft_of}", (0.5, 1.0, 0.5, 1.0)))
    blf.size(0, 12)
    for row, (text, colour) in enumerate(reversed(lines)):
        blf.position(0, 12, 12 + 18 * row, 0)
        blf.color(0, *colour)
        blf.draw(0, text)


# --------------------------------------------------------------------------
# Заголовок мини-окна (HOTSPOT_EDITOR_UX_V1)
# --------------------------------------------------------------------------


class HOTSPOTUV_MT_HotspotMarkupMenu(bpy.types.Menu):
    bl_idname = "HOTSPOTUV_MT_hotspot_markup_menu"
    bl_label = "Markup"

    def draw(self, context):
        layout = self.layout
        editor = editor_of(context.scene)
        layout.prop(editor, "show_in_uv_editor")
        layout.separator()
        layout.label(text=f"Text {editor.draft_of}", icon="TEXT")
        layout.label(text=f"Texture {editor.image}: {editor.width} x {editor.height} px", icon="TEXTURE")
        layout.separator()
        layout.operator(HOTSPOTUV_OT_HotspotBackdropShow.bl_idname, icon="IMAGE_DATA")
        row = layout.row()
        row.enabled = bool(editor.dirty)
        row.operator(HOTSPOTUV_OT_HotspotMarkupDiscard.bl_idname, icon="TRASH")


def draw_mini_header(layout, context) -> None:
    """Заголовок области мини-окна вместо меню Image Editor: имя окна, подложка, меню Markup, сетка и состояние по именам - сохранено или
    NOT SAVED с именами находок (подробности - строками внизу картинки), отказы правки и подложки, инструмент не работает - кнопка запуска."""

    editor = editor_of(context.scene)
    area = context.area
    layout.label(text=window_title(editor))
    row = layout.row()
    row.ui_units_x = 8  # мини-окно узкое: имя картинки в списке подложки сокращается, состояние справа должно читаться
    row.prop(editor, "backdrop_choice", text="", icon="IMAGE_DATA")
    layout.menu(HOTSPOTUV_MT_HotspotMarkupMenu.bl_idname)
    changed = backdrop_changed(area, editor)
    if changed:
        row = layout.row()
        row.alert = True
        row.operator(HOTSPOTUV_OT_HotspotBackdropShow.bl_idname, text=f"{HOTSPOT_BACKDROP_CHANGED}: Show Backdrop", icon="ERROR")
    layout.separator_spacer()
    layout.label(text=f"Grid {editor.grid} px")
    if area.as_pointer() not in _RUNNING:
        row = layout.row()
        row.alert = True
        row.operator(HOTSPOTUV_OT_HotspotMarkupEdit.bl_idname, icon="PLAY")
    for text in (editor.notice, editor.backdrop_problem):
        if text:
            row = layout.row()
            row.alert = True
            row.label(text=", ".join(problem_names(text)), icon="ERROR")
    if editor.dirty:
        row = layout.row(align=True)
        row.alert = True
        names = problem_names(editor.problems)
        row.label(text=f"NOT SAVED: {', '.join(names[:3])}" + (f" +{len(names) - 3}" if len(names) > 3 else ""), icon="ERROR")
        row.operator(HOTSPOTUV_OT_HotspotMarkupDiscard.bl_idname, text="", icon="TRASH")
    else:
        layout.label(text="saved", icon="CHECKMARK")


#: Исходный `IMAGE_HT_header.draw` Blender: обёртка рисует свой заголовок только в области мини-окна, всюду иначе - его.
_HEADER: dict = {}


def _image_header_draw(self, context):
    if not _HEADER.get("off") and is_mini_context(context):
        draw_mini_header(self.layout, context)
        return
    _HEADER["draw"](self, context)


def _wrap_image_header() -> None:
    """Заголовок Image Editor: в области мини-окна - `draw_mini_header`, во всех прочих - как был (HOTSPOT_EDITOR_UX_V1). `header_text_set`
    не годится: текст заменяет заголовок целиком, и в нём не будет списка подложки."""

    header = bpy.types.IMAGE_HT_header
    if header.draw is _image_header_draw:
        return
    _HEADER["draw"], _HEADER["off"] = header.draw, False
    header.draw = _image_header_draw


def _unwrap_image_header() -> None:
    header = getattr(bpy.types, "IMAGE_HT_header", None)
    if header is not None and header.draw is _image_header_draw:
        header.draw = _HEADER["draw"]
    else:  # поверх обёртки кто-то добавил свои строки (`IMAGE_HT_header.append`): обёртка остаётся и только пропускает
        _HEADER["off"] = True


_CLASSES = (
    HOTSPOTUV_HotspotMark,
    HOTSPOTUV_HotspotEditor,
    HOTSPOTUV_OT_HotspotMarkUp,
    HOTSPOTUV_OT_HotspotMarkupDiscard,
    HOTSPOTUV_OT_HotspotBackdropShow,
    HOTSPOTUV_OT_HotspotMarkupEdit,
    HOTSPOTUV_OT_HotspotMarkRotate,
    HOTSPOTUV_OT_HotspotMarkFlip,
    HOTSPOTUV_OT_HotspotMarkDelete,
    HOTSPOTUV_OT_HotspotMarkRename,
    HOTSPOTUV_OT_HotspotMarkKind,
    HOTSPOTUV_MT_HotspotMarkContext,
    HOTSPOTUV_MT_HotspotMarkupMenu,
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
    _wrap_image_header()


def unregister_editor() -> None:
    _unwrap_image_header()
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
    "BAND_HANDLE",
    "EDITOR_ATTRIBUTE",
    "HOTSPOT_EDITOR_TOOL_NOT_STARTED",
    "HOTSPOT_EDITOR_TOOL_STOPPED",
    "HOTSPOT_EDITOR_WINDOW_NOT_OPENED",
    "KEY_HINTS",
    "MARK_MENU",
    "MARKUP_DRAFT_UNSAVED",
    "HOTSPOT_BACKDROP_CHANGED",
    "WINDOW_TITLE",
    "area_key",
    "backdrop_changed",
    "backdrop_images",
    "draw_mini_header",
    "flip_active",
    "mark_caption",
    "mark_of",
    "rotate_active",
    "show_backdrop",
    "switch_backdrop",
    "commit_draft",
    "draft_markup",
    "is_mini_context",
    "load_draft",
    "mini_key",
    "new_mark_id",
    "open_mini_editor",
    "overlay_mode",
    "window_title",
    "prepare_markup",
    "register_editor",
    "unregister_editor",
)
