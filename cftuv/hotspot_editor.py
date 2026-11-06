"""Редактор разметки Hotspot в Image Editor (HOTSPOT_MARKUP_EDITOR_V1): показ меток, инструмент рисования и список.

ЧЕРНОВИК И ТЕКСТ. Правится черновик - коллекция меток сцены (`hotspotuv_hotspot_editor`, она сохраняется вместе с .blend). После каждой правки
черновик проверяется (`hotspot_markup.save_markup`): принятый пишется в текстовый датаблок, непринятый НЕ пишется - текст остаётся прежним
принятым, черновик остаётся в сцене, панель и заголовок называют находки по именам. Ничего не теряется молча: загрузить другой текст поверх
непринятого черновика - отказ `MARKUP_DRAFT_UNSAVED`, пока черновик не исправлен или явно не отброшен.

ИНСТРУМЕНТ (`hotspotuv.hotspot_markup_edit`, модальный, в области Image Editor): перетаскивание по пустому месту - новый прямоугольник по сетке
пикселей (шаг `[` и `]`), щелчок по метке - выбор, перетаскивание внутри - перенос, за ручку (угол, середина стороны) - размер, `D` - копия,
`X`/`Delete` - удаление, `U` - следующее направление «вверх», `Esc`/правая кнопка - выход. Колесо и средняя кнопка проходят в Image Editor.

ПОКАЗ (обработчик `POST_PIXEL` Image Editor): контур, номер и имя, стрелка «вверх», нижний край (шов `trim`) или средняя линия (шов `corner`)
выделены, у `corner` - центральная полоса `band`.
"""

from __future__ import annotations

import bpy
from bpy.props import BoolProperty, CollectionProperty, EnumProperty, IntProperty, PointerProperty, StringProperty

from .hotspot_blender import link_markup, read_markup_text, write_markup_text, decal_material
from .hotspot_markup import KIND_CORNER, KIND_TRIM, KINDS, UP_VECTORS, HotspotMarkV1, HotspotMarkupV1, empty_markup, next_mark_id, save_markup

EDITOR_ATTRIBUTE = "hotspotuv_hotspot_editor"
MARKUP_DRAFT_UNSAVED = "MARKUP_DRAFT_UNSAVED"
MARKUP_NO_IMAGE = "MARKUP_NO_IMAGE"
UP_ORDER = ("+y", "-y", "+x", "-x")
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


class HOTSPOTUV_HotspotEditor(bpy.types.PropertyGroup):
    text_name: StringProperty(name="Markup", description="Text datablock to load (Load Markup); edits are saved to the text the draft was loaded from")
    draft_of: StringProperty(name="Draft of", description="Text datablock the draft was loaded from and is saved to")
    width: IntProperty(name="Texture width", min=0)
    height: IntProperty(name="Texture height", min=0)
    image: StringProperty(name="Image")
    marks: CollectionProperty(type=HOTSPOTUV_HotspotMark)
    active: IntProperty(name="Active mark", default=-1)
    grid: IntProperty(name="Grid", default=8, min=1, max=1024, description="Pixel grid of the edit tool ([ and ] halve and double it)")
    dirty: BoolProperty(name="Unsaved draft", default=False)
    problems: StringProperty(name="Problems")


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
        editor.text_name, editor.draft_of, editor.width, editor.height, editor.image = text_name, text_name, int(markup.width), int(markup.height), str(markup.image)
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


def _space_image(context):
    space = getattr(context, "space_data", None)
    return getattr(space, "image", None) if getattr(space, "type", "") == "IMAGE_EDITOR" else None


class HOTSPOTUV_OT_HotspotMarkupNew(bpy.types.Operator):
    bl_idname = "hotspotuv.hotspot_markup_new"
    bl_label = "New Markup"
    bl_description = "Create an empty hotspot markup for the image shown here, link it to the decal material and edit it"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        image = _space_image(context)
        editor = editor_of(context.scene)
        if image is None or not image.size[0] or not image.size[1]:
            self.report({"ERROR"}, f"{MARKUP_NO_IMAGE}: show an image with pixels in this Image Editor")
            return {"CANCELLED"}
        if editor.dirty:
            self.report({"ERROR"}, f"{MARKUP_DRAFT_UNSAVED}: fix or discard the draft of {editor.draft_of!r} first")
            return {"CANCELLED"}
        name = f"{image.name}.hotspot"
        text = bpy.data.texts.get(name)
        if text is None:
            write_markup_text(name, empty_markup(image.size[0], image.size[1], image.name))
        material = decal_material(context.scene)
        if material is not None:
            link_markup(material, name)
        refusal = load_draft(context.scene, name)
        if refusal:
            self.report({"ERROR"}, refusal)
            return {"CANCELLED"}
        return {"FINISHED"}


class HOTSPOTUV_OT_HotspotMarkupLoad(bpy.types.Operator):
    bl_idname = "hotspotuv.hotspot_markup_load"
    bl_label = "Load Markup"
    bl_description = "Load the markup text into the editor draft (refused over an unsaved draft unless discarded)"
    bl_options = {"REGISTER", "UNDO"}

    discard: BoolProperty(name="Discard draft", default=False)

    def execute(self, context):
        editor = editor_of(context.scene)
        refusal = load_draft(context.scene, editor.text_name, discard=self.discard)
        if refusal:
            self.report({"ERROR"}, f"{refusal}: {editor.problems or editor.text_name}")
            return {"CANCELLED"}
        return {"FINISHED"}


class HOTSPOTUV_OT_HotspotMarkupLink(bpy.types.Operator):
    bl_idname = "hotspotuv.hotspot_markup_link"
    bl_label = "Link to Decal Material"
    bl_description = "The decal material names this markup (custom property cftuv_hotspot_markup)"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        material = decal_material(context.scene)
        editor = editor_of(context.scene)
        if material is None or not editor.draft_of:
            self.report({"ERROR"}, "no decal material or no markup to link")
            return {"CANCELLED"}
        link_markup(material, editor.draft_of)
        return {"FINISHED"}


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
        return getattr(context.area, "type", "") == "IMAGE_EDITOR" and editor is not None and bool(editor.draft_of) and editor.width > 0

    def invoke(self, context, event):
        self._drag = None
        context.window_manager.modal_handler_add(self)
        self._header(context)
        return {"RUNNING_MODAL"}

    def _header(self, context):
        editor = editor_of(context.scene)
        state = f"unsaved: {editor.problems}" if editor.dirty else "saved"
        context.area.header_text_set(f"Hotspot marks | grid {editor.grid} px ([ ]) | drag: new, move, resize | D duplicate | X delete | U up | Esc exit | {state}")
        context.area.tag_redraw()

    def _finish(self, context):
        _SUSPEND["commit"] = False
        context.area.header_text_set(None)
        context.area.tag_redraw()
        return {"FINISHED"}

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
        region = context.region
        if region is None or region.type != "WINDOW":
            return {"PASS_THROUGH"}
        mx, my = event.mouse_region_x, event.mouse_region_y
        if event.type in {"ESC", "RIGHTMOUSE"} and event.value == "PRESS":
            if self._drag is not None:
                self._restore(editor)
            commit_draft(context.scene)
            return self._finish(context)
        if event.type == "MOUSEMOVE" and self._drag is not None:
            self._move(editor, region, mx, my)
            context.area.tag_redraw()
            return {"RUNNING_MODAL"}
        if event.type == "LEFTMOUSE" and event.value == "PRESS":
            self._press(editor, region, mx, my)
            context.area.tag_redraw()
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
    if editor is None or not editor.draft_of or editor.width <= 0 or region is None:
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
        fill(band, (1.0, 0.85, 0.2, 0.3))
        lines(band, (1.0, 0.85, 0.2, 0.9), 1.0)
        lines(outline, (1.0, 1.0, 1.0, 1.0) if active else (0.3, 0.8, 1.0, 0.9), 2.0 if active else 1.0)
        lines(seam, (1.0, 0.3, 0.2, 1.0), 3.0)
        lines(arrow, (0.4, 1.0, 0.4, 1.0), 2.0)
        x, y = _to_region(region, editor, mark.x, mark.y + mark.h)
        blf.position(0, x + 4, y - 16, 0)
        blf.size(0, 13)
        blf.color(0, 1.0, 1.0, 1.0, 1.0)
        blf.draw(0, f"{mark.mark_id}: {mark.label} ({mark.kind}, up {mark.up})")
    if editor.dirty:
        blf.position(0, 12, 12, 0)
        blf.size(0, 13)
        blf.color(0, 1.0, 0.4, 0.3, 1.0)
        blf.draw(0, f"UNSAVED DRAFT: {editor.problems[:160]}")
    gpu.state.line_width_set(1.0)
    gpu.state.blend_set("NONE")


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

    def draw(self, context):
        layout = self.layout
        editor = editor_of(context.scene)
        if editor is None:
            return
        layout.prop_search(editor, "text_name", bpy.data, "texts")
        row = layout.row(align=True)
        row.operator(HOTSPOTUV_OT_HotspotMarkupNew.bl_idname, icon="ADD")
        row.operator(HOTSPOTUV_OT_HotspotMarkupLoad.bl_idname, icon="FILE_REFRESH").discard = False
        layout.operator(HOTSPOTUV_OT_HotspotMarkupLink.bl_idname, icon="MATERIAL")
        if not editor.draft_of:
            return
        layout.label(text=f"Draft of {editor.draft_of}: texture {editor.width} x {editor.height} px {editor.image}")
        if editor.dirty:
            box = layout.box()
            box.label(text="Unsaved draft: the text keeps the last valid markup", icon="ERROR")
            for line in editor.problems.split("; ")[:8]:
                box.label(text=line[:120])
            box.operator(HOTSPOTUV_OT_HotspotMarkupLoad.bl_idname, text="Discard Draft and Reload", icon="TRASH").discard = True
        else:
            layout.label(text="Saved", icon="CHECKMARK")
        layout.operator(HOTSPOTUV_OT_HotspotMarkupEdit.bl_idname, icon="SELECT_SET")
        layout.prop(editor, "grid")
        layout.template_list("HOTSPOTUV_UL_HotspotMarks", "", editor, "marks", editor, "active", rows=4)
        if 0 <= editor.active < len(editor.marks):
            mark = editor.marks[editor.active]
            column = layout.column(align=True)
            for name in ("mark_id", "label", "kind", "up", "x", "y", "w", "h"):
                column.prop(mark, name)
            if mark.kind == KIND_CORNER:
                column.prop(mark, "band")


_CLASSES = (
    HOTSPOTUV_HotspotMark,
    HOTSPOTUV_HotspotEditor,
    HOTSPOTUV_OT_HotspotMarkupNew,
    HOTSPOTUV_OT_HotspotMarkupLoad,
    HOTSPOTUV_OT_HotspotMarkupLink,
    HOTSPOTUV_OT_HotspotMarkupEdit,
    HOTSPOTUV_UL_HotspotMarks,
    HOTSPOTUV_PT_HotspotMarkup,
)
_HANDLE: dict = {}


def register_editor() -> None:
    for cls in _CLASSES:
        if getattr(cls, "is_registered", False):
            bpy.utils.unregister_class(cls)
        bpy.utils.register_class(cls)
    setattr(bpy.types.Scene, EDITOR_ATTRIBUTE, PointerProperty(type=HOTSPOTUV_HotspotEditor))
    if "draw" not in _HANDLE:
        _HANDLE["draw"] = bpy.types.SpaceImageEditor.draw_handler_add(_draw_marks, (), "WINDOW", "POST_PIXEL")


def unregister_editor() -> None:
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
    "MARKUP_DRAFT_UNSAVED",
    "MARKUP_NO_IMAGE",
    "commit_draft",
    "draft_markup",
    "load_draft",
    "register_editor",
    "unregister_editor",
)
