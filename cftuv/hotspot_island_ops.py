"""Метки по островам готовой декали (HOTSPOT_ISLAND_MARKS_V1): Edit Mode на `<источник>.CFTUV_Decal`, выделить грани, выбрать метку в строках
Hotspot - UV островов выделения сразу ложатся в метку, без пересборки и без кнопки (HOTSPOT_APPLY_ON_PICK_V1: выбор метки и Swap wings - операторы
`hotspotuv.hotspot_pick_mark` / `hotspotuv.hotspot_swap_wings`, один шаг Undo; у свойств настроек нет обратных вызовов, поэтому Undo/Redo,
загрузка файла, рисование панели и присваивание из кода ничего не применяют). Чистая часть - `hotspot_islands`, цель выбора - `pick_target`.

* Выделение расширяется до целых островов (сторона шва домена и владельца); corner-метка - и до соседей по шву, крылья бывают только парой.
  После нажатия выделены именно острова, которые метку получили.
* UV: `M_new(M_old^-1(UV))` - текущая метка острова обращается, новая прикладывается; острова, чьи код и метка не изменились, остаются
  побитово. Декаль, собранная без Hotspot, - тождественный случай (UV слоя и есть UV ядра): первое нажатие кладёт ей запись (по умолчанию
  `NO_MARK`), пока на ней не нажали, она та же побитово.
* Запись Edit Mode: оператор выходит в Object Mode, пишет UV, коды граней и запись меша и возвращается; один шаг Undo возвращает UV и коды
  граней (правду островов), запись меша догоняет их перед следующей пересборкой граней.
* HOTSPOT_OBJECT_APPLY_V1: в Object Mode выбор метки кладёт её ВСЕМ островам каждой выделенной декали (маркер декали, не имя), один шаг
  Undo; не декали пропускаются и считаются по именам (HOTSPOT_NOT_A_DECAL_MESH), пары corner - то же правило топологии шва.
* Ничего применимого не выделено (Edit Mode не на декали или без граней, Object Mode без декалей, другой режим) - метка только выбрана для
  следующей сборки, строка панели говорит это словами, отчёта-ошибки нет. Отказы декали - по именам (WARNING): метки нет в разметке
  (HOTSPOT_MARK_NOT_FOUND), разметка панели не та, что у декали (HOTSPOT_MARKUP_MISMATCH), запись меша не читается
  (HOTSPOT_MAPPING_RECORD_INVALID); «Select by Mark» вне Edit Mode декали - HOTSPOT_NOT_IN_EDIT_MODE / HOTSPOT_NOT_A_DECAL_MESH.
"""

from __future__ import annotations

import bmesh
import bpy
import numpy as np
from bpy.props import IntProperty

from .envelope_production_mesh import DECAL_REVISION_PROPERTY, DECAL_UV_LAYER
from .hotspot_blender import (
    HOTSPOT_APPLY_REENTERED,
    HOTSPOT_MAPPING_RECORD_INVALID,
    HOTSPOT_MARK_NOT_FOUND,
    KERNEL_UVS,
    LAST_COUNTERS,
    MARKS_MENU,
    PICK_DECALS,
    PICK_FACES,
    SETTINGS_ATTRIBUTE,
    decal_material,
    COUNTER_UNWARPED,
    current_atlas,
    exact_strip,
    face_codes,
    map_with_codes,
    mark_items,
    mesh_arrays,
    mesh_record,
    pick_idle_line,
    pick_target,
    read_material_markup,
    remap_mesh,
    remembered_record,
    split_decal_targets,
)
from .hotspot_islands import (
    COUNTER_EXPANDED,
    COUNTER_SELECTED,
    MARK_ATTRIBUTE,
    NO_MARK,
    assign,
    chosen_islands,
    code_for,
    decode,
    map_islands,
    record_to_json,
    labels_text,
    refresh_marks,
    selection_labels,
    selection_text,
    unmapped_record,
)
from .hotspot_mapping import COUNTER_UNPAIRED, MESH_PROPERTY

HOTSPOT_NOT_A_DECAL_MESH = "HOTSPOT_NOT_A_DECAL_MESH"
HOTSPOT_NOT_IN_EDIT_MODE = "HOTSPOT_NOT_IN_EDIT_MODE"
HOTSPOT_SELECTION_EMPTY = "HOTSPOT_SELECTION_EMPTY"
HOTSPOT_MARKUP_MISMATCH = "HOTSPOT_MARKUP_MISMATCH"
#: Путь «Apply Mark»: от UV ядра точного кадра сессии (точно) либо обращением текущей метки острова.
HOTSPOT_REMAP_FROM_BUILD = "HOTSPOT_REMAP_FROM_BUILD"
HOTSPOT_REMAP_BY_INVERSION = "HOTSPOT_REMAP_BY_INVERSION"
#: Обращение, где стороны corner-пары развёрнуты по длине ряда шва (UV ядра сессии нет).
HOTSPOT_REMAP_BY_SEAM_ARC = "HOTSPOT_REMAP_BY_SEAM_ARC"


def remap_path(from_build: bool, unwarped: int) -> str:
    """Имя пути «Apply Mark»: UV ядра сессии (точно) -> развёртка ряда шва (стороны пары) -> обращение меток (trim и corner без пары - точно)."""

    return HOTSPOT_REMAP_FROM_BUILD if from_build else HOTSPOT_REMAP_BY_SEAM_ARC if unwarped else HOTSPOT_REMAP_BY_INVERSION


def decal_state(context) -> tuple:
    """`(объект, запись | None, отказ, подробность)` активной декали; запись `None` без отказа - декаль собрана без Hotspot (UV ядра)."""

    obj = getattr(context, "edit_object", None) or getattr(context, "active_object", None)
    if obj is None or obj.type != "MESH" or DECAL_REVISION_PROPERTY not in obj.keys():
        return obj, None, HOTSPOT_NOT_A_DECAL_MESH, "make a CFTUV decal (<source>.CFTUV_Decal) the active object"
    record, error = mesh_record(obj.data)
    if error:
        return obj, None, HOTSPOT_MAPPING_RECORD_INVALID, error
    return obj, record, "", ""


def resolve_mark(context, record) -> tuple:
    """`(метка | None, swap, имя разметки, разметка, отказ, подробность)`: метка панели из разметки материала декали; у декали с записью - той же
    разметки и того же размера текстуры."""

    scene = context.scene
    settings = getattr(scene, SETTINGS_ATTRIBUTE)
    name, markup, problems = read_material_markup(decal_material(scene))
    if markup is None or problems:
        first = problems[0] if problems else None
        return None, False, name, None, (first.name if first else HOTSPOT_MARKUP_MISMATCH), (first.detail if first else "the decal material has no markup")
    if record is not None and (name, markup.width, markup.height) != (record.markup, record.width, record.height):
        return None, False, name, markup, HOTSPOT_MARKUP_MISMATCH, (
            f"the decal is mapped with markup {record.markup!r} ({record.width} x {record.height}), the panel material links {name!r} "
            f"({markup.width} x {markup.height}): pick the decal's material or rebuild"
        )
    mark = markup.mark(int(settings.mark_id))
    if mark is None:
        return None, False, name, markup, HOTSPOT_MARK_NOT_FOUND, f"markup {name!r} has no mark {int(settings.mark_id)}"
    return mark, bool(settings.swap_wings), name, markup, "", ""


def build_strip(mesh, record, codes):
    """UV ядра последней записи писателя в этой сессии (`KERNEL_UVS`; у декали без Hotspot - сам слой UV), если меш показывает ровно их
    отображение текущими кодами (побитово). Иначе (файл перезагружен, меш правили) - `None`, и метка обращается."""

    uvs = KERNEL_UVS.get(mesh.session_uid)
    if uvs is None and MESH_PROPERTY not in mesh.keys():
        uvs = current_atlas(mesh, DECAL_UV_LAYER)
    if uvs is None:
        return None
    kernel = np.asarray(uvs, dtype=np.float32).reshape(-1, 2)
    if kernel.shape[0] != len(mesh.loops):
        return None
    vertices, totals, group = mesh_arrays(mesh)
    expected, _counters = map_islands(kernel, vertices, np.repeat(group, totals), np.repeat(codes, totals), record)
    return kernel if np.array_equal(expected.astype(np.float32), current_atlas(mesh, DECAL_UV_LAYER)) else None


def apply_to_selection(mesh, record, markup, mark, swap, source=None, faces=None) -> tuple:
    """Object Mode: `(маска граней островов | None, числа | (отказ, подробность))` - метка выделенным островам, запись меша. UV ядра - путь
    `remap_path`: UV ядра последней записи сессии, если меш показывает ровно их (HOTSPOT_REMAP_FROM_BUILD), иначе обращение меток, у сторон
    corner-пары - с развёрткой по длине ряда шва по вершинам источника `source` (HOTSPOT_REMAP_BY_SEAM_ARC; HOTSPOT_REMAP_BY_INVERSION).
    `faces` - маска граней вместо выделения меша (HOTSPOT_OBJECT_APPLY_V1: все грани декали)."""

    _vertices, totals, group = mesh_arrays(mesh)
    selected = np.zeros(len(mesh.polygons), dtype=bool)
    if faces is None:
        mesh.polygons.foreach_get("select", selected)
    else:
        selected[:] = np.asarray(faces, dtype=bool)
    if not selected.any():
        return None, (HOTSPOT_SELECTION_EMPTY, "select faces of the decal")
    old_codes = face_codes(mesh, len(totals))
    old_codes = np.full(len(totals), record.default, dtype=np.int64) if old_codes is None else old_codes
    kernel = build_strip(mesh, record, old_codes)
    numbers = {}
    if kernel is None:
        strip, numbers = exact_strip(mesh, record, old_codes, source, DECAL_UV_LAYER)
    else:
        strip = kernel
    groups, expanded = chosen_islands(group, selected, mark.kind, _vertices, np.repeat(group, totals), strip[:, 1])
    new = refresh_marks(record, markup).with_marks([mark])
    new_codes = assign(old_codes, group, groups, code_for(mark, swap))
    if kernel is None:
        counters = remap_mesh(mesh, record, new, old_codes, new_codes, DECAL_UV_LAYER, strip)
    else:
        _codes, counters = map_with_codes(mesh, new, kernel, new_codes, DECAL_UV_LAYER)
    counters.update(numbers)
    counters[remap_path(kernel is not None, numbers.get(COUNTER_UNWARPED, 0))] = 1
    mesh[MESH_PROPERTY] = record_to_json(remembered_record(mesh, new, DECAL_UV_LAYER))
    counters[COUNTER_SELECTED], counters[COUNTER_EXPANDED] = int(groups.shape[0]), expanded
    LAST_COUNTERS[mesh.session_uid] = counters
    mesh.update()
    return np.isin(group, groups), counters


def _select_faces(mesh, mask) -> int:
    """Edit Mode: выделены ровно грани `mask` (порядок граней BMesh - порядок граней меша после входа в режим)."""

    bm = bmesh.from_edit_mesh(mesh)
    bm.faces.ensure_lookup_table()
    for item in (*bm.verts, *bm.edges, *bm.faces):
        item.select = False
    for index in np.flatnonzero(mask).tolist():
        bm.faces[index].select_set(True)
    bm.select_flush_mode()
    bmesh.update_edit_mesh(mesh, loop_triangles=False, destructive=False)
    return int(np.count_nonzero(mask))


def _refused(operator, refusal, detail):
    print(f"[CFTUV][Hotspot] {refusal}: {detail}", flush=True)
    operator.report({"ERROR"}, f"{refusal}: {detail}")
    return {"CANCELLED"}


def _refusal(refusal, detail) -> tuple:
    """Метка выбрана, но декаль её не приняла: `(WARNING, строка)` с именем отказа; выбор метки панели всё равно остаётся."""

    line = f"{refusal}: {detail}"
    print(f"[CFTUV][Hotspot] {line}", flush=True)
    return "WARNING", line


def apply_selected_faces(context) -> tuple:
    """Edit Mode на декали (HOTSPOT_APPLY_ON_PICK_V1): метка панели островам выделенных граней - `(уровень отчёта, строка)`."""

    obj, record, refusal, detail = decal_state(context)
    if not refusal:
        mark, swap, name, markup, refusal, detail = resolve_mark(context, record)
    if refusal:
        return _refusal(refusal, detail)
    record = unmapped_record(name, markup.width, markup.height) if record is None else record
    bpy.ops.object.mode_set(mode="OBJECT")
    try:
        mask, counters = apply_to_selection(obj.data, record, markup, mark, swap, obj.parent)
    finally:
        bpy.ops.object.mode_set(mode="EDIT")
    if mask is None:
        return _refusal(*counters)
    faces = _select_faces(obj.data, mask)
    line = (
        f"{mark.id}: {mark.name} ({mark.kind}{', swapped' if swap and mark.kind == 'corner' else ''}) -> {counters[COUNTER_SELECTED]} island(s), {faces} face(s); "
        f"{COUNTER_EXPANDED}={counters[COUNTER_EXPANDED]}, {COUNTER_UNPAIRED}={counters.get(COUNTER_UNPAIRED, 0)}, {remap_name(counters)}"
    )
    print(f"[CFTUV][Hotspot] {obj.data.name}: {line}; " + ", ".join(f"{key}={value}" for key, value in counters.items()), flush=True)
    return "INFO", line


def apply_selected_decals(context, decals, skipped) -> tuple:
    """Object Mode (HOTSPOT_OBJECT_APPLY_V1): метка всем островам каждой выделенной декали - `(уровень отчёта, строка)`; не декали - счёт
    HOTSPOT_NOT_A_DECAL_MESH по именам, декаль с отказом (запись, другая разметка) - по имени, остальные всё равно получают метку."""

    skip = f"{HOTSPOT_NOT_A_DECAL_MESH}={len(skipped)}" + (f" ({', '.join(skipped)[:160]})" if skipped else "")
    done, refused, mark, swap = [], [], None, False
    for obj in decals:
        mask, counters, mark_used, swap = apply_to_decal(context, obj)
        if mask is None:
            refused.append((obj.name, *counters))
            continue
        mark = mark_used
        done.append(counters)
        print(f"[CFTUV][Hotspot] {obj.data.name}: {mark.id}: {mark.name} -> {counters[COUNTER_SELECTED]} island(s), {remap_name(counters)}; " + ", ".join(f"{key}={value}" for key, value in counters.items()), flush=True)
    refusals = "; ".join(f"{name}: {refusal}: {detail}" for name, refusal, detail in refused)
    if not done:
        return _refusal(refused[0][1], f"{refusals}; {skip}")
    line = (
        f"{mark.id}: {mark.name} ({mark.kind}{', swapped' if swap and mark.kind == 'corner' else ''}) -> {len(done)} decal(s), "
        f"{sum(item[COUNTER_SELECTED] for item in done)} island(s); {COUNTER_UNPAIRED}={sum(item.get(COUNTER_UNPAIRED, 0) for item in done)}, {skip}"
        + (f"; refused {len(refused)}: {refusals}" if refused else "")
    )
    print(f"[CFTUV][Hotspot] {line}", flush=True)
    return ("WARNING" if skipped or refused else "INFO"), line


def apply_panel_mark(context) -> tuple:
    """Метка панели только что выбрана (HOTSPOT_APPLY_ON_PICK_V1): `(уровень отчёта | "", строка)`. Цель - `pick_target`: грани выделения
    декали в Edit Mode, все острова выделенных декалей в Object Mode; иначе ничего не трогается и уровень пуст - метка ждёт кнопку сборки."""

    mode = getattr(context, "mode", "")
    obj = getattr(context, "edit_object", None) or getattr(context, "active_object", None)
    active_decal = obj is not None and obj.type == "MESH" and DECAL_REVISION_PROPERTY in obj.keys()
    faces = int(obj.data.total_face_sel) if active_decal and mode == "EDIT_MESH" else 0
    decals, skipped = split_decal_targets(context.selected_objects, DECAL_REVISION_PROPERTY) if mode == "OBJECT" else ((), ())
    target = pick_target(mode, active_decal, faces, len(decals))
    if target == PICK_FACES:
        return apply_selected_faces(context)
    if target == PICK_DECALS:
        return apply_selected_decals(context, decals, skipped)
    return "", pick_idle_line(mode, active_decal)


#: Повторный вход в применение (обработчик, вызванный самим применением, позвал выбор снова): не применяется второй раз.
_BUSY = {"apply": False}


def _picked(operator, context) -> set:
    """Общий хвост выбора метки и Swap wings: применить к выделению (один шаг Undo оператора), строка - в панель и в отчёт."""

    settings = getattr(context.scene, SETTINGS_ATTRIBUTE)
    if _BUSY["apply"]:
        print(f"[CFTUV][Hotspot] {HOTSPOT_APPLY_REENTERED}: mark {int(settings.mark_id)} is being applied already", flush=True)
        return {"CANCELLED"}
    _BUSY["apply"] = True
    try:
        level, line = apply_panel_mark(context)
    finally:
        _BUSY["apply"] = False
    settings.apply_note = line[:400]
    if level:
        operator.report({level}, line)
    return {"FINISHED"}


class HOTSPOTUV_OT_HotspotPickMark(bpy.types.Operator):
    bl_idname = "hotspotuv.hotspot_pick_mark"
    bl_label = "Pick Mark"
    bl_description = (
        "Use this mark. Edit Mode on a decal: the islands of the selected faces take it at once (whole pChain sides; corner marks: both sides "
        "of the seam). Object Mode: every island of every selected decal. Nothing of that selected: the next Build Decal Mesh uses it"
    )
    bl_options = {"UNDO", "INTERNAL"}

    mark_id: IntProperty(name="Mark ID", min=0)

    def execute(self, context):
        getattr(context.scene, SETTINGS_ATTRIBUTE).mark_id = int(self.mark_id)
        return _picked(self, context)


class HOTSPOTUV_OT_HotspotSwapWings(bpy.types.Operator):
    bl_idname = "hotspotuv.hotspot_swap_wings"
    bl_label = "Swap wings"
    bl_description = "Corner marks: exchange which side of the seam takes which half of the butterfly; the selection (as for picking a mark) takes it at once"
    bl_options = {"UNDO", "INTERNAL"}

    def execute(self, context):
        settings = getattr(context.scene, SETTINGS_ATTRIBUTE)
        settings.swap_wings = not settings.swap_wings
        return _picked(self, context)


class HOTSPOTUV_MT_HotspotMarks(bpy.types.Menu):
    bl_idname = MARKS_MENU
    bl_label = "Mark"

    def draw(self, context):
        settings = getattr(context.scene, SETTINGS_ATTRIBUTE)
        _name, markup, _problems = read_material_markup(decal_material(context.scene))
        for _key, label, _description, value in mark_items(markup):
            if value < 0:
                self.layout.label(text=label, icon="INFO")
                continue
            current = value == int(settings.mark_id)
            self.layout.operator(HOTSPOTUV_OT_HotspotPickMark.bl_idname, text=label, icon="RADIOBUT_ON" if current else "RADIOBUT_OFF").mark_id = value


def remap_name(counters) -> str:
    return next((name for name in (HOTSPOT_REMAP_FROM_BUILD, HOTSPOT_REMAP_BY_SEAM_ARC, HOTSPOT_REMAP_BY_INVERSION) if name in counters), "")


def apply_to_decal(context, obj) -> tuple:
    """Object Mode: метка панели всем граням декали `obj` - `(маска | None, числа | (отказ, подробность), метка, swap)`; та же функция и пути,
    что у Edit Mode (пары corner по топологии шва, сторона без пары - одно крыло)."""

    record, error = mesh_record(obj.data)
    if error:
        return None, (HOTSPOT_MAPPING_RECORD_INVALID, error), None, False
    mark, swap, name, markup, refusal, detail = resolve_mark(context, record)
    if refusal:
        return None, (refusal, detail), None, False
    record = unmapped_record(name, markup.width, markup.height) if record is None else record
    mask, counters = apply_to_selection(obj.data, record, markup, mark, swap, obj.parent, faces=np.ones(len(obj.data.polygons), dtype=bool))
    return mask, counters, mark, swap


class HOTSPOTUV_OT_HotspotSelectMark(bpy.types.Operator):
    bl_idname = "hotspotuv.hotspot_select_mark"
    bl_label = "Select Islands by Mark"
    bl_description = "Select the decal islands that carry the mark of the panel (either wing order)"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        obj, _record, refusal, detail = decal_state(context)
        if not refusal and obj.mode != "EDIT":
            refusal, detail = HOTSPOT_NOT_IN_EDIT_MODE, "enter Edit Mode on the decal"
        if refusal:
            return _refused(self, refusal, detail)
        bm = bmesh.from_edit_mesh(obj.data)
        layer = bm.faces.layers.int.get(MARK_ATTRIBUTE)
        wanted = int(getattr(context.scene, SETTINGS_ATTRIBUTE).mark_id)
        mask = np.array([layer is not None and decode(face[layer])[0] == wanted for face in bm.faces], dtype=bool)
        faces = _select_faces(obj.data, mask)
        self.report({"INFO"}, f"mark {wanted}: {faces} face(s) selected")
        return {"FINISHED"}


def _draw_object_rows(layout, context) -> None:
    """Object Mode (HOTSPOT_OBJECT_APPLY_V1): метка всех граней выделенных декалей (одна или `mixed`), пропущенные не-декали числом, кнопка."""

    decals, skipped = split_decal_targets(context.selected_objects, DECAL_REVISION_PROPERTY)
    if not decals:
        return
    box = layout.box()
    labels, invalid = [], []
    for decal in decals:
        record, error = mesh_record(decal.data)
        if error:
            invalid.append(decal.name)
            continue
        if not len(decal.data.polygons):
            continue
        record = unmapped_record("", 0, 0) if record is None else record
        codes = face_codes(decal.data, len(decal.data.polygons))  # атрибута нет - все грани с меткой по умолчанию записи
        labels += selection_labels(record, [record.default] if codes is None else np.unique(codes).tolist())
    text = labels_text(labels)
    box.label(text=f"Selection, {len(decals)} decal(s): {text or 'no faces'}", icon="UV")
    if skipped:
        box.label(text=f"{HOTSPOT_NOT_A_DECAL_MESH}: {len(skipped)} selected object(s) skipped", icon="INFO")
    if invalid:
        box.label(text=f"{HOTSPOT_MAPPING_RECORD_INVALID}: {', '.join(invalid)[:160]}", icon="ERROR")


def draw_island_rows(layout, context) -> None:
    """Строки островов в строках Hotspot: в Object Mode - метки выделенных декалей; в Edit Mode активной декали - метка выделения граней и
    «Select by Mark». Кнопки «Apply» нет: метку кладёт сам выбор метки (HOTSPOT_APPLY_ON_PICK_V1)."""

    if getattr(context, "mode", "") == "OBJECT":
        _draw_object_rows(layout, context)
        return
    obj = getattr(context, "active_object", None)
    if obj is None or obj.type != "MESH" or DECAL_REVISION_PROPERTY not in obj.keys():
        return
    box = layout.box()
    _obj, record, refusal, detail = decal_state(context)
    if refusal:
        box.label(text=refusal, icon="ERROR")
        box.label(text=detail[:200])
        return
    if obj.mode != "EDIT":
        box.label(text="Island marks: Object Mode (selected decals) or Edit Mode (selected faces)", icon="INFO")
        return
    bm = bmesh.from_edit_mesh(obj.data)
    layer = bm.faces.layers.int.get(MARK_ATTRIBUTE)
    default = NO_MARK if record is None else record.default
    codes = [face[layer] if layer is not None else default for face in bm.faces if face.select]
    text = selection_text(unmapped_record("", 0, 0) if record is None else record, codes)
    box.label(text=f"Selection: {text}" if text else "Selection: no faces", icon="UV")
    box.operator(HOTSPOTUV_OT_HotspotSelectMark.bl_idname, text="Select by Mark", icon="RESTRICT_SELECT_OFF")


_CLASSES = (HOTSPOTUV_OT_HotspotPickMark, HOTSPOTUV_OT_HotspotSwapWings, HOTSPOTUV_MT_HotspotMarks, HOTSPOTUV_OT_HotspotSelectMark)


def register_island_ops() -> None:
    for cls in _CLASSES:
        if getattr(cls, "is_registered", False):
            bpy.utils.unregister_class(cls)
        bpy.utils.register_class(cls)


def unregister_island_ops() -> None:
    for cls in reversed(_CLASSES):
        if getattr(cls, "is_registered", False):
            bpy.utils.unregister_class(cls)


__all__ = (
    "HOTSPOTUV_MT_HotspotMarks",
    "HOTSPOTUV_OT_HotspotPickMark",
    "HOTSPOTUV_OT_HotspotSelectMark",
    "HOTSPOTUV_OT_HotspotSwapWings",
    "HOTSPOT_MARKUP_MISMATCH",
    "HOTSPOT_NOT_A_DECAL_MESH",
    "HOTSPOT_NOT_IN_EDIT_MODE",
    "HOTSPOT_REMAP_BY_INVERSION",
    "HOTSPOT_REMAP_BY_SEAM_ARC",
    "HOTSPOT_REMAP_FROM_BUILD",
    "HOTSPOT_SELECTION_EMPTY",
    "apply_panel_mark",
    "apply_selected_decals",
    "apply_selected_faces",
    "apply_to_decal",
    "apply_to_selection",
    "remap_name",
    "build_strip",
    "decal_state",
    "draw_island_rows",
    "register_island_ops",
    "remap_path",
    "resolve_mark",
    "unregister_island_ops",
)
