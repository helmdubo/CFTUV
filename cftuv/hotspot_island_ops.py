"""Метки по островам готовой декали (HOTSPOT_ISLAND_MARKS_V1): Edit Mode на `<источник>.CFTUV_Decal`, выделить грани, выбрать метку в строках
Hotspot, «Apply Mark to Selection» - UV островов выделения сразу ложатся в метку, без пересборки. Чистая часть - `hotspot_islands`.

* Выделение расширяется до целых островов (сторона шва домена и владельца); corner-метка - и до соседей по шву, крылья бывают только парой.
  После нажатия выделены именно острова, которые метку получили.
* UV: `M_new(M_old^-1(UV))` - текущая метка острова обращается, новая прикладывается; острова, чьи код и метка не изменились, остаются
  побитово. Декаль, собранная без Hotspot, - тождественный случай (UV слоя и есть UV ядра): первое нажатие кладёт ей запись (по умолчанию
  `NO_MARK`), пока на ней не нажали, она та же побитово.
* Запись Edit Mode: оператор выходит в Object Mode, пишет UV, коды граней и запись меша и возвращается; один шаг Undo возвращает UV и коды
  граней (правду островов), запись меша догоняет их перед следующей пересборкой граней.
* Отказы по именам: не декаль (HOTSPOT_NOT_A_DECAL_MESH), не Edit Mode (HOTSPOT_NOT_IN_EDIT_MODE), пустое выделение
  (HOTSPOT_SELECTION_EMPTY), метки нет в разметке (HOTSPOT_MARK_NOT_FOUND), разметка панели не та, что у декали (HOTSPOT_MARKUP_MISMATCH),
  запись меша не читается (HOTSPOT_MAPPING_RECORD_INVALID).
"""

from __future__ import annotations

import bmesh
import bpy
import numpy as np

from .envelope_production_mesh import DECAL_REVISION_PROPERTY, DECAL_UV_LAYER
from .hotspot_blender import (
    HOTSPOT_MAPPING_RECORD_INVALID,
    HOTSPOT_MARK_NOT_FOUND,
    KERNEL_UVS,
    LAST_COUNTERS,
    SETTINGS_ATTRIBUTE,
    decal_material,
    COUNTER_UNWARPED,
    current_atlas,
    exact_strip,
    face_codes,
    map_with_codes,
    mesh_arrays,
    mesh_record,
    read_material_markup,
    remap_mesh,
    remembered_record,
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
    refresh_marks,
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


def apply_to_selection(mesh, record, markup, mark, swap, source=None) -> tuple:
    """Object Mode: `(маска граней островов | None, числа | (отказ, подробность))` - метка выделенным островам, запись меша. UV ядра - путь
    `remap_path`: UV ядра последней записи сессии, если меш показывает ровно их (HOTSPOT_REMAP_FROM_BUILD), иначе обращение меток, у сторон
    corner-пары - с развёрткой по длине ряда шва по вершинам источника `source` (HOTSPOT_REMAP_BY_SEAM_ARC; HOTSPOT_REMAP_BY_INVERSION)."""

    _vertices, totals, group = mesh_arrays(mesh)
    selected = np.zeros(len(mesh.polygons), dtype=bool)
    mesh.polygons.foreach_get("select", selected)
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


class HOTSPOTUV_OT_HotspotApplyMark(bpy.types.Operator):
    bl_idname = "hotspotuv.hotspot_apply_mark"
    bl_label = "Apply Mark to Selection"
    bl_description = "Map the UVs of the selected decal islands (whole pChain sides; corner marks: both sides of the seam) into the mark of the panel, without a rebuild"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        obj, record, refusal, detail = decal_state(context)
        if not refusal and obj.mode != "EDIT":
            refusal, detail = HOTSPOT_NOT_IN_EDIT_MODE, "enter Edit Mode on the decal and select faces"
        if not refusal:
            mark, swap, name, markup, refusal, detail = resolve_mark(context, record)
        if not refusal and not obj.data.total_face_sel:
            refusal, detail = HOTSPOT_SELECTION_EMPTY, "select faces of the decal"
        if refusal:
            return _refused(self, refusal, detail)
        record = unmapped_record(name, markup.width, markup.height) if record is None else record
        bpy.ops.object.mode_set(mode="OBJECT")
        try:
            mask, counters = apply_to_selection(obj.data, record, markup, mark, swap, obj.parent)
        finally:
            bpy.ops.object.mode_set(mode="EDIT")
        if mask is None:
            return _refused(self, *counters)
        faces = _select_faces(obj.data, mask)
        line = (
            f"{mark.id}: {mark.name} ({mark.kind}{', swapped' if swap and mark.kind == 'corner' else ''}) -> {counters[COUNTER_SELECTED]} island(s), {faces} face(s); "
            f"{COUNTER_EXPANDED}={counters[COUNTER_EXPANDED]}, {COUNTER_UNPAIRED}={counters.get(COUNTER_UNPAIRED, 0)}, "
            f"{next(name for name in (HOTSPOT_REMAP_FROM_BUILD, HOTSPOT_REMAP_BY_SEAM_ARC, HOTSPOT_REMAP_BY_INVERSION) if name in counters)}"
        )
        print(f"[CFTUV][Hotspot] {obj.data.name}: {line}; " + ", ".join(f"{key}={value}" for key, value in counters.items()), flush=True)
        self.report({"INFO"}, line)
        return {"FINISHED"}


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


def draw_island_rows(layout, context) -> None:
    """Строки островов в строках Hotspot: только у активной декали; в Edit Mode - метка выделения и кнопки."""

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
        box.label(text="Island marks: Edit Mode on the decal, select faces, Apply Mark", icon="INFO")
        return
    bm = bmesh.from_edit_mesh(obj.data)
    layer = bm.faces.layers.int.get(MARK_ATTRIBUTE)
    default = NO_MARK if record is None else record.default
    codes = [face[layer] if layer is not None else default for face in bm.faces if face.select]
    text = selection_text(unmapped_record("", 0, 0) if record is None else record, codes)
    box.label(text=f"Selection: {text}" if text else "Selection: no faces", icon="UV")
    row = box.row(align=True)
    row.operator(HOTSPOTUV_OT_HotspotApplyMark.bl_idname, icon="CHECKMARK")
    row.operator(HOTSPOTUV_OT_HotspotSelectMark.bl_idname, text="Select by Mark", icon="RESTRICT_SELECT_OFF")


_CLASSES = (HOTSPOTUV_OT_HotspotApplyMark, HOTSPOTUV_OT_HotspotSelectMark)


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
    "HOTSPOTUV_OT_HotspotApplyMark",
    "HOTSPOTUV_OT_HotspotSelectMark",
    "HOTSPOT_MARKUP_MISMATCH",
    "HOTSPOT_NOT_A_DECAL_MESH",
    "HOTSPOT_NOT_IN_EDIT_MODE",
    "HOTSPOT_REMAP_BY_INVERSION",
    "HOTSPOT_REMAP_BY_SEAM_ARC",
    "HOTSPOT_REMAP_FROM_BUILD",
    "HOTSPOT_SELECTION_EMPTY",
    "apply_to_selection",
    "build_strip",
    "decal_state",
    "draw_island_rows",
    "register_island_ops",
    "remap_path",
    "resolve_mark",
    "unregister_island_ops",
)
