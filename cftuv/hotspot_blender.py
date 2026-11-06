"""Тонкий слой Blender режима Hotspot (HOTSPOT_MARKUP_V1, HOTSPOT_ATLAS_MAPPING_V1): текст, материал, метка нажатия и UV меша декали (строки панели здесь, настройки - `hotspot_panel`).

* Разметка - JSON в текстовом датаблоке .blend (`hotspot_markup`); материал декали называет свой датаблок свойством `cftuv_hotspot_markup`.
  Запись идёт только после проверки: непринятая разметка не пишется, прежний текст остаётся, находки возвращаются по именам.
* Режим - НЕОБЯЗАТЕЛЬНЫЙ переключатель панели декали. Выключен - кнопка и меш те же побитово: меш не получает свойства `cftuv_hotspot`, и
  писатель меша (`envelope_production_mesh`) не трогает UV. Включён - кнопка разрешает метку ДО прогона ядра (отказ по имени, ядро не считает),
  кладёт отображение в свойство НОВОГО меша, и каждая запись UV этого меша (кнопка, подтверждение и кадры живой ширины) проходит через
  `map_mesh_uvs`: превью и точный кадр отображаются одной функцией.
* Uniform trim под Hotspot не поддержан: именованный отказ кнопки `HOTSPOT_UV_POLICY_UNSUPPORTED` (и строка панели).
"""

from __future__ import annotations

import bpy
import numpy as np

from .hotspot_mapping import MESH_PROPERTY, atlas_uvs, mapping_for, mapping_from_json, mapping_to_json
from .hotspot_markup import KIND_CORNER, MarkupProblemV1, load_markup, save_markup

#: Свойство материала: имя текстового датаблока его разметки.
MATERIAL_MARKUP_PROPERTY = "cftuv_hotspot_markup"
SETTINGS_ATTRIBUTE = "hotspotuv_hotspot"
MARKUP_TEXT_MISSING = "MARKUP_TEXT_MISSING"
MARKUP_NOT_LINKED = "MARKUP_NOT_LINKED"
HOTSPOT_UV_POLICY_UNSUPPORTED = "HOTSPOT_UV_POLICY_UNSUPPORTED"
HOTSPOT_MARK_NOT_FOUND = "HOTSPOT_MARK_NOT_FOUND"
HOTSPOT_MARKUP_REFUSED = "HOTSPOT_MARKUP_REFUSED"
#: Свойство меша есть, но не читается: UV остаются UV ядра, исход назван в консоли и в числах меша.
HOTSPOT_MAPPING_RECORD_INVALID = "HOTSPOT_MAPPING_RECORD_INVALID"
#: Числа последнего отображения по session_uid меша (имя нового меша кнопки меняется при подмене): консоль кнопки и смоки; не данные Blender.
LAST_COUNTERS: dict = {}


def linked_markup_name(material) -> str:
    """Имя датаблока разметки, на которую ссылается материал; пусто - ссылки нет."""

    if material is None:
        return ""
    value = material.get(MATERIAL_MARKUP_PROPERTY)
    return value if isinstance(value, str) else ""


def link_markup(material, text_name: str) -> None:
    """Ссылка материала на разметку; пустое имя снимает ссылку."""

    if not text_name:
        if MATERIAL_MARKUP_PROPERTY in material.keys():
            del material[MATERIAL_MARKUP_PROPERTY]
        return
    material[MATERIAL_MARKUP_PROPERTY] = str(text_name)


def read_markup_text(text_name: str) -> tuple:
    """`(разметка | None, находки)` датаблока `text_name`; нет датаблока - `MARKUP_TEXT_MISSING`."""

    text = bpy.data.texts.get(text_name) if text_name else None
    if text is None:
        return None, (MarkupProblemV1(MARKUP_TEXT_MISSING, None, f"no text datablock {text_name!r}"),)
    return load_markup(text.as_string())


def read_material_markup(material) -> tuple:
    """`(имя датаблока, разметка | None, находки)` разметки материала; без ссылки - `MARKUP_NOT_LINKED`."""

    name = linked_markup_name(material)
    if not name:
        label = "no material" if material is None else f"material {material.name!r}"
        return "", None, (MarkupProblemV1(MARKUP_NOT_LINKED, None, f"{label} links no hotspot markup"),)
    markup, problems = read_markup_text(name)
    return name, markup, problems


def write_markup_text(text_name: str, markup) -> tuple:
    """Пишет разметку в датаблок (создаёт его, если нет) ТОЛЬКО принятой: находки проверки, пусто - записано."""

    payload, problems = save_markup(markup)
    if problems:
        return problems
    text = bpy.data.texts.get(text_name) or bpy.data.texts.new(text_name)
    if text.as_string() != payload:
        text.from_string(payload)
    return ()


# --------------------------------------------------------------------------
# Настройки и разрешение метки нажатия
# --------------------------------------------------------------------------


def decal_material(scene):
    """Материал декали сцены (настройка «Decal Material» кнопки) либо `None`, если его ещё нет."""

    from .envelope_production_mesh import resolve_decal_material_name

    mesh_settings = getattr(scene, "hotspotuv_decal_mesh", None)
    if mesh_settings is None:
        return None
    return bpy.data.materials.get(resolve_decal_material_name(mesh_settings.material_name, bpy.data.materials))


def resolve_press(scene) -> tuple:
    """`(отображение | None, отказ, подробность)` нажатия: режим выключен - `(None, "", "")`, иначе метка или отказ по имени ДО прогона ядра."""

    from .envelope_request_policy import ENVELOPE_UV_POLICY_DIRECT_STRIP

    settings = getattr(scene, SETTINGS_ATTRIBUTE, None)
    if settings is None or not settings.enabled:
        return None, "", ""
    mesh_settings = getattr(scene, "hotspotuv_decal_mesh", None)
    policy = getattr(mesh_settings, "uv_policy_id", ENVELOPE_UV_POLICY_DIRECT_STRIP)
    if policy != ENVELOPE_UV_POLICY_DIRECT_STRIP:
        return None, HOTSPOT_UV_POLICY_UNSUPPORTED, "Hotspot maps Straight strips only; switch UV layout to Straight strips or turn Hotspot off"
    name, markup, problems = read_material_markup(decal_material(scene))
    if markup is None or problems:
        first = problems[0].name if problems else MARKUP_NOT_LINKED
        refusal = first if first in (MARKUP_NOT_LINKED, MARKUP_TEXT_MISSING) else HOTSPOT_MARKUP_REFUSED
        detail = problems[0].detail if refusal == first and problems else "; ".join(problem.text() for problem in problems)
        return None, refusal, detail
    mapping = mapping_for(name, markup, int(settings.mark_id), bool(settings.swap_wings))
    if mapping is None:
        return None, HOTSPOT_MARK_NOT_FOUND, f"markup {name!r} has no mark {int(settings.mark_id)}"
    return mapping, "", ""


def press_record(scene) -> tuple:
    """`(запись для свойства меша | None, отказ, подробность)`: то, что кнопка кладёт в новый меш."""

    mapping, refusal, detail = resolve_press(scene)
    return (None if mapping is None else mapping_to_json(mapping)), refusal, detail


def draw_hotspot_rows(layout, context) -> None:
    """Строки Hotspot в продуктовой секции: переключатель, разметка, метка, крылья и причина, если кнопка откажет."""

    scene = context.scene
    settings = getattr(scene, SETTINGS_ATTRIBUTE, None)
    if settings is None:
        return
    box = layout.box()
    box.prop(settings, "enabled")
    if not settings.enabled:
        return
    if decal_material(scene) is None:
        box.label(text="The decal material does not exist yet: build once or create it", icon="INFO")
    else:
        box.prop_search(settings, "markup_name", bpy.data, "texts")
    box.prop(settings, "mark_choice")
    mapping, refusal, detail = resolve_press(scene)
    if mapping is not None and mapping.mark.kind == KIND_CORNER:
        box.prop(settings, "swap_wings")
    if refusal:
        box.label(text=refusal, icon="ERROR")
        if detail:
            box.label(text=detail[:200])
    elif mapping is not None:
        mark = mapping.mark
        box.label(text=f"{mark.kind} {mark.w} x {mark.h} px, up {mark.up}, texture {mapping.width} x {mapping.height}")


# --------------------------------------------------------------------------
# UV меша декали
# --------------------------------------------------------------------------


def _read(collection, name: str, dtype, size: int = 1):
    buffer = np.empty(len(collection) * size, dtype=dtype)
    collection.foreach_get(name, buffer)
    return buffer


def _uv_store(mesh, layer_name: str):
    attribute = mesh.attributes.get(layer_name)
    if attribute is not None and attribute.data_type == "FLOAT2" and attribute.domain == "CORNER":
        return attribute.data, "vector"
    return mesh.uv_layers[layer_name].data, "uv"


def map_mesh_uvs(mesh, layer_name: str = "UVMap") -> dict:
    """UV ядра в слое `layer_name` меша -> UV атласа по записи `cftuv_hotspot` меша (её кладёт кнопка). Числа отображения; они же в `LAST_COUNTERS`.

    Читается то, что лежит в меше (float32 UV ядра, вершины углов, домен и владелец граней), поэтому кадр модели, подтверждение и кнопка
    отображаются одной функцией от одних чисел. Запись не читается - UV ядра остаются, исход `HOTSPOT_MAPPING_RECORD_INVALID` назван.
    """

    try:
        mapping = mapping_from_json(str(mesh[MESH_PROPERTY]))
    except (KeyError, TypeError, ValueError) as exc:
        counters = {HOTSPOT_MAPPING_RECORD_INVALID: 1}
        print(f"[CFTUV][Hotspot] {HOTSPOT_MAPPING_RECORD_INVALID}: mesh {mesh.name!r}: {type(exc).__name__}: {exc}", flush=True)
        LAST_COUNTERS[mesh.session_uid] = counters
        return counters
    loops = len(mesh.loops)
    store, field = _uv_store(mesh, layer_name)
    uvs = _read(store, field, np.float32, 2).reshape(-1, 2)
    corner = mesh.attributes.get(".corner_vert")
    vertices = _read(corner.data, "value", np.int32) if corner is not None and corner.domain == "CORNER" else _read(mesh.loops, "vertex_index", np.int32)
    totals = _read(mesh.polygons, "loop_total", np.int32)
    group = np.zeros(len(totals), dtype=np.int64)
    for name, scale in (("cftuv_domain", 1 << 20), ("cftuv_owner", 1)):
        attribute = mesh.attributes.get(name)
        if attribute is not None and attribute.domain == "FACE" and len(attribute.data) == len(totals):
            group += _read(attribute.data, "value", np.int32).astype(np.int64) * scale
    loop_group = np.repeat(group, totals) if loops else group[:0]
    atlas, counters = atlas_uvs(uvs, vertices, loop_group, mapping)
    store.foreach_set(field, atlas.astype(np.float32).reshape(-1))
    LAST_COUNTERS[mesh.session_uid] = counters
    return counters


def counter_lines(object_name) -> tuple:
    """Строки консоли кнопки: числа последнего отображения меша объекта декали, если режим был включён."""

    decal = bpy.data.objects.get(object_name or "")
    mesh = None if decal is None else decal.data
    counters = None if mesh is None else LAST_COUNTERS.get(mesh.session_uid)
    if mesh is None or MESH_PROPERTY not in mesh.keys() or counters is None:
        return ()
    return (f"[CFTUV][Hotspot] {mesh.name}: " + ", ".join(f"{key}={value}" for key, value in counters.items()),)


__all__ = (
    "HOTSPOT_MAPPING_RECORD_INVALID",
    "HOTSPOT_MARKUP_REFUSED",
    "HOTSPOT_MARK_NOT_FOUND",
    "HOTSPOT_UV_POLICY_UNSUPPORTED",
    "LAST_COUNTERS",
    "MARKUP_NOT_LINKED",
    "MARKUP_TEXT_MISSING",
    "MATERIAL_MARKUP_PROPERTY",
    "SETTINGS_ATTRIBUTE",
    "counter_lines",
    "decal_material",
    "draw_hotspot_rows",
    "link_markup",
    "linked_markup_name",
    "map_mesh_uvs",
    "press_record",
    "read_markup_text",
    "read_material_markup",
    "resolve_press",
    "write_markup_text",
)
