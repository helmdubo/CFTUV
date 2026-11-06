"""Тонкий слой Blender режима Hotspot (HOTSPOT_MARKUP_V1, HOTSPOT_ATLAS_MAPPING_V1): текст, материал, метка нажатия и UV меша декали (строки панели здесь, настройки - `hotspot_panel`).

* Разметка - JSON в текстовом датаблоке .blend (`hotspot_markup`); материал декали называет свой датаблок свойством `cftuv_hotspot_markup`.
  Запись идёт только после проверки: непринятая разметка не пишется, прежний текст остаётся, находки возвращаются по именам.
* HOTSPOT_PANEL_EDITOR_V1: материал разметки - материал декали панели; картинка - из его узлов (Image Texture, питающий Base Color узла
  Principled, иначе первый с картинкой); «Mark Up» сам создаёт и связывает датаблок (`ensure_material_markup`) и открывает мини-редактор.
* Режим - НЕОБЯЗАТЕЛЬНЫЙ переключатель панели декали. Выключен - кнопка и меш те же побитово: меш не получает свойства `cftuv_hotspot`, и
  писатель меша (`envelope_production_mesh`) не трогает UV. Включён - кнопка разрешает метку ДО прогона ядра (отказ по имени, ядро не считает),
  кладёт отображение в свойство НОВОГО меша, и каждая запись UV этого меша (кнопка, подтверждение и кадры живой ширины) проходит через
  `map_mesh_uvs`: превью и точный кадр отображаются одной функцией.
* Uniform trim под Hotspot не поддержан: именованный отказ кнопки `HOTSPOT_UV_POLICY_UNSUPPORTED` (и строка панели).
"""

from __future__ import annotations

from dataclasses import replace

import bpy
import numpy as np

from .hotspot_islands import (
    COUNTER_DROPPED,
    MARK_ATTRIBUTE,
    anchor_tolerance,
    carry_islands,
    island_codes,
    loop_next,
    map_islands,
    paired_corner_groups,
    record_from_json,
    record_of,
    record_to_json,
    remap_atlas,
    remember_islands,
    seam_points,
    strip_from_atlas,
    unwarp_paired,
)
from .hotspot_mapping import MESH_PROPERTY, mapping_for
from .hotspot_markup import KIND_CORNER, MarkupProblemV1, empty_markup, load_markup, save_markup

#: Свойство материала: имя текстового датаблока его разметки.
MATERIAL_MARKUP_PROPERTY = "cftuv_hotspot_markup"
SETTINGS_ATTRIBUTE = "hotspotuv_hotspot"
MARKUP_TEXT_MISSING = "MARKUP_TEXT_MISSING"
MARKUP_NOT_LINKED = "MARKUP_NOT_LINKED"
HOTSPOT_UV_POLICY_UNSUPPORTED = "HOTSPOT_UV_POLICY_UNSUPPORTED"
HOTSPOT_MARK_NOT_FOUND = "HOTSPOT_MARK_NOT_FOUND"
HOTSPOT_MARKUP_REFUSED = "HOTSPOT_MARKUP_REFUSED"
#: Разметка открывается от материала (HOTSPOT_PANEL_EDITOR_V1): материала нет, у него нет картинки, у картинки нет пикселей.
HOTSPOT_MATERIAL_MISSING = "HOTSPOT_MATERIAL_MISSING"
HOTSPOT_MATERIAL_HAS_NO_IMAGE = "HOTSPOT_MATERIAL_HAS_NO_IMAGE"
HOTSPOT_IMAGE_HAS_NO_PIXELS = "HOTSPOT_IMAGE_HAS_NO_PIXELS"
#: Свойство меша есть, но не читается: UV остаются UV ядра, исход назван в консоли и в числах меша.
HOTSPOT_MAPPING_RECORD_INVALID = "HOTSPOT_MAPPING_RECORD_INVALID"
#: Числа последнего отображения по session_uid меша (имя нового меша кнопки меняется при подмене): консоль кнопки и смоки; не данные Blender.
LAST_COUNTERS: dict = {}
#: Ширина, с которой записан меш декали (`envelope_production_mesh.DECAL_WIDTH_PROPERTY`): развёртка ряда шва делит его длину на неё.
DECAL_WIDTH_KEY = "cftuv_decal_width"
#: «Apply Mark» без UV ядра сессии: сторон corner-пары, развёрнутых по ряду шва, и вершин ряда без вершины источника рядом.
COUNTER_UNWARPED = "HOTSPOT_CORNER_SIDES_UNWARPED_BY_SEAM_ARC"
COUNTER_ROW_WITHOUT_SOURCE = "HOTSPOT_SEAM_ROW_VERTEX_WITHOUT_SOURCE"
#: UV ядра последней записи писателя по session_uid меша (HOTSPOT_ISLAND_MARKS_V1): «Apply Mark» отображает от них точно, если меш показывает ровно
#: их отображение текущими кодами; после перезагрузки файла их нет - метка обращается. Только память сессии, в .blend не пишется.
KERNEL_UVS: dict = {}
KERNEL_UVS_SIZE = 8


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
# Текстура материала и разметка материала (HOTSPOT_PANEL_EDITOR_V1)
# --------------------------------------------------------------------------


def _upstream_image_node(socket):
    """Первый узел Image Texture с картинкой выше по связям сокета (обход в ширину, приглушённые связи не идут)."""

    queue, seen = [link.from_node for link in getattr(socket, "links", ()) if not getattr(link, "is_muted", False)], set()
    while queue:
        node = queue.pop(0)
        if id(node) in seen:
            continue
        seen.add(id(node))
        if node.type == "TEX_IMAGE" and node.image is not None:
            return node
        for upstream in node.inputs:
            queue.extend(link.from_node for link in getattr(upstream, "links", ()) if not getattr(link, "is_muted", False))
    return None


def material_images(material) -> tuple:
    """Картинки узлов Image Texture материала: та, что питает Base Color узла Principled, первой, затем остальные в порядке узлов, без повторов."""

    tree = getattr(material, "node_tree", None) if material is not None and getattr(material, "use_nodes", True) else None
    if tree is None:
        return ()
    nodes = list(tree.nodes)
    ordered = []
    for node in nodes:
        base = node.inputs.get("Base Color") if node.type == "BSDF_PRINCIPLED" else None
        found = None if base is None else _upstream_image_node(base)
        if found is not None:
            ordered.append(found)
    ordered += [node for node in nodes if node.type == "TEX_IMAGE" and node.image is not None]
    images, names = [], set()
    for node in ordered:
        if node.image.name not in names:
            names.add(node.image.name)
            images.append(node.image)
    return tuple(images)


def resolve_material_image(material, preferred: str = "", markup_image: str = "") -> tuple:
    """`(картинка | None, отказ, подробность, кандидаты)`: выбранная, иначе картинка разметки, иначе первая; отказ - по имени."""

    if material is None:
        return None, HOTSPOT_MATERIAL_MISSING, "pick an existing material that has an Image Texture node", ()
    candidates = material_images(material)
    if not candidates:
        return None, HOTSPOT_MATERIAL_HAS_NO_IMAGE, f"material {material.name!r} has no Image Texture node with an image", ()
    by_name = {image.name: image for image in candidates}
    image = by_name.get(preferred) or by_name.get(markup_image) or candidates[0]
    if not image.size[0] or not image.size[1]:
        return None, HOTSPOT_IMAGE_HAS_NO_PIXELS, f"image {image.name!r} has no pixels (missing file?)", candidates
    return image, "", "", candidates


def ensure_material_markup(material, image) -> tuple:
    """`(имя датаблока разметки, создан ли, находки)`: связанная разметка материала, иначе `<картинка>.hotspot`; нет датаблока - пустая разметка
    картинки; материал ссылается на него. Пользователь не трогает ни текст, ни ссылку."""

    name = linked_markup_name(material) or f"{image.name}.hotspot"
    created = bpy.data.texts.get(name) is None
    if created:
        problems = write_markup_text(name, empty_markup(int(image.size[0]), int(image.size[1]), image.name))
        if problems:
            return "", False, problems
    link_markup(material, name)
    return name, created, ()


#: HOTSPOT_BACKDROP_V1: исход выбора подложки мини-окна - показать картинку как есть или (разметка без меток) перевести разметку в её размер.
BACKDROP_SHOW = "show"
BACKDROP_REBASE = "rebase"
#: Подложка другого размера, чем пиксели разметки с метками: отказ по имени, метки не растягиваются на чужую картинку молча.
HOTSPOT_BACKDROP_SIZE_MISMATCH = "HOTSPOT_BACKDROP_SIZE_MISMATCH"


def backdrop_label(image, size=None) -> str:
    """Пункт списка подложек: `имя (Ш x В px)`; другой размер, чем у разметки, назван в самом пункте."""

    label = f"{image.name} ({image.size[0]} x {image.size[1]} px)"
    return label if size is None or tuple(image.size[:2]) == tuple(size) else f"{label} - not the markup's {size[0]} x {size[1]}"


def backdrop_switch(image_size, markup_size, mark_count: int) -> tuple:
    """`(исход, отказ, подробность)` подложки размера `image_size` у разметки `markup_size` с `mark_count` метками (HOTSPOT_BACKDROP_V1).

    Тот же размер - показать. Другой: меток нет - разметка переводится в размер картинки (`BACKDROP_REBASE`: пикселей, которые сдвинулись бы,
    нет); метки есть - отказ `HOTSPOT_BACKDROP_SIZE_MISMATCH`: метка - целые пиксели своей текстуры, растягивать её на картинку другого
    размера молча нельзя, а пересчитывать пиксели - значит менять метки, которых пользователь не трогал."""

    width, height = int(image_size[0]), int(image_size[1])
    if (width, height) == (int(markup_size[0]), int(markup_size[1])):
        return BACKDROP_SHOW, "", ""
    if not width or not height:
        return "", HOTSPOT_IMAGE_HAS_NO_PIXELS, "the image has no pixels (missing file?)"
    if not mark_count:
        return BACKDROP_REBASE, "", ""
    return "", HOTSPOT_BACKDROP_SIZE_MISMATCH, (
        f"the markup is {int(markup_size[0])} x {int(markup_size[1])} px with {mark_count} mark(s), the image is {width} x {height} px: "
        "marks are pixels of their texture and are never scaled"
    )


def pick_backdrop(candidates, first, markup) -> tuple:
    """`(картинка | None, отказ, подробность)` подложки «Mark Up»: `first` (выбор пользователя, иначе картинка разметки, иначе первая), если
    её размер - размер разметки; иначе картинка разметки или первая картинка материала того же размера; иначе - `backdrop_switch` от `first`."""

    if markup is None or first is None:
        return first, "", ""
    size = (int(markup.width), int(markup.height))
    named = next((image for image in candidates if image.name == markup.image), None)
    for image in (first, named, *candidates):
        if image is not None and tuple(image.size[:2]) == size:
            return image, "", ""
    action, refusal, detail = backdrop_switch(tuple(first.size[:2]), size, len(markup.marks))
    return (first if action else None), refusal, detail


def scene_material_image(scene) -> tuple:
    """`(материал, картинка | None, отказ, подробность, кандидаты)` разметки сцены: материал декали и его картинка-подложка (выбор подложки в
    мини-окне, иначе картинка разметки, иначе первая; размер - размер разметки, HOTSPOT_BACKDROP_V1)."""

    material = decal_material(scene)
    settings = getattr(scene, SETTINGS_ATTRIBUTE, None)
    _name, markup, _problems = read_material_markup(material)
    image, refusal, detail, candidates = resolve_material_image(material, getattr(settings, "image_name", ""), "" if markup is None else markup.image)
    if image is not None:
        image, refusal, detail = pick_backdrop(candidates, image, markup)
    return material, image, refusal, detail, candidates


#: HOTSPOT_EDITOR_WINDOW_V1: где рисуются метки - область мини-окна разметки или (по выбору пользователя) другой Image Editor с её картинкой.
OVERLAY_EDITOR = "editor"
OVERLAY_ELSEWHERE = "elsewhere"


def overlay_where(area_key, editor_key, shown_image: str, backdrops, show_elsewhere: bool) -> str:
    """Где рисовать метки (HOTSPOT_EDITOR_WINDOW_V1): `OVERLAY_EDITOR` - область мини-окна (ключ `(окно, экран, область)` совпал целиком),
    `OVERLAY_ELSEWHERE` - другой Image Editor с картинкой разметки или подложки, если пользователь включил «Show marks in UV Editor»,
    иначе пусто: ни второй Image Editor с той же картинкой, ни UV Editor декали меток не показывают."""

    if editor_key and None not in editor_key and tuple(area_key) == tuple(editor_key):
        return OVERLAY_EDITOR
    if show_elsewhere and shown_image and shown_image in {name for name in backdrops if name}:
        return OVERLAY_ELSEWHERE
    return ""


def split_decal_targets(objects, marker: str) -> tuple:
    """«Apply Mark» в Object Mode (HOTSPOT_OBJECT_APPLY_V1): `(декали, имена пропущенных)` выделения. Декаль - меш с маркером декали
    (`marker` - свойство объекта, а не имя `<источник>.CFTUV_Decal`); остальные - по имени в счёт HOTSPOT_NOT_A_DECAL_MESH. Объекты с одним
    мешем (связанные копии) - одна декаль: метка ложится на меш один раз."""

    decals, skipped, meshes = [], [], set()
    for obj in objects:
        if getattr(obj, "type", "") != "MESH" or marker not in obj.keys():
            skipped.append(obj.name)
            continue
        key = id(obj.data) if not hasattr(obj.data, "as_pointer") else obj.data.as_pointer()
        if key not in meshes:
            meshes.add(key)
            decals.append(obj)
    return tuple(decals), tuple(skipped)


#: HOTSPOT_APPLY_ON_PICK_V1: на что ложится метка, только что выбранная в панели (или Swap wings): грани выделения декали в Edit Mode, все
#: острова выделенных декалей в Object Mode, иначе ни на что - метка только выбрана для следующей сборки.
PICK_FACES = "faces"
PICK_DECALS = "decals"
PICK_NONE = ""
#: Выбор метки вошёл сам в себя, пока прежний выбор ещё применялся: второй раз не применяется (имя - в консоли).
HOTSPOT_APPLY_REENTERED = "HOTSPOT_APPLY_REENTERED"
#: Меню выбора метки панели (пункт - оператор `hotspotuv.hotspot_pick_mark`).
MARKS_MENU = "HOTSPOTUV_MT_hotspot_marks"


def pick_target(mode: str, active_decal: bool, selected_faces: int, selected_decals: int) -> str:
    """Цель выбора метки (HOTSPOT_APPLY_ON_PICK_V1): Edit Mode на декали с выделенными гранями - `PICK_FACES`, Object Mode с выделенными
    декалями - `PICK_DECALS`, иначе `PICK_NONE` (Edit Mode источника перед сборкой, пустое выделение, Sculpt...): не ошибка, а выбор метки."""

    if mode == "EDIT_MESH":
        return PICK_FACES if active_decal and selected_faces > 0 else PICK_NONE
    if mode == "OBJECT":
        return PICK_DECALS if selected_decals > 0 else PICK_NONE
    return PICK_NONE


def pick_idle_line(mode: str, active_decal: bool) -> str:
    """Строка панели, когда выбор метки ничего не применил: что выбрано и почему ничего не тронуто."""

    if mode == "EDIT_MESH":
        why = "no faces of the decal selected" if active_decal else "Edit Mode is not on a decal"
    elif mode == "OBJECT":
        why = "no decal selected"
    else:
        why = f"{mode.replace('_', ' ').title()} applies no mark"
    return f"Mark set for the next Build Decal Mesh ({why})"


def mark_label(markup, mark_id: int) -> str:
    """Подпись меню выбора метки: пункт `mark_items` выбранной метки, иначе её номер и что её в разметке нет."""

    item = next((item for item in mark_items(markup) if item[3] == int(mark_id)), None)
    return item[1] if item is not None else f"{int(mark_id)}: not in the markup"


def mark_items(markup) -> list:
    """Пункты выбора метки кнопки: `id: name (kind)`; нет меток - один пункт «нет меток»."""

    items = [] if markup is None else [(str(mark.id), f"{mark.id}: {mark.name} ({mark.kind})", f"{mark.kind}, up {mark.up}, {mark.w} x {mark.h} px", mark.id) for mark in markup.marks]
    return items or [("-1", "no marks", "the markup of the material has no marks yet: click Mark Up", -1)]


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
    """`(запись для свойства меша | None, отказ, подробность)`: то, что кнопка кладёт в новый меш (запись v2 без островов, HOTSPOT_ISLAND_MARKS_V1;
    острова прежнего меша декали переносит `carry_record` писателя)."""

    mapping, refusal, detail = resolve_press(scene)
    return (None if mapping is None else record_to_json(record_of(mapping))), refusal, detail


def draw_hotspot_rows(layout, context) -> None:
    """Строки Hotspot в продуктовой секции: переключатель, разметка, метка, крылья и причина, если кнопка откажет."""

    scene = context.scene
    settings = getattr(scene, SETTINGS_ATTRIBUTE, None)
    if settings is None:
        return
    mesh_settings = getattr(scene, "hotspotuv_decal_mesh", None)
    box = layout.box()
    box.prop(settings, "enabled")
    if not settings.enabled or mesh_settings is None:
        return
    # Материал разметки И есть материал декали: метка берётся из текстуры, которую декаль покажет (HOTSPOT_PANEL_EDITOR_V1).
    box.prop_search(mesh_settings, "material_name", bpy.data, "materials", text="Material", results_are_suggestions=True)
    # Текстура-подложка выбирается в самом мини-окне (HOTSPOT_BACKDROP_V1); здесь - только отказ, если подложки нет вовсе.
    _material, image, image_refusal, image_detail, _candidates = scene_material_image(scene)
    if image is None:
        _refusal_rows(box, image_refusal, image_detail)
    row = box.row()
    row.scale_y = 1.4
    row.enabled = image is not None
    row.operator("hotspotuv.hotspot_mark_up", text="Mark Up", icon="GREASEPENCIL")
    mapping, refusal, detail = resolve_press(scene)
    if refusal == MARKUP_NOT_LINKED:  # до первой разметки выбирать нечего: имя отказа кнопки и что сделать
        if image is not None:
            box.label(text=f"{MARKUP_NOT_LINKED}: no marks yet, click Mark Up", icon="INFO")
        return
    # HOTSPOT_APPLY_ON_PICK_V1: выбор метки и Swap wings - операторы (один шаг Undo), они же кладут метку на выделение; кнопки Apply нет.
    _name, markup, _problems = read_material_markup(decal_material(scene))
    row = box.row(align=True)
    row.label(text="Mark")
    row.menu(MARKS_MENU, text=mark_label(markup, settings.mark_id))
    if mapping is not None and mapping.mark.kind == KIND_CORNER:
        swap = bool(settings.swap_wings)
        box.operator("hotspotuv.hotspot_swap_wings", text="Swap wings", icon="CHECKBOX_HLT" if swap else "CHECKBOX_DEHLT", depress=swap)
    if settings.apply_note:
        box.label(text=settings.apply_note[:200], icon="INFO")
    if refusal and refusal != image_refusal:
        _refusal_rows(box, refusal, detail)
    elif mapping is not None:
        mark = mapping.mark
        box.label(text=f"{mark.kind} {mark.w} x {mark.h} px, up {mark.up}, texture {mapping.width} x {mapping.height}")
    from .hotspot_island_ops import draw_island_rows

    draw_island_rows(box, context)  # HOTSPOT_ISLAND_MARKS_V1: метка выделенных островов готовой декали


def _refusal_rows(box, refusal: str, detail: str) -> None:
    box.label(text=refusal, icon="ERROR")
    if detail:
        box.label(text=detail[:200])


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


def mesh_record(mesh) -> tuple:
    """`(запись v2 | None, ошибка)` свойства `cftuv_hotspot` меша; свойства нет - `(None, "")`."""

    if MESH_PROPERTY not in mesh.keys():
        return None, ""
    try:
        return record_from_json(str(mesh[MESH_PROPERTY])), ""
    except (KeyError, TypeError, ValueError, AttributeError) as exc:
        return None, f"{type(exc).__name__}: {exc}"


def mesh_arrays(mesh) -> tuple:
    """`(вершины углов, число углов граней, группа грани)` меша декали: группа - `домен * 2^20 + владелец` (сторона шва отображения, остров)."""

    corner = mesh.attributes.get(".corner_vert")
    vertices = _read(corner.data, "value", np.int32) if corner is not None and corner.domain == "CORNER" else _read(mesh.loops, "vertex_index", np.int32)
    totals = _read(mesh.polygons, "loop_total", np.int32)
    group = np.zeros(len(totals), dtype=np.int64)
    for name, scale in (("cftuv_domain", 1 << 20), ("cftuv_owner", 1)):
        attribute = mesh.attributes.get(name)
        if attribute is not None and attribute.domain == "FACE" and len(attribute.data) == len(totals):
            group += _read(attribute.data, "value", np.int32).astype(np.int64) * scale
    return vertices.astype(np.int64), totals, group


def _positions(mesh):
    attribute = mesh.attributes.get("position")
    if attribute is not None and attribute.data_type == "FLOAT_VECTOR" and attribute.domain == "POINT":
        return _read(attribute.data, "vector", np.float32, 3).reshape(-1, 3).astype(np.float64)
    return _read(mesh.vertices, "co", np.float32, 3).reshape(-1, 3).astype(np.float64)


def face_codes(mesh, count: int):
    """Коды меток граней (атрибут `cftuv_hotspot_mark`) либо `None`: атрибута нет или он не того вида и размера (грани пересобраны)."""

    attribute = mesh.attributes.get(MARK_ATTRIBUTE)
    if attribute is None or attribute.domain != "FACE" or attribute.data_type != "INT" or len(attribute.data) != count:
        return None
    return _read(attribute.data, "value", np.int32).astype(np.int64)


def current_atlas(mesh, layer_name: str = "UVMap"):
    """UV слоя меша, (L, 2) float32."""

    store, field = _uv_store(mesh, layer_name)
    return _read(store, field, np.float32, 2).reshape(-1, 2)


def recovered_strip(mesh, record, codes, layer_name: str = "UVMap"):
    """UV ядра островов по текущим UV слоя: обращение метки каждого острова (`strip_from_atlas`; `NO_MARK` - как есть)."""

    _vertices, totals, group = mesh_arrays(mesh)
    return strip_from_atlas(current_atlas(mesh, layer_name), np.repeat(group, totals), np.repeat(codes, totals), record)


def row_positions(mesh, source, vertices) -> tuple:
    """`(позиции вершин меша, вершин ряда без вершины источника)`: вершины `vertices` (ряд шва) - ближайшая вершина источника (декаль - потомок
    источника с единичным преобразованием: координаты общие), чтобы длина ряда была длиной цепи источника, а не сдвинутой митрой копии."""

    positions = _positions(mesh)
    vertices = np.unique(np.asarray(vertices, dtype=np.int64))
    data = getattr(source, "data", None)
    if data is None or not hasattr(data, "vertices") or not vertices.shape[0] or not len(data.vertices):
        return positions, int(vertices.shape[0])
    width = float(mesh.get(DECAL_WIDTH_KEY, 0.0)) if hasattr(mesh, "get") else 0.0
    reach = max(0.5 * width, 1e-4)
    points = positions[vertices]
    low, high = points.min(axis=0) - reach, points.max(axis=0) + reach
    source_points = _read(data.vertices, "co", np.float32, 3).reshape(-1, 3).astype(np.float64)
    source_points = source_points[np.all((source_points >= low) & (source_points <= high), axis=1)]
    lost = 0
    if not source_points.shape[0]:
        return positions, int(vertices.shape[0])
    for start in range(0, vertices.shape[0], 256):
        chunk = points[start:start + 256]
        distance = np.linalg.norm(chunk[:, None, :] - source_points[None, :, :], axis=2)
        nearest = np.argmin(distance, axis=1)
        close = distance[np.arange(chunk.shape[0]), nearest] <= reach
        lost += int(np.count_nonzero(~close))
        positions[vertices[start:start + 256][close]] = source_points[nearest[close]]
    return positions, lost


def exact_strip(mesh, record, codes, source=None, layer_name: str = "UVMap") -> tuple:
    """`(UV ядра, числа)` по текущим UV слоя: обращение меток; у сторон corner-пары `along` выровнен - он разворачивается по длине ряда шва
    (`unwarp_paired`, длина - по вершинам источника `source`, без него - по самому мешу). Числа: сколько сторон развёрнуто и сколько вершин ряда
    не нашли вершину источника."""

    vertices, totals, group = mesh_arrays(mesh)
    loop_group, loop_code = np.repeat(group, totals), np.repeat(codes, totals)
    strip = strip_from_atlas(current_atlas(mesh, layer_name), loop_group, loop_code, record)
    paired = paired_corner_groups(vertices, loop_group, loop_code, strip[:, 1], record)
    if not paired.shape[0]:
        return strip, {COUNTER_UNWARPED: 0}
    row = np.isin(loop_group, paired) & (strip[:, 1] == 0.0)
    positions, lost = row_positions(mesh, source, vertices[row])
    strip, done = unwarp_paired(strip, vertices, loop_group, paired, positions, float(mesh.get(DECAL_WIDTH_KEY, 0.0)))
    return strip, {COUNTER_UNWARPED: done, COUNTER_ROW_WITHOUT_SOURCE: lost}


def _write_attribute(mesh, name: str, kind: str, domain: str, field: str, values) -> None:
    """Атрибут (создаётся, если нет; иного вида - заводится заново). Зовётся ПОСЛЕ записи UV: новый слой может сдвинуть массив слоёв."""

    attribute = mesh.attributes.get(name)
    if attribute is not None and (attribute.data_type != kind or attribute.domain != domain):
        mesh.attributes.remove(attribute)
        attribute = None
    if attribute is None:
        attribute = mesh.attributes.new(name=name, type=kind, domain=domain)
    attribute.data.foreach_set(field, values)


def _seam_points(mesh, strip, vertices, totals, group) -> tuple:
    positions = _positions(mesh)
    points, groups = seam_points(strip[:, 1], vertices, loop_next(totals), np.repeat(group, totals), positions)
    return points, groups, anchor_tolerance(positions)


def _write_uvs_and_codes(mesh, layer_name, atlas, face_code, counters) -> None:
    store, field = _uv_store(mesh, layer_name)
    store.foreach_set(field, np.asarray(atlas, dtype=np.float32).reshape(-1))
    _write_attribute(mesh, MARK_ATTRIBUTE, "INT", "FACE", "value", np.asarray(face_code, dtype=np.int32))
    LAST_COUNTERS[mesh.session_uid] = counters


def map_with_codes(mesh, record, strip, codes, layer_name: str = "UVMap") -> tuple:
    """`(коды граней, числа)`: UV атласа по UV ядра `strip` и кодам граней (`None` - по якорям записи) в слой, коды - в атрибут граней."""

    vertices, totals, group = mesh_arrays(mesh)
    points = groups = None
    tolerance = 0.0
    if codes is None and record.islands:
        points, groups, tolerance = _seam_points(mesh, strip, vertices, totals, group)
    face_code, counters = island_codes(record, group, codes, points, groups, tolerance)
    atlas, numbers = map_islands(strip, vertices, np.repeat(group, totals), np.repeat(face_code, totals), record)
    counters.update(numbers)
    _write_uvs_and_codes(mesh, layer_name, atlas, face_code, counters)
    return face_code, counters


def remap_mesh(mesh, old, new, old_codes, new_codes, layer_name: str = "UVMap", strip=None) -> dict:
    """«Apply Mark»: UV слоя `M_new(M_old^-1(UV))` по кодам граней до и после (`remap_atlas`; `strip` - готовые UV ядра), коды - в атрибут."""

    vertices, totals, group = mesh_arrays(mesh)
    loop_group = np.repeat(group, totals)
    atlas, counters = remap_atlas(current_atlas(mesh, layer_name), vertices, loop_group, np.repeat(old_codes, totals), np.repeat(new_codes, totals), old, new, strip)
    _face, numbers = island_codes(new, group, new_codes)
    counters.update(numbers)
    _write_uvs_and_codes(mesh, layer_name, atlas, new_codes, counters)
    return counters


def map_mesh_uvs(mesh, layer_name: str = "UVMap") -> dict:
    """UV ядра в слое `layer_name` меша -> UV атласа по записи `cftuv_hotspot` меша (её кладёт кнопка). Числа отображения; они же в `LAST_COUNTERS`.

    Читается то, что лежит в меше (float32 UV ядра, вершины углов, домен и владелец граней), поэтому кадр модели, подтверждение и кнопка
    отображаются одной функцией от одних чисел. Метки островов - из атрибута граней (меш тот же), иначе по якорям записи (грани пересобраны;
    HOTSPOT_ISLAND_MARKS_V1). Запись не читается - UV ядра остаются, исход `HOTSPOT_MAPPING_RECORD_INVALID` назван.
    """

    record, error = mesh_record(mesh)
    if record is None:
        counters = {HOTSPOT_MAPPING_RECORD_INVALID: 1}
        print(f"[CFTUV][Hotspot] {HOTSPOT_MAPPING_RECORD_INVALID}: mesh {mesh.name!r}: {error or 'no record'}", flush=True)
        LAST_COUNTERS[mesh.session_uid] = counters
        return counters
    kernel = current_atlas(mesh, layer_name)
    KERNEL_UVS.pop(mesh.session_uid, None)
    if len(KERNEL_UVS) >= KERNEL_UVS_SIZE:
        KERNEL_UVS.pop(next(iter(KERNEL_UVS)))
    KERNEL_UVS[mesh.session_uid] = kernel
    return map_with_codes(mesh, record, kernel, face_codes(mesh, len(mesh.polygons)), layer_name)[1]


def remembered_record(mesh, record, layer_name: str = "UVMap"):
    """Запись с островами НЕ по умолчанию по атрибуту граней меша (его правда; запись могла отстать после Undo в Edit Mode). Атрибута нет - та же запись."""

    vertices, totals, group = mesh_arrays(mesh)
    codes = face_codes(mesh, len(totals))
    if codes is None:
        return record
    if not np.any(codes != record.default):
        return replace(record, islands=())
    strip = recovered_strip(mesh, record, codes, layer_name)
    points, groups, _tolerance = _seam_points(mesh, strip, vertices, totals, group)
    return remember_islands(record, group, codes, points, groups)



def remember_mesh_islands(mesh) -> None:
    """Перед пересборкой граней (`_clear_mesh_in_place`): метки островов из атрибута граней - в запись меша, чтобы новые грани нашли их по якорям."""

    record, _error = mesh_record(mesh)
    if record is None:
        return
    text = record_to_json(remembered_record(mesh, record))
    if str(mesh[MESH_PROPERTY]) != text:
        mesh[MESH_PROPERTY] = text


def carry_record(old_mesh, text: str) -> str:
    """Запись кнопки `text` поверх прежнего меша декали: метка по умолчанию - выбор панели, острова прежнего меша - их метки (свежие из разметки)."""

    old, _error = mesh_record(old_mesh) if old_mesh is not None else (None, "")
    if old is None:
        return text
    new = record_from_json(text)
    markup, _problems = read_markup_text(new.markup)
    record, dropped = carry_islands(remembered_record(old_mesh, old), new, () if markup is None else markup.marks)
    if dropped:
        print(f"[CFTUV][Hotspot] {COUNTER_DROPPED}={dropped}: the previous decal mesh used markup {old.markup!r} ({old.width} x {old.height}), the press uses {new.markup!r} ({new.width} x {new.height})", flush=True)
    if record.islands:
        print(f"[CFTUV][Hotspot] {len(record.islands)} island mark(s) of the previous decal mesh carried over", flush=True)
    return record_to_json(record)


def counter_lines(object_name) -> tuple:
    """Строки консоли кнопки: числа последнего отображения меша объекта декали, если режим был включён."""

    decal = bpy.data.objects.get(object_name or "")
    mesh = None if decal is None else decal.data
    counters = None if mesh is None else LAST_COUNTERS.get(mesh.session_uid)
    if mesh is None or MESH_PROPERTY not in mesh.keys() or counters is None:
        return ()
    return (f"[CFTUV][Hotspot] {mesh.name}: " + ", ".join(f"{key}={value}" for key, value in counters.items()),)


__all__ = (
    "HOTSPOT_IMAGE_HAS_NO_PIXELS",
    "HOTSPOT_MAPPING_RECORD_INVALID",
    "HOTSPOT_MARKUP_REFUSED",
    "HOTSPOT_MARK_NOT_FOUND",
    "HOTSPOT_MATERIAL_HAS_NO_IMAGE",
    "HOTSPOT_MATERIAL_MISSING",
    "HOTSPOT_UV_POLICY_UNSUPPORTED",
    "KERNEL_UVS",
    "LAST_COUNTERS",
    "MARKUP_NOT_LINKED",
    "MARKUP_TEXT_MISSING",
    "MATERIAL_MARKUP_PROPERTY",
    "OVERLAY_EDITOR",
    "OVERLAY_ELSEWHERE",
    "SETTINGS_ATTRIBUTE",
    "BACKDROP_REBASE",
    "BACKDROP_SHOW",
    "HOTSPOT_BACKDROP_SIZE_MISMATCH",
    "HOTSPOT_APPLY_REENTERED",
    "MARKS_MENU",
    "PICK_DECALS",
    "PICK_FACES",
    "PICK_NONE",
    "mark_label",
    "pick_idle_line",
    "pick_target",
    "backdrop_label",
    "backdrop_switch",
    "overlay_where",
    "pick_backdrop",
    "split_decal_targets",
    "carry_record",
    "COUNTER_ROW_WITHOUT_SOURCE",
    "COUNTER_UNWARPED",
    "DECAL_WIDTH_KEY",
    "counter_lines",
    "current_atlas",
    "decal_material",
    "draw_hotspot_rows",
    "ensure_material_markup",
    "exact_strip",
    "face_codes",
    "link_markup",
    "linked_markup_name",
    "map_mesh_uvs",
    "map_with_codes",
    "mark_items",
    "material_images",
    "mesh_arrays",
    "mesh_record",
    "press_record",
    "recovered_strip",
    "remap_mesh",
    "remember_mesh_islands",
    "remembered_record",
    "row_positions",
    "read_markup_text",
    "read_material_markup",
    "resolve_material_image",
    "resolve_press",
    "scene_material_image",
    "write_markup_text",
)
