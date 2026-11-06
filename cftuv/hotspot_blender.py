"""Тонкий слой Blender разметки Hotspot (HOTSPOT_MARKUP_V1): текстовый датаблок и ссылка материала.

Разметка - JSON в текстовом датаблоке .blend (`hotspot_markup`); материал называет свой датаблок свойством `cftuv_hotspot_markup`.
Запись идёт только после проверки: непринятая разметка не пишется, прежний текст остаётся, находки возвращаются по именам.
"""

from __future__ import annotations

import bpy

from .hotspot_markup import MarkupProblemV1, load_markup, save_markup

#: Свойство материала: имя текстового датаблока его разметки.
MATERIAL_MARKUP_PROPERTY = "cftuv_hotspot_markup"
MARKUP_TEXT_MISSING = "MARKUP_TEXT_MISSING"
MARKUP_NOT_LINKED = "MARKUP_NOT_LINKED"


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


__all__ = (
    "MARKUP_NOT_LINKED",
    "MARKUP_TEXT_MISSING",
    "MATERIAL_MARKUP_PROPERTY",
    "link_markup",
    "linked_markup_name",
    "read_markup_text",
    "read_material_markup",
    "write_markup_text",
)
