"""Настройки режима Hotspot (HOTSPOT_MARKUP_V1): переключатель, разметка материала, метка, крылья; регистрация режима и редактора разметки (hotspot_editor). Строки панели - `hotspot_blender.draw_hotspot_rows`.

Разметку выбирает МАТЕРИАЛ декали: поле «Markup» читает и пишет его свойство `cftuv_hotspot_markup` (своего значения у настройки нет), поэтому
материал и разметка не расходятся. Метка одна на всё выделение (`mark_id`); перечень меток строится из разметки материала.
"""

from __future__ import annotations

import bpy
from bpy.props import BoolProperty, EnumProperty, IntProperty, PointerProperty, StringProperty

from .hotspot_blender import SETTINGS_ATTRIBUTE, decal_material, link_markup, linked_markup_name, read_material_markup


def _markup_name_get(self) -> str:
    return linked_markup_name(decal_material(self.id_data))


def _markup_name_set(self, value) -> None:
    material = decal_material(self.id_data)
    if material is not None:
        link_markup(material, value)


#: Пункты выбора метки держатся здесь: Blender не копирует строки динамического перечня.
_MARK_ITEMS: list = []


def _mark_items(self, context):
    scene = getattr(context, "scene", None) or self.id_data
    _name, markup, _problems = read_material_markup(decal_material(scene))
    items = [] if markup is None else [(str(mark.id), f"{mark.id}: {mark.name} ({mark.kind})", f"{mark.kind}, up {mark.up}, {mark.w} x {mark.h} px", mark.id) for mark in markup.marks]
    _MARK_ITEMS[:] = items or [("-1", "no marks", "the markup of the decal material has no marks", -1)]
    return _MARK_ITEMS


def _mark_get(self) -> int:
    return int(self.mark_id)


def _mark_set(self, value) -> None:
    self.mark_id = max(0, int(value))


class HOTSPOTUV_HotspotSettings(bpy.types.PropertyGroup):
    enabled: BoolProperty(
        name="Hotspot",
        default=False,
        description="Map the decal UVs into one mark of the hotspot markup linked to the decal material (Straight strips only). Off: the decal is exactly as without Hotspot",
    )
    markup_name: StringProperty(
        name="Markup",
        get=_markup_name_get,
        set=_markup_name_set,
        description="Text datablock with the hotspot markup (JSON) of the decal material; stored on the material as cftuv_hotspot_markup",
    )
    mark_id: IntProperty(name="Mark ID", default=0, min=0, description="One mark for the whole selection")
    mark_choice: EnumProperty(name="Mark", items=_mark_items, get=_mark_get, set=_mark_set, description="Mark of the markup the decal lands in")
    swap_wings: BoolProperty(name="Swap wings", default=False, description="Corner marks: exchange which side of the seam takes which half of the butterfly")


_CLASSES = (HOTSPOTUV_HotspotSettings,)


def register_hotspot() -> None:
    """Регистрация настроек Hotspot (свойство сцены) и редактора разметки в Image Editor. Повтор безопасен."""

    from .hotspot_editor import register_editor

    for cls in _CLASSES:
        if getattr(cls, "is_registered", False):
            bpy.utils.unregister_class(cls)
        bpy.utils.register_class(cls)
    setattr(bpy.types.Scene, SETTINGS_ATTRIBUTE, PointerProperty(type=HOTSPOTUV_HotspotSettings))
    register_editor()


def unregister_hotspot() -> None:
    from .hotspot_editor import unregister_editor

    unregister_editor()
    if hasattr(bpy.types.Scene, SETTINGS_ATTRIBUTE):
        delattr(bpy.types.Scene, SETTINGS_ATTRIBUTE)
    for cls in reversed(_CLASSES):
        if getattr(cls, "is_registered", False):
            bpy.utils.unregister_class(cls)


__all__ = ("HOTSPOTUV_HotspotSettings", "register_hotspot", "unregister_hotspot")
