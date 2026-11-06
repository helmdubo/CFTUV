"""Настройки режима Hotspot (HOTSPOT_MARKUP_V1, HOTSPOT_PANEL_EDITOR_V1): переключатель, картинка материала, метка, крылья; регистрация режима и
редактора разметки (hotspot_editor). Строки панели - `hotspot_blender.draw_hotspot_rows`.

Материал разметки - материал декали панели (`hotspotuv_decal_mesh.material_name`): своего поля материала у режима нет, поэтому метка всегда
берётся из текстуры, которую декаль покажет. Картинку-подложку выбирают в самом мини-окне (HOTSPOT_BACKDROP_V1), `image_name` её помнит. Метка `mark_id` -
метка по умолчанию нажатия кнопки и метка «Apply Mark to Selection» островов готовой декали (HOTSPOT_ISLAND_MARKS_V1, `hotspot_island_ops`);
перечень меток строится из разметки материала.
"""

from __future__ import annotations

import bpy
from bpy.props import BoolProperty, EnumProperty, IntProperty, PointerProperty, StringProperty

from .hotspot_blender import SETTINGS_ATTRIBUTE, decal_material, mark_items, read_material_markup

#: Пункты динамических перечней держатся здесь: Blender не копирует строки динамического перечня.
_MARK_ITEMS: list = []


def _scene_of(self, context):
    return getattr(context, "scene", None) or self.id_data


def _mark_items(self, context):
    _name, markup, _problems = read_material_markup(decal_material(_scene_of(self, context)))
    _MARK_ITEMS[:] = mark_items(markup)
    return _MARK_ITEMS


def _mark_get(self) -> int:
    return int(self.mark_id)


def _mark_set(self, value) -> None:
    self.mark_id = max(0, int(value))


class HOTSPOTUV_HotspotSettings(bpy.types.PropertyGroup):
    enabled: BoolProperty(
        name="Hotspot",
        default=False,
        description="Map the decal UVs into one mark of the hotspot markup of the decal material (Straight strips only). Off: the decal is exactly as without Hotspot",
    )
    image_name: StringProperty(name="Image", description="Backdrop image last picked in the Mark Up window: the next Mark Up opens it")
    mark_id: IntProperty(name="Mark ID", default=0, min=0, description="Mark of the press (every island) and of Apply Mark to Selection (the selected islands of a decal)")
    mark_choice: EnumProperty(name="Mark", items=_mark_items, get=_mark_get, set=_mark_set, description="Mark of the material's markup the decal lands in")
    swap_wings: BoolProperty(name="Swap wings", default=False, description="Corner marks: exchange which side of the seam takes which half of the butterfly (per seam pair with Apply Mark to Selection)")


_CLASSES = (HOTSPOTUV_HotspotSettings,)


def register_hotspot() -> None:
    """Регистрация настроек Hotspot (свойство сцены) и редактора разметки в Image Editor. Повтор безопасен."""

    from .hotspot_editor import register_editor
    from .hotspot_island_ops import register_island_ops

    for cls in _CLASSES:
        if getattr(cls, "is_registered", False):
            bpy.utils.unregister_class(cls)
        bpy.utils.register_class(cls)
    setattr(bpy.types.Scene, SETTINGS_ATTRIBUTE, PointerProperty(type=HOTSPOTUV_HotspotSettings))
    register_editor()
    register_island_ops()  # HOTSPOT_ISLAND_MARKS_V1: метка выделенным островам готовой декали


def unregister_hotspot() -> None:
    from .hotspot_editor import unregister_editor
    from .hotspot_island_ops import unregister_island_ops

    unregister_island_ops()
    unregister_editor()
    if hasattr(bpy.types.Scene, SETTINGS_ATTRIBUTE):
        delattr(bpy.types.Scene, SETTINGS_ATTRIBUTE)
    for cls in reversed(_CLASSES):
        if getattr(cls, "is_registered", False):
            bpy.utils.unregister_class(cls)


__all__ = ("HOTSPOTUV_HotspotSettings", "register_hotspot", "unregister_hotspot")
