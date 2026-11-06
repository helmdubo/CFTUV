"""Настройки режима Hotspot (HOTSPOT_MARKUP_V1, HOTSPOT_PANEL_EDITOR_V1): переключатель, картинка материала, метка, крылья; регистрация режима и
редактора разметки (hotspot_editor). Строки панели - `hotspot_blender.draw_hotspot_rows`.

Материал разметки - материал декали панели (`hotspotuv_decal_mesh.material_name`): своего поля материала у режима нет, поэтому метка всегда
берётся из текстуры, которую декаль покажет. Картинку выбирает `image_choice`, если узлов Image Texture у материала несколько. Метка одна на
всё выделение (`mark_id`); перечень меток строится из разметки материала.
"""

from __future__ import annotations

import bpy
from bpy.props import BoolProperty, EnumProperty, IntProperty, PointerProperty, StringProperty

from .hotspot_blender import SETTINGS_ATTRIBUTE, decal_material, mark_items, material_images, read_material_markup, scene_material_image

#: Пункты динамических перечней держатся здесь: Blender не копирует строки динамического перечня.
_MARK_ITEMS: list = []
_IMAGE_ITEMS: list = []


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


def _image_items(self, context):
    images = material_images(decal_material(_scene_of(self, context)))
    _IMAGE_ITEMS[:] = [(image.name, f"{image.name} ({image.size[0]} x {image.size[1]} px)", "Image Texture of the material", index) for index, image in enumerate(images)] or [("-", "no image", "the material has no Image Texture node with an image", 0)]
    return _IMAGE_ITEMS


def _image_get(self) -> int:
    _material, image, _refusal, _detail, candidates = scene_material_image(self.id_data)
    names = [item.name for item in candidates]
    return names.index(image.name) if image is not None and image.name in names else 0


def _image_set(self, value) -> None:
    images = material_images(decal_material(self.id_data))
    if 0 <= int(value) < len(images):
        self.image_name = images[int(value)].name


class HOTSPOTUV_HotspotSettings(bpy.types.PropertyGroup):
    enabled: BoolProperty(
        name="Hotspot",
        default=False,
        description="Map the decal UVs into one mark of the hotspot markup of the decal material (Straight strips only). Off: the decal is exactly as without Hotspot",
    )
    image_name: StringProperty(name="Image", description="Image of the decal material to mark up when it has several Image Texture nodes")
    image_choice: EnumProperty(name="Texture", items=_image_items, get=_image_get, set=_image_set, description="Image Texture of the material the marks are drawn on")
    mark_id: IntProperty(name="Mark ID", default=0, min=0, description="One mark for the whole selection")
    mark_choice: EnumProperty(name="Mark", items=_mark_items, get=_mark_get, set=_mark_set, description="Mark of the material's markup the decal lands in")
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
