"""Настройки режима Hotspot (HOTSPOT_MARKUP_V1, HOTSPOT_PANEL_EDITOR_V1): переключатель, картинка материала, метка, крылья; регистрация режима и
редактора разметки (hotspot_editor). Строки панели - `hotspot_blender.draw_hotspot_rows`.

Материал разметки - материал декали панели (`hotspotuv_decal_mesh.material_name`): своего поля материала у режима нет, поэтому метка всегда
берётся из текстуры, которую декаль покажет. Картинку-подложку выбирают в самом мини-окне (HOTSPOT_BACKDROP_V1), `image_name` её помнит. Метка `mark_id` -
метка по умолчанию нажатия кнопки; выбирают её меню панели, и сам выбор кладёт её на выделение готовой декали (HOTSPOT_APPLY_ON_PICK_V1,
`hotspot_island_ops`). У свойств здесь нет обратных вызовов `update`/`set` - их выставляют только операторы выбора: Undo/Redo, загрузка файла,
рисование панели и присваивание из кода ничего не применяют (`tests/test_hotspot_panel.py` это проверяет).
"""

from __future__ import annotations

import bpy
from bpy.props import BoolProperty, IntProperty, PointerProperty, StringProperty

from .hotspot_blender import SETTINGS_ATTRIBUTE


class HOTSPOTUV_HotspotSettings(bpy.types.PropertyGroup):
    enabled: BoolProperty(
        name="Hotspot",
        default=False,
        description="Map the decal UVs into one mark of the hotspot markup of the decal material (Straight strips only). Off: the decal is exactly as without Hotspot",
    )
    image_name: StringProperty(name="Image", description="Backdrop image last picked in the Mark Up window: the next Mark Up opens it")
    mark_id: IntProperty(name="Mark ID", default=0, min=0, description="Mark of the press (every island); picking it in the panel also maps the selected islands of a decal")
    swap_wings: BoolProperty(name="Swap wings", default=False, description="Corner marks: exchange which side of the seam takes which half of the butterfly")
    apply_note: StringProperty(name="Last pick", description="What the last pick of a mark (or Swap wings) did to the selection")


_CLASSES = (HOTSPOTUV_HotspotSettings,)


def register_hotspot() -> None:
    """Регистрация настроек Hotspot (свойство сцены) и редактора разметки в Image Editor. Повтор безопасен."""

    from .hotspot_editor import register_editor
    from .hotspot_island_ops import register_island_ops
    from .hotspot_refresh import register_refresh

    for cls in _CLASSES:
        if getattr(cls, "is_registered", False):
            bpy.utils.unregister_class(cls)
        bpy.utils.register_class(cls)
    setattr(bpy.types.Scene, SETTINGS_ATTRIBUTE, PointerProperty(type=HOTSPOTUV_HotspotSettings))
    register_editor()
    register_island_ops()  # HOTSPOT_ISLAND_MARKS_V1, HOTSPOT_APPLY_ON_PICK_V1: выбор метки кладёт её выделенным островам готовой декали
    register_refresh()  # HOTSPOT_REFRESH_ON_CLOSE_V1: закрытие окна разметки обновляет её декали одним шагом отмены


def unregister_hotspot() -> None:
    from .hotspot_editor import unregister_editor
    from .hotspot_island_ops import unregister_island_ops
    from .hotspot_refresh import unregister_refresh

    unregister_refresh()
    unregister_island_ops()
    unregister_editor()
    if hasattr(bpy.types.Scene, SETTINGS_ATTRIBUTE):
        delattr(bpy.types.Scene, SETTINGS_ATTRIBUTE)
    for cls in reversed(_CLASSES):
        if getattr(cls, "is_registered", False):
            bpy.utils.unregister_class(cls)


__all__ = ("HOTSPOTUV_HotspotSettings", "register_hotspot", "unregister_hotspot")
