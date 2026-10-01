"""Настройка «Worker Python»: интерпретатор воркеров пула доменов.

Пустая строка — встроенный интерпретатор Blender, умолчание. Путь — внешний
CPython, на котором то же ядро быстрее (замер в `DECISIONS.md`): пул принимает его
только после сверки версий и исходников ядра с родителем (`envelope_domain_pool`),
а при отказе воркеры идут на встроенном, и причина названа.

Свойство живёт в ПРЕДПОЧТЕНИЯХ аддона, а не в настройках сцены: это настройка
машины, а путь к интерпретатору, сохранённый в `.blend`, который откроют на
другой машине, — мина. Класс предпочтений объявлен в `operators.py`, и дописать
свойство туда нельзя (файл стоит на своём потолке), поэтому оно пристёгивается
при регистрации: класс снимается, свойство вносится в его аннотации и класс
регистрируется снова (присвоение `setattr` уже зарегистрированному классу
свойства не создаёт — проверено на Blender 4.5). Значения остальных свойств
предпочтений при этом сохраняются.

Модуль не импортирует `bpy` при загрузке, чтобы пакет оставался импортируемым
вне Blender; вне Blender настройка просто пуста.
"""

from __future__ import annotations

import os

PREFERENCE_NAME = "worker_python"


def install_worker_python_preference() -> bool:
    """Пристёгивает свойство к зарегистрированному классу предпочтений аддона.

    `False` — класса нет (вне Blender либо аддон не зарегистрирован), и тогда
    настройка пуста, то есть воркеры идут на встроенном интерпретаторе.
    """

    try:
        import bpy
    except ImportError:
        return False
    preferences_type = getattr(getattr(bpy, "types", None), "AddonPreferences", None)
    if preferences_type is None:
        return False
    for cls in preferences_type.__subclasses__():
        # `__subclasses__` помнит и сброшенные классы прежней загрузки пакета
        # (тот же `bl_idname`): `is_registered` отличает живой от них.
        if getattr(cls, "bl_idname", "") != __package__:
            continue
        if not getattr(cls, "is_registered", False):
            continue
        annotations = cls.__dict__.get("__annotations__")
        if annotations is None:
            return False
        if PREFERENCE_NAME in annotations:
            return True
        bpy.utils.unregister_class(cls)
        annotations[PREFERENCE_NAME] = bpy.props.StringProperty(
            name="Worker Python",
            subtype="FILE_PATH",
            default="",
            description=(
                "External CPython for the Envelope domain pool workers; empty "
                "keeps Blender's bundled Python. It must run the same sympy, "
                "mpmath and kernel sources as Blender, otherwise the pool names "
                "the mismatch and falls back to the bundled Python"
            ),
        )
        try:
            bpy.utils.register_class(cls)
        except (RuntimeError, ValueError):
            del annotations[PREFERENCE_NAME]
            bpy.utils.register_class(cls)
            return False
        return True
    return False


def _preferences():
    try:
        import bpy
    except ImportError:
        return None
    context = getattr(bpy, "context", None)
    addons = getattr(getattr(context, "preferences", None), "addons", None)
    addon = addons.get(__package__) if addons is not None else None
    return getattr(addon, "preferences", None)


def read_worker_python() -> str:
    """Путь внешнего интерпретатора из предпочтений либо `""` (встроенный).

    Путь, скопированный из проводника, приходит в кавычках; тильда раскрывается.
    """

    text = str(getattr(_preferences(), PREFERENCE_NAME, "") or "")
    text = text.strip().strip('"').strip()
    return os.path.expanduser(text) if text else ""


def draw_worker_python_row(layout) -> None:
    """Строка настройки в панели: предпочтения аддона не видны из сцены."""

    preferences = _preferences()
    if preferences is not None and hasattr(preferences, PREFERENCE_NAME):
        layout.prop(preferences, PREFERENCE_NAME)


__all__ = (
    "PREFERENCE_NAME",
    "draw_worker_python_row",
    "install_worker_python_preference",
    "read_worker_python",
)
