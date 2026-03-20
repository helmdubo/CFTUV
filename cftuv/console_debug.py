from __future__ import annotations


def _get_hotspotuv_settings():
    try:
        import bpy  # type: ignore
    except Exception:
        return None

    context = getattr(bpy, "context", None)
    if context is None:
        return None

    scene = getattr(context, "scene", None)
    if scene is None:
        return None

    return getattr(scene, "hotspotuv_settings", None)


def is_verbose_console_enabled() -> bool:
    settings = _get_hotspotuv_settings()
    return bool(getattr(settings, "dbg_verbose_console", False))


def trace_console(message: str) -> None:
    if is_verbose_console_enabled():
        print(message)
