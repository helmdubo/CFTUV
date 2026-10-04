"""Оверлей мгновенного превью ширины: обработчик отрисовки вьюпорта, который ТОЛЬКО рисует.

Геометрию превью считает чистая функция (`envelope_width_preview.compute_width_preview`), результат лежит в
`controller.width_preview` (`WidthPreviewStateV1`). Здесь нет ни одной формулы геометрии: обработчик берёт готовые
ломаные в локальных координатах источника, раскладывает их в отрезки один раз на порядковый номер превью и
рисует линиями `POST_VIEW` (модуль `gpu`) с матрицей объекта. Фоновый Blender обработчиков отрисовки не вызывает,
поэтому всё, что можно проверить без экрана (геометрия, жизненный цикл состояния), проверено в чистых модулях и
смоке, а этот модуль проверяется только глазами владельца.

ЖИЗНЕННЫЙ ЦИКЛ. Оверлей снимается (`controller.width_preview = None`): когда применён точный результат на
последней ширине (`envelope_width_live._apply`), при смене объекта (активным стал не источник и не его декаль:
`envelope_width_live.retarget` из обработчика depsgraph и проверка здесь, на отрисовке), при удалении декали под
линиями (`follow_active_object`), при Undo/Redo и загрузке файла (`envelope_width_modal`: обработчики истории), при
новом «Build Decal Mesh», при отмене инструмента. Превью НЕ пишется в меш и не живёт дольше этих событий.

Отказ отрисовки не молчит: первая же ошибка `gpu` — строка консоли, оверлей отключён до перерегистрации;
строка статуса панели (`PREVIEW_BINARY64_V1 preview, not final: ...`) при этом остаётся.
"""

from __future__ import annotations

import bpy

from .envelope_production_mesh import decal_object_name

#: Цвет линий превью (оранжевый: отличим от слоёв отладки и от самой декали).
LINE_COLOR = (1.0, 0.62, 0.1, 1.0)
LINE_WIDTH = 3.0

_handle = None
_cache: dict = {"key": None, "batch": None, "shader": None}
_failed = False


def line_segments(polylines) -> list:
    """Ломаные -> плоский список концов отрезков для `LINES` (чистая раскладка, без `gpu`)."""

    segments = []
    for polyline in polylines:
        for first, second in zip(polyline, polyline[1:]):
            segments.append(first)
            segments.append(second)
    return segments


def _belongs(active, source_name: str) -> bool:
    """Оверлей показан, пока активного объекта нет либо это источник превью либо его декаль."""

    return active is None or active.name in (source_name, decal_object_name(source_name))


def _batch_for(controller, state):
    import gpu
    from gpu_extras.batch import batch_for_shader

    key = (id(controller), state.serial)
    if _cache["key"] != key:
        shader = gpu.shader.from_builtin("POLYLINE_UNIFORM_COLOR")
        coords = line_segments(state.preview.polylines)
        _cache.update(
            key=key,
            shader=shader,
            batch=batch_for_shader(shader, "LINES", {"pos": coords}) if coords else None,
        )
    return _cache["shader"], _cache["batch"]


def _draw() -> None:
    global _failed
    if _failed:
        return
    try:
        controller = getattr(bpy.context.window_manager, "_cftuv_envelope_debug_session", None)
        state = None if controller is None else controller.width_preview
        if state is None:
            return
        source = bpy.data.objects.get(state.source_name)
        if source is None or not _belongs(bpy.context.view_layer.objects.active, state.source_name):
            controller.width_preview = None  # объект сменился или исчез: превью про другой объект
            return
        _draw_lines(controller, state, source)
    except Exception as exc:  # noqa: BLE001 - отказ рисования называется, а не глотается
        _failed = True
        print(
            f"[CFTUV][WidthOverlay] draw failed: {type(exc).__name__}: {exc}; "
            "the overlay is off until the add-on is registered again",
            flush=True,
        )


def _draw_lines(controller, state, source) -> None:
    import gpu

    shader, batch = _batch_for(controller, state)
    if batch is None:
        return
    region = bpy.context.region
    gpu.state.depth_test_set("LESS_EQUAL")
    gpu.state.blend_set("ALPHA")
    gpu.matrix.push()
    try:
        gpu.matrix.multiply_matrix(source.matrix_world)
        shader.bind()
        shader.uniform_float("viewportSize", (region.width, region.height))
        shader.uniform_float("lineWidth", LINE_WIDTH)
        shader.uniform_float("color", LINE_COLOR)
        batch.draw(shader)
    finally:
        gpu.matrix.pop()
        gpu.state.depth_test_set("NONE")
        gpu.state.blend_set("NONE")


def register_overlay() -> None:
    """Обработчик отрисовки `SpaceView3D`; повтор безопасен."""

    global _handle, _failed
    unregister_overlay()
    _failed = False
    _handle = bpy.types.SpaceView3D.draw_handler_add(_draw, (), "WINDOW", "POST_VIEW")


def unregister_overlay() -> None:
    global _handle
    if _handle is not None:
        bpy.types.SpaceView3D.draw_handler_remove(_handle, "WINDOW")
        _handle = None
    _cache.update(key=None, shader=None, batch=None)


def overlay_registered() -> bool:
    return _handle is not None


__all__ = (
    "LINE_COLOR",
    "LINE_WIDTH",
    "line_segments",
    "overlay_registered",
    "register_overlay",
    "unregister_overlay",
)
