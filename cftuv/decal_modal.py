"""Экранная математика modal-регулировки размеров декалей."""

from bpy_extras.view3d_utils import location_3d_to_region_2d
from mathutils import Vector


DECAL_SIZE_MIN = 0.001
_VIEWPORT_INSET = 24


def decal_drag_value(base_value, mouse_delta, precise=False):
    sensitivity = max(0.001, abs(base_value) * 0.01)
    if precise:
        sensitivity *= 0.1
    return max(DECAL_SIZE_MIN, base_value + mouse_delta * sensitivity)


def decal_drag_anchor(context, obj, fallback):
    """Возвращает экранный ноль: центр bbox или центр 3D viewport."""

    area = getattr(context, "area", None)
    regions = getattr(area, "regions", ())
    region = next((item for item in regions if item.type == "WINDOW"), None)
    if region is None:
        return fallback

    viewport_center = (
        region.x + region.width // 2,
        region.y + region.height // 2,
    )
    region_3d = getattr(getattr(context, "space_data", None), "region_3d", None)
    corners = tuple(getattr(obj, "bound_box", ()))
    if region_3d is None or not corners:
        return viewport_center

    center_local = Vector(
        tuple(
            sum(float(corner[axis]) for corner in corners) / len(corners)
            for axis in range(3)
        )
    )
    try:
        projected = location_3d_to_region_2d(
            region,
            region_3d,
            obj.matrix_world @ center_local,
        )
    except (AttributeError, RuntimeError, TypeError, ValueError):
        return viewport_center
    if projected is None:
        return viewport_center

    anchor_x = region.x + round(projected.x)
    anchor_y = region.y + round(projected.y)
    if not (
        region.x <= anchor_x < region.x + region.width
        and region.y <= anchor_y < region.y + region.height
    ):
        return viewport_center

    inset = min(
        _VIEWPORT_INSET,
        max(0, (region.width - 1) // 2),
        max(0, (region.height - 1) // 2),
    )
    return (
        max(region.x + inset, min(anchor_x, region.x + region.width - inset - 1)),
        max(region.y + inset, min(anchor_y, region.y + region.height - inset - 1)),
    )


def warp_decal_drag_cursor(context, obj, event):
    fallback = (event.mouse_x, event.mouse_y)
    anchor = decal_drag_anchor(context, obj, fallback)
    try:
        context.window.cursor_warp(*anchor)
    except (AttributeError, RuntimeError, TypeError):
        return fallback
    return anchor
