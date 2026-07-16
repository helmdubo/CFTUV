"""Экранная математика и targets modal-регулировки декалей."""

from dataclasses import dataclass
from math import degrees, pi

from bpy_extras.view3d_utils import location_3d_to_region_2d
from mathutils import Vector


DECAL_SIZE_MIN = 0.001
_VIEWPORT_INSET = 24

DECAL_DRAG_DISTANCE = "DISTANCE"
DECAL_DRAG_ANGLE = "ANGLE"
DECAL_DRAG_RATIO = "RATIO"


@dataclass(frozen=True)
class DecalDragTarget:
    """Operator-owned параметр одного horizontal drag gesture."""

    key: str
    label: str
    settings_field: str
    scene_property: str
    kind: str
    minimum: float
    maximum: float | None
    sensitivity: float


_WIDTH_TARGETS = {
    "TOP": DecalDragTarget(
        "W",
        "Trim Height",
        "height_trim",
        "decal_height_trim",
        DECAL_DRAG_DISTANCE,
        DECAL_SIZE_MIN,
        None,
        0.01,
    ),
    "BOTTOM": DecalDragTarget(
        "W",
        "Trim Height",
        "height_trim",
        "decal_height_trim",
        DECAL_DRAG_DISTANCE,
        DECAL_SIZE_MIN,
        None,
        0.01,
    ),
    "CORNERS": DecalDragTarget(
        "W",
        "Corner Width",
        "width_corner",
        "decal_width_corner",
        DECAL_DRAG_DISTANCE,
        DECAL_SIZE_MIN,
        None,
        0.01,
    ),
    "SEAMS": DecalDragTarget(
        "W",
        "Seam Width",
        "width_seam",
        "decal_width_seam",
        DECAL_DRAG_DISTANCE,
        DECAL_SIZE_MIN,
        None,
        0.01,
    ),
}

_ACUTE_ANGLE_TARGET = DecalDragTarget(
    "A",
    "Acute Split Angle",
    "corner_acute_split_angle",
    "decal_corner_acute_split_angle",
    DECAL_DRAG_ANGLE,
    pi / 180.0,
    pi * 179.0 / 180.0,
    pi / 360.0,
)

_APEX_LIMIT_TARGET = DecalDragTarget(
    "M",
    "Apex Limit",
    "corner_apex_limit",
    "decal_corner_miter_limit",
    DECAL_DRAG_RATIO,
    1.0,
    None,
    0.05,
)


def decal_drag_targets(mode):
    """Возвращает W/A/M descriptors в стабильном keyboard order."""

    width_target = _WIDTH_TARGETS.get(mode)
    if width_target is None:
        raise ValueError(f"Unsupported decal drag mode: {mode}")
    return (width_target, _ACUTE_ANGLE_TARGET, _APEX_LIMIT_TARGET)


def _drag_step(target, base_value):
    if target.kind == DECAL_DRAG_DISTANCE:
        return max(target.minimum, abs(base_value) * target.sensitivity)
    return target.sensitivity


def decal_drag_target_value(target, base_value, mouse_delta, precise=False):
    """Вычисляет target value и применяет descriptor-owned clamp."""

    step = _drag_step(target, base_value)
    if precise:
        step *= 0.1
    value = base_value + mouse_delta * step
    value = max(target.minimum, value)
    if target.maximum is not None:
        value = min(target.maximum, value)
    return value


def format_decal_drag_value(target, value):
    """Форматирует header value; ANGLE хранится в радианах."""

    if target.kind == DECAL_DRAG_ANGLE:
        return f"{degrees(value):.1f}\N{DEGREE SIGN}"
    return f"{value:.4f}"


def decal_drag_value(base_value, mouse_delta, precise=False):
    """Compatibility wrapper прежней distance-only modal математики."""

    return decal_drag_target_value(
        _WIDTH_TARGETS["TOP"],
        base_value,
        mouse_delta,
        precise=precise,
    )


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
    projected_anchor = decal_drag_anchor(context, obj, fallback)
    # Регулировка строго горизонтальная: меняем только экранный X,
    # вертикальную позицию курсора после клика не трогаем.
    anchor = (projected_anchor[0], event.mouse_y)
    try:
        context.window.cursor_warp(*anchor)
    except (AttributeError, RuntimeError, TypeError):
        return fallback
    return anchor
