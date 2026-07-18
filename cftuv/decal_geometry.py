"""Общие geometry-контракты decal backend'ов.

Модуль не знает о legacy network, Patch Voronoi, bpy или bmesh. Здесь живут
только данные готовой face, минимальные 2D primitives и lift на пересечение
offset-плоскостей owner surfaces.
"""

from __future__ import annotations

from dataclasses import dataclass

from mathutils import Vector

from .constants import DECAL_COPLANAR_DOT


__all__ = (
    "DecalGeometryFace",
    "DomainLocation",
    "lift_offset_position",
    "polygon_area2",
    "segment_point_distance2",
)


@dataclass(frozen=True)
class DomainLocation:
    """Chart coordinate с source provenance для conforming lift.

    2D clipping продолжает передавать plain ``uv`` tuples. Location создаётся
    только у station/materialization boundary, где уже требуется выбрать
    source triangle, fold edge или source vertex.
    """

    chart_id: int
    triangle_id: int
    uv: tuple[float, float]
    barycentric: tuple[float, float, float]
    source_feature: str
    source_feature_id: object
    transition_key: object | None = None


@dataclass
class DecalGeometryFace:
    """Готовая face декали: позиции, shared vertex keys и UV-факты."""

    surface_id: int
    surface_normal: Vector
    vert_keys: list
    positions: list
    u_fracs: list
    v_lengths: list
    component_kind: str = "SURFACE"
    component_side: str = ""


def polygon_area2(points):
    """Signed area простого 2D polygon."""

    area = 0.0
    for index in range(len(points)):
        point = points[index]
        other = points[(index + 1) % len(points)]
        area += point[0] * other[1] - point[1] * other[0]
    return area * 0.5


def segment_point_distance2(seg_a, seg_b, point):
    """Возвращает distance и clamped parameter точки на 2D segment."""

    delta = (seg_b[0] - seg_a[0], seg_b[1] - seg_a[1])
    relative = (point[0] - seg_a[0], point[1] - seg_a[1])
    denominator = delta[0] * delta[0] + delta[1] * delta[1]
    if denominator < 1e-24:
        distance = (
            relative[0] * relative[0] + relative[1] * relative[1]
        ) ** 0.5
        return distance, 0.0
    parameter = (
        relative[0] * delta[0] + relative[1] * delta[1]
    ) / denominator
    parameter = max(0.0, min(1.0, parameter))
    closest = (
        seg_a[0] + delta[0] * parameter,
        seg_a[1] + delta[1] * parameter,
    )
    distance_delta = (point[0] - closest[0], point[1] - closest[1])
    distance = (
        distance_delta[0] * distance_delta[0]
        + distance_delta[1] * distance_delta[1]
    ) ** 0.5
    return distance, parameter


def _group_average_normals(
    normals,
    *,
    coplanar_dot=DECAL_COPLANAR_DOT,
):
    """Группирует почти одинаковые oriented normals."""

    groups = []
    for normal in normals:
        if normal.length_squared < 1e-12:
            continue
        candidate = normal.normalized()
        placed = False
        for group in groups:
            if candidate.dot(group[0]) > coplanar_dot:
                group[1].append(candidate)
                placed = True
                break
        if not placed:
            groups.append([candidate, [candidate]])
    averaged = []
    for _representative, members in groups:
        total = Vector((0.0, 0.0, 0.0))
        for member in members:
            total = total + member
        if total.length_squared > 1e-12:
            averaged.append(total.normalized())
    return averaged


def lift_offset_position(
    source_pos,
    normals,
    offset,
    *,
    coplanar_dot=DECAL_COPLANAR_DOT,
):
    """Least-squares пересечение offset-плоскостей owner surfaces точки."""

    unique = _group_average_normals(
        normals,
        coplanar_dot=coplanar_dot,
    )
    if not unique:
        return source_pos.copy()
    if len(unique) == 1:
        return source_pos + unique[0] * offset
    if len(unique) == 2:
        dot = max(-1.0, min(1.0, unique[0].dot(unique[1])))
        denominator = 1.0 + dot
        if denominator > 1e-8:
            delta = (unique[0] + unique[1]) * (offset / denominator)
            return source_pos + delta

    xx = sum(normal.x * normal.x for normal in unique)
    xy = sum(normal.x * normal.y for normal in unique)
    xz = sum(normal.x * normal.z for normal in unique)
    yy = sum(normal.y * normal.y for normal in unique)
    yz = sum(normal.y * normal.z for normal in unique)
    zz = sum(normal.z * normal.z for normal in unique)
    rhs = Vector(
        tuple(
            offset * sum(normal[axis] for normal in unique)
            for axis in range(3)
        )
    )
    trace = xx + yy + zz
    determinant_limit = max(1e-14, trace * trace * trace * 1e-12)
    determinant = (
        xx * (yy * zz - yz * yz)
        - xy * (xy * zz - yz * xz)
        + xz * (xy * yz - yy * xz)
    )
    if abs(determinant) <= determinant_limit:
        delta = Vector((0.0, 0.0, 0.0))
        for _iteration in range(128):
            correction = Vector((0.0, 0.0, 0.0))
            for normal in unique:
                correction += normal * (offset - normal.dot(delta))
            correction /= len(unique)
            delta += correction
            if correction.length_squared < 1e-20:
                break
        return source_pos + delta
    cofactor_xx = yy * zz - yz * yz
    cofactor_xy = xz * yz - xy * zz
    cofactor_xz = xy * yz - xz * yy
    cofactor_yy = xx * zz - xz * xz
    cofactor_yz = xy * xz - xx * yz
    cofactor_zz = xx * yy - xy * xy
    delta = Vector(
        (
            (
                cofactor_xx * rhs.x
                + cofactor_xy * rhs.y
                + cofactor_xz * rhs.z
            )
            / determinant,
            (
                cofactor_xy * rhs.x
                + cofactor_yy * rhs.y
                + cofactor_yz * rhs.z
            )
            / determinant,
            (
                cofactor_xz * rhs.x
                + cofactor_yz * rhs.y
                + cofactor_zz * rhs.z
            )
            / determinant,
        )
    )
    return source_pos + delta
