"""Общие geometry-контракты decal backend'ов.

Модуль не знает о legacy network, Patch Voronoi, bpy или bmesh. Здесь живут
только данные готовой face, минимальные 2D primitives и lift на пересечение
offset-плоскостей owner surfaces.
"""

from __future__ import annotations

from dataclasses import dataclass, fields, is_dataclass
from enum import Enum
from math import isfinite

from mathutils import Vector

from .constants import DECAL_COPLANAR_DOT


__all__ = (
    "DecalGeometryFace",
    "DomainLocation",
    "GeometryBatch",
    "GeometryBatchValidationError",
    "GeometryFace",
    "geometry_batch_from_faces",
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
    provenance: object | None = None


@dataclass(frozen=True)
class GeometryFace:
    """Deeply immutable adapter payload emitted by a decal evaluator."""

    surface_id: int
    surface_normal: tuple[float, float, float]
    vert_keys: tuple
    positions: tuple[tuple[float, float, float], ...]
    u_fracs: tuple[float, ...]
    v_lengths: tuple[float, ...]
    component_kind: str = "SURFACE"
    component_side: str = ""
    provenance: object | None = None


@dataclass(frozen=True)
class GeometryBatch:
    """Single immutable source consumed by both preview and materializer."""

    faces: tuple[GeometryFace, ...]
    diagnostics: tuple[tuple[str, object], ...] = ()

    def __post_init__(self):
        if not isinstance(self.faces, tuple):
            raise GeometryBatchValidationError(-1, "faces_not_tuple")
        if not isinstance(self.diagnostics, tuple):
            raise GeometryBatchValidationError(-1, "diagnostics_not_tuple")
        for face_index, face in enumerate(self.faces):
            if not isinstance(face, GeometryFace):
                raise GeometryBatchValidationError(
                    face_index, "face_not_immutable_geometry_face"
                )
            lengths = (
                len(face.vert_keys),
                len(face.positions),
                len(face.u_fracs),
                len(face.v_lengths),
            )
            if len(set(lengths)) != 1 or lengths[0] < 3:
                raise GeometryBatchValidationError(
                    face_index, f"loop_lengths={lengths}"
                )
            if not all(
                isinstance(values, tuple)
                for values in (
                    face.surface_normal,
                    face.vert_keys,
                    face.positions,
                    face.u_fracs,
                    face.v_lengths,
                )
            ):
                raise GeometryBatchValidationError(
                    face_index, "mutable_loop_payload"
                )
            if len(face.surface_normal) != 3 or any(
                not isfinite(float(value)) for value in face.surface_normal
            ):
                raise GeometryBatchValidationError(
                    face_index, "invalid_surface_normal"
                )
            if any(
                len(position) != 3
                or any(not isfinite(float(value)) for value in position)
                for position in face.positions
            ):
                raise GeometryBatchValidationError(
                    face_index, "invalid_position"
                )
            if any(
                not isfinite(float(value))
                for values in (face.u_fracs, face.v_lengths)
                for value in values
            ):
                raise GeometryBatchValidationError(
                    face_index, "invalid_uv_fact"
                )
            try:
                if len(set(face.vert_keys)) != len(face.vert_keys):
                    raise GeometryBatchValidationError(
                        face_index, "repeated_vert_keys"
                    )
            except TypeError as exc:
                raise GeometryBatchValidationError(
                    face_index, "unhashable_vert_key"
                ) from exc
            if not _deeply_immutable(face.provenance):
                raise GeometryBatchValidationError(
                    face_index, "mutable_provenance"
                )
        for diagnostic in self.diagnostics:
            if (
                not isinstance(diagnostic, tuple)
                or len(diagnostic) != 2
                or not isinstance(diagnostic[0], str)
                or not _deeply_immutable(diagnostic[1])
            ):
                raise GeometryBatchValidationError(-1, "mutable_diagnostic")


class GeometryBatchValidationError(ValueError):
    """Evaluator output is invalid before either adapter can observe it."""

    def __init__(self, face_index: int, reason: str):
        self.face_index = int(face_index)
        self.reason = str(reason)
        super().__init__(
            "DECAL_GEOMETRY_BATCH_INVALID: "
            f"face={self.face_index} {self.reason}"
        )


def _deeply_immutable(value):
    if value is None or isinstance(value, (str, bytes, int, float, bool, Enum)):
        return True
    if isinstance(value, tuple):
        return all(_deeply_immutable(item) for item in value)
    if isinstance(value, frozenset):
        return all(_deeply_immutable(item) for item in value)
    if is_dataclass(value):
        params = getattr(type(value), "__dataclass_params__", None)
        return bool(params and params.frozen) and all(
            _deeply_immutable(getattr(value, item.name)) for item in fields(value)
        )
    return False


def geometry_batch_from_faces(faces, *, diagnostics=()) -> GeometryBatch:
    """Freeze mutable builder faces before any display/materialize adapter."""

    frozen_faces = []
    for face in faces:
        frozen_faces.append(
            GeometryFace(
                surface_id=int(face.surface_id),
                surface_normal=tuple(float(value) for value in face.surface_normal),
                vert_keys=tuple(face.vert_keys),
                positions=tuple(
                    tuple(float(value) for value in position)
                    for position in face.positions
                ),
                u_fracs=tuple(float(value) for value in face.u_fracs),
                v_lengths=tuple(float(value) for value in face.v_lengths),
                component_kind=str(
                    getattr(face, "component_kind", "") or "SURFACE"
                ),
                component_side=str(getattr(face, "component_side", "") or ""),
                provenance=(
                    getattr(face, "provenance", None)
                    or getattr(face, "rail_provenance", None)
                ),
            )
        )
    return GeometryBatch(
        faces=tuple(frozen_faces),
        diagnostics=tuple(diagnostics),
    )


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
