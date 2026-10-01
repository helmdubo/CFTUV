"""Нормаль смещения над развёрнутой поверхностью: закон `SOURCE_VERTEX_ANGLE_WEIGHTED_NORMAL_V1`.

У развёрнутого домена нет плоскости источника, а значит и нормали плоскости, вдоль
которой хост сдвигает декаль против z-fighting. Решение ВЛАДЕЛЬЦА (2026-10-03): нормаль
ВЕРШИНЫ источника — сумма единичных нормалей инцидентных треугольников владельца с
весами углов при этой вершине, нормированная. Вершины домена общие, поэтому смещение
по нормали вершины не открывает щель на складке (нормаль треугольника открыла бы щель
на сгибе 90° при смещении 0.02), а у точки меша внутри треугольника нормаль — барицентрическое
смешение нормалей трёх его вершин, нормированное: непрерывно через ребро источника.

Арифметика — binary64 с фиксированным порядком операций над позициями, один раз
округлёнными из точных дробей: это направление смещения (политика отображения), а не
решение о геометрии, и закон именован, а не выведен из кода. Ни одна нормаль не
пропадает молча: нулевая нормаль вершины либо нормаль, смотрящая против нормали одного из
её треугольников (`n_v · n_T <= 0`), — именованный отказ `SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE`.
Знак нормали треугольника — обход вершин владельца (`SurfaceTriangleV1.vertex_ids`); карта
развёртки против часовой стрелки согласована с владельцем по построению, поэтому лицевая
сторона батча — именно эта.
"""

from __future__ import annotations

import math

from .admit import MaterializationOutcome
from .frames import MaterializationRefusal

OFFSET_NORMAL_LAW = "SOURCE_VERTEX_ANGLE_WEIGHTED_NORMAL_V1"


def _vector(first, second):
    return (second[0] - first[0], second[1] - first[1], second[2] - first[2])


def _cross(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _dot(left, right) -> float:
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2]


def _length(vector) -> float:
    return math.sqrt(_dot(vector, vector))


def _unit(vector):
    length = _length(vector)
    return (vector[0] / length, vector[1] / length, vector[2] / length)


def _refusal(detail: str) -> MaterializationRefusal:
    return MaterializationRefusal(
        MaterializationOutcome.SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE, detail
    )


def source_vertex_normals(triangles, position) -> dict:
    """`vertex_id -> единичная нормаль смещения` по треугольникам владельца.

    `triangles` — треугольники владельца (по имени), `position` — точные привязанные
    3D-позиции вершин. Отказывает, если нормаль вершины нулевая либо смотрит против
    нормали своего треугольника.
    """

    corners = {
        item.triangle_id: tuple(
            tuple(float(axis) for axis in position[vertex]) for vertex in item.vertex_ids
        )
        for item in triangles
    }
    unit = {}
    total: dict = {}
    for item in sorted(triangles, key=lambda entry: entry.triangle_id.value):
        points = corners[item.triangle_id]
        normal = _cross(_vector(points[0], points[1]), _vector(points[0], points[2]))
        if not _length(normal):
            raise _refusal(f"triangle {item.triangle_id.value} has no normal")
        unit[item.triangle_id] = _unit(normal)
        for index, vertex in enumerate(item.vertex_ids):
            first = _vector(points[index], points[(index + 1) % 3])
            second = _vector(points[index], points[(index + 2) % 3])
            angle = math.atan2(_length(_cross(first, second)), _dot(first, second))
            weighted = tuple(angle * axis for axis in unit[item.triangle_id])
            previous = total.get(vertex, (0.0, 0.0, 0.0))
            total[vertex] = tuple(a + b for a, b in zip(previous, weighted))
    result = {}
    for vertex, vector in sorted(total.items(), key=lambda entry: entry[0].value):
        if not _length(vector):
            raise _refusal(
                f"the angle-weighted normal of vertex {vertex.value} is zero"
            )
        result[vertex] = _unit(vector)
    for item in triangles:
        for vertex in item.vertex_ids:
            if not _dot(result[vertex], unit[item.triangle_id]) > 0.0:
                raise _refusal(
                    f"the offset normal of vertex {vertex.value} opposes the normal "
                    f"of its triangle {item.triangle_id.value}"
                )
    return result


def blend(weights, normals):
    """Нормаль точки внутри треугольника: барицентрическое смешение, нормированное."""

    mixed = tuple(
        sum(weight * normal[axis] for weight, normal in zip(weights, normals))
        for axis in range(3)
    )
    if not _length(mixed):
        raise _refusal("the blended offset normal of a mesh vertex is zero")
    return _unit(mixed)
