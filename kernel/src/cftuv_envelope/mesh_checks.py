"""Проверки собранного меша декали (MESH_ASSEMBLY_NATIVE_V1, срез B1): эталон, с которым побитово сверен нативный порт (`cftuv_native.mesh_*`).

Чистые функции чисел над массивами меша после сварки: плоскость граней после смещения, складки между доменами с разрывом UV и
полурёбра двух граней. Хост (`cftuv/envelope_mesh_assembly.py`) зовёт их через `backend.mesh_assembly_compute`, когда заказан
нативный путь, и сюда, когда заказан эталон или порт отказал по имени. Семантика — CPython: вычитание и сумма слева направо
(без FMA), `max` берёт первое из равных, `round` — к чётному (его делает хост над `worst`), `!=` над UV — равенство чисел
(`-0.0 == 0.0`).
"""

from __future__ import annotations

import math


def _cross(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _dot(left, right) -> float:
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2]


def off_plane_measure(positions, faces) -> tuple:
    """`(наибольшее отклонение, метры; число граней с отклонением от нанометра)` граней от четырёх вершин.

    Плоскость грани — через её первую вершину с нормалью вектора площади (веер из первой вершины); мера — наибольшее расстояние
    вершины грани от неё. Грань нулевой площади ничего не меряет. Разрешение записи — нанометр: шум округления binary64
    (1e-16 м) гранью «вне плоскости» не числится (`round(отклонение * 1e9) > 0`).
    """

    worst, off = 0.0, 0
    for loop in faces:
        if len(loop) < 4:
            continue
        points = [positions[index] for index in loop]
        origin = points[0]
        normal = (0.0, 0.0, 0.0)
        for index in range(1, len(points) - 1):
            part = _cross(
                tuple(b - a for a, b in zip(origin, points[index])),
                tuple(b - a for a, b in zip(origin, points[index + 1])),
            )
            normal = tuple(a + b for a, b in zip(normal, part))
        length = math.sqrt(_dot(normal, normal))
        if not length:
            continue
        deviation = max(
            abs(_dot(tuple(b - a for a, b in zip(origin, point)), normal)) / length
            for point in points[1:]
        )
        worst = max(worst, deviation)
        off += int(round(deviation * 10**9) > 0)
    return worst, off


def _edge_uvs(faces, uvs, face_domain):
    """`{ребро: [(грань, домен, uv у меньшей вершины, uv у большей)]}` по петлям меша."""

    edges: dict = {}
    cursor = 0
    for number, loop in enumerate(faces):
        size = len(loop)
        for position in range(size):
            first, second = loop[position], loop[(position + 1) % size]
            uv_first = uvs[cursor + position]
            uv_second = uvs[cursor + (position + 1) % size]
            if first > second:
                first, second, uv_first, uv_second = second, first, uv_second, uv_first
            edges.setdefault((first, second), []).append(
                (number, face_domain[number], uv_first, uv_second)
            )
        cursor += size
    return edges


def cross_domain_seams(faces, uvs, face_domain) -> tuple:
    """Рёбра меша между гранями РАЗНЫХ доменов, где UV разрывен: шов на сваренной складке (по возрастанию)."""

    return tuple(
        sorted(
            edge
            for edge, sides in _edge_uvs(faces, uvs, face_domain).items()
            if len(sides) == 2
            and sides[0][1] != sides[1][1]
            and (sides[0][2] != sides[1][2] or sides[0][3] != sides[1][3])
        )
    )


def half_edge_conflicts(faces) -> int:
    """Направленные рёбра, лежащие в двух гранях: так сваренные соседи с разным обходом."""

    seen: dict = {}
    for loop in faces:
        size = len(loop)
        for position in range(size):
            half = (loop[position], loop[(position + 1) % size])
            seen[half] = seen.get(half, 0) + 1
    return sum(1 for count in seen.values() if count > 1)


__all__ = ("cross_domain_seams", "half_edge_conflicts", "off_plane_measure")
