"""Контактные дефекты источника (`source_defects`): T-вершина и самопересечение грани, названные ДО привязки к решётке.

Полевой случай `cover.008` (патчи 18 и 285) воспроизведён на МАЛЫХ синтетических мешах с теми же зазорами: вершина в 5.7 мкм от
ребра, которому не принадлежит (две грани делят общий отрезок без общих вершин), и «бабочка», чьи рёбра разведены по высоте на
4e-08 м. Широкая фаза (сетка, полоса вдоль оси) сверяется с перебором всех пар: каждая находка точная, ни одна не теряется.
"""

from __future__ import annotations

import random
from fractions import Fraction

from cftuv_envelope._authoring_intent import AUTHOR_ANGULAR_ERROR
from cftuv_envelope.robust.snapping import grid_window_for_patch
from cftuv_envelope.source_defects import (
    find_source_contact_defects,
    point_segment_interior,
    segment_crossing,
    segment_distance_squared,
    source_contact_gap,
)


def _extent(positions) -> float:
    points = list(positions.values())
    return max(max(p[axis] for p in points) - min(p[axis] for p in points) for axis in range(3))


def _defects(positions, faces):
    return find_source_contact_defects(positions, faces, extent=_extent(positions))


def _two_faces_sharing_a_stretch(offset):
    """Нижняя грань с ребром 1-2 и верхняя с ребром 3-4 вдоль той же прямой, сдвинутые на `offset` по y и без общих вершин."""

    positions = {
        0: (0.0, 0.0, -1.0), 1: (0.0, 0.0, 0.0), 2: (10.0, 0.0, 0.0), 3: (10.0, 0.0, -1.0),
        4: (3.0, offset, 0.0), 5: (13.0, offset, 0.0), 6: (13.0, offset, 1.0), 7: (3.0, offset, 1.0),
    }
    faces = [(10, (0, 1, 2, 3)), (11, (4, 5, 6, 7))]
    return positions, faces


def test_a_vertex_beside_an_edge_within_the_authoring_gap_is_named():
    """Положительная фикстура реестра: зазор 5.7e-06 на габарите 13 м (граница зазора 9.1e-05) — T-вершины названы с местом на ребре."""

    positions, faces = _two_faces_sharing_a_stretch(5.7e-06)
    found = _defects(positions, faces)
    assert not found.clean
    assert {(item.vertex, item.edge) for item in found.t_vertices} == {(4, (1, 2)), (2, (4, 5))}
    nearest = found.t_vertices[0]
    assert Fraction(5.7e-06) ** 2 == nearest.distance_squared
    assert 0 < nearest.parameter < 1
    assert found.face_crossings == ()


def test_a_vertex_beyond_the_authoring_gap_is_not_a_defect():
    """Отрицательная фикстура реестра: тот же меш с зазором в 2 мм (наименьший честный зазор `cover.008` — 2.1 мм) чист."""

    positions, faces = _two_faces_sharing_a_stretch(2.1e-03)
    assert _defects(positions, faces).clean


def test_an_exact_touch_and_a_vertex_near_an_endpoint_are_not_t_vertices():
    """Вершина ровно на ребре — существующий контакт (привязка его не создаёт); ближайшая к КОНЦУ — почти совпавшие вершины, другой класс."""

    positions, faces = _two_faces_sharing_a_stretch(0.0)
    assert _defects(positions, faces).t_vertices == ()
    close_to_an_end = {0: (0.0, 0.0, 0.0), 1: (1.0, 0.0, 0.0), 2: (1.0, 1.0, 0.0), 3: (0.0, 1.0, 0.0),
                       4: (1.0 + 1e-6, 0.0, 0.0), 5: (2.0, 0.0, 0.0), 6: (2.0, 1.0, 0.0), 7: (1.0 + 1e-6, 1.0, 0.0)}
    found = _defects(close_to_an_end, [(0, (0, 1, 2, 3)), (1, (4, 5, 6, 7))])
    assert found.t_vertices == ()


def test_the_gap_is_half_of_the_lower_bound_of_the_grid_window():
    """Зазор — не новое число: `AUTHOR_ANGULAR_ERROR * extent` ровно половина нижней границы окна шага (`2 * AUTHOR_ANGULAR_ERROR * extent`)."""

    for extent in (Fraction(1, 2), Fraction(13), Fraction(100)):
        window = grid_window_for_patch(extent=extent, author_angular_error=AUTHOR_ANGULAR_ERROR, decal_detail=Fraction(1, 100))
        assert 2 * source_contact_gap(extent) == window.lower_bound
    assert source_contact_gap(0.0) == 0


def test_a_bowtie_face_is_a_self_intersection_even_when_its_edges_are_skew():
    """Грань-«бабочка» (рёбра пересекаются в плане, разведены по высоте на 4e-08 м) названа; обычный выпуклый четырёхугольник чист."""

    bowtie = {0: (0.0, 0.0, 0.0), 1: (4.0, 4.0, 4e-08), 2: (4.0, 0.0, 0.0), 3: (0.0, 4.0, 0.0)}
    found = _defects(bowtie, [(7, (0, 1, 2, 3))])
    assert [(item.face, item.edges) for item in found.face_crossings] == [(7, ((0, 1), (2, 3)))]
    assert abs(float(found.face_crossings[0].distance_squared) ** 0.5 - 2e-08) < 1e-12
    flat = {0: (0.0, 0.0, 0.0), 1: (4.0, 0.0, 0.0), 2: (4.0, 4.0, 0.0), 3: (0.0, 4.0, 0.0)}
    assert _defects(flat, [(7, (0, 1, 2, 3))]).clean
    exactly_crossing = {0: (0.0, 0.0, 0.0), 1: (4.0, 4.0, 0.0), 2: (4.0, 0.0, 0.0), 3: (0.0, 4.0, 0.0)}
    assert _defects(exactly_crossing, [(7, (0, 1, 2, 3))]).face_crossings[0].distance_squared == 0


def test_a_duplicated_vertex_is_not_a_crossing_or_a_t_vertex_of_the_neighbour():
    """Совпавшие вершины (`ZERO_LENGTH_EDGE` называет их раньше) не выдают себя за T-вершину и не за «бабочку»."""

    positions = {0: (0.0, 0.0, 0.0), 1: (1.0, 0.0, 0.0), 2: (1.0, 1.0, 0.0), 3: (0.0, 1.0, 0.0), 4: (1.0, 0.0, 0.0), 5: (2.0, 0.0, 0.0), 6: (2.0, 1.0, 0.0)}
    found = _defects(positions, [(0, (0, 1, 2, 3)), (1, (4, 5, 6))])
    assert found.clean


def test_the_findings_are_ordered_tightest_first_and_the_order_is_stable():
    positions, faces = _two_faces_sharing_a_stretch(5.7e-06)
    positions[8], positions[9], positions[10] = (5.0, 3e-06, 0.0), (6.0, 3e-06, 0.5), (5.0, 3e-06, 0.5)
    faces = faces + [(12, (8, 9, 10))]
    first = _defects(positions, faces)
    again = _defects(dict(reversed(list(positions.items()))), list(reversed(faces)))
    assert first == again
    distances = [item.distance_squared for item in first.t_vertices]
    assert distances == sorted(distances)


def test_the_exact_helpers_agree_with_a_hand_computation():
    assert point_segment_interior((1.0, 1.0, 0.0), (0.0, 0.0, 0.0), (2.0, 0.0, 0.0)) == (Fraction(1, 2), Fraction(1))
    assert point_segment_interior((3.0, 1.0, 0.0), (0.0, 0.0, 0.0), (2.0, 0.0, 0.0)) is None
    assert segment_crossing((0.0, 0.0, 0.0), (2.0, 0.0, 0.0), (1.0, -1.0, 3.0), (1.0, 1.0, 3.0)) == 9
    assert segment_crossing((0.0, 0.0, 0.0), (2.0, 0.0, 0.0), (0.0, 1.0, 0.0), (2.0, 1.0, 0.0)) is None
    assert segment_distance_squared((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (3.0, 0.0, 0.0), (4.0, 0.0, 0.0)) == 4


def _brute_force(positions, faces, extent):
    """Эталон: все пары (вершина, ребро) и (ребро, ребро грани) без широкой фазы."""

    gap = source_contact_gap(extent)
    edges = sorted({(min(a, b), max(a, b)) for _f, cycle in faces for a, b in zip(cycle, cycle[1:] + cycle[:1]) if a != b})
    t_found = set()
    for vertex in positions:
        for a, b in edges:
            if vertex in (a, b):
                continue
            hit = point_segment_interior(positions[vertex], positions[a], positions[b])
            if hit is not None and 0 < hit[1] <= gap * gap:
                t_found.add((vertex, (a, b)))
    crossings = set()
    for face, cycle in faces:
        sides = list(zip(cycle, cycle[1:] + cycle[:1]))
        for i in range(len(sides)):
            for j in range(i + 2, len(sides)):
                if i == 0 and j == len(sides) - 1:
                    continue
                if len({*sides[i], *sides[j]}) < 4:
                    continue
                distance = segment_crossing(*(positions[v] for v in (*sides[i], *sides[j])))
                if distance is not None and distance <= gap * gap:
                    crossings.add((face, (sides[i], sides[j])))
    return t_found, crossings


def test_the_broad_phase_loses_nothing_against_a_brute_force_search():
    """Случайные меши с вершинами, насаженными на рёбра с зазором внутри и снаружи границы: сетка и полоса вдоль оси = перебор всех пар."""

    rng = random.Random(20261010)
    for trial in range(12):
        positions, faces = {}, []
        count = 0
        for cell_x in range(5):
            for cell_y in range(4):
                base = (cell_x * 2.0, cell_y * 2.0, rng.uniform(-0.01, 0.01))
                quad = []
                for corner in ((0, 0), (1, 0), (1, 1), (0, 1)):
                    positions[count] = (base[0] + corner[0], base[1] + corner[1], base[2])
                    quad.append(count)
                    count += 1
                faces.append((len(faces), tuple(quad)))
        edges = [(face[1][k], face[1][(k + 1) % 4]) for face in faces for k in range(4)]
        for _ in range(8):
            a, b = rng.choice(edges)
            t = rng.uniform(0.1, 0.9)
            lift = rng.choice((0.0, 1e-7, 3e-6, 5e-5, 4e-4))
            point = tuple(positions[a][axis] + t * (positions[b][axis] - positions[a][axis]) for axis in range(3))
            positions[count] = (point[0], point[1], point[2] + lift)
            faces.append((len(faces), (count, count + 1, count + 2)))
            positions[count + 1] = (point[0] + 0.3, point[1] + 0.2, point[2] + lift)
            positions[count + 2] = (point[0] + 0.1, point[1] + 0.4, point[2] + lift)
            count += 3
        extent = _extent(positions)
        found = find_source_contact_defects(positions, faces, extent=extent)
        t_found, crossings = _brute_force(positions, faces, extent)
        assert {(item.vertex, item.edge) for item in found.t_vertices} == t_found, trial
        assert {(item.face, item.edges) for item in found.face_crossings} == crossings, trial


def test_a_long_edge_beyond_the_cell_limit_still_finds_its_vertex_on_the_axis_strip():
    """Ребро, покрывающее больше ячеек сетки, чем разрешено, идёт полосой вдоль оси — и находит вершину рядом с собой."""

    positions = {0: (0.0, 0.0, 0.0), 1: (1000.0, 0.0, 0.0), 2: (1000.0, 1.0, 0.0), 3: (0.0, 1.0, 0.0)}
    faces = [(0, (0, 1, 2, 3))]
    count = 4
    for k in range(60):
        x = 10.0 + k * 0.05
        for dx, dy in ((0, 5.0), (0.02, 5.0), (0.02, 5.02), (0, 5.02)):
            positions[count] = (x + dx, dy, 0.0)
            count += 1
        faces.append((len(faces), (count - 4, count - 3, count - 2, count - 1)))
    positions[count] = (500.0, 1e-3, 0.0)
    positions[count + 1] = (501.0, 2.0, 0.0)
    positions[count + 2] = (499.0, 2.0, 0.0)
    faces.append((len(faces), (count, count + 1, count + 2)))
    found = _defects(positions, faces)
    assert [(item.vertex, item.edge) for item in found.t_vertices] == [(count, (0, 1))]
