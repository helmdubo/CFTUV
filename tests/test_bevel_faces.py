"""Нажатие по граням бевела (BEVEL_FACES_INWARD_OUTWARD_V1, BEVEL_NODE_STRIP_SPLIT_V1): разбиение «узел | полоса», стороны и отказы.

Меши - аналитические, без Blender: куб с фаской в один сегмент (6 плоских граней, 12 полос, 8 треугольных узлов) и сгиб с профилем
в два сегмента без узлов. Каждый отброс эвристики и каждый отказ проверяется по имени.
"""

from __future__ import annotations

import itertools

import pytest

from cftuv import bevel_faces as bf

W = 0.1


def _chamfered_cube():
    """Вершины - углы плоских граней (ось, знак, знаки двух других осей); полосы и узлы - между ними."""

    index = {}
    coords = {}
    for axis, sign in itertools.product(range(3), (-1, 1)):
        others = [item for item in range(3) if item != axis]
        for a, b in itertools.product((-1, 1), (-1, 1)):
            point = [0.0, 0.0, 0.0]
            point[axis] = float(sign)
            point[others[0]] = a * (1.0 - W)
            point[others[1]] = b * (1.0 - W)
            key = (axis, sign, a, b)
            index[key] = len(coords)
            coords[index[key]] = tuple(point)

    def vertex(axis, sign, signs):
        others = [item for item in range(3) if item != axis]
        return index[(axis, sign, signs[others[0]], signs[others[1]])]

    faces, kinds = {}, {}

    def add(cycle, kind):
        faces[len(faces)] = tuple(cycle)
        kinds[len(kinds)] = kind

    for axis, sign in itertools.product(range(3), (-1, 1)):
        others = [item for item in range(3) if item != axis]
        corners = []
        for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            signs = {axis: sign, others[0]: a, others[1]: b}
            corners.append(vertex(axis, sign, signs))
        add(corners, "flat")
    for first, second in itertools.combinations(range(3), 2):
        free = 3 - first - second
        for s1, s2 in itertools.product((-1, 1), (-1, 1)):
            cycle = []
            for t in (-1, 1):
                signs = {first: s1, second: s2, free: t}
                cycle.append(vertex(first, s1, signs))
            for t in (1, -1):
                signs = {first: s1, second: s2, free: t}
                cycle.append(vertex(second, s2, signs))
            add(cycle, "strip")
    for signs in itertools.product((-1, 1), repeat=3):
        sign_of = dict(enumerate(signs))
        add([vertex(axis, signs[axis], sign_of) for axis in range(3)], "node")
    return coords, faces, kinds


def _edge_ids(faces):
    ids = {}
    for cycle in faces.values():
        for position in range(len(cycle)):
            key = frozenset((cycle[position], cycle[(position + 1) % len(cycle)]))
            ids.setdefault(key, len(ids))
    return ids


def _view(coords, faces, selected, strength=None):
    return bf.BevelFacesViewV1(coords, faces, frozenset(selected), _edge_ids(faces), strength)


def _cube_view(strength=None):
    coords, faces, kinds = _chamfered_cube()
    selected = {face for face, kind in kinds.items() if kind != "flat"}
    return _view(coords, faces, selected, strength), kinds


def test_the_chamfered_cube_is_a_closed_manifold():
    coords, faces, kinds = _chamfered_cube()
    edges = {}
    for cycle in faces.values():
        for position in range(len(cycle)):
            key = frozenset((cycle[position], cycle[(position + 1) % len(cycle)]))
            edges[key] = edges.get(key, 0) + 1
    assert len(coords) - len(edges) + len(faces) == 2 and set(edges.values()) == {2}
    assert sorted(kinds.values()).count("strip") == 12 and sorted(kinds.values()).count("node") == 8


def test_the_geometric_split_finds_every_strip_and_node_of_the_cube():
    view, kinds = _cube_view()
    split = bf.geometric_split(view)
    assert split.source == bf.BEVEL_SPLIT_GEOMETRIC
    assert split.strip == {face for face, kind in kinds.items() if kind == "strip"}
    assert split.node == {face for face, kind in kinds.items() if kind == "node"}
    assert not split.unresolved


def test_the_face_strength_attribute_wins_and_a_disagreement_is_named():
    view, kinds = _cube_view()
    truth = {face: (-16384 if kind == "node" else 0) for face, kind in kinds.items() if kind != "flat"}
    split = bf.split_bevel_faces(bf.BevelFacesViewV1(view.coords, view.faces, view.selected, view.edge_ids, truth))
    assert split.source == bf.BEVEL_SPLIT_FACE_STRENGTH and split.node == {f for f, k in kinds.items() if k == "node"}
    assert dict(split.events).get(bf.BEVEL_CLASS_DISAGREES) is None
    wrong_strip = min(face for face, kind in kinds.items() if kind == "strip")
    spoiled = {**truth, wrong_strip: 16384}
    split = bf.split_bevel_faces(bf.BevelFacesViewV1(view.coords, view.faces, view.selected, view.edge_ids, spoiled))
    assert wrong_strip in split.node and dict(split.events)[bf.BEVEL_CLASS_DISAGREES] == 1


def test_inward_analyses_only_the_selection_and_cuts_every_node_from_its_strips():
    view, kinds = _cube_view()
    plan = bf.plan_faces(view, bf.INWARD, kinds)
    assert not plan.refusal and plan.scope == tuple(sorted(view.selected))
    assert len(plan.cut_edges) == 8 * 3  # три ребра каждого треугольного узла
    assert dict(plan.events)[bf.BEVEL_SPLIT_GEOMETRIC] == 20
    assert "12 strip / 8 node (BEVEL_SPLIT_GEOMETRIC)" in bf.plan_summary(plan)


def test_outward_analyses_only_the_faces_the_selection_leaves_and_needs_no_split():
    view, kinds = _cube_view()
    plan = bf.plan_faces(view, bf.OUTWARD, kinds)
    assert plan.scope == tuple(sorted(face for face, kind in kinds.items() if kind == "flat"))
    assert len(plan.outward_sources) == 24 and plan.split is None and not plan.cut_edges


def _grid(size=3):
    coords = {row * (size + 1) + column: (float(column), float(row), 0.0) for row in range(size + 1) for column in range(size + 1)}
    faces = {}
    for row, column in itertools.product(range(size), range(size)):
        first = row * (size + 1) + column
        faces[row * size + column] = (first, first + 1, first + size + 2, first + size + 1)
    return coords, faces


def test_a_selection_without_a_profile_is_refused_by_name():
    """Средний квад плоской сетки: путь поперёк соединяет две грани одной плоскости - не профиль, грань не разрешена."""

    coords, faces = _grid()
    plan = bf.plan_faces(_view(coords, faces, {4}), bf.INWARD, faces)
    assert plan.refusal == bf.BEVEL_SPLIT_UNRESOLVED and plan.split.unresolved == {4} and not plan.scope
    assert dict(plan.events)[bf.BEVEL_SPLIT_UNRESOLVED] == 1
    assert dict(plan.events)[bf.BEVEL_COLUMN_SAME_SIDE] == 4
    assert any(line.endswith(f"REFUSED {bf.BEVEL_SPLIT_UNRESOLVED}: {plan.detail}") for line in bf.plan_console_lines(plan))


def test_unresolved_faces_leave_the_analysis_and_the_rest_is_built():
    """Куб с фаской плюс лишняя плоская грань сетки в выделении: она не разрешена и вне анализа, остальное - в анализе."""

    view, kinds = _cube_view()
    coords, faces = dict(view.coords), dict(view.faces)
    base = max(coords) + 1
    grid_coords, grid_faces = _grid()
    for key, point in grid_coords.items():
        coords[base + key] = (point[0] + 10.0, point[1], point[2])
    for key, cycle in grid_faces.items():
        faces[1000 + key] = tuple(base + item for item in cycle)
    selected = set(view.selected) | {1004}
    plan = bf.plan_faces(_view(coords, faces, selected), bf.INWARD, faces)
    assert not plan.refusal and plan.split.unresolved == {1004} and 1004 not in plan.scope
    assert len(plan.scope) == 20 and dict(plan.events)[bf.BEVEL_SPLIT_UNRESOLVED] == 1
    assert "BEVEL_SPLIT_UNRESOLVED 1" in bf.plan_summary(plan)


def test_empty_and_full_selections_are_refused_by_name():
    coords, faces, kinds = _chamfered_cube()
    assert bf.plan_faces(_view(coords, faces, ()), bf.INWARD, kinds).refusal == bf.BEVEL_FACES_EMPTY_SELECTION
    assert bf.plan_faces(_view(coords, faces, kinds), bf.OUTWARD, kinds).refusal == bf.BEVEL_FACES_NOTHING_OUTSIDE
    with pytest.raises(ValueError):
        bf.plan_faces(_view(coords, faces, {0}), "SIDEWAYS", kinds)


def test_a_triangulated_strip_is_not_a_column_and_its_faces_are_unresolved_by_name():
    coords, faces, kinds = _chamfered_cube()
    faces, kinds = dict(faces), dict(kinds)
    strip = min(face for face, kind in kinds.items() if kind == "strip")
    a, b, c, d = faces.pop(strip)
    kinds.pop(strip)
    for cycle in ((a, b, c), (a, c, d)):
        faces[len(faces) + 100] = cycle
        kinds[max(faces)] = "strip"
    selected = {face for face, kind in kinds.items() if kind != "flat"}
    split = bf.geometric_split(_view(coords, faces, selected))
    events = dict(split.events)
    assert events[bf.BEVEL_COLUMN_NOT_QUAD] == 2  # два рельса треугольной полосы
    assert len(split.strip) == 11
    # две треугольные половины касаются полос только через узлы: компонента узлов и половин - узел (касается других полос)
    assert {face for face in faces if face >= 100} <= split.node


def _fold(segments):
    """Пол (z=0) и стена (x=0), между ними профиль из `segments` квадов вдоль оси y; концы полосы - открытая граница."""

    import math

    radius = 0.2
    coords = {}
    profile = []
    for step in range(segments + 1):
        angle = math.pi / 2 * step / segments
        profile.append((radius - radius * math.sin(angle), radius - radius * math.cos(angle)))
    rows = [(1.0, 0.0)] + profile + [(0.0, 1.0)]
    for row, (x, z) in enumerate(rows):
        for column, y in enumerate((0.0, 1.0)):
            coords[row * 2 + column] = (x, y, z)
    faces = {}
    for row in range(len(rows) - 1):
        faces[row] = (row * 2, row * 2 + 1, row * 2 + 3, row * 2 + 2)
    selected = set(range(1, len(rows) - 2))
    return coords, faces, selected


@pytest.mark.parametrize("segments", [1, 2, 4])
def test_a_profile_without_nodes_is_one_strip_column(segments):
    coords, faces, selected = _fold(segments)
    split = bf.geometric_split(_view(coords, faces, selected))
    assert split.strip == selected and not split.node and not split.unresolved


def test_the_scope_digest_follows_the_side_the_faces_and_the_cut():
    view, kinds = _cube_view()
    inward = bf.plan_faces(view, bf.INWARD, kinds)
    outward = bf.plan_faces(view, bf.OUTWARD, kinds)
    assert bf.scope_digest("mesh", inward) == bf.scope_digest("mesh", inward)
    assert len({bf.scope_digest("mesh", inward), bf.scope_digest("mesh", outward), bf.scope_digest("other", inward)}) == 3
    uncut = bf.BevelFacesPlanV1(inward.side, inward.selected, inward.scope)
    assert bf.scope_digest("mesh", uncut) != bf.scope_digest("mesh", inward)
