"""Нажатие по граням бевела (BEVEL_FACES_INWARD_OUTWARD_V1, BEVEL_NODE_STRIP_SPLIT_V1): разбиение «узел | полоса», стороны и отказы.

Меши - аналитические, без Blender: куб с фаской в один сегмент (6 плоских граней, 12 полос, 8 треугольных узлов) и сгиб с профилем
в два сегмента без узлов. Каждый отброс эвристики и каждый отказ проверяется по имени.
"""

from __future__ import annotations

import itertools

import pytest

from cftuv import bevel_faces as bf
from cftuv.bevel_seed import (
    BEVEL_SEED_NOT_BEVEL,
    BEVEL_SEED_OFFSET_MISMATCH,
    BEVEL_SELECT_FROM_SEED,
    bevel_offset,
    select_from_seeds,
)

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


def test_seams_close_the_bevel_rails_and_every_node_into_one_network():
    """BEVEL_SEAM_MARK_V1: весь бевел куба - 24 ребра рельсов и 24 ребра узлов; каждый конец шва рельса лежит на шве узла (стык)."""

    view, kinds = _cube_view()
    seams, split = bf.seam_edges(view)
    assert split.node == {face for face, kind in kinds.items() if kind == "node"}
    rails = bf.outward_sources(view)
    assert len(rails) == 24 and len(seams) == 48 and rails <= seams
    by_id = {edge_id: key for key, edge_id in view.edge_ids.items()}
    node_vertices = {vertex for face in split.node for vertex in view.faces[face]}
    assert all(by_id[edge_id] <= node_vertices for edge_id in seams - rails)


def test_a_single_selected_node_gets_seams_on_its_whole_boundary():
    coords, faces, kinds = _chamfered_cube()
    node = min(face for face, kind in kinds.items() if kind == "node")
    view = _view(coords, faces, {node})
    seams, _split = bf.seam_edges(view)
    cycle = faces[node]
    assert seams == {view.edge_ids[frozenset((cycle[i], cycle[(i + 1) % len(cycle)]))] for i in range(len(cycle))}


def _noisy_top_corner(rise):
    """Угол торца поля (`rounded_wall_noise_top`, BEVEL_SPLIT_RAIL_LOCAL_PLANE_V1) в миниатюре: фаска в один сегмент на трёх рёбрах угла
    (верх z = 0, фасад y = 0, бок x = 0), треугольный узел T; полоса вдоль x разрезана на S1 (у узла) и S2. Верх у S1 - неплоский квад
    Q1, дальний угол поднят на `rise`: нормаль Ньюэлла всей грани уходит от ребра бевела на ~30 градусов, а у рельса грань плоская."""

    w, a, b, end, low = 0.1, 0.3, 0.6, 1.3, -1.0
    points: dict = {}
    named: dict = {}

    def add(name, cycle):
        named[name] = tuple(points.setdefault(point, len(points)) for point in cycle)

    add("T", [(w, w, 0.0), (w, 0.0, -w), (0.0, w, -w)])
    add("S1", [(w, w, 0.0), (a, w, 0.0), (a, 0.0, -w), (w, 0.0, -w)])
    add("S2", [(a, w, 0.0), (end, w, 0.0), (end, 0.0, -w), (a, 0.0, -w)])
    add("Y", [(w, w, 0.0), (0.0, w, -w), (0.0, b, -w), (w, b, 0.0)])
    add("Z", [(w, 0.0, -w), (w, 0.0, low), (0.0, w, low), (0.0, w, -w)])
    add("Q1", [(w, w, 0.0), (w, b, 0.0), (a, b, rise), (a, w, 0.0)])
    add("Q2", [(a, w, 0.0), (a, b, rise), (end, b, 0.0), (end, w, 0.0)])
    add("front", [(w, 0.0, -w), (a, 0.0, -w), (end, 0.0, -w), (end, 0.0, low), (w, 0.0, low)])
    add("side", [(0.0, w, -w), (0.0, w, low), (0.0, b, low), (0.0, b, -w)])
    ids = {name: index for index, name in enumerate(sorted(named))}
    faces = {ids[name]: cycle for name, cycle in named.items()}
    coords = {index: point for point, index in points.items()}
    selected = {ids[name] for name in ("T", "S1", "S2", "Y", "Z")}
    return _view(coords, faces, selected), ids


def _edge(view, ids, first, second):
    shared = set(bf._edges_of(view.faces[ids[first]])) & set(bf._edges_of(view.faces[ids[second]]))
    return view.edge_ids[shared.pop()]


def test_a_strip_piece_beside_a_corner_node_under_a_noisy_top_is_a_strip_by_name():
    """BEVEL_SPLIT_RAIL_LOCAL_PLANE_V1 (поле: Seam Bevel на `rounded_wall_noise_top`, верхний угол торца): кусок полосы S1 у узла лежит
    под неплоской гранью шумного верха; по нормали всей грани колонка S1 отбрасывалась (RAILS_NOT_PARALLEL), S1 уходил в узел - шва
    между узлом и S1 не было, а шов вставал поперёк полосы S1 | S2. Плоскость у рельса принимает колонку, и это названо числом."""

    flat_view, _ids = _noisy_top_corner(0.0)
    assert dict(bf.geometric_split(flat_view).events).get(bf.BEVEL_COLUMN_RAIL_LOCAL_PLANE) is None
    view, ids = _noisy_top_corner(0.25)
    seams, split = bf.seam_edges(view)
    assert split.node == {ids["T"]} and split.strip == {ids[name] for name in ("S1", "S2", "Y", "Z")}
    assert dict(split.events) == {bf.BEVEL_COLUMN_RAIL_LOCAL_PLANE: 1}
    node_edges = {_edge(view, ids, "T", name) for name in ("S1", "Y", "Z")}
    assert node_edges <= seams and _edge(view, ids, "S1", "S2") not in seams  # узел - свой патч, поперёк полосы шва нет
    assert seams == bf.outward_sources(view) | node_edges


def test_seam_bevel_keeps_a_node_only_where_the_column_walk_and_the_selection_classifier_agree():
    """BEVEL_SEAM_NODE_AGREEMENT_V1: без атрибута узел Seam Bevel - грань, которую узлом зовут и обход колонок, и классификатор
    кнопки выделения; каждое несогласие - полоса с числом по имени, грани вне группы классификатора - за обходом; атрибут - как есть."""

    view, ids = _noisy_top_corner(0.25)
    wrong = bf.BevelSelectionV1(bf.BEVEL_SELECT_GEOMETRIC, bf.BEVEL_SELECT_GEOMETRIC, strip=frozenset({ids["S2"], ids["Y"], ids["Z"]}),
                                node=frozenset({ids["T"], ids["S1"]}))  # классификатор зовёт кусок у узла узлом (кластер узла)
    seams, split = bf.seam_edges(view, wrong)
    assert split.node == {ids["T"]} and dict(split.events)[bf.BEVEL_SEAM_CLASSIFIER_NODE_IS_COLUMN] == 1
    assert _edge(view, ids, "S1", "S2") not in seams and _edge(view, ids, "T", "S1") in seams

    cube, kinds = _cube_view()
    nodes = {face for face, kind in kinds.items() if kind == "node"}
    strips = {face for face, kind in kinds.items() if kind == "strip"}
    demoted, outside = min(nodes), max(nodes)
    partial = bf.BevelSelectionV1(bf.BEVEL_SELECT_GEOMETRIC, bf.BEVEL_SELECT_GEOMETRIC, strip=frozenset(strips | {demoted}),
                                  node=frozenset(nodes - {demoted, outside}))
    _seams, split = bf.seam_edges(cube, partial)
    assert split.node == nodes - {demoted} and demoted in split.strip
    assert dict(split.events) == {bf.BEVEL_SEAM_WALK_NODE_IS_STRIP: 1, bf.BEVEL_SEAM_OUTSIDE_CLASSIFIER: 1}

    truth = {face: (-16384 if kind == "node" else 0) for face, kind in kinds.items() if kind != "flat"}
    exact = bf.BevelFacesViewV1(cube.coords, cube.faces, cube.selected, cube.edge_ids, truth)
    assert bf.seam_edges(exact, partial) == bf.seam_edges(exact)  # Face Strength - verbatim, классификатор не читается


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


# --- Выделение граней бевела по всему мешу (BEVEL_SELECT_FACES_V1) ------------------------------------------------------------


def _outward_cube():
    """Куб с фаской с согласованными нормалями наружу (геометрия выделения читает знак двугранного угла)."""

    coords, faces, kinds = _chamfered_cube()
    oriented = {}
    for face, cycle in faces.items():
        normal = bf.face_normal(coords, cycle)
        center = [sum(coords[index][axis] for index in cycle) / len(cycle) for axis in range(3)]
        oriented[face] = cycle if sum(normal[axis] * center[axis] for axis in range(3)) > 0.0 else tuple(reversed(cycle))
    return coords, oriented, kinds


def _kind(kinds, name):
    return frozenset(face for face, kind in kinds.items() if kind == name)


def test_select_without_the_attribute_is_the_named_geometric_guess_and_finds_the_cube():
    coords, faces, kinds = _outward_cube()
    selection = bf.select_bevel_faces(bf.BevelMeshViewV1(coords, faces))
    assert selection.outcome == selection.source == bf.BEVEL_SELECT_GEOMETRIC and not selection.events
    assert selection.strip == _kind(kinds, "strip") and selection.node == _kind(kinds, "node")
    assert bf.selection_line("Cube", selection) == "Cube: BEVEL_SELECT_GEOMETRIC 12 strip / 8 node"


def test_select_with_face_strength_all_is_exact_and_needs_no_geometry():
    """Режим All: исходные грани STRONG, полоса 0, узел WEAK; ответ точен и там, где геометрия слепа (треугольная полоса)."""

    coords, faces, kinds = _outward_cube()
    faces, kinds = dict(faces), dict(kinds)
    strip = min(_kind(kinds, "strip"))
    a, b, c, d = faces.pop(strip)
    kinds.pop(strip)
    for offset, cycle in enumerate(((a, b, c), (a, c, d))):
        faces[100 + offset] = cycle
        kinds[100 + offset] = "strip"
    value = {"flat": bf.FACE_STRENGTH_STRONG, "strip": 0, "node": bf.FACE_STRENGTH_WEAK}
    selection = bf.select_bevel_faces(bf.BevelMeshViewV1(coords, faces, {face: value[kind] for face, kind in kinds.items()}))
    assert selection.outcome == bf.BEVEL_SELECT_FACE_STRENGTH and not selection.events
    assert selection.strip == _kind(kinds, "strip") and selection.node == _kind(kinds, "node")


def test_face_strength_without_strong_is_ambiguous_by_name_and_its_weak_nodes_seed_the_geometry():
    """Режим New: полоса и исходная грань обе 0 - путь геометрии под именем неоднозначности; узлы WEAK точны, расхождения - числом."""

    coords, faces, kinds = _outward_cube()
    strength = {face: (bf.FACE_STRENGTH_WEAK if kind == "node" else 0) for face, kind in kinds.items()}
    strength[min(_kind(kinds, "node"))] = 0  # узел, которого атрибут не пометил: геометрия его находит и оставляет, число названо
    lonely = max(faces) + 1  # WEAK на оторванном треугольнике, которого геометрия не нашла бы: добавлен, число названо
    coords = {**coords, 1000: (5.0, 0.0, 0.0), 1001: (5.1, 0.0, 0.0), 1002: (5.0, 0.1, 0.0)}
    faces = {**faces, lonely: (1000, 1001, 1002)}
    strength[lonely] = bf.FACE_STRENGTH_WEAK
    selection = bf.select_bevel_faces(bf.BevelMeshViewV1(coords, faces, strength))
    assert selection.outcome == selection.source == bf.BEVEL_SELECT_FACE_STRENGTH_AMBIGUOUS
    assert selection.strip == _kind(kinds, "strip") and selection.node == _kind(kinds, "node") | {lonely}
    assert dict(selection.events) == {bf.BEVEL_SELECT_NODE_FROM_ATTRIBUTE: 1, bf.BEVEL_SELECT_NODE_NOT_WEAK: 1}
    line = bf.selection_line("Cube", selection)
    assert line.startswith("Cube: BEVEL_SELECT_FACE_STRENGTH_AMBIGUOUS 12 strip / 9 node") and "Face Strength mode to All" in line
    lines = bf.selection_console_lines("Cube", selection)
    assert len(lines) == 3 and all(item.startswith("[CFTUV][BevelSelect] ") for item in lines)


def test_nothing_found_is_named_on_every_path():
    coords, faces = _grid()
    for strength, source in (
        (None, bf.BEVEL_SELECT_GEOMETRIC),
        ({face: 0 for face in faces}, bf.BEVEL_SELECT_FACE_STRENGTH_AMBIGUOUS),
        ({face: bf.FACE_STRENGTH_STRONG for face in faces}, bf.BEVEL_SELECT_FACE_STRENGTH),
    ):
        selection = bf.select_bevel_faces(bf.BevelMeshViewV1(coords, faces, strength))
        assert selection.outcome == bf.BEVEL_SELECT_NONE_FOUND and selection.source == source and not selection.faces
        assert bf.selection_line("Grid", selection).startswith("Grid: BEVEL_SELECT_NONE_FOUND (")


@pytest.mark.parametrize("segments", [1, 2, 4])
def test_a_profile_without_nodes_is_found_within_the_max_width_only(segments):
    """Сгиб с профилем радиуса 0.2 (полоса ~0.3): предел 0.5 - полоса найдена целиком; предел 0.25 по умолчанию - один квад отброшен
    по имени, ряд из нескольких режется по широчайшей грани по имени и целиком не находится."""

    coords, faces, profile = _fold(segments)
    wide = bf.select_bevel_faces(bf.BevelMeshViewV1(coords, faces), max_width=0.5)
    assert wide.outcome == bf.BEVEL_SELECT_GEOMETRIC and wide.strip == profile and not wide.node and not wide.events
    narrow = bf.select_bevel_faces(bf.BevelMeshViewV1(coords, faces))
    assert narrow.strip < profile
    expected = bf.BEVEL_FACE_WIDER_THAN_MAX if segments == 1 else bf.BEVEL_ROW_SPLIT_AT_WIDEST_FACE
    assert dict(narrow.events).get(expected, 0) >= 1, narrow.events


def _box(sx, sy, sz):
    """Прямоугольный ящик с нормалями наружу."""

    coords = {index: (sx * (index & 1), sy * ((index >> 1) & 1), sz * ((index >> 2) & 1)) for index in range(8)}
    faces = {0: (0, 2, 3, 1), 1: (4, 5, 7, 6), 2: (0, 1, 5, 4), 3: (2, 6, 7, 3), 4: (0, 4, 6, 2), 5: (1, 3, 7, 5)}
    return coords, faces


def test_a_thin_wall_is_a_ledge_not_a_bevel_by_name():
    """Стенка 2 x 0.1 x 1: узкие грани поворачивают 90 + 90 = 180 градусов - уступ, а не фаска."""

    coords, faces = _box(2.0, 0.1, 1.0)
    selection = bf.select_bevel_faces(bf.BevelMeshViewV1(coords, faces))
    assert selection.outcome == bf.BEVEL_SELECT_NONE_FOUND and dict(selection.events) == {bf.BEVEL_ROW_REJECTED: 4}


def test_a_ring_of_narrow_faces_wraps_by_name():
    """Боковина цилиндра: ряд узких квадов замкнут - не полоса."""

    import math

    coords, faces = {}, {}
    for step in range(32):
        angle = 2.0 * math.pi * step / 32
        coords[step] = (math.cos(angle), math.sin(angle), 0.0)
        coords[32 + step] = (math.cos(angle), math.sin(angle), 2.0)
    for step in range(32):
        following = (step + 1) % 32
        faces[step] = (step, following, 32 + following, 32 + step)
    selection = bf.select_bevel_faces(bf.BevelMeshViewV1(coords, faces))
    assert selection.outcome == bf.BEVEL_SELECT_NONE_FOUND and dict(selection.events) == {bf.BEVEL_ROW_WRAPS: 1}


def test_a_narrow_planar_row_is_trimmed_by_name():
    """Плоская сетка, средний ряд шириной 0.05: рельсы не гнут поверхность, каждый ряд обрезан до пустого."""

    rows = (0.0, 1.0, 1.05, 2.05)
    coords = {row * 4 + column: (float(column), y, 0.0) for row, y in enumerate(rows) for column in range(4)}
    faces = {row * 3 + column: (row * 4 + column, row * 4 + column + 1, row * 4 + column + 5, row * 4 + column + 4)
             for row in range(3) for column in range(3)}
    selection = bf.select_bevel_faces(bf.BevelMeshViewV1(coords, faces))
    assert selection.outcome == bf.BEVEL_SELECT_NONE_FOUND and dict(selection.events) == {bf.BEVEL_ROW_TRIMMED_FLAT_END: 3}


def test_the_dihedral_sign_reads_the_edge_order_not_the_centre_of_a_non_convex_face():
    """Стенка под ребром внутреннего угла L-образной грани: центр L лежит по другую сторону ребра, угол всё равно выпуклый."""

    coords = {0: (0, 0, 1), 1: (2, 0, 1), 2: (2, 0.6, 1), 3: (0.6, 0.6, 1), 4: (0.6, 2, 1), 5: (0, 2, 1),
              6: (0.6, 0.6, 0), 7: (2, 0.6, 0), 8: (0, 0, 0), 9: (2, 0, 0)}
    faces = {0: (3, 2, 7, 6), 1: (1, 0, 8, 9), 2: (0, 1, 2, 3, 4, 5)}  # стенки раньше L: в паре ребра первой идёт стенка
    geometry = bf._MeshGeometry(bf.BevelMeshViewV1(coords, faces), bf.BEVEL_SELECT_MAX_WIDTH)
    assert geometry.dihedral(frozenset((2, 3))) == pytest.approx(90.0)
    assert geometry.dihedral(frozenset((0, 1))) == pytest.approx(90.0)


# --- Концы профиля у угла бевелей (BEVEL_SELECT_CORNERS_V1): поле, `rounded_wall_noise_top`, фаски у торцов верха пропущены ----------


def _profile(points, rails=None):
    """Сечение `points` (x, z) вытянуто вдоль y на 1 (рельс k - вдоль `rails[k]`, по умолчанию ось y); ряды между соседними точками.
    Первый и последний ряд - пол и стена, остальные - профиль."""

    coords, faces = {}, {}
    for row, (x, z) in enumerate(points):
        along = (0.0, 1.0, 0.0) if rails is None else rails[row]
        coords[row * 2] = (x, 0.0, z)
        coords[row * 2 + 1] = (x + along[0], along[1], z + along[2])
    for row in range(len(points) - 1):
        faces[row] = (row * 2, row * 2 + 1, row * 2 + 3, row * 2 + 2)
    return coords, faces, frozenset(range(1, len(points) - 2))


def _arc(segments, radius=0.1):
    import math

    return [(radius - radius * math.sin(math.pi / 2 * step / segments), radius - radius * math.cos(math.pi / 2 * step / segments))
            for step in range(segments + 1)]


def test_a_strip_beside_a_face_the_clamp_collapsed_is_found_through_it_by_name():
    """Clamp overlap сжимает до нуля плоскую щель между двумя полосами (торец стены поля со своей фаской 45 градусов): у рельса
    рядом вырожденный квад без нормали. Раньше полоса без угла рельса молча выпадала; теперь угол читается сквозь щель - по имени."""

    arc = _arc(2)
    points = [(1.0, 0.0), arc[0], arc[1], arc[1], arc[2], (0.0, 1.0)]  # ряд 2 - щель нулевой ширины
    coords, faces, _rows = _profile(points)
    assert bf.face_normal(coords, faces[2]) is None
    selection = bf.select_bevel_faces(bf.BevelMeshViewV1(coords, faces))
    assert selection.outcome == bf.BEVEL_SELECT_GEOMETRIC and selection.strip == {1, 3} and not selection.node
    assert dict(selection.events)[bf.BEVEL_EDGE_THROUGH_DEGENERATE_FACE] == 2


def test_a_row_whose_outer_rail_counter_turns_slightly_stays_whole_by_name():
    """Шумный верх стены: грань у края фаски наклонена, внешний рельс гнёт против профиля на 6 градусов. Раньше смешанные знаки
    резали ряд по широчайшей грани и крайняя полоса терялась; теперь ряд целый, решение названо."""

    import math

    def step(point, degrees, length):
        return point[0] - length * math.cos(math.radians(degrees)), point[1] + length * math.sin(math.radians(degrees))

    floor_end = (0.2, 0.0)
    points = [step(floor_end, 6.0, -0.8), floor_end]  # пол поднимается к фаске на 6 градусов
    for degrees in (0.0, 30.0, 60.0):
        points.append(step(points[-1], degrees, 0.05))
    points.append(step(points[-1], 90.0, 0.8))
    coords, faces, rows = _profile(points)
    selection = bf.select_bevel_faces(bf.BevelMeshViewV1(coords, faces))
    assert selection.strip == rows == {1, 2, 3} and not selection.node
    assert dict(selection.events)[bf.BEVEL_ROW_END_COUNTER_TURN] == 1
    ledge = bf.select_bevel_faces(bf.BevelMeshViewV1(*_box(2.0, 0.1, 1.0)))  # уступ гнёт против на 90: исключение его не трогает
    assert ledge.outcome == bf.BEVEL_SELECT_NONE_FOUND


def test_a_row_whose_rails_fan_out_towards_a_corner_is_split_not_dropped():
    """У угла митра поворачивает рельсы профиля на 3 градуса за грань: соседние рельсы параллельны, первый и последний - нет.
    Раньше весь ряд отбрасывался (`BEVEL_BAND_NOT_PARALLEL` от первой грани); теперь ряд целый, пока уход в допуске рельса."""

    import math

    rails = [(math.sin(math.radians(3.0 * index)), math.cos(math.radians(3.0 * index)), 0.0) for index in range(5)]
    coords, faces, rows = _profile([(1.0, 0.0), *_arc(3), (0.0, 1.0)], rails=[rails[0], *rails, rails[-1]])
    selection = bf.select_bevel_faces(bf.BevelMeshViewV1(coords, faces))
    assert selection.strip == rows == {1, 2, 3} and bf.BEVEL_BAND_NOT_PARALLEL not in dict(selection.events)


def _sections(sections):
    """Сечения `(y, [(x, z), ...])` с одним числом точек; ряд r между сечениями s и s + 1 - грань `s * 100 + r`."""

    count = len(sections[0][1])
    coords = {s * count + r: (x, y, z) for s, (y, points) in enumerate(sections) for r, (x, z) in enumerate(points)}
    faces = {s * 100 + r: (s * count + r, (s + 1) * count + r, (s + 1) * count + r + 1, s * count + r + 1)
             for s in range(len(sections) - 1) for r in range(count - 1)}
    return coords, faces


def test_end_pieces_that_continue_a_strip_are_strips_by_name_and_a_wider_one_is_not():
    """Кусок фаски у торца: короткий (митра съела длину, квад шире половины длины), трапеция (clamp overlap срезал отступ у одной
    вершины: рельсы сходятся на 9.5 градуса) и слишком широкий (наклонная грань шумного верха у края). Раньше первые два молча
    выпадали из кандидатов; теперь кусок, что концом продолжает полосу не шире её в 1.5 раза, - полоса, каждый случай по имени."""

    profile = [(1.0, 0.0), (0.1, 0.0), (0.0, 0.1), (0.0, 1.0)]
    for far, expected in (
        ((1.16, profile), {bf.BEVEL_ROW_SHORT_STRIP: 1}),
        ((1.3, [(1.0, 0.0), (0.05, 0.0), (0.0, 0.1), (0.0, 1.0)]), {bf.BEVEL_FACE_TAPERED: 1}),
        ((2.0, [(1.0, 0.0), (0.1, 0.0), (0.0, 0.3), (0.0, 1.0)]), {bf.BEVEL_END_PIECE_WIDER: 1}),
    ):
        coords, faces = _sections([(0.0, profile), (1.0, profile), far])
        selection = bf.select_bevel_faces(bf.BevelMeshViewV1(coords, faces))
        piece = {101} if bf.BEVEL_END_PIECE_WIDER not in expected else set()
        assert selection.strip == {1} | piece and not selection.node, (far, selection)
        assert {name: count for name, count in selection.events if name in expected} == expected, selection.events


def _fanned_corner_cube(rise):
    """Куб с фаской, каждый узел - веер из трёх треугольников к центру, поднятому на `rise` наружу: так выглядит угол трёх бевелей
    в 2+ сегмента - каждая грань угла касается одной полосы."""

    coords, faces, kinds = _outward_cube()
    coords, faces, kinds = dict(coords), dict(faces), dict(kinds)
    for node in sorted(face for face, kind in kinds.items() if kind == "node"):
        cycle = faces.pop(node)
        kinds.pop(node)
        centre = [sum(coords[index][axis] for index in cycle) / 3.0 for axis in range(3)]
        length = sum(value * value for value in centre) ** 0.5
        apex = len(coords)
        coords[apex] = tuple(value * (1.0 + rise / length) for value in centre)
        for position in range(3):
            fan = max(faces) + 1
            faces[fan] = (cycle[position], cycle[(position + 1) % 3], apex)
            kinds[fan] = "node"
    return coords, faces, kinds


def test_a_corner_whose_faces_each_touch_one_strip_is_one_node_cluster_by_name():
    coords, faces, kinds = _fanned_corner_cube(0.01)
    selection = bf.select_bevel_faces(bf.BevelMeshViewV1(coords, faces))
    assert selection.strip == _kind(kinds, "strip") and selection.node == _kind(kinds, "node")
    assert dict(selection.events) == {bf.BEVEL_NODE_CLUSTER: 24}


def test_a_corner_lying_in_one_plane_stays_unselected_and_is_counted():
    """Веер в плоскости узла: рёбра между треугольниками не гнут поверхность - не узел геометрии (как плоская обрезка сетки у фаски),
    и каждая такая грань названа числом, а не молчит."""

    coords, faces, kinds = _fanned_corner_cube(0.0)
    selection = bf.select_bevel_faces(bf.BevelMeshViewV1(coords, faces))
    assert selection.strip == _kind(kinds, "strip") and not selection.node
    assert dict(selection.events) == {bf.BEVEL_NODE_FLAT_EDGE: 24}


# --- Лишние грани у торца шумного верха (BEVEL_SELECT_SEED_V1): поле, `rounded_wall_noise_top.001`, две лишние грани у угла торца x --


def test_a_wide_end_face_that_keeps_turning_is_trimmed_from_the_row_by_name():
    """Фаска в один сегмент (0.06) и рядом наклонная грань шумного верха вдвое шире: вместе - профиль 90 градусов уже предела, и ряд
    брал обе. Профиль к широкой грани не выполаживается (её внешний рельс гнёт больше общего) - она не сегмент: ряд без неё, по имени.
    Эллипс (Percent: широкая хорда там, где поворот мал) остаётся целым."""

    import math

    def step(point, degrees, length):
        return point[0] - length * math.cos(math.radians(degrees)), point[1] + length * math.sin(math.radians(degrees))

    for tilt, expected, trimmed in ((60.0, {1}, 1), (80.0, {1, 2}, 0)):
        points = [(1.0, 0.0), (0.2, 0.0)]
        for degrees, length in ((45.0, 0.06), (tilt, 0.12), (90.0, 0.8)):
            points.append(step(points[-1], degrees, length))
        coords, faces, _rows = _profile(points)
        selection = bf.select_bevel_faces(bf.BevelMeshViewV1(coords, faces))
        assert selection.strip == expected and not selection.node, (tilt, selection)
        assert dict(selection.events).get(bf.BEVEL_ROW_UNEVEN_FACE, 0) == trimmed, selection.events


def test_a_node_between_nodes_is_measured_by_their_width_not_by_the_max_width():
    """Грань, что касается только узлов, раньше мерялась долей предела ширины (0.25 / 2.5 = 0.1 - предел узла 0.375), и n-угольник
    исходного меша у угла торца (поперечник 0.33 при фасках 0.06) проходил в узлы. Теперь узел наследует наименьшую ширину узлов,
    которых касается; полоса по-прежнему мерило, если касается; ни того ни другого - доля предела."""

    coords, faces = _grid()
    geometry = bf._MeshGeometry(bf.BevelMeshViewV1(coords, faces), bf.BEVEL_SELECT_MAX_WIDTH)
    geometry.node_width.update({1: 0.06, 2: 0.09})
    assert bf._node_width(geometry, {}, {1, 2}) == 0.06
    assert bf._node_width(geometry, {3: 0.2}, {1, 3}) == 0.2
    assert bf._node_width(geometry, {}, {4}) == pytest.approx(bf.BEVEL_SELECT_MAX_WIDTH / bf.BEVEL_NODE_DIAMETER_FACTOR)
    assert bf._node_limit(geometry, {}, {1, 2}) < 0.33 < bf._node_limit(geometry, {}, {4})


# --- Группа бевела от семян (BEVEL_SELECT_SEED_V1): владелец выделяет грань бевела, кнопка восстанавливает её группу -------------


def _two_cubes():
    """Два куба с фаской (второй сдвинут на 5 по x): две группы бевела, не связанные рёбрами."""

    coords, faces, kinds = _outward_cube()
    shift = max(coords) + 1
    coords = {**coords, **{index + shift: (x + 5.0, y, z) for index, (x, y, z) in coords.items()}}
    second = {face + 100: tuple(index + shift for index in cycle) for face, cycle in faces.items()}
    return coords, {**faces, **second}, {**kinds, **{face + 100: kind for face, kind in kinds.items()}}


def _two_width_cube(wide, narrow):
    """Куб с фаской: вертикальные рёбра сняты на `wide` с обеих сторон, горизонтальные - на `narrow` по вертикальной грани и `wide`
    по верху и низу (отступ другой фаски); нормали наружу."""

    inset = {0: wide, 1: wide, 2: narrow}  # отступ грани оси k от ребра с гранью оси, вдоль которой меряется
    index, coords = {}, {}
    for axis, sign in itertools.product(range(3), (-1, 1)):
        others = [item for item in range(3) if item != axis]
        for a, b in itertools.product((-1, 1), (-1, 1)):
            point = [0.0, 0.0, 0.0]
            point[axis] = float(sign)
            point[others[0]] = a * (1.0 - inset[others[0]])
            point[others[1]] = b * (1.0 - inset[others[1]])
            index[(axis, sign, a, b)] = len(coords)
            coords[len(coords)] = tuple(point)
    base_coords, faces, kinds = _chamfered_cube()
    remap = {base: index[key] for key, base in _cube_keys().items()}
    faces = {face: tuple(remap[vertex] for vertex in cycle) for face, cycle in faces.items()}
    oriented = {}
    for face, cycle in faces.items():
        normal = bf.face_normal(coords, cycle)
        center = [sum(coords[vertex][axis] for vertex in cycle) / len(cycle) for axis in range(3)]
        oriented[face] = cycle if sum(normal[axis] * center[axis] for axis in range(3)) > 0.0 else tuple(reversed(cycle))
    return coords, oriented, kinds


def _cube_keys():
    """Номера вершин `_chamfered_cube` по ключу (ось, знак, знаки двух других осей): тот же порядок обхода."""

    keys = {}
    for axis, sign in itertools.product(range(3), (-1, 1)):
        for a, b in itertools.product((-1, 1), (-1, 1)):
            keys[(axis, sign, a, b)] = len(keys)
    return keys


def _vertical(coords, faces, face):
    """Полоса вдоль оси z (вертикальное ребро куба)."""

    zs = [coords[vertex][2] for vertex in faces[face]]
    return max(zs) - min(zs) > 1.0


def test_a_seed_grows_its_own_bevel_group_and_nothing_else():
    coords, faces, kinds = _two_cubes()
    seed = min(face for face, kind in kinds.items() if kind == "strip" and face < 100)
    selection = select_from_seeds(bf.BevelMeshViewV1(coords, faces, None, frozenset({seed})))
    assert selection.outcome == BEVEL_SELECT_FROM_SEED and selection.source == bf.BEVEL_SELECT_GEOMETRIC
    assert selection.strip == {face for face in _kind(kinds, "strip") if face < 100}
    assert selection.node == {face for face in _kind(kinds, "node") if face < 100}
    assert seed in selection.faces and selection.seeds == {seed}
    assert bf.selection_line("Cubes", selection) == "Cubes: BEVEL_SELECT_FROM_SEED 1 seeds -> 12 strip / 8 node (geometric)"
    node_seed = min(face for face, kind in kinds.items() if kind == "node" and face >= 100)  # семя-узел: мерило - ряд соседей
    grown = select_from_seeds(bf.BevelMeshViewV1(coords, faces, None, frozenset({node_seed})))
    assert grown.faces == {face for face, kind in kinds.items() if kind != "flat" and face >= 100}


def test_a_seed_that_is_not_a_bevel_face_is_named_and_grows_nothing():
    coords, faces, kinds = _two_cubes()
    flat = min(_kind(kinds, "flat"))
    alone = select_from_seeds(bf.BevelMeshViewV1(coords, faces, None, frozenset({flat})))
    assert alone.outcome == BEVEL_SEED_NOT_BEVEL and not alone.faces and alone.seeds == {flat}
    assert dict(alone.events)[BEVEL_SEED_NOT_BEVEL] == 1
    assert bf.selection_line("Cubes", alone) == "Cubes: BEVEL_SEED_NOT_BEVEL 1 seeds (geometric)"
    strip = min(face for face, kind in kinds.items() if kind == "strip" and face >= 100)
    mixed = select_from_seeds(bf.BevelMeshViewV1(coords, faces, None, frozenset({flat, strip})))
    assert mixed.outcome == BEVEL_SELECT_FROM_SEED and dict(mixed.events)[BEVEL_SEED_NOT_BEVEL] == 1
    assert flat not in mixed.faces and mixed.faces == {face for face, kind in kinds.items() if kind != "flat" and face >= 100}


def test_with_face_strength_all_the_seed_group_is_the_exact_connected_bevel_set():
    coords, faces, kinds = _two_cubes()
    value = {"flat": bf.FACE_STRENGTH_STRONG, "strip": 0, "node": bf.FACE_STRENGTH_WEAK}
    strength = {face: value[kind] for face, kind in kinds.items()}
    seed = min(face for face, kind in kinds.items() if kind == "node" and face >= 100)
    selection = select_from_seeds(bf.BevelMeshViewV1(coords, faces, strength, frozenset({seed})))
    assert selection.outcome == BEVEL_SELECT_FROM_SEED and selection.source == bf.BEVEL_SELECT_FACE_STRENGTH and not selection.events
    assert selection.strip == {face for face in _kind(kinds, "strip") if face >= 100}
    assert selection.node == {face for face in _kind(kinds, "node") if face >= 100}
    flat = min(_kind(kinds, "flat"))
    refused = select_from_seeds(bf.BevelMeshViewV1(coords, faces, strength, frozenset({flat})))
    assert refused.outcome == BEVEL_SEED_NOT_BEVEL and dict(refused.events) == {BEVEL_SEED_NOT_BEVEL: 1}


def test_a_bevel_of_another_offset_beyond_a_node_is_not_taken_by_name():
    """Вертикальные рёбра сняты на 0.1 (отступ 0.1), горизонтальные - на 0.1 x 0.02 (отступ дуги ~0.07). Поиск по всему мешу берёт
    все полосы. От полосы верха группа - кольцо верха: 4 полосы и 4 узла (узел касается двух полос верха), 4 вертикальные за узлами -
    другой бевел, отказ по имени; от вертикальной - она и два её узла: дальше только горизонтальные, 4 отказа."""

    coords, faces, kinds = _two_width_cube(0.1, 0.02)
    view = bf.BevelMeshViewV1(coords, faces)
    assert bf.select_bevel_faces(view).faces == _kind(kinds, "strip") | _kind(kinds, "node")
    vertical = {face for face in _kind(kinds, "strip") if _vertical(coords, faces, face)}
    assert len(vertical) == 4
    top = {face for face in _kind(kinds, "strip") - vertical if min(coords[vertex][2] for vertex in faces[face]) > 0.0}
    corners = {face for face in _kind(kinds, "node") if min(coords[vertex][2] for vertex in faces[face]) > 0.0}
    across = select_from_seeds(view, {min(top)})
    assert len(top) == len(corners) == 4 and across.strip == top and across.node == corners
    assert dict(across.events)[BEVEL_SEED_OFFSET_MISMATCH] == 4
    upright = select_from_seeds(view, {min(vertical)})
    assert upright.strip == {min(vertical)} and len(upright.node) == 2 and dict(upright.events)[BEVEL_SEED_OFFSET_MISMATCH] == 4


def test_the_bevel_offset_does_not_follow_the_chord():
    """Одна фаска 0.045 в один сегмент: у ребра 90 градусов хорда 0.0636, у ребра 50 - 0.038; отступ - 0.045 у обоих; дуга в 3
    сегмента радиуса 0.05 на 90 градусов - отступ 0.05."""

    import math

    for turn in (90.0, 50.0):
        chord = 2.0 * 0.045 * math.cos(math.radians(turn) / 2.0)
        assert bevel_offset(chord, turn, 1) == pytest.approx(0.045)
    chords = 3 * 2.0 * 0.05 * math.sin(math.radians(15.0))
    assert bevel_offset(chords, 90.0, 3) == pytest.approx(0.05)

# --- BOTH (BEVEL_FACES_BOTH_V2) ------------------------------------------------------------------------------------------------


def test_both_is_the_first_face_side_and_analyses_exactly_what_outward_analyses():
    """Both V2: анализ и источники - как у Outward (заливки граней бевела ядром нет, их копирует хост); разбиение есть - только для UV копии."""

    view, kinds = _cube_view()
    assert bf.FACE_SIDES[0] == bf.BOTH
    plan = bf.plan_faces(view, bf.BOTH, kinds)
    inward = bf.plan_faces(view, bf.INWARD, kinds)
    outward = bf.plan_faces(view, bf.OUTWARD, kinds)
    assert not plan.refusal and plan.scope == outward.scope and len(plan.scope) == 6
    assert plan.outward_sources == outward.outward_sources and len(plan.outward_sources) == 24
    assert not plan.cut_edges, "Both cuts nothing: the bevel faces are not analysed"
    assert plan.split == inward.split and plan.events == inward.events
    assert bf.scope_digest("mesh", plan) not in {bf.scope_digest("mesh", inward), bf.scope_digest("mesh", outward)}
    assert bf.plan_summary(plan).startswith("faces both: 20 selected, 12 strip / 8 node")


def test_both_names_a_selection_without_outside_faces_and_keeps_unresolved_faces_as_an_event():
    coords, faces, kinds = _chamfered_cube()
    every = _view(coords, faces, kinds)
    plan = bf.plan_faces(every, bf.BOTH, kinds)
    assert plan.refusal == bf.BEVEL_FACES_NOTHING_OUTSIDE and not plan.scope
    grid_coords, grid_faces = _grid()
    plan = bf.plan_faces(_view(grid_coords, grid_faces, {4}), bf.BOTH, grid_faces)
    assert not plan.refusal and plan.scope == tuple(sorted(set(grid_faces) - {4}))
    assert dict(plan.events)[bf.BEVEL_SPLIT_UNRESOLVED] == 1, "an unresolved face is copied too; its count is named"
    assert bf.plan_faces(_view(coords, faces, ()), bf.BOTH, kinds).refusal == bf.BEVEL_FACES_EMPTY_SELECTION
