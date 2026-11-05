"""Законы конца стены над числами: `WALL_NODE_NEAR_WELD_V1` (сварка узлов в пределах шума), `WALL_EDGE_SPLIT_V1` (короткий узел делит ребро длинной стены) и `WALL_NODE_MEAN_LEVEL_V1` (оба узла в среднем по ребру)."""

from __future__ import annotations

from dataclasses import dataclass

from cftuv_envelope.mesh_glue import glue_domains
from cftuv_envelope.mesh_weld import WALL_NODE_WELD_TOLERANCE, weld_core

LAW = (4.0, 1e-9, 1e-9, 1e-2, WALL_NODE_WELD_TOLERANCE, 1.0, 0.0)
MEAN_LAW = (*LAW[:6], 1.0)
ANCHOR = "location:src:host:A"
TOP = (1.0, 0.0, 0.0)
N1 = (0.0, 0.0, 1.0)
N2 = (-1.0, 0.0, 0.0)


@dataclass(frozen=True)
class _Domain:
    positions: tuple
    normals: tuple
    refs: tuple
    walls: tuple
    faces: tuple = ()


def _wall(patch: int, depth: float, normal: tuple) -> _Domain:
    """Стена `[якорь, узел]` вдоль +y и ещё две вершины грани за ней."""

    node = (TOP[0], TOP[1] + depth, TOP[2])
    side = 1.0 if patch else -1.0
    far_top = (TOP[0] + side, TOP[1], TOP[2])
    far_node = (TOP[0] + side, TOP[1] + depth, TOP[2])
    return _Domain((TOP, node, far_node, far_top), (normal,) * 4, (ANCHOR, f"location:node:{patch}", None, None), ((0, 1),), ((0, 1, 2, 3),))


def _view(depth: float) -> tuple:
    """`(грани, uv, владельцы, швы)` одного четырёхугольника: U вдоль верха, V = глубина / depth."""

    return (((0, 1, 2, 3),), ((0.0, 0.0), (0.0, 1.0), (1.0, 1.0), (1.0, 0.0)), (0,), ())


def _answer(first: float, second: float) -> tuple:
    return weld_core((_wall(0, first, N1), _wall(1, second, N2)), 0.0, LAW)


def test_wall_nodes_within_the_tolerance_are_one_vertex():
    positions, index, *_rest = _answer(0.5, 0.5 + WALL_NODE_WELD_TOLERANCE / 2)
    near, splits = _rest[-4:-2]
    assert index[0][1] == index[1][1]
    assert near == 1 and splits == ()
    assert len(positions) == 6


def test_bitwise_equal_wall_nodes_are_not_named_near():
    _positions, index, *rest = _answer(0.5, 0.5)
    near, splits = rest[-4:-2]
    assert index[0][1] == index[1][1]
    assert near == 0 and splits == ()


def test_wall_nodes_beyond_the_tolerance_stay_and_the_shorter_splits_the_longer():
    positions, index, *rest = _answer(0.5, 0.5 + WALL_NODE_WELD_TOLERANCE * 2)
    near, splits = rest[-4:-2]
    assert index[0][1] != index[1][1]
    assert near == 0
    assert len(splits) == 1 and splits[0][:5] == (1, 0, 1, 0, 1)
    assert len(positions) == 7


def test_a_real_step_stays_in_three_dimensions_and_the_shared_part_becomes_one_edge():
    positions, index, *rest = _answer(0.25, 1.0)
    splits = rest[-3]
    assert splits[0][5] == 0.25
    before = {tuple(p) for p in positions}
    faces, uvs, _domains, _owners, _seams, placed, unplaced = glue_domains((10, 20), (_view(0.25), _view(1.0)), index, splits)
    assert unplaced == 0
    assert placed == ((), ((1, 0, 1, 0.25),))
    short_node = index[0][1]
    assert short_node in faces[1] and faces[1][:3] == (index[1][0], short_node, index[1][1])
    # новая петля — та же доля вдоль ребра в UV; позиции не тронуты, ступень осталась
    assert uvs[4 + 1] == (0.0, 0.25)
    assert {tuple(p) for p in positions} == before
    assert positions[index[0][1]][1] == 0.25 and positions[index[1][1]][1] == 1.0
    edges = [frozenset((f[i], f[(i + 1) % len(f)])) for f in faces for i in range(len(f))]
    assert edges.count(frozenset((index[0][0], short_node))) == 2
    assert edges.count(frozenset((short_node, index[1][1]))) == 1


def test_a_split_without_its_edge_is_counted_and_not_placed():
    _positions, index, *rest = _answer(0.25, 1.0)
    faces, uvs, *_middle, placed, unplaced = glue_domains((10, 20), (_view(0.25), (((2, 3, 3),), ((0.0, 0.0),) * 3, (0,), ())), index, rest[-3])
    assert unplaced == 1 and placed == ((), ())
    assert len(uvs) == 7


def test_two_wall_nodes_become_one_node_at_the_mean_depth():
    positions, index, *rest = weld_core((_wall(0, 0.25, N1), _wall(1, 1.0, N2)), 0.0, MEAN_LAW)
    near, splits, means, refused = rest[-4:]
    assert index[0][1] == index[1][1]
    assert positions[index[0][1]] == (1.0, 0.625, 0.0)
    assert means == ((0, 1, 1, 1),) and refused == 0 and near == 0 and splits == ()


def test_the_mean_is_refused_by_name_when_a_face_would_fold_and_the_split_stays():
    """Вершина кольца стоит между узлом и средним: ребро нового узла пересекает кольцо, узлы остаются на месте, разрез тот же, что без закона."""

    long_wall = _wall(1, 1.0, N2)
    # Кольцо якорь-узел-a-b: сторона a-b проходит через ребро стены выше среднего, а ниже него кольцо вывернулось бы.
    positions = (TOP, (1.0, 1.0, 0.0), (1.5, 0.3, 0.0), (1.02, 0.9, 0.0), (1.02, 0.1, 0.0))
    folding = _Domain(positions, (N2,) * 5, (ANCHOR, "location:node:1", None, None, None), ((0, 1),), ((0, 1, 2, 3, 4),))
    short = _wall(0, 0.25, N1)
    answer = weld_core((short, folding), 0.0, MEAN_LAW)
    means, refused = answer[-2:]
    assert means == () and refused == 2  # оба узла пары
    assert answer[-3] != ()  # разрез закона WALL_EDGE_SPLIT_V1 на месте
    assert answer[1][0][1] != answer[1][1][1]
    assert long_wall.walls == ((0, 1),)


def test_without_faces_the_mean_is_refused_by_name():
    plain = lambda wall: _Domain(wall.positions, wall.normals, wall.refs, wall.walls)
    answer = weld_core((plain(_wall(0, 0.25, N1)), plain(_wall(1, 1.0, N2))), 0.0, MEAN_LAW)
    assert answer[-2:] == ((), 2)

