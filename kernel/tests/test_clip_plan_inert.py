"""План станций цепей в резке (`CHAIN_STATION_PLAN_V1`, S1): инертные поперечные рёбра `FREE`-вершин не режут.

Ребро между двумя гранями, которое план назвал инертным (обе стороны в допуске хорды от плоскости друг друга), резка по граням обязана
пропустить так же, как диагональ грани: многоугольник поперёк ребра остаётся одним куском, вершины на ребре не рождаются. Остальное — как
прежде: чужие пары плана (грани нет у домена) не делают ничего, настоящее ребро вне плана режет, невыпуклая грань внутри группы плана
сохраняет свою защиту от складки, грань со складкой обхода в группу не идёт. Пары плана — часть ключа памяти резки.
"""

from __future__ import annotations

from fractions import Fraction

from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.materialize import clip, clip_cells
from cftuv_envelope.materialize.clip import _cut_by_faces, clip_policy
from cftuv_envelope.materialize.clip_memo import clip_key
from cftuv_envelope.materialize.clip_cells import build_cells

from test_clip_faces_law import L_CHART, L_FAN, budget, lift_of, prepared, square

POLYGONS = DecalTopologyLawV1.PLANAR_POLYGONS_V1
FLAT = (0, 0, 0, 0)
#: Прямоугольник поперёк общего ребра `x = 4` двух квадратов; диагонали `y = x` (у `f0`) и `y = x - 4` (у `f1`) он не пересекает.
ACROSS_EDGE = [[(3, 1), (5, 1), (5, 2), (3, 2)]]


def pair(first: str, second: str):
    return frozenset({frozenset((first, second))})


def cut(lift, polygons_xy, inert=frozenset()):
    spend = budget()
    keys, points, cycles, polygons = prepared(polygons_xy)
    plane = lift.bind(spend)
    result = _cut_by_faces(
        plane, spend, points, cycles, polygons, POLYGONS, frozenset(), [False] * len(polygons_xy), inert=inert
    )
    return result, keys


def two_squares(second_heights=FLAT):
    return lift_of([square("f0", 0, 0, FLAT), square("f1", 4, 0, second_heights)])


def found_points(result):
    return {tuple(axis.as_rational() for axis in value) for value in result.points.values()}


def test_an_inert_edge_of_the_plan_is_not_cut_and_the_polygon_stays_one_piece():
    result, _keys = cut(two_squares(), ACROSS_EDGE, pair("f0", "f1"))
    assert not result.points
    assert [len(item) for item in result.polygons[0]] == [4]
    found = dict(result.counters)
    assert found[clip.PLAN_INERT_FACE_PAIRS] == 1 and found[clip.PLAN_INERT_CUTS_AVOIDED] == 1
    assert found[clip.FACES_CUT] == 0 and found[clip.FACES_BOUNDARY_MISMATCH] == found[clip.FACES_OVERHANG] == 0


def test_without_the_plan_the_real_edge_cuts_as_before_and_no_plan_counter_is_written():
    result, _keys = cut(two_squares(), ACROSS_EDGE)
    assert found_points(result) == {(4, 1), (4, 2)}
    assert sorted(len(item) for item in result.polygons[0]) == [4, 4]
    assert clip.PLAN_INERT_FACE_PAIRS not in dict(result.counters)


def test_a_pair_of_a_face_the_domain_does_not_have_changes_nothing():
    """Пара плана с гранью чужого патча (в подъёме её нет) не склеивает ничего: ответ побитово тот же, что без плана."""

    plain, _keys = cut(two_squares(), ACROSS_EDGE)
    foreign, _foreign_keys = cut(two_squares(), ACROSS_EDGE, pair("f1", "elsewhere"))
    assert foreign.polygons == plain.polygons and list(foreign.points) == list(plain.points)
    assert clip.PLAN_INERT_FACE_PAIRS not in dict(foreign.counters)


def test_a_pair_that_is_not_the_edge_the_polygon_crosses_does_not_hide_that_edge():
    """Три грани: инертна пара `f1`-`f2`, а многоугольник пересекает ребро `f0`-`f1`: оно режет."""

    lift = lift_of([square("f0", 0, 0, FLAT), square("f1", 4, 0, FLAT), square("f2", 8, 0, FLAT)])
    result, _keys = cut(lift, ACROSS_EDGE, pair("f1", "f2"))
    assert found_points(result) == {(4, 1), (4, 2)}


def test_pairs_chain_into_one_group_across_three_faces():
    lift = lift_of([square("f0", 0, 0, FLAT), square("f1", 4, 0, FLAT), square("f2", 8, 0, FLAT)])
    inert = frozenset({frozenset(("f0", "f1")), frozenset(("f1", "f2"))})
    plan = build_cells(lift.triangles, inert=inert)
    assert plan.plan_pairs == 2 and len({cell.group for cell in plan.cells}) == 1
    assert all(cell.group == ("p", "f0") for cell in plan.cells)
    result, _keys = cut(lift, [[(3, 1), (9, 1), (9, 2), (3, 2)]], inert)
    assert not result.points and [len(item) for item in result.polygons[0]] == [4]


def test_the_group_of_two_flat_faces_has_no_chord_estimate_of_its_own():
    lift = two_squares()
    plan = build_cells(lift.triangles, inert=pair("f0", "f1"))
    assert {cell.group for cell in plan.cells} == {("p", "f0")}
    assert {cell.flat_square for cell in plan.cells} == {Fraction(0)}


def l_item(bump):
    """Г-образная грань `L` (стена с проёмом) веером из угла; `bump` — высота вершины `(4, 2)`, метры."""

    heights = [0, 0, Fraction(bump), 0, 0, 0]
    return ("L", L_CHART, [(x, y, Fraction(h)) for (x, y), h in zip(L_CHART, heights)], L_FAN)


def test_a_non_convex_face_inside_a_plan_group_keeps_its_own_chord_estimate():
    """Г-образная грань в группе плана: оценка глубины группы — её `2ρ`, защита от складки не пропадает."""

    alone = build_cells(lift_of([l_item(Fraction(1, 5))]).triangles)
    (alone_flat,) = {cell.flat_square for cell in alone.cells}
    assert alone_flat > clip_cells.CLIP_DIAGONAL_CHORD_BUDGET**2
    mixed = lift_of([l_item(Fraction(1, 5)), square("next", 4, 0, FLAT)])
    plan = build_cells(mixed.triangles, inert=pair("L", "next"))
    assert plan.plan_pairs == 1 and {cell.group for cell in plan.cells} == {("p", "L")}
    assert {cell.flat_square for cell in plan.cells} == {alone_flat}


def test_a_face_with_mixed_winding_stays_out_of_the_group_and_keeps_its_name():
    bow = (
        "bow",
        [(4, 0), (6, 0), (8, 2), (6, 4)],
        [(x, y, Fraction(0)) for x, y in [(4, 0), (6, 0), (8, 2), (6, 4)]],
        ((0, 1, 2), (3, 2, 1)),
    )
    lift = lift_of([square("f0", 0, 0, FLAT), bow])
    plan = build_cells(lift.triangles, inert=pair("f0", "bow"))
    assert plan.plan_pairs == 0 and {cell.group for cell in plan.cells} == {None}
    assert [name for name, _reason in plan.unmergeable] == ["bow"]


def test_the_pairs_are_part_of_the_memory_key_of_the_stage():
    """Те же многоугольники и подъём с планом и без него — разные ответы, поэтому разные ключи памяти резки."""

    lift = two_squares()
    spend = budget()
    keys, points, cycles, polygons = prepared(ACROSS_EDGE)
    plane = lift.bind(spend)
    inputs = {
        "points": points,
        "cycles": cycles,
        "polygons": polygons,
        "law": POLYGONS,
        "seam": frozenset(),
        "fans": [False],
        "flows": None,
        "by_faces": True,
    }
    plain = clip_key({**inputs, "inert": frozenset()}, plane.triangles, clip_policy())
    planned = clip_key({**inputs, "inert": pair("f0", "f1")}, plane.triangles, clip_policy())
    other = clip_key({**inputs, "inert": pair("f0", "f2")}, plane.triangles, clip_policy())
    assert len({plain, planned, other}) == 3


def test_only_the_silhouette_law_with_a_clip_by_faces_reads_the_plan():
    """Три прежних закона и резка по треугольникам план не читают: их ответы побитово прежние."""

    from types import SimpleNamespace

    from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
    from cftuv_envelope.materialize import domain

    pair_record = SimpleNamespace(
        disposition=domain.inert_face_pairs.__globals__["FREE"],
        inert_face_pairs=((SimpleNamespace(value="f0"), SimpleNamespace(value="f1")),),
    )
    prepared = SimpleNamespace(compilation=SimpleNamespace(chain_station_plans=(SimpleNamespace(stations=(pair_record,)),)))
    by_faces = SimpleNamespace(lift_law=NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1)
    by_triangles = SimpleNamespace(lift_law=NearPlanarLiftLawV1.SOURCE_TRIANGLES_CLIPPED_V1)
    not_clipped = SimpleNamespace(lift_law=NearPlanarLiftLawV1.SOURCE_TRIANGLES_V1)
    assert domain._inert_pairs(prepared, True, by_faces) == pair("f0", "f1")
    for silhouette, admission in ((False, by_faces), (True, by_triangles), (True, not_clipped)):
        assert domain._inert_pairs(prepared, silhouette, admission) == frozenset()
