"""Сварка вершин соседних доменов и митра смещения (`envelope_production_weld`): чистая математика.

Модуль не знает Blender: он получает вершины доменов (позиция батча, нормаль смещения, семантическая
ссылка) и возвращает вершины меша. Здесь проверяются три вещи: митра как пересечение сдвинутых плоскостей
(k = 1, 2, 3 и больше), сварка ТОЛЬКО по ссылке `location:src:` и ТОЛЬКО при побитово равных позициях,
и именованные отказы (расхождение позиций, вырожденная митра, швы складок, конфликт обхода).
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from cftuv import envelope_production_weld as weld
from cftuv.envelope_production_weld import (
    DomainVerticesV1,
    cross_domain_seams,
    half_edge_conflicts,
    miter_offset,
    weld_vertices,
)

UP = (0.0, 0.0, 1.0)
SIDE = (-1.0, 0.0, 0.0)
FRONT = (0.0, -1.0, 0.0)
REF = "location:src:host:7"


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _domain(patch_id, points, normals, refs):
    return DomainVerticesV1(patch_id, tuple(points), tuple(normals), tuple(refs))


# --------------------------------------------------------------------------
# Митра: n_i . o = d для каждого домена общей вершины
# --------------------------------------------------------------------------


def test_one_domain_offsets_along_its_own_normal():
    offset, factor, reason = miter_offset([UP])

    assert offset == UP and factor == 1.0 and reason == ""


def test_equal_normals_give_exactly_the_plain_offset_bitwise():
    normal = (0.6, 0.8, 0.0)

    offset, factor, _reason = miter_offset([normal, (0.6, 0.8, 0.0)])

    assert offset == normal and factor == 1.0


def test_two_domains_at_a_right_angle_offset_to_the_sum_of_the_normals():
    offset, factor, reason = miter_offset([UP, SIDE])

    assert reason == ""
    assert offset == pytest.approx((-1.0, 0.0, 1.0))
    assert factor == pytest.approx(math.sqrt(2.0))
    assert _dot(UP, offset) == pytest.approx(1.0) and _dot(SIDE, offset) == pytest.approx(1.0)


def test_two_domains_follow_the_closed_form_for_any_angle():
    n1 = (0.0, 0.0, 1.0)
    n2 = (math.sin(0.7), 0.0, math.cos(0.7))
    cosine = _dot(n1, n2)

    offset, factor, _reason = miter_offset([n1, n2])

    expected = tuple((a + b) / (1.0 + cosine) for a, b in zip(n1, n2))
    assert offset == pytest.approx(expected, abs=1e-12)
    assert factor == pytest.approx(1.0 / math.cos(0.35), rel=1e-9)


def test_three_orthogonal_domains_meet_in_one_point():
    offset, factor, reason = miter_offset([UP, SIDE, FRONT])

    assert reason == "" and offset == pytest.approx((-1.0, -1.0, 1.0))
    assert factor == pytest.approx(math.sqrt(3.0))


def test_three_skew_domains_have_one_solution_on_every_offset_plane():
    normals = [
        (0.0, 0.0, 1.0),
        (0.6, 0.0, 0.8),
        (0.0, -0.6, 0.8),
    ]

    offset, _factor, reason = miter_offset(normals)

    assert reason == ""
    for normal in normals:
        assert _dot(normal, offset) == pytest.approx(1.0, abs=1e-12)


def test_a_fourth_consistent_plane_is_accepted_and_an_inconsistent_one_is_refused():
    # Четвёртая единичная нормаль, проходящая через ту же точку `(-1, -1, 1)`: `n . o = 1`.
    consistent = [UP, SIDE, FRONT, (-2 / 3, -2 / 3, -1 / 3)]
    offset, _factor, reason = miter_offset(consistent)
    assert reason == ""
    assert offset == pytest.approx((-1.0, -1.0, 1.0), abs=1e-9)
    for normal in consistent:
        assert _dot(normal, offset) == pytest.approx(1.0, abs=1e-9)

    inconsistent = [UP, SIDE, FRONT, (0.0, 0.0, -1.0)]
    refused, factor, reason = miter_offset(inconsistent)
    assert refused is None and "do not meet" in reason


def test_two_nearly_opposite_normals_exceed_the_miter_limit_by_name():
    knife = (math.sin(3.1), 0.0, math.cos(3.1))

    offset, factor, reason = miter_offset([UP, knife])

    assert offset is None and reason == "the miter exceeds the limit"
    assert factor > weld.MITER_LIMIT


def test_exactly_opposite_normals_have_no_common_point():
    offset, factor, reason = miter_offset([UP, (0.0, 0.0, -1.0)])

    assert offset is None and factor is None and "do not meet" in reason


# --------------------------------------------------------------------------
# Сварка
# --------------------------------------------------------------------------


def _fold(first_position=(1.0, 0.0, 0.0), second_position=(1.0, 0.0, 0.0), ref=REF):
    """Два домена складки в одной вершине исходника: пол (`UP`) и стена (`SIDE`)."""

    floor = _domain(0, [first_position], [UP], [ref])
    wall = _domain(1, [second_position], [SIDE], [ref])
    return [floor, wall]


def test_one_shared_source_vertex_becomes_one_vertex_with_the_miter_offset():
    result = weld_vertices(_fold(), 0.02)

    assert len(result.positions) == 1
    assert result.index == ((0,), (0,))
    x, y, z = result.positions[0]
    assert (x, y, z) == pytest.approx((1.0 - 0.02, 0.0, 0.02))
    counters = dict(result.counters)
    assert counters[weld.COUNTER_WELD_GROUPS] == 1
    assert counters[weld.COUNTER_WELD_VERTICES_MERGED] == 1
    assert counters[weld.OUTCOME_WELD_POSITION_MISMATCH] == 0
    assert counters[weld.OUTCOME_WELD_MITER_FALLBACK] == 0
    assert result.warnings == ()


def test_the_welded_vertex_stays_on_every_domains_offset_plane():
    base = (3.0, -2.0, 0.5)
    result = weld_vertices(_fold(base, base), 0.5)

    (position,) = result.positions
    delta = tuple(a - b for a, b in zip(position, base))
    assert _dot(UP, delta) == pytest.approx(0.5) and _dot(SIDE, delta) == pytest.approx(0.5)


def test_an_unshared_vertex_keeps_its_plain_offset_bitwise():
    domain = _domain(0, [(0.1, 0.2, 0.3)], [(0.0, 0.6, 0.8)], ["location:src:host:9"])

    result = weld_vertices([domain], 0.02)

    assert result.positions == ((0.1 + 0.02 * 0.0, 0.2 + 0.02 * 0.6, 0.3 + 0.02 * 0.8),)
    assert dict(result.counters)[weld.COUNTER_WELD_GROUPS] == 0


def test_a_zero_offset_still_welds_and_leaves_the_base_position():
    result = weld_vertices(_fold(), 0.0)

    assert result.positions == ((1.0, 0.0, 0.0),)


def test_positions_that_differ_by_one_bit_are_never_welded_and_the_mismatch_is_named():
    nudged = math.nextafter(1.0, 2.0)

    result = weld_vertices(_fold(second_position=(nudged, 0.0, 0.0)), 0.02)

    assert len(result.positions) == 2
    assert result.index == ((0,), (1,))
    # Каждая вершина — со смещением по СВОЕЙ нормали: ничего не подтягивается друг к другу.
    assert result.positions[0] == (1.0, 0.0, 0.02)
    assert result.positions[1] == (nudged - 0.02, 0.0, 0.0)
    assert dict(result.counters)[weld.OUTCOME_WELD_POSITION_MISMATCH] == 1
    (warning,) = result.warnings
    assert warning[1] == weld.OUTCOME_WELD_POSITION_MISMATCH and "bitwise" in warning[2]


def test_negative_zero_is_not_bitwise_equal_to_zero():
    result = weld_vertices(_fold((1.0, 0.0, 0.0), (1.0, -0.0, 0.0)), 0.0)

    assert len(result.positions) == 2
    assert dict(result.counters)[weld.OUTCOME_WELD_POSITION_MISMATCH] == 1


def test_domain_local_node_references_never_weld():
    result = weld_vertices(_fold(ref="location:node:3"), 0.02)

    assert len(result.positions) == 2
    assert dict(result.counters)[weld.COUNTER_WELD_GROUPS] == 0


def test_a_missing_reference_never_welds():
    result = weld_vertices(
        [_domain(0, [(0.0, 0.0, 0.0)], [UP], [None]), _domain(1, [(0.0, 0.0, 0.0)], [UP], [None])],
        0.02,
    )

    assert len(result.positions) == 2


def test_two_of_three_domains_with_equal_positions_weld_and_the_third_stays_apart():
    nudged = math.nextafter(1.0, 2.0)
    domains = [
        _domain(0, [(1.0, 0.0, 0.0)], [UP], [REF]),
        _domain(1, [(1.0, 0.0, 0.0)], [SIDE], [REF]),
        _domain(2, [(nudged, 0.0, 0.0)], [FRONT], [REF]),
    ]

    result = weld_vertices(domains, 0.02)

    assert len(result.positions) == 2
    assert result.index == ((0,), (0,), (1,))
    counters = dict(result.counters)
    assert counters[weld.COUNTER_WELD_GROUPS] == 1 and counters[weld.COUNTER_WELD_VERTICES_MERGED] == 1
    assert counters[weld.OUTCOME_WELD_POSITION_MISMATCH] == 1


def test_three_domains_weld_into_one_vertex_at_the_cube_corner_miter():
    domains = [
        _domain(0, [(0.0, 0.0, 0.0)], [UP], [REF]),
        _domain(1, [(0.0, 0.0, 0.0)], [SIDE], [REF]),
        _domain(2, [(0.0, 0.0, 0.0)], [FRONT], [REF]),
    ]

    result = weld_vertices(domains, 0.1)

    assert len(result.positions) == 1
    assert result.positions[0] == pytest.approx((-0.1, -0.1, 0.1))
    assert dict(result.counters)[weld.COUNTER_WELD_VERTICES_MERGED] == 2


def test_a_degenerate_miter_keeps_the_vertices_apart_each_on_its_own_normal_and_is_named():
    domains = [
        _domain(0, [(0.0, 0.0, 0.0)], [UP], [REF]),
        _domain(1, [(0.0, 0.0, 0.0)], [(0.0, 0.0, -1.0)], [REF]),
    ]

    result = weld_vertices(domains, 0.02)

    assert result.positions == ((0.0, 0.0, 0.02), (0.0, 0.0, -0.02))
    assert dict(result.counters)[weld.OUTCOME_WELD_MITER_FALLBACK] == 1
    assert dict(result.counters)[weld.COUNTER_WELD_GROUPS] == 0
    (warning,) = result.warnings
    assert warning[1] == weld.OUTCOME_WELD_MITER_FALLBACK and "do not meet" in warning[2]


def test_the_per_vertex_normals_of_an_unfolded_domain_drive_the_miter():
    """Домен развёртки приносит нормаль вершины, а не нормаль плоскости."""

    tilted = (0.0, -0.6, 0.8)
    domains = [
        _domain(0, [(0.0, 0.0, 0.0)], [tilted], [REF]),
        _domain(1, [(0.0, 0.0, 0.0)], [SIDE], [REF]),
    ]

    result = weld_vertices(domains, 1.0)

    (position,) = result.positions
    assert _dot(tilted, position) == pytest.approx(1.0) and _dot(SIDE, position) == pytest.approx(1.0)


def test_vertex_numbers_follow_the_first_occurrence_by_domain_order():
    domains = [
        _domain(0, [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)], [UP, UP], ["location:node:0", REF]),
        _domain(1, [(1.0, 0.0, 0.0), (2.0, 0.0, 0.0)], [UP, UP], [REF, "location:node:1"]),
    ]

    result = weld_vertices(domains, 0.0)

    assert result.index == ((0, 1), (1, 2))
    assert len(result.positions) == 3


# --------------------------------------------------------------------------
# Швы складки и конфликт обхода
# --------------------------------------------------------------------------


def test_an_edge_between_two_domains_with_different_uv_is_a_fold_seam():
    faces = [(0, 1, 2), (1, 0, 3)]
    uvs = [(0.0, 0.0), (1.0, 0.0), (0.5, 1.0), (1.0, 0.5), (0.0, 0.5), (0.5, -1.0)]

    assert cross_domain_seams(faces, uvs, [0, 1]) == ((0, 1),)


def test_an_edge_with_equal_uv_on_both_sides_is_continuous_and_not_a_seam():
    faces = [(0, 1, 2), (1, 0, 3)]
    uvs = [(0.0, 0.0), (1.0, 0.0), (0.5, 1.0), (1.0, 0.0), (0.0, 0.0), (0.5, -1.0)]

    assert cross_domain_seams(faces, uvs, [0, 1]) == ()


def test_an_edge_inside_one_domain_is_left_to_its_interface_chains():
    faces = [(0, 1, 2), (1, 0, 3)]
    uvs = [(0.0, 0.0), (1.0, 0.0), (0.5, 1.0), (1.0, 0.5), (0.0, 0.5), (0.5, -1.0)]

    assert cross_domain_seams(faces, uvs, [4, 4]) == ()


def test_a_directed_edge_used_twice_after_the_weld_is_counted():
    assert half_edge_conflicts([(0, 1, 2), (1, 0, 3)]) == 0
    assert half_edge_conflicts([(0, 1, 2), (0, 1, 3)]) == 1


def test_the_weld_module_never_merges_by_distance():
    """Исполняемая форма запрета: сварка по ссылке и побитовому равенству, а не по расстоянию."""

    source = Path(weld.__file__).read_text(encoding="utf-8")
    for forbidden in ("remove_doubles", "merge_distance", "isclose", "hypot", "threshold"):
        assert forbidden not in source, forbidden


# --------------------------------------------------------------------------
# Плоскость граней ПОСЛЕ смещения: запись, а не суд (закон ядра `SOURCE_TRIANGLES_CLIPPED_V1`)
# --------------------------------------------------------------------------


def test_a_planar_face_has_no_off_plane_deviation_after_the_offset():
    from cftuv.envelope_production_weld import off_plane_after_offset

    square = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (0.0, 1.0, 0.0)]
    counters = dict(off_plane_after_offset(square, [(0, 1, 2, 3), (0, 1, 2)]))

    assert counters["ADAPTER_MAX_OFF_PLANE_AFTER_OFFSET_NANOMETRES"] == 0
    assert counters["ADAPTER_FACES_OFF_PLANE_AFTER_OFFSET"] == 0


def test_a_quad_with_one_vertex_lifted_names_the_deviation_in_nanometres():
    from cftuv.envelope_production_weld import off_plane_after_offset

    lifted = 0.004  # 4 мм: вершина одной нормали смещения дальше остальных
    quad = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, lifted), (0.0, 1.0, 0.0)]
    counters = dict(off_plane_after_offset(quad, [(0, 1, 2, 3)]))

    # Плоскость — через первую вершину по вектору площади веера: отклонение того же порядка, что подъём.
    assert 1_000_000 < counters["ADAPTER_MAX_OFF_PLANE_AFTER_OFFSET_NANOMETRES"] <= 4_000_000
    assert counters["ADAPTER_FACES_OFF_PLANE_AFTER_OFFSET"] == 1


def test_triangles_and_degenerate_faces_measure_nothing():
    from cftuv.envelope_production_weld import off_plane_after_offset

    points = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.5), (2.0, 0.0, 1.0), (3.0, 0.0, 1.5)]
    counters = dict(off_plane_after_offset(points, [(0, 1, 2), (0, 1, 2, 3)]))

    assert counters["ADAPTER_MAX_OFF_PLANE_AFTER_OFFSET_NANOMETRES"] == 0
    assert counters["ADAPTER_FACES_OFF_PLANE_AFTER_OFFSET"] == 0


# --------------------------------------------------------------------------
# Шов по цепям батчей: T-стыки между доменами и вершины `clip:` на шовных цепях
# --------------------------------------------------------------------------


def _chain(kind, keys, number=0):
    from types import SimpleNamespace

    return SimpleNamespace(
        semantic_boundary_id=SimpleNamespace(value=f"boundary:{kind}:0:{number}"),
        ordered_vert_keys=tuple(SimpleNamespace(value=key) for key in keys),
    )


def _batch(*chains):
    from types import SimpleNamespace

    return SimpleNamespace(boundary_chains=tuple(chains))


def test_neighbour_domains_with_the_same_vertices_on_a_source_chain_have_no_t_junction():
    from cftuv.envelope_production_weld import seam_report

    left = _batch(_chain("SOURCE", ["src:a", "node:0", "src:b"]))
    right = _batch(_chain("SOURCE", ["src:b", "node:7", "src:a"]))

    assert dict(seam_report([left, right])) == {
        "ADAPTER_SEAM_T_JUNCTIONS": 0,
        "ADAPTER_SEAM_CLIP_VERTICES": 0,
    }


def test_a_source_segment_with_a_vertex_on_one_side_only_is_a_t_junction():
    from cftuv.envelope_production_weld import seam_report

    cut = _batch(_chain("SOURCE", ["src:a", "clip:1", "clip:2", "src:b"]))
    plain = _batch(_chain("SOURCE", ["src:a", "src:b"]))

    counters = dict(seam_report([cut, plain]))
    assert counters["ADAPTER_SEAM_T_JUNCTIONS"] == 1
    assert counters["ADAPTER_SEAM_CLIP_VERTICES"] == 2
    # Лишняя вершина другого вида (не `clip:`) — тот же T-стык, но не вершина резки.
    other = dict(seam_report([_batch(_chain("SOURCE", ["src:a", "node:3", "src:b"])), plain]))
    assert other == {"ADAPTER_SEAM_T_JUNCTIONS": 1, "ADAPTER_SEAM_CLIP_VERTICES": 0}


def test_a_source_vertex_dissolved_on_one_side_only_is_a_t_junction_and_dissolved_on_both_sides_is_not():
    from cftuv.envelope_production_weld import seam_report

    kept = _batch(_chain("SOURCE", ["src:a", "src:m", "src:b"]))
    dissolved = _batch(_chain("SOURCE", ["src:a", "src:b"]))

    assert dict(seam_report([kept, dissolved]))["ADAPTER_SEAM_T_JUNCTIONS"] == 1
    assert dict(seam_report([dissolved, kept]))["ADAPTER_SEAM_T_JUNCTIONS"] == 1  # порядок доменов не важен
    assert dict(seam_report([dissolved, _batch(_chain("SOURCE", ["src:b", "src:a"]))]))["ADAPTER_SEAM_T_JUNCTIONS"] == 0
    # Цепи, у которых пара общая лишь по краю (другой участок цепи соседа), не T-стык: ни одной общей пары нет.
    assert dict(seam_report([kept, _batch(_chain("SOURCE", ["src:b", "src:c"]))]))["ADAPTER_SEAM_T_JUNCTIONS"] == 0


def _open_pairs_by_scanning_every_domain(anchored, held) -> int:
    """Независимый эталон: пара против КАЖДОГО другого домена (квадратично по числу доменов, зато очевидно)."""

    runs: dict = {}
    consecutive: dict = {}
    for number, _chain, anchors in anchored:
        runs.setdefault(number, []).append(anchors)
        consecutive.setdefault(number, set()).update(frozenset(pair) for pair in zip(anchors, anchors[1:]))
    found = set()
    for number in sorted(consecutive):
        for pair in consecutive[number]:
            for other in sorted(consecutive):
                if other != number and pair not in consecutive[other] and any(
                    weld._run_between(anchors, pair, held[number]) for anchors in runs[other]
                ):
                    found.add(pair)
    return len(found)


def test_the_indexed_anchor_count_equals_the_scan_over_every_domain_on_random_seams():
    import random

    generator = random.Random(20261006)
    for _round in range(300):
        keys = [f"src:{index}" for index in range(8)]
        anchored = []
        for number in range(generator.randint(1, 6)):
            for part in range(generator.randint(1, 2)):
                anchored.append((number, f"boundary:SOURCE:{number}:{part}", generator.sample(keys, generator.randint(2, 6))))
        held: dict = {}
        for number, _chain, anchors in anchored:
            held.setdefault(number, set()).update(anchors)
        assert weld._anchors_between(anchored, held) == _open_pairs_by_scanning_every_domain(anchored, held)


def test_the_anchor_count_does_not_scan_every_domain_for_every_pair():
    """Две тысячи доменов по одной цепи: счёт идёт по индексу вершины (пара видит лишь домены с обеими вершинами), а не по всем доменам."""

    import time

    anchored = [(number, f"boundary:SOURCE:{number}:0", [f"src:{number}", f"src:{number + 1}", f"src:{number + 2}"]) for number in range(2000)]
    held: dict = {}
    for number, _chain, anchors in anchored:
        held.setdefault(number, set()).update(anchors)
    started = time.perf_counter()
    count = weld._anchors_between(anchored, held)
    # соседние домены делят пару лишь как соседнюю у обоих (или не делят её): открытых пар нет
    assert count == 0 and time.perf_counter() - started < 1.0


def test_a_segment_owned_by_one_domain_cannot_be_a_t_junction_and_the_front_is_not_a_seam():
    from cftuv.envelope_production_weld import seam_report

    single = _batch(_chain("SOURCE", ["src:a", "clip:1", "src:b"]), _chain("RIM", ["node:0", "clip:2", "node:1"], 1))

    assert dict(seam_report([single])) == {
        "ADAPTER_SEAM_T_JUNCTIONS": 0,
        "ADAPTER_SEAM_CLIP_VERTICES": 1,
    }


def test_a_clip_vertex_on_a_wall_chain_is_counted_as_a_seam_defect():
    from cftuv.envelope_production_weld import seam_report

    wall = _batch(_chain("WALL", ["src:a", "clip:5", "src:b"]))

    assert dict(seam_report([wall]))["ADAPTER_SEAM_CLIP_VERTICES"] == 1


def test_a_batch_without_chains_reports_nothing():
    from types import SimpleNamespace

    from cftuv.envelope_production_weld import seam_report

    assert dict(seam_report([SimpleNamespace()])) == {
        "ADAPTER_SEAM_T_JUNCTIONS": 0,
        "ADAPTER_SEAM_CLIP_VERTICES": 0,
    }


# --------------------------------------------------------------------------
# Порядок цепей батча (`frozenset`, ходит с PYTHONHASHSEED) не меняет шов (walls.003: ложный T-стык у 3 из 6 зёрен)
# --------------------------------------------------------------------------


def _ring_around_a_hexagon():
    """`walls.003`: патч-шестиугольник (домен 2) целиком в кольце (домен 0), цепь кольца обходит его шесть вершин.

    Ребро `(13, 14)` есть только у шестиугольника; кольцо идёт `14 -> 21 -> 20 -> 19 -> 18 -> 13` одной цепью, а шестиугольник
    режет тот же путь на три цепи. Пары пути совпадают, ребра `(13, 14)` у кольца нет вовсе: шва там нет.
    """

    hexagon = [
        _chain("SOURCE", ["src:20", "src:19"], 0),
        _chain("SOURCE", ["src:13", "src:14"], 1),
        _chain("SOURCE", ["src:14", "src:21", "src:20"], 2),
        _chain("SOURCE", ["src:19", "src:18", "src:13"], 3),
    ]
    ring = [
        _chain("SOURCE", ["src:15", "src:14"], 0),
        _chain("SOURCE", ["src:13", "src:12"], 1),
        _chain("SOURCE", ["src:14", "src:21", "src:20", "src:19", "src:18", "src:13"], 4),
    ]
    return hexagon, ring


def test_a_ring_chain_going_around_a_hexagon_patch_is_no_t_junction_in_any_chain_order():
    from itertools import permutations

    from cftuv.envelope_production_weld import seam_report

    hexagon, ring = _ring_around_a_hexagon()
    for hexagon_order in permutations(hexagon):
        for ring_order in permutations(ring):
            inner, outer = _batch(*hexagon_order), _batch(*ring_order)
            assert dict(seam_report([inner, outer]))["ADAPTER_SEAM_T_JUNCTIONS"] == 0
            assert dict(seam_report([outer, inner]))["ADAPTER_SEAM_T_JUNCTIONS"] == 0


def test_an_edge_chain_next_to_a_bypass_chain_of_the_same_neighbour_is_no_t_junction_in_any_order():
    from itertools import permutations

    from cftuv.envelope_production_weld import seam_report

    mine = _batch(_chain("SOURCE", ["src:13", "src:14"]))
    neighbour = [
        _chain("SOURCE", ["src:13", "src:14"], 0),
        _chain("SOURCE", ["src:14", "src:21", "src:20", "src:19", "src:18", "src:13"], 1),
        _chain("SOURCE", ["src:30", "src:31"], 2),
    ]
    for order in permutations(neighbour):
        assert dict(seam_report([mine, _batch(*order)]))["ADAPTER_SEAM_T_JUNCTIONS"] == 0
        assert dict(seam_report([_batch(*order), mine]))["ADAPTER_SEAM_T_JUNCTIONS"] == 0


def test_a_bypass_through_vertices_the_other_domain_does_not_have_stays_a_t_junction_in_any_order():
    from itertools import permutations

    from cftuv.envelope_production_weld import seam_report

    mine = _batch(_chain("SOURCE", ["src:13", "src:14"]))
    neighbour = [
        _chain("SOURCE", ["src:14", "src:21", "src:20", "src:19", "src:18", "src:13"], 1),
        _chain("SOURCE", ["src:30", "src:31"], 2),
    ]
    for order in permutations(neighbour):
        assert dict(seam_report([mine, _batch(*order)]))["ADAPTER_SEAM_T_JUNCTIONS"] == 1
        assert dict(seam_report([_batch(*order), mine]))["ADAPTER_SEAM_T_JUNCTIONS"] == 1


def test_a_vertex_ending_one_chain_and_starting_another_is_judged_on_the_chain_that_has_both_anchors():
    from itertools import permutations

    from cftuv.envelope_production_weld import seam_report

    # `src:a` — конец цепи 0 и начало цепи 1; обе вершины пары лежат на цепи 1, и между ними там стоит `src:m`.
    chains = [_chain("SOURCE", ["src:x", "src:a"], 0), _chain("SOURCE", ["src:a", "src:m", "src:b"], 1)]
    mine = _batch(_chain("SOURCE", ["src:a", "src:b"]))
    for order in permutations(chains):
        assert dict(seam_report([mine, _batch(*order)]))["ADAPTER_SEAM_T_JUNCTIONS"] == 1


#: Зонд для подпроцесса: модуль грузится по пути, цепи лежат в `frozenset` хешируемых записей (их порядок обхода зависит от
#: `PYTHONHASHSEED`, как у `GeometryBatchV1.boundary_chains`). `walls003` — кольцо вокруг шестиугольника (ответ 0); `clean` —
#: шесть мест «ребро рядом с обходом» (0); `genuine` — шесть мест, где у соседа между концами пары стоит вершина, которой у
#: домена пары нет (6). Ответ каждого обязан быть одним и тем же на любом зерне.
_HASH_SEED_PROBE = """
import importlib.util, json, sys
from dataclasses import dataclass

spec = importlib.util.spec_from_file_location("weld_under_test", sys.argv[1])
weld = importlib.util.module_from_spec(spec)
sys.modules["weld_under_test"] = weld
spec.loader.exec_module(weld)


@dataclass(frozen=True)
class Name:
    value: str


@dataclass(frozen=True)
class Chain:
    semantic_boundary_id: Name
    ordered_vert_keys: tuple


@dataclass(frozen=True)
class Batch:
    boundary_chains: frozenset


def chain(number, keys):
    return Chain(Name(f"boundary:SOURCE:0:{number}"), tuple(Name(key) for key in keys))


def walls003():
    hexagon = [[20, 19], [13, 14], [14, 21, 20], [19, 18, 13]]
    ring = [[15, 14], [13, 12], [0, 23, 22, 15], [12, 17, 16, 1], [14, 21, 20, 19, 18, 13], [1, 4, 5, 6, 7, 0]]
    return [
        Batch(frozenset(chain(n, [f"src:{k}" for k in keys]) for n, keys in enumerate(group)))
        for group in (hexagon, ring)
    ]


def places(genuine):
    mine, theirs = [], []
    for index in range(6):
        a, b, m, c = (f"src:{index}{tag}" for tag in "abmc")
        mine.append(chain(index, [a, b]))
        theirs.append(chain(2 * index, [a, c, b] if genuine else [a, b]))
        theirs.append(chain(2 * index + 1, [b, m, a]))
    return [Batch(frozenset(mine)), Batch(frozenset(theirs))]


scenes = {"walls003": walls003(), "clean": places(False), "genuine": places(True)}
print(json.dumps({name: dict(weld.seam_report(batches)) for name, batches in scenes.items()}, sort_keys=True))
"""

_HASH_SEEDS = (0, 1, 2, 3, 4, 5, 6, 7)


def _seam_report_by_hash_seed(module_path) -> dict:
    import json
    import os
    import subprocess
    import sys

    answers = {}
    for seed in _HASH_SEEDS:
        run = subprocess.run(
            [sys.executable, "-c", _HASH_SEED_PROBE, str(module_path)],
            env={**os.environ, "PYTHONHASHSEED": str(seed)},
            capture_output=True,
            text=True,
            timeout=120,
            check=False,
        )
        assert run.returncode == 0, run.stderr
        answers[seed] = json.loads(run.stdout)
    return answers


def test_the_seam_report_is_the_same_under_every_hash_seed():
    answers = _seam_report_by_hash_seed(Path(weld.__file__))

    none = {"ADAPTER_SEAM_T_JUNCTIONS": 0, "ADAPTER_SEAM_CLIP_VERTICES": 0}
    six = {"ADAPTER_SEAM_T_JUNCTIONS": 6, "ADAPTER_SEAM_CLIP_VERTICES": 0}
    assert answers == {seed: {"walls003": none, "clean": none, "genuine": six} for seed in _HASH_SEEDS}
