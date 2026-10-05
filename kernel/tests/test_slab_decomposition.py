"""Закон `SLAB_DECOMPOSITION_BY_STATIONS_V1`: кусок с неаффинной UV режется по станциям на трапеции, и каждая — допустимая грань.

Что здесь доказано и чем.

* ГЕОМЕТРИЯ ЗАКОНА (чистые функции `slabs.plan_slabs` и `slabs.verify_plan`, без домена): трапеция сама есть своё разбиение;
  вершина фронта под хордой режет полосу по `s = const` ровно одним разрезом, а его конец получает положение и `(s, r)`
  линейной интерполяцией вдоль ребра (ТОЧНО); равные `s` решаются без допуска (разрез кончается на существующей вершине,
  вертикальная сторона — ребро, вершина на вертикали входит в сторону трапеции); прямая вершина разрезов не даёт, но остаётся
  на гранях; зеркало карты (знак площади) не мешает. Свойство на 300 случайных простых многоугольниках двух семейств:
  разбиение проходит доказательство, суммы площадей в UV и на карте точны, а число граней равно `1 + число разрезов` по
  НЕЗАВИСИМОЙ классификации вершин (начало/конец/разделение/слияние/обычная).
* ДОКАЗАТЕЛЬСТВО ОТКАЗЫВАЕТ: образ с шипом, касанием и перекрытием рёбер — `UV_NOT_SIMPLE`; разбиение с недостающей гранью,
  вывернутой гранью, наложенной гранью или вершиной вне своего ребра — `NOT_A_SUBDIVISION` (красные контроли на испорченных
  планах).
* ШОВ: закон шва (`endpoint_refusal`) и вопрос `may_insert` ДО точных делений: отказанный кусок записывает число граней, которое
  дал бы, и не получает вершин.
* КУСОК В ДОМЕНЕ (`slabs` через `assemble._slab_faces` со `scope`): разрез на свободном ребре фронта — грани и вершина `slab:0`
  в сборнике; на ребре источника, общем ребре — отказ по имени, следов нет.
* ПОЛЕ: домен `sagging_wall` (патч 1, alpha 0.987, Max stretch 42 %, слепок кнопки владельца). Под политикой по умолчанию (шов
  закрыт) закон отказан на всех 13 кусках по шву; ответ не рождает вершин и не меняет ни одного граничного ребра. Контроль
  «шов открыт» (политика владельца) строит 12 разбиений с вершинами `slab:` на цепях источника, и тест границы батча краснеет —
  тест умеет падать.
"""

from __future__ import annotations

import dataclasses
import math
import random
from collections import Counter
from fractions import Fraction
from functools import lru_cache
from pathlib import Path
from types import SimpleNamespace

import pytest

import cftuv_envelope as kernel
from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
from cftuv_envelope.ids import PolicyId
from cftuv_envelope.materialize import assemble, slabs
from cftuv_envelope.materialize.admit import MaterializationOutcome, materialization_request
from cftuv_envelope.materialize.domain import materialize_domain
from cftuv_envelope.materialize.slabs import (
    REASON_NOT_A_SUBDIVISION,
    REASON_SEAM_EDGE,
    REASON_SHARED_EDGE,
    REASON_UV_NOT_SIMPLE,
    EdgeLedgerV1,
    SlabPlanV1,
    SlabPointV1,
    SlabRefusalV1,
    SlabSinkV1,
    SlabVertexV1,
    endpoint_refusal,
    face_notes,
    plan_slabs,
    slab_counters,
    verify_plan,
)
from cftuv_envelope.materialize.tessellate import contour_is_simple
from cftuv_envelope.validation import validate_geometry_batch
from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor
from cftuv_envelope.wavefront.faces import doubled_shoelace, segments_cross

R = SqrtSumV1.rational
FIXTURE = Path(__file__).resolve().parents[1] / "fixtures" / "sagging_wall_slab_stations_v1"
UV = PolicyId("UV_DIRECT_STRIP_V1")


def P(x, y):
    return (R(Fraction(x)), R(Fraction(y)))


def ring(*pairs):
    return tuple(P(*pair) for pair in pairs)


def planned(points, values):
    result = plan_slabs(points, values, None)
    assert isinstance(result, SlabPlanV1), result
    cuts, bad = verify_plan(points, values, result, None)
    assert bad is None, bad
    return result, cuts


def area_in(space, plan):
    """Сумма ПОЛОЖИТЕЛЬНЫХ удвоенных площадей граней по узлам (`space` — координаты всех узлов)."""

    total = SqrtSumV1.zero()
    for face in plan.faces:
        area = doubled_shoelace(tuple(space[node] for node in face))
        total = total + (area if area.sign() > 0 else SqrtSumV1.zero() - area)
    return total


def abs_area(points):
    area = doubled_shoelace(tuple(points))
    return area if area.sign() > 0 else SqrtSumV1.zero() - area


# ---------------------------------------------------------------------------
# 1. Геометрия закона
# ---------------------------------------------------------------------------


def test_a_trapezoid_is_its_own_decomposition():
    values = ring((0, 0), (4, 0), (4, 2), (0, 1))
    plan, cuts = planned(values, values)

    assert len(plan.faces) == 1 and not plan.points and cuts == 0
    assert sorted(plan.faces[0]) == [0, 1, 2, 3]


def test_a_reflex_front_vertex_cuts_the_strip_once_along_constant_s():
    """Пятиугольник с вершиной фронта `(3, 2)` ниже хорды: один разрез вниз, конец — точка `(3, 0)` ребра источника."""

    values = ring((0, 0), (6, 0), (6, 3), (3, 2), (0, 3))
    plan, cuts = planned(values, values)

    assert len(plan.faces) == 2 and cuts == 1 and len(plan.points) == 1
    (new,) = plan.points
    assert new.edge == (0, 1) and new.share == R(Fraction(1, 2))
    assert new.point == P(3, 0) and new.value == P(3, 0)
    assert sorted(sorted(face) for face in plan.faces) == [[0, 3, 4, 5], [1, 2, 3, 5]]
    assert area_in(list(values) + [new.value], plan) == abs_area(values)


def test_the_cut_end_is_the_exact_interpolation_along_the_map_edge_by_the_uv_share():
    """Карта не равна UV: точка берёт долю по UV и кладёт её на ребро КАРТЫ (положение — линейно вдоль ребра)."""

    values = ring((0, 0), (6, 0), (6, 3), (2, 2), (0, 3))
    points = ring((0, 0), (9, 1), (9, 5), (3, 3), (0, 4))
    plan, _cuts = planned(points, values)

    (new,) = plan.points
    share = Fraction(1, 3)
    assert new.share == R(share)
    assert new.value == (R(2), R(0))
    assert new.point == (R(9 * share), R(1 * share))


def test_equal_stations_are_exact_the_cut_ends_on_the_existing_vertex():
    """Вершина фронта и вершина источника с ОДНИМ `s`: разрез кончается на ней, новой вершины нет, допуска нет."""

    values = ring((0, 0), (3, 0), (6, 0), (6, 3), (3, 2), (0, 3))
    plan, cuts = planned(values, values)

    assert not plan.points and cuts == 1 and len(plan.faces) == 2
    assert any({1, 4} <= set(face) for face in plan.faces)


def test_straight_vertices_make_no_cut_but_stay_on_the_faces():
    values = ring((0, 0), (2, 0), (4, 0), (4, 3), (2, 3), (0, 3))
    plan, cuts = planned(values, values)

    assert len(plan.faces) == 1 and cuts == 0
    assert sorted(plan.faces[0]) == [0, 1, 2, 3, 4, 5]


def test_a_vertical_side_is_a_polygon_edge_and_a_vertex_on_it_is_on_the_face():
    values = ring((0, 0), (4, 0), (4, 1), (4, 2), (4, 3), (0, 3))
    plan, cuts = planned(values, values)

    assert len(plan.faces) == 1 and cuts == 0 and len(plan.faces[0]) == 6


def test_a_notch_tip_on_the_chord_becomes_a_vertex_of_the_side_it_touches():
    """Паз справа с остриём `(3, 2)` влево: остриё стоит на хорде левой трапеции и входит в её правую сторону (иначе T-стык
    внутри куска); справа от него две трапеции, и разрезов ровно два — остриё вверх и вниз."""

    values = ring((0, 0), (6, 0), (6, 1), (3, 2), (6, 3), (6, 4), (0, 4))
    plan, cuts = planned(values, values)

    tip = 3
    assert len(plan.faces) == 3 and cuts == 2 and len(plan.points) == 2
    left = max(plan.faces, key=len)
    assert len(left) == 5 and tip in left
    assert sum(tip in face for face in plan.faces) == 3


def test_vertices_on_equal_stations_on_both_sides_are_joined_by_cuts_without_new_vertices():
    """Остриё снизу `(3, 1)` и остриё сверху `(3, 3)`, а также пары `(2, 0)-(2, 4)` и `(4, 0)-(4, 4)` на одних `s`: каждый
    разрез соединяет вершины, допуска и новых вершин нет, граней на единицу больше разрезов."""

    values = ring((0, 0), (2, 0), (3, 1), (4, 0), (6, 0), (6, 4), (4, 4), (3, 3), (2, 4), (0, 4))
    plan, cuts = planned(values, values)

    assert len(plan.faces) == 4 and not plan.points and cuts == 3
    assert any({2, 7} <= set(face) for face in plan.faces)


def test_the_map_may_be_a_mirror_of_the_uv_image():
    values = ring((0, 0), (6, 0), (6, 3), (3, 2), (0, 3))
    mirrored = tuple((point[0], SqrtSumV1.zero() - point[1]) for point in values)
    plan, cuts = planned(mirrored, values)

    assert len(plan.faces) == 2 and cuts == 1
    assert plan.map_sign == -1
    assert area_in(list(mirrored) + [item.point for item in plan.points], plan) == abs_area(mirrored)


def test_a_clockwise_image_is_walked_counter_clockwise():
    values = ring((0, 0), (0, 3), (3, 2), (6, 3), (6, 0))
    plan, cuts = planned(values, values)

    assert len(plan.faces) == 2 and cuts == 1
    assert plan.order == (4, 3, 2, 1, 0)


@pytest.mark.parametrize(
    "pairs, why",
    [
        (((0, 0), (4, 0), (4, 4), (2, 4), (2, 2), (2, 4), (0, 4)), "a spike"),
        (((0, 0), (4, 0), (4, 4), (2, 0), (0, 4)), "an edge through a vertex"),
        (((0, 0), (4, 0), (4, 2), (0, 2), (2, 0), (2, 2)), "a crossing"),
    ],
)
def test_an_image_that_is_not_a_simple_polygon_is_refused_by_name(pairs, why):
    values = ring(*pairs)
    refusal = plan_slabs(values, values, None)

    assert isinstance(refusal, SlabRefusalV1), why
    assert refusal.reason == REASON_UV_NOT_SIMPLE, (why, refusal)


def test_two_vertices_on_one_uv_point_are_refused():
    points = ring((0, 0), (4, 0), (4, 4), (0, 4))
    values = (points[0], points[1], points[2], points[0])

    refusal = plan_slabs(points, values, None)

    assert isinstance(refusal, SlabRefusalV1) and refusal.reason == REASON_UV_NOT_SIMPLE


# ---------------------------------------------------------------------------
# 2. Красные контроли доказательства
# ---------------------------------------------------------------------------


def _good_plan():
    values = ring((0, 0), (6, 0), (6, 3), (3, 2), (0, 3))
    plan, _cuts = planned(values, values)
    return values, plan


def test_the_proof_refuses_a_plan_with_a_missing_face():
    values, plan = _good_plan()
    broken = dataclasses.replace(plan, faces=plan.faces[:1])

    _cuts, bad = verify_plan(values, values, broken, None)

    assert bad is not None and bad.reason == REASON_NOT_A_SUBDIVISION


def test_the_proof_refuses_a_reversed_face():
    values, plan = _good_plan()
    broken = dataclasses.replace(plan, faces=(tuple(reversed(plan.faces[0])), *plan.faces[1:]))

    _cuts, bad = verify_plan(values, values, broken, None)

    assert bad is not None and bad.reason == REASON_NOT_A_SUBDIVISION


def test_the_proof_refuses_a_cut_vertex_that_is_not_on_its_edge():
    values, plan = _good_plan()
    item = plan.points[0]
    moved = SlabPointV1(item.edge, item.share, (item.point[0], item.point[1] + R(1)), item.value)
    broken = dataclasses.replace(plan, points=(moved,))

    _cuts, bad = verify_plan(values, values, broken, None)

    assert bad is not None and bad.reason == REASON_NOT_A_SUBDIVISION


def test_the_proof_refuses_a_face_that_overlaps_another():
    values, plan = _good_plan()
    broken = dataclasses.replace(plan, faces=(*plan.faces, plan.faces[0]))

    _cuts, bad = verify_plan(values, values, broken, None)

    assert bad is not None and bad.reason == REASON_NOT_A_SUBDIVISION


def test_a_fold_of_the_uv_image_over_the_map_is_refused_by_the_proof():
    """Карта той же формы, но вершина фронта вывернута под основание: грань получает ОБРАТНУЮ ориентацию — складка UV."""

    values = ring((0, 0), (6, 0), (6, 3), (3, 2), (0, 3))
    points = ring((0, 0), (6, 0), (6, 3), (3, -1), (0, 3))
    plan = plan_slabs(points, values, None)
    assert isinstance(plan, SlabPlanV1)

    _cuts, bad = verify_plan(points, values, plan, None)

    assert bad is not None and bad.reason == REASON_NOT_A_SUBDIVISION


# ---------------------------------------------------------------------------
# 3. Шов: вопрос до делений, закон шва, запись
# ---------------------------------------------------------------------------


def test_the_seam_question_is_asked_before_any_division_and_the_refusal_keeps_the_face_count():
    values = ring((0, 0), (6, 0), (6, 3), (3, 2), (0, 3))
    asked = []

    def deny(first, second):
        asked.append((first, second))
        return REASON_SEAM_EDGE

    refusal = plan_slabs(values, values, None, deny)

    assert isinstance(refusal, SlabRefusalV1) and refusal.reason == REASON_SEAM_EDGE
    assert refusal.would_have == 2 and asked == [(0, 1)]


def test_the_endpoint_law_closes_shared_edges_always_and_seam_edges_by_policy(monkeypatch):
    assert endpoint_refusal("SHARED") == REASON_SHARED_EDGE
    assert endpoint_refusal("RIM") is None
    assert endpoint_refusal("SOURCE") == REASON_SEAM_EDGE and endpoint_refusal("WALL") == REASON_SEAM_EDGE
    monkeypatch.setattr(slabs, "SEAM_ENDPOINTS_ALLOWED", True)
    assert endpoint_refusal("SOURCE") is None and endpoint_refusal("WALL") is None
    assert endpoint_refusal("SHARED") == REASON_SHARED_EDGE


def test_the_ledger_names_shared_source_rim_and_wall_edges():
    uv = {"a": P(0, 0), "b": P(4, 0), "c": P(4, 5), "d": P(0, 5), "e": P(8, 0), "f": P(8, 2)}
    face = SimpleNamespace()
    first = [("a", uv["a"]), ("b", uv["b"]), ("c", uv["c"]), ("d", uv["d"])]
    second = [("c", uv["c"]), ("b", uv["b"]), ("e", uv["e"]), ("f", uv["f"])]
    ledger = EdgeLedgerV1([(face, first), (face, second)], lambda _face, key: uv[key], Fraction(5))

    assert ledger.kind(face, "b", "c") == "SHARED" and ledger.kind(face, "c", "b") == "SHARED"
    assert ledger.kind(face, "a", "b") == "SOURCE"
    assert ledger.kind(face, "c", "d") == "RIM"
    assert ledger.kind(face, "d", "a") == "WALL"


def test_the_notes_count_thin_stations_and_sharp_faces_without_judging_them():
    values = ring((0, 0), (200, 0), (200, 3), (0, 3))
    wide = SlabPlanV1(faces=((0, 1, 2, 3),), points=(), order=(0, 1, 2, 3), map_sign=1)
    assert face_notes(values, values, wide, Fraction(100), None) == (0, 0)

    thin_values = ring((0, 0), (1, 0), (1, 3), (0, 3))
    thin = SlabPlanV1(faces=((0, 1, 2, 3),), points=(), order=(0, 1, 2, 3), map_sign=1)
    assert face_notes(thin_values, thin_values, thin, Fraction(1000), None) == (1, 0)

    sharp_values = ring((0, 0), (100, 0), (100, 1))
    sharp = SlabPlanV1(faces=((0, 1, 2),), points=(), order=(0, 1, 2), map_sign=1)
    assert face_notes(sharp_values, sharp_values, sharp, Fraction(1), None) == (0, 1)


def test_the_counters_name_only_what_happened():
    tally: Counter = Counter()
    assert slab_counters(tally) == ()
    tally[slabs.PIECES_REFUSED] += 2
    tally[slabs.REFUSED_PREFIX + REASON_SEAM_EDGE] += 2
    tally[slabs.REFUSED_WOULD_HAVE_FACES] += 7
    assert dict(slab_counters(tally)) == {
        slabs.PIECES_REFUSED: 2,
        slabs.REFUSED_PREFIX + REASON_SEAM_EDGE: 2,
        slabs.REFUSED_WOULD_HAVE_FACES: 7,
    }


def test_the_sink_inserts_new_vertices_in_the_direction_of_the_contour():
    cycle = [("a", P(0, 0)), ("b", P(8, 0)), ("c", P(8, 4)), ("d", P(0, 4))]
    one = SlabVertexV1(0, "slab:0", "a", "b", R(Fraction(1, 4)), P(2, 0), P(2, 0))
    two = SlabVertexV1(0, "slab:1", "a", "b", R(Fraction(3, 4)), P(6, 0), P(6, 0))
    forward = SlabSinkV1()
    forward.add(two)
    forward.add(one)
    assert [key for key, _point in forward.refined_cycles([cycle], None)[0]] == ["a", "slab:0", "slab:1", "b", "c", "d"]

    reverse_cycle = [("b", P(8, 0)), ("a", P(0, 0)), ("d", P(0, 4)), ("c", P(8, 4))]
    backward = SlabSinkV1()
    backward.add(one)
    backward.add(two)
    assert [key for key, _point in backward.refined_cycles([reverse_cycle], None)[0]] == ["b", "slab:1", "slab:0", "a", "d", "c"]

    lost = SlabSinkV1()
    lost.add(SlabVertexV1(0, "slab:0", "a", "c", R(Fraction(1, 2)), P(4, 2), P(4, 2)))
    with pytest.raises(ValueError):
        lost.refined_cycles([cycle], None)
    assert backward.facts(lambda index: 7) == {(7, "slab:0"): P(2, 0), (7, "slab:1"): P(6, 0)}
    assert backward.points() == {"slab:0": P(2, 0), "slab:1": P(6, 0)}


# ---------------------------------------------------------------------------
# 4. Кусок в домене: `assemble._slab_faces` со `scope`
# ---------------------------------------------------------------------------

ALPHA = Fraction(4)
#: Кусок на карте — пятиугольник с невыпуклой вершиной `v3`, низ — ребро `v0 -> v1`.
PIECE_POINTS = {"v0": P(0, 0), "v1": P(6, 0), "v2": P(6, 3), "v3": P(3, 2), "v4": P(0, 3)}
#: UV: у `v3` излом (3, 2.5): образ неаффинен; низ на `r = 0` — ребро ИСТОЧНИКА.
UV_SOURCE_BELOW = {**PIECE_POINTS, "v3": (R(3), R(Fraction(5, 2)))}
#: UV: низ на `r = alpha` — свободное ребро ФРОНТА; зеркало карты по `r`.
UV_RIM_BELOW = {
    "v0": (R(0), R(ALPHA)),
    "v1": (R(6), R(ALPHA)),
    "v2": (R(6), R(1)),
    "v3": (R(3), R(Fraction(1, 2))),
    "v4": (R(0), R(1)),
}


def _piece():
    cycle = [(key, PIECE_POINTS[key]) for key in PIECE_POINTS]
    return SimpleNamespace(owner="piece", doubled_area=doubled_shoelace(tuple(point for _key, point in cycle))), cycle


def _scope(uv, partners=()):
    """Область куска: `partners` — циклы соседей (общее ребро), остальное — граница домена."""

    piece, cycle = _piece()
    face = SimpleNamespace()
    pieces = [(face, cycle), *((face, other) for other in partners)]
    sink = SlabSinkV1()
    size = len(cycle)
    scope = assemble._SlabScope(
        sink,
        lambda: EdgeLedgerV1(pieces, lambda _frame, key: uv[key], ALPHA),
        0,
        face,
        frozenset((cycle[at][0], cycle[(at + 1) % size][0]) for at in range(size)),
    )
    return piece, cycle, scope, sink


def _slab(uv, scope, piece, cycle, tally):
    return assemble._slab_faces(piece, cycle, None, False, tally, uv.__getitem__, ALPHA, True, scope)


def test_a_cut_on_a_free_front_edge_emits_the_slabs_and_records_the_new_vertex():
    """Низ куска — свободное ребро фронта (`r = alpha` на обоих концах): разрез законен, вершина `slab:0` рождена."""

    piece, cycle, scope, sink = _scope(UV_RIM_BELOW)
    tally: Counter = Counter()

    polygons = _slab(UV_RIM_BELOW, scope, piece, cycle, tally)

    assert polygons is not None and len(polygons) == 2
    (vertex,) = sink.vertices
    assert vertex.key == "slab:0" and {vertex.first, vertex.second} == {"v0", "v1"} and vertex.point == P(3, 0)
    assert vertex.value == (R(3), R(ALPHA))
    assert all("slab:0" in polygon for polygon in polygons)
    assert tally[slabs.PIECES_DECOMPOSED] == 1 and tally[slabs.FACES_EMITTED] == 2
    assert tally[slabs.VERTICES_INSERTED] == 1 and tally[slabs.CUTS] == 1
    assert tally[assemble.QUADS_UV_BILINEAR] == 2 and not tally[slabs.PIECES_REFUSED]


def test_a_cut_on_the_source_edge_is_refused_by_name_and_leaves_no_trace():
    piece, cycle, scope, sink = _scope(UV_SOURCE_BELOW)
    tally: Counter = Counter()

    polygons = _slab(UV_SOURCE_BELOW, scope, piece, cycle, tally)

    assert polygons is None and not sink
    assert dict(slab_counters(tally)) == {
        slabs.PIECES_REFUSED: 1,
        slabs.REFUSED_PREFIX + REASON_SEAM_EDGE: 1,
        slabs.REFUSED_WOULD_HAVE_FACES: 2,
    }


def test_a_cut_on_an_edge_shared_with_a_neighbour_is_refused_by_name():
    neighbour = [("v1", PIECE_POINTS["v1"]), ("v0", PIECE_POINTS["v0"]), ("x", P(3, -3))]
    piece, cycle, scope, sink = _scope(UV_RIM_BELOW, partners=[neighbour])
    tally: Counter = Counter()

    polygons = _slab(UV_RIM_BELOW, scope, piece, cycle, tally)

    assert polygons is None and not sink and tally[slabs.REFUSED_PREFIX + REASON_SHARED_EDGE] == 1


def test_a_piece_with_an_affine_uv_never_asks_the_law():
    piece, cycle, scope, sink = _scope(PIECE_POINTS)
    tally: Counter = Counter()

    polygons = assemble._contour_polygons(
        piece, cycle, None, False, True, tally, PIECE_POINTS.__getitem__, ALPHA, True, scope
    )

    assert len(polygons) == 1 and len(polygons[0]) == 5 and not sink and not slab_counters(tally)


# ---------------------------------------------------------------------------
# 5. Свойство на случайных простых многоугольниках
# ---------------------------------------------------------------------------


def _star(rng, size, grid):
    while True:
        found = list({(rng.randint(0, grid), rng.randint(0, grid)) for _ in range(size)})
        if len(found) < 4:
            continue
        cx = sum(point[0] for point in found) / len(found)
        cy = sum(point[1] for point in found) / len(found)
        found.sort(key=lambda point: (math.atan2(point[1] - cy, point[0] - cx), (point[0] - cx) ** 2 + (point[1] - cy) ** 2))
        if contour_is_simple(ring(*found), None):
            return found


def _two_opt(rng, size, grid):
    while True:
        found = list({(rng.randint(0, grid), rng.randint(0, grid)) for _ in range(size)})
        if len(found) < 4:
            continue
        rng.shuffle(found)
        for _ in range(200):
            points = ring(*found)
            count = len(found)
            crossing = next(
                (
                    (i, j)
                    for i in range(count)
                    for j in range(i + 2, count)
                    if (j + 1) % count != i
                    and segments_cross(points[i], points[(i + 1) % count], points[j], points[(j + 1) % count], None)
                ),
                None,
            )
            if crossing is None:
                break
            i, j = crossing
            found[i + 1 : j + 1] = reversed(found[i + 1 : j + 1])
        if contour_is_simple(ring(*found), None):
            return found


def _chords(found):
    """Число вертикальных разрезов по НЕЗАВИСИМОЙ классификации вершин (общее положение, прямые вершины не режут)."""

    count = len(found)
    area = sum(found[i][0] * found[(i + 1) % count][1] - found[(i + 1) % count][0] * found[i][1] for i in range(count))
    walk = found[::-1] if area < 0 else found
    chords = 0
    for index in range(count):
        before, here, after = walk[index - 1], walk[index], walk[(index + 1) % count]
        cross = (here[0] - before[0]) * (after[1] - here[1]) - (here[1] - before[1]) * (after[0] - here[0])
        if cross == 0:
            continue
        if (before < here) == (after < here):
            chords += 0 if cross > 0 else 2
        else:
            chords += 1
    return chords


def _has_touch_or_spike(found):
    """Независимо от закона: вершина лежит на чужом ребре либо шип (разворот на 180 градусов)."""

    count = len(found)
    for index in range(count):
        before, here, after = found[index - 1], found[index], found[(index + 1) % count]
        cross = (here[0] - before[0]) * (after[1] - here[1]) - (here[1] - before[1]) * (after[0] - here[0])
        dot = (here[0] - before[0]) * (after[0] - here[0]) + (here[1] - before[1]) * (after[1] - here[1])
        if cross == 0 and dot < 0:
            return True
        for other in range(count):
            if index in (other, (other + 1) % count):
                continue
            a, b = found[other], found[(other + 1) % count]
            side = (b[0] - a[0]) * (here[1] - a[1]) - (b[1] - a[1]) * (here[0] - a[0])
            inside = min(a[0], b[0]) <= here[0] <= max(a[0], b[0]) and min(a[1], b[1]) <= here[1] <= max(a[1], b[1])
            if side == 0 and inside:
                return True
    return False


@pytest.mark.parametrize("seed", range(300))
def test_random_simple_polygons_decompose_into_a_valid_subdivision_with_exact_areas(seed):
    rng = random.Random(seed)
    size = rng.randint(4, 9)
    grid = rng.choice([6, 10, 40, 400])
    found = _two_opt(rng, size, grid) if seed % 2 else _star(rng, size, grid)
    values = ring(*found)
    mirror = rng.choice([1, -1])
    points = tuple((R(Fraction(point[0]) + Fraction(point[1], 3)), R(mirror * point[1])) for point in found)

    plan = plan_slabs(points, values, None)

    if isinstance(plan, SlabRefusalV1):
        # Отказ допустим ТОЛЬКО на вырожденном образе (касание, шип), и тест знает это независимо от закона.
        assert plan.reason == REASON_UV_NOT_SIMPLE and _has_touch_or_spike(found), (found, plan)
        return
    _cuts, bad = verify_plan(points, values, plan, None)
    assert bad is None, (found, bad)
    uv_space = list(values) + [item.value for item in plan.points]
    map_space = list(points) + [item.point for item in plan.points]
    assert area_in(uv_space, plan) == abs_area(values)
    assert area_in(map_space, plan) == abs_area(points)
    stations = {value[0] for value in values}
    assert all(item.value[0] in stations for item in plan.points)
    if len({point[0] for point in found}) == len(found):
        assert len(plan.faces) == 1 + _chords(found), found


# ---------------------------------------------------------------------------
# 6. Поле: `sagging_wall`, патч 1, alpha 0.987, Max stretch 42 %
# ---------------------------------------------------------------------------


@lru_cache(maxsize=None)
def _field_domain():
    snapshot = kernel.AnalysisSnapshotCodecV1.loads((FIXTURE / "analysis_snapshot.json").read_bytes())
    request = kernel.DecalRequestCodecV1.loads((FIXTURE / "decal_request.json").read_bytes())
    prepared = prepare_conveyor(snapshot, request)
    assert prepared.outcome.value == "EXACT", prepared.detail
    coverage = conveyor_coverage(prepared, request.requested_alpha.value)
    assert coverage.outcome.value == "EXACT", coverage.detail
    return prepared, coverage


def _materialize():
    prepared, coverage = _field_domain()
    result = materialize_domain(
        prepared,
        coverage,
        request=materialization_request(prepared, uv_policy_id=UV),
        near_planar_lift_law=NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1,
        decal_topology_law=DecalTopologyLawV1.PLANAR_POLYGONS_V1,
    )
    assert result.outcome is MaterializationOutcome.MATERIALIZED, result.detail
    return result, dict(result.counters)


def _law_off(monkeypatch):
    """Домен так, как его строил `f362871`: ни закона станций, ни разбиения диагоналями."""

    monkeypatch.setattr(assemble, "_slab_faces", lambda *args, **kwargs: None)
    monkeypatch.setattr(assemble, "_convex_faces", lambda *args, **kwargs: None)


def _open_edges(batch):
    """Граница декали: рёбра с ровно одной гранью, по ПОЛОЖЕНИЯМ концов (ключи `clip:` зависят от порядка выпуска)."""

    where = {vertex.vert_key.value: str(vertex.position) for vertex in batch.vertices}
    seen = Counter()
    for face in batch.faces:
        keys = [key.value for key in face.ordered_vert_keys]
        for position, key in enumerate(keys):
            seen[frozenset((where[key], where[keys[(position + 1) % len(keys)]]))] += 1
    return {edge for edge, times in seen.items() if times == 1}


def _without_clip(batch):
    return {vertex.vert_key.value for vertex in batch.vertices if not vertex.vert_key.value.startswith("clip:")}


def test_the_field_domain_under_the_default_seam_policy_refuses_every_cut_and_keeps_the_boundary(monkeypatch):
    with monkeypatch.context() as patch:
        _law_off(patch)
        without, base = _materialize()
    law, counters = _materialize()

    assert len(without.batch.faces) == 151 and not any(name.startswith("MATERIALIZE_SLAB") for name in base)
    assert counters[slabs.PIECES_REFUSED] == 13 and counters[slabs.REFUSED_PREFIX + REASON_SEAM_EDGE] == 13
    assert counters[slabs.REFUSED_WOULD_HAVE_FACES] == 38
    assert not counters.get(slabs.PIECES_DECOMPOSED) and not counters.get(slabs.VERTICES_INSERTED)
    # Закон шва держит ГРАНИЦУ: ни одного нового ребра границы, ни одной новой вершины, кроме вершин резки.
    assert _without_clip(law.batch) == _without_clip(without.batch)
    assert _open_edges(law.batch) == _open_edges(without.batch)
    assert not validate_geometry_batch(law.batch)


def test_with_the_seam_open_the_law_decomposes_the_pieces_but_the_boundary_changes_and_the_test_notices(monkeypatch):
    """КОНТРОЛЬ: политика «шов открыт» (решение владельца, не умолчание). Закон срабатывает, вершины `slab:` встают на цепи
    источника, и граница батча уже не та — проверка границы из соседнего теста на этом ответе краснеет."""

    with monkeypatch.context() as patch:
        _law_off(patch)
        without, _base = _materialize()
    monkeypatch.setattr(slabs, "SEAM_ENDPOINTS_ALLOWED", True)
    law, counters = _materialize()

    assert counters[slabs.PIECES_DECOMPOSED] == 12 and counters[slabs.FACES_EMITTED] == 35
    assert counters[slabs.VERTICES_INSERTED] == 23 and counters[slabs.CUTS] == 23
    assert counters[slabs.REFUSED_PREFIX + REASON_SHARED_EDGE] == 1
    slab_keys = {vertex.vert_key.value for vertex in law.batch.vertices if vertex.vert_key.value.startswith("slab:")}
    assert len(slab_keys) == 23
    on_source = {
        key.value
        for chain in law.batch.boundary_chains
        if chain.semantic_boundary_id.value.startswith("boundary:SOURCE")
        for key in chain.ordered_vert_keys
    }
    assert slab_keys <= on_source
    assert _open_edges(law.batch) != _open_edges(without.batch)
    assert not validate_geometry_batch(law.batch)
