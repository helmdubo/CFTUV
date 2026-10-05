"""Закон `CONVEX_PARTITION_BY_DIAGONALS_V1`: кусок, которому не хватило закона станций, режется на наименьшее число допустимых граней диагоналями между его вершинами.

Что здесь доказано и чем.

* НАИМЕНЬШЕЕ ЧИСЛО ЧАСТЕЙ — по НЕЗАВИСИМОМУ перебору: 150 случайных простых многоугольников из 4-8 вершин, число частей
  закона равно минимуму по ВСЕМ наборам непересекающихся диагоналей (диагональ допустима по другому предикату — середина
  внутри многоугольника, нет пересечений и вершин на ней), в которых каждая часть выпукла.
* ДОКАЗАТЕЛЬСТВО: красные контроли на испорченных планах (недостающая часть, вывернутая часть, лишний разрез).
* ГРАНИЦА РАБОТЫ И ОТКАЗЫ НАЗВАНЫ: куски длиннее `MAX_VERTICES`; нет ни одной допустимой диагонали; разбиение не короче ушей
  (`NO_GAIN_OVER_EARS`: уши прежние побитово); складка UV (`UV_FOLD`).
* КУСОК В ДОМЕНЕ (`assemble._convex_faces`): диагонали только между вершинами куска — вершин не рождается, части допускаются
  прежним законом (билинейный выпуклый многоугольник потока).
* ПОЛЕ: `sagging_wall` (alpha 0.987, Max stretch 42 %): патч 1 — 13 кусков, 49 ушей превращены в 27 частей, 14 диагоналей,
  в домене 151 -> 88 граней; патч 0 — один кусок разбит, второй отказан складкой UV и остаётся на ушах побитово.
"""

from __future__ import annotations

import dataclasses
import itertools
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
from cftuv_envelope.materialize import assemble, convex_partition
from cftuv_envelope.materialize.admit import MaterializationOutcome, materialization_request
from cftuv_envelope.materialize.convex_partition import (
    REASON_NO_PARTITION,
    REASON_NOT_A_SUBDIVISION,
    REASON_TOO_MANY_VERTICES,
    PartitionPlanV1,
    PartitionRefusalV1,
    partition_counters,
    plan_partition,
    verify_partition,
)
from cftuv_envelope.materialize.domain import materialize_domain
from cftuv_envelope.materialize.tessellate import contour_is_simple, convex_polygon_ring
from cftuv_envelope.validation import validate_geometry_batch
from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor
from cftuv_envelope.wavefront.faces import doubled_shoelace, segments_cross

R = SqrtSumV1.rational
FIXTURES = Path(__file__).resolve().parents[1] / "fixtures"
UV = PolicyId("UV_DIRECT_STRIP_V1")


def P(x, y):
    return (R(Fraction(x)), R(Fraction(y)))


def ring(*pairs):
    return tuple(P(*pair) for pair in pairs)


def convex_only(points):
    return lambda verts: convex_polygon_ring(tuple(points[index] for index in verts), None) is not None


# ---------------------------------------------------------------------------
# 1. Разбиение и независимый перебор
# ---------------------------------------------------------------------------


def test_a_convex_polygon_is_one_part():
    points = ring((0, 0), (4, 0), (5, 2), (2, 4), (-1, 2))

    plan = plan_partition(points, None, convex_only(points))

    assert isinstance(plan, PartitionPlanV1) and len(plan.pieces) == 1
    assert verify_partition(points, plan, None) is None


def test_a_pentagon_with_one_reflex_vertex_is_two_parts_cut_by_one_diagonal():
    points = ring((0, 0), (6, 0), (6, 3), (3, 2), (0, 3))

    plan = plan_partition(points, None, convex_only(points))

    assert isinstance(plan, PartitionPlanV1) and len(plan.pieces) == 2
    assert verify_partition(points, plan, None) is None
    assert sorted(len(piece) for piece in plan.pieces) == [3, 4]


def test_the_partition_is_deterministic_and_counter_clockwise_whatever_the_input_walk():
    points = ring((0, 0), (6, 0), (6, 3), (3, 2), (0, 3))
    reversed_points = tuple(reversed(points))

    first = plan_partition(points, None, convex_only(points))
    second = plan_partition(reversed_points, None, convex_only(reversed_points))

    assert plan_partition(points, None, convex_only(points)) == first
    assert len(first.pieces) == len(second.pieces) == 2
    assert verify_partition(reversed_points, second, None) is None


def _valid_diagonal_by_midpoint(found, first, second):
    """Диагональ по ДРУГОМУ предикату, чем в законе: без пересечений, без вершин на ней, середина внутри многоугольника."""

    count = len(found)
    (ax, ay), (bx, by) = found[first], found[second]
    for index in range(count):
        c, d = found[index], found[(index + 1) % count]
        if index in (first, second) or (index + 1) % count in (first, second):
            continue
        if segments_cross(P(ax, ay), P(bx, by), P(*c), P(*d), None):
            return False
    for index in range(count):
        if index in (first, second):
            continue
        px, py = found[index]
        if (bx - ax) * (py - ay) - (by - ay) * (px - ax) == 0 and min(ax, bx) <= px <= max(ax, bx) and min(ay, by) <= py <= max(ay, by):
            return False
    mx, my = Fraction(ax + bx, 2), Fraction(ay + by, 2)
    inside = False
    for index in range(count):
        (x1, y1), (x2, y2) = found[index], found[(index + 1) % count]
        if (y1 > my) != (y2 > my) and mx < x1 + Fraction((my - y1) * (x2 - x1), (y2 - y1)):
            inside = not inside
    return inside


def _minimum_convex_parts(found):
    """Наименьшее число выпуклых частей по ВСЕМ наборам непересекающихся диагоналей (перебор, n <= 8)."""

    count = len(found)
    diagonals = [
        (i, j)
        for i in range(count)
        for j in range(i + 2, count)
        if not (i == 0 and j == count - 1) and _valid_diagonal_by_midpoint(found, i, j)
    ]

    def crosses(one, other):
        (a, b), (c, d) = one, other
        if len({a, b, c, d}) < 4:
            return False
        return (a < c < b) != (a < d < b)

    def convex_parts(chosen):
        parts = [list(range(count))]
        for first, second in chosen:
            for position, part in enumerate(parts):
                if first in part and second in part:
                    i, j = part.index(first), part.index(second)
                    low, high = min(i, j), max(i, j)
                    parts[position : position + 1] = [part[low : high + 1], part[high:] + part[: low + 1]]
                    break
        return parts

    best = [10**9]

    def walk(start, chosen):
        parts = convex_parts(chosen)
        if all(
            len(part) == 3 or convex_polygon_ring(tuple(P(*found[index]) for index in part), None) is not None for part in parts
        ):
            best[0] = min(best[0], len(parts))
        for number in range(start, len(diagonals)):
            if all(not crosses(diagonals[number], other) for other in chosen):
                walk(number + 1, chosen + [diagonals[number]])

    walk(0, [])
    return best[0]


def _touches_or_spikes(found):
    """Независимо от закона: вершина лежит на чужом ребре либо шип (разворот на 180 градусов) — многоугольник вырожден."""

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
            if side == 0 and min(a[0], b[0]) <= here[0] <= max(a[0], b[0]) and min(a[1], b[1]) <= here[1] <= max(a[1], b[1]):
                return True
    return False


def _random_polygon(rng, size, grid):
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
        if contour_is_simple(ring(*found), None) and not _touches_or_spikes(found):
            return found


@pytest.mark.parametrize("seed", range(150))
def test_the_partition_has_the_minimum_number_of_convex_parts_by_independent_search(seed):
    rng = random.Random(seed)
    found = _random_polygon(rng, rng.randint(4, 8), rng.choice([8, 12, 60]))
    points = ring(*found)

    plan = plan_partition(points, None, convex_only(points))

    assert isinstance(plan, PartitionPlanV1), (found, plan)
    assert verify_partition(points, plan, None) is None, found
    assert len(plan.pieces) == _minimum_convex_parts(found), found
    total = SqrtSumV1.zero()
    for piece in plan.pieces:
        total = total + doubled_shoelace(tuple(points[index] for index in piece))
    reference = doubled_shoelace(points)
    assert (total - (reference if reference.sign() > 0 else SqrtSumV1.zero() - reference)).is_zero


# ---------------------------------------------------------------------------
# 2. Красные контроли доказательства и названные отказы
# ---------------------------------------------------------------------------


def _good_plan():
    points = ring((0, 0), (6, 0), (6, 3), (3, 2), (0, 3))
    return points, plan_partition(points, None, convex_only(points))


def test_the_proof_refuses_a_plan_with_a_missing_part():
    points, plan = _good_plan()

    bad = verify_partition(points, dataclasses.replace(plan, pieces=plan.pieces[:1]), None)

    assert isinstance(bad, PartitionRefusalV1) and bad.reason == REASON_NOT_A_SUBDIVISION


def test_the_proof_refuses_a_reversed_part():
    points, plan = _good_plan()
    broken = dataclasses.replace(plan, pieces=(tuple(reversed(plan.pieces[0])), *plan.pieces[1:]))

    bad = verify_partition(points, broken, None)

    assert isinstance(bad, PartitionRefusalV1) and bad.reason == REASON_NOT_A_SUBDIVISION


def test_the_proof_refuses_an_extra_cut_that_is_not_in_the_contour():
    points, plan = _good_plan()
    quad = next(piece for piece in plan.pieces if len(piece) == 4)
    split = (quad[:3], (quad[2], quad[3], quad[0]))
    broken = dataclasses.replace(plan, pieces=(*(piece for piece in plan.pieces if piece is not quad), *split))

    # Уши четырёхугольника — настоящее разбиение (доказательство его принимает), а часть, вывернутая внутрь, — нет.
    assert verify_partition(points, broken, None) is None
    bad = verify_partition(points, dataclasses.replace(broken, pieces=(*broken.pieces, quad)), None)
    assert isinstance(bad, PartitionRefusalV1) and bad.reason == REASON_NOT_A_SUBDIVISION


def test_a_piece_longer_than_the_work_boundary_is_refused_by_name(monkeypatch):
    points = ring((0, 0), (6, 0), (6, 3), (3, 2), (0, 3))
    monkeypatch.setattr(convex_partition, "MAX_VERTICES", 4)

    refusal = plan_partition(points, None, convex_only(points))

    assert isinstance(refusal, PartitionRefusalV1) and refusal.reason == REASON_TOO_MANY_VERTICES


def test_no_valid_diagonal_means_no_partition_by_name(monkeypatch):
    points = ring((0, 0), (6, 0), (6, 3), (3, 2), (0, 3))
    monkeypatch.setattr(convex_partition, "_valid_diagonal", lambda *args: False)

    refusal = plan_partition(points, None, convex_only(points))

    assert isinstance(refusal, PartitionRefusalV1) and refusal.reason == REASON_NO_PARTITION


def test_the_counters_name_only_what_happened():
    tally: Counter = Counter()
    assert partition_counters(tally) == ()
    tally[convex_partition.PIECES_PARTITIONED] += 3
    tally[convex_partition.FACES_EMITTED] += 7
    tally[convex_partition.DIAGONALS] += 4
    assert dict(partition_counters(tally)) == {
        convex_partition.PIECES_PARTITIONED: 3,
        convex_partition.FACES_EMITTED: 7,
        convex_partition.DIAGONALS: 4,
    }


# ---------------------------------------------------------------------------
# 3. Кусок в домене: `assemble._convex_faces`
# ---------------------------------------------------------------------------

ALPHA = Fraction(4)
PIECE_POINTS = {"v0": P(0, 0), "v1": P(6, 0), "v2": P(6, 3), "v3": P(3, 2), "v4": P(0, 3)}
#: Неаффинная UV: у `v3` излом `(3, 2.5)`; низ на `r = 0`.
UV_KINK = {**PIECE_POINTS, "v3": (R(3), R(Fraction(5, 2)))}
#: Складка: `v4` уведена под основание, образ треугольника `v3 v4 v0` развёрнут, а образ четырёхугольника `v0 v1 v2 v3` нет.
UV_FOLD = {**PIECE_POINTS, "v3": (R(3), R(Fraction(5, 2))), "v4": (R(0), R(-3))}


def _piece():
    cycle = [(key, PIECE_POINTS[key]) for key in PIECE_POINTS]
    return SimpleNamespace(owner="piece", doubled_area=doubled_shoelace(tuple(point for _key, point in cycle))), cycle


def _convex(uv, in_flow=True):
    piece, cycle = _piece()
    tally: Counter = Counter()
    polygons = assemble._convex_faces(piece, cycle, None, False, tally, uv.__getitem__, ALPHA, in_flow)
    return polygons, tally


def test_a_flow_piece_is_cut_by_one_diagonal_into_admitted_faces_without_a_new_vertex():
    polygons, tally = _convex(UV_KINK)

    assert polygons is not None and len(polygons) == 2
    assert {key for polygon in polygons for key in polygon} == set(PIECE_POINTS)
    assert tally[convex_partition.PIECES_PARTITIONED] == 1 and tally[convex_partition.FACES_EMITTED] == 2
    assert tally[convex_partition.DIAGONALS] == 1 and not tally[convex_partition.PIECES_REFUSED]
    assert tally[assemble.QUADS_UV_BILINEAR] == 1


def test_a_piece_outside_a_flow_has_only_triangles_so_there_is_no_gain_and_the_ears_stay_bit_for_bit():
    piece, cycle = _piece()
    ears_tally: Counter = Counter()
    ears = assemble._contour_polygons(piece, cycle, None, False, True, ears_tally, UV_KINK.__getitem__, ALPHA, False, None)
    tally: Counter = Counter()

    law = assemble._contour_polygons(
        piece,
        cycle,
        None,
        False,
        True,
        tally,
        UV_KINK.__getitem__,
        ALPHA,
        False,
        assemble._SlabScope(
            assemble.SlabSinkV1(),
            lambda: assemble.EdgeLedgerV1([(SimpleNamespace(), cycle)], lambda _f, key: UV_KINK[key], ALPHA),
            0,
            SimpleNamespace(),
            frozenset(),
        ),
    )

    assert law == ears and all(len(polygon) == 3 for polygon in law)
    assert tally[convex_partition.REFUSED_PREFIX + convex_partition.REASON_NO_GAIN] == 1


def test_a_folded_uv_is_refused_by_name_and_the_piece_stays_on_the_ears_bit_for_bit():
    piece, cycle = _piece()
    ears = assemble._contour_polygons(piece, cycle, None, False, True, Counter(), UV_FOLD.__getitem__, ALPHA, True, None)

    polygons, convex_tally = _convex(UV_FOLD)

    assert polygons is None
    assert convex_tally[convex_partition.REFUSED_PREFIX + convex_partition.REASON_UV_FOLD] == 1
    assert all(len(polygon) == 3 for polygon in ears) and len(ears) == 3


# ---------------------------------------------------------------------------
# 4. Поле: `sagging_wall`, alpha 0.987, Max stretch 42 %
# ---------------------------------------------------------------------------


def _materialize(prepared, coverage):
    result = materialize_domain(
        prepared,
        coverage,
        request=materialization_request(prepared, uv_policy_id=UV),
        near_planar_lift_law=NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1,
        decal_topology_law=DecalTopologyLawV1.PLANAR_POLYGONS_V1,
    )
    assert result.outcome is MaterializationOutcome.MATERIALIZED, result.detail
    return result, dict(result.counters)


@lru_cache(maxsize=None)
def _patch_one():
    folder = FIXTURES / "sagging_wall_slab_stations_v1"
    snapshot = kernel.AnalysisSnapshotCodecV1.loads((folder / "analysis_snapshot.json").read_bytes())
    request = kernel.DecalRequestCodecV1.loads((folder / "decal_request.json").read_bytes())
    prepared = prepare_conveyor(snapshot, request)
    return prepared, conveyor_coverage(prepared, request.requested_alpha.value)


@lru_cache(maxsize=None)
def _patch_zero():
    folder = FIXTURES / "sagging_wall_rung_chord_v1"
    snapshot = kernel.AnalysisSnapshotCodecV1.loads((folder / "analysis_snapshot.json").read_bytes())
    text = (folder / "decal_request_alpha_0.6.json").read_text(encoding="utf-8").replace('"value":"0.6"', '"value":"0.987"')
    request = kernel.DecalRequestCodecV1.loads(text.encode("utf-8"))
    prepared = prepare_conveyor(snapshot, request)
    return prepared, conveyor_coverage(prepared, request.requested_alpha.value)


def _law_off(monkeypatch):
    monkeypatch.setattr(assemble, "_slab_faces", lambda *args, **kwargs: None)
    monkeypatch.setattr(assemble, "_convex_faces", lambda *args, **kwargs: None)


def test_the_field_patch_one_turns_49_ears_into_27_parts_and_151_faces_into_88(monkeypatch):
    with monkeypatch.context() as patch:
        _law_off(patch)
        without, base = _materialize(*_patch_one())
    law, counters = _materialize(*_patch_one())

    assert base[assemble.POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE] == 13 and len(without.batch.faces) == 151
    assert counters[convex_partition.PIECES_PARTITIONED] == 13
    assert counters[convex_partition.FACES_EMITTED] == 27 and counters[convex_partition.DIAGONALS] == 14
    assert not counters.get(convex_partition.PIECES_REFUSED)
    assert not counters.get(assemble.POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE)
    assert len(law.batch.faces) == 88
    assert not validate_geometry_batch(law.batch)
    assert {vertex.vert_key.value for vertex in law.batch.vertices if not vertex.vert_key.value.startswith("clip:")} == {
        vertex.vert_key.value for vertex in without.batch.vertices if not vertex.vert_key.value.startswith("clip:")
    }


def test_the_field_patch_zero_splits_one_piece_and_leaves_the_folded_one_on_the_ears(monkeypatch):
    with monkeypatch.context() as patch:
        _law_off(patch)
        without, base = _materialize(*_patch_zero())
    law, counters = _materialize(*_patch_zero())

    assert base[assemble.POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE] == 2
    assert counters[convex_partition.PIECES_PARTITIONED] == 1 and counters[convex_partition.FACES_EMITTED] == 2
    assert counters[convex_partition.REFUSED_PREFIX + convex_partition.REASON_UV_FOLD] == 1
    assert counters[assemble.POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE] == 1
    assert counters[assemble.SLAB_REFUSED_PREFIX + "UV_NOT_SIMPLE"] == 1
    assert len(law.batch.faces) < len(without.batch.faces)
    assert not validate_geometry_batch(law.batch)
