"""План станций цепей (`CHAIN_STATION_PLAN_V1`): решение «несёт ли декаль внутреннюю вершину цепи» по сырому снапшоту, один раз.

Решение принимает компиляция, пересчитывает проверяющий плана; домены общей цепи читают одну запись (швов с вершиной с одной стороны
не бывает по построению). Вход закона — поверхность вокруг вершины на ОБЕИХ сторонах цепи: своя из `surface_ir`, чужая из
`seam_neighbour_faces` (факт хоста). Лента из перекошенных четырёхгранников вдоль цепи по оси `x`: высота внешнего края каждой стороны
задаёт склад поверхности, а глубину граней друг от друга проверяет независимая формула в самом тесте (расстояние точки до плоскости
Ньюэлла грани), а не проверяемый код.

| что проверяется                                                                     | тест |
|-------------------------------------------------------------------------------------|------|
| ровная полоса с обеих сторон — все внутренние вершины `FREE`, пары рёбер названы       | `..._a_flat_wall_frees_...` |
| складка свыше допуска хорды — `REQUIRED:FOLD`, под допуском — `FREE`; допуск реестровый | `..._a_fold_over_the_budget_...`, `..._the_budget_is_...` |
| каждое ребро в допуске, а лента дрейфует — `GROUP_NOT_FLAT` у вершин группы            | `..._a_drifting_strip_...` |
| малые изломы копятся (пологая дуга): окно по линии держит каждый пропуск в допуске хорды   | `..._small_bends_that_drift_together_...` |
| окно идёт через стык: цепь, разрезанная на куски, решается как целая, направление не важно  | `..._the_window_runs_across_a_joint_...`, `..._does_not_depend_...` |
| чужая сторона шва не выгружена — `NEIGHBOUR_SIDE_UNKNOWN`; выгружена — решение то же    | `..._the_other_side_...` |
| два домена общей цепи читают ОДНО решение (стороны меняются местами)                   | `..._both_domains_...` |
| решение не зависит от других цепей и выбора: закон читает один снапшот                 | `..._selection_...` |
| замкнутая цепь, нет положений, другой шов в вершине, грань без площади, угол — названы   | `..._named_...` |
| проверяющий: подделка, пропажа, лишняя запись, чужой допуск — именованный отказ          | `..._validator_...` |
| грани соседа проверяются; кодек читает их, а без них байты снапшота прежние              | `..._neighbour_faces_...`, `..._codec_...` |
| настоящий домен: компиляция несёт план, пересчёт его принимает, подделка отказывает       | `..._compilation_...` |
"""

from __future__ import annotations

import dataclasses
import math
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace

import pytest

import cftuv_envelope as kernel
from cftuv_envelope import _chain_station as law
from cftuv_envelope.contracts.analysis import PhysicalChainKind, PhysicalChainV1, SeamNeighbourFaceV1, SourceVertexV1
from cftuv_envelope.contracts.chain_station import ChainStationDispositionV1, ChainStationReasonV1
from cftuv_envelope.contracts.surface import SourceFaceV1, SurfacePayloadMode
from cftuv_envelope.ids import (
    LineageId,
    PatchDomainId,
    PatchId,
    PhysicalChainId,
    PhysicalEdgeId,
    SourceFaceId,
    SourceVertexId,
)
from cftuv_envelope.materialize.clip_cells import CLIP_DIAGONAL_CHORD_BUDGET
from cftuv_envelope.numeric import LocalPoint3V1, LocalVector3V1
from cftuv_envelope.validation_chain_station import (
    seam_neighbour_face_issues,
    validate_plan_chain_stations,
    validate_plan_chain_stations_against_snapshot,
)
from cftuv_envelope.validation_issues import ValidationCode

FREE = ChainStationDispositionV1.FREE
REQUIRED = ChainStationDispositionV1.REQUIRED
R = ChainStationReasonV1
A, B = PatchId("patch:A"), PatchId("patch:B")
DOMAIN_A, DOMAIN_B = PatchDomainId("domain:A"), PatchDomainId("domain:B")
CHAIN = PhysicalChainId("chain:C")
ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "fixtures"
BUDGET = float(CLIP_DIAGONAL_CHORD_BUDGET)
LENGTH = 0.4


def _name(value) -> SourceVertexId:
    return SourceVertexId(value)


def _face(name, patch, names) -> SourceFaceV1:
    return SourceFaceV1(
        SourceFaceId(name),
        patch,
        tuple(_name(item) for item in names),
        tuple(PhysicalEdgeId(f"{name}:e{index}") for index in range(len(names))),
        LocalVector3V1(0.0, 0.0, 1.0),
        (),
    )


@dataclasses.dataclass(frozen=True)
class Use:
    patch_domain_id: object
    physical_chain_id: object


@dataclasses.dataclass(frozen=True)
class Patch:
    patch_id: object


@dataclasses.dataclass(frozen=True)
class Relation:
    source_vertex_id: object


class World:
    """Две ленты по обе стороны цепи `c0 .. cN` по оси `x` (`y = 0`, `z = 0`): сторона `A` при `y = -1`, сторона `B` при `y = +1`.

    `heights_*[i]` — высота внешней вершины `i` (`a<i>`, `b<i>`): цепь прямая, а внешний край идёт по заданному профилю, и четырёхгранник
    между двумя соседними внешними вершинами перекошен ровно настолько, насколько различаются высоты.
    """

    def __init__(self, count, heights_a=None, heights_b=None, kind=PhysicalChainKind.PHYSICAL_SEAM, length=LENGTH, split=(), bends=None):
        self.count = count
        heights_a = heights_a or [0.0] * (count + 1)
        heights_b = heights_b or [0.0] * (count + 1)
        bends = bends or {}
        self.position = {}
        for index in range(count + 1):
            shift = bends.get(index, (0.0, 0.0))
            dx, dy, dz = shift if len(shift) == 3 else (0.0, *shift)
            self.position[f"c{index}"] = (length * index + dx, dy, dz)
            self.position[f"a{index}"] = (length * index, -1.0, heights_a[index])
            self.position[f"b{index}"] = (length * index, 1.0, heights_b[index])
        self.faces_a = [_face(f"fa{i}", A, (f"c{i}", f"c{i + 1}", f"a{i + 1}", f"a{i}")) for i in range(count)]
        self.faces_b = [_face(f"fb{i}", B, (f"c{i}", f"b{i}", f"b{i + 1}", f"c{i + 1}")) for i in range(count)]
        cuts = (0, *split, count)
        self.chains = []
        for number, (start, end) in enumerate(zip(cuts, cuts[1:])):
            self.chains.append(
                PhysicalChainV1(
                    CHAIN if not split else PhysicalChainId(f"chain:C{number}"),
                    kind,
                    False,
                    tuple(_name(f"c{index}") for index in range(start, end + 1)),
                    tuple(PhysicalEdgeId(f"chain:e{index}") for index in range(start, end)),
                    frozenset({LineageId("lineage:C")}),
                    frozenset({LineageId("lineage:C")}),
                )
            )
        self.chain = self.chains[0]

    def _neighbour(self, faces):
        return frozenset(
            SeamNeighbourFaceV1(
                item.face_id, item.patch_id, item.vertex_cycle, tuple(LocalPoint3V1(*self.position[v.value]) for v in item.vertex_cycle)
            )
            for item in faces
        )

    def snapshot(self, domain=DOMAIN_A, own="A", neighbour=True, **changes):
        """Снапшот домена: `own` — стороны в `surface_ir` (`A`, `B` либо `AB`), грани другой стороны — соседские, если `neighbour`."""

        mine = {"A": self.faces_a, "B": self.faces_b, "AB": self.faces_a + self.faces_b}[own]
        other = [] if own == "AB" or not neighbour else (self.faces_b if own == "A" else self.faces_a)
        names = {v.value for item in mine for v in item.vertex_cycle}
        base = dict(
            surface_ir=SimpleNamespace(payload_mode=SurfacePayloadMode.FULL_HOST_SURFACE, source_faces=frozenset(mine)),
            source_vertices=frozenset(SourceVertexV1(_name(n), LocalPoint3V1(*self.position[n])) for n in names),
            seam_neighbour_faces=self._neighbour(other),
            corner_relations=frozenset(),
            junction_relations=frozenset(),
            terminal_relations=frozenset(),
            chain_uses=frozenset(Use(domain, item.physical_chain_id) for item in self.chains),
            physical_chains=frozenset(self.chains),
            surface_metric_descriptors=frozenset(),
            patches=frozenset({Patch(A if own != "B" else B)}),
        )
        base.update(changes)
        return SimpleNamespace(**base)


def _stations(snapshot, domain=DOMAIN_A):
    (plan,) = law.chain_station_plans(snapshot, domain)
    return plan.stations


def _plans(snapshot, domain=DOMAIN_A):
    return {item.physical_chain_id.value: item for item in law.chain_station_plans(snapshot, domain)}


def _verdicts(snapshot, domain=DOMAIN_A):
    return [(item.disposition, item.reason) for item in _stations(snapshot, domain)]


def _depth(points, cycle):
    """Независимая мера: наибольшее расстояние `points` от плоскости Ньюэлла грани с обходом `cycle` (центроид, нормаль), метры."""

    origin = cycle[0]
    normal = [0.0, 0.0, 0.0]
    for index in range(1, len(cycle) - 1):
        u = [cycle[index][k] - origin[k] for k in range(3)]
        v = [cycle[index + 1][k] - origin[k] for k in range(3)]
        cross = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
        normal = [normal[k] + cross[k] for k in range(3)]
    length = math.sqrt(sum(item * item for item in normal))
    centroid = [sum(p[k] for p in cycle) / len(cycle) for k in range(3)]
    return max(abs(sum(normal[k] * (p[k] - centroid[k]) for k in range(3))) / length for p in points)


def _pair_depth(world, first, second):
    """Наибольшая из двух глубин граней друг от друга, метры (независимая формула)."""

    a = [world.position[v.value] for v in first.vertex_cycle]
    b = [world.position[v.value] for v in second.vertex_cycle]
    return max(_depth(b, a), _depth(a, b))


def test_a_flat_wall_frees_every_interior_vertex_of_the_chain_and_names_the_inert_edges():
    world = World(4)
    stations = _stations(world.snapshot())

    assert [item.source_vertex_id.value for item in stations] == ["c1", "c2", "c3"]
    assert [item.ordinal for item in stations] == [1, 2, 3]
    assert {(item.disposition, item.reason) for item in stations} == {(FREE, R.TRANSVERSE_EDGES_INERT)}
    # у `c1` поперечные рёбра — `c1 - a1` (грани `fa0`, `fa1`) и `c1 - b1` (грани `fb0`, `fb1`)
    assert stations[0].inert_face_pairs == (
        (SourceFaceId("fa0"), SourceFaceId("fa1")),
        (SourceFaceId("fb0"), SourceFaceId("fb1")),
    )
    plans = law.chain_station_plans(world.snapshot(), DOMAIN_A)
    assert law.free_vertices(plans) == {"c1", "c2", "c3"}
    assert law.inert_face_pairs(plans) == frozenset(
        frozenset((f"f{side}{index}", f"f{side}{index + 1}")) for side in "ab" for index in range(3)
    )


def test_a_fold_over_the_budget_requires_the_vertex_and_under_it_the_vertex_is_free():
    over = World(2, heights_b=[0.0, 0.0, 4 * BUDGET])
    under = World(2, heights_b=[0.0, 0.0, 0.4 * BUDGET])

    # независимо: глубина `fb1` от плоскости `fb0` (и наоборот) над допуском у первой ленты и под ним у второй
    assert _pair_depth(over, *over.faces_b) > BUDGET > _pair_depth(under, *under.faces_b)
    assert _verdicts(over.snapshot()) == [(REQUIRED, R.FOLD)]
    assert _stations(over.snapshot())[0].inert_face_pairs == ()
    assert _verdicts(under.snapshot()) == [(FREE, R.TRANSVERSE_EDGES_INERT)]


def test_the_budget_is_the_registered_chord_depth_and_the_plan_records_it():
    assert law.flatness_budget() is CLIP_DIAGONAL_CHORD_BUDGET and CLIP_DIAGONAL_CHORD_BUDGET == Fraction(1, 200)
    assert law.budget_record() == kernel.ExactRatioV1(1, 200)
    (plan,) = law.chain_station_plans(World(2).snapshot(), DOMAIN_A)
    assert plan.flatness_budget == law.budget_record() and plan.law is law.LAW


def test_a_drifting_strip_whose_every_edge_is_inert_is_not_flat_together():
    count = 8
    drift = [0.0003 * index * index for index in range(count + 1)]
    world = World(count, heights_b=drift)

    # независимо: соседние грани каждая в допуске, а первая и последняя глубже допуска друг от друга
    assert max(_pair_depth(world, world.faces_b[i], world.faces_b[i + 1]) for i in range(count - 1)) < BUDGET
    assert _pair_depth(world, world.faces_b[0], world.faces_b[-1]) > BUDGET
    stations = _stations(world.snapshot())

    assert {item.reason for item in stations} == {R.GROUP_NOT_FLAT}
    assert all(item.disposition is REQUIRED and not item.inert_face_pairs for item in stations)


def test_the_other_side_of_a_seam_is_named_unknown_without_it_and_the_decision_is_the_same_with_it():
    world = World(3)
    with_neighbour = law.chain_station_plans(world.snapshot(own="A"), DOMAIN_A)
    complete = law.chain_station_plans(world.snapshot(own="AB"), DOMAIN_A)
    unknown = world.snapshot(own="A", neighbour=False)

    assert {(item.disposition, item.reason) for item in _stations(unknown)} == {(REQUIRED, R.NEIGHBOUR_SIDE_UNKNOWN)}
    assert with_neighbour == complete and with_neighbour != law.chain_station_plans(unknown, DOMAIN_A)


def test_a_border_chain_needs_no_other_side():
    world = World(3, kind=PhysicalChainKind.PHYSICAL_DECAL_SOURCE)

    assert {(item.disposition, item.reason) for item in _stations(world.snapshot(own="A", neighbour=False))} == {(FREE, R.TRANSVERSE_EDGES_INERT)}


@pytest.mark.parametrize("tall", (0.0, 4 * BUDGET))
def test_both_domains_of_a_shared_chain_read_one_decision(tall):
    """Домен `A` видит `B` через грани соседа, домен `B` — `A` через них же: решение побитово одно; складка с любой стороны его меняет у обоих."""

    for heights in ({"heights_a": [0.0, 0.0, tall, 0.0]}, {"heights_b": [0.0, 0.0, tall, 0.0]}):
        world = World(3, **heights)
        seen_by_a = law.chain_station_plans(world.snapshot(DOMAIN_A, own="A"), DOMAIN_A)
        seen_by_b = law.chain_station_plans(world.snapshot(DOMAIN_B, own="B"), DOMAIN_B)

        assert seen_by_a == seen_by_b
        assert (law.free_vertices(seen_by_a) == set()) == bool(tall)


def test_the_plan_reads_the_snapshot_only_so_no_selection_and_no_other_chain_can_move_it():
    world = World(4)
    snapshot = world.snapshot()
    other = Use(DOMAIN_B, PhysicalChainId("chain:other"))
    changed = world.snapshot(chain_uses=snapshot.chain_uses | {other})

    assert law.chain_station_plans(changed, DOMAIN_A) == law.chain_station_plans(snapshot, DOMAIN_A)
    assert law.chain_station_plans.__code__.co_argcount == 3 and "request" not in law.chain_station_plans.__code__.co_varnames


def test_named_reasons_closed_chain_no_positions_other_seam_a_face_without_area_and_a_relation_vertex():
    world = World(3)
    closed = world.snapshot(physical_chains=frozenset({dataclasses.replace(world.chain, is_closed=True)}))
    assert {item.reason for item in _stations(closed)} == {R.CLOSED_CHAIN}

    plain = world.snapshot()
    free = SimpleNamespace(
        **{**vars(plain), "surface_ir": SimpleNamespace(payload_mode=SurfacePayloadMode.EC0_COORDINATE_FREE_FIXTURE_V5, source_faces=plain.surface_ir.source_faces)}
    )
    assert {item.reason for item in _stations(free)} == {R.POSITIONS_UNAVAILABLE}

    swapped = {dataclasses.replace(item, patch_id=B) if item.face_id.value == "fa1" else item for item in plain.surface_ir.source_faces}
    other_seam = world.snapshot(surface_ir=SimpleNamespace(payload_mode=SurfacePayloadMode.FULL_HOST_SURFACE, source_faces=frozenset(swapped)))
    assert _stations(other_seam)[0].reason is R.JUNCTION

    squashed = world.snapshot(
        source_vertices=frozenset(
            dataclasses.replace(item, position=LocalPoint3V1(item.position.x, 0.0, 0.0)) if item.vertex_id.value in {"a1", "a2"} else item
            for item in plain.source_vertices
        )
    )
    assert _stations(squashed)[0].reason is R.NOT_MANIFOLD

    related = world.snapshot(corner_relations=frozenset({Relation(_name("c1"))}))
    assert _stations(related)[0].reason is R.JUNCTION and _stations(related)[1].disposition is FREE
    # конец куска цепи — вершина двух терминальных отношений (по одному на кусок), и это не угол: стык решает закон
    terminals = world.snapshot(terminal_relations=frozenset({Relation(_name("c1"))}))
    assert _stations(terminals)[0].disposition is FREE


def test_a_joint_of_two_pieces_of_one_chain_is_planned_like_an_interior_vertex_and_recorded_in_both_pieces():
    world = World(4, split=(2,))
    plans = _plans(world.snapshot())
    first, second = plans["chain:C0"], plans["chain:C1"]

    # куски `c0 .. c2` и `c2 .. c4`: стык `c2` — конец обоих, внутренние вершины `c1` и `c3`
    assert [(item.source_vertex_id.value, item.ordinal, item.kind.value) for item in first.stations] == [
        ("c1", 1, "INTERIOR_VERTEX"),
        ("c2", 2, "CHAIN_JOINT"),
    ]
    assert [(item.source_vertex_id.value, item.ordinal, item.kind.value) for item in second.stations] == [
        ("c2", 0, "CHAIN_JOINT"),
        ("c3", 1, "INTERIOR_VERTEX"),
    ]
    assert {(item.disposition, item.reason) for plan in plans.values() for item in plan.stations} == {(FREE, R.TRANSVERSE_EDGES_INERT)}
    # один стык — одна запись у обоих кусков
    assert first.stations[1].inert_face_pairs == second.stations[0].inert_face_pairs
    assert law.free_vertices(plans.values()) == {"c1", "c2", "c3"}
    # без стыка: цепь, где вершина — конец одного куска и больше ничего, стыка не образует
    assert [item.source_vertex_id.value for item in _stations(World(4).snapshot())] == ["c1", "c2", "c3"]


def test_a_joint_bends_within_the_artist_error_or_it_is_a_corner_of_the_chain_and_has_no_record():
    limit = float(law.BEND_LIMIT)
    inside = math.degrees(0.5 * limit)
    beyond = math.degrees(2.0 * limit)

    def joint(height, length=LENGTH, more=None):
        # стык `c2` смещён вдоль нормали: излом между плечами равен 2 * atan(height / length)
        plan = _plans(World(4, split=(2,), length=length, bends={2: (0.0, height), **(more or {})}).snapshot())["chain:C0"]
        return [item for item in plan.stations if item.kind.value == "CHAIN_JOINT"]

    (found,) = joint(LENGTH * math.tan(math.radians(inside) / 2.0))
    assert (found.disposition, found.reason) == (FREE, R.TRANSVERSE_EDGES_INERT)
    # излом за допуском — настоящий угол цепи: записи нет, вершина несётся как всегда
    assert joint(LENGTH * math.tan(math.radians(beyond) / 2.0)) == []
    # допуск излома — запись реестра `CANONICAL_RESTORATION_ARTIST_ERROR`, а не число закона
    from cftuv_envelope._authoring_intent import CANONICAL_RESTORATION_ARTIST_ERROR

    assert law.BEND_LIMIT is CANONICAL_RESTORATION_ARTIST_ERROR
    # длинные рёбра: излом в допуске, а вершина стоит от хорды дальше 5 мм — тоже угол
    assert joint(20.0 * math.tan(math.radians(inside) / 2.0), length=20.0) == []
    # цепь заворачивает назад (плечи против хода) — угол
    assert joint(0.0, more={3: (-0.8, 0.0, 0.0)}) == []
    # то же правило у внутренней вершины куска: она точно на прямой, и закон это проверяет, а не принимает на веру
    off = _plans(World(4, bends={2: (0.0, 0.05)}).snapshot())["chain:C"].stations
    assert {item.source_vertex_id.value for item in off if item.reason is R.BEND_BEYOND_STRAIGHT} == {"c1", "c2", "c3"}


def test_a_joint_with_a_third_edge_of_another_seam_is_required_in_the_domain_that_sees_it_and_has_no_record_where_it_is_a_corner_of_three_chains():
    world = World(4, split=(2,))
    snapshot = world.snapshot()
    third = dataclasses.replace(world.chains[0], physical_chain_id=PhysicalChainId("chain:third"), ordered_source_vertex_ids=(_name("c2"), _name("a2")))
    crowded = world.snapshot(
        physical_chains=snapshot.physical_chains | {third},
        chain_uses=snapshot.chain_uses | {Use(DOMAIN_A, third.physical_chain_id)},
    )
    plans = _plans(crowded)

    assert [item.source_vertex_id.value for item in plans["chain:C0"].stations] == ["c1"]
    assert [item.source_vertex_id.value for item in plans["chain:C1"].stations] == ["c3"]


def _arc(count, split=()):
    """Лента, где цепь идёт по пологой параболе `y = k i^2`: излом в вершине ~1e-3 рад (в допуске), хорда по соседям 0.2 мм, а целиком дуга уходит далеко."""

    sag = 2e-4
    return World(count, split=split, bends={index: (sag * index * index, 0.0) for index in range(count + 1)})


def _distance_to_line(world, names, name):
    """Независимая мера: расстояние (метры) вершины `name` от прямой между `names[0]` и `names[-1]`, binary64."""

    a, b, p = (world.position[item] for item in (names[0], names[-1], name))
    w = [b[k] - a[k] for k in range(3)]
    v = [p[k] - a[k] for k in range(3)]
    cross = (v[1] * w[2] - v[2] * w[1], v[2] * w[0] - v[0] * w[2], v[0] * w[1] - v[1] * w[0])
    return math.sqrt(sum(item * item for item in cross)) / math.sqrt(sum(item * item for item in w))


def test_small_bends_that_drift_together_cannot_all_be_free_and_the_window_keeps_every_run_inside_the_budget():
    world = _arc(24)
    stations = _stations(world.snapshot())
    by_name = {item.source_vertex_id.value: item for item in stations}

    # каждая вершина по соседям проходит (излом и хорда в допуске), но вся дуга от первой до последней глубже допуска
    assert len(stations) == 23
    names = [f"c{index}" for index in range(25)]
    assert max(_distance_to_line(world, names, name) for name in names) > BUDGET
    carried = [name for name in names[1:-1] if by_name[name].disposition is REQUIRED]
    assert carried and all(by_name[name].reason is R.RUN_CHORD_BEYOND_BUDGET for name in carried)
    assert all(not by_name[name].inert_face_pairs for name in carried)
    # независимо: между двумя несомыми (и концами линии) каждая пропущенная вершина отстоит от прямой, их соединяющей, не дальше допуска
    anchors = ["c0", *carried, "c24"]
    for left, right in zip(anchors, anchors[1:]):
        inside = names[names.index(left) : names.index(right) + 1]
        assert all(_distance_to_line(world, inside, name) <= BUDGET for name in inside)
    # окно жадное: несомая вершина — первая, которую окно не вместило бы (с ней вместе пропущенные уже глубже допуска)
    for name in carried:
        at = names.index(name)
        start = max(index for index in range(at) if names[index] == "c0" or names[index] in carried)
        assert any(_distance_to_line(world, names[start : at + 2], item) > BUDGET for item in names[start + 1 : at + 1])
    # без окна все вершины были бы свободны: тест различает закон и его отсутствие
    free_without_window = [item for item in stations if item.disposition is FREE]
    assert len(free_without_window) < 23


def test_the_window_runs_across_a_joint_so_a_chain_cut_into_pieces_decides_like_the_whole_chain():
    whole = {item.source_vertex_id.value: (item.disposition, item.reason) for item in _stations(_arc(24).snapshot())}
    pieces = _plans(_arc(24, split=(12,)).snapshot())
    joined = {
        item.source_vertex_id.value: (item.disposition, item.reason) for plan in pieces.values() for item in plan.stations
    }

    assert whole == joined and set(whole) == {f"c{index}" for index in range(1, 24)}
    assert {value[1] for value in whole.values()} >= {R.RUN_CHORD_BEYOND_BUDGET, R.TRANSVERSE_EDGES_INERT}
    # стык решён один раз и записан в обоих кусках одинаково
    (first, second) = (pieces["chain:C0"], pieces["chain:C1"])
    assert first.stations[-1].source_vertex_id == second.stations[0].source_vertex_id == SourceVertexId("c12")
    assert (first.stations[-1].disposition, first.stations[-1].reason) == (second.stations[0].disposition, second.stations[0].reason)


def test_the_window_does_not_depend_on_which_end_of_a_piece_the_snapshot_lists_first():
    """Куски цепи в снапшоте могут идти в любом направлении и порядке: решение то же."""

    base = _arc(24, split=(12,))
    flipped = _arc(24, split=(12,))
    flipped.chains = [
        dataclasses.replace(item, ordered_source_vertex_ids=tuple(reversed(item.ordered_source_vertex_ids)))
        if index == 1
        else item
        for index, item in enumerate(flipped.chains)
    ]
    expected = {item.source_vertex_id.value: item.disposition for plan in _plans(base.snapshot()).values() for item in plan.stations}
    # кусок `C1` записан от `c24` к `c12`: у линии те же вершины, окно идёт по той же линии
    got = {
        item.source_vertex_id.value: item.disposition
        for plan in _plans(flipped.snapshot(physical_chains=frozenset(flipped.chains))).values()
        for item in plan.stations
    }
    assert got == expected


def test_the_free_reasons_are_exactly_the_two_that_free():
    assert set(law._FREE_REASONS) == {R.TRANSVERSE_EDGES_INERT, R.FREE_UNDER_CHART}


def test_the_validator_accepts_the_honest_plan_and_names_every_forgery():
    world = World(4)
    snapshot = world.snapshot()
    plans = law.chain_station_plans(snapshot, DOMAIN_A)
    (plan,) = plans

    def against(candidate, snap=snapshot):
        found: list = []
        validate_plan_chain_stations_against_snapshot(found, SimpleNamespace(chain_station_plans=candidate), snap, DOMAIN_A, ("plans", "p"))
        return [item.message for item in found if item.code is ValidationCode.CHAIN_STATION_PLAN]

    def structural(candidate):
        found: list = []
        validate_plan_chain_stations(found, SimpleNamespace(chain_station_plans=candidate))
        return [item.message for item in found if item.code is ValidationCode.CHAIN_STATION_PLAN]

    assert against(plans) == [] and structural(plans) == []
    forged_station = dataclasses.replace(plan.stations[0], disposition=REQUIRED, reason=R.FOLD)
    forged = frozenset({dataclasses.replace(plan, stations=(forged_station, *plan.stations[1:]))})
    assert any("differs from the raw snapshot" in message for message in against(forged))
    wrong_reason = dataclasses.replace(plan.stations[0], reason=R.FOLD)
    assert structural(frozenset({dataclasses.replace(plan, stations=(wrong_reason, *plan.stations[1:]))}))
    assert structural(frozenset({dataclasses.replace(plan, flatness_budget=kernel.ExactRatioV1(1, 100))}))
    ghost = dataclasses.replace(plan, physical_chain_id=PhysicalChainId("chain:ghost"))
    assert any("no stations" in message for message in against(frozenset({plan, ghost})))
    two = dataclasses.replace(world.chain, physical_chain_id=PhysicalChainId("chain:two"))
    bigger = world.snapshot(
        chain_uses=snapshot.chain_uses | {Use(DOMAIN_A, two.physical_chain_id)},
        physical_chains=snapshot.physical_chains | {two},
    )
    assert any("lacks" in message for message in against(plans, bigger))
    assert against(frozenset()) == []


def test_neighbour_faces_of_the_snapshot_are_validated():
    world = World(2)
    snapshot = world.snapshot(own="A")
    face = next(iter(snapshot.seam_neighbour_faces))
    clash = world.snapshot(
        surface_ir=SimpleNamespace(
            payload_mode=SurfacePayloadMode.FULL_HOST_SURFACE,
            source_faces=snapshot.surface_ir.source_faces | {_face(face.face_id.value, B, ("c0", "c1", "b1"))},
        )
    )

    assert seam_neighbour_face_issues(snapshot) == ()
    assert any(item.code is ValidationCode.DUPLICATE_ID for item in seam_neighbour_face_issues(clash))
    inside = world.snapshot(patches=frozenset({Patch(B)}))
    assert any(item.code is ValidationCode.CROSS_CONTRACT_MISMATCH for item in seam_neighbour_face_issues(inside))


def test_the_codec_reads_neighbour_faces_and_a_snapshot_without_them_keeps_its_bytes():
    raw = (FIXTURES / "mesh2_patch0_cut_fans_v1" / "analysis_snapshot.json").read_bytes()
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(raw)

    assert snapshot.seam_neighbour_faces == frozenset()
    assert b"seam_neighbour_faces" not in kernel.AnalysisSnapshotCodecV1.dumps(snapshot)

    face = SeamNeighbourFaceV1(
        SourceFaceId("neighbour:1"),
        PatchId("patch:other"),
        (SourceVertexId("n0"), SourceVertexId("n1"), SourceVertexId("n2")),
        (LocalPoint3V1(0.0, 0.0, 0.0), LocalPoint3V1(1.0, 0.0, 0.0), LocalPoint3V1(0.0, 1.0, 0.0)),
    )
    carried = dataclasses.replace(snapshot, seam_neighbour_faces=frozenset({face}))
    wire = kernel.AnalysisSnapshotCodecV1.dumps(carried)

    assert b"seam_neighbour_faces" in wire and kernel.AnalysisSnapshotCodecV1.loads(wire) == carried
    with pytest.raises(ValueError):
        SeamNeighbourFaceV1(SourceFaceId("bad"), PatchId("p"), (SourceVertexId("a"),), (LocalPoint3V1(0.0, 0.0, 0.0),))


def test_a_compilation_carries_the_plan_and_the_recount_accepts_it_and_refuses_a_forged_one():
    from cftuv_envelope.reference.contracts import ReferenceOutcome
    from cftuv_envelope.wavefront import prepare_conveyor

    root = FIXTURES / "mesh2_patch0_cut_fans_v1"
    snapshot = kernel.AnalysisSnapshotCodecV1.loads((root / "analysis_snapshot.json").read_bytes())
    request_file = next(path for path in sorted(root.glob("decal_request*.json")))
    request = kernel.DecalRequestCodecV1.loads(request_file.read_bytes())
    compilation = prepare_conveyor(snapshot, request).compilation
    domain = compilation.plan_key.patch_domain_id
    expected = law.chain_station_plans(snapshot, domain)

    assert compilation.chain_station_plans == expected and expected
    assert law.plan_errors(snapshot, domain, compilation.chain_station_plans) == ()
    plan = min(expected, key=lambda item: item.physical_chain_id.value)
    wrong = dataclasses.replace(plan.stations[0], disposition=REQUIRED, reason=R.FOLD)
    forged = frozenset({dataclasses.replace(plan, stations=(wrong, *plan.stations[1:])), *(item for item in expected if item is not plan)})
    assert law.plan_errors(snapshot, domain, forged)
    assert ReferenceOutcome.CHAIN_STATION_PLAN_INVALID.value == "CHAIN_STATION_PLAN_INVALID"
