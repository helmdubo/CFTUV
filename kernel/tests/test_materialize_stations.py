"""Таблица станций `(s, r)`: точная, без sympy, под бюджетом.

Утверждения стоят на числах, посчитанных НЕЗАВИСИМО от проверяемого кода: у
синтетического домена длины рёбер известны из входа (6 и 4 при базисе длины 6),
а не выведены тем же `chain_station_table`, который проверяется.
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass
from fractions import Fraction
from types import SimpleNamespace

import pytest

from cftuv_envelope.exact_sqrt_sum import (
    ExactCanonicalizationWorkBudgetExhausted,
    SqrtSumV1,
    reset_factorization_memory,
)
from cftuv_envelope.materialize import stations
from cftuv_envelope.materialize.coalesce import region_contours, undirected_span
from cftuv_envelope.materialize.stations import (
    chain_station_table,
    length_squared_g,
    source_chain_by_span,
    station_of,
    transverse_of,
    transverse_root,
)

import materialize_factories as factories


def _table(prepared):
    return chain_station_table(prepared, factories.budget())


def test_the_table_names_every_loop_edge_of_the_field_domains():
    for name in factories.FIELD_FIXTURES:
        prepared, _coverage, _request = factories.field_domain(name)
        table = _table(prepared)
        loop_edges = sum(
            len(source.segments)
            for region in prepared.domain.domain_regions
            for source in (region.outer, *region.holes)
        )
        assert len(table.edges) == loop_edges, name
        assert not table.unnamed_chain_ids, name
        assert table.scale == prepared.lattice.scale
        # Каждый узел петли назван исходной вершиной, и двусмысленных нет.
        assert table.node_vertex_ids
        assert all(value is not None for value in table.node_vertex_ids.values())


def test_the_chain_by_span_answer_and_the_table_agree_on_every_edge():
    """`source_chain_by_span` (его читает слияние) и таблица — один ответ."""

    prepared, _coverage, _request = factories.field_domain(
        "building_002_point_contact_v1"
    )
    table = _table(prepared)
    spans = source_chain_by_span(prepared)
    region_id = prepared.regions[0].region_id
    assert len(spans[region_id]) == 12
    for (rid, key), edge in table.edges.items():
        span = undirected_span(key)
        assert spans[rid][span] == edge.chain_id


def test_the_chain_use_direction_flag_follows_the_directed_chain_order():
    """`along_chain_use` — это правда про порядок вершин цепи, а не про угадку."""

    prepared, _coverage, _request = factories.field_domain(
        "building_002_weighted_normals_v1"
    )
    table = _table(prepared)
    context = prepared.context
    uses = {key.value: value for key, value in context.uses_by_id.items()}
    orientations = set()
    for edge in table.edges.values():
        use = uses[edge.chain_use_id]
        orientations.add(use.orientation.value)
        order = [item.value for item in context.directed_chain_vertices(use)]
        pair = (edge.start_vertex_id, edge.end_vertex_id)
        forward = (order[0], order[1])
        assert pair == (forward if edge.along_chain_use else forward[::-1])
    # Фикстура несёт оба направления, значит проверка не вырождена.
    assert {"A_START_TO_END", "B_START_TO_END"} <= orientations


def test_two_collinear_edges_of_one_chain_make_one_run_with_cumulative_station():
    prepared, _coverage, _request = factories.two_edge_chain_domain()
    table = _table(prepared)
    source_runs = [
        run for run in table.runs.values() if run.chain_use_id.startswith("use:")
    ]
    assert len(source_runs) == 1
    run = source_runs[0]
    assert run.physical_edge_ids == ("e0", "e1")
    assert run.s_origin.is_zero

    scale = table.scale
    nodes = {
        edge.start_vertex_id: edge.start
        for edge in table.edges.values()
        if edge.chain_use_id == run.chain_use_id
    }
    nodes["v2"] = next(
        edge.end for edge in table.edges.values() if edge.end_vertex_id == "v2"
    )

    def physical(node):
        point = (SqrtSumV1.rational(node[0]), SqrtSumV1.rational(node[1]))
        return station_of(run, point).scaled(Fraction(1, scale))

    # Базис карты — ребро длины 6, поэтому станция узла = 6 * (узел / scale):
    # v1 -> 6, v2 -> 6 * 218453 / 131072 (привязка к решётке видна числом).
    assert physical(nodes["v0"]) == SqrtSumV1.zero()
    assert physical(nodes["v1"]) == SqrtSumV1.rational(6)
    assert physical(nodes["v2"]) == SqrtSumV1.rational(
        Fraction(6 * 218453, 131072)
    )


def test_the_station_equals_the_projection_computed_from_the_3d_basis():
    """`s` через Грама сверена с проекцией по ТРЁХМЕРНОМУ базису, независимо.

    Дескриптор несёт и Грама, и базис; станция считается через Грама, а здесь
    — через скалярное произведение трёхмерных рациональных векторов. Совпасть
    они обязаны точно: `s^2 * |D|^2 = <V, D>^2`.
    """

    prepared, _coverage, _request = factories.two_edge_chain_domain()
    table = _table(prepared)
    run = next(
        item for item in table.runs.values() if item.chain_use_id.startswith("use:")
    )
    frame = prepared.context.frame
    from cftuv_envelope.planar_metric import fraction_from_exact

    def vector(basis, x, y):
        a = [fraction_from_exact(item) for item in (
            frame.exact_basis_a.x, frame.exact_basis_a.y, frame.exact_basis_a.z
        )]
        b = [fraction_from_exact(item) for item in (
            frame.exact_basis_b.x, frame.exact_basis_b.y, frame.exact_basis_b.z
        )]
        return [a[axis] * x + b[axis] * y for axis in range(3)]

    def dot(left, right):
        return sum((p * q for p, q in zip(left, right)), Fraction(0))

    direction = vector(None, *run.direction)
    for target in ((65536, 40000), (131072, 0), (218453, -7), (-5, 3)):
        offset = (target[0] - run.origin[0], target[1] - run.origin[1])
        projection = dot(vector(None, *offset), direction)
        s = station_of(
            run, (SqrtSumV1.rational(target[0]), SqrtSumV1.rational(target[1]))
        )
        assert s * s == SqrtSumV1.rational(
            projection * projection / dot(direction, direction)
        )


def test_the_run_splits_at_a_bend_and_the_station_accumulates_across_runs():
    """Излом цепи: у соседних рёбер разные системы, а `s0` копится длиной."""

    @dataclass(frozen=True)
    class _Id:
        value: str

    @dataclass(frozen=True)
    class _Edge:
        region_id: str
        key: tuple
        start: tuple
        end: tuple
        start_vertex_id: str
        end_vertex_id: str
        chain_id: str
        chain_use_id: str
        physical_edge_id: str

    class _Chain:
        source_lineage = frozenset({_Id("lineage")})

    class _Use:
        chain_use_id = _Id("use")

    # Ломаная `(0,0) -> (4,0) -> (4,3)` на ортонормальной карте.
    gram = (Fraction(1), Fraction(0), Fraction(1))
    first = _Edge("r", (0, 0, 4, 0), (0, 0), (4, 0), "a", "b", "chain", "use", "e0")
    second = _Edge("r", (4, 0, 4, 3), (4, 0), (4, 3), "b", "c", "chain", "use", "e1")
    runs, records = stations._use_runs(
        _Use(), _Chain(), [(first, True, False), (second, True, False)], gram, factories.budget()
    )

    assert [run.run_id for run in runs] == ["use#run0", "use#run1"]
    assert runs[0].s_origin.is_zero
    assert runs[1].s_origin == SqrtSumV1.rational(4)
    assert [edge.run_id for _key, edge in records] == ["use#run0", "use#run1"]
    # Точка на втором ребре: `s = 4 + 2` для `(4, 2)`.
    point = (SqrtSumV1.rational(4), SqrtSumV1.rational(2))
    assert station_of(runs[1], point) == SqrtSumV1.rational(6)
    # Хода против ChainUse: направление пробега разворачивается, станция идёт
    # от другого конца той же цепи.
    reversed_runs, _ = stations._use_runs(
        _Use(), _Chain(), [(second, False, False), (first, False, False)], gram, factories.budget()
    )
    assert reversed_runs[0].origin == (4, 3)
    assert reversed_runs[1].s_origin == SqrtSumV1.rational(3)


def test_a_chain_edge_outside_the_loop_restarts_the_station_and_says_so():
    prepared, _coverage, _request = factories.two_edge_chain_domain()
    by_use, _corners, _nodes = stations._collect_loops(prepared)
    context = prepared.context
    use_id = next(key for key in by_use if key.startswith("use:"))
    chain_use = next(
        value for key, value in context.uses_by_id.items() if key.value == use_id
    )
    edges = by_use[use_id]
    whole, _reason = stations._order_or_reason(context, chain_use, edges)
    assert [item[2] for item in whole] == [False, False]
    # Первого ребра цепи в петле нет: накопление начинается заново, и флаг
    # рестарта стоит на первом оставшемся ребре.
    cut, _reason = stations._order_or_reason(context, chain_use, edges[1:])
    assert [item[2] for item in cut] == [True]
    # Неоднозначность — другое дело: то же ребро дважды станции не даёт вовсе.
    assert stations._order_or_reason(context, chain_use, edges + edges[:1])[0] is None


def test_transverse_is_zero_on_the_source_and_exactly_alpha_on_the_front():
    for name in factories.FIELD_FIXTURES:
        prepared, coverage, _request = factories.field_domain(name)
        lattice_alpha = SqrtSumV1.rational(coverage.lattice_alpha)
        budget = factories.budget()
        for region in prepared.regions:
            contours = region_contours(region, coverage.lattice_alpha, budget)
            for face, contour in zip(region.partition.faces, contours):
                root = transverse_root(face.line, budget)
                values = [
                    transverse_of(face.line, point, root) for point in contour.points
                ]
                if not face.is_fan_support:
                    # Концы опорного ребра лежат на самой прямой источника.
                    assert values[0].is_zero and values[1].is_zero, name
                # Время прихода внутри грани не отрицательно и не больше alpha.
                assert all(value.sign(budget=budget) >= 0 for value in values), name
                assert all(
                    (lattice_alpha - value).sign(budget=budget) >= 0
                    for value in values
                ), name
                # А на фронте равно alpha ТОЧНО: у УСЕЧЁННОЙ грани хоть одна точка
                # с разностью 0 (грань целиком позади фронта усечения не имеет).
                if contour.points != face.points:
                    assert any(
                        (value - lattice_alpha).is_zero for value in values
                    ), (name, face.owner)


def test_length_squared_uses_the_gram_metric_not_the_euclidean_one():
    gram = (Fraction(4), Fraction(1), Fraction(9))
    assert length_squared_g(gram, (1, 0)) == 4
    assert length_squared_g(gram, (0, 1)) == 9
    assert length_squared_g(gram, (1, 1)) == 4 + 2 * 1 + 9


def test_an_exhausted_budget_surfaces_with_its_own_name():
    """Каждый радикал длины — новый радиканд: бюджет нулевой — отказ, а не счёт."""

    prepared, _coverage, _request = factories.field_domain(
        "building_002_weighted_normals_v1"
    )
    reset_factorization_memory()
    with pytest.raises(ExactCanonicalizationWorkBudgetExhausted):
        chain_station_table(prepared, factories.budget(cap=0))


# --------------------------------------------------------------------------
# Пропуск станции назван причиной (аудит 2026-10-02, предложение 8)
# --------------------------------------------------------------------------


def test_a_skipped_region_is_named_with_its_reason_not_dropped_silently(monkeypatch):
    prepared, _coverage, _request = factories.two_edge_chain_domain()
    region_id = prepared.domain.domain_regions[0].region_id

    skips: list = []
    assert len(list(stations.region_lattice_loops(prepared, skips))) == 1
    assert skips == []

    monkeypatch.setattr(stations, "_region_loops", lambda region: (None, "irrational"))
    skips = []
    assert list(stations.region_lattice_loops(prepared, skips)) == []
    assert skips == [(region_id, stations.SKIP_REGION_LOOPS_UNREADABLE)]
    monkeypatch.undo()

    monkeypatch.setattr(stations, "_lattice_image", lambda loops, lattice: (None, (), None))
    skips = []
    assert list(stations.region_lattice_loops(prepared, skips)) == []
    assert skips == [(region_id, stations.SKIP_REGION_LATTICE_IMAGE_FAILED)]
    monkeypatch.undo()

    monkeypatch.setattr(
        stations, "_lattice_image", lambda loops, lattice: ((), (), Fraction(0))
    )
    skips = []
    assert list(stations.region_lattice_loops(prepared, skips)) == []
    assert skips == [(region_id, stations.SKIP_REGION_NODE_COUNT_MISMATCH)]
    monkeypatch.undo()

    skips = []
    assert list(stations.region_lattice_loops(SimpleNamespace(domain=None), skips)) == []
    assert skips == [("domain", stations.SKIP_DOMAIN_MISSING)]
    # Без `skips` (так зовёт отладочный хост) поведение прежнее: тот же пропуск.
    assert list(stations.region_lattice_loops(SimpleNamespace(domain=None))) == []


def test_a_chain_use_the_table_cannot_order_is_counted_under_its_reason():
    prepared, _coverage, _request = factories.two_edge_chain_domain()
    context = prepared.context
    cases = {
        # `ChainUse` петли нет в снапшоте.
        stations.SKIP_USE_NOT_IN_SNAPSHOT: dataclasses.replace(context, uses_by_id={}),
    }
    for reason, broken in cases.items():
        swapped = SimpleNamespace(
            context=broken, lattice=prepared.lattice, domain=prepared.domain
        )
        table = chain_station_table(swapped, factories.budget())
        counters = dict(table.counters)
        assert counters["STATION_SKIPS"] == len(table.skips) >= 1
        assert counters[f"STATION_SKIP_{reason}"] == len(table.skips)
        assert {item[1] for item in table.skips} == {reason}
        assert table.unnamed_chain_ids and not table.edges
        assert reason in table.skip_text()
    # Здоровый домен: все причины названы и равны нулю.
    healthy = dict(_table(prepared).counters)
    assert healthy["STATION_SKIPS"] == 0
    assert all(
        healthy[f"STATION_SKIP_{reason}"] == 0 for reason in stations.STATION_SKIP_REASONS
    )


def test_the_order_failure_names_which_of_the_two_things_went_wrong():
    prepared, _coverage, _request = factories.two_edge_chain_domain()
    by_use, _corners, _nodes = stations._collect_loops(prepared)
    context = prepared.context
    use_id = next(key for key in by_use if key.startswith("use:"))
    chain_use = next(
        value for key, value in context.uses_by_id.items() if key.value == use_id
    )
    edges = by_use[use_id]
    ordered, reason = stations._order_or_reason(context, chain_use, edges)
    assert ordered is not None and reason == ""
    nameless = [dataclasses.replace(edges[0], start_vertex_id=None), *edges[1:]]
    assert stations._order_or_reason(context, chain_use, nameless) == (
        None,
        stations.SKIP_USE_EDGE_VERTEX_UNNAMED,
    )
    assert stations._order_or_reason(context, chain_use, edges + edges[:1]) == (
        None,
        stations.SKIP_USE_EDGE_PAIR_UNRESOLVED,
    )


def test_a_refusal_for_an_unnamed_chain_carries_the_cause_of_the_skip(monkeypatch):
    """Грань знает только «нет ребра»; причину ей даёт таблица."""

    from cftuv_envelope.ids import PolicyId
    from cftuv_envelope.materialize import domain
    from cftuv_envelope.materialize.admit import MaterializationOutcome

    prepared, coverage, request = factories.field_domain(
        "building_002_weighted_normals_v1"
    )
    original = domain.chain_station_table

    def without_edges(prepared_domain, budget):
        return dataclasses.replace(
            original(prepared_domain, budget),
            edges={},
            skips=(("use:one", stations.SKIP_USE_EDGE_PAIR_UNRESOLVED),),
        )

    monkeypatch.setattr(domain, "chain_station_table", without_edges)
    result = domain.materialize_domain(
        prepared,
        coverage,
        request=dataclasses.replace(request, uv_policy_id=PolicyId("UV_DIRECT_STRIP_V1")),
    )
    assert result.outcome is MaterializationOutcome.STATION_CHAIN_UNNAMED
    assert "no chain-use edge" in result.detail
    assert "station skips: use:one:USE_EDGE_PAIR_UNRESOLVED" in result.detail
    # Числа ранних стадий переживают отказ поздней.
    counters = dict(result.counters)
    assert counters["STATION_SKIPS"] == 0 and "MATERIALIZE_FACES_IN" in counters
