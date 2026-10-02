"""Закон `SOURCE_VERTEX_STATIONED_ON_CHORD_V1`: внутренняя вершина прямой цепи стоит на хорде там, где её источник.

Малый слой — синтетика с НЕДВОИЧНОЙ станцией (`projection_k_gram` из привязки подменяется
значением `k + дробь`: настоящий источник кладёт вершину на узел, дробь появляется только на
крупной хорде поля): точка точно на хорде (векторное произведение нуль), `s` — точная проекция
Грама, площади слитых граней и частей замыкаются точно, имя `src:` сохранено, счётчики дают
`TOTAL`. Большой слой — настоящий домен `building` (патч 17, d2): был `DISPLACED_BY_LATTICE`, стало
нуль, и позиции не зависят от закона топологии.
"""

from __future__ import annotations

import dataclasses
import json
from fractions import Fraction
from functools import lru_cache
from pathlib import Path
from types import SimpleNamespace

import pytest

import cftuv_envelope as kernel
from cftuv_envelope import ExactRationalV1
from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1, GeometryDiagnosticSeverity
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
from cftuv_envelope.ids import PolicyId, SourceVertexId
from cftuv_envelope.materialize import assemble, chord_station, domain
from cftuv_envelope.materialize.admit import MaterializationOutcome
from cftuv_envelope.materialize.chord_station import (
    AT_NODE,
    FACES_RESTATIONED,
    NOT_IN_COVERAGE,
    PLACED,
    SKIPPED_NODE_SHARED,
    SKIPPED_NOT_MONOTONE,
    TOTAL,
    ChordStationsV1,
)
from cftuv_envelope.materialize.coalesce import lattice_node, point_key
from cftuv_envelope.materialize.domain import materialize_domain
from cftuv_envelope.materialize.frames import MaterializationRefusal
from cftuv_envelope.materialize.source_lift import DISPLACED, LIFTED
from cftuv_envelope.materialize.stations import chain_station_table, length_squared_g, station_of
from cftuv_envelope.outcomes import NamedOutcome
from cftuv_envelope.wavefront.faces import doubled_shoelace

import materialize_factories as factories

UV = PolicyId("UV_DIRECT_STRIP_V1")
LAWS = (
    DecalTopologyLawV1.TRIANGLES_V1,
    DecalTopologyLawV1.QUAD_STRIPS_V1,
    DecalTopologyLawV1.PLANAR_POLYGONS_V1,
)
CHORD_COUNTERS = (
    TOTAL,
    PLACED,
    AT_NODE,
    NOT_IN_COVERAGE,
    SKIPPED_NOT_MONOTONE,
    SKIPPED_NODE_SHARED,
    FACES_RESTATIONED,
)


# --------------------------------------------------------------------------
# Входы
# --------------------------------------------------------------------------


def _oblique():
    """Цепь `(10,0)-(11.5,4)-(13,8)` на косой хорде: направление `(-1, 1)`, одна внутренняя вершина `v2`."""

    mid = (11.5, 4.0)
    snapshot, request = factories.affine_domain(
        faces=(((0.0, 0.0), (10.0, 0.0), mid, (13.0, 8.0)),),
        routes=({"name": "source", "points": ((10.0, 0.0), mid, (13.0, 8.0))},),
    )
    return factories.prepare_and_cover(snapshot, request) + (request,)


@lru_cache(maxsize=None)
def _building_patch17_d2():
    """`building`, патч 17, d2, alpha 0.45: две прямые цепи по две внутренних вершины, хорда на крупной решётке."""

    folder = Path(__file__).resolve().parents[1] / "fixtures" / "building_patch17_crowded_v1"
    manifest = json.loads((folder / "manifest.json").read_text(encoding="utf-8"))
    snapshot = kernel.AnalysisSnapshotCodecV1.loads((folder / "analysis_snapshot.json").read_bytes())
    request = kernel.DecalRequestCodecV1.loads((folder / "decal_request_density2.json").read_bytes())
    (patch,) = [
        item
        for item in snapshot.patch_domains
        if item.patch_domain_id.value == manifest["patch_domain_ids"][0]
    ]
    return factories.prepare_and_cover(snapshot, request, domain_id=patch.patch_domain_id) + (request,)


def _restationed(parts, *, slide=None, absolute=None, rename=None):
    """Те же покрытие и запрос, но `projection_k_gram` внутренних вершин подменён (до материализации).

    `slide` — `{id вершины: дробь}`: станция `selected_k + дробь`; `absolute` — `{id: станция}`;
    `rename` — `{id: другой id}` в привязке (узел называют две вершины).
    """

    prepared, coverage, request = parts
    compilation = prepared.compilation
    binding = compilation.evaluation_geometry_binding
    chains = []
    for chain in binding.straight_chain_bindings:
        items = []
        for item in chain.internal_assignments:
            name = item.source_vertex_id.value
            station = Fraction(item.projection_k_gram.numerator, item.projection_k_gram.denominator)
            if slide and name in slide:
                station = Fraction(item.selected_k) + slide[name]
            if absolute and name in absolute:
                station = Fraction(absolute[name])
            changes = {"projection_k_gram": ExactRationalV1(station.numerator, station.denominator)}
            if rename and name in rename:
                changes["source_vertex_id"] = SourceVertexId(rename[name])
            items.append(dataclasses.replace(item, **changes))
        chains.append(dataclasses.replace(chain, internal_assignments=tuple(items)))
    edited = dataclasses.replace(binding, straight_chain_bindings=frozenset(chains))
    moved = dataclasses.replace(
        prepared,
        compilation=dataclasses.replace(compilation, evaluation_geometry_binding=edited),
    )
    return moved, coverage, request


def _run(parts, law=DecalTopologyLawV1.PLANAR_POLYGONS_V1):
    prepared, coverage, request = parts
    return materialize_domain(
        prepared,
        coverage,
        request=dataclasses.replace(request, uv_policy_id=UV),
        decal_topology_law=law,
    )


def _spy(monkeypatch):
    """Грани до закона и после него, итог закона и таблица станций: перехват вызова из `_build`."""

    captured: dict = {}
    real = domain.station_chord_vertices

    def spy(prepared, items, table):
        result, stats = real(prepared, items, table)
        captured.update(before=items, after=result, stats=stats, table=table)
        return result, stats

    monkeypatch.setattr(domain, "station_chord_vertices", spy)
    return captured


def _without_the_law(monkeypatch):
    """Отрицательный контроль: материализатор без закона (грани как их сдала стадия слияния)."""

    monkeypatch.setattr(
        domain, "station_chord_vertices", lambda prepared, items, table: (items, ChordStationsV1())
    )


#: Числа прочих стадий, которых закон не трогает (счёты точной работы от самих чисел зависят).
STABLE = (
    "MATERIALIZE_FACES_EMITTED",
    "MATERIALIZE_VERTICES",
    "MATERIALIZE_TRIANGLES",
    "MATERIALIZE_STATION_FACTS",
    "MATERIALIZE_MERGED_RUNS_SPLIT_AT_RUNGS",
    LIFTED,
    DISPLACED,
)


def _numbers(result, names=CHORD_COUNTERS):
    counters = dict(result.counters)
    return {name: counters[name] for name in names}


def _accounted(numbers) -> bool:
    return numbers[TOTAL] == sum(
        numbers[name] for name in (PLACED, AT_NODE, NOT_IN_COVERAGE, SKIPPED_NOT_MONOTONE, SKIPPED_NODE_SHARED)
    )


def _chords(prepared):
    """`[(цепь, точка станции каждой внутренней вершины)]` по привязке: `anchor * 2^r + t * d`, точно."""

    binding = prepared.compilation.evaluation_geometry_binding
    factor = 1 << binding.refinement_power
    found = []
    for chain in binding.straight_chain_bindings:
        anchor = tuple(axis * factor for axis in chain.base_start_node)
        direction = chain.primitive_direction
        for item in chain.internal_assignments:
            value = item.projection_k_gram
            station = Fraction(value.numerator, value.denominator)
            point = tuple(anchor[axis] + station * direction[axis] for axis in (0, 1))
            found.append((chain, item, station, anchor, direction, point))
    return found


def _rational(point):
    return tuple(axis.as_rational() for axis in point)


# --------------------------------------------------------------------------
# Точность: на хорде, проекция Грама, площади
# --------------------------------------------------------------------------


@pytest.mark.parametrize("slide", [Fraction(1, 4), Fraction(-1, 3), Fraction(2, 7)])
def test_a_non_dyadic_station_is_exactly_on_the_chord_at_the_gram_projection(monkeypatch, slide):
    parts = _restationed(_oblique(), slide={"v2": slide})
    captured = _spy(monkeypatch)
    result = _run(parts)

    assert result.is_materialized, result.detail
    ((chain, item, station, anchor, direction, expected),) = _chords(parts[0])
    assert station.denominator not in (1, 2) and station == item.selected_k + slide
    old_node = tuple(item.assigned_refined_node)
    before = {point_key(point): point for _r, face, _l, _s in captured["before"] for point in face.points}
    after = {point_key(point): point for _r, face, _l, _s in captured["after"] for point in face.points}
    (old,) = [point for point in before.values() if lattice_node(point) == old_node]
    (new,) = [point for key, point in after.items() if key not in before]
    # Узел ушёл из всех контуров, а станция стоит ровно в `anchor + t * d`.
    assert point_key(old) not in after
    assert _rational(new) == expected and lattice_node(new) is None
    # Точка лежит на прямой хорды: векторное произведение нуль, без допуска.
    assert (expected[0] - anchor[0]) * direction[1] - (expected[1] - anchor[1]) * direction[0] == 0
    # `s` — проекция Грама на пробег: сдвиг станции на `slide` шагов хорды сдвигает `s` ровно на
    # `slide * |d|_G` (знак — направление пробега относительно хорды).
    budget = factories.budget()
    table = chain_station_table(parts[0], budget)
    (run,) = [item for item in table.runs.values() if item.chain_id == chain.physical_chain_id.value]
    step = SqrtSumV1.radical(1, length_squared_g(table.gram, direction), budget).scaled(slide)
    moved = station_of(run, new) - station_of(run, old)
    assert (moved - step).is_zero or (moved + step).is_zero


@pytest.mark.parametrize("slide", [Fraction(1, 4), Fraction(-1, 3)])
def test_areas_close_exactly_in_faces_parts_and_regions(monkeypatch, slide):
    captured = _spy(monkeypatch)
    assert _run(_restationed(_oblique(), slide={"v2": slide})).is_materialized
    before, after = captured["before"], captured["after"]

    assert [item[0] for item in before] == [item[0] for item in after]
    restationed = 0
    for (_r, old, _l, _s), (_r2, new, _l2, _s2) in zip(before, after):
        if new.points == old.points and new.parts == old.parts:
            assert new == old
            continue
        restationed += 1
        # Площадь следует за контуром точно, у слитой грани и у каждой её части.
        for source, target in [(old, new), *zip(old.parts, new.parts)]:
            assert target.doubled_area - source.doubled_area == (
                doubled_shoelace(target.points) - doubled_shoelace(source.points)
            )
            if doubled_shoelace(source.points) == source.doubled_area:
                assert doubled_shoelace(target.points) == target.doubled_area
        if new.parts:
            total = SqrtSumV1.zero()
            for part in new.parts:
                total = total + part.doubled_area
            assert (total - new.doubled_area).is_zero
    assert restationed >= 1
    # Сдвиг вдоль хорды отдаёт площадь соседу: сумма по региону не меняется.
    for region in {item[0] for item in before}:
        drift = SqrtSumV1.zero()
        for (name, old, _l, _s), (_n, new, _l2, _s2) in zip(before, after):
            if name == region:
                drift = drift + new.doubled_area - old.doubled_area
        assert drift.is_zero


def test_the_sum_of_region_drift_that_is_not_zero_is_a_named_refusal(monkeypatch):
    """Площадь, которая не замкнулась, — `COVERAGE_FACE_LOST` с именем, а не молчаливая дыра."""

    real = chord_station.doubled_shoelace

    def station_of_the_chord(point):
        x = point[0].as_rational()
        return x is not None and x.denominator != 1

    def biased(points):
        value = real(points)
        return value + SqrtSumV1.rational(1) if any(station_of_the_chord(point) for point in points) else value

    monkeypatch.setattr(chord_station, "doubled_shoelace", biased)
    result = _run(_restationed(_oblique(), slide={"v2": Fraction(1, 4)}))

    assert result.outcome is MaterializationOutcome.COVERAGE_FACE_LOST
    assert "CHORD_STATION_AREA_DOES_NOT_CLOSE" in result.detail
    # Отказ несёт числа закона: без чисел его не отличить от «не дошли».
    assert (dict(result.counters)[TOTAL], dict(result.counters)[PLACED]) == (1, 1)


# --------------------------------------------------------------------------
# Имя, счётчики, диагностика
# --------------------------------------------------------------------------


def test_the_station_keeps_its_source_name_and_the_counters_account_for_every_vertex():
    plain = _run(_oblique())
    moved = _run(_restationed(_oblique(), slide={"v2": Fraction(1, 4)}))

    assert plain.is_materialized and moved.is_materialized
    assert _numbers(plain) == {**dict.fromkeys(CHORD_COUNTERS, 0), TOTAL: 1, AT_NODE: 1}
    assert _numbers(moved) == {
        **dict.fromkeys(CHORD_COUNTERS, 0),
        TOTAL: 1,
        PLACED: 1,
        FACES_RESTATIONED: 1,
    }
    # Те же вершины под теми же именами (`src:v2` не стала `node:`), те же грани и те же счётчики
    # прочих стадий: сдвигаются точка и станция, а не состав.
    assert {item.vert_key for item in moved.batch.vertices} == {item.vert_key for item in plain.batch.vertices}
    assert any(item.vert_key.value == "src:v2" for item in moved.batch.vertices)
    assert [face.ordered_vert_keys for face in moved.batch.faces] == [
        face.ordered_vert_keys for face in plain.batch.faces
    ]
    assert _numbers(plain, STABLE) == _numbers(moved, STABLE)
    # `s` у станции другая, поэтому смысл батча (дайджест) сдвинулся: диагностика названа.
    assert moved.batch.semantic_digest != plain.batch.semantic_digest
    named = {item.outcome: item.severity for item in moved.batch.diagnostics}
    assert named[NamedOutcome.SOURCE_VERTEX_STATIONED_ON_CHORD_V1] is GeometryDiagnosticSeverity.INFO
    assert NamedOutcome.SOURCE_VERTEX_STATIONED_ON_CHORD_V1 not in {
        item.outcome for item in plain.batch.diagnostics
    }
    assert any("largest slide 0.25 chord steps" in line for line in moved.diagnostics)


def test_a_station_on_a_node_leaves_the_batch_bitwise_as_it_was(monkeypatch):
    """Станция целая (настоящий источник кладёт вершину на узел): закон нуль действий, дайджест тот же."""

    with_law = _run(_oblique())
    _without_the_law(monkeypatch)
    without = _run(_oblique())

    assert with_law.batch == without.batch
    assert with_law.content_digest == without.content_digest
    assert with_law.vertex_normals == without.vertex_normals


def test_a_domain_without_chain_bindings_reports_zeroes_and_changes_nothing(monkeypatch):
    parts = factories.field_domain("building_002_weighted_normals_v1")
    result = _run(parts)
    _without_the_law(monkeypatch)
    plain = _run(parts)

    assert result.is_materialized
    assert _numbers(result) == dict.fromkeys(CHORD_COUNTERS, 0)
    assert result.batch == plain.batch and result.content_digest == plain.content_digest


def test_a_chain_whose_stations_do_not_increase_stays_on_nodes_and_is_named():
    parts = factories.straight_chain_domain()
    plain = _run(parts)
    # Станция первой вершины стала дальше второй: порядок вдоль хорды нарушен (ничья на полушаге).
    result = _run(_restationed(parts, absolute={"v1": Fraction(131072) + Fraction(1, 2), "v2": Fraction(131072)}))

    assert result.is_materialized
    numbers = _numbers(result)
    assert numbers[TOTAL] == numbers[SKIPPED_NOT_MONOTONE] == 2 and numbers[PLACED] == 0
    assert _accounted(numbers)
    assert result.batch.vertices == plain.batch.vertices and result.batch.faces == plain.batch.faces
    warned = [item for item in result.batch.diagnostics if item.outcome is NamedOutcome.SOURCE_VERTEX_CHORD_STATION_SKIPPED]
    assert len(warned) == 1 and warned[0].severity is GeometryDiagnosticSeverity.WARNING
    assert any("chain:source:NOT_MONOTONE(2)" in line for line in result.diagnostics)
    assert not any(
        item.outcome is NamedOutcome.SOURCE_VERTEX_STATIONED_ON_CHORD_V1 for item in result.batch.diagnostics
    )


def test_a_node_that_two_vertices_claim_is_not_stationed_under_a_foreign_name():
    parts = _restationed(_oblique(), slide={"v2": Fraction(1, 4)}, rename={"v2": "v9"})
    result = _run(parts)

    assert result.is_materialized
    numbers = _numbers(result)
    assert numbers[SKIPPED_NODE_SHARED] == numbers[TOTAL] == 1 and numbers[PLACED] == 0
    assert _accounted(numbers)
    assert any("NODE_NAMES_ANOTHER_VERTEX(1)" in line for line in result.diagnostics)


def test_a_vertex_whose_node_no_face_reaches_is_counted_not_dropped():
    """Узла нет ни в одной грани: ставить нечего, и это число, а не молчание."""

    prepared = _restationed(_oblique(), slide={"v2": Fraction(1, 4)})[0]
    table = chain_station_table(prepared, factories.budget())
    items, stats = chord_station.station_chord_vertices(prepared, [], table)

    assert items == [] and stats.names == {}
    assert (stats.total, stats.placed, stats.not_in_coverage) == (1, 0, 1)
    assert _accounted(dict(stats.counters()))


# --------------------------------------------------------------------------
# Положения не зависят от закона топологии
# --------------------------------------------------------------------------


@pytest.mark.parametrize("slide", [Fraction(1, 4), Fraction(-1, 3)])
def test_the_positions_and_the_digest_agree_across_the_topology_laws(slide):
    results = [_run(_restationed(_oblique(), slide={"v2": slide}), law) for law in LAWS]

    assert all(item.is_materialized for item in results)
    first = results[0]
    for other in results[1:]:
        assert other.batch.vertices == first.batch.vertices
        assert other.batch.semantic_digest == first.batch.semantic_digest
        assert _numbers(other) == _numbers(first)


# --------------------------------------------------------------------------
# Настоящий домен `building`, патч 17
# --------------------------------------------------------------------------


def test_the_field_domain_has_no_displaced_source_vertices_any_more(monkeypatch):
    parts = _building_patch17_d2()
    stationed = _run(parts)
    _without_the_law(monkeypatch)
    plain = _run(parts)

    assert stationed.is_materialized and plain.is_materialized
    # До закона: вершины внутренностей хорды остаются на подъёме узла, дальше ячейки от хоста.
    assert dict(plain.counters)[DISPLACED] > 0
    numbers = _numbers(stationed)
    assert dict(stationed.counters)[DISPLACED] == 0
    assert numbers[TOTAL] == numbers[PLACED] == 4 and _accounted(numbers)
    assert dict(stationed.counters)[LIFTED] > dict(plain.counters)[LIFTED]
    # Тот же состав граней и вершин: закон двигает точки и станции.
    assert len(stationed.batch.faces) == len(plain.batch.faces)
    assert len(stationed.batch.vertices) == len(plain.batch.vertices)


def test_every_station_of_the_field_domain_is_on_its_chord_exactly(monkeypatch):
    parts = _building_patch17_d2()
    captured = _spy(monkeypatch)
    assert _run(parts).is_materialized
    after = {point_key(point) for _r, face, _l, _s in captured["after"] for point in face.points}
    for chain, item, station, anchor, direction, expected in _chords(parts[0]):
        assert station.denominator > 1 << 40
        point = tuple(SqrtSumV1.rational(axis) for axis in expected)
        assert point_key(point) in after
        assert (expected[0] - anchor[0]) * direction[1] - (expected[1] - anchor[1]) * direction[0] == 0
        assert abs(station - item.selected_k) <= Fraction(1, 2)
    stats = captured["stats"]
    assert stats.placed == 4 and set(stats.names.values()) == {
        item.source_vertex_id.value for _c, item, *_rest in _chords(parts[0])
    }


def test_the_positions_of_the_field_domain_do_not_depend_on_the_topology_law():
    parts = _building_patch17_d2()
    results = [_run(parts, law) for law in LAWS]

    assert all(item.is_materialized for item in results)
    for other in results[1:]:
        assert other.batch.vertices == results[0].batch.vertices
        assert other.batch.semantic_digest == results[0].batch.semantic_digest


# --------------------------------------------------------------------------
# Вторая точка имени при сварке: `intern_vertices`
# --------------------------------------------------------------------------


def _point(x, y):
    return (SqrtSumV1.rational(Fraction(x)), SqrtSumV1.rational(Fraction(y)))


def _item(region, *points):
    return (region, SimpleNamespace(face=SimpleNamespace(points=tuple(_point(*p) for p in points))))


def _table(**names):
    mapping = {}
    for spec, vertex_id in names.items():
        region, x, y = spec.split("_")
        mapping[(region, (int(x), int(y)))] = vertex_id
    return SimpleNamespace(node_vertex_ids=mapping)


STATION = (Fraction(7, 3), 0)
CORNERS = ((0, 0), (4, 0), (0, 4))
FACE_WITH_STATION = ((0, 0), STATION, (4, 0), (0, 4))


def test_a_point_that_is_not_a_node_is_named_by_the_chord_map_and_not_by_the_table():
    named = {("a", point_key(_point(*STATION))): "v1"}
    cycles, points = assemble.intern_vertices(
        [_item("a", *FACE_WITH_STATION)], _table(a_0_0="v0", a_4_0="v2", a_0_4="v3"), [], named
    )
    assert [key for key, _ in cycles[0]] == ["src:v0", "src:v1", "src:v2", "src:v3"]
    assert points["src:v1"] == _point(*STATION)
    # Без карты имён точка вне решётки остаётся безымянной, как раньше.
    cycles, _points = assemble.intern_vertices(
        [_item("a", *FACE_WITH_STATION)], _table(a_0_0="v0", a_4_0="v2", a_0_4="v3"), []
    )
    assert [key for key, _ in cycles[0]] == ["src:v0", "node:0", "src:v2", "src:v3"]


def test_two_regions_naming_one_station_differently_report_the_dropped_name():
    key = point_key(_point(*STATION))
    notes: list = []
    cycles, _points = assemble.intern_vertices(
        [_item("a", *FACE_WITH_STATION), _item("b", *FACE_WITH_STATION)],
        _table(),
        notes,
        {("a", key): "v1", ("b", key): "w1"},
    )
    assert cycles[0][1][0] == cycles[1][1][0] == "src:v1"
    assert len(notes) == 1 and "w1" in notes[0] and "its chord station" in notes[0]


def test_two_stations_under_one_source_name_are_a_named_refusal():
    first, second = point_key(_point(*STATION)), point_key(_point(Fraction(8, 3), 0))
    with pytest.raises(MaterializationRefusal) as refusal:
        assemble.intern_vertices(
            [_item("a", (0, 0), STATION, (Fraction(8, 3), 0), (0, 4))],
            _table(),
            [],
            {("a", first): "v1", ("a", second): "v1"},
        )
    assert "VERTEX_KEY_COLLISION: src:v1" in refusal.value.detail
    assert "its chord station" in refusal.value.detail


def test_lattice_node_is_the_integer_point_and_nothing_else():
    assert lattice_node(_point(3, -2)) == (3, -2)
    assert lattice_node(_point(Fraction(7, 3), 0)) is None
    root = SqrtSumV1.radical(1, 2)
    assert lattice_node((root, SqrtSumV1.rational(1))) is None
