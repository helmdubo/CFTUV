"""Заверенный интервал ширины (`materialize.interval`) и подпись структуры батча (`materialize.structure`).

Что утверждается, и чем оно проверено:

1. ВНУТРИ ИНТЕРВАЛА СТРУКТУРА ТА ЖЕ. Для доменов-развёрток (тройка законов резки по треугольникам и тесселяции многоугольниками - то
   есть только заверенные события) и полевых доменов с резкой подпись структуры батча при ширинах внутри `(low, high)` равна подписи
   при alpha0: те же вершины, грани, цепи и факты станций, регионы пересчитаны по содержанию.
2. ЧЕРЕЗ ГРАНИЦУ СТРУКТУРА МЕНЯЕТСЯ. Шаг чуть за границу интервала у каждого домена, у которого граница есть, меняет подпись хотя бы
   у одной из границ: интервал не пустая оговорка, а настоящий порог (границы округлены внутрь, поэтому не каждый шаг за границу
   меняет структуру, но порог ловится).
3. НАЗВАНО, ЧЕГО ИНТЕРВАЛ НЕ ЗАВЕРЯЕТ. Запись несёт названия заверенных классов событий и классов, у которых корня в замкнутой
   форме нет (диагонали ячейки, допуск силуэта и др.): честная граница утверждения, а не молчание.
4. ЗАПИСЬ НИЧЕГО НЕ ДВИГАЕТ. `certify=True` не меняет ни батча, ни счётчиков, ни дайджестов; интервал и подпись не входят в
   сравнение результата.
5. НЕСЧИТАЕМОЕ НАЗВАНО. Событие, которое посчитать нельзя (источник не целочисленная решётка, перебор длиннее предела), даёт интервал,
   схлопнутый в точку, со статусом и причиной, а не заниженный интервал.
6. ТАБЛИЦА СОБЫТИЙ — ПАМЯТЬ ПОДГОТОВКИ: второй вызов берёт её из памяти и отвечает так же.
"""

from __future__ import annotations

import dataclasses
import pickle
from fractions import Fraction

import pytest

import developable_factories as df
import materialize_factories as factories
from developable_route import developable_domain
from materialize_factories import prepare_and_cover

from cftuv_envelope.canonical import canonical_json_bytes
from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
from cftuv_envelope.materialize import interval as interval_module
from cftuv_envelope.materialize.admit import materialization_request
from cftuv_envelope.materialize.domain import materialize_domain
from cftuv_envelope.materialize.interval import (
    AT_EVENT,
    CERTIFIED,
    NOT_CERTIFIED,
    SCOPE,
    UNCERTIFIED,
    alpha_interval,
)
from cftuv_envelope.materialize.memo import memo_disabled, memo_of
from cftuv_envelope.materialize.structure import batch_structure
from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor

ROUTE = ("r0a", "r0b")
BY_TRIANGLES = NearPlanarLiftLawV1.SOURCE_TRIANGLES_CLIPPED_V1
BY_FACES = NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1
POLYGONS = DecalTopologyLawV1.PLANAR_POLYGONS_V1
SILHOUETTE = DecalTopologyLawV1.SILHOUETTE_TOPOLOGY_V1


def developable(make, alpha="1"):
    snapshot, request = developable_domain(make(), ROUTE, alpha=alpha)
    prepared, _coverage = prepare_and_cover(snapshot, request)
    return prepared


def field(name):
    snapshot, request = factories.load_fixture(name)
    prepared = prepare_conveyor(snapshot, request)
    assert prepared.outcome.value == "EXACT", prepared.detail
    return prepared


def materialize(prepared, alpha, lift=BY_TRIANGLES, topology=POLYGONS, **kwargs):
    coverage = conveyor_coverage(prepared, str(alpha))
    assert coverage.outcome.value == "EXACT", coverage.detail
    return materialize_domain(
        prepared,
        coverage,
        request=materialization_request(prepared, uv_policy_id="UV_DIRECT_STRIP_V1"),
        near_planar_lift_law=lift,
        decal_topology_law=topology,
        certify=True,
        **kwargs,
    )


#: `(имя, постройка домена, ширины alpha0)`: развёртки с резкой и поле с резкой (шум крыши).
CLIPPED = (
    ("fold", lambda: developable(df.fold_strip), ("0.3", "1.3")),
    ("slant", lambda: developable(df.slant_fold), ("0.7", "2.0")),
    ("quarter", lambda: developable(df.quarter_cylinder), ("0.7", "2.0")),
    ("noise_top", lambda: field("wall_noise_top_rung_clip_v1"), ("0.25", "0.85")),
)
#: Полевые домены без резки: заверены события покрытия, законы хоста целиком.
COVERAGE_ONLY = (
    "building_002_weighted_normals_v1",
    "building_002_point_contact_v1",
    "building_002_full_selection_v1",
    "mesh2_patch0_cut_fans_v1",
)


def samples(record, count=4):
    """Ширины строго внутри интервала: у обоих краёв и в середине каждой половины."""

    high = record.high if record.high is not None else record.alpha * 3
    inside = []
    for fraction in (0.03, 0.5, 0.97):
        inside.append(record.alpha + (record.low - record.alpha) * fraction)
        inside.append(record.alpha + (high - record.alpha) * fraction)
    return [value for value in inside if record.low < value < high and value > 0][: 2 * count]


def beyond(record):
    """Ширины чуть за границами (если граница есть)."""

    found = []
    if record.low > 0:
        found.append(record.low - (record.low * 0.002 + 1e-7))
    if record.high is not None:
        found.append(record.high + (record.high * 0.002 + 1e-7))
    return [value for value in found if value > 0]


@pytest.mark.parametrize("name,build,alphas", CLIPPED, ids=[item[0] for item in CLIPPED])
def test_the_structure_holds_inside_the_interval_and_changes_across_a_bound(name, build, alphas):
    prepared = build()
    detected = 0
    for alpha in alphas:
        base = materialize(prepared, alpha)
        assert base.is_materialized, base.detail
        record = base.interval
        assert record.status == CERTIFIED, record
        assert record.low < record.alpha and (record.high is None or record.alpha < record.high)
        inside = samples(record)
        assert inside, record
        for width in inside:
            other = materialize(prepared, repr(width))
            assert other.is_materialized, (width, other.detail)
            assert base.structure.differing(other.structure) == (), (name, alpha, width, record)
            # Тот же интервал: он не зависит от того, в какой точке внутри посчитан домен.
            assert other.interval.low == record.low and other.interval.high == record.high
        for width in beyond(record):
            other = materialize(prepared, repr(width))
            if not other.is_materialized or base.structure.differing(other.structure):
                detected += 1
    assert detected >= 1, f"{name}: a step beyond the bounds changed nothing at any bound - the interval would be an empty claim"


@pytest.mark.parametrize("name", COVERAGE_ONLY)
def test_a_domain_without_clipping_certifies_the_coverage_events_under_the_host_laws(name):
    prepared = field(name)
    alpha = prepared.requested_alpha.value
    base = materialize(prepared, alpha, BY_FACES, SILHOUETTE, digests=False)
    assert base.is_materialized and base.interval.status == CERTIFIED
    assert base.interval.clip_events == 0, "an exact-plane domain is not clipped: no clip events"
    for width in samples(base.interval, count=3):
        other = materialize(prepared, repr(width), BY_FACES, SILHOUETTE, digests=False)
        assert other.is_materialized
        assert base.structure.differing(other.structure) == (), (name, width, base.interval)
    changed = [
        base.structure.differing(materialize(prepared, repr(width), BY_FACES, SILHOUETTE, digests=False).structure)
        for width in beyond(base.interval)
    ]
    assert not beyond(base.interval) or any(changed)


def test_an_alpha_that_is_itself_an_event_has_no_neighbourhood_and_is_named():
    prepared = developable(df.fold_strip)
    base = materialize(prepared, "2.0")  # ширина полосы 1: событие покрытия при alpha 1, 2 - сумма двух - событие резки
    assert base.interval.status == AT_EVENT
    assert base.interval.low == base.interval.high == base.interval.alpha == 2.0
    assert not base.interval.contains(2.0)


def test_the_record_names_what_is_certified_and_what_is_not():
    prepared = developable(df.quarter_cylinder)
    record = materialize(prepared, "0.7").interval.as_record()
    assert record["law"] == "COVERAGE_AND_CLIP_CROSSINGS_V1" and record["status"] == CERTIFIED
    assert record["scope"] == list(SCOPE) and record["uncertified"] == list(UNCERTIFIED)
    assert "CELL_DIAGONAL_CHORD_VERDICT" in record["uncertified"] and "SILHOUETTE_DISSOLVE_TOLERANCE" in record["uncertified"]
    assert record["events"] > record["clip_events"] > 0
    assert pickle.loads(pickle.dumps(materialize(prepared, "0.7").interval)) == materialize(prepared, "0.7").interval


@pytest.mark.parametrize("law", [(BY_TRIANGLES, POLYGONS), (BY_FACES, SILHOUETTE)], ids=["certified-laws", "host-laws"])
def test_the_certification_changes_nothing_in_the_answer(law):
    prepared = developable(df.quarter_cylinder)
    lift, topology = law
    plain = materialize_domain(
        prepared,
        conveyor_coverage(prepared, "0.7"),
        request=materialization_request(prepared, uv_policy_id="UV_DIRECT_STRIP_V1"),
        near_planar_lift_law=lift,
        decal_topology_law=topology,
    )
    certified = materialize(prepared, "0.7", lift, topology)
    assert plain.interval is None and plain.structure is None
    assert certified.interval is not None and certified.structure is not None
    assert (plain.counters, plain.content_digest, plain.diagnostics, plain.vertex_normals) == (
        certified.counters,
        certified.content_digest,
        certified.diagnostics,
        certified.vertex_normals,
    )
    assert canonical_json_bytes(plain.batch) == canonical_json_bytes(certified.batch)
    # Секунды стадий - не ответ; записанные факты в равенство результата не входят.
    assert dataclasses.replace(plain, timings=()) == dataclasses.replace(certified, timings=())


def test_an_event_that_cannot_be_computed_collapses_the_interval_and_is_named(monkeypatch):
    prepared = developable(df.fold_strip)
    triangles = materialize(prepared, "0.3").interval  # прогрев: таблица событий лежит в памяти подготовки
    assert triangles.status == CERTIFIED
    monkeypatch.setattr(interval_module, "PAIR_LIMIT", 0)
    other = developable(df.fold_strip)
    limited = materialize(other, "0.3").interval
    assert limited.status == NOT_CERTIFIED and limited.reason == interval_module.REASON_PAIRS
    assert limited.low == limited.high == limited.alpha and not limited.contains(limited.alpha)

    @dataclasses.dataclass
    class Triangle:
        chart: tuple

    fractional = (Triangle(((Fraction(1, 2), Fraction(0)), (Fraction(1), Fraction(0)), (Fraction(0), Fraction(1)))),)
    named = alpha_interval(developable(df.fold_strip), Fraction(3, 10), fractional, "SOURCE_TRIANGLES_CLIPPED_V1")
    assert named.status == NOT_CERTIFIED and named.reason == interval_module.REASON_LINES


def test_the_event_table_lives_in_the_memory_of_the_preparation():
    prepared = developable(df.quarter_cylinder)
    memo = memo_of(prepared)
    first = materialize(prepared, "0.7").interval
    hits = memo.hits
    again = materialize(prepared, "1.3").interval
    assert memo.hits > hits, "the second width must take the table from the memory of the preparation"
    assert again.events == first.events
    with memo_disabled():
        recomputed = materialize(prepared, "0.7").interval
    assert recomputed == first


def test_the_structure_signature_ignores_region_numbers_and_sees_every_real_change():
    prepared = developable(df.quarter_cylinder)
    batch = materialize(prepared, "0.7").batch
    signature = batch_structure(batch)
    assert batch_structure(batch) == signature

    def renumbered(item):
        return dataclasses.replace(item, semantic_region_id=type(item.semantic_region_id)(item.semantic_region_id.value + "-other"))

    same = dataclasses.replace(
        batch,
        faces=tuple(renumbered(face) for face in batch.faces),
        station_facts=frozenset(renumbered(fact) for fact in batch.station_facts),
    )
    assert signature.differing(batch_structure(same)) == (), "region numbers come from instance names and are not structure"
    fewer = dataclasses.replace(batch, faces=batch.faces[:-1])
    assert "faces" in signature.differing(batch_structure(fewer))
    rotated = dataclasses.replace(
        batch,
        faces=tuple(
            dataclasses.replace(
                face,
                ordered_vert_keys=face.ordered_vert_keys[1:] + face.ordered_vert_keys[:1],
                uv_facts=face.uv_facts[1:] + face.uv_facts[:1],
            )
            for face in batch.faces
        ),
    )
    assert batch_structure(rotated) == signature, "a ring is the same contour from any start"
    assert signature.digest and len(signature.digest) == 16
