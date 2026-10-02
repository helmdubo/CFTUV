"""JOIN мягкого излома одной цепи (`CORNER_JOIN_SOFT_BEND_V1`): ядро, проверяющий, потоки.

Фикстура — пятиугольник с ВОГНУТОЙ вершиной `(10, 0)` между рёбрами `(0,0)-(10,0)` и
`(10,0)-(20,-2)`: поворот вправо на `atan(1/5) = 11.31°`, `δ/π = 0.0628 < 1/6`. Два
маршрута — два куска; общая запись `chain-source` в `data_record_lineage` делает их
одной цепью хоста. Числа, на которых стоят утверждения, посчитаны НЕ проверяемым кодом:
длины рёбер `10` и `sqrt(104)` известны из входа, `tan(δ/2) = 0.0990` — из геометрии.

| что проверяется                                                        | тест |
|------------------------------------------------------------------------|------|
| мягкий излом одной цепи — JOIN: `k = 0`, закон, запись с причиной       | `..._soft_bend_in_one_source_chain_joins` |
| без общей записи хоста закон инертен: прежний счёт, причина названа     | `..._without_a_shared_source_lineage_the_profile_law_stands` |
| угол от 30° и интервал поверх порога — прежний закон, названы           | `..._hard_and_uncertain_bends_keep_the_profile_by_name` |
| подделанная или пропавшая запись — именованный отказ                    | `..._a_tampered_or_missing_record_is_refused` |
| поток: `s` копится сквозь угол, регион один, шва нет, перекладина       | `..._the_flow_accumulates_s_through_the_join_without_a_seam` |
| без JOIN те же два куска — два региона и шов                            | `..._without_the_join_the_pieces_stay_two_regions_with_a_seam` |
| `PLANAR_POLYGONS_V1`: четырёхгранья у угла билинейны и остаются целыми  | `..._the_join_quads_are_bilinear_and_stay_whole` |
"""

from __future__ import annotations

import dataclasses
import math
from decimal import Decimal

import pytest

import cftuv_envelope as kernel
from cftuv_envelope.contracts.envelopes import (
    CornerTreatmentReasonV1,
    CornerTreatmentV1,
    SelectionLaw,
)
from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.ids import PolicyId
from cftuv_envelope.materialize.admit import MaterializationOutcome
from cftuv_envelope.materialize.domain import materialize_domain
from cftuv_envelope.planar_metric import fraction_from_exact
from cftuv_envelope.reference.common import GeometryContext, ReferenceGeometryError
from cftuv_envelope.reference.contracts import ReferenceOutcome
from cftuv_envelope.reference.corner_treatment import corner_treatment_errors
from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor

import materialize_factories as factories
from reference_factories import _interval, straight_snapshot

UV = PolicyId("UV_DIRECT_STRIP_V1")
SOFT_FAR = (20.0, -2.0)
HARD_FAR = (20.0, -6.0)
#: `δ/π` для поворота на `atan(1/5)` и на `atan(3/5)`: 0.062833 и 0.172021.
SOFT_BOUNDS = ("0.0628", "0.0629")
HARD_BOUNDS = ("0.1720", "0.1721")
WIDE_BOUNDS = ("0.1600", "0.1800")
SHARED = kernel.LineageId("chain-source:wall")


def _chart_normal(descriptor, start: str, end: str):
    """Левая нормаль ребра `start -> end` В КООРДИНАТАХ КАРТЫ (как её пишет хост).

    Карта косая: евклидова нормаль `J t` переводится в координаты карты через
    `M^{-1}`, где `M` — базис `(A, B)`. Так направление опоры G-перпендикулярно
    ребру, и проверяющий ядра находит в сертификате ровно угол поворота.
    """

    coords = {
        item.source_vertex_id.value: (
            fraction_from_exact(item.domain_coordinate.x),
            fraction_from_exact(item.domain_coordinate.y),
        )
        for item in descriptor.exact_source_vertex_coordinates
    }
    ax, ay = (fraction_from_exact(getattr(descriptor.exact_basis_a, axis)) for axis in "xy")
    bx, by = (fraction_from_exact(getattr(descriptor.exact_basis_b, axis)) for axis in "xy")
    tc = (coords[end][0] - coords[start][0], coords[end][1] - coords[start][1])
    tx, ty = ax * tc[0] + bx * tc[1], ay * tc[0] + by * tc[1]
    nx, ny = -ty, tx
    det = ax * by - bx * ay
    ncx, ncy = (by * nx - bx * ny) / det, (-ay * nx + ax * ny) / det
    return kernel.CertifiedAffineSupportDirectionV2(
        kernel.ExactVector2V1(
            kernel.ExactRationalV1(ncx.numerator, ncx.denominator),
            kernel.ExactRationalV1(ncy.numerator, ncy.denominator),
        ),
        descriptor.reference_metric_id,
    )


def _snapshot(far=SOFT_FAR, bounds=SOFT_BOUNDS, *, shared=True, alpha="1"):
    snapshot, request = straight_snapshot(
        faces=(((0.0, 0.0), (10.0, 0.0), far, (20.0, 10.0), (0.0, 10.0)),),
        source_routes=(
            {"name": "in", "points": ((0.0, 0.0), (10.0, 0.0))},
            {"name": "out", "points": ((10.0, 0.0), far)},
        ),
        alpha=alpha,
    )
    snapshot = factories.with_affine_metric(snapshot)
    descriptor = next(iter(snapshot.surface_metric_descriptors))
    uses = (kernel.ChainUseId("use:in:use"), kernel.ChainUseId("use:out:use"))
    launches = tuple(kernel.BoundaryConstraintId(f"launch:{item.value}") for item in uses)
    sector_id = kernel.OwnerSectorId("sector:use:in:use")
    sector = kernel.OrientedOwnerSectorV1(
        owner_sector_id=sector_id,
        owner_patch_id=kernel.PatchId("patch"),
        patch_domain_id=kernel.PatchDomainId("domain"),
        analysis_proven=True,
        ordered_incident_chain_use_ids=uses,
        incoming_support_ref=kernel.SourceSupportRefV1(
            uses[0], launches[0], _chart_normal(descriptor, "v0", "v1")
        ),
        outgoing_support_ref=kernel.SourceSupportRefV1(
            uses[1], launches[1], _chart_normal(descriptor, "v1", "v2")
        ),
        turn_orientation=kernel.TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION,
        interior_selection_law=kernel.InteriorSelectionLaw.OWNER_PATCH_INTERIOR_BETWEEN_ORDERED_SUPPORTS,
    )
    angle_id = kernel.AngleCertificateId("angle")
    certificate = kernel.ReflexAngleCertificateV1(
        angle_id,
        sector_id,
        kernel.AngleMeasureLaw.ORIENTED_OWNER_SECTOR_ANGLE,
        kernel.AngleMeasureSource.HOST_ANALYSIS_EXACT_OR_CERTIFIED,
        kernel.StrictAngleRangeCertificate.STRICT_PI_LT_PHI_LT_2PI,
        kernel.ReflexExcessLaw.DELTA_EQUALS_PHI_MINUS_PI,
        kernel.CertifiedReflexAngleMeasureV1(
            _interval(str(Decimal(1) + Decimal(bounds[0])), str(Decimal(1) + Decimal(bounds[1]))),
            _interval(*bounds),
            kernel.TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION,
            kernel.AngleNormalizationLaw.VALUE_OVER_SYMBOLIC_PI_V1,
        ),
        False,
    )
    corner = kernel.CornerRelationV1(
        kernel.CornerRelationId("corner"), kernel.SourceVertexId("v1"), sector_id, angle_id, False
    )
    chains = frozenset(
        dataclasses.replace(item, data_record_lineage=item.data_record_lineage | {SHARED})
        if shared
        else item
        for item in snapshot.physical_chains
    )
    snapshot = dataclasses.replace(
        snapshot,
        physical_chains=chains,
        angular_owner_sectors=frozenset({sector}),
        reflex_angle_certificates=frozenset({certificate}),
        corner_relations=frozenset({corner}),
    )
    return snapshot, request


def _density_request(request, density=1):
    value, symbol = {
        1: (kernel.MaxSubturnValueId.LINEAR_REFLEX_DENSITY_1_V1, kernel.ExactAngleSymbol.PI_OVER_3),
        2: (kernel.MaxSubturnValueId.LINEAR_REFLEX_DENSITY_2_V1, kernel.ExactAngleSymbol.PI_OVER_4),
    }[density]
    return dataclasses.replace(
        request,
        angular_profile_selection_policy_id=kernel.AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_A_V1,
        max_subturn_parameter_id=kernel.MaxSubturnParameterId.LINEAR_REFLEX_DENSITY_A_V1,
        max_subturn_value_id=value,
        max_subturn_exact_value=kernel.ExactAngleV1(symbol),
    )


def _prepared(snapshot, request):
    prepared = prepare_conveyor(snapshot, request)
    assert prepared.outcome.value == "EXACT", prepared.detail
    return prepared


def _record(prepared):
    (record,) = prepared.compilation.corner_treatments
    return record


def _selection(prepared):
    (selection,) = prepared.compilation.profile_selection_certificates
    return selection


def _facts(batch):
    return {
        (fact.semantic_region_id.value, fact.vert_key.value): (
            float(fact.source_s.value),
            float(fact.source_r.value),
        )
        for fact in batch.station_facts
    }


def _materialized(prepared, request, law=DecalTopologyLawV1.TRIANGLES_V1):
    coverage = conveyor_coverage(prepared, None)
    assert coverage.outcome.value == "EXACT", coverage.detail
    result = materialize_domain(
        prepared,
        coverage,
        request=dataclasses.replace(request, uv_policy_id=UV),
        decal_topology_law=law,
    )
    assert result.outcome is MaterializationOutcome.MATERIALIZED, result.detail
    return result, dict(result.counters)


# --------------------------------------------------------------------------
# Ядро: решение и запись
# --------------------------------------------------------------------------


@pytest.mark.parametrize("density", (None, 1, 2))
def test_a_soft_bend_in_one_source_chain_joins(density):
    snapshot, request = _snapshot()
    request = request if density is None else _density_request(request, density)
    prepared = _prepared(snapshot, request)
    record, selection = _record(prepared), _selection(prepared)
    assert record.treatment is CornerTreatmentV1.JOIN_CONTINUATION
    assert record.reason is CornerTreatmentReasonV1.SOFT_BEND_IN_ONE_SOURCE_CHAIN
    assert record.shared_source_lineage_ids == frozenset({SHARED})
    assert record.incoming_chain_use_id.value == "use:in:use"
    assert record.outgoing_chain_use_id.value == "use:out:use"
    assert (record.threshold_over_pi.numerator, record.threshold_over_pi.denominator) == (1, 6)
    assert selection.selection_law is SelectionLaw.CORNER_JOIN_SOFT_BEND_V1
    assert selection.resolved_hidden_edge_count == 0
    assert selection.certificate_id == record.selection_certificate_id
    counters = dict(prepared.counters)
    assert counters["CONVEYOR_MITERED_CORNERS"] == 1
    assert counters.get("CONVEYOR_VERTEX_FANS", 0) == 0
    assert not corner_treatment_errors(prepared.compilation)


@pytest.mark.parametrize("density", (None, 1))
def test_without_a_shared_source_lineage_the_profile_law_stands(density):
    """Нет общей записи хоста — закон инертен: прежний счёт, причина `SOURCE_CHAINS_DIFFER`."""

    snapshot, request = _snapshot(shared=False)
    request = request if density is None else _density_request(request, density)
    prepared = _prepared(snapshot, request)
    record, selection = _record(prepared), _selection(prepared)
    assert record.treatment is CornerTreatmentV1.ANGULAR_PROFILE
    assert record.reason is CornerTreatmentReasonV1.SOURCE_CHAINS_DIFFER
    assert record.shared_source_lineage_ids == frozenset()
    if density is None:
        # Прежний legacy-закон: `δ <= π/3` даёт `k = 0` сам по себе.
        assert selection.selection_law is SelectionLaw.MIN_K_FOR_MAX_SUBTURN
        assert selection.resolved_hidden_edge_count == 0
    else:
        # Прежний закон плотности: `max(1, C - 1) = 1` — одна опора по биссектрисе.
        assert selection.selection_law is SelectionLaw.HUBER_EMANATED_DENSITY_FLOOR_V1
        assert selection.resolved_hidden_edge_count == 1
        assert dict(prepared.counters)["CONVEYOR_MITERED_CORNERS"] == 0


@pytest.mark.parametrize(
    ("bounds", "reason"),
    (
        (HARD_BOUNDS, CornerTreatmentReasonV1.REFLEX_EXCESS_NOT_SOFT),
        (WIDE_BOUNDS, CornerTreatmentReasonV1.REFLEX_EXCESS_INTERVAL_CONTAINS_THRESHOLD),
    ),
)
def test_hard_and_uncertain_bends_keep_the_profile_by_name(bounds, reason):
    snapshot, request = _snapshot(HARD_FAR, bounds)
    prepared = _prepared(snapshot, _density_request(request, 1))
    record, selection = _record(prepared), _selection(prepared)
    assert record.treatment is CornerTreatmentV1.ANGULAR_PROFILE
    assert record.reason is reason
    assert record.shared_source_lineage_ids == frozenset({SHARED})
    assert selection.selection_law is SelectionLaw.HUBER_EMANATED_DENSITY_FLOOR_V1
    assert selection.resolved_hidden_edge_count == 1


def test_a_tampered_or_missing_record_is_refused():
    snapshot, request = _snapshot()
    prepared = _prepared(snapshot, _density_request(request, 1))
    compilation = prepared.compilation
    record = _record(prepared)
    forged = dataclasses.replace(record, treatment=CornerTreatmentV1.ANGULAR_PROFILE)
    tampered = dataclasses.replace(compilation, corner_treatments=frozenset({forged}))
    assert corner_treatment_errors(tampered)
    missing = dataclasses.replace(compilation, corner_treatments=frozenset())
    assert corner_treatment_errors(missing)
    with pytest.raises(ReferenceGeometryError) as caught:
        GeometryContext.build(tampered, prepared.context.frame)
    assert caught.value.outcome is ReferenceOutcome.CORNER_TREATMENT_INVALID
    # Честная запись проходит ту же дверь.
    GeometryContext.build(compilation, prepared.context.frame)


# --------------------------------------------------------------------------
# Материализатор: потоки
# --------------------------------------------------------------------------


def test_the_flow_accumulates_s_through_the_join_without_a_seam():
    snapshot, request = _snapshot()
    prepared = _prepared(snapshot, request)
    result, counters = _materialized(prepared, request)
    assert counters["STATION_FLOWS"] == 1
    assert counters["STATION_JOIN_CORNERS"] == 1
    assert counters["STATION_SKIP_JOIN_CORNER_NOT_ADJACENT"] == 0
    assert counters["MATERIALIZE_REGIONS"] == 1
    assert counters["MATERIALIZE_INTERFACE_CHAINS"] == 0
    assert counters["MATERIALIZE_RUNG_STATIONS_FROM_CHAIN_VERTEX"] >= 1
    facts = _facts(result.batch)
    by_vertex = {key: value for (_region, key), value in facts.items()}
    assert by_vertex["src:v0"] == pytest.approx((0.0, 0.0), abs=1e-9)
    assert by_vertex["src:v1"] == pytest.approx((10.0, 0.0), abs=1e-9)
    # Конец второго куска: `s` продолжает первый — `10 + sqrt(104)`, не `sqrt(104)`.
    assert by_vertex["src:v2"] == pytest.approx((10.0 + math.sqrt(104.0), 0.0), abs=1e-9)
    # Вершина митры на биссектрисе: `r = alpha`, `s` — станция вершины цепи.
    rung = [value for value in by_vertex.values() if abs(value[1] - 1.0) < 1e-9 and abs(value[0] - 10.0) < 1e-9]
    assert len(rung) == 1
    assert not result.batch.interface_chains
    # `u` монотонна вдоль фронта потока: порядок станций на фронте тот же, что вдоль источника.
    front = sorted(value[0] for value in by_vertex.values() if abs(value[1] - 1.0) < 1e-9)
    assert front == sorted(front) and len(front) >= 3


def test_without_the_join_the_pieces_stay_two_regions_with_a_seam():
    snapshot, request = _snapshot(shared=False)
    prepared = _prepared(snapshot, request)
    result, counters = _materialized(prepared, request)
    assert counters["STATION_FLOWS"] == 0
    assert counters["STATION_JOIN_CORNERS"] == 0
    assert counters["MATERIALIZE_REGIONS"] == 2
    assert counters["MATERIALIZE_INTERFACE_CHAINS"] == 1
    assert counters["MATERIALIZE_RUNG_STATIONS_FROM_CHAIN_VERTEX"] == 0
    by_vertex = {}
    for (_region, key), value in _facts(result.batch).items():
        by_vertex.setdefault(key, []).append(value)
    # Второй кусок начинает счёт заново: его конец на `sqrt(104)`.
    assert any(abs(value[0] - math.sqrt(104.0)) < 1e-9 for value in by_vertex["src:v2"])
    # Вершина угла и митра имеют по ДВА набора `(s, r)` — по одному на регион.
    assert len(by_vertex["src:v1"]) == 2


def test_the_join_quads_are_bilinear_and_stay_whole():
    snapshot, request = _snapshot()
    prepared = _prepared(snapshot, request)
    result, counters = _materialized(prepared, request, DecalTopologyLawV1.PLANAR_POLYGONS_V1)
    assert counters["MATERIALIZE_POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE"] == 0
    assert counters["MATERIALIZE_QUADS_UV_BILINEAR"] == 2
    # Излом UV на диагонали показа: `alpha * tan(δ/2) = 0.0990` -> 99..100 тысячных alpha.
    assert 90 <= counters["MATERIALIZE_QUADS_UV_BILINEAR_MAX_MILLI_ALPHA"] <= 110
    sizes = {len(face.ordered_vert_keys) for face in result.batch.faces}
    assert sizes == {4}
    assert counters["MATERIALIZE_INTERFACE_CHAINS"] == 0
    # Без JOIN та же тесселяция режет оба четырёхгранья? Нет: без JOIN UV аффинна в каждом
    # регионе, и четырёхгранья целы по старому закону — билинейных нет.
    control_snapshot, control_request = _snapshot(shared=False)
    _result, control = _materialized(
        _prepared(control_snapshot, control_request), control_request, DecalTopologyLawV1.PLANAR_POLYGONS_V1
    )
    assert control["MATERIALIZE_QUADS_UV_BILINEAR"] == 0
    assert control["MATERIALIZE_POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE"] == 0
