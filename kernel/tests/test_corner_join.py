"""JOIN мягкого излома одной цепи (`CORNER_JOIN_SOFT_BEND_V1`): ядро, проверяющий, потоки.

Фикстура — пятиугольник с ВОГНУТОЙ вершиной `(10, 0)` между рёбрами `(0,0)-(10,0)` и
`(10,0)-(20,-2)`: поворот вправо на `atan(1/5) = 11.31°`, `δ/π = 0.0628 < 1/4`. Два
маршрута — два куска; общая запись `chain-source` в `data_record_lineage` делает их
одной цепью хоста. Числа, на которых стоят утверждения, посчитаны НЕ проверяемым кодом:
длины рёбер `10` и `sqrt(104)` известны из входа, `tan(δ/2) = 0.0990` — из геометрии.

| что проверяется                                                        | тест |
|------------------------------------------------------------------------|------|
| мягкий излом одной цепи — JOIN: `k = 0`, закон, запись с причиной       | `..._soft_bend_in_one_source_chain_joins` |
| без общей записи хоста закон инертен: прежний счёт, причина названа     | `..._without_a_shared_source_lineage_the_profile_law_stands` |
| угол от 45° и интервал поверх порога — прежний закон, названы           | `..._hard_and_uncertain_bends_keep_the_profile_by_name` |
| подделанная или пропавшая запись — именованный отказ                    | `..._a_tampered_or_missing_record_is_refused` |
| поток: `s` копится сквозь угол, регион один, шва нет, перекладина       | `..._the_flow_accumulates_s_through_the_join_without_a_seam` |
| без JOIN те же два куска — два региона и шов                            | `..._without_the_join_the_pieces_stay_two_regions_with_a_seam` |
| `PLANAR_POLYGONS_V1`: четырёхгранья у угла билинейны и остаются целыми  | `..._the_join_quads_are_bilinear_and_stay_whole` |
| кольцо из 16 мягких изломов: один разрез, два региона, один шов          | `..._a_closed_ring_of_soft_kinks_opens_at_one_named_cut` |
| запись обработки несётся планом и пересчитывается по сырому снапшоту     | `..._the_plan_validator_...`, `..._a_join_selection_without_a_record_...` |
| одна дверь угла и «одна цепь» ВЛАДЕЛЬЦА                                  | `..._the_treatment_reads_the_angle_...`, `..._only_the_owner_patch_...` |
| имена экземпляров остаются в происхождении граней потока                 | `..._flow_faces_keep_the_instance_names_...` |
| потоки: цикл размыкается и назван; угол вне домена и без петли названы    | `..._flows_open_a_cycle_...`, `..._join_successors_...` |
"""

from __future__ import annotations

import dataclasses
import math
from decimal import Decimal
from types import SimpleNamespace

import pytest

import cftuv_envelope as kernel
from cftuv_envelope.contracts.envelopes import (
    CornerTreatmentReasonV1,
    CornerTreatmentRecordV1,
    CornerTreatmentV1,
    SelectionLaw,
)
from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.ids import PolicyId
from cftuv_envelope.numeric import ExactRatioV1
from cftuv_envelope.codec import CompiledPlanCodecV1
from cftuv_envelope.materialize.admit import MaterializationOutcome
from cftuv_envelope.materialize.domain import materialize_domain
from cftuv_envelope.materialize.stations import (
    SKIP_JOIN_CORNER_NOT_ADJACENT,
    SKIP_JOIN_CYCLE_OF_ONE_USE,
    SKIP_JOIN_USE_NOT_IN_DOMAIN_LOOPS,
    _flows,
    _join_successors,
)
from cftuv_envelope.planar_metric import fraction_from_exact
from cftuv_envelope.reference.common import GeometryContext, ReferenceGeometryError
from cftuv_envelope.reference.contracts import ReferenceOutcome
from cftuv_envelope.reference.corner_treatment import corner_treatment_errors
from cftuv_envelope.validation import validate_compiled_plan, validate_cross_contract_references
from cftuv_envelope.validation_corner_treatment import (
    validate_plan_corner_treatments,
    validate_plan_corner_treatments_against_snapshot,
)
from cftuv_envelope.validation_issues import ValidationCode
from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor

import materialize_factories as factories
from reference_factories import _interval, straight_snapshot

UV = PolicyId("UV_DIRECT_STRIP_V1")
SOFT_FAR = (20.0, -2.0)
HARD_FAR = (20.0, -15.0)
#: Поворот на `atan(1.1) = 47.7°` (`δ/π = 0.2651`): широкий интервал накрывает порог 1/4, точка не решает.
WIDE_FAR = (20.0, -11.0)
#: `δ/π` для поворота на `atan(1/5)` и `atan(3/2)`: 0.062833 и 0.312833.
SOFT_BOUNDS = ("0.0628", "0.0629")
HARD_BOUNDS = ("0.3128", "0.3129")
WIDE_BOUNDS = ("0.2450", "0.2700")
SHARED = kernel.LineageId("chain-source:patch:wall")


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
    """`shared` — общая запись хоста двух кусков: `True` (запись владельца), `False` (нет) либо `LineageId`."""

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
    record = SHARED if shared is True else shared
    chains = frozenset(
        dataclasses.replace(item, data_record_lineage=item.data_record_lineage | {record})
        if record
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
    assert (record.threshold_over_pi.numerator, record.threshold_over_pi.denominator) == (1, 4)
    assert selection.selection_law is SelectionLaw.CORNER_JOIN_SOFT_BEND_V1
    assert selection.resolved_hidden_edge_count == 0
    assert selection.certificate_id == record.selection_certificate_id
    counters = dict(prepared.counters)
    assert counters["CONVEYOR_MITERED_CORNERS"] == 1
    assert counters.get("CONVEYOR_VERTEX_FANS", 0) == 0
    assert not corner_treatment_errors(prepared.compilation)


@pytest.mark.parametrize("density", (None, 1))
def test_without_a_shared_source_lineage_the_profile_law_stands(density):
    """Нет общей записи хоста — закон инертен: прежний счёт, причина `SOURCE_CHAIN_UNPROVEN`."""

    snapshot, request = _snapshot(shared=False)
    request = request if density is None else _density_request(request, density)
    prepared = _prepared(snapshot, request)
    record, selection = _record(prepared), _selection(prepared)
    assert record.treatment is CornerTreatmentV1.ANGULAR_PROFILE
    assert record.reason is CornerTreatmentReasonV1.SOURCE_CHAIN_UNPROVEN
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
    ("far", "bounds", "reason"),
    (
        (HARD_FAR, HARD_BOUNDS, CornerTreatmentReasonV1.REFLEX_EXCESS_NOT_SOFT),
        (WIDE_FAR, WIDE_BOUNDS, CornerTreatmentReasonV1.REFLEX_EXCESS_INTERVAL_CONTAINS_THRESHOLD),
    ),
)
def test_hard_and_uncertain_bends_keep_the_profile_by_name(far, bounds, reason):
    snapshot, request = _snapshot(far, bounds)
    prepared = _prepared(snapshot, _density_request(request, 1))
    record, selection = _record(prepared), _selection(prepared)
    assert record.treatment is CornerTreatmentV1.ANGULAR_PROFILE
    assert record.reason is reason
    assert record.shared_source_lineage_ids == frozenset({SHARED})
    assert selection.selection_law is SelectionLaw.HUBER_EMANATED_DENSITY_FLOOR_V1
    assert selection.resolved_hidden_edge_count == 1


def test_the_kinks_of_a_flat_wall_that_the_old_threshold_refused_join_now():
    """Излом 35° (контур плоской стены `sagging_wall`: 31–36°) был веером при пороге 30° и стал продолжением при 45°."""

    snapshot, request = _snapshot((20.0, -7.0), ("0.1944", "0.1945"))
    prepared = _prepared(snapshot, _density_request(request, 1))
    record, selection = _record(prepared), _selection(prepared)
    assert record.treatment is CornerTreatmentV1.JOIN_CONTINUATION
    assert record.reason is CornerTreatmentReasonV1.SOFT_BEND_IN_ONE_SOURCE_CHAIN
    assert (record.threshold_over_pi.numerator, record.threshold_over_pi.denominator) == (1, 4)
    assert selection.selection_law is SelectionLaw.CORNER_JOIN_SOFT_BEND_V1
    assert selection.resolved_hidden_edge_count == 0
    assert not corner_treatment_errors(prepared.compilation)


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


# --------------------------------------------------------------------------
# «Одна цепь» — цепь ВЛАДЕЛЬЦА угла, и одна дверь углового факта
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("shared", "treatment", "reason"),
    (
        (True, CornerTreatmentV1.JOIN_CONTINUATION, CornerTreatmentReasonV1.SOFT_BEND_IN_ONE_SOURCE_CHAIN),
        # Запись СОСЕДНЕГО патча (шовная цепь несёт записи обоих) одной цепью владельца не делает.
        (
            kernel.LineageId("chain-source:neighbour-patch:wall"),
            CornerTreatmentV1.ANGULAR_PROFILE,
            CornerTreatmentReasonV1.SOURCE_CHAIN_UNPROVEN,
        ),
        # Префикс патча без разделителя: патч `patch` не патч `patch-2`.
        (
            kernel.LineageId("chain-source:patch-2:wall"),
            CornerTreatmentV1.ANGULAR_PROFILE,
            CornerTreatmentReasonV1.SOURCE_CHAIN_UNPROVEN,
        ),
        # Запись другого рода (кусок, а не цепь) тоже не свидетельство.
        (
            kernel.LineageId("chain-record:patch:wall"),
            CornerTreatmentV1.ANGULAR_PROFILE,
            CornerTreatmentReasonV1.SOURCE_CHAIN_UNPROVEN,
        ),
        (False, CornerTreatmentV1.ANGULAR_PROFILE, CornerTreatmentReasonV1.SOURCE_CHAIN_UNPROVEN),
    ),
    ids=("owner-record", "neighbour-record", "prefix-of-another-patch", "other-record-kind", "no-record"),
)
def test_only_the_owner_patch_chain_source_makes_two_pieces_one_chain(shared, treatment, reason):
    snapshot, request = _snapshot(shared=shared)
    prepared = _prepared(snapshot, _density_request(request, 1))
    record = _record(prepared)
    assert (record.treatment, record.reason) == (treatment, reason)
    joined = treatment is CornerTreatmentV1.JOIN_CONTINUATION
    assert bool(record.shared_source_lineage_ids) is joined
    assert (_selection(prepared).selection_law is SelectionLaw.CORNER_JOIN_SOFT_BEND_V1) is joined
    assert not corner_treatment_errors(prepared.compilation)


def test_the_treatment_reads_the_angle_through_the_door_of_the_count_law(monkeypatch):
    """Одна дверь: `selector_reflex_excess_interval` — и для счёта, и для порога JOIN."""

    from cftuv_envelope import _corner_treatment as law
    from cftuv_envelope.reference import compile as compiled

    assert compiled.selector_reflex_excess_interval is law.selector_reflex_excess_interval
    snapshot, request = _snapshot(HARD_FAR, HARD_BOUNDS)
    (sector,) = snapshot.angular_owner_sectors
    (angle,) = snapshot.reflex_angle_certificates
    uses = {item.chain_use_id: item for item in snapshot.chain_uses}
    chains = {item.physical_chain_id: item for item in snapshot.physical_chains}
    measure = angle.measure_payload
    assert law.decide(sector, measure, uses, chains)[:2] == (
        CornerTreatmentV1.ANGULAR_PROFILE,
        CornerTreatmentReasonV1.REFLEX_EXCESS_NOT_SOFT,
    )
    soft = _interval("0.0100", "0.0200")
    monkeypatch.setattr(law, "selector_reflex_excess_interval", lambda interval: (soft, None))
    assert law.decide(sector, measure, uses, chains)[:2] == (
        CornerTreatmentV1.JOIN_CONTINUATION,
        CornerTreatmentReasonV1.SOFT_BEND_IN_ONE_SOURCE_CHAIN,
    )


# --------------------------------------------------------------------------
# План несёт запись, проверяющий пересчитывает её по сырому снапшоту
# --------------------------------------------------------------------------


def _plan_like(compilation, **changes):
    """То, что видит проверяющий записей плана: сертификаты селекции и записи обработки компиляции."""

    return SimpleNamespace(
        angular_profile_selection_certificates=changes.get(
            "certificates", compilation.profile_selection_certificates
        ),
        corner_treatments=changes.get("records", compilation.corner_treatments),
    )


def _treatment_issues(plan, snapshot):
    issues: list = []
    validate_plan_corner_treatments(issues, plan)
    validate_plan_corner_treatments_against_snapshot(issues, plan, snapshot, ("plans", "p"))
    return issues


def test_the_plan_validator_recomputes_every_treatment_from_the_raw_snapshot():
    snapshot, request = _snapshot()
    compilation = _prepared(snapshot, _density_request(request, 1)).compilation
    (record,) = compilation.corner_treatments
    (selection,) = compilation.profile_selection_certificates
    assert selection.selection_law is SelectionLaw.CORNER_JOIN_SOFT_BEND_V1
    assert not _treatment_issues(_plan_like(compilation), snapshot)
    codes = lambda issues: {issue.code for issue in issues}  # noqa: E731

    # Подделанная запись (другая цепь, другая причина): пересчёт по снапшоту расходится.
    forged = dataclasses.replace(record, shared_source_lineage_ids=frozenset({kernel.LineageId("chain-source:patch:forged")}))
    issues = _treatment_issues(_plan_like(compilation, records=frozenset({forged})), snapshot)
    assert codes(issues) == {ValidationCode.CORNER_TREATMENT}
    assert any("differs from the raw snapshot" in issue.message for issue in issues)

    # Сертификат под законом JOIN, а снапшот цепи владельца не доказывает: `k = 0` на слово не принимается.
    stranger = dataclasses.replace(
        snapshot,
        physical_chains=frozenset(
            dataclasses.replace(item, data_record_lineage=frozenset()) for item in snapshot.physical_chains
        ),
    )
    issues = _treatment_issues(_plan_like(compilation), stranger)
    assert codes(issues) == {ValidationCode.CORNER_TREATMENT}
    assert any("contradicts the raw snapshot" in issue.message for issue in issues)

    # Доказанный JOIN под прежним счётом без записи — тоже отказ (закон не оставляет выбора).
    legacy = dataclasses.replace(
        selection,
        selection_law=SelectionLaw.HUBER_EMANATED_DENSITY_FLOOR_V1,
        resolved_hidden_edge_count=1,
    )
    issues = _treatment_issues(
        _plan_like(compilation, certificates=frozenset({legacy}), records=frozenset()), snapshot
    )
    assert codes(issues) == {ValidationCode.CORNER_TREATMENT}

    # Структура: JOIN-сертификат без записи; запись без сертификата; две записи на один сертификат.
    issues = _treatment_issues(_plan_like(compilation, records=frozenset()), snapshot)
    assert any("has no corner treatment record" in issue.message for issue in issues)
    orphan = dataclasses.replace(record, selection_certificate_id=kernel.SelectionCertificateId("no-such"))
    issues: list = []
    validate_plan_corner_treatments(issues, _plan_like(compilation, records=frozenset({record, orphan})))
    assert {issue.code for issue in issues} == {ValidationCode.MISSING_REFERENCE}
    contradicting = dataclasses.replace(record, treatment=CornerTreatmentV1.ANGULAR_PROFILE)
    issues = _treatment_issues(_plan_like(compilation, records=frozenset({contradicting})), snapshot)
    assert ValidationCode.CORNER_TREATMENT in codes(issues)


def _forged_join_plan(projections):
    """План EC0-C02 (k = 0 под legacy-счётом) с сертификатом, объявленным JOIN, и (по желанию) записью."""

    projection = next(item for item in projections if item.case_id == "EC0-C02")
    plan = projection.plans[0]
    (selection,) = plan.angular_profile_selection_certificates
    assert selection.resolved_hidden_edge_count == 0
    forged = dataclasses.replace(selection, selection_law=SelectionLaw.CORNER_JOIN_SOFT_BEND_V1)
    record = CornerTreatmentRecordV1(
        treatment_law="CORNER_TREATMENT_V1",
        corner_relation_id=selection.corner_relation_id,
        selection_certificate_id=selection.certificate_id,
        incoming_chain_use_id=next(iter(projection.snapshot.chain_uses)).chain_use_id,
        outgoing_chain_use_id=next(iter(projection.snapshot.chain_uses)).chain_use_id,
        treatment=CornerTreatmentV1.JOIN_CONTINUATION,
        reason=CornerTreatmentReasonV1.SOFT_BEND_IN_ONE_SOURCE_CHAIN,
        threshold_over_pi=ExactRatioV1(1, 4),
        reflex_excess_over_pi=_interval("0.0100", "0.0200"),
        shared_source_lineage_ids=frozenset({kernel.LineageId("chain-source:forged")}),
    )
    return projection, plan, forged, record


def test_a_join_selection_without_a_record_is_refused_by_the_plan(projections):
    projection, plan, forged, record = _forged_join_plan(projections)
    assert not [item for item in validate_compiled_plan(plan) if item.code is ValidationCode.CORNER_TREATMENT]
    bare = dataclasses.replace(plan, angular_profile_selection_certificates=frozenset({forged}))
    issues = [item for item in validate_compiled_plan(bare) if item.code is ValidationCode.CORNER_TREATMENT]
    assert any("has no corner treatment record" in item.message for item in issues)


def test_a_forged_join_record_in_the_plan_is_refused_by_the_raw_snapshot(projections):
    """Снапшот EC0 не несёт сертифицированного угла: JOIN недоказуем, и запись не принимается на слово."""

    projection, plan, forged, record = _forged_join_plan(projections)
    sealed = dataclasses.replace(
        plan,
        angular_profile_selection_certificates=frozenset({forged}),
        corner_treatments=frozenset({record}),
    )
    # Структурно запись согласна с сертификатом: проверяет её только пересчёт по снапшоту.
    assert not [item for item in validate_compiled_plan(sealed) if item.code is ValidationCode.CORNER_TREATMENT]
    issues = validate_cross_contract_references(projection.snapshot, projection.request, (sealed,))
    named = [item for item in issues if item.code is ValidationCode.CORNER_TREATMENT]
    assert named and all(item.path[:3] == ("plans", str(sealed.evaluation_plan_id), "corner_treatments") for item in named)
    assert any("cannot be read" in item.message for item in named)
    # Честный план (записей нет, закон прежний) проходит ту же дверь, как и до закона.
    honest = validate_cross_contract_references(projection.snapshot, projection.request, (plan,))
    assert not [item for item in honest if item.code is ValidationCode.CORNER_TREATMENT]


def test_the_plan_codec_carries_the_treatment_records(projections):
    _projection, plan, forged, record = _forged_join_plan(projections)
    sealed = dataclasses.replace(
        plan,
        angular_profile_selection_certificates=frozenset({forged}),
        corner_treatments=frozenset({record}),
    )
    assert CompiledPlanCodecV1.loads(CompiledPlanCodecV1.dumps(sealed)) == sealed


# --------------------------------------------------------------------------
# Потоки: цикл, названные пропуски стыков
# --------------------------------------------------------------------------


def test_flows_open_a_cycle_at_the_smallest_use_and_say_so():
    flows = _flows(["d", "b", "a", "c", "x", "y", "z"], {"x": "y", "y": "z", "a": "b", "b": "a"})
    # Пути идут от вхождения без предшественника; цикл (`замкнут`) — после них, от наименьшего имени.
    assert flows == [(["c"], False), (["d"], False), (["x", "y", "z"], False), (["a", "b"], True)]
    assert _flows(["q", "p", "r"], {"p": "q", "q": "r", "r": "p"}) == [(["p", "q", "r"], True)]


def _use(name, vertices):
    return SimpleNamespace(chain_use_id=kernel.ChainUseId(name), vertices=tuple(vertices))


def _join_context(records, relations):
    return SimpleNamespace(
        snapshot=SimpleNamespace(corner_relations=tuple(relations)),
        compilation=SimpleNamespace(corner_treatments=tuple(records)),
        directed_chain_vertices=lambda use: use.vertices,
    )


def _join_record(corner, incoming, outgoing, treatment=CornerTreatmentV1.JOIN_CONTINUATION):
    return SimpleNamespace(
        corner_relation_id=kernel.CornerRelationId(corner),
        treatment=treatment,
        incoming_chain_use_id=kernel.ChainUseId(incoming),
        outgoing_chain_use_id=kernel.ChainUseId(outgoing),
    )


def _relation(corner, vertex):
    return SimpleNamespace(corner_relation_id=kernel.CornerRelationId(corner), source_vertex_id=vertex)


def test_join_successors_name_what_they_do_not_join_and_count_what_is_not_theirs():
    a, b, c = _use("a", ("v0", "v1")), _use("b", ("v1", "v2")), _use("c", ("v8", "v9"))
    uses = {"a": a, "b": b, "c": c}
    records = [
        _join_record("ok", "a", "b"),  # конец a == начало b == v1: стык
        _join_record("elsewhere", "x", "y"),  # ни одного вхождения в петлях домена: не пропуск, а число
        _join_record("half", "b", "z"),  # одно вхождение в петлях, другого нет: назван
        _join_record("self", "c", "c"),  # цепь, замкнутая на одно вхождение: назван
        _join_record("apart", "a", "c"),  # вхождения не стыкуются в вершине угла: назван
        _join_record("profile", "a", "b", CornerTreatmentV1.ANGULAR_PROFILE),  # не JOIN: не читается
    ]
    relations = [
        _relation("ok", "v1"), _relation("elsewhere", "v1"), _relation("half", "v1"),
        _relation("self", "v9"), _relation("apart", "v1"), _relation("profile", "v1"),
    ]
    skips: list = []
    successors, outside = _join_successors(
        _join_context(records, relations), uses, {"a", "b", "c"}, skips
    )
    assert successors == {"a": "b"} and outside == 1
    assert sorted(skips) == sorted(
        [
            ("half", SKIP_JOIN_USE_NOT_IN_DOMAIN_LOOPS),
            ("self", SKIP_JOIN_CYCLE_OF_ONE_USE),
            ("apart", SKIP_JOIN_CORNER_NOT_ADJACENT),
        ]
    )


def test_join_successors_do_not_swallow_a_missing_field():
    """Нет `compilation` или `snapshot` — исключение, а не молчаливое «потоков нет»."""

    with pytest.raises(AttributeError):
        _join_successors(SimpleNamespace(snapshot=SimpleNamespace(corner_relations=())), {}, set(), [])
    with pytest.raises(AttributeError):
        _join_successors(SimpleNamespace(compilation=SimpleNamespace(corner_treatments=())), {}, set(), [])


# --------------------------------------------------------------------------
# Кольцо из мягких изломов: замкнутый поток размыкается в одном названном месте
# --------------------------------------------------------------------------

RING_SIDES = 16
RING_INNER, RING_OUTER = 20, 50


def _ring_points(radius, sides=RING_SIDES):
    return [
        (float(round(radius * math.cos(2 * math.pi * i / sides))), float(round(radius * math.sin(2 * math.pi * i / sides))))
        for i in range(sides)
    ]


def _ring_snapshot(sides=RING_SIDES, alpha="2"):
    """Плоский патч с круглым отверстием из `sides` отрезков: цепь отверстия замкнута из `sides` кусков одной цепи.

    Отверстие обходится по часовой стрелке (патч слева), каждый поворот — вправо на 360/sides градусов, то
    есть вогнутый для патча на ту же долю π: при `sides >= 9` каждый излом меньше 45° и получает JOIN.
    """

    inner, outer = _ring_points(RING_INNER, sides), _ring_points(RING_OUTER, sides)
    faces = tuple(
        (inner[i], outer[i], outer[(i + 1) % sides], inner[(i + 1) % sides]) for i in range(sides)
    )
    hole = list(reversed(inner))
    routes = tuple(
        {"name": f"p{i:02d}", "points": (hole[i], hole[(i + 1) % sides])} for i in range(sides)
    )
    snapshot, request = straight_snapshot(faces=faces, source_routes=routes, alpha=alpha)
    snapshot = factories.with_affine_metric(snapshot)
    descriptor = next(iter(snapshot.surface_metric_descriptors))
    order = []
    for face in faces:
        for point in face:
            if point not in order:
                order.append(point)
    name = {point: f"v{index}" for index, point in enumerate(order)}
    sectors, certificates, corners = [], [], []
    for i in range(sides):
        before, here, after = hole[(i - 1) % sides], hole[i], hole[(i + 1) % sides]
        uses = tuple(kernel.ChainUseId(f"use:p{j:02d}:use") for j in ((i - 1) % sides, i))
        launches = tuple(kernel.BoundaryConstraintId(f"launch:{item.value}") for item in uses)
        sector_id = kernel.OwnerSectorId(f"sector:corner{i:02d}")
        sectors.append(
            kernel.OrientedOwnerSectorV1(
                owner_sector_id=sector_id,
                owner_patch_id=kernel.PatchId("patch"),
                patch_domain_id=kernel.PatchDomainId("domain"),
                analysis_proven=True,
                ordered_incident_chain_use_ids=uses,
                incoming_support_ref=kernel.SourceSupportRefV1(
                    uses[0], launches[0], _chart_normal(descriptor, name[before], name[here])
                ),
                outgoing_support_ref=kernel.SourceSupportRefV1(
                    uses[1], launches[1], _chart_normal(descriptor, name[here], name[after])
                ),
                turn_orientation=kernel.TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION,
                interior_selection_law=kernel.InteriorSelectionLaw.OWNER_PATCH_INTERIOR_BETWEEN_ORDERED_SUPPORTS,
            )
        )
        first = (here[0] - before[0], here[1] - before[1])
        second = (after[0] - here[0], after[1] - here[1])
        turn = math.atan2(first[0] * second[1] - first[1] * second[0], first[0] * second[0] + first[1] * second[1])
        assert turn < 0  # поворот вправо — вогнутый для патча слева
        delta = -turn / math.pi
        low, high = f"{delta - 0.002:.4f}", f"{delta + 0.002:.4f}"
        angle_id = kernel.AngleCertificateId(f"angle{i:02d}")
        certificates.append(
            kernel.ReflexAngleCertificateV1(
                angle_id,
                sector_id,
                kernel.AngleMeasureLaw.ORIENTED_OWNER_SECTOR_ANGLE,
                kernel.AngleMeasureSource.HOST_ANALYSIS_EXACT_OR_CERTIFIED,
                kernel.StrictAngleRangeCertificate.STRICT_PI_LT_PHI_LT_2PI,
                kernel.ReflexExcessLaw.DELTA_EQUALS_PHI_MINUS_PI,
                kernel.CertifiedReflexAngleMeasureV1(
                    _interval(str(Decimal(1) + Decimal(low)), str(Decimal(1) + Decimal(high))),
                    _interval(low, high),
                    kernel.TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION,
                    kernel.AngleNormalizationLaw.VALUE_OVER_SYMBOLIC_PI_V1,
                ),
                False,
            )
        )
        corners.append(
            kernel.CornerRelationV1(
                kernel.CornerRelationId(f"corner{i:02d}"),
                kernel.SourceVertexId(name[here]),
                sector_id,
                angle_id,
                False,
            )
        )
    chains = frozenset(
        dataclasses.replace(item, data_record_lineage=item.data_record_lineage | {SHARED})
        for item in snapshot.physical_chains
    )
    snapshot = dataclasses.replace(
        snapshot,
        physical_chains=chains,
        angular_owner_sectors=frozenset(sectors),
        reflex_angle_certificates=frozenset(certificates),
        corner_relations=frozenset(corners),
    )
    return snapshot, request, hole


@pytest.mark.parametrize("law", (DecalTopologyLawV1.TRIANGLES_V1, DecalTopologyLawV1.PLANAR_POLYGONS_V1))
def test_a_closed_ring_of_soft_kinks_opens_at_one_named_cut(law):
    """Замкнутая цепь из 16 мягких изломов (круглое отверстие): раньше `STATION_VALUE_CONFLICT` на всём домене.

    Поток-цикл размыкается в наименьшем вхождении: угол между последним и первым — разрез, где вершина
    несёт два набора `(s, r)` в двух регионах; остальные 15 углов — стыки потока, на которых UV непрерывна.
    Шов один — на разрезе; граница регионов у открывателя шва не даёт (UV на ней не рвётся).
    """

    snapshot, request, hole = _ring_snapshot()
    prepared = _prepared(snapshot, request)
    treatments = prepared.compilation.corner_treatments
    assert len(treatments) == RING_SIDES
    assert {item.treatment for item in treatments} == {CornerTreatmentV1.JOIN_CONTINUATION}
    result, counters = _materialized(prepared, request, law)
    assert counters["STATION_FLOWS"] == 1
    assert counters["STATION_JOIN_CORNERS"] == RING_SIDES - 1
    assert counters["STATION_FLOW_CYCLES_OPENED"] == 1
    assert counters["STATION_SKIPS"] == 0
    assert counters["MATERIALIZE_REGIONS"] == 2
    assert counters["MATERIALIZE_INTERFACE_CHAINS"] == 1
    assert counters["MATERIALIZE_FACES_EMITTED"] == RING_SIDES or law is DecalTopologyLawV1.TRIANGLES_V1
    batch = result.batch
    (cut,) = batch.interface_chains
    cut_keys = {item.value for item in cut.ordered_vert_keys}
    by_vertex: dict = {}
    for (_region, key), value in _facts(batch).items():
        by_vertex.setdefault(key, []).append(value)
    two = {key: values for key, values in by_vertex.items() if len(values) == 2}
    torn = {key for key, values in two.items() if values[0] != values[1]}
    # Вершины, у которых UV рвётся, — ровно вершины шва; у стыка открывателя вершины общие и непрерывны.
    assert torn == cut_keys
    assert len(two) - len(torn) == 2
    perimeter = sum(
        math.hypot(hole[(i + 1) % RING_SIDES][0] - hole[i][0], hole[(i + 1) % RING_SIDES][1] - hole[i][1])
        for i in range(RING_SIDES)
    )
    source_vertex = next(key for key in torn if key.startswith("src:"))
    # Метрика аффинной карты рациональна и приближает евклидову (до ~1e-5 относительно): периметр с неё.
    assert sorted(value[0] for value in by_vertex[source_vertex]) == pytest.approx([0.0, perimeter], rel=1e-4, abs=1e-9)
    assert all(abs(value[1]) < 1e-9 for value in by_vertex[source_vertex])
    # Разрез назван диагностикой, а не потерян.
    assert [item.outcome.value for item in batch.diagnostics].count("U_RESTARTS_AT_CLOSED_FLOW_OPENING") == 1


def test_flow_faces_keep_the_instance_names_in_their_provenance():
    snapshot, request = _snapshot()
    prepared = _prepared(snapshot, request)
    result, _counters = _materialized(prepared, request)
    flow_claims = {
        item.value
        for region in result.batch.semantic_regions
        for item in region.provenance.lineage_ids
        if item.value.startswith("claim:")
    }
    # Имя потока — один префикс, не `flow:flow:`; имена экземпляров, которые поток заместил, не потеряны.
    assert "claim:flow:use:in:use" in flow_claims
    assert not any(name.startswith("claim:flow:flow:") for name in flow_claims)
    control_snapshot, control_request = _snapshot(shared=False)
    control, _ = _materialized(_prepared(control_snapshot, control_request), control_request)
    named = {
        item.value
        for region in control.batch.semantic_regions
        for item in region.provenance.lineage_ids
        if item.value.startswith("claim:")
    }
    assert named and named <= flow_claims
