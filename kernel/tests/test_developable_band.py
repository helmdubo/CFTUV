"""ПОЛОСОВАЯ КАРТА (C1): носитель вокруг выбранных цепей, когда целый патч не развёртывается.

Полоса пробуется ТОЛЬКО после именованного отказа развёртки целого патча; принятый целый патч до неё не доходит
(байты прежние). Власть полосы - запас в сертификате, а не носитель: наименьший квадрат расстояния НА КАРТЕ между
ободом и стеной досягаемости не меньше `cap^2`. Кольцо (носитель - кольцо) в этом срезе остаётся
`PERIODIC_CUT_REQUIRED`.
"""

from __future__ import annotations

import dataclasses
from fractions import Fraction

import pytest

import cftuv_envelope as kernel
from cftuv_envelope import _band_chart, _band_support
from cftuv_envelope.chart_band import chart_band_request, directed_use_edges
from cftuv_envelope.codec import canonical_json_bytes
from cftuv_envelope.contracts.analysis import ChainUseOrientation
from cftuv_envelope.contracts.metric import (
    BandBoundaryRoleV1,
    DevelopableBandChartCertificateV1,
    DevelopableUnfoldCertificateV1,
    ExactRationalV1,
    NearPlanarLiftLawV1,
    NearPlanarProjectionCertificateV1,
    chart_reach_cap_is_lawful,
    is_unfolded_certificate,
)
from cftuv_envelope.contracts.request import DecalRequestV1
from cftuv_envelope.materialize.admit import materialization_request
from cftuv_envelope.materialize.domain import materialize_domain
from cftuv_envelope.outcomes import NamedOutcome
from cftuv_envelope.planar_metric import BAND_TRIGGER_OUTCOMES, PlanarMetricAdmissionError
from cftuv_envelope.validation_issues import ValidationCode
from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor

import band_factories as factories

#: Малый свод: 8 колонок, 8 строк цилиндра по 0.1 м и апсида в 3 строки. Стена досягаемости при cap 0.5 и допуске 1/5
#: (D = 0.6) приходится на строки цилиндра, а целый свод отказывает растяжением апсиды.
ARCH = dict(cols=8, barrel_rows=8, apse_rows=3)
CAP = "1/2"


def _certificate(snapshot):
    return next(iter(snapshot.surface_metric_descriptors)).planarity_certificate


@pytest.fixture(scope="module")
def arch():
    """`(снапшот, запрос, вход полосы)` свода с полосой у переднего края; целый свод строится один раз на модуль."""

    return factories.band_domain(factories.arch_intrados(**ARCH), reach_cap=CAP)


@pytest.fixture(scope="module")
def dome():
    return factories.band_domain(factories.dome_rim(), reach_cap=CAP)


@pytest.fixture(scope="module")
def materialized_arch(arch):
    snapshot, request, _ = arch
    prepared = prepare_conveyor(snapshot, request)
    assert prepared.outcome.value == "EXACT", prepared.detail
    coverage = conveyor_coverage(prepared)
    assert coverage.outcome.value == "EXACT", coverage.detail
    result = materialize_domain(
        prepared,
        coverage,
        request=materialization_request(prepared, uv_policy_id="UV_DIRECT_STRIP_V1"),
        near_planar_lift_law=NearPlanarLiftLawV1.SOURCE_TRIANGLES_V1,
    )
    return result, prepared


# --------------------------------------------------------------------------
# Лестница: когда пробуется полоса
# --------------------------------------------------------------------------


def test_the_arch_front_arc_materializes_through_a_band_chart(arch, materialized_arch):
    snapshot, request, band = arch
    result, _ = materialized_arch
    certificate = _certificate(snapshot)
    assert type(certificate) is DevelopableBandChartCertificateV1
    assert result.outcome.value == "MATERIALIZED", result.detail
    bound = Fraction(
        certificate.stretch.worst_band_squared_upper.numerator,
        certificate.stretch.worst_band_squared_upper.denominator,
    )
    # Цилиндр развёртывается изометрично: растяжение - только шум привязки к решётке.
    assert 1 <= bound <= 1 + Fraction(1, 100)
    assert certificate.excluded_triangle_count > 0
    # Грани вне носителя названы в батче, а не молча отброшены: число и первая грань - в диагностике.
    lines = [item for item in result.diagnostics if item.startswith("FACE_BEYOND_CHART_REACH")]
    assert len(lines) == 1
    assert f"excluded_triangles={certificate.excluded_triangle_count} " in lines[0]
    assert f"first={certificate.first_excluded_triangle_id.value} " in lines[0]
    assert certificate.reach_cap == ExactRationalV1(1, 2)
    assert certificate.selected_chain_use_ids == band.selected_chain_use_ids == request.selected_chain_use_ids
    # Запас на карте не меньше досягаемости: ВЛАСТЬ полосы.
    assert Fraction(
        certificate.chart_reach_margin_squared.numerator, certificate.chart_reach_margin_squared.denominator
    ) >= Fraction(1, 4)


def test_the_band_follows_the_whole_patch_refusal_and_names_it(arch):
    certificate = _certificate(arch[0])
    assert certificate.previous_refusals[0] == "NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED"
    assert certificate.previous_refusals[1] in {item.value for item in BAND_TRIGGER_OUTCOMES}
    sides = certificate.strip_boundary
    assert {item.role for item in sides} == set(BandBoundaryRoleV1)
    assert sum(item.role is BandBoundaryRoleV1.RIM for item in sides) == 8
    assert all((item.role is BandBoundaryRoleV1.REACH_WALL) == (item.chain_use_id is None) for item in sides)
    assert all(a.end_vertex_id == b.start_vertex_id for a, b in zip(sides, sides[1:] + sides[:1]))


def test_without_a_band_request_the_whole_patch_refusal_stands():
    with pytest.raises(PlanarMetricAdmissionError) as failure:
        factories.band_domain(factories.arch_intrados(**ARCH))
    assert failure.value.outcome in BAND_TRIGGER_OUTCOMES
    assert "whole-patch" not in str(failure.value)


def test_a_whole_patch_that_unfolds_never_reaches_the_band_and_keeps_its_bytes():
    """Принятый целый патч (полуцилиндр без апсиды) с названной полосой и без неё - побитово одна и та же метрика."""

    parts = factories.arch_intrados(cols=8, barrel_rows=8, apse_rows=1, apse_degrees=0.001)
    with_band, _, _ = factories.band_domain(parts, reach_cap=CAP)
    without, _, _ = factories.band_domain(parts)
    certificate = _certificate(without)
    assert type(certificate) is DevelopableUnfoldCertificateV1
    assert canonical_json_bytes(with_band) == canonical_json_bytes(without)


def test_the_band_is_only_a_proposal_and_the_margin_is_the_authority(monkeypatch):
    """Носитель уже досягаемости: стена ближе `cap` - именованный отказ, а не усечённая карта."""

    original = _band_support.band_support
    monkeypatch.setattr(
        _band_chart,
        "band_support",
        lambda triangles, snapped, rim, reach: original(triangles, snapped, rim, Fraction(1, 10)),
    )
    with pytest.raises(PlanarMetricAdmissionError) as failure:
        factories.band_domain(factories.arch_intrados(**ARCH), reach_cap=CAP)
    assert failure.value.outcome is NamedOutcome.CHART_REACH_SHORT_OF_CAP
    assert "after the whole-patch unfolding" in str(failure.value)


def test_a_ring_support_stays_periodic_cut_required():
    with pytest.raises(PlanarMetricAdmissionError) as failure:
        factories.band_domain(factories.column_top(), reach_cap=CAP)
    assert failure.value.outcome is NamedOutcome.PERIODIC_CUT_REQUIRED
    assert "after the whole-patch unfolding PERIODIC_CUT_REQUIRED" in str(failure.value)


def test_a_band_preparation_survives_the_pool_pickle_and_covers_after_the_trip(arch):
    """Подготовка полосы едет в воркер пула пиклом: покрытие после поездки тем же ответом."""

    import pickle

    snapshot, request, _ = arch
    prepared = prepare_conveyor(snapshot, request)
    shipped = pickle.loads(pickle.dumps(prepared))
    local, remote = conveyor_coverage(prepared), conveyor_coverage(shipped)
    assert remote.outcome.value == local.outcome.value == "EXACT"
    assert remote.doubled_area == local.doubled_area


def test_a_dome_with_an_open_rim_chain_materializes(dome):
    snapshot, request, _ = dome
    certificate = _certificate(snapshot)
    assert type(certificate) is DevelopableBandChartCertificateV1
    prepared = prepare_conveyor(snapshot, request)
    assert prepared.outcome.value == "EXACT", prepared.detail
    coverage = conveyor_coverage(prepared)
    assert coverage.outcome.value == "EXACT", coverage.detail
    result = materialize_domain(
        prepared,
        coverage,
        request=materialization_request(prepared, uv_policy_id="UV_DIRECT_STRIP_V1"),
        near_planar_lift_law=NearPlanarLiftLawV1.SOURCE_TRIANGLES_V1,
    )
    assert result.outcome.value == "MATERIALIZED", result.detail
    # Купол недевелопабелен: растяжение здесь настоящее, но в бюджете запроса (по умолчанию 1/5).
    bound = Fraction(
        certificate.stretch.worst_band_squared_upper.numerator,
        certificate.stretch.worst_band_squared_upper.denominator,
    )
    assert 1 < bound <= Fraction(36, 25)


# --------------------------------------------------------------------------
# Валидатор не верит записи
# --------------------------------------------------------------------------


def _forged(snapshot, **changes):
    metric = next(iter(snapshot.surface_metric_descriptors))
    certificate = dataclasses.replace(metric.planarity_certificate, **changes)
    return dataclasses.replace(
        snapshot,
        surface_metric_descriptors=frozenset({dataclasses.replace(metric, planarity_certificate=certificate)}),
    )


def test_the_snapshot_validates_and_round_trips_through_the_codec(arch):
    snapshot, request, _ = arch
    assert kernel.validate_analysis_snapshot(snapshot) == ()
    assert kernel.validate_snapshot_request_references(snapshot, request) == ()
    payload = kernel.AnalysisSnapshotCodecV1.dumps(snapshot)
    decoded = kernel.AnalysisSnapshotCodecV1.loads(payload)
    assert decoded == snapshot
    assert kernel.AnalysisSnapshotCodecV1.dumps(decoded) == payload
    assert type(_certificate(decoded)) is DevelopableBandChartCertificateV1


@pytest.mark.parametrize(
    "field,value",
    [
        ("excluded_triangle_count", None),
        ("support_reach", ExactRationalV1(7, 10)),
        ("chart_reach_margin_squared", ExactRationalV1(9, 1)),
        ("chart_scale_trials", None),
    ],
)
def test_the_validator_catches_a_forged_band_certificate(arch, field, value):
    snapshot, _, _ = arch
    certificate = _certificate(snapshot)
    if field == "excluded_triangle_count":
        value = certificate.excluded_triangle_count + 1
    if field == "chart_scale_trials":
        value = 2 if certificate.chart_scale_trials != 2 else 3
    issues = kernel.validate_analysis_snapshot(_forged(snapshot, **{field: value}))
    assert any(item.code is ValidationCode.SURFACE_METRIC for item in issues), field


def test_a_forged_support_or_rim_selection_is_caught_by_recomputation(arch):
    snapshot, _, _ = arch
    certificate = _certificate(snapshot)
    smaller = frozenset(sorted(certificate.support_triangle_ids, key=lambda item: item.value)[1:])
    issues = kernel.validate_analysis_snapshot(_forged(snapshot, support_triangle_ids=smaller))
    assert any(item.code is ValidationCode.SURFACE_METRIC for item in issues)


def test_the_request_policy_binds_the_band_cap_selection_and_alpha(arch):
    snapshot, request, _ = arch
    other_cap = dataclasses.replace(request, chart_reach_cap=ExactRationalV1(3, 4))
    assert any(
        item.code is ValidationCode.POLICY_MISMATCH and item.path[-1] == "reach_cap"
        for item in kernel.validate_snapshot_request_references(snapshot, other_cap)
    )
    one_chain = dataclasses.replace(
        request, selected_chain_use_ids=frozenset(sorted(request.selected_chain_use_ids, key=lambda i: i.value)[:1])
    )
    assert any(
        item.path[-1] == "selected_chain_use_ids"
        for item in kernel.validate_snapshot_request_references(snapshot, one_chain)
    )
    unlawful = dataclasses.replace(request, chart_reach_cap=ExactRationalV1(0, 1))
    assert any(item.path == ("chart_reach_cap",) for item in kernel.validate_decal_request(unlawful))


def test_an_alpha_beyond_the_reach_cap_is_a_named_refusal(arch):
    snapshot, request, _ = arch
    wide = dataclasses.replace(request, requested_alpha=kernel.LocalLengthV1(request.requested_alpha.value * 3))
    issues = kernel.validate_snapshot_request_references(snapshot, wide)
    assert any("REQUEST_ALPHA_EXCEEDS_CHART_REACH" in item.message for item in issues)
    prepared = prepare_conveyor(snapshot, request)
    coverage = conveyor_coverage(prepared, "0.75")
    assert coverage.outcome.value == "REQUEST_ALPHA_EXCEEDS_CHART_REACH"
    assert "0.75" in coverage.detail and "0.5" in coverage.detail
    assert conveyor_coverage(prepared, "0.5").outcome.value == "EXACT"


# --------------------------------------------------------------------------
# Контракты и входы
# --------------------------------------------------------------------------


def test_the_band_certificate_repeats_the_unfold_fields_in_order_and_adds_its_own():
    unfold = [item.name for item in dataclasses.fields(DevelopableUnfoldCertificateV1)]
    band = [item.name for item in dataclasses.fields(DevelopableBandChartCertificateV1)]
    assert band[: len(unfold)] == unfold
    assert is_unfolded_certificate.__doc__
    hints_unfold = DevelopableUnfoldCertificateV1.__annotations__
    hints_band = DevelopableBandChartCertificateV1.__annotations__
    assert all(hints_unfold[name] == hints_band[name] for name in unfold)
    assert type(NearPlanarProjectionCertificateV1) is type


def test_the_request_omits_the_default_reach_cap_on_the_wire(arch):
    _, request, _ = arch
    default = kernel.DecalRequestCodecV1.dumps(request)
    assert b"chart_reach_cap" not in default
    wide = dataclasses.replace(request, chart_reach_cap=ExactRationalV1(3, 4))
    assert b"chart_reach_cap" in kernel.DecalRequestCodecV1.dumps(wide)
    assert kernel.DecalRequestCodecV1.loads(kernel.DecalRequestCodecV1.dumps(wide)) == wide
    assert isinstance(request, DecalRequestV1)
    assert chart_reach_cap_is_lawful(Fraction(1, 2)) and not chart_reach_cap_is_lawful(Fraction(0))


def test_the_band_request_lists_directed_rim_and_boundary_edges(arch):
    snapshot, request, band = arch
    domain = next(iter(snapshot.patch_domains))
    rebuilt = chart_band_request(
        snapshot.physical_chains, snapshot.chain_uses, request.selected_chain_use_ids, domain.patch_domain_id, CAP
    )
    assert rebuilt == band
    assert len(band.rim_edges) == 8 and len(band.boundary_uses) > len(band.rim_edges)
    chain_use = next(item for item in snapshot.chain_uses if item.chain_use_id in band.selected_chain_use_ids)
    chain = next(item for item in snapshot.physical_chains if item.physical_chain_id == chain_use.physical_chain_id)
    forward = directed_use_edges(chain, chain_use)
    backward = directed_use_edges(chain, dataclasses.replace(chain_use, orientation=ChainUseOrientation.B_START_TO_END))
    assert [(a, b) for a, b, _ in forward] == [(b, a) for a, b, _ in reversed(backward)]
    assert chart_band_request(
        snapshot.physical_chains, snapshot.chain_uses, frozenset(), domain.patch_domain_id, CAP
    ) is None


# --------------------------------------------------------------------------
# Носитель
# --------------------------------------------------------------------------


def _grid_patch(columns, rows):
    """Клетки-грани единичной сетки: `(треугольники, позиции)`; клетка `(i, j)` - две треугольника в одной грани."""

    from cftuv_envelope.contracts.surface import SurfaceTriangleV1
    from cftuv_envelope.ids import SourceFaceId, SourceVertexId, SurfaceTriangleId
    from cftuv_envelope.numeric import LocalVector3V1

    def vertex(i, j):
        return SourceVertexId(f"v{i}_{j}")

    triangles = []
    for i in columns:
        for j in rows:
            face = SourceFaceId(f"f{i}_{j}")
            corners = (vertex(i, j), vertex(i + 1, j), vertex(i + 1, j + 1), vertex(i, j + 1))
            for index, order in enumerate(((0, 1, 2), (0, 2, 3))):
                triangles.append(
                    SurfaceTriangleV1(
                        SurfaceTriangleId(f"t{i}_{j}_{index}"),
                        face,
                        tuple(corners[item] for item in order),
                        (None, None, None),
                        LocalVector3V1(0.0, 0.0, 1.0),
                    )
                )
    snapped = {
        vertex(i, j): (Fraction(i), Fraction(j), Fraction(0))
        for i in range(min(columns), max(columns) + 2)
        for j in range(min(rows), max(rows) + 2)
    }
    return tuple(triangles), snapped, vertex


def test_a_pinched_vertex_of_the_support_is_healed_by_its_whole_fan(monkeypatch):
    """Две принятые клетки касаются в одной вершине, клетки между ними отброшены: клетки веера добавляются.

    Включённые клетки идут `C`-образно вокруг клетки `(0, 1)`: северо-восточная `(1, 1)` и юго-западная `(0, 0)` в вершине
    `(1, 1)` соприкасаются только углом, а клетки `(0, 1)` и `(1, 0)` между ними вне носителя. Близкими объявлены
    вершины, не принадлежащие этим двум клеткам: выбор носителя - предложение, и подсказка (`_near_vertices`) подменена.
    """

    triangles, snapped, vertex = _grid_patch(range(-2, 3), range(0, 4))
    near = {vertex(i, j) for i, j in ((2, 2), (2, 3), (0, 3), (-1, 3), (-1, 1), (-1, 0), (0, 0))}
    monkeypatch.setattr(_band_support, "_near_vertices", lambda snapped_, segments, reach: near)
    support = _band_support.band_support(triangles, snapped, ((vertex(0, 0), vertex(1, 0)),), Fraction(1))
    faces = {item.source_face_id.value for item in support.triangles}
    assert support.pinch_closure_rounds == 1
    assert {"f0_1", "f1_0"} <= faces
    by_id = {item.triangle_id: item for item in triangles}
    pairs = _band_support._pair_map(triangles)
    included = {item.triangle_id for item in support.triangles}
    assert _band_support._pinched_vertices(included, by_id, pairs) == []
    assert support.excluded_count == len(triangles) - len(included)
    # Без подмены защемление есть: две клетки вершины `(1, 1)` без клеток между ними.
    chosen = {item.triangle_id for item in triangles if item.source_face_id.value in {"f1_1", "f0_0"}}
    assert _band_support._pinched_vertices(chosen, by_id, pairs) == [vertex(1, 1)]
