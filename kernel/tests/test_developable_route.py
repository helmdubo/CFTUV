"""DEVELOPABLE (S1), срез C2: лестница метрики EXACT -> NEAR_PLANAR -> DEVELOPABLE и валидатор.

Развёртка пробуется ТОЛЬКО после именованного отказа near-planar по ширине, перевороту
либо вложению проекции; принятый домен не перемаршрутизируется (его байты прежние). Валидатор
не верит записи: карта и сертификат строятся заново из треугольников и сравниваются на
равенство, а красные контроли показывают, что подмена любого поля ловится.
"""

from __future__ import annotations

import math
from dataclasses import replace
from fractions import Fraction

import pytest

import cftuv_envelope as kernel
from cftuv_envelope.codec import canonical_json_bytes
from cftuv_envelope.contracts.metric import (
    CurvatureLadderPolicyV1,
    DevelopableUnfoldCertificateV1,
    ExactRationalV1,
    GridSnappingLawV1,
    NearPlanarFramePolicyV1,
    NearPlanarLiftLawV1,
    NearPlanarProjectionCertificateV1,
    PlanarityAdmissionLawV1,
)
from cftuv_envelope.outcomes import NamedOutcome
from cftuv_envelope.planar_metric import PlanarMetricAdmissionError
from cftuv_envelope.validation_metric import (
    validate_embedding_certified_rational_affine_planar_metric,
    validate_rational_affine_planar_metric,
)

import developable_factories as factories
from developable_factories import DOMAIN, PATCH, REVISION
from developable_route import build_metric, developable_domain

OFF = CurvatureLadderPolicyV1.NEAR_PLANAR_ONLY_V1
ON = CurvatureLadderPolicyV1.NEAR_PLANAR_THEN_DEVELOPABLE_UNFOLD_V1


def _refusal(parts, **overrides) -> PlanarMetricAdmissionError:
    with pytest.raises(PlanarMetricAdmissionError) as failure:
        build_metric(parts, **overrides)
    return failure.value


def _issues(record, parts, **options):
    vertices, faces, triangles = parts
    return validate_embedding_certified_rational_affine_planar_metric(
        record,
        source_vertices=vertices,
        source_faces=faces,
        owner_patch_id=PATCH,
        expected_source_revision=REVISION,
        expected_patch_domain_id=DOMAIN,
        expected_source_lineage=frozenset(),
        surface_triangles=triangles,
        **options,
    )


# --------------------------------------------------------------------------
# Лестница: когда пробуется развёртка
# --------------------------------------------------------------------------


def test_the_ladder_is_off_by_default_and_the_near_planar_refusal_stands():
    error = _refusal(factories.fold_strip(), ladder=OFF)
    assert error.outcome is NamedOutcome.NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED


def test_with_the_ladder_a_width_refusal_is_followed_by_an_unfold():
    record = build_metric(factories.fold_strip(), ladder=ON)
    certificate = record.metric.planarity_certificate
    assert type(certificate) is DevelopableUnfoldCertificateV1
    assert certificate.previous_refusals == (
        "NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED",
    )
    assert record.near_planar_projection_embedding_certificate is None
    assert record.metric.frame_selection_law.value == "UNFOLDED_DEVELOPMENT_FRAME_V1"
    assert record.metric.chart_orientation.value == "COORDINATE_CCW_MATCHES_OWNER_PATCH"


def test_the_unfolded_frame_is_an_ordinary_affine_metric_over_the_chart_plane():
    from fractions import Fraction

    record = build_metric(factories.bevel_strip(4), ladder=ON)
    metric = record.metric
    scale = metric.planarity_certificate.chart_scale
    unit = Fraction(1, scale)
    point = lambda value: tuple(  # noqa: E731
        Fraction(axis.numerator, axis.denominator) for axis in (value.x, value.y, value.z)
    )
    assert point(metric.exact_origin) == (0, 0, 0)
    assert point(metric.exact_basis_a) == (unit, 0, 0)
    assert point(metric.exact_basis_b) == (0, unit, 0)
    gram = metric.exact_gram_matrix
    assert (Fraction(gram.m00.numerator, gram.m00.denominator), gram.m01.numerator) == (
        unit * unit,
        0,
    )
    for item in metric.exact_source_vertex_coordinates:
        assert item.domain_coordinate.x.denominator == 1
        assert item.domain_coordinate.y.denominator == 1


@pytest.mark.parametrize(
    "parts",
    (
        lambda: factories.surface(
            {"a": (0.0, 0.0, 0.0), "b": (2.0, 0.0, 0.0), "c": (2.0, 1.0, 0.0), "d": (0.0, 1.0, 0.0)},
            [["a", "b", "c", "d"]],
        ),
        lambda: factories.bevel_strip(2, step_degrees=5.0),
    ),
    ids=("exact-plane", "gentle-bevel-accepted-by-near-planar"),
)
def test_an_accepted_domain_is_not_rerouted_bitwise(parts):
    """Принятый ниже по лестнице домен получает те же байты при включённой лестнице."""

    without = build_metric(parts(), ladder=OFF)
    with_ladder = build_metric(parts(), ladder=ON)
    assert canonical_json_bytes(without) == canonical_json_bytes(with_ladder)
    assert type(with_ladder.metric.planarity_certificate) is not DevelopableUnfoldCertificateV1


def test_a_near_planar_domain_accepted_by_width_keeps_its_certificate_type():
    record = build_metric(factories.bevel_strip(2, step_degrees=5.0), ladder=ON)
    assert type(record.metric.planarity_certificate) is NearPlanarProjectionCertificateV1


def test_a_refusal_the_unfold_cannot_cure_is_not_laddered():
    """Невязка плоскости под укладкой на плоскость — не триггер: остаётся как есть."""

    error = _refusal(
        factories.quarter_cylinder(4),
        ladder=ON,
        near_planar_lift_law=NearPlanarLiftLawV1.CERTIFIED_PLANE_V1,
    )
    assert error.outcome is NamedOutcome.NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED


def test_without_surface_triangles_the_original_refusal_stands():
    error = _refusal(
        factories.fold_strip(),
        ladder=ON,
        surface_triangles=None,
        near_planar_lift_law=NearPlanarLiftLawV1.SOURCE_TRIANGLES_V1,
    )
    assert error.outcome is NamedOutcome.NEAR_PLANAR_OWNER_SURFACE_TRIANGLES_UNAVAILABLE


def test_an_embedding_refusal_is_a_trigger_and_the_spiral_overlap_is_named():
    error = _refusal(factories.spiral_strip(), ladder=ON)
    assert error.outcome is NamedOutcome.DEVELOPABLE_CHART_SELF_OVERLAP
    assert "after near-planar NEAR_PLANAR_PROJECTION_" in str(error)


def test_the_final_refusal_names_both_rungs():
    error = _refusal(factories.cone(8, rise=1.0, boundary_apex=False), ladder=ON)
    assert error.outcome is NamedOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED
    assert "worst_vertex=v:apex" in str(error)
    assert "after near-planar NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED" in str(error)


def test_an_unsnapped_source_is_named_after_the_near_planar_refusal():
    error = _refusal(
        factories.fold_strip(),
        ladder=ON,
        grid_policy=GridSnappingLawV1.UNSNAPPED_EXACT_V1,
        near_planar_frame_policy=NearPlanarFramePolicyV1.CANONICAL_ONLY_V1,
    )
    assert error.outcome is NamedOutcome.DEVELOPABLE_REQUIRES_SOURCE_SNAP
    assert "after near-planar NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED" in str(error)


def test_a_closed_column_has_no_patch_plane_and_the_ladder_names_the_ring():
    """Нулевая нормаль Ньюэлла (замкнутая колонна): без лестницы `ValueError`, как был."""

    from cftuv_envelope.numeric import PlaneNormalUndefinedError

    with pytest.raises(PlaneNormalUndefinedError):
        build_metric(factories.closed_cylinder(), ladder=OFF)
    error = _refusal(factories.closed_cylinder(), ladder=ON)
    assert error.outcome is NamedOutcome.PERIODIC_CUT_REQUIRED
    assert "after near-planar PATCH_PLANE_NORMAL_UNDEFINED" in str(error)


def test_a_conical_ring_is_named_through_the_width_refusal():
    """Кольцо с ненулевой нормалью: near-planar отказывает по ширине, развёртка — кольцо."""

    points, cycles = {}, []
    sides = 8
    for k in range(sides):
        theta = 2.0 * math.pi * k / sides
        points[f"a{k}"] = (math.cos(theta), math.sin(theta), 0.0)
        points[f"b{k}"] = (0.5 * math.cos(theta), 0.5 * math.sin(theta), 1.0)
    for k in range(sides):
        n = (k + 1) % sides
        cycles.append([f"a{k}", f"a{n}", f"b{n}", f"b{k}"])
    error = _refusal(factories.surface(points, cycles), ladder=ON)
    assert error.outcome is NamedOutcome.PERIODIC_CUT_REQUIRED


def test_the_public_metric_builder_returns_the_same_chart_as_the_direct_chart():
    from developable_factories import developable_chart

    parts = factories.quarter_cylinder(8)
    record = build_metric(parts, ladder=ON)
    direct = developable_chart(parts)
    assert record.metric.planarity_certificate.stretch == direct.certificate.stretch
    assert {
        item.source_vertex_id: (
            item.domain_coordinate.x.numerator,
            item.domain_coordinate.y.numerator,
        )
        for item in record.metric.exact_source_vertex_coordinates
    } == direct.nodes


# --------------------------------------------------------------------------
# Валидатор: пересчёт и красные контроли
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    "parts",
    (factories.fold_strip, factories.fold_grid, factories.quarter_cylinder),
    ids=("fold", "fold-grid", "quarter-cylinder"),
)
def test_the_validator_accepts_what_the_builder_wrote(parts):
    parts = parts()
    record = build_metric(parts, ladder=ON)
    assert validate_rational_affine_planar_metric(record.metric) == ()
    assert _issues(record, parts) == ()


def _tampered(record, **changes):
    certificate = replace(record.metric.planarity_certificate, **changes)
    return replace(record, metric=replace(record.metric, planarity_certificate=certificate))


def test_the_validator_catches_a_moved_chart_node():
    parts = factories.fold_strip()
    record = build_metric(parts, ladder=ON)
    coordinates = sorted(
        record.metric.exact_source_vertex_coordinates,
        key=lambda item: item.source_vertex_id.value,
    )
    moved = replace(
        coordinates[1],
        domain_coordinate=replace(
            coordinates[1].domain_coordinate,
            x=ExactRationalV1(coordinates[1].domain_coordinate.x.numerator + 1, 1),
        ),
    )
    forged = replace(
        record,
        metric=replace(
            record.metric,
            exact_source_vertex_coordinates=frozenset(
                [moved, *(item for item in coordinates if item is not coordinates[1])]
            ),
        ),
    )
    assert any("unfolded chart coordinates differ" in item.message for item in _issues(forged, parts))


def test_the_validator_catches_a_swapped_frame_law():
    from cftuv_envelope.contracts.metric import AffineFrameSelectionLawV1

    parts = factories.fold_strip()
    record = build_metric(parts, ladder=ON)
    forged = replace(
        record,
        metric=replace(
            record.metric,
            frame_selection_law=AffineFrameSelectionLawV1.CANONICAL_SOURCE_VERTEX_BASIS_V1,
        ),
    )
    messages = [item.message for item in _issues(forged, parts)]
    assert any("unfolded development frame" in message for message in messages)


def test_the_validator_catches_a_forged_stretch_record():
    parts = factories.bevel_strip(3)
    record = build_metric(parts, ladder=ON)
    stretch = record.metric.planarity_certificate.stretch
    hidden = replace(
        stretch,
        worst_band_squared_upper=ExactRationalV1(1, 1),
    )
    messages = [item.message for item in _issues(_tampered(record, stretch=hidden), parts)]
    assert "developable certificate differs from exact recomputation" in messages


def test_the_validator_catches_an_admitted_certificate_that_the_judge_would_refuse():
    parts = factories.fold_strip()
    record = build_metric(parts, ladder=ON)
    stretch = record.metric.planarity_certificate.stretch
    liar = replace(
        stretch,
        triangles_outside_budget=1,
        first_outside_triangle_id=stretch.worst_triangle_id,
    )
    messages = [item.message for item in _issues(_tampered(record, stretch=liar), parts)]
    assert any("DEVELOPABLE_STRETCH_BUDGET_EXCEEDED" in message for message in messages)


def test_the_validator_catches_a_forged_budget_or_chart_scale_or_trial_count():
    parts = factories.fold_strip()
    record = build_metric(parts, ladder=ON)
    certificate = record.metric.planarity_certificate
    budget = replace(certificate.stretch, stretch_budget=ExactRationalV1(1, 10))
    # Допуск — политика запроса: запись под чужим допуском ловится против ЗАПРОСА (1/5), а снапшот без запроса
    # судится под собственным записанным допуском и подделку допуска не видит (она закрыта связью с запросом).
    assert _issues(_tampered(record, stretch=budget), parts, developable_stretch_budget=Fraction(1, 5))
    assert not _issues(_tampered(record, stretch=budget), parts, developable_stretch_budget=Fraction(1, 10))
    for forged in (
        _tampered(record, chart_scale=certificate.chart_scale * 2),
        _tampered(record, chart_scale_trials=certificate.chart_scale_trials + 1),
        _tampered(record, previous_refusals=()),
        _tampered(record, snapped_source_positions=frozenset()),
    ):
        assert _issues(forged, parts), forged.metric.planarity_certificate


def test_the_validator_catches_a_certificate_id_that_is_not_the_deterministic_one():
    from cftuv_envelope.ids import PlanarityCertificateId

    parts = factories.fold_strip()
    record = build_metric(parts, ladder=ON)
    forged = _tampered(record, certificate_id=PlanarityCertificateId("developable-unfold:forged"))
    messages = [item.message for item in _issues(forged, parts)]
    assert any("certificate ID differs" in message for message in messages)


def test_the_validator_catches_a_developable_certificate_on_a_canonical_frame():
    """Закон репера и сертификат неразделимы: ни тот без другого."""

    from cftuv_envelope.contracts.metric import AffineFrameSelectionLawV1

    parts = factories.fold_strip()
    record = build_metric(parts, ladder=ON)
    forged = replace(
        record.metric,
        frame_selection_law=AffineFrameSelectionLawV1.REDUCED_INTEGER_PLANE_LATTICE_BASIS_V1,
    )
    assert validate_rational_affine_planar_metric(forged)


def test_the_unfolded_frame_law_on_a_near_planar_certificate_is_refused():
    from cftuv_envelope.contracts.metric import AffineFrameSelectionLawV1

    parts = factories.bevel_strip(2, step_degrees=5.0)
    record = build_metric(parts, ladder=ON)
    assert type(record.metric.planarity_certificate) is NearPlanarProjectionCertificateV1
    forged = replace(
        record.metric,
        frame_selection_law=AffineFrameSelectionLawV1.UNFOLDED_DEVELOPMENT_FRAME_V1,
    )
    assert any(
        "belongs to the developable certificate" in item.message
        for item in validate_rational_affine_planar_metric(forged)
    )


# --------------------------------------------------------------------------
# Снапшот: домен-развёртка проходит валидатор снапшота, провод и кодек
# --------------------------------------------------------------------------


def test_a_snapshot_with_an_unfolded_domain_validates_and_roundtrips():
    snapshot, _request = developable_domain(
        factories.quarter_cylinder(), ("r0a", "r0b"), alpha="0.8"
    )
    assert kernel.validate_analysis_snapshot(snapshot) == ()
    loaded = kernel.AnalysisSnapshotCodecV1.loads(
        kernel.AnalysisSnapshotCodecV1.dumps(snapshot)
    )
    assert loaded == snapshot


def test_a_snapshot_with_a_moved_source_vertex_fails_the_unfold_recomputation():
    snapshot, _request = developable_domain(
        factories.quarter_cylinder(), ("r0a", "r0b"), alpha="0.8"
    )
    vertices = sorted(snapshot.source_vertices, key=lambda item: item.vertex_id.value)
    moved = replace(
        vertices[3],
        position=replace(vertices[3].position, z=vertices[3].position.z + 0.01),
    )
    forged = replace(
        snapshot,
        source_vertices=frozenset([moved, *(item for item in vertices if item is not vertices[3])]),
    )
    messages = [item.message for item in kernel.validate_analysis_snapshot(forged)]
    assert any("differs from exact recomputation" in message or "cannot be recomputed" in message
               for message in messages)


def test_the_embedding_certified_record_roundtrips_through_its_codec():
    record = build_metric(factories.bevel_strip(3), ladder=ON)
    loaded = kernel.EmbeddingCertifiedRationalAffinePlanarMetricCodecV1.loads(
        kernel.EmbeddingCertifiedRationalAffinePlanarMetricCodecV1.dumps(record)
    )
    assert loaded == record


def test_the_chart_orientation_is_counter_clockwise_against_the_owner_patch():
    """Любая развёртка кладёт первый треугольник против часовой: обход владельца сохранён."""

    for parts in (factories.fold_strip(), factories.quarter_cylinder(8)):
        record = build_metric(parts, ladder=ON)
        coordinates = {
            item.source_vertex_id: (
                item.domain_coordinate.x.numerator,
                item.domain_coordinate.y.numerator,
            )
            for item in record.metric.exact_source_vertex_coordinates
        }
        for triangle in parts[2]:
            (ax, ay), (bx, by), (cx, cy) = (coordinates[v] for v in triangle.vertex_ids)
            assert (bx - ax) * (cy - ay) - (by - ay) * (cx - ax) > 0
