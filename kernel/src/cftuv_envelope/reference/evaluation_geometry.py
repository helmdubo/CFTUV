"""Единственная chart-lattice evaluation-геометрия Reference и QUEUE."""

from __future__ import annotations

from fractions import Fraction

from ..contracts.analysis import PlanarPatchFrameV1
from ..contracts.envelopes import (
    AngularEnvelopeSpec,
    CertifiedBoundHiddenSupportSpecV1,
)
from ..contracts.metric import (
    ExactPoint2V1,
    ExactRationalV1,
    RationalAffinePlanarMetricV2,
)
from ..contracts.plan import (
    EVALUATION_GEOMETRY_BINDING_SCHEMA_V1,
    EvaluationGeometryBindingLawV1,
    EvaluationGeometryBindingV1,
    EvaluationGeometrySourceVertexV1,
)
from ..robust.grid import GridSpecV1, snap_value
from ..validation import validate_evaluation_geometry_binding
from .contracts import ReferenceEnvelopeCompilationV1
from .metric import ExactPlanarMetric


class EvaluationGeometryBindingInvalid(ValueError):
    pass


def chart_lattice_for_frame(
    frame: PlanarPatchFrameV1 | RationalAffinePlanarMetricV2,
) -> GridSpecV1 | None:
    """Вывести chart lattice только из записанного сертификата метрики."""

    certificate = getattr(frame, "grid_certificate", None)
    if certificate is None or certificate.window_step is None:
        return None
    from ..source_grid import chart_grid_for

    step = Fraction(
        certificate.window_step.numerator,
        certificate.window_step.denominator,
    )
    return chart_grid_for(ExactPlanarMetric.from_descriptor(frame).gram, step)


def _exact_rational(value: Fraction) -> ExactRationalV1:
    value = Fraction(value)
    return ExactRationalV1(value.numerator, value.denominator)


def _bound_coordinate(coordinate, lattice: GridSpecV1) -> ExactPoint2V1:
    x = Fraction(coordinate.x.numerator, coordinate.x.denominator)
    y = Fraction(coordinate.y.numerator, coordinate.y.denominator)
    bound_x = Fraction(snap_value(x, lattice), lattice.scale)
    bound_y = Fraction(snap_value(y, lattice), lattice.scale)
    return ExactPoint2V1(_exact_rational(bound_x), _exact_rational(bound_y))


def build_evaluation_geometry_binding(
    compilation: ReferenceEnvelopeCompilationV1,
    frame: PlanarPatchFrameV1 | RationalAffinePlanarMetricV2,
) -> EvaluationGeometryBindingV1 | None:
    """Один раз привязать все chart points до запуска любого evaluator'а."""

    lattice = chart_lattice_for_frame(frame)
    if lattice is None:
        return None
    if not isinstance(frame, RationalAffinePlanarMetricV2):
        raise EvaluationGeometryBindingInvalid(
            "chart lattice requires RationalAffinePlanarMetricV2"
        )
    binding = EvaluationGeometryBindingV1(
        schema_version=EVALUATION_GEOMETRY_BINDING_SCHEMA_V1,
        source_revision=compilation.source_revision,
        patch_domain_id=compilation.plan_key.patch_domain_id,
        reference_metric_id=frame.reference_metric_id,
        binding_law=(
            EvaluationGeometryBindingLawV1.EVALUATION_GEOMETRY_CHART_LATTICE_BOUND_V1
        ),
        lattice_scale=lattice.scale,
        source_vertex_coordinates=frozenset(
            EvaluationGeometrySourceVertexV1(
                item.source_vertex_id,
                _bound_coordinate(item.domain_coordinate, lattice),
            )
            for item in frame.exact_source_vertex_coordinates
        ),
        bound_hidden_support_ids=frozenset(),
    )
    verify_evaluation_geometry_binding(
        binding,
        compilation,
        frame,
        require_certified_bound_supports=False,
    )
    return binding


def verify_evaluation_geometry_binding(
    binding: EvaluationGeometryBindingV1 | None,
    compilation: ReferenceEnvelopeCompilationV1,
    frame: PlanarPatchFrameV1 | RationalAffinePlanarMetricV2,
    *,
    require_certified_bound_supports: bool = True,
) -> None:
    """Перепроверить закон, identity и каждую bound coordinate fail-closed."""

    lattice = chart_lattice_for_frame(frame)
    if lattice is None:
        if binding is not None:
            raise EvaluationGeometryBindingInvalid(
                "binding is forbidden when chart lattice is undeclared"
            )
        return
    if binding is None or not isinstance(frame, RationalAffinePlanarMetricV2):
        raise EvaluationGeometryBindingInvalid(
            "declared chart lattice requires evaluation geometry binding"
        )
    issues = validate_evaluation_geometry_binding(binding)
    expected = frozenset(
        EvaluationGeometrySourceVertexV1(
            item.source_vertex_id,
            _bound_coordinate(item.domain_coordinate, lattice),
        )
        for item in frame.exact_source_vertex_coordinates
    )
    metadata_matches = (
        binding.source_revision == compilation.source_revision
        and binding.patch_domain_id == compilation.plan_key.patch_domain_id
        and binding.reference_metric_id == frame.reference_metric_id
        and binding.lattice_scale == lattice.scale
    )
    angular_supports = tuple(
        support
        for spec in compilation.envelope_specs
        if isinstance(spec, AngularEnvelopeSpec)
        for support in spec.hidden_supports
    )
    expected_support_ids = frozenset(
        support.hidden_support_id
        for support in angular_supports
        if type(support) is CertifiedBoundHiddenSupportSpecV1
    )
    support_authority_matches = (
        binding.bound_hidden_support_ids == expected_support_ids
        and (
            not require_certified_bound_supports
            or all(
                type(support) is CertifiedBoundHiddenSupportSpecV1
                for support in angular_supports
                if support.hidden_support_id
                in binding.bound_hidden_support_ids
            )
        )
    )
    if (
        issues
        or not metadata_matches
        or binding.source_vertex_coordinates != expected
        or not support_authority_matches
    ):
        raise EvaluationGeometryBindingInvalid(
            "evaluation geometry binding does not match its plan and metric"
        )
