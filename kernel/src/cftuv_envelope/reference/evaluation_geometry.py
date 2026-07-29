"""Единственная chart-lattice evaluation-геометрия Reference и QUEUE."""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction
from math import gcd

from ..contracts.analysis import (
    PhysicalChainV1,
    PlanarPatchFrameV1,
)
from ..contracts.envelopes import (
    AngularEnvelopeSpec,
    CertifiedBoundHiddenSupportSpecV1,
)
from ..contracts.metric import (
    ExactPoint2V1,
    ExactRationalV1,
    ExactVector2V1,
    RationalAffinePlanarMetricV2,
)
from ..contracts.plan import (
    CHAIN_STRAIGHT_EVALUATION_GEOMETRY_BINDING_SCHEMA_V2,
    EVALUATION_GEOMETRY_BINDING_SCHEMA_V1,
    ChainStraightAssignmentDispositionV2,
    ChainStraightCapacityDeficitV2,
    ChainStraightEvaluationGeometryBindingLawV2,
    ChainStraightEvaluationGeometryBindingV2,
    ChainStraightInternalAssignmentV2,
    ChainStraightPhysicalChainBindingV2,
    ChainStraightVertexAuthorityRecordV2,
    ChainStraightVertexAuthorityV2,
    EvaluationGeometryBinding,
    EvaluationGeometryBindingLawV1,
    EvaluationGeometryBindingV1,
    EvaluationGeometrySourceVertexV1,
)
from ..ids import PhysicalChainId, SourceVertexId
from ..robust.grid import GridSpecV1, snap_value
from ..validation import (
    validate_chain_straight_evaluation_geometry_binding,
    validate_evaluation_geometry_binding,
)
from .contracts import ReferenceEnvelopeCompilationV1
from .metric import ExactPlanarMetric


MAX_CHAIN_STRAIGHT_REFINEMENT_POWER = 8


class EvaluationGeometryBindingInvalid(ValueError):
    pass


class EvaluationGeometryRefinementBudgetExhausted(
    EvaluationGeometryBindingInvalid
):
    pass


class SourceDeclaredStraightEndpointsCoincide(
    EvaluationGeometryBindingInvalid
):
    pass


class SourceDeclaredStraightChainIsNotLinear(
    EvaluationGeometryBindingInvalid
):
    pass


@dataclass(frozen=True, slots=True)
class _StraightChainInfo:
    chain: PhysicalChainV1
    source: tuple[tuple[Fraction, Fraction], ...]
    base_start_node: tuple[int, int]
    base_end_node: tuple[int, int]
    primitive_direction: tuple[int, int]
    base_endpoint_span: int


def chart_lattice_for_frame(
    frame: PlanarPatchFrameV1 | RationalAffinePlanarMetricV2,
) -> GridSpecV1 | None:
    """Вывести базовую chart lattice только из сертификата метрики."""

    certificate = getattr(frame, "grid_certificate", None)
    if certificate is None or certificate.window_step is None:
        return None
    from ..source_grid import chart_grid_for

    step = Fraction(
        certificate.window_step.numerator,
        certificate.window_step.denominator,
    )
    return chart_grid_for(ExactPlanarMetric.from_descriptor(frame).gram, step)


def evaluation_geometry_lattice(
    frame: PlanarPatchFrameV1 | RationalAffinePlanarMetricV2,
    binding: EvaluationGeometryBinding | None,
) -> GridSpecV1 | None:
    """Вернуть S или записанный S' без повторного вывода масштаба из frame."""

    base_lattice = chart_lattice_for_frame(frame)
    if base_lattice is None:
        return None
    if isinstance(binding, ChainStraightEvaluationGeometryBindingV2):
        if binding.base_lattice_scale != base_lattice.scale:
            raise EvaluationGeometryBindingInvalid(
                "V2 base lattice scale does not match its metric"
            )
        return GridSpecV1(
            binding.lattice_scale,
            base_lattice.magnitude_bound,
        )
    return base_lattice


def _exact_rational(value: Fraction | int) -> ExactRationalV1:
    value = Fraction(value)
    return ExactRationalV1(value.numerator, value.denominator)


def _exact_point(value: tuple[Fraction, Fraction]) -> ExactPoint2V1:
    return ExactPoint2V1(
        _exact_rational(value[0]),
        _exact_rational(value[1]),
    )


def _exact_vector(value: tuple[Fraction, Fraction]) -> ExactVector2V1:
    return ExactVector2V1(
        _exact_rational(value[0]),
        _exact_rational(value[1]),
    )


def _bound_coordinate(coordinate, lattice: GridSpecV1) -> ExactPoint2V1:
    source = _fraction_pair(coordinate)
    return _exact_point(
        (
            Fraction(snap_value(source[0], lattice), lattice.scale),
            Fraction(snap_value(source[1], lattice), lattice.scale),
        )
    )


def _fraction_pair(coordinate: ExactPoint2V1) -> tuple[Fraction, Fraction]:
    return (
        Fraction(coordinate.x.numerator, coordinate.x.denominator),
        Fraction(coordinate.y.numerator, coordinate.y.denominator),
    )


def _gram_matrix(
    frame: RationalAffinePlanarMetricV2,
) -> tuple[
    tuple[Fraction, Fraction],
    tuple[Fraction, Fraction],
]:
    matrix = frame.exact_gram_matrix
    return (
        (
            _fraction(matrix.m00),
            _fraction(matrix.m01),
        ),
        (
            _fraction(matrix.m10),
            _fraction(matrix.m11),
        ),
    )


def _fraction(value: ExactRationalV1) -> Fraction:
    return Fraction(value.numerator, value.denominator)


def _subtract(left, right) -> tuple[Fraction, Fraction]:
    return left[0] - right[0], left[1] - right[1]


def _cross(left, right) -> Fraction:
    return left[0] * right[1] - left[1] * right[0]


def _gram_dot(left, right, gram) -> Fraction:
    return (
        left[0] * (gram[0][0] * right[0] + gram[0][1] * right[1])
        + left[1] * (gram[1][0] * right[0] + gram[1][1] * right[1])
    )


def _half_up(value: Fraction) -> int:
    lower = value.numerator // value.denominator
    twice_remainder = 2 * (
        value.numerator - lower * value.denominator
    )
    return lower + int(twice_remainder >= value.denominator)


def _projection(
    source_scaled: tuple[Fraction, Fraction],
    anchor: tuple[int, int],
    direction: tuple[int, int],
    gram,
) -> Fraction:
    offset = _subtract(source_scaled, anchor)
    denominator = _gram_dot(direction, direction, gram)
    if denominator <= 0:
        raise EvaluationGeometryBindingInvalid(
            "straight-chain direction has non-positive Gram norm"
        )
    return _gram_dot(direction, offset, gram) / denominator


def _declared_internal_corner_ids(snapshot, chain: PhysicalChainV1) -> set:
    internal_ids = set(chain.ordered_source_vertex_ids[1:-1])
    uses = tuple(
        item
        for item in snapshot.chain_uses
        if item.physical_chain_id == chain.physical_chain_id
    )
    if not uses:
        raise EvaluationGeometryBindingInvalid(
            f"{chain.physical_chain_id.value} has no ChainUse declaration"
        )
    sector_ids = {
        sector.owner_sector_id
        for sector in snapshot.angular_owner_sectors
        if any(
            use.chain_use_id in sector.ordered_incident_chain_use_ids
            for use in uses
        )
    }
    return {
        relation.source_vertex_id
        for relation in snapshot.corner_relations
        if relation.owner_sector_id in sector_ids
        and relation.source_vertex_id in internal_ids
    }


def _declared_straight_chains(
    compilation: ReferenceEnvelopeCompilationV1,
) -> tuple[PhysicalChainV1, ...]:
    snapshot = compilation.analysis_snapshot
    domain_chain_ids = {
        item.physical_chain_id
        for item in snapshot.chain_uses
        if item.patch_domain_id == compilation.plan_key.patch_domain_id
    }
    return tuple(
        chain
        for chain in sorted(
            snapshot.physical_chains,
            key=lambda item: item.physical_chain_id.value,
        )
        if chain.physical_chain_id in domain_chain_ids
        and not chain.is_closed
        and len(chain.ordered_source_vertex_ids) >= 3
        and not _declared_internal_corner_ids(snapshot, chain)
    )


def _base_nodes(
    frame: RationalAffinePlanarMetricV2,
    lattice: GridSpecV1,
) -> dict[SourceVertexId, tuple[int, int]]:
    return {
        item.source_vertex_id: (
            snap_value(_fraction(item.domain_coordinate.x), lattice),
            snap_value(_fraction(item.domain_coordinate.y), lattice),
        )
        for item in frame.exact_source_vertex_coordinates
    }


def _source_coordinates(
    frame: RationalAffinePlanarMetricV2,
) -> dict[SourceVertexId, tuple[Fraction, Fraction]]:
    return {
        item.source_vertex_id: _fraction_pair(item.domain_coordinate)
        for item in frame.exact_source_vertex_coordinates
    }


def _chain_info(
    chain: PhysicalChainV1,
    source_by_id: dict[SourceVertexId, tuple[Fraction, Fraction]],
    base_node_by_id: dict[SourceVertexId, tuple[int, int]],
) -> _StraightChainInfo:
    try:
        source = tuple(
            source_by_id[item] for item in chain.ordered_source_vertex_ids
        )
        start_node = base_node_by_id[chain.ordered_source_vertex_ids[0]]
        end_node = base_node_by_id[chain.ordered_source_vertex_ids[-1]]
    except KeyError as exc:
        raise EvaluationGeometryBindingInvalid(
            "declared straight chain references an unbound source vertex"
        ) from exc
    source_direction = _subtract(source[-1], source[0])
    if source_direction == (0, 0):
        raise SourceDeclaredStraightEndpointsCoincide(
            "SOURCE_DECLARED_STRAIGHT_ENDPOINTS_COINCIDE"
        )
    if any(
        _cross(source_direction, _subtract(point, source[0])) != 0
        for point in source
    ):
        raise SourceDeclaredStraightChainIsNotLinear(
            "SOURCE_DECLARED_STRAIGHT_CHAIN_IS_NOT_LINEAR"
        )
    delta = _subtract(end_node, start_node)
    endpoint_span = gcd(abs(delta[0]), abs(delta[1]))
    if endpoint_span == 0:
        raise EvaluationGeometryBindingInvalid(
            "declared straight chain base-bound endpoints coincide"
        )
    return _StraightChainInfo(
        chain=chain,
        source=source,
        base_start_node=start_node,
        base_end_node=end_node,
        primitive_direction=(
            delta[0] // endpoint_span,
            delta[1] // endpoint_span,
        ),
        base_endpoint_span=endpoint_span,
    )


def _minimum_refinement_power(
    chain_infos: tuple[_StraightChainInfo, ...],
) -> int:
    for refinement_power in range(
        MAX_CHAIN_STRAIGHT_REFINEMENT_POWER + 1
    ):
        factor = 1 << refinement_power
        if all(
            info.base_endpoint_span * factor
            >= len(info.chain.ordered_source_vertex_ids) - 1
            for info in chain_infos
        ):
            return refinement_power
    raise EvaluationGeometryRefinementBudgetExhausted(
        "REFINEMENT_BUDGET_EXHAUSTED"
    )


def _assignment_disposition(
    *,
    clamped: bool,
    gram_squared: Fraction,
    half_gram_squared: Fraction,
    longitudinal: Fraction,
) -> ChainStraightAssignmentDispositionV2:
    if not clamped:
        return (
            ChainStraightAssignmentDispositionV2.UNCLAMPED_WITHIN_HALF_STEP
        )
    if (
        gram_squared <= half_gram_squared
        and longitudinal <= Fraction(1, 2)
    ):
        return (
            ChainStraightAssignmentDispositionV2.CLAMPED_WITHIN_HALF_STEP
        )
    return (
        ChainStraightAssignmentDispositionV2.CLAMPED_CONSTRAINT_EXCESS_ALLOWED
    )


def _chain_binding(
    info: _StraightChainInfo,
    base_scale: int,
    refinement_power: int,
    gram,
) -> ChainStraightPhysicalChainBindingV2:
    factor = 1 << refinement_power
    scale = base_scale * factor
    anchor = tuple(item * factor for item in info.base_start_node)
    endpoint_k = info.base_endpoint_span * factor
    direction = info.primitive_direction
    half_gram_squared = (
        _gram_dot(direction, direction, gram)
        / (scale * scale)
        / 4
    )
    assignments = []
    previous_k = 0
    vertex_ids = info.chain.ordered_source_vertex_ids
    for ordinal, (vertex_id, source_point) in enumerate(
        zip(vertex_ids[1:-1], info.source[1:-1], strict=True),
        start=1,
    ):
        remaining = len(vertex_ids) - 1 - ordinal
        lower = previous_k + 1
        upper = endpoint_k - remaining
        projection = _projection(
            (
                source_point[0] * scale,
                source_point[1] * scale,
            ),
            anchor,
            direction,
            gram,
        )
        unconstrained = _half_up(projection)
        selected = min(max(unconstrained, lower), upper)
        clamped = selected != unconstrained
        assigned_node = (
            anchor[0] + selected * direction[0],
            anchor[1] + selected * direction[1],
        )
        assigned_coordinate = (
            Fraction(assigned_node[0], scale),
            Fraction(assigned_node[1], scale),
        )
        displacement = _subtract(assigned_coordinate, source_point)
        gram_squared = _gram_dot(displacement, displacement, gram)
        longitudinal = abs(Fraction(selected) - projection)
        assignments.append(
            ChainStraightInternalAssignmentV2(
                physical_chain_id=info.chain.physical_chain_id,
                ordinal=ordinal,
                source_vertex_id=vertex_id,
                lower_k=lower,
                upper_k=upper,
                projection_k_gram=_exact_rational(projection),
                unconstrained_canonical_k=unconstrained,
                selected_k=selected,
                clamped=clamped,
                disposition=_assignment_disposition(
                    clamped=clamped,
                    gram_squared=gram_squared,
                    half_gram_squared=half_gram_squared,
                    longitudinal=longitudinal,
                ),
                assigned_refined_node=assigned_node,
                exact_offset_from_source=_exact_vector(displacement),
                exact_gram_displacement_squared=_exact_rational(
                    gram_squared
                ),
                half_step_gram_squared_bound=_exact_rational(
                    half_gram_squared
                ),
                exact_longitudinal_displacement_k=_exact_rational(
                    longitudinal
                ),
                longitudinal_half_step_bound_k=_exact_rational(
                    Fraction(1, 2)
                ),
            )
        )
        previous_k = selected
    return ChainStraightPhysicalChainBindingV2(
        physical_chain_id=info.chain.physical_chain_id,
        ordered_source_vertex_ids=vertex_ids,
        primitive_direction=direction,
        base_start_node=info.base_start_node,
        base_end_node=info.base_end_node,
        base_endpoint_span_k=info.base_endpoint_span,
        refined_endpoint_span_k=endpoint_k,
        internal_assignments=tuple(assignments),
    )


def _v2_vertex_records(
    source_by_id: dict[SourceVertexId, tuple[Fraction, Fraction]],
    base_node_by_id: dict[SourceVertexId, tuple[int, int]],
    chain_bindings: tuple[ChainStraightPhysicalChainBindingV2, ...],
    base_lattice: GridSpecV1,
    factor: int,
    scale: int,
) -> tuple[
    list[ChainStraightVertexAuthorityRecordV2],
    list[EvaluationGeometrySourceVertexV1],
]:
    memberships: dict[SourceVertexId, set[PhysicalChainId]] = {}
    internal_assignments = {}
    for chain_binding in chain_bindings:
        for vertex_id in chain_binding.ordered_source_vertex_ids:
            memberships.setdefault(vertex_id, set()).add(
                chain_binding.physical_chain_id
            )
        for assignment in chain_binding.internal_assignments:
            if assignment.source_vertex_id in internal_assignments:
                raise EvaluationGeometryBindingInvalid(
                    "source vertex has multiple straight-chain internal authorities"
                )
            internal_assignments[assignment.source_vertex_id] = assignment
    for vertex_id in internal_assignments:
        if len(memberships[vertex_id]) != 1:
            raise EvaluationGeometryBindingInvalid(
                "straight-chain endpoint and internal authorities overlap"
            )

    authority_records = []
    coordinate_records = []
    for vertex_id, source in sorted(
        source_by_id.items(),
        key=lambda item: item[0].value,
    ):
        base_node = base_node_by_id[vertex_id]
        base_coordinate = (
            Fraction(base_node[0], base_lattice.scale),
            Fraction(base_node[1], base_lattice.scale),
        )
        assignment = internal_assignments.get(vertex_id)
        if assignment is not None:
            authority = (
                ChainStraightVertexAuthorityV2.CHAIN_STRAIGHT_INTERNAL_REFINED_V2
            )
            assigned_node = assignment.assigned_refined_node
            assigned_coordinate = (
                Fraction(assigned_node[0], scale),
                Fraction(assigned_node[1], scale),
            )
        else:
            authority = (
                ChainStraightVertexAuthorityV2.BASE_BOUND_ENDPOINT_V1
                if vertex_id in memberships
                else ChainStraightVertexAuthorityV2.BASE_BOUND_NON_CHAIN_V1
            )
            assigned_node = tuple(item * factor for item in base_node)
            assigned_coordinate = base_coordinate
        offset = _subtract(assigned_coordinate, source)
        exact_coordinate = _exact_point(assigned_coordinate)
        coordinate_records.append(
            EvaluationGeometrySourceVertexV1(
                vertex_id,
                exact_coordinate,
            )
        )
        authority_records.append(
            ChainStraightVertexAuthorityRecordV2(
                source_vertex_id=vertex_id,
                authority=authority,
                physical_chain_ids=frozenset(
                    memberships.get(vertex_id, ())
                ),
                source_domain_coordinate=_exact_point(source),
                base_bound_node=base_node,
                base_bound_coordinate=_exact_point(base_coordinate),
                assigned_refined_node=assigned_node,
                assigned_domain_coordinate=exact_coordinate,
                exact_offset_from_source=_exact_vector(offset),
            )
        )
    return authority_records, coordinate_records


def _v2_binding(
    compilation: ReferenceEnvelopeCompilationV1,
    frame: RationalAffinePlanarMetricV2,
    base_lattice: GridSpecV1,
    declared_chains: tuple[PhysicalChainV1, ...],
    bound_hidden_support_ids: frozenset,
) -> ChainStraightEvaluationGeometryBindingV2:
    source_by_id = _source_coordinates(frame)
    base_node_by_id = _base_nodes(frame, base_lattice)
    chain_infos = tuple(
        _chain_info(chain, source_by_id, base_node_by_id)
        for chain in declared_chains
    )
    refinement_power = _minimum_refinement_power(chain_infos)
    factor = 1 << refinement_power
    scale = base_lattice.scale * factor
    gram = _gram_matrix(frame)
    chain_bindings = tuple(
        _chain_binding(
            info,
            base_lattice.scale,
            refinement_power,
            gram,
        )
        for info in chain_infos
    )
    authority_records, coordinate_records = _v2_vertex_records(
        source_by_id,
        base_node_by_id,
        chain_bindings,
        base_lattice,
        factor,
        scale,
    )
    deficits = frozenset()
    if refinement_power > 0:
        previous_power = refinement_power - 1
        previous_factor = 1 << previous_power
        deficits = frozenset(
            ChainStraightCapacityDeficitV2(
                physical_chain_id=info.chain.physical_chain_id,
                previous_refinement_power=previous_power,
                previous_endpoint_span_k=(
                    info.base_endpoint_span * previous_factor
                ),
                capacity_required=(
                    len(info.chain.ordered_source_vertex_ids) - 1
                ),
                exact_deficit=(
                    len(info.chain.ordered_source_vertex_ids)
                    - 1
                    - info.base_endpoint_span * previous_factor
                ),
            )
            for info in chain_infos
            if info.base_endpoint_span * previous_factor
            < len(info.chain.ordered_source_vertex_ids) - 1
        )
    return ChainStraightEvaluationGeometryBindingV2(
        schema_version=(
            CHAIN_STRAIGHT_EVALUATION_GEOMETRY_BINDING_SCHEMA_V2
        ),
        source_revision=compilation.source_revision,
        patch_domain_id=compilation.plan_key.patch_domain_id,
        reference_metric_id=frame.reference_metric_id,
        binding_law=(
            ChainStraightEvaluationGeometryBindingLawV2.EVALUATION_GEOMETRY_CHART_LATTICE_BOUND_CHAIN_STRAIGHT_V2
        ),
        base_lattice_scale=base_lattice.scale,
        refinement_power=refinement_power,
        lattice_scale=scale,
        source_vertex_coordinates=frozenset(coordinate_records),
        vertex_authorities=frozenset(authority_records),
        straight_chain_bindings=frozenset(chain_bindings),
        previous_refinement_capacity_deficits=deficits,
        bound_hidden_support_ids=bound_hidden_support_ids,
    )


def evaluation_geometry_binding_residual(
    frame: PlanarPatchFrameV1 | RationalAffinePlanarMetricV2,
    binding: EvaluationGeometryBinding | None,
) -> Fraction:
    """Точная цена binding как максимум сдвига одной координаты."""

    if (
        binding is None
        or not isinstance(frame, RationalAffinePlanarMetricV2)
        or binding.source_revision != frame.source_revision
        or binding.patch_domain_id != frame.patch_domain_id
        or binding.reference_metric_id != frame.reference_metric_id
    ):
        raise EvaluationGeometryBindingInvalid(
            "evaluation geometry binding identity does not match its metric"
        )
    source = {
        item.source_vertex_id: _fraction_pair(item.domain_coordinate)
        for item in frame.exact_source_vertex_coordinates
    }
    bound = {
        item.source_vertex_id: _fraction_pair(item.domain_coordinate)
        for item in binding.source_vertex_coordinates
    }
    if (
        len(source) != len(frame.exact_source_vertex_coordinates)
        or len(bound) != len(binding.source_vertex_coordinates)
        or source.keys() != bound.keys()
    ):
        raise EvaluationGeometryBindingInvalid(
            "evaluation geometry binding source vertex identity mismatch"
        )
    return max(
        (
            abs(source_value - bound_value)
            for vertex_id in source
            for source_value, bound_value in zip(
                source[vertex_id],
                bound[vertex_id],
                strict=True,
            )
        ),
        default=Fraction(0),
    )


def build_evaluation_geometry_binding(
    compilation: ReferenceEnvelopeCompilationV1,
    frame: PlanarPatchFrameV1 | RationalAffinePlanarMetricV2,
) -> EvaluationGeometryBinding | None:
    """Выбрать V1 либо минимальный общий S' V2 до запуска evaluators."""

    lattice = chart_lattice_for_frame(frame)
    if lattice is None:
        return None
    if not isinstance(frame, RationalAffinePlanarMetricV2):
        raise EvaluationGeometryBindingInvalid(
            "chart lattice requires RationalAffinePlanarMetricV2"
        )
    declared_chains = _declared_straight_chains(compilation)
    if declared_chains:
        return _v2_binding(
            compilation,
            frame,
            lattice,
            declared_chains,
            frozenset(),
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


def _expected_support_ids(
    compilation: ReferenceEnvelopeCompilationV1,
) -> frozenset:
    return frozenset(
        support.hidden_support_id
        for spec in compilation.envelope_specs
        if isinstance(spec, AngularEnvelopeSpec)
        for support in spec.hidden_supports
        if type(support) is CertifiedBoundHiddenSupportSpecV1
    )


def verify_evaluation_geometry_binding(
    binding: EvaluationGeometryBinding | None,
    compilation: ReferenceEnvelopeCompilationV1,
    frame: PlanarPatchFrameV1 | RationalAffinePlanarMetricV2,
    *,
    require_certified_bound_supports: bool = True,
) -> None:
    """Независимо пересобрать V1/V2, identity и каждую bound coordinate."""

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
    declared_chains = _declared_straight_chains(compilation)
    support_ids = _expected_support_ids(compilation)
    if declared_chains:
        if type(binding) is not ChainStraightEvaluationGeometryBindingV2:
            raise EvaluationGeometryBindingInvalid(
                "declared straight multi-vertex chain requires V2 binding"
            )
        issues = validate_chain_straight_evaluation_geometry_binding(binding)
        expected = _v2_binding(
            compilation,
            frame,
            lattice,
            declared_chains,
            support_ids,
        )
    else:
        if type(binding) is not EvaluationGeometryBindingV1:
            raise EvaluationGeometryBindingInvalid(
                "V2 binding is forbidden without declared straight chains"
            )
        issues = validate_evaluation_geometry_binding(binding)
        expected = EvaluationGeometryBindingV1(
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
            bound_hidden_support_ids=support_ids,
        )
    support_authority_matches = (
        binding.bound_hidden_support_ids == support_ids
        and (
            not require_certified_bound_supports
            or all(
                type(support) is CertifiedBoundHiddenSupportSpecV1
                for spec in compilation.envelope_specs
                if isinstance(spec, AngularEnvelopeSpec)
                for support in spec.hidden_supports
                if support.hidden_support_id
                in binding.bound_hidden_support_ids
            )
        )
    )
    if issues or binding != expected or not support_authority_matches:
        raise EvaluationGeometryBindingInvalid(
            "evaluation geometry binding does not match its plan and metric"
        )
