"""Pure structural validation; геометрические predicates здесь не вычисляются."""

from __future__ import annotations

from dataclasses import dataclass
from decimal import Decimal
from enum import Enum
from fractions import Fraction
from math import gcd

from .canonical import geometry_batch_semantic_digest
from .contracts.analysis import (
    ANALYSIS_SNAPSHOT_SCHEMA_V1,
    AnalysisCapability,
    AnalysisSnapshotV1,
    BoundaryLoopConstraintTargetV1,
    CertifiedPlanarSupportDirectionV1,
    CertifiedReflexAngleMeasureV1,
    ChainUseConstraintTargetV1,
    DeclaredRoutePairsTopologyV1,
    IntrinsicSurfaceMetricDescriptorV1,
    JunctionRelationKind,
    JunctionRoutePairV1,
    PatchFrameHandedness,
    PhysicalEdgeSequenceConstraintTargetV1,
    PlanarPatchFrameV1,
    RationalAffinePlanarMetricV2,
    SurfaceRegime,
    TerminalEndpointRole,
    TJunctionRouteTopologyV1,
    UnavailableBoundaryConstraintTargetV1,
    UnavailableJunctionRouteTopologyV1,
    UnavailableReflexAngleMeasureV1,
    UnavailableSourceSupportDirectionV1,
    UnavailableSurfaceMetricDescriptorV1,
    XJunctionRouteTopologyV1,
    YJunctionRouteTopologyV1,
)
from .contracts.coverage import CoverageEffect
from .contracts.envelopes import (
    AngularEnvelopeSpec,
    CapEnvelopeSpec,
    CertifiedBoundHiddenSupportDirectionLawV1,
    CertifiedBoundHiddenSupportSpecV1,
    EnvelopeSpecVariant,
    HiddenSupportDirectionLaw,
    HiddenSupportSpecV1,
    JunctionEnvelopeSpec,
    StripEnvelopeSpec,
)
from .contracts.events import EventParticipantKind, InitialFrontFeatureKind
from .contracts.geometry_batch import GEOMETRY_BATCH_SCHEMA_V1, GeometryBatchV1
from .contracts.metric import (
    AffineFrameSelectionLawV1,
    AffineReconstructionLawV1,
    ExactMatrix2V1,
    ExactPoint3V1,
    ExactRationalV1,
    ExactVector3V1,
    MetricSemanticIdentityLawV1,
    PlanarityAdmissionLawV1,
    RuntimeMetricFallbackLawV1,
    RuntimePlanarMetricV1,
    RuntimePredicateFilterLawV1,
    RuntimePredicateResultV1,
)
from .contracts.ownership import (
    ClaimClosureObligation,
    ClaimInteriorsObligation,
    EqualityBoundaryOwner,
    OwnershipPartitionContractId,
    SilhouetteEffect,
)
from .contracts.plan import (
    CHAIN_STRAIGHT_EVALUATION_GEOMETRY_BINDING_SCHEMA_V2,
    COMPILED_PLAN_SCHEMA_V1,
    EVALUATION_GEOMETRY_BINDING_SCHEMA_V1,
    ChainStraightAssignmentDispositionV2,
    ChainStraightEvaluationGeometryBindingLawV2,
    ChainStraightEvaluationGeometryBindingV2,
    ChainStraightVertexAuthorityV2,
    CompiledPatchEvaluationPlanV1,
    EvaluationGeometryBindingLawV1,
    EvaluationGeometryBindingV1,
)
from .contracts.request import (
    DECAL_REQUEST_SCHEMA_V1,
    AngularProfileFamilyId,
    AngularProfileSelectionPolicyId,
    BoundaryPolicyId,
    CapPolicyId,
    DecalRequestV1,
    InteractionPolicyId,
    MaxSubturnParameterId,
    MaxSubturnValueId,
    OwnershipPolicyId,
)
from .contracts.seeds import (
    CapSeedV1,
    CornerSeedV1,
    EndpointClaimSeedV1,
    FrontSeedV1,
    JunctionSeedV1,
)
from .contracts.surface import SurfacePayloadMode
from .contracts.tessellation import (
    ForbiddenTessellationOperation,
    SharedVertexKeyPolicy,
    TessellationContractId,
    TessellationDigestEquivalence,
    TessellationStartStage,
)
from .ids import (
    BoundaryConstraintId,
    ChainUseId,
    EnvelopeInstanceId,
    EnvelopeSpecId,
    FrontComponentId,
    HiddenSupportId,
    OpaqueId,
    OwnerSectorId,
    PhysicalEdgeId,
    SemanticDigestValue,
)
from .numeric import (
    ExactAngleSymbol,
    IntervalEndpointKind,
    LocalPoint3V1,
    MetricSpace,
)
from .outcomes import NamedOutcome


class ValidationCode(str, Enum):
    SCHEMA_VERSION = "SCHEMA_VERSION"
    CAPABILITY = "CAPABILITY"
    DUPLICATE_ID = "DUPLICATE_ID"
    MISSING_REFERENCE = "MISSING_REFERENCE"
    CROSS_CONTRACT_MISMATCH = "CROSS_CONTRACT_MISMATCH"
    POLICY_MISMATCH = "POLICY_MISMATCH"
    ANGULAR_CERTIFICATE = "ANGULAR_CERTIFICATE"
    SEED_VARIANT_MISMATCH = "SEED_VARIANT_MISMATCH"
    PLAN_KEY_MISMATCH = "PLAN_KEY_MISMATCH"
    OWNERSHIP_DECLARATION = "OWNERSHIP_DECLARATION"
    TESSELLATION_AUTHORITY = "TESSELLATION_AUTHORITY"
    GEOMETRY_BATCH = "GEOMETRY_BATCH"
    FORBIDDEN_TOPOLOGY_IDENTITY = "FORBIDDEN_TOPOLOGY_IDENTITY"
    SURFACE_TOPOLOGY = "SURFACE_TOPOLOGY"
    SURFACE_METRIC = "SURFACE_METRIC"
    ROUTE_TOPOLOGY = "ROUTE_TOPOLOGY"
    TERMINAL_RELATION = "TERMINAL_RELATION"
    ANGULAR_SELECTION_UNCERTAIN = "ANGULAR_SELECTION_UNCERTAIN"
    STATION_FACT = "STATION_FACT"
    EVALUATION_GEOMETRY = "EVALUATION_GEOMETRY"


@dataclass(frozen=True, slots=True)
class ValidationIssue:
    code: ValidationCode
    path: tuple[str, ...]
    message: str


class ContractValidationError(ValueError):
    def __init__(self, issues: tuple[ValidationIssue, ...]) -> None:
        self.issues = issues
        detail = "; ".join(
            f"{issue.code.value}@{'.'.join(issue.path)}: {issue.message}"
            for issue in issues
        )
        super().__init__(detail)


def raise_for_issues(issues: tuple[ValidationIssue, ...]) -> None:
    if issues:
        raise ContractValidationError(issues)


def _issue(
    issues: list[ValidationIssue],
    code: ValidationCode,
    path: tuple[str, ...],
    message: str,
) -> None:
    issues.append(ValidationIssue(code, path, message))


def _values(records: frozenset[object], attribute: str) -> set[OpaqueId]:
    return {getattr(record, attribute) for record in records}


def _check_unique(
    issues: list[ValidationIssue],
    records: frozenset[object],
    attribute: str,
    path: str,
) -> set[OpaqueId]:
    values = [getattr(record, attribute) for record in records]
    if len(values) != len(set(values)):
        _issue(issues, ValidationCode.DUPLICATE_ID, (path,), f"duplicate {attribute}")
    return set(values)


def _require_refs(
    issues: list[ValidationIssue],
    refs: set[OpaqueId],
    targets: set[OpaqueId],
    path: tuple[str, ...],
) -> None:
    missing = refs - targets
    if missing:
        _issue(
            issues,
            ValidationCode.MISSING_REFERENCE,
            path,
            "missing: " + ", ".join(sorted(str(value) for value in missing)),
        )


def _interval_strictly_above(interval: object, value: Decimal) -> bool:
    return interval.lower > value or (
        interval.lower == value and interval.lower_kind is IntervalEndpointKind.OPEN
    )


def _interval_strictly_below(interval: object, value: Decimal) -> bool:
    return interval.upper < value or (
        interval.upper == value and interval.upper_kind is IntervalEndpointKind.OPEN
    )


def _fraction(value: ExactRationalV1) -> Fraction:
    return Fraction(value.numerator, value.denominator)


def _fraction_point3(
    value: ExactPoint3V1 | ExactVector3V1,
) -> tuple[Fraction, Fraction, Fraction]:
    return _fraction(value.x), _fraction(value.y), _fraction(value.z)


def _position_under_grid_law(
    position: LocalPoint3V1, certificate
) -> tuple[Fraction, Fraction, Fraction]:
    """Позиция источника такой, какой её делает ОБЪЯВЛЕННЫЙ закон решётки.

    Проверка «карта восстанавливает вершину точно» обязана сверяться с тем
    входом, который метрика реально получила. До решётки это была сама
    координата binary64; у всякого закона, который двигает источник, — её
    узел. Сверяться с непривязанной координатой значило бы требовать от
    привязанной метрики невозможного, а снять проверку — потерять
    единственное место, где карта сверяется с источником.

    Условие — `snaps_source`, а не имя закона: привязку конструкций эта
    проверка не видит вовсе (она про вершины источника), поэтому
    `SOURCE_ONLY_GRID_SNAP_V1` обязан идти здесь тем же путём, что и
    `INTEGER_GRID_SNAP_V1`.

    Узел перевычисляется здесь заново, из объявленного в сертификате масштаба,
    а не берётся у построителя: так проверяется и то, что метрика привязана к
    ТОМУ масштабу, который она объявляет.
    """

    from .robust.grid import GridSpecV1, snap_value

    exact = tuple(
        Fraction(*float(value).as_integer_ratio())
        for value in (position.x, position.y, position.z)
    )
    if (
        not certificate.snapping_law.snaps_source
        or certificate.source_scale is None
    ):
        return exact
    grid = GridSpecV1(scale=certificate.source_scale)
    return tuple(
        Fraction(snap_value(item, grid), certificate.source_scale)
        for item in exact
    )


def _fraction_matrix2(
    value: ExactMatrix2V1,
) -> tuple[tuple[Fraction, Fraction], tuple[Fraction, Fraction]]:
    return (
        (_fraction(value.m00), _fraction(value.m01)),
        (_fraction(value.m10), _fraction(value.m11)),
    )


def validate_evaluation_geometry_binding(
    binding: EvaluationGeometryBindingV1,
) -> tuple[ValidationIssue, ...]:
    """Проверить самодостаточную запись общей evaluation-геометрии."""

    issues: list[ValidationIssue] = []
    path = ("EvaluationGeometryBindingV1",)
    if binding.schema_version != EVALUATION_GEOMETRY_BINDING_SCHEMA_V1:
        _issue(
            issues,
            ValidationCode.SCHEMA_VERSION,
            path + ("schema_version",),
            "unexpected evaluation-geometry binding schema",
        )
    if (
        binding.binding_law
        is not EvaluationGeometryBindingLawV1.EVALUATION_GEOMETRY_CHART_LATTICE_BOUND_V1
    ):
        _issue(
            issues,
            ValidationCode.EVALUATION_GEOMETRY,
            path + ("binding_law",),
            "unsupported evaluation-geometry binding law",
        )
    scale = binding.lattice_scale
    if type(scale) is not int or scale <= 0 or scale & (scale - 1):
        _issue(
            issues,
            ValidationCode.EVALUATION_GEOMETRY,
            path + ("lattice_scale",),
            "chart-lattice scale must be a positive power of two",
        )
        return tuple(issues)
    seen = set()
    for vertex in binding.source_vertex_coordinates:
        item_path = path + (
            "source_vertex_coordinates",
            vertex.source_vertex_id.value,
        )
        if vertex.source_vertex_id in seen:
            _issue(
                issues,
                ValidationCode.DUPLICATE_ID,
                item_path,
                "source vertex occurs more than once",
            )
        seen.add(vertex.source_vertex_id)
        for axis, value in (
            ("x", vertex.domain_coordinate.x),
            ("y", vertex.domain_coordinate.y),
        ):
            if value.numerator * scale % value.denominator:
                _issue(
                    issues,
                    ValidationCode.EVALUATION_GEOMETRY,
                    item_path + ("domain_coordinate", axis),
                    "bound coordinate is not a node of the declared chart lattice",
                )
    if not seen:
        _issue(
            issues,
            ValidationCode.EVALUATION_GEOMETRY,
            path + ("source_vertex_coordinates",),
            "evaluation geometry cannot be empty",
        )
    return tuple(issues)


def validate_chain_straight_evaluation_geometry_binding(
    binding: ChainStraightEvaluationGeometryBindingV2,
) -> tuple[ValidationIssue, ...]:
    """Проверить самодостаточную структуру V2 без доверия записанным offsets."""

    issues: list[ValidationIssue] = []
    path = ("ChainStraightEvaluationGeometryBindingV2",)
    if (
        binding.schema_version
        != CHAIN_STRAIGHT_EVALUATION_GEOMETRY_BINDING_SCHEMA_V2
    ):
        _issue(
            issues,
            ValidationCode.SCHEMA_VERSION,
            path + ("schema_version",),
            "unexpected chain-straight evaluation-geometry schema",
        )
    if (
        binding.binding_law
        is not ChainStraightEvaluationGeometryBindingLawV2.EVALUATION_GEOMETRY_CHART_LATTICE_BOUND_CHAIN_STRAIGHT_V2
    ):
        _issue(
            issues,
            ValidationCode.EVALUATION_GEOMETRY,
            path + ("binding_law",),
            "unsupported chain-straight evaluation-geometry law",
        )
    base_scale = binding.base_lattice_scale
    scale = binding.lattice_scale
    refinement_power = binding.refinement_power
    scales_valid = (
        type(base_scale) is int
        and base_scale > 0
        and not base_scale & (base_scale - 1)
        and type(refinement_power) is int
        and 0 <= refinement_power <= 8
        and type(scale) is int
        and scale == base_scale * (1 << refinement_power)
    )
    if not scales_valid:
        _issue(
            issues,
            ValidationCode.EVALUATION_GEOMETRY,
            path + ("lattice_scale",),
            "V2 scales must satisfy S'=S*2^r for r in 0..8",
        )
        return tuple(issues)

    coordinates = {
        item.source_vertex_id: item
        for item in binding.source_vertex_coordinates
    }
    if len(coordinates) != len(binding.source_vertex_coordinates):
        _issue(
            issues,
            ValidationCode.DUPLICATE_ID,
            path + ("source_vertex_coordinates",),
            "source vertex occurs more than once",
        )
    authorities = {
        item.source_vertex_id: item for item in binding.vertex_authorities
    }
    if len(authorities) != len(binding.vertex_authorities):
        _issue(
            issues,
            ValidationCode.DUPLICATE_ID,
            path + ("vertex_authorities",),
            "vertex authority occurs more than once",
        )
    if not coordinates or coordinates.keys() != authorities.keys():
        _issue(
            issues,
            ValidationCode.EVALUATION_GEOMETRY,
            path + ("vertex_authorities",),
            "coordinate and authority identity sets must be equal and non-empty",
        )
    for vertex_id, authority in authorities.items():
        item_path = path + ("vertex_authorities", vertex_id.value)
        source = (
            _fraction(authority.source_domain_coordinate.x),
            _fraction(authority.source_domain_coordinate.y),
        )
        base = (
            _fraction(authority.base_bound_coordinate.x),
            _fraction(authority.base_bound_coordinate.y),
        )
        assigned = (
            _fraction(authority.assigned_domain_coordinate.x),
            _fraction(authority.assigned_domain_coordinate.y),
        )
        offset = (
            _fraction(authority.exact_offset_from_source.x),
            _fraction(authority.exact_offset_from_source.y),
        )
        expected_base = tuple(
            Fraction(item, base_scale) for item in authority.base_bound_node
        )
        expected_base_node = tuple(
            (
                2 * (value * base_scale).numerator
                + (value * base_scale).denominator
            )
            // (2 * (value * base_scale).denominator)
            for value in source
        )
        expected_assigned = tuple(
            Fraction(item, scale) for item in authority.assigned_refined_node
        )
        coordinate_record = coordinates.get(vertex_id)
        if (
            authority.base_bound_node != expected_base_node
            or base != expected_base
            or assigned != expected_assigned
            or offset
            != tuple(
                assigned_value - source_value
                for assigned_value, source_value in zip(
                    assigned, source, strict=True
                )
            )
            or coordinate_record is None
            or coordinate_record.domain_coordinate
            != authority.assigned_domain_coordinate
        ):
            _issue(
                issues,
                ValidationCode.EVALUATION_GEOMETRY,
                item_path,
                "vertex authority coordinates, nodes, or offset disagree",
            )
        base_authority = authority.authority in (
            ChainStraightVertexAuthorityV2.BASE_BOUND_ENDPOINT_V1,
            ChainStraightVertexAuthorityV2.BASE_BOUND_NON_CHAIN_V1,
        )
        if base_authority and (
            assigned != base
            or authority.assigned_refined_node
            != tuple(
                item * (1 << refinement_power)
                for item in authority.base_bound_node
            )
        ):
            _issue(
                issues,
                ValidationCode.EVALUATION_GEOMETRY,
                item_path + ("authority",),
                "BASE_BOUND authority must preserve the V1 coordinate",
            )

    chains = {
        item.physical_chain_id: item
        for item in binding.straight_chain_bindings
    }
    if not chains or len(chains) != len(binding.straight_chain_bindings):
        _issue(
            issues,
            ValidationCode.DUPLICATE_ID,
            path + ("straight_chain_bindings",),
            "straight-chain identities must be unique and non-empty",
        )
    for chain_id, chain in chains.items():
        item_path = path + ("straight_chain_bindings", chain_id.value)
        ordered = chain.ordered_source_vertex_ids
        if (
            len(ordered) < 3
            or len(set(ordered)) != len(ordered)
            or chain.primitive_direction == (0, 0)
            or gcd(*map(abs, chain.primitive_direction)) != 1
            or chain.base_endpoint_span_k <= 0
            or chain.base_end_node
            != tuple(
                start + chain.base_endpoint_span_k * direction
                for start, direction in zip(
                    chain.base_start_node,
                    chain.primitive_direction,
                    strict=True,
                )
            )
            or chain.refined_endpoint_span_k
            != chain.base_endpoint_span_k * (1 << refinement_power)
        ):
            _issue(
                issues,
                ValidationCode.EVALUATION_GEOMETRY,
                item_path,
                "straight-chain identity, primitive direction, or span is invalid",
            )
        assignments = chain.internal_assignments
        if (
            tuple(item.ordinal for item in assignments)
            != tuple(range(1, len(ordered) - 1))
            or tuple(item.source_vertex_id for item in assignments)
            != ordered[1:-1]
        ):
            _issue(
                issues,
                ValidationCode.EVALUATION_GEOMETRY,
                item_path + ("internal_assignments",),
                "assignments must cover internal vertices in source order",
            )
        sequence = (
            0,
            *(item.selected_k for item in assignments),
            chain.refined_endpoint_span_k,
        )
        if any(
            current <= previous
            for previous, current in zip(sequence, sequence[1:])
        ):
            _issue(
                issues,
                ValidationCode.EVALUATION_GEOMETRY,
                item_path + ("internal_assignments",),
                "selected k sequence must be strictly increasing",
            )
        for assignment in assignments:
            assignment_path = item_path + (
                "internal_assignments",
                str(assignment.ordinal),
            )
            expected_clamped = (
                assignment.selected_k
                != assignment.unconstrained_canonical_k
            )
            excess_named = (
                assignment.disposition
                is ChainStraightAssignmentDispositionV2.CLAMPED_CONSTRAINT_EXCESS_ALLOWED
            )
            expected_dispositions = (
                {
                    ChainStraightAssignmentDispositionV2.CLAMPED_WITHIN_HALF_STEP,
                    ChainStraightAssignmentDispositionV2.CLAMPED_CONSTRAINT_EXCESS_ALLOWED,
                }
                if assignment.clamped
                else {
                    ChainStraightAssignmentDispositionV2.UNCLAMPED_WITHIN_HALF_STEP
                }
            )
            expected_node = tuple(
                start * (1 << refinement_power)
                + assignment.selected_k * direction
                for start, direction in zip(
                    chain.base_start_node,
                    chain.primitive_direction,
                    strict=True,
                )
            )
            authority = authorities.get(assignment.source_vertex_id)
            if (
                assignment.physical_chain_id != chain_id
                or assignment.clamped is not expected_clamped
                or not assignment.lower_k
                <= assignment.selected_k
                <= assignment.upper_k
                or excess_named and not assignment.clamped
                or assignment.disposition not in expected_dispositions
                or assignment.assigned_refined_node != expected_node
                or authority is None
                or authority.authority
                is not ChainStraightVertexAuthorityV2.CHAIN_STRAIGHT_INTERNAL_REFINED_V2
                or chain_id not in authority.physical_chain_ids
                or authority.assigned_refined_node
                != assignment.assigned_refined_node
                or authority.exact_offset_from_source
                != assignment.exact_offset_from_source
            ):
                _issue(
                    issues,
                    ValidationCode.EVALUATION_GEOMETRY,
                    assignment_path,
                    "CLAMPED, selected interval, or disposition is inconsistent",
                )
        start_authority = authorities.get(ordered[0]) if ordered else None
        end_authority = authorities.get(ordered[-1]) if ordered else None
        if (
            start_authority is None
            or end_authority is None
            or start_authority.authority
            is not ChainStraightVertexAuthorityV2.BASE_BOUND_ENDPOINT_V1
            or end_authority.authority
            is not ChainStraightVertexAuthorityV2.BASE_BOUND_ENDPOINT_V1
            or chain_id not in start_authority.physical_chain_ids
            or chain_id not in end_authority.physical_chain_ids
            or start_authority.base_bound_node != chain.base_start_node
            or end_authority.base_bound_node != chain.base_end_node
        ):
            _issue(
                issues,
                ValidationCode.EVALUATION_GEOMETRY,
                item_path + ("ordered_source_vertex_ids",),
                "chain endpoints must retain BASE_BOUND endpoint authority",
            )

    used_vertex_ids = frozenset(
        vertex_id
        for chain in chains.values()
        for vertex_id in chain.ordered_source_vertex_ids
    )
    for vertex_id, authority in authorities.items():
        is_non_chain = (
            authority.authority
            is ChainStraightVertexAuthorityV2.BASE_BOUND_NON_CHAIN_V1
        )
        if is_non_chain != (
            vertex_id not in used_vertex_ids
            and not authority.physical_chain_ids
        ):
            _issue(
                issues,
                ValidationCode.EVALUATION_GEOMETRY,
                path + ("vertex_authorities", vertex_id.value, "authority"),
                "non-chain authority must match straight-chain membership",
            )

    deficits = binding.previous_refinement_capacity_deficits
    if refinement_power == 0 and deficits:
        _issue(
            issues,
            ValidationCode.EVALUATION_GEOMETRY,
            path + ("previous_refinement_capacity_deficits",),
            "r=0 cannot have a previous-refinement witness",
        )
    for deficit in deficits:
        if (
            deficit.previous_refinement_power != refinement_power - 1
            or deficit.physical_chain_id not in chains
            or deficit.exact_deficit
            != deficit.capacity_required - deficit.previous_endpoint_span_k
            or deficit.exact_deficit <= 0
        ):
            _issue(
                issues,
                ValidationCode.EVALUATION_GEOMETRY,
                path
                + (
                    "previous_refinement_capacity_deficits",
                    deficit.physical_chain_id.value,
                ),
                "previous-refinement capacity witness is inconsistent",
            )
    return tuple(issues)


def _fraction_dot3(left, right) -> Fraction:
    return sum(
        (a * b for a, b in zip(left, right, strict=True)),
        Fraction(0),
    )


def _fraction_cross3(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def validate_rational_affine_planar_metric(
    metric: RationalAffinePlanarMetricV2,
) -> tuple[ValidationIssue, ...]:
    issues: list[ValidationIssue] = []
    path = ("RationalAffinePlanarMetricV2",)
    if (
        metric.frame_selection_law
        is not AffineFrameSelectionLawV1.CANONICAL_SOURCE_VERTEX_BASIS_V1
    ):
        _issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("frame_selection_law",),
            "unsupported deterministic affine-frame law",
        )
    certificate = metric.planarity_certificate
    if (
        certificate.patch_domain_id != metric.patch_domain_id
        or certificate.admission_law
        is not PlanarityAdmissionLawV1.EXACT_SOURCE_PLANE_V1
        or certificate.reconstruction_law
        is not AffineReconstructionLawV1.O_PLUS_U_A_PLUS_V_B_V1
        or not certificate.exact
    ):
        _issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("planarity_certificate",),
            "metric requires an exact same-domain source-plane certificate",
        )
    basis_a = _fraction_point3(metric.exact_basis_a)
    basis_b = _fraction_point3(metric.exact_basis_b)
    normal = _fraction_cross3(basis_a, basis_b)
    if not any(normal):
        _issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("exact_basis_a", "exact_basis_b"),
            "affine basis vectors must be linearly independent",
        )
    if _fraction_point3(certificate.exact_plane_normal) != normal:
        _issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("planarity_certificate", "exact_plane_normal"),
            "plane normal must be the exact A cross B construction",
        )
    gram = _fraction_matrix2(metric.exact_gram_matrix)
    expected_gram = (
        (
            _fraction_dot3(basis_a, basis_a),
            _fraction_dot3(basis_a, basis_b),
        ),
        (
            _fraction_dot3(basis_b, basis_a),
            _fraction_dot3(basis_b, basis_b),
        ),
    )
    if gram != expected_gram:
        _issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("exact_gram_matrix",),
            "Gram matrix does not equal the exact A/B dot products",
        )
    determinant = gram[0][0] * gram[1][1] - gram[0][1] * gram[1][0]
    if determinant <= 0:
        _issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("exact_gram_matrix",),
            "Gram matrix must be positive definite",
        )
    else:
        inverse = _fraction_matrix2(metric.exact_inverse_gram_matrix)
        expected_inverse = (
            (
                gram[1][1] / determinant,
                -gram[0][1] / determinant,
            ),
            (
                -gram[1][0] / determinant,
                gram[0][0] / determinant,
            ),
        )
        if inverse != expected_inverse:
            _issue(
                issues,
                ValidationCode.SURFACE_METRIC,
                path + ("exact_inverse_gram_matrix",),
                "inverse Gram matrix is not exact",
            )
    coordinate_ids = [
        item.source_vertex_id
        for item in metric.exact_source_vertex_coordinates
    ]
    if len(coordinate_ids) != len(set(coordinate_ids)):
        _issue(
            issues,
            ValidationCode.DUPLICATE_ID,
            path + ("exact_source_vertex_coordinates",),
            "duplicate source vertex coordinate",
        )
    if set(coordinate_ids) != set(certificate.source_vertex_ids):
        _issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("exact_source_vertex_coordinates",),
            "coordinate and planarity-certificate vertex sets differ",
        )
    return tuple(issues)


def validate_runtime_planar_metric(
    metric: RuntimePlanarMetricV1,
) -> tuple[ValidationIssue, ...]:
    issues: list[ValidationIssue] = []
    path = ("RuntimePlanarMetricV1",)
    filter_contract = metric.predicate_filter_contract
    fallback = metric.fallback_contract
    if (
        filter_contract.filter_law
        is not RuntimePredicateFilterLawV1.BINARY64_OUTWARD_INTERVAL_V1
        or filter_contract.uncertain_result
        is not RuntimePredicateResultV1.EXACT_FALLBACK_REQUIRED
        or not filter_contract.exact_zero_requires_fallback
        or filter_contract.semantic_identity_law
        is not MetricSemanticIdentityLawV1.EXACT_CONSTRUCTION_CERTIFICATES_ONLY
    ):
        _issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("predicate_filter_contract",),
            "runtime predicate filter can only certify non-zero signs",
        )
    if (
        fallback.fallback_law
        is not RuntimeMetricFallbackLawV1.AUTHORITATIVE_REFERENCE_METRIC_V2
        or fallback.authoritative_reference_metric_id
        != metric.reference_metric_id
        or not fallback.rounded_runtime_reconstruction_forbidden
    ):
        _issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("fallback_contract",),
            "runtime fallback must name the authoritative reference metric",
        )
    return tuple(issues)


def _junction_route_pairs(topology: object) -> frozenset[JunctionRoutePairV1]:
    if isinstance(topology, TJunctionRouteTopologyV1):
        return frozenset({topology.through_route_pair})
    if isinstance(
        topology,
        (
            XJunctionRouteTopologyV1,
            YJunctionRouteTopologyV1,
            DeclaredRoutePairsTopologyV1,
        ),
    ):
        return topology.route_pairs
    return frozenset()


def validate_analysis_snapshot(snapshot: AnalysisSnapshotV1) -> tuple[ValidationIssue, ...]:
    issues: list[ValidationIssue] = []
    if snapshot.schema_version != ANALYSIS_SNAPSHOT_SCHEMA_V1:
        _issue(issues, ValidationCode.SCHEMA_VERSION, ("schema_version",), "unsupported snapshot schema")
    if snapshot.surface_ir.schema_version != "cftuv.envelope.patch_surface_ir.v1":
        _issue(issues, ValidationCode.SCHEMA_VERSION, ("surface_ir", "schema_version"), "unsupported surface schema")
    if snapshot.source_revision != snapshot.surface_ir.source_revision:
        _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("surface_ir", "source_revision"), "surface and snapshot revisions differ")

    required = {
        AnalysisCapability.PATCH_DOMAIN_V1,
        AnalysisCapability.PHYSICAL_CHAIN_DIRECTED_USES_V1,
        AnalysisCapability.PATCH_SURFACE_IR_V1,
        AnalysisCapability.ORIENTED_OWNER_SECTOR_V1,
        AnalysisCapability.REFLEX_ANGLE_CERTIFICATE_V1,
        AnalysisCapability.RELATION_LINEAGE_V1,
    }
    missing_capabilities = required - set(snapshot.analysis_capabilities)
    if missing_capabilities:
        _issue(issues, ValidationCode.CAPABILITY, ("analysis_capabilities",), "missing required v1 capabilities")

    patch_ids = _check_unique(issues, snapshot.patches, "patch_id", "patches")
    domain_ids = _check_unique(issues, snapshot.patch_domains, "patch_domain_id", "patch_domains")
    vertex_ids = _check_unique(issues, snapshot.source_vertices, "vertex_id", "source_vertices")
    chain_ids = _check_unique(issues, snapshot.physical_chains, "physical_chain_id", "physical_chains")
    use_ids = _check_unique(issues, snapshot.chain_uses, "chain_use_id", "chain_uses")
    loop_ids = _check_unique(issues, snapshot.boundary_loops, "boundary_loop_id", "boundary_loops")
    boundary_ids = _check_unique(issues, snapshot.boundary_constraints, "boundary_constraint_id", "boundary_constraints")
    angular_sector_ids = _check_unique(issues, snapshot.angular_owner_sectors, "owner_sector_id", "angular_owner_sectors")
    angle_certificate_ids = _check_unique(issues, snapshot.reflex_angle_certificates, "certificate_id", "reflex_angle_certificates")
    corner_relation_ids = _check_unique(issues, snapshot.corner_relations, "corner_relation_id", "corner_relations")
    junction_relation_ids = _check_unique(issues, snapshot.junction_relations, "junction_relation_id", "junction_relations")
    terminal_relation_ids = _check_unique(issues, snapshot.terminal_relations, "terminal_relation_id", "terminal_relations")
    del corner_relation_ids, junction_relation_ids, terminal_relation_ids

    metric_domain_ids = [item.patch_domain_id for item in snapshot.surface_metric_descriptors]
    if len(metric_domain_ids) != len(set(metric_domain_ids)):
        _issue(issues, ValidationCode.DUPLICATE_ID, ("surface_metric_descriptors",), "duplicate patch_domain_id")
    _require_refs(issues, set(metric_domain_ids), domain_ids, ("surface_metric_descriptors", "patch_domain_id"))
    if set(metric_domain_ids) != domain_ids:
        _issue(issues, ValidationCode.SURFACE_METRIC, ("surface_metric_descriptors",), "exactly one metric descriptor is required per PatchDomain")

    _require_refs(issues, {domain.owner_patch_id for domain in snapshot.patch_domains}, patch_ids, ("patch_domains", "owner_patch_id"))
    _require_refs(issues, snapshot.surface_ir.source_vertex_ids, vertex_ids, ("surface_ir", "source_vertex_ids"))
    if snapshot.surface_ir.source_vertex_ids != vertex_ids:
        _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("surface_ir", "source_vertex_ids"), "surface vertex set must equal snapshot vertex set")

    full_surface = snapshot.surface_ir.payload_mode is SurfacePayloadMode.FULL_HOST_SURFACE
    fixture_payload = AnalysisCapability.EC0_COORDINATE_FREE_FIXTURE_V5 in snapshot.analysis_capabilities
    if full_surface:
        if fixture_payload:
            _issue(issues, ValidationCode.CAPABILITY, ("analysis_capabilities",), "fixture capability cannot label full host surface")
        full_required = {
            AnalysisCapability.AUTHORITATIVE_SURFACE_METRIC_V1,
            AnalysisCapability.PHYSICAL_EDGE_TABLE_V1,
            AnalysisCapability.BOUNDARY_CONSTRAINT_TARGET_V1,
            AnalysisCapability.TYPED_JUNCTION_ROUTE_TOPOLOGY_V1,
            AnalysisCapability.TERMINAL_RELATION_V1,
        }
        if full_required - set(snapshot.analysis_capabilities):
            _issue(issues, ValidationCode.CAPABILITY, ("analysis_capabilities",), "full host surface lacks R1 geometry-boundary capabilities")
        for vertex in snapshot.source_vertices:
            if not isinstance(vertex.position, LocalPoint3V1):
                _issue(issues, ValidationCode.CAPABILITY, ("source_vertices", str(vertex.vertex_id), "position"), "full surface requires local coordinates")
    elif not fixture_payload:
        _issue(issues, ValidationCode.CAPABILITY, ("analysis_capabilities",), "coordinate-free payload requires explicit fixture capability")

    domain_by_id = {item.patch_domain_id: item for item in snapshot.patch_domains}
    use_by_id = {item.chain_use_id: item for item in snapshot.chain_uses}
    chain_by_id = {item.physical_chain_id: item for item in snapshot.physical_chains}
    loop_by_id = {item.boundary_loop_id: item for item in snapshot.boundary_loops}
    boundary_by_id = {
        item.boundary_constraint_id: item for item in snapshot.boundary_constraints
    }
    edge_ids = _check_unique(issues, snapshot.surface_ir.source_edges, "edge_id", "surface_ir.source_edges")
    edge_by_id = {item.edge_id: item for item in snapshot.surface_ir.source_edges}
    face_ids = _check_unique(issues, snapshot.surface_ir.source_faces, "face_id", "surface_ir.source_faces")
    triangle_ids = _check_unique(issues, snapshot.surface_ir.surface_triangles, "triangle_id", "surface_ir.surface_triangles")
    triangle_by_id = {item.triangle_id: item for item in snapshot.surface_ir.surface_triangles}
    for edge in snapshot.surface_ir.source_edges:
        _require_refs(issues, {edge.vertex_a_id, edge.vertex_b_id}, vertex_ids, ("surface_ir", "source_edges", str(edge.edge_id), "endpoints"))
    for face in snapshot.surface_ir.source_faces:
        _require_refs(issues, {face.patch_id}, patch_ids, ("surface_ir", "source_faces", str(face.face_id), "patch_id"))
        _require_refs(issues, set(face.vertex_cycle), vertex_ids, ("surface_ir", "source_faces", str(face.face_id), "vertex_cycle"))
        _require_refs(issues, set(face.edge_cycle), edge_ids, ("surface_ir", "source_faces", str(face.face_id), "edge_cycle"))
        _require_refs(issues, set(face.triangle_ids), triangle_ids, ("surface_ir", "source_faces", str(face.face_id), "triangle_ids"))
        for index, edge_id in enumerate(face.edge_cycle):
            edge = edge_by_id.get(edge_id)
            if edge is None:
                continue
            expected = {
                face.vertex_cycle[index],
                face.vertex_cycle[(index + 1) % len(face.vertex_cycle)],
            }
            if {edge.vertex_a_id, edge.vertex_b_id} != expected:
                _issue(issues, ValidationCode.SURFACE_TOPOLOGY, ("surface_ir", "source_faces", str(face.face_id), "edge_cycle", str(index)), "physical edge endpoints do not match the face vertex cycle")
    for triangle in snapshot.surface_ir.surface_triangles:
        _require_refs(issues, {triangle.source_face_id}, face_ids, ("surface_ir", "surface_triangles", str(triangle.triangle_id), "source_face_id"))
        _require_refs(issues, set(triangle.vertex_ids), vertex_ids, ("surface_ir", "surface_triangles", str(triangle.triangle_id), "vertex_ids"))
        _require_refs(issues, {edge_id for edge_id in triangle.physical_edge_ids if edge_id is not None}, edge_ids, ("surface_ir", "surface_triangles", str(triangle.triangle_id), "physical_edge_ids"))
        triangle_pairs = (
            {triangle.vertex_ids[0], triangle.vertex_ids[1]},
            {triangle.vertex_ids[1], triangle.vertex_ids[2]},
            {triangle.vertex_ids[2], triangle.vertex_ids[0]},
        )
        for index, edge_id in enumerate(triangle.physical_edge_ids):
            if edge_id is None or edge_id not in edge_by_id:
                continue
            edge = edge_by_id[edge_id]
            if {edge.vertex_a_id, edge.vertex_b_id} != triangle_pairs[index]:
                _issue(issues, ValidationCode.SURFACE_TOPOLOGY, ("surface_ir", "surface_triangles", str(triangle.triangle_id), "physical_edge_ids", str(index)), "triangle physical edge does not match its endpoint pair")

    listed_triangle_owners: dict[OpaqueId, list[OpaqueId]] = {}
    for face in snapshot.surface_ir.source_faces:
        expected = {triangle.triangle_id for triangle in snapshot.surface_ir.surface_triangles if triangle.source_face_id == face.face_id}
        if set(face.triangle_ids) != expected:
            _issue(issues, ValidationCode.SURFACE_TOPOLOGY, ("surface_ir", "source_faces", str(face.face_id), "triangle_ids"), "face must list all and only triangles whose source_face_id names it")
        for triangle_id in face.triangle_ids:
            listed_triangle_owners.setdefault(triangle_id, []).append(face.face_id)
            triangle = triangle_by_id.get(triangle_id)
            if triangle is not None and triangle.source_face_id != face.face_id:
                _issue(issues, ValidationCode.SURFACE_TOPOLOGY, ("surface_ir", "source_faces", str(face.face_id), "triangle_ids"), "triangle source_face_id disagrees with face listing")
    for triangle_id in triangle_ids:
        if len(listed_triangle_owners.get(triangle_id, [])) != 1:
            _issue(issues, ValidationCode.SURFACE_TOPOLOGY, ("surface_ir", "surface_triangles", str(triangle_id)), "triangle must be listed by exactly one source face")

    patch_vertices: dict[OpaqueId, set[OpaqueId]] = {patch_id: set() for patch_id in patch_ids}
    for face in snapshot.surface_ir.source_faces:
        patch_vertices.setdefault(face.patch_id, set()).update(face.vertex_cycle)
    planarity_ids: set[OpaqueId] = set()
    for descriptor in snapshot.surface_metric_descriptors:
        path = ("surface_metric_descriptors", str(descriptor.patch_domain_id))
        domain = domain_by_id.get(descriptor.patch_domain_id)
        if isinstance(descriptor, UnavailableSurfaceMetricDescriptorV1):
            if full_surface:
                _issue(issues, ValidationCode.SURFACE_METRIC, path, "FULL_HOST_SURFACE cannot use an unavailable metric descriptor")
            continue
        if domain is None:
            continue
        if isinstance(descriptor, PlanarPatchFrameV1):
            certificate = descriptor.planarity_certificate
            if certificate.certificate_id in planarity_ids:
                _issue(issues, ValidationCode.DUPLICATE_ID, path + ("planarity_certificate",), "duplicate planarity certificate ID")
            planarity_ids.add(certificate.certificate_id)
            if domain.surface_regime is not SurfaceRegime.PLANAR:
                _issue(issues, ValidationCode.SURFACE_METRIC, path, "PlanarPatchFrameV1 requires a PLANAR PatchDomain")
            if certificate.patch_domain_id != descriptor.patch_domain_id or not certificate.exact or certificate.max_absolute_plane_distance.value != 0:
                _issue(issues, ValidationCode.SURFACE_METRIC, path + ("planarity_certificate",), "exact planar frame requires a zero-distance certificate for the same domain")
            for field_name in ("axis_u", "axis_v", "normal"):
                vector = getattr(descriptor, field_name)
                if vector.x == 0 and vector.y == 0 and vector.z == 0:
                    _issue(issues, ValidationCode.SURFACE_METRIC, path + (field_name,), "authoritative frame vector cannot be zero")
            coordinate_ids = [item.source_vertex_id for item in descriptor.source_vertex_coordinates]
            if len(coordinate_ids) != len(set(coordinate_ids)):
                _issue(issues, ValidationCode.DUPLICATE_ID, path + ("source_vertex_coordinates",), "duplicate source vertex coordinate")
            _require_refs(issues, set(coordinate_ids), vertex_ids, path + ("source_vertex_coordinates",))
            required_vertices = patch_vertices.get(domain.owner_patch_id, set())
            if full_surface and set(coordinate_ids) != required_vertices:
                _issue(issues, ValidationCode.SURFACE_METRIC, path + ("source_vertex_coordinates",), "planar coordinates must cover all and only owner-patch surface vertices")
        elif isinstance(descriptor, RationalAffinePlanarMetricV2):
            for metric_issue in validate_rational_affine_planar_metric(
                descriptor
            ):
                _issue(
                    issues,
                    metric_issue.code,
                    path + metric_issue.path[1:],
                    metric_issue.message,
                )
            certificate = descriptor.planarity_certificate
            if certificate.certificate_id in planarity_ids:
                _issue(issues, ValidationCode.DUPLICATE_ID, path + ("planarity_certificate",), "duplicate planarity certificate ID")
            planarity_ids.add(certificate.certificate_id)
            if domain.surface_regime is not SurfaceRegime.PLANAR:
                _issue(issues, ValidationCode.SURFACE_METRIC, path, "RationalAffinePlanarMetricV2 requires a PLANAR PatchDomain")
            if descriptor.source_revision != snapshot.source_revision:
                _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, path + ("source_revision",), "metric and snapshot revisions differ")
            coordinate_by_id = {
                item.source_vertex_id: item.domain_coordinate
                for item in descriptor.exact_source_vertex_coordinates
            }
            coordinate_ids = set(coordinate_by_id)
            _require_refs(issues, coordinate_ids, vertex_ids, path + ("exact_source_vertex_coordinates",))
            required_vertices = patch_vertices.get(domain.owner_patch_id, set())
            if full_surface and coordinate_ids != required_vertices:
                _issue(issues, ValidationCode.SURFACE_METRIC, path + ("exact_source_vertex_coordinates",), "exact affine coordinates must cover all and only owner-patch surface vertices")
            origin = _fraction_point3(descriptor.exact_origin)
            basis_a = _fraction_point3(descriptor.exact_basis_a)
            basis_b = _fraction_point3(descriptor.exact_basis_b)
            for vertex_id in coordinate_ids:
                source = next(
                    (
                        item
                        for item in snapshot.source_vertices
                        if item.vertex_id == vertex_id
                    ),
                    None,
                )
                if source is None or not isinstance(source.position, LocalPoint3V1):
                    continue
                position = _position_under_grid_law(
                    source.position, descriptor.grid_certificate
                )
                coordinate = coordinate_by_id[vertex_id]
                u = _fraction(coordinate.x)
                v = _fraction(coordinate.y)
                reconstructed = tuple(
                    origin[index]
                    + u * basis_a[index]
                    + v * basis_b[index]
                    for index in range(3)
                )
                if reconstructed != position:
                    _issue(issues, ValidationCode.SURFACE_METRIC, path + ("exact_source_vertex_coordinates", str(vertex_id)), "exact affine reconstruction disagrees with the source position the declared grid law produces")
        elif isinstance(descriptor, IntrinsicSurfaceMetricDescriptorV1):
            if descriptor.surface_regime != domain.surface_regime or descriptor.surface_regime is SurfaceRegime.PLANAR:
                _issue(issues, ValidationCode.SURFACE_METRIC, path, "intrinsic metric regime must match a non-planar PatchDomain")

    all_sector_ids: set[OpaqueId] = set(angular_sector_ids)
    for domain in snapshot.patch_domains:
        all_sector_ids.update(sector.owner_sector_id for sector in domain.sectors)
        _require_refs(issues, set(domain.boundary_constraint_ids), boundary_ids, ("patch_domains", str(domain.patch_domain_id), "boundary_constraint_ids"))
        _require_refs(issues, {sector.chain_use_id for sector in domain.sectors}, use_ids, ("patch_domains", str(domain.patch_domain_id), "sectors"))
        for constraint_id in domain.boundary_constraint_ids:
            constraint = boundary_by_id.get(constraint_id)
            if constraint is not None and constraint.patch_domain_id != domain.patch_domain_id:
                _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("patch_domains", str(domain.patch_domain_id), "boundary_constraint_ids"), "boundary constraint belongs to another PatchDomain")
        for sector in domain.sectors:
            use = use_by_id.get(sector.chain_use_id)
            if use is not None and (
                use.patch_domain_id != domain.patch_domain_id
                or use.owner_patch_id != domain.owner_patch_id
            ):
                _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("patch_domains", str(domain.patch_domain_id), "sectors", str(sector.owner_sector_id)), "domain sector ChainUse has another owner/domain")
    for chain in snapshot.physical_chains:
        _require_refs(issues, set(chain.ordered_source_vertex_ids), vertex_ids, ("physical_chains", str(chain.physical_chain_id), "ordered_source_vertex_ids"))
        _require_refs(issues, set(chain.ordered_physical_edge_ids), edge_ids, ("physical_chains", str(chain.physical_chain_id), "ordered_physical_edge_ids"))
        if full_surface:
            vertex_count = len(chain.ordered_source_vertex_ids)
            edge_count = len(chain.ordered_physical_edge_ids)
            expected_edge_count = vertex_count if chain.is_closed else vertex_count - 1
            if vertex_count < 2 or edge_count != expected_edge_count:
                _issue(issues, ValidationCode.SURFACE_TOPOLOGY, ("physical_chains", str(chain.physical_chain_id)), "open chains require E=V-1 and closed chains require E=V")
            else:
                for index, edge_id in enumerate(chain.ordered_physical_edge_ids):
                    edge = edge_by_id.get(edge_id)
                    if edge is None:
                        continue
                    next_index = (index + 1) % vertex_count
                    expected = {chain.ordered_source_vertex_ids[index], chain.ordered_source_vertex_ids[next_index]}
                    if {edge.vertex_a_id, edge.vertex_b_id} != expected:
                        _issue(issues, ValidationCode.SURFACE_TOPOLOGY, ("physical_chains", str(chain.physical_chain_id), "ordered_physical_edge_ids", str(index)), "chain edge endpoints do not match the ordered vertex path")
    for use in snapshot.chain_uses:
        _require_refs(issues, {use.physical_chain_id}, chain_ids, ("chain_uses", str(use.chain_use_id), "physical_chain_id"))
        _require_refs(issues, {use.owner_patch_id}, patch_ids, ("chain_uses", str(use.chain_use_id), "owner_patch_id"))
        _require_refs(issues, {use.patch_domain_id}, domain_ids, ("chain_uses", str(use.chain_use_id), "patch_domain_id"))
        _require_refs(issues, {use.launch_locus.boundary_constraint_id}, boundary_ids, ("chain_uses", str(use.chain_use_id), "launch_locus"))
        domain = domain_by_id.get(use.patch_domain_id)
        if domain is not None and domain.owner_patch_id != use.owner_patch_id:
            _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("chain_uses", str(use.chain_use_id)), "ChainUse owner differs from PatchDomain owner")
        launch_constraint = boundary_by_id.get(use.launch_locus.boundary_constraint_id)
        if launch_constraint is not None and launch_constraint.patch_domain_id != use.patch_domain_id:
            _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("chain_uses", str(use.chain_use_id), "launch_locus"), "launch constraint belongs to another PatchDomain")
        if use.boundary_loop_id is not None:
            _require_refs(issues, {use.boundary_loop_id}, loop_ids, ("chain_uses", str(use.chain_use_id), "boundary_loop_id"))
        elif full_surface:
            _issue(issues, ValidationCode.MISSING_REFERENCE, ("chain_uses", str(use.chain_use_id), "boundary_loop_id"), "full host ChainUse requires a boundary loop")
    for loop in snapshot.boundary_loops:
        _require_refs(issues, {loop.patch_domain_id}, domain_ids, ("boundary_loops", str(loop.boundary_loop_id), "patch_domain_id"))
        _require_refs(issues, set(loop.ordered_chain_use_ids), use_ids, ("boundary_loops", str(loop.boundary_loop_id), "ordered_chain_use_ids"))
        for use_id in loop.ordered_chain_use_ids:
            use = use_by_id.get(use_id)
            if use is not None and (
                use.patch_domain_id != loop.patch_domain_id
                or use.boundary_loop_id != loop.boundary_loop_id
            ):
                _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("boundary_loops", str(loop.boundary_loop_id), "ordered_chain_use_ids"), "BoundaryLoop and ChainUse references are not reciprocal")
    for constraint in snapshot.boundary_constraints:
        _require_refs(issues, {constraint.patch_domain_id}, domain_ids, ("boundary_constraints", str(constraint.boundary_constraint_id), "patch_domain_id"))
        if constraint.originating_chain_use_id is not None:
            _require_refs(issues, {constraint.originating_chain_use_id}, use_ids, ("boundary_constraints", str(constraint.boundary_constraint_id), "originating_chain_use_id"))
            originating_use = use_by_id.get(constraint.originating_chain_use_id)
            if originating_use is not None and originating_use.patch_domain_id != constraint.patch_domain_id:
                _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("boundary_constraints", str(constraint.boundary_constraint_id), "originating_chain_use_id"), "originating ChainUse belongs to another PatchDomain")
        target_path = ("boundary_constraints", str(constraint.boundary_constraint_id), "target")
        if isinstance(constraint.target, BoundaryLoopConstraintTargetV1):
            _require_refs(issues, {constraint.target.boundary_loop_id}, loop_ids, target_path)
            loop = loop_by_id.get(constraint.target.boundary_loop_id)
            if loop is not None and loop.patch_domain_id != constraint.patch_domain_id:
                _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, target_path, "target BoundaryLoop belongs to another PatchDomain")
        elif isinstance(constraint.target, ChainUseConstraintTargetV1):
            _require_refs(issues, {constraint.target.chain_use_id}, use_ids, target_path)
            use = use_by_id.get(constraint.target.chain_use_id)
            if use is not None and use.patch_domain_id != constraint.patch_domain_id:
                _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, target_path, "target ChainUse belongs to another PatchDomain")
        elif isinstance(constraint.target, PhysicalEdgeSequenceConstraintTargetV1):
            _require_refs(issues, set(constraint.target.ordered_physical_edge_ids), edge_ids, target_path)
            if full_surface and not constraint.target.ordered_physical_edge_ids:
                _issue(issues, ValidationCode.SURFACE_TOPOLOGY, target_path, "physical-edge boundary target cannot be empty")
        elif isinstance(constraint.target, UnavailableBoundaryConstraintTargetV1) and full_surface:
            _issue(issues, ValidationCode.SURFACE_TOPOLOGY, target_path, "FULL_HOST_SURFACE requires a concrete boundary target")
    for sector in snapshot.angular_owner_sectors:
        _require_refs(issues, {sector.owner_patch_id}, patch_ids, ("angular_owner_sectors", str(sector.owner_sector_id), "owner_patch_id"))
        _require_refs(issues, {sector.patch_domain_id}, domain_ids, ("angular_owner_sectors", str(sector.owner_sector_id), "patch_domain_id"))
        _require_refs(issues, set(sector.ordered_incident_chain_use_ids), use_ids, ("angular_owner_sectors", str(sector.owner_sector_id), "ordered_incident_chain_use_ids"))
        for use_id in sector.ordered_incident_chain_use_ids:
            use = use_by_id.get(use_id)
            if use is not None and (
                use.owner_patch_id != sector.owner_patch_id
                or use.patch_domain_id != sector.patch_domain_id
            ):
                _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("angular_owner_sectors", str(sector.owner_sector_id), "ordered_incident_chain_use_ids"), "oriented sector support has another owner/domain")
        if len(sector.ordered_incident_chain_use_ids) < 2:
            _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, ("angular_owner_sectors", str(sector.owner_sector_id)), "ordered incident supports are required")
        elif (
            sector.incoming_support_ref.chain_use_id != sector.ordered_incident_chain_use_ids[0]
            or sector.outgoing_support_ref.chain_use_id != sector.ordered_incident_chain_use_ids[-1]
        ):
            _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, ("angular_owner_sectors", str(sector.owner_sector_id)), "incoming/outgoing refs must match ordered supports")
        _require_refs(issues, {sector.incoming_support_ref.source_launch_boundary_id, sector.outgoing_support_ref.source_launch_boundary_id}, boundary_ids, ("angular_owner_sectors", str(sector.owner_sector_id), "support_refs"))
        for label, support_ref in (
            ("incoming_support_ref", sector.incoming_support_ref),
            ("outgoing_support_ref", sector.outgoing_support_ref),
        ):
            payload = support_ref.direction_payload
            path = ("angular_owner_sectors", str(sector.owner_sector_id), label, "direction_payload")
            if isinstance(payload, UnavailableSourceSupportDirectionV1):
                if full_surface:
                    _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, path, "FULL_HOST_SURFACE requires an authoritative planar support direction")
            elif isinstance(payload, CertifiedPlanarSupportDirectionV1):
                direction = payload.direction_in_domain
                if direction.x == 0 and direction.y == 0:
                    _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, path, "support direction cannot be zero")
    for certificate in snapshot.reflex_angle_certificates:
        _require_refs(issues, {certificate.owner_sector_id}, angular_sector_ids, ("reflex_angle_certificates", str(certificate.certificate_id), "owner_sector_id"))
        if certificate.exact_two_pi:
            _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, ("reflex_angle_certificates", str(certificate.certificate_id), "exact_two_pi"), "2*pi is terminal/junction, never Angular")
        sector = next((item for item in snapshot.angular_owner_sectors if item.owner_sector_id == certificate.owner_sector_id), None)
        payload = certificate.measure_payload
        if isinstance(payload, UnavailableReflexAngleMeasureV1):
            if full_surface:
                _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, ("reflex_angle_certificates", str(certificate.certificate_id), "measure_payload"), "FULL_HOST_SURFACE requires a certified angle payload")
        elif isinstance(payload, CertifiedReflexAngleMeasureV1):
            phi = payload.phi_over_pi
            delta = payload.reflex_excess_over_pi
            if not _interval_strictly_above(phi, Decimal(1)) or not _interval_strictly_below(phi, Decimal(2)):
                _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, ("reflex_angle_certificates", str(certificate.certificate_id), "phi_over_pi"), "certified phi interval must be strictly inside (1, 2)")
            if (
                delta.lower != phi.lower - Decimal(1)
                or delta.upper != phi.upper - Decimal(1)
                or delta.lower_kind is not phi.lower_kind
                or delta.upper_kind is not phi.upper_kind
            ):
                _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, ("reflex_angle_certificates", str(certificate.certificate_id), "reflex_excess_over_pi"), "delta interval must equal phi_over_pi minus one exactly")
            if sector is not None and payload.orientation is not sector.turn_orientation:
                _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, ("reflex_angle_certificates", str(certificate.certificate_id), "orientation"), "angle orientation differs from owner sector")
    for relation in snapshot.corner_relations:
        _require_refs(issues, {relation.source_vertex_id}, vertex_ids, ("corner_relations", str(relation.corner_relation_id), "source_vertex_id"))
        _require_refs(issues, {relation.owner_sector_id}, angular_sector_ids, ("corner_relations", str(relation.corner_relation_id), "owner_sector_id"))
        _require_refs(issues, {relation.reflex_angle_certificate_id}, angle_certificate_ids, ("corner_relations", str(relation.corner_relation_id), "reflex_angle_certificate_id"))
        if relation.exact_two_pi:
            _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, ("corner_relations", str(relation.corner_relation_id)), "2*pi CornerRelation is forbidden")
    all_route_pairing_ids: set[OpaqueId] = set()
    for relation in snapshot.junction_relations:
        _require_refs(issues, {relation.source_vertex_id}, vertex_ids, ("junction_relations", str(relation.junction_relation_id), "source_vertex_id"))
        _require_refs(issues, set(relation.incident_chain_use_ids), use_ids, ("junction_relations", str(relation.junction_relation_id), "incident_chain_use_ids"))
        topology = relation.route_topology
        path = ("junction_relations", str(relation.junction_relation_id), "route_topology")
        if isinstance(topology, UnavailableJunctionRouteTopologyV1):
            if full_surface:
                _issue(issues, ValidationCode.ROUTE_TOPOLOGY, path, "FULL_HOST_SURFACE requires typed junction route topology")
            continue
        expected_type = {
            JunctionRelationKind.T: TJunctionRouteTopologyV1,
            JunctionRelationKind.X: XJunctionRouteTopologyV1,
            JunctionRelationKind.Y: YJunctionRouteTopologyV1,
            JunctionRelationKind.CROSS_PATCH: DeclaredRoutePairsTopologyV1,
            JunctionRelationKind.MIXED_ALPHA: DeclaredRoutePairsTopologyV1,
        }[relation.relation_kind]
        if not isinstance(topology, expected_type):
            _issue(issues, ValidationCode.ROUTE_TOPOLOGY, path, "route topology variant does not match junction kind")
        route_pairs = _junction_route_pairs(topology)
        pair_ids = [pair.route_pairing_id for pair in route_pairs]
        if len(pair_ids) != len(set(pair_ids)):
            _issue(issues, ValidationCode.DUPLICATE_ID, path, "duplicate route pairing ID")
        duplicate_global_ids = set(pair_ids) & all_route_pairing_ids
        if duplicate_global_ids:
            _issue(issues, ValidationCode.DUPLICATE_ID, path, "route pairing ID is reused by another JunctionRelation")
        all_route_pairing_ids.update(pair_ids)
        referenced_uses: set[OpaqueId] = set()
        for pair in route_pairs:
            if pair.first_chain_use_id == pair.second_chain_use_id:
                _issue(issues, ValidationCode.ROUTE_TOPOLOGY, path + (str(pair.route_pairing_id),), "route pair endpoints must differ")
            referenced_uses.update({pair.first_chain_use_id, pair.second_chain_use_id})
        if isinstance(topology, TJunctionRouteTopologyV1):
            referenced_uses.update(topology.branch_chain_use_ids)
        elif isinstance(topology, YJunctionRouteTopologyV1):
            referenced_uses.add(topology.trunk_chain_use_id)
            referenced_uses.update(topology.ordered_branch_chain_use_ids)
        _require_refs(issues, referenced_uses, set(relation.incident_chain_use_ids), path)
        if full_surface and referenced_uses != set(relation.incident_chain_use_ids):
            _issue(issues, ValidationCode.ROUTE_TOPOLOGY, path, "typed route topology must cover every incident ChainUse")

    for relation in snapshot.terminal_relations:
        path = ("terminal_relations", str(relation.terminal_relation_id))
        _require_refs(issues, {relation.source_vertex_id}, vertex_ids, path + ("source_vertex_id",))
        _require_refs(issues, {relation.chain_use_id}, use_ids, path + ("chain_use_id",))
        _require_refs(issues, {relation.owner_patch_id}, patch_ids, path + ("owner_patch_id",))
        _require_refs(issues, {relation.patch_domain_id}, domain_ids, path + ("patch_domain_id",))
        if relation.owner_sector_id is not None:
            _require_refs(issues, {relation.owner_sector_id}, all_sector_ids, path + ("owner_sector_id",))
        use = use_by_id.get(relation.chain_use_id)
        if use is not None:
            if use.owner_patch_id != relation.owner_patch_id or use.patch_domain_id != relation.patch_domain_id:
                _issue(issues, ValidationCode.TERMINAL_RELATION, path, "terminal owner patch/domain differs from ChainUse")
            chain = chain_by_id.get(use.physical_chain_id)
            if chain is not None and chain.ordered_source_vertex_ids:
                if relation.endpoint_role is TerminalEndpointRole.START and relation.source_vertex_id != chain.ordered_source_vertex_ids[0]:
                    _issue(issues, ValidationCode.TERMINAL_RELATION, path, "START terminal is not the first physical-chain vertex")
                if relation.endpoint_role is TerminalEndpointRole.END and relation.source_vertex_id != chain.ordered_source_vertex_ids[-1]:
                    _issue(issues, ValidationCode.TERMINAL_RELATION, path, "END terminal is not the last physical-chain vertex")
    return tuple(issues)


def validate_decal_request(request: DecalRequestV1) -> tuple[ValidationIssue, ...]:
    issues: list[ValidationIssue] = []
    expected = (
        (request.schema_version == DECAL_REQUEST_SCHEMA_V1, "schema_version"),
        (request.metric_space is MetricSpace.SOURCE_LOCAL_INTRINSIC, "metric_space"),
        (request.angular_profile_family_id is AngularProfileFamilyId.LINEAR_REFLEX_EQUAL_V1, "angular_profile_family_id"),
        (request.angular_profile_selection_policy_id is AngularProfileSelectionPolicyId.MIN_K_FOR_MAX_SUBTURN_V1, "angular_profile_selection_policy_id"),
        (request.max_subturn_parameter_id is MaxSubturnParameterId.LINEAR_REFLEX_MAX_SUBTURN_V1, "max_subturn_parameter_id"),
        (request.max_subturn_value_id is MaxSubturnValueId.LINEAR_REFLEX_MAX_SUBTURN_60_DEGREES_V1, "max_subturn_value_id"),
        (request.max_subturn_exact_value.symbol is ExactAngleSymbol.PI_OVER_3, "max_subturn_exact_value"),
        (request.cap_policy_id is CapPolicyId.PHYSICAL_TERMINAL_LINEAR_CLOSURE_V1, "cap_policy_id"),
        (request.boundary_policy_id is BoundaryPolicyId.BOUNDARY_LIMITED_PROPAGATION, "boundary_policy_id"),
        (request.interaction_policy_id is InteractionPolicyId.INTRAPATCH_POLICY_B_V1, "interaction_policy_id"),
        (request.ownership_policy_id is OwnershipPolicyId.TOTAL_DISJOINT_RESOLVED_COVERAGE_V1, "ownership_policy_id"),
    )
    for valid, field in expected:
        if not valid:
            code = ValidationCode.SCHEMA_VERSION if field == "schema_version" else ValidationCode.POLICY_MISMATCH
            _issue(issues, code, (field,), "unsupported v1 value")
    if not request.selected_chain_use_ids:
        _issue(issues, ValidationCode.MISSING_REFERENCE, ("selected_chain_use_ids",), "at least one ChainUse is required")
    return tuple(issues)


def validate_snapshot_request_references(
    snapshot: AnalysisSnapshotV1,
    request: DecalRequestV1,
) -> tuple[ValidationIssue, ...]:
    """Validate the complete public compile input before any scope resolution."""

    issues = list(validate_analysis_snapshot(snapshot))
    issues.extend(validate_decal_request(request))
    use_ids = _values(snapshot.chain_uses, "chain_use_id")
    _require_refs(
        issues,
        set(request.selected_chain_use_ids),
        use_ids,
        ("request", "selected_chain_use_ids"),
    )
    return tuple(issues)


def _seed_id(seed: object) -> OpaqueId:
    return seed.seed_id


def _spec_variant(spec: object) -> EnvelopeSpecVariant:
    if isinstance(spec, StripEnvelopeSpec):
        return EnvelopeSpecVariant.STRIP
    if isinstance(spec, AngularEnvelopeSpec):
        return EnvelopeSpecVariant.ANGULAR
    if isinstance(spec, JunctionEnvelopeSpec):
        return EnvelopeSpecVariant.JUNCTION
    if isinstance(spec, CapEnvelopeSpec):
        return EnvelopeSpecVariant.CAP
    raise TypeError(type(spec).__name__)


def validate_compiled_plan(plan: CompiledPatchEvaluationPlanV1) -> tuple[ValidationIssue, ...]:
    issues: list[ValidationIssue] = []
    if plan.schema_version != COMPILED_PLAN_SCHEMA_V1:
        _issue(issues, ValidationCode.SCHEMA_VERSION, ("schema_version",), "unsupported plan schema")
    request_id = plan.plan_key.decal_request_id
    domain_id = plan.plan_key.patch_domain_id
    seed_ids = {_seed_id(seed) for seed in plan.seeds}
    if len(seed_ids) != len(plan.seeds):
        _issue(issues, ValidationCode.DUPLICATE_ID, ("seeds",), "duplicate seed ID")
    component_ids = _check_unique(issues, plan.front_components, "front_component_id", "front_components")
    certificate_ids = _check_unique(issues, plan.angular_profile_selection_certificates, "certificate_id", "angular_profile_selection_certificates")
    spec_ids = _check_unique(issues, plan.envelope_specs, "envelope_spec_id", "envelope_specs")
    instance_ids = _check_unique(issues, plan.envelope_instances, "instance_id", "envelope_instances")
    predicate_ids = _check_unique(issues, plan.event_predicates, "predicate_id", "event_predicates")

    scoped_records = (
        list(plan.seeds)
        + list(plan.front_components)
        + list(plan.angular_profile_selection_certificates)
        + list(plan.envelope_specs)
        + list(plan.envelope_instances)
        + list(plan.event_predicates)
        + list(plan.event_transitions)
        + list(plan.interactions)
    )
    for record in scoped_records:
        if record.decal_request_id != request_id or record.patch_domain_id != domain_id:
            _issue(issues, ValidationCode.PLAN_KEY_MISMATCH, (type(record).__name__,), "record request/domain differs from plan key")

    front_seed_ids = {seed.seed_id for seed in plan.seeds if isinstance(seed, FrontSeedV1)}
    corner_seed_ids = {seed.seed_id for seed in plan.seeds if isinstance(seed, CornerSeedV1)}
    junction_seed_ids = {seed.seed_id for seed in plan.seeds if isinstance(seed, JunctionSeedV1)}
    cap_seed_ids = {seed.seed_id for seed in plan.seeds if isinstance(seed, CapSeedV1)}
    for component in plan.front_components:
        _require_refs(issues, {component.front_seed_id}, front_seed_ids, ("front_components", str(component.front_component_id), "front_seed_id"))
        if component.initial_branch_count != 1:
            _issue(issues, ValidationCode.POLICY_MISMATCH, ("front_components", str(component.front_component_id), "initial_branch_count"), "ordinary component starts with one branch")

    certificate_by_id = {certificate.certificate_id: certificate for certificate in plan.angular_profile_selection_certificates}
    for certificate in plan.angular_profile_selection_certificates:
        k = certificate.resolved_hidden_edge_count
        interval = certificate.selection_interval_certificate
        if k < 0 or certificate.resolved_subturn_count != k + 1 or certificate.local_profile_support_count != k + 2 or certificate.local_profile_segment_count != k + 2:
            _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, ("angular_profile_selection_certificates", str(certificate.certificate_id)), "k+1/k+2 cardinality law violated")
        if interval.lower_bound_integer != k or interval.upper_bound_integer != k + 1:
            _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, ("angular_profile_selection_certificates", str(certificate.certificate_id), "selection_interval_certificate"), "certificate must encode open k < ratio <= k+1")

    spec_by_id = {spec.envelope_spec_id: spec for spec in plan.envelope_specs}
    for spec in plan.envelope_specs:
        path = ("envelope_specs", str(spec.envelope_spec_id))
        if isinstance(spec, StripEnvelopeSpec):
            _require_refs(issues, {spec.source_seed_id}, front_seed_ids, path + ("source_seed_id",))
            _require_refs(issues, set(spec.front_component_ids), component_ids, path + ("front_component_ids",))
            _require_refs(issues, set(spec.terminal_interface_spec_ids), spec_ids, path + ("terminal_interface_spec_ids",))
        elif isinstance(spec, AngularEnvelopeSpec):
            _require_refs(issues, {spec.source_seed_id}, corner_seed_ids, path + ("source_seed_id",))
            _require_refs(issues, {spec.selection_certificate_id}, certificate_ids, path + ("selection_certificate_id",))
            _require_refs(issues, set(spec.incident_front_component_ids), component_ids, path + ("incident_front_component_ids",))
            for support in spec.hidden_supports:
                support_path = path + (
                    "hidden_supports",
                    str(support.ordinal),
                    "direction_law",
                )
                if (
                    type(support) is HiddenSupportSpecV1
                    and support.direction_law
                    is not HiddenSupportDirectionLaw.ORIENTED_OWNER_SECTOR_ORDINAL_SUBTURN
                ):
                    _issue(
                        issues,
                        ValidationCode.ANGULAR_CERTIFICATE,
                        support_path,
                        "legacy hidden support must declare the oriented owner-sector ordinal subturn law",
                    )
                elif (
                    type(support) is CertifiedBoundHiddenSupportSpecV1
                    and support.direction_law
                    is not CertifiedBoundHiddenSupportDirectionLawV1.CERTIFIED_RATIONAL_BINDING_IN_ORDINAL_SUBTURN_V1
                ):
                    _issue(
                        issues,
                        ValidationCode.ANGULAR_CERTIFICATE,
                        support_path,
                        "bound support must declare the certified rational binding law",
                    )
            certificate = certificate_by_id.get(spec.selection_certificate_id)
            if certificate is not None:
                k = certificate.resolved_hidden_edge_count
                if spec.resolved_hidden_edge_count != k or len(spec.hidden_supports) != k:
                    _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, path, "spec k must equal selection certificate and hidden support count")
                expected_ordinals = set(range(1, k + 1))
                if {item.ordinal for item in spec.hidden_supports} != expected_ordinals:
                    _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, path + ("hidden_supports",), "hidden support ordinals must be exactly 1..k")
                for support in spec.hidden_supports:
                    if support.turn_fraction.numerator != support.ordinal or support.turn_fraction.denominator != k + 1:
                        _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, path + ("hidden_supports", str(support.ordinal)), "turn fraction must be ordinal/(k+1)")
                    if isinstance(support, CertifiedBoundHiddenSupportSpecV1):
                        binding = support.direction_binding
                        vector = binding.bound_primitive_integer_vector
                        if (
                            len(vector) != 2
                            or any(type(value) is not int for value in vector)
                            or vector == (0, 0)
                            or gcd(abs(vector[0]), abs(vector[1])) != 1
                        ):
                            _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, path + ("hidden_supports", str(support.ordinal), "direction_binding", "bound_primitive_integer_vector"), "bound direction must be a nonzero primitive integer vector")
                        lower = binding.ideal_window_lower_slope_envelope
                        upper = binding.ideal_window_upper_slope_envelope
                        width = binding.certified_window_width_lower_bound
                        if lower.upper >= upper.lower or width <= 0 or width > upper.lower - lower.upper:
                            _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, path + ("hidden_supports", str(support.ordinal), "direction_binding"), "certified slope envelopes must be strictly ordered and cover the declared positive width")
                        expected_predicates = {
                            "BINDING_MONOTONE",
                            "BINDING_SUBTURN_LE_DELTA_MAX",
                            "BINDING_INSIDE_OWN_ORDINAL_WINDOW",
                        }
                        if set(binding.proven_predicates) != expected_predicates:
                            _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, path + ("hidden_supports", str(support.ordinal), "direction_binding", "proven_predicates"), "direction binding must name exactly the three required predicates")
            if spec.all_support_normal_speed != 1:
                _issue(issues, ValidationCode.POLICY_MISMATCH, path + ("all_support_normal_speed",), "v1 support speed is UNIT")
        elif isinstance(spec, JunctionEnvelopeSpec):
            _require_refs(issues, {spec.source_seed_id}, junction_seed_ids, path + ("source_seed_id",))
            _require_refs(issues, set(spec.incident_front_component_ids), component_ids, path + ("incident_front_component_ids",))
        elif isinstance(spec, CapEnvelopeSpec):
            _require_refs(issues, {spec.source_seed_id}, cap_seed_ids, path + ("source_seed_id",))
            _require_refs(issues, {spec.incident_strip_spec_id}, spec_ids, path + ("incident_strip_spec_id",))

    for instance in plan.envelope_instances:
        path = ("envelope_instances", str(instance.instance_id))
        _require_refs(issues, {instance.spec_id}, spec_ids, path + ("spec_id",))
        _require_refs(issues, set(instance.effective_alpha_binding.front_component_ids), component_ids, path + ("effective_alpha_binding",))
        spec = spec_by_id.get(instance.spec_id)
        if spec is not None and instance.spec_variant is not _spec_variant(spec):
            _issue(issues, ValidationCode.SEED_VARIANT_MISMATCH, path + ("spec_variant",), "instance variant differs from spec")

    if plan.initial_front_spec.decal_request_id != request_id or plan.initial_front_spec.patch_domain_id != domain_id:
        _issue(issues, ValidationCode.PLAN_KEY_MISMATCH, ("initial_front_spec",), "front spec differs from plan key")
    for feature in plan.initial_front_spec.support_features:
        source = feature.source_id
        if isinstance(source, EnvelopeSpecId):
            _require_refs(issues, {source}, spec_ids, ("initial_front_spec", "support_features"))
        elif isinstance(source, HiddenSupportId):
            hidden_ids = {
                support.hidden_support_id
                for spec in plan.envelope_specs
                if isinstance(spec, AngularEnvelopeSpec)
                for support in spec.hidden_supports
            }
            _require_refs(issues, {source}, hidden_ids, ("initial_front_spec", "support_features"))
        if feature.kind is InitialFrontFeatureKind.ANGULAR_HIDDEN_SUPPORT and not isinstance(source, HiddenSupportId):
            _issue(issues, ValidationCode.SEED_VARIANT_MISMATCH, ("initial_front_spec",), "angular hidden feature requires HiddenSupportId")

    expected_participant_types = {
        EventParticipantKind.FRONT_COMPONENT: FrontComponentId,
        EventParticipantKind.ENVELOPE_SPEC: EnvelopeSpecId,
        EventParticipantKind.ENVELOPE_INSTANCE: EnvelopeInstanceId,
        EventParticipantKind.HIDDEN_SUPPORT: HiddenSupportId,
        EventParticipantKind.BOUNDARY_CONSTRAINT: BoundaryConstraintId,
    }
    for predicate in plan.event_predicates:
        for participant in predicate.participants:
            expected_type = expected_participant_types[participant.kind]
            if not isinstance(participant.participant_id, expected_type):
                _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("event_predicates", str(predicate.predicate_id), "participants"), "participant kind and typed ID differ")
    for transition in plan.event_transitions:
        _require_refs(issues, {transition.predicate_id}, predicate_ids, ("event_transitions", str(transition.transition_id), "predicate_id"))

    if plan.raw_coverage.decal_request_id != request_id or plan.raw_coverage.patch_domain_id != domain_id:
        _issue(issues, ValidationCode.PLAN_KEY_MISMATCH, ("raw_coverage",), "raw coverage differs from plan key")
    _require_refs(issues, set(plan.raw_coverage.envelope_instance_ids), instance_ids, ("raw_coverage", "envelope_instance_ids"))
    if plan.resolved_coverage.raw_coverage_id != plan.raw_coverage.coverage_id:
        _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("resolved_coverage", "raw_coverage_id"), "resolved coverage must reference raw coverage")
    interaction_ids = _values(plan.interactions, "interaction_id")
    if plan.resolved_coverage.interaction_ids != interaction_ids:
        _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("resolved_coverage", "interaction_ids"), "resolved coverage interaction set differs")
    for interaction in plan.interactions:
        _require_refs(issues, set(interaction.participant_envelope_instance_ids), instance_ids, ("interactions", str(interaction.interaction_id), "participants"))
        _require_refs(issues, {interaction.event_gate_id}, predicate_ids, ("interactions", str(interaction.interaction_id), "event_gate_id"))
        if interaction.creates_new_matter:
            _issue(issues, ValidationCode.POLICY_MISMATCH, ("interactions", str(interaction.interaction_id), "creates_new_matter"), "policy B never creates new matter")
        if interaction.coverage_effect is CoverageEffect.NONE:
            _issue(issues, ValidationCode.POLICY_MISMATCH, ("interactions", str(interaction.interaction_id), "coverage_effect"), "declared interaction must state a clipping effect")

    ownership = plan.ownership
    if ownership.coverage_id != plan.resolved_coverage.coverage_id:
        _issue(issues, ValidationCode.OWNERSHIP_DECLARATION, ("ownership", "coverage_id"), "ownership must partition resolved coverage")
    ownership_contract_valid = (
        ownership.partition_contract_id is OwnershipPartitionContractId.TOTAL_DISJOINT_RESOLVED_COVERAGE_V1
        and ownership.total
        and ownership.disjoint
        and ownership.silhouette_effect is SilhouetteEffect.NONE
        and ownership.equality_boundary_owner is EqualityBoundaryOwner.NONE
        and ownership.named_unproven_outcome is NamedOutcome.OWNERSHIP_PARTITION_UNPROVEN
        and ownership.claim_interiors_obligation is ClaimInteriorsObligation.PAIRWISE_DISJOINT
        and ownership.claim_closure_obligation is ClaimClosureObligation.EQUALS_RESOLVED_COVERAGE
    )
    if not ownership_contract_valid:
        _issue(issues, ValidationCode.OWNERSHIP_DECLARATION, ("ownership",), "v1 total/disjoint declarations are incomplete")
    claim_ids = _check_unique(issues, ownership.claims, "ownership_claim_id", "ownership.claims")
    del claim_ids
    for claim in ownership.claims:
        if claim.coverage_id != ownership.coverage_id:
            _issue(issues, ValidationCode.OWNERSHIP_DECLARATION, ("ownership", "claims", str(claim.ownership_claim_id), "coverage_id"), "claim points to foreign coverage")
        _require_refs(issues, set(claim.envelope_spec_ids), spec_ids, ("ownership", "claims", str(claim.ownership_claim_id), "envelope_spec_ids"))
        _require_refs(issues, set(claim.envelope_instance_ids), instance_ids, ("ownership", "claims", str(claim.ownership_claim_id), "envelope_instance_ids"))

    tessellation = plan.tessellation_plan
    required_forbidden = set(ForbiddenTessellationOperation)
    tessellation_valid = (
        tessellation.contract_id is TessellationContractId.DOWNSTREAM_TESSELLATION_AFTER_SEMANTIC_COALESCING_V1
        and tessellation.starts_after is TessellationStartStage.SEMANTIC_COALESCING
        and tessellation.shared_vertex_key_policy is SharedVertexKeyPolicy.SEMANTIC_IDENTITY_NOT_COORDINATE
        and tessellation.semantic_digest_equivalence is TessellationDigestEquivalence.ALL_VALID_TESSELLATIONS_SHARE_ONE_SEMANTIC_DIGEST
        and set(tessellation.forbidden_operations) == required_forbidden
    )
    if not tessellation_valid:
        _issue(issues, ValidationCode.TESSELLATION_AUTHORITY, ("tessellation_plan",), "downstream authority boundary is incomplete")
    if plan.geometry_batch_provenance.resolved_coverage_id != plan.resolved_coverage.coverage_id:
        _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("geometry_batch_provenance", "resolved_coverage_id"), "batch provenance must reference resolved coverage")
    return tuple(issues)


def validate_geometry_batch(batch: GeometryBatchV1) -> tuple[ValidationIssue, ...]:
    issues: list[ValidationIssue] = []
    if batch.schema_version != GEOMETRY_BATCH_SCHEMA_V1:
        _issue(issues, ValidationCode.SCHEMA_VERSION, ("schema_version",), "unsupported GeometryBatch schema")
    vertex_keys = _check_unique(issues, batch.vertices, "vert_key", "vertices")
    region_ids = _check_unique(issues, batch.semantic_regions, "semantic_region_id", "semantic_regions")
    _check_unique(issues, batch.station_facts, "station_fact_id", "station_facts")
    region_by_id = {region.semantic_region_id: region for region in batch.semantic_regions}
    face_ids = [face.face_id for face in batch.faces]
    if len(face_ids) != len(set(face_ids)):
        _issue(issues, ValidationCode.DUPLICATE_ID, ("faces",), "duplicate face ID")
    ownership_claim_ids = {region.ownership_claim_id for region in batch.semantic_regions}
    semantic_uv: dict[tuple[OpaqueId, OpaqueId, OpaqueId], object] = {}
    required_station_keys: set[tuple[OpaqueId, OpaqueId, OpaqueId]] = set()
    for vertex in batch.vertices:
        lowered = vertex.vert_key.value.lower()
        if lowered.startswith(("coord:", "xyz:", "rounded:")):
            _issue(issues, ValidationCode.FORBIDDEN_TOPOLOGY_IDENTITY, ("vertices", str(vertex.vert_key), "vert_key"), "VertexKey must not be coordinate-derived")
    for face in batch.faces:
        _require_refs(issues, set(face.ordered_vert_keys), vertex_keys, ("faces", str(face.face_id), "ordered_vert_keys"))
        _require_refs(issues, {face.semantic_region_id}, region_ids, ("faces", str(face.face_id), "semantic_region_id"))
        _require_refs(issues, {face.ownership_claim_id}, ownership_claim_ids, ("faces", str(face.face_id), "ownership_claim_id"))
        region = region_by_id.get(face.semantic_region_id)
        if region is not None and (
            region.ownership_claim_id != face.ownership_claim_id
            or region.material_id != face.material_id
        ):
            _issue(issues, ValidationCode.GEOMETRY_BATCH, ("faces", str(face.face_id)), "face ownership/material differs from its semantic region")
        for uv_fact in face.uv_facts:
            key = (face.semantic_region_id, face.ownership_claim_id, uv_fact.vert_key)
            required_station_keys.add(key)
            previous = semantic_uv.setdefault(key, uv_fact.uv)
            if previous != uv_fact.uv:
                _issue(issues, ValidationCode.GEOMETRY_BATCH, ("faces", str(face.face_id), "uv_facts", str(uv_fact.vert_key)), "semantic vertex has conflicting UV facts")
    station_vertex_keys: set[OpaqueId] = set()
    station_values: dict[tuple[OpaqueId, OpaqueId, OpaqueId], object] = {}
    for fact in batch.station_facts:
        path = ("station_facts", str(fact.station_fact_id))
        station_vertex_keys.add(fact.vert_key)
        station_key = (
            fact.semantic_region_id,
            fact.ownership_claim_id,
            fact.vert_key,
        )
        station_value = (
            fact.source_s,
            fact.source_r,
            fact.station_model_id,
            fact.lineage_ids,
        )
        if station_key in station_values:
            message = (
                "semantic vertex has conflicting station facts"
                if station_values[station_key] != station_value
                else "semantic station key is duplicated"
            )
            _issue(issues, ValidationCode.STATION_FACT, path, message)
        else:
            station_values[station_key] = station_value
        _require_refs(issues, {fact.vert_key}, vertex_keys, path + ("vert_key",))
        _require_refs(issues, {fact.semantic_region_id}, region_ids, path + ("semantic_region_id",))
        _require_refs(issues, {fact.ownership_claim_id}, ownership_claim_ids, path + ("ownership_claim_id",))
        region = region_by_id.get(fact.semantic_region_id)
        if region is not None and region.ownership_claim_id != fact.ownership_claim_id:
            _issue(issues, ValidationCode.STATION_FACT, path, "station ownership differs from semantic region")
    if station_vertex_keys != vertex_keys:
        _issue(issues, ValidationCode.STATION_FACT, ("station_facts",), "every emitted vertex requires at least one station fact")
    if set(station_values) != required_station_keys:
        _issue(issues, ValidationCode.STATION_FACT, ("station_facts",), "station facts must cover every face region/claim/vertex exactly by semantic key")
    for chain in batch.boundary_chains:
        _require_refs(issues, set(chain.ordered_vert_keys), vertex_keys, ("boundary_chains", str(chain.semantic_boundary_id), "ordered_vert_keys"))
    for chain in batch.interface_chains:
        _require_refs(issues, set(chain.ordered_vert_keys), vertex_keys, ("interface_chains", str(chain.semantic_interface_id), "ordered_vert_keys"))
    expected_digest = geometry_batch_semantic_digest(batch).sha256_hex
    if batch.semantic_digest != SemanticDigestValue(expected_digest):
        _issue(issues, ValidationCode.GEOMETRY_BATCH, ("semantic_digest",), "declared semantic digest differs from semantic projection")
    return tuple(issues)


def validate_cross_contract_references(
    snapshot: AnalysisSnapshotV1,
    request: DecalRequestV1,
    plans: tuple[CompiledPatchEvaluationPlanV1, ...],
    geometry_batches: tuple[GeometryBatchV1, ...] = (),
) -> tuple[ValidationIssue, ...]:
    issues = list(validate_analysis_snapshot(snapshot))
    issues.extend(validate_decal_request(request))
    use_by_id = {item.chain_use_id: item for item in snapshot.chain_uses}
    domain_by_id = {item.patch_domain_id: item for item in snapshot.patch_domains}
    sector_by_id = {item.owner_sector_id: item for item in snapshot.angular_owner_sectors}
    front_sector_ids = set(sector_by_id)
    front_sector_ids.update(
        sector.owner_sector_id
        for domain in snapshot.patch_domains
        for sector in domain.sectors
    )
    angle_by_id = {item.certificate_id: item for item in snapshot.reflex_angle_certificates}
    corner_by_id = {item.corner_relation_id: item for item in snapshot.corner_relations}
    junction_by_id = {item.junction_relation_id: item for item in snapshot.junction_relations}
    terminal_by_id = {item.terminal_relation_id: item for item in snapshot.terminal_relations}
    face_ids = _values(snapshot.surface_ir.source_faces, "face_id")
    edge_ids = _values(snapshot.surface_ir.source_edges, "edge_id")
    use_ids = set(use_by_id)
    domain_ids = set(domain_by_id)
    _require_refs(issues, set(request.selected_chain_use_ids), use_ids, ("request", "selected_chain_use_ids"))
    plan_keys: set[tuple[object, object]] = set()
    selected_domains = {
        use_by_id[use_id].patch_domain_id
        for use_id in request.selected_chain_use_ids
        if use_id in use_by_id
    }
    expected_plan_keys = {
        (request.decal_request_id, domain_id) for domain_id in selected_domains
    }
    front_seed_counts = {use_id: 0 for use_id in request.selected_chain_use_ids}
    fixture_payload = AnalysisCapability.EC0_COORDINATE_FREE_FIXTURE_V5 in snapshot.analysis_capabilities
    for plan in plans:
        issues.extend(validate_compiled_plan(plan))
        key = (plan.plan_key.decal_request_id, plan.plan_key.patch_domain_id)
        if key in plan_keys:
            _issue(issues, ValidationCode.DUPLICATE_ID, ("plans",), "duplicate request/domain plan key")
        plan_keys.add(key)
        if plan.source_revision != snapshot.source_revision:
            _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("plans", str(plan.evaluation_plan_id), "source_revision"), "plan and snapshot revisions differ")
        if plan.plan_key.decal_request_id != request.decal_request_id:
            _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("plans", str(plan.evaluation_plan_id), "decal_request_id"), "plan belongs to another request")
        _require_refs(issues, {plan.plan_key.patch_domain_id}, domain_ids, ("plans", str(plan.evaluation_plan_id), "patch_domain_id"))
        domain = domain_by_id.get(plan.plan_key.patch_domain_id)
        if domain is not None and plan.owner_patch_id != domain.owner_patch_id:
            _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("plans", str(plan.evaluation_plan_id), "owner_patch_id"), "plan owner patch differs from Snapshot PatchDomain")

        selection_by_id = {
            item.certificate_id: item
            for item in plan.angular_profile_selection_certificates
        }
        front_seed_by_id = {
            seed.seed_id: seed for seed in plan.seeds if isinstance(seed, FrontSeedV1)
        }
        corner_seed_by_id = {
            seed.seed_id: seed for seed in plan.seeds if isinstance(seed, CornerSeedV1)
        }
        junction_seed_by_id = {
            seed.seed_id: seed for seed in plan.seeds if isinstance(seed, JunctionSeedV1)
        }
        cap_seed_by_id = {
            seed.seed_id: seed for seed in plan.seeds if isinstance(seed, CapSeedV1)
        }
        component_by_id = {
            item.front_component_id: item for item in plan.front_components
        }
        spec_by_id = {item.envelope_spec_id: item for item in plan.envelope_specs}
        instance_ids = _values(plan.envelope_instances, "instance_id")
        hidden_ids = {
            support.hidden_support_id
            for spec in plan.envelope_specs
            if isinstance(spec, AngularEnvelopeSpec)
            for support in spec.hidden_supports
        }

        for seed in plan.seeds:
            path = ("plans", str(plan.evaluation_plan_id), "seeds", str(seed.seed_id))
            if isinstance(seed, FrontSeedV1):
                _require_refs(issues, {seed.chain_use_id}, use_ids, path + ("chain_use_id",))
                use = use_by_id.get(seed.chain_use_id)
                if use is not None and (
                    use.owner_patch_id != seed.owner_patch_id
                    or use.patch_domain_id != seed.patch_domain_id
                ):
                    _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, path, "FrontSeed owner patch/domain differs from ChainUse")
                if seed.chain_use_id not in request.selected_chain_use_ids:
                    _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, path + ("chain_use_id",), "FrontSeed is not selected by the request")
                else:
                    front_seed_counts[seed.chain_use_id] += 1
                _require_refs(issues, set(seed.sector_ids), front_sector_ids, path + ("sector_ids",))
            elif isinstance(seed, CornerSeedV1):
                _require_refs(issues, {seed.source_relation_id}, set(corner_by_id), path + ("source_relation_id",))
                _require_refs(issues, {seed.owner_sector_id}, set(sector_by_id), path + ("owner_sector_id",))
                _require_refs(issues, {seed.profile_selection_certificate_id}, set(selection_by_id), path + ("profile_selection_certificate_id",))
                relation = corner_by_id.get(seed.source_relation_id)
                sector = sector_by_id.get(seed.owner_sector_id)
                if relation is not None and relation.owner_sector_id != seed.owner_sector_id:
                    _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, path, "CornerSeed sector differs from CornerRelation")
                if sector is not None and (
                    sector.owner_patch_id != seed.owner_patch_id
                    or sector.patch_domain_id != seed.patch_domain_id
                    or sector.ordered_incident_chain_use_ids != seed.ordered_incident_chain_use_ids
                ):
                    _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, path, "CornerSeed differs from the oriented owner sector")
            elif isinstance(seed, JunctionSeedV1):
                _require_refs(issues, {seed.source_relation_id}, set(junction_by_id), path + ("source_relation_id",))
                relation = junction_by_id.get(seed.source_relation_id)
                if relation is not None and not set(seed.incident_chain_use_ids).issubset(set(relation.incident_chain_use_ids)):
                    _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, path, "JunctionSeed uses are not contained in JunctionRelation")
            elif isinstance(seed, (CapSeedV1, EndpointClaimSeedV1)):
                _require_refs(issues, {seed.chain_use_id}, use_ids, path + ("chain_use_id",))
                _require_refs(issues, {seed.source_vertex_id}, _values(snapshot.source_vertices, "vertex_id"), path + ("source_vertex_id",))
                use = use_by_id.get(seed.chain_use_id)
                if use is not None and (
                    use.owner_patch_id != seed.owner_patch_id
                    or use.patch_domain_id != seed.patch_domain_id
                ):
                    _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, path, "terminal seed owner patch/domain differs from ChainUse")
                if seed.terminal_relation_id is None:
                    if not fixture_payload:
                        _issue(issues, ValidationCode.TERMINAL_RELATION, path + ("terminal_relation_id",), "production terminal seed requires TerminalRelation")
                else:
                    _require_refs(issues, {seed.terminal_relation_id}, set(terminal_by_id), path + ("terminal_relation_id",))
                    relation = terminal_by_id.get(seed.terminal_relation_id)
                    if relation is not None and (
                        relation.source_vertex_id != seed.source_vertex_id
                        or relation.chain_use_id != seed.chain_use_id
                    ):
                        _issue(issues, ValidationCode.TERMINAL_RELATION, path, "terminal seed differs from TerminalRelation")

        for component in plan.front_components:
            path = ("plans", str(plan.evaluation_plan_id), "front_components", str(component.front_component_id))
            seed = front_seed_by_id.get(component.front_seed_id)
            if seed is not None and (
                component.chain_use_id != seed.chain_use_id
                or component.owner_patch_id != seed.owner_patch_id
                or component.patch_domain_id != seed.patch_domain_id
                or component.sector_id not in seed.sector_ids
            ):
                _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, path, "FrontComponent differs from its FrontSeed")

        for certificate in plan.angular_profile_selection_certificates:
            path = ("plans", str(plan.evaluation_plan_id), "angular_profile_selection_certificates", str(certificate.certificate_id))
            relation = corner_by_id.get(certificate.corner_relation_id)
            sector = sector_by_id.get(certificate.owner_sector_id)
            angle = angle_by_id.get(certificate.reflex_angle_certificate_id)
            _require_refs(issues, {certificate.corner_relation_id}, set(corner_by_id), path + ("corner_relation_id",))
            _require_refs(issues, {certificate.owner_sector_id}, set(sector_by_id), path + ("owner_sector_id",))
            _require_refs(issues, {certificate.reflex_angle_certificate_id}, set(angle_by_id), path + ("reflex_angle_certificate_id",))
            policy_matches = (
                certificate.profile_family_id is request.angular_profile_family_id
                and certificate.selection_policy_id is request.angular_profile_selection_policy_id
                and certificate.max_subturn_value_id is request.max_subturn_value_id
            )
            if not policy_matches:
                _issue(issues, ValidationCode.POLICY_MISMATCH, path, "selection certificate differs from DecalRequest angular policy")
            if relation is not None and (
                relation.owner_sector_id != certificate.owner_sector_id
                or relation.reflex_angle_certificate_id != certificate.reflex_angle_certificate_id
            ):
                _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, path, "selection certificate differs from CornerRelation")
            if sector is not None and sector.patch_domain_id != certificate.patch_domain_id:
                _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, path, "selection certificate domain differs from owner sector")
            if angle is not None and angle.owner_sector_id != certificate.owner_sector_id:
                _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, path, "selection certificate angle differs from owner sector")
            if angle is not None and isinstance(angle.measure_payload, CertifiedReflexAngleMeasureV1):
                delta = angle.measure_payload.reflex_excess_over_pi
                ratio_lower = delta.lower * Decimal(3)
                ratio_upper = delta.upper * Decimal(3)
                k = Decimal(certificate.resolved_hidden_edge_count)
                lower_proven = ratio_lower > k or (
                    ratio_lower == k and delta.lower_kind is IntervalEndpointKind.OPEN
                )
                upper_proven = ratio_upper <= k + Decimal(1)
                if not lower_proven or not upper_proven:
                    _issue(issues, ValidationCode.ANGULAR_SELECTION_UNCERTAIN, path, NamedOutcome.ANGULAR_PROFILE_SELECTION_UNCERTAIN.value)

        for spec in plan.envelope_specs:
            path = ("plans", str(plan.evaluation_plan_id), "envelope_specs", str(spec.envelope_spec_id))
            if isinstance(spec, AngularEnvelopeSpec):
                seed = corner_seed_by_id.get(spec.source_seed_id)
                certificate = selection_by_id.get(spec.selection_certificate_id)
                if seed is not None and (
                    seed.source_relation_id != spec.source_relation_id
                    or seed.owner_sector_id != spec.owner_sector_id
                    or seed.profile_selection_certificate_id != spec.selection_certificate_id
                ):
                    _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, path, "AngularEnvelopeSpec differs from CornerSeed")
                if certificate is not None and (
                    certificate.corner_relation_id != spec.source_relation_id
                    or certificate.owner_sector_id != spec.owner_sector_id
                    or certificate.reflex_angle_certificate_id != spec.angle_certificate_id
                ):
                    _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, path, "AngularEnvelopeSpec differs from selection certificate")
            elif isinstance(spec, JunctionEnvelopeSpec):
                seed = junction_seed_by_id.get(spec.source_seed_id)
                relation = junction_by_id.get(spec.source_relation_id)
                if seed is not None and seed.source_relation_id != spec.source_relation_id:
                    _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, path, "JunctionEnvelopeSpec differs from JunctionSeed")
                if relation is not None:
                    expected_pairing_ids = {pair.route_pairing_id for pair in _junction_route_pairs(relation.route_topology)}
                    if set(spec.route_pairing_ids) != expected_pairing_ids:
                        _issue(issues, ValidationCode.ROUTE_TOPOLOGY, path + ("route_pairing_ids",), "JunctionEnvelopeSpec route references differ from JunctionRelation")
            elif isinstance(spec, CapEnvelopeSpec):
                seed = cap_seed_by_id.get(spec.source_seed_id)
                if seed is not None and (
                    seed.source_vertex_id != spec.physical_terminal_source_vertex_id
                    or seed.chain_use_id != spec.incident_chain_use_id
                    or seed.terminal_relation_id != spec.terminal_relation_id
                ):
                    _issue(issues, ValidationCode.TERMINAL_RELATION, path, "CapEnvelopeSpec differs from CapSeed")
                if spec.terminal_relation_id is None and not fixture_payload:
                    _issue(issues, ValidationCode.TERMINAL_RELATION, path + ("terminal_relation_id",), "production CapEnvelopeSpec requires TerminalRelation")

        if domain is not None and plan.initial_front_spec.fixed_constraint_ids != domain.boundary_constraint_ids:
            _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("plans", str(plan.evaluation_plan_id), "initial_front_spec", "fixed_constraint_ids"), "InitialFrontSpec constraints must equal Snapshot PatchDomain constraints")
        predicate_targets = {
            EventParticipantKind.FRONT_COMPONENT: set(component_by_id),
            EventParticipantKind.ENVELOPE_SPEC: set(spec_by_id),
            EventParticipantKind.ENVELOPE_INSTANCE: instance_ids,
            EventParticipantKind.HIDDEN_SUPPORT: hidden_ids,
            EventParticipantKind.BOUNDARY_CONSTRAINT: _values(snapshot.boundary_constraints, "boundary_constraint_id"),
        }
        for predicate in plan.event_predicates:
            for participant in predicate.participants:
                _require_refs(
                    issues,
                    {participant.participant_id},
                    predicate_targets[participant.kind],
                    ("plans", str(plan.evaluation_plan_id), "event_predicates", str(predicate.predicate_id), "participants"),
                )
    if plan_keys != expected_plan_keys:
        _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("plans",), "plans must be exactly one per selected request/PatchDomain")
    for use_id, count in front_seed_counts.items():
        if count != 1:
            _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("request", "selected_chain_use_ids", str(use_id)), "selected ChainUse must compile to exactly one FrontSeed in its domain plan")
    plan_by_key = {(plan.plan_key.decal_request_id, plan.plan_key.patch_domain_id): plan for plan in plans}
    for batch in geometry_batches:
        issues.extend(validate_geometry_batch(batch))
        key = (batch.decal_request_id, batch.patch_domain_id)
        if key not in plan_by_key:
            _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("geometry_batches",), "batch has no compiled plan")
        if batch.source_revision != snapshot.source_revision:
            _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("geometry_batches", batch.patch_domain_id.value, "source_revision"), "batch and snapshot revisions differ")
        plan = plan_by_key.get(key)
        if plan is not None:
            ownership_claim_ids = _values(plan.ownership.claims, "ownership_claim_id")
            _require_refs(issues, {region.ownership_claim_id for region in batch.semantic_regions}, ownership_claim_ids, ("geometry_batches", str(batch.patch_domain_id), "semantic_regions", "ownership_claim_id"))
        provenance_records = (
            tuple(("vertices", str(item.vert_key), item.provenance) for item in batch.vertices)
            + tuple(("faces", str(item.face_id), item.provenance) for item in batch.faces)
            + tuple(
                ("semantic_regions", str(item.semantic_region_id), item.provenance)
                for item in batch.semantic_regions
            )
        )
        for collection, identity, provenance in provenance_records:
            path = (
                "geometry_batches",
                str(batch.patch_domain_id),
                collection,
                identity,
            )
            _require_refs(issues, set(provenance.source_face_ids), face_ids, path + ("source_face_ids",))
            _require_refs(issues, set(provenance.physical_edge_ids), edge_ids, path + ("physical_edge_ids",))
            _require_refs(issues, set(provenance.chain_use_ids), use_ids, path + ("chain_use_ids",))
    return tuple(issues)
