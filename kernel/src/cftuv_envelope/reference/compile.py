"""Request/domain compilation for the permanent EC2 reference path."""

from __future__ import annotations

from dataclasses import replace
from decimal import Decimal
from fractions import Fraction
from hashlib import sha256

from ..contracts.analysis import (
    AnalysisSnapshotV1,
    CertifiedReflexAngleMeasureV1,
    DeclaredRoutePairsTopologyV1,
    JunctionRelationKind,
    TJunctionRouteTopologyV1,
    UnavailableJunctionRouteTopologyV1,
    XJunctionRouteTopologyV1,
    YJunctionRouteTopologyV1,
)
from ..contracts.envelopes import (
    AdmissibilityUpperBound,
    AngularEnvelopeSpec,
    AngularExposurePolicy,
    AngularProfileSelectionCertificateV1,
    AngularRegressionFixtureId,
    AngularSubdivisionPolicy,
    CapClosureLawId,
    CapEnvelopeSpec,
    CertifiedBoundHiddenSupportDirectionLawV1,
    CertifiedBoundHiddenSupportSpecV1,
    DirectionBindingReasonV1,
    EffectiveAlphaBindingKind,
    EvaluationGeometryDirectionBindingCertificateV1,
    ExactTwoPiHandling,
    HiddenSupportDirectionLaw,
    HiddenSupportScope,
    HiddenSupportSpecV1,
    IntervalBoundKind,
    JunctionEnvelopeSpec,
    JunctionSupportLawId,
    MinimalityLowerBound,
    MixedAlphaPolicy,
    SelectionCertificateAuthority,
    SelectionIntervalCertificateV1,
    SelectionLaw,
    StationModelId,
    StripEnvelopeSpec,
    StripSupportLawId,
    TerminalInterfacePolicy,
    TransverseModelId,
)
from ..contracts.events import (
    InitialFrontFeatureKind,
    InitialFrontFeatureRefV1,
    InitialFrontSpec,
)
from ..contracts.plan import (
    ActiveIntervalModel,
    BranchCountPolicy,
    CapacityReason,
    FrontComponentV1,
    FrontReadingDeclarationV1,
    FrontLifecycle,
    FrontTerminalOutcome,
    PhysicalSupportIntervalV1,
    PlanKeyV1,
    SelfContactPairDeclarationV1,
)
from ..contracts.request import DecalRequestV1
from ..contracts.surface import SurfacePayloadMode
from ..contracts.seeds import (
    CapSeedV1,
    CornerSeedV1,
    EndpointRole,
    FrontSeedV1,
    JunctionProjectionRole,
    JunctionSeedV1,
)
from ..ids import (
    CapSeedId,
    CornerSeedId,
    EnvelopeSpecId,
    FrontComponentId,
    FrontReadingId,
    FrontSeedId,
    HiddenSupportId,
    JunctionSeedId,
    LawId,
    PatchDomainId,
    PerPatchProjectionId,
    SelectionCertificateId,
    SelfContactPairDeclarationId,
    SourceSupportId,
)
from ..numeric import ExactRatioV1, IntervalEndpointKind, LocalLengthV1
from ..validation import (
    ValidationCode,
    validate_snapshot_request_references,
)
from .contracts import (
    EnvelopeSourceProvenanceV1,
    REFERENCE_COMPILATION_SCHEMA_V1,
    ReferenceCompileResultV1,
    ReferenceDiagnosticSeverity,
    ReferenceEnvelopeCompilationV1,
    ReferenceEvaluationDiagnosticV1,
    ReferenceOutcome,
)
from .direction_binding import (
    DirectionBindingCertificateUnproven,
    _certify_k1_recipe_direction_bindings,
    _verify_k1_recipe_direction_bindings,
    certify_direction_bindings,
    has_rational_support_direction,
    verify_direction_bindings,
)
from .evaluation_geometry import (
    EvaluationGeometryBindingInvalid,
    EvaluationGeometryRefinementBudgetExhausted,
    build_evaluation_geometry_binding,
    verify_evaluation_geometry_binding,
)
from .provenance import make_reference_provenance
from .validation import (
    validate_reference_geometry_certificates,
    validate_reference_geometry_payload,
)


def _stable_value(kind: str, *parts: object) -> str:
    payload = "\x1f".join((kind, *(str(part) for part in parts))).encode("utf-8")
    return f"{kind}:{sha256(payload).hexdigest()[:24]}"


def _failure(outcome: ReferenceOutcome, message: str) -> ReferenceCompileResultV1:
    diagnostic = ReferenceEvaluationDiagnosticV1(
        outcome=outcome,
        severity=ReferenceDiagnosticSeverity.UNSUPPORTED,
        message=message,
    )
    return ReferenceCompileResultV1(outcome, None, (diagnostic,))


def _fraction(value: Decimal) -> Fraction:
    return Fraction(value)


def _strictly_above(interval, threshold: Fraction) -> bool:
    lower = _fraction(interval.lower)
    return lower > threshold or (
        lower == threshold and interval.lower_kind is IntervalEndpointKind.OPEN
    )


def _at_most(interval, threshold: Fraction) -> bool:
    return _fraction(interval.upper) <= threshold


def _resolve_hidden_edge_count(measure: CertifiedReflexAngleMeasureV1) -> int | None:
    interval = measure.reflex_excess_over_pi
    for hidden_count in range(3):
        lower = Fraction(hidden_count, 3)
        upper = Fraction(hidden_count + 1, 3)
        lower_proven = hidden_count == 0 or _strictly_above(interval, lower)
        if lower_proven and _at_most(interval, upper):
            return hidden_count
    return None


def _route_pair_ids(topology) -> frozenset:
    if isinstance(topology, TJunctionRouteTopologyV1):
        return frozenset({topology.through_route_pair.route_pairing_id})
    if isinstance(
        topology,
        (XJunctionRouteTopologyV1, YJunctionRouteTopologyV1, DeclaredRoutePairsTopologyV1),
    ):
        return frozenset(pair.route_pairing_id for pair in topology.route_pairs)
    return frozenset()


def _physical_endpoint_order(chain_use, chain) -> tuple:
    vertices = chain.ordered_source_vertex_ids
    if not vertices:
        return ()
    if chain_use.orientation.value == "B_START_TO_END":
        vertices = tuple(reversed(vertices))
    return vertices[0], vertices[-1]


def _attach_front_reading_declarations(
    compilation: ReferenceEnvelopeCompilationV1,
) -> ReferenceEnvelopeCompilationV1 | ReferenceCompileResultV1:
    """Bind every exact source interval to one immutable reading declaration."""

    from .common import GeometryContext, ReferenceGeometryError

    if (
        compilation.analysis_snapshot.surface_ir.payload_mode
        is not SurfacePayloadMode.FULL_HOST_SURFACE
    ):
        return compilation
    frame, diagnostics = validate_reference_geometry_payload(
        compilation.analysis_snapshot,
        compilation.plan_key.patch_domain_id,
    )
    if frame is None:
        return ReferenceCompileResultV1(
            diagnostics[0].outcome,
            None,
            diagnostics,
        )
    context = GeometryContext.build(compilation, frame)
    declarations = set()
    by_component: dict[FrontComponentId, list[FrontReadingDeclarationV1]] = {}
    try:
        for spec in sorted(
            (
                item
                for item in compilation.envelope_specs
                if isinstance(item, StripEnvelopeSpec)
            ),
            key=lambda item: item.envelope_spec_id.value,
        ):
            for source in context.support_segments_for_use(
                next(
                    item.chain_use_id
                    for item in compilation.seeds
                    if getattr(item, "seed_id", None) == spec.source_seed_id
                ),
                spec.envelope_spec_id.value,
            ):
                component_id = FrontComponentId(source.front_component_id)
                support_id = SourceSupportId(source.support_id)
                reading = FrontReadingDeclarationV1(
                    front_reading_id=FrontReadingId(
                        _stable_value(
                            "front-reading",
                            component_id,
                            support_id,
                        )
                    ),
                    front_component_id=component_id,
                    source_support_id=support_id,
                    physical_interval=PhysicalSupportIntervalV1(
                        physical_edge_id=source.physical_edge_id,
                        start_vertex_id=source.source_vertex_start_id,
                        end_vertex_id=source.source_vertex_end_id,
                    ),
                    chain_use_id=source.chain_use_id,
                    owner_sector_id=next(
                        item.sector_id
                        for item in compilation.front_components
                        if item.front_component_id == component_id
                    ),
                    arrival_law_id=LawId(
                        _stable_value(
                            "strip-arrival-law",
                            spec.envelope_spec_id,
                            support_id,
                        )
                    ),
                )
                declarations.add(reading)
                by_component.setdefault(component_id, []).append(reading)
    except ReferenceGeometryError:
        # Compile Session C остаётся topology-only для геометрии, которую
        # evaluator отклоняет named outcome. Session D не выдумывает reading
        # declarations без exact source supports.
        return compilation

    reading_ids_by_component = {
        component_id: frozenset(
            item.front_reading_id for item in component_declarations
        )
        for component_id, component_declarations in by_component.items()
    }
    return replace(
        compilation,
        front_components=frozenset(
            replace(
                component,
                front_reading_ids=reading_ids_by_component.get(
                    component.front_component_id,
                    frozenset(),
                ),
            )
            for component in compilation.front_components
        ),
        front_reading_declarations=frozenset(declarations),
        self_contact_pair_declarations=frozenset(),
    )


def _attach_direction_bindings(
    compilation: ReferenceEnvelopeCompilationV1,
) -> ReferenceEnvelopeCompilationV1 | ReferenceCompileResultV1:
    """Сертифицировать фактические hidden directions evaluation-геометрии."""

    from .angular import angular_support_data
    from .common import GeometryContext, ReferenceGeometryError

    if (
        compilation.analysis_snapshot.surface_ir.payload_mode
        is not SurfacePayloadMode.FULL_HOST_SURFACE
    ):
        return compilation
    frame, diagnostics = validate_reference_geometry_payload(
        compilation.analysis_snapshot,
        compilation.plan_key.patch_domain_id,
    )
    if frame is None:
        return ReferenceCompileResultV1(
            diagnostics[0].outcome,
            None,
            diagnostics,
        )
    context = GeometryContext.build(
        compilation,
        frame,
        require_certified_bound_supports=False,
    )
    source_context = GeometryContext.build(
        replace(compilation, evaluation_geometry_binding=None),
        frame,
        require_evaluation_binding=False,
    )
    changed_specs = set(compilation.envelope_specs)
    try:
        for spec in sorted(
            (
                item
                for item in compilation.envelope_specs
                if isinstance(item, AngularEnvelopeSpec)
                and item.resolved_hidden_edge_count > 0
            ),
            key=lambda item: item.envelope_spec_id.value,
        ):
            sector = next(
                item
                for item in context.snapshot.angular_owner_sectors
                if item.owner_sector_id == spec.owner_sector_id
            )
            _, _, _, ideal = angular_support_data(context, spec)
            _, _, _, source_ideal = angular_support_data(source_context, spec)
            source_is_rational = tuple(
                has_rational_support_direction(
                    source_context.metric,
                    source_ideal[ordinal],
                )
                for ordinal in range(
                    1,
                    spec.resolved_hidden_edge_count + 1,
                )
            )
            needs_binding = tuple(
                (
                    not source_is_rational[ordinal - 1]
                    or not has_rational_support_direction(
                        context.metric,
                        ideal[ordinal],
                    )
                )
                for ordinal in range(
                    1,
                    spec.resolved_hidden_edge_count + 1,
                )
            )
            if not any(needs_binding):
                continue
            certificates = (
                _certify_k1_recipe_direction_bindings(
                    context.metric,
                    ideal[0],
                    ideal[-1],
                    ideal,
                    sector.turn_orientation,
                )
                if spec.resolved_hidden_edge_count == 1
                else certify_direction_bindings(
                    context.metric,
                    ideal,
                    sector.turn_orientation,
                )
            )
            selected_certificates = tuple(
                (
                    _evaluation_geometry_certificate(
                        certificate,
                        (
                            DirectionBindingReasonV1.SOURCE_DIRECTION_IRRATIONAL
                            if not source_rational
                            else DirectionBindingReasonV1.EVALUATION_GEOMETRY_UNBINDS_SOURCE_RATIONAL
                        ),
                    )
                    if (
                        needed
                        and compilation.evaluation_geometry_binding is not None
                    )
                    else certificate if needed else None
                )
                for needed, source_rational, certificate in zip(
                    needs_binding,
                    source_is_rational,
                    certificates,
                    strict=True,
                )
            )
            if spec.resolved_hidden_edge_count == 1:
                _verify_k1_recipe_direction_bindings(
                    context.metric,
                    ideal[0],
                    ideal[-1],
                    ideal,
                    sector.turn_orientation,
                    selected_certificates,
                )
            else:
                verify_direction_bindings(
                    context.metric,
                    ideal,
                    sector.turn_orientation,
                    selected_certificates,
                )
            supports = frozenset(
                _bound_support(support, selected_certificates[support.ordinal - 1])
                for support in spec.hidden_supports
            )
            changed_specs.remove(spec)
            changed_specs.add(replace(spec, hidden_supports=supports))
    except DirectionBindingCertificateUnproven as exc:
        return _failure(
            ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE,
            f"direction binding certificate is not proven: {exc}",
        )
    except ReferenceGeometryError as exc:
        return _failure(exc.outcome, str(exc))
    return replace(compilation, envelope_specs=frozenset(changed_specs))


def _evaluation_geometry_certificate(certificate, reason):
    return EvaluationGeometryDirectionBindingCertificateV1(
        bound_primitive_integer_vector=(
            certificate.bound_primitive_integer_vector
        ),
        ideal_window_lower_slope_envelope=(
            certificate.ideal_window_lower_slope_envelope
        ),
        ideal_window_upper_slope_envelope=(
            certificate.ideal_window_upper_slope_envelope
        ),
        certified_window_width_lower_bound=(
            certificate.certified_window_width_lower_bound
        ),
        proven_predicates=certificate.proven_predicates,
        binding_reason=reason,
    )


def _bound_support(support, certificate):
    if certificate is None:
        return support
    return CertifiedBoundHiddenSupportSpecV1(
        hidden_support_id=support.hidden_support_id,
        ordinal=support.ordinal,
        turn_fraction=support.turn_fraction,
        direction_law=(
            CertifiedBoundHiddenSupportDirectionLawV1.CERTIFIED_RATIONAL_BINDING_IN_ORDINAL_SUBTURN_V1
        ),
        zero_length_at_alpha_zero=support.zero_length_at_alpha_zero,
        scope=support.scope,
        source_relation_id=support.source_relation_id,
        owner_sector_id=support.owner_sector_id,
        selection_certificate_id=support.selection_certificate_id,
        direction_binding=certificate,
    )


def _finalize_compilation(
    compilation: ReferenceEnvelopeCompilationV1,
) -> ReferenceCompileResultV1:
    compiled_with_geometry = _attach_evaluation_geometry(compilation)
    if isinstance(compiled_with_geometry, ReferenceCompileResultV1):
        return compiled_with_geometry
    compiled_with_bindings = _attach_direction_bindings(compiled_with_geometry)
    if isinstance(compiled_with_bindings, ReferenceCompileResultV1):
        return compiled_with_bindings
    sealed_geometry = _seal_evaluation_geometry_authority(
        compiled_with_bindings
    )
    if isinstance(sealed_geometry, ReferenceCompileResultV1):
        return sealed_geometry
    compiled_with_readings = _attach_front_reading_declarations(
        sealed_geometry
    )
    if isinstance(compiled_with_readings, ReferenceCompileResultV1):
        return compiled_with_readings
    return ReferenceCompileResultV1(
        ReferenceOutcome.EXACT,
        compiled_with_readings,
        (),
    )


def _attach_evaluation_geometry(
    compilation: ReferenceEnvelopeCompilationV1,
) -> ReferenceEnvelopeCompilationV1 | ReferenceCompileResultV1:
    """Построить общую geometry binding ровно один раз, до обоих consumers."""

    if (
        compilation.analysis_snapshot.surface_ir.payload_mode
        is not SurfacePayloadMode.FULL_HOST_SURFACE
    ):
        return compilation
    frame, diagnostics = validate_reference_geometry_payload(
        compilation.analysis_snapshot,
        compilation.plan_key.patch_domain_id,
    )
    if frame is None:
        return ReferenceCompileResultV1(
            diagnostics[0].outcome,
            None,
            diagnostics,
        )
    try:
        binding = build_evaluation_geometry_binding(compilation, frame)
    except EvaluationGeometryRefinementBudgetExhausted as exc:
        return _failure(
            ReferenceOutcome.REFINEMENT_BUDGET_EXHAUSTED,
            str(exc),
        )
    except EvaluationGeometryBindingInvalid as exc:
        return _failure(
            ReferenceOutcome.REFERENCE_EVALUATION_GEOMETRY_BINDING_INVALID,
            str(exc),
        )
    return replace(compilation, evaluation_geometry_binding=binding)


def _seal_evaluation_geometry_authority(
    compilation: ReferenceEnvelopeCompilationV1,
) -> ReferenceEnvelopeCompilationV1 | ReferenceCompileResultV1:
    """Связать evaluation-запись с единственными plan-authority supports."""

    binding = compilation.evaluation_geometry_binding
    if binding is None:
        return compilation
    frame, diagnostics = validate_reference_geometry_payload(
        compilation.analysis_snapshot,
        compilation.plan_key.patch_domain_id,
    )
    if frame is None:
        return ReferenceCompileResultV1(
            diagnostics[0].outcome,
            None,
            diagnostics,
        )
    bound_ids = frozenset(
        support.hidden_support_id
        for spec in compilation.envelope_specs
        if isinstance(spec, AngularEnvelopeSpec)
        for support in spec.hidden_supports
        if type(support) is CertifiedBoundHiddenSupportSpecV1
    )
    sealed = replace(
        compilation,
        evaluation_geometry_binding=replace(
            binding,
            bound_hidden_support_ids=bound_ids,
        ),
    )
    try:
        verify_evaluation_geometry_binding(
            sealed.evaluation_geometry_binding,
            sealed,
            frame,
        )
    except EvaluationGeometryBindingInvalid as exc:
        return _failure(
            ReferenceOutcome.REFERENCE_EVALUATION_GEOMETRY_BINDING_INVALID,
            str(exc),
        )
    return sealed


def declare_reference_self_contacts(
    compilation: ReferenceEnvelopeCompilationV1,
    reading_pairs: tuple[tuple[FrontReadingId, FrontReadingId], ...],
) -> ReferenceCompileResultV1:
    """Attach only explicitly authorized same-lineage reading pairs."""

    readings = {
        item.front_reading_id: item
        for item in compilation.front_reading_declarations
    }
    uses = {
        item.chain_use_id: item
        for item in compilation.analysis_snapshot.chain_uses
    }
    chains = {
        item.physical_chain_id: item
        for item in compilation.analysis_snapshot.physical_chains
    }
    declarations = set()
    canonical_pairs = set()
    for supplied_left, supplied_right in reading_pairs:
        if supplied_left == supplied_right:
            return _failure(
                ReferenceOutcome.REFERENCE_INPUT_CONTRACT_INVALID,
                "SelfContactPairDeclarationV1 requires two distinct readings",
            )
        left = readings.get(supplied_left)
        right = readings.get(supplied_right)
        if left is None or right is None:
            return _failure(
                ReferenceOutcome.REFERENCE_INPUT_CONTRACT_INVALID,
                "SelfContactPairDeclarationV1 references an absent FrontReading",
            )
        pair_key = frozenset({supplied_left, supplied_right})
        if pair_key in canonical_pairs:
            return _failure(
                ReferenceOutcome.REFERENCE_INPUT_CONTRACT_INVALID,
                "duplicate SelfContactPairDeclarationV1 reading pair",
            )
        canonical_pairs.add(pair_key)
        left_lineage = chains[
            uses[left.chain_use_id].physical_chain_id
        ].source_lineage
        right_lineage = chains[
            uses[right.chain_use_id].physical_chain_id
        ].source_lineage
        if not left_lineage or left_lineage != right_lineage:
            return _failure(
                ReferenceOutcome.REFERENCE_INPUT_CONTRACT_INVALID,
                "self-contact readings must carry one identical non-empty "
                "source lineage set",
            )
        ordered = tuple(
            sorted(
                (left, right),
                key=lambda item: item.front_reading_id.value,
            )
        )
        declarations.add(
            SelfContactPairDeclarationV1(
                declaration_id=SelfContactPairDeclarationId(
                    _stable_value(
                        "self-contact-pair",
                        ordered[0].front_component_id,
                        ordered[0].front_reading_id,
                        ordered[1].front_component_id,
                        ordered[1].front_reading_id,
                    )
                ),
                left_front_component_id=ordered[0].front_component_id,
                right_front_component_id=ordered[1].front_component_id,
                left_front_reading_id=ordered[0].front_reading_id,
                right_front_reading_id=ordered[1].front_reading_id,
                source_lineage_ids=left_lineage,
            )
        )
    declared = replace(
        compilation,
        self_contact_pair_declarations=frozenset(declarations),
    )
    return ReferenceCompileResultV1(
        ReferenceOutcome.EXACT,
        declared,
        (),
    )


def compile_reference_envelopes(
    snapshot: AnalysisSnapshotV1,
    request: DecalRequestV1,
    patch_domain_id: PatchDomainId | None = None,
) -> ReferenceCompileResultV1:
    """Compile immutable seeds/specs for exactly one request/domain plan."""

    input_issues = validate_snapshot_request_references(snapshot, request)
    if input_issues:
        if all(item.code is ValidationCode.ROUTE_TOPOLOGY for item in input_issues):
            return _failure(
                ReferenceOutcome.JUNCTION_ROUTE_PAIRING_REQUIRED,
                "; ".join(
                    f"{'.'.join(item.path)}: {item.message}" for item in input_issues
                ),
            )
        return _failure(
            ReferenceOutcome.REFERENCE_INPUT_CONTRACT_INVALID,
            "; ".join(
                f"{'.'.join(item.path)}: {item.message}" for item in input_issues
            ),
        )

    uses_by_id = {item.chain_use_id: item for item in snapshot.chain_uses}
    selected_uses = [uses_by_id[item] for item in request.selected_chain_use_ids]
    if frozenset(item.chain_use_id for item in selected_uses) != (
        request.selected_chain_use_ids
    ):
        return _failure(
            ReferenceOutcome.REFERENCE_INPUT_CONTRACT_INVALID,
            "selected ChainUse resolution must be total and exact",
        )
    domain_ids = {item.patch_domain_id for item in selected_uses}
    if patch_domain_id is None:
        if len(domain_ids) != 1:
            return _failure(
                ReferenceOutcome.REFERENCE_PATCH_DOMAIN_SELECTION_REQUIRED,
                "request spans multiple PatchDomains; select one explicit plan key",
            )
        patch_domain_id = next(iter(domain_ids))
    if patch_domain_id not in domain_ids:
        return _failure(
            ReferenceOutcome.REFERENCE_PATCH_DOMAIN_SELECTION_REQUIRED,
            "selected PatchDomain has no selected ChainUse",
        )

    domain = next(
        (item for item in snapshot.patch_domains if item.patch_domain_id == patch_domain_id),
        None,
    )
    if domain is None:
        return _failure(
            ReferenceOutcome.REFERENCE_PATCH_DOMAIN_SELECTION_REQUIRED,
            "selected PatchDomain is absent from AnalysisSnapshotV1",
        )
    geometry_diagnostics = validate_reference_geometry_certificates(
        snapshot, patch_domain_id
    )
    if geometry_diagnostics:
        return ReferenceCompileResultV1(
            geometry_diagnostics[0].outcome,
            None,
            geometry_diagnostics,
        )
    local_uses = sorted(
        (item for item in selected_uses if item.patch_domain_id == patch_domain_id),
        key=lambda item: item.chain_use_id.value,
    )
    local_use_ids = frozenset(item.chain_use_id for item in local_uses)
    chains_by_id = {item.physical_chain_id: item for item in snapshot.physical_chains}
    sectors_by_use = {}
    for sector in domain.sectors:
        sectors_by_use.setdefault(sector.chain_use_id, []).append(sector)

    seeds = set()
    components = set()
    component_by_use_sector = {}
    sector_id_by_use = {}
    selection_certificates = set()
    specs_by_id = {}
    provenance_by_spec_id = {}
    strip_ids_by_use = {}
    interface_ids_by_use = {item.chain_use_id: set() for item in local_uses}

    for chain_use in local_uses:
        sectors = sorted(
            sectors_by_use.get(chain_use.chain_use_id, ()),
            key=lambda item: item.owner_sector_id.value,
        )
        if not sectors:
            return _failure(
                ReferenceOutcome.PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED,
                f"ChainUse {chain_use.chain_use_id} has no analysis-proven owner sector",
            )
        if len(sectors) != 1:
            return _failure(
                ReferenceOutcome.REFERENCE_MULTISECTOR_CHAIN_USE_UNSUPPORTED,
                "planar reference v1 requires exactly one analysis-proven "
                f"OwnerSector per ChainUse; {chain_use.chain_use_id} has {len(sectors)}",
            )
        sector_id_by_use[chain_use.chain_use_id] = sectors[0].owner_sector_id
        front_seed_id = FrontSeedId(
            _stable_value("front-seed", request.decal_request_id, patch_domain_id, chain_use.chain_use_id)
        )
        seed = FrontSeedV1(
            seed_id=front_seed_id,
            decal_request_id=request.decal_request_id,
            patch_domain_id=patch_domain_id,
            chain_use_id=chain_use.chain_use_id,
            owner_patch_id=domain.owner_patch_id,
            sector_ids=frozenset(item.owner_sector_id for item in sectors),
        )
        seeds.add(seed)
        component_ids = set()
        for sector in sectors:
            component_id = FrontComponentId(
                _stable_value("front-component", front_seed_id, sector.owner_sector_id)
            )
            component_ids.add(component_id)
            component = FrontComponentV1(
                front_component_id=component_id,
                decal_request_id=request.decal_request_id,
                patch_domain_id=patch_domain_id,
                front_seed_id=front_seed_id,
                chain_use_id=chain_use.chain_use_id,
                owner_patch_id=domain.owner_patch_id,
                sector_id=sector.owner_sector_id,
                active_interval_model=ActiveIntervalModel.CONTINUOUS_INTERVAL_SET,
                initial_branch_count=1,
                branch_count_policy=BranchCountPolicy.NON_INCREASING,
                lifecycle=FrontLifecycle.COMPILE_TRACKED,
                front_reading_ids=frozenset(),
                requested_alpha=request.requested_alpha,
                effective_alpha=request.requested_alpha,
                capacity_reason=CapacityReason.NONE,
                terminal_outcome=FrontTerminalOutcome.NONE,
            )
            components.add(component)
            component_by_use_sector[
                (chain_use.chain_use_id, sector.owner_sector_id)
            ] = component
        strip_id = EnvelopeSpecId(
            _stable_value("strip-spec", request.decal_request_id, patch_domain_id, chain_use.chain_use_id)
        )
        strip_ids_by_use[chain_use.chain_use_id] = strip_id
        chain = chains_by_id[chain_use.physical_chain_id]
        provenance_by_spec_id[strip_id] = make_reference_provenance(
            envelope_spec_ids=frozenset({strip_id.value}),
            physical_edge_ids=frozenset(item.value for item in chain.ordered_physical_edge_ids),
            chain_use_ids=frozenset({chain_use.chain_use_id.value}),
            patch_domain_ids=frozenset({patch_domain_id.value}),
            boundary_constraint_ids=frozenset(
                {chain_use.launch_locus.boundary_constraint_id.value}
            ),
            lineage_ids=frozenset(item.value for item in chain.source_lineage),
        )
        specs_by_id[strip_id] = (seed, component_ids, chain)

    angular_specs = []
    owner_sectors = {item.owner_sector_id: item for item in snapshot.angular_owner_sectors}
    angle_certificates = {
        item.certificate_id: item for item in snapshot.reflex_angle_certificates
    }
    for relation in sorted(snapshot.corner_relations, key=lambda item: item.corner_relation_id.value):
        sector = owner_sectors.get(relation.owner_sector_id)
        if sector is None or sector.patch_domain_id != patch_domain_id:
            continue
        if not frozenset(sector.ordered_incident_chain_use_ids).issubset(local_use_ids):
            continue
        angle_certificate = angle_certificates.get(relation.reflex_angle_certificate_id)
        if angle_certificate is None or not isinstance(
            angle_certificate.measure_payload, CertifiedReflexAngleMeasureV1
        ):
            return _failure(
                ReferenceOutcome.ANGULAR_PROFILE_SELECTION_UNCERTAIN,
                f"CornerRelation {relation.corner_relation_id} lacks a certified numeric angle",
            )
        hidden_count = _resolve_hidden_edge_count(angle_certificate.measure_payload)
        if hidden_count is None:
            return _failure(
                ReferenceOutcome.ANGULAR_PROFILE_SELECTION_UNCERTAIN,
                f"CornerRelation {relation.corner_relation_id} does not prove a unique k",
            )
        selection_id = SelectionCertificateId(
            _stable_value("profile-selection", request.decal_request_id, relation.corner_relation_id)
        )
        selection = AngularProfileSelectionCertificateV1(
            certificate_id=selection_id,
            decal_request_id=request.decal_request_id,
            patch_domain_id=patch_domain_id,
            corner_relation_id=relation.corner_relation_id,
            owner_sector_id=relation.owner_sector_id,
            reflex_angle_certificate_id=relation.reflex_angle_certificate_id,
            profile_family_id=request.angular_profile_family_id,
            selection_policy_id=request.angular_profile_selection_policy_id,
            max_subturn_value_id=request.max_subturn_value_id,
            resolved_hidden_edge_count=hidden_count,
            resolved_subturn_count=hidden_count + 1,
            local_profile_support_count=hidden_count + 2,
            local_profile_segment_count=hidden_count + 2,
            selection_law=SelectionLaw.MIN_K_FOR_MAX_SUBTURN,
            minimality_lower_bound=MinimalityLowerBound.K_ZERO_OR_STRICT_LOWER,
            admissibility_upper_bound=AdmissibilityUpperBound.CLOSED_UPPER,
            selection_interval_certificate=SelectionIntervalCertificateV1(
                lower_bound_kind=IntervalBoundKind.OPEN,
                lower_bound_integer=hidden_count,
                upper_bound_kind=IntervalBoundKind.CLOSED,
                upper_bound_integer=hidden_count + 1,
            ),
            certificate_authority=SelectionCertificateAuthority.EXACT_OR_CERTIFIED_ANGLE_COMPARISON,
            regression_fixture_id=(
                AngularRegressionFixtureId.K0
                if hidden_count == 0
                else AngularRegressionFixtureId.K1 if hidden_count == 1 else None
            ),
        )
        selection_certificates.add(selection)
        seed_id = CornerSeedId(
            _stable_value("corner-seed", request.decal_request_id, relation.corner_relation_id)
        )
        seed = CornerSeedV1(
            seed_id=seed_id,
            decal_request_id=request.decal_request_id,
            patch_domain_id=patch_domain_id,
            source_relation_id=relation.corner_relation_id,
            owner_patch_id=domain.owner_patch_id,
            owner_sector_id=relation.owner_sector_id,
            ordered_incident_chain_use_ids=sector.ordered_incident_chain_use_ids,
            profile_selection_certificate_id=selection_id,
        )
        seeds.add(seed)
        hidden_supports = frozenset(
            HiddenSupportSpecV1(
                hidden_support_id=HiddenSupportId(
                    _stable_value("hidden-support", relation.corner_relation_id, ordinal)
                ),
                ordinal=ordinal,
                turn_fraction=ExactRatioV1(ordinal, hidden_count + 1),
                direction_law=HiddenSupportDirectionLaw.ORIENTED_OWNER_SECTOR_ORDINAL_SUBTURN,
                zero_length_at_alpha_zero=True,
                scope=HiddenSupportScope.ANGULAR_ENVELOPE_SPEC_LOCAL,
                source_relation_id=relation.corner_relation_id,
                owner_sector_id=relation.owner_sector_id,
                selection_certificate_id=selection_id,
            )
            for ordinal in range(1, hidden_count + 1)
        )
        spec_id = EnvelopeSpecId(
            _stable_value("angular-spec", request.decal_request_id, relation.corner_relation_id)
        )
        incident_components = tuple(
            component_by_use_sector[
                (
                    chain_use_id,
                    sector_id_by_use[chain_use_id],
                )
            ].front_component_id
            for chain_use_id in sector.ordered_incident_chain_use_ids
        )
        spec = AngularEnvelopeSpec(
            envelope_spec_id=spec_id,
            source_seed_id=seed_id,
            decal_request_id=request.decal_request_id,
            patch_domain_id=patch_domain_id,
            source_lineage_ids=frozenset(
                item
                for chain_use_id in sector.ordered_incident_chain_use_ids
                for item in chains_by_id[uses_by_id[chain_use_id].physical_chain_id].source_lineage
            ),
            source_relation_id=relation.corner_relation_id,
            owner_sector_id=relation.owner_sector_id,
            angle_certificate_id=relation.reflex_angle_certificate_id,
            selection_certificate_id=selection_id,
            profile_family_id=request.angular_profile_family_id,
            resolved_hidden_edge_count=hidden_count,
            subdivision_policy=AngularSubdivisionPolicy.EQUAL_REFLEX_EXCESS,
            hidden_supports=hidden_supports,
            incident_front_component_ids=incident_components,
            all_support_normal_speed=1,
            exposure_policy=AngularExposurePolicy.MAY_BECOME_INTERNAL_AFTER_EXACT_UNION,
            mixed_alpha_policy=MixedAlphaPolicy.INCIDENT_EFFECTIVE_ALPHA_VECTOR,
        )
        angular_specs.append(spec)
        for chain_use_id in sector.ordered_incident_chain_use_ids:
            interface_ids_by_use[chain_use_id].add(spec_id)
        physical_edges = {
            item.value
            for chain_use_id in sector.ordered_incident_chain_use_ids
            for item in chains_by_id[
                uses_by_id[chain_use_id].physical_chain_id
            ].ordered_physical_edge_ids
        }
        provenance_by_spec_id[spec_id] = make_reference_provenance(
            envelope_spec_ids=frozenset({spec_id.value}),
            support_ids=frozenset(item.hidden_support_id.value for item in hidden_supports),
            physical_edge_ids=frozenset(physical_edges),
            chain_use_ids=frozenset(item.value for item in sector.ordered_incident_chain_use_ids),
            patch_domain_ids=frozenset({patch_domain_id.value}),
            lineage_ids=frozenset(item.value for item in spec.source_lineage_ids),
        )

    junction_specs = []
    junction_vertices = set()
    for relation in sorted(snapshot.junction_relations, key=lambda item: item.junction_relation_id.value):
        local_incident = frozenset(
            item for item in relation.incident_chain_use_ids if item in local_use_ids
        )
        if not local_incident:
            continue
        junction_vertices.add(relation.source_vertex_id)
        if isinstance(relation.route_topology, UnavailableJunctionRouteTopologyV1):
            return _failure(
                ReferenceOutcome.JUNCTION_ROUTE_PAIRING_REQUIRED,
                f"JunctionRelation {relation.junction_relation_id} lacks declared route topology",
            )
        route_pairing_ids = _route_pair_ids(relation.route_topology)
        if not route_pairing_ids:
            return _failure(
                ReferenceOutcome.JUNCTION_ROUTE_PAIRING_REQUIRED,
                f"JunctionRelation {relation.junction_relation_id} has no route pair",
            )
        seed_id = JunctionSeedId(
            _stable_value("junction-seed", request.decal_request_id, patch_domain_id, relation.junction_relation_id)
        )
        seed = JunctionSeedV1(
            seed_id=seed_id,
            decal_request_id=request.decal_request_id,
            patch_domain_id=patch_domain_id,
            source_relation_id=relation.junction_relation_id,
            owner_patch_id=domain.owner_patch_id,
            incident_chain_use_ids=local_incident,
            projection_role=(
                JunctionProjectionRole.PER_PATCH_PROJECTION
                if relation.relation_kind is JunctionRelationKind.CROSS_PATCH
                else JunctionProjectionRole.GLOBAL_RELATION
            ),
            shared_semantic_anchor_id=relation.shared_semantic_anchor_id,
        )
        seeds.add(seed)
        spec_id = EnvelopeSpecId(
            _stable_value("junction-spec", request.decal_request_id, patch_domain_id, relation.junction_relation_id)
        )
        lineage = frozenset(
            item
            for chain_use_id in local_incident
            for item in chains_by_id[uses_by_id[chain_use_id].physical_chain_id].source_lineage
        )
        spec = JunctionEnvelopeSpec(
            envelope_spec_id=spec_id,
            source_seed_id=seed_id,
            decal_request_id=request.decal_request_id,
            patch_domain_id=patch_domain_id,
            source_lineage_ids=lineage,
            source_relation_id=relation.junction_relation_id,
            incident_front_component_ids=frozenset(
                component.front_component_id
                for component in components
                if component.chain_use_id in local_incident
            ),
            support_law_id=JunctionSupportLawId.DECLARED_JUNCTION_RELATION_SUPPORTS_V1,
            route_pairing_ids=route_pairing_ids,
            shared_semantic_anchor_id=relation.shared_semantic_anchor_id,
            per_patch_projection_id=(
                PerPatchProjectionId(
                    _stable_value("junction-projection", relation.junction_relation_id, patch_domain_id)
                )
                if relation.relation_kind is JunctionRelationKind.CROSS_PATCH
                else None
            ),
            mixed_alpha_policy=MixedAlphaPolicy.INCIDENT_EFFECTIVE_ALPHA_VECTOR,
        )
        junction_specs.append(spec)
        for chain_use_id in local_incident:
            interface_ids_by_use[chain_use_id].add(spec_id)
        provenance_by_spec_id[spec_id] = make_reference_provenance(
            envelope_spec_ids=frozenset({spec_id.value}),
            physical_edge_ids=frozenset(
                item.value
                for chain_use_id in local_incident
                for item in chains_by_id[
                    uses_by_id[chain_use_id].physical_chain_id
                ].ordered_physical_edge_ids
            ),
            chain_use_ids=frozenset(item.value for item in local_incident),
            patch_domain_ids=frozenset({patch_domain_id.value}),
            lineage_ids=frozenset(item.value for item in lineage),
        )

    corner_vertices = {item.source_vertex_id for item in snapshot.corner_relations}
    cap_specs = []
    terminal_by_use_vertex = {
        (item.chain_use_id, item.source_vertex_id): item
        for item in snapshot.terminal_relations
        if item.patch_domain_id == patch_domain_id
    }
    for chain_use in local_uses:
        chain = chains_by_id[chain_use.physical_chain_id]
        if chain.is_closed:
            continue
        endpoints = _physical_endpoint_order(chain_use, chain)
        if not endpoints:
            # Accepted EC0 v5 projections deliberately have no coordinates or
            # physical endpoint path.  Compile keeps the Strip law but never
            # invents Cap geometry/identity from missing facts.
            continue
        start_vertex, end_vertex = endpoints
        for endpoint_role, source_vertex_id in (
            (EndpointRole.START, start_vertex),
            (EndpointRole.END, end_vertex),
        ):
            if source_vertex_id in corner_vertices or source_vertex_id in junction_vertices:
                continue
            terminal = terminal_by_use_vertex.get((chain_use.chain_use_id, source_vertex_id))
            seed_id = CapSeedId(
                _stable_value("cap-seed", request.decal_request_id, chain_use.chain_use_id, endpoint_role.value)
            )
            seed = CapSeedV1(
                seed_id=seed_id,
                decal_request_id=request.decal_request_id,
                patch_domain_id=patch_domain_id,
                source_vertex_id=source_vertex_id,
                terminal_relation_id=terminal.terminal_relation_id if terminal else None,
                chain_use_id=chain_use.chain_use_id,
                owner_patch_id=domain.owner_patch_id,
                endpoint_role=endpoint_role,
            )
            seeds.add(seed)
            spec_id = EnvelopeSpecId(
                _stable_value("cap-spec", request.decal_request_id, chain_use.chain_use_id, endpoint_role.value)
            )
            spec = CapEnvelopeSpec(
                envelope_spec_id=spec_id,
                source_seed_id=seed_id,
                decal_request_id=request.decal_request_id,
                patch_domain_id=patch_domain_id,
                source_lineage_ids=chain.source_lineage,
                physical_terminal_source_vertex_id=source_vertex_id,
                terminal_relation_id=terminal.terminal_relation_id if terminal else None,
                incident_chain_use_id=chain_use.chain_use_id,
                incident_strip_spec_id=strip_ids_by_use[chain_use.chain_use_id],
                closure_law_id=CapClosureLawId.PHYSICAL_TERMINAL_LINEAR_CLOSURE_V1,
                endpoint_role=endpoint_role,
                station_law=StationModelId.CONSTANT_PHYSICAL_ENDPOINT_S,
                effective_alpha_binding=EffectiveAlphaBindingKind.INCIDENT_STRIP_EFFECTIVE_ALPHA,
                exact_two_pi_handling=ExactTwoPiHandling.TERMINAL_OR_JUNCTION_NEVER_ANGULAR_PROFILE,
            )
            cap_specs.append(spec)
            interface_ids_by_use[chain_use.chain_use_id].add(spec_id)
            provenance_by_spec_id[spec_id] = make_reference_provenance(
                envelope_spec_ids=frozenset({spec_id.value}),
                physical_edge_ids=frozenset(item.value for item in chain.ordered_physical_edge_ids),
                chain_use_ids=frozenset({chain_use.chain_use_id.value}),
                patch_domain_ids=frozenset({patch_domain_id.value}),
                lineage_ids=frozenset(item.value for item in chain.source_lineage),
            )

    strip_specs = []
    for chain_use in local_uses:
        seed, component_ids, chain = specs_by_id[strip_ids_by_use[chain_use.chain_use_id]]
        strip_specs.append(
            StripEnvelopeSpec(
                envelope_spec_id=strip_ids_by_use[chain_use.chain_use_id],
                source_seed_id=seed.seed_id,
                decal_request_id=request.decal_request_id,
                patch_domain_id=patch_domain_id,
                source_lineage_ids=chain.source_lineage,
                front_component_ids=frozenset(component_ids),
                support_law_id=StripSupportLawId.PLANAR_LINEAR_NORMAL_OFFSET_V1,
                station_model_id=StationModelId.SEMANTIC_CHAIN_USE_S,
                transverse_model_id=TransverseModelId.SIGNED_OWNER_INTERIOR_R,
                terminal_interface_spec_ids=frozenset(
                    interface_ids_by_use[chain_use.chain_use_id]
                ),
                terminal_interface_policy=TerminalInterfacePolicy.REFERENCED_ENVELOPE_SPECS,
            )
        )

    all_specs = frozenset((*strip_specs, *angular_specs, *junction_specs, *cap_specs))
    support_features = set()
    for spec in all_specs:
        if isinstance(spec, StripEnvelopeSpec):
            support_features.add(
                InitialFrontFeatureRefV1(
                    InitialFrontFeatureKind.STRIP_SUPPORT, spec.envelope_spec_id
                )
            )
        elif isinstance(spec, AngularEnvelopeSpec):
            support_features.update(
                InitialFrontFeatureRefV1(
                    InitialFrontFeatureKind.ANGULAR_HIDDEN_SUPPORT,
                    item.hidden_support_id,
                )
                for item in spec.hidden_supports
            )
        elif isinstance(spec, JunctionEnvelopeSpec):
            support_features.add(
                InitialFrontFeatureRefV1(
                    InitialFrontFeatureKind.JUNCTION_SUPPORT, spec.envelope_spec_id
                )
            )
        elif isinstance(spec, CapEnvelopeSpec):
            support_features.add(
                InitialFrontFeatureRefV1(
                    InitialFrontFeatureKind.CAP_CLOSURE, spec.envelope_spec_id
                )
            )

    compilation = ReferenceEnvelopeCompilationV1(
        schema_version=REFERENCE_COMPILATION_SCHEMA_V1,
        source_revision=snapshot.source_revision,
        analysis_snapshot=snapshot,
        decal_request=request,
        plan_key=PlanKeyV1(request.decal_request_id, patch_domain_id),
        owner_patch_id=domain.owner_patch_id,
        seeds=frozenset(seeds),
        front_components=frozenset(components),
        profile_selection_certificates=frozenset(selection_certificates),
        envelope_specs=all_specs,
        initial_front_spec=InitialFrontSpec(
            decal_request_id=request.decal_request_id,
            patch_domain_id=patch_domain_id,
            support_features=frozenset(support_features),
            fixed_constraint_ids=domain.boundary_constraint_ids,
        ),
        source_provenance=frozenset(
            EnvelopeSourceProvenanceV1(spec_id.value, provenance_by_spec_id[spec_id])
            for spec_id in sorted(
                provenance_by_spec_id, key=lambda item: item.value
            )
        ),
    )
    return _finalize_compilation(compilation)
