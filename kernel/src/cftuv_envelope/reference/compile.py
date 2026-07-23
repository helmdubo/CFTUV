"""Request/domain compilation for the permanent EC2 reference path."""

from __future__ import annotations

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
    EffectiveAlphaBindingKind,
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
    FrontLifecycle,
    FrontTerminalOutcome,
    PlanKeyV1,
)
from ..contracts.request import DecalRequestV1
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
    FrontSeedId,
    HiddenSupportId,
    JunctionSeedId,
    PatchDomainId,
    PerPatchProjectionId,
    SelectionCertificateId,
)
from ..numeric import ExactRatioV1, IntervalEndpointKind, LocalLengthV1
from ..validation import validate_decal_request
from .contracts import (
    EnvelopeSourceProvenanceV1,
    REFERENCE_COMPILATION_SCHEMA_V1,
    ReferenceCompileResultV1,
    ReferenceDiagnosticSeverity,
    ReferenceEnvelopeCompilationV1,
    ReferenceEvaluationDiagnosticV1,
    ReferenceOutcome,
)
from .provenance import ReferenceProvenanceV1


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


def compile_reference_envelopes(
    snapshot: AnalysisSnapshotV1,
    request: DecalRequestV1,
    patch_domain_id: PatchDomainId | None = None,
) -> ReferenceCompileResultV1:
    """Compile immutable seeds/specs for exactly one request/domain plan."""

    input_issues = validate_decal_request(request)
    if input_issues:
        return _failure(
            ReferenceOutcome.REFERENCE_INPUT_CONTRACT_INVALID,
            "; ".join(f"{item.path}: {item.message}" for item in input_issues),
        )

    uses_by_id = {item.chain_use_id: item for item in snapshot.chain_uses}
    selected_uses = [
        uses_by_id[item]
        for item in request.selected_chain_use_ids
        if item in uses_by_id
    ]
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
            components.add(
                FrontComponentV1(
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
            )
        strip_id = EnvelopeSpecId(
            _stable_value("strip-spec", request.decal_request_id, patch_domain_id, chain_use.chain_use_id)
        )
        strip_ids_by_use[chain_use.chain_use_id] = strip_id
        chain = chains_by_id[chain_use.physical_chain_id]
        provenance_by_spec_id[strip_id] = ReferenceProvenanceV1(
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
            next(
                component.front_component_id
                for component in components
                if component.chain_use_id == chain_use_id
            )
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
        provenance_by_spec_id[spec_id] = ReferenceProvenanceV1(
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
        provenance_by_spec_id[spec_id] = ReferenceProvenanceV1(
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
            provenance_by_spec_id[spec_id] = ReferenceProvenanceV1(
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
    return ReferenceCompileResultV1(ReferenceOutcome.EXACT, compilation, ())
