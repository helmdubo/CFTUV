"""Test-only projection of the accepted EC0 v5 oracle into production contracts."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from cftuv_envelope import *  # noqa: F403


@dataclass(frozen=True, slots=True)
class Ec0ProjectionV5:
    case_id: str
    snapshot: AnalysisSnapshotV1  # noqa: F405
    request: DecalRequestV1  # noqa: F405
    plans: tuple[CompiledPatchEvaluationPlanV1, ...]  # noqa: F405


def _lineage(value: Any) -> frozenset[LineageId]:  # noqa: F405
    if value is None:
        return frozenset()
    if isinstance(value, str):
        return frozenset({LineageId(value)})  # noqa: F405
    return frozenset(LineageId(str(item)) for item in value)  # noqa: F405


def _symbolic(value: str) -> SymbolicLocalLengthV1:  # noqa: F405
    return SymbolicLocalLengthV1(SymbolicScalarId(value))  # noqa: F405


def _endpoint_role(value: str) -> EndpointRole:  # noqa: F405
    return EndpointRole.START if "start" in value.lower() else EndpointRole.END  # noqa: F405


def _snapshot(case: dict[str, Any]) -> AnalysisSnapshotV1:  # noqa: F405
    raw = case["analysis_snapshot"]
    revision = SourceRevision(f"ec0-v5:{case['case_id']}")  # noqa: F405
    domains_by_boundary: dict[str, str] = {}
    regimes_by_patch: dict[str, str] = {}
    for domain in raw["patch_domains"]:
        regimes_by_patch[domain["owner_patch_id"]] = domain["surface_regime"]
        for boundary_id in domain["boundary_constraint_ids"]:
            if boundary_id in domains_by_boundary:
                raise AssertionError(f"boundary {boundary_id} belongs to multiple domains")
            domains_by_boundary[boundary_id] = domain["id"]

    patches = frozenset(
        PatchDescriptorV1(  # noqa: F405
            patch_id=PatchId(item["id"]),  # noqa: F405
            surface_regime=SurfaceRegime(regimes_by_patch[item["id"]]),  # noqa: F405
            shape_class=PatchShapeClass.NOT_RECORDED_IN_EC0_V5,  # noqa: F405
            context_tags=frozenset(),
        )
        for item in raw["patches"]
    )
    patch_domains = frozenset(
        PatchDomainV1(  # noqa: F405
            patch_domain_id=PatchDomainId(item["id"]),  # noqa: F405
            owner_patch_id=PatchId(item["owner_patch_id"]),  # noqa: F405
            surface_regime=SurfaceRegime(item["surface_regime"]),  # noqa: F405
            boundary_constraint_ids=frozenset(
                BoundaryConstraintId(value) for value in item["boundary_constraint_ids"]  # noqa: F405
            ),
            sectors=frozenset(
                PatchSectorV1(  # noqa: F405
                    owner_sector_id=OwnerSectorId(sector["id"]),  # noqa: F405
                    chain_use_id=ChainUseId(sector["chain_use_id"]),  # noqa: F405
                    analysis_proven=sector["analysis_proven"],
                    multiplicity_reason=LawId(sector["multiplicity_reason"]),  # noqa: F405
                )
                for sector in item.get("sectors", [])
            ),
        )
        for item in raw["patch_domains"]
    )
    source_vertices = frozenset(
        SourceVertexV1(  # noqa: F405
            SourceVertexId(item["id"]),  # noqa: F405
            UnavailableSourcePositionV1(  # noqa: F405
                SurfaceCoordinateUnavailableReason.EC0_COORDINATE_FREE_FIXTURE_V5  # noqa: F405
            ),
        )
        for item in raw["source_vertices"]
    )
    physical_chains = frozenset(
        PhysicalChainV1(  # noqa: F405
            physical_chain_id=PhysicalChainId(item["id"]),  # noqa: F405
            kind=PhysicalChainKind(item["kind"]),  # noqa: F405
            ordered_source_vertex_ids=(),
            source_lineage=_lineage(item.get("source_lineage")),
            data_record_lineage=_lineage(item.get("data_record_lineage")),
        )
        for item in raw["physical_chains"]
    )
    chain_uses = frozenset(
        ChainUseV1(  # noqa: F405
            chain_use_id=ChainUseId(item["id"]),  # noqa: F405
            physical_chain_id=PhysicalChainId(item["physical_chain_id"]),  # noqa: F405
            owner_patch_id=PatchId(item["owner_patch_id"]),  # noqa: F405
            patch_domain_id=PatchDomainId(item["patch_domain_id"]),  # noqa: F405
            boundary_loop_id=None,
            orientation=ChainUseOrientation(item["orientation"]),  # noqa: F405
            roles=frozenset(ChainUseRole(role) for role in item["roles"]),  # noqa: F405
            launch_locus=LaunchLocusV1(  # noqa: F405
                boundary_constraint_id=BoundaryConstraintId(item["launch_locus"]["boundary_id"]),  # noqa: F405
                direction=OwnerInteriorDirection(item["launch_locus"]["direction"]),  # noqa: F405
                source_support_non_blocking=item["launch_locus"]["source_support_non_blocking"],
            ),
        )
        for item in raw["chain_uses"]
    )
    boundary_constraints = frozenset(
        BoundaryConstraintV1(  # noqa: F405
            boundary_constraint_id=BoundaryConstraintId(item["id"]),  # noqa: F405
            patch_domain_id=PatchDomainId(domains_by_boundary[item["id"]]),  # noqa: F405
            constraint_kind=BoundaryConstraintKind(item["constraint_kind"]),  # noqa: F405
            originating_chain_use_id=(
                ChainUseId(item["originating_chain_use_id"])  # noqa: F405
                if item.get("originating_chain_use_id")
                else None
            ),
            blocks_originating_seed=item.get("blocks_originating_seed", False),
            blocks_other_fronts=item.get(
                "blocks_other_fronts",
                item["constraint_kind"] == "PHYSICAL_DOMAIN_BARRIER",
            ),
        )
        for item in raw["boundary_constraints"]
    )
    angular_owner_sectors = frozenset(
        OrientedOwnerSectorV1(  # noqa: F405
            owner_sector_id=OwnerSectorId(item["id"]),  # noqa: F405
            owner_patch_id=PatchId(item["owner_patch_id"]),  # noqa: F405
            patch_domain_id=PatchDomainId(item["patch_domain_id"]),  # noqa: F405
            analysis_proven=item["analysis_proven"],
            ordered_incident_chain_use_ids=tuple(
                ChainUseId(value) for value in item["ordered_incident_chain_use_ids"]  # noqa: F405
            ),
            incoming_support_ref=SourceSupportRefV1(  # noqa: F405
                ChainUseId(item["incoming_support_ref"]["chain_use_id"]),  # noqa: F405
                BoundaryConstraintId(item["incoming_support_ref"]["source_launch_boundary_id"]),  # noqa: F405
            ),
            outgoing_support_ref=SourceSupportRefV1(  # noqa: F405
                ChainUseId(item["outgoing_support_ref"]["chain_use_id"]),  # noqa: F405
                BoundaryConstraintId(item["outgoing_support_ref"]["source_launch_boundary_id"]),  # noqa: F405
            ),
            turn_orientation=TurnOrientation(item["turn_orientation"]),  # noqa: F405
            interior_selection_law=InteriorSelectionLaw(item["interior_selection_law"]),  # noqa: F405
        )
        for item in raw["angular_owner_sectors"]
    )
    reflex_angle_certificates = frozenset(
        ReflexAngleCertificateV1(  # noqa: F405
            certificate_id=AngleCertificateId(item["id"]),  # noqa: F405
            owner_sector_id=OwnerSectorId(item["owner_sector_id"]),  # noqa: F405
            measure_law=AngleMeasureLaw(item["measure_law"]),  # noqa: F405
            measure_source=AngleMeasureSource(item["measure_source"]),  # noqa: F405
            strict_range_certificate=StrictAngleRangeCertificate(item["strict_range_certificate"]),  # noqa: F405
            reflex_excess_law=ReflexExcessLaw(item["reflex_excess_law"]),  # noqa: F405
            exact_two_pi=item["exact_two_pi"],
        )
        for item in raw["reflex_angle_certificates"]
    )
    corner_relations = frozenset(
        CornerRelationV1(  # noqa: F405
            corner_relation_id=CornerRelationId(item["id"]),  # noqa: F405
            source_vertex_id=SourceVertexId(item["source_vertex_id"]),  # noqa: F405
            owner_sector_id=OwnerSectorId(item["owner_sector_id"]),  # noqa: F405
            reflex_angle_certificate_id=AngleCertificateId(item["reflex_angle_measure_ref"]),  # noqa: F405
            exact_two_pi=item["exact_two_pi"],
        )
        for item in raw["corner_relations"]
    )
    junction_relations = frozenset(
        JunctionRelationV1(  # noqa: F405
            junction_relation_id=JunctionRelationId(item["id"]),  # noqa: F405
            source_vertex_id=SourceVertexId(item["source_vertex_id"]),  # noqa: F405
            incident_chain_use_ids=tuple(
                ChainUseId(value) for value in item["incident_chain_use_ids"]  # noqa: F405
            ),
            relation_kind=JunctionRelationKind(item["relation_kind"]),  # noqa: F405
            route_pairing=tuple(ChainUseId(value) for value in item.get("route_pairing", [])),  # noqa: F405
            shared_semantic_anchor_id=SharedSemanticAnchorId(item["shared_semantic_anchor_id"]),  # noqa: F405
        )
        for item in raw["junction_relations"]
    )
    capabilities = frozenset(
        {
            AnalysisCapability.PATCH_DOMAIN_V1,  # noqa: F405
            AnalysisCapability.PHYSICAL_CHAIN_DIRECTED_USES_V1,  # noqa: F405
            AnalysisCapability.PATCH_SURFACE_IR_V1,  # noqa: F405
            AnalysisCapability.ORIENTED_OWNER_SECTOR_V1,  # noqa: F405
            AnalysisCapability.REFLEX_ANGLE_CERTIFICATE_V1,  # noqa: F405
            AnalysisCapability.RELATION_LINEAGE_V1,  # noqa: F405
            AnalysisCapability.EC0_COORDINATE_FREE_FIXTURE_V5,  # noqa: F405
        }
    )
    return AnalysisSnapshotV1(  # noqa: F405
        schema_version=ANALYSIS_SNAPSHOT_SCHEMA_V1,  # noqa: F405
        source_revision=revision,
        analysis_capabilities=capabilities,
        patches=patches,
        patch_domains=patch_domains,
        surface_ir=PatchSurfaceIRV1(  # noqa: F405
            schema_version="cftuv.envelope.patch_surface_ir.v1",
            source_revision=revision,
            payload_mode=SurfacePayloadMode.EC0_COORDINATE_FREE_FIXTURE_V5,  # noqa: F405
            source_vertex_ids=frozenset(vertex.vertex_id for vertex in source_vertices),
            source_faces=frozenset(),
            surface_triangles=frozenset(),
        ),
        source_vertices=source_vertices,
        physical_chains=physical_chains,
        chain_uses=chain_uses,
        boundary_loops=frozenset(),
        boundary_constraints=boundary_constraints,
        angular_owner_sectors=angular_owner_sectors,
        reflex_angle_certificates=reflex_angle_certificates,
        corner_relations=corner_relations,
        junction_relations=junction_relations,
    )


def _request(case: dict[str, Any]) -> DecalRequestV1:  # noqa: F405
    raw = case["decal_request"]
    for binding in raw["angular_profile_selection_bindings"]:
        assert binding["angular_profile_id"] == "LINEAR_REFLEX_EQUAL_V1"
        assert binding["selection_policy_id"] == "MIN_K_FOR_MAX_SUBTURN_V1"
        assert binding["max_subturn_value_id"] == "LINEAR_REFLEX_MAX_SUBTURN_60_DEGREES_V1"
    return DecalRequestV1(  # noqa: F405
        schema_version=DECAL_REQUEST_SCHEMA_V1,  # noqa: F405
        decal_request_id=DecalRequestId(raw["decal_request_id"]),  # noqa: F405
        selected_chain_use_ids=frozenset(ChainUseId(value) for value in raw["selected_chain_use_ids"]),  # noqa: F405
        requested_alpha=_symbolic(raw["requested_alpha"]),
        metric_space=MetricSpace.SOURCE_LOCAL_INTRINSIC,  # noqa: F405
        angular_profile_family_id=AngularProfileFamilyId.LINEAR_REFLEX_EQUAL_V1,  # noqa: F405
        angular_profile_selection_policy_id=AngularProfileSelectionPolicyId.MIN_K_FOR_MAX_SUBTURN_V1,  # noqa: F405
        max_subturn_parameter_id=MaxSubturnParameterId.LINEAR_REFLEX_MAX_SUBTURN_V1,  # noqa: F405
        max_subturn_value_id=MaxSubturnValueId.LINEAR_REFLEX_MAX_SUBTURN_60_DEGREES_V1,  # noqa: F405
        max_subturn_exact_value=ExactAngleV1(ExactAngleSymbol.PI_OVER_3),  # noqa: F405
        cap_policy_id=CapPolicyId.PHYSICAL_TERMINAL_LINEAR_CLOSURE_V1,  # noqa: F405
        boundary_policy_id=BoundaryPolicyId.BOUNDARY_LIMITED_PROPAGATION,  # noqa: F405
        interaction_policy_id=InteractionPolicyId.INTRAPATCH_POLICY_B_V1,  # noqa: F405
        ownership_policy_id=OwnershipPolicyId.TOTAL_DISJOINT_RESOLVED_COVERAGE_V1,  # noqa: F405
        material_policy_id=PolicyId(raw["material_policy"]),  # noqa: F405
        uv_policy_id=PolicyId(raw["uv_policy"]),  # noqa: F405
    )


def _selection_certificates(
    raw_eval: dict[str, Any], request_id: DecalRequestId, domain_id: PatchDomainId  # noqa: F405
) -> frozenset[AngularProfileSelectionCertificateV1]:  # noqa: F405
    result = []
    for item in raw_eval["angular_profile_selection_certificates"]:
        k = item["resolved_hidden_edge_count"]
        interval = item["certified_ratio_interval"]
        result.append(
            AngularProfileSelectionCertificateV1(  # noqa: F405
                certificate_id=SelectionCertificateId(item["id"]),  # noqa: F405
                decal_request_id=request_id,
                patch_domain_id=domain_id,
                corner_relation_id=CornerRelationId(item["source_relation_id"]),  # noqa: F405
                owner_sector_id=OwnerSectorId(item["owner_sector_id"]),  # noqa: F405
                reflex_angle_certificate_id=AngleCertificateId(item["reflex_angle_measure_ref"]),  # noqa: F405
                profile_family_id=AngularProfileFamilyId(item["angular_profile_id"]),  # noqa: F405
                selection_policy_id=AngularProfileSelectionPolicyId(item["selection_policy_id"]),  # noqa: F405
                max_subturn_value_id=MaxSubturnValueId(item["max_subturn_value_id"]),  # noqa: F405
                resolved_hidden_edge_count=k,
                resolved_subturn_count=k + 1,
                local_profile_support_count=k + 2,
                local_profile_segment_count=k + 2,
                selection_law=SelectionLaw(item["selection_law"]),  # noqa: F405
                minimality_lower_bound=MinimalityLowerBound(item["lower_bound_certificate"]),  # noqa: F405
                admissibility_upper_bound=AdmissibilityUpperBound(item["upper_bound_certificate"]),  # noqa: F405
                selection_interval_certificate=SelectionIntervalCertificateV1(  # noqa: F405
                    lower_bound_kind=IntervalBoundKind(interval["lower_bound_kind"]),  # noqa: F405
                    lower_bound_integer=interval["lower_bound_integer"],
                    upper_bound_kind=IntervalBoundKind(interval["upper_bound_kind"]),  # noqa: F405
                    upper_bound_integer=interval["upper_bound_integer"],
                ),
                certificate_authority=SelectionCertificateAuthority(item["certificate_authority"]),  # noqa: F405
                regression_fixture_id=(
                    AngularRegressionFixtureId(item["regression_fixture_id"])  # noqa: F405
                    if item.get("regression_fixture_id")
                    else None
                ),
            )
        )
    return frozenset(result)


def _seeds(
    raw_eval: dict[str, Any], request_id: DecalRequestId, domain_id: PatchDomainId  # noqa: F405
) -> frozenset[SeedV1]:  # noqa: F405
    result: list[SeedV1] = []  # noqa: F405
    for item in raw_eval["seeds"]:
        common = {"decal_request_id": request_id, "patch_domain_id": domain_id}
        if item["kind"] == "FrontSeed":
            result.append(
                FrontSeedV1(  # noqa: F405
                    seed_id=FrontSeedId(item["id"]),  # noqa: F405
                    chain_use_id=ChainUseId(item["chain_use_id"]),  # noqa: F405
                    owner_patch_id=PatchId(item["owner_patch_id"]),  # noqa: F405
                    sector_ids=frozenset(OwnerSectorId(value) for value in item["sector_ids"]),  # noqa: F405
                    **common,
                )
            )
        elif item["kind"] == "CornerSeed":
            result.append(
                CornerSeedV1(  # noqa: F405
                    seed_id=CornerSeedId(item["id"]),  # noqa: F405
                    source_relation_id=CornerRelationId(item["source_relation_id"]),  # noqa: F405
                    owner_patch_id=PatchId(item["owner_patch_id"]),  # noqa: F405
                    owner_sector_id=OwnerSectorId(item["owner_sector_id"]),  # noqa: F405
                    ordered_incident_chain_use_ids=tuple(
                        ChainUseId(value) for value in item["ordered_incident_chain_use_ids"]  # noqa: F405
                    ),
                    profile_selection_certificate_id=SelectionCertificateId(item["profile_selection_certificate_id"]),  # noqa: F405
                    **common,
                )
            )
        elif item["kind"] == "JunctionSeed":
            result.append(
                JunctionSeedV1(  # noqa: F405
                    seed_id=JunctionSeedId(item["id"]),  # noqa: F405
                    source_relation_id=JunctionRelationId(item["source_relation_id"]),  # noqa: F405
                    owner_patch_id=PatchId(item["owner_patch_id"]),  # noqa: F405
                    incident_chain_use_ids=frozenset(
                        ChainUseId(value) for value in item["incident_chain_use_ids"]  # noqa: F405
                    ),
                    projection_role=JunctionProjectionRole(
                        item.get("projection_role", "GLOBAL_RELATION")
                    ),
                    shared_semantic_anchor_id=(
                        SharedSemanticAnchorId(item["shared_semantic_anchor_id"])  # noqa: F405
                        if item.get("shared_semantic_anchor_id")
                        else None
                    ),
                    **common,
                )
            )
        elif item["kind"] == "CapSeed":
            result.append(
                CapSeedV1(  # noqa: F405
                    seed_id=CapSeedId(item["id"]),  # noqa: F405
                    source_vertex_id=SourceVertexId(item["source_vertex_id"]),  # noqa: F405
                    chain_use_id=ChainUseId(item["chain_use_id"]),  # noqa: F405
                    owner_patch_id=PatchId(item["owner_patch_id"]),  # noqa: F405
                    endpoint_role=_endpoint_role(item["source_vertex_id"]),
                    **common,
                )
            )
        elif item["kind"] == "EndpointClaimSeed":
            result.append(
                EndpointClaimSeedV1(  # noqa: F405
                    seed_id=EndpointClaimSeedId(item["id"]),  # noqa: F405
                    source_vertex_id=SourceVertexId(item["source_vertex_id"]),  # noqa: F405
                    chain_use_id=ChainUseId(item["chain_use_id"]),  # noqa: F405
                    owner_patch_id=PatchId(item["owner_patch_id"]),  # noqa: F405
                    endpoint_role=_endpoint_role(item["source_vertex_id"]),
                    **common,
                )
            )
        else:
            raise AssertionError(f"unknown seed kind: {item['kind']}")
    return frozenset(result)


def _envelope_specs(
    raw_eval: dict[str, Any],
    snapshot: AnalysisSnapshotV1,  # noqa: F405
    request_id: DecalRequestId,  # noqa: F405
    domain_id: PatchDomainId,  # noqa: F405
    certificates: frozenset[AngularProfileSelectionCertificateV1],  # noqa: F405
) -> frozenset[EnvelopeSpec]:  # noqa: F405
    certificates_by_id = {item.certificate_id: item for item in certificates}
    relations = {
        item.junction_relation_id: item for item in snapshot.junction_relations
    }
    raw_seeds = {item["id"]: item for item in raw_eval["seeds"]}
    result: list[EnvelopeSpec] = []  # noqa: F405
    for item in raw_eval["envelope_specs"]:
        common = {
            "envelope_spec_id": EnvelopeSpecId(item["id"]),  # noqa: F405
            "decal_request_id": request_id,
            "patch_domain_id": domain_id,
            "source_lineage_ids": frozenset(LineageId(value) for value in item["source_lineage_ids"]),  # noqa: F405
        }
        if item["variant"] == "StripEnvelopeSpec":
            result.append(
                StripEnvelopeSpec(  # noqa: F405
                    source_seed_id=FrontSeedId(item["source_seed_id"]),  # noqa: F405
                    front_component_ids=frozenset(
                        FrontComponentId(value) for value in item["front_component_ids"]  # noqa: F405
                    ),
                    support_law_id=StripSupportLawId(item["support_law_id"]),  # noqa: F405
                    station_model_id=StationModelId(item["station_model_id"]),  # noqa: F405
                    transverse_model_id=TransverseModelId(item["transverse_model_id"]),  # noqa: F405
                    terminal_interface_spec_ids=frozenset(
                        EnvelopeSpecId(value) for value in item["terminal_interface_spec_ids"]  # noqa: F405
                    ),
                    terminal_interface_policy=TerminalInterfacePolicy(item["terminal_interface_policy"]),  # noqa: F405
                    **common,
                )
            )
        elif item["variant"] == "AngularEnvelopeSpec":
            certificate_id = SelectionCertificateId(item["profile_selection_certificate_id"])  # noqa: F405
            certificate = certificates_by_id[certificate_id]
            hidden_supports = frozenset(
                HiddenSupportSpecV1(  # noqa: F405
                    hidden_support_id=HiddenSupportId(support["id"]),  # noqa: F405
                    ordinal=support["ordinal"],
                    turn_fraction=ExactRatioV1(  # noqa: F405
                        *map(int, support["turn_fraction"].split("/"))
                    ),
                    direction_law=HiddenSupportDirectionLaw(support["direction_law"]),  # noqa: F405
                    zero_length_at_alpha_zero=support["zero_length_at_alpha_zero"],
                    scope=HiddenSupportScope(support["scope"]),  # noqa: F405
                    source_relation_id=CornerRelationId(support["lineage"]["source_relation_id"]),  # noqa: F405
                    owner_sector_id=OwnerSectorId(support["lineage"]["owner_sector_id"]),  # noqa: F405
                    selection_certificate_id=SelectionCertificateId(support["lineage"]["selection_certificate_id"]),  # noqa: F405
                )
                for support in item["hidden_supports"]
            )
            result.append(
                AngularEnvelopeSpec(  # noqa: F405
                    source_seed_id=CornerSeedId(item["source_seed_id"]),  # noqa: F405
                    source_relation_id=CornerRelationId(item["source_relation_id"]),  # noqa: F405
                    owner_sector_id=OwnerSectorId(item["owner_sector_id"]),  # noqa: F405
                    angle_certificate_id=certificate.reflex_angle_certificate_id,
                    selection_certificate_id=certificate_id,
                    profile_family_id=AngularProfileFamilyId(item["angular_profile_id"]),  # noqa: F405
                    resolved_hidden_edge_count=item["hidden_edge_count"],
                    subdivision_policy=AngularSubdivisionPolicy(item["subdivision_policy"]),  # noqa: F405
                    hidden_supports=hidden_supports,
                    incident_front_component_ids=tuple(
                        FrontComponentId(value)
                        for value in item["ordered_incident_front_component_ids"]
                    ),
                    all_support_normal_speed=1,
                    exposure_policy=AngularExposurePolicy(item["exposure_policy"]),  # noqa: F405
                    mixed_alpha_policy=MixedAlphaPolicy(
                        item.get("incident_effective_alpha_policy", "INCIDENT_EFFECTIVE_ALPHA_VECTOR")
                    ),
                    **common,
                )
            )
        elif item["variant"] == "JunctionEnvelopeSpec":
            relation_id = JunctionRelationId(item["source_relation_id"])  # noqa: F405
            relation = relations[relation_id]
            seed = raw_seeds[item["source_seed_id"]]
            projected = seed.get("projection_role") == "PER_PATCH_PROJECTION"
            result.append(
                JunctionEnvelopeSpec(  # noqa: F405
                    source_seed_id=JunctionSeedId(item["source_seed_id"]),  # noqa: F405
                    source_relation_id=relation_id,
                    incident_front_component_ids=frozenset(
                        FrontComponentId(value) for value in item["incident_front_component_ids"]  # noqa: F405
                    ),
                    support_law_id=JunctionSupportLawId(item["support_law_id"]),  # noqa: F405
                    route_pairing_chain_use_ids=relation.route_pairing,
                    shared_semantic_anchor_id=relation.shared_semantic_anchor_id,
                    per_patch_projection_id=(
                        PerPatchProjectionId(item["source_seed_id"]) if projected else None  # noqa: F405
                    ),
                    mixed_alpha_policy=MixedAlphaPolicy(
                        item.get("incident_effective_alpha_policy", "INCIDENT_EFFECTIVE_ALPHA_VECTOR")
                    ),
                    **common,
                )
            )
        elif item["variant"] == "CapEnvelopeSpec":
            result.append(
                CapEnvelopeSpec(  # noqa: F405
                    source_seed_id=CapSeedId(item["source_seed_id"]),  # noqa: F405
                    physical_terminal_source_vertex_id=SourceVertexId(item["terminal_source_vertex_id"]),  # noqa: F405
                    incident_chain_use_id=ChainUseId(item["incident_chain_use_id"]),  # noqa: F405
                    incident_strip_spec_id=EnvelopeSpecId(item["incident_strip_spec_id"]),  # noqa: F405
                    closure_law_id=CapClosureLawId(item["closure_law_id"]),  # noqa: F405
                    endpoint_role=EndpointRole(item["endpoint_role"]),  # noqa: F405
                    station_law=StationModelId(item["station_law"]),  # noqa: F405
                    effective_alpha_binding=EffectiveAlphaBindingKind.INCIDENT_STRIP_EFFECTIVE_ALPHA,  # noqa: F405
                    exact_two_pi_handling=ExactTwoPiHandling(item["exact_two_pi_handling"]),  # noqa: F405
                    **common,
                )
            )
        else:
            raise AssertionError(f"unknown EnvelopeSpec variant: {item['variant']}")
    return frozenset(result)


def _event_declarations(
    raw_eval: dict[str, Any],
    request_id: DecalRequestId,  # noqa: F405
    domain_id: PatchDomainId,  # noqa: F405
) -> tuple[
    frozenset[EventPredicateDeclarationV1],  # noqa: F405
    frozenset[EventTransitionDeclarationV1],  # noqa: F405
    dict[str, EventPredicateId],  # noqa: F405
]:
    predicates: dict[str, EventPredicateDeclarationV1] = {}  # noqa: F405
    transitions: list[EventTransitionDeclarationV1] = []  # noqa: F405
    interaction_gates: dict[str, EventPredicateId] = {}  # noqa: F405

    def add(
        identity: str,
        kind: EventPredicateKind,  # noqa: F405
        participants: frozenset[EventParticipantRefV1],  # noqa: F405
        batch_key: str,
    ) -> EventPredicateId:  # noqa: F405
        predicate_id = EventPredicateId(identity)  # noqa: F405
        predicates[identity] = EventPredicateDeclarationV1(  # noqa: F405
            predicate_id=predicate_id,
            decal_request_id=request_id,
            patch_domain_id=domain_id,
            kind=kind,
            participants=participants,
            provenance=frozenset({LineageId(identity)}),  # noqa: F405
        )
        transitions.append(
            EventTransitionDeclarationV1(  # noqa: F405
                transition_id=EventTransitionId(f"{identity}:transition"),  # noqa: F405
                decal_request_id=request_id,
                patch_domain_id=domain_id,
                predicate_id=predicate_id,
                input_topology_state_version=TopologyStateVersionId(f"{identity}:before"),  # noqa: F405
                output_topology_state_version=TopologyStateVersionId(f"{identity}:after"),  # noqa: F405
                same_alpha_batch_key=SameAlphaBatchKeyId(batch_key),  # noqa: F405
                provenance=frozenset({LineageId(identity)}),  # noqa: F405
            )
        )
        return predicate_id

    boundary_map = {
        "ENDPOINT_CONTACT": EventPredicateKind.BOUNDARY_CONTACT,  # noqa: F405
        "INTERIOR_CONTACT": EventPredicateKind.BARRIER_SPLIT_REQUIRED,  # noqa: F405
        "INTERVAL_COLLAPSE": EventPredicateKind.FRONT_EXHAUSTED,  # noqa: F405
    }
    for item in raw_eval["boundary_events"]:
        participants = frozenset(
            {
                EventParticipantRefV1(  # noqa: F405
                    EventParticipantKind.FRONT_COMPONENT,  # noqa: F405
                    FrontComponentId(item["front_component_id"]),  # noqa: F405
                ),
                EventParticipantRefV1(  # noqa: F405
                    EventParticipantKind.BOUNDARY_CONSTRAINT,  # noqa: F405
                    BoundaryConstraintId(item["boundary_id"]),  # noqa: F405
                ),
            }
        )
        add(item["id"], boundary_map[item["kind"]], participants, item["exact_height_key"])

    topology_map = {
        "BOUNDARY_CONTACT_AND_COLLAPSE": EventPredicateKind.BOUNDARY_CONTACT,  # noqa: F405
        "COLLAPSE": EventPredicateKind.FRONT_EDGE_COLLAPSE,  # noqa: F405
        "FREEZE": EventPredicateKind.FREEZE,  # noqa: F405
        "MERGE": EventPredicateKind.COVERAGE_COMPONENT_MERGE,  # noqa: F405
        "SATURATION": EventPredicateKind.CAPACITY,  # noqa: F405
    }
    for item in raw_eval["topology_events"]:
        add(
            item["id"],
            topology_map[item["kind"]],
            frozenset(),
            item["exact_height_key"],
        )

    for item in raw_eval["interactions"]:
        identity = f"{item['id']}:gate"
        participants = frozenset(
            EventParticipantRefV1(  # noqa: F405
                EventParticipantKind.ENVELOPE_INSTANCE,  # noqa: F405
                EnvelopeInstanceId(value),  # noqa: F405
            )
            for value in item["participant_envelope_instance_ids"]
        )
        interaction_gates[item["id"]] = add(
            identity,
            EventPredicateKind.MUTUAL_ARRIVAL,  # noqa: F405
            participants,
            item["gate"],
        )
    return frozenset(predicates.values()), frozenset(transitions), interaction_gates


def _plan(
    raw_eval: dict[str, Any], snapshot: AnalysisSnapshotV1, request: DecalRequestV1  # noqa: F405
) -> CompiledPatchEvaluationPlanV1:  # noqa: F405
    request_id = DecalRequestId(raw_eval["plan_key"]["decal_request_id"])  # noqa: F405
    domain_id = PatchDomainId(raw_eval["plan_key"]["patch_domain_id"])  # noqa: F405
    seeds = _seeds(raw_eval, request_id, domain_id)
    capacity_by_component = {
        item["front_component_id"]: item for item in raw_eval["capacity_states"]
    }
    front_components = frozenset(
        FrontComponentV1(  # noqa: F405
            front_component_id=FrontComponentId(item["id"]),  # noqa: F405
            decal_request_id=request_id,
            patch_domain_id=domain_id,
            front_seed_id=FrontSeedId(item["front_seed_id"]),  # noqa: F405
            chain_use_id=ChainUseId(item["chain_use_id"]),  # noqa: F405
            owner_patch_id=PatchId(item["owner_patch_id"]),  # noqa: F405
            sector_id=OwnerSectorId(item["sector_id"]),  # noqa: F405
            active_interval_model=ActiveIntervalModel(item["active_interval_model"]),  # noqa: F405
            initial_branch_count=item["initial_branch_count"],
            branch_count_policy=BranchCountPolicy(item["branch_count_policy"]),  # noqa: F405
            lifecycle=FrontLifecycle(item["lifecycle"]),  # noqa: F405
            front_reading_ids=frozenset(
                FrontReadingId(value) for value in item.get("front_readings", [])  # noqa: F405
            ),
            requested_alpha=_symbolic(
                capacity_by_component[item["id"]]["requested_alpha"]
            ),
            effective_alpha=_symbolic(
                capacity_by_component[item["id"]]["effective_alpha"]
            ),
            capacity_reason=CapacityReason(
                capacity_by_component[item["id"]]["capacity_reason"]
            ),
            terminal_outcome=FrontTerminalOutcome(
                capacity_by_component[item["id"]]["terminal_outcome"]
            ),
        )
        for item in raw_eval["front_components"]
    )
    certificates = _selection_certificates(raw_eval, request_id, domain_id)
    specs = _envelope_specs(raw_eval, snapshot, request_id, domain_id, certificates)
    instances = frozenset(
        EnvelopeInstanceV1(  # noqa: F405
            instance_id=EnvelopeInstanceId(item["id"]),  # noqa: F405
            spec_id=EnvelopeSpecId(item["spec_id"]),  # noqa: F405
            spec_variant=EnvelopeSpecVariant(item["spec_variant"]),  # noqa: F405
            decal_request_id=request_id,
            patch_domain_id=domain_id,
            effective_alpha_binding=EffectiveAlphaBindingV1(  # noqa: F405
                kind=EffectiveAlphaBindingKind(item["effective_alpha_binding"]["kind"]),  # noqa: F405
                front_component_ids=frozenset(
                    FrontComponentId(value)
                    for value in item["effective_alpha_binding"]["front_component_ids"]
                ),
            ),
            boundary_resolution_state=BoundaryResolutionState(item["boundary_resolution"]),  # noqa: F405
        )
        for item in raw_eval["envelope_instances"]
    )
    predicates, transitions, interaction_gates = _event_declarations(
        raw_eval, request_id, domain_id
    )
    interactions = frozenset(
        InteractionDeclarationV1(  # noqa: F405
            interaction_id=InteractionId(item["id"]),  # noqa: F405
            decal_request_id=request_id,
            patch_domain_id=domain_id,
            kind=(
                InteractionKind.ANGULAR_PROFILE_FRONT_CONTACT  # noqa: F405
                if item["kind"] == "ANGULAR_PROFILE_FRONT_CONTACT"
                else InteractionKind.INTRAPATCH_POLICY_B  # noqa: F405
            ),
            participant_envelope_instance_ids=frozenset(
                EnvelopeInstanceId(value) for value in item["participant_envelope_instance_ids"]  # noqa: F405
            ),
            event_gate_id=interaction_gates[item["id"]],
            coverage_effect=(
                CoverageEffect.CLIP_TO_EXPOSED_ANGULAR_PROFILE_BOUNDARY  # noqa: F405
                if item["coverage_effect"] == "CLIP_TO_EXPOSED_ANGULAR_PROFILE_BOUNDARY"
                else CoverageEffect.CLIP_EXISTING_CONTRIBUTIONS_AT_EQUALITY_LOCUS  # noqa: F405
            ),
            creates_new_matter=item.get("creates_new_matter", False),
            provenance=frozenset({LineageId(item["id"])}),  # noqa: F405
        )
        for item in raw_eval["interactions"]
    )
    coverage = raw_eval["patch_coverage"]
    raw_coverage = RawCoverageRef(  # noqa: F405
        coverage_id=CoverageId(coverage["id"]),  # noqa: F405
        decal_request_id=request_id,
        patch_domain_id=domain_id,
        stage=RawCoverageStage(coverage["raw_coverage_stage"]),  # noqa: F405
        envelope_instance_ids=frozenset(
            EnvelopeInstanceId(value) for value in coverage["envelope_instance_ids"]  # noqa: F405
        ),
    )
    resolved_coverage = ResolvedCoverageRef(  # noqa: F405
        coverage_id=CoverageId(coverage["id"]),  # noqa: F405
        raw_coverage_id=CoverageId(coverage["id"]),  # noqa: F405
        decal_request_id=request_id,
        patch_domain_id=domain_id,
        stage=ResolvedCoverageStage.AFTER_APPROVED_INTERACTIONS,  # noqa: F405
        interaction_ids=frozenset(item.interaction_id for item in interactions),
    )
    raw_ownership = raw_eval["ownership"]
    ownership = OwnershipPartitionV1(  # noqa: F405
        coverage_id=CoverageId(raw_ownership["coverage_id"]),  # noqa: F405
        partition_contract_id=OwnershipPartitionContractId(raw_ownership["partition_contract_ref"]),  # noqa: F405
        total=raw_ownership["total"],
        disjoint=raw_ownership["disjoint"],
        silhouette_effect=SilhouetteEffect(raw_ownership["silhouette_effect"]),  # noqa: F405
        equality_boundary_owner=EqualityBoundaryOwner(raw_ownership["tie_boundary_owner"]),  # noqa: F405
        named_unproven_outcome=NamedOutcome(raw_ownership["named_unproven_outcome"]),  # noqa: F405
        claims=frozenset(
            OwnershipClaimV1(  # noqa: F405
                ownership_claim_id=OwnershipClaimId(item["id"]),  # noqa: F405
                coverage_id=CoverageId(item["coverage_id"]),  # noqa: F405
                claimant_kind=OwnershipClaimantKind(item["claimant_kind"]),  # noqa: F405
                claimant_lineage_id=LineageId(item["claimant_id"]),  # noqa: F405
                envelope_spec_ids=frozenset(
                    EnvelopeSpecId(value) for value in item["envelope_spec_ids"]  # noqa: F405
                ),
                envelope_instance_ids=frozenset(
                    EnvelopeInstanceId(value) for value in item["envelope_instance_ids"]  # noqa: F405
                ),
                selection_law_id=LineageId(item["selection_law"]),  # noqa: F405
                station_model_ref=StationModelId(item["station_model_ref"]),  # noqa: F405
            )
            for item in raw_ownership["claims"]
        ),
        equality_boundaries=frozenset(),
        claim_interiors_obligation=ClaimInteriorsObligation(
            raw_ownership["proof_obligations"]["claim_interiors"]
        ),
        claim_closure_obligation=ClaimClosureObligation(
            raw_ownership["proof_obligations"]["claim_closure_union"]
        ),
    )
    support_features = set()
    for spec in specs:
        if isinstance(spec, StripEnvelopeSpec):  # noqa: F405
            support_features.add(
                InitialFrontFeatureRefV1(InitialFrontFeatureKind.STRIP_SUPPORT, spec.envelope_spec_id)  # noqa: F405
            )
        elif isinstance(spec, AngularEnvelopeSpec):  # noqa: F405
            support_features.update(
                InitialFrontFeatureRefV1(  # noqa: F405
                    InitialFrontFeatureKind.ANGULAR_HIDDEN_SUPPORT,  # noqa: F405
                    support.hidden_support_id,
                )
                for support in spec.hidden_supports
            )
        elif isinstance(spec, JunctionEnvelopeSpec):  # noqa: F405
            support_features.add(
                InitialFrontFeatureRefV1(InitialFrontFeatureKind.JUNCTION_SUPPORT, spec.envelope_spec_id)  # noqa: F405
            )
        elif isinstance(spec, CapEnvelopeSpec):  # noqa: F405
            support_features.add(
                InitialFrontFeatureRefV1(InitialFrontFeatureKind.CAP_CLOSURE, spec.envelope_spec_id)  # noqa: F405
            )
    domain = next(item for item in snapshot.patch_domains if item.patch_domain_id == domain_id)
    ledger = raw_eval["event_ledger"]
    tessellation = raw_eval["downstream_tessellation"]
    provenance = raw_eval["geometry_batch_provenance"]
    named_outcomes = {
        NamedOutcome.BARRIER_SPLIT_REQUIRED
        for component in front_components
        if component.capacity_reason is CapacityReason.BARRIER_SPLIT_REQUIRED  # noqa: F405
    }
    for spec_item in raw_eval["envelope_specs"]:
        failure = spec_item.get("mixed_alpha_failure")
        if failure:
            named_outcomes.add(NamedOutcome(failure))  # noqa: F405
    return CompiledPatchEvaluationPlanV1(  # noqa: F405
        schema_version=COMPILED_PLAN_SCHEMA_V1,  # noqa: F405
        evaluation_plan_id=EvaluationPlanId(raw_eval["id"]),  # noqa: F405
        source_revision=snapshot.source_revision,
        plan_key=PlanKeyV1(request_id, domain_id),  # noqa: F405
        owner_patch_id=PatchId(raw_eval["owner_patch_id"]),  # noqa: F405
        seeds=seeds,
        front_components=front_components,
        angular_profile_selection_certificates=certificates,
        envelope_specs=specs,
        envelope_instances=instances,
        initial_front_spec=InitialFrontSpec(  # noqa: F405
            decal_request_id=request_id,
            patch_domain_id=domain_id,
            support_features=frozenset(support_features),
            fixed_constraint_ids=domain.boundary_constraint_ids,
        ),
        event_predicates=predicates,
        event_transitions=transitions,
        event_ledger=EventLedgerContractV1(  # noqa: F405
            contract_id=EventLedgerContractId(ledger["contract_ref"]),  # noqa: F405
            startup_scope=EventLedgerStartupScope(ledger["startup_scope"]),  # noqa: F405
            successor_scheduling=SuccessorSchedulingPolicy(ledger["successor_scheduling"]),  # noqa: F405
            same_alpha_policy=SameAlphaPolicy(ledger["same_alpha_policy"]),  # noqa: F405
            publish_policy=EventPublishPolicy(ledger["publish_policy"]),  # noqa: F405
            pending_outcome=NamedOutcome(ledger["pending_outcome"]),  # noqa: F405
        ),
        raw_coverage=raw_coverage,
        resolved_coverage=resolved_coverage,
        interactions=interactions,
        ownership=ownership,
        station_flow=StationFlowV1(  # noqa: F405
            authority=StationFlowAuthority(raw_eval["station_flow"]["authority"]),  # noqa: F405
            longitudinal=StationAxis(raw_eval["station_flow"]["longitudinal"]),  # noqa: F405
            transverse=StationAxis(raw_eval["station_flow"]["transverse"]),  # noqa: F405
            storage_reversal_effect=StorageReversalEffect(raw_eval["station_flow"]["storage_reversal_effect"]),  # noqa: F405
        ),
        tessellation_plan=TessellationPlanV1(  # noqa: F405
            contract_id=TessellationContractId(tessellation["contract_ref"]),  # noqa: F405
            starts_after=TessellationStartStage(tessellation["starts_after"]),  # noqa: F405
            input_arrangement_id=SemanticArrangementId(f"{raw_eval['id']}:semantic_arrangement"),  # noqa: F405
            semantic_region_ids=frozenset(),
            preserved_boundary_ids=frozenset(),
            preserved_interface_ids=frozenset(),
            shared_vertex_key_policy=SharedVertexKeyPolicy.SEMANTIC_IDENTITY_NOT_COORDINATE,  # noqa: F405
            allowed_internal_variation=frozenset(
                InternalTessellationVariation(value)
                for value in tessellation["allowed_internal_variation"]
            ),
            forbidden_operations=frozenset(
                ForbiddenTessellationOperation(value)
                for value in tessellation["forbidden"]
            ),
            semantic_digest_equivalence=TessellationDigestEquivalence(
                tessellation["semantic_digest_equivalence"]
            ),
        ),
        geometry_batch_provenance=GeometryBatchProvenanceV1(  # noqa: F405
            decal_request_id=request_id,
            patch_domain_id=domain_id,
            resolved_coverage_id=CoverageId(provenance["patch_coverage_id"]),  # noqa: F405
            source_lineage_ids=frozenset({LineageId(provenance["patch_coverage_id"])}),  # noqa: F405
            shared_semantic_anchor_id=(
                SharedSemanticAnchorId(provenance["shared_semantic_anchor_id"])  # noqa: F405
                if provenance.get("shared_semantic_anchor_id")
                else None
            ),
        ),
        named_outcomes=frozenset(named_outcomes),
    )


def project_case_data(case: dict[str, Any]) -> Ec0ProjectionV5:
    if case["schema"] != "cftuv.envelope.ec0.case.v5":
        raise AssertionError(f"unexpected EC0 case schema: {case['schema']}")
    snapshot = _snapshot(case)
    request = _request(case)
    plans = tuple(
        _plan(item, snapshot, request)
        for item in case["expected_compiled_plan"]["patch_evaluations"]
    )
    return Ec0ProjectionV5(case["case_id"], snapshot, request, plans)


def load_projection(path: Path) -> Ec0ProjectionV5:
    return project_case_data(json.loads(path.read_text(encoding="utf-8")))
