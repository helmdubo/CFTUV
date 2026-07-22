from __future__ import annotations

from dataclasses import replace
from decimal import Decimal

from cftuv_envelope import *


def full_host_snapshot() -> AnalysisSnapshotV1:
    revision = SourceRevision("full-host-revision")
    patch_id = PatchId("patch")
    domain_id = PatchDomainId("domain")
    loop_id = BoundaryLoopId("outer-loop")
    chain_id = PhysicalChainId("chain")
    use_id = ChainUseId("chain-use")
    constraint_id = BoundaryConstraintId("constraint")
    sector_id = OwnerSectorId("owner-sector")
    vertex_ids = tuple(SourceVertexId(f"v{index}") for index in range(3))
    edge_ids = tuple(PhysicalEdgeId(f"e{index}") for index in range(3))
    face_id = SourceFaceId("face")
    triangle_id = SurfaceTriangleId("triangle")
    normal = LocalVector3V1(0.0, 0.0, 1.0)
    points = (
        LocalPoint3V1(0.0, 0.0, 0.0),
        LocalPoint3V1(1.0, 0.0, 0.0),
        LocalPoint3V1(0.0, 1.0, 0.0),
    )
    source_vertices = frozenset(
        SourceVertexV1(vertex_id, point)
        for vertex_id, point in zip(vertex_ids, points, strict=True)
    )
    source_edges = frozenset(
        {
            SourceEdgeV1(edge_ids[0], vertex_ids[0], vertex_ids[1]),
            SourceEdgeV1(edge_ids[1], vertex_ids[1], vertex_ids[2]),
            SourceEdgeV1(edge_ids[2], vertex_ids[2], vertex_ids[0]),
        }
    )
    surface_ir = PatchSurfaceIRV1(
        schema_version="cftuv.envelope.patch_surface_ir.v1",
        source_revision=revision,
        payload_mode=SurfacePayloadMode.FULL_HOST_SURFACE,
        source_vertex_ids=frozenset(vertex_ids),
        source_edges=source_edges,
        source_faces=frozenset(
            {
                SourceFaceV1(
                    face_id=face_id,
                    patch_id=patch_id,
                    vertex_cycle=vertex_ids,
                    edge_cycle=edge_ids,
                    polygon_normal=normal,
                    triangle_ids=(triangle_id,),
                )
            }
        ),
        surface_triangles=frozenset(
            {
                SurfaceTriangleV1(
                    triangle_id=triangle_id,
                    source_face_id=face_id,
                    vertex_ids=vertex_ids,
                    physical_edge_ids=edge_ids,
                    triangle_normal=normal,
                )
            }
        ),
    )
    constraint = BoundaryConstraintV1(
        boundary_constraint_id=constraint_id,
        patch_domain_id=domain_id,
        constraint_kind=BoundaryConstraintKind.SOURCE_LAUNCH_BOUNDARY,
        target=ChainUseConstraintTargetV1(use_id),
        originating_chain_use_id=use_id,
        blocks_originating_seed=False,
        blocks_other_fronts=True,
    )
    chain_use = ChainUseV1(
        chain_use_id=use_id,
        physical_chain_id=chain_id,
        owner_patch_id=patch_id,
        patch_domain_id=domain_id,
        boundary_loop_id=loop_id,
        orientation=ChainUseOrientation.SEMANTIC_START_TO_END,
        roles=frozenset({ChainUseRole.DECAL_SOURCE}),
        launch_locus=LaunchLocusV1(
            boundary_constraint_id=constraint_id,
            direction=OwnerInteriorDirection.OWNER_INTERIOR,
            source_support_non_blocking=True,
        ),
    )
    frame = PlanarPatchFrameV1(
        patch_domain_id=domain_id,
        origin=points[0],
        axis_u=LocalVector3V1(1.0, 0.0, 0.0),
        axis_v=LocalVector3V1(0.0, 1.0, 0.0),
        normal=normal,
        handedness=PatchFrameHandedness.RIGHT_HANDED_U_V_NORMAL,
        planarity_certificate=PlanarityCertificateV1(
            certificate_id=PlanarityCertificateId("planarity"),
            patch_domain_id=domain_id,
            law=PlanarityLaw.ALL_DOMAIN_SOURCE_VERTICES_ON_CERTIFIED_PLANE_V1,
            exact=True,
            max_absolute_plane_distance=LocalLengthV1(Decimal(0)),
            authority=SurfaceMetricAuthority.HOST_ANALYSIS_AUTHORITATIVE_V1,
        ),
        projection_law=PlanarProjectionLaw.DOT_WITH_AUTHORITATIVE_U_V_BASIS_V1,
        source_vertex_coordinates=frozenset(
            {
                PlanarSourceVertexCoordinateV1(vertex_ids[0], LocalPoint2V1(0.0, 0.0)),
                PlanarSourceVertexCoordinateV1(vertex_ids[1], LocalPoint2V1(1.0, 0.0)),
                PlanarSourceVertexCoordinateV1(vertex_ids[2], LocalPoint2V1(0.0, 1.0)),
            }
        ),
    )
    return AnalysisSnapshotV1(
        schema_version=ANALYSIS_SNAPSHOT_SCHEMA_V1,
        source_revision=revision,
        analysis_capabilities=frozenset(
            {
                AnalysisCapability.PATCH_DOMAIN_V1,
                AnalysisCapability.PHYSICAL_CHAIN_DIRECTED_USES_V1,
                AnalysisCapability.PATCH_SURFACE_IR_V1,
                AnalysisCapability.ORIENTED_OWNER_SECTOR_V1,
                AnalysisCapability.REFLEX_ANGLE_CERTIFICATE_V1,
                AnalysisCapability.RELATION_LINEAGE_V1,
                AnalysisCapability.AUTHORITATIVE_SURFACE_METRIC_V1,
                AnalysisCapability.PHYSICAL_EDGE_TABLE_V1,
                AnalysisCapability.BOUNDARY_CONSTRAINT_TARGET_V1,
                AnalysisCapability.TYPED_JUNCTION_ROUTE_TOPOLOGY_V1,
                AnalysisCapability.TERMINAL_RELATION_V1,
            }
        ),
        patches=frozenset(
            {
                PatchDescriptorV1(
                    patch_id=patch_id,
                    surface_regime=SurfaceRegime.PLANAR,
                    shape_class=PatchShapeClass.MIX,
                    context_tags=frozenset({PatchContextTag.WALL}),
                )
            }
        ),
        patch_domains=frozenset(
            {
                PatchDomainV1(
                    patch_domain_id=domain_id,
                    owner_patch_id=patch_id,
                    surface_regime=SurfaceRegime.PLANAR,
                    boundary_constraint_ids=frozenset({constraint_id}),
                    sectors=frozenset(
                        {
                            PatchSectorV1(
                                owner_sector_id=sector_id,
                                chain_use_id=use_id,
                                analysis_proven=True,
                                multiplicity_reason=LawId("single-owner-interior-front"),
                            )
                        }
                    ),
                )
            }
        ),
        surface_metric_descriptors=frozenset({frame}),
        surface_ir=surface_ir,
        source_vertices=source_vertices,
        physical_chains=frozenset(
            {
                PhysicalChainV1(
                    physical_chain_id=chain_id,
                    kind=PhysicalChainKind.PHYSICAL_DECAL_SOURCE,
                    is_closed=False,
                    ordered_source_vertex_ids=vertex_ids[:2],
                    ordered_physical_edge_ids=edge_ids[:1],
                    source_lineage=frozenset({LineageId("source-chain")}),
                    data_record_lineage=frozenset(),
                )
            }
        ),
        chain_uses=frozenset({chain_use}),
        boundary_loops=frozenset(
            {
                BoundaryLoopV1(
                    boundary_loop_id=loop_id,
                    patch_domain_id=domain_id,
                    kind=BoundaryLoopKind.OUTER,
                    ordered_chain_use_ids=(use_id,),
                )
            }
        ),
        boundary_constraints=frozenset({constraint}),
        angular_owner_sectors=frozenset(),
        reflex_angle_certificates=frozenset(),
        corner_relations=frozenset(),
        junction_relations=frozenset(),
        terminal_relations=frozenset(
            {
                TerminalRelationV1(
                    terminal_relation_id=TerminalRelationId("terminal-start"),
                    source_vertex_id=vertex_ids[0],
                    chain_use_id=use_id,
                    owner_patch_id=patch_id,
                    patch_domain_id=domain_id,
                    relation_kind=TerminalRelationKind.PHYSICAL_CHAIN_ENDPOINT,
                    endpoint_role=TerminalEndpointRole.START,
                    owner_sector_id=sector_id,
                    source_lineage=frozenset({LineageId("source-chain")}),
                )
            }
        ),
    )


def geometry_batch(*, alternate_diagonal: bool = False) -> GeometryBatchV1:
    provenance = GeometryProvenanceV1(
        source_face_ids=frozenset({SourceFaceId("source-face")}),
        physical_edge_ids=frozenset({PhysicalEdgeId("source-edge")}),
        chain_use_ids=frozenset({ChainUseId("chain-use")}),
        lineage_ids=frozenset({LineageId("semantic-lineage")}),
    )
    points = (
        ("v0", 0.0, 0.0),
        ("v1", 1.0, 0.0),
        ("v2", 1.0, 1.0),
        ("v3", 0.0, 1.0),
    )
    vertices = frozenset(
        GeometryVertexV1(
            vert_key=VertexKey(name),
            position=LocalPoint3V1(x, y, 0.0),
            semantic_location_ref=SemanticLocationId(f"location:{name}"),
            provenance=provenance,
        )
        for name, x, y in points
    )
    region_id = SemanticRegionId("region")
    claim_id = OwnershipClaimId("claim")
    material_id = MaterialId("material")

    def face(identity: str, cycle: tuple[str, str, str]) -> GeometryFaceV1:
        keys = tuple(VertexKey(value) for value in cycle)
        uv_by_key = {
            "v0": UvPoint2V1(0.0, 0.0),
            "v1": UvPoint2V1(1.0, 0.0),
            "v2": UvPoint2V1(1.0, 1.0),
            "v3": UvPoint2V1(0.0, 1.0),
        }
        return GeometryFaceV1(
            face_id=GeometryFaceId(identity),
            ordered_vert_keys=keys,
            uv_facts=tuple(GeometryUvFactV1(key, uv_by_key[key.value]) for key in keys),
            semantic_region_id=region_id,
            ownership_claim_id=claim_id,
            provenance=provenance,
            material_id=material_id,
        )

    faces = (
        (face("face-a", ("v0", "v1", "v2")), face("face-b", ("v0", "v2", "v3")))
        if not alternate_diagonal
        else (face("face-c", ("v0", "v1", "v3")), face("face-d", ("v1", "v2", "v3")))
    )
    batch = GeometryBatchV1(
        schema_version=GEOMETRY_BATCH_SCHEMA_V1,
        source_revision=SourceRevision("source-revision"),
        decal_request_id=DecalRequestId("request"),
        patch_domain_id=PatchDomainId("domain"),
        vertices=vertices,
        faces=faces,
        station_facts=frozenset(
            GeometryStationFactV1(
                station_fact_id=GeometryStationFactId(f"station:{name}"),
                vert_key=VertexKey(name),
                semantic_region_id=region_id,
                ownership_claim_id=claim_id,
                source_s=LocalCoordinateV1(Decimal(str(x))),
                source_r=LocalCoordinateV1(Decimal(str(y))),
                station_model_id=StationModelId.SEMANTIC_CHAIN_USE_S,
                lineage_ids=frozenset({LineageId("semantic-lineage")}),
            )
            for name, x, y in points
        ),
        semantic_regions=frozenset(
            {
                GeometrySemanticRegionV1(
                    semantic_region_id=region_id,
                    ownership_claim_id=claim_id,
                    material_id=material_id,
                    provenance=provenance,
                )
            }
        ),
        boundary_chains=frozenset(
            {
                GeometryBoundaryChainV1(
                    semantic_boundary_id=SemanticBoundaryId("outer-boundary"),
                    ordered_vert_keys=tuple(VertexKey(value) for value in ("v0", "v1", "v2", "v3", "v0")),
                )
            }
        ),
        interface_chains=frozenset(),
        diagnostics=frozenset(),
        contract_versions=frozenset(
            {
                ContractVersionId("cftuv.envelope.geometry_batch.v1"),
                ContractVersionId("cftuv.envelope.compiled_patch_evaluation_plan.v1"),
            }
        ),
        semantic_digest=SemanticDigestValue("pending"),
    )
    digest = geometry_batch_semantic_digest(batch).sha256_hex
    return replace(batch, semantic_digest=SemanticDigestValue(digest))
