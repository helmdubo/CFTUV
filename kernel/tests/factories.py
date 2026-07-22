from __future__ import annotations

from dataclasses import replace

from cftuv_envelope import *


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

