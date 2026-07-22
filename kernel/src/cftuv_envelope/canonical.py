"""Canonical digests separated by semantic boundary."""

from __future__ import annotations

from dataclasses import dataclass
from hashlib import sha256

from .codec import canonical_json_bytes
from .contracts.analysis import AnalysisSnapshotV1
from .contracts.geometry_batch import GeometryBatchV1
from .contracts.plan import CompiledPatchEvaluationPlanV1


@dataclass(frozen=True, slots=True)
class SnapshotDigest:
    sha256_hex: str


@dataclass(frozen=True, slots=True)
class SemanticPlanDigest:
    sha256_hex: str


@dataclass(frozen=True, slots=True)
class GeometryBatchSemanticDigest:
    sha256_hex: str


def _digest(payload: bytes) -> str:
    return sha256(payload).hexdigest()


def snapshot_digest(snapshot: AnalysisSnapshotV1) -> SnapshotDigest:
    return SnapshotDigest(_digest(canonical_json_bytes(snapshot)))


def semantic_plan_digest(plan: CompiledPatchEvaluationPlanV1) -> SemanticPlanDigest:
    return SemanticPlanDigest(_digest(canonical_json_bytes(plan)))


def geometry_batch_semantic_digest(batch: GeometryBatchV1) -> GeometryBatchSemanticDigest:
    boundary_vertex_keys = {
        key
        for chain in batch.boundary_chains
        for key in chain.ordered_vert_keys
    }
    interface_vertex_keys = {
        key
        for chain in batch.interface_chains
        for key in chain.ordered_vert_keys
    }
    semantic_vertex_keys = boundary_vertex_keys | interface_vertex_keys
    semantic_vertices = frozenset(
        vertex for vertex in batch.vertices if vertex.vert_key in semantic_vertex_keys
    )
    projection = {
        "schema_version": batch.schema_version,
        "source_revision": batch.source_revision,
        "decal_request_id": batch.decal_request_id,
        "patch_domain_id": batch.patch_domain_id,
        "semantic_regions": batch.semantic_regions,
        "boundary_chains": batch.boundary_chains,
        "interface_chains": batch.interface_chains,
        "semantic_vertices": semantic_vertices,
        "contract_versions": batch.contract_versions,
        "diagnostics": batch.diagnostics,
    }
    return GeometryBatchSemanticDigest(_digest(canonical_json_bytes(projection)))
