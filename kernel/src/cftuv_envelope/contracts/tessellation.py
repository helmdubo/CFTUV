"""TessellationPlanV1 — authority boundary, а не triangulator."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from ..ids import (
    SemanticArrangementId,
    SemanticBoundaryId,
    SemanticInterfaceId,
    SemanticRegionId,
)


class TessellationContractId(str, Enum):
    DOWNSTREAM_TESSELLATION_AFTER_SEMANTIC_COALESCING_V1 = (
        "DOWNSTREAM_TESSELLATION_AFTER_SEMANTIC_COALESCING_V1"
    )


class TessellationStartStage(str, Enum):
    SEMANTIC_COALESCING = "SEMANTIC_COALESCING"


class SharedVertexKeyPolicy(str, Enum):
    SEMANTIC_IDENTITY_NOT_COORDINATE = "SEMANTIC_IDENTITY_NOT_COORDINATE"


class InternalTessellationVariation(str, Enum):
    INTERNAL_TRIANGULATION = "INTERNAL_TRIANGULATION"
    INTERNAL_POLYGONIZATION = "INTERNAL_POLYGONIZATION"
    FACE_ORDER = "FACE_ORDER"
    SEMANTICALLY_EQUIVALENT_INTERNAL_DIAGONALS = (
        "SEMANTICALLY_EQUIVALENT_INTERNAL_DIAGONALS"
    )


class ForbiddenTessellationOperation(str, Enum):
    PROFILE_SELECTION = "PROFILE_SELECTION"
    HIDDEN_EDGE_COUNT_SELECTION = "HIDDEN_EDGE_COUNT_SELECTION"
    SILHOUETTE_GENERATION_OR_REPAIR = "SILHOUETTE_GENERATION_OR_REPAIR"
    OWNER_OR_PROVENANCE_INFERENCE = "OWNER_OR_PROVENANCE_INFERENCE"
    PROFILE_BOUNDARY_DISSOLVE = "PROFILE_BOUNDARY_DISSOLVE"
    FACE_COUNT_AS_SEMANTIC_AUTHORITY = "FACE_COUNT_AS_SEMANTIC_AUTHORITY"


class TessellationDigestEquivalence(str, Enum):
    ALL_VALID_TESSELLATIONS_SHARE_ONE_SEMANTIC_DIGEST = (
        "ALL_VALID_TESSELLATIONS_SHARE_ONE_SEMANTIC_DIGEST"
    )


@dataclass(frozen=True, slots=True)
class TessellationPlanV1:
    contract_id: TessellationContractId
    starts_after: TessellationStartStage
    input_arrangement_id: SemanticArrangementId
    semantic_region_ids: frozenset[SemanticRegionId]
    preserved_boundary_ids: frozenset[SemanticBoundaryId]
    preserved_interface_ids: frozenset[SemanticInterfaceId]
    shared_vertex_key_policy: SharedVertexKeyPolicy
    allowed_internal_variation: frozenset[InternalTessellationVariation]
    forbidden_operations: frozenset[ForbiddenTessellationOperation]
    semantic_digest_equivalence: TessellationDigestEquivalence


TessellationPlan = TessellationPlanV1
