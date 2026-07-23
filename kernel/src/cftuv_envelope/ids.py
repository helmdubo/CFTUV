"""Типизированные непрозрачные идентификаторы публичного контракта."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class OpaqueId:
    """Базовый value object; разные подклассы намеренно не равны друг другу."""

    value: str

    def __post_init__(self) -> None:
        if not isinstance(self.value, str) or not self.value.strip():
            raise ValueError(f"{type(self).__name__} requires a non-empty string")

    def __str__(self) -> str:
        return self.value


@dataclass(frozen=True, slots=True)
class SourceRevision(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class PatchId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class PatchDomainId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class PhysicalChainId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class ChainUseId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class SourceVertexId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class SourceFaceId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class SurfaceTriangleId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class PhysicalEdgeId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class BoundaryLoopId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class BoundaryConstraintId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class OwnerSectorId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class AngleCertificateId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class PlanarityCertificateId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class CornerRelationId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class JunctionRelationId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class TerminalRelationId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class DecalRequestId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class FrontSeedId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class CornerSeedId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class JunctionSeedId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class CapSeedId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class EndpointClaimSeedId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class FrontComponentId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class FrontReadingId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class SourceSupportId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class SelfContactPairDeclarationId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class EnvelopeSpecId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class EnvelopeInstanceId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class HiddenSupportId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class SelectionCertificateId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class CoverageId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class InteractionId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class InteractionComponentId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class ArrivalModelId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class InteractionCandidateId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class InteractionRelationId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class MutualArrivalCertificateId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class EqualityLocusId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class EqualityLocusSegmentId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class InteractionApplicationId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class ResolvedContributionId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class SameAlphaInteractionBatchId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class ActiveDomainCertificateId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class ExactSegmentId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class ExactRegionId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class ConstructionCertificateId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class OwnershipClaimId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class EqualityBoundaryId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class EventPredicateId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class EventTransitionId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class TopologyStateVersionId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class SameAlphaBatchKeyId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class SemanticRegionId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class SemanticArrangementId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class SemanticBoundaryId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class SemanticInterfaceId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class SharedSemanticAnchorId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class PerPatchProjectionId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class RoutePairingId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class VertexKey(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class GeometryFaceId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class GeometryDiagnosticId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class GeometryStationFactId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class SemanticLocationId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class MaterialId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class LineageId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class PolicyId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class LawId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class ContractVersionId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class SymbolicScalarId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class EvaluationPlanId(OpaqueId):
    pass


@dataclass(frozen=True, slots=True)
class SemanticDigestValue(OpaqueId):
    pass
