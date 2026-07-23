"""Public planar metric contracts for exact reference and filtered runtime."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import gcd, isfinite

from ..ids import (
    ReferenceMetricId,
    LineageId,
    PatchDomainId,
    PlanarityCertificateId,
    RuntimeMetricId,
    SourceRevision,
    SourceVertexId,
)


REFERENCE_PLANAR_METRIC_SCHEMA_V2 = (
    "cftuv.envelope.rational_affine_planar_metric.v2"
)
RUNTIME_PLANAR_METRIC_SCHEMA_V1 = "cftuv.envelope.runtime_planar_metric.v1"


class PlanarityAdmissionLawV1(str, Enum):
    EXACT_SOURCE_PLANE_V1 = "EXACT_SOURCE_PLANE_V1"


class AffineFrameSelectionLawV1(str, Enum):
    CANONICAL_SOURCE_VERTEX_BASIS_V1 = (
        "CANONICAL_SOURCE_VERTEX_BASIS_V1"
    )


class AffineReconstructionLawV1(str, Enum):
    O_PLUS_U_A_PLUS_V_B_V1 = "O_PLUS_U_A_PLUS_V_B_V1"


class AffineChartOrientationV1(str, Enum):
    COORDINATE_CCW_MATCHES_OWNER_PATCH = (
        "COORDINATE_CCW_MATCHES_OWNER_PATCH"
    )
    COORDINATE_CW_MATCHES_OWNER_PATCH = (
        "COORDINATE_CW_MATCHES_OWNER_PATCH"
    )


class RuntimePredicateFilterLawV1(str, Enum):
    BINARY64_OUTWARD_INTERVAL_V1 = "BINARY64_OUTWARD_INTERVAL_V1"


class RuntimePredicateResultV1(str, Enum):
    CERTIFIED_NEGATIVE = "CERTIFIED_NEGATIVE"
    CERTIFIED_ZERO = "CERTIFIED_ZERO"
    CERTIFIED_POSITIVE = "CERTIFIED_POSITIVE"
    EXACT_FALLBACK_REQUIRED = "EXACT_FALLBACK_REQUIRED"


class RuntimeMetricFallbackLawV1(str, Enum):
    AUTHORITATIVE_REFERENCE_METRIC_V2 = (
        "AUTHORITATIVE_REFERENCE_METRIC_V2"
    )


class MetricSemanticIdentityLawV1(str, Enum):
    EXACT_CONSTRUCTION_CERTIFICATES_ONLY = (
        "EXACT_CONSTRUCTION_CERTIFICATES_ONLY"
    )


@dataclass(frozen=True, slots=True)
class ExactRationalV1:
    numerator: int
    denominator: int

    def __post_init__(self) -> None:
        if type(self.numerator) is not int or type(self.denominator) is not int:
            raise TypeError("ExactRationalV1 requires integer components")
        if self.denominator <= 0:
            raise ValueError("ExactRationalV1 denominator must be positive")
        if gcd(abs(self.numerator), self.denominator) != 1:
            raise ValueError("ExactRationalV1 must be reduced")


@dataclass(frozen=True, slots=True)
class ExactPoint2V1:
    x: ExactRationalV1
    y: ExactRationalV1


@dataclass(frozen=True, slots=True)
class ExactVector2V1:
    x: ExactRationalV1
    y: ExactRationalV1


@dataclass(frozen=True, slots=True)
class ExactPoint3V1:
    x: ExactRationalV1
    y: ExactRationalV1
    z: ExactRationalV1


@dataclass(frozen=True, slots=True)
class ExactVector3V1:
    x: ExactRationalV1
    y: ExactRationalV1
    z: ExactRationalV1


@dataclass(frozen=True, slots=True)
class ExactMatrix2V1:
    m00: ExactRationalV1
    m01: ExactRationalV1
    m10: ExactRationalV1
    m11: ExactRationalV1


@dataclass(frozen=True, slots=True)
class ExactSourceVertexCoordinateV2:
    source_vertex_id: SourceVertexId
    domain_coordinate: ExactPoint2V1


@dataclass(frozen=True, slots=True)
class CertifiedAffineSupportDirectionV2:
    direction_in_domain: ExactVector2V1
    reference_metric_id: ReferenceMetricId


@dataclass(frozen=True, slots=True)
class ExactSourcePlaneCertificateV1:
    certificate_id: PlanarityCertificateId
    patch_domain_id: PatchDomainId
    admission_law: PlanarityAdmissionLawV1
    exact: bool
    exact_plane_normal: ExactVector3V1
    source_vertex_ids: frozenset[SourceVertexId]
    reconstruction_law: AffineReconstructionLawV1

    def __post_init__(self) -> None:
        if not self.exact:
            raise ValueError(
                "ExactSourcePlaneCertificateV1 must be exact"
            )


@dataclass(frozen=True, slots=True)
class RationalAffinePlanarMetricV2:
    reference_metric_id: ReferenceMetricId
    patch_domain_id: PatchDomainId
    source_revision: SourceRevision
    exact_origin: ExactPoint3V1
    exact_basis_a: ExactVector3V1
    exact_basis_b: ExactVector3V1
    exact_gram_matrix: ExactMatrix2V1
    exact_inverse_gram_matrix: ExactMatrix2V1
    exact_source_vertex_coordinates: frozenset[
        ExactSourceVertexCoordinateV2
    ]
    chart_orientation: AffineChartOrientationV1
    frame_selection_law: AffineFrameSelectionLawV1
    planarity_certificate: ExactSourcePlaneCertificateV1
    source_lineage: frozenset[LineageId]


@dataclass(frozen=True, slots=True)
class Binary64Point2V1:
    x: float
    y: float

    def __post_init__(self) -> None:
        if not all(isfinite(item) for item in (self.x, self.y)):
            raise ValueError("Binary64Point2V1 requires finite coordinates")


@dataclass(frozen=True, slots=True)
class Binary64Point3V1:
    x: float
    y: float
    z: float

    def __post_init__(self) -> None:
        if not all(isfinite(item) for item in (self.x, self.y, self.z)):
            raise ValueError("Binary64Point3V1 requires finite coordinates")


@dataclass(frozen=True, slots=True)
class Binary64Vector3V1:
    x: float
    y: float
    z: float

    def __post_init__(self) -> None:
        if not all(isfinite(item) for item in (self.x, self.y, self.z)):
            raise ValueError("Binary64Vector3V1 requires finite components")


@dataclass(frozen=True, slots=True)
class Binary64Matrix2V1:
    m00: float
    m01: float
    m10: float
    m11: float

    def __post_init__(self) -> None:
        if not all(
            isfinite(item)
            for item in (self.m00, self.m01, self.m10, self.m11)
        ):
            raise ValueError("Binary64Matrix2V1 requires finite components")


@dataclass(frozen=True, slots=True)
class Binary64SourceVertexCoordinateV1:
    source_vertex_id: SourceVertexId
    domain_coordinate: Binary64Point2V1


@dataclass(frozen=True, slots=True)
class DerivedBinary64AffineViewV1:
    origin: Binary64Point3V1
    basis_a: Binary64Vector3V1
    basis_b: Binary64Vector3V1
    gram_matrix: Binary64Matrix2V1
    inverse_gram_matrix: Binary64Matrix2V1
    source_vertex_coordinates: frozenset[
        Binary64SourceVertexCoordinateV1
    ]


@dataclass(frozen=True, slots=True)
class RuntimePredicateFilterContractV1:
    filter_law: RuntimePredicateFilterLawV1
    uncertain_result: RuntimePredicateResultV1
    exact_zero_requires_fallback: bool
    semantic_identity_law: MetricSemanticIdentityLawV1

    def __post_init__(self) -> None:
        if (
            self.uncertain_result
            is not RuntimePredicateResultV1.EXACT_FALLBACK_REQUIRED
        ):
            raise ValueError(
                "uncertain runtime predicates must require exact fallback"
            )
        if not self.exact_zero_requires_fallback:
            raise ValueError(
                "binary64 zero cannot be semantic authority"
            )


@dataclass(frozen=True, slots=True)
class RuntimeMetricFallbackContractV1:
    fallback_law: RuntimeMetricFallbackLawV1
    authoritative_reference_metric_id: ReferenceMetricId
    rounded_runtime_reconstruction_forbidden: bool

    def __post_init__(self) -> None:
        if not self.rounded_runtime_reconstruction_forbidden:
            raise ValueError(
                "runtime fallback cannot reconstruct exact facts from floats"
            )


@dataclass(frozen=True, slots=True)
class RuntimePlanarMetricV1:
    runtime_metric_id: RuntimeMetricId
    reference_metric_id: ReferenceMetricId
    derived_binary64_view: DerivedBinary64AffineViewV1
    predicate_filter_contract: RuntimePredicateFilterContractV1
    fallback_contract: RuntimeMetricFallbackContractV1

    def __post_init__(self) -> None:
        if (
            self.fallback_contract.authoritative_reference_metric_id
            != self.reference_metric_id
        ):
            raise ValueError(
                "RuntimePlanarMetricV1 must fallback to its reference metric"
            )
